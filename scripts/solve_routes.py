#!/usr/bin/env python3
"""
NextEra Energy Drone Optimization – Improved Solver

Features:
- Loads .npy data; validates symmetry/NaN/Inf; enforces [lon, lat] ordering
- Global TSP on required nodes OR multi-vehicle VRP that auto-splits under cap
- Greedy split fallback (kept from original)
- Robust predecessor expansion with graceful fallback
- Emits routes.json, metrics.json, targets.json, and routes.geojson (+ polygon if provided)
- Clear coverage/constraint metrics; reproducible search with --seed
- Unit consistency via --units (feet|meters) and --max-cap (same units as matrix)

Usage:
  python scripts/solve_routes.py \
    --data-dir "../NEE UCF Hackathon Challenge Data & Guide" \
    --out "../flite/public/routes.json" \
    --units feet --max-cap 37725 \
    [--vrp] [--polygon "../.../polygon_lon_lat.wkt"] [--seed 0]
"""

from __future__ import annotations
import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Dict, Any, Optional

import numpy as np
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Optional: for GeoJSON polygon export
try:
    from shapely import wkt as shapely_wkt  # type: ignore
    _HAS_SHAPELY = True
except Exception:
    _HAS_SHAPELY = False

# --------------------------- Data IO ---------------------------

def load_data(data_dir: Path):
    dist = np.load(data_dir / 'distance_matrix.npy')           # (N,N), units = feet or meters (see --units)
    preds = np.load(data_dir / 'predecessors.npy')             # (N,N), scipy dijkstra predecessor matrix
    pts   = np.load(data_dir / 'points_lat_long.npy')          # (N,2) expected [lon, lat]
    assets_idxs = np.load(data_dir / 'asset_indexes.npy')
    photos_idxs = np.load(data_dir / 'photo_indexes.npy')
    return dist, preds, pts, assets_idxs, photos_idxs


def normalize_targets(arr: np.ndarray, n: int) -> List[int]:
    """
    Accept either:
      - pair [start, end] (end exclusive)
      - explicit list of indices
    Returns sorted unique indices within [0, n), excluding depot 0.
    """
    arr = np.asarray(arr).astype(int).ravel()
    if arr.size == 2:
        start, end = int(arr[0]), int(arr[1])
        start = max(0, start)
        end = max(start, min(n, end))  # end EXCLUSIVE
        idxs = list(range(start, end))
    else:
        idxs = [int(x) for x in arr.tolist()]
    return sorted({i for i in idxs if 0 <= i < n and i != 0})


def validate_inputs(dist: np.ndarray, preds: np.ndarray, pts: np.ndarray, nodes: List[int]) -> None:
    if dist.shape[0] != dist.shape[1]:
        raise ValueError("distance_matrix must be square.")
    if preds.shape != dist.shape:
        raise ValueError("predecessors shape must match distance_matrix.")
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points_lat_long must be of shape (N,2).")
    if np.isnan(dist).any() or np.isinf(dist).any():
        raise ValueError("distance_matrix has NaN or Inf entries. Clean or impute them.")

    # Symmetrize if slightly off
    if not np.allclose(dist, dist.T, atol=1e-6):
        print("Warning: distance_matrix not exactly symmetric; symmetrizing.", file=sys.stderr)
        dist[:] = (dist + dist.T) / 2.0

    # Zero diagonal
    if (np.diag(dist) != 0).any():
        np.fill_diagonal(dist, 0.0)

    # [lon, lat] sanity (not strict; just guard against swapped arrays)
    lon_rng = (float(np.nanmin(pts[:, 0])), float(np.nanmax(pts[:, 0])))
    lat_rng = (float(np.nanmin(pts[:, 1])), float(np.nanmax(pts[:, 1])))
    if not (-200.0 < lon_rng[0] < 200.0 and -200.0 < lon_rng[1] < 200.0 and
            -100.0 < lat_rng[0] < 100.0 and -100.0 < lat_rng[1] < 100.0):
        raise ValueError("points_lat_long may not be [lon, lat]. Verify ordering in the file.")

    # Filter out-of-bounds targets
    max_idx = pts.shape[0] - 1
    bad = [i for i in nodes if i < 0 or i > max_idx]
    if bad:
        print(f"Filtering {len(bad)} nodes outside points range: {bad}", file=sys.stderr)
        nodes[:] = [i for i in nodes if 0 <= i <= max_idx]


# --------------------------- Path reconstruction ---------------------------

def reconstruct_path(i: int, j: int, predecessors: np.ndarray) -> List[int]:
    """
    Robust expansion using scipy dijkstra predecessor conventions.
    Falls back to [i, j] if broken.
    """
    if i == j:
        return [i]
    path = [j]
    cur = j
    N = predecessors.shape[0]
    for _ in range(N + 5):
        pre = int(predecessors[i, cur])
        if pre < 0:  # -1 or -9999
            return [i, j]
        path.append(pre)
        if pre == i:
            break
        cur = pre
    path.reverse()
    return path if (path and path[0] == i and path[-1] == j) else [i, j]


def to_coords_sequence(indices: List[int], predecessors: np.ndarray) -> List[int]:
    """
    Expand a sequence of index hops (e.g., [0, a, b, 0]) into a full waypoint path via predecessors.
    """
    expanded: List[int] = []
    for a, b in zip(indices[:-1], indices[1:]):
        seg = reconstruct_path(a, b, predecessors)
        if expanded:
            expanded.extend(seg[1:])  # avoid duplicate junction
        else:
            expanded.extend(seg)
    return expanded


# --------------------------- Solvers ---------------------------

def solve_global_tsp(distance_matrix: np.ndarray, nodes: List[int], depot: int = 0,
                     time_limit_s: int = 10, random_seed: int = 0) -> List[int]:
    """
    Single-vehicle TSP that returns a closed tour: [depot, ...nodes..., depot]
    """
    indices = [depot] + nodes
    index_to_node = dict(enumerate(indices))
    node_to_index = {n: i for i, n in index_to_node.items()}
    n = len(indices)

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 vehicle, depot=0 in model-space
    routing = pywrapcp.RoutingModel(manager)

    def dist_cb(from_index, to_index):
        a = index_to_node[manager.IndexToNode(from_index)]
        b = index_to_node[manager.IndexToNode(to_index)]
        # Keep native matrix units; OR-Tools requires integer cost
        return int(round(distance_matrix[a, b]))

    transit = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.FromSeconds(int(time_limit_s))
    if random_seed is not None:
        search_params.random_seed = int(random_seed)

    solution = routing.SolveWithParameters(search_params)
    if solution is None:
        print("TSP solver failed; falling back to greedy order.", file=sys.stderr)
        return indices + [depot]

    order = []
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        node = index_to_node[manager.IndexToNode(idx)]
        order.append(node)
        idx = solution.Value(routing.NextVar(idx))
    order.append(index_to_node[manager.IndexToNode(idx)])  # end (depot)

    return order


def split_legs(order: List[int], distance_matrix: np.ndarray, cap: float, depot: int = 0) -> List[List[int]]:
    """
    Greedy partition: pack nodes into legs under cap, ensuring depot returns.
    order: a closed TSP tour [0, ..., 0]
    Returns: list of legs, each leg is node indices (without depot).
    """
    if order and order[0] == order[-1]:
        order = order[:-1]
    if order and order[0] == depot:
        order = order[1:]

    legs: List[List[int]] = []
    cur_leg: List[int] = []
    cur = depot
    flown = 0.0

    for node in order:
        add = float(distance_matrix[cur, node])
        back = float(distance_matrix[node, depot])
        needed = (flown + add + back)
        if not cur_leg:
            # starting new leg: cost = cur->node + node->depot
            if add + back <= cap + 1e-9:
                cur_leg.append(node)
                flown = add
                cur = node
            else:
                # cannot even reach one node + return, start alone (will violate; let metrics flag)
                legs.append([node])
                cur = depot
                flown = 0.0
                cur_leg = []
        else:
            if needed <= cap + 1e-9:
                cur_leg.append(node)
                flown += add
                cur = node
            else:
                legs.append(cur_leg)
                cur_leg = [node]
                flown = float(distance_matrix[depot, node])
                cur = node
    if cur_leg:
        legs.append(cur_leg)
    return legs


def solve_vrp(distance_matrix: np.ndarray, required: List[int], depot: int,
              cap: float, max_vehicles: int = 32,
              time_limit_s: int = 30, random_seed: int = 0) -> Optional[List[List[int]]]:
    """
    Multi-vehicle VRP under per-route distance cap; returns legs (node lists w/o depot).
    """
    idxs = [depot] + required
    idx_to_node = dict(enumerate(idxs))
    node_to_idx = {n: i for i, n in idx_to_node.items()}
    n = len(idxs)

    manager = pywrapcp.RoutingIndexManager(n, max_vehicles, node_to_idx[depot])
    routing = pywrapcp.RoutingModel(manager)

    def transit_cb(fi, ti):
        a = idx_to_node[manager.IndexToNode(fi)]
        b = idx_to_node[manager.IndexToNode(ti)]
        return int(round(distance_matrix[a, b]))

    t_idx = routing.RegisterTransitCallback(transit_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(t_idx)

    # Distance dimension (battery cap)
    routing.AddDimension(
        t_idx,
        0,                         # no slack
        int(round(cap)),           # per-route cap, in matrix units
        True,                      # start cumul at zero
        "Distance"
    )

    # Make all required nodes mandatory
    for nidx in range(n):
        node = idx_to_node[nidx]
        if node == depot:
            continue
        routing.AddDisjunction([manager.NodeToIndex(nidx)], 0)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(int(time_limit_s))
    if random_seed is not None:
        params.random_seed = int(random_seed)

    sol = routing.SolveWithParameters(params)
    if sol is None:
        return None

    legs: List[List[int]] = []
    for v in range(max_vehicles):
        idx = routing.Start(v)
        if routing.IsEnd(idx):
            continue
        route: List[int] = []
        while not routing.IsEnd(idx):
            model_node = manager.IndexToNode(idx)
            node = idx_to_node[model_node]
            if node != depot:
                route.append(node)
            idx = sol.Value(routing.NextVar(idx))
        if route:
            legs.append(route)
    return legs


# --------------------------- Metrics & Export ---------------------------

def compute_metrics(legs_idx: List[List[int]], dist: np.ndarray,
                    required: List[int], depot: int, cap: Optional[float]) -> Dict[str, Any]:
    visited = set()
    legs_metrics = []
    for k, leg in enumerate(legs_idx):
        seq = [depot] + leg + [depot]
        length = float(sum(dist[a, b] for a, b in zip(seq[:-1], seq[1:])))
        ok = (cap is None) or (length <= cap + 1e-6)
        legs_metrics.append({'id': k, 'distance': length, 'underCap': bool(ok), 'stops': len(leg)})
        visited.update(leg)

    req_set = set(required)
    cov_ok = req_set.issubset(visited)
    total = sum(m['distance'] for m in legs_metrics)
    longest = max((m['distance'] for m in legs_metrics), default=0.0)
    return {
        'coverage_ok': cov_ok,
        'required_total': len(required),
        'visited_required': len(req_set & visited),
        'total_distance': total,
        'legs': len(legs_idx),
        'longest_leg': longest,
        'any_over_cap': any(not m['underCap'] for m in legs_metrics),
        'legs_detail': legs_metrics
    }


def write_geojson(legs_out: List[Dict[str, Any]], pts: np.ndarray,
                  polygon_wkt_path: Optional[Path], out_geojson: Path) -> None:
    features: List[Dict[str, Any]] = []
    # Legs as LineStrings
    for k, leg in enumerate(legs_out):
        features.append({
            "type": "Feature",
            "properties": {"id": k, "distance": leg['distance']},
            "geometry": {"type": "LineString", "coordinates": leg['coords']}
        })
    # Depot as Point
    features.append({
        "type": "Feature",
        "properties": {"role": "depot"},
        "geometry": {"type": "Point", "coordinates": [float(pts[0, 0]), float(pts[0, 1])]}
    })
    # Polygon (if provided and shapely available)
    if polygon_wkt_path and polygon_wkt_path.exists() and _HAS_SHAPELY:
        poly = shapely_wkt.loads(polygon_wkt_path.read_text())
        if hasattr(poly, "exterior"):
            features.append({
                "type": "Feature",
                "properties": {"role": "airspace"},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [list(map(list, poly.exterior.coords))]
                }
            })
    geo = {"type": "FeatureCollection", "features": features}
    out_geojson.write_text(json.dumps(geo))


# --------------------------- Main ---------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', type=str, default=str(Path(__file__).resolve().parents[2] / 'NEE UCF Hackathon Challenge Data & Guide'))
    ap.add_argument('--out', type=str, default=str(Path(__file__).resolve().parents[1] / 'public' / 'routes.json'))
    ap.add_argument('--units', choices=['feet','meters'], default='feet', help='Units of distance_matrix and cap.')
    ap.add_argument('--max-cap', type=float, default=37725.0, help='Per-leg cap, in --units.')
    ap.add_argument('--use-assets', action='store_true', help='Use asset indices instead of photos.')
    ap.add_argument('--vrp', action='store_true', help='Use multi-vehicle VRP (auto-partition under cap).')
    ap.add_argument('--vrp-vehicles', type=int, default=32, help='Max vehicles for VRP mode.')
    ap.add_argument('--tsp-time', type=int, default=10, help='TSP time limit (s).')
    ap.add_argument('--vrp-time', type=int, default=30, help='VRP time limit (s).')
    ap.add_argument('--seed', type=int, default=0, help='Random seed for reproducibility.')
    ap.add_argument('--polygon', type=str, default='', help='Path to polygon_lon_lat.wkt for GeoJSON export.')
    args = ap.parse_args()

    data_dir = Path(args.data_dir)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    polygon_path = Path(args.polygon) if args.polygon else None

    dist, preds, pts, assets_raw, photos_raw = load_data(data_dir)
    N = int(dist.shape[0])

    # choose targets (required set)
    raw = assets_raw if args.use_assets else photos_raw
    nodes = normalize_targets(raw, N)

    if not nodes:
        print("No target nodes resolved; check input arrays.", file=sys.stderr)
        sys.exit(1)

    validate_inputs(dist, preds, pts, nodes)

    depot = 0
    cap = float(args.max_cap)  # keep native units throughout

    # ---- Solve ----
    if args.vrp:
        legs_idx = solve_vrp(
            distance_matrix=dist,
            required=nodes,
            depot=depot,
            cap=cap,
            max_vehicles=int(args.vrp_vehicles),
            time_limit_s=int(args.vrp_time),
            random_seed=int(args.seed),
        )
        if legs_idx is None:
            print("VRP failed; falling back to TSP + greedy split.", file=sys.stderr)
            order = solve_global_tsp(dist, nodes, depot=depot, time_limit_s=args.tsp_time, random_seed=args.seed)
            legs_idx = split_legs(order, dist, cap, depot=depot)
    else:
        order = solve_global_tsp(dist, nodes, depot=depot, time_limit_s=args.tsp_time, random_seed=args.seed)
        legs_idx = split_legs(order, dist, cap, depot=depot)

    # ---- Build outputs ----
    legs_out: List[Dict[str, Any]] = []
    for leg in legs_idx:
        if not leg:
            continue
        seq = [depot] + leg + [depot]               # compact indices with depot at both ends
        expanded_idx = to_coords_sequence(seq, preds)
        # pts is [lon, lat]
        coords = [[float(pts[i, 0]), float(pts[i, 1])] for i in expanded_idx]
        distance = float(sum(dist[a, b] for a, b in zip(seq[:-1], seq[1:])))
        legs_out.append({
            'indices': list(map(int, leg)),
            'expanded_indices': list(map(int, expanded_idx)),
            'coords': coords,
            'distance': distance,                    # native units (feet or meters)
        })

    totals = {
        'distance': float(sum(l['distance'] for l in legs_out)),
        'legs': len(legs_out)
    }

    # provenance
    try:
        import ortools  # type: ignore
        ortools_ver = getattr(ortools, "__version__", "unknown")
    except Exception:
        ortools_ver = "unknown"

    out = {
        'source': 'NEE dataset',
        'params': {
            'units': args.units,
            'capPerLeg': cap,
            'mode': 'vrp' if args.vrp else 'tsp+greedy',
            'seed': args.seed
        },
        'legs': legs_out,
        'totals': totals,
        'provenance': {
            'python': sys.version,
            'numpy': np.__version__,
            'ortools': ortools_ver,
        }
    }

    out_path.write_text(json.dumps(out))
    print(f'Wrote {out_path} with {len(legs_out)} legs')

    # targets.json (for frontend layers)
    primary_type = 'asset' if args.use_assets else 'photo'
    targets = [
        {
            'id': int(i),
            'coord': [float(pts[i, 0]), float(pts[i, 1])],
            'type': primary_type,
        }
        for i in nodes
    ]
    targets_out = out_path.parent / 'targets.json'
    targets_out.write_text(json.dumps({
        'targets': targets,
        'count': len(targets),
        'source': 'assets' if args.use_assets else 'photos'
    }))
    print(f'Wrote {targets_out} with {len(targets)} targets')

    # metrics.json
    metrics = compute_metrics(legs_idx, dist, required=nodes, depot=depot, cap=cap)
    metrics_out = out_path.parent / 'metrics.json'
    metrics_out.write_text(json.dumps(metrics, indent=2))
    print(f"Coverage OK: {metrics['coverage_ok']} | Legs: {metrics['legs']} | Total: {metrics['total_distance']:.2f} {args.units}")

    # routes.geojson
    geo_out = out_path.parent / 'routes.geojson'
    try:
        write_geojson(legs_out, pts, polygon_path, geo_out)
        print(f'Wrote {geo_out}')
    except Exception as e:
        print(f'GeoJSON export failed (non-fatal): {e}', file=sys.stderr)

    # Exit with nonzero if constraints violated (useful in CI/demo guards)
    if (not metrics['coverage_ok']) or metrics['any_over_cap']:
        sys.exit(2)


if __name__ == '__main__':
    main()
