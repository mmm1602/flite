/**
 * Battery-constrained Mission Planner for Drone Pole Inspections
 * -------------------------------------------------------------
 * Heuristics: Nearest-Neighbor + 2-Opt per leg, with greedy leg splitting
 * to respect energy (battery) capacity and reserve. Pure JS, no deps.
 *
 * Contract
 * - Inputs:
 *   - base: [lon, lat]
 *   - poles: Array<{ id?: string|number, coord: [lon, lat] }>
 *   - params:
 *       speedMps: number (cruise speed, m/s)
 *       whPerMeter: number (cruise energy per meter)
 *       inspectSeconds: number (hover time per pole)
 *       hoverWhPerSecond: number (hover energy rate, Wh/s)
 *       batteryWh: number (battery capacity, Wh)
 *       reservePct: number (0..1)
 * - Output:
 *   - { legs: Array<{ orderIdx: number[], coords: [lon,lat][], meters: number, timeSec: number, energyWh: number }>,
 *       totals: { meters: number, timeSec: number, energyWh: number } }
 *
 * Notes
 * - Coordinates are treated on a sphere via haversine distances (meters).
 * - Greedy leg pack: add nearest next pole until (routeEnergy + return) exceeds usableWh.
 * - 2-Opt local search improves each leg order.
 */

// ---------- Geometry ----------
export function haversine([lon1, lat1], [lon2, lat2]) {
  const toRad = (d) => (d * Math.PI) / 180;
  const R = 6371e3;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return R * c; // meters
}

export function buildDistanceMatrix(coords) {
  const n = coords.length;
  const D = Array.from({ length: n }, () => Array(n).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const d = haversine(coords[i], coords[j]);
      D[i][j] = D[j][i] = d;
    }
  }
  return D;
}

function pathLength(coords) {
  if (!Array.isArray(coords) || coords.length < 2) return 0;
  let total = 0;
  for (let i = 1; i < coords.length; i += 1) {
    total += haversine(coords[i - 1], coords[i]);
  }
  return total;
}

const EPSILON = 1e-9;

function pointOnSegment([px, py], [ax, ay], [bx, by], eps = EPSILON) {
  const cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax);
  if (Math.abs(cross) > eps) return false;
  const dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay);
  if (dot < -eps) return false;
  const squaredLen = (bx - ax) ** 2 + (by - ay) ** 2;
  if (dot - squaredLen > eps) return false;
  return true;
}

// ---------- Lightweight route + animation helpers ----------
// legacy export kept for compatibility (nearest-neighbor chaining)
export function buildRoute(base, poles) {
  const remaining = poles
    .filter((p) => Array.isArray(p?.coord) && p.coord.length === 2)
    .map((p) => p.coord.slice());
  const path = [base.slice()];
  let current = base;

  while (remaining.length) {
    let bestIdx = 0;
    let bestD = Infinity;
    for (let i = 0; i < remaining.length; i++) {
      const d = haversine(current, remaining[i]);
      if (d < bestD) {
        bestD = d;
        bestIdx = i;
      }
    }
    const next = remaining.splice(bestIdx, 1)[0];
    path.push(next);
    current = next;
  }

  path.push(base.slice());
  return path;
}

function pointInRing([x, y], ring) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [xi, yi] = ring[i];
    const [xj, yj] = ring[j];
    if (pointOnSegment([x, y], [xi, yi], [xj, yj])) return true;
    const intersect = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / Math.max(yj - yi, 1e-12) + xi;
    if (intersect) inside = !inside;
  }
  return inside;
}

export function pointInPolygon(coord, polygon) {
  if (!Array.isArray(polygon) || polygon.length === 0) return false;
  const [outer, ...holes] = polygon;
  if (!pointInRing(coord, outer)) return false;
  for (const hole of holes) {
    if (pointInRing(coord, hole)) return false;
  }
  return true;
}

export function pointInPolygons(coord, polygons) {
  if (!Array.isArray(polygons)) return false;
  for (const polygon of polygons) {
    if (pointInPolygon(coord, polygon)) return true;
  }
  return false;
}

export function filterCoordsInPolygons(coords, polygons) {
  if (!Array.isArray(coords)) return [];
  return coords.filter((coord) => pointInPolygons(coord, polygons));
}

function coordsEqual([ax, ay], [bx, by], eps = EPSILON) {
  return Math.abs(ax - bx) <= eps && Math.abs(ay - by) <= eps;
}

function orientation([px, py], [qx, qy], [rx, ry]) {
  const val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
  if (Math.abs(val) <= EPSILON) return 0;
  return val > 0 ? 1 : 2; // 1: clockwise, 2: counterclockwise
}

function onSegment([px, py], [qx, qy], [rx, ry]) {
  return (
    qx <= Math.max(px, rx) + EPSILON &&
    qx + EPSILON >= Math.min(px, rx) &&
    qy <= Math.max(py, ry) + EPSILON &&
    qy + EPSILON >= Math.min(py, ry)
  );
}

function segmentsIntersect(p1, q1, p2, q2) {
  const o1 = orientation(p1, q1, p2);
  const o2 = orientation(p1, q1, q2);
  const o3 = orientation(p2, q2, p1);
  const o4 = orientation(p2, q2, q1);

  if (o1 !== o2 && o3 !== o4) return true;

  if (o1 === 0 && onSegment(p1, p2, q1)) return true;
  if (o2 === 0 && onSegment(p1, q2, q1)) return true;
  if (o3 === 0 && onSegment(p2, p1, q2)) return true;
  if (o4 === 0 && onSegment(p2, q1, q2)) return true;
  return false;
}

function segmentTouchesOnlyAtSharedEndpoint(a, b, c, d) {
  return (
    coordsEqual(a, c) ||
    coordsEqual(a, d) ||
    coordsEqual(b, c) ||
    coordsEqual(b, d)
  );
}

function segmentCrossesRing(a, b, ring) {
  const n = ring.length;
  for (let i = 0; i < n - 1; i++) {
    const c = ring[i];
    const d = ring[i + 1];
    if (segmentTouchesOnlyAtSharedEndpoint(a, b, c, d)) continue;
    if (segmentsIntersect(a, b, c, d)) return true;
  }
  return false;
}

function segmentWithinPolygon(a, b, polygon) {
  if (!polygon || polygon.length === 0) return false;
  if (!pointInPolygon(a, polygon) || !pointInPolygon(b, polygon)) return false;
  if (coordsEqual(a, b)) return true;

  const [outer, ...holes] = polygon;
  if (segmentCrossesRing(a, b, outer)) return false;

  for (const hole of holes) {
    if (segmentCrossesRing(a, b, hole)) return false;
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    if (pointInRing(mid, hole)) return false;
  }
  return true;
}

function addUniqueNode(nodes, coord) {
  for (const existing of nodes) {
    if (coordsEqual(existing, coord)) return;
  }
  nodes.push(coord.slice());
}

function shortestPathWithinPolygon(start, end, polygon) {
  if (!segmentWithinPolygon(start, end, polygon)) {
    const nodes = [start.slice(), end.slice()];
    const [outer, ...holes] = polygon;
    for (const vertex of outer) addUniqueNode(nodes, vertex);
    for (const hole of holes) {
      for (const vertex of hole) addUniqueNode(nodes, vertex);
    }

    const n = nodes.length;
    const graph = Array.from({ length: n }, () => []);
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (segmentWithinPolygon(nodes[i], nodes[j], polygon)) {
          const weight = haversine(nodes[i], nodes[j]);
          graph[i].push({ to: j, w: weight });
          graph[j].push({ to: i, w: weight });
        }
      }
    }

    const dist = Array(n).fill(Infinity);
    const visited = Array(n).fill(false);
    const prev = Array(n).fill(-1);
    dist[0] = 0;

    for (let iter = 0; iter < n; iter++) {
      let u = -1;
      let best = Infinity;
      for (let i = 0; i < n; i++) {
        if (!visited[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u === -1 || best === Infinity) break;
      visited[u] = true;
      if (u === 1) break;

      for (const edge of graph[u]) {
        if (visited[edge.to]) continue;
        const nextDist = dist[u] + edge.w;
        if (nextDist + EPSILON < dist[edge.to]) {
          dist[edge.to] = nextDist;
          prev[edge.to] = u;
        }
      }
    }

    if (!visited[1]) {
      return [start.slice(), end.slice()];
    }

    const sequence = [];
    let cur = 1;
    while (cur !== -1) {
      sequence.push(cur);
      cur = prev[cur];
    }
    sequence.reverse();
    return sequence.map((idx) => nodes[idx].slice());
  }

  return [start.slice(), end.slice()];
}

function resolvePolygonForSegment(start, end, polygons) {
  if (!Array.isArray(polygons) || polygons.length === 0) {
    return null;
  }

  const startPolygons = [];
  const endPolygons = [];

  for (const polygon of polygons) {
    if (pointInPolygon(start, polygon)) startPolygons.push(polygon);
    if (pointInPolygon(end, polygon)) endPolygons.push(polygon);
  }

  for (const polygon of startPolygons) {
    if (endPolygons.includes(polygon)) return polygon;
  }

  if (startPolygons.length === 1) return startPolygons[0];
  if (endPolygons.length === 1) return endPolygons[0];
  if (startPolygons.length) return startPolygons[0];
  if (endPolygons.length) return endPolygons[0];

  return null;
}

function pathWithinPolygons(start, end, polygons) {
  const polygon = resolvePolygonForSegment(start, end, polygons);
  if (!polygon) {
    return [start.slice(), end.slice()];
  }

  return shortestPathWithinPolygon(start, end, polygon);
}

function computeTspPlan(base, coords) {
  if (!Array.isArray(base) || base.length !== 2) {
    throw new Error("Base coordinate must be [lon, lat]");
  }

  const cleanCoords = (coords || []).filter((c) => Array.isArray(c) && c.length === 2)
    .map((c) => c.slice());
  const all = [[base[0], base[1]], ...cleanCoords.map((c) => c.slice())];
  if (cleanCoords.length === 0) {
    return { all, optimized: [] };
  }

  const indices = cleanCoords.map((_, i) => i + 1);
  const D = buildDistanceMatrix(all);
  const dist = (i, j) => D[i][j];

  const seedOrder = nearestNeighborOrder(0, indices, dist);
  const optimized = twoOpt(seedOrder, dist);

  return { all, optimized };
}

export function solveTspRoute(base, coords, polygons = null) {
  const { all, optimized } = computeTspPlan(base, coords);
  if (!optimized.length) {
    return [all[0].slice(), all[0].slice()];
  }

  const path = [all[0].slice()];
  let currentIdx = 0;

  const appendSegment = (fromIdx, toIdx) => {
    const from = all[fromIdx];
    const to = all[toIdx];
    if (polygons && polygons.length) {
      const segmentPath = pathWithinPolygons(from, to, polygons);
      for (let i = 1; i < segmentPath.length; i++) {
        path.push(segmentPath[i].slice());
      }
    } else {
      path.push([to[0], to[1]]);
    }
  };

  for (const idx of optimized) {
    appendSegment(currentIdx, idx);
    currentIdx = idx;
  }

  appendSegment(currentIdx, 0);
  return path;
}

export function solveBatteryTrips(base, coords, polygons = null, maxTripMeters = Infinity) {
  let maxMeters = Number.isFinite(maxTripMeters) && maxTripMeters > 0 ? maxTripMeters : Infinity;
  const { all, optimized } = computeTspPlan(base, coords);
  const baseCoord = all[0];

  if (!optimized.length) {
    return {
      legs: [],
      totalMeters: 0,
      longestLegMeters: 0,
      path: [baseCoord.slice(), baseCoord.slice()],
      maxMeters,
    };
  }

  const legs = [];
  let prevIdx = 0;
  let currentPath = [baseCoord.slice()];
  let currentDistance = 0;
  let currentTargets = [];

  const addReturnToBase = () => {
    if (prevIdx === 0) return;
    const backSegment = pathWithinPolygons(all[prevIdx], baseCoord, polygons);
    for (let i = 1; i < backSegment.length; i++) {
      currentPath.push(backSegment[i].slice());
    }
    currentDistance += pathLength(backSegment);
    prevIdx = 0;
  };

  const commitLeg = () => {
    if (!currentTargets.length) return;
    addReturnToBase();
    const index = legs.length;
    legs.push({
      index,
      path: currentPath.map((p) => p.slice()),
      meters: currentDistance,
      targetCount: currentTargets.length,
      targets: currentTargets.map((p) => p.slice()),
    });
    currentPath = [baseCoord.slice()];
    currentDistance = 0;
    currentTargets = [];
  };

  for (let i = 0; i < optimized.length; i += 1) {
    const idx = optimized[i];
    const fromCoord = all[prevIdx];
    const toCoord = all[idx];
    const segmentPath = pathWithinPolygons(fromCoord, toCoord, polygons);
    const segmentMeters = pathLength(segmentPath);
    const returnPath = pathWithinPolygons(toCoord, baseCoord, polygons);
    const returnMeters = pathLength(returnPath);
    const projected = currentDistance + segmentMeters + returnMeters;

    if (currentTargets.length > 0 && projected > maxMeters + EPSILON) {
      commitLeg();
      i -= 1;
      continue;
    }

    for (let j = 1; j < segmentPath.length; j += 1) {
      currentPath.push(segmentPath[j].slice());
    }
    currentDistance += segmentMeters;
    currentTargets.push(toCoord.slice());
    prevIdx = idx;
  }

  commitLeg();

  const totalMeters = legs.reduce((acc, leg) => acc + leg.meters, 0);
  const longestLegMeters = legs.reduce((max, leg) => Math.max(max, leg.meters), 0);
  let combinedPath = [];

  if (legs.length) {
    legs.forEach((leg, index) => {
      if (index === 0) {
        combinedPath = leg.path.map((p) => p.slice());
      } else {
        for (let i = 1; i < leg.path.length; i++) {
          combinedPath.push(leg.path[i].slice());
        }
      }
    });
  }

  if (!combinedPath.length) {
    combinedPath = [baseCoord.slice(), baseCoord.slice()];
  }

  return {
    legs,
    totalMeters,
    longestLegMeters,
    path: combinedPath,
    maxMeters,
  };
}

export function pathToTrip(path, speedMps) {
  if (speedMps <= 0) throw new Error("speedMps must be > 0");
  let t = 0;
  const timestamps = [0];
  for (let i = 1; i < path.length; i++) {
    const seg = haversine(path[i - 1], path[i]);
    const dt = seg / speedMps;
    t += dt;
    timestamps.push(t);
  }
  return { path, timestamps };
}

export function lerpCoord([lon1, lat1], [lon2, lat2], f) {
  return [lon1 + (lon2 - lon1) * f, lat1 + (lat2 - lat1) * f];
}

export function positionAtTime(path, timestamps, t) {
  const T = timestamps[timestamps.length - 1];
  if (T <= 0) return path[0];
  const time = ((t % T) + T) % T;
  let i = 0;
  while (i < timestamps.length - 1 && !(timestamps[i] <= time && time < timestamps[i + 1])) i++;
  if (i >= timestamps.length - 1) return path[path.length - 1];
  const t0 = timestamps[i];
  const t1 = timestamps[i + 1];
  const f = (time - t0) / Math.max(1e-9, t1 - t0);
  return lerpCoord(path[i], path[i + 1], f);
}

export function trailPath(path, timestamps, t, windowSec) {
  const T = timestamps[timestamps.length - 1];
  if (T <= 0) return [path[0]];
  const endTime = ((t % T) + T) % T;
  const startTime = Math.max(0, endTime - windowSec);

  const segs = [];
  const pushClipped = (a, b, ta, tb) => {
    const clampedA = Math.max(startTime, ta);
    const clampedB = Math.min(endTime, tb);
    if (clampedB <= clampedA) return;
    const f0 = (clampedA - ta) / Math.max(1e-9, tb - ta);
    const f1 = (clampedB - ta) / Math.max(1e-9, tb - ta);
    const p0 = lerpCoord(a, b, f0);
    const p1 = lerpCoord(a, b, f1);
    segs.push(p0, p1);
  };

  for (let i = 0; i < timestamps.length - 1; i++) {
    const ta = timestamps[i];
    const tb = timestamps[i + 1];
    if (tb >= startTime && ta <= endTime) {
      pushClipped(path[i], path[i + 1], ta, tb);
    }
  }

  return segs.length ? segs : [positionAtTime(path, timestamps, endTime)];
}

// ---------- Energy/Time Model ----------
export function energyForMeters(meters, params) {
  return meters * (params.whPerMeter || 0); // Wh
}

export function energyForInspection(params) {
  return (params.inspectSeconds || 0) * (params.hoverWhPerSecond || 0);
}

export function timeForMeters(meters, params) {
  const v = Math.max(0.001, params.speedMps || 1);
  return meters / v; // seconds
}

// ---------- TSP Heuristics ----------
export function nearestNeighborOrder(startIdx, nodes, dist) {
  // nodes = array of indices into a coord array; dist(i,j) returns meters
  const remaining = new Set(nodes);
  const order = [];
  let cur = startIdx;
  while (remaining.size) {
    // pick nearest remaining
    let best = -1;
    let bestD = Infinity;
    for (const i of remaining) {
      const d = dist(cur, i);
      if (d < bestD) {
        bestD = d; best = i;
      }
    }
    order.push(best);
    remaining.delete(best);
    cur = best;
  }
  return order; // sequence of indices (excluding base)
}

export function twoOpt(order, dist) {
  // classic 2-opt improvement on sequence
  const n = order.length;
  if (n < 4) return order;
  let improved = true;
  const seq = order.slice();
  const tourDistance = (a, b) => dist(a, b);

  function delta(i, k) {
    const n = seq.length;
    const a = seq[(i - 1 + n) % n], b = seq[i];
    const c = seq[k], d = seq[(k + 1) % n];
    const before = tourDistance(a, b) + tourDistance(c, d);
    const after = tourDistance(a, c) + tourDistance(b, d);
    return after - before;
  }

  while (improved) {
    improved = false;
    for (let i = 1; i < n - 1; i++) {
      for (let k = i + 1; k < n; k++) {
        if (delta(i, k) < -1e-6) {
          // reverse segment [i, k]
          let lo = i, hi = k;
          while (lo < hi) {
            [seq[lo], seq[hi]] = [seq[hi], seq[lo]];
            lo++; hi--;
          }
          improved = true;
        }
      }
    }
  }
  return seq;
}

// ---------- Leg evaluation ----------
export function routeMeters(baseIdx, order, D) {
  if (!order.length) return 0;
  let m = 0;
  let prev = baseIdx;
  for (const i of order) { m += D[prev][i]; prev = i; }
  m += D[prev][baseIdx]; // return to base
  return m;
}

export function legMetrics(baseIdx, order, D, params, inspections) {
  const meters = routeMeters(baseIdx, order, D);
  const timeSec = timeForMeters(meters, params) + inspections * (params.inspectSeconds || 0);
  const energyWh = energyForMeters(meters, params) + inspections * energyForInspection(params);
  return { meters, timeSec, energyWh };
}

// ---------- Mission planner ----------
export function planMissions(base, poles, params) {
  const coords = [base, ...poles.map((p) => p.coord)];
  const baseIdx = 0;
  const idxs = poles.map((_, i) => i + 1); // indices in coords[]
  const D = buildDistanceMatrix(coords);
  const dist = (i, j) => D[i][j];

  const batteryWh = params.batteryWh || 200;
  const usableWh = batteryWh * (1 - (params.reservePct ?? 0.2));

  const remaining = new Set(idxs);
  const legs = [];

  while (remaining.size) {
    // seed leg with nearest to base
    let cur = baseIdx;
    let leg = [];

    function orderMetrics(ord) {
      return legMetrics(baseIdx, ord, D, params, ord.length);
    }

    // Greedily add nearest that keeps leg within usableWh
    while (remaining.size) {
      let cand = -1;
      let bestD = Infinity;
      for (const i of remaining) {
        const d = dist(cur, i);
        if (d < bestD) { bestD = d; cand = i; }
      }
      if (cand === -1) break;
      const trial = [...leg, cand];
      const metrics = orderMetrics(trial);
      if (metrics.energyWh <= usableWh) {
        leg = trial;
        remaining.delete(cand);
        cur = cand;
      } else {
        break; // can't add more, finalize this leg
      }
    }

    if (leg.length === 0) {
      // Fallback: take the closest single pole even if it exceeds usableWh — caller should adjust params
      const one = [...remaining][0];
      leg = [one];
      remaining.delete(one);
    }

    // Build initial NN sequence for the nodes in this leg w.r.t base
    const nnOrder = nearestNeighborOrder(baseIdx, leg, dist);
    const improved = twoOpt(nnOrder, (a, b) => (a === baseIdx || b === baseIdx ? Infinity : dist(a, b)));
    const metrics = orderMetrics(improved);

    legs.push({
      orderIdx: improved.map((i) => i - 0),
      coords: improved.map((i) => coords[i]),
      meters: metrics.meters,
      timeSec: metrics.timeSec,
      energyWh: metrics.energyWh,
    });
  }

  const totals = legs.reduce((acc, l) => {
    acc.meters += l.meters; acc.timeSec += l.timeSec; acc.energyWh += l.energyWh; return acc;
  }, { meters: 0, timeSec: 0, energyWh: 0 });

  return { legs, totals };
}

// ---------- Example Usage ----------
/*
import { planMissions } from './lib/routeSolver';

const params = {
  speedMps: 12,
  whPerMeter: 0.02,          // tune with field data
  inspectSeconds: 8,          // per pole
  hoverWhPerSecond: 0.15,     // tune with field data
  batteryWh: 200,
  reservePct: 0.2,
};

const { legs, totals } = planMissions(BASE, POLES, params);
*/
