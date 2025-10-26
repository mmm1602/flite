import { useCallback, useEffect, useMemo, useRef, useState, useId } from "react";
import DeckGL from "@deck.gl/react";
import { ScatterplotLayer, PolygonLayer, PathLayer } from "@deck.gl/layers";
import maplibregl from "maplibre-gl";
import MapView from "react-map-gl/maplibre";
import {
  pathToTrip,
  positionAtTime,
  trailPath,
  solveBatteryTrips,
  pointInPolygons,
  haversine,
} from "./lib/routeSolver";

/**
 * Flite – Minimal 2D Prototype (React + deck.gl core only)
 * ---------------------------------------------------------
 * FIX: Removed all imports from @deck.gl/geo-layers (TripsLayer, TileLayer) to prevent
 *      luma.gl multiple-version conflicts. Implemented our own animated path + drone marker
 *      using PathLayer + ScatterplotLayer so we rely on a single deck.gl/luma.gl version.
 *
 * What you get:
 * - Animated drone position moving along a route
 * - Visual trail (last N seconds of travel) using a clipped PathLayer
 * - Play/pause, speed, and trail length controls
 * - Poles (inspection points), Base marker, and a sample No-Fly polygon
 * - Pannable/zoomable canvas (no external basemap)
 *
 * How to extend:
 * - Replace `POLES` with your CSV/GeoJSON data
 * - Replace `buildRoute()` with your solver (k-means + 2-opt, battery legs, etc.)
 * - Compute timestamps from your energy/speed model and feed the animation loop
 */

// MapLibre GL styles (no token)
const MAP_STYLE_LIGHT = "https://basemaps.cartocdn.com/gl/positron-gl-style/style.json";
const MAP_STYLE_DARK = "https://basemaps.cartocdn.com/gl/dark-matter-gl-style/style.json";

const DEFAULT_FLIGHT_RING = [
  [-81.205, 28.6035],
  [-81.202, 28.6035],
  [-81.202, 28.6022],
  [-81.205, 28.6022],
  [-81.205, 28.6035],
];

const DEFAULT_BASE = DEFAULT_FLIGHT_RING[0];

// ---- View State ----
const INITIAL_VIEW_STATE = {
  longitude: DEFAULT_BASE[0],
  latitude: DEFAULT_BASE[1],
  zoom: 15,
  pitch: 40, // subtle tilt for depth
  bearing: 0,
};

const POLES = [];

const DEFAULT_FLIGHT_POLYGON = [[DEFAULT_FLIGHT_RING]];

const COMPANY_MAX_TRIP_METERS = 37725 * 0.3048; // company battery constraint (37,725 ft)

const TARGET_TYPE_COLORS = {
  asset: [70, 200, 255, 220],
  photo: [255, 140, 60, 220],
  unknown: [200, 200, 200, 220],
};

const TARGET_TYPE_LABELS = {
  asset: "Asset Pin",
  photo: "Photo Pin",
  unknown: "Target",
};

const MEDIA_UAV09 = new URL("../../media/14586258_3840_2160_30fps.mp4", import.meta.url).href;
const MEDIA_UAV14 = new URL("../../media/14586258_3840_2160_30fps.mp4", import.meta.url).href;
const MEDIA_UGV03 = MEDIA_UAV14;

const getTargetColor = (type) => {
  if (type && TARGET_TYPE_COLORS[type]) return TARGET_TYPE_COLORS[type];
  return TARGET_TYPE_COLORS.unknown;
};

const getTargetLabel = (type) => {
  if (type && TARGET_TYPE_LABELS[type]) return TARGET_TYPE_LABELS[type];
  return TARGET_TYPE_LABELS.unknown;
};

const normalizeTargetType = (value, fallback = "unknown") => {
  if (typeof value !== "string") return fallback;
  const lower = value.toLowerCase();
  if (lower === "assets") return "asset";
  if (lower === "photos") return "photo";
  if (TARGET_TYPE_COLORS[lower]) return lower;
  return fallback;
};

const DEPLOYED_VEHICLES = [
  {
    id: "UAV-09",
    type: "VTOL Recon",
    status: "Active",
    task: "Grid 7 Sweep",
    battery: 68,
    uplink: "Encrypted",
    altitude: "820 m",
    payload: "EO/IR",
    stream: {
      title: "EO/IR Fusion",
      url: MEDIA_UAV09,
      poster: "https://images.unsplash.com/photo-1500530855697-b586d89ba3ee?auto=format&fit=crop&w=800&q=80",
      note: "Thermal overlay with optical assist"
    },
  },
  {
    id: "UAV-14",
    type: "Fixed Wing",
    status: "Standby",
    task: "Reserve Orbit",
    battery: 92,
    uplink: "Encrypted",
    altitude: "Hangar",
    payload: "Mapping Lidar",
    stream: {
      title: "Circle Pattern",
      url: MEDIA_UAV14,
      poster: "https://images.unsplash.com/photo-1473186578172-c141e6798cf4?auto=format&fit=crop&w=800&q=80",
      note: "Awaiting launch; feed shows standby bay"
    },
  },
  {
    id: "UGV-03",
    type: "Ground Rover",
    status: "Maintenance",
    task: "Diagnostics",
    battery: 34,
    uplink: "Line",
    altitude: "N/A",
    payload: "Sensor Mast",
    stream: {
      title: "Rig Diagnostics",
      url: MEDIA_UGV03,
      poster: "https://images.unsplash.com/photo-1582719478181-2cf4e7a0b055?auto=format&fit=crop&w=800&q=80",
      note: "Service bay visual diagnostics"
    },
  },
];

function parsePolygonWkt(text) {
  if (!text) return null;
  const trimmed = text.trim();
  if (!trimmed) return null;
  const typeMatch = /^\s*([A-Z]+)(?:\s+[A-Z]+)?/i.exec(trimmed);
  const type = typeMatch ? typeMatch[1].toUpperCase() : '';
  const bodyStart = trimmed.indexOf('(');
  const bodyEnd = trimmed.lastIndexOf(')');
  if (bodyStart === -1 || bodyEnd === -1 || bodyEnd <= bodyStart) return null;

  const stripOuterParens = (str) => {
    let result = str.trim();
    if (!result.startsWith('(') || !result.endsWith(')')) return result;
    let depth = 0;
    for (let i = 0; i < result.length; i += 1) {
      const ch = result[i];
      if (ch === '(') depth += 1;
      else if (ch === ')') depth -= 1;
      if (depth === 0 && i < result.length - 1) {
        return result;
      }
    }
    return result.slice(1, -1).trim();
  };

  const parseCoordinateList = (segment) => {
    const coords = [];
    for (const token of segment.split(',')) {
      const parts = token.trim().split(/\s+/).filter(Boolean);
      if (parts.length < 2) continue;
      const lon = parseFloat(parts[0]);
      const lat = parseFloat(parts[1]);
      if (Number.isFinite(lon) && Number.isFinite(lat)) {
        coords.push([lon, lat]);
      }
    }
    if (coords.length >= 3) {
      const [firstLon, firstLat] = coords[0];
      const [lastLon, lastLat] = coords[coords.length - 1];
      if (firstLon !== lastLon || firstLat !== lastLat) {
        coords.push([firstLon, firstLat]);
      }
    }
    return coords;
  };

  const parsePolygonContent = (content) => {
    const rings = [];
    let depth = 0;
    let buffer = '';
    for (const ch of content) {
      if (ch === '(') {
        depth += 1;
        if (depth === 1) {
          buffer = '';
        } else if (depth > 1) {
          buffer += '(';
        }
      } else if (ch === ')') {
        if (depth === 1) {
          const ring = parseCoordinateList(buffer);
          if (ring.length) rings.push(ring);
          buffer = '';
        } else if (depth > 1) {
          buffer += ')';
        }
        depth -= 1;
      } else if (depth >= 1) {
        buffer += ch;
      }
    }
    return rings;
  };

  const splitTopLevelPolygons = (content) => {
    const polygons = [];
    let depth = 0;
    let buffer = '';
    for (const ch of content) {
      if (ch === '(') {
        depth += 1;
        if (depth > 1) buffer += '(';
      } else if (ch === ')') {
        if (depth > 1) buffer += ')';
        depth -= 1;
        if (depth === 0) {
          polygons.push(buffer.trim());
          buffer = '';
        }
      } else if (depth >= 1) {
        buffer += ch;
      }
    }
    return polygons.filter((p) => p.length > 0);
  };

  const rawBody = trimmed.slice(bodyStart, bodyEnd + 1);
  const body = stripOuterParens(rawBody);

  if (type === 'MULTIPOLYGON') {
    const polygons = splitTopLevelPolygons(body)
      .map((poly) => parsePolygonContent(poly))
      .filter((poly) => poly.length > 0);
    return polygons.length ? polygons : null;
  }

  if (type === 'POLYGON') {
    const polygon = parsePolygonContent(stripOuterParens(body));
    return polygon.length ? [polygon] : null;
  }

  return null;
}


export default function App() {
  const [speed, setSpeed] = useState(12); // m/s cruise speed
  const [trail, setTrail] = useState(30); // seconds of visible trail
  const [playing, setPlaying] = useState(false);
  const [time, setTime] = useState(0);
  const [follow, setFollow] = useState(false);
  const [theme, setTheme] = useState('dark');
  const [routes, setRoutes] = useState(null); // routes.json from solver
  const [showSolver, setShowSolver] = useState(false);
  const [routesData, setRoutesData] = useState(null); // simplified + prepped paths
  const [poles, setPoles] = useState(POLES); // dataset targets if available
  const [hoverCoord, setHoverCoord] = useState(null);
  const [clock, setClock] = useState(() => new Date().toLocaleTimeString());
  const [intelOpen, setIntelOpen] = useState(true);
  const [intelTab, setIntelTab] = useState("feed");
  const [activeVehicleId, setActiveVehicleId] = useState(DEPLOYED_VEHICLES[0]?.id ?? null);
  const [flightPolygons, setFlightPolygons] = useState(DEFAULT_FLIGHT_POLYGON);
  const [baseCoord, setBaseCoord] = useState(DEFAULT_BASE);
  const [activeLegIndex, setActiveLegIndex] = useState(null);
  const [videoErrors, setVideoErrors] = useState({});
  const idBase = useId();
  const speedSliderId = `${idBase}-speed`; // tie sliders to labels for accessibility
  const trailSliderId = `${idBase}-trail`;

  const missionTargets = useMemo(() => {
    if (!Array.isArray(flightPolygons) || !flightPolygons.length) return poles;
    const filtered = poles.filter(
      (target) => Array.isArray(target?.coord) && target.coord.length === 2 && pointInPolygons(target.coord, flightPolygons),
    );
    return filtered.length ? filtered : poles;
  }, [poles, flightPolygons]);

  const missionPlan = useMemo(() => {
    if (!baseCoord || baseCoord.length !== 2) {
      const fallbackBase = [DEFAULT_BASE[0], DEFAULT_BASE[1]];
      return {
        path: [fallbackBase, fallbackBase],
        legs: [],
        totalMeters: 0,
        longestLegMeters: 0,
        maxMeters: COMPANY_MAX_TRIP_METERS,
      };
    }
    const coords = missionTargets.map((t) => t.coord);
    return solveBatteryTrips(baseCoord, coords, flightPolygons, COMPANY_MAX_TRIP_METERS);
  }, [baseCoord, missionTargets, flightPolygons]);

  const routePath = missionPlan.path;
  const missionLegs = missionPlan.legs;
  const missionTotalMeters = missionPlan.totalMeters;
  const missionLongestLeg = missionPlan.longestLegMeters;
  const missionMaxMeters = missionPlan.maxMeters;

  useEffect(() => {
    if (activeLegIndex == null) return;
    if (!missionLegs.some((leg) => leg.index === activeLegIndex)) {
      setActiveLegIndex(null);
    }
  }, [missionLegs, activeLegIndex]);

  const routeSegments = useMemo(() => {
    if (!missionLegs.length) {
      return routePath && routePath.length
        ? [{ path: routePath, legIndex: 0, meters: missionTotalMeters ?? 0, targetCount: missionTargets.length }]
        : [];
    }
    return missionLegs.map((leg) => ({
      path: leg.path,
      legIndex: leg.index,
      meters: leg.meters,
      targetCount: leg.targetCount,
    }));
  }, [missionLegs, routePath, missionTotalMeters, missionTargets.length]);

  const filteredSegments = useMemo(() => {
    if (activeLegIndex == null) return routeSegments;
    return routeSegments.filter((segment) => segment.legIndex === activeLegIndex);
  }, [routeSegments, activeLegIndex]);

  const visibleBreakpoints = useMemo(() => {
    if (!missionLegs.length) return [];
    const sourceLegs = activeLegIndex == null
      ? missionLegs
      : missionLegs.filter((leg) => leg.index === activeLegIndex);
    return sourceLegs
      .filter((leg) => leg.targetCount > 0 && Array.isArray(leg.targets) && leg.targets.length)
      .map((leg) => ({
        position: leg.targets[0].slice(),
        legIndex: leg.index,
      }));
  }, [missionLegs, activeLegIndex]);

  const longestLegIndex = useMemo(() => {
    if (!missionLegs.length || missionLongestLeg == null) return -1;
    const target = missionLongestLeg;
    return missionLegs.findIndex((leg) => Math.abs(leg.meters - target) < 1e-6);
  }, [missionLegs, missionLongestLeg]);

  const legUsageRatio = missionMaxMeters > 0 && missionLongestLeg != null
    ? Math.min(1, missionLongestLeg / missionMaxMeters)
    : 0;
  const batteryUsagePercent = Math.round(legUsageRatio * 100);
  const missionLegCount = missionLegs.length;
  const activeLegDetails = activeLegIndex == null
    ? null
    : missionLegs.find((leg) => leg.index === activeLegIndex) ?? null;
  const legFilterLabel = activeLegIndex == null ? "All" : `Leg ${activeLegIndex + 1}`;
  const activeLegDistanceKm = activeLegDetails ? (activeLegDetails.meters / 1000).toFixed(2) : null;
  const activeLegTargets = activeLegDetails ? activeLegDetails.targetCount : null;

  const targetSummary = useMemo(() => {
    if (!missionTargets.length) {
      return {
        counts: [],
        total: 0,
        uniqueLocations: 0,
        primaryCoord: null,
      };
    }
    const countsMap = new Map();
    const uniqueCoords = new Set();
    missionTargets.forEach((target) => {
      const type = target?.type || "unknown";
      countsMap.set(type, (countsMap.get(type) || 0) + 1);
      if (Array.isArray(target?.coord) && target.coord.length === 2) {
        const key = `${target.coord[0].toFixed(5)}|${target.coord[1].toFixed(5)}`;
        uniqueCoords.add(key);
      }
    });
    const counts = Array.from(countsMap.entries())
      .map(([type, count]) => ({ type, count }))
      .sort((a, b) => b.count - a.count);
    return {
      counts,
      total: missionTargets.length,
      uniqueLocations: uniqueCoords.size,
      primaryCoord: missionTargets[0]?.coord ?? null,
    };
  }, [missionTargets]);

  // Build route + timestamps
  const trip = useMemo(() => pathToTrip(routePath, speed), [routePath, speed]);

  const { segmentDistances, routeTotalMeters } = useMemo(() => {
    if (!Array.isArray(routePath) || routePath.length < 2) {
      return { segmentDistances: [], routeTotalMeters: 0 };
    }
    const segs = [];
    let total = 0;
    for (let i = 1; i < routePath.length; i += 1) {
      const segDistance = haversine(routePath[i - 1], routePath[i]);
      segs.push(segDistance);
      total += segDistance;
    }
    return { segmentDistances: segs, routeTotalMeters: total };
  }, [routePath]);

  const distanceCoveredMeters = useMemo(() => {
    const times = trip.timestamps;
    if (!Array.isArray(times) || times.length < 2 || routeTotalMeters <= 0) return 0;
    const totalTime = times[times.length - 1];
    if (!Number.isFinite(totalTime) || totalTime <= 0) return 0;
    const currentTime = ((time % totalTime) + totalTime) % totalTime;
    let covered = 0;
    for (let i = 1; i < times.length; i += 1) {
      const start = times[i - 1];
      const end = times[i];
      const segDistance = segmentDistances[i - 1] ?? 0;
      if (currentTime >= end) {
        covered += segDistance;
        continue;
      }
      if (currentTime > start) {
        const span = Math.max(1e-9, end - start);
        const ratio = (currentTime - start) / span;
        covered += segDistance * ratio;
      }
      break;
    }
    return Math.min(routeTotalMeters, covered);
  }, [segmentDistances, time, trip.timestamps, routeTotalMeters]);

  const distanceRemainingMeters = Math.max(0, routeTotalMeters - distanceCoveredMeters);

  const legBoundaries = useMemo(() => {
    if (!missionLegs.length) return [];
    let cursor = 0;
    return missionLegs.map((leg) => {
      const start = cursor;
      const meters = Number.isFinite(leg.meters) ? leg.meters : 0;
      cursor += meters;
      return { legIndex: leg.index, start, end: cursor };
    });
  }, [missionLegs]);

  const currentRouteLegIndex = useMemo(() => {
    if (!missionLegs.length) return null;
    const epsilon = 1e-3;
    for (const boundary of legBoundaries) {
      if (distanceCoveredMeters <= boundary.end + epsilon) {
        return boundary.legIndex;
      }
    }
    return missionLegs[missionLegs.length - 1]?.index ?? null;
  }, [distanceCoveredMeters, legBoundaries, missionLegs]);

  const currentLegDisplay = missionLegs.length
    ? `Leg ${(currentRouteLegIndex ?? 0) + 1} / ${missionLegCount || 1}`
    : "Single Hop";

  const currentLegNote = missionLegs.length
    ? `${missionLegCount || 1} planned`
    : "All targets one sortie";

  const totalMetersFallback = routeTotalMeters || missionTotalMeters || 0;
  const totalKilometersPlanned = totalMetersFallback > 0 ? (totalMetersFallback / 1000).toFixed(2) : null;
  const distanceCoveredLabel = totalMetersFallback > 0
    ? `${(distanceCoveredMeters / 1000).toFixed(2)} km`
    : "--";
  const distanceCoveredNote = totalKilometersPlanned != null ? `of ${totalKilometersPlanned} km` : "Awaiting plan";
  const distanceRemainingLabel = totalMetersFallback > 0
    ? `${(distanceRemainingMeters / 1000).toFixed(2)} km`
    : "--";
  const distanceRemainingNote = totalMetersFallback > 0 ? "Remaining" : "Standby";

  const activeLegPrimaryCoord = Array.isArray(activeLegDetails?.targets) && activeLegDetails.targets.length
    ? activeLegDetails.targets[0]
    : null;
  const nextWaypointCoord = activeLegPrimaryCoord || targetSummary.primaryCoord;
  const nextWaypointLabel = nextWaypointCoord
    ? `${nextWaypointCoord[0].toFixed(3)}, ${nextWaypointCoord[1].toFixed(3)}`
    : "--";
  const nextWaypointNote = activeLegPrimaryCoord
    ? `Leg ${(activeLegDetails?.index ?? 0) + 1}`
    : (targetSummary.counts[0]
      ? `${(targetSummary.counts[0].type || 'unknown').toUpperCase()} focus`
      : "Awaiting queue");
  const activeLoadValue = activeLegTargets != null
    ? `${activeLegTargets} targets`
    : `${targetSummary.total} total`;
  const activeLoadNote = activeLegDetails
    ? `${(activeLegDetails.meters / 1000).toFixed(2)} km arc`
    : (missionLegs.length ? `${missionLegCount || 1} legs queued` : "Single sortie");
  const uniqueCoordinateLabel = targetSummary.uniqueLocations ? targetSummary.uniqueLocations.toLocaleString() : "0";
  const uniqueCoordinateNote = targetSummary.total
    ? `${targetSummary.total.toLocaleString()} markers`
    : "No targets loaded";
  const primaryTypeEntry = targetSummary.counts[0] ?? null;
  const primaryTypeLabel = primaryTypeEntry
    ? (primaryTypeEntry.type || 'unknown').toUpperCase()
    : "--";
  const primaryTypeNote = primaryTypeEntry
    ? `${primaryTypeEntry.count.toLocaleString()} detections`
    : "Awaiting intel";
  const targetComposition = useMemo(() => {
    if (!targetSummary.total) return [];
    return targetSummary.counts.map((entry) => ({
      ...entry,
      percent: Math.round((entry.count / targetSummary.total) * 100),
    }));
  }, [targetSummary]);

  // Animation loop
  const startTsRef = useRef(null);
  const lastFrameRef = useRef(0);
  useEffect(() => {
    let raf = 0;
    function loop(ts) {
      if (!startTsRef.current) startTsRef.current = ts;
      const elapsed = (ts - startTsRef.current) / 1000; // seconds
      const duration = trip.timestamps[trip.timestamps.length - 1] || 1;
      const baseRate = 1;
      const rate = baseRate * (speed / 12);
      if (playing) {
        setTime((prev) => (prev + (elapsed - lastFrameRef.current) * rate) % duration);
      }
      lastFrameRef.current = elapsed;
      raf = requestAnimationFrame(loop);
    }
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, trip.timestamps]);

  useEffect(() => {
    const id = setInterval(() => setClock(new Date().toLocaleTimeString()), 1000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
    if (!Array.isArray(flightPolygons) || !flightPolygons.length) return;
    const first = flightPolygons[0]?.[0]?.[0];
    if (Array.isArray(first) && first.length === 2) {
      const next = [first[0], first[1]];
      if (!baseCoord || next[0] !== baseCoord[0] || next[1] !== baseCoord[1]) {
        setBaseCoord(next);
      }
    }
  }, [flightPolygons, baseCoord]);

  // Drone position + trail polyline for current time
  const dronePosition = useMemo(() => positionAtTime(trip.path, trip.timestamps, time), [trip, time]);
  const droneTrail = useMemo(() => trailPath(trip.path, trip.timestamps, time, trail), [trip, time, trail]);

  // Telemetry derived values (simulated from trip/time)
  const duration = trip.timestamps[trip.timestamps.length - 1] || 1;
  const progress = Math.max(0, Math.min(1, (time % duration) / duration));
  // battery: decrease from 100 to ~20 across mission
  const battery = Math.max(12, Math.round(100 - progress * 80));
  // altitude: simulate a gentle sinewave between 45m and 120m
  const altitude = Math.round(80 + Math.sin(time * 0.1) * 10);
  // heading: angle towards next point (approx)
  const heading = useMemo(() => {
    // pick next small offset time to get a vector
    const next = positionAtTime(trip.path, trip.timestamps, time + 0.5);
    const [lon1, lat1] = dronePosition;
    const [lon2, lat2] = next;
    const dx = lon2 - lon1;
    const dy = lat2 - lat1;
    const ang = (Math.atan2(dy, dx) * 180) / Math.PI; // degrees
    return Math.round((ang + 360) % 360);
  }, [dronePosition, trip, time]);
  // signal: small jitter around 80-100
  const signal = 70 + Math.round(20 * Math.abs(Math.sin(time * 0.8)));

  const nextActionItems = useMemo(() => {
    const actions = [];
    if (activeLegDetails) {
      const legKm = (activeLegDetails.meters / 1000).toFixed(2);
      actions.push(`Sustain Leg ${activeLegDetails.index + 1} coverage (${legKm} km segment)`);
    } else if (missionLegs.length > 0) {
      const firstLeg = missionLegs[0];
      actions.push(`Prep Leg ${firstLeg.index + 1} launch (${(firstLeg.meters / 1000).toFixed(2)} km)`);
    } else {
      actions.push("Validate single-hop sortie timing");
    }

    if (distanceRemainingMeters > 0) {
      actions.push(`Monitor remaining ${distanceRemainingLabel} corridor`);
    } else {
      actions.push("Initiate RTB sequence and archive telemetry");
    }

    if (battery <= 35) {
      actions.push("Schedule battery swap upon leg completion");
    } else if (signal < 65) {
      actions.push("Check uplink stability — signal trending low");
    } else {
      actions.push("Maintain payload recording for post-flight review");
    }

    return actions.slice(0, 3);
  }, [activeLegDetails, missionLegs, distanceRemainingMeters, distanceRemainingLabel, battery, signal]);

  // ---- DeckGL Layers ----
  const primaryLayers = useMemo(() => [
    new PolygonLayer({
      id: "flight-area",
      data: flightPolygons,
      getPolygon: (d) => d,
      getFillColor: [255, 0, 0, 30],
      getLineColor: [255, 60, 60],
      lineWidthMinPixels: 1,
      stroked: true,
      pickable: true,
    }),
    new PathLayer({
      id: "trail",
      data: [{ path: droneTrail }],
      getPath: (d) => d.path,
      getColor: [0, 200, 255, 180],
      widthMinPixels: 4,
      jointRounded: true,
      capRounded: true,
    }),
    new PathLayer({
      id: "route",
      data: filteredSegments,
      getPath: (d) => d.path,
      getColor: (d) => {
        const legIndex = d.legIndex ?? 0;
        const [r, g, b] = hslToRgb(((legIndex * 59) % 360) / 360, 0.6, 0.55);
        return [r, g, b, 140];
      },
      widthMinPixels: 3,
      jointRounded: true,
      capRounded: true,
    }),
    new ScatterplotLayer({
      id: "leg-breakpoints",
      data: visibleBreakpoints,
      getPosition: (d) => d.position,
      getFillColor: (d) => {
        const legIndex = d.legIndex ?? 0;
        const [r, g, b] = hslToRgb(((legIndex * 59) % 360) / 360, 0.7, 0.55);
        return [r, g, b, 220];
      },
      getLineColor: [12, 20, 28, 200],
      lineWidthMinPixels: 2,
      stroked: true,
      getRadius: 5,
      radiusMinPixels: 5,
      radiusMaxPixels: 11,
      pickable: true,
    }),
    new ScatterplotLayer({
      id: "base",
      data: baseCoord ? [{ position: baseCoord }] : [],
      getPosition: (d) => d.position,
      getFillColor: [0, 255, 120],
      getRadius: 6,
      radiusMinPixels: 6,
      radiusMaxPixels: 12,
      pickable: true,
    }),
    new ScatterplotLayer({
      id: "poles",
      data: missionTargets,
      getPosition: (d) => d.coord,
      getFillColor: (d) => getTargetColor(d?.type),
      getLineColor: [8, 16, 24, 180],
      lineWidthMinPixels: 1,
      getRadius: 5,
      radiusMinPixels: 4,
      radiusMaxPixels: 10,
      pickable: true,
    }),
    
    new ScatterplotLayer({
      id: "drone",
      data: [{ position: dronePosition }],
      getPosition: (d) => d.position,
      getFillColor: [0, 200, 255],
      getRadius: 7,
      radiusMinPixels: 7,
      radiusMaxPixels: 14,
    }),
  ], [droneTrail, filteredSegments, visibleBreakpoints, baseCoord, missionTargets, flightPolygons, dronePosition]);

  const [viewState, setViewState] = useState(INITIAL_VIEW_STATE);
  useEffect(() => {
    if (!baseCoord) return;
    setViewState((vs) => ({
      ...vs,
      longitude: baseCoord[0],
      latitude: baseCoord[1],
    }));
  }, [baseCoord]);
  useEffect(() => {
    if (follow) {
      const [lon, lat] = dronePosition;
      setViewState((vs) => ({
        ...vs,
        longitude: lon,
        latitude: lat,
        // subtle smoothing when following
        transitionDuration: 200,
      }));
    }
  }, [follow, dronePosition]);

  // apply theme to document for CSS variables and map style
  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
  }, [theme]);

  // Load solver routes if available
  useEffect(() => {
    let cancelled = false;
    fetch('/routes.json', { cache: 'no-store' })
      .then((res) => (res.ok ? res.json() : null))
      .then((json) => {
        if (!cancelled && json && Array.isArray(json.legs)) setRoutes(json);
      })
      .catch((e) => console.warn('routes.json not loaded', e));
    return () => { cancelled = true; };
  }, []);

  // Load dataset targets (assets/photos) into poles if available
  useEffect(() => {
    let cancelled = false;
    fetch('/targets.json', { cache: 'no-store' })
      .then((res) => (res.ok ? res.json() : null))
      .then((json) => {
        if (!cancelled && json && Array.isArray(json.targets)) {
          const fallbackType = normalizeTargetType(json.source, "unknown");
          const items = json.targets
            .filter((t) => Array.isArray(t.coord) && t.coord.length === 2)
            .map((t) => ({
              id: t.id ?? t.node ?? Math.random(),
              coord: t.coord,
              type: normalizeTargetType(t.type ?? t.category ?? t.kind, fallbackType),
            }));
          if (items.length) setPoles(items);
        }
      })
      .catch(() => {});
    return () => { cancelled = true; };
  }, []);

  // Load airspace boundary polygon if present in /public
  useEffect(() => {
    let cancelled = false;
    fetch('/polygon_lon_lat.wkt', { cache: 'no-store' })
      .then((res) => (res.ok ? res.text() : null))
      .then((text) => {
        if (cancelled || !text) return;
        const parsed = parsePolygonWkt(text);
        if (Array.isArray(parsed) && parsed.length) {
          setFlightPolygons(parsed);
        }
      })
      .catch(() => {});
    return () => { cancelled = true; };
  }, []);

  // Preprocess routes: decimate paths and prepare for a single PathLayer
  useEffect(() => {
    if (!routes) { setRoutesData(null); return; }
    const maxPerLeg = 300; // cap vertices per leg for perf
    const simplify = (coordsRaw) => {
      const coords = coordsRaw;
      const n = coords.length;
      if (n <= maxPerLeg) return coords;
      const stride = Math.ceil(n / maxPerLeg);
      const out = [];
      for (let i = 0; i < n; i += stride) out.push(coords[i]);
      if (out[out.length - 1][0] !== coords[n - 1][0] || out[out.length - 1][1] !== coords[n - 1][1]) {
        out.push(coords[n - 1]);
      }
      return out;
    };
    const data = routes.legs.map((leg) => ({ path: simplify(leg.coords || []) }));
    // compute bounds for zoom-to-fit
    let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
    for (const item of data) {
      for (const [lon, lat] of item.path) {
        if (lon < minLon) minLon = lon;
        if (lat < minLat) minLat = lat;
        if (lon > maxLon) maxLon = lon;
        if (lat > maxLat) maxLat = lat;
      }
    }
    setRoutesData({ data, bounds: { minLon, minLat, maxLon, maxLat } });
  }, [routes]);

  // Keyboard shortcut: press "f" to toggle follow camera
  useEffect(() => {
    const onKey = (e) => {
      if (e.key.toLowerCase() === 'f') {
        setFollow((prev) => !prev);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  // Map ref to fit bounds
  const mapRef = useRef(null);
  const hoverFrameRef = useRef(null);
  const hoverPendingRef = useRef(null);

  const handleHover = useCallback((info) => {
    if (info && Array.isArray(info.coordinate)) {
      const [lon, lat] = info.coordinate;
      hoverPendingRef.current = [lon, lat];
    } else {
      hoverPendingRef.current = null;
    }

    if (hoverFrameRef.current == null) {
      hoverFrameRef.current = requestAnimationFrame(() => {
        const next = hoverPendingRef.current ? [...hoverPendingRef.current] : null;
        hoverFrameRef.current = null;
        setHoverCoord(next);
      });
    }
  }, []);

  useEffect(() => () => {
    if (hoverFrameRef.current != null) {
      cancelAnimationFrame(hoverFrameRef.current);
      hoverFrameRef.current = null;
    }
  }, []);

  const handleSelectLeg = useCallback((index) => {
    setActiveLegIndex((prev) => (prev === index ? null : index));
  }, []);

  const clearLegFilter = useCallback(() => {
    setActiveLegIndex(null);
  }, []);

  const handleVideoError = useCallback((vehicleId) => {
    if (!vehicleId) return;
    setVideoErrors((prev) => (
      prev[vehicleId] ? prev : { ...prev, [vehicleId]: true }
    ));
  }, []);

  const clearVideoError = useCallback((vehicleId) => {
    if (!vehicleId) return;
    setVideoErrors((prev) => {
      if (!prev[vehicleId]) return prev;
      const next = { ...prev };
      delete next[vehicleId];
      return next;
    });
  }, []);

  const zoomToRoutes = () => {
    if (!routesData || !mapRef.current) return;
    const map = mapRef.current.getMap ? mapRef.current.getMap() : null;
    if (!map) return;
    const { minLon, minLat, maxLon, maxLat } = routesData.bounds;
    try {
      map.fitBounds([[minLon, minLat], [maxLon, maxLat]], { padding: 40, duration: 600 });
    }
  // eslint-disable-next-line no-empty, no-unused-vars
  catch (e) {}
  };

  // Auto-zoom once when routes load
  useEffect(() => {
    if (routesData && showSolver) {
      zoomToRoutes();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [routesData]);

  // Build PathLayers for solver legs (if any)
  const solverLayer = useMemo(() => {
    if (!routesData || !showSolver) return null;
    return new PathLayer({
      id: 'solver-legs',
      data: routesData.data,
      getPath: (d) => d.path,
      getColor: (d, {index}) => {
        const [r,g,b] = hslToRgb(((index * 57) % 360) / 360, 0.7, 0.55);
        return [r, g, b, 180];
      },
      widthMinPixels: 3,
      jointRounded: true,
      capRounded: true,
      pickable: false,
    });
  }, [routesData, showSolver]);

  const deckLayers = useMemo(
    () => (solverLayer ? [...primaryLayers, solverLayer] : primaryLayers),
    [primaryLayers, solverLayer],
  );

  // tiny HSL->RGB helper returning [r,g,b]
  function hslToRgb(h, s, l) {
    let r, g, b;
    if (s === 0) {
      r = g = b = l; // achromatic
    } else {
      const hue2rgb = (p, q, t) => {
        if (t < 0) t += 1;
        if (t > 1) t -= 1;
        if (t < 1 / 6) return p + (q - p) * 6 * t;
        if (t < 1 / 2) return q;
        if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
        return p;
      };
      const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
      const p = 2 * l - q;
      r = hue2rgb(p, q, h + 1 / 3);
      g = hue2rgb(p, q, h);
      b = hue2rgb(p, q, h - 1 / 3);
    }
    return [Math.round(r * 255), Math.round(g * 255), Math.round(b * 255)];
  }

  const statusLabel = battery > 45 ? "Nominal" : battery > 25 ? "Reserve" : "Critical";
  const missionSubtitle = showSolver ? "VRP Overlay • Tactical Grid" : "Simulation Only";
  const solverDistance = useMemo(() => {
    if (!routes) return null;

    // Try common fields set by the solver output (best-effort)
    const candidate =
      routes.tsp?.meters ??
      routes.tspMeters ??
      routes.tsp_distance ??
      routes.totals?.tspMeters ??
      routes.totals?.tsp?.meters ??
      null;

    if (typeof candidate === "number") return candidate;

    // Fallback: sum legs that are explicitly marked as TSP (if solver tags legs)
    if (Array.isArray(routes.legs)) {
      const tspLegs = routes.legs.filter(
        (l) =>
          l.solver === "tsp" ||
          l.type === "tsp" ||
          (typeof l.solver === "string" && l.solver.toLowerCase() === "tsp")
      );
      if (tspLegs.length) {
        return tspLegs.reduce((acc, l) => acc + (l.meters ?? l.distance ?? 0), 0);
      }
    }

    // If no explicit TSP info, return null so UI doesn't show total
    return null;
  }, [routes]);

  const displayedTotalMeters = missionTotalMeters > 0 ? missionTotalMeters : solverDistance;
  const totalKmLabel = displayedTotalMeters != null ? (displayedTotalMeters / 1000).toFixed(1) : null;
  const longestLegKmLabel = missionLongestLeg != null ? (missionLongestLeg / 1000).toFixed(2) : null;
  const cappedUsagePercent = Number.isFinite(batteryUsagePercent) ? batteryUsagePercent : 0;
  const usagePercentLabel = `${Math.min(100, Math.max(0, cappedUsagePercent))}%`;
  const bottomDistanceLabel = activeLegDetails ? "Active Distance" : "Total Distance";
  const bottomDistanceValue = activeLegDetails
    ? (activeLegDistanceKm != null ? `${activeLegDistanceKm} km` : "--")
    : (displayedTotalMeters != null ? `${(displayedTotalMeters / 1000).toFixed(2)} km` : "--");

  const intelEvents = useMemo(() => {
    const now = clock;
    return [
      { code: "FLT", text: playing ? "Flight computer ENGAGED" : "Flight computer PAUSED", time: now },
      { code: "NAV", text: follow ? "Camera locked to drone" : "Manual pan enabled", time: now },
      { code: "SYS", text: `Battery ${statusLabel}`, time: now },
      { code: "BAT", text: `Longest leg running ${usagePercentLabel} of cap`, time: now },
      { code: "LEG", text: activeLegIndex == null ? "All legs visible" : `Focused on leg ${activeLegIndex + 1}`, time: now },
      { code: "RTK", text: `${missionLegCount || 1} battery legs online`, time: now },
    ];
  }, [playing, follow, statusLabel, usagePercentLabel, missionLegCount, activeLegIndex, clock]);

  const activeVehicle = useMemo(
    () => DEPLOYED_VEHICLES.find((v) => v.id === activeVehicleId) ?? null,
    [activeVehicleId],
  );
  const activeVehicleStatusClass = activeVehicle
    ? (activeVehicle.status || '').toLowerCase().replace(/\s+/g, '-')
    : '';
  const activeStream = activeVehicle?.stream ?? null;
  const activeStreamError = activeVehicle ? Boolean(videoErrors[activeVehicle.id]) : false;

  useEffect(() => {
    if (activeVehicleId && activeStream) {
      clearVideoError(activeVehicleId);
    }
  }, [activeVehicleId, activeStream, clearVideoError]);

  const channelMetrics = activeVehicle
    ? [
        { label: "Tasking", value: activeVehicle.task },
        { label: "Charge", value: `${activeVehicle.battery}%` },
        { label: "Link", value: activeVehicle.uplink },
        { label: "Altitude", value: activeVehicle.altitude },
        { label: "Payload", value: activeVehicle.payload },
      ]
    : [];

  const primaryStats = [
    { label: "Altitude", value: `${altitude} m`, tag: "AGL" },
    { label: "Cruise", value: `${speed} m/s`, tag: "TARGET" },
    { label: "Battery", value: `${battery}%`, tag: statusLabel },
    { label: "Signal", value: `${signal}%`, tag: signal > 80 ? "Secure" : signal > 55 ? "Stable" : "Degraded" },
    { label: "Heading", value: `${heading}°`, tag: "TRUE" },
    { label: "Progress", value: `${Math.round(progress * 100)}%`, tag: "MISSION" },
  ];

  const missionMetrics = [
    { label: "Targets", value: missionTargets.length },
    { label: "Legs", value: missionLegCount || 1 },
    totalKmLabel != null ? { label: "Total Km", value: totalKmLabel } : null,
    longestLegKmLabel != null ? { label: "Longest Km", value: longestLegKmLabel } : null,
    activeLegDistanceKm != null ? { label: "Active Km", value: activeLegDistanceKm } : null,
    activeLegTargets != null ? { label: "Active Targets", value: activeLegTargets } : null,
    { label: "Cap Usage", value: usagePercentLabel },
  ].filter(Boolean);

  return (
    <div className="app">
      <header className="command-bar">
        <div className="mission-stack">
          <div className="mission-pill">DASHBOARD</div>
          <div className={`status-dot ${playing ? '' : 'status-paused'}`} />
          <div className="mission-meta">
            <h1>flite</h1>
            <div className="subtitle">{missionSubtitle}</div>
          </div>
        </div>
        <div className="command-actions">
          <div className="hud-clock">{clock}</div>
          <button
            onClick={() => setTheme((t) => (t === 'dark' ? 'light' : 'dark'))}
            className="btn btn-ghost"
            title="Toggle color scheme"
          >
            {theme === 'dark' ? 'Dark Mode' : 'Light Mode'}
          </button>
          <button
            onClick={() => setIntelOpen((prev) => !prev)}
            className="btn"
            aria-pressed={intelOpen}
            title={intelOpen ? 'Collapse intel stack' : 'Expand intel stack'}
          >
            {intelOpen ? 'Hide Intel' : 'Show Intel'}
          </button>
        </div>
      </header>

      <div className="command-body">
        <aside className="sidebar">
          <section className="sidebar-module">
            <header className="module-header">
              <span className="module-title">Flight Telemetry</span>
              <span className="module-note">Live readouts</span>
            </header>
            <div className="dial-grid">
              {primaryStats.map((stat) => (
                <div key={stat.label} className="dial-card">
                  <span className="dial-label">{stat.label}</span>
                  <span className="dial-value">{stat.value}</span>
                  <span className="dial-tag">{stat.tag}</span>
                </div>
              ))}
            </div>
          </section>

          <section className="sidebar-module">
            <header className="module-header">
              <span className="module-title">Mission Controls</span>
              <span className="module-note">Adjustments deck</span>
            </header>
            <div className="control-deck">
              <div className="control-button-grid">
                <button onClick={() => setPlaying((p) => !p)} className="btn btn-primary control-btn">
                  {playing ? 'Pause' : 'Resume'}
                </button>
                <button onClick={() => setTime(0)} className="btn control-btn">
                  Reset Time
                </button>
                <button
                  onClick={() => setFollow((f) => !f)}
                  className="btn control-btn"
                  aria-pressed={follow}
                >
                  {follow ? 'Disengage Follow' : 'Lock Follow'}
                </button>
                {routes && (
                  <button
                    onClick={() => setShowSolver((v) => !v)}
                    className="btn control-btn"
                    aria-pressed={showSolver}
                  >
                    {showSolver ? 'Hide Legs' : 'Show Legs'}
                  </button>
                )}
                {routesData && (
                  <button onClick={zoomToRoutes} className="btn control-btn">
                    Snap to Grid
                  </button>
                )}
              </div>

              <div className="slider-deck">
                <div className="slider-block">
                  <div className="slider-head">
                    <label className="slider-label" htmlFor={speedSliderId}>Cruise Speed</label>
                    <span className="slider-readout">{speed} m/s</span>
                  </div>
                  <input
                    id={speedSliderId}
                    type="range"
                    min={1}
                    max={500}
                    value={speed}
                    onChange={(e) => setSpeed(parseInt(e.target.value, 10))}
                    className="slider"
                  />
                </div>
                <div className="slider-block">
                  <div className="slider-head">
                    <label className="slider-label" htmlFor={trailSliderId}>Trail Window</label>
                    <span className="slider-readout">{trail}s</span>
                  </div>
                  <input
                    id={trailSliderId}
                    type="range"
                    min={5}
                    max={120}
                    value={trail}
                    onChange={(e) => setTrail(parseInt(e.target.value, 10))}
                    className="slider"
                  />
                </div>
              </div>
            </div>
          </section>

          <section className="sidebar-module">
            <header className="module-header">
              <span className="module-title">Mission Metrics</span>
              <span className="module-note">Totals & pacing</span>
            </header>
            <div className="metric-grid">
              {missionMetrics.map((metric) => (
                <div key={metric.label} className="metric-card">
                  <span className="metric-label">{metric.label}</span>
                  <span className="metric-value">{metric.value}</span>
                </div>
              ))}
            </div>
          </section>

          <section className="sidebar-module">
            <header className="module-header module-header-spaced">
              <div className="module-title-stack">
                <span className="module-title">Battery Legs</span>
                <span className="module-note">Segment planner</span>
              </div>
              {missionLegs.length > 0 && activeLegIndex != null && (
                <button type="button" className="btn small control-btn" onClick={clearLegFilter}>
                  Show All
                </button>
              )}
            </header>
            {missionLegs.length === 0 ? (
              <div className="leg-empty">
                <span className="leg-empty-label">Single Hop</span>
                <span className="leg-empty-value">
                  {displayedTotalMeters != null
                    ? `${(displayedTotalMeters / 1000).toFixed(2)} km`
                    : `${(missionTotalMeters / 1000).toFixed(2)} km`}
                </span>
                <span className="leg-empty-note">All targets one sortie</span>
              </div>
            ) : (
              <div className="leg-scroll">
                {missionLegs.map((leg) => {
                  const ratio = missionMaxMeters > 0 ? leg.meters / missionMaxMeters : 0;
                  const capPercent = Math.round(Math.min(1, Math.max(0, ratio)) * 100);
                  const isLongest = leg.index === longestLegIndex;
                  const isSelected = activeLegIndex === leg.index;
                  const metaParts = [
                    `${leg.targetCount} targets`,
                    `${capPercent}% cap`,
                  ];
                  if (isLongest) metaParts.unshift('Peak distance');
                  if (isSelected) metaParts.push('Focused');
                  const metaText = metaParts.join(' • ');

                  const legClasses = ["leg-card"];
                  if (isLongest) legClasses.push("leg-card-longest");
                  if (isSelected) legClasses.push("leg-card-selected");

                  return (
                    <button
                      key={leg.index}
                      type="button"
                      className={legClasses.join(' ')}
                      onClick={() => handleSelectLeg(leg.index)}
                      aria-pressed={isSelected}
                    >
                      <div className="leg-card-header">
                        <span className="leg-id">Leg {leg.index + 1}</span>
                        <span className="leg-distance">{(leg.meters / 1000).toFixed(2)} km</span>
                      </div>
                      <div className="leg-card-meta">{metaText}</div>
                    </button>
                  );
                })}
              </div>
            )}
          </section>
        </aside>

        <main className="map-console">
          <DeckGL
            layers={deckLayers}
            controller={true}
            viewState={viewState}
            onViewStateChange={({ viewState: vs }) => setViewState(vs)}
            getTooltip={({ object, layer }) => {
              if (layer?.id === "poles" && object) {
                const label = getTargetLabel(object.type);
                const coord = Array.isArray(object.coord) ? object.coord : object.position;
                if (Array.isArray(coord) && coord.length === 2) {
                  return `${label}\n${coord[0].toFixed(4)}, ${coord[1].toFixed(4)}`;
                }
                return label;
              }
              if (layer?.id === "leg-breakpoints" && object) {
                const legNumber = (object.legIndex ?? 0) + 1;
                return `Leg ${legNumber} insertion`;
              }
              if (layer?.id === "base" && object) {
                return "Mission Base";
              }
              return null;
            }}
            onHover={handleHover}
          >
            <MapView
              mapLib={maplibregl}
              ref={mapRef}
              reuseMaps
              mapStyle={theme === 'dark' ? MAP_STYLE_DARK : MAP_STYLE_LIGHT}
              attributionControl={false}
            />
          </DeckGL>

          <div className="map-overlay">
            <div className="top-hud">
              <span className="callsign">UAV-09 Tactical Feed</span>
              <span className="badge">Mode {playing ? 'AUTO' : 'MANUAL'}</span>
            </div>
            <div className="map-bottom-bar">
              <div className="map-stat">
                <span className="label">Route Window</span>
                <span className="value">{trail}s</span>
              </div>
              <div className="map-stat">
                <span className="label">Leg View</span>
                <span className="value">{legFilterLabel}</span>
              </div>
              <div className="map-stat">
                <span className="label">{bottomDistanceLabel}</span>
                <span className="value">{bottomDistanceValue}</span>
              </div>
              <div className="map-stat">
                <span className="label">Grid Cursor</span>
                <span className="value">
                  {hoverCoord ? `${hoverCoord[0].toFixed(4)}, ${hoverCoord[1].toFixed(4)}` : '--'}
                </span>
              </div>
            </div>
          </div>
        </main>

        <aside className={`intel-panel ${intelOpen ? '' : 'collapsed'}`}>
          <div className="intel-header">
            <div className="panel-title">Operations Console</div>
            <div className="tab-group">
              <button
                type="button"
                className={`tab-button ${intelTab === 'feed' ? 'active' : ''}`}
                onClick={() => setIntelTab('feed')}
              >
                Intel Feed
              </button>
              <button
                type="button"
                className={`tab-button ${intelTab === 'vehicles' ? 'active' : ''}`}
                onClick={() => setIntelTab('vehicles')}
              >
                Vehicles
              </button>
            </div>
          </div>

          {intelTab === 'feed' ? (
            <>
              <div>
                <div className="panel-title">Mission Status</div>
                <div className="intel-status-grid">
                  <div className="intel-status-card">
                    <span className="intel-status-label">Current Leg</span>
                    <span className="intel-status-value">{currentLegDisplay}</span>
                    <span className="intel-status-note">{currentLegNote}</span>
                  </div>
                  <div className="intel-status-card">
                    <span className="intel-status-label">Distance Covered</span>
                    <span className="intel-status-value">{distanceCoveredLabel}</span>
                    <span className="intel-status-note">{distanceCoveredNote}</span>
                  </div>
                  <div className="intel-status-card">
                    <span className="intel-status-label">Distance Remaining</span>
                    <span className="intel-status-value">{distanceRemainingLabel}</span>
                    <span className="intel-status-note">{distanceRemainingNote}</span>
                  </div>
                </div>
              </div>

              <div>
                <div className="panel-title">Intel Feed</div>
                <div className="intel-log">
                  {intelEvents.map((entry, idx) => (
                    <div key={`${entry.code}-${idx}`} className="entry">
                      <span className="code">[{entry.code}] {entry.text}</span>
                      <span className="time">{entry.time}</span>
                    </div>
                  ))}
                </div>
              </div>

              <div>
                <div className="panel-title">Target Stack</div>
                <div className="target-summary-grid">
                  <div className="target-summary-card">
                    <span className="target-summary-label">Active Load</span>
                    <span className="target-summary-value">{activeLoadValue}</span>
                    <span className="target-summary-note">{activeLoadNote}</span>
                  </div>
                  <div className="target-summary-card">
                    <span className="target-summary-label">Unique Coordinates</span>
                    <span className="target-summary-value">{uniqueCoordinateLabel}</span>
                    <span className="target-summary-note">{uniqueCoordinateNote}</span>
                  </div>
                  <div className="target-summary-card">
                    <span className="target-summary-label">Next Waypoint</span>
                    <span className="target-summary-value">{nextWaypointLabel}</span>
                    <span className="target-summary-note">{nextWaypointNote}</span>
                  </div>
                  <div className="target-summary-card">
                    <span className="target-summary-label">Primary Type</span>
                    <span className="target-summary-value">{primaryTypeLabel}</span>
                    <span className="target-summary-note">{primaryTypeNote}</span>
                  </div>
                </div>
                <div className="target-list target-composition">
                  {targetComposition.length === 0 && <div className="target-line">No targets loaded</div>}
                  {targetComposition.map((entry) => (
                    <div key={entry.type} className="target-line target-composition-row">
                      <span className={`target-type target-type-${entry.type || 'unknown'}`}>
                        {(entry.type || 'unknown').toUpperCase()}
                      </span>
                      <span className="target-count">{entry.count}</span>
                      <span className="target-percent">{entry.percent}%</span>
                    </div>
                  ))}
                </div>
              </div>

              <div>
                <div className="panel-title">Next Actions</div>
                <div className="next-actions-list">
                  {nextActionItems.map((action, index) => (
                    <div key={action} className="next-action-item">
                      <span className="next-action-step">{index + 1}</span>
                      <span className="next-action-text">{action}</span>
                    </div>
                  ))}
                </div>
              </div>
            </>
          ) : (
            <div className="vehicle-pane">
              <div className="vehicle-columns">
                <section className="vehicle-list-card">
                  <div className="list-header">
                    <div className="panel-title">Channels</div>
                    <div className="list-subtitle">Choose source</div>
                  </div>
                  <div className="vehicle-list">
                    {DEPLOYED_VEHICLES.map((vehicle) => {
                      const statusClass = (vehicle.status || '').toLowerCase().replace(/\s+/g, '-');
                      return (
                        <button
                          key={vehicle.id}
                          type="button"
                          className={`vehicle-row ${vehicle.id === activeVehicleId ? 'active' : ''}`}
                          onClick={() => setActiveVehicleId(vehicle.id)}
                        >
                          <span className={`channel-dot ${statusClass ? `status-${statusClass}` : ''}`} aria-hidden="true" />
                          <div className="vehicle-meta">
                            <div className="vehicle-callsign">{vehicle.id}</div>
                            <div className="vehicle-type">{vehicle.type}</div>
                          </div>
                          <span className={`status-chip ${statusClass ? `status-${statusClass}` : ''}`}>
                            {vehicle.status}
                          </span>
                        </button>
                      );
                    })}
                    {DEPLOYED_VEHICLES.length === 0 && <div className="vehicle-empty">No channels available</div>}
                  </div>
                </section>

                {activeVehicle && (
                  <section className="vehicle-channel">
                    <div className="channel-header">
                      <div className="channel-ident">
                        <span className="channel-tag">{activeVehicle.id}</span>
                        <span className="channel-type">{activeVehicle.type}</span>
                      </div>
                      <span className={`channel-status ${activeVehicleStatusClass ? `status-${activeVehicleStatusClass}` : ''}`}>
                        {activeVehicle.status || 'Unknown'}
                      </span>
                    </div>

                    <div className="video-stack">
                      <div className="video-header">Live Feed • {activeStream?.title ?? 'Unavailable'}</div>
                      {activeStream ? (
                        activeStreamError ? (
                          <div className="video-placeholder">
                            <span className="video-error-message">Live Feed Unavailable</span>
                          </div>
                        ) : (
                          <>
                            <div className="video-frame">
                              <video
                                key={`${activeVehicle.id}-${activeStream.url}`}
                                src={activeStream.url}
                                poster={activeStream.poster}
                                autoPlay
                                muted
                                loop
                                playsInline
                                controls
                                onError={() => handleVideoError(activeVehicle.id)}
                                onLoadedData={() => clearVideoError(activeVehicle.id)}
                              >
                                <track kind="captions" />
                              </video>
                              <div className="video-overlay" aria-hidden="true">LIVE</div>
                            </div>
                            {activeStream.note && <div className="channel-note">{activeStream.note}</div>}
                          </>
                        )
                      ) : (
                        <div className="video-placeholder">
                          <span>No feed linked to this asset.</span>
                        </div>
                      )}
                    </div>

                    <div className="channel-meta-grid">
                      {channelMetrics.map((metric) => (
                        <div key={metric.label} className="channel-metric">
                          <span className="label">{metric.label}</span>
                          <span className="value">{metric.value}</span>
                        </div>
                      ))}
                    </div>
                  </section>
                )}
              </div>
            </div>
          )}
        </aside>
      </div>
    </div>
  );
}
