import {
  useEffect,
  useRef,
  useCallback,
  useLayoutEffect,
  useState,
} from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import { Button } from "./ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "./ui/dropdown-menu";
import { ChevronDownIcon, Ruler, Trash2, X } from "lucide-react";
import { useDebug } from "../store";
import { calculatePointBounds, calculateSegmentBounds } from "../lib/bounds";
import {
  buildPredecessorChain,
  getFinalRouteSegmentMatches,
  getLabelSignature,
  getPointMatchSelection,
  getSegmentMatch,
  kInspectorNodeHoverEvent,
  sortDebugNodeLabels,
} from "../lib/segment-debug";
import type { DebugMatch, DebugWay, InputPoint } from "../types";

function haversineDistance(
  pt1: [number, number],
  pt2: [number, number],
): number {
  const R = 6371000; // Earth radius in meters
  const lat1 = (pt1[1] * Math.PI) / 180;
  const lat2 = (pt2[1] * Math.PI) / 180;
  const dLat = ((pt2[1] - pt1[1]) * Math.PI) / 180;
  const dLng = ((pt2[0] - pt1[0]) * Math.PI) / 180;

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLng / 2) * Math.sin(dLng / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

  return R * c;
}

function findClosestPointIdx(
  target: [number, number],
  geometry: [number, number][],
  startIdx: number,
): number {
  let minIdx = startIdx;
  let minDist = Infinity;
  for (let i = startIdx; i < geometry.length; i++) {
    const dist = haversineDistance(target, geometry[i]);
    if (dist <= minDist) {
      minDist = dist;
      minIdx = i;
    }
  }
  return minIdx;
}

function getSegmentGeometryFromOffsets(
  geometry: [number, number][],
  segmentOffsets: number[] | undefined,
  segmentIndex: number,
): [number, number][] | null {
  if (!segmentOffsets || segmentOffsets.length < segmentIndex + 2) {
    return null;
  }

  const startIdx = segmentOffsets[segmentIndex];
  const endIdx = segmentOffsets[segmentIndex + 1];
  if (
    !Number.isInteger(startIdx) ||
    !Number.isInteger(endIdx) ||
    startIdx < 0 ||
    endIdx < startIdx ||
    endIdx > geometry.length
  ) {
    return null;
  }

  const segmentGeometry = geometry.slice(startIdx, endIdx);
  return segmentGeometry.length > 1 ? segmentGeometry : null;
}

function destinationPoint(
  center: [number, number],
  distanceMeters: number,
  bearingRadians: number,
): [number, number] {
  const earthRadius = 6371000;
  const angularDistance = distanceMeters / earthRadius;
  const lng1 = (center[0] * Math.PI) / 180;
  const lat1 = (center[1] * Math.PI) / 180;

  const lat2 = Math.asin(
    Math.sin(lat1) * Math.cos(angularDistance) +
      Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(bearingRadians),
  );
  const lng2 =
    lng1 +
    Math.atan2(
      Math.sin(bearingRadians) * Math.sin(angularDistance) * Math.cos(lat1),
      Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2),
    );

  return [(((lng2 * 180) / Math.PI + 540) % 360) - 180, (lat2 * 180) / Math.PI];
}

function createRadiusPolygon(
  center: [number, number],
  radiusMeters: number,
  steps = 48,
): [number, number][][] | null {
  if (!Number.isFinite(radiusMeters) || radiusMeters <= 0) {
    return null;
  }

  const ring: [number, number][] = [];
  for (let i = 0; i <= steps; i++) {
    ring.push(
      destinationPoint(center, radiusMeters, (i / steps) * Math.PI * 2),
    );
  }
  return [ring];
}

function getFallbackMatchingDistance(
  point: InputPoint,
  matches: DebugMatch[],
): number | null {
  if (matches.length === 0) {
    return null;
  }

  const center: [number, number] = [point.lng, point.lat];
  return matches.reduce((maxDistance, match) => {
    return Math.max(
      maxDistance,
      haversineDistance(center, match.projectedPoint),
    );
  }, 0);
}

function getMatchingDistance(
  point: InputPoint,
  matches: DebugMatch[],
): number | null {
  if (
    typeof point.matchingDistance === "number" &&
    Number.isFinite(point.matchingDistance) &&
    point.matchingDistance > 0
  ) {
    return point.matchingDistance;
  }

  return getFallbackMatchingDistance(point, matches);
}

function getMatchCost(match: DebugMatch): number | null {
  if (
    typeof match.matchPenalty === "number" &&
    Number.isFinite(match.matchPenalty)
  ) {
    return match.matchPenalty;
  }

  if (
    typeof match.startLabel?.matchPenalty === "number" &&
    Number.isFinite(match.startLabel.matchPenalty)
  ) {
    return match.startLabel.matchPenalty;
  }

  return null;
}

function formatMatchCost(match: DebugMatch): string | null {
  const cost = getMatchCost(match);
  return cost === null ? null : cost.toLocaleString("en-US");
}

function formatMatchConnectorLabel(match: DebugMatch): string | null {
  const matchPenalty = formatMatchCost(match);
  const totalStartCost =
    typeof match.startLabel?.totalStartCost === "number" &&
    Number.isFinite(match.startLabel.totalStartCost)
      ? match.startLabel.totalStartCost.toLocaleString("en-US")
      : null;

  if (matchPenalty && totalStartCost) {
    return `${matchPenalty} | ${totalStartCost}`;
  }

  return matchPenalty;
}

function getLineMidpoint(
  start: [number, number],
  end: [number, number],
): [number, number] {
  return [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2];
}

export function Map() {
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<maplibregl.Map | null>(null);
  const hoveredInspectorNodeRef = useRef<number | null>(null);
  const prevViewModeRef = useRef<string | null>(null);
  const prevHighlightedSegmentRef = useRef<number | null>(null);
  const prevSelectedSegmentRef = useRef<number | null>(null);
  const prevHighlightedWayRef = useRef<number | null>(null);
  const prevHighlightedMatchRef = useRef<number | null>(null);
  const [isMeasuring, setIsMeasuring] = useState(false);
  const [measurePoints, setMeasurePoints] = useState<[number, number][]>([]);
  const {
    data,
    viewMode,
    selectedPointIdx,
    selectedSegmentIdx,
    selectedNodeIdx,
    selectedNodeLabelSignature,
    selectedMatchIdx,
    selectedRouteCandidate,
    highlightedSegmentIdx,
    highlightedWayIdx,
    highlightedMatchIdx,
    getBoundingBox,
    getWay,
    getNode,
    showPoint,
    showSegment,
    selectNode,
    selectNodeLabel,
    selectWay,
    selectMatch,
    selectRouteCandidate,
    setHighlightedSegment,
  } = useDebug();

  const selectedSegment =
    data && selectedSegmentIdx !== null
      ? (data.routeSegments.find(
          (segment) => segment.segmentIdx === selectedSegmentIdx,
        ) ?? null)
      : null;
  const selectedSegmentMatch = getSegmentMatch(
    selectedSegment,
    selectedMatchIdx,
  );

  // Initialize map with raster tiles that allow overzoom
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const map = new maplibregl.Map({
      container: containerRef.current,
      style: {
        version: 8,
        sources: {
          carto: {
            type: "raster",
            tiles: [
              "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png",
              "https://b.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png",
              "https://c.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png",
            ],
            tileSize: 256,
            maxzoom: 20, // Tiles available up to this zoom
            attribution: "© CARTO © OpenStreetMap contributors",
          },
        },
        layers: [
          {
            id: "carto-base",
            type: "raster",
            source: "carto",
            minzoom: 0,
            maxzoom: 24, // Allow display up to this zoom (overzoom)
          },
        ],
      },
      center: [8.4, 49.0],
      zoom: 10,
      maxZoom: 24, // Allow zooming beyond tile availability
    });

    map.addControl(new maplibregl.NavigationControl(), "top-right");
    map.addControl(
      new maplibregl.ScaleControl({ unit: "metric" }),
      "bottom-left",
    );
    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // Fit bounds when data loads
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !data) return;

    const bounds = getBoundingBox();
    if (bounds) {
      map.fitBounds(bounds, { padding: 50, duration: 300 });
    }
  }, [data, getBoundingBox]);

  // Fit bounds on selection
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !data) return;

    let bounds: [[number, number], [number, number]] | null = null;

    if (viewMode === "route") {
      if (prevViewModeRef.current !== "route") {
        bounds = getBoundingBox();
      }
    } else if (viewMode === "points" && selectedPointIdx !== null) {
      bounds = calculatePointBounds(data, selectedPointIdx);
    } else if (viewMode === "segment" && selectedSegmentIdx !== null) {
      bounds = calculateSegmentBounds(data, selectedSegmentIdx);
    }

    if (bounds) {
      map.fitBounds(bounds, { padding: 50, animate: true, duration: 300 });
    }

    prevViewModeRef.current = viewMode;
  }, [data, viewMode, selectedPointIdx, selectedSegmentIdx]);

  // All layer IDs we manage (in order: first added = bottom)
  const LAYER_IDS = [
    "segment-overview",
    "segment-overview-beeline",
    "segment-overview-labels",
    "segment-beeline",
    "final-route",
    "final-route-arrows",
    "segment-route",
    "segment-route-arrows",
    "matched-ways",
    "matched-ways-arrows",
    "additional-edges",
    "additional-edges-markers",
    "additional-edges-arrows",
    "unreached-nodes",
    "reached-nodes",
    "start-projected-points",
    "dest-projected-points",
    "input-points",
    "input-point-labels",
    "selected-point-match-connectors",
    "selected-point-match-connectors-labels",
    "selected-point-radius",
    "selected-point-radius-outline",
    "segment-start-match-connectors",
    "segment-start-match-connectors-labels",
    "final-route-start-connector-glow",
    "final-route-start-connector",
    "segment-start-radius",
    "segment-start-radius-outline",
    "segment-dest-match-connectors",
    "segment-dest-match-connectors-labels",
    "final-route-dest-connector-glow",
    "final-route-dest-connector",
    "segment-dest-radius",
    "segment-dest-radius-outline",
    "segment-start",
    "segment-dest",
    "segment-labels",
    "final-route-start-point",
    "final-route-dest-point",
    "selected-predecessor-chain-glow",
    "selected-predecessor-chain",
    "selected-node-highlight",
    "selected-route-candidate",
  ];

  // Update map layers based on view mode and selection
  const updateLayers = useCallback(() => {
    const map = mapRef.current;
    if (!map || !data) return;

    // Ensure arrow image exists
    if (!map.hasImage("arrow")) {
      const width = 24;
      const height = 24;
      const ctx = document.createElement("canvas").getContext("2d");
      if (ctx) {
        ctx.canvas.width = width;
        ctx.canvas.height = height;
        ctx.strokeStyle = "#ffffff"; // White stroke
        ctx.lineWidth = 3;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        ctx.moveTo(8, 6); // Top-leftish
        ctx.lineTo(16, 12); // Tip
        ctx.lineTo(8, 18); // Bottom-leftish
        ctx.stroke();
        const imageData = ctx.getImageData(0, 0, width, height);
        map.addImage("arrow", imageData, { sdf: true });
      }
    }

    // Ensure tick image exists (for additional edge markers)
    if (!map.hasImage("tick")) {
      const width = 24;
      const height = 24;
      const ctx = document.createElement("canvas").getContext("2d");
      if (ctx) {
        ctx.canvas.width = width;
        ctx.canvas.height = height;
        ctx.strokeStyle = "#ffffff"; // White stroke
        ctx.lineWidth = 3;
        ctx.lineCap = "round";
        ctx.beginPath();
        ctx.moveTo(12, 6);
        ctx.lineTo(12, 18);
        ctx.stroke();
        const imageData = ctx.getImageData(0, 0, width, height);
        map.addImage("tick", imageData, { sdf: true });
      }
    }

    // Remove existing debug layers
    for (const id of [...LAYER_IDS].reverse()) {
      if (map.getLayer(id)) map.removeLayer(id);
    }
    for (const id of [...LAYER_IDS].reverse()) {
      if (map.getSource(id)) map.removeSource(id);
    }

    // Helper to add line layer
    const addLineLayer = (
      id: string,
      coordinates: [number, number][] | [number, number][][],
      color: string,
      width: number,
      dasharray?: number[],
    ) => {
      const isMulti = Array.isArray(coordinates[0]?.[0]);
      map.addSource(id, {
        type: "geojson",
        data: isMulti
          ? {
              type: "FeatureCollection",
              features: (coordinates as [number, number][][]).map((coords) => ({
                type: "Feature",
                properties: {},
                geometry: { type: "LineString", coordinates: coords },
              })),
            }
          : {
              type: "Feature",
              properties: {},
              geometry: {
                type: "LineString",
                coordinates: coordinates as [number, number][],
              },
            },
      });
      map.addLayer({
        id,
        type: "line",
        source: id,
        paint: {
          "line-color": color,
          "line-width": width,
          ...(dasharray ? { "line-dasharray": dasharray } : {}),
        },
      });
    };

    // Helper to add circle layer
    const addCircleLayer = (
      id: string,
      features: GeoJSON.Feature[],
      color: string,
      radius: number,
    ) => {
      map.addSource(id, {
        type: "geojson",
        data: { type: "FeatureCollection", features },
      });
      map.addLayer({
        id,
        type: "circle",
        source: id,
        paint: {
          "circle-radius": radius,
          "circle-color": color,
          "circle-stroke-color": "#fff",
          "circle-stroke-width": radius > 6 ? 2 : 1,
        },
      });
    };

    const addInputPointLayers = (
      features: GeoJSON.Feature[],
      radius: number,
      withNumbers = false,
    ) => {
      addCircleLayer("input-points", features, "#ef4444", radius);

      if (!withNumbers || features.length === 0) {
        return;
      }

      map.addLayer({
        id: "input-point-labels",
        type: "symbol",
        source: "input-points",
        layout: {
          "text-field": ["to-string", ["+", ["get", "idx"], 1]],
          "text-size": [
            "interpolate",
            ["linear"],
            ["zoom"],
            10,
            10,
            14,
            12,
            18,
            14,
          ],
          "text-font": ["Open Sans Bold"],
          "text-allow-overlap": true,
          "text-ignore-placement": true,
        },
        paint: {
          "text-color": "#ffffff",
          "text-halo-color": "#7f1d1d",
          "text-halo-width": 2,
        },
      });
    };

    const addRadiusLayer = (
      id: string,
      center: [number, number],
      radiusMeters: number | null,
      fillColor: string,
      outlineColor: string,
    ) => {
      if (radiusMeters === null) {
        return;
      }

      const coordinates = createRadiusPolygon(center, radiusMeters);
      if (!coordinates) {
        return;
      }

      map.addSource(id, {
        type: "geojson",
        data: {
          type: "FeatureCollection",
          features: [
            {
              type: "Feature",
              properties: { radiusMeters },
              geometry: {
                type: "Polygon",
                coordinates,
              },
            },
          ],
        },
      });
      map.addLayer({
        id,
        type: "fill",
        source: id,
        paint: {
          "fill-color": fillColor,
          "fill-opacity": 1,
        },
      });
      map.addLayer({
        id: `${id}-outline`,
        type: "line",
        source: id,
        paint: {
          "line-color": outlineColor,
          "line-width": 1.5,
          "line-opacity": 0.9,
          "line-dasharray": [3, 2],
        },
      });
    };

    const addMatchConnectorLayers = (
      id: string,
      point: InputPoint,
      matches: DebugMatch[],
      lineColor: string,
      labelColor: string,
    ) => {
      if (matches.length === 0) {
        return;
      }

      const lineFeatures: GeoJSON.Feature[] = matches.map((match) => ({
        type: "Feature",
        id: match.matchIdx,
        properties: {
          matchIdx: match.matchIdx,
          wayIdx: match.wayIdx,
          distToWay: match.distToWay,
          matchPenalty: getMatchCost(match),
        },
        geometry: {
          type: "LineString",
          coordinates: [[point.lng, point.lat], match.projectedPoint],
        },
      }));

      const labelFeatures: GeoJSON.Feature[] = matches.map((match) => ({
        type: "Feature",
        id: match.matchIdx,
        properties: {
          matchIdx: match.matchIdx,
          wayIdx: match.wayIdx,
          distToWay: match.distToWay,
          matchPenalty: getMatchCost(match),
          label: formatMatchConnectorLabel(match),
        },
        geometry: {
          type: "Point",
          coordinates: getLineMidpoint(
            [point.lng, point.lat],
            match.projectedPoint,
          ),
        },
      }));

      map.addSource(id, {
        type: "geojson",
        data: { type: "FeatureCollection", features: lineFeatures },
        promoteId: "matchIdx",
      });
      map.addLayer({
        id,
        type: "line",
        source: id,
        paint: {
          "line-color": [
            "case",
            ["boolean", ["feature-state", "hover"], false],
            "#22d3ee",
            lineColor,
          ],
          "line-width": [
            "case",
            ["boolean", ["feature-state", "hover"], false],
            2.25,
            1.25,
          ],
          "line-opacity": [
            "case",
            ["boolean", ["feature-state", "hover"], false],
            0.95,
            0.7,
          ],
          "line-dasharray": [3, 2],
        },
      });

      map.addSource(`${id}-labels`, {
        type: "geojson",
        data: { type: "FeatureCollection", features: labelFeatures },
        promoteId: "matchIdx",
      });
      map.addLayer({
        id: `${id}-labels`,
        type: "symbol",
        source: `${id}-labels`,
        layout: {
          "text-field": ["coalesce", ["get", "label"], ""],
          "text-anchor": "center",
          "text-size": [
            "interpolate",
            ["linear"],
            ["zoom"],
            11,
            12,
            15,
            13,
            18,
            14,
          ],
          "text-allow-overlap": true,
          "text-ignore-placement": true,
          "text-offset": [0, -0.4],
        },
        paint: {
          "text-color": [
            "case",
            ["boolean", ["feature-state", "hover"], false],
            "#ecfeff",
            labelColor,
          ],
          "text-halo-color": "#020617",
          "text-halo-width": 4,
          "text-halo-blur": 1,
        },
      });
    };

    const addFinalRouteMatchHighlight = (
      pointId: string,
      connectorId: string,
      point: InputPoint,
      match: DebugMatch,
      connectorColor: string,
      pointColor: string,
    ) => {
      map.addSource(connectorId, {
        type: "geojson",
        data: {
          type: "Feature",
          properties: {
            matchIdx: match.matchIdx,
            wayIdx: match.wayIdx,
            distToWay: match.distToWay,
          },
          geometry: {
            type: "LineString",
            coordinates: [[point.lng, point.lat], match.projectedPoint],
          },
        },
      });
      map.addLayer({
        id: `${connectorId}-glow`,
        type: "line",
        source: connectorId,
        paint: {
          "line-color": connectorColor,
          "line-width": 7,
          "line-opacity": 0.28,
          "line-blur": 2.2,
        },
      });
      map.addLayer({
        id: connectorId,
        type: "line",
        source: connectorId,
        paint: {
          "line-color": connectorColor,
          "line-width": 1.5,
          "line-opacity": 0.95,
          "line-dasharray": [4, 2],
        },
      });

      map.addSource(pointId, {
        type: "geojson",
        data: {
          type: "Feature",
          properties: {
            matchIdx: match.matchIdx,
            wayIdx: match.wayIdx,
            distToWay: match.distToWay,
          },
          geometry: { type: "Point", coordinates: match.projectedPoint },
        },
      });
      map.addLayer({
        id: pointId,
        type: "circle",
        source: pointId,
        paint: {
          "circle-radius": 8.5,
          "circle-color": pointColor,
          "circle-stroke-color": "#f8fafc",
          "circle-stroke-width": 3,
        },
      });
    };

    // Route mode: show final route and input points
    // Route mode: show final route and input points
    if (viewMode === "route") {
      const segmentFeatures: GeoJSON.Feature[] = [];
      const finalRouteGeometry = data.finalRoute.geometry;
      const finalRouteSegmentOffsets = data.finalRoute.segmentOffsets;

      let currentGeoIdx = 0;
      for (const [routeSegmentIndex, seg] of data.routeSegments.entries()) {
        const from = data.inputPoints[seg.fromPointIdx];
        const to = data.inputPoints[seg.toPointIdx];

        let segmentGeom = getSegmentGeometryFromOffsets(
          finalRouteGeometry,
          finalRouteSegmentOffsets,
          routeSegmentIndex,
        );
        let isBeeline = false;

        if (segmentGeom) {
          isBeeline = seg.allBeelined || Boolean(seg.beeline);
          const nextOffset = finalRouteSegmentOffsets?.[routeSegmentIndex + 1];
          if (
            Number.isInteger(nextOffset) &&
            nextOffset !== undefined &&
            nextOffset >= 0
          ) {
            currentGeoIdx = nextOffset;
          }
        } else if (
          seg.allBeelined ||
          seg.beeline ||
          finalRouteGeometry.length === 0
        ) {
          segmentGeom = [
            [from.lng, from.lat],
            [to.lng, to.lat],
          ];
          isBeeline = true;
        } else {
          const startIdx =
            currentGeoIdx === 0
              ? findClosestPointIdx(
                  [from.lng, from.lat],
                  finalRouteGeometry,
                  currentGeoIdx,
                )
              : currentGeoIdx;
          let endIdx = findClosestPointIdx(
            [to.lng, to.lat],
            finalRouteGeometry,
            startIdx,
          );

          if (
            endIdx === startIdx &&
            endIdx + 1 < data.finalRoute.geometry.length
          ) {
            endIdx += 1;
          }

          segmentGeom = finalRouteGeometry.slice(startIdx, endIdx + 1);
          if (segmentGeom.length < 2) {
            segmentGeom = [
              finalRouteGeometry[startIdx] ?? [from.lng, from.lat],
              finalRouteGeometry[endIdx] ?? [to.lng, to.lat],
            ];
          }

          currentGeoIdx = endIdx;
        }

        if (segmentGeom.length > 1) {
          segmentFeatures.push({
            type: "Feature",
            id: seg.segmentIdx,
            properties: {
              segmentIdx: seg.segmentIdx,
              label: `#${seg.segmentIdx}`,
              isBeeline,
            },
            geometry: { type: "LineString", coordinates: segmentGeom },
          });
        }
      }

      if (segmentFeatures.length > 0) {
        map.addSource("final-route", {
          type: "geojson",
          data: { type: "FeatureCollection", features: segmentFeatures },
        });

        // Routed segments (blue)
        map.addLayer({
          id: "final-route",
          type: "line",
          source: "final-route",
          filter: ["==", ["get", "isBeeline"], false],
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#67e8f9",
              "#3b82f6",
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              7,
              4,
            ],
            "line-opacity": 0.95,
          },
        });

        // Beeline segments (red dashed)
        map.addLayer({
          id: "segment-beeline",
          type: "line",
          source: "final-route",
          filter: ["==", ["get", "isBeeline"], true],
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#fda4af",
              "#dc2626",
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              5,
              3,
            ],
            "line-dasharray": [4, 2],
            "line-opacity": 0.95,
          },
        });

        map.addLayer({
          id: "final-route-arrows",
          type: "symbol",
          source: "final-route",
          filter: ["==", ["get", "isBeeline"], false],
          layout: {
            "symbol-placement": "line",
            "symbol-spacing": 100,
            "icon-image": "arrow",
            "icon-size": 0.6,
            "icon-allow-overlap": true,
          },
          paint: {
            "icon-color": "#ffffff",
            "icon-halo-color": "#3b82f6",
            "icon-halo-width": 2,
          },
        });

        map.addLayer({
          id: "segment-labels",
          type: "symbol",
          source: "final-route",
          layout: {
            "text-field": ["get", "label"],
            "symbol-placement": "line-center",
            "text-keep-upright": true,
            "text-max-angle": 35,
            "text-size": [
              "interpolate",
              ["linear"],
              ["zoom"],
              10,
              12,
              14,
              14,
              18,
              16,
            ],
            "text-allow-overlap": true,
            "text-ignore-placement": true,
          },
          paint: {
            "text-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#ffffff",
              ["get", "isBeeline"],
              "#fee2e2",
              "#dbeafe",
            ],
            "text-halo-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#020617",
              ["get", "isBeeline"],
              "#991b1b",
              "#1d4ed8",
            ],
            "text-halo-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              4,
              3,
            ],
          },
        });
      }

      // Input points
      const pointFeatures = data.inputPoints.map((pt, i) => ({
        type: "Feature" as const,
        properties: { idx: i, type: "input" },
        geometry: { type: "Point" as const, coordinates: [pt.lng, pt.lat] },
      }));
      addInputPointLayers(pointFeatures, 8, true);
    }

    // Points mode
    if (viewMode === "points" && selectedPointIdx !== null) {
      const pt = data.inputPoints[selectedPointIdx];
      const { segment, matches } = getPointMatchSelection(
        data,
        selectedPointIdx,
      );

      if (segment) {
        const matchingDistance = getMatchingDistance(pt, matches);

        const wayFeatures: GeoJSON.Feature[] = [];
        const pointFeatures: GeoJSON.Feature[] = [];

        for (const m of matches) {
          const way = data.ways[m.wayIdx];
          if (way?.geometry?.length > 1) {
            wayFeatures.push({
              type: "Feature",
              properties: {
                wayIdx: m.wayIdx,
                matchIdx: m.matchIdx,
                osmId: way.osmId,
                internalId: way.internalId,
              },
              geometry: { type: "LineString", coordinates: way.geometry },
            });
          }
          pointFeatures.push({
            type: "Feature",
            properties: {
              matchIdx: m.matchIdx,
              wayIdx: m.wayIdx,
              distToWay: m.distToWay,
              type: "projected",
            },
            geometry: { type: "Point", coordinates: m.projectedPoint },
          });
        }

        if (wayFeatures.length > 0) {
          map.addSource("matched-ways", {
            type: "geojson",
            data: { type: "FeatureCollection", features: wayFeatures },
            promoteId: "wayIdx",
          });
          map.addLayer({
            id: "matched-ways",
            type: "line",
            source: "matched-ways",
            paint: {
              "line-color": [
                "case",
                ["boolean", ["feature-state", "hover"], false],
                "#22d3ee", // cyan when highlighted
                "#22c55e", // green default
              ],
              "line-width": [
                "case",
                ["boolean", ["feature-state", "hover"], false],
                6, // wider when highlighted
                3, // default
              ],
            },
          });
        }

        addMatchConnectorLayers(
          "selected-point-match-connectors",
          pt,
          matches,
          "#60a5fa",
          "#dbeafe",
        );

        if (pointFeatures.length > 0) {
          map.addSource("start-projected-points", {
            type: "geojson",
            data: { type: "FeatureCollection", features: pointFeatures },
            promoteId: "matchIdx",
          });
          map.addLayer({
            id: "start-projected-points",
            type: "circle",
            source: "start-projected-points",
            paint: {
              "circle-radius": [
                "case",
                ["boolean", ["feature-state", "hover"], false],
                10,
                6,
              ],
              "circle-color": [
                "case",
                ["boolean", ["feature-state", "hover"], false],
                "#22d3ee",
                "#3b82f6",
              ],
              "circle-stroke-color": "#fff",
              "circle-stroke-width": [
                "case",
                ["boolean", ["feature-state", "hover"], false],
                3,
                1, // radius 6 -> stroke 1 matches original logic roughly
              ],
            },
          });
        }

        addRadiusLayer(
          "selected-point-radius",
          [pt.lng, pt.lat],
          matchingDistance,
          "rgba(239, 68, 68, 0.08)",
          "rgba(239, 68, 68, 0.4)",
        );
      }

      const gpsFeature = {
        type: "Feature" as const,
        properties: { type: "gps" },
        geometry: { type: "Point" as const, coordinates: [pt.lng, pt.lat] },
      };
      addInputPointLayers([gpsFeature], 10);
    }

    // Segment mode
    if (viewMode === "segment") {
      const segmentOverviewFeatures = data.routeSegments.map((seg) => {
        const from = data.inputPoints[seg.fromPointIdx];
        const to = data.inputPoints[seg.toPointIdx];
        return {
          type: "Feature" as const,
          id: seg.segmentIdx,
          properties: {
            segmentIdx: seg.segmentIdx,
            label: `#${seg.segmentIdx}`,
            isBeeline: seg.allBeelined || Boolean(seg.beeline),
          },
          geometry: {
            type: "LineString" as const,
            coordinates: [
              [from.lng, from.lat],
              [to.lng, to.lat],
            ],
          },
        };
      });

      if (segmentOverviewFeatures.length > 0) {
        map.addSource("segment-overview", {
          type: "geojson",
          data: {
            type: "FeatureCollection",
            features: segmentOverviewFeatures,
          },
        });

        map.addLayer({
          id: "segment-overview",
          type: "line",
          source: "segment-overview",
          filter: ["==", ["get", "isBeeline"], false],
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              "#f8fafc",
              ["boolean", ["feature-state", "hover"], false],
              "#fde68a",
              "#64748b",
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              6,
              ["boolean", ["feature-state", "hover"], false],
              5,
              3,
            ],
            "line-opacity": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              1,
              ["boolean", ["feature-state", "hover"], false],
              0.95,
              0.7,
            ],
          },
        });

        map.addLayer({
          id: "segment-overview-beeline",
          type: "line",
          source: "segment-overview",
          filter: ["==", ["get", "isBeeline"], true],
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              "#fecdd3",
              ["boolean", ["feature-state", "hover"], false],
              "#fda4af",
              "#ef4444",
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              5,
              ["boolean", ["feature-state", "hover"], false],
              4,
              2.5,
            ],
            "line-dasharray": [4, 2],
            "line-opacity": 0.9,
          },
        });

        map.addLayer({
          id: "segment-overview-labels",
          type: "symbol",
          source: "segment-overview",
          layout: {
            "text-field": ["get", "label"],
            "symbol-placement": "line-center",
            "text-keep-upright": true,
            "text-allow-overlap": true,
            "text-ignore-placement": true,
            "text-size": [
              "interpolate",
              ["linear"],
              ["zoom"],
              9,
              11,
              14,
              13,
              18,
              15,
            ],
          },
          paint: {
            "text-color": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              "#ffffff",
              ["boolean", ["feature-state", "hover"], false],
              "#ffffff",
              ["get", "isBeeline"],
              "#fecaca",
              "#cbd5e1",
            ],
            "text-halo-color": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              "#020617",
              ["get", "isBeeline"],
              "#7f1d1d",
              "#0f172a",
            ],
            "text-halo-width": [
              "case",
              ["boolean", ["feature-state", "selected"], false],
              4,
              3,
            ],
          },
        });
      }

      const pointFeatures = data.inputPoints.map((pt, i) => ({
        type: "Feature" as const,
        properties: { idx: i, type: "input" },
        geometry: { type: "Point" as const, coordinates: [pt.lng, pt.lat] },
      }));
      addInputPointLayers(pointFeatures, 6);

      if (selectedSegmentIdx === null) {
        return;
      }

      const seg =
        data.routeSegments.find(
          (segment) => segment.segmentIdx === selectedSegmentIdx,
        ) ?? null;
      if (!seg) {
        return;
      }

      const fromPt = data.inputPoints[seg.fromPointIdx];
      const toPt = data.inputPoints[seg.toPointIdx];
      const { startMatch: finalStartMatch, destMatch: finalDestMatch } =
        getFinalRouteSegmentMatches(seg);
      const startMatchingDistance = getMatchingDistance(
        fromPt,
        seg.startMatches,
      );
      const destMatchingDistance = getMatchingDistance(toPt, seg.destMatches);

      // Beeline (bottom)
      if (seg.allBeelined || seg.beeline) {
        addLineLayer(
          "segment-beeline",
          [
            [fromPt.lng, fromPt.lat],
            [toPt.lng, toPt.lat],
          ],
          "#dc2626",
          3,
          [4, 2],
        );
      }

      // Route geometries
      const routeCoords: [number, number][][] = [];
      for (const dm of seg.destMatches) {
        if (dm.fwdResult?.reached && dm.fwdResult.geometry) {
          routeCoords.push(dm.fwdResult.geometry);
        }
        if (dm.bwdResult?.reached && dm.bwdResult.geometry) {
          routeCoords.push(dm.bwdResult.geometry);
        }
      }
      if (routeCoords.length > 0) {
        addLineLayer("segment-route", routeCoords, "#8b5cf6", 3);
        map.addLayer({
          id: "segment-route-arrows",
          type: "symbol",
          source: "segment-route",
          layout: {
            "symbol-placement": "line",
            "symbol-spacing": 100,
            "icon-image": "arrow",
            "icon-size": 0.6,
            "icon-allow-overlap": true,
          },
          paint: {
            "icon-color": "#ffffff",
            "icon-halo-color": "#8b5cf6",
            "icon-halo-width": 2,
          },
        });
      }

      // Matched ways - split into regular graph ways and additional edges
      const graphWayFeatures: GeoJSON.Feature[] = [];
      const additionalEdgeFeatures: GeoJSON.Feature[] = [];
      const additionalEdgeMarkerFeatures: GeoJSON.Feature[] = [];
      const visibleWayNodeIndices = new Set<number>();
      const allMatches = [...seg.startMatches, ...seg.destMatches];
      const seenWays = new Set<number>();

      const collectWayNodes = (way: DebugWay) => {
        for (const nodeIdx of way.nodeIndices ?? []) {
          if (Number.isFinite(nodeIdx)) {
            visibleWayNodeIndices.add(nodeIdx);
          }
        }
      };

      // Helper to create a tiny segment centered at 'center' aligned with p1->p2
      const createCenteredSegment = (
        p1: [number, number],
        p2: [number, number],
        center: [number, number],
      ): [number, number][] | null => {
        const dx = p2[0] - p1[0];
        const dy = p2[1] - p1[1];
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 0.0000001) return null;

        const eps = 0.00005; // approx 5m
        const ux = dx / dist;
        const uy = dy / dist;

        const start = [center[0] - ux * eps, center[1] - uy * eps];
        const end = [center[0] + ux * eps, center[1] + uy * eps];

        return [
          [start[0], start[1]],
          [end[0], end[1]],
        ];
      };

      const processAdditionalEdge = (way: DebugWay) => {
        if (!way.geometry || way.geometry.length < 2) return;

        // Start marker: centered at geom[0], aligned with geom[0]->geom[1]
        const startSeg = createCenteredSegment(
          way.geometry[0] as [number, number],
          way.geometry[1] as [number, number],
          way.geometry[0] as [number, number],
        );
        if (startSeg) {
          additionalEdgeMarkerFeatures.push({
            type: "Feature",
            properties: {},
            geometry: { type: "LineString", coordinates: startSeg },
          });
        }

        // End marker: centered at geom[last], aligned with geom[last-1]->geom[last]
        const last = way.geometry.length - 1;
        const endSeg = createCenteredSegment(
          way.geometry[last - 1] as [number, number],
          way.geometry[last] as [number, number],
          way.geometry[last] as [number, number],
        );
        if (endSeg) {
          additionalEdgeMarkerFeatures.push({
            type: "Feature",
            properties: {},
            geometry: { type: "LineString", coordinates: endSeg },
          });
        }
      };

      for (const m of allMatches) {
        if (seenWays.has(m.wayIdx)) continue;
        seenWays.add(m.wayIdx);
        const way = data.ways[m.wayIdx];
        if (way?.geometry?.length > 1) {
          const feature: GeoJSON.Feature = {
            type: "Feature",
            id: m.wayIdx, // Add ID for feature-state
            properties: {
              wayIdx: m.wayIdx,
              osmId: way.osmId,
              internalId: way.internalId,
              isAdditionalEdge: way.isAdditionalEdge,
              reverse: way.reverse ?? false,
            },
            geometry: { type: "LineString", coordinates: way.geometry },
          };

          if (way.isAdditionalEdge) {
            additionalEdgeFeatures.push(feature);
            processAdditionalEdge(way);
          } else {
            graphWayFeatures.push(feature);
          }
          collectWayNodes(way);
        }
      }

      // Also add explicitly listed additional edges from the segment
      if (seg.additionalEdgeWays) {
        for (const wayIdx of seg.additionalEdgeWays) {
          if (seenWays.has(wayIdx)) continue;
          seenWays.add(wayIdx);
          const way = data.ways[wayIdx];
          if (way?.geometry?.length > 1) {
            const feature: GeoJSON.Feature = {
              type: "Feature",
              properties: {
                wayIdx: wayIdx,
                osmId: way.osmId,
                internalId: way.internalId,
                isAdditionalEdge: true,
                reverse: way.reverse ?? false,
              },
              geometry: { type: "LineString", coordinates: way.geometry },
            };
            additionalEdgeFeatures.push(feature);
            processAdditionalEdge(way);
            collectWayNodes(way);
          }
        }
      }

      if (graphWayFeatures.length > 0) {
        map.addSource("matched-ways", {
          type: "geojson",
          data: { type: "FeatureCollection", features: graphWayFeatures },
          promoteId: "wayIdx", // Use wayIdx as feature ID for feature-state
        });
        map.addLayer({
          id: "matched-ways",
          type: "line",
          source: "matched-ways",
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#22d3ee", // cyan when highlighted
              "#6366f1", // indigo default
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              6, // wider when highlighted
              3, // default
            ],
          },
        });
        map.addLayer({
          id: "matched-ways-arrows",
          type: "symbol",
          source: "matched-ways",
          layout: {
            "symbol-placement": "line",
            "symbol-spacing": 100,
            "icon-image": "arrow",
            "icon-size": 0.6,
            "icon-allow-overlap": true,
          },
          paint: {
            "icon-color": "#ffffff",
            "icon-halo-color": "#6366f1",
            "icon-halo-width": 2,
          },
        });
      }

      if (additionalEdgeFeatures.length > 0) {
        map.addSource("additional-edges", {
          type: "geojson",
          data: { type: "FeatureCollection", features: additionalEdgeFeatures },
          promoteId: "wayIdx", // Use wayIdx as feature ID for feature-state
        });
        map.addLayer({
          id: "additional-edges",
          type: "line",
          source: "additional-edges",
          paint: {
            "line-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#22d3ee", // cyan when highlighted
              "#facc15", // yellow default
            ],
            "line-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              6, // wider when highlighted
              3, // default
            ],
            "line-dasharray": [2, 2],
            "line-offset": 10, // Always offset right (forward=right, backward=right-of-reversed=left-of-base)
          },
        });
        map.addLayer({
          id: "additional-edges-arrows",
          type: "symbol",
          source: "additional-edges",
          layout: {
            "symbol-placement": "line",
            "symbol-spacing": 50,
            "icon-image": "arrow",
            "icon-size": 0.5,
            "icon-allow-overlap": true,
            "icon-offset": ["literal", [0, 20]],
          },
          paint: {
            "icon-color": "#ffffff",
            "icon-halo-color": "#facc15",
            "icon-halo-width": 2,
          },
        });
      }

      if (additionalEdgeMarkerFeatures.length > 0) {
        map.addSource("additional-edges-markers", {
          type: "geojson",
          data: {
            type: "FeatureCollection",
            features: additionalEdgeMarkerFeatures,
          },
        });
        map.addLayer({
          id: "additional-edges-markers",
          type: "symbol",
          source: "additional-edges-markers",
          layout: {
            "symbol-placement": "line-center",
            "icon-image": "tick",
            "icon-allow-overlap": true,
            "icon-offset": ["literal", [0, 20]],
          },
          paint: {
            "icon-color": "#ffffff",
            "icon-halo-color": "#facc15",
            "icon-halo-width": 2,
          },
        });
      }

      // Nodes - display all nodes that have labels in this segment
      const reachedNodeFeatures: GeoJSON.Feature[] = [];
      const unreachedNodeFeatures: GeoJSON.Feature[] = [];
      const nodeLabels = seg.nodeLabels || {};

      // Display all nodes with labels in this segment
      for (const [nodeIdxStr, labels] of Object.entries(nodeLabels)) {
        const nodeArrIdx = parseInt(nodeIdxStr, 10);
        const node = data.nodes[nodeArrIdx];
        if (!node) continue;

        const isReached = labels && labels.length > 0;

        const feature: GeoJSON.Feature = {
          type: "Feature",
          properties: {
            nodeIdx: nodeArrIdx,
            internalId: node.internalId,
            osmId: node.osmId,
            isAdditional: node.isAdditionalNode,
            isReached,
          },
          geometry: { type: "Point", coordinates: node.pos },
        };

        if (isReached) {
          reachedNodeFeatures.push(feature);
        } else {
          unreachedNodeFeatures.push(feature);
        }
      }

      // Also add nodes from matches that might not have labels (unreached)
      // allMatches is already declared above
      const displayedNodes = new Set(
        Object.keys(nodeLabels).map((k) => parseInt(k, 10)),
      );
      for (const idx of visibleWayNodeIndices) {
        if (displayedNodes.has(idx)) {
          continue;
        }

        const node = data.nodes[idx];
        if (!node) {
          continue;
        }

        displayedNodes.add(idx);
        unreachedNodeFeatures.push({
          type: "Feature",
          properties: {
            nodeIdx: idx,
            internalId: node.internalId,
            osmId: node.osmId,
            isAdditional: node.isAdditionalNode,
            isReached: false,
          },
          geometry: { type: "Point", coordinates: node.pos },
        });
      }

      for (const m of allMatches) {
        for (const idx of [
          m.additionalNodeIdx,
          m.fwdOut?.nodeIdx,
          m.fwdIn?.nodeIdx,
          m.bwdOut?.nodeIdx,
          m.bwdIn?.nodeIdx,
        ]) {
          if (idx !== undefined && !displayedNodes.has(idx)) {
            const node = data.nodes[idx];
            if (node) {
              displayedNodes.add(idx);
              unreachedNodeFeatures.push({
                type: "Feature",
                properties: {
                  nodeIdx: idx,
                  internalId: node.internalId,
                  osmId: node.osmId,
                  isAdditional: node.isAdditionalNode,
                  isReached: false,
                },
                geometry: { type: "Point", coordinates: node.pos },
              });
            }
          }
        }
      }

      // Add unreached nodes first (beneath reached nodes)
      if (unreachedNodeFeatures.length > 0) {
        addCircleLayer(
          "unreached-nodes",
          unreachedNodeFeatures,
          "#6b7280", // gray
          4,
        );
      }

      // Add reached nodes (cyan/teal color to distinguish from green)
      if (reachedNodeFeatures.length > 0) {
        addCircleLayer(
          "reached-nodes",
          reachedNodeFeatures,
          "#06b6d4", // cyan
          5,
        );
      }

      // Projected points (include startLabel info + match details)
      const startPointFeatures: GeoJSON.Feature[] = seg.startMatches.map(
        (m) => ({
          type: "Feature",
          id: m.matchIdx, // Add ID for feature-state
          properties: {
            matchIdx: m.matchIdx,
            wayIdx: m.wayIdx,
            distToWay: m.distToWay,
            additionalNodeIdx: m.additionalNodeIdx,
            type: "start-projected",
            // Include start label info for popup
            ...(m.startLabel
              ? {
                  matchPenalty: getMatchCost(m),
                  prevSegmentCost: m.startLabel.prevSegmentCost,
                  costOffset: m.startLabel.costOffset,
                  totalStartCost: m.startLabel.totalStartCost,
                }
              : {}),
          },
          geometry: { type: "Point", coordinates: m.projectedPoint },
        }),
      );

      const destPointFeatures: GeoJSON.Feature[] = seg.destMatches.map((m) => ({
        type: "Feature",
        id: m.matchIdx, // Add ID for feature-state
        properties: {
          matchIdx: m.matchIdx,
          wayIdx: m.wayIdx,
          distToWay: m.distToWay,
          matchPenalty: getMatchCost(m),
          additionalNodeIdx: m.additionalNodeIdx,
          type: "dest-projected",
          // Include route result info
          fwdReached: m.fwdResult?.reached ?? false,
          bwdReached: m.bwdResult?.reached ?? false,
          fwdCost: m.fwdResult?.cost ?? null,
          bwdCost: m.bwdResult?.cost ?? null,
        },
        geometry: { type: "Point", coordinates: m.projectedPoint },
      }));

      addMatchConnectorLayers(
        "segment-start-match-connectors",
        fromPt,
        seg.startMatches,
        "#c084fc",
        "#f3e8ff",
      );
      addMatchConnectorLayers(
        "segment-dest-match-connectors",
        toPt,
        seg.destMatches,
        "#fb923c",
        "#ffedd5",
      );

      if (startPointFeatures.length > 0) {
        map.addSource("start-projected-points", {
          type: "geojson",
          data: { type: "FeatureCollection", features: startPointFeatures },
          promoteId: "matchIdx",
        });
        map.addLayer({
          id: "start-projected-points",
          type: "circle",
          source: "start-projected-points",
          paint: {
            "circle-radius": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              10, // larger when highlighted
              5, // default
            ],
            "circle-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#22d3ee", // cyan when highlighted
              "#a855f7", // purple default
            ],
            "circle-stroke-color": "#fff",
            "circle-stroke-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              3,
              1,
            ],
          },
        });
      }

      if (destPointFeatures.length > 0) {
        map.addSource("dest-projected-points", {
          type: "geojson",
          data: { type: "FeatureCollection", features: destPointFeatures },
          promoteId: "matchIdx",
        });
        map.addLayer({
          id: "dest-projected-points",
          type: "circle",
          source: "dest-projected-points",
          paint: {
            "circle-radius": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              10, // larger when highlighted
              5, // default
            ],
            "circle-color": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              "#22d3ee", // cyan when highlighted
              "#f97316", // orange default
            ],
            "circle-stroke-color": "#fff",
            "circle-stroke-width": [
              "case",
              ["boolean", ["feature-state", "hover"], false],
              3,
              1,
            ],
          },
        });
      }

      if (finalStartMatch) {
        addFinalRouteMatchHighlight(
          "final-route-start-point",
          "final-route-start-connector",
          fromPt,
          finalStartMatch,
          "#e9d5ff",
          "#c084fc",
        );
      }

      if (finalDestMatch) {
        addFinalRouteMatchHighlight(
          "final-route-dest-point",
          "final-route-dest-connector",
          toPt,
          finalDestMatch,
          "#fdba74",
          "#fb923c",
        );
      }

      addRadiusLayer(
        "segment-start-radius",
        [fromPt.lng, fromPt.lat],
        startMatchingDistance,
        "rgba(34, 197, 94, 0.07)",
        "rgba(34, 197, 94, 0.35)",
      );
      addRadiusLayer(
        "segment-dest-radius",
        [toPt.lng, toPt.lat],
        destMatchingDistance,
        "rgba(239, 68, 68, 0.07)",
        "rgba(239, 68, 68, 0.35)",
      );

      // GPS points (top) - use distinct colors
      const startFeature = {
        type: "Feature" as const,
        properties: { type: "start-gps" },
        geometry: {
          type: "Point" as const,
          coordinates: [fromPt.lng, fromPt.lat],
        },
      };
      const destFeature = {
        type: "Feature" as const,
        properties: { type: "dest-gps" },
        geometry: { type: "Point" as const, coordinates: [toPt.lng, toPt.lat] },
      };
      addCircleLayer("segment-start", [startFeature], "#22c55e", 10); // green for start GPS
      addCircleLayer("segment-dest", [destFeature], "#ef4444", 10); // red for dest GPS
    }
  }, [data, viewMode, selectedPointIdx, selectedSegmentIdx]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded() || !data) return;

    const pointLayerId = "selected-node-highlight";
    const chainGlowLayerId = "selected-predecessor-chain-glow";
    const chainLineId = "selected-predecessor-chain";

    for (const id of [pointLayerId, chainGlowLayerId, chainLineId]) {
      if (map.getLayer(id)) map.removeLayer(id);
      if (map.getSource(id)) map.removeSource(id);
    }

    if (viewMode !== "segment" || selectedNodeIdx === null) return;

    const node = data.nodes[selectedNodeIdx];
    if (!node?.pos) return;

    const selectedNodeFeature: GeoJSON.Feature = {
      type: "Feature",
      properties: { nodeIdx: selectedNodeIdx },
      geometry: { type: "Point", coordinates: node.pos },
    };

    map.addSource(pointLayerId, {
      type: "geojson",
      data: { type: "FeatureCollection", features: [selectedNodeFeature] },
    });

    map.addLayer({
      id: pointLayerId,
      type: "circle",
      source: pointLayerId,
      paint: {
        "circle-radius": 10,
        "circle-color": "#38bdf8",
        "circle-opacity": 0.85,
        "circle-stroke-width": 3,
        "circle-stroke-color": "#e0f2fe",
        "circle-blur": 0.25,
      },
    });

    if (!selectedSegment?.nodeLabels || !selectedNodeLabelSignature) return;

    const nodeLabels =
      selectedSegment.nodeLabels[String(selectedNodeIdx)] ?? [];
    const selectedLabel = nodeLabels.find(
      (label) => getLabelSignature(label) === selectedNodeLabelSignature,
    );
    if (!selectedLabel) return;

    const predecessorChain = buildPredecessorChain(
      selectedNodeIdx,
      selectedLabel,
      selectedSegment.nodeLabels,
    );

    const beforeNodeLayerId = map.getLayer("unreached-nodes")
      ? "unreached-nodes"
      : map.getLayer("reached-nodes")
        ? "reached-nodes"
        : undefined;

    const chainCoordinates = predecessorChain
      .map((entry) => data.nodes[entry.nodeIdx]?.pos)
      .filter((pos): pos is [number, number] => Array.isArray(pos));

    if (chainCoordinates.length > 1) {
      map.addSource(chainLineId, {
        type: "geojson",
        data: {
          type: "Feature",
          properties: {},
          geometry: {
            type: "LineString",
            coordinates: chainCoordinates,
          },
        },
      });

      map.addLayer(
        {
          id: chainGlowLayerId,
          type: "line",
          source: chainLineId,
          paint: {
            "line-color": "#fb7185",
            "line-width": 18,
            "line-opacity": 0.42,
            "line-blur": 2.4,
          },
        },
        beforeNodeLayerId,
      );

      map.addLayer(
        {
          id: chainLineId,
          type: "line",
          source: chainLineId,
          paint: {
            "line-color": "#fff7ed",
            "line-width": 5,
            "line-opacity": 0.96,
          },
        },
        beforeNodeLayerId,
      );
    }
  }, [
    data,
    selectedNodeIdx,
    selectedNodeLabelSignature,
    selectedSegment,
    viewMode,
  ]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) {
      return;
    }

    const hoverPointLayerId = "hovered-inspector-node-highlight";
    const hoverGlowLayerId = "hovered-inspector-node-glow";
    const emptyData: GeoJSON.FeatureCollection = {
      type: "FeatureCollection",
      features: [],
    };

    const hasUsableStyle = () => {
      try {
        return map.isStyleLoaded();
      } catch {
        return false;
      }
    };

    const getHoverSource = () => {
      try {
        return map.getSource(hoverPointLayerId) as
          | maplibregl.GeoJSONSource
          | undefined;
      } catch {
        return undefined;
      }
    };

    const clearHoverNode = () => {
      const existingSource = getHoverSource();
      existingSource?.setData(emptyData);
      try {
        map.triggerRepaint();
      } catch {
        // Map is being torn down.
      }
    };

    const ensureHoverLayer = () => {
      if (!hasUsableStyle()) {
        return undefined;
      }

      try {
        if (!map.getSource(hoverPointLayerId)) {
          map.addSource(hoverPointLayerId, {
            type: "geojson",
            data: emptyData,
          });
        }

        if (!map.getLayer(hoverGlowLayerId)) {
          map.addLayer({
            id: hoverGlowLayerId,
            type: "circle",
            source: hoverPointLayerId,
            paint: {
              "circle-radius": 18,
              "circle-color": "#fb7185",
              "circle-opacity": 0.28,
              "circle-blur": 1,
            },
          });
        }

        if (!map.getLayer(hoverPointLayerId)) {
          map.addLayer({
            id: hoverPointLayerId,
            type: "circle",
            source: hoverPointLayerId,
            paint: {
              "circle-radius": 11,
              "circle-color": "rgba(251, 113, 133, 0.14)",
              "circle-stroke-width": 4,
              "circle-stroke-color": "#fb7185",
              "circle-opacity": 1,
              "circle-blur": 0.05,
            },
          });
        }

        if (map.getLayer(hoverGlowLayerId)) {
          map.moveLayer(hoverGlowLayerId);
        }
        if (map.getLayer(hoverPointLayerId)) {
          map.moveLayer(hoverPointLayerId);
        }
      } catch {
        return undefined;
      }

      return getHoverSource();
    };

    const setHoverNode = (nodeIdx: number | null) => {
      hoveredInspectorNodeRef.current = nodeIdx;

      const hoverSource = ensureHoverLayer();
      if (!hoverSource) {
        return;
      }

      if (
        nodeIdx === null ||
        !data ||
        viewMode !== "segment" ||
        !data.nodes[nodeIdx]?.pos
      ) {
        hoverSource.setData(emptyData);
        try {
          map.triggerRepaint();
        } catch {
          // Map is being torn down.
        }
        return;
      }

      hoverSource.setData({
        type: "FeatureCollection",
        features: [
          {
            type: "Feature",
            properties: { nodeIdx },
            geometry: { type: "Point", coordinates: data.nodes[nodeIdx].pos },
          },
        ],
      });
      if (map.getLayer(hoverPointLayerId)) {
        map.moveLayer(hoverPointLayerId);
      }
      try {
        map.triggerRepaint();
      } catch {
        // Map is being torn down.
      }
    };

    const handleInspectorHover = (event: Event) => {
      const customEvent = event as CustomEvent<{ nodeIdx: number | null }>;
      setHoverNode(customEvent.detail?.nodeIdx ?? null);
    };

    const handleMapLoadOrStyle = () => {
      setHoverNode(hoveredInspectorNodeRef.current);
    };

    window.addEventListener(kInspectorNodeHoverEvent, handleInspectorHover);
    map.on("load", handleMapLoadOrStyle);
    map.on("styledata", handleMapLoadOrStyle);

    if (!data || viewMode !== "segment") {
      clearHoverNode();
    } else {
      setHoverNode(hoveredInspectorNodeRef.current);
    }

    return () => {
      window.removeEventListener(
        kInspectorNodeHoverEvent,
        handleInspectorHover,
      );
      map.off("load", handleMapLoadOrStyle);
      map.off("styledata", handleMapLoadOrStyle);
      clearHoverNode();
    };
  }, [data, viewMode]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    const layerId = "selected-route-candidate";
    if (map.getLayer(layerId)) map.removeLayer(layerId);
    if (map.getSource(layerId)) map.removeSource(layerId);

    if (
      viewMode !== "segment" ||
      !selectedSegmentMatch ||
      !selectedRouteCandidate
    ) {
      return;
    }

    const result =
      selectedRouteCandidate.dir === "fwd"
        ? selectedSegmentMatch.match.fwdResult
        : selectedSegmentMatch.match.bwdResult;

    if (!result?.geometry || result.geometry.length < 2) {
      return;
    }

    map.addSource(layerId, {
      type: "geojson",
      data: {
        type: "Feature",
        properties: {},
        geometry: {
          type: "LineString",
          coordinates: result.geometry,
        },
      },
    });

    map.addLayer({
      id: layerId,
      type: "line",
      source: layerId,
      paint: {
        "line-color": "#22d3ee",
        "line-width": 6,
        "line-opacity": 0.95,
      },
    });
  }, [selectedRouteCandidate, selectedSegmentMatch, viewMode]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    const prevSegmentIdx = prevHighlightedSegmentRef.current;

    const setSegmentHoverState = (
      source: string,
      id: number | null,
      hover: boolean,
    ) => {
      if (!map.getSource(source) || id === null) {
        return;
      }
      try {
        map.setFeatureState({ source, id }, { hover });
      } catch {
        // Feature may not exist yet.
      }
    };

    if (prevSegmentIdx !== null && prevSegmentIdx !== highlightedSegmentIdx) {
      setSegmentHoverState("final-route", prevSegmentIdx, false);
      setSegmentHoverState("segment-overview", prevSegmentIdx, false);
    }

    if (highlightedSegmentIdx !== null) {
      setSegmentHoverState("final-route", highlightedSegmentIdx, true);
      setSegmentHoverState("segment-overview", highlightedSegmentIdx, true);
    }

    prevHighlightedSegmentRef.current = highlightedSegmentIdx;
  }, [highlightedSegmentIdx]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    const prevSelectedSegmentIdx = prevSelectedSegmentRef.current;

    const setSelectedState = (id: number | null, selected: boolean) => {
      if (!map.getSource("segment-overview") || id === null) {
        return;
      }
      try {
        map.setFeatureState({ source: "segment-overview", id }, { selected });
      } catch {
        // Feature may not exist yet.
      }
    };

    if (
      prevSelectedSegmentIdx !== null &&
      prevSelectedSegmentIdx !== selectedSegmentIdx
    ) {
      setSelectedState(prevSelectedSegmentIdx, false);
    }

    if (viewMode === "segment" && selectedSegmentIdx !== null) {
      setSelectedState(selectedSegmentIdx, true);
    }

    prevSelectedSegmentRef.current =
      viewMode === "segment" ? selectedSegmentIdx : null;
  }, [selectedSegmentIdx, viewMode, data]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    const handlePointerMove = (e: maplibregl.MapMouseEvent) => {
      if (viewMode !== "route" && viewMode !== "segment") {
        map.getCanvas().style.cursor = "";
        setHighlightedSegment(null);
        return;
      }

      const segmentLayers = (
        viewMode === "route"
          ? ["segment-labels", "segment-beeline", "final-route"]
          : [
              "segment-overview-labels",
              "segment-overview-beeline",
              "segment-overview",
            ]
      ).filter((layerId) => map.getLayer(layerId));

      if (segmentLayers.length === 0) {
        map.getCanvas().style.cursor = "";
        setHighlightedSegment(null);
        return;
      }

      const hoveredFeature = map
        .queryRenderedFeatures(e.point, { layers: segmentLayers })
        .find((feature) => feature.properties?.segmentIdx !== undefined);

      const hoveredSegmentIdx = hoveredFeature?.properties?.segmentIdx;
      const nextSegmentIdx =
        hoveredSegmentIdx === undefined ? null : Number(hoveredSegmentIdx);

      map.getCanvas().style.cursor = nextSegmentIdx !== null ? "pointer" : "";
      setHighlightedSegment(
        Number.isFinite(nextSegmentIdx) ? nextSegmentIdx : null,
      );
    };

    const handlePointerLeave = () => {
      map.getCanvas().style.cursor = "";
      setHighlightedSegment(null);
    };

    map.on("mousemove", handlePointerMove);
    map.getCanvas().addEventListener("mouseleave", handlePointerLeave);

    return () => {
      map.off("mousemove", handlePointerMove);
      map.getCanvas().removeEventListener("mouseleave", handlePointerLeave);
    };
  }, [setHighlightedSegment, viewMode]);

  // Update feature state for highlighted ways when hovering in sidebar
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    // Helper to set feature state on a source
    const setHoverState = (
      source: string,
      id: number | null,
      hover: boolean,
    ) => {
      if (!map.getSource(source) || id === null) return;
      try {
        map.setFeatureState({ source, id }, { hover });
      } catch {
        // Feature may not exist
      }
    };

    // Get previous highlighted way from ref
    const prevWayIdx = prevHighlightedWayRef.current;

    // Clear previous highlight
    if (prevWayIdx !== null && prevWayIdx !== highlightedWayIdx) {
      setHoverState("matched-ways", prevWayIdx, false);
      setHoverState("additional-edges", prevWayIdx, false);
    }

    // Set new highlight
    if (highlightedWayIdx !== null) {
      setHoverState("matched-ways", highlightedWayIdx, true);
      setHoverState("additional-edges", highlightedWayIdx, true);
    }

    // Store current for next cleanup
    prevHighlightedWayRef.current = highlightedWayIdx;
  }, [highlightedWayIdx]);

  // Update feature state for highlighted projected points
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    const setHoverState = (
      source: string,
      id: number | null,
      hover: boolean,
    ) => {
      if (!map.getSource(source) || id === null) return;
      try {
        map.setFeatureState({ source, id }, { hover });
      } catch {
        // Feature may not exist
      }
    };

    const prevMatchIdx = prevHighlightedMatchRef.current;

    if (prevMatchIdx !== null && prevMatchIdx !== highlightedMatchIdx) {
      setHoverState("start-projected-points", prevMatchIdx, false);
      setHoverState("dest-projected-points", prevMatchIdx, false);
      setHoverState("selected-point-match-connectors", prevMatchIdx, false);
      setHoverState(
        "selected-point-match-connectors-labels",
        prevMatchIdx,
        false,
      );
      setHoverState("segment-start-match-connectors", prevMatchIdx, false);
      setHoverState(
        "segment-start-match-connectors-labels",
        prevMatchIdx,
        false,
      );
      setHoverState("segment-dest-match-connectors", prevMatchIdx, false);
      setHoverState(
        "segment-dest-match-connectors-labels",
        prevMatchIdx,
        false,
      );
    }

    if (highlightedMatchIdx !== null) {
      setHoverState("start-projected-points", highlightedMatchIdx, true);
      setHoverState("dest-projected-points", highlightedMatchIdx, true);
      setHoverState(
        "selected-point-match-connectors",
        highlightedMatchIdx,
        true,
      );
      setHoverState(
        "selected-point-match-connectors-labels",
        highlightedMatchIdx,
        true,
      );
      setHoverState(
        "segment-start-match-connectors",
        highlightedMatchIdx,
        true,
      );
      setHoverState(
        "segment-start-match-connectors-labels",
        highlightedMatchIdx,
        true,
      );
      setHoverState("segment-dest-match-connectors", highlightedMatchIdx, true);
      setHoverState(
        "segment-dest-match-connectors-labels",
        highlightedMatchIdx,
        true,
      );
    }

    prevHighlightedMatchRef.current = highlightedMatchIdx;
  }, [highlightedMatchIdx]);

  // Global click handler - collect ALL features at click point and merge info
  const handleMapClick = useCallback(
    (e: maplibregl.MapMouseEvent) => {
      const map = mapRef.current;
      if (!map || !data) return;

      if (isMeasuring) {
        setMeasurePoints((prev) => [...prev, [e.lngLat.lng, e.lngLat.lat]]);
        return;
      }

      const interactiveLayers = LAYER_IDS.filter((id) => map.getLayer(id));
      const features = map.queryRenderedFeatures(e.point, {
        layers: interactiveLayers,
      });

      if (features.length === 0) return;

      let foundNodeIdx: number | null = null;
      let foundProjectedPoint = false;
      let foundPointIdx: number | null = null;
      let foundSegmentIdx: number | null = null;
      let foundWayIdx: number | null = null;
      let foundMatchIdx: number | null = null;

      for (const feature of features) {
        const props = feature.properties || {};
        const layerId = feature.layer.id;

        if (props.segmentIdx !== undefined) {
          const segmentIdx = Number(props.segmentIdx);
          if (Number.isFinite(segmentIdx)) {
            foundSegmentIdx = segmentIdx;
          }
        }

        if (layerId === "input-points" || layerId === "input-point-labels") {
          const pointIdx = Number(props.idx);
          if (Number.isFinite(pointIdx)) {
            foundPointIdx = pointIdx;
          }
        }

        // Check if it's a node
        if (layerId === "reached-nodes" || layerId === "unreached-nodes") {
          const nodeIdx = props.nodeIdx as number;
          if (nodeIdx !== undefined) {
            if (getNode(nodeIdx)) {
              foundNodeIdx = nodeIdx;
            }
          }
        }

        // Check if it's a projected point
        if (
          layerId === "start-projected-points" ||
          layerId === "dest-projected-points" ||
          layerId === "final-route-start-point" ||
          layerId === "final-route-dest-point" ||
          layerId === "final-route-start-connector" ||
          layerId === "final-route-dest-connector" ||
          layerId === "selected-point-match-connectors" ||
          layerId === "selected-point-match-connectors-labels" ||
          layerId === "segment-start-match-connectors" ||
          layerId === "segment-start-match-connectors-labels" ||
          layerId === "segment-dest-match-connectors" ||
          layerId === "segment-dest-match-connectors-labels"
        ) {
          foundProjectedPoint = true;
          foundMatchIdx = Number(props.matchIdx);
          foundWayIdx = Number(props.wayIdx);
        }

        // Check way
        if (
          layerId === "matched-ways" ||
          layerId === "additional-edges" ||
          layerId === "segment-route" ||
          layerId === "final-route"
        ) {
          if (props.wayIdx !== undefined) {
            const wayIdx = Number(props.wayIdx);
            if (getWay(wayIdx)) {
              foundWayIdx = wayIdx;
            }
          }
        }
      }

      if (foundPointIdx !== null) {
        showPoint(foundPointIdx);
        return;
      }

      if (foundSegmentIdx !== null) {
        showSegment(foundSegmentIdx);
        return;
      }

      if (foundNodeIdx !== null) {
        selectNode(foundNodeIdx);
        if (selectedSegment?.nodeLabels?.[String(foundNodeIdx)]?.length) {
          const sortedLabels = sortDebugNodeLabels(
            selectedSegment.nodeLabels[String(foundNodeIdx)],
          );
          selectNodeLabel(getLabelSignature(sortedLabels[0]));
        }
      }

      if (foundProjectedPoint && foundMatchIdx !== null) {
        selectRouteCandidate(null);
        selectMatch(foundMatchIdx);
      }

      if (foundWayIdx !== null) {
        selectRouteCandidate(null);
        selectWay(foundWayIdx);
      }

      if (!foundProjectedPoint) {
        selectMatch(null);
      }
    },
    [
      data,
      getWay,
      getNode,
      isMeasuring,
      selectNode,
      selectNodeLabel,
      showPoint,
      showSegment,
      selectMatch,
      selectRouteCandidate,
      selectWay,
      selectedSegment,
    ],
  );

  // Setup map event handlers
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    map.on("click", handleMapClick);

    const handleMouseMove = (e: maplibregl.MapMouseEvent) => {
      const interactiveLayers = LAYER_IDS.filter((id) => map.getLayer(id));
      const features = map.queryRenderedFeatures(e.point, {
        layers: interactiveLayers,
      });
      map.getCanvas().style.cursor = features.length > 0 ? "pointer" : "";
    };

    map.on("mousemove", handleMouseMove);

    return () => {
      map.off("click", handleMapClick);
      map.off("mousemove", handleMouseMove);
    };
  }, [handleMapClick]);

  // Update measurement layers
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !map.isStyleLoaded()) return;

    const lineId = "measure-line";
    const pointsId = "measure-points";

    if (map.getLayer(lineId)) map.removeLayer(lineId);
    if (map.getSource(lineId)) map.removeSource(lineId);
    if (map.getLayer(pointsId)) map.removeLayer(pointsId);
    if (map.getSource(pointsId)) map.removeSource(pointsId);

    if (measurePoints.length === 0) return;

    // Add points
    map.addSource(pointsId, {
      type: "geojson",
      data: {
        type: "FeatureCollection",
        features: measurePoints.map((pt, i) => ({
          type: "Feature",
          properties: { index: i },
          geometry: { type: "Point", coordinates: pt },
        })),
      },
    });
    map.addLayer({
      id: pointsId,
      type: "circle",
      source: pointsId,
      paint: {
        "circle-radius": 5,
        "circle-color": "#c800de",
        "circle-stroke-width": 2,
        "circle-stroke-color": "#ffffff",
      },
    });

    // Add line
    if (measurePoints.length > 1) {
      map.addSource(lineId, {
        type: "geojson",
        data: {
          type: "Feature",
          properties: {},
          geometry: {
            type: "LineString",
            coordinates: measurePoints,
          },
        },
      });
      map.addLayer(
        {
          id: lineId,
          type: "line",
          source: lineId,
          paint: {
            "line-color": "#c800de",
            "line-width": 3,
          },
        },
        pointsId,
      ); // Draw below points
    }
  }, [measurePoints]);

  // Main effect for layer updates
  useLayoutEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    let animationFrameId: number | null = null;
    let timeoutId: number | null = null;

    const doUpdate = () => {
      updateLayers();
      map.triggerRepaint();
    };

    if (map.isStyleLoaded()) {
      doUpdate();
      animationFrameId = window.requestAnimationFrame(doUpdate);
      timeoutId = window.setTimeout(doUpdate, 0);
    } else {
      map.on("load", doUpdate);
      return () => {
        map.off("load", doUpdate);
        if (animationFrameId !== null) {
          window.cancelAnimationFrame(animationFrameId);
        }
        if (timeoutId !== null) {
          window.clearTimeout(timeoutId);
        }
      };
    }

    return () => {
      if (animationFrameId !== null) {
        window.cancelAnimationFrame(animationFrameId);
      }
      if (timeoutId !== null) {
        window.clearTimeout(timeoutId);
      }
    };
  }, [data, selectedPointIdx, selectedSegmentIdx, updateLayers, viewMode]);

  const openInJOSM = useCallback(() => {
    const map = mapRef.current;
    if (!map) return;

    const bounds = map.getBounds();
    const left = bounds.getWest();
    const right = bounds.getEast();
    const top = bounds.getNorth();
    const bottom = bounds.getSouth();

    const url = `http://127.0.0.1:8111/load_and_zoom?left=${left}&right=${right}&top=${top}&bottom=${bottom}`;

    fetch(url).catch((err) => {
      console.error("Failed to open JOSM:", err);
      alert(
        "Failed to open JOSM. Make sure JOSM is running with Remote Control enabled.",
      );
    });
  }, []);

  const openInGoogleMaps = useCallback(() => {
    const map = mapRef.current;
    if (!map) return;

    const center = map.getCenter();
    const zoom = map.getZoom();
    const url = `https://www.google.com/maps/@${center.lat},${center.lng},${zoom}z`;
    window.open(url, "_blank");
  }, []);

  const openInOSM = useCallback(() => {
    const map = mapRef.current;
    if (!map) return;

    const center = map.getCenter();
    const zoom = Math.round(map.getZoom());
    const url = `https://www.openstreetmap.org/#map=${zoom}/${center.lat}/${center.lng}`;
    window.open(url, "_blank");
  }, []);

  const totalDistance = measurePoints.reduce((acc, pt, i) => {
    if (i === 0) return 0;
    return acc + haversineDistance(measurePoints[i - 1], pt);
  }, 0);

  return (
    <div className="relative w-full h-full">
      <div ref={containerRef} className="w-full h-full" />
      <div className="absolute top-4 left-4 z-10 flex flex-col gap-2">
        <DropdownMenu>
          <DropdownMenuTrigger
            render={
              <Button
                variant="outline"
                size="sm"
                className="gap-1 bg-white/90 backdrop-blur-sm"
              />
            }
          >
            Open in...
            <ChevronDownIcon className="h-4 w-4" />
          </DropdownMenuTrigger>
          <DropdownMenuContent align="start" className="w-56">
            <DropdownMenuItem onClick={openInJOSM}>
              JOSM (Remote Control)
            </DropdownMenuItem>
            <DropdownMenuItem onClick={openInGoogleMaps}>
              Google Maps
            </DropdownMenuItem>
            <DropdownMenuItem onClick={openInOSM}>
              OpenStreetMap.org
            </DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>

        <div className="flex items-center gap-1">
          <Button
            variant={isMeasuring ? "default" : "outline"}
            size="sm"
            className={`gap-1 bg-white/90 backdrop-blur-sm ${isMeasuring ? "bg-blue-600 text-white hover:bg-blue-700" : ""}`}
            onClick={() => {
              setIsMeasuring(!isMeasuring);
            }}
            title="Measure distance"
          >
            <Ruler className="h-4 w-4" />
            {isMeasuring && <span>Measuring</span>}
          </Button>

          {isMeasuring && (
            <>
              <Button
                variant="outline"
                size="sm"
                className="bg-white/90 backdrop-blur-sm p-2"
                onClick={() => setMeasurePoints([])}
                title="Clear measurement"
              >
                <Trash2 className="h-4 w-4" />
              </Button>
              <Button
                variant="outline"
                size="sm"
                className="bg-white/90 backdrop-blur-sm p-2"
                onClick={() => {
                  setIsMeasuring(false);
                  setMeasurePoints([]);
                }}
                title="Exit measurement mode"
              >
                <X className="h-4 w-4" />
              </Button>
            </>
          )}
        </div>

        {isMeasuring && measurePoints.length > 0 && (
          <div className="bg-white/10 backdrop-blur-sm px-3 py-1.5 rounded-md border shadow-sm text-sm font-medium">
            Distance:{" "}
            {totalDistance < 1000
              ? `${totalDistance.toFixed(1)} m`
              : `${(totalDistance / 1000).toFixed(2)} km`}
          </div>
        )}
      </div>
    </div>
  );
}
