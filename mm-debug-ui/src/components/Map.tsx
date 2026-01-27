import { useEffect, useRef, useCallback, useState } from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import { Button } from "./ui/button";
import {
    DropdownMenu,
    DropdownMenuContent,
    DropdownMenuItem,
    DropdownMenuTrigger,
} from "./ui/dropdown-menu";
import { ChevronDownIcon } from "lucide-react";
import { useDebug } from "../store";
import { calculatePointBounds, calculateSegmentBounds } from "../lib/bounds";
import type { DebugWay, DebugNode, DebugNodeLabel, DebugStartLabel } from "../types";

interface PopupInfo {
    type: "way" | "node" | "point" | "combined";
    data: DebugWay | DebugNode | Record<string, unknown>;
    lngLat: [number, number];
    nodeLabels?: DebugNodeLabel[];
    startLabel?: DebugStartLabel;
}



export function Map() {
    const containerRef = useRef<HTMLDivElement>(null);
    const mapRef = useRef<maplibregl.Map | null>(null);
    const popupRef = useRef<maplibregl.Popup | null>(null);
    const prevHighlightedWayRef = useRef<number | null>(null);
    const prevHighlightedMatchRef = useRef<number | null>(null);
    const [popupInfo, setPopupInfo] = useState<PopupInfo | null>(null);
    const [highlightedPredIdx, setHighlightedPredIdx] = useState<number | null>(null);
    const {
        data,
        viewMode,
        selectedPointIdx,
        selectedSegmentIdx,
        highlightedWayIdx,
        highlightedMatchIdx,
        getBoundingBox,
        getWay,
        getNode,
    } = useDebug();

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
        map.addControl(new maplibregl.ScaleControl({ unit: "metric" }), "bottom-left");
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
            map.fitBounds(bounds, { padding: 50 });
        }
    }, [data, getBoundingBox]);

    // Fit bounds on selection
    useEffect(() => {
        const map = mapRef.current;
        if (!map || !data) return;

        let bounds: [[number, number], [number, number]] | null = null;

        if (viewMode === "points" && selectedPointIdx !== null) {
            bounds = calculatePointBounds(data, selectedPointIdx);
        } else if (viewMode === "segment" && selectedSegmentIdx !== null) {
            bounds = calculateSegmentBounds(data, selectedSegmentIdx);
        }

        if (bounds) {
            map.fitBounds(bounds, { padding: 50, animate: true });
        }
    }, [data, viewMode, selectedPointIdx, selectedSegmentIdx]);

    // Close popup when selection changes
    useEffect(() => {
        setPopupInfo(null);
        if (popupRef.current) {
            popupRef.current.remove();
            popupRef.current = null;
        }
    }, [viewMode, selectedPointIdx, selectedSegmentIdx]);

    // All layer IDs we manage (in order: first added = bottom)
    const LAYER_IDS = [
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
        "segment-start",
        "segment-dest",
    ];

    // Update map layers based on view mode and selection
    const updateLayers = useCallback(() => {
        const map = mapRef.current;
        if (!map || !data) return;

        // Ensure arrow image exists
        if (!map.hasImage("arrow")) {
            const width = 24;
            const height = 24;
            const ctx = document.createElement('canvas').getContext('2d');
            if (ctx) {
                ctx.canvas.width = width;
                ctx.canvas.height = height;
                ctx.strokeStyle = '#ffffff'; // White stroke
                ctx.lineWidth = 3;
                ctx.lineCap = "round";
                ctx.lineJoin = "round";
                ctx.beginPath();
                ctx.moveTo(8, 6);   // Top-leftish
                ctx.lineTo(16, 12); // Tip
                ctx.lineTo(8, 18);  // Bottom-leftish
                ctx.stroke();
                const imageData = ctx.getImageData(0, 0, width, height);
                map.addImage("arrow", imageData, { sdf: true });
            }
        }

        // Ensure tick image exists (for additional edge markers)
        if (!map.hasImage("tick")) {
            const width = 24;
            const height = 24;
            const ctx = document.createElement('canvas').getContext('2d');
            if (ctx) {
                ctx.canvas.width = width;
                ctx.canvas.height = height;
                ctx.strokeStyle = '#ffffff'; // White stroke
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
        // Iterate in reverse to remove top layers (which might depend on sources) first
        for (const id of [...LAYER_IDS].reverse()) {
            if (map.getLayer(id)) map.removeLayer(id);
            if (map.getSource(id)) map.removeSource(id);
        }

        // Helper to add line layer
        const addLineLayer = (
            id: string,
            coordinates: [number, number][] | [number, number][][],
            color: string,
            width: number,
            dasharray?: number[]
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
            radius: number
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

        // Route mode: show final route and input points
        if (viewMode === "route") {
            // Final route line
            if (data.finalRoute.geometry.length > 0) {
                addLineLayer("final-route", data.finalRoute.geometry, "#3b82f6", 4);
                map.addLayer({
                    id: "final-route-arrows",
                    type: "symbol",
                    source: "final-route",
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
            }

            // Input points
            const pointFeatures = data.inputPoints.map((pt, i) => ({
                type: "Feature" as const,
                properties: { idx: i, type: "input" },
                geometry: { type: "Point" as const, coordinates: [pt.lng, pt.lat] },
            }));
            addCircleLayer("input-points", pointFeatures, "#ef4444", 8);

            // Beeline segments (for segments that couldn't be routed)
            const beelineCoords: [number, number][][] = [];
            for (const seg of data.routeSegments) {
                if (seg.allBeelined || seg.beeline) {
                    const from = data.inputPoints[seg.fromPointIdx];
                    const to = data.inputPoints[seg.toPointIdx];
                    beelineCoords.push([[from.lng, from.lat], [to.lng, to.lat]]);
                }
            }
            if (beelineCoords.length > 0) {
                addLineLayer("segment-beeline", beelineCoords, "#dc2626", 3, [4, 2]);
            }
        }

        // Points mode
        if (viewMode === "points" && selectedPointIdx !== null) {
            const pt = data.inputPoints[selectedPointIdx];
            const segment = data.routeSegments.find(
                (s) =>
                    s.fromPointIdx === selectedPointIdx ||
                    s.toPointIdx === selectedPointIdx
            );

            if (segment) {
                const matches =
                    segment.fromPointIdx === selectedPointIdx
                        ? segment.startMatches
                        : segment.destMatches;

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
            }

            const gpsFeature = {
                type: "Feature" as const,
                properties: { type: "gps" },
                geometry: { type: "Point" as const, coordinates: [pt.lng, pt.lat] },
            };
            addCircleLayer("input-points", [gpsFeature], "#ef4444", 10);
        }

        // Segment mode
        if (viewMode === "segment" && selectedSegmentIdx !== null) {
            const seg = data.routeSegments[selectedSegmentIdx];
            const fromPt = data.inputPoints[seg.fromPointIdx];
            const toPt = data.inputPoints[seg.toPointIdx];

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
                    [4, 2]
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
            const allMatches = [...seg.startMatches, ...seg.destMatches];
            const seenWays = new Set<number>();

            // Helper to create a tiny segment centered at 'center' aligned with p1->p2
            const createCenteredSegment = (
                p1: [number, number],
                p2: [number, number],
                center: [number, number]
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

                return [[start[0], start[1]], [end[0], end[1]]];
            };

            const processAdditionalEdge = (way: DebugWay) => {
                if (!way.geometry || way.geometry.length < 2) return;

                // Start marker: centered at geom[0], aligned with geom[0]->geom[1]
                const startSeg = createCenteredSegment(
                    way.geometry[0] as [number, number],
                    way.geometry[1] as [number, number],
                    way.geometry[0] as [number, number]
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
                    way.geometry[last] as [number, number]
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
                        "icon-offset": ["literal", [0, 20]]
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
                    data: { type: "FeatureCollection", features: additionalEdgeMarkerFeatures },
                });
                map.addLayer({
                    id: "additional-edges-markers",
                    type: "symbol",
                    source: "additional-edges-markers",
                    layout: {
                        "symbol-placement": "line-center",
                        "icon-image": "tick",
                        "icon-allow-overlap": true,
                        "icon-offset": ["literal", [0, 20]]
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
            const displayedNodes = new Set(Object.keys(nodeLabels).map(k => parseInt(k, 10)));
            for (const m of allMatches) {
                for (const idx of [m.additionalNodeIdx, m.fwdOut?.nodeIdx, m.fwdIn?.nodeIdx, m.bwdOut?.nodeIdx, m.bwdIn?.nodeIdx]) {
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
                    4
                );
            }

            // Add reached nodes (cyan/teal color to distinguish from green)
            if (reachedNodeFeatures.length > 0) {
                addCircleLayer(
                    "reached-nodes",
                    reachedNodeFeatures,
                    "#06b6d4", // cyan
                    5
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
                                matchPenalty: m.startLabel.matchPenalty,
                                prevSegmentCost: m.startLabel.prevSegmentCost,
                                costOffset: m.startLabel.costOffset,
                                totalStartCost: m.startLabel.totalStartCost,
                            }
                            : {}),
                    },
                    geometry: { type: "Point", coordinates: m.projectedPoint },
                })
            );

            const destPointFeatures: GeoJSON.Feature[] = seg.destMatches.map((m) => ({
                type: "Feature",
                id: m.matchIdx, // Add ID for feature-state
                properties: {
                    matchIdx: m.matchIdx,
                    wayIdx: m.wayIdx,
                    distToWay: m.distToWay,
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

    // Update highlighted predecessor layer when highlightedPredIdx changes
    useEffect(() => {
        const map = mapRef.current;
        if (!map || !map.isStyleLoaded() || !data) return;

        const layerId = "highlighted-predecessor";

        // Remove existing highlight layer
        if (map.getLayer(layerId)) map.removeLayer(layerId);
        if (map.getSource(layerId)) map.removeSource(layerId);

        if (highlightedPredIdx === null) return;

        // Find the node position
        const node = data.nodes[highlightedPredIdx];
        if (!node || !node.pos) return;

        const feature: GeoJSON.Feature = {
            type: "Feature",
            properties: { nodeIdx: highlightedPredIdx },
            geometry: { type: "Point", coordinates: node.pos },
        };

        map.addSource(layerId, {
            type: "geojson",
            data: { type: "FeatureCollection", features: [feature] },
        });

        map.addLayer({
            id: layerId,
            type: "circle",
            source: layerId,
            paint: {
                "circle-radius": 12,
                "circle-color": "#facc15", // yellow
                "circle-opacity": 0.8,
                "circle-stroke-width": 3,
                "circle-stroke-color": "#fef08a", // light yellow glow
                "circle-blur": 0.4,
            },
        });
    }, [data, highlightedPredIdx]);

    // Update feature state for highlighted ways when hovering in sidebar
    useEffect(() => {
        const map = mapRef.current;
        if (!map || !map.isStyleLoaded()) return;

        // Helper to set feature state on a source
        const setHoverState = (source: string, id: number | null, hover: boolean) => {
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

        const setHoverState = (source: string, id: number | null, hover: boolean) => {
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
        }

        if (highlightedMatchIdx !== null) {
            setHoverState("start-projected-points", highlightedMatchIdx, true);
            setHoverState("dest-projected-points", highlightedMatchIdx, true);
        }

        prevHighlightedMatchRef.current = highlightedMatchIdx;
    }, [highlightedMatchIdx]);

    // Global click handler - collect ALL features at click point and merge info
    const handleMapClick = useCallback(
        (e: maplibregl.MapMouseEvent) => {
            const map = mapRef.current;
            if (!map || !data) return;

            const interactiveLayers = LAYER_IDS.filter((id) => map.getLayer(id));
            const features = map.queryRenderedFeatures(e.point, {
                layers: interactiveLayers,
            });

            if (features.length === 0) return;

            const lngLat: [number, number] = [e.lngLat.lng, e.lngLat.lat];

            // Track what we found
            let foundNode: DebugNode | null = null;
            let foundNodeIdx: number | null = null;
            let foundProjectedPoint = false;
            const foundWays: DebugWay[] = [];
            let nodeLabels: DebugNodeLabel[] | undefined;
            let startLabel: DebugStartLabel | undefined;
            const projectedProps: Record<string, unknown> = {};

            for (const feature of features) {
                const props = feature.properties || {};
                const layerId = feature.layer.id;

                // Check if it's a node
                if (layerId === "reached-nodes" || layerId === "unreached-nodes") {
                    const nodeIdx = props.nodeIdx as number;
                    if (nodeIdx !== undefined) {
                        const node = getNode(nodeIdx);
                        if (node) {
                            foundNode = node;
                            foundNodeIdx = nodeIdx;

                            // Get labels for this node from current segment
                            if (selectedSegmentIdx !== null) {
                                const seg = data.routeSegments[selectedSegmentIdx];
                                if (seg.nodeLabels) {
                                    nodeLabels = seg.nodeLabels[String(nodeIdx)];
                                }
                            }
                        }
                    }
                }

                // Check if it's a projected point
                if (
                    layerId === "start-projected-points" ||
                    layerId === "dest-projected-points"
                ) {
                    foundProjectedPoint = true;
                    projectedProps.type = props.type;
                    projectedProps.matchIdx = props.matchIdx;
                    projectedProps.wayIdx = props.wayIdx;
                    projectedProps.distToWay = props.distToWay;
                    projectedProps.additionalNodeIdx = props.additionalNodeIdx;

                    // Get start label info
                    if (props.matchPenalty !== undefined) {
                        startLabel = {
                            matchPenalty: props.matchPenalty as number,
                            prevSegmentCost: props.prevSegmentCost as number,
                            costOffset: props.costOffset as number,
                            totalStartCost: props.totalStartCost as number,
                        };
                    }

                    // Route result info
                    if (props.fwdReached !== undefined) {
                        projectedProps.fwdReached = props.fwdReached;
                        projectedProps.bwdReached = props.bwdReached;
                        projectedProps.fwdCost = props.fwdCost;
                        projectedProps.bwdCost = props.bwdCost;
                    }
                }

                // Check way
                if (
                    layerId === "matched-ways" ||
                    layerId === "additional-edges" ||
                    layerId === "segment-route" ||
                    layerId === "final-route"
                ) {
                    if (props.wayIdx !== undefined) {
                        const way = getWay(props.wayIdx as number);
                        // Add if not already found
                        if (way && !foundWays.some(w => w.idx === way.idx)) {
                            foundWays.push(way);
                        }
                    }
                }
            }

            // Determine popup type and data
            let type: "way" | "node" | "point" | "combined";
            let primaryData: DebugWay | DebugNode | Record<string, unknown>;

            const combinedData: Record<string, unknown> = {};

            if (foundWays.length > 0) {
                combinedData.ways = foundWays;
            }

            if (foundNode && foundProjectedPoint) {
                type = "combined";
                primaryData = { ...foundNode, ...projectedProps, nodeIdx: foundNodeIdx, ...combinedData };
            } else if (foundNode) {
                type = "node";
                primaryData = { ...foundNode, nodeIdx: foundNodeIdx, ...combinedData };
            } else if (foundProjectedPoint) {
                type = "point";
                primaryData = { ...projectedProps, layerType: "Projected Point", ...combinedData };
            } else if (foundWays.length > 0) {
                type = "combined"; // treat multi-way or single-way as combined to show custom list
                primaryData = { ways: foundWays };
            } else {
                // GPS point or other
                type = "point";
                primaryData = { layerType: "GPS Location" };
            }

            setPopupInfo({
                type,
                data: primaryData,
                lngLat,
                nodeLabels,
                startLabel,
            });
        },
        [data, getWay, getNode, selectedSegmentIdx]
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

    // Update popup
    useEffect(() => {
        const map = mapRef.current;
        if (!map) return;

        if (popupRef.current) {
            popupRef.current.remove();
            popupRef.current = null;
        }

        if (!popupInfo) return;

        const { type, data: itemData, lngLat, nodeLabels, startLabel } = popupInfo;

        let html = '<div class="popup-content">';

        if (type === "way") {
            // Legacy single way handling (fallback)
            const way = itemData as DebugWay;
            html += renderWayDetails(way);
        } else if (type === "node" || type === "combined") {
            const props = itemData as Record<string, unknown>;

            // Node details
            if (props.internalId !== undefined && props.nodeIdx !== undefined) {
                html += "<h3>Node</h3>";
                html += "<table>";
                html += `<tr><td>Internal ID</td><td>${props.internalId}</td></tr>`;
                if (props.osmId) {
                    html += `<tr><td>OSM ID</td><td><a href="https://openstreetmap.org/node/${props.osmId}" target="_blank">${props.osmId}</a></td></tr>`;
                }
                html += `<tr><td>Array Index</td><td>${props.nodeIdx}</td></tr>`;
                html += `<tr><td>Additional</td><td>${props.isAdditionalNode ? "Yes" : "No"}</td></tr>`;
                html += "</table>";

                // Show labels if available
                if (nodeLabels && nodeLabels.length > 0) {
                    html += "<h4>Labels</h4>";
                    html += "<table class='label-table'>";
                    html += "<tr><th class='pr-2 text-left'>Label</th><th class='pr-2 text-left'>Cost</th><th class='text-left'>Pred</th></tr>";
                    for (const label of nodeLabels) {
                        const predIdx = label.predNodeIdx;
                        const predAttr = predIdx !== null ? `data-pred-idx="${predIdx}"` : '';
                        html += `<tr class="label-row" ${predAttr} style="cursor: ${predIdx !== null ? 'pointer' : 'default'};">`;
                        html += `<td style="font-family: monospace; font-size: 11px;">${label.label}</td>`;
                        html += `<td class="pr-2">${label.cost}</td>`;
                        html += `<td>${predIdx !== null ? predIdx : "-"}</td></tr>`;
                    }
                    html += "</table>";
                } else {
                    html += "<p><em>No labels (unreached)</em></p>";
                }
            }

            // Projected Point details
            if (props.layerType === "Projected Point" || props.projectedPointType) {
                html += "<h4>Projected Point</h4>";
                html += "<table>";
                if (props.matchIdx !== undefined) html += `<tr><td>Match Index</td><td>${props.matchIdx}</td></tr>`;
                if (props.distToWay !== undefined) html += `<tr><td>Dist to Way</td><td>${(props.distToWay as number).toFixed(2)}m</td></tr>`;
                html += "</table>";
            }

            // Ways details (Multiple)
            if (props.ways && Array.isArray(props.ways)) {
                const ways = props.ways as DebugWay[];
                ways.forEach(way => {
                    html += renderWayDetails(way);
                });
            }
        } else {
            // Point type
            const props = itemData as Record<string, unknown>;
            const layerType = (props.layerType as string) || "Point";
            html += `<h3>${layerType}</h3>`;
            html += "<table>";

            // Show key info first
            if (props.matchIdx !== undefined) {
                html += `<tr><td>Match Index</td><td>${props.matchIdx}</td></tr>`;
            }
            if (props.wayIdx !== undefined) {
                const way = getWay(props.wayIdx as number);
                if (way?.osmId) {
                    html += `<tr><td>Way OSM ID</td><td><a href="https://openstreetmap.org/way/${way.osmId}" target="_blank">${way.osmId}</a></td></tr>`;
                }
                html += `<tr><td>Way Idx</td><td>${props.wayIdx}</td></tr>`;
            }
            if (props.distToWay !== undefined) {
                const dist = props.distToWay as number;
                html += `<tr><td>Dist to Way</td><td>${dist.toFixed(2)}m</td></tr>`;
            }
            if (props.additionalNodeIdx !== undefined) {
                html += `<tr><td>Node Idx</td><td>${props.additionalNodeIdx}</td></tr>`;
            }

            // Route result info
            if (props.fwdReached !== undefined) {
                html += `<tr><td>Fwd Reached</td><td>${props.fwdReached ? "Yes" : "No"}${props.fwdCost !== null ? ` (cost: ${props.fwdCost})` : ""}</td></tr>`;
                html += `<tr><td>Bwd Reached</td><td>${props.bwdReached ? "Yes" : "No"}${props.bwdCost !== null ? ` (cost: ${props.bwdCost})` : ""}</td></tr>`;
            }

            html += "</table>";
        }

        // Show start label info if available
        if (startLabel) {
            html += "<h4>Start Label</h4>";
            html += "<table>";
            html += `<tr><td>Match Penalty</td><td>${startLabel.matchPenalty}</td></tr>`;
            html += `<tr><td>Prev Segment Cost</td><td>${startLabel.prevSegmentCost}</td></tr>`;
            html += `<tr><td>Cost Offset</td><td>${startLabel.costOffset}</td></tr>`;
            html += `<tr><td>Total Start Cost</td><td>${startLabel.totalStartCost}</td></tr>`;
            html += "</table>";
        }

        html += "</div>";

        const popup = new maplibregl.Popup({
            closeButton: true,
            closeOnClick: true,
            maxWidth: "400px",
        })
            .setLngLat(lngLat)
            .setHTML(html)
            .addTo(map);

        // Attach hover event listeners to label rows
        const labelRows = popup.getElement().querySelectorAll('.label-row[data-pred-idx]');
        labelRows.forEach((row) => {
            row.addEventListener('mouseenter', () => {
                const predIdx = parseInt(row.getAttribute('data-pred-idx') || '', 10);
                if (!isNaN(predIdx)) {
                    setHighlightedPredIdx(predIdx);
                    (row as HTMLElement).style.backgroundColor = 'rgba(250, 204, 21, 0.3)'; // subtle yellow
                }
            });
            row.addEventListener('mouseleave', () => {
                setHighlightedPredIdx(null);
                (row as HTMLElement).style.backgroundColor = '';
            });
        });

        popup.on("close", () => {
            setPopupInfo(null);
            setHighlightedPredIdx(null);
        });

        popupRef.current = popup;
    }, [popupInfo, getWay, getNode]);

    // Main effect for layer updates
    useEffect(() => {
        const map = mapRef.current;
        if (!map) return;

        const doUpdate = () => {
            updateLayers();
        };

        if (map.isStyleLoaded()) {
            doUpdate();
        } else {
            map.on("load", doUpdate);
            return () => {
                map.off("load", doUpdate);
            };
        }
    }, [updateLayers]);

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
            alert("Failed to open JOSM. Make sure JOSM is running with Remote Control enabled.");
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

    return (
        <div className="relative w-full h-full">
            <div ref={containerRef} className="w-full h-full" />
            <div className="absolute top-4 left-4 z-10">
                <DropdownMenu>
                    <DropdownMenuTrigger render={<Button variant="outline" size="sm" className="gap-1" />}>
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
            </div>
        </div>
    );
}

function renderWayDetails(way: DebugWay): string {
    let res = `<h3>Way ${way.isAdditionalEdge ? "(Additional)" : ""}</h3>`;
    res += "<table>";
    res += `<tr><td>Array Index</td><td>${way.idx}</td></tr>`;
    res += `<tr><td>Internal ID</td><td>${way.internalId}</td></tr>`;
    if (way.osmId) {
        res += `<tr><td>OSM ID</td><td><a href="https://openstreetmap.org/way/${way.osmId}" target="_blank">${way.osmId}</a></td></tr>`;
    }
    if (way.fromNodeId !== undefined) {
        res += `<tr><td>From Node</td><td>${way.fromNodeId}</td></tr>`;
    }
    if (way.toNodeId !== undefined) {
        res += `<tr><td>To Node</td><td>${way.toNodeId}</td></tr>`;
    }
    if (way.reverse !== undefined) {
        res += `<tr><td>Reverse</td><td>${way.reverse ? "Yes" : "No"}</td></tr>`;
    }
    if (way.properties) {
        const p = way.properties;
        const access = [];
        if (p.car) access.push("car");
        if (p.bike) access.push("bike");
        if (p.foot) access.push("foot");
        if (p.bus) access.push("bus");
        res += `<tr><td>Access</td><td>${access.join(", ") || "none"}</td></tr>`;
        res += `<tr><td>Max Speed</td><td>${p.maxSpeedKmh} km/h</td></tr>`;
        res += `<tr><td>Oneway (car)</td><td>${p.onewayCar ? "Yes" : "No"}</td></tr>`;
        // res += `<tr><td>Oneway (bike)</td><td>${p.onewayBike ? "Yes" : "No"}</td></tr>`;
        // res += `<tr><td>Big Street</td><td>${p.isBigStreet ? "Yes" : "No"}</td></tr>`;
        res += `<tr><td>Destination</td><td>${p.isDestination ? "Yes" : "No"}</td></tr>`;
        res += `<tr><td>Component</td><td>${p.component}</td></tr>`;
    }
    res += "</table>";
    return res;
}
