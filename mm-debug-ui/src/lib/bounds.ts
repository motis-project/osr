import type { MapMatchDebugData } from "../types";

type Bounds = [[number, number], [number, number]];

function extend(bounds: [number, number, number, number], coord: [number, number]) {
    bounds[0] = Math.min(bounds[0], coord[0]);
    bounds[1] = Math.min(bounds[1], coord[1]);
    bounds[2] = Math.max(bounds[2], coord[0]);
    bounds[3] = Math.max(bounds[3], coord[1]);
}

export function calculatePointBounds(data: MapMatchDebugData, pointIdx: number): Bounds | null {
    const pt = data.inputPoints[pointIdx];
    if (!pt) return null;

    // minLng, minLat, maxLng, maxLat
    const bounds: [number, number, number, number] = [pt.lng, pt.lat, pt.lng, pt.lat];

    const segment = data.routeSegments.find(
        (s) => s.fromPointIdx === pointIdx || s.toPointIdx === pointIdx
    );

    if (segment) {
        const matches =
            segment.fromPointIdx === pointIdx
                ? segment.startMatches
                : segment.destMatches;

        for (const m of matches) {
            // Projected point
            extend(bounds, m.projectedPoint);

            // Way geometry
            const way = data.ways[m.wayIdx];
            if (way?.geometry) {
                for (const coord of way.geometry) {
                    extend(bounds, coord);
                }
            }
        }
    }

    return [[bounds[0], bounds[1]], [bounds[2], bounds[3]]];
}

export function calculateSegmentBounds(data: MapMatchDebugData, segIdx: number): Bounds | null {
    const seg = data.routeSegments[segIdx];
    if (!seg) return null;

    const fromPt = data.inputPoints[seg.fromPointIdx];
    const toPt = data.inputPoints[seg.toPointIdx];

    const bounds: [number, number, number, number] = [fromPt.lng, fromPt.lat, fromPt.lng, fromPt.lat];
    extend(bounds, [toPt.lng, toPt.lat]);

    // Route geometries
    for (const dm of seg.destMatches) {
        if (dm.fwdResult?.reached && dm.fwdResult.geometry) {
            for (const coord of dm.fwdResult.geometry) extend(bounds, coord);
        }
        if (dm.bwdResult?.reached && dm.bwdResult.geometry) {
            for (const coord of dm.bwdResult.geometry) extend(bounds, coord);
        }
    }

    // Matches (ways and projected points)
    const allMatches = [...seg.startMatches, ...seg.destMatches];
    const seenWays = new Set<number>();
    
    for (const m of allMatches) {
        // Projected point
        extend(bounds, m.projectedPoint);

        // Ways
        if (!seenWays.has(m.wayIdx)) {
            seenWays.add(m.wayIdx);
            const way = data.ways[m.wayIdx];
            if (way?.geometry) {
                for (const coord of way.geometry) extend(bounds, coord);
            }
        }
    }

    // Additional Edge Ways explicitly listed
    if (seg.additionalEdgeWays) {
        for (const wayIdx of seg.additionalEdgeWays) {
            if (!seenWays.has(wayIdx)) {
                seenWays.add(wayIdx);
                const way = data.ways[wayIdx];
                if (way?.geometry) {
                    for (const coord of way.geometry) extend(bounds, coord);
                }
            }
        }
    }

    // Nodes (Labels)
    if (seg.nodeLabels) {
        for (const nodeIdxStr of Object.keys(seg.nodeLabels)) {
            const idx = parseInt(nodeIdxStr, 10);
            const node = data.nodes[idx];
            if (node?.pos) {
                extend(bounds, node.pos);
            }
        }
    }

    // Nodes from matches (unreached/reached implicitly)
    const displayedNodes = new Set(Object.keys(seg.nodeLabels || {}).map(k => parseInt(k, 10)));
    for (const m of allMatches) {
        for (const idx of [m.additionalNodeIdx, m.fwdOut?.nodeIdx, m.fwdIn?.nodeIdx, m.bwdOut?.nodeIdx, m.bwdIn?.nodeIdx]) {
            if (idx !== undefined && !displayedNodes.has(idx)) {
                 const node = data.nodes[idx];
                 if (node?.pos) {
                     extend(bounds, node.pos);
                 }
                 displayedNodes.add(idx);
            }
        }
    }

    return [[bounds[0], bounds[1]], [bounds[2], bounds[3]]];
}
