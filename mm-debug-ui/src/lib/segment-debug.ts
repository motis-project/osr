import type {
  DebugMatch,
  DebugNodeLabel,
  DebugRouteSegment,
  MapMatchDebugData,
} from "../types";

export const kInspectorNodeHoverEvent = "mm-debug:hover-inspector-node";

export type RouteCandidateDirection = "fwd" | "bwd";

export type RouteCandidate = {
  key: string;
  dir: "FWD" | "BWD";
  dirKey: RouteCandidateDirection;
  matchIdx: number;
  wayIdx: number;
  cost: number;
  pathSteps: number;
  geometryPoints: number;
  startNodeIdx: number | null;
  endNodeIdx: number | null;
  beelineFromMatchIdx?: number;
  beelineDist?: number;
};

export type SegmentNodeLabels = {
  nodeIdx: number;
  labels: DebugNodeLabel[];
  minCost: number;
};

export type PredecessorChainEntry = {
  nodeIdx: number;
  label: DebugNodeLabel | null;
  exactLabel: boolean;
};

export function getLabelSignature(label: DebugNodeLabel): string {
  return `${label.label}::${label.cost}::${label.predNodeIdx ?? "root"}`;
}

export function segmentHasReachedDestination(
  segment: DebugRouteSegment,
): boolean {
  return segment.destMatches.some(
    (match) => match.fwdResult?.reached || match.bwdResult?.reached,
  );
}

export function getRouteCandidates(
  segment: DebugRouteSegment,
): RouteCandidate[] {
  return segment.destMatches
    .flatMap((match) => {
      const candidates: RouteCandidate[] = [];

      if (match.fwdResult?.reached && match.fwdResult.cost !== null) {
        candidates.push({
          key: `${match.matchIdx}-fwd`,
          dir: "FWD",
          dirKey: "fwd",
          matchIdx: match.matchIdx,
          wayIdx: match.wayIdx,
          cost: match.fwdResult.cost,
          pathSteps: match.fwdResult.path?.length ?? 0,
          geometryPoints: match.fwdResult.geometry?.length ?? 0,
          startNodeIdx: match.fwdResult.path?.[0]?.nodeIdx ?? null,
          endNodeIdx:
            match.fwdResult.path?.[match.fwdResult.path.length - 1]?.nodeIdx ??
            null,
          beelineFromMatchIdx: match.beelineFromMatchIdx,
          beelineDist: match.beelineDist,
        });
      }

      if (match.bwdResult?.reached && match.bwdResult.cost !== null) {
        candidates.push({
          key: `${match.matchIdx}-bwd`,
          dir: "BWD",
          dirKey: "bwd",
          matchIdx: match.matchIdx,
          wayIdx: match.wayIdx,
          cost: match.bwdResult.cost,
          pathSteps: match.bwdResult.path?.length ?? 0,
          geometryPoints: match.bwdResult.geometry?.length ?? 0,
          startNodeIdx: match.bwdResult.path?.[0]?.nodeIdx ?? null,
          endNodeIdx:
            match.bwdResult.path?.[match.bwdResult.path.length - 1]?.nodeIdx ??
            null,
          beelineFromMatchIdx: match.beelineFromMatchIdx,
          beelineDist: match.beelineDist,
        });
      }

      return candidates;
    })
    .sort((left, right) => left.cost - right.cost);
}

export function getSegmentLabelNodes(
  segment: DebugRouteSegment | null,
): SegmentNodeLabels[] {
  if (!segment?.nodeLabels) {
    return [];
  }

  return Object.entries(segment.nodeLabels)
    .map(([nodeIdx, labels]) => ({
      nodeIdx: Number(nodeIdx),
      labels,
      minCost: labels.reduce(
        (best, entry) => Math.min(best, entry.cost),
        Number.POSITIVE_INFINITY,
      ),
    }))
    .filter((entry) => entry.labels.length > 0)
    .sort((left, right) => left.minCost - right.minCost);
}

export function buildPredecessorChain(
  nodeIdx: number,
  label: DebugNodeLabel,
  nodeLabels: Record<string, DebugNodeLabel[]> | undefined,
): PredecessorChainEntry[] {
  const chain: PredecessorChainEntry[] = [
    {
      nodeIdx,
      label,
      exactLabel: true,
    },
  ];
  const seenNodes = new Set<number>([nodeIdx]);

  let currentLabel: DebugNodeLabel | null = label;

  for (let depth = 0; depth < 24; ++depth) {
    const predNodeIdx: number | null | undefined = currentLabel?.predNodeIdx;
    if (predNodeIdx === null || predNodeIdx === undefined) {
      break;
    }
    if (seenNodes.has(predNodeIdx)) {
      chain.push({ nodeIdx: predNodeIdx, label: null, exactLabel: false });
      break;
    }

    seenNodes.add(predNodeIdx);
    const predecessorLabels: DebugNodeLabel[] =
      nodeLabels?.[String(predNodeIdx)] ?? [];
    const exactLabel: DebugNodeLabel | undefined = predecessorLabels.find(
      (entry) => entry.label === currentLabel?.label,
    );
    const fallbackLabel = predecessorLabels.reduce<DebugNodeLabel | null>(
      (best, entry) => (best === null || entry.cost < best.cost ? entry : best),
      null,
    );
    const nextLabel: DebugNodeLabel | null = exactLabel ?? fallbackLabel;

    chain.push({
      nodeIdx: predNodeIdx,
      label: nextLabel,
      exactLabel: exactLabel !== undefined,
    });

    if (!nextLabel || nextLabel.predNodeIdx === predNodeIdx) {
      break;
    }

    currentLabel = nextLabel;
  }

  return chain;
}

export function getSegmentMatch(
  segment: DebugRouteSegment | null,
  matchIdx: number | null,
): { match: DebugMatch; kind: "start" | "dest" } | null {
  if (!segment || matchIdx === null) {
    return null;
  }

  const startMatch = segment.startMatches.find(
    (match) => match.matchIdx === matchIdx,
  );
  if (startMatch) {
    return { match: startMatch, kind: "start" };
  }

  const destMatch = segment.destMatches.find(
    (match) => match.matchIdx === matchIdx,
  );
  return destMatch ? { match: destMatch, kind: "dest" } : null;
}

export function getFinalRouteSegmentMatches(
  segment: DebugRouteSegment | null,
): {
  startMatch: DebugMatch | null;
  destMatch: DebugMatch | null;
} {
  if (!segment) {
    return { startMatch: null, destMatch: null };
  }

  const resolveStartMatch = (matchIdx: number | null | undefined) =>
    typeof matchIdx === "number"
      ? (segment.startMatches.find((match) => match.matchIdx === matchIdx) ??
        null)
      : null;
  const resolveDestMatch = (matchIdx: number | null | undefined) =>
    typeof matchIdx === "number"
      ? (segment.destMatches.find((match) => match.matchIdx === matchIdx) ??
        null)
      : null;

  let startMatch = resolveStartMatch(segment.selectedStartMatchIdx);
  let destMatch = resolveDestMatch(segment.selectedDestMatchIdx);

  if (startMatch || destMatch) {
    return { startMatch, destMatch };
  }

  const bestRouteCandidate = getRouteCandidates(segment)[0] ?? null;
  if (bestRouteCandidate) {
    destMatch = resolveDestMatch(bestRouteCandidate.matchIdx);
    const result =
      bestRouteCandidate.dirKey === "fwd"
        ? destMatch?.fwdResult
        : destMatch?.bwdResult;
    const startNodeIdx =
      result?.path?.[0]?.nodeIdx ?? bestRouteCandidate.startNodeIdx;
    startMatch =
      startNodeIdx === null
        ? null
        : (segment.startMatches.find(
            (match) => match.additionalNodeIdx === startNodeIdx,
          ) ?? null);
    return { startMatch, destMatch };
  }

  startMatch = resolveStartMatch(segment.beeline?.fromMatchIdx);
  destMatch = resolveDestMatch(segment.beeline?.toMatchIdx);
  return { startMatch, destMatch };
}

export function getPointMatchSelection(
  data: MapMatchDebugData | null,
  pointIdx: number | null,
): {
  segment: DebugRouteSegment | null;
  matches: DebugMatch[];
  kind: "start" | "dest" | null;
} {
  if (!data || pointIdx === null) {
    return { segment: null, matches: [], kind: null };
  }

  const segmentAsStart =
    data.routeSegments.find((segment) => segment.fromPointIdx === pointIdx) ??
    null;
  if (segmentAsStart) {
    return {
      segment: segmentAsStart,
      matches: segmentAsStart.startMatches,
      kind: "start",
    };
  }

  const segmentAsDest =
    data.routeSegments.find((segment) => segment.toPointIdx === pointIdx) ??
    null;
  if (segmentAsDest) {
    return {
      segment: segmentAsDest,
      matches: segmentAsDest.destMatches,
      kind: "dest",
    };
  }

  return { segment: null, matches: [], kind: null };
}
