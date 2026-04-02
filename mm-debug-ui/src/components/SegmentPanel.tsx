import { useMemo, useState } from "react";
import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import {
  getRouteCandidates,
  getSegmentLabelNodes,
  segmentHasReachedDestination,
} from "../lib/segment-debug";
import type { DebugRouteSegment } from "../types";

type SegmentFilter = "all" | "routed" | "beeline" | "unreachable";

function formatDurationUs(durationUs: number | undefined): string {
  const safeDuration = durationUs ?? 0;
  return safeDuration >= 1000
    ? `${(safeDuration / 1000).toLocaleString("en-US")} ms`
    : `${safeDuration.toLocaleString("en-US")} us`;
}

function getSelectedSegment(
  segments: DebugRouteSegment[],
  selectedSegmentIdx: number | null,
): DebugRouteSegment | null {
  if (selectedSegmentIdx === null) {
    return null;
  }
  return (
    segments.find((segment) => segment.segmentIdx === selectedSegmentIdx) ??
    null
  );
}

export function SegmentPanel() {
  const {
    data,
    selectedSegmentIdx,
    highlightedSegmentIdx,
    selectedNodeIdx,
    selectedRouteCandidate,
    selectSegment,
    selectNode,
    selectWay,
    selectMatch,
    selectRouteCandidate,
    getNode,
    getWay,
    setHighlightedSegment,
    setHighlightedWay,
    setHighlightedMatch,
  } = useDebug();
  const [filter, setFilter] = useState<SegmentFilter>("all");

  const routeSegments = data?.routeSegments ?? [];
  const selectedSeg = getSelectedSegment(routeSegments, selectedSegmentIdx);

  const beelinedSegments = routeSegments.filter(
    (segment) => segment.allBeelined,
  ).length;
  const unreachableSegments = routeSegments.filter(
    (segment) => !segmentHasReachedDestination(segment),
  ).length;

  const filteredSegments = useMemo(() => {
    return routeSegments.filter((segment) => {
      switch (filter) {
        case "all":
          return true;
        case "routed":
          return !segment.allBeelined;
        case "beeline":
          return segment.allBeelined;
        case "unreachable":
          return !segmentHasReachedDestination(segment);
      }
    });
  }, [filter, routeSegments]);

  const routeCandidates = useMemo(
    () => (selectedSeg ? getRouteCandidates(selectedSeg) : []),
    [selectedSeg],
  );
  const bestRouteCandidate = routeCandidates[0] ?? null;
  const topRouteCandidates = routeCandidates.slice(0, 6);
  const labelNodes = useMemo(
    () => getSegmentLabelNodes(selectedSeg),
    [selectedSeg],
  );

  if (!data) return null;

  const filterBoxes: Array<{
    value: SegmentFilter;
    label: string;
    count: number;
    activeClassName: string;
  }> = [
    {
      value: "all",
      label: "All",
      count: routeSegments.length,
      activeClassName: "border-foreground/30 bg-accent text-foreground",
    },
    {
      value: "routed",
      label: "Routed",
      count: routeSegments.length - beelinedSegments,
      activeClassName: "border-primary/60 bg-primary/15 text-primary",
    },
    {
      value: "beeline",
      label: "Beeline",
      count: beelinedSegments,
      activeClassName:
        "border-destructive/50 bg-destructive/10 text-destructive",
    },
    {
      value: "unreachable",
      label: "No Route",
      count: unreachableSegments,
      activeClassName: "border-amber-300/50 bg-amber-400/10 text-amber-200",
    },
  ];

  return (
    <div className="flex flex-col h-full min-h-0 overflow-hidden">
      <div className="flex-1 flex flex-col min-h-0">
        <div className="px-4 py-3 border-b border-border bg-muted/30 shrink-0 space-y-3">
          <h2 className="text-sm font-medium text-muted-foreground uppercase tracking-wider text-[10px]">
            Route Segments ({data.routeSegments.length})
          </h2>
          <div className="grid grid-cols-4 gap-2 text-[10px]">
            {filterBoxes.map((box) => (
              <button
                key={box.value}
                type="button"
                className={cn(
                  "rounded border px-2 py-1.5 text-left transition-colors",
                  filter === box.value
                    ? box.activeClassName
                    : "border-border/60 bg-background/70 hover:bg-accent",
                )}
                onClick={() => setFilter(box.value)}
              >
                <div className="text-muted-foreground uppercase tracking-tight">
                  {box.label}
                </div>
                <div className="mt-0.5 font-semibold">{box.count}</div>
              </button>
            ))}
          </div>
          <div className="text-[10px] text-muted-foreground">
            Showing {filteredSegments.length} of {data.routeSegments.length}
          </div>
        </div>

        <div className="flex-1 overflow-y-auto p-4 space-y-1.5">
          {filteredSegments.map((segment) => {
            const isSelected = selectedSegmentIdx === segment.segmentIdx;
            const isHighlighted = highlightedSegmentIdx === segment.segmentIdx;
            const validStarts = segment.startMatches.filter(
              (match) => match.fwdNode || match.bwdNode,
            ).length;
            const validDests = segment.destMatches.filter(
              (match) => match.fwdResult?.reached || match.bwdResult?.reached,
            ).length;
            const costShare =
              data.metadata.maxSegmentCost > 0
                ? Math.min(
                    100,
                    (segment.minCost / data.metadata.maxSegmentCost) * 100,
                  )
                : 0;

            return (
              <Card
                key={segment.segmentIdx}
                className={cn(
                  "cursor-pointer border-border/50 px-2.5 py-1.5 transition-colors",
                  isSelected
                    ? "ring-2 ring-primary bg-accent border-primary/50"
                    : isHighlighted
                      ? "border-primary/40 bg-primary/10"
                      : "hover:bg-accent",
                )}
                onClick={() =>
                  selectSegment(isSelected ? null : segment.segmentIdx)
                }
                onMouseEnter={() => setHighlightedSegment(segment.segmentIdx)}
                onMouseLeave={() => setHighlightedSegment(null)}
              >
                <div className="flex items-center justify-between gap-3">
                  <div className="font-medium leading-none">
                    Segment #{segment.segmentIdx}
                  </div>
                  <Badge
                    variant={segment.allBeelined ? "destructive" : "default"}
                    className="h-4 px-1 text-[10px]"
                  >
                    {segment.allBeelined ? "Beeline" : "Routed"}
                  </Badge>
                </div>

                <div className="mt-1.5 h-1 overflow-hidden rounded-full bg-secondary/70">
                  <div
                    className={cn(
                      "h-full rounded-full",
                      segment.allBeelined ? "bg-destructive" : "bg-primary",
                    )}
                    style={{ width: `${costShare}%` }}
                  />
                </div>

                <div className="mt-1.5 flex flex-wrap items-center gap-x-3 gap-y-1 text-[10px] text-muted-foreground">
                  <span>
                    Start {validStarts}/{segment.startMatches.length}
                  </span>
                  <span>
                    Dest {validDests}/{segment.destMatches.length}
                  </span>
                  <span className="font-mono">
                    Cost {segment.minCost.toLocaleString("en-US")} -{" "}
                    {segment.maxCost.toLocaleString("en-US")}
                  </span>
                  <span className="tabular-nums">
                    {formatDurationUs(segment.dijkstraDurationUs)}
                  </span>
                </div>
              </Card>
            );
          })}

          {filteredSegments.length === 0 && (
            <div className="rounded border border-dashed border-border/70 bg-background/50 px-3 py-6 text-center text-xs text-muted-foreground">
              No segments match the current filter.
            </div>
          )}
        </div>
      </div>

      {selectedSeg && (
        <div className="flex-1 flex flex-col min-h-0 border-t-2 border-border bg-background/50">
          <div className="px-4 py-2 border-b border-border bg-muted/30 shrink-0 flex justify-between items-center">
            <h3 className="text-sm font-medium uppercase tracking-wider text-[10px]">
              Segment Details (#{selectedSeg.segmentIdx})
            </h3>
            <button
              onClick={() => selectSegment(null)}
              className="text-[10px] text-muted-foreground hover:text-foreground transition-colors"
            >
              Close
            </button>
          </div>

          <div className="flex-1 overflow-y-auto p-4 pt-2 space-y-3 text-xs">
            <div className="grid grid-cols-4 gap-1.5">
              <div className="rounded border border-border/50 bg-secondary/30 p-2">
                <div className="text-[10px] uppercase tracking-tight text-muted-foreground">
                  Start
                </div>
                <div className="mt-0.5 font-medium">
                  Point {selectedSeg.fromPointIdx}
                </div>
              </div>
              <div className="rounded border border-border/50 bg-secondary/30 p-2">
                <div className="text-[10px] uppercase tracking-tight text-muted-foreground">
                  Dest
                </div>
                <div className="mt-0.5 font-medium">
                  Point {selectedSeg.toPointIdx}
                </div>
              </div>
              <div className="rounded border border-border/50 bg-secondary/30 p-2">
                <div className="text-[10px] uppercase tracking-tight text-muted-foreground">
                  Node Labels
                </div>
                <div className="mt-0.5 font-medium">
                  {Object.keys(selectedSeg.nodeLabels ?? {}).length}
                </div>
              </div>
              <div className="rounded border border-border/50 bg-secondary/30 p-2">
                <div className="text-[10px] uppercase tracking-tight text-muted-foreground">
                  Extra Edges
                </div>
                <div className="mt-0.5 font-medium">
                  {selectedSeg.additionalEdgeWays?.length ?? 0}
                </div>
              </div>
            </div>

            <div className="grid grid-cols-2 gap-1.5">
              <div className="bg-secondary/40 rounded p-2 border border-border/50">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Min Cost
                </div>
                <div className="font-mono text-sm">
                  {selectedSeg.minCost.toLocaleString("en-US")}
                </div>
              </div>
              <div className="bg-secondary/40 rounded p-2 border border-border/50">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Max Cost
                </div>
                <div className="font-mono text-sm">
                  {selectedSeg.maxCost.toLocaleString("en-US")}
                </div>
              </div>
              <div className="bg-secondary/40 rounded p-2 border border-border/50">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Dijkstra Limit
                </div>
                <div className="font-mono text-sm">
                  {selectedSeg.dijkstraCostLimit.toLocaleString("en-US")}
                </div>
              </div>
              <div
                className={cn(
                  "bg-secondary/40 rounded p-2 border border-border/50",
                  selectedSeg.maxReachedInDijkstra &&
                    "bg-warning/10 border-warning/30",
                )}
              >
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Max Reached
                </div>
                <div className="font-medium text-sm">
                  {selectedSeg.maxReachedInDijkstra ? "Yes" : "No"}
                </div>
              </div>
              <div className="bg-secondary/40 rounded p-2 border border-border/50">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Early Termination
                </div>
                <div className="font-mono text-sm">
                  {selectedSeg.dijkstraEarlyTerminationMaxCost.toLocaleString(
                    "en-US",
                  )}
                </div>
              </div>
              <div className="bg-secondary/40 rounded p-2 border border-border/50">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Search Exit
                </div>
                <div className="font-medium text-[11px] leading-tight">
                  {selectedSeg.dijkstraTerminatedEarlyMaxCost
                    ? "Terminated (Cost)"
                    : selectedSeg.dijkstraRemainingDestinations == 0
                      ? "Completed (All)"
                      : "Stopped With Remaining"}
                  <div className="text-muted-foreground font-normal">
                    {selectedSeg.dijkstraRemainingDestinations ?? 0} remaining
                  </div>
                </div>
              </div>
              <div className="bg-secondary/40 rounded p-2 border border-border/50 col-span-2">
                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">
                  Search Duration
                </div>
                <div className="font-mono text-sm">
                  {selectedSeg.dijkstraDurationUs >= 1000
                    ? `${(selectedSeg.dijkstraDurationUs / 1000).toLocaleString("en-US")} ms`
                    : `${(selectedSeg.dijkstraDurationUs ?? 0).toLocaleString("en-US")} us`}
                </div>
              </div>
            </div>

            {selectedSeg.beeline && (
              <div className="bg-destructive/10 rounded p-2 border border-destructive/20">
                <div className="font-bold text-destructive uppercase text-[10px] mb-1">
                  Beeline Reason
                </div>
                <div className="text-muted-foreground leading-relaxed">
                  {selectedSeg.beeline.reason} -{" "}
                  {selectedSeg.beeline.distance.toLocaleString("en-US")}m -
                  Cost: {selectedSeg.beeline.cost}
                </div>
              </div>
            )}

            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <h4 className="font-bold text-[10px] uppercase tracking-wider text-muted-foreground">
                  Route Candidates
                </h4>
                <span className="text-[10px] text-muted-foreground">
                  {routeCandidates.length} reached
                </span>
              </div>

              {bestRouteCandidate ? (
                <button
                  type="button"
                  className={cn(
                    "w-full rounded border p-2.5 text-left transition-colors",
                    selectedRouteCandidate?.matchIdx ===
                      bestRouteCandidate.matchIdx &&
                      selectedRouteCandidate?.dir === bestRouteCandidate.dirKey
                      ? "border-cyan-300/60 bg-cyan-400/10"
                      : "border-primary/30 bg-primary/10 hover:bg-primary/15",
                  )}
                  onClick={() => {
                    selectRouteCandidate({
                      matchIdx: bestRouteCandidate.matchIdx,
                      dir: bestRouteCandidate.dirKey,
                    });
                    selectMatch(bestRouteCandidate.matchIdx);
                    selectWay(bestRouteCandidate.wayIdx);
                  }}
                >
                  <div className="flex items-start justify-between gap-3">
                    <div>
                      <div className="text-[10px] uppercase tracking-wide text-primary/80">
                        Best Route
                      </div>
                      <div className="mt-1 text-sm font-semibold">
                        {bestRouteCandidate.dir} cost{" "}
                        {bestRouteCandidate.cost.toLocaleString("en-US")}
                      </div>
                      <div className="mt-1 text-[10px] text-muted-foreground">
                        Match #{bestRouteCandidate.matchIdx} on way #
                        {bestRouteCandidate.wayIdx}
                      </div>
                    </div>
                    <Badge variant="default" className="h-5 px-1.5 text-[10px]">
                      Active
                    </Badge>
                  </div>
                  <div className="mt-1 text-[10px] text-muted-foreground">
                    Match #{bestRouteCandidate.matchIdx} on way #
                    {bestRouteCandidate.wayIdx}
                  </div>
                </button>
              ) : (
                <div className="rounded border border-dashed border-border/70 bg-background/40 px-3 py-2 text-[10px] text-muted-foreground">
                  No destination candidate reached the target for this segment.
                </div>
              )}

              {topRouteCandidates.length > 0 && (
                <div className="space-y-1">
                  {topRouteCandidates.map((candidate, index) => {
                    const way = getWay(candidate.wayIdx);

                    const isSelectedCandidate =
                      selectedRouteCandidate?.matchIdx === candidate.matchIdx &&
                      selectedRouteCandidate?.dir === candidate.dirKey;

                    return (
                      <button
                        key={candidate.key}
                        type="button"
                        className={cn(
                          "w-full rounded border px-2 py-1.5 text-left text-[10px] transition-colors",
                          isSelectedCandidate
                            ? "border-cyan-300/60 bg-cyan-400/10"
                            : index === 0
                              ? "border-primary/30 bg-primary/8 hover:bg-primary/12"
                              : "border-border/40 bg-secondary/20 hover:bg-accent",
                        )}
                        onClick={() => {
                          selectRouteCandidate({
                            matchIdx: candidate.matchIdx,
                            dir: candidate.dirKey,
                          });
                          selectMatch(candidate.matchIdx);
                          selectWay(candidate.wayIdx);
                        }}
                        onMouseEnter={() => {
                          setHighlightedWay(candidate.wayIdx);
                          setHighlightedMatch(candidate.matchIdx);
                        }}
                        onMouseLeave={() => {
                          setHighlightedWay(null);
                          setHighlightedMatch(null);
                        }}
                      >
                        <div className="flex items-center justify-between gap-2">
                          <div className="font-medium">
                            {candidate.dir} match #{candidate.matchIdx}
                          </div>
                          <div className="font-mono text-muted-foreground">
                            {candidate.cost.toLocaleString("en-US")}
                          </div>
                        </div>
                        <div className="mt-1 flex items-center justify-between gap-2 text-muted-foreground">
                          <span>
                            way #{candidate.wayIdx}
                            {way?.osmId ? ` / OSM ${way.osmId}` : ""}
                          </span>
                          <span>
                            {candidate.pathSteps} steps,{" "}
                            {candidate.geometryPoints} pts
                          </span>
                        </div>
                      </button>
                    );
                  })}
                </div>
              )}
            </div>

            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <h4 className="font-bold text-[10px] uppercase tracking-wider text-muted-foreground">
                  Reached Nodes
                </h4>
                <span className="text-[10px] text-muted-foreground">
                  {labelNodes.length} with labels
                </span>
              </div>

              {labelNodes.length > 0 ? (
                <>
                  <div className="max-h-56 space-y-1 overflow-y-auto rounded border border-border/50 bg-background/40 p-1.5">
                    {labelNodes.map((entry) => {
                      const node = getNode(entry.nodeIdx);
                      const isSelectedNode = selectedNodeIdx === entry.nodeIdx;

                      return (
                        <button
                          key={entry.nodeIdx}
                          type="button"
                          className={cn(
                            "w-full rounded border px-2 py-1.5 text-left text-[10px] transition-colors",
                            isSelectedNode
                              ? "border-primary/60 bg-primary/10"
                              : "border-border/40 bg-secondary/20 hover:bg-accent",
                          )}
                          onClick={() => selectNode(entry.nodeIdx)}
                        >
                          <div className="flex items-center justify-between gap-2">
                            <span className="font-medium text-foreground">
                              Node #{entry.nodeIdx}
                            </span>
                            <span className="font-mono text-muted-foreground">
                              {entry.minCost.toLocaleString("en-US")}
                            </span>
                          </div>
                          <div className="mt-1 flex items-center justify-between gap-2 text-muted-foreground">
                            <span>{entry.labels.length} labels</span>
                            <span className="truncate">
                              {node?.osmId
                                ? `OSM ${node.osmId}`
                                : `Internal ${node?.internalId ?? "-"}`}
                            </span>
                          </div>
                        </button>
                      );
                    })}
                  </div>

                  <div className="rounded border border-border/50 bg-secondary/20 px-3 py-2 text-[10px] text-muted-foreground">
                    Node details and the full predecessor chain are shown in the
                    right-side inspector.
                  </div>
                </>
              ) : (
                <div className="rounded border border-dashed border-border/70 bg-background/40 px-3 py-2 text-[10px] text-muted-foreground">
                  This segment has no exported node labels.
                </div>
              )}
            </div>

            <div>
              <h4 className="font-bold text-[10px] uppercase tracking-wider mb-2 text-muted-foreground">
                Start Matches ({selectedSeg.startMatches.length})
              </h4>
              <div className="space-y-1">
                {selectedSeg.startMatches.map((match) => {
                  const way = getWay(match.wayIdx);

                  return (
                    <div
                      key={match.matchIdx}
                      className="bg-secondary/20 rounded p-2 border border-border/30 cursor-pointer hover:bg-secondary/40 transition-colors group"
                      onMouseEnter={() => {
                        setHighlightedWay(match.wayIdx);
                        setHighlightedMatch(match.matchIdx);
                      }}
                      onMouseLeave={() => {
                        setHighlightedWay(null);
                        setHighlightedMatch(null);
                      }}
                      onClick={() => {
                        selectMatch(match.matchIdx);
                        selectWay(match.wayIdx);
                        selectRouteCandidate(null);
                      }}
                    >
                      <div className="flex justify-between items-start">
                        <span className="font-medium">Way #{match.wayIdx}</span>
                        {way?.osmId && (
                          <a
                            href={`https://openstreetmap.org/way/${way.osmId}`}
                            target="_blank"
                            rel="noopener noreferrer"
                            className="text-blue-400 hover:underline opacity-70 group-hover:opacity-100"
                            onClick={(event) => event.stopPropagation()}
                          >
                            OSM
                          </a>
                        )}
                      </div>
                      <div className="text-muted-foreground text-[10px] mt-0.5">
                        Dist: {match.distToWay.toFixed(1)}m | Line Segment:{" "}
                        {match.waySegmentIdx}
                      </div>
                      {match.startLabel && (
                        <div className="mt-1.5 text-[10px] text-muted-foreground">
                          Start cost{" "}
                          {match.startLabel.totalStartCost.toLocaleString(
                            "en-US",
                          )}{" "}
                          (
                          {match.startLabel.matchPenalty.toLocaleString(
                            "en-US",
                          )}{" "}
                          penalty)
                        </div>
                      )}
                    </div>
                  );
                })}
              </div>
            </div>

            <div>
              <h4 className="font-bold text-[10px] uppercase tracking-wider mb-2 text-muted-foreground">
                Dest Matches ({selectedSeg.destMatches.length})
              </h4>
              <div className="space-y-1">
                {selectedSeg.destMatches.map((match) => {
                  const way = getWay(match.wayIdx);

                  return (
                    <div
                      key={match.matchIdx}
                      className="bg-secondary/20 rounded p-2 border border-border/30 cursor-pointer hover:bg-secondary/40 transition-colors group"
                      onMouseEnter={() => {
                        setHighlightedWay(match.wayIdx);
                        setHighlightedMatch(match.matchIdx);
                      }}
                      onMouseLeave={() => {
                        setHighlightedWay(null);
                        setHighlightedMatch(null);
                      }}
                      onClick={() => {
                        selectMatch(match.matchIdx);
                        selectWay(match.wayIdx);
                        selectRouteCandidate(null);
                      }}
                    >
                      <div className="flex justify-between items-start">
                        <span className="font-medium">Way #{match.wayIdx}</span>
                        {way?.osmId && (
                          <a
                            href={`https://openstreetmap.org/way/${way.osmId}`}
                            target="_blank"
                            rel="noopener noreferrer"
                            className="text-blue-400 hover:underline opacity-70 group-hover:opacity-100"
                            onClick={(event) => event.stopPropagation()}
                          >
                            OSM
                          </a>
                        )}
                      </div>
                      <div className="flex gap-1.5 mt-1.5">
                        {match.fwdResult && (
                          <Badge
                            variant={
                              match.fwdResult.reached ? "default" : "outline"
                            }
                            className="text-[9px] px-1 h-3.5 leading-none"
                          >
                            FWD:{" "}
                            {match.fwdResult.reached
                              ? match.fwdResult.cost
                              : "✗"}
                          </Badge>
                        )}
                        {match.bwdResult && (
                          <Badge
                            variant={
                              match.bwdResult.reached ? "default" : "outline"
                            }
                            className="text-[9px] px-1 h-3.5 leading-none"
                          >
                            BWD:{" "}
                            {match.bwdResult.reached
                              ? match.bwdResult.cost
                              : "✗"}
                          </Badge>
                        )}
                      </div>
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
