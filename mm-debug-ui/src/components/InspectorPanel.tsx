import { useEffect, useMemo, useState } from "react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import { X } from "lucide-react";
import { useDebug } from "../store";
import {
  buildPredecessorChain,
  getLabelSignature,
  getRouteCandidates,
  getSegmentLabelNodes,
  getSegmentMatch,
  kInspectorNodeHoverEvent,
  sortDebugNodeLabels,
} from "../lib/segment-debug";

function formatBool(value: boolean | undefined): string {
  return value ? "Yes" : "No";
}

type InspectorProperty = {
  label: string;
  value: string;
};

function formatLevels(levels: number[] | undefined): string {
  return levels && levels.length > 0 ? levels.join(", ") : "-";
}

function OsmLink({ kind, id }: { kind: "node" | "way"; id?: number }) {
  if (id === undefined) {
    return <span className="font-mono">-</span>;
  }

  return (
    <a
      href={`https://www.openstreetmap.org/${kind}/${id}`}
      target="_blank"
      rel="noopener noreferrer"
      className="font-mono text-sky-400 hover:underline"
    >
      {id}
    </a>
  );
}

function PropertyGrid({ items }: { items: InspectorProperty[] }) {
  if (items.length === 0) {
    return null;
  }

  return (
    <div className="overflow-hidden rounded border border-border/60 bg-secondary/20">
      <table className="w-full border-collapse text-[11px]">
        <tbody>
          {items.map((item, index) => (
            <tr
              key={item.label}
              className={cn(
                index !== items.length - 1 && "border-b border-border/50",
              )}
            >
              <th className="w-32 border-r border-border/50 px-2 py-1 text-left align-top font-medium text-muted-foreground">
                {item.label}
              </th>
              <td className="px-2 py-1 align-top font-mono whitespace-pre-wrap wrap-break-word text-foreground">
                {item.value}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

export function InspectorPanel() {
  const {
    data,
    selectedSegmentIdx,
    selectedNodeIdx,
    selectedNodeLabelSignature,
    selectedWayIdx,
    selectedMatchIdx,
    selectedRouteCandidate,
    selectNode,
    selectNodeLabel,
    selectWay,
    clearInspectorSelection,
    getNode,
    getWay,
  } = useDebug();
  const [hoveredChainNodeIdx, setHoveredChainNodeIdx] = useState<number | null>(
    null,
  );

  const dispatchInspectorNodeHover = (nodeIdx: number | null) => {
    window.dispatchEvent(
      new CustomEvent(kInspectorNodeHoverEvent, {
        detail: { nodeIdx },
      }),
    );
  };

  const selectedSegment = useMemo(() => {
    if (!data || selectedSegmentIdx === null) {
      return null;
    }
    return (
      data.routeSegments.find(
        (segment) => segment.segmentIdx === selectedSegmentIdx,
      ) ?? null
    );
  }, [data, selectedSegmentIdx]);

  const labelNodes = useMemo(
    () => getSegmentLabelNodes(selectedSegment),
    [selectedSegment],
  );

  const selectedNodeLabels = useMemo(() => {
    if (selectedNodeIdx === null) {
      return null;
    }
    return (
      labelNodes.find((entry) => entry.nodeIdx === selectedNodeIdx) ?? null
    );
  }, [labelNodes, selectedNodeIdx]);

  const sortedSelectedNodeLabels = useMemo(() => {
    if (!selectedNodeLabels) {
      return null;
    }
    return {
      ...selectedNodeLabels,
      labels: sortDebugNodeLabels(selectedNodeLabels.labels),
    };
  }, [selectedNodeLabels]);

  useEffect(() => {
    if (!sortedSelectedNodeLabels) {
      if (selectedNodeLabelSignature !== null) {
        selectNodeLabel(null);
      }
      return;
    }

    const hasSelectedLabel =
      selectedNodeLabelSignature !== null &&
      sortedSelectedNodeLabels.labels.some(
        (label) => getLabelSignature(label) === selectedNodeLabelSignature,
      );

    if (!hasSelectedLabel) {
      selectNodeLabel(getLabelSignature(sortedSelectedNodeLabels.labels[0]));
    }
  }, [selectNodeLabel, selectedNodeLabelSignature, sortedSelectedNodeLabels]);

  const selectedLabel = useMemo(() => {
    if (!sortedSelectedNodeLabels || selectedNodeLabelSignature === null) {
      return null;
    }
    return (
      sortedSelectedNodeLabels.labels.find(
        (label) => getLabelSignature(label) === selectedNodeLabelSignature,
      ) ?? null
    );
  }, [selectedNodeLabelSignature, sortedSelectedNodeLabels]);

  const predecessorChain = useMemo(() => {
    if (!sortedSelectedNodeLabels || !selectedLabel || !selectedSegment) {
      return [];
    }
    return buildPredecessorChain(
      sortedSelectedNodeLabels.nodeIdx,
      selectedLabel,
      selectedSegment.nodeLabels,
    );
  }, [selectedLabel, sortedSelectedNodeLabels, selectedSegment]);

  const routeCandidates = useMemo(
    () => (selectedSegment ? getRouteCandidates(selectedSegment) : []),
    [selectedSegment],
  );

  const activeRouteCandidate = useMemo(() => {
    if (!selectedRouteCandidate) {
      return null;
    }
    return (
      routeCandidates.find(
        (candidate) =>
          candidate.matchIdx === selectedRouteCandidate.matchIdx &&
          candidate.dirKey === selectedRouteCandidate.dir,
      ) ?? null
    );
  }, [routeCandidates, selectedRouteCandidate]);

  const activeMatch = useMemo(
    () => getSegmentMatch(selectedSegment, selectedMatchIdx),
    [selectedMatchIdx, selectedSegment],
  );

  const activeWay = useMemo(() => {
    if (selectedWayIdx !== null) {
      return getWay(selectedWayIdx) ?? null;
    }
    if (activeMatch) {
      return getWay(activeMatch.match.wayIdx) ?? null;
    }
    return null;
  }, [activeMatch, getWay, selectedWayIdx]);

  const selectedNode =
    selectedNodeIdx !== null ? getNode(selectedNodeIdx) : null;
  const selectedNodeWayRefs =
    selectedNode?.wayIndices?.map((wayIdx) => ({
      wayIdx,
      way: getWay(wayIdx) ?? null,
    })) ?? [];
  const activeWayNodeRefs =
    activeWay?.nodeIndices?.map((nodeIdx) => ({
      nodeIdx,
      node: getNode(nodeIdx) ?? null,
    })) ?? [];
  const nodePropertyItems: InspectorProperty[] = selectedNode?.properties
    ? [
        { label: "Car", value: formatBool(selectedNode.properties.car) },
        { label: "Bike", value: formatBool(selectedNode.properties.bike) },
        { label: "Foot", value: formatBool(selectedNode.properties.foot) },
        { label: "Bus", value: formatBool(selectedNode.properties.bus) },
        {
          label: "Bus Penalty",
          value: formatBool(selectedNode.properties.busWithPenalty),
        },
        {
          label: "Restricted",
          value: formatBool(selectedNode.properties.isRestricted),
        },
        {
          label: "Entrance",
          value: formatBool(selectedNode.properties.isEntrance),
        },
        {
          label: "Elevator",
          value: formatBool(selectedNode.properties.isElevator),
        },
        {
          label: "Parking",
          value: formatBool(selectedNode.properties.isParking),
        },
        {
          label: "Multi Level",
          value: formatBool(selectedNode.properties.isMultiLevel),
        },
        {
          label: "From Level",
          value: selectedNode.properties.fromLevel.toString(),
        },
        {
          label: "To Level",
          value: selectedNode.properties.toLevel.toString(),
        },
        {
          label: "Levels",
          value: formatLevels(selectedNode.properties.levels),
        },
      ]
    : [];
  const wayPropertyItems: InspectorProperty[] = activeWay?.properties
    ? [
        { label: "Car", value: formatBool(activeWay.properties.car) },
        { label: "Bike", value: formatBool(activeWay.properties.bike) },
        { label: "Foot", value: formatBool(activeWay.properties.foot) },
        { label: "Bus", value: formatBool(activeWay.properties.bus) },
        {
          label: "Bus Penalty",
          value: formatBool(activeWay.properties.busWithPenalty),
        },
        {
          label: "Railway",
          value: formatBool(activeWay.properties.railway),
        },
        {
          label: "Rail Penalty",
          value: formatBool(activeWay.properties.railwayWithPenalty),
        },
        { label: "Ferry", value: formatBool(activeWay.properties.ferry) },
        {
          label: "Big Street",
          value: formatBool(activeWay.properties.isBigStreet),
        },
        {
          label: "Destination",
          value: formatBool(activeWay.properties.isDestination),
        },
        {
          label: "One Way Car",
          value: formatBool(activeWay.properties.onewayCar),
        },
        {
          label: "One Way Bike",
          value: formatBool(activeWay.properties.onewayBike),
        },
        {
          label: "One Way PSV",
          value: formatBool(activeWay.properties.onewayPsv),
        },
        {
          label: "Max Speed",
          value: `${activeWay.properties.maxSpeedKmh} km/h`,
        },
        {
          label: "Speed Limit",
          value: activeWay.properties.speedLimit.toString(),
        },
        {
          label: "From Level",
          value: activeWay.properties.fromLevel.toString(),
        },
        {
          label: "To Level",
          value: activeWay.properties.toLevel.toString(),
        },
        {
          label: "Elevator",
          value: formatBool(activeWay.properties.isElevator),
        },
        {
          label: "Sidewalk Separate",
          value: formatBool(activeWay.properties.isSidewalkSeparate),
        },
        { label: "Steps", value: formatBool(activeWay.properties.isSteps) },
        {
          label: "Parking",
          value: formatBool(activeWay.properties.isParking),
        },
        { label: "Ramp", value: formatBool(activeWay.properties.isRamp) },
        {
          label: "In Route",
          value: formatBool(activeWay.properties.inRoute),
        },
        {
          label: "Component",
          value: activeWay.properties.component.toString(),
        },
      ]
    : [];
  const hasContent =
    selectedNode !== null ||
    activeRouteCandidate !== null ||
    activeMatch !== null ||
    activeWay !== null;

  useEffect(() => {
    return () => {
      dispatchInspectorNodeHover(null);
    };
  }, []);

  if (!data || !hasContent) {
    return null;
  }

  return (
    <aside className="w-100 shrink-0 border-l border-border bg-background/95 backdrop-blur-sm flex flex-col min-h-0">
      <div className="flex items-center justify-between border-b border-border px-4 py-3">
        <div>
          <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
            Inspector
          </div>
          <div className="mt-1 text-sm font-semibold">
            Segment {selectedSegmentIdx ?? "-"}
          </div>
        </div>
        <Button
          variant="ghost"
          size="sm"
          className="h-8 w-8 p-0"
          onClick={clearInspectorSelection}
          aria-label="Close inspector"
          title="Close inspector"
        >
          <X className="h-4 w-4" />
        </Button>
      </div>

      <div className="flex-1 overflow-y-auto px-4 py-3 space-y-4 text-xs">
        {selectedNode && (
          <section className="space-y-2 rounded-lg border border-border/70 bg-card/70 p-3">
            <div className="flex items-start justify-between gap-3">
              <div>
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Node
                </div>
                <div className="mt-1 text-sm font-semibold">
                  Node #{selectedNode.idx}
                </div>
              </div>
              <Badge variant="secondary">
                {selectedNode.isAdditionalNode ? "Additional" : "Base"}
              </Badge>
            </div>

            <div className="grid grid-cols-2 gap-2 text-[11px]">
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Internal ID</div>
                <div className="mt-1 font-mono">{selectedNode.internalId}</div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">OSM ID</div>
                <div className="mt-1">
                  <OsmLink kind="node" id={selectedNode.osmId} />
                </div>
              </div>
            </div>

            <PropertyGrid items={nodePropertyItems} />

            {selectedNodeWayRefs.length > 0 && (
              <div className="space-y-2">
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Connected Ways
                </div>
                <div className="max-h-40 space-y-1 overflow-y-auto rounded border border-border/60 bg-secondary/20 p-1.5">
                  {selectedNodeWayRefs.map(({ wayIdx, way }) => (
                    <div
                      key={wayIdx}
                      className="flex items-center justify-between gap-3 rounded border border-border/40 bg-background px-2 py-1.5 text-[11px]"
                    >
                      <button
                        type="button"
                        className="min-w-0 flex-1 text-left font-medium text-foreground hover:text-primary"
                        onClick={() => selectWay(wayIdx)}
                      >
                        Way #{wayIdx}
                      </button>
                      <span>
                        <OsmLink kind="way" id={way?.osmId} />
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            )}

            {sortedSelectedNodeLabels && (
              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                    Labels
                  </div>
                  <span className="text-[10px] text-muted-foreground">
                    {sortedSelectedNodeLabels.labels.length} exported
                  </span>
                </div>
                <div className="space-y-1">
                  {sortedSelectedNodeLabels.labels.map((label) => {
                    const signature = getLabelSignature(label);
                    const isSelected = signature === selectedNodeLabelSignature;

                    return (
                      <button
                        key={signature}
                        type="button"
                        className={cn(
                          "w-full rounded border px-2 py-1.5 text-left transition-colors",
                          isSelected
                            ? "border-primary/60 bg-primary/10"
                            : "border-border/60 bg-background hover:bg-accent",
                        )}
                        onClick={() => selectNodeLabel(signature)}
                      >
                        <div className="font-mono text-[11px] leading-snug text-foreground break-all">
                          {label.label}
                        </div>
                        <div className="mt-1 flex items-center justify-between gap-3 text-[11px] text-muted-foreground">
                          <span className="min-w-0 flex-1 text-left">
                            {label.predNodeIdx !== null
                              ? `Predecessor: ${label.predNodeIdx}`
                              : "Root label"}
                          </span>
                          <span className="shrink-0 rounded border border-primary/25 bg-primary/10 px-1.5 py-0.5 font-mono font-semibold text-foreground">
                            {label.cost.toLocaleString("en-US")}
                          </span>
                        </div>
                      </button>
                    );
                  })}
                </div>
              </div>
            )}

            {predecessorChain.length > 0 && (
              <div className="space-y-2">
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Predecessor Chain
                </div>
                <div className="space-y-1">
                  {predecessorChain.map((entry, index) => {
                    const chainNode = getNode(entry.nodeIdx);

                    return (
                      <button
                        key={`${entry.nodeIdx}-${index}`}
                        type="button"
                        className={cn(
                          "w-full rounded border px-2 py-1.5 text-left transition-colors",
                          index === 0
                            ? "border-primary/60 bg-primary/10"
                            : hoveredChainNodeIdx === entry.nodeIdx
                              ? "border-amber-300/60 bg-amber-400/10"
                              : "border-border/60 bg-secondary/20 hover:bg-accent",
                        )}
                        onClick={() => {
                          selectNode(entry.nodeIdx);
                          selectNodeLabel(
                            entry.label ? getLabelSignature(entry.label) : null,
                          );
                        }}
                        onMouseEnter={() => {
                          setHoveredChainNodeIdx(entry.nodeIdx);
                          dispatchInspectorNodeHover(entry.nodeIdx);
                        }}
                        onMouseLeave={() => {
                          setHoveredChainNodeIdx(null);
                          dispatchInspectorNodeHover(null);
                        }}
                      >
                        <div className="flex items-center justify-between gap-2 text-[11px]">
                          <span className="font-medium">
                            Node #{entry.nodeIdx}
                          </span>
                          <span className="text-muted-foreground/80">
                            {chainNode?.osmId
                              ? `OSM ${chainNode.osmId}`
                              : `Internal ${chainNode?.internalId ?? "-"}`}
                          </span>
                        </div>
                        {entry.label ? (
                          <div className="mt-1 flex items-start justify-between gap-3 text-[11px] text-muted-foreground">
                            <span className="min-w-0 flex-1 break-all">
                              {entry.label.label}
                            </span>
                            <span className="shrink-0 rounded border border-primary/25 bg-primary/10 px-1.5 py-0.5 font-mono font-semibold text-foreground">
                              {entry.label.cost.toLocaleString("en-US")}
                            </span>
                          </div>
                        ) : (
                          <div className="mt-1 text-[11px] text-muted-foreground">
                            No exported label for this predecessor
                          </div>
                        )}
                      </button>
                    );
                  })}
                </div>
              </div>
            )}
          </section>
        )}

        {activeRouteCandidate && (
          <section className="space-y-2 rounded-lg border border-cyan-400/30 bg-cyan-500/5 p-3">
            <div className="flex items-start justify-between gap-3">
              <div>
                <div className="text-[10px] uppercase tracking-[0.18em] text-cyan-200/80">
                  Route Candidate
                </div>
                <div className="mt-1 text-sm font-semibold">
                  {activeRouteCandidate.dir} match #
                  {activeRouteCandidate.matchIdx}
                </div>
              </div>
              <Badge variant="secondary">Isolated On Map</Badge>
            </div>

            <div className="grid grid-cols-2 gap-2 text-[11px]">
              <div className="rounded border border-border/60 bg-secondary/20 p-2">
                <div className="text-muted-foreground">Cost</div>
                <div className="mt-1 font-mono">
                  {activeRouteCandidate.cost.toLocaleString("en-US")}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/20 p-2">
                <div className="text-muted-foreground">Way</div>
                <div className="mt-1 font-mono">
                  #{activeRouteCandidate.wayIdx}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/20 p-2">
                <div className="text-muted-foreground">Path Steps</div>
                <div className="mt-1 font-mono">
                  {activeRouteCandidate.pathSteps}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/20 p-2">
                <div className="text-muted-foreground">Geometry Points</div>
                <div className="mt-1 font-mono">
                  {activeRouteCandidate.geometryPoints}
                </div>
              </div>
            </div>
          </section>
        )}

        {activeMatch && (
          <section className="space-y-2 rounded-lg border border-border/70 bg-card/70 p-3">
            <div className="flex items-start justify-between gap-3">
              <div>
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Match
                </div>
                <div className="mt-1 text-sm font-semibold">
                  #{activeMatch.match.matchIdx} ({activeMatch.kind})
                </div>
              </div>
              <Badge variant="secondary">Way #{activeMatch.match.wayIdx}</Badge>
            </div>

            <div className="grid grid-cols-2 gap-2 text-[11px]">
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Distance To Way</div>
                <div className="mt-1 font-mono">
                  {activeMatch.match.distToWay.toFixed(2)} m
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Line Segment</div>
                <div className="mt-1 font-mono">
                  {activeMatch.match.waySegmentIdx}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Forward</div>
                <div className="mt-1 font-mono">
                  {activeMatch.match.fwdResult
                    ? activeMatch.match.fwdResult.reached
                      ? activeMatch.match.fwdResult.cost
                      : "unreached"
                    : "-"}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Backward</div>
                <div className="mt-1 font-mono">
                  {activeMatch.match.bwdResult
                    ? activeMatch.match.bwdResult.reached
                      ? activeMatch.match.bwdResult.cost
                      : "unreached"
                    : "-"}
                </div>
              </div>
            </div>

            {activeMatch.match.startLabel && (
              <div className="grid grid-cols-2 gap-2 text-[11px]">
                <div className="rounded border border-border/60 bg-secondary/25 p-2">
                  <div className="text-muted-foreground">Match Penalty</div>
                  <div className="mt-1 font-mono">
                    {activeMatch.match.startLabel.matchPenalty.toLocaleString(
                      "en-US",
                    )}
                  </div>
                </div>
                <div className="rounded border border-border/60 bg-secondary/25 p-2">
                  <div className="text-muted-foreground">Total Start Cost</div>
                  <div className="mt-1 font-mono">
                    {activeMatch.match.startLabel.totalStartCost.toLocaleString(
                      "en-US",
                    )}
                  </div>
                </div>
              </div>
            )}
          </section>
        )}

        {activeWay && (
          <section className="space-y-2 rounded-lg border border-border/70 bg-card/70 p-3">
            <div className="flex items-start justify-between gap-3">
              <div>
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Way
                </div>
                <div className="mt-1 text-sm font-semibold">
                  Way #{activeWay.idx}
                </div>
              </div>
              <Badge variant="secondary">
                {activeWay.isAdditionalEdge ? "Additional Edge" : "Network Way"}
              </Badge>
            </div>

            <div className="grid grid-cols-2 gap-2 text-[11px]">
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Internal ID</div>
                <div className="mt-1 font-mono">{activeWay.internalId}</div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">OSM ID</div>
                <div className="mt-1">
                  <OsmLink kind="way" id={activeWay.osmId} />
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Reverse</div>
                <div className="mt-1 font-mono">
                  {formatBool(activeWay.reverse)}
                </div>
              </div>
              <div className="rounded border border-border/60 bg-secondary/25 p-2">
                <div className="text-muted-foreground">Points</div>
                <div className="mt-1 font-mono">
                  {activeWay.geometry.length}
                </div>
              </div>
            </div>

            <PropertyGrid items={wayPropertyItems} />

            {activeWayNodeRefs.length > 0 && (
              <div className="space-y-2">
                <div className="text-[10px] uppercase tracking-[0.18em] text-muted-foreground">
                  Graph Nodes
                </div>
                <div className="max-h-48 space-y-1 overflow-y-auto rounded border border-border/60 bg-secondary/20 p-1.5">
                  {activeWayNodeRefs.map(({ nodeIdx, node }) => (
                    <div
                      key={nodeIdx}
                      className="flex items-center justify-between gap-3 rounded border border-border/40 bg-background px-2 py-1.5 text-[11px]"
                    >
                      <button
                        type="button"
                        className="min-w-0 flex-1 text-left font-medium text-foreground hover:text-primary"
                        onClick={() => selectNode(nodeIdx)}
                      >
                        Node #{nodeIdx}
                      </button>
                      <span>
                        <OsmLink kind="node" id={node?.osmId} />
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </section>
        )}
      </div>
    </aside>
  );
}
