import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { cn } from "@/lib/utils";

export function SegmentPanel() {
    const { data, selectedSegmentIdx, selectSegment, getWay, setHighlightedWay, setHighlightedMatch } = useDebug();

    if (!data) return null;

    const selectedSeg = selectedSegmentIdx !== null
        ? data.routeSegments[selectedSegmentIdx]
        : null;

    return (
        <div className="flex flex-col h-full min-h-0 overflow-hidden">
            {/* Top Half: Segment List */}
            <div className="flex-1 flex flex-col min-h-0">
                <div className="px-4 py-2 border-b border-border bg-muted/30 shrink-0">
                    <h2 className="text-sm font-medium text-muted-foreground uppercase tracking-wider text-[10px]">
                        Route Segments ({data.routeSegments.length})
                    </h2>
                </div>
                <div className="flex-1 overflow-y-auto p-4 space-y-2">
                    {data.routeSegments.map((seg) => {
                        const isSelected = selectedSegmentIdx === seg.segmentIdx;
                        const validStarts = seg.startMatches.filter(
                            (m) => m.fwdNode || m.bwdNode
                        ).length;
                        const validDests = seg.destMatches.filter(
                            (m) => m.fwdResult?.reached || m.bwdResult?.reached
                        ).length;

                        return (
                            <Card
                                key={seg.segmentIdx}
                                className={cn(
                                    "p-3 cursor-pointer transition-colors hover:bg-accent border-border/50",
                                    isSelected && "ring-2 ring-primary bg-accent border-primary/50"
                                )}
                                onClick={() => selectSegment(isSelected ? null : seg.segmentIdx)}
                            >
                                <div className="flex items-center justify-between mb-2">
                                    <span className="font-medium">
                                        Segment #{seg.segmentIdx}
                                    </span>
                                    <div className="flex gap-1">
                                        {seg.allBeelined && (
                                            <Badge variant="destructive" className="text-[10px] px-1 h-4">Beeline</Badge>
                                        )}
                                        {!seg.allBeelined && (
                                            <Badge variant="default" className="text-[10px] px-1 h-4">Routed</Badge>
                                        )}
                                    </div>
                                </div>
                                <div className="flex gap-2 text-xs">
                                    <Badge variant="secondary" className={cn("text-[10px] px-1 h-4", validStarts === 0 && "bg-destructive/20 text-destructive")}>
                                        {validStarts} starts
                                    </Badge>
                                    <Badge variant="secondary" className={cn("text-[10px] px-1 h-4", validDests === 0 && "bg-destructive/20 text-destructive")}>
                                        {validDests} dests
                                    </Badge>
                                    <span className="text-muted-foreground ml-auto text-[10px] font-mono">
                                        {seg.minCost.toLocaleString("en-US")} – {seg.maxCost.toLocaleString("en-US")}
                                    </span>
                                </div>
                            </Card>
                        );
                    })}
                </div>
            </div>

            {/* Bottom Half: Segment Details */}
            {selectedSeg && (
                <div className="flex-1 flex flex-col min-h-0 border-t-2 border-border bg-background/50">
                    <div className="px-4 py-2 border-b border-border bg-muted/30 shrink-0 flex justify-between items-center">
                        <h3 className="text-sm font-medium uppercase tracking-wider text-[10px]">Segment Details (#{selectedSeg.segmentIdx})</h3>
                        <button
                            onClick={() => selectSegment(null)}
                            className="text-[10px] text-muted-foreground hover:text-foreground transition-colors"
                        >
                            Close
                        </button>
                    </div>
                    <div className="flex-1 overflow-y-auto p-4 pt-2 space-y-3 text-xs">
                        <div className="grid grid-cols-2 gap-1.5">
                            <div className="bg-secondary/40 rounded p-2 border border-border/50">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Min Cost</div>
                                <div className="font-mono text-sm">{selectedSeg.minCost.toLocaleString("en-US")}</div>
                            </div>
                            <div className="bg-secondary/40 rounded p-2 border border-border/50">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Max Cost</div>
                                <div className="font-mono text-sm">{selectedSeg.maxCost.toLocaleString("en-US")}</div>
                            </div>
                            <div className="bg-secondary/40 rounded p-2 border border-border/50">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Dijkstra Limit</div>
                                <div className="font-mono text-sm">{selectedSeg.dijkstraCostLimit.toLocaleString("en-US")}</div>
                            </div>
                            <div className={cn("bg-secondary/40 rounded p-2 border border-border/50", selectedSeg.maxReachedInDijkstra && "bg-warning/10 border-warning/30")}>
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Max Reached</div>
                                <div className="font-medium text-sm">{selectedSeg.maxReachedInDijkstra ? "Yes" : "No"}</div>
                            </div>
                            <div className="bg-secondary/40 rounded p-2 border border-border/50">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Early Termination</div>
                                <div className="font-mono text-sm">{selectedSeg.dijkstraEarlyTerminationMaxCost.toLocaleString("en-US")}</div>
                            </div>
                            <div className="bg-secondary/40 rounded p-2 border border-border/50">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Status</div>
                                <div className="font-medium text-[11px] leading-tight">
                                    {selectedSeg.dijkstraTerminatedEarlyMaxCost ? "Terminated (Cost)" : (selectedSeg.dijkstraRemainingDestinations == 0 ? "Completed (All)" : "Incomplete")}
                                    <div className="text-muted-foreground font-normal">{selectedSeg.dijkstraRemainingDestinations ?? 0} remaining</div>
                                </div>
                            </div>
                            <div className="bg-secondary/40 rounded p-2 border border-border/50 col-span-2">
                                <div className="text-muted-foreground text-[10px] uppercase tracking-tight mb-0.5">Search Duration</div>
                                <div className="font-mono text-sm">
                                    {selectedSeg.dijkstraDurationUs >= 1000
                                        ? `${(selectedSeg.dijkstraDurationUs / 1000).toLocaleString("en-US")} ms`
                                        : `${(selectedSeg.dijkstraDurationUs ?? 0).toLocaleString("en-US")} μs`}
                                </div>
                            </div>
                        </div>

                        {selectedSeg.beeline && (
                            <div className="bg-destructive/10 rounded p-2 border border-destructive/20">
                                <div className="font-bold text-destructive uppercase text-[10px] mb-1">Beeline Reason</div>
                                <div className="text-muted-foreground leading-relaxed">
                                    {selectedSeg.beeline.reason} • {selectedSeg.beeline.distance.toLocaleString("en-US")}m • Cost: {selectedSeg.beeline.cost}
                                </div>
                            </div>
                        )}

                        <div>
                            <h4 className="font-bold text-[10px] uppercase tracking-wider mb-2 text-muted-foreground">Start Matches ({selectedSeg.startMatches.length})</h4>
                            <div className="space-y-1">
                                {selectedSeg.startMatches.map((m) => {
                                    const way = getWay(m.wayIdx);
                                    return (
                                        <div
                                            key={m.matchIdx}
                                            className="bg-secondary/20 rounded p-2 border border-border/30 cursor-pointer hover:bg-secondary/40 transition-colors group"
                                            onMouseEnter={() => { setHighlightedWay(m.wayIdx); setHighlightedMatch(m.matchIdx); }}
                                            onMouseLeave={() => { setHighlightedWay(null); setHighlightedMatch(null); }}
                                        >
                                            <div className="flex justify-between items-start">
                                                <span className="font-medium">Way #{m.wayIdx}</span>
                                                {way?.osmId && (
                                                    <a
                                                        href={`https://openstreetmap.org/way/${way.osmId}`}
                                                        target="_blank"
                                                        rel="noopener noreferrer"
                                                        className="text-blue-400 hover:underline opacity-70 group-hover:opacity-100"
                                                        onClick={(e) => e.stopPropagation()}
                                                    >
                                                        OSM
                                                    </a>
                                                )}
                                            </div>
                                            <div className="text-muted-foreground text-[10px] mt-0.5">
                                                Dist: {m.distToWay.toFixed(1)}m | Line Segment: {m.waySegmentIdx}
                                            </div>
                                        </div>
                                    );
                                })}
                            </div>
                        </div>

                        <div>
                            <h4 className="font-bold text-[10px] uppercase tracking-wider mb-2 text-muted-foreground">Dest Matches ({selectedSeg.destMatches.length})</h4>
                            <div className="space-y-1">
                                {selectedSeg.destMatches.map((m) => {
                                    const way = getWay(m.wayIdx);
                                    return (
                                        <div
                                            key={m.matchIdx}
                                            className="bg-secondary/20 rounded p-2 border border-border/30 cursor-pointer hover:bg-secondary/40 transition-colors group"
                                            onMouseEnter={() => { setHighlightedWay(m.wayIdx); setHighlightedMatch(m.matchIdx); }}
                                            onMouseLeave={() => { setHighlightedWay(null); setHighlightedMatch(null); }}
                                        >
                                            <div className="flex justify-between items-start">
                                                <span className="font-medium">Way #{m.wayIdx}</span>
                                                {way?.osmId && (
                                                    <a
                                                        href={`https://openstreetmap.org/way/${way.osmId}`}
                                                        target="_blank"
                                                        rel="noopener noreferrer"
                                                        className="text-blue-400 hover:underline opacity-70 group-hover:opacity-100"
                                                        onClick={(e) => e.stopPropagation()}
                                                    >
                                                        OSM
                                                    </a>
                                                )}
                                            </div>
                                            <div className="flex gap-1.5 mt-1.5">
                                                {m.fwdResult && (
                                                    <Badge variant={m.fwdResult.reached ? "default" : "outline"} className="text-[9px] px-1 h-3.5 leading-none">
                                                        FWD: {m.fwdResult.reached ? m.fwdResult.cost : "✗"}
                                                    </Badge>
                                                )}
                                                {m.bwdResult && (
                                                    <Badge variant={m.bwdResult.reached ? "default" : "outline"} className="text-[9px] px-1 h-3.5 leading-none">
                                                        BWD: {m.bwdResult.reached ? m.bwdResult.cost : "✗"}
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
