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
        <div className="space-y-2">
            <h2 className="text-sm font-medium text-muted-foreground mb-3">
                Route Segments ({data.routeSegments.length})
            </h2>

            {/* Segment list */}
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
                            "p-3 cursor-pointer transition-colors hover:bg-accent",
                            isSelected && "ring-2 ring-primary bg-accent"
                        )}
                        onClick={() => selectSegment(isSelected ? null : seg.segmentIdx)}
                    >
                        <div className="flex items-center justify-between mb-2">
                            <span className="font-medium">
                                Segment #{seg.segmentIdx}
                            </span>
                            <div className="flex gap-1">
                                {seg.allBeelined && (
                                    <Badge variant="destructive">Beeline</Badge>
                                )}
                                {!seg.allBeelined && (
                                    <Badge variant="default">Routed</Badge>
                                )}
                            </div>
                        </div>
                        <div className="flex gap-2 text-xs">
                            <Badge variant="secondary" className={cn(validStarts === 0 && "bg-destructive/20")}>
                                {validStarts} starts
                            </Badge>
                            <Badge variant="secondary" className={cn(validDests === 0 && "bg-destructive/20")}>
                                {validDests} dests
                            </Badge>
                            <span className="text-muted-foreground ml-auto">
                                {seg.minCost} – {seg.maxCost}
                            </span>
                        </div>
                    </Card>
                );
            })}

            {/* Selected segment details */}
            {selectedSeg && (
                <div className="mt-4 pt-4 border-t border-border">
                    <h3 className="text-sm font-medium mb-3">Segment Details</h3>

                    <div className="space-y-3 text-xs">
                        <div className="grid grid-cols-2 gap-2">
                            <div className="bg-secondary/50 rounded p-2">
                                <div className="text-muted-foreground">Min Cost</div>
                                <div className="font-medium">{selectedSeg.minCost}</div>
                            </div>
                            <div className="bg-secondary/50 rounded p-2">
                                <div className="text-muted-foreground">Max Cost</div>
                                <div className="font-medium">{selectedSeg.maxCost}</div>
                            </div>
                            <div className="bg-secondary/50 rounded p-2">
                                <div className="text-muted-foreground">Dijkstra Cost Limit</div>
                                <div className="font-medium">{selectedSeg.dijkstraCostLimit}</div>
                            </div>
                            <div className={cn("bg-secondary/50 rounded p-2", selectedSeg.maxReachedInDijkstra && "bg-warning/20")}>
                                <div className="text-muted-foreground">Max Reached</div>
                                <div className="font-medium">{selectedSeg.maxReachedInDijkstra ? "Yes" : "No"}</div>
                            </div>
                        </div>

                        {selectedSeg.beeline && (
                            <div className="bg-destructive/20 rounded p-2">
                                <div className="font-medium text-destructive">Beeline</div>
                                <div className="text-muted-foreground">
                                    Reason: {selectedSeg.beeline.reason}<br />
                                    Distance: {selectedSeg.beeline.distance}m<br />
                                    Cost: {selectedSeg.beeline.cost}
                                </div>
                            </div>
                        )}

                        <div>
                            <h4 className="font-medium mb-2">Start Matches ({selectedSeg.startMatches.length})</h4>
                            {selectedSeg.startMatches.map((m) => {
                                const way = getWay(m.wayIdx);
                                return (
                                    <div
                                        key={m.matchIdx}
                                        className="bg-secondary/30 rounded p-2 mb-1 cursor-pointer hover:bg-secondary/50"
                                        onMouseEnter={() => { setHighlightedWay(m.wayIdx); setHighlightedMatch(m.matchIdx); }}
                                        onMouseLeave={() => { setHighlightedWay(null); setHighlightedMatch(null); }}
                                    >
                                        <div className="flex justify-between">
                                            <span>Way #{m.wayIdx}</span>
                                            {way?.osmId && (
                                                <a
                                                    href={`https://openstreetmap.org/way/${way.osmId}`}
                                                    target="_blank"
                                                    rel="noopener noreferrer"
                                                    className="text-blue-400 hover:underline"
                                                    onClick={(e) => e.stopPropagation()}
                                                >
                                                    OSM: {way.osmId}
                                                </a>
                                            )}
                                        </div>
                                        <div className="text-muted-foreground">
                                            Dist: {m.distToWay.toFixed(1)}m | Segment: {m.waySegmentIdx}
                                        </div>
                                    </div>
                                );
                            })}
                        </div>

                        <div>
                            <h4 className="font-medium mb-2">Dest Matches ({selectedSeg.destMatches.length})</h4>
                            {selectedSeg.destMatches.map((m) => {
                                const way = getWay(m.wayIdx);
                                return (
                                    <div
                                        key={m.matchIdx}
                                        className="bg-secondary/30 rounded p-2 mb-1 cursor-pointer hover:bg-secondary/50"
                                        onMouseEnter={() => { setHighlightedWay(m.wayIdx); setHighlightedMatch(m.matchIdx); }}
                                        onMouseLeave={() => { setHighlightedWay(null); setHighlightedMatch(null); }}
                                    >
                                        <div className="flex justify-between">
                                            <span>Way #{m.wayIdx}</span>
                                            {way?.osmId && (
                                                <a
                                                    href={`https://openstreetmap.org/way/${way.osmId}`}
                                                    target="_blank"
                                                    rel="noopener noreferrer"
                                                    className="text-blue-400 hover:underline"
                                                    onClick={(e) => e.stopPropagation()}
                                                >
                                                    OSM: {way.osmId}
                                                </a>
                                            )}
                                        </div>
                                        <div className="flex gap-2 mt-1">
                                            {m.fwdResult && (
                                                <Badge variant={m.fwdResult.reached ? "default" : "outline"} className="text-[10px]">
                                                    fwd: {m.fwdResult.reached ? `${m.fwdResult.cost}` : "✗"}
                                                </Badge>
                                            )}
                                            {m.bwdResult && (
                                                <Badge variant={m.bwdResult.reached ? "default" : "outline"} className="text-[10px]">
                                                    bwd: {m.bwdResult.reached ? `${m.bwdResult.cost}` : "✗"}
                                                </Badge>
                                            )}
                                        </div>
                                    </div>
                                );
                            })}
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}
