import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { cn } from "@/lib/utils";

export function PointsPanel() {
    const { data, selectedPointIdx, selectPoint, getWay, setHighlightedWay, setHighlightedMatch } = useDebug();

    if (!data) return null;

    return (
        <div className="space-y-2">
            <h2 className="text-sm font-medium text-muted-foreground mb-3">
                Input Points ({data.inputPoints.length})
            </h2>

            {data.inputPoints.map((pt, idx) => {
                // Find matches for this point
                const segAsStart = data.routeSegments.find((s) => s.fromPointIdx === idx);
                const segAsDest = data.routeSegments.find((s) => s.toPointIdx === idx);
                const matches = segAsStart?.startMatches ?? segAsDest?.destMatches ?? [];
                const isSelected = selectedPointIdx === idx;

                return (
                    <Card
                        key={idx}
                        className={cn(
                            "p-3 cursor-pointer transition-colors hover:bg-accent",
                            isSelected && "ring-2 ring-primary bg-accent"
                        )}
                        onClick={() => selectPoint(isSelected ? null : idx)}
                    >
                        <div className="flex items-center justify-between mb-2">
                            <span className="font-medium">Point #{idx}</span>
                            {matches.length === 0 ? (
                                <Badge variant="destructive">No matches</Badge>
                            ) : (
                                <Badge variant="secondary">{matches.length} matches</Badge>
                            )}
                        </div>
                        <div className="text-xs text-muted-foreground font-mono">
                            {pt.lat.toFixed(6)}, {pt.lng.toFixed(6)}
                        </div>

                        {/* Show matches when selected */}
                        {isSelected && matches.length > 0 && (
                            <div className="mt-3 pt-3 border-t border-border space-y-2">
                                <p className="text-xs font-medium text-muted-foreground">Matches:</p>
                                {matches.map((m) => {
                                    const way = getWay(m.wayIdx);
                                    return (
                                        <div
                                            key={m.matchIdx}
                                            className="text-xs bg-secondary/50 rounded p-2 cursor-pointer hover:bg-secondary/70"
                                            onMouseEnter={() => { setHighlightedWay(m.wayIdx); setHighlightedMatch(m.matchIdx); }}
                                            onMouseLeave={() => { setHighlightedWay(null); setHighlightedMatch(null); }}
                                        >
                                            <div className="flex justify-between">
                                                <span>Way #{m.wayIdx}</span>
                                                <span className="text-muted-foreground">
                                                    {m.distToWay.toFixed(1)}m
                                                </span>
                                            </div>
                                            {way?.osmId && (
                                                <div className="mt-1">
                                                    <a
                                                        href={`https://openstreetmap.org/way/${way.osmId}`}
                                                        target="_blank"
                                                        rel="noopener noreferrer"
                                                        className="text-blue-400 hover:underline"
                                                        onClick={(e) => e.stopPropagation()}
                                                    >
                                                        OSM: {way.osmId}
                                                    </a>
                                                </div>
                                            )}
                                            {m.startLabel && (
                                                <div className="text-muted-foreground mt-1">
                                                    Penalty: {m.startLabel.matchPenalty},
                                                    Total: {m.startLabel.totalStartCost}
                                                </div>
                                            )}
                                            {(m.fwdResult || m.bwdResult) && (
                                                <div className="flex gap-2 mt-1">
                                                    {m.fwdResult && (
                                                        <Badge variant={m.fwdResult.reached ? "default" : "secondary"} className="text-[10px]">
                                                            fwd: {m.fwdResult.reached ? `${m.fwdResult.cost}` : "✗"}
                                                        </Badge>
                                                    )}
                                                    {m.bwdResult && (
                                                        <Badge variant={m.bwdResult.reached ? "default" : "secondary"} className="text-[10px]">
                                                            bwd: {m.bwdResult.reached ? `${m.bwdResult.cost}` : "✗"}
                                                        </Badge>
                                                    )}
                                                </div>
                                            )}
                                        </div>
                                    );
                                })}
                            </div>
                        )}
                    </Card>
                );
            })}
        </div>
    );
}
