import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";

export function RoutePanel() {
    const { data } = useDebug();

    if (!data) return null;

    const routedSegs = data.routeSegments.filter((s) => !s.allBeelined);
    const beelinedSegs = data.routeSegments.filter((s) => s.allBeelined);

    return (
        <div className="space-y-4">
            <h2 className="text-sm font-medium text-muted-foreground">
                Route Summary
            </h2>

            <Card className="p-4">
                <div className="space-y-3">
                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Total Cost</span>
                        <span className="text-lg font-semibold">
                            {data.finalRoute.totalCost}s
                        </span>
                    </div>

                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Input Points</span>
                        <span className="font-medium">{data.inputPoints.length}</span>
                    </div>

                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Segments</span>
                        <span className="font-medium">{data.routeSegments.length}</span>
                    </div>

                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Routed</span>
                        <Badge variant="default">{routedSegs.length}</Badge>
                    </div>

                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Beelined</span>
                        <Badge variant={beelinedSegs.length > 0 ? "destructive" : "secondary"}>
                            {beelinedSegs.length}
                        </Badge>
                    </div>

                    <div className="flex justify-between items-center">
                        <span className="text-sm text-muted-foreground">Max Segment Cost</span>
                        <span className="font-medium">{data.metadata.maxSegmentCost}s</span>
                    </div>
                </div>
            </Card>

            {/* Geometry stats */}
            <Card className="p-4">
                <h3 className="text-sm font-medium mb-3">Geometry</h3>
                <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                        <span className="text-muted-foreground">Route Points</span>
                        <span>{data.finalRoute.geometry.length}</span>
                    </div>
                    <div className="flex justify-between">
                        <span className="text-muted-foreground">Ways in Debug</span>
                        <span>{data.ways.length}</span>
                    </div>
                    <div className="flex justify-between">
                        <span className="text-muted-foreground">Nodes in Debug</span>
                        <span>{data.nodes.length}</span>
                    </div>
                </div>
            </Card>

            {/* Per-segment breakdown */}
            <div>
                <h3 className="text-sm font-medium mb-2">Segment Breakdown</h3>
                <div className="space-y-1">
                    {data.routeSegments.map((seg) => (
                        <div
                            key={seg.segmentIdx}
                            className="flex items-center gap-2 text-xs py-1"
                        >
                            <span className="w-8 text-muted-foreground">#{seg.segmentIdx}</span>
                            <div className="flex-1 h-2 bg-secondary rounded-full overflow-hidden">
                                <div
                                    className={`h-full ${seg.allBeelined ? "bg-destructive" : "bg-primary"}`}
                                    style={{
                                        width: `${Math.min(100, (seg.minCost / data.metadata.maxSegmentCost) * 100)}%`,
                                    }}
                                />
                            </div>
                            <span className="w-12 text-right">{seg.minCost}s</span>
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
}
