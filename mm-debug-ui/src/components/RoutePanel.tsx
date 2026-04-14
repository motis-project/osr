import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { cn } from "@/lib/utils";

export function RoutePanel() {
  const { data, highlightedSegmentIdx, setHighlightedSegment, showSegment } =
    useDebug();

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
              {data.finalRoute.totalCost}
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
            <Badge
              variant={beelinedSegs.length > 0 ? "destructive" : "secondary"}
            >
              {beelinedSegs.length}
            </Badge>
          </div>

          <div className="flex justify-between items-center">
            <span className="text-sm text-muted-foreground">
              Max Segment Cost
            </span>
            <span className="font-medium">{data.metadata.maxSegmentCost}</span>
          </div>

          <div className="flex justify-between items-center">
            <span className="text-sm text-muted-foreground">
              Total Duration
            </span>
            <span className="font-medium">
              {(data.totalDurationMs ?? 0).toLocaleString("en-US")} ms
            </span>
          </div>
        </div>
      </Card>

      {/* Geometry stats */}
      <Card className="p-4">
        <h3 className="text-sm font-medium mb-3">Geometry</h3>
        <div className="space-y-2 text-sm">
          <div className="flex justify-between">
            <span className="text-muted-foreground">Route Points</span>
            <span>
              {data.finalRoute.geometry.length.toLocaleString("en-US")}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-muted-foreground">Ways in Debug</span>
            <span>{data.ways.length.toLocaleString("en-US")}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-muted-foreground">Nodes in Debug</span>
            <span>{data.nodes.length.toLocaleString("en-US")}</span>
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
              className={cn(
                "flex cursor-pointer items-center gap-3 rounded-md border border-transparent px-2 py-2 text-xs transition-colors",
                highlightedSegmentIdx === seg.segmentIdx
                  ? "border-primary/40 bg-primary/10"
                  : "hover:border-border hover:bg-accent/60",
              )}
              onClick={() => showSegment(seg.segmentIdx)}
              onMouseEnter={() => setHighlightedSegment(seg.segmentIdx)}
              onMouseLeave={() => setHighlightedSegment(null)}
            >
              <span
                className={cn(
                  "w-8 text-muted-foreground transition-colors",
                  highlightedSegmentIdx === seg.segmentIdx && "text-primary",
                )}
              >
                #{seg.segmentIdx}
              </span>
              <div className="flex-1 h-2 bg-secondary rounded-full overflow-hidden">
                <div
                  className={`h-full ${seg.allBeelined ? "bg-destructive" : "bg-primary"}`}
                  style={{
                    width: `${Math.min(100, (seg.minCost / data.metadata.maxSegmentCost) * 100)}%`,
                  }}
                />
              </div>
              <span className="w-12 text-right font-medium">{seg.minCost}</span>
            </div>
          ))}
        </div>
      </div>

      {/* Caller-specific data */}
      {data.caller && (
        <div>
          <h3 className="text-sm font-medium mb-2 uppercase tracking-wider text-[10px] text-muted-foreground">
            Caller Debug Data
          </h3>
          <Card className="p-0 overflow-hidden">
            <textarea
              readOnly
              className="w-full h-48 p-3 text-[10px] font-mono bg-secondary/20 text-foreground border-none resize-none focus:outline-none"
              value={JSON.stringify(data.caller, null, 2)}
            />
          </Card>
        </div>
      )}
    </div>
  );
}
