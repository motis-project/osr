import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Badge } from "@/components/ui/badge";
import { useDebug, type ViewMode } from "../store";
import { PointsPanel } from "./PointsPanel";
import { SegmentPanel } from "./SegmentPanel";
import { RoutePanel } from "./RoutePanel";

export function Sidebar() {
    const { data, viewMode, setViewMode } = useDebug();

    if (!data) {
        return (
            <div className="w-80 h-full bg-card border-r border-border flex items-center justify-center p-4">
                <div className="text-center text-muted-foreground">
                    <p className="text-lg font-medium mb-2">No data loaded</p>
                    <p className="text-sm">Drag and drop a debug JSON file onto the page</p>
                </div>
            </div>
        );
    }

    return (
        <div className="w-96 h-full bg-card border-r border-border flex flex-col">
            {/* Header with metadata */}
            <div className="p-4 border-b border-border">
                <h1 className="text-lg font-semibold mb-2">Map Match Debug</h1>
                <div className="flex flex-wrap gap-2 text-xs">
                    <Badge variant="secondary">
                        {data.metadata.nInputPoints} points
                    </Badge>
                    <Badge variant="secondary">
                        {data.metadata.nRouteSegments} segments
                    </Badge>
                    <Badge variant="default">
                        {data.metadata.nRouted} routed
                    </Badge>
                    {data.metadata.nBeelined > 0 && (
                        <Badge variant="destructive">
                            {data.metadata.nBeelined} beelined
                        </Badge>
                    )}
                </div>
            </div>

            {/* Tabs */}
            <Tabs
                value={viewMode}
                onValueChange={(v) => setViewMode(v as ViewMode)}
                className="flex-1 flex flex-col overflow-hidden"
            >
                <TabsList className="mx-4 mt-2 grid w-[calc(100%-2rem)] grid-cols-3">
                    <TabsTrigger value="points">Points</TabsTrigger>
                    <TabsTrigger value="segment">Segments</TabsTrigger>
                    <TabsTrigger value="route">Route</TabsTrigger>
                </TabsList>

                <div className="flex-1 min-h-0 overflow-hidden">
                    <TabsContent value="points" className="h-full p-4 mt-0 overflow-y-auto">
                        <PointsPanel />
                    </TabsContent>
                    <TabsContent value="segment" className="h-full mt-0 flex flex-col overflow-hidden">
                        <SegmentPanel />
                    </TabsContent>
                    <TabsContent value="route" className="h-full p-4 mt-0 overflow-y-auto">
                        <RoutePanel />
                    </TabsContent>
                </div>
            </Tabs>
        </div>
    );
}
