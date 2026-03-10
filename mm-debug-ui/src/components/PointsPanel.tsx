import { useState, useMemo } from "react";
import { useDebug } from "../store";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
} from "@/components/ui/select";
import { Textarea } from "@/components/ui/textarea";
import { Button } from "@/components/ui/button";
import { CopyIcon, CheckIcon } from "lucide-react";

export function PointsPanel() {
    const {
        data,
        selectedPointIdx,
        selectPoint,
        getWay,
        setHighlightedWay,
        setHighlightedMatch,
    } = useDebug();
    const [format, setFormat] = useState("simple");
    const [copied, setCopied] = useState(false);

    const formattedContent = useMemo(() => {
        if (!data) return "";
        switch (format) {
            case "simple":
                return data.inputPoints
                    .map((pt) => `${pt.lat.toFixed(6)}, ${pt.lng.toFixed(6)}`)
                    .join("\n");
            case "geojson":
                return JSON.stringify(
                    {
                        type: "FeatureCollection",
                        features: data.inputPoints.map((pt, idx) => ({
                            type: "Feature",
                            geometry: {
                                type: "Point",
                                coordinates: [pt.lng, pt.lat],
                            },
                            properties: {
                                index: idx,
                                level: pt.level,
                            },
                        })),
                    },
                    null,
                    2
                );
            case "cpp":
                return `std::vector{
${data.inputPoints
                        .map(
                            (pt) =>
                                `  osr::location{.pos_ = {${pt.lat.toFixed(6)}, ${pt.lng.toFixed(
                                    6
                                )}}, .lvl_ = osr::kNoLevel}`
                        )
                        .join(",\n")},
}`;
            default:
                return "";
        }
    }, [data, format]);

    const handleCopy = () => {
        navigator.clipboard.writeText(formattedContent);
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
    };

    if (!data) return null;

    return (
        <div className="space-y-4">
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

            <div className="pt-4 border-t border-border space-y-3">
                <div className="flex items-center justify-between">
                    <h2 className="text-sm font-medium text-muted-foreground">
                        Export Points
                    </h2>
                    <Select value={format} onValueChange={(v) => v && setFormat(v)}>
                        <SelectTrigger className="w-32 h-8 text-xs">
                            <SelectValue placeholder="Format" />
                        </SelectTrigger>
                        <SelectContent>
                            <SelectItem value="simple">Simple</SelectItem>
                            <SelectItem value="geojson">GeoJSON</SelectItem>
                            <SelectItem value="cpp">C++</SelectItem>
                        </SelectContent>
                    </Select>
                </div>

                <div className="relative group">
                    <Textarea
                        readOnly
                        value={formattedContent}
                        className="font-mono text-[10px] h-64 resize-none pr-10"
                    />
                    <Button
                        variant="ghost"
                        size="icon"
                        className="absolute top-2 right-4 h-7 w-7 opacity-0 group-hover:opacity-100 transition-opacity"
                        onClick={handleCopy}
                    >
                        {copied ? (
                            <CheckIcon className="h-4 w-4 text-green-500" />
                        ) : (
                            <CopyIcon className="h-4 w-4" />
                        )}
                    </Button>
                </div>
            </div>
        </div>
    );
}
