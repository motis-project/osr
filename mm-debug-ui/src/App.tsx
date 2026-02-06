import { useCallback, useState } from "react";
import { DebugProvider, useDebug } from "./store";
import { Sidebar } from "./components/Sidebar";
import { Map } from "./components/Map";
import type { MapMatchDebugData } from "./types";

function DropZone() {
    const { data, loadData } = useDebug();
    const [isDragging, setIsDragging] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const handleDrop = useCallback(
        async (e: React.DragEvent) => {
            e.preventDefault();
            setIsDragging(false);
            setError(null);

            const file = e.dataTransfer.files[0];
            if (!file) return;

            const isGzip = file.name.endsWith(".json.gz");
            if (!file.name.endsWith(".json") && !isGzip) {
                setError("Please drop a JSON or .json.gz file");
                return;
            }

            try {
                let text: string;
                if (isGzip) {
                    const decompressedStream = file.stream().pipeThrough(
                        new DecompressionStream("gzip")
                    );
                    text = await new Response(decompressedStream).text();
                } else {
                    text = await file.text();
                }

                const json = JSON.parse(text) as MapMatchDebugData;

                // Basic validation
                if (!json.metadata || !json.inputPoints || !json.routeSegments) {
                    setError("Invalid debug JSON format");
                    return;
                }

                loadData(json);
            } catch (err) {
                setError(`Failed to read or parse file: ${err}`);
            }
        },
        [loadData]
    );

    const handleDragOver = useCallback((e: React.DragEvent) => {
        e.preventDefault();
        setIsDragging(true);
    }, []);

    const handleDragLeave = useCallback((e: React.DragEvent) => {
        e.preventDefault();
        setIsDragging(false);
    }, []);

    return (
        <div
            className="h-screen w-screen flex dark"
            onDrop={handleDrop}
            onDragOver={handleDragOver}
            onDragLeave={handleDragLeave}
        >
            <Sidebar />
            <div className="flex-1 relative">
                <Map />

                {/* Drop overlay */}
                {isDragging && (
                    <div className="absolute inset-0 bg-primary/20 backdrop-blur-sm flex items-center justify-center z-50 pointer-events-none">
                        <div className="bg-card border-2 border-dashed border-primary rounded-lg p-8 text-center">
                            <p className="text-lg font-medium">Drop JSON or .json.gz file here</p>
                        </div>
                    </div>
                )}

                {/* Initial state overlay */}
                {!data && !isDragging && (
                    <div className="absolute inset-0 bg-background/80 flex items-center justify-center z-40">
                        <div className="text-center p-8">
                            <div className="w-16 h-16 mx-auto mb-4 rounded-full bg-secondary flex items-center justify-center">
                                <svg
                                    className="w-8 h-8 text-muted-foreground"
                                    fill="none"
                                    stroke="currentColor"
                                    viewBox="0 0 24 24"
                                >
                                    <path
                                        strokeLinecap="round"
                                        strokeLinejoin="round"
                                        strokeWidth={2}
                                        d="M7 16a4 4 0 01-.88-7.903A5 5 0 1115.9 6L16 6a5 5 0 011 9.9M15 13l-3-3m0 0l-3 3m3-3v12"
                                    />
                                </svg>
                            </div>
                            <h2 className="text-xl font-semibold mb-2">Map Match Debug Viewer</h2>
                            <p className="text-muted-foreground mb-4">
                                Drag and drop a debug JSON or .json.gz file to get started
                            </p>
                            {error && (
                                <p className="text-destructive text-sm">{error}</p>
                            )}
                        </div>
                    </div>
                )}

                {/* Error toast */}
                {error && data && (
                    <div className="absolute bottom-4 right-4 bg-destructive text-destructive-foreground px-4 py-2 rounded-lg shadow-lg">
                        {error}
                    </div>
                )}
            </div>
        </div>
    );
}

export function App() {
    return (
        <DebugProvider>
            <DropZone />
        </DebugProvider>
    );
}

export default App;