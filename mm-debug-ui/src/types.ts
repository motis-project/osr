// Types for map match debug JSON

export interface DebugMetadata {
    nInputPoints: number;
    nRouteSegments: number;
    nRouted: number;
    nBeelined: number;
    maxSegmentCost: number;
    boundingBox?: [[number, number], [number, number]]; // [[minLng, minLat], [maxLng, maxLat]]
}

export interface InputPoint {
    lat: number;
    lng: number;
    level: number;
}

export interface WayProperties {
    car: boolean;
    bike: boolean;
    foot: boolean;
    bus: boolean;
    tram: boolean;
    railway: boolean;
    isBigStreet: boolean;
    isDestination: boolean;
    onewayCar: boolean;
    onewayBike: boolean;
    maxSpeedKmh: number;
    speedLimit: number;
    fromLevel: number;
    toLevel: number;
    isElevator: boolean;
    isSidewalkSeparate: boolean;
    isSteps: boolean;
    isParking: boolean;
    isRamp: boolean;
    inRoute: boolean;
    component: number;
}

export interface NodeProperties {
    car: boolean;
    bike: boolean;
    foot: boolean;
    bus: boolean;
    isRestricted: boolean;
    isEntrance: boolean;
    isElevator: boolean;
    isParking: boolean;
    isMultiLevel: boolean;
    fromLevel: number;
    toLevel: number;
}

export interface DebugWay {
    idx: number;
    internalId: number;
    osmId?: number;
    isAdditionalEdge: boolean;
    fromNodeId?: number;
    toNodeId?: number;
    reverse?: boolean;
    underlyingWayIdx?: number;
    geometry: [number, number][]; // [lng, lat][]
    nodeIndices?: (number | null)[];
    nodeDistances?: number[];
    properties?: WayProperties;
}

export interface DebugNode {
    idx: number;
    internalId: number;
    osmId?: number;
    isAdditionalNode: boolean;
    pos: [number, number]; // [lng, lat]
    properties?: NodeProperties;
    wayIndices?: number[];
}

export interface DebugNodeRef {
    nodeIdx: number;
    wayPos: number;
    dir: "fwd" | "bwd";
}

export interface DebugMatchPointSegment {
    nodeIdx: number;
    distToNode: number;
    cost: number;
}

export interface DebugStartLabel {
    matchPenalty: number;
    prevSegmentCost: number;
    costOffset: number;
    totalStartCost: number;
}

export interface DebugRouteStep {
    nodeIdx: number;
    wayPos: number;
    dir: "fwd" | "bwd";
    cost: number;
    predNodeIdx: number | null;
    wayIdx: number | null;
}

export interface DebugRouteResult {
    reached: boolean;
    cost: number | null;
    path?: DebugRouteStep[];
    geometry?: [number, number][];
}

export interface DebugMatch {
    matchIdx: number;
    wayIdx: number;
    projectedPoint: [number, number];
    distToWay: number;
    waySegmentIdx: number;
    oneway: boolean;
    additionalNodeIdx: number;
    fwdNode: DebugNodeRef | null;
    bwdNode: DebugNodeRef | null;
    fwdOut: DebugMatchPointSegment | null;
    fwdIn: DebugMatchPointSegment | null;
    bwdOut: DebugMatchPointSegment | null;
    bwdIn: DebugMatchPointSegment | null;
    startLabel?: DebugStartLabel;
    fwdResult?: DebugRouteResult;
    bwdResult?: DebugRouteResult;
    beelineFromMatchIdx?: number;
    beelineDist?: number;
}

export interface DebugBeeline {
    reason: string;
    fromMatchIdx: number | null;
    toMatchIdx: number | null;
    fromPoint: [number, number];
    toPoint: [number, number];
    distance: number;
    cost: number;
}

export interface DebugNodeLabel {
    label: string;
    cost: number;
    predNodeIdx: number | null;
}

export interface DebugRouteSegment {
    segmentIdx: number;
    fromPointIdx: number;
    toPointIdx: number;
    allBeelined: boolean;
    minCost: number;
    maxCost: number;
    dijkstraCostLimit: number;
    dijkstraEarlyTerminationMaxCost: number;
    maxReachedInDijkstra: boolean;
    dijkstraTerminatedEarlyMaxCost: boolean;
    dijkstraRemainingDestinations: number;
    startMatches: DebugMatch[];
    destMatches: DebugMatch[];
    beeline?: DebugBeeline;
    nodeLabels?: Record<string, DebugNodeLabel[]>; // keyed by node array index
    additionalEdgeWays?: number[];
    dijkstraDurationUs: number;
}

export interface DebugFinalRoute {
    totalCost: number;
    geometry: [number, number][];
}

export interface MapMatchDebugData {
    metadata: DebugMetadata;
    inputPoints: InputPoint[];
    ways: DebugWay[];
    nodes: DebugNode[];
    routeSegments: DebugRouteSegment[];
    finalRoute: DebugFinalRoute;
    totalDurationMs: number;
}
