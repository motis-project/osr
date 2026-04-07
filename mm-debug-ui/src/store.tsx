import {
  createContext,
  useContext,
  useState,
  useCallback,
  type ReactNode,
} from "react";
import type { MapMatchDebugData, DebugWay, DebugNode } from "./types";

export type SelectedRouteCandidate = {
  matchIdx: number;
  dir: "fwd" | "bwd";
};

export type ViewMode = "points" | "segment" | "route";

interface DebugState {
  data: MapMatchDebugData | null;
  viewMode: ViewMode;
  selectedPointIdx: number | null;
  selectedSegmentIdx: number | null;
  highlightedSegmentIdx: number | null;
  selectedNodeIdx: number | null;
  selectedNodeLabelSignature: string | null;
  selectedWayIdx: number | null;
  selectedMatchIdx: number | null;
  selectedRouteCandidate: SelectedRouteCandidate | null;
  highlightedWayIdx: number | null;
  highlightedMatchIdx: number | null;
}

interface DebugContextType extends DebugState {
  loadData: (data: MapMatchDebugData) => void;
  setViewMode: (mode: ViewMode) => void;
  showSegment: (idx: number) => void;
  selectPoint: (idx: number | null) => void;
  selectSegment: (idx: number | null) => void;
  selectNode: (idx: number | null) => void;
  selectNodeLabel: (signature: string | null) => void;
  selectWay: (idx: number | null) => void;
  selectMatch: (idx: number | null) => void;
  selectRouteCandidate: (candidate: SelectedRouteCandidate | null) => void;
  clearInspectorSelection: () => void;
  getWay: (idx: number) => DebugWay | undefined;
  getNode: (idx: number) => DebugNode | undefined;
  getBoundingBox: () => [[number, number], [number, number]] | null;
  setHighlightedSegment: (idx: number | null) => void;
  setHighlightedWay: (idx: number | null) => void;
  setHighlightedMatch: (idx: number | null) => void;
}

const DebugContext = createContext<DebugContextType | null>(null);

export function DebugProvider({ children }: { children: ReactNode }) {
  const [state, setState] = useState<DebugState>({
    data: null,
    viewMode: "route",
    selectedPointIdx: null,
    selectedSegmentIdx: null,
    highlightedSegmentIdx: null,
    selectedNodeIdx: null,
    selectedNodeLabelSignature: null,
    selectedWayIdx: null,
    selectedMatchIdx: null,
    selectedRouteCandidate: null,
    highlightedWayIdx: null,
    highlightedMatchIdx: null,
  });

  const loadData = (data: MapMatchDebugData) => {
    setState({
      data,
      viewMode: "route",
      selectedPointIdx: null,
      selectedSegmentIdx: null,
      highlightedSegmentIdx: null,
      selectedNodeIdx: null,
      selectedNodeLabelSignature: null,
      selectedWayIdx: null,
      selectedMatchIdx: null,
      selectedRouteCandidate: null,
      highlightedWayIdx: null,
      highlightedMatchIdx: null,
    });
  };

  const setViewMode = (mode: ViewMode) => {
    setState((s) => ({
      ...s,
      viewMode: mode,
      selectedPointIdx: null,
      selectedSegmentIdx: null,
      highlightedSegmentIdx: null,
      selectedNodeIdx: null,
      selectedNodeLabelSignature: null,
      selectedWayIdx: null,
      selectedMatchIdx: null,
      selectedRouteCandidate: null,
      highlightedWayIdx: null,
      highlightedMatchIdx: null,
    }));
  };

  const selectPoint = (idx: number | null) => {
    setState((s) => ({ ...s, selectedPointIdx: idx }));
  };

  const showSegment = (idx: number) => {
    setState((s) => ({
      ...s,
      viewMode: "segment",
      selectedPointIdx: null,
      selectedSegmentIdx: idx,
      highlightedSegmentIdx: null,
      selectedNodeIdx: null,
      selectedNodeLabelSignature: null,
      selectedWayIdx: null,
      selectedMatchIdx: null,
      selectedRouteCandidate: null,
      highlightedWayIdx: null,
      highlightedMatchIdx: null,
    }));
  };

  const selectSegment = (idx: number | null) => {
    setState((s) => ({
      ...s,
      selectedSegmentIdx: idx,
      selectedNodeIdx: null,
      selectedNodeLabelSignature: null,
      selectedWayIdx: null,
      selectedMatchIdx: null,
      selectedRouteCandidate: null,
    }));
  };

  const selectNode = (idx: number | null) => {
    setState((s) => ({
      ...s,
      selectedNodeIdx: idx,
      selectedNodeLabelSignature:
        idx === null ? null : s.selectedNodeLabelSignature,
    }));
  };

  const selectNodeLabel = (signature: string | null) => {
    setState((s) => ({ ...s, selectedNodeLabelSignature: signature }));
  };

  const selectWay = (idx: number | null) => {
    setState((s) => ({ ...s, selectedWayIdx: idx }));
  };

  const selectMatch = (idx: number | null) => {
    setState((s) => ({ ...s, selectedMatchIdx: idx }));
  };

  const selectRouteCandidate = (candidate: SelectedRouteCandidate | null) => {
    setState((s) => ({ ...s, selectedRouteCandidate: candidate }));
  };

  const clearInspectorSelection = () => {
    setState((s) => ({
      ...s,
      selectedNodeIdx: null,
      selectedNodeLabelSignature: null,
      selectedWayIdx: null,
      selectedMatchIdx: null,
      selectedRouteCandidate: null,
    }));
  };

  const setHighlightedSegment = useCallback((idx: number | null) => {
    setState((s) =>
      s.highlightedSegmentIdx === idx
        ? s
        : { ...s, highlightedSegmentIdx: idx },
    );
  }, []);

  const setHighlightedWay = (idx: number | null) => {
    setState((s) => ({ ...s, highlightedWayIdx: idx }));
  };

  const setHighlightedMatch = (idx: number | null) => {
    setState((s) => ({ ...s, highlightedMatchIdx: idx }));
  };

  const getWay = useCallback(
    (idx: number) => state.data?.ways[idx],
    [state.data],
  );
  const getNode = useCallback(
    (idx: number) => state.data?.nodes[idx],
    [state.data],
  );

  const getBoundingBox = useCallback(():
    | [[number, number], [number, number]]
    | null => {
    if (!state.data) return null;

    // Check if bounding box is in metadata
    if (state.data.metadata.boundingBox) {
      return state.data.metadata.boundingBox;
    }

    // Compute from data
    let minLng = Infinity,
      minLat = Infinity,
      maxLng = -Infinity,
      maxLat = -Infinity;

    // From input points
    for (const pt of state.data.inputPoints) {
      minLng = Math.min(minLng, pt.lng);
      maxLng = Math.max(maxLng, pt.lng);
      minLat = Math.min(minLat, pt.lat);
      maxLat = Math.max(maxLat, pt.lat);
    }

    // From nodes
    for (const node of state.data.nodes) {
      if (node.pos) {
        minLng = Math.min(minLng, node.pos[0]);
        maxLng = Math.max(maxLng, node.pos[0]);
        minLat = Math.min(minLat, node.pos[1]);
        maxLat = Math.max(maxLat, node.pos[1]);
      }
    }

    // From final route geometry
    for (const pt of state.data.finalRoute.geometry) {
      minLng = Math.min(minLng, pt[0]);
      maxLng = Math.max(maxLng, pt[0]);
      minLat = Math.min(minLat, pt[1]);
      maxLat = Math.max(maxLat, pt[1]);
    }

    if (!isFinite(minLng)) return null;

    // Add some padding
    const padding = 0.01;

    return [
      [minLng - padding, minLat - padding],
      [maxLng + padding, maxLat + padding],
    ];
  }, [state.data]);

  return (
    <DebugContext.Provider
      value={{
        ...state,
        loadData,
        setViewMode,
        showSegment,
        selectPoint,
        selectSegment,
        selectNode,
        selectNodeLabel,
        selectWay,
        selectMatch,
        selectRouteCandidate,
        clearInspectorSelection,
        getWay,
        getNode,
        getBoundingBox,
        setHighlightedSegment,
        setHighlightedWay,
        setHighlightedMatch,
      }}
    >
      {children}
    </DebugContext.Provider>
  );
}

export function useDebug() {
  const ctx = useContext(DebugContext);
  if (!ctx) throw new Error("useDebug must be used within DebugProvider");
  return ctx;
}
