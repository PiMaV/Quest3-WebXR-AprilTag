import type {
  AnchorAssociation,
  CameraIntrinsics,
  CameraSample,
  Matrix4Array,
  PixelPoint,
  RawMarker,
  ReferenceObservation,
  Vec2,
  Vec3,
} from './common.js';

export interface ToolScanLimits {
  minSamples: number;
  minSpanMs: number;
  maxWindowMs: number;
  maxGapMs: number;
  maxAgeMs: number;
  minEdgePx: number;
  minAreaPx: number;
  maxRmsPx: number;
  maxPositionSpreadM: number;
  maxOrientationSpreadDeg: number;
  minCameraBaselineM: number;
  minViewAngleDeg: number;
  maxIncidenceDeg: number;
  outlierPositionM: number;
  outlierOrientationDeg: number;
  minInlierRatio: number;
  mode: string;
  captureRequestMs: number;
}

export const TOOL_SCAN_LIMITS: Readonly<ToolScanLimits>;
export const VISUAL_TOOL_SCAN_LIMITS: Readonly<ToolScanLimits>;

export interface IndependentToolSpec {
  id: string;
  tagId: number;
  role?: string;
  tagCorners: [Vec2, Vec2, Vec2, Vec2];
}

export interface IndependentToolsSpec {
  schema: 'webxr-apriltag-independent-tools-v1';
  units: 'm';
  family: 'apriltag-36h11';
  tools: IndependentToolSpec[];
}

export interface RegisteredToolEntry {
  id: string;
  tagId: number;
  role?: string;
  lFromTool: Matrix4Array;
  quality: Record<string, unknown>;
  capturedWithProfileId: string | null;
  acceptedAt: string;
}

export interface ToolLayout {
  schema: 'webxr-apriltag-layout-v1';
  units: 'm';
  rigHash: string;
  toolsHash: string;
  referenceToolId: 'origin-l';
  createdAt: string;
  assumption: string;
  tools: RegisteredToolEntry[];
}

export interface ToolScan {
  begin(): void;
  observe(
    observation: ReferenceObservation,
    sample: CameraSample,
    association: AnchorAssociation | null | undefined,
    now: number,
  ): Record<string, unknown>;
  state(now?: number): Record<string, unknown>;
  accept(now?: number, profileId?: string | null): RegisteredToolEntry | null;
  requestCapture(now?: number, profileId?: string | null): { status: string; tagId: number | null };
  reopen(tagId: number): boolean;
  layout(): ToolLayout;
  loadLayout(value: unknown): ToolLayout;
  resetPending(): void;
}

export function validateToolSpecs(
  spec: IndependentToolsSpec,
  opts?: { referenceTagIds?: number[] },
): IndependentToolsSpec;
export function observeIndependentTool(
  raw: RawMarker,
  spec: IndependentToolSpec,
  K: CameraIntrinsics,
  association: AnchorAssociation,
  limits?: ToolScanLimits,
): {
  id: string;
  tagId: number;
  lFromTool: Matrix4Array;
  cameraPositionL: Vec3;
  reprojectionRmsPx: number;
  areaPx: number;
  minEdgePx: number;
  incidenceDeg: number;
  pixels: PixelPoint[];
} | null;
export function createToolScan(options: {
  spec: IndependentToolsSpec;
  rigHash: string;
  toolsHash: string;
  limits?: Partial<ToolScanLimits>;
  onAccepted?: (entry: RegisteredToolEntry) => void;
}): ToolScan;
export function validateToolLayout(
  value: unknown,
  opts: { spec: IndependentToolsSpec; rigHash: string; toolsHash: string },
): ToolLayout;
export function createToolLayoutStore(
  opts: { spec: IndependentToolsSpec; rigHash: string; toolsHash: string },
  storage?: Pick<Storage, 'getItem' | 'setItem'>,
): {
  load(): Promise<ToolLayout | null>;
  save(value: unknown): Promise<ToolLayout>;
};
