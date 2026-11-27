import type {
  AnchorCaptureCandidate,
  AnchorQualityOptions,
  CameraSample,
  Matrix4Array,
  ObservedTool,
  XRFrameLike,
  XRReferenceSpaceLike,
  XRSessionLike,
  XrContext,
} from './common.js';

export type AnchorPhase =
  | 'idle'
  | 'collecting'
  | 'ready'
  | 'creating'
  | 'created'
  | 'tracking'
  | 'lost'
  | 'failed'
  | 'unsupported'
  | 'ended';

export const ANCHOR_QUALITY: Readonly<Required<Omit<AnchorQualityOptions, 'tagIds'>>>;

export interface AnchorSeed {
  worldFromL: Matrix4Array;
  sessionId: string | null;
  spaceEpoch: number;
  calibrationProfileId: string;
  generation: number;
  serials: number[];
  firstAtPerformanceMs: number;
  lastAtPerformanceMs: number;
  spanMs: number;
  positionSpreadMm: number;
  orientationSpreadDeg: number;
  maxReprojectionRmsPx: number;
}

export interface StableLCollectorState {
  count: number;
  required: number;
  spanMs: number;
  ready: boolean;
  reason: string;
  tagIds: number[];
  seenTagCount: number;
  spreadMm: number | null;
  spreadDeg: number | null;
  lastRmsPx: number | null;
  limits: AnchorQualityOptions;
  quality: AnchorSeed | null;
}

export interface StableLCollector {
  observe(
    input: {
      sample: CameraSample;
      tool: ObservedTool | null | undefined;
      candidate: AnchorCaptureCandidate | null | undefined;
      worldFromL: Matrix4Array | null | undefined;
    },
    context: XrContext,
    now: number,
  ): StableLCollectorState;
  reset(why?: string): void;
  setTagIds(nextIds: number[]): void;
  state(): StableLCollectorState;
  summary(): Omit<StableLCollectorState, 'tagIds' | 'quality'>;
}

export interface LAnchorState {
  phase: AnchorPhase;
  created: boolean;
  active: boolean;
  pending: boolean;
  createAttempts: number;
  posesTracked: number;
  posesMissing: number;
  createdAtPerformanceMs: number | null;
  creationStartedAtPerformanceMs: number | null;
  error: string | null;
  capability: { enabled: boolean | null; hasMethod: boolean; canAttempt: boolean } | null;
  quality: StableLCollectorState;
  seed: AnchorSeed | null;
  persistent: false;
}

export interface LAnchorControllerOptions {
  onEvent?: (type: string, detail: Record<string, unknown>) => void;
  onCreated?: (state: LAnchorState) => void;
  makeRigidTransform?: (
    position: { x: number; y: number; z: number },
    orientation: { x: number; y: number; z: number; w: number },
  ) => unknown;
  quality?: AnchorQualityOptions;
  creationTimeoutMs?: number;
  clock?: () => number;
}

export interface LAnchorController {
  begin(
    newSession: XRSessionLike,
    newReference: XRReferenceSpaceLike,
    newContext: XrContext,
    now: number,
  ): void;
  reanchor(now: number, reason?: string): boolean;
  observe(
    input: {
      sample: CameraSample;
      tool: ObservedTool | null | undefined;
      candidate: AnchorCaptureCandidate | null | undefined;
      worldFromL: Matrix4Array | null | undefined;
    },
    now: number,
  ): LAnchorState;
  update(frame: XRFrameLike, now: number, opts?: { viewerTracked?: boolean }): Matrix4Array | null;
  resetReference(newReference: XRReferenceSpaceLike, newContext: XrContext, now: number): void;
  setTagIds(nextIds: number[]): void;
  end(now: number, reason?: string): void;
  state(): LAnchorState;
  phase(): AnchorPhase;
  summary(): {
    phase: AnchorPhase;
    created: boolean;
    active: boolean;
    pending: boolean;
    posesTracked: number;
    posesMissing: number;
    quality: Omit<StableLCollectorState, 'tagIds' | 'quality'>;
  };
}

export function isRigidPose(array: ArrayLike<number> | null | undefined): boolean;
export function createStableLCollector(options?: AnchorQualityOptions): StableLCollector;
export function createLAnchorController(options?: LAnchorControllerOptions): LAnchorController;
