import type {
  AnchorAssociation,
  Matrix4Array,
  ReferenceObservation,
  Vec3,
  XrContext,
  XrHistoryEntry,
} from './common.js';

// src/tag-tap.mjs
export const INDEX_TIP_HOVER_OFFSET_M: number;
export function pointInTagFrame(
  tagMatrix: ArrayLike<number>,
  pointL: ArrayLike<number>,
): { u: number; v: number; n: number; centerDist: number };
export function tagSurfaceMetrics(
  tagMatrix: ArrayLike<number>,
  edgeMm: number,
  pointL: ArrayLike<number>,
): {
  u: number;
  v: number;
  n: number;
  half: number;
  du: number;
  dv: number;
  lateralOverflow: number;
  surfaceDist: number;
  centerDist: number;
};
export function isPinchHandEligible(opts?: {
  kind?: string;
  handedness?: string;
  pinHoldHand?: string | null;
  leftPalmFacingViewer?: boolean;
  leftMenuOpen?: boolean;
}): boolean;
export function classifyCursorProximity(
  minDistM: number,
  opts?: { hotDistM?: number; nearDistM?: number },
): 'hot' | 'near' | 'idle';
export function projectIndexSelectorPos(
  tipPos: ArrayLike<number> | null | undefined,
  refProximalPos: ArrayLike<number> | null | undefined,
  offsetM?: number,
): Vec3 | null;

// src/tag-callout.mjs
export const HOVER_CALLOUT_DEFAULTS: {
  cardWidthM: number;
  cardHeightM: number;
  pillWidthM: number;
  pillHeightM: number;
  minTableZ: number;
  minPillTableZ: number;
};
export function viewBasisAtPointL(
  targetL: Vec3,
  viewerPosL: Vec3 | null | undefined,
  viewerRightL: Vec3 | null | undefined,
  viewerUpL: Vec3 | null | undefined,
): { toEye: Vec3; screenRight: Vec3; screenUp: Vec3 };
export function computeHoverCardLayout(opts: {
  tagCenterL: Vec3;
  edgeMm?: number;
  viewerPosL?: Vec3 | null;
  viewerRightL?: Vec3 | null;
  viewerUpL?: Vec3 | null;
  handedness?: string | null;
  probePosL?: Vec3 | null;
  avoidPosL?: Vec3 | null;
  cardWidthM?: number;
  cardHeightM?: number;
  minTableZ?: number;
}): {
  cardPos: Vec3;
  anchorPos: Vec3;
  screenRight: Vec3;
  screenUp: Vec3;
  toEye: Vec3;
  sideSign: number;
};
export function computePinPillPositions(
  pins: Array<{ key: string; centerL: Vec3; edgeMm?: number }>,
  opts?: {
    viewerPosL?: Vec3 | null;
    viewerRightL?: Vec3 | null;
    viewerUpL?: Vec3 | null;
    pillWidthM?: number;
    pillHeightM?: number;
    minPillTableZ?: number;
  },
): Map<
  string,
  { pillPos: Vec3; anchorPos: Vec3; closePosL: Vec3; screenRight: Vec3; screenUp: Vec3 }
>;

// src/motion-guard.mjs
export const MOTION_GATE_DEFAULTS: Readonly<Record<string, number>>;
export const OPTICAL_BRIDGE_DEFAULTS: Readonly<Record<string, number>>;
export const CARRIED_MOTION_GATE_DEFAULTS: Readonly<Record<string, number>>;
export function viewerVelocityAt(
  history: XrHistoryEntry[],
  targetTimeMs: number,
  context: XrContext | null | undefined,
  opts?: { windowMs?: number; searchPadMs?: number },
): { angDegPerSec: number; linMPerSec: number; dtMs: number } | null;
export function resolveCameraInL(
  observation: ReferenceObservation | null | undefined,
  assoc: AnchorAssociation | null | undefined,
  options?: Record<string, number>,
): {
  lFromCamera: Matrix4Array;
  source: 'optical-l' | 'xr-history';
  opticalTagCount: number;
  opticalRmsPx: number | null;
  xrDeltaMm: number | null;
  xrDeltaDeg: number | null;
} | null;
export function evaluateMotionGate(
  velocity: { angDegPerSec: number; linMPerSec: number } | null | undefined,
  opts?: { source?: 'optical-l' | 'xr-history' },
  limits?: Record<string, number>,
): {
  confidence: number;
  frozen: boolean;
  angDegPerSec: number;
  linMPerSec: number;
  source: string;
  maxTagRmsPx: number;
};
export function evaluateCarriedJumpGuard(opts?: {
  prevMatrix?: Matrix4Array | null;
  nextMatrix?: Matrix4Array | null;
  dtSinceLastMs?: number;
  confidence?: number;
  pendingJump?: { atMs: number; matrix: Matrix4Array } | null;
  atMs?: number;
}): {
  accept: boolean;
  nextPendingJump: { atMs: number; matrix: Matrix4Array } | null;
  reason: string | null;
};

// src/hand-coupling.mjs
export const HAND_COUPLING_DEFAULTS: Readonly<Record<string, number>>;
export interface HandSample {
  handedness: 'left' | 'right';
  frameKind: 'palm' | 'wrist';
  xrTimeMs: number;
  lFromHand: Matrix4Array;
  palmCenter: Vec3;
  wrist: Vec3 | null;
  anchors: Vec3[];
  thumbTip: Vec3 | null;
  indexTip: Vec3 | null;
  middleTip: Vec3 | null;
  pinchMidL: Vec3 | null;
  pinchDistM: number | null;
}
export function buildHandSample(
  jointsInL: {
    wrist?: Vec3 | null;
    indexMcp?: Vec3 | null;
    middleMcp?: Vec3 | null;
    pinkyMcp?: Vec3 | null;
    thumbTip?: Vec3 | null;
    indexTip?: Vec3 | null;
    middleTip?: Vec3 | null;
    wristMatrix?: Matrix4Array | null;
  },
  xrTimeMs: number,
  handedness: 'left' | 'right',
): HandSample | null;
export function createHandHistory(opts?: { maxMs?: number; maxGapMs?: number; futureClampMs?: number }): {
  push(sample: HandSample | null | undefined): void;
  sampleAt(handedness: 'left' | 'right', targetTimeMs: number): HandSample | null;
  latest(handedness: 'left' | 'right', nowXrTimeMs: number, maxStaleMs?: number): HandSample | null;
  clear(): void;
  size(handedness: 'left' | 'right'): number;
};
export function createCoupledTrailBuffer(opts?: { minStepM?: number; minStepDeg?: number; maxPoints?: number }): {
  push(entry: { xrTimeMs: number; atMs?: number; pose: Matrix4Array }): boolean;
  commitUpTo(targetXrTimeMs: number): Array<{ xrTimeMs: number; atMs: number; pose: Matrix4Array }>;
  discard(): number;
  size(): number;
};
export function createTagHandCoupler(options?: Record<string, number>): {
  observeOptical(input: {
    tagId: number;
    lFromTag: Matrix4Array;
    targetXrTimeMs: number;
    handHistory: ReturnType<typeof createHandHistory>;
    opticalConfidence?: number;
  }): Record<string, unknown>;
  predict(tagId: number, handHistory: ReturnType<typeof createHandHistory>, nowXrTimeMs: number): Record<string, unknown>;
  isHandCarrying(handedness: 'left' | 'right', tagId?: number | null): boolean;
  stateOf(tagId: number): { mode: string; handedness: 'left' | 'right' | null; gripKind: string | null; leverArmM?: number };
  reset(tagId?: number | null): void;
};

// src/hand-gesture.mjs
export const FIST_GESTURE_DEFAULTS: Readonly<Record<string, number>>;
export function classifyFistPose(
  joints: Record<string, unknown>,
  opts?: Record<string, number>,
): Record<string, unknown>;
export function createFistToggleController(opts?: Record<string, number>): {
  update(input?: {
    pose?: Record<string, unknown> | null;
    inView?: boolean;
    speedMps?: number;
    enabled?: boolean;
    suppressed?: boolean;
    nowMs?: number;
  }): { phase: string; progress: number; showVisual: boolean; triggered: boolean; latched: boolean };
  reset(): void;
};

// src/perf-stats.mjs
export function createPerfTracker(opts?: { capacity?: number }): {
  recordFrame(mode: string, dtMs: number, cpuMs: number, renderCpuMs?: number): void;
  recordHudFrame(visible: boolean): void;
  recordHudUpload(slot: 'left' | 'right' | 'top', drawMs: number): void;
  recordAnchorStep(stepMm: number, stepDeg: number, jitterMm?: number | null, driftMm?: number | null): void;
  recordOpticalLag(lagMs: number): void;
  recordTagSpeed(speedMps: number, coupled?: boolean): void;
  reset(): void;
  snapshot(): Record<string, unknown>;
};

// src/adaptive-quality.mjs
export const ADAPTIVE_QUALITY_MODES: readonly ['auto', 'fluid', 'crisp'];
export const ADAPTIVE_QUALITY_DEFAULTS: Readonly<Record<string, unknown>>;
export function createAdaptiveQualityController(opts?: {
  mode?: 'auto' | 'fluid' | 'crisp';
  longRange?: boolean;
  [key: string]: unknown;
}): {
  setMode(nextMode: 'auto' | 'fluid' | 'crisp'): Record<string, unknown>;
  getMode(): 'auto' | 'fluid' | 'crisp';
  setLongRange(enabled: boolean): Record<string, unknown>;
  getLongRange(): boolean;
  reset(): void;
  update(input?: { angDegPerSec?: number; linMPerSec?: number; nowMs?: number; xrFps?: number | null }): Record<string, unknown>;
  state(): Record<string, unknown>;
  stats(): Record<string, unknown>;
};

// src/path-smoother.mjs
export const SPEED_RADIUS_DEFAULTS: Readonly<{
  minRadiusM: number;
  maxRadiusM: number;
  refSpeedMps: number;
  exponent: number;
}>;
export const RIBBON_SPEED_DEFAULTS: Readonly<{
  minHalfWidthM: number;
  maxHalfWidthM: number;
  refSpeedMps: number;
  exponent: number;
}>;
export function fuseCommittedTrailSegment<T extends { pose: Matrix4Array }>(
  confirmed: T[],
  opticalPose: Matrix4Array | null | undefined,
  opts?: { blendWeight?: number; maxStepCorrectionM?: number },
): T[];
export function smoothLineTail(
  rawPositions: ArrayLike<number>,
  outPositions: { [index: number]: number },
  count: number,
  opts?: { maxDeviationM?: number },
): void;
export function speedToTubeRadius(
  speedMps: number,
  opts?: Partial<typeof SPEED_RADIUS_DEFAULTS>,
): number;
export function speedToRibbonHalfWidth(
  speedMps: number,
  opts?: Partial<typeof RIBBON_SPEED_DEFAULTS>,
): number;
export function speedToHeatColor(
  speedMps: number,
  opts?: { refSpeedMps?: number; out?: Vec3 | Float32Array | null },
): Vec3 | Float32Array;
export function smoothAxisTail(
  rawAxes: ArrayLike<number>,
  outAxes: { [index: number]: number },
  count: number,
): void;

// src/tag-predictor.mjs
export const TAG_PREDICTOR_DEFAULTS: Readonly<Record<string, number>>;
export function createTagMotionPredictor(options?: Record<string, number>): {
  observe(tagId: number, lFromTag: Matrix4Array, obsTimeMs: number): { speedMps: number; angDegPerSec: number } | null;
  predict(tagId: number, nowTimeMs: number): Matrix4Array | null;
  speedOf(tagId: number): { speedMps: number; angDegPerSec: number } | null;
  reset(tagId?: number | null): void;
};

// src/version.mjs
export const APP_VERSION: string;
export const APP_BUILD: string;
export const APP_VERSION_LABEL: string;
