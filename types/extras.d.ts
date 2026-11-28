import type {
  AnchorAssociation,
  CameraIntrinsics,
  Matrix4Array,
  ObservedTag,
  PlanarPoseCV,
  ReferenceObservation,
  Vec3,
  XrContext,
  XrHistoryEntry,
} from './common.js';
import type { PerimeterLandmark } from './perimeter-fence.js';

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
export const TAG_ACTION_PILL_DEFAULTS: {
  minRestMs: number;
  maxRestPosDispM: number;
  minRestAxisDot: number;
  midAirMinZ: number;
  coupledMaxRestPosDispM: number;
  coupledMinRestAxisDot: number;
  holdMaxRestPosDispM: number;
  holdMinRestAxisDot: number;
  tagNearM: number;
  zoneNearM: number;
  pillNearM: number;
  exitNearM: number;
  belowOffsetM: number;
  sideHalfSpanM: number;
  pillSizeM: number;
  pillHotDistM: number;
  pillPokeDistM: number;
  pillReleaseDistM: number;
  minTableZ: number;
};
export interface TagRestState {
  anchorPos: Vec3 | null;
  anchorX: Vec3 | null;
  anchorZ: Vec3 | null;
  sinceMs: number | null;
}
export function createTagRestState(): TagRestState;
export function resetTagRestState(state: TagRestState | null | undefined): void;
export function updateTagRestState(
  state: TagRestState | null | undefined,
  tagMatrix: ArrayLike<number> | null | undefined,
  nowMs: number,
  opts?: {
    handCoupled?: boolean;
    pillsOpen?: boolean;
    minRestMs?: number;
    maxRestPosDispM?: number;
    minRestAxisDot?: number;
    midAirMinZ?: number;
    coupledMaxRestPosDispM?: number;
    coupledMinRestAxisDot?: number;
    holdMaxRestPosDispM?: number;
    holdMinRestAxisDot?: number;
  },
): { isResting: boolean; restingMs: number };
export function computeTagActionPillLayout(opts: {
  tagMatrix: ArrayLike<number>;
  edgeMm?: number;
  viewerPosL?: ArrayLike<number> | null;
  viewerRightL?: ArrayLike<number> | null;
  viewerUpL?: ArrayLike<number> | null;
  recording?: boolean;
  belowOffsetM?: number;
  sideHalfSpanM?: number;
  minTableZ?: number;
}): { zoneCenterL: Vec3; recPosL: Vec3; markPosL: Vec3 };
export function evaluateTagActionPills(opts?: {
  isResting?: boolean;
  recording?: boolean;
  recordAllActive?: boolean;
  hasPose?: boolean;
  wasOpen?: boolean;
  holding?: boolean;
  minTagDist?: number;
  minZoneDist?: number;
  minRecDist?: number;
  minMarkDist?: number;
  tagNearM?: number;
  zoneNearM?: number;
  pillNearM?: number;
  exitNearM?: number;
}): { showRec: boolean; showMark: boolean };

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
export const OPTICAL_BRIDGE_DEFAULTS: Readonly<{
  allowOpticalBridge: boolean;
  minTags: number;
  maxOpticalRmsPx: number;
  maxPerimeterRmsPx: number;
  minPerimeterSpanMinorM: number;
  maxTagOutlierPx: number;
  maxDivergenceM: number;
  maxDivergenceDeg: number;
}>;
export const ANCHOR_CORRECTION_DEFAULTS: Readonly<{
  maxPosM: number;
  maxAngDeg: number;
  tauSec: number;
  maxSpeedMps: number;
  maxAngSpeedDegPerSec: number;
  deadbandPosM: number;
  deadbandAngDeg: number;
  minConfidence: number;
  measAlpha: number;
}>;
export const CARRIED_MOTION_GATE_DEFAULTS: Readonly<Record<string, number>>;
export interface DampedAnchorCorrectionState {
  matrix: Matrix4Array;
  targetMatrix: Matrix4Array;
  posMm: number;
  angDeg: number;
  updates: number;
  active: boolean;
}
export interface DampedAnchorCorrection {
  observeOptical(opts?: {
    lFromCameraXrRaw?: Matrix4Array;
    lFromCameraOpt?: Matrix4Array;
    confidence?: number;
  }): DampedAnchorCorrectionState;
  step(dtSec: number): DampedAnchorCorrectionState;
  reset(): void;
  state(): DampedAnchorCorrectionState;
}
export function viewerVelocityAt(
  history: XrHistoryEntry[],
  targetTimeMs: number,
  context: XrContext | null | undefined,
  opts?: { windowMs?: number; searchPadMs?: number },
): { angDegPerSec: number; linMPerSec: number; dtMs: number } | null;
export function landmarkSpan2D(tags: ObservedTag[]): { majorM: number; minorM: number; count: number };
export function solveLandmarkConsensus(
  tags: ObservedTag[],
  K: CameraIntrinsics,
  opts?: {
    fixedIds?: number[];
    maxTagRmsPx?: number;
    minTags?: number;
    minMinorSpanM?: number;
  },
): {
  pose: PlanarPoseCV;
  inliers: ObservedTag[];
  outliers: Array<{ id: number; rmsPx: number }>;
  rmsPx: number;
  minorSpanM: number;
  majorSpanM: number;
} | null;
export function resolveCameraInL(
  observation: ReferenceObservation | null | undefined,
  assoc: AnchorAssociation | null | undefined,
  options?: {
    allowOpticalBridge?: boolean;
    minTags?: number;
    maxOpticalRmsPx?: number;
    maxPerimeterRmsPx?: number;
    minPerimeterSpanMinorM?: number;
    maxTagOutlierPx?: number;
    maxDivergenceM?: number;
    maxDivergenceDeg?: number;
    perimeterLandmarks?: PerimeterLandmark[];
    anchorCorrection?: Matrix4Array | null;
  },
): {
  lFromCamera: Matrix4Array;
  source: 'optical-l' | 'optical-perimeter' | 'xr-history';
  opticalTagCount: number;
  fixedTagCount?: number;
  perimeterTagCount?: number;
  opticalRmsPx: number | null;
  xrDeltaMm: number | null;
  xrDeltaDeg: number | null;
  inlierIds?: number[];
  outliers?: Array<{ id: number; rmsPx: number }>;
  baselineMinorM?: number;
  baselineMajorM?: number;
} | null;
export function createDampedAnchorCorrection(
  options?: Partial<Record<string, number>>,
): DampedAnchorCorrection;
export function evaluateMotionGate(
  velocity: { angDegPerSec: number; linMPerSec: number } | null | undefined,
  opts?: { source?: 'optical-l' | 'optical-perimeter' | 'xr-history' },
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
  predictAt(tagId: number, handHistory: ReturnType<typeof createHandHistory>, targetXrTimeMs: number): Matrix4Array | null;
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
export const TURBO_SPEED_DEFAULTS: Readonly<{
  refSpeedMps: number;
}>;
export const TRAIL_GAP_DEFAULTS: Readonly<{
  minGapMs: number;
  minGapDistM: number;
  hardJumpM: number;
  hardJumpMinDtMs: number;
}>;
export interface TrailGapEvaluation {
  isBreak: boolean;
  distM: number;
  dtMs: number;
  reason: 'explicit-break' | 'occlusion-gap' | 'occluded-jump' | null;
}
export function shouldStartNewTrailSegment(opts?: {
  prevPose?: Matrix4Array | null;
  nextPose?: Matrix4Array | null;
  dtMs?: number | null;
  explicitBreak?: boolean;
  options?: Partial<typeof TRAIL_GAP_DEFAULTS> | null;
}): TrailGapEvaluation;
export function fuseCommittedTrailSegment<T extends { pose: Matrix4Array }>(
  confirmed: T[],
  opticalPose: Matrix4Array | null | undefined,
  opts?: { blendWeight?: number; maxStepCorrectionM?: number },
): T[];
export function smoothLineTail(
  rawPositions: ArrayLike<number>,
  outPositions: { [index: number]: number },
  count: number,
  opts?: { maxDeviationM?: number; breakFlags?: ArrayLike<number> | null },
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
export function turboColormap(
  t: number,
  out?: Vec3 | Float32Array | null,
): Vec3 | Float32Array;
export function speedToTurboColor(
  speedMps: number,
  opts?: { refSpeedMps?: number; out?: Vec3 | Float32Array | null },
): Vec3 | Float32Array;
export function smoothAxisTail(
  rawAxes: ArrayLike<number>,
  outAxes: { [index: number]: number },
  count: number,
): void;
export const MOTION_WAKE_DEFAULTS: Readonly<{
  durationMs: number;
  maxPoints: number;
  minStepM: number;
  minIntervalMs: number;
  maxJumpM: number;
  headFilterTauSec: number;
  smoothPasses: number;
  maxSmoothDeviationM: number;
  minVisibleArcM: number;
  minRadiusM: number;
  maxRadiusM: number;
  radiusExponent: number;
  fadeExponent: number;
  maxOpacity: number;
}>;
export interface MotionWakeSample {
  count: number;
  positions: Float32Array;
  radii: Float32Array;
  fades: Float32Array;
  speeds: Float32Array;
  arcLengthM: number;
}
export interface MotionWakeBuffer {
  clear(): void;
  seed(pos3: ArrayLike<number>, nowMs: number): boolean;
  push(pos3: ArrayLike<number>, nowMs: number, opts?: { speedMps?: number | null }): boolean;
  sample(nowMs: number, opts?: { includeLiveHead?: boolean }): MotionWakeSample;
  hasPoints(): boolean;
}
export function createMotionWakeBuffer(
  options?: Partial<typeof MOTION_WAKE_DEFAULTS>,
): MotionWakeBuffer;
export const TRAIL_VERTEX_ROLE: Readonly<{
  NORMAL: 0;
  UNRELATED: 1;
  PINNED_OUTER: 2;
  PINNED_SINGLE: 3;
  HOVERED: 4;
  PINNED_PREVIEW: 5;
  PINNED_SPAN: 6;
}>;
export const TRAIL_MEASURE_STYLE_DEFAULTS: Readonly<{
  normalRadiusScale: number;
  unrelatedRadiusScale: number;
  hoveredRadiusScale: number;
  pinnedSingleRadiusScale: number;
  pinnedOuterRadiusScale: number;
  pinnedPreviewRadiusScale: number;
  pinnedSpanRadiusScale: number;
  normalFade: number;
  unrelatedFade: number;
  hoveredFade: number;
  pinnedSingleFade: number;
  pinnedOuterFade: number;
  pinnedPreviewFade: number;
  pinnedSpanFade: number;
  unrelatedGrayLinearRgb: readonly [number, number, number];
  pinnedSingleDesat: number;
  pinnedOuterDesat: number;
  hoveredLiftId: number;
  pinnedPreviewLiftId: number;
  pinnedSpanLiftId: number;
}>;
export interface TrailMeasurePointRef {
  tagId: number;
  segmentIndex: number;
  lineSeq?: number | null;
}
export interface TrailStrandFocus {
  key: string;
  tagId: number;
  segmentIndex: number;
  pinSeqs: number[];
  hoverSeqs: number[];
  pinTotal: number;
  pinCount: number;
  isHovered: boolean;
  state: 'hovered' | 'pinned-single' | 'pinned-span';
}
export interface TrailMeasureFocus {
  hasFocus: boolean;
  strands: Map<string, TrailStrandFocus>;
  pinnedStrands: Set<string>;
  hoveredStrands: Set<string>;
}
export interface TrailMeasureEntrySummary {
  hasFocus: boolean;
  hasActiveVertex: boolean;
  hasDimVertex: boolean;
  hasMixedLayers: boolean;
  segmentStates: Map<number, string>;
}
export function buildTrailMeasureFocus(opts?: {
  active?: boolean;
  pinnedPoints?: Iterable<TrailMeasurePointRef | null | undefined>;
  hoveredPoints?: Iterable<TrailMeasurePointRef | null | undefined>;
}): TrailMeasureFocus;
export function evaluateEntryTrailMeasureStyle(opts?: {
  tagId?: number;
  lineCount?: number;
  lineSegmentIds?: ArrayLike<number> | null;
  lineSeqs?: ArrayLike<number> | null;
  baseColors?: ArrayLike<number> | null;
  focus?: TrailMeasureFocus | null;
  baseRadiusM?: number;
  preserveChroma?: boolean;
  outRoles?: { [index: number]: number } | null;
  outRadii?: { [index: number]: number } | null;
  outFades?: { [index: number]: number } | null;
  outColors?: { [index: number]: number } | null;
  style?: Partial<typeof TRAIL_MEASURE_STYLE_DEFAULTS> | null;
}): TrailMeasureEntrySummary;
export function measureTrailArcLength(opts?: {
  linePositions?: ArrayLike<number> | null;
  lineCount?: number;
  lineSegmentIds?: ArrayLike<number> | null;
  lineSeqs?: ArrayLike<number> | null;
  lineBreaks?: ArrayLike<number> | null;
  segmentIndex?: number;
  seqA?: number | null;
  seqB?: number | null;
}): number | null;

// src/tag-predictor.mjs
export const TAG_PREDICTOR_DEFAULTS: Readonly<Record<string, number | boolean>>;
export function createTagMotionPredictor(options?: Record<string, number | boolean>): {
  observe(tagId: number, lFromTag: Matrix4Array, obsTimeMs: number): { speedMps: number; angDegPerSec: number } | null;
  seed(
    tagId: number,
    lFromTag: Matrix4Array,
    obsTimeMs: number,
    opts?: { velL?: Vec3 | null; angLocal?: Vec3 | null },
  ): { speedMps: number; angDegPerSec: number } | null;
  predict(tagId: number, nowTimeMs: number, opts?: { minLeadSec?: number } | null): Matrix4Array | null;
  speedOf(tagId: number): { speedMps: number; angDegPerSec: number } | null;
  offsetEstimate(tagId: number, nowTimeMs: number): number;
  trackingFade(tagId: number, nowTimeMs: number, viewerPos?: Vec3 | null): number;
  noteLoss(tagId: number): number;
  limitMps(): number;
  reset(tagId?: number | null): void;
};

// src/focus-scheduler.mjs
export const FOCUS_SCHEDULER_DEFAULTS: Readonly<Record<string, number>>;
export interface FocusSchedulerCandidate {
  tagId: number;
  pos: Vec3;
  halfPx?: number;
  minDistM?: number;
  maxDistM?: number;
  recording?: boolean;
  handCoupled?: boolean;
  moving?: boolean;
  settling?: boolean;
  nearHand?: boolean;
  locked?: boolean;
  stillDurationMs?: number;
  lastOpticalAtMs?: number;
  speedMps?: number;
  apparentEdgePx?: number;
}
export interface FocusSchedulerPlan {
  mode: 'single-focus' | 'multi-focus' | 'wide';
  rois: [number, number, number, number][] | undefined;
  tagIds: number[];
  activeTagIds: number[];
  searchingTagIds: number[];
  speedFirst: boolean;
  isBackgroundSweep: boolean;
  sweepDecimate: 1 | 1.5 | 2 | 3 | 4;
  skipReason: string | null;
}
export function projectTagGazeMetrics(
  posL: Vec3,
  camGLFromL: Matrix4Array,
  K: { fx: number; fy: number; cx: number; cy: number },
  width: number,
  height: number,
): { inFrustum: boolean; angleDeg: number; distM: number; u: number; v: number; zCV: number };
export function createFocusScheduler(options?: Record<string, number>): {
  plan(input?: {
    candidates?: FocusSchedulerCandidate[];
    worldFromAnchor?: Matrix4Array | null;
    worldFromViewer?: Matrix4Array | null;
    viewerFromCameraGL?: Matrix4Array | null;
    K?: { fx: number; fy: number; cx: number; cy: number } | null;
    width?: number;
    height?: number;
    nowMs?: number;
    farMode?: boolean;
    recentDetectMs?: number;
    maxForegroundTags?: number;
  }): FocusSchedulerPlan;
  reportObservation(input?: {
    seenIds?: Iterable<number>;
    focusTagIds?: number[];
    isFocusFrame?: boolean;
    isBackgroundSweep?: boolean;
    focusHit?: boolean;
    nowMs?: number;
  }): void;
  reset(): void;
  state(): {
    mode: 'single-focus' | 'multi-focus' | 'wide';
    primaryTagId: number | null;
    slotIds: number[];
    lastSweepAtMs: number;
    partialMissStreak: number;
  };
};

// src/version.mjs
export const APP_VERSION: string;
export const APP_BUILD: string;
export const APP_VERSION_LABEL: string;
