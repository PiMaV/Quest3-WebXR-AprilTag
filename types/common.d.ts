export type Matrix4Array = number[];
export type Vec2 = [number, number];
export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];

export interface PixelPoint {
  x: number;
  y: number;
}

export interface CameraIntrinsics {
  width: number;
  height: number;
  fx: number;
  fy: number;
  cx: number;
  cy: number;
}

export interface PlanarPoseCV {
  R: [Vec3, Vec3, Vec3];
  t: Vec3;
}

export interface RigTagSpec {
  id: number;
  corners: [Vec2, Vec2, Vec2, Vec2];
}

export interface RigToolSpec {
  id: string;
  tags: RigTagSpec[];
}

export interface RigDefinition {
  schema: 'webxr-apriltag-rig-v1';
  id: string;
  family: 'apriltag-36h11';
  units: 'm';
  cornerOrder?: string;
  tools: RigToolSpec[];
}

export interface RawMarker {
  id: number;
  family?: string;
  corners: [PixelPoint, PixelPoint, PixelPoint, PixelPoint];
  center?: PixelPoint;
  decision_margin?: number;
  softFallback?: boolean;
}

export interface ObservedTag {
  id: number;
  toolId: string;
  points: [Vec2, Vec2, Vec2, Vec2];
  pixels: [PixelPoint, PixelPoint, PixelPoint, PixelPoint];
}

export interface ObservedTool {
  id: string;
  scenePose: Matrix4Array | null;
  tagIds: number[];
  pointCount: number;
  mode: 'joint-rigid-tags' | 'single-tag';
  cameraFromTool: PlanarPoseCV;
  reprojectionRmsPx: number;
  perTagRmsPx: Record<string, number>;
}

export interface RejectedDetection {
  id: number;
  reason: string;
}

export interface UnassignedDetection {
  id: number;
  family: string;
  corners: PixelPoint[];
  registered: boolean;
  pose: PlanarPoseCV | null;
  softFallback?: boolean;
  workerSolved?: boolean;
  edgeMm?: number;
  minEdgePx?: number;
  areaPx?: number;
  rmsPx?: number;
  cosView?: number;
  depthM?: number;
}

export interface ReferenceObservation {
  layoutId: string;
  printScale: number;
  imageSize: [number, number];
  cameraModel: CameraIntrinsics;
  calibrated: boolean;
  scenePose: Matrix4Array | null;
  tags: ObservedTag[];
  tools: ObservedTool[];
  rejected: RejectedDetection[];
  unassignedDetections?: UnassignedDetection[];
}

export interface DeviceMode {
  sourceSize: [number, number];
  resizeMode: string | null;
  facingMode: string | null;
  zoom: number | null;
  focusMode: string | null;
}

export interface DeviceIdentity {
  key: string;
  cameraHash: string;
  mode: DeviceMode;
  rigHash: string;
  pipeline: string;
  trackLabel: string | null;
  userAgent: string;
  cacheable: boolean;
}

export interface CalibrationValidation {
  ok: boolean;
  passed: boolean;
  positionP95Mm: number;
  angleP95Deg: number;
  trainingViews: number;
  heldOutViews: number;
  trainingRmsPx: number;
  heldOutRmsPx: number;
  jointTrainingRmsPx: number;
  jointHeldOutRmsPx: number;
  planarFx: number;
  jointFx: number;
  intrinsicStd: number[];
  uncertainty: boolean;
  offsetSafe: boolean;
  translationSafe: boolean;
  delayIdentified: boolean;
  reference?: Matrix4Array;
}

export interface CalibrationVerification {
  ok: boolean;
  passed: boolean;
  positionP95Mm: number;
  angleP95Deg: number;
  reprojectionRmsPx?: number;
  views: number;
  checkedAt: string;
  kind: 'new-session-check' | 'new-capture-after-fit';
}

export interface CalibrationProfile {
  schema: 'webxr-apriltag-profile-v1';
  id: string;
  device: DeviceIdentity;
  imageSize: [number, number];
  K: CameraIntrinsics;
  distCoeffs: [number, number, number, number, number];
  viewerFromCameraGL: Matrix4Array;
  timeOffsetMs: number;
  cost: number;
  delaySpreadMs: number;
  convergedSeeds: number;
  usedViews: number;
  model: 'centered-square' | 'pinhole';
  assumptions: string[];
  validation: CalibrationValidation;
  verification: CalibrationVerification;
  createdAt: string;
  pipeline: string;
  independentlyValidated: false;
}

export interface XrContext {
  sessionId: string | null;
  spaceEpoch: number;
}

export interface XrHistoryEntry {
  xrRafTimeMs: number;
  worldFromViewer: Matrix4Array | null;
  worldFromAnchor?: Matrix4Array | null;
  anchorTrackingState?: string | null;
  sessionId?: string | null;
  spaceEpoch?: number;
  emulatedPosition?: boolean;
  scope?: string;
}

export interface VideoFrameMeta {
  callbackAtPerformanceMs: number;
  mediaTime: number;
  presentedFrames: number;
  captureTime: number | null;
  presentationTime: number | null;
  expectedDisplayTime: number | null;
  processingDuration: number | null;
}

export interface FrameEvidence {
  fresh: boolean;
  callbackAdvanced: boolean;
  pixelsChanged: boolean;
  fingerprint: string;
  callbackCount: number;
}

export type UnassignedTagEdgeSpec =
  | number
  | ({ default?: number } & Record<string | number, number | undefined>);

export interface CameraSettings {
  horizontalFovDeg?: number;
  printScale?: number;
  maxLongEdge?: number;
  maxHz?: number;
  decimate?: 1 | 1.5 | 2 | 3 | 4;
  detectMode?: 'full' | 'two-stage' | 'fast';
  trackRefIds?: number[] | null;
  solveUnassigned?: boolean;
  unassignedTagEdgeMm?: UnassignedTagEdgeSpec | null;
  priorRois?: [number, number, number, number][];
  focusRois?: [number, number, number, number][];
  focusSpeedFirst?: boolean;
  focusFallbackDecimate?: 1 | 1.5 | 2 | 3 | 4;
  experimentalCalibration?: CalibrationProfile | null;
}

export interface CameraSample {
  serial: number;
  generation: number;
  context: XrContext;
  mainThreadReadbackMs: number;
  decimate: 1 | 1.5 | 2 | 3 | 4;
  cameraModel: CameraIntrinsics;
  geometry: RigDefinition;
  submittedAtPerformanceMs: number;
  receivedAtPerformanceMs?: number;
  videoTimeAtDraw: number;
  videoFrameMetadata: VideoFrameMeta | null;
  presentedCallbacks: number;
  metadataMediaTimeDeltaS: number | null;
  capturedInRvfc: boolean;
  readbackMode: string;
  detectMode?: string;
  sourceSize: [number, number];
  imageSize: [number, number];
  settings: CameraSettings;
  screenOrientation: { angle: number | null; type: string | null };
  captureTimeKnown: boolean;
  frameSynchronizationValidated: boolean;
  frameEvidence?: FrameEvidence;
  workerPreprocessMs?: number;
  detectMs?: number;
  poseFitMs?: number;
  coarseMs?: number;
  roiMs?: number;
  roiCount?: number;
  priorRoiCount?: number;
  focusRoiCount?: number;
  focusHit?: boolean;
  focusMissedBoxes?: number;
}

export interface CameraState {
  running: boolean;
  starting: boolean;
  generation: number;
  presentedCallbacks: number;
  sourceFrames: number;
  requestedConstraints: MediaStreamConstraints | null;
  resizeModeFallback: boolean;
  lastError: { name: string; message: string; constraint: string | null } | null;
  rvfcStalled: boolean;
  decimate: 1 | 1.5 | 2 | 3 | 4;
  detectMode: string;
  frameStalled: boolean;
  evidenceAgeMs: number | null;
  trackState: string;
  trackMuted: boolean | null;
  trackLabel: string | null;
  trackSettings: MediaTrackSettings | null;
  trackCapabilities: MediaTrackCapabilities | null;
  sourceSize: [number, number];
}

export interface AnchorQualityOptions {
  minSamples?: number;
  minSpanMs?: number;
  maxWindowMs?: number;
  maxGapMs?: number;
  maxAgeMs?: number;
  maxRmsPx?: number;
  maxPositionSpreadM?: number;
  maxOrientationSpreadDeg?: number;
  softReject?: boolean;
  tagIds?: number[];
}

export interface AnchorCaptureCandidate extends XrContext {
  worldFromViewer: Matrix4Array;
  calibrationProfileId: string;
  association: 'validated-profile-at-capture-time';
  targetXrTimeMs: number;
  timeAligned: boolean;
}

export interface AnchorAssociation {
  candidate: AnchorCaptureCandidate;
  worldFromAnchor: Matrix4Array;
  worldFromCamera: Matrix4Array;
  targetXrTimeMs: number;
}

export interface XRRigidTransformLike {
  position: { x: number; y: number; z: number; w?: number };
  orientation: { x: number; y: number; z: number; w: number };
  matrix: Float32Array | number[];
}

export interface XRReferenceSpaceLike {
  addEventListener?(type: string, listener: EventListenerOrEventListenerObject): void;
  removeEventListener?(type: string, listener: EventListenerOrEventListenerObject): void;
}

export interface XRAnchorLike {
  anchorSpace: unknown;
  delete?(): void;
}

export interface XRInputSourceLike {
  handedness?: 'left' | 'right' | 'none';
  hand?: unknown;
  gamepad?: {
    buttons?: ReadonlyArray<{ pressed?: boolean; value?: number }>;
  } | null;
}

export interface XRSessionLike {
  enabledFeatures?: Iterable<string>;
  inputSources?: Iterable<XRInputSourceLike>;
  end?(): Promise<void>;
}

export interface XRFrameLike {
  session?: XRSessionLike;
  createAnchor?(pose: unknown, space: XRReferenceSpaceLike): Promise<XRAnchorLike>;
  getPose?(space: unknown, baseSpace: XRReferenceSpaceLike): {
    emulatedPosition?: boolean;
    transform?: { matrix?: ArrayLike<number> };
  } | null;
}
