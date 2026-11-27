import type { CameraIntrinsics, Matrix4Array, PixelPoint, RigDefinition, Vec2, Vec3 } from './common.js';

export const FENCE_HEIGHT_M: number;
export const PERIMETER_TRIGGER_TAG_ID: number;
export const PERIMETER_OUTSIDE_HYSTERESIS_M: Readonly<{ enterOutsideM: number; exitOutsideM: number }>;
export const PERIMETER_LANDMARK_DEFAULTS: Readonly<{
  minKeyframes: number;
  maxKeyframes: number;
  minKeyframeDistM: number;
  minKeyframeAngleDeg: number;
  minBaselineM: number;
  minParallaxDeg: number;
  minSpanMs: number;
  minConfidence: number;
  minEdgePx: number;
  minCosView: number;
  maxSquareFitRmsMm: number;
  maxPosSpreadMm: number;
  maxYawSpreadDeg: number;
  maxReprojRmsPxXr: number;
  maxReprojRmsPxOptical: number;
  outlierDemoteStreak: number;
  severeOutlierPx: number;
  severeOutlierStreak: number;
  lockedJumpDemoteM: number;
}>;
export const PERIMETER_MARKERS: ReadonlyArray<Readonly<{ id: number; role: string }>>;

export type PerimeterCoordinateMode = 'planar-z0' | 'free-6dof';
export type PerimeterLandmarkState = 'tracking' | 'locked' | 'demoted' | 'fixed';

export interface PerimeterLandmark {
  id: number;
  role: string;
  x: number;
  y: number;
  yaw: number;
  points: [Vec2, Vec2, Vec2, Vec2];
  rmsPx: number;
  baselineM: number;
  parallaxDeg: number;
  views: number;
}

export interface PerimeterOutsideResult {
  outside: boolean;
  inside: boolean;
  signedDistanceM: number;
  edgeDistanceM: number;
}

export interface PerimeterCrossingResult {
  pos: Vec3;
  rawZ: number;
  overTop: boolean;
  normal: [number, number];
  edgeIndex: number;
  edgeT: number;
  arcS: number;
  perimeterM: number;
  distanceM: number;
  hit: 'intersection' | 'closest';
}

export interface PerimeterVertex {
  id: number;
  role: string;
  fixed: boolean;
  pos: Vec3;
  yaw?: number;
  matrix?: Matrix4Array;
  lastAt: number;
  updates: number;
  landmarkState?: PerimeterLandmarkState;
  landmarkRmsPx?: number | null;
}

export interface PerimeterFence {
  heightM: number;
  triggerTagId: number;
  isActive(): boolean;
  isOffered(): boolean;
  status(): 'idle' | 'offered' | 'active';
  getCoordinateMode(): PerimeterCoordinateMode;
  setCoordinateMode(nextMode: PerimeterCoordinateMode): PerimeterCoordinateMode;
  triggerMarker(): Omit<PerimeterVertex, 'fixed' | 'updates'> | null;
  activate(opts?: { force?: boolean }): boolean;
  deactivate(): boolean;
  setRig(nextRig: RigDefinition): void;
  reset(): void;
  observe(
    tagId: number,
    lFromTag: Matrix4Array,
    cameraPosL: Vec3 | null,
    atMs: number,
    opts?: {
      weight?: number;
      corners?: PixelPoint[] | null;
      lFromCamera?: Matrix4Array | null;
      K?: CameraIntrinsics | null;
      minEdgePx?: number;
      poseSource?: string;
    },
  ): {
    accepted: boolean;
    isNew: boolean;
    justOffered?: boolean;
    justLocked?: boolean;
    demoted?: boolean;
    locked?: boolean;
    landmarkState?: PerimeterLandmarkState;
    status?: 'idle' | 'offered' | 'active';
    id: number;
    role: string;
    pos: Vec3;
    reason?: string;
  } | null;
  remove(tagId: number): {
    removed: true;
    id: number;
    role: string;
    pos: Vec3;
    deactivated: boolean;
    status: 'idle' | 'offered' | 'active';
  } | null;
  reconcileView(opts?: {
    seenIds?: Iterable<number>;
    lFromCamera?: Matrix4Array | null;
    K?: { fx: number; fy: number; cx: number; cy: number; width: number; height: number } | null;
    atMs?: number;
    confidence?: number;
    occludedIds?: Iterable<number> | null;
    decimate?: number;
    priorRois?: ReadonlyArray<ReadonlyArray<number>> | null;
  }): {
    removed: Array<{ id: number; role: string; pos: Vec3 }>;
    deactivated: boolean;
    status: 'idle' | 'offered' | 'active';
  };
  step(dtSec: number): boolean;
  orderedVertices(): PerimeterVertex[];
  discoveredMarkers(): PerimeterVertex[];
  lockedLandmarks(): PerimeterLandmark[];
  lockedCount(): number;
  reportLandmarkInlier(tagId: number): boolean;
  reportLandmarkOutlier(
    tagId: number,
    opts?: { residualPx?: number },
  ): { demoted: boolean; id: number; role?: string; residualPx?: number; streak?: number } | null;
  dynamicCount(): number;
  isOutsideXY(
    posL: ArrayLike<number> | null | undefined,
    opts?: { wasOutside?: boolean; enterOutsideM?: number; exitOutsideM?: number },
  ): PerimeterOutsideResult & { active: boolean };
  locateCrossing(
    prevPosL: ArrayLike<number> | null | undefined,
    currPosL: ArrayLike<number> | null | undefined,
    opts?: { heightM?: number; minZ?: number },
  ): PerimeterCrossingResult | null;
  version(): number;
  snapshot(): {
    status: 'idle' | 'offered' | 'active';
    active: boolean;
    offered: boolean;
    coordinateMode: PerimeterCoordinateMode;
    triggerTagId: number;
    heightM: number;
    dynamicCount: number;
    lockedCount: number;
    totalDynamicMarkers: number;
    perimeterM: number;
    areaM2: number;
    vertices: Array<{
      id: number;
      role: string;
      fixed: boolean;
      posL: Vec3;
      updates: number;
      landmarkState: PerimeterLandmarkState;
      landmarkRmsPx: number | null;
    }>;
  };
}

export function isPerimeterTag(id: number): boolean;
export function perimeterRole(id: number): string | null;
export function extractRigAnchors(rig: RigDefinition | null | undefined): {
  origin: { id: number; role: string; fixed: true; pos: Vec3 };
  xArm: { id: number; role: string; fixed: true; pos: Vec3 };
  yArm: { id: number; role: string; fixed: true; pos: Vec3 };
};
export function projectPointToTablePlane(
  tagPosL: Vec3,
  cameraPosL?: Vec3 | null,
  opts?: {
    maxTableOffsetM?: number;
    minCameraHeightM?: number;
    minRayDropM?: number;
    maxHorizontalShiftM?: number;
  },
): Vec3;
export function tagCornersOnZ0(x: number, y: number, yaw: number, edgeM?: number): [Vec2, Vec2, Vec2, Vec2];
export function fitTagSquareOnZ0(
  cornersPx: ReadonlyArray<PixelPoint>,
  lFromCamera: Matrix4Array,
  K: CameraIntrinsics,
  opts?: {
    edgeM?: number;
    minCameraHeightM?: number;
    minRayDrop?: number;
    maxDistM?: number;
  },
): {
  x: number;
  y: number;
  yaw: number;
  squareRmsMm: number;
  cornersL: [Vec2, Vec2, Vec2, Vec2];
  rayDir: Vec3;
  distM: number;
  cosView: number;
} | null;
export function triangulateTagOnZ0(
  keyframes: ReadonlyArray<{ lFromCamera: Matrix4Array; pixels: ReadonlyArray<PixelPoint>; weight?: number }>,
  K: CameraIntrinsics,
  opts?: {
    edgeM?: number;
    maxIterations?: number;
    huberDeltaPx?: number;
  },
): {
  x: number;
  y: number;
  yaw: number;
  rmsPx: number;
  corners: [Vec2, Vec2, Vec2, Vec2];
  views: number;
  stdXYMm: number;
  stdYawDeg: number;
} | null;
export function polygonMetricsXY(vertices: Array<{ pos: Vec3 }>): { perimeterM: number; areaM2: number };
export function signedDistanceToPolygonXY(
  x: number,
  y: number,
  vertices: ReadonlyArray<{ pos?: ArrayLike<number> } | ArrayLike<number>>,
): { inside: boolean; signedDistanceM: number; edgeDistanceM: number };
export function evaluatePerimeterOutside(
  posL: ArrayLike<number> | null | undefined,
  vertices: ReadonlyArray<{ pos?: ArrayLike<number> } | ArrayLike<number>>,
  opts?: { wasOutside?: boolean; enterOutsideM?: number; exitOutsideM?: number },
): PerimeterOutsideResult;
export function locatePerimeterCrossing(
  prevPosL: ArrayLike<number> | null | undefined,
  currPosL: ArrayLike<number> | null | undefined,
  vertices: ReadonlyArray<{ pos?: ArrayLike<number> } | ArrayLike<number>>,
  opts?: { heightM?: number; minZ?: number },
): PerimeterCrossingResult | null;
export function isTablePointObservableInCamera(
  posL: Vec3,
  lFromCamera: Matrix4Array | null | undefined,
  K: { fx: number; fy: number; cx: number; cy: number; width: number; height: number } | null | undefined,
  opts?: {
    minDistM?: number;
    maxDistM?: number;
    minCosView?: number;
    borderMarginFrac?: number;
    minBorderMarginPx?: number;
    deadZoneM?: number;
    minExpectedMinorPx?: number;
    decimate?: number;
    priorRois?: ReadonlyArray<ReadonlyArray<number>> | null;
    tagEdgeM?: number;
  },
): boolean;
export function createPerimeterFence(
  rig: RigDefinition | null | undefined,
  opts?: {
    heightM?: number;
    triggerTagId?: number;
    smoothTauSec?: number;
    jumpDistanceM?: number;
    jumpWindowMs?: number;
    jumpConfirmM?: number;
    missingFramesToRemove?: number;
    missingMsToRemove?: number;
    minGraceAfterSeenMs?: number;
    minRemoveConfidence?: number;
    inViewMinDistM?: number;
    inViewMaxDistM?: number;
    inViewMinCosView?: number;
    inViewBorderMarginFrac?: number;
    inViewMinBorderMarginPx?: number;
    inViewDeadZoneM?: number;
    inViewMinExpectedMinorPx?: number;
    tagEdgeM?: number;
    coordinateMode?: PerimeterCoordinateMode;
    landmark?: Partial<Record<string, number>>;
  },
): PerimeterFence;
