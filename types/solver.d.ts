import type {
  CalibrationModelName,
  CalibrationValidation,
  CameraIntrinsics,
  Matrix4Array,
  ObservedTag,
  PlanarPoseCV,
  RadialResidualSummary,
  XrHistoryEntry,
} from './common.js';

export interface CalibrationRow {
  scope: string;
  captureTime: number;
  tags: ObservedTag[];
  serial?: number;
}

export interface CalibrationSolveResult {
  K: CameraIntrinsics;
  distCoeffs: [number, number, number, number, number];
  viewerFromCameraGL: Matrix4Array;
  worldFromRig: Matrix4Array;
  timeOffsetMs: number;
  cost: number;
  delaySpreadMs: number;
  convergedSeeds: number;
  usedViews: number;
  model: CalibrationModelName;
  assumptions: string[];
  validation: CalibrationValidation;
  solverVersion: number;
  independentlyValidated: false;
}

export function linear(A: number[][], b: number[]): number[] | null;
export function projectResidual(pose: PlanarPoseCV, tags: ObservedTag[], K: CameraIntrinsics): number[] | null;
export function summarizeRadialResiduals(
  rows: CalibrationRow[],
  K: CameraIntrinsics,
  options?: {
    poses?: Array<PlanarPoseCV | null> | null;
    history?: XrHistoryEntry[] | null;
    viewerFromCameraGL?: Matrix4Array | null;
    worldFromRig?: Matrix4Array | null;
    timeOffsetMs?: number;
  },
): RadialResidualSummary;
export function fitIntrinsics(
  rows: CalibrationRow[],
  size: [number, number],
  options?: { model?: CalibrationModelName; maxIterations?: number },
): {
  K: CameraIntrinsics;
  model: CalibrationModelName;
  rmsPx: number;
  std: number[];
  poses: Array<PlanarPoseCV | null>;
  schur: number[][] | null;
};
export function worldPoses(
  rows: CalibrationRow[],
  history: XrHistoryEntry[],
  K: CameraIntrinsics,
  X: Matrix4Array,
  offset: number,
): Array<Matrix4Array | null>;
export function consistency(
  worlds: Array<Matrix4Array | null>,
  reference?: Matrix4Array | null,
): {
  ok: boolean;
  positionP95Mm: number;
  angleP95Deg: number;
  reference?: Matrix4Array;
};
export function fitHandEye(
  rows: CalibrationRow[],
  history: XrHistoryEntry[],
  K: CameraIntrinsics,
): {
  viewerFromCameraGL: Matrix4Array;
  worldFromRig: Matrix4Array;
  timeOffsetMs: number;
  cost: number;
  delaySpreadMs: number;
  convergedSeeds: number;
  usedViews: number;
};
export function diversity(
  rows: CalibrationRow[],
  history: XrHistoryEntry[],
  imageSize?: [number, number] | null,
): {
  ok: boolean;
  maxAngleDeg?: number;
  rotationArea?: number;
  translationSpanM?: number;
  imageCentreSpanPx?: [number, number];
  maxCornerRadiusFrac?: number | null;
  visitedQuadrants?: number | null;
  reason: string;
};
export function solveCalibration(
  input: {
    rows: CalibrationRow[];
    history: XrHistoryEntry[];
    imageSize: [number, number];
    model?: CalibrationModelName;
  },
  progress?: (message: string) => void,
): CalibrationSolveResult;
