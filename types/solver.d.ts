import type {
  CalibrationValidation,
  CameraIntrinsics,
  Matrix4Array,
  ObservedTag,
  PlanarPoseCV,
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
  model: 'centered-square' | 'pinhole';
  assumptions: string[];
  validation: CalibrationValidation;
  independentlyValidated: false;
}

export function linear(A: number[][], b: number[]): number[] | null;
export function projectResidual(pose: PlanarPoseCV, tags: ObservedTag[], K: CameraIntrinsics): number[] | null;
export function fitIntrinsics(
  rows: CalibrationRow[],
  size: [number, number],
  options?: { model?: 'centered-square' | 'pinhole'; maxIterations?: number },
): {
  K: CameraIntrinsics;
  model: 'centered-square' | 'pinhole';
  rmsPx: number;
  std: number[];
  poses: Array<PlanarPoseCV | null>;
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
): {
  ok: boolean;
  maxAngleDeg?: number;
  rotationArea?: number;
  translationSpanM?: number;
  imageCentreSpanPx?: [number, number];
  reason: string;
};
export function solveCalibration(
  input: {
    rows: CalibrationRow[];
    history: XrHistoryEntry[];
    imageSize: [number, number];
    model?: 'centered-square' | 'pinhole';
  },
  progress?: (message: string) => void,
): CalibrationSolveResult;
