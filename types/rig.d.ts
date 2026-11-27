import type {
  CameraIntrinsics,
  PixelPoint,
  PlanarPoseCV,
  RawMarker,
  ReferenceObservation,
  RigDefinition,
  Vec2,
} from './common.js';

export const WASM_PRINT_ORDER: readonly [0, 1, 2, 3];

export interface PlanarTagInput {
  id?: number;
  points: Vec2[];
  pixels: PixelPoint[];
}

export interface SingleTagPoseOptions {
  edgeMm?: number | null;
  getTagEdgeMm?: ((markerId: number | undefined) => number | null | undefined) | null;
  minEdgePx?: number;
  minAreaPx?: number;
  maxRmsPx?: number;
  minCosView?: number;
  maxDepthM?: number;
}

export interface SingleTagPoseResult {
  edgeMm: number;
  pose: PlanarPoseCV;
  corners: [PixelPoint, PixelPoint, PixelPoint, PixelPoint];
  minEdgePx: number;
  areaPx: number;
  rmsPx: number;
  cosView: number;
  depthM: number;
}

export interface ObserveReferenceOptions {
  printScale?: number;
  maxRmsPx?: number;
  minAreaPx?: number;
}

export function requiredTagEdgeMm(markerId: number): number;
export function distortNormalized(xn: number, yn: number, k1?: number): [number, number];
export function undistortPoint(u: number, v: number, K: CameraIntrinsics): PixelPoint;
export function projectCameraPoint(px: number, py: number, pz: number, K: CameraIntrinsics): PixelPoint | null;
export function referenceTools(
  g: RigDefinition,
  printScale?: number,
): Array<{ id: string; tags: Array<{ id: number; corners: number[][] }> }>;
export function fitPlanarPose(tags: PlanarTagInput[], K: CameraIntrinsics): PlanarPoseCV | null;
export function incrementPose(pose: PlanarPoseCV, step: ArrayLike<number>): PlanarPoseCV;
export function solveSingleTagPose(
  raw: RawMarker,
  K: CameraIntrinsics,
  options?: SingleTagPoseOptions,
): SingleTagPoseResult | null;
export function observeReference(
  markers: RawMarker[],
  geometry: RigDefinition,
  K: CameraIntrinsics,
  options?: ObserveReferenceOptions,
): ReferenceObservation;
