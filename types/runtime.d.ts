import type {
  CameraIntrinsics,
  Matrix4Array,
  PlanarPoseCV,
  Quat,
  Vec3,
  XRFrameLike,
  XRSessionLike,
  XrContext,
  XrHistoryEntry,
} from './common.js';

export function identity(): Matrix4Array;
export function product(...mats: ArrayLike<number>[]): Matrix4Array;
export function det4(m: ArrayLike<number>): number;
export function inverse(m: ArrayLike<number>): Matrix4Array;
export function quatNormalize(q: ArrayLike<number>): Quat;
export function quatDot(a: ArrayLike<number>, b: ArrayLike<number>): number;
export function quatAngle(a: ArrayLike<number>, b: ArrayLike<number>): number;
export function quatSlerp(a: ArrayLike<number>, b: ArrayLike<number>, t: number): Quat;
export function matToQuat(m: ArrayLike<number>): Quat;
export function composeRigid(p: ArrayLike<number>, q: ArrayLike<number>): Matrix4Array;
export function cvToGL(pose: PlanarPoseCV): Matrix4Array;
export function rigidVector(a: ArrayLike<number>): [number, number, number, number, number, number];
export function vectorRigid(v: ArrayLike<number>): Matrix4Array;
export function meanRigid(poses: ArrayLike<number>[]): Matrix4Array;
export function poseAt(history: XrHistoryEntry[], time: number): Matrix4Array | null;
export function anchorFeatureState(
  session: XRSessionLike,
  frame?: XRFrameLike | null,
): { enabled: boolean | null; hasMethod: boolean; canAttempt: boolean };
export function poseToRigidParts(a: ArrayLike<number>): {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
} | null;
export function intrinsics(w: number, h: number, fov?: number): CameraIntrinsics;
export function observationIsFresh(
  s: { context?: XrContext | null; submittedAtPerformanceMs?: number; frameEvidence?: { fresh?: boolean } | null } | null | undefined,
  c: XrContext | null | undefined,
  now: number,
  maxAge?: number,
): boolean;
