import type { CameraIntrinsics, Matrix4Array, RawMarker } from './common.js';

export type RoiBox = [number, number, number, number];

export interface PriorRoiPointItem {
  pos: [number, number, number];
  minDistM?: number;
  maxDistM?: number;
  halfPx?: number;
}

export interface ProjectPriorRoisOptions {
  pointsL: Array<[number, number, number] | PriorRoiPointItem>;
  worldFromAnchor: Matrix4Array;
  worldFromViewer: Matrix4Array;
  viewerFromCameraGL: Matrix4Array;
  K: CameraIntrinsics;
  width: number;
  height: number;
  minDistM?: number;
  maxDistM?: number;
  defaultHalfPx?: number;
  maxRois?: number;
}

export interface FocusCandidate {
  pos?: [number, number, number];
  recording?: boolean;
  handCoupled?: boolean;
  moving?: boolean;
  settling?: boolean;
  distToCameraM?: number;
  apparentEdgePx?: number;
}

export interface AprilTagDetectionRun {
  markers: RawMarker[];
  coarseMs: number;
  roiMs: number;
  roiCount: number;
  trackedHit?: boolean;
  focusHit?: boolean;
  focusHitBoxes?: number;
  focusMissedBoxes?: number;
}

export interface AprilTagDetector {
  setDecimate(next: 1 | 1.5 | 2 | 3 | 4): void;
  setOptions(opts?: { decimate?: 1 | 1.5 | 2 | 3 | 4; refineEdges?: boolean | number }): void;
  detect(gray: Uint8Array, width: number, height: number): RawMarker[];
  detectTwoStage(
    gray: Uint8Array,
    width: number,
    height: number,
    opts?: {
      coarseDecimate?: 1 | 1.5 | 2 | 3 | 4;
      padFrac?: number;
      minPadPx?: number;
      maxRois?: number;
      refine?: boolean;
      priorRois?: RoiBox[] | null;
    },
  ): AprilTagDetectionRun;
  detectTrackedRoi(
    gray: Uint8Array,
    width: number,
    height: number,
    hintBox: RoiBox,
    opts?: { requiredIds?: number[]; fallbackOpts?: Record<string, unknown> },
  ): AprilTagDetectionRun;
  detectFocusRois(
    gray: Uint8Array,
    width: number,
    height: number,
    focusRois: RoiBox[],
    opts?: {
      maxRois?: number;
      maxTotalAreaFrac?: number;
      coarseAreaThreshold?: number;
      speedFirst?: boolean;
      speedSmallAreaThreshold?: number;
      allowFullFallback?: boolean;
      fallbackOpts?: Record<string, unknown>;
    },
  ): AprilTagDetectionRun;
}

export function projectPriorRois(options?: ProjectPriorRoisOptions): RoiBox[];
export function prioritizeFocusCandidates<T extends FocusCandidate>(
  candidates: T[],
  options?: { maxCount?: number; siblingRadiusM?: number },
): T[];
export function createAprilTagDetector(wasm: unknown, decimate?: 1 | 1.5 | 2 | 3 | 4): AprilTagDetector;
