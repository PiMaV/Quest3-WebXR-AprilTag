import type {
  AnchorAssociation,
  CalibrationProfile,
  CameraSample,
  Matrix4Array,
  PixelPoint,
  ReferenceObservation,
  RigDefinition,
  XrContext,
  XrHistoryEntry,
} from './common.js';

export interface AnchorAuditRow {
  serial: number;
  generation: number;
  context: XrContext;
  captureTime: number;
  targetXrTimeMs: number;
  positionDifferenceMm: number;
  orientationDifferenceDeg: number;
  nativePredictionRmsPx: number;
  opticalFitRmsPx: number;
  worldFromNativeL: Matrix4Array;
  worldFromOpticalL: Matrix4Array;
  projections: Array<{
    id: number;
    observed: PixelPoint[];
    predicted: PixelPoint[];
    rmsPx: number;
  }>;
  independentPhysicalAccuracy: false;
  usedToCorrectAnchor: false;
}

export interface AuditMetricStats {
  median: number;
  p95: number;
  max: number;
}

export interface AnchorAudit {
  reset(): void;
  observe(row: AnchorAuditRow | null | undefined): boolean;
  state(): {
    samples: number;
    positionDifferenceMm: AuditMetricStats | null;
    orientationDifferenceDeg: AuditMetricStats | null;
    nativePredictionRmsPx: AuditMetricStats | null;
    independentPhysicalAccuracy: false;
    usedToCorrectAnchor: false;
  };
}

export function associateAnchorCapture(
  history: XrHistoryEntry[],
  sample: CameraSample,
  profile: CalibrationProfile,
  now: number,
): AnchorAssociation | null;
export function auditAnchorObservation(
  observation: ReferenceObservation,
  sample: CameraSample,
  association: AnchorAssociation | null | undefined,
  rig: RigDefinition,
): AnchorAuditRow | null;
export function createAnchorAudit(options?: { maxSamples?: number }): AnchorAudit;
