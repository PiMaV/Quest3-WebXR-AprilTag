import type {
  CalibrationModelName,
  CalibrationProfile,
  CameraSample,
  DeviceIdentity,
  ReferenceObservation,
  RigDefinition,
  XrHistoryEntry,
} from './common.js';
import type { ProfileStore } from './profile.js';

export type CalibrationPhase =
  | 'idle'
  | 'checking'
  | 'check-failed'
  | 'scanning'
  | 'solving'
  | 'verifying'
  | 'saving'
  | 'ready'
  | 'failed'
  | 'disposed';

export interface CalibrationAssistantState {
  phase: CalibrationPhase;
  message: string;
  count: number;
  required: number;
  profileId: string | null;
  persisted: boolean;
  verification: Record<string, unknown> | null;
  checkFailure: Record<string, unknown> | null;
  upgradeReason: string | null;
}

export interface CalibrationAssistantOptions {
  rig: RigDefinition;
  device: DeviceIdentity;
  imageSize: [number, number];
  store?: ProfileStore;
  onChange?: (state: CalibrationAssistantState) => void;
  onProfile?: (profile: CalibrationProfile | null) => void;
  workerFactory?: () => Worker;
  model?: CalibrationModelName;
  scanViews?: number;
  verificationViews?: number;
  minSolverVersion?: number | null;
}

export interface CalibrationAssistant {
  begin(opts?: { force?: boolean }): Promise<void>;
  observe(
    observation: ReferenceObservation,
    sample: CameraSample,
    history: XrHistoryEntry[],
    now: number,
  ): CalibrationAssistantState;
  state(): CalibrationAssistantState;
  phase(): CalibrationPhase;
  summary(): { phase: CalibrationPhase; count: number; required: number; message: string };
  diagnosticsData(): Record<string, unknown> | null;
  retryCheck(): boolean;
  retry(): void;
  profile(): CalibrationProfile | null;
  reset(): void;
  dispose(): void;
}

export function createCalibrationAssistant(options?: CalibrationAssistantOptions): CalibrationAssistant;
