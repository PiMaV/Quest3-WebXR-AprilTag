import type {
  AnchorCaptureCandidate,
  CalibrationProfile,
  CameraIntrinsics,
  CameraSample,
  CameraState,
  DeviceIdentity,
  ReferenceObservation,
  RigDefinition,
  XrHistoryEntry,
} from './common.js';

export const PROFILE_SCHEMA: 'webxr-apriltag-profile-v1';
export const PIPELINE: 'rgba-gray-arena36h11-planar-capture-xrraf-v1';

export interface ProfileStore {
  load(device: DeviceIdentity): Promise<CalibrationProfile | null>;
  save(profile: CalibrationProfile): Promise<CalibrationProfile>;
}

export interface ObservationBufferEntry {
  observation: ReferenceObservation;
  sample: CameraSample;
}

export interface ObservationBuffer {
  push(entry: ObservationBufferEntry): void;
  drain(
    history: XrHistoryEntry[],
    profile: CalibrationProfile | null | undefined,
    now: number,
  ): Array<ObservationBufferEntry & { candidate: AnchorCaptureCandidate }>;
  clear(): void;
  size(): number;
}

export function digest(value: unknown): Promise<string>;
export function deviceIdentity(
  camera: Pick<CameraState, 'sourceSize' | 'trackLabel' | 'trackSettings'>,
  rig: RigDefinition,
  userAgent?: string,
): Promise<DeviceIdentity>;
export function validateCalibration(profile: unknown): CalibrationProfile;
export function extractCalibrationProfile(payload: unknown): CalibrationProfile;
export function assertProfileDevice(profile: unknown, device: DeviceIdentity): CalibrationProfile;
export function importCalibrationProfile(
  payload: unknown,
  device: DeviceIdentity,
  store?: ProfileStore,
): Promise<CalibrationProfile>;
export function calibratedCameraModel(
  profile: CalibrationProfile,
  imageSize: [number, number],
  sourceSize: [number, number],
  label: string | null | undefined,
  userAgent: string,
  settings?: Partial<MediaTrackSettings>,
): CameraIntrinsics;
export function profileCameraConstraints(
  profile: CalibrationProfile,
  devices?: MediaDeviceInfo[],
  supported?: MediaTrackSupportedConstraints,
): MediaStreamConstraints;
export function viewerPoseHorizon(
  history: XrHistoryEntry[],
  sample: CameraSample,
  profile: CalibrationProfile,
): {
  status: 'invalid' | 'future' | 'ready';
  targetXrTimeMs: number | null;
  pose: AnchorCaptureCandidate | null;
};
export function calibratedViewerPose(
  history: XrHistoryEntry[],
  sample: CameraSample,
  profile: CalibrationProfile,
): AnchorCaptureCandidate | null;
export function createObservationBuffer(options?: { maxAgeMs?: number; maxItems?: number }): ObservationBuffer;
export function createProfileStore(storage?: Pick<Storage, 'getItem' | 'setItem'>): ProfileStore;
