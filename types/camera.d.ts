import type {
  CameraSample,
  CameraSettings,
  CameraState,
  ReferenceObservation,
  RigDefinition,
  XrContext,
} from './common.js';

export interface DetectorWorkerContext {
  workerUrl: URL;
  wasmBaseUrl: string | null;
}

export interface ReferenceCameraOptions {
  video: HTMLVideoElement;
  geometry: RigDefinition;
  getSettings: () => CameraSettings;
  getContext: () => XrContext;
  onObservation?: (observation: ReferenceObservation, sample: CameraSample, canvas: HTMLCanvasElement) => void;
  onState?: (state: CameraState) => void;
  onError?: (error: Error) => void;
  workerFactory?: ((ctx: DetectorWorkerContext) => Worker) | null;
  wasmBaseUrl?: string | null;
}

export interface ReferenceCamera {
  prepare(): Promise<void>;
  start(): Promise<boolean>;
  stop(): void;
  tick(now: number): void;
  state(): CameraState;
  isRunning(): boolean;
  presentedCallbacks(): number;
  canvas: HTMLCanvasElement;
  setGeometry(next: RigDefinition): void;
  dispose(): void;
}

export function createReferenceCamera(options: ReferenceCameraOptions): ReferenceCamera;
