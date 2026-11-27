import type { Matrix4Array, RigDefinition, Vec3 } from './common.js';

export const FENCE_HEIGHT_M: number;
export const PERIMETER_TRIGGER_TAG_ID: number;
export const PERIMETER_MARKERS: ReadonlyArray<Readonly<{ id: number; role: string }>>;

export interface PerimeterVertex {
  id: number;
  role: string;
  fixed: boolean;
  pos: Vec3;
  yaw?: number;
  matrix?: Matrix4Array;
  lastAt: number;
  updates: number;
}

export interface PerimeterFence {
  heightM: number;
  triggerTagId: number;
  isActive(): boolean;
  isOffered(): boolean;
  status(): 'idle' | 'offered' | 'active';
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
    opts?: { weight?: number },
  ): {
    accepted: boolean;
    isNew: boolean;
    justOffered?: boolean;
    status?: 'idle' | 'offered' | 'active';
    id: number;
    role: string;
    pos: Vec3;
    reason?: string;
  } | null;
  step(dtSec: number): boolean;
  orderedVertices(): PerimeterVertex[];
  discoveredMarkers(): PerimeterVertex[];
  dynamicCount(): number;
  version(): number;
  snapshot(): {
    status: 'idle' | 'offered' | 'active';
    active: boolean;
    offered: boolean;
    triggerTagId: number;
    heightM: number;
    dynamicCount: number;
    totalDynamicMarkers: number;
    perimeterM: number;
    areaM2: number;
    vertices: Array<{ id: number; role: string; fixed: boolean; posL: Vec3; updates: number }>;
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
export function polygonMetricsXY(vertices: Array<{ pos: Vec3 }>): { perimeterM: number; areaM2: number };
export function createPerimeterFence(
  rig: RigDefinition | null | undefined,
  opts?: {
    heightM?: number;
    triggerTagId?: number;
    smoothTauSec?: number;
    jumpDistanceM?: number;
    jumpWindowMs?: number;
    jumpConfirmM?: number;
  },
): PerimeterFence;
