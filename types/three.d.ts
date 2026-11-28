import type { Matrix4Array, Vec3 } from './common.js';

export interface ThreeObjectLike {
  visible: boolean;
  renderOrder: number;
  frustumCulled: boolean;
  name: string;
}

export interface InstancedPool {
  mesh: ThreeObjectLike & { count: number; dispose(): void };
  capacity: number;
  alloc(): number;
  free(slot: number): void;
  setMatrix(slot: number, mat16: ArrayLike<number>): void;
  setRod(slot: number, ax: number, ay: number, az: number, bx: number, by: number, bz: number, radius?: number): void;
  setPoint(slot: number, x: number, y: number, z: number, scale?: number): void;
  setColor(slot: number, hexOrColor: number | { r: number; g: number; b: number }): void;
  setAlert(slot: number, active: boolean): void;
  setVisible(slot: number, flag: boolean): void;
  clear(): void;
  commit(): number;
  activeCount(): number;
  alertCount(): number;
  dispose(): void;
}

export interface AxesBatch {
  lines: ThreeObjectLike & { geometry: { dispose(): void } };
  capacity: number;
  alloc(): number;
  free(slot: number): void;
  setPose(slot: number, mat16: ArrayLike<number>, axisLen?: number): void;
  setVisible(slot: number, flag: boolean): void;
  clear(): void;
  commit(): number;
  activeCount(): number;
  dispose(): void;
}

export interface PathTubeMesh {
  mesh: ThreeObjectLike & { geometry: { dispose(): void } };
  capacity: number;
  sides: number;
  syncFromPositions(
    rawPositions: ArrayLike<number>,
    count: number,
    opts?: {
      radii?: ArrayLike<number> | null;
      colors?: ArrayLike<number> | null;
      outsideFlags?: ArrayLike<number> | null;
      fades?: ArrayLike<number> | null;
      breakFlags?: ArrayLike<number> | null;
      fromIndex?: number;
      sortByFade?: boolean;
    },
  ): number;
  syncColors(
    sampleColors: ArrayLike<number>,
    count: number,
    opts?: { fromIndex?: number },
  ): number;
  syncOutsideFlags(
    outsideFlags: ArrayLike<number>,
    count: number,
    opts?: { fromIndex?: number },
  ): number;
  clear(): void;
  activeSegments(): number;
  dispose(): void;
}

export interface PathRibbonMesh {
  mesh: ThreeObjectLike & { geometry: { dispose(): void } };
  edgeLines: ThreeObjectLike & { geometry: { dispose(): void } };
  capacity: number;
  syncFromPositions(
    rawPositions: ArrayLike<number>,
    rawRightAxes: ArrayLike<number>,
    count: number,
    halfWidths?: ArrayLike<number> | null,
    colorsRgb?: ArrayLike<number> | null,
  ): number;
  clear(): void;
  activeSegments(): number;
  dispose(): void;
}

export interface BadgeAtlasUvRect {
  px: number;
  py: number;
  u0: number;
  u1: number;
  vTop: number;
  vBot: number;
}

export interface BadgeAtlas {
  width: number;
  height: number;
  cellW: number;
  cellH: number;
  cols: number;
  rows: number;
  capacity: number;
  canvas: HTMLCanvasElement;
  texture: { needsUpdate: boolean; dispose(): void };
  acquire(): number;
  release(slot: number): void;
  draw(slot: number, key: string, paintFn: (ctx: CanvasRenderingContext2D, w: number, h: number) => void): boolean;
  invalidate(slot: number): void;
  uvRect(slot: number): BadgeAtlasUvRect | null;
  flush(): boolean;
  clear(): void;
  dispose(): void;
}

export interface BillboardAtlasMesh {
  mesh: ThreeObjectLike & { geometry: { dispose(): void }; material: { dispose(): void } };
  capacity: number;
  setQuad(slot: number, posL: Vec3, widthM: number, heightM: number, atlasSlot: number): void;
  setQuadVisible(slot: number, flag: boolean): void;
  clearSlot(slot: number): void;
  clear(): void;
  updateBillboards(viewerRightL: Vec3, viewerUpL: Vec3): number;
  activeCount(): number;
  dispose(): void;
}

export interface OrientedAtlasMesh {
  mesh: ThreeObjectLike & { geometry: { dispose(): void }; material: { dispose(): void } };
  capacity: number;
  setOrientedQuad(
    slot: number,
    mat16: Matrix4Array,
    widthM: number,
    heightM: number,
    atlasSlot: number,
    zOffsetM?: number,
  ): void;
  setQuadVisible(slot: number, flag: boolean): void;
  clearSlot(slot: number): void;
  clear(): void;
  commit(): number;
  activeCount(): number;
  dispose(): void;
}

export function createInstancedPool(
  geometry: unknown,
  material: unknown,
  capacity: number,
  opts?: { renderOrder?: number; name?: string; alertAttribute?: boolean },
): InstancedPool;
export function createAxesBatch(maxAxes: number, opts?: { renderOrder?: number }): AxesBatch;
export function createPathTubeMesh(
  maxPoints: number,
  material: unknown,
  opts?: {
    radius?: number;
    sides?: number;
    vertexColors?: boolean;
    dashAttribute?: boolean;
    fadeAttribute?: boolean;
    renderOrder?: number;
    name?: string;
  },
): PathTubeMesh;
export function createPathRibbonMesh(
  maxPoints: number,
  material: unknown,
  opts?: {
    edgeMaterial?: unknown;
    halfWidth?: number;
    rightEdgeTint?: number;
    renderOrder?: number;
    name?: string;
  },
): PathRibbonMesh;
export function createBadgeAtlas(opts?: {
  width?: number;
  height?: number;
  cellW?: number;
  cellH?: number;
  createCanvas?: (width: number, height: number) => unknown;
  createTexture?: (canvas: unknown) => unknown;
}): BadgeAtlas;
export function createBillboardAtlasMesh(
  atlas: BadgeAtlas,
  maxQuads?: number,
  opts?: { renderOrder?: number; depthTest?: boolean },
): BillboardAtlasMesh;
export function createOrientedAtlasMesh(
  atlas: BadgeAtlas,
  maxQuads?: number,
  opts?: { renderOrder?: number; depthTest?: boolean; depthWrite?: boolean },
): OrientedAtlasMesh;
