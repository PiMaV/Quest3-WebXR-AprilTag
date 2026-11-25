// Shared CanvasTexture badge atlas & single-draw-call billboard quad batch.
import * as THREE from '../vendor/three/three.module.js';

function defaultCreateCanvas(width, height) {
  const c = document.createElement('canvas');
  c.width = width;
  c.height = height;
  return c;
}

function defaultCreateTexture(canvas) {
  const tex = new THREE.CanvasTexture(canvas);
  tex.generateMipmaps = false;
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  return tex;
}

/**
 * Pack up to (width/cellW) * (height/cellH) small 2D badges into one CanvasTexture
 * with key-cached slot redraws and 1-px inset UVs to avoid neighbor bleed.
 */
export function createBadgeAtlas({
  width = 1024,
  height = 1920,
  cellW = 256,
  cellH = 96,
  createCanvas = defaultCreateCanvas,
  createTexture = defaultCreateTexture,
} = {}) {
  const cols = Math.max(1, Math.floor(width / cellW));
  const rows = Math.max(1, Math.floor(height / cellH));
  const cap = cols * rows;

  const canvas = createCanvas(width, height);
  const texture = createTexture(canvas);

  const allocated = new Uint8Array(cap);
  const lastKeys = new Array(cap).fill(null);
  const freeStack = new Int32Array(cap);
  let freeTop = cap;
  for (let i = 0; i < cap; i++) freeStack[i] = cap - 1 - i;

  const uvs = new Array(cap);
  for (let s = 0; s < cap; s++) {
    const px = (s % cols) * cellW;
    const py = Math.floor(s / cols) * cellH;
    uvs[s] = {
      px,
      py,
      u0: (px + 1) / width,
      u1: (px + cellW - 1) / width,
      // WebGL Texture.flipY defaults true: canvas y=0 maps to v=1.
      vTop: 1 - (py + 1) / height,
      vBot: 1 - (py + cellH - 1) / height,
    };
  }

  let atlasDirty = false;

  function acquire() {
    if (freeTop <= 0) return -1;
    const slot = freeStack[--freeTop];
    allocated[slot] = 1;
    lastKeys[slot] = null;
    return slot;
  }

  function release(slot) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    allocated[slot] = 0;
    lastKeys[slot] = null;
    freeStack[freeTop++] = slot;
  }

  function draw(slot, cacheKey, drawFn) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return false;
    if (cacheKey != null && lastKeys[slot] === cacheKey) return false;
    lastKeys[slot] = cacheKey;
    const { px, py } = uvs[slot];
    const ctx = canvas.getContext('2d');
    ctx.save();
    ctx.beginPath();
    ctx.rect(px, py, cellW, cellH);
    ctx.clip();
    ctx.clearRect(px, py, cellW, cellH);
    ctx.translate(px, py);
    drawFn(ctx, cellW, cellH);
    ctx.restore();
    atlasDirty = true;
    return true;
  }

  function invalidate(slot) {
    if (slot >= 0 && slot < cap) lastKeys[slot] = null;
  }

  function uvRect(slot) {
    return (slot >= 0 && slot < cap) ? uvs[slot] : null;
  }

  function flush() {
    if (!atlasDirty) return false;
    texture.needsUpdate = true;
    atlasDirty = false;
    return true;
  }

  function clear() {
    allocated.fill(0);
    lastKeys.fill(null);
    freeTop = cap;
    for (let i = 0; i < cap; i++) freeStack[i] = cap - 1 - i;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, width, height);
    atlasDirty = true;
  }

  function dispose() {
    texture?.dispose?.();
  }

  return {
    width,
    height,
    cellW,
    cellH,
    cols,
    rows,
    capacity: cap,
    canvas,
    texture,
    acquire,
    release,
    draw,
    invalidate,
    uvRect,
    flush,
    clear,
    dispose,
  };
}

/**
 * Render all active atlas badges in a single dynamic indexed BufferGeometry draw call,
 * aligned to the viewer's right/up basis in the parent frame (e.g., L-frame).
 */
export function createBillboardAtlasMesh(atlas, maxQuads = atlas.capacity, { renderOrder = 12, depthTest = false } = {}) {
  const cap = Math.max(1, Math.min(maxQuads | 0, atlas.capacity));
  const positions = new Float32Array(cap * 4 * 3);
  const uvs = new Float32Array(cap * 4 * 2);
  const indices = new Uint16Array(cap * 6);
  for (let i = 0; i < cap; i++) {
    const v = i * 4;
    const idx = i * 6;
    indices[idx] = v;
    indices[idx + 1] = v + 1;
    indices[idx + 2] = v + 2;
    indices[idx + 3] = v;
    indices[idx + 4] = v + 2;
    indices[idx + 5] = v + 3;
  }

  const geometry = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(positions, 3);
  posAttr.setUsage(THREE.DynamicDrawUsage);
  const uvAttr = new THREE.BufferAttribute(uvs, 2);
  uvAttr.setUsage(THREE.DynamicDrawUsage);
  geometry.setAttribute('position', posAttr);
  geometry.setAttribute('uv', uvAttr);
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  geometry.setDrawRange(0, 0);

  const material = new THREE.MeshBasicMaterial({
    map: atlas.texture,
    transparent: true,
    depthTest,
    depthWrite: false,
    side: THREE.DoubleSide,
  });

  const mesh = new THREE.Mesh(geometry, material);
  mesh.frustumCulled = false;
  mesh.renderOrder = renderOrder;

  const slotPos = new Float32Array(cap * 3);
  const slotHalfSize = new Float32Array(cap * 2);
  const slotHasQuad = new Uint8Array(cap);
  const slotVisible = new Uint8Array(cap);

  let dirty = false;
  let lastCount = 0;
  let prevRx = 1, prevRy = 0, prevRz = 0, prevUx = 0, prevUy = 0, prevUz = 1;

  function setQuad(slot, posL, widthM, heightM, visible = true) {
    if (slot < 0 || slot >= cap || !posL) return;
    const pOff = slot * 3;
    const sOff = slot * 2;
    const hw = widthM * 0.5;
    const hh = heightM * 0.5;
    const v = visible ? 1 : 0;
    if (
      slotHasQuad[slot] &&
      slotVisible[slot] === v &&
      slotPos[pOff] === posL[0] &&
      slotPos[pOff + 1] === posL[1] &&
      slotPos[pOff + 2] === posL[2] &&
      slotHalfSize[sOff] === hw &&
      slotHalfSize[sOff + 1] === hh
    ) {
      return;
    }
    slotPos[pOff] = posL[0];
    slotPos[pOff + 1] = posL[1];
    slotPos[pOff + 2] = posL[2];
    slotHalfSize[sOff] = hw;
    slotHalfSize[sOff + 1] = hh;
    slotHasQuad[slot] = 1;
    slotVisible[slot] = v;
    dirty = true;
  }

  function setQuadVisible(slot, visible) {
    if (slot < 0 || slot >= cap) return;
    const v = visible ? 1 : 0;
    if (slotVisible[slot] === v) return;
    slotVisible[slot] = v;
    dirty = true;
  }

  function clearSlot(slot) {
    if (slot < 0 || slot >= cap) return;
    if (!slotHasQuad[slot] && !slotVisible[slot]) return;
    slotHasQuad[slot] = 0;
    slotVisible[slot] = 0;
    dirty = true;
  }

  function clear() {
    slotHasQuad.fill(0);
    slotVisible.fill(0);
    geometry.setDrawRange(0, 0);
    lastCount = 0;
    dirty = false;
  }

  function updateBillboards(rightL, upL) {
    atlas.flush();
    let rx = rightL?.[0] ?? 1, ry = rightL?.[1] ?? 0, rz = rightL?.[2] ?? 0;
    let ux = upL?.[0] ?? 0, uy = upL?.[1] ?? 0, uz = upL?.[2] ?? 1;
    const rLen = Math.hypot(rx, ry, rz);
    if (rLen > 1e-6) { rx /= rLen; ry /= rLen; rz /= rLen; }
    else { rx = 1; ry = 0; rz = 0; }
    const uLen = Math.hypot(ux, uy, uz);
    if (uLen > 1e-6) { ux /= uLen; uy /= uLen; uz /= uLen; }
    else { ux = 0; uy = 0; uz = 1; }

    const basisChanged =
      Math.abs(rx - prevRx) > 1e-5 || Math.abs(ry - prevRy) > 1e-5 || Math.abs(rz - prevRz) > 1e-5 ||
      Math.abs(ux - prevUx) > 1e-5 || Math.abs(uy - prevUy) > 1e-5 || Math.abs(uz - prevUz) > 1e-5;

    if (!dirty && !(basisChanged && lastCount > 0)) return lastCount;

    prevRx = rx; prevRy = ry; prevRz = rz;
    prevUx = ux; prevUy = uy; prevUz = uz;

    let count = 0;
    for (let s = 0; s < cap; s++) {
      if (!slotHasQuad[s] || !slotVisible[s]) continue;
      const uv = atlas.uvRect(s);
      if (!uv) continue;
      const pOff = s * 3;
      const sOff = s * 2;
      const cx = slotPos[pOff], cy = slotPos[pOff + 1], cz = slotPos[pOff + 2];
      const hw = slotHalfSize[sOff], hh = slotHalfSize[sOff + 1];

      const dxR = rx * hw, dyR = ry * hw, dzR = rz * hw;
      const dxU = ux * hh, dyU = uy * hh, dzU = uz * hh;

      const vOff = count * 12;
      // v0: bottom-left
      positions[vOff] = cx - dxR - dxU;
      positions[vOff + 1] = cy - dyR - dyU;
      positions[vOff + 2] = cz - dzR - dzU;
      // v1: bottom-right
      positions[vOff + 3] = cx + dxR - dxU;
      positions[vOff + 4] = cy + dyR - dyU;
      positions[vOff + 5] = cz + dzR - dzU;
      // v2: top-right
      positions[vOff + 6] = cx + dxR + dxU;
      positions[vOff + 7] = cy + dyR + dyU;
      positions[vOff + 8] = cz + dzR + dzU;
      // v3: top-left
      positions[vOff + 9] = cx - dxR + dxU;
      positions[vOff + 10] = cy - dyR + dyU;
      positions[vOff + 11] = cz - dzR + dzU;

      const uOff = count * 8;
      uvs[uOff] = uv.u0; uvs[uOff + 1] = uv.vBot;
      uvs[uOff + 2] = uv.u1; uvs[uOff + 3] = uv.vBot;
      uvs[uOff + 4] = uv.u1; uvs[uOff + 5] = uv.vTop;
      uvs[uOff + 6] = uv.u0; uvs[uOff + 7] = uv.vTop;

      count++;
    }

    geometry.setDrawRange(0, count * 6);
    posAttr.needsUpdate = true;
    uvAttr.needsUpdate = true;
    lastCount = count;
    dirty = false;
    return count;
  }

  function dispose() {
    geometry.dispose();
    material.dispose();
  }

  return {
    mesh,
    capacity: cap,
    setQuad,
    setQuadVisible,
    clearSlot,
    clear,
    updateBillboards,
    activeCount: () => lastCount,
    dispose,
  };
}
