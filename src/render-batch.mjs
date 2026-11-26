// Pre-allocated InstancedMesh & LineSegments batch pools for low-draw-call WebXR rendering.
import * as THREE from '../vendor/three/three.module.js';

const _upVec = new THREE.Vector3(0, 1, 0);
const _vA = new THREE.Vector3();
const _vB = new THREE.Vector3();
const _vDir = new THREE.Vector3();
const _mid = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _scale = new THREE.Vector3();
const _mat4 = new THREE.Matrix4();
const _tmpColor = new THREE.Color();

/**
 * Manage a fixed-capacity THREE.InstancedMesh with O(1) slot allocation,
 * per-instance color pre-initialized at construction, and tight dirty-only packing.
 */
export function createInstancedPool(geometry, material, capacity, { renderOrder = 0, name = '' } = {}) {
  const cap = Math.max(1, capacity | 0);
  const mesh = new THREE.InstancedMesh(geometry, material, cap);
  mesh.frustumCulled = false;
  mesh.count = 0;
  mesh.renderOrder = renderOrder;
  if (name) mesh.name = name;
  // Ensure instanceColor buffer exists before initial shader compilation.
  mesh.setColorAt(0, _tmpColor.setHex(0xffffff));

  const allocated = new Uint8Array(cap);
  const visible = new Uint8Array(cap);
  const valid = new Uint8Array(cap);
  const matrices = new Float32Array(cap * 16);
  const colors = new Float32Array(cap * 3);
  colors.fill(1);

  const freeStack = new Int32Array(cap);
  let freeTop = cap;
  for (let i = 0; i < cap; i++) {
    freeStack[i] = cap - 1 - i;
  }

  let dirty = false;
  let lastCount = 0;

  function alloc() {
    if (freeTop <= 0) return -1;
    const slot = freeStack[--freeTop];
    allocated[slot] = 1;
    visible[slot] = 1;
    valid[slot] = 0;
    const cOff = slot * 3;
    colors[cOff] = 1;
    colors[cOff + 1] = 1;
    colors[cOff + 2] = 1;
    dirty = true;
    return slot;
  }

  function free(slot) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    allocated[slot] = 0;
    visible[slot] = 0;
    valid[slot] = 0;
    freeStack[freeTop++] = slot;
    dirty = true;
  }

  function setMatrix(slot, mat4OrArray16) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    const src = mat4OrArray16?.elements ?? mat4OrArray16;
    if (!src || src.length < 16) return;
    const off = slot * 16;
    for (let i = 0; i < 16; i++) matrices[off + i] = src[i];
    valid[slot] = 1;
    dirty = true;
  }

  function setRod(slot, ax, ay, az, bx, by, bz, radius = 0.0018) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    _vA.set(ax, ay, az);
    _vB.set(bx, by, bz);
    _vDir.subVectors(_vB, _vA);
    const len = _vDir.length();
    if (!(len >= 1e-4)) {
      if (valid[slot] !== 0) {
        valid[slot] = 0;
        dirty = true;
      }
      return;
    }
    _mid.addVectors(_vA, _vB).multiplyScalar(0.5);
    _quat.setFromUnitVectors(_upVec, _vDir.divideScalar(len));
    _scale.set(radius, len, radius);
    _mat4.compose(_mid, _quat, _scale);
    const off = slot * 16;
    const el = _mat4.elements;
    for (let i = 0; i < 16; i++) matrices[off + i] = el[i];
    valid[slot] = 1;
    dirty = true;
  }

  function setPoint(slot, x, y, z, scale = 1) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    const off = slot * 16;
    matrices[off] = scale; matrices[off + 1] = 0; matrices[off + 2] = 0; matrices[off + 3] = 0;
    matrices[off + 4] = 0; matrices[off + 5] = scale; matrices[off + 6] = 0; matrices[off + 7] = 0;
    matrices[off + 8] = 0; matrices[off + 9] = 0; matrices[off + 10] = scale; matrices[off + 11] = 0;
    matrices[off + 12] = x; matrices[off + 13] = y; matrices[off + 14] = z; matrices[off + 15] = 1;
    valid[slot] = 1;
    dirty = true;
  }

  function setColor(slot, hex) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    _tmpColor.setHex(hex);
    const off = slot * 3;
    if (colors[off] === _tmpColor.r && colors[off + 1] === _tmpColor.g && colors[off + 2] === _tmpColor.b) return;
    colors[off] = _tmpColor.r;
    colors[off + 1] = _tmpColor.g;
    colors[off + 2] = _tmpColor.b;
    dirty = true;
  }

  function setVisible(slot, vis) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    const v = vis ? 1 : 0;
    if (visible[slot] === v) return;
    visible[slot] = v;
    dirty = true;
  }

  function clear() {
    allocated.fill(0);
    visible.fill(0);
    valid.fill(0);
    freeTop = cap;
    for (let i = 0; i < cap; i++) freeStack[i] = cap - 1 - i;
    mesh.count = 0;
    lastCount = 0;
    dirty = false;
  }

  function commit() {
    if (!dirty) return lastCount;
    const dstM = mesh.instanceMatrix.array;
    const dstC = mesh.instanceColor?.array;
    let count = 0;
    for (let s = 0; s < cap; s++) {
      if (!allocated[s] || !visible[s] || !valid[s]) continue;
      dstM.set(matrices.subarray(s * 16, s * 16 + 16), count * 16);
      if (dstC) {
        const cSrc = s * 3;
        const cDst = count * 3;
        dstC[cDst] = colors[cSrc];
        dstC[cDst + 1] = colors[cSrc + 1];
        dstC[cDst + 2] = colors[cSrc + 2];
      }
      count++;
    }
    mesh.count = count;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    lastCount = count;
    dirty = false;
    return count;
  }

  function dispose() {
    mesh.dispose();
  }

  return {
    mesh,
    capacity: cap,
    alloc,
    free,
    setMatrix,
    setRod,
    setPoint,
    setColor,
    setVisible,
    clear,
    commit,
    activeCount: () => lastCount,
    dispose,
  };
}

/**
 * Single LineSegments batch for all tag 3D coordinate axes (X red, Y green, Z cyan),
 * eliminating per-tag AxesHelper geometry/material allocations and GPU leaks.
 */
export function createAxesBatch(maxAxes, { renderOrder = 0 } = {}) {
  const cap = Math.max(1, maxAxes | 0);
  const floatsPerAxis = 18; // 6 vertices * 3 floats
  const posArray = new Float32Array(cap * floatsPerAxis);
  const colArray = new Float32Array(cap * floatsPerAxis);

  // Pre-fill RGB vertex colors once since packed layout repeats X,X,Y,Y,Z,Z per axis.
  for (let i = 0; i < cap; i++) {
    const o = i * floatsPerAxis;
    // X axis (red)
    colArray[o] = 0.94; colArray[o + 1] = 0.27; colArray[o + 2] = 0.27;
    colArray[o + 3] = 0.94; colArray[o + 4] = 0.27; colArray[o + 5] = 0.27;
    // Y axis (green)
    colArray[o + 6] = 0.20; colArray[o + 7] = 0.88; colArray[o + 8] = 0.42;
    colArray[o + 9] = 0.20; colArray[o + 10] = 0.88; colArray[o + 11] = 0.42;
    // Z axis (cyan/blue)
    colArray[o + 12] = 0.22; colArray[o + 13] = 0.74; colArray[o + 14] = 0.97;
    colArray[o + 15] = 0.22; colArray[o + 16] = 0.74; colArray[o + 17] = 0.97;
  }

  const geometry = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(posArray, 3);
  posAttr.setUsage(THREE.DynamicDrawUsage);
  geometry.setAttribute('position', posAttr);
  geometry.setAttribute('color', new THREE.BufferAttribute(colArray, 3));
  geometry.setDrawRange(0, 0);

  const material = new THREE.LineBasicMaterial({ vertexColors: true });
  const lines = new THREE.LineSegments(geometry, material);
  lines.frustumCulled = false;
  lines.renderOrder = renderOrder;

  const allocated = new Uint8Array(cap);
  const visible = new Uint8Array(cap);
  const valid = new Uint8Array(cap);
  const slotPos = new Float32Array(cap * floatsPerAxis);
  const freeStack = new Int32Array(cap);
  let freeTop = cap;
  for (let i = 0; i < cap; i++) freeStack[i] = cap - 1 - i;

  let dirty = false;
  let lastCount = 0;

  function alloc() {
    if (freeTop <= 0) return -1;
    const slot = freeStack[--freeTop];
    allocated[slot] = 1;
    visible[slot] = 1;
    valid[slot] = 0;
    dirty = true;
    return slot;
  }

  function free(slot) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    allocated[slot] = 0;
    visible[slot] = 0;
    valid[slot] = 0;
    freeStack[freeTop++] = slot;
    dirty = true;
  }

  function setPose(slot, mat4OrArray16, axisLen = 0.025) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    const m = mat4OrArray16?.elements ?? mat4OrArray16;
    if (!m || m.length < 16) return;
    const ox = m[12], oy = m[13], oz = m[14];
    const o = slot * floatsPerAxis;
    // X
    slotPos[o] = ox; slotPos[o + 1] = oy; slotPos[o + 2] = oz;
    slotPos[o + 3] = ox + m[0] * axisLen; slotPos[o + 4] = oy + m[1] * axisLen; slotPos[o + 5] = oz + m[2] * axisLen;
    // Y
    slotPos[o + 6] = ox; slotPos[o + 7] = oy; slotPos[o + 8] = oz;
    slotPos[o + 9] = ox + m[4] * axisLen; slotPos[o + 10] = oy + m[5] * axisLen; slotPos[o + 11] = oz + m[6] * axisLen;
    // Z
    slotPos[o + 12] = ox; slotPos[o + 13] = oy; slotPos[o + 14] = oz;
    slotPos[o + 15] = ox + m[8] * axisLen; slotPos[o + 16] = oy + m[9] * axisLen; slotPos[o + 17] = oz + m[10] * axisLen;
    valid[slot] = 1;
    dirty = true;
  }

  function setVisible(slot, vis) {
    if (slot < 0 || slot >= cap || !allocated[slot]) return;
    const v = vis ? 1 : 0;
    if (visible[slot] === v) return;
    visible[slot] = v;
    dirty = true;
  }

  function clear() {
    allocated.fill(0);
    visible.fill(0);
    valid.fill(0);
    freeTop = cap;
    for (let i = 0; i < cap; i++) freeStack[i] = cap - 1 - i;
    geometry.setDrawRange(0, 0);
    lastCount = 0;
    dirty = false;
  }

  function commit() {
    if (!dirty) return lastCount;
    let count = 0;
    for (let s = 0; s < cap; s++) {
      if (!allocated[s] || !visible[s] || !valid[s]) continue;
      posArray.set(slotPos.subarray(s * floatsPerAxis, s * floatsPerAxis + floatsPerAxis), count * floatsPerAxis);
      count++;
    }
    geometry.setDrawRange(0, count * 6);
    posAttr.needsUpdate = true;
    lastCount = count;
    dirty = false;
    return count;
  }

  function dispose() {
    geometry.dispose();
    material.dispose();
  }

  return {
    lines,
    capacity: cap,
    alloc,
    free,
    setPose,
    setVisible,
    clear,
    commit,
    activeCount: () => lastCount,
    dispose,
  };
}

/**
 * Single-draw-call pre-allocated volumetric 3D polyline tube with parallel-transport
 * frames and per-sample radii (e.g., velocity-modulated path thickness).
 */
export function createPathTubeMesh(maxPoints, material, {
  radius = 0.0014,
  sides = 5,
  renderOrder = 0,
  name = '',
} = {}) {
  const cap = Math.max(2, maxPoints | 0);
  const segSides = Math.max(3, Math.min(12, sides | 0));
  const defaultRadius = Number.isFinite(radius) && radius > 1e-5 ? radius : 0.0014;

  const positions = new Float32Array(cap * segSides * 3);
  const ringNormals = new Float32Array(cap * 3);
  const totalIndices = (cap - 1) * segSides * 6;
  const indices = (cap * segSides <= 65535)
    ? new Uint16Array(totalIndices)
    : new Uint32Array(totalIndices);

  let idxOff = 0;
  for (let i = 0; i < cap - 1; i++) {
    const r0 = i * segSides;
    const r1 = (i + 1) * segSides;
    for (let s = 0; s < segSides; s++) {
      const sn = (s + 1) % segSides;
      const a = r0 + s;
      const b = r0 + sn;
      const c = r1 + sn;
      const d = r1 + s;
      indices[idxOff++] = a;
      indices[idxOff++] = b;
      indices[idxOff++] = c;
      indices[idxOff++] = a;
      indices[idxOff++] = c;
      indices[idxOff++] = d;
    }
  }

  const cosTable = new Float32Array(segSides);
  const sinTable = new Float32Array(segSides);
  for (let s = 0; s < segSides; s++) {
    const ang = (s / segSides) * Math.PI * 2;
    cosTable[s] = Math.cos(ang);
    sinTable[s] = Math.sin(ang);
  }

  const geometry = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(positions, 3);
  posAttr.setUsage(THREE.DynamicDrawUsage);
  geometry.setAttribute('position', posAttr);
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  geometry.setDrawRange(0, 0);

  const mesh = new THREE.Mesh(geometry, material);
  mesh.frustumCulled = false;
  mesh.renderOrder = renderOrder;
  if (name) mesh.name = name;

  let activePoints = 0;

  function syncFromPositions(linePositions, count, { radii = null, fromIndex = 0 } = {}) {
    const n = Math.max(0, Math.min(cap, count | 0));
    activePoints = n;
    if (n < 2 || !linePositions) {
      geometry.setDrawRange(0, 0);
      return 0;
    }

    const start = Math.max(0, Math.min(n - 1, fromIndex | 0));
    let prevTx = 1, prevTy = 0, prevTz = 0;
    if (start > 0) {
      const pPrev = (start - 1) * 3;
      const pNext = Math.min(n - 1, start + 1) * 3;
      const dtx = linePositions[pNext] - linePositions[pPrev];
      const dty = linePositions[pNext + 1] - linePositions[pPrev + 1];
      const dtz = linePositions[pNext + 2] - linePositions[pPrev + 2];
      const dLen = Math.hypot(dtx, dty, dtz);
      if (dLen > 1e-6) {
        prevTx = dtx / dLen;
        prevTy = dty / dLen;
        prevTz = dtz / dLen;
      }
    }

    for (let i = start; i < n; i++) {
      const pOff = i * 3;
      const px = linePositions[pOff];
      const py = linePositions[pOff + 1];
      const pz = linePositions[pOff + 2];

      let tx, ty, tz;
      if (i === 0) {
        tx = linePositions[3] - px;
        ty = linePositions[4] - py;
        tz = linePositions[5] - pz;
      } else if (i === n - 1) {
        tx = px - linePositions[pOff - 3];
        ty = py - linePositions[pOff - 2];
        tz = pz - linePositions[pOff - 1];
      } else {
        tx = linePositions[pOff + 3] - linePositions[pOff - 3];
        ty = linePositions[pOff + 4] - linePositions[pOff - 2];
        tz = linePositions[pOff + 5] - linePositions[pOff - 1];
      }
      let tLen = Math.hypot(tx, ty, tz);
      if (tLen < 1e-6 && i > 0) {
        tx = px - linePositions[pOff - 3];
        ty = py - linePositions[pOff - 2];
        tz = pz - linePositions[pOff - 1];
        tLen = Math.hypot(tx, ty, tz);
      }
      if (tLen >= 1e-6) {
        tx /= tLen;
        ty /= tLen;
        tz /= tLen;
      } else {
        tx = prevTx;
        ty = prevTy;
        tz = prevTz;
      }

      let nx, ny, nz;
      if (i > 0) {
        const prevNOff = (i - 1) * 3;
        const pnx = ringNormals[prevNOff];
        const pny = ringNormals[prevNOff + 1];
        const pnz = ringNormals[prevNOff + 2];
        // Project previous normal onto the plane orthogonal to current tangent (parallel transport).
        const dotTN = tx * pnx + ty * pny + tz * pnz;
        nx = pnx - dotTN * tx;
        ny = pny - dotTN * ty;
        nz = pnz - dotTN * tz;
        const nLen = Math.hypot(nx, ny, nz);
        if (nLen >= 1e-4) {
          nx /= nLen;
          ny /= nLen;
          nz /= nLen;
        } else {
          const rx = Math.abs(tz) < 0.9 ? 0 : 1;
          const ry = 0;
          const rz = Math.abs(tz) < 0.9 ? 1 : 0;
          nx = ry * tz - rz * ty;
          ny = rz * tx - rx * tz;
          nz = rx * ty - ry * tx;
          const fbLen = Math.hypot(nx, ny, nz) || 1;
          nx /= fbLen;
          ny /= fbLen;
          nz /= fbLen;
        }
      } else {
        const rx = Math.abs(tz) < 0.9 ? 0 : 1;
        const ry = 0;
        const rz = Math.abs(tz) < 0.9 ? 1 : 0;
        nx = ry * tz - rz * ty;
        ny = rz * tx - rx * tz;
        nz = rx * ty - ry * tx;
        const fbLen = Math.hypot(nx, ny, nz) || 1;
        nx /= fbLen;
        ny /= fbLen;
        nz /= fbLen;
      }

      ringNormals[pOff] = nx;
      ringNormals[pOff + 1] = ny;
      ringNormals[pOff + 2] = nz;
      prevTx = tx;
      prevTy = ty;
      prevTz = tz;

      // Binormal B = T x N
      const bx = ty * nz - tz * ny;
      const by = tz * nx - tx * nz;
      const bz = tx * ny - ty * nx;

      const rawR = radii ? radii[i] : defaultRadius;
      const r = Number.isFinite(rawR) && rawR > 1e-5 ? rawR : defaultRadius;

      const vBase = i * segSides * 3;
      for (let s = 0; s < segSides; s++) {
        const c = cosTable[s] * r;
        const sn = sinTable[s] * r;
        const vOff = vBase + s * 3;
        positions[vOff] = px + nx * c + bx * sn;
        positions[vOff + 1] = py + ny * c + by * sn;
        positions[vOff + 2] = pz + nz * c + bz * sn;
      }
    }

    geometry.setDrawRange(0, (n - 1) * segSides * 6);
    posAttr.needsUpdate = true;
    return n - 1;
  }

  function clear() {
    activePoints = 0;
    geometry.setDrawRange(0, 0);
  }

  function dispose() {
    geometry.dispose();
  }

  return {
    mesh,
    capacity: cap,
    sides: segSides,
    syncFromPositions,
    clear,
    activeSegments: () => Math.max(0, activePoints - 1),
    dispose,
  };
}

/**
 * Paired single-draw-call pre-allocated 2-edge twist ribbon + border rail LineSegments
 * encoding both 3D tag roll (via rightAxes) and speed (via per-vertex heatmap + halfWidths).
 */
export function createPathRibbonMesh(maxPoints, material, {
  edgeMaterial = null,
  halfWidth = 0.005,
  rightEdgeTint = 0.18,
  renderOrder = 0,
  name = '',
} = {}) {
  const cap = Math.max(2, maxPoints | 0);
  const defaultHalfWidth = Number.isFinite(halfWidth) && halfWidth > 1e-5 ? halfWidth : 0.005;
  const tint = Number.isFinite(rightEdgeTint) ? Math.max(0, Math.min(1, rightEdgeTint)) : 0.18;

  const positions = new Float32Array(cap * 2 * 3);
  const colors = new Float32Array(cap * 2 * 3);
  const IndexArray = (cap * 2 <= 65535) ? Uint16Array : Uint32Array;
  const ribbonIndices = new IndexArray((cap - 1) * 6);
  const edgeIndices = new IndexArray((cap - 1) * 4);

  let rIdx = 0;
  let eIdx = 0;
  for (let i = 0; i < cap - 1; i++) {
    const v0 = i * 2;
    const v1 = v0 + 1;
    const v2 = v0 + 2;
    const v3 = v0 + 3;
    ribbonIndices[rIdx++] = v0;
    ribbonIndices[rIdx++] = v1;
    ribbonIndices[rIdx++] = v3;
    ribbonIndices[rIdx++] = v0;
    ribbonIndices[rIdx++] = v3;
    ribbonIndices[rIdx++] = v2;
    edgeIndices[eIdx++] = v0;
    edgeIndices[eIdx++] = v2;
    edgeIndices[eIdx++] = v1;
    edgeIndices[eIdx++] = v3;
  }

  const posAttr = new THREE.BufferAttribute(positions, 3);
  posAttr.setUsage(THREE.DynamicDrawUsage);
  const colAttr = new THREE.BufferAttribute(colors, 3);
  colAttr.setUsage(THREE.DynamicDrawUsage);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', posAttr);
  geometry.setAttribute('color', colAttr);
  geometry.setIndex(new THREE.BufferAttribute(ribbonIndices, 1));
  geometry.setDrawRange(0, 0);

  const edgeGeometry = new THREE.BufferGeometry();
  edgeGeometry.setAttribute('position', posAttr);
  edgeGeometry.setAttribute('color', colAttr);
  edgeGeometry.setIndex(new THREE.BufferAttribute(edgeIndices, 1));
  edgeGeometry.setDrawRange(0, 0);

  const mesh = new THREE.Mesh(geometry, material);
  mesh.frustumCulled = false;
  mesh.renderOrder = renderOrder;
  if (name) mesh.name = name;

  const ownsEdgeMaterial = !edgeMaterial;
  const resolvedEdgeMat = edgeMaterial || new THREE.LineBasicMaterial({
    color: 0xffffff,
    vertexColors: true,
    transparent: true,
    opacity: 0.95,
  });
  const edgeLines = new THREE.LineSegments(edgeGeometry, resolvedEdgeMat);
  edgeLines.frustumCulled = false;
  edgeLines.renderOrder = renderOrder + 1;
  if (name) edgeLines.name = `${name}-edges`;

  let activePoints = 0;

  function syncFromPositions(linePositions, count, {
    rightAxes = null,
    halfWidths = null,
    colors: sampleColors = null,
    fromIndex = 0,
  } = {}) {
    const n = Math.max(0, Math.min(cap, count | 0));
    activePoints = n;
    if (n < 2 || !linePositions) {
      geometry.setDrawRange(0, 0);
      edgeGeometry.setDrawRange(0, 0);
      return 0;
    }

    const start = Math.max(0, Math.min(n - 1, fromIndex | 0));
    for (let i = start; i < n; i++) {
      const pOff = i * 3;
      const px = linePositions[pOff];
      const py = linePositions[pOff + 1];
      const pz = linePositions[pOff + 2];

      let rx = NaN, ry = NaN, rz = NaN;
      if (rightAxes) {
        const ax = rightAxes[pOff];
        const ay = rightAxes[pOff + 1];
        const az = rightAxes[pOff + 2];
        const aLen = Math.hypot(ax, ay, az);
        if (Number.isFinite(aLen) && aLen >= 1e-4) {
          const inv = 1 / aLen;
          rx = ax * inv;
          ry = ay * inv;
          rz = az * inv;
        }
      }

      if (!Number.isFinite(rx)) {
        let tx, ty, tz;
        if (i === 0) {
          tx = linePositions[3] - px;
          ty = linePositions[4] - py;
          tz = linePositions[5] - pz;
        } else if (i === n - 1) {
          tx = px - linePositions[pOff - 3];
          ty = py - linePositions[pOff - 2];
          tz = pz - linePositions[pOff - 1];
        } else {
          tx = linePositions[pOff + 3] - linePositions[pOff - 3];
          ty = linePositions[pOff + 4] - linePositions[pOff - 2];
          tz = linePositions[pOff + 5] - linePositions[pOff - 1];
        }
        let tLen = Math.hypot(tx, ty, tz);
        if (tLen < 1e-6 && i > 0) {
          tx = px - linePositions[pOff - 3];
          ty = py - linePositions[pOff - 2];
          tz = pz - linePositions[pOff - 1];
          tLen = Math.hypot(tx, ty, tz);
        }
        if (tLen >= 1e-6) {
          tx /= tLen;
          ty /= tLen;
          tz /= tLen;
        } else {
          tx = 1;
          ty = 0;
          tz = 0;
        }
        const ux = Math.abs(tz) < 0.9 ? 0 : 1;
        const uy = 0;
        const uz = Math.abs(tz) < 0.9 ? 1 : 0;
        rx = uy * tz - uz * ty;
        ry = uz * tx - ux * tz;
        rz = ux * ty - uy * tx;
        const rLen = Math.hypot(rx, ry, rz) || 1;
        rx /= rLen;
        ry /= rLen;
        rz /= rLen;
      }

      const rawHW = halfWidths ? halfWidths[i] : defaultHalfWidth;
      const hw = Number.isFinite(rawHW) && rawHW > 1e-5 ? rawHW : defaultHalfWidth;

      let cr = 0.12, cg = 0.68, cb = 0.98;
      if (sampleColors) {
        const sr = sampleColors[pOff];
        const sg = sampleColors[pOff + 1];
        const sb = sampleColors[pOff + 2];
        if (Number.isFinite(sr) && Number.isFinite(sg) && Number.isFinite(sb)) {
          cr = sr;
          cg = sg;
          cb = sb;
        }
      }

      const lOff = i * 6;
      const rOff = lOff + 3;
      positions[lOff] = px - rx * hw;
      positions[lOff + 1] = py - ry * hw;
      positions[lOff + 2] = pz - rz * hw;
      positions[rOff] = px + rx * hw;
      positions[rOff + 1] = py + ry * hw;
      positions[rOff + 2] = pz + rz * hw;

      colors[lOff] = cr;
      colors[lOff + 1] = cg;
      colors[lOff + 2] = cb;
      colors[rOff] = cr + (1 - cr) * tint;
      colors[rOff + 1] = cg + (1 - cg) * tint;
      colors[rOff + 2] = cb + (1 - cb) * tint;
    }

    geometry.setDrawRange(0, (n - 1) * 6);
    edgeGeometry.setDrawRange(0, (n - 1) * 4);
    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    return n - 1;
  }

  function clear() {
    activePoints = 0;
    geometry.setDrawRange(0, 0);
    edgeGeometry.setDrawRange(0, 0);
  }

  function dispose() {
    geometry.dispose();
    edgeGeometry.dispose();
    if (ownsEdgeMaterial) resolvedEdgeMat.dispose();
  }

  return {
    mesh,
    edgeLines,
    capacity: cap,
    syncFromPositions,
    clear,
    activeSegments: () => Math.max(0, activePoints - 1),
    dispose,
  };
}
