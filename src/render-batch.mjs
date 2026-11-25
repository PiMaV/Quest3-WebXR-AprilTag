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
