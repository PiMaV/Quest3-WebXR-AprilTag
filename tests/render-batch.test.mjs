import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../vendor/three/three.module.js';
import { createInstancedPool, createAxesBatch, createPathTubeMesh, createPathRibbonMesh } from '../src/render-batch.mjs';
import { createBadgeAtlas, createBillboardAtlasMesh, createOrientedAtlasMesh } from '../src/ui-atlas.mjs';

test('createInstancedPool packs active visible instances, handles colors, rods, and reuses freed slots', () => {
  const geo = new THREE.BoxGeometry(1, 1, 1);
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const pool = createInstancedPool(geo, mat, 4, { renderOrder: 7, name: 'test-pool' });

  assert.equal(pool.mesh.renderOrder, 7);
  assert.ok(pool.mesh.instanceColor, 'instanceColor must exist at construction time');
  assert.equal(pool.commit(), 0);

  const s0 = pool.alloc();
  const s1 = pool.alloc();
  const s2 = pool.alloc();
  const s3 = pool.alloc();
  assert.equal(pool.alloc(), -1, 'returns -1 when pool is full');

  pool.setPoint(s0, 1, 2, 3, 2);
  pool.setColor(s0, 0xff0000);
  pool.setRod(s1, 0, 0, 0, 0, 0, 0.5, 0.01);
  pool.setColor(s1, 0x00ff00);
  // Degenerate rod (<1e-4) must hide automatically until restored
  pool.setRod(s2, 1, 1, 1, 1, 1, 1, 0.01);
  pool.setMatrix(s3, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 4, 5, 6, 1]);
  pool.setVisible(s1, false);

  assert.equal(pool.commit(), 2);
  assert.equal(pool.mesh.count, 2);
  // First packed slot is s0 at (1,2,3), second is s3 at (4,5,6)
  assert.equal(pool.mesh.instanceMatrix.array[12], 1);
  assert.equal(pool.mesh.instanceMatrix.array[13], 2);
  assert.equal(pool.mesh.instanceMatrix.array[14], 3);
  assert.equal(pool.mesh.instanceMatrix.array[16 + 12], 4);
  assert.equal(pool.mesh.instanceMatrix.array[16 + 13], 5);
  assert.equal(pool.mesh.instanceMatrix.array[16 + 14], 6);

  // Restore s1 visibility and fix s2 rod
  pool.setVisible(s1, true);
  pool.setRod(s2, 0, 0, 0, 0.4, 0, 0, 0.005);
  assert.equal(pool.commit(), 4);

  // Midpoint of s1 rod (0,0,0)->(0,0,0.5) sits at z=0.25
  const s1Off = 16;
  assert.ok(Math.abs(pool.mesh.instanceMatrix.array[s1Off + 14] - 0.25) < 1e-5);

  pool.free(s0);
  const s0Reused = pool.alloc();
  assert.equal(s0Reused, s0);
  pool.clear();
  assert.equal(pool.mesh.count, 0);
  pool.dispose();
  geo.dispose();
  mat.dispose();
});

test('createAxesBatch transforms 3 orthogonal axes per slot and packs tightly', () => {
  const batch = createAxesBatch(3, { renderOrder: 3 });
  assert.equal(batch.lines.renderOrder, 3);
  assert.equal(batch.commit(), 0);

  const a0 = batch.alloc();
  const a1 = batch.alloc();
  // 90 deg rotation around Z: local X->+Y, local Y->-X, local Z->+Z, origin (1, 2, 3)
  const rotZ90 = [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1];
  batch.setPose(a0, rotZ90, 0.1);
  batch.setPose(a1, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1], 0.2);

  assert.equal(batch.commit(), 2);
  assert.equal(batch.lines.geometry.drawRange.count, 12);

  const pos = batch.lines.geometry.attributes.position.array;
  // a0 X tip should be (1, 2.1, 3)
  assert.ok(Math.abs(pos[3] - 1.0) < 1e-5);
  assert.ok(Math.abs(pos[4] - 2.1) < 1e-5);
  assert.ok(Math.abs(pos[5] - 3.0) < 1e-5);
  // a0 Y tip should be (0.9, 2.0, 3)
  assert.ok(Math.abs(pos[9] - 0.9) < 1e-5);
  assert.ok(Math.abs(pos[10] - 2.0) < 1e-5);

  batch.setVisible(a0, false);
  assert.equal(batch.commit(), 1);
  assert.equal(batch.lines.geometry.drawRange.count, 6);
  // Packed slot 0 now holds a1 at origin (5,6,7)
  assert.equal(pos[0], 5);
  assert.equal(pos[1], 6);
  assert.equal(pos[2], 7);

  batch.clear();
  assert.equal(batch.lines.geometry.drawRange.count, 0);
  batch.dispose();
});

function makeMockAtlasDeps() {
  const calls = [];
  const ctx = {
    save() { calls.push(['save']); },
    beginPath() {},
    rect(x, y, w, h) { calls.push(['rect', x, y, w, h]); },
    clip() {},
    clearRect(x, y, w, h) { calls.push(['clearRect', x, y, w, h]); },
    translate(x, y) { calls.push(['translate', x, y]); },
    restore() { calls.push(['restore']); },
  };
  const fakeCanvas = { width: 0, height: 0, getContext: () => ctx };
  const fakeTexture = { needsUpdate: false, disposed: false, dispose() { this.disposed = true; } };
  return {
    calls,
    createCanvas: (w, h) => { fakeCanvas.width = w; fakeCanvas.height = h; return fakeCanvas; },
    createTexture: () => fakeTexture,
    fakeTexture,
  };
}

test('createBadgeAtlas caches draws by key, computes inset flipY UVs, and flushes once when dirty', () => {
  const deps = makeMockAtlasDeps();
  const atlas = createBadgeAtlas({
    width: 512,
    height: 384,
    cellW: 256,
    cellH: 96,
    createCanvas: deps.createCanvas,
    createTexture: deps.createTexture,
  });
  assert.equal(atlas.capacity, 8);

  const s0 = atlas.acquire();
  const s1 = atlas.acquire();
  let draws = 0;
  assert.equal(atlas.draw(s0, 'k1', () => { draws++; }), true);
  assert.equal(atlas.draw(s0, 'k1', () => { draws++; }), false, 'same key must skip redraw');
  assert.equal(draws, 1);

  assert.equal(atlas.flush(), true);
  assert.equal(deps.fakeTexture.needsUpdate, true);
  deps.fakeTexture.needsUpdate = false;
  assert.equal(atlas.flush(), false, 'flush without new draw is a no-op');

  // Verify UV math for slot 3 (col 1, row 1 -> px=256, py=96)
  const s2 = atlas.acquire();
  const s3 = atlas.acquire();
  assert.ok(s2 >= 0 && s3 >= 0);
  const uv0 = atlas.uvRect(0);
  assert.ok(uv0.u0 > 0 && uv0.u1 < 0.5);
  assert.ok(uv0.vTop <= 1 && uv0.vTop > uv0.vBot);

  atlas.release(s1);
  assert.equal(atlas.acquire(), s1);
  atlas.dispose();
  assert.equal(deps.fakeTexture.disposed, true);
});

test('createBillboardAtlasMesh orients quads against viewerRight/viewerUp in a single draw call', () => {
  const deps = makeMockAtlasDeps();
  const atlas = createBadgeAtlas({
    width: 512,
    height: 192,
    cellW: 256,
    cellH: 96,
    createCanvas: deps.createCanvas,
    createTexture: deps.createTexture,
  });
  const bb = createBillboardAtlasMesh(atlas, 4, { renderOrder: 12, depthTest: false });
  const s0 = atlas.acquire();
  const s1 = atlas.acquire();
  atlas.draw(s0, 'a', () => {});
  atlas.draw(s1, 'b', () => {});

  bb.setQuad(s0, [1, 2, 3], 0.2, 0.1, true);
  bb.setQuad(s1, [4, 5, 6], 0.4, 0.2, false);
  const count = bb.updateBillboards([2, 0, 0], [0, 0, 3]); // non-unit basis must normalize
  assert.equal(count, 1);
  assert.equal(bb.mesh.geometry.drawRange.count, 6);
  assert.equal(deps.fakeTexture.needsUpdate, true);

  const pos = bb.mesh.geometry.attributes.position.array;
  // Center (1,2,3), hw=0.1 along +X, hh=0.05 along +Z:
  // v0 (bottom-left) = (0.9, 2, 2.95)
  assert.ok(Math.abs(pos[0] - 0.9) < 1e-5);
  assert.ok(Math.abs(pos[1] - 2.0) < 1e-5);
  assert.ok(Math.abs(pos[2] - 2.95) < 1e-5);
  // v2 (top-right) = (1.1, 2, 3.05)
  assert.ok(Math.abs(pos[6] - 1.1) < 1e-5);
  assert.ok(Math.abs(pos[7] - 2.0) < 1e-5);
  assert.ok(Math.abs(pos[8] - 3.05) < 1e-5);

  bb.setQuadVisible(s1, true);
  assert.equal(bb.updateBillboards([1, 0, 0], [0, 0, 1]), 2);
  assert.equal(bb.mesh.geometry.drawRange.count, 12);

  bb.clear();
  assert.equal(bb.mesh.geometry.drawRange.count, 0);
  bb.dispose();
  atlas.dispose();
});

test('createPathTubeMesh builds continuous variable-radius tube with incremental tail sync and clear()', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0x22d3ee });
  const tube = createPathTubeMesh(16, mat, { radius: 0.002, sides: 5, renderOrder: 4, name: 'test-tube' });
  assert.equal(tube.mesh.renderOrder, 4);
  assert.equal(tube.mesh.geometry.drawRange.count, 0);

  const pts = new Float32Array(16 * 3);
  const radii = new Float32Array(16);
  // 0 -> 1 -> 2 (straight +X) -> 3 (90 deg +Y) -> 4 (180 deg -Y reversal)
  const coords = [
    [0, 0, 0, 0.001],
    [0.05, 0, 0, 0.002],
    [0.10, 0, 0, 0.003],
    [0.10, 0.05, 0, 0.0025],
    [0.10, 0.01, 0, 0.0015],
  ];
  for (let i = 0; i < coords.length; i++) {
    pts[i * 3] = coords[i][0];
    pts[i * 3 + 1] = coords[i][1];
    pts[i * 3 + 2] = coords[i][2];
    radii[i] = coords[i][3];
  }

  // Count = 1 produces 0 segments
  assert.equal(tube.syncFromPositions(pts, 1, { radii }), 0);
  assert.equal(tube.mesh.geometry.drawRange.count, 0);

  // Build first 3 points, then incremental update for points 3..4
  assert.equal(tube.syncFromPositions(pts, 3, { radii, fromIndex: 0 }), 2);
  assert.equal(tube.mesh.geometry.drawRange.count, 2 * 5 * 6);
  assert.equal(tube.syncFromPositions(pts, 5, { radii, fromIndex: 2 }), 4);
  assert.equal(tube.mesh.geometry.drawRange.count, 4 * 5 * 6);
  assert.equal(tube.activeSegments(), 4);

  const pos = tube.mesh.geometry.attributes.position.array;
  for (let i = 0; i < 5 * 5 * 3; i++) {
    assert.ok(Number.isFinite(pos[i]), `vertex float ${i} must be finite`);
  }
  // Ring 2 vertices should sit at radius ~0.003 around (0.10, 0, 0)
  const v20x = pos[2 * 5 * 3], v20y = pos[2 * 5 * 3 + 1], v20z = pos[2 * 5 * 3 + 2];
  const dist2 = Math.hypot(v20x - 0.10, v20y, v20z);
  assert.ok(Math.abs(dist2 - 0.003) < 1e-5, `expected ring radius 0.003, got ${dist2}`);

  tube.clear();
  assert.equal(tube.mesh.geometry.drawRange.count, 0);
  assert.equal(tube.activeSegments(), 0);
  tube.dispose();
  mat.dispose();
});

test('createOrientedAtlasMesh places quads flat on rotated 3D frames and skips clean commits', () => {
  const deps = makeMockAtlasDeps();
  const atlas = createBadgeAtlas({
    width: 256,
    height: 256,
    cellW: 128,
    cellH: 128,
    createCanvas: deps.createCanvas,
    createTexture: deps.createTexture,
  });
  const meshBatch = createOrientedAtlasMesh(atlas, 4, { renderOrder: 6, depthTest: true });
  assert.equal(meshBatch.mesh.renderOrder, 6);
  assert.equal(meshBatch.commit(), 0);

  const s0 = atlas.acquire();
  const s1 = atlas.acquire();
  atlas.draw(s0, 'id301', () => {});
  atlas.draw(s1, 'id302', () => {});

  // 90 deg rotation around X: local +X -> +X, local +Y -> +Z, local +Z (normal) -> -Y
  const matRotX90 = [1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0.5, 0.6, 0.2, 1];
  meshBatch.setOrientedQuad(s0, matRotX90, 0.02, 0.02, 0.001, true);
  meshBatch.setOrientedQuad(s1, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], 0.02, 0.02, 0.001, false);

  assert.equal(meshBatch.commit(), 1);
  assert.equal(meshBatch.mesh.geometry.drawRange.count, 6);
  assert.equal(deps.fakeTexture.needsUpdate, true);

  const pos = meshBatch.mesh.geometry.attributes.position.array;
  // Center shifted along normal (-Y) by 0.001 -> y = 0.599; v0 = center - 0.01*X - 0.01*Z = (0.49, 0.599, 0.19)
  assert.ok(Math.abs(pos[0] - 0.49) < 1e-5);
  assert.ok(Math.abs(pos[1] - 0.599) < 1e-5);
  assert.ok(Math.abs(pos[2] - 0.19) < 1e-5);
  // v2 = center + 0.01*X + 0.01*Z = (0.51, 0.599, 0.21)
  assert.ok(Math.abs(pos[6] - 0.51) < 1e-5);
  assert.ok(Math.abs(pos[7] - 0.599) < 1e-5);
  assert.ok(Math.abs(pos[8] - 0.21) < 1e-5);

  // Unchanged inputs skip dirty rewrite
  const posVersion = meshBatch.mesh.geometry.attributes.position.version;
  meshBatch.setOrientedQuad(s0, matRotX90, 0.02, 0.02, 0.001, true);
  assert.equal(meshBatch.commit(), 1);
  assert.equal(meshBatch.mesh.geometry.attributes.position.version, posVersion);

  meshBatch.setQuadVisible(s1, true);
  assert.equal(meshBatch.commit(), 2);
  meshBatch.clearSlot(s0);
  assert.equal(meshBatch.commit(), 1);

  meshBatch.clear();
  assert.equal(meshBatch.mesh.geometry.drawRange.count, 0);
  meshBatch.dispose();
  atlas.dispose();
});

test('createPathRibbonMesh builds 2-edge twist ribbon + border rails with heatmap colors and clear()', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, vertexColors: true, transparent: true, opacity: 0.42, side: THREE.DoubleSide });
  const ribbon = createPathRibbonMesh(16, mat, { halfWidth: 0.005, rightEdgeTint: 0.2, renderOrder: 2, name: 'test-ribbon' });
  assert.equal(ribbon.mesh.renderOrder, 2);
  assert.equal(ribbon.edgeLines.renderOrder, 3);
  assert.equal(ribbon.mesh.geometry.drawRange.count, 0);
  assert.equal(ribbon.edgeLines.geometry.drawRange.count, 0);
  // Shared vertex buffers between ribbon face and border edge lines
  assert.equal(ribbon.mesh.geometry.attributes.position, ribbon.edgeLines.geometry.attributes.position);
  assert.equal(ribbon.mesh.geometry.attributes.color, ribbon.edgeLines.geometry.attributes.color);

  const pts = new Float32Array(16 * 3);
  const axes = new Float32Array(16 * 3);
  const hw = new Float32Array(16);
  const cols = new Float32Array(16 * 3);
  // 4 samples moving along +Y while tag twists from +X to +Z:
  const samples = [
    { p: [0, 0.00, 0], r: [1, 0, 0], hw: 0.004, c: [0.1, 0.7, 1.0] },
    { p: [0, 0.05, 0], r: [0.7071, 0, 0.7071], hw: 0.008, c: [0.2, 0.8, 0.3] },
    { p: [0, 0.10, 0], r: [0, 0, 1], hw: 0.012, c: [1.0, 0.8, 0.1] },
    { p: [0, 0.15, 0], r: [0, 0, 1], hw: 0.014, c: [0.95, 0.2, 0.2] },
  ];
  for (let i = 0; i < samples.length; i++) {
    pts.set(samples[i].p, i * 3);
    axes.set(samples[i].r, i * 3);
    hw[i] = samples[i].hw;
    cols.set(samples[i].c, i * 3);
  }

  assert.equal(ribbon.syncFromPositions(pts, 1, { rightAxes: axes, halfWidths: hw, colors: cols }), 0);
  assert.equal(ribbon.mesh.geometry.drawRange.count, 0);

  assert.equal(ribbon.syncFromPositions(pts, 3, { rightAxes: axes, halfWidths: hw, colors: cols, fromIndex: 0 }), 2);
  assert.equal(ribbon.mesh.geometry.drawRange.count, 2 * 6);
  assert.equal(ribbon.edgeLines.geometry.drawRange.count, 2 * 4);

  // Incremental append of index 3
  assert.equal(ribbon.syncFromPositions(pts, 4, { rightAxes: axes, halfWidths: hw, colors: cols, fromIndex: 2 }), 3);
  assert.equal(ribbon.mesh.geometry.drawRange.count, 3 * 6);
  assert.equal(ribbon.edgeLines.geometry.drawRange.count, 3 * 4);
  assert.equal(ribbon.activeSegments(), 3);

  const pos = ribbon.mesh.geometry.attributes.position.array;
  const col = ribbon.mesh.geometry.attributes.color.array;
  // Sample 0: right axis +X, hw 0.004 -> left=(-0.004,0,0), right=(+0.004,0,0)
  assert.ok(Math.abs(pos[0] - (-0.004)) < 1e-5 && Math.abs(pos[3] - 0.004) < 1e-5);
  // Sample 2: right axis +Z (90 deg twist), hw 0.012 -> left=(0,0.10,-0.012), right=(0,0.10,+0.012)
  assert.ok(Math.abs(pos[2 * 6 + 2] - (-0.012)) < 1e-5);
  assert.ok(Math.abs(pos[2 * 6 + 5] - 0.012) < 1e-5);
  // Sample 3 color: left gets exact red, right is tinted slightly lighter so twist edges stay distinct
  assert.ok(Math.abs(col[3 * 6] - 0.95) < 1e-5 && Math.abs(col[3 * 6 + 1] - 0.2) < 1e-5);
  assert.ok(col[3 * 6 + 4] > col[3 * 6 + 1]);

  // Fallback axis path (no rightAxes passed) must stay finite and orthogonal
  assert.equal(ribbon.syncFromPositions(pts, 3), 2);
  for (let i = 0; i < 3 * 6; i++) {
    assert.ok(Number.isFinite(pos[i]));
  }

  ribbon.clear();
  assert.equal(ribbon.mesh.geometry.drawRange.count, 0);
  assert.equal(ribbon.edgeLines.geometry.drawRange.count, 0);
  assert.equal(ribbon.activeSegments(), 0);
  ribbon.dispose();
  mat.dispose();
});

test('createInstancedPool and createAxesBatch skip dirty uploads on repeated identical poses', () => {
  const geo = new THREE.BoxGeometry(1, 1, 1);
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const pool = createInstancedPool(geo, mat, 4);
  const s0 = pool.alloc();
  const s1 = pool.alloc();
  const s2 = pool.alloc();
  const m = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.2, 0.3, 0.4, 1];
  pool.setMatrix(s0, m);
  pool.setPoint(s1, 1, 2, 3, 0.5);
  pool.setRod(s2, 0, 0, 0, 0, 0, 0.4, 0.002);
  assert.equal(pool.commit(), 3);

  const vAfterFirst = pool.mesh.instanceMatrix.version;
  pool.setMatrix(s0, m);
  pool.setPoint(s1, 1, 2, 3, 0.5);
  pool.setRod(s2, 0, 0, 0, 0, 0, 0.4, 0.002);
  assert.equal(pool.commit(), 3);
  assert.equal(pool.mesh.instanceMatrix.version, vAfterFirst, 'identical matrix/point/rod calls must not re-upload');

  pool.setMatrix(s0, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.25, 0.3, 0.4, 1]);
  assert.equal(pool.commit(), 3);
  assert.ok(pool.mesh.instanceMatrix.version > vAfterFirst, 'changed matrix must upload');

  const axes = createAxesBatch(2);
  const a0 = axes.alloc();
  const pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.1, 0.2, 0.3, 1];
  axes.setPose(a0, pose, 0.05);
  assert.equal(axes.commit(), 1);
  const axesVer = axes.lines.geometry.attributes.position.version;
  axes.setPose(a0, pose, 0.05);
  assert.equal(axes.commit(), 1);
  assert.equal(axes.lines.geometry.attributes.position.version, axesVer, 'identical axes pose must not re-upload');

  axes.setPose(a0, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.15, 0.2, 0.3, 1], 0.05);
  assert.equal(axes.commit(), 1);
  assert.ok(axes.lines.geometry.attributes.position.version > axesVer, 'changed axes pose must upload');

  axes.dispose();
  pool.dispose();
  geo.dispose();
  mat.dispose();
});

test('createInstancedPool alertAttribute isolates shared geometry, packs aAlert cleanly, and resets on free/alloc/clear', () => {
  const sharedGeo = new THREE.BoxGeometry(1, 1, 1);
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const pool = createInstancedPool(sharedGeo, mat, 4, { alertAttribute: true, name: 'alert-test' });

  assert.equal(sharedGeo.getAttribute('aAlert'), undefined, 'must not mutate caller shared geometry');
  const aAlert = pool.mesh.geometry.getAttribute('aAlert');
  assert.ok(aAlert, 'cloned mesh geometry must have aAlert attribute');

  const s0 = pool.alloc();
  const s1 = pool.alloc();
  const s2 = pool.alloc();
  const idMat = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  pool.setMatrix(s0, idMat);
  pool.setMatrix(s1, idMat);
  pool.setMatrix(s2, idMat);
  pool.setAlert(s0, true);
  pool.setAlert(s1, false);
  pool.setAlert(s2, true);
  pool.setVisible(s1, false);

  assert.equal(pool.commit(), 2);
  assert.equal(pool.alertCount(), 2);
  assert.equal(aAlert.array[0], 1);
  assert.equal(aAlert.array[1], 1);

  const alertVer = aAlert.version;
  pool.setAlert(s0, true);
  assert.equal(pool.commit(), 2);
  assert.equal(aAlert.version, alertVer, 'no-op setAlert must not mark buffer dirty');

  pool.setAlert(s0, false);
  assert.equal(pool.commit(), 2);
  assert.equal(pool.alertCount(), 1);
  assert.equal(aAlert.array[0], 0);
  assert.equal(aAlert.array[1], 1);
  assert.ok(aAlert.version > alertVer);

  pool.free(s2);
  const s2Reused = pool.alloc();
  assert.equal(s2Reused, s2);
  pool.setMatrix(s2Reused, idMat);
  assert.equal(pool.commit(), 2);
  assert.equal(pool.alertCount(), 0, 'alert flag must reset on free/alloc');

  pool.setAlert(s2Reused, true);
  pool.commit();
  assert.equal(pool.alertCount(), 1);
  pool.clear();
  assert.equal(pool.alertCount(), 0);

  pool.dispose();
  sharedGeo.dispose();
  mat.dispose();
});

test('createPathTubeMesh supports vertexColors + aDash attributes and zero-position syncColors / syncOutsideFlags', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, vertexColors: true });
  const tube = createPathTubeMesh(16, mat, {
    radius: 0.0015,
    sides: 5,
    vertexColors: true,
    dashAttribute: true,
    renderOrder: 2,
    name: 'test-colored-dashed-tube',
  });

  const colAttr = tube.mesh.geometry.getAttribute('color');
  const dashAttr = tube.mesh.geometry.getAttribute('aDash');
  const posAttr = tube.mesh.geometry.getAttribute('position');
  assert.ok(colAttr, 'color attribute must exist');
  assert.ok(dashAttr, 'aDash attribute must exist');

  const pts = new Float32Array(16 * 3);
  const cols = new Float32Array(16 * 3);
  const outside = new Uint8Array(16);
  // 4 collinear points spaced 0.03 m along +X
  for (let i = 0; i < 4; i++) {
    pts[i * 3] = i * 0.03;
    cols[i * 3] = i * 0.25;
    cols[i * 3 + 1] = 0.5;
    cols[i * 3 + 2] = 1 - i * 0.25;
    outside[i] = i >= 2 ? 1 : 0;
  }

  assert.equal(tube.syncFromPositions(pts, 3, { colors: cols, outsideFlags: outside, fromIndex: 0 }), 2);
  assert.equal(tube.syncFromPositions(pts, 4, { colors: cols, outsideFlags: outside, fromIndex: 2 }), 3);

  const colArr = colAttr.array;
  const dashArr = dashAttr.array;
  // Ring 2 color replicated across all 5 ring vertices
  for (let s = 0; s < 5; s++) {
    const off = (2 * 5 + s) * 3;
    assert.ok(Math.abs(colArr[off] - 0.5) < 1e-6);
    assert.ok(Math.abs(colArr[off + 1] - 0.5) < 1e-6);
    assert.ok(Math.abs(colArr[off + 2] - 0.5) < 1e-6);
  }
  // Ring 3 cumulative arc length = 0.09 m, outside flag = 1
  for (let s = 0; s < 5; s++) {
    const dOff = (3 * 5 + s) * 2;
    assert.ok(Math.abs(dashArr[dOff] - 0.09) < 1e-5);
    assert.equal(dashArr[dOff + 1], 1);
  }
  // Ring 1 outside flag = 0, arc length = 0.03 m
  assert.ok(Math.abs(dashArr[5 * 2] - 0.03) < 1e-5);
  assert.equal(dashArr[5 * 2 + 1], 0);

  // syncColors must update color buffer without touching position.version or aDash.version
  const posVerBefore = posAttr.version;
  const dashVerBefore = dashAttr.version;
  const colVerBefore = colAttr.version;
  const nextCols = new Float32Array(16 * 3);
  for (let i = 0; i < 4; i++) {
    nextCols[i * 3] = 0.9;
    nextCols[i * 3 + 1] = 0.2;
    nextCols[i * 3 + 2] = 0.1;
  }
  assert.equal(tube.syncColors(nextCols, 4), 4);
  assert.equal(posAttr.version, posVerBefore, 'syncColors must not mark position dirty');
  assert.equal(dashAttr.version, dashVerBefore, 'syncColors must not mark aDash dirty');
  assert.ok(colAttr.version > colVerBefore);
  assert.ok(Math.abs(colArr[3 * 5 * 3] - 0.9) < 1e-6);

  // syncOutsideFlags must update aDash.y while preserving arc lengths and not touching position/color
  const colVerBeforeFlags = colAttr.version;
  const flipped = new Uint8Array([1, 1, 0, 0]);
  assert.equal(tube.syncOutsideFlags(flipped, 4), 4);
  assert.equal(posAttr.version, posVerBefore);
  assert.equal(colAttr.version, colVerBeforeFlags);
  assert.ok(dashAttr.version > dashVerBefore);
  assert.ok(Math.abs(dashArr[3 * 5 * 2] - 0.09) < 1e-5, 'arc length must be preserved');
  assert.equal(dashArr[3 * 5 * 2 + 1], 0);
  assert.equal(dashArr[1], 1);

  tube.dispose();
  mat.dispose();
});

test('createPathTubeMesh supports optional per-vertex aFade attribute with clamping and preserves default omission', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, vertexColors: true, transparent: true });
  const plainTube = createPathTubeMesh(8, mat, { sides: 4, vertexColors: true });
  assert.equal(plainTube.mesh.geometry.getAttribute('aFade'), undefined, 'aFade must not exist when fadeAttribute is omitted');
  plainTube.dispose();

  const fadeTube = createPathTubeMesh(8, mat, { sides: 4, vertexColors: true, fadeAttribute: true, name: 'fade-tube' });
  const fadeAttr = fadeTube.mesh.geometry.getAttribute('aFade');
  assert.ok(fadeAttr, 'aFade attribute must exist when fadeAttribute: true');
  assert.equal(fadeAttr.array.length, 8 * 4);

  const pts = new Float32Array(4 * 3);
  const cols = new Float32Array(4 * 3);
  for (let i = 0; i < 4; i++) {
    pts[i * 3] = i * 0.02;
    cols[i * 3] = 1; cols[i * 3 + 1] = 1; cols[i * 3 + 2] = 1;
  }
  const fades = new Float32Array([-0.4, 0.25, 1.8, NaN]);
  assert.equal(fadeTube.syncFromPositions(pts, 4, { colors: cols, fades, fromIndex: 0 }), 3);

  const arr = fadeAttr.array;
  const verBefore = fadeAttr.version;
  // Ring 0 clamped from -0.4 to 0, Ring 1 = 0.25, Ring 2 clamped from 1.8 to 1, Ring 3 NaN fallback to 1
  for (let s = 0; s < 4; s++) {
    assert.equal(arr[0 * 4 + s], 0);
    assert.ok(Math.abs(arr[1 * 4 + s] - 0.25) < 1e-6);
    assert.equal(arr[2 * 4 + s], 1);
    assert.equal(arr[3 * 4 + s], 1);
  }
  assert.ok(verBefore >= 1);

  fadeTube.dispose();
  mat.dispose();
});

test('createPathTubeMesh supports breakFlags: skips gap bridges, resets aDash arc length, handles isolated points, and restores contiguous indices', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, vertexColors: true });
  const tube = createPathTubeMesh(16, mat, {
    radius: 0.0015,
    sides: 4,
    vertexColors: true,
    dashAttribute: true,
    name: 'break-tube',
  });

  // Two segments: 0->1->2 (along +X, 0.03 m each) and 3->4 (shifted to y=0.5, 0.04 m along +X)
  const pts = new Float32Array(16 * 3);
  const cols = new Float32Array(16 * 3);
  const breaks = new Uint8Array(16);
  const coords = [
    [0.00, 0.0, 0.0],
    [0.03, 0.0, 0.0],
    [0.06, 0.0, 0.0],
    [0.50, 0.5, 0.2],
    [0.54, 0.5, 0.2],
  ];
  for (let i = 0; i < coords.length; i++) {
    pts[i * 3] = coords[i][0];
    pts[i * 3 + 1] = coords[i][1];
    pts[i * 3 + 2] = coords[i][2];
    cols[i * 3] = 1; cols[i * 3 + 1] = 1; cols[i * 3 + 2] = 1;
  }
  breaks[3] = 1;

  const segs = tube.syncFromPositions(pts, 5, { colors: cols, breakFlags: breaks, fromIndex: 0 });
  assert.equal(segs, 3, '5 points with 1 break must yield 3 tube segments (0->1, 1->2, 3->4)');
  assert.equal(tube.activeSegments(), 3);
  assert.equal(tube.mesh.geometry.drawRange.count, 3 * 4 * 6);

  // Verify no triangle connects ring 2 (verts 8..11) to ring 3 (verts 12..15)
  const idxArr = tube.mesh.geometry.getIndex().array;
  const drawCount = tube.mesh.geometry.drawRange.count;
  for (let k = 0; k < drawCount; k += 3) {
    const a = idxArr[k], b = idxArr[k + 1], c = idxArr[k + 2];
    const minRing = Math.floor(Math.min(a, b, c) / 4);
    const maxRing = Math.floor(Math.max(a, b, c) / 4);
    assert.ok(!(minRing <= 2 && maxRing >= 3), `bridge triangle across break found: rings ${minRing}..${maxRing}`);
  }

  // Verify aDash.x resets to 0 at segment start index 3, and advances to 0.04 at index 4
  const dashArr = tube.mesh.geometry.getAttribute('aDash').array;
  assert.ok(Math.abs(dashArr[2 * 4 * 2] - 0.06) < 1e-5);
  assert.equal(dashArr[3 * 4 * 2], 0, 'arc length must reset to 0 at break start');
  assert.ok(Math.abs(dashArr[4 * 4 * 2] - 0.04) < 1e-5);

  // Isolated single points: [0, 1] (two 1-pt segments) and [0, 1, 1] (three 1-pt segments) -> 0 active segments & finite vertices
  const isoPts = new Float32Array([0, 0, 0, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4]);
  const isoBreaks2 = new Uint8Array([0, 1]);
  assert.equal(tube.syncFromPositions(isoPts, 2, { breakFlags: isoBreaks2, fromIndex: 0 }), 0);
  assert.equal(tube.activeSegments(), 0);
  assert.equal(tube.mesh.geometry.drawRange.count, 0);

  const isoBreaks3 = new Uint8Array([0, 1, 1]);
  assert.equal(tube.syncFromPositions(isoPts, 3, { breakFlags: isoBreaks3, fromIndex: 0 }), 0);
  const posArr = tube.mesh.geometry.getAttribute('position').array;
  for (let i = 0; i < 3 * 4 * 3; i++) {
    assert.ok(Number.isFinite(posArr[i]), `isolated ring float ${i} must be finite`);
  }

  // Rebuilding longer unbroken polyline after clear() must restore contiguous indices cleanly (no stale packed indices)
  tube.clear();
  assert.equal(tube.activeSegments(), 0);
  const unbroken = new Float32Array(7 * 3);
  const unbrokenCols = new Float32Array(7 * 3);
  for (let i = 0; i < 7; i++) {
    unbroken[i * 3] = i * 0.03;
    unbrokenCols[i * 3] = 1; unbrokenCols[i * 3 + 1] = 1; unbrokenCols[i * 3 + 2] = 1;
  }
  assert.equal(tube.syncFromPositions(unbroken, 7, { colors: unbrokenCols, fromIndex: 0 }), 6);
  assert.equal(tube.activeSegments(), 6);
  assert.equal(tube.mesh.geometry.drawRange.count, 6 * 4 * 6);
  // Segment 5 connects ring 5 (20..23) -> ring 6 (24..27)
  const seg5Start = 5 * 4 * 6;
  const maxIdxInSeg5 = Math.max(...idxArr.slice(seg5Start, seg5Start + 4 * 6));
  assert.equal(maxIdxInSeg5, 27);

  tube.dispose();
  mat.dispose();
});

test('createPathTubeMesh sortByFade orders low -> mid -> high opacity segments, respects breaks, and restores contiguous order', () => {
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, vertexColors: true, transparent: true });
  const tube = createPathTubeMesh(16, mat, {
    sides: 4,
    vertexColors: true,
    fadeAttribute: true,
    name: 'sorted-fade-tube',
  });

  // 8 points across 2 runs (break at index 4):
  // Seg A (rings 0..1): high fade (1.0 -> bucket 2)
  // Seg B (rings 1..2): mid fade (0.48 -> bucket 1)
  // Seg C (rings 2..3): high fade (1.0 -> bucket 2)
  // Break at 4 (3->4 skipped)
  // Seg D (rings 4..5): low fade (0.22 -> bucket 0)
  // Seg E (rings 5..6): low fade (0.22 -> bucket 0)
  // Seg F (rings 6..7): mid fade (0.72 -> bucket 1)
  const pts = new Float32Array(8 * 3);
  const cols = new Float32Array(8 * 3).fill(1);
  const fades = new Float32Array([1.0, 1.0, 0.48, 1.0, 0.22, 0.22, 0.72, 0.72]);
  const breaks = new Uint8Array([0, 0, 0, 0, 1, 0, 0, 0]);
  for (let i = 0; i < 8; i++) pts[i * 3] = i * 0.03;

  const segs = tube.syncFromPositions(pts, 8, { colors: cols, fades, breakFlags: breaks, fromIndex: 0, sortByFade: true });
  assert.equal(segs, 6);
  const idxArr = tube.mesh.geometry.getIndex().array;
  const trisPerSeg = 4 * 6;
  const order = [];
  for (let s = 0; s < segs; s++) {
    order.push(Math.floor(idxArr[s * trisPerSeg] / 4));
  }
  // Segs 4 (4->5, min=0.22) and 5 (5->6, min=0.22) -> bucket 0
  // Segs 1 (1->2, min=0.48), 2 (2->3, min=0.48), 6 (6->7, min=0.72) -> bucket 1
  // Seg 0 (0->1, min=1.0) -> bucket 2
  assert.deepEqual(order, [4, 5, 1, 2, 6, 0]);

  // Restoring without breaks and sortByFade: false must rebuild contiguous indices 0..6
  assert.equal(tube.syncFromPositions(pts, 8, { colors: cols, fades, fromIndex: 0, sortByFade: false }), 7);
  const restored = [];
  for (let s = 0; s < 7; s++) {
    restored.push(Math.floor(idxArr[s * trisPerSeg] / 4));
  }
  assert.deepEqual(restored, [0, 1, 2, 3, 4, 5, 6]);

  tube.dispose();
  mat.dispose();
});
