import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../vendor/three/three.module.js';
import { createInstancedPool, createAxesBatch, createPathTubeMesh } from '../src/render-batch.mjs';
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
