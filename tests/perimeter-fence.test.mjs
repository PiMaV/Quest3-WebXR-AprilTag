import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  FENCE_HEIGHT_M,
  PERIMETER_MARKERS,
  isPerimeterTag,
  perimeterRole,
  projectPointToTablePlane,
  createPerimeterFence,
} from '../src/perimeter-fence.mjs';

const mainRig = JSON.parse(readFileSync(new URL('../rigs/ms-qr-main-l.json', import.meta.url), 'utf8'));
const tabletRig = JSON.parse(readFileSync(new URL('../rigs/ms-qr-tablet-l.json', import.meta.url), 'utf8'));

function translationMatrix(x, y, z = 0) {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1];
}

test('perimeter tag classification matches 353-359 clockwise table markers only', () => {
  assert.equal(FENCE_HEIGHT_M, 0.20);
  assert.deepEqual(
    PERIMETER_MARKERS.map((m) => [m.id, m.role]),
    [
      [358, 'YL'],
      [353, 'TL'],
      [357, 'XT'],
      [354, 'TR'],
      [359, 'YR'],
      [355, 'BR'],
      [356, 'XB'],
    ],
  );
  for (const id of [353, 354, 355, 356, 357, 358, 359]) {
    assert.equal(isPerimeterTag(id), true);
    assert.ok(perimeterRole(id));
  }
  for (const id of [300, 349, 350, 351, 352, 360, 370]) {
    assert.equal(isPerimeterTag(id), false);
    assert.equal(perimeterRole(id), null);
  }
});

test('projectPointToTablePlane cancels single-tag depth scale error along camera ray', () => {
  const cam = [0.20, 0.20, 0.60];
  // True tag at (0.50, 0.50, 0) pushed 5% too far along the viewing ray.
  const noisy = [
    cam[0] + 1.05 * (0.50 - cam[0]),
    cam[1] + 1.05 * (0.50 - cam[1]),
    cam[2] + 1.05 * (0.00 - cam[2]),
  ];
  const projected = projectPointToTablePlane(noisy, cam);
  assert.ok(Math.abs(projected[0] - 0.50) < 1e-9);
  assert.ok(Math.abs(projected[1] - 0.50) < 1e-9);
  assert.equal(projected[2], 0);

  // Grazing or missing camera falls back safely to orthographic z=0 projection.
  const fallback = projectPointToTablePlane([0.42, 0.31, 0.25], [0.2, 0.2, 0.6]);
  assert.deepEqual(fallback, [0.42, 0.31, 0]);
});

test('fence initializes from L-angle and maintains strict clockwise order regardless of discovery sequence', () => {
  const fence = createPerimeterFence(mainRig);
  assert.deepEqual(fence.orderedVertices().map((v) => v.id), [350, 352, 351]);
  assert.equal(fence.dynamicCount(), 0);

  const cam = [0.4, 0.3, 0.8];
  // Feed markers in scrambled order to form an 80 cm x 60 cm table rectangle.
  fence.observe(355, translationMatrix(0.8175, 0.015, 0), cam, 100); // BR
  fence.observe(353, translationMatrix(0.0175, 0.615, 0), cam, 110); // TL
  fence.observe(359, translationMatrix(0.8175, 0.315, 0), cam, 120); // YR
  fence.observe(358, translationMatrix(0.0175, 0.350, 0), cam, 130); // YL
  fence.observe(354, translationMatrix(0.8175, 0.615, 0), cam, 140); // TR
  fence.observe(356, translationMatrix(0.5000, 0.015, 0), cam, 150); // XB
  fence.observe(357, translationMatrix(0.4175, 0.615, 0), cam, 160); // XT

  assert.equal(fence.dynamicCount(), 7);
  assert.deepEqual(
    fence.orderedVertices().map((v) => v.id),
    [350, 352, 358, 353, 357, 354, 359, 355, 356, 351],
  );

  const snap = fence.snapshot();
  // Exact 0.80 m x 0.60 m rectangle -> area = 0.48 m², perimeter = 2.80 m.
  assert.ok(Math.abs(snap.areaM2 - 0.48) < 1e-6, `Expected 0.48 m², got ${snap.areaM2}`);
  assert.ok(Math.abs(snap.perimeterM - 2.80) < 1e-6, `Expected 2.80 m, got ${snap.perimeterM}`);

  // Pulling TR/BR/YR inward shrinks the enclosed area cleanly.
  fence.observe(354, translationMatrix(0.5175, 0.615, 0), cam, 1000);
  fence.observe(359, translationMatrix(0.5175, 0.315, 0), cam, 1000);
  fence.observe(355, translationMatrix(0.5175, 0.015, 0), cam, 1000);
  for (let i = 0; i < 40; i++) fence.step(0.02);
  const tighter = fence.snapshot();
  assert.ok(Math.abs(tighter.areaM2 - 0.30) < 1e-3, `Expected ~0.30 m², got ${tighter.areaM2}`);
});

test('single-frame spike is rejected while confirmed relocation and smooth sliding update cleanly', () => {
  const fence = createPerimeterFence(mainRig);
  const cam = [0.3, 0.3, 0.8];
  fence.observe(354, translationMatrix(0.70, 0.50, 0), cam, 1000);

  // Single-frame 30 cm glitch within 500 ms is ignored.
  const glitch = fence.observe(354, translationMatrix(1.00, 0.50, 0), cam, 1150);
  assert.equal(glitch.accepted, false);

  // A second consecutive frame agreeing with the new position confirms the move.
  const confirmed = fence.observe(354, translationMatrix(0.99, 0.50, 0), cam, 1300);
  assert.equal(confirmed.accepted, true);

  // Smooth step moves gradually toward target, then converges.
  assert.equal(fence.step(0.016), true);
  const mid = fence.orderedVertices().find((v) => v.id === 354).pos;
  assert.ok(mid[0] > 0.70 && mid[0] < 0.99);
  for (let i = 0; i < 40; i++) fence.step(0.02);
  const settled = fence.orderedVertices().find((v) => v.id === 354).pos;
  assert.ok(Math.abs(settled[0] - 0.99) < 0.001);

  // Switching rig updates the fixed L-angle anchors while reset clears dynamic markers.
  fence.setRig(tabletRig);
  assert.deepEqual(fence.orderedVertices().map((v) => v.id), [370, 372, 354, 371]);
  fence.reset();
  assert.deepEqual(fence.orderedVertices().map((v) => v.id), [370, 372, 371]);
});

test('perimeter stays idle until TL (353) is discovered, then offers and activates on confirmation', () => {
  const fence = createPerimeterFence(mainRig);
  const cam = [0.3, 0.3, 0.8];

  assert.equal(fence.status(), 'idle');
  assert.equal(fence.isActive(), false);
  assert.equal(fence.isOffered(), false);
  assert.equal(fence.triggerMarker(), null);
  assert.equal(fence.activate(), false);

  // Spotting another marker (e.g. TR 354) before TL does not offer or activate yet.
  const tr = fence.observe(354, translationMatrix(0.80, 0.60, 0), cam, 100);
  assert.equal(tr.justOffered, false);
  assert.equal(fence.status(), 'idle');

  // Spotting TL (353) transitions to 'offered' without activating automatically.
  const tl = fence.observe(353, translationMatrix(0.02, 0.60, 0), cam, 200);
  assert.equal(tl.justOffered, true);
  assert.equal(fence.status(), 'offered');
  assert.equal(fence.isOffered(), true);
  assert.equal(fence.isActive(), false);
  assert.equal(fence.triggerMarker()?.id, 353);

  // Re-observing TL does not re-fire justOffered.
  const tlAgain = fence.observe(353, translationMatrix(0.025, 0.60, 0), cam, 350);
  assert.equal(tlAgain.justOffered, false);

  // User presses TL -> activates perimeter with all already-observed markers intact.
  assert.equal(fence.activate(), true);
  assert.equal(fence.activate(), false);
  assert.equal(fence.status(), 'active');
  assert.equal(fence.isActive(), true);
  assert.equal(fence.isOffered(), false);
  assert.deepEqual(fence.orderedVertices().map((v) => v.id), [350, 352, 353, 354, 351]);

  // Reset returns cleanly to idle.
  fence.reset();
  assert.equal(fence.status(), 'idle');
  assert.equal(fence.isActive(), false);
  assert.equal(fence.isOffered(), false);
});

test('perimeter fence freezes updates under fast head motion and damps moderate motion', () => {
  const fence = createPerimeterFence(mainRig);
  const cam = [0.3, 0.3, 0.8];

  // Frozen or low-confidence motion must not seed a brand-new marker.
  assert.equal(fence.observe(354, translationMatrix(0.70, 0.50, 0), cam, 1000, { weight: 0 }).accepted, false);
  assert.equal(fence.observe(354, translationMatrix(0.70, 0.50, 0), cam, 1100, { weight: 0.3 }).accepted, false);
  assert.equal(fence.dynamicCount(), 0);

  // Clean steady frame seeds the marker.
  assert.equal(fence.observe(354, translationMatrix(0.70, 0.50, 0), cam, 1200, { weight: 1 }).accepted, true);
  assert.equal(fence.dynamicCount(), 1);

  // Frozen frame leaves target untouched.
  const frozen = fence.observe(354, translationMatrix(0.75, 0.50, 0), cam, 1300, { weight: 0 });
  assert.equal(frozen.accepted, false);
  assert.equal(frozen.reason, 'motion-frozen');

  // Tiny 5 mm jitter under moderate motion (weight 0.4) is ignored by deadband.
  const wiggle = fence.observe(354, translationMatrix(0.705, 0.50, 0), cam, 1400, { weight: 0.4 });
  assert.equal(wiggle.accepted, false);
  assert.equal(wiggle.reason, 'motion-deadband');

  // Larger shift under moderate motion (weight 0.5) blends halfway rather than snapping.
  const damped = fence.observe(354, translationMatrix(0.74, 0.50, 0), cam, 1500, { weight: 0.5 });
  assert.equal(damped.accepted, true);
  for (let i = 0; i < 60; i++) fence.step(0.02);
  const settled = fence.orderedVertices().find((v) => v.id === 354).pos;
  assert.ok(Math.abs(settled[0] - 0.72) < 0.001, `expected damped blend to 0.72, got ${settled[0]}`);
});

function yawMatrix(x, y, yawRad) {
  const c = Math.cos(yawRad);
  const s = Math.sin(yawRad);
  return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, x, y, 0, 1];
}

test('discoveredMarkers exposes yaw-aligned table poses before activation and wraps ±π smoothly', () => {
  const fence = createPerimeterFence(mainRig);
  const cam = [0.3, 0.3, 0.8];

  const angA = (175 * Math.PI) / 180;
  const ang90 = Math.PI / 2;
  const angNeg90 = -Math.PI / 2;

  // Observe markers BEFORE activate() (status stays 'idle' then 'offered')
  fence.observe(355, yawMatrix(0.80, 0.05, angNeg90), cam, 100);
  fence.observe(354, yawMatrix(0.80, 0.60, ang90), cam, 110);
  assert.equal(fence.status(), 'idle');

  let disc = fence.discoveredMarkers();
  assert.deepEqual(disc.map((m) => m.id), [354, 355]);
  assert.ok(Math.abs(disc[0].yaw - ang90) < 1e-6);
  assert.ok(Math.abs(disc[1].yaw - angNeg90) < 1e-6);
  assert.equal(disc[0].matrix.length, 16);
  assert.ok(Math.abs(disc[0].matrix[0] - Math.cos(ang90)) < 1e-6);
  assert.ok(Math.abs(disc[0].matrix[1] - Math.sin(ang90)) < 1e-6);

  // Discover TL (353) at +175° -> offered, clockwise order preserved
  fence.observe(353, yawMatrix(0.02, 0.60, angA), cam, 200);
  assert.equal(fence.status(), 'offered');
  disc = fence.discoveredMarkers();
  assert.deepEqual(disc.map((m) => m.id), [353, 354, 355]);
  assert.ok(Math.abs(disc[0].yaw - angA) < 1e-6);
  assert.ok(Math.abs(fence.triggerMarker().yaw - angA) < 1e-6);

  // Rotate TL (353) across the ±π seam (+175° -> -175°, short delta = +10°) and verify shortest-arc smoothing
  const angB = (-175 * Math.PI) / 180;
  fence.observe(353, yawMatrix(0.02, 0.60, angB), cam, 300);
  fence.step(0.02);
  const midYaw = fence.triggerMarker().yaw;
  // Midpoint between +175° and -175° via shortest arc sits near ±180° (|yaw| > 175°)
  assert.ok(Math.abs(midYaw) > (174 * Math.PI) / 180, `expected wraparound near ±π, got ${midYaw}`);
  for (let i = 0; i < 40; i++) fence.step(0.02);
  const settledYaw = fence.triggerMarker().yaw;
  assert.ok(Math.abs(settledYaw - angB) < 0.005, `expected yaw ~${angB}, got ${settledYaw}`);

  // Reset clears discoveredMarkers cleanly
  fence.reset();
  assert.deepEqual(fence.discoveredMarkers(), []);
});
