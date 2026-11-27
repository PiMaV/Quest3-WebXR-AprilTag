import test from 'node:test';
import assert from 'node:assert/strict';
import {
  pointInTagFrame,
  tagSurfaceMetrics,
  isPinchHandEligible,
  classifyCursorProximity,
  INDEX_TIP_HOVER_OFFSET_M,
  projectIndexSelectorPos,
  TAG_ACTION_PILL_DEFAULTS,
  createTagRestState,
  resetTagRestState,
  updateTagRestState,
  computeTagActionPillLayout,
  evaluateTagActionPills,
} from '../src/tag-tap.mjs';

// Column-major identity (flat on L-plane Z=0, outward normal +Z) placed at (tx, ty, tz).
function flatTagMatrix(tx = 0.2, ty = 0.1, tz = 0) {
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    tx, ty, tz, 1,
  ];
}

// Column-major vertical wall tag whose outward normal (+Z_tag) points along +X_L,
// local X_tag along +Y_L, local Y_tag along +Z_L.
function wallTagMatrix(tx = 0.5, ty = 0.0, tz = 0.3) {
  return [
    0, 1, 0, 0,
    0, 0, 1, 0,
    1, 0, 0, 0,
    tx, ty, tz, 1,
  ];
}

test('tagSurfaceMetrics measures distance to the full tag square on large static and vertical tags', () => {
  const flat80 = flatTagMatrix(0.2, 0.1, 0);
  // Approaching near the corner (u=+34 mm, v=-32 mm) of an 80 mm tag (half=40 mm) at 10 mm height:
  const m = tagSurfaceMetrics(flat80, 80, [0.234, 0.068, 0.010]);
  assert.ok(Math.abs(m.u - 0.034) < 1e-9);
  assert.ok(Math.abs(m.v - (-0.032)) < 1e-9);
  assert.ok(Math.abs(m.n - 0.010) < 1e-9);
  assert.equal(m.lateralOverflow, 0);
  assert.ok(Math.abs(m.surfaceDist - 0.010) < 1e-9);
  assert.ok(m.centerDist > 0.045);

  const wall = wallTagMatrix(0.5, 0.0, 0.3);
  const mw = pointInTagFrame(wall, [0.515, 0.02, 0.31]);
  assert.ok(Math.abs(mw.n - 0.015) < 1e-9);
  assert.ok(Math.abs(mw.u - 0.020) < 1e-9);
  assert.ok(Math.abs(mw.v - 0.010) < 1e-9);
});

test('isPinchHandEligible supports both left and right hands while guarding palm-facing OS menu and active pinch lock', () => {
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'right' }), true);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'left' }), true);
  assert.equal(isPinchHandEligible({ kind: 'ctrl', handedness: 'right' }), false);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: null }), false);

  // Left-hand pinch is suppressed only when looking directly at the left palm or wrist menu is open.
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'left', leftPalmFacingViewer: true }), false);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'left', leftMenuOpen: true }), false);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'right', leftPalmFacingViewer: true, leftMenuOpen: true }), true);

  // Active pinch hold locks onto the initiating hand so the opposite hand cannot hijack it.
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'left', pinHoldHand: 'left' }), true);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'right', pinHoldHand: 'left' }), false);
  assert.equal(isPinchHandEligible({ kind: 'hand', handedness: 'left', pinHoldHand: 'right' }), false);
});

test('classifyCursorProximity classifies independent per-probe hot, near, and idle distances', () => {
  assert.equal(classifyCursorProximity(0.02), 'hot');
  assert.equal(classifyCursorProximity(0.048), 'hot');
  assert.equal(classifyCursorProximity(0.08), 'near');
  assert.equal(classifyCursorProximity(0.14), 'near');
  assert.equal(classifyCursorProximity(0.20), 'idle');
  assert.equal(classifyCursorProximity(Infinity), 'idle');
  assert.equal(classifyCursorProximity(NaN), 'idle');
});

test('projectIndexSelectorPos hovers 18 mm ahead of tip along distal axis and falls back safely', () => {
  assert.equal(INDEX_TIP_HOVER_OFFSET_M, 0.018);

  // Unit -Z pointing finger: distal at z=-0.02, tip at z=-0.04 -> selector at z=-0.058
  const forward = projectIndexSelectorPos([0.1, 0.2, -0.04], [0.1, 0.2, -0.02]);
  assert.ok(Math.abs(forward[0] - 0.1) < 1e-9);
  assert.ok(Math.abs(forward[1] - 0.2) < 1e-9);
  assert.ok(Math.abs(forward[2] - (-0.058)) < 1e-9);

  // Diagonal 3D vector (1, 2, 2), length = 3 -> unit = (1/3, 2/3, 2/3)
  const diag = projectIndexSelectorPos([1.0, 2.0, 2.0], [0.0, 0.0, 0.0], 0.018);
  assert.ok(Math.abs(diag[0] - (1.0 + 0.006)) < 1e-9);
  assert.ok(Math.abs(diag[1] - (2.0 + 0.012)) < 1e-9);
  assert.ok(Math.abs(diag[2] - (2.0 + 0.012)) < 1e-9);
  assert.ok(Math.abs(Math.hypot(diag[0] - 1.0, diag[1] - 2.0, diag[2] - 2.0) - 0.018) < 1e-9);

  // Missing or coincident reference joint falls back cleanly to raw tip position without NaN
  assert.deepEqual(projectIndexSelectorPos([0.1, 0.2, 0.3], null), [0.1, 0.2, 0.3]);
  assert.deepEqual(projectIndexSelectorPos([0.1, 0.2, 0.3], [0.1, 0.2, 0.3]), [0.1, 0.2, 0.3]);

  // Invalid tip returns null
  assert.equal(projectIndexSelectorPos(null, [0, 0, 0]), null);
  assert.equal(projectIndexSelectorPos([0, NaN, 1], [0, 0, 0]), null);
});

test('updateTagRestState requires 500 ms stillness, supports bimanual hand-coupled tremor tolerance, and resets on motion/yaw', () => {
  assert.equal(TAG_ACTION_PILL_DEFAULTS.minRestMs, 500);
  const state = createTagRestState();
  const base = flatTagMatrix(0.2, 0.1, 0.0);

  // Initial observation starts the rest window.
  assert.deepEqual(updateTagRestState(state, base, 1000), { isResting: false, restingMs: 0 });
  // Small optical jitter (4 mm) does not reset the rest timer.
  assert.deepEqual(
    updateTagRestState(state, flatTagMatrix(0.204, 0.1, 0.0), 1499),
    { isResting: false, restingMs: 499 },
  );
  assert.deepEqual(
    updateTagRestState(state, flatTagMatrix(0.204, 0.1, 0.0), 1500),
    { isResting: true, restingMs: 500 },
  );

  // Translation > 12 mm when free immediately clears rest and restarts the 500 ms window.
  assert.deepEqual(
    updateTagRestState(state, flatTagMatrix(0.225, 0.1, 0.0), 1600),
    { isResting: false, restingMs: 0 },
  );
  assert.equal(updateTagRestState(state, flatTagMatrix(0.225, 0.1, 0.0), 2100).isResting, true);

  // In-plane yaw rotation (20 deg) resets rest even with zero translation.
  const rad = (20 * Math.PI) / 180;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  const yawed = [
    c, s, 0, 0,
    -s, c, 0, 0,
    0, 0, 1, 0,
    0.225, 0.1, 0.0, 1,
  ];
  assert.deepEqual(updateTagRestState(state, yawed, 2200), { isResting: false, restingMs: 0 });
  assert.equal(updateTagRestState(state, yawed, 2700).isResting, true);

  // Hand-coupled tag tolerates 20 mm natural hand tremor when held steady in one hand...
  const heldBase = flatTagMatrix(0.3, 0.2, 0.4);
  resetTagRestState(state);
  assert.deepEqual(updateTagRestState(state, heldBase, 3000, { handCoupled: true }), { isResting: false, restingMs: 0 });
  assert.deepEqual(
    updateTagRestState(state, flatTagMatrix(0.320, 0.2, 0.4), 3500, { handCoupled: true }),
    { isResting: true, restingMs: 500 },
  );
  // Mid-air tag (z = 0.4 m) ALSO gets hand-tremor tolerance even when handCoupled is false!
  assert.equal(
    updateTagRestState(state, flatTagMatrix(0.320, 0.2, 0.4), 3600, { handCoupled: false }).isResting,
    true,
  );
  // Once pills are open (pillsOpen: true), hysteresis tolerates 32 mm sway during a bimanual hold...
  assert.equal(
    updateTagRestState(state, flatTagMatrix(0.332, 0.2, 0.4), 3700, { handCoupled: false, pillsOpen: true }).isResting,
    true,
  );
  // ...but moving decisively (> 38 mm) resets rest immediately.
  assert.deepEqual(
    updateTagRestState(state, flatTagMatrix(0.345, 0.2, 0.4), 3800, { handCoupled: false, pillsOpen: true }),
    { isResting: false, restingMs: 0 },
  );

  resetTagRestState(state);
  assert.equal(state.anchorPos, null);
  assert.equal(state.sinceMs, null);
});

test('computeTagActionPillLayout places pills side-by-side below mid-air tags and in front of flat tabletop tags', () => {
  // 1. Mid-air tag at z = 0.40 m, viewer looking along +Y (viewerRight = +X, viewerUp = +Z).
  const midAir = flatTagMatrix(0.2, 0.3, 0.4);
  const airLayout = computeTagActionPillLayout({
    tagMatrix: midAir,
    edgeMm: 20,
    viewerPosL: [0.2, -0.2, 0.5],
    viewerRightL: [1, 0, 0],
    viewerUpL: [0, 0, 1],
    recording: false,
  });
  // Both pills sit below the tag (z < 0.40) and separated by 2 * 43 mm = 86 mm along X.
  assert.ok(airLayout.recPosL[2] < 0.36);
  assert.ok(airLayout.markPosL[2] < 0.36);
  const sepAir = Math.hypot(
    airLayout.markPosL[0] - airLayout.recPosL[0],
    airLayout.markPosL[1] - airLayout.recPosL[1],
    airLayout.markPosL[2] - airLayout.recPosL[2],
  );
  assert.ok(Math.abs(sepAir - 0.086) < 1e-6);
  assert.ok(airLayout.recPosL[0] < airLayout.zoneCenterL[0]);
  assert.ok(airLayout.markPosL[0] > airLayout.zoneCenterL[0]);

  // Active recording centers [■ STOP] directly on zoneCenterL below the tag.
  const recLayout = computeTagActionPillLayout({
    tagMatrix: midAir,
    edgeMm: 20,
    viewerPosL: [0.2, -0.2, 0.5],
    viewerRightL: [1, 0, 0],
    viewerUpL: [0, 0, 1],
    recording: true,
  });
  assert.deepEqual(recLayout.recPosL, recLayout.zoneCenterL);

  // 2. Flat tabletop tag at z = 0, viewer looking down from (0.2, -0.4, 0.5) with viewerUp tilted toward +Y/+Z.
  const tableTag = flatTagMatrix(0.2, 0.1, 0.0);
  const tableLayout = computeTagActionPillLayout({
    tagMatrix: tableTag,
    edgeMm: 20,
    viewerPosL: [0.2, -0.4, 0.5],
    viewerRightL: [1, 0, 0],
    viewerUpL: [0, 0.7071, 0.7071],
    recording: false,
  });
  // Both pills stay above the table plane (z >= 18 mm) and in front of the tag toward the viewer (y < 0.1).
  assert.ok(tableLayout.recPosL[2] >= TAG_ACTION_PILL_DEFAULTS.minTableZ - 1e-9);
  assert.ok(tableLayout.markPosL[2] >= TAG_ACTION_PILL_DEFAULTS.minTableZ - 1e-9);
  assert.ok(tableLayout.zoneCenterL[1] < 0.1 - 0.04);
  assert.ok(Math.abs(tableLayout.recPosL[2] - tableLayout.markPosL[2]) < 1e-9);
});

test('evaluateTagActionPills enforces resting + bottom-zone/finger proximity (including active recording)', () => {
  // Resting on table, no hand nearby -> no pills.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, hasPose: true, minTagDist: 0.40, minZoneDist: 0.35 }),
    { showRec: false, showMark: false },
  );

  // Approaching the bottom hover zone (without touching or occluding the tag) opens both pills.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, hasPose: true, minTagDist: 0.18, minZoneDist: 0.11 }),
    { showRec: true, showMark: true },
  );

  // Carrying hand excluded by caller -> all non-carrying distances are Infinity -> both hidden.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, hasPose: true }),
    { showRec: false, showMark: false },
  );

  // Active recording while moving -> [■ STOP] stays hidden (only red tag badge/frame shows).
  assert.deepEqual(
    evaluateTagActionPills({ isResting: false, recording: true, hasPose: true, minZoneDist: 0.05 }),
    { showRec: false, showMark: false },
  );

  // Active recording once resting >= 500 ms + finger approaches bottom zone -> [■ STOP] appears, [⏱ MARK] stays hidden.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: true, hasPose: true, minZoneDist: 0.10 }),
    { showRec: true, showMark: false },
  );

  // Active REC ALL suppresses the individual [● REC / ■ STOP] pill while still allowing [⏱ MARK] when not recording.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, recordAllActive: true, hasPose: true, minZoneDist: 0.10 }),
    { showRec: false, showMark: true },
  );
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: true, recordAllActive: true, hasPose: true, minZoneDist: 0.10 }),
    { showRec: false, showMark: false },
  );

  // Open/hold hysteresis keeps pills open at 17 cm (< 20 cm exitNearM) and closes once > 20 cm.
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, hasPose: true, wasOpen: true, minZoneDist: 0.17 }),
    { showRec: true, showMark: true },
  );
  assert.deepEqual(
    evaluateTagActionPills({ isResting: true, recording: false, hasPose: true, wasOpen: true, minZoneDist: 0.22 }),
    { showRec: false, showMark: false },
  );
});
