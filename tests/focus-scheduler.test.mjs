import test from 'node:test';
import assert from 'node:assert/strict';
import {
  FOCUS_SCHEDULER_DEFAULTS,
  createFocusScheduler,
  projectTagGazeMetrics,
} from '../src/focus-scheduler.mjs';
import { vectorRigid } from '../src/runtime.mjs';

const I = vectorRigid([0, 0, 0, 0, 0, 0]);
const K = { width: 1280, height: 1080, fx: 886, fy: 886, cx: 640, cy: 540 };

test('projectTagGazeMetrics computes accurate angle, distance, and frustum bounds in GL->CV camera coordinates', () => {
  // Dead ahead at 1.0 m (-Z in GL)
  const center = projectTagGazeMetrics([0, 0, -1.0], I, K, K.width, K.height);
  assert.equal(center.inFrustum, true);
  assert.ok(center.angleDeg < 1e-6);
  assert.ok(Math.abs(center.distM - 1.0) < 1e-6);
  assert.ok(Math.abs(center.u - 640) < 1e-6);
  assert.ok(Math.abs(center.v - 540) < 1e-6);

  // 18 deg, 24 deg, 30 deg off-axis horizontally
  for (const deg of [18, 24, 30]) {
    const x = Math.tan((deg * Math.PI) / 180) * 1.0;
    const m = projectTagGazeMetrics([x, 0, -1.0], I, K, K.width, K.height);
    assert.ok(Math.abs(m.angleDeg - deg) < 1e-4, `expected ${deg} deg, got ${m.angleDeg}`);
    assert.equal(m.inFrustum, true);
  }

  // Behind camera (+Z in GL)
  const behind = projectTagGazeMetrics([0, 0, 0.8], I, K, K.width, K.height);
  assert.equal(behind.inFrustum, false);
  assert.equal(behind.angleDeg, Infinity);
});

test('createFocusScheduler selects single-focus for 1 center tag and disables speedFirst when resting locked', () => {
  const sched = createFocusScheduler();

  const movingPlan = sched.plan({
    candidates: [
      { tagId: 300, pos: [0.05, 0, -0.8], halfPx: 110, moving: true, lastOpticalAtMs: 1000, speedMps: 0.4, apparentEdgePx: 45 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1000,
  });
  assert.equal(movingPlan.mode, 'single-focus');
  assert.equal(movingPlan.speedFirst, true);
  assert.equal(movingPlan.isBackgroundSweep, false);
  assert.deepEqual(movingPlan.tagIds, [300]);
  assert.equal(movingPlan.rois.length, 1);

  // When single center tag settles into locked, it stays in single-focus with speedFirst: false.
  const lockedPlan = sched.plan({
    candidates: [
      { tagId: 300, pos: [0.05, 0, -0.8], halfPx: 110, locked: true, stillDurationMs: 3000, lastOpticalAtMs: 1150, apparentEdgePx: 45 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1150,
  });
  assert.equal(lockedPlan.mode, 'single-focus');
  assert.equal(lockedPlan.speedFirst, false);
  assert.equal(lockedPlan.isBackgroundSweep, false);
  assert.deepEqual(lockedPlan.tagIds, [300]);
  assert.equal(lockedPlan.rois.length, 1);
});

test('multi-focus holds lost-in-cone tag across 500 ms dropout with expanded ROI, then demotes after 1.8 s', () => {
  const sched = createFocusScheduler();

  // Track 2 tags inside the gaze cone (~11 deg off-axis, non-overlapping ROIs) for 3 seconds so both become stable foreground slots.
  const twoMoving = (t, last301 = t) => [
    { tagId: 300, pos: [-0.15, 0, -0.8], halfPx: 95, moving: true, lastOpticalAtMs: t, speedMps: 0.2, apparentEdgePx: 40 },
    { tagId: 301, pos: [0.17, 0, -0.8], halfPx: 95, moving: true, lastOpticalAtMs: last301, speedMps: 0.2, apparentEdgePx: 40 },
  ];

  for (let t = 1000; t <= 4000; t += 100) {
    const p = sched.plan({
      candidates: twoMoving(t),
      worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
      K, width: K.width, height: K.height, nowMs: t,
    });
    assert.equal(p.mode, 'multi-focus');
    if (!p.isBackgroundSweep) {
      sched.reportObservation({
        seenIds: [300, 301], focusTagIds: p.tagIds, isFocusFrame: true, isBackgroundSweep: false, focusHit: true, nowMs: t,
      });
    } else {
      sched.reportObservation({
        seenIds: [300, 301], focusTagIds: p.tagIds, isFocusFrame: false, isBackgroundSweep: true, focusHit: false, nowMs: t,
      });
    }
  }

  // At t=4100..4450 (450 ms dropout), #301 is optically lost while still projected in the cone.
  // First miss expands #301 ROI by 1.35x and second miss by 1.75x while staying in multi-focus!
  const pMiss1 = sched.plan({
    candidates: twoMoving(4100, 4000),
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 4100,
  });
  assert.equal(pMiss1.mode, 'multi-focus');
  assert.equal(pMiss1.isBackgroundSweep, false);
  assert.equal(pMiss1.rois.length, 2);
  assert.deepEqual(pMiss1.tagIds.slice().sort(), [300, 301]);
  sched.reportObservation({ seenIds: [300], focusTagIds: pMiss1.tagIds, isFocusFrame: true, focusHit: true, nowMs: 4100 });

  const pMiss2 = sched.plan({
    candidates: twoMoving(4450, 4000),
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 4450,
  });
  assert.equal(pMiss2.mode, 'multi-focus');
  assert.equal(pMiss2.isBackgroundSweep, false);
  assert.equal(pMiss2.rois.length, 2);
  assert.ok(pMiss2.searchingTagIds.includes(301));
  const wRoi1 = pMiss1.rois[1][2] - pMiss1.rois[1][0];
  const wRoi2 = pMiss2.rois[1][2] - pMiss2.rois[1][0];
  assert.ok(wRoi2 > wRoi1, `expected expanded ROI on missStreak >= 1 (${wRoi2} > ${wRoi1})`);

  // After > lostInConeHoldMs (1800 ms), #301 finally demotes and #300 transitions cleanly to single-focus.
  const pExpired = sched.plan({
    candidates: twoMoving(5900, 4000),
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 5900,
  });
  assert.equal(pExpired.mode, 'single-focus');
  assert.deepEqual(pExpired.tagIds, [300]);
});

test('cone hysteresis keeps tag at 23 deg, demotes after >500 ms outside 26 deg, and yields resting 2nd-class tags to moving foreground', () => {
  const sched = createFocusScheduler();

  // Promote #300 at 0 deg and #301 at 16 deg (inside 20 deg cone)
  const x16 = Math.tan((16 * Math.PI) / 180);
  const p0 = sched.plan({
    candidates: [
      { tagId: 300, pos: [0, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1000, apparentEdgePx: 40 },
      { tagId: 301, pos: [x16, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1000, apparentEdgePx: 40 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1000,
  });
  assert.equal(p0.mode, 'multi-focus');

  // Rotate head so #301 sits at 23 deg (between enter 20 deg and exit 26 deg) -> hysteresis keeps multi-focus!
  const x23 = Math.tan((23 * Math.PI) / 180);
  const p23 = sched.plan({
    candidates: [
      { tagId: 300, pos: [0, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1200, apparentEdgePx: 40 },
      { tagId: 301, pos: [x23, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1200, apparentEdgePx: 40 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1200,
  });
  assert.equal(p23.mode, 'multi-focus');

  // Move #301 outside 26 deg (to 30 deg): still kept for < outOfConeHoldMs (500 ms), then demoted at >= 500 ms.
  const x30 = Math.tan((30 * Math.PI) / 180);
  const p30Start = sched.plan({
    candidates: [
      { tagId: 300, pos: [0, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1400, apparentEdgePx: 40 },
      { tagId: 301, pos: [x30, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1400, apparentEdgePx: 40 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1400,
  });
  assert.equal(p30Start.mode, 'multi-focus');

  const p30Demote = sched.plan({
    candidates: [
      { tagId: 300, pos: [0, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1950, apparentEdgePx: 40 },
      { tagId: 301, pos: [x30, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 1950, apparentEdgePx: 40 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 1950,
  });
  assert.equal(p30Demote.mode, 'single-focus');
  assert.deepEqual(p30Demote.tagIds, [300]);

  // Resting 2nd-class background tag (#302, locked >= 2500 ms) yields immediately when #300 moves in center!
  const pYield = sched.plan({
    candidates: [
      { tagId: 300, pos: [0, 0, -1.0], halfPx: 100, moving: true, lastOpticalAtMs: 2100, apparentEdgePx: 40 },
      { tagId: 302, pos: [0.08, 0.05, -1.0], halfPx: 100, locked: true, stillDurationMs: FOCUS_SCHEDULER_DEFAULTS.restingDemoteMs + 500, lastOpticalAtMs: 2100, apparentEdgePx: 40 },
    ],
    worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I,
    K, width: K.width, height: K.height, nowMs: 2100,
  });
  assert.equal(pYield.mode, 'single-focus');
  assert.deepEqual(pYield.tagIds, [300]);
});

test('adaptive 1-3 Hz background sweeps use decimate=3 while moving, decimate=2 when locked, and wide fallback for >=4 active tags', () => {
  const sched = createFocusScheduler();

  // Rapidly moving single tag: sweep interval is ~950 ms and uses sweepDecimate=3
  const moving = [{ tagId: 300, pos: [0, 0, -0.8], halfPx: 100, recording: true, moving: true, speedMps: 0.65, lastOpticalAtMs: 1000, apparentEdgePx: 45 }];
  const m0 = sched.plan({ candidates: moving, worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I, K, width: K.width, height: K.height, nowMs: 1000, recentDetectMs: 12 });
  assert.equal(m0.isBackgroundSweep, false);

  const mMid = sched.plan({ candidates: [{ ...moving[0], lastOpticalAtMs: 1600 }], worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I, K, width: K.width, height: K.height, nowMs: 1600, recentDetectMs: 12 });
  assert.equal(mMid.isBackgroundSweep, false, 'fast tag should not sweep at 600 ms');

  const mSweep = sched.plan({ candidates: [{ ...moving[0], lastOpticalAtMs: 1980 }], worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I, K, width: K.width, height: K.height, nowMs: 1980, recentDetectMs: 12 });
  assert.equal(mSweep.isBackgroundSweep, true);
  assert.equal(mSweep.sweepDecimate, 3);
  assert.equal(mSweep.rois, undefined);
  sched.reportObservation({ seenIds: [300], focusTagIds: mSweep.tagIds, isFocusFrame: false, isBackgroundSweep: true, focusHit: false, nowMs: 1980 });

  // Calm locked tag: sweeps faster (~360 ms = ~2.8 Hz) at decimate=2
  const locked = [{ tagId: 300, pos: [0, 0, -0.8], halfPx: 100, locked: true, stillDurationMs: 3000, lastOpticalAtMs: 2360, apparentEdgePx: 45 }];
  const lSweep = sched.plan({ candidates: locked, worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I, K, width: K.width, height: K.height, nowMs: 2360, recentDetectMs: 10 });
  assert.equal(lSweep.isBackgroundSweep, true);
  assert.equal(lSweep.sweepDecimate, 2);

  // >= 4 simultaneously active tags in cone -> cleanly falls back to wide mode so tag #4 never starves
  const fourActive = [300, 301, 302, 303].map((tagId, idx) => ({
    tagId,
    pos: [(idx - 1.5) * 0.06, 0, -1.0],
    halfPx: 100,
    moving: true,
    lastOpticalAtMs: 2500,
    apparentEdgePx: 40,
  }));
  const crowd = sched.plan({ candidates: fourActive, worldFromAnchor: I, worldFromViewer: I, viewerFromCameraGL: I, K, width: K.width, height: K.height, nowMs: 2500 });
  assert.equal(crowd.mode, 'wide');
  assert.equal(crowd.skipReason, 'crowd-wide');
  assert.equal(crowd.rois, undefined);
});
