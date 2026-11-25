import test from 'node:test';
import assert from 'node:assert/strict';
import {
  FIST_GESTURE_DEFAULTS,
  classifyFistPose,
  createFistToggleController,
} from '../src/hand-gesture.mjs';

function deg(d) {
  return (d * Math.PI) / 180;
}

function makeHandPose({
  handScale = 0.085,
  mcpDeg = { index: 0, middle: 0, ring: 0, pinky: 0 },
  curlDeg = { index: 0, middle: 0, ring: 0, pinky: 0 },
  thumbOffset = [-0.55, 0.72, 0.15],
} = {}) {
  const s = handScale;
  const wrist = [0, 0, 0];
  const baseProx = {
    index: [-0.26 * s, 0.96 * s, 0],
    middle: [0.0 * s, 1.0 * s, 0],
    ring: [0.24 * s, 0.96 * s, 0],
    pinky: [0.46 * s, 0.86 * s, 0],
  };

  const joints = {
    wrist,
    thumbTip: [thumbOffset[0] * s, thumbOffset[1] * s, thumbOffset[2] * s],
  };

  for (const name of ['index', 'middle', 'ring', 'pinky']) {
    const prox = baseProx[name];
    const l1 = (name === 'pinky' ? 0.36 : 0.45) * s;
    const l2 = (name === 'pinky' ? 0.34 : 0.48) * s;
    const a1 = deg(mcpDeg[name] ?? 0);
    const a2 = a1 + deg(curlDeg[name] ?? 0);
    const inter = [
      prox[0],
      prox[1] + l1 * Math.cos(a1),
      prox[2] + l1 * Math.sin(a1),
    ];
    const tip = [
      inter[0],
      inter[1] + l2 * Math.cos(a2),
      inter[2] + l2 * Math.sin(a2),
    ];
    joints[name] = { prox, inter, tip };
  }

  return joints;
}

function tightFistPose(opts = {}) {
  return makeHandPose({
    mcpDeg: { index: 85, middle: 85, ring: 85, pinky: 82 },
    curlDeg: { index: 125, middle: 125, ring: 125, pinky: 120 },
    thumbOffset: [0.05, 1.0, 0.35],
    ...opts,
  });
}

function relaxedGripPose(opts = {}) {
  // Within hold hysteresis (isHeldFist: true) but not tight enough for strict enter (isFist: false)
  return makeHandPose({
    mcpDeg: { index: 65, middle: 65, ring: 65, pinky: 65 },
    curlDeg: { index: 84, middle: 84, ring: 84, pinky: 84 },
    thumbOffset: [0.05, 1.0, 0.35],
    ...opts,
  });
}

function openHandPose(opts = {}) {
  return makeHandPose({
    mcpDeg: { index: 5, middle: 5, ring: 5, pinky: 5 },
    curlDeg: { index: 8, middle: 8, ring: 8, pinky: 10 },
    thumbOffset: [-0.65, 0.78, 0.12],
    ...opts,
  });
}

test('classifyFistPose detects tight fists regardless of thumb tuck style and hand size', () => {
  const thumbAcross = classifyFistPose(tightFistPose());
  assert.equal(thumbAcross.valid, true);
  assert.equal(thumbAcross.curledCount, 4);
  assert.equal(thumbAcross.thumbOk, true);
  assert.equal(thumbAcross.isFist, true);
  assert.equal(thumbAcross.isHeldFist, true);
  assert.equal(thumbAcross.isOpen, false);

  const thumbAlongIndex = classifyFistPose(
    tightFistPose({ thumbOffset: [-0.36, 1.02, 0.22] }),
  );
  assert.equal(thumbAlongIndex.isFist, true);

  // Scale invariance across small (60 mm) and large (105 mm) hands
  assert.equal(classifyFistPose(tightFistPose({ handScale: 0.060 })).isFist, true);
  assert.equal(classifyFistPose(tightFistPose({ handScale: 0.105 })).isFist, true);
  assert.equal(classifyFistPose(openHandPose({ handScale: 0.060 })).isOpen, true);
  assert.equal(classifyFistPose(openHandPose({ handScale: 0.105 })).isOpen, true);
});

test('classifyFistPose rejects open, pointing, pinching, resting, thumbs-up, and invalid poses', () => {
  const open = classifyFistPose(openHandPose());
  assert.equal(open.isFist, false);
  assert.equal(open.isOpen, true);

  // Pointing with index finger extended while middle/ring/pinky are curled (classic false-trigger killer)
  const pointing = classifyFistPose(
    makeHandPose({
      mcpDeg: { index: 5, middle: 85, ring: 85, pinky: 82 },
      curlDeg: { index: 8, middle: 125, ring: 125, pinky: 120 },
      thumbOffset: [-0.15, 0.95, 0.32],
    }),
  );
  assert.equal(pointing.curledCount, 3);
  assert.equal(pointing.isFist, false);

  // Index + thumb pinch
  const pinch = classifyFistPose(
    makeHandPose({
      mcpDeg: { index: 30, middle: 75, ring: 78, pinky: 76 },
      curlDeg: { index: 35, middle: 105, ring: 110, pinky: 108 },
      thumbOffset: [-0.26, 1.55, 0.48],
    }),
  );
  assert.equal(pinch.isFist, false);

  // Semi-curled resting hand
  const resting = classifyFistPose(
    makeHandPose({
      mcpDeg: { index: 35, middle: 38, ring: 40, pinky: 42 },
      curlDeg: { index: 45, middle: 48, ring: 50, pinky: 52 },
      thumbOffset: [-0.45, 0.95, 0.25],
    }),
  );
  assert.equal(resting.isFist, false);

  // Thumbs-up (4 fingers curled, thumb sticking far out)
  const thumbsUp = classifyFistPose(
    tightFistPose({ thumbOffset: [-1.15, 1.45, 0.65] }),
  );
  assert.equal(thumbsUp.curledCount, 4);
  assert.equal(thumbsUp.thumbOk, false);
  assert.equal(thumbsUp.isFist, false);

  // Invalid / missing joints or out-of-range scale
  assert.equal(classifyFistPose(null).valid, false);
  assert.equal(classifyFistPose({ wrist: [0, 0, 0] }).valid, false);
  assert.equal(classifyFistPose(tightFistPose({ handScale: 0.02 })).valid, false);
  assert.equal(classifyFistPose(tightFistPose({ handScale: 0.20 })).valid, false);
});

test('createFistToggleController enforces 550 ms hold, deadzone, hysteresis, latch, and open-hand reset', () => {
  const ctrl = createFistToggleController();
  const fist = classifyFistPose(tightFistPose());
  const relaxed = classifyFistPose(relaxedGripPose());
  const midGap = classifyFistPose(
    makeHandPose({
      mcpDeg: { index: 45, middle: 45, ring: 45, pinky: 45 },
      curlDeg: { index: 65, middle: 65, ring: 65, pinky: 65 },
      thumbOffset: [0.05, 1.0, 0.35],
    }),
  );
  const open = classifyFistPose(openHandPose());

  // Inside deadzone (< 120 ms): pending, visual hidden, never triggers
  let res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 1000 });
  assert.equal(res.phase, 'pending');
  assert.equal(res.showVisual, false);
  assert.equal(res.triggered, false);

  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 1119 });
  assert.equal(res.phase, 'pending');
  assert.equal(res.showVisual, false);

  // Past deadzone (120..549 ms): charging smoothly
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 1250 });
  assert.equal(res.phase, 'charging');
  assert.equal(res.showVisual, true);
  assert.ok(res.progress > 0.2 && res.progress < 0.4);

  // Relaxed hysteresis pose keeps charging; brief tracking gap (< 90 ms) survives
  res = ctrl.update({ pose: relaxed, inView: true, speedMps: 0.05, nowMs: 1320 });
  assert.equal(res.phase, 'charging');
  assert.equal(relaxed.isFist, false);
  assert.equal(relaxed.isHeldFist, true);

  res = ctrl.update({ pose: midGap, inView: true, speedMps: 0.05, nowMs: 1360 });
  assert.equal(res.phase, 'charging');
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 1410 });
  assert.equal(res.phase, 'charging');

  // Reaches 550 ms -> fires once and latches
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 1550 });
  assert.equal(res.phase, 'triggered');
  assert.equal(res.triggered, true);
  assert.equal(res.latched, true);
  assert.equal(res.progress, 1);

  // Holding fist closed past cooldown does NOT re-trigger while latched
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 2600 });
  assert.equal(res.phase, 'latched');
  assert.equal(res.triggered, false);
  assert.equal(res.showVisual, false);

  // Opening hand for >= 100 ms clears latch; after cooldown it can toggle again
  ctrl.update({ pose: open, inView: true, speedMps: 0.05, nowMs: 2650 });
  res = ctrl.update({ pose: open, inView: true, speedMps: 0.05, nowMs: 2760 });
  assert.equal(res.latched, false);

  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 2800 });
  assert.equal(res.phase, 'pending');
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.05, nowMs: 2800 + FIST_GESTURE_DEFAULTS.holdMs });
  assert.equal(res.triggered, true);
});

test('createFistToggleController aborts on suppression, fast hand speed, or leaving view, and resets on loss', () => {
  const ctrl = createFistToggleController();
  const fist = classifyFistPose(tightFistPose());

  // Suppressed or fast motion prevents charging
  let res = ctrl.update({ pose: fist, inView: true, speedMps: 0.30, nowMs: 100 });
  assert.equal(res.phase, 'idle');
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.02, suppressed: true, nowMs: 200 });
  assert.equal(res.phase, 'idle');

  // Start charging, then abort cleanly if suppressed mid-hold
  ctrl.update({ pose: fist, inView: true, speedMps: 0.02, nowMs: 300 });
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.02, nowMs: 500 });
  assert.equal(res.phase, 'charging');
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.02, suppressed: true, nowMs: 550 });
  assert.equal(res.phase, 'idle');

  // Leaving view aborts active charge and clears latch after lostResetMs
  ctrl.update({ pose: fist, inView: true, speedMps: 0.02, nowMs: 1000 });
  res = ctrl.update({ pose: fist, inView: true, speedMps: 0.02, nowMs: 1000 + FIST_GESTURE_DEFAULTS.holdMs });
  assert.equal(res.triggered, true);

  res = ctrl.update({ pose: null, inView: false, nowMs: 1600 });
  assert.equal(res.latched, true);
  res = ctrl.update({ pose: null, inView: false, nowMs: 1600 + FIST_GESTURE_DEFAULTS.lostResetMs });
  assert.equal(res.latched, false);
});
