/**
 * Scale-invariant WebXR bare-hand fist posture classifier and latched hold
 * controller for false-trigger-free mode switching (SCAN <-> MEASURE).
 */

export const FIST_GESTURE_DEFAULTS = Object.freeze({
  holdMs: 550,
  deadzoneMs: 120,
  breakGraceMs: 90,
  releaseHoldMs: 100,
  lostResetMs: 350,
  cooldownMs: 800,
  maxSpeedMps: 0.22,
  minHandScaleM: 0.045,
  maxHandScaleM: 0.14,
  enterCurlCos: 0.05,
  enterTipPalmRatio: 0.85,
  enterTipWristRatio: 0.96,
  enterPinkyCurlCos: 0.15,
  enterPinkyTipPalmRatio: 0.90,
  enterPinkyTipWristRatio: 1.00,
  holdCurlCos: 0.22,
  holdTipPalmRatio: 0.95,
  holdTipWristRatio: 1.04,
  holdPinkyCurlCos: 0.30,
  holdPinkyTipPalmRatio: 1.00,
  holdPinkyTipWristRatio: 1.08,
  openCurlCos: 0.45,
  openTipWristRatio: 1.12,
  openTipPalmRatio: 1.08,
  thumbMaxRatio: 0.95,
});

const FINGER_NAMES = ['index', 'middle', 'ring', 'pinky'];

const isVec3 = (v) =>
  Array.isArray(v) &&
  v.length >= 3 &&
  Number.isFinite(v[0]) &&
  Number.isFinite(v[1]) &&
  Number.isFinite(v[2]);

const dist3 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

const lerp3 = (a, b, t) => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
];

function unitVec3(a, b) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const dz = b[2] - a[2];
  const len = Math.hypot(dx, dy, dz);
  if (len <= 1e-4) return null;
  return [dx / len, dy / len, dz / len];
}

function invalidPoseResult() {
  return {
    valid: false,
    isFist: false,
    isHeldFist: false,
    isOpen: false,
    curledCount: 0,
    openCount: 0,
    thumbOk: false,
    handScale: 0,
    palmCenter: null,
    fistCenter: null,
    fingers: null,
  };
}

/**
 * Classify whether a WebXR hand skeleton forms a closed fist (all four fingers
 * curled into the palm with the thumb tucked in, not sticking out).
 */
export function classifyFistPose(joints, opts = {}) {
  if (!joints || !isVec3(joints.wrist)) return invalidPoseResult();

  for (const name of FINGER_NAMES) {
    const f = joints[name];
    if (!f || !isVec3(f.prox) || !isVec3(f.inter) || !isVec3(f.tip)) {
      return invalidPoseResult();
    }
  }

  const cfg = { ...FIST_GESTURE_DEFAULTS, ...opts };
  const wrist = joints.wrist;
  const handScale = dist3(wrist, joints.middle.prox);
  if (!Number.isFinite(handScale) || handScale < cfg.minHandScaleM || handScale > cfg.maxHandScaleM) {
    return invalidPoseResult();
  }

  const palmCenter = lerp3(wrist, joints.middle.prox, 0.58);
  const fistCenter = lerp3(wrist, joints.middle.prox, 0.65);

  const fingers = {};
  let curledCount = 0;
  let heldCount = 0;
  let openCount = 0;

  for (const name of FINGER_NAMES) {
    const f = joints[name];
    const u1 = unitVec3(f.prox, f.inter);
    const u2 = unitVec3(f.inter, f.tip);
    const curlCos = u1 && u2 ? u1[0] * u2[0] + u1[1] * u2[1] + u1[2] * u2[2] : 1;
    const tipToPalmRatio = dist3(f.tip, palmCenter) / handScale;
    const interToWrist = Math.max(1e-4, dist3(f.inter, wrist));
    const tipToWristRatio = dist3(f.tip, wrist) / interToWrist;

    const isPinky = name === 'pinky';
    const cEnter = isPinky ? cfg.enterPinkyCurlCos : cfg.enterCurlCos;
    const pEnter = isPinky ? cfg.enterPinkyTipPalmRatio : cfg.enterTipPalmRatio;
    const wEnter = isPinky ? cfg.enterPinkyTipWristRatio : cfg.enterTipWristRatio;
    const cHold = isPinky ? cfg.holdPinkyCurlCos : cfg.holdCurlCos;
    const pHold = isPinky ? cfg.holdPinkyTipPalmRatio : cfg.holdTipPalmRatio;
    const wHold = isPinky ? cfg.holdPinkyTipWristRatio : cfg.holdTipWristRatio;

    const curledStrict = curlCos <= cEnter && tipToPalmRatio <= pEnter && tipToWristRatio <= wEnter;
    const curledHold = curlCos <= cHold && tipToPalmRatio <= pHold && tipToWristRatio <= wHold;
    const extendedClear =
      curlCos >= cfg.openCurlCos ||
      tipToWristRatio >= cfg.openTipWristRatio ||
      tipToPalmRatio >= cfg.openTipPalmRatio;

    if (curledStrict) curledCount++;
    if (curledHold) heldCount++;
    if (extendedClear) openCount++;

    fingers[name] = {
      curlCos,
      tipToPalmRatio,
      tipToWristRatio,
      curledStrict,
      curledHold,
      extendedClear,
    };
  }

  let thumbOk = true;
  if (isVec3(joints.thumbTip)) {
    const minThumbDist = Math.min(
      dist3(joints.thumbTip, joints.index.prox),
      dist3(joints.thumbTip, joints.index.inter),
      dist3(joints.thumbTip, joints.middle.inter),
    );
    thumbOk = minThumbDist / handScale <= cfg.thumbMaxRatio;
  }

  const isFist = curledCount === 4 && thumbOk;
  const isHeldFist = heldCount === 4 && thumbOk;
  const isOpen = openCount >= 2;

  return {
    valid: true,
    isFist,
    isHeldFist,
    isOpen,
    curledCount,
    openCount,
    thumbOk,
    handScale,
    palmCenter,
    fistCenter,
    fingers,
  };
}

/**
 * Stateful hold/latch controller for triggering mode switches from a deliberate
 * closed-fist hold while rejecting accidental hand shapes, fast passes, and repeats.
 */
export function createFistToggleController(opts = {}) {
  const cfg = { ...FIST_GESTURE_DEFAULTS, ...opts };

  let holdingSince = null;
  let breakSince = null;
  let latched = false;
  let openSince = null;
  let lostSince = null;
  let cooldownUntil = -Infinity;

  function idleResult(phase = 'idle') {
    return {
      phase,
      progress: 0,
      showVisual: false,
      triggered: false,
      latched,
    };
  }

  function reset() {
    holdingSince = null;
    breakSince = null;
    latched = false;
    openSince = null;
    lostSince = null;
    cooldownUntil = -Infinity;
  }

  function update({
    pose,
    inView = false,
    speedMps = 0,
    enabled = true,
    suppressed = false,
    nowMs = 0,
  } = {}) {
    if (!pose?.valid || !inView) {
      if (lostSince == null) lostSince = nowMs;
      if (nowMs - lostSince >= cfg.lostResetMs) {
        latched = false;
        openSince = null;
      }
      holdingSince = null;
      breakSince = null;
      return idleResult(latched ? 'latched' : 'idle');
    }

    lostSince = null;

    if (latched) {
      if (pose.isOpen) {
        if (openSince == null) openSince = nowMs;
        if (nowMs - openSince >= cfg.releaseHoldMs) {
          latched = false;
          openSince = null;
        }
      } else {
        openSince = null;
      }
      if (latched) {
        holdingSince = null;
        breakSince = null;
        return idleResult('latched');
      }
    }

    if (!enabled || suppressed || nowMs < cooldownUntil || speedMps > cfg.maxSpeedMps) {
      holdingSince = null;
      breakSince = null;
      return idleResult('idle');
    }

    if (holdingSince == null) {
      if (pose.isFist) {
        holdingSince = nowMs;
        breakSince = null;
      } else {
        return idleResult('idle');
      }
    } else if (pose.isHeldFist) {
      breakSince = null;
    } else if (pose.isOpen) {
      holdingSince = null;
      breakSince = null;
      return idleResult('idle');
    } else {
      if (breakSince == null) breakSince = nowMs;
      if (nowMs - breakSince > cfg.breakGraceMs) {
        holdingSince = null;
        breakSince = null;
        return idleResult('idle');
      }
    }

    const elapsed = nowMs - holdingSince;
    if (elapsed >= cfg.holdMs) {
      latched = true;
      cooldownUntil = nowMs + cfg.cooldownMs;
      holdingSince = null;
      breakSince = null;
      openSince = null;
      return {
        phase: 'triggered',
        progress: 1,
        showVisual: true,
        triggered: true,
        latched: true,
      };
    }

    if (elapsed < cfg.deadzoneMs) {
      return idleResult('pending');
    }

    const progress = Math.max(0, Math.min(1, (elapsed - cfg.deadzoneMs) / Math.max(1, cfg.holdMs - cfg.deadzoneMs)));
    return {
      phase: 'charging',
      progress,
      showVisual: true,
      triggered: false,
      latched: false,
    };
  }

  return { update, reset };
}
