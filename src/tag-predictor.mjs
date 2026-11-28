// Damped constant-velocity 6DOF optical lead predictor for uncoupled moving AprilTags.
import { isRigidPose } from './anchor.mjs';
import { inverse, product, rigidVector, vectorRigid } from './runtime.mjs';

export const TAG_PREDICTOR_DEFAULTS = Object.freeze({
  maxLeadSec: 0.120,
  leadGain: 0.90,
  maxLinearMps: 1.8,
  maxAngularRadPerSec: 8.0,
  velocityAlpha: 0.72,
  decayTauSec: 0.085,
  minDtSec: 0.012,
  maxDtSec: 0.35,
  // Tracking-confidence fade (O(1) scalar math per tag per frame, no allocations).
  fadeTagRadiusM: 0.04,     // lever arm that turns angular speed into a corner displacement
  fadeSigmaSec: 0.025,      // unmodelled timing jitter that prediction cannot remove
  fadeErrLowM: 0.012,       // estimated offset below this: fully opaque
  fadeErrHighM: 0.050,      // estimated offset above this: minimum opacity
  fadeMinAlpha: 0.25,
  fadeAttackTauSec: 0.05,   // fade out quickly...
  fadeReleaseTauSec: 0.25,  // ...come back slowly (no flicker)
});

function clampVec3Norm(v, maxNorm) {
  const n = Math.hypot(v[0], v[1], v[2]);
  if (!Number.isFinite(n) || n <= maxNorm) return v;
  const s = maxNorm / n;
  return [v[0] * s, v[1] * s, v[2] * s];
}

/**
 * Tracks recent 6DOF pose velocities per tagId at observation timestamps (targetXrTimeMs)
 * and extrapolates a bounded, strictly rigid lead pose to compensate camera pipeline latency.
 */
export function createTagMotionPredictor(options = {}) {
  const cfg = { ...TAG_PREDICTOR_DEFAULTS, ...options };
  const states = new Map();

  function observe(tagId, lFromTag, obsTimeMs) {
    if (!isRigidPose(lFromTag) || !Number.isFinite(obsTimeMs)) return null;
    const prev = states.get(tagId);
    if (!prev || obsTimeMs <= prev.obsTimeMs) {
      states.set(tagId, {
        pose: lFromTag.slice(),
        obsTimeMs,
        velL: [0, 0, 0],
        angLocal: [0, 0, 0],
        speedMps: 0,
        angDegPerSec: 0,
        fade: prev?.fade ?? 1,
        fadeTimeMs: prev?.fadeTimeMs,
      });
      return { speedMps: 0, angDegPerSec: 0 };
    }

    const dtSec = (obsTimeMs - prev.obsTimeMs) / 1000;
    if (dtSec < cfg.minDtSec || dtSec > cfg.maxDtSec) {
      states.set(tagId, {
        pose: lFromTag.slice(),
        obsTimeMs,
        velL: [0, 0, 0],
        angLocal: [0, 0, 0],
        speedMps: 0,
        angDegPerSec: 0,
        fade: prev?.fade ?? 1,
        fadeTimeMs: prev?.fadeTimeMs,
      });
      return { speedMps: 0, angDegPerSec: 0 };
    }

    const rawVel = clampVec3Norm([
      (lFromTag[12] - prev.pose[12]) / dtSec,
      (lFromTag[13] - prev.pose[13]) / dtSec,
      (lFromTag[14] - prev.pose[14]) / dtSec,
    ], cfg.maxLinearMps);

    const dLocal = rigidVector(product(inverse(prev.pose), lFromTag));
    const rawAng = clampVec3Norm([
      dLocal[0] / dtSec,
      dLocal[1] / dtSec,
      dLocal[2] / dtSec,
    ], cfg.maxAngularRadPerSec);

    const reversal = (
      rawVel[0] * prev.velL[0] +
      rawVel[1] * prev.velL[1] +
      rawVel[2] * prev.velL[2]
    ) < 0;
    const a = reversal ? 1.0 : cfg.velocityAlpha;
    const velL = [
      prev.velL[0] * (1 - a) + rawVel[0] * a,
      prev.velL[1] * (1 - a) + rawVel[1] * a,
      prev.velL[2] * (1 - a) + rawVel[2] * a,
    ];
    const angLocal = [
      prev.angLocal[0] * (1 - a) + rawAng[0] * a,
      prev.angLocal[1] * (1 - a) + rawAng[1] * a,
      prev.angLocal[2] * (1 - a) + rawAng[2] * a,
    ];
    const speedMps = Math.hypot(velL[0], velL[1], velL[2]);
    const angDegPerSec = (Math.hypot(angLocal[0], angLocal[1], angLocal[2]) * 180) / Math.PI;

    states.set(tagId, {
      pose: lFromTag.slice(),
      obsTimeMs,
      velL,
      angLocal,
      speedMps,
      angDegPerSec,
      fade: prev.fade ?? 1,
      fadeTimeMs: prev.fadeTimeMs,
    });
    return { speedMps, angDegPerSec };
  }

  function predict(tagId, nowTimeMs) {
    const st = states.get(tagId);
    if (!st || !Number.isFinite(nowTimeMs)) return null;
    const ageSec = Math.max(0, (nowTimeMs - st.obsTimeMs) / 1000);
    if (ageSec <= 0 || (st.speedMps < 0.015 && st.angDegPerSec < 4)) {
      return st.pose.slice();
    }
    const decay = Math.exp(-Math.max(0, ageSec - 0.12) / cfg.decayTauSec);
    const leadSec = Math.min(cfg.maxLeadSec, ageSec * cfg.leadGain) * decay;
    if (leadSec <= 1e-4) return st.pose.slice();

    const dRot = vectorRigid([
      st.angLocal[0] * leadSec,
      st.angLocal[1] * leadSec,
      st.angLocal[2] * leadSec,
      0, 0, 0,
    ]);
    const out = product(st.pose, dRot);
    out[12] = st.pose[12] + st.velL[0] * leadSec;
    out[13] = st.pose[13] + st.velL[1] * leadSec;
    out[14] = st.pose[14] + st.velL[2] * leadSec;
    return out;
  }

  /**
   * Cheap estimate of the remaining draw offset (metres) of the predicted pose:
   * speed × (uncovered latency + jitter). Pure scalar math, no allocation.
   */
  function offsetEstimate(tagId, nowTimeMs) {
    const st = states.get(tagId);
    if (!st || !Number.isFinite(nowTimeMs)) return 0;
    const ageSec = Math.max(0, (nowTimeMs - st.obsTimeMs) / 1000);
    const decay = Math.exp(-Math.max(0, ageSec - 0.12) / cfg.decayTauSec);
    const covered = Math.min(cfg.maxLeadSec, ageSec * cfg.leadGain) * decay;
    const uncovered = Math.max(0, ageSec - covered) + cfg.fadeSigmaSec;
    const angRad = (st.angDegPerSec * Math.PI) / 180;
    return (st.speedMps + angRad * cfg.fadeTagRadiusM) * uncovered;
  }

  /**
   * Smoothed opacity 1..fadeMinAlpha: opaque while the tag is slow/fresh, fading when the estimated
   * offset grows (fast motion or stale optical data) = "about to leave the trackable range".
   */
  function trackingFade(tagId, nowTimeMs) {
    const st = states.get(tagId);
    if (!st || !Number.isFinite(nowTimeMs)) return 1;
    const err = offsetEstimate(tagId, nowTimeMs);
    let t = (err - cfg.fadeErrLowM) / (cfg.fadeErrHighM - cfg.fadeErrLowM);
    t = t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t);
    const target = 1 - t * (1 - cfg.fadeMinAlpha);
    const prevFade = st.fade ?? 1;
    const prevT = st.fadeTimeMs;
    const dt = Number.isFinite(prevT) ? Math.max(0, Math.min(0.1, (nowTimeMs - prevT) / 1000)) : 0;
    const tau = target < prevFade ? cfg.fadeAttackTauSec : cfg.fadeReleaseTauSec;
    const k = dt > 0 ? 1 - Math.exp(-dt / tau) : 0;
    st.fade = prevFade + (target - prevFade) * k;
    st.fadeTimeMs = nowTimeMs;
    return st.fade;
  }

  function seed(tagId, lFromTag, obsTimeMs, { velL = null, angLocal = null } = {}) {
    if (!isRigidPose(lFromTag) || !Number.isFinite(obsTimeMs)) return null;
    const vRaw = Array.isArray(velL) && velL.length >= 3 && velL.every(Number.isFinite)
      ? [velL[0], velL[1], velL[2]]
      : [0, 0, 0];
    const wRaw = Array.isArray(angLocal) && angLocal.length >= 3 && angLocal.every(Number.isFinite)
      ? [angLocal[0], angLocal[1], angLocal[2]]
      : [0, 0, 0];
    const clampedVel = clampVec3Norm(vRaw, cfg.maxLinearMps);
    const clampedAng = clampVec3Norm(wRaw, cfg.maxAngularRadPerSec);
    const speedMps = Math.hypot(clampedVel[0], clampedVel[1], clampedVel[2]);
    const angDegPerSec = (Math.hypot(clampedAng[0], clampedAng[1], clampedAng[2]) * 180) / Math.PI;
    states.set(tagId, {
      pose: lFromTag.slice(),
      obsTimeMs,
      velL: clampedVel,
      angLocal: clampedAng,
      speedMps,
      angDegPerSec,
    });
    return { speedMps, angDegPerSec };
  }

  function speedOf(tagId) {
    const st = states.get(tagId);
    if (!st) return null;
    return { speedMps: st.speedMps, angDegPerSec: st.angDegPerSec };
  }

  function reset(tagId = null) {
    if (tagId != null) states.delete(tagId);
    else states.clear();
  }

  return { observe, seed, predict, speedOf, offsetEstimate, trackingFade, reset };
}
