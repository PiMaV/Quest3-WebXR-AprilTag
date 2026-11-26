// Damped constant-velocity 6DOF optical lead predictor for uncoupled moving AprilTags.
import { isRigidPose } from './anchor.mjs';
import { inverse, product, rigidVector, vectorRigid } from './runtime.mjs';

export const TAG_PREDICTOR_DEFAULTS = Object.freeze({
  maxLeadSec: 0.105,
  leadGain: 0.72,
  maxLinearMps: 1.8,
  maxAngularRadPerSec: 8.0,
  velocityAlpha: 0.58,
  decayTauSec: 0.085,
  minDtSec: 0.012,
  maxDtSec: 0.35,
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

  function speedOf(tagId) {
    const st = states.get(tagId);
    if (!st) return null;
    return { speedMps: st.speedMps, angDegPerSec: st.angDegPerSec };
  }

  function reset(tagId = null) {
    if (tagId != null) states.delete(tagId);
    else states.clear();
  }

  return { observe, predict, speedOf, reset };
}
