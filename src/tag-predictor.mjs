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
  // Fade is keyed on *view-relative* speed: lateral motion blurs the image / leaves the view (tracking loss),
  // motion along the view ray (z) barely changes pixels and is tolerated much better.
  fadeAxialWeight: 0.30,    // weight of speed along the view ray vs. lateral speed
  fadeStartFrac: 0.60,      // fade begins at 60 % of the limit (= 0.6 m/s, start of the red/unstable range)...
  fadeEndFrac: 1.00,        // ...and reaches fadeMinAlpha at 1.0 m/s
  fadeInitialLimitMps: 1.00,// field-measured: stable to ~0.6 m/s, unstable 0.6..1.0 m/s
  fadeLearnLoss: false,     // true = noteLoss() adapts the limit (off: the fixed scale is the spec)
  fadeLimitMinMps: 0.15,
  fadeLimitMaxMps: 1.5,
  fadeMinAlpha: 0.25,       // tag border stays faintly visible while tracking is alive
  fadeStaleStartSec: 0.35,  // well above Quest 3 p95 opticalLagMs (~190 ms) to avoid dropout flicker
  fadeStaleEndSec: 0.90,    // fades smoothly to 0 just before the 950 ms live dropout cutoff
  fadeAttackTauSec: 0.06,
  fadeReleaseTauSec: 0.22,
  fadeTagRadiusM: 0.04,     // offsetEstimate(): lever arm for angular speed
  fadeSigmaSec: 0.025,      // offsetEstimate(): unmodelled timing jitter
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
  let limitMps = cfg.fadeInitialLimitMps;

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
        effPeak: prev?.effPeak ?? 0,
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
        effPeak: prev?.effPeak ?? 0,
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
      effPeak: prev.effPeak ?? 0,
    });
    return { speedMps, angDegPerSec };
  }

  function predict(tagId, nowTimeMs, opts = null) {
    const st = states.get(tagId);
    if (!st || !Number.isFinite(nowTimeMs)) return null;
    if (st.speedMps < 0.015 && st.angDegPerSec < 4) {
      return st.pose.slice();
    }
    const rawAgeSec = (nowTimeMs - st.obsTimeMs) / 1000;
    const minL = opts && Number.isFinite(opts.minLeadSec) ? Math.min(0, opts.minLeadSec) : 0;
    let leadSec;
    if (rawAgeSec >= 0) {
      const decay = Math.exp(-Math.max(0, rawAgeSec - 0.12) / cfg.decayTauSec);
      leadSec = Math.min(cfg.maxLeadSec, rawAgeSec * cfg.leadGain) * decay;
    } else {
      if (minL >= 0) return st.pose.slice();
      leadSec = Math.max(minL, rawAgeSec * cfg.leadGain);
    }
    if (Math.abs(leadSec) <= 1e-4) return st.pose.slice();

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

  function effectiveSpeed(st, viewerPos) {
    const vx = st.velL[0], vy = st.velL[1], vz = st.velL[2];
    const full = st.speedMps;
    if (!viewerPos || viewerPos.length < 3) return full;
    const rx = st.pose[12] - viewerPos[0], ry = st.pose[13] - viewerPos[1], rz = st.pose[14] - viewerPos[2];
    const dist = Math.hypot(rx, ry, rz);
    if (!(dist > 0.05)) return full;
    const ax = (vx * rx + vy * ry + vz * rz) / dist;
    const lat2 = Math.max(0, full * full - ax * ax);
    return Math.hypot(Math.sqrt(lat2), cfg.fadeAxialWeight * ax);
  }

  /**
   * Smoothed opacity 1..fadeMinAlpha: opaque while the tag is slow (view-relative), fading as the speed
   * enters the unstable optical range, and fading to 0 as optical data goes stale.
   * Pass the viewer position (same frame as the tag pose) for the lateral/axial split.
   */
  function trackingFade(tagId, nowTimeMs, viewerPos = null) {
    const st = states.get(tagId);
    if (!st || !Number.isFinite(nowTimeMs)) return 1;
    const prevT = st.fadeTimeMs;
    const dt = Number.isFinite(prevT) ? Math.max(0, Math.min(0.1, (nowTimeMs - prevT) / 1000)) : 0;
    const eff = effectiveSpeed(st, viewerPos);
    st.effPeak = Math.max(eff, (st.effPeak ?? 0) * Math.exp(-dt / 0.3));
    const lo = limitMps * cfg.fadeStartFrac, hi = limitMps * cfg.fadeEndFrac;
    let t = (eff - lo) / Math.max(1e-6, hi - lo);
    t = t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t);
    let target = 1 - t * (1 - cfg.fadeMinAlpha);
    const ageSec = Math.max(0, (nowTimeMs - st.obsTimeMs) / 1000);
    if (ageSec > cfg.fadeStaleStartSec) {
      let u = (ageSec - cfg.fadeStaleStartSec) / (cfg.fadeStaleEndSec - cfg.fadeStaleStartSec);
      u = u >= 1 ? 1 : u * u * (3 - 2 * u);
      target = Math.min(target, 1 - u);
    }
    const prevFade = st.fade ?? 1;
    const tau = target < prevFade ? cfg.fadeAttackTauSec : cfg.fadeReleaseTauSec;
    const k = dt > 0 ? 1 - Math.exp(-dt / tau) : 0;
    st.fade = prevFade + (target - prevFade) * k;
    st.fadeTimeMs = nowTimeMs;
    return st.fade;
  }

  /** Call when optical tracking of a moving tag is lost: learns the view-relative speed where that happens. */
  function noteLoss(tagId) {
    const st = states.get(tagId);
    const peak = st?.effPeak ?? 0;
    if (!cfg.fadeLearnLoss || peak < 0.08) return limitMps;
    limitMps = Math.min(cfg.fadeLimitMaxMps, Math.max(cfg.fadeLimitMinMps, limitMps * 0.7 + peak * 1.15 * 0.3));
    return limitMps;
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
      fade: 1,
      fadeTimeMs: undefined,
      effPeak: 0,
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

  return { observe, seed, predict, speedOf, offsetEstimate, trackingFade, noteLoss, limitMps: () => limitMps, reset };
}
