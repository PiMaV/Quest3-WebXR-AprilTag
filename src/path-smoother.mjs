// Zero-dependency path fusion and tail smoothing for 3D recorded AprilTag trails.

/**
 * Distribute the small endpoint disagreement between 90 Hz hand prediction and its
 * confirming optical pose across a committed PendingTrail segment so sequential
 * segments connect without visible sawtooth steps.
 */
export function fuseCommittedTrailSegment(confirmed, opticalPose, {
  blendWeight = 0.45,
  maxStepCorrectionM = 0.035,
} = {}) {
  if (!Array.isArray(confirmed) || confirmed.length === 0) return [];
  const w = Number.isFinite(blendWeight) ? Math.max(0, Math.min(1, blendWeight)) : 0.45;
  if (!opticalPose || opticalPose.length < 16 || w <= 0) {
    return confirmed.map(pt => ({ ...pt, pose: pt.pose.slice() }));
  }

  const endPose = confirmed[confirmed.length - 1]?.pose;
  if (!endPose || endPose.length < 16) {
    return confirmed.map(pt => ({ ...pt, pose: pt.pose.slice() }));
  }

  let dx = opticalPose[12] - endPose[12];
  let dy = opticalPose[13] - endPose[13];
  let dz = opticalPose[14] - endPose[14];
  const errM = Math.hypot(dx, dy, dz);
  if (!Number.isFinite(errM)) {
    return confirmed.map(pt => ({ ...pt, pose: pt.pose.slice() }));
  }
  if (Number.isFinite(maxStepCorrectionM) && maxStepCorrectionM > 0 && errM > maxStepCorrectionM) {
    const scale = maxStepCorrectionM / errM;
    dx *= scale;
    dy *= scale;
    dz *= scale;
  }

  const n = confirmed.length;
  const out = new Array(n);
  for (let i = 0; i < n; i++) {
    const pt = confirmed[i];
    const pose = pt.pose.slice();
    const frac = ((i + 1) / n) * w;
    pose[12] += dx * frac;
    pose[13] += dy * frac;
    pose[14] += dz * frac;
    out[i] = { ...pt, pose };
  }
  return out;
}

export const TRAIL_GAP_DEFAULTS = Object.freeze({
  minGapMs: 550,        // optical absence above hand-coupling opticalStaleTimeoutMs (520 ms)
  minGapDistM: 0.030,   // >= 30 mm shift across absence/stop -> real unrecorded relocation
  hardJumpM: 0.160,     // >= 160 mm unbridged jump across >= 320 ms optical absence
  hardJumpMinDtMs: 320,
});

function hasFinitePosePos(pose) {
  return Boolean(
    pose &&
    pose.length >= 16 &&
    Number.isFinite(pose[12]) &&
    Number.isFinite(pose[13]) &&
    Number.isFinite(pose[14]),
  );
}

/**
 * Decide whether transitioning from the last committed trail pose to nextPose
 * crosses an unrecorded occlusion / pause gap and should start a new segment.
 */
export function shouldStartNewTrailSegment({
  prevPose,
  nextPose,
  dtMs = null,
  explicitBreak = false,
  options = null,
} = {}) {
  if (!hasFinitePosePos(prevPose) || !hasFinitePosePos(nextPose)) {
    return { isBreak: false, distM: 0, dtMs: 0, reason: null };
  }
  const distM = Math.hypot(
    nextPose[12] - prevPose[12],
    nextPose[13] - prevPose[13],
    nextPose[14] - prevPose[14],
  );
  const gapMs = Number.isFinite(dtMs) ? Math.max(0, dtMs) : 0;
  const cfg = options ? { ...TRAIL_GAP_DEFAULTS, ...options } : TRAIL_GAP_DEFAULTS;

  // Resuming at virtually the same spot never spawns overlapping end/start caps or a tiny bridge.
  if (distM < cfg.minGapDistM) {
    return { isBreak: false, distM, dtMs: gapMs, reason: null };
  }
  if (explicitBreak) {
    return { isBreak: true, distM, dtMs: gapMs, reason: 'explicit-break' };
  }
  if (gapMs >= cfg.minGapMs) {
    return { isBreak: true, distM, dtMs: gapMs, reason: 'occlusion-gap' };
  }
  if (gapMs >= cfg.hardJumpMinDtMs && distM >= cfg.hardJumpM) {
    return { isBreak: true, distM, dtMs: gapMs, reason: 'occluded-jump' };
  }
  return { isBreak: false, distM, dtMs: gapMs, reason: null };
}

/**
 * Zero-phase binomial FIR tail smoother ([1,4,6,4,1]/16 interior, [1,2,1]/4 near
 * endpoints) reading raw xyz points into out xyz points while pinning segment
 * endpoints (respecting optional breakFlags) and bounding corner deviation to maxDeviationM.
 */
export function smoothLineTail(rawPositions, outPositions, count, {
  maxDeviationM = 0.012,
  breakFlags = null,
} = {}) {
  if (!rawPositions || !outPositions || count <= 0) return;

  outPositions[0] = rawPositions[0];
  outPositions[1] = rawPositions[1];
  outPositions[2] = rawPositions[2];
  if (count === 1) return;

  const lastOff = (count - 1) * 3;
  outPositions[lastOff] = rawPositions[lastOff];
  outPositions[lastOff + 1] = rawPositions[lastOff + 1];
  outPositions[lastOff + 2] = rawPositions[lastOff + 2];
  if (count < 3) return;

  const start = Math.max(1, count - 4);
  const end = count - 2;
  const cap = Number.isFinite(maxDeviationM) && maxDeviationM > 0 ? maxDeviationM : Infinity;

  for (let i = start; i <= end; i++) {
    const o = i * 3;
    const rx = rawPositions[o];
    const ry = rawPositions[o + 1];
    const rz = rawPositions[o + 2];
    const isSegStart = i === 0 || Boolean(breakFlags && breakFlags[i]);
    const isSegEnd = i === count - 1 || Boolean(breakFlags && breakFlags[i + 1]);
    if (isSegStart || isSegEnd) {
      outPositions[o] = rx;
      outPositions[o + 1] = ry;
      outPositions[o + 2] = rz;
      continue;
    }

    let sx, sy, sz;
    const can5Tap =
      i >= 2 &&
      i <= count - 3 &&
      !(breakFlags && (breakFlags[i - 1] || breakFlags[i] || breakFlags[i + 1] || breakFlags[i + 2]));

    if (can5Tap) {
      const m2 = o - 6, m1 = o - 3, p1 = o + 3, p2 = o + 6;
      sx = (rawPositions[m2] + 4 * rawPositions[m1] + 6 * rx + 4 * rawPositions[p1] + rawPositions[p2]) * 0.0625;
      sy = (rawPositions[m2 + 1] + 4 * rawPositions[m1 + 1] + 6 * ry + 4 * rawPositions[p1 + 1] + rawPositions[p2 + 1]) * 0.0625;
      sz = (rawPositions[m2 + 2] + 4 * rawPositions[m1 + 2] + 6 * rz + 4 * rawPositions[p1 + 2] + rawPositions[p2 + 2]) * 0.0625;
    } else {
      const m1 = o - 3, p1 = o + 3;
      sx = (rawPositions[m1] + 2 * rx + rawPositions[p1]) * 0.25;
      sy = (rawPositions[m1 + 1] + 2 * ry + rawPositions[p1 + 1]) * 0.25;
      sz = (rawPositions[m1 + 2] + 2 * rz + rawPositions[p1 + 2]) * 0.25;
    }

    const dx = sx - rx;
    const dy = sy - ry;
    const dz = sz - rz;
    const dev = Math.hypot(dx, dy, dz);
    if (dev > cap) {
      const scale = cap / dev;
      sx = rx + dx * scale;
      sy = ry + dy * scale;
      sz = rz + dz * scale;
    }

    outPositions[o] = sx;
    outPositions[o + 1] = sy;
    outPositions[o + 2] = sz;
  }
}

export const SPEED_RADIUS_DEFAULTS = Object.freeze({
  minRadiusM: 0.00065, // 1.3 mm diameter at standstill / turning points
  maxRadiusM: 0.0034,  // 6.8 mm diameter at >= 550 mm/s
  refSpeedMps: 0.55,   // calibrated to Quest 3 ~500+ mm/s handheld tracking envelope
  exponent: 1.15,
});

/**
 * Map instantaneous tangential path speed (m/s) to a smooth volumetric tube radius (m).
 */
export function speedToTubeRadius(speedMps, {
  minRadiusM = SPEED_RADIUS_DEFAULTS.minRadiusM,
  maxRadiusM = SPEED_RADIUS_DEFAULTS.maxRadiusM,
  refSpeedMps = SPEED_RADIUS_DEFAULTS.refSpeedMps,
  exponent = SPEED_RADIUS_DEFAULTS.exponent,
} = {}) {
  const minR = Number.isFinite(minRadiusM) && minRadiusM > 0 ? minRadiusM : SPEED_RADIUS_DEFAULTS.minRadiusM;
  const maxR = Number.isFinite(maxRadiusM) && maxRadiusM >= minR ? maxRadiusM : minR;
  const refV = Number.isFinite(refSpeedMps) && refSpeedMps > 1e-4 ? refSpeedMps : SPEED_RADIUS_DEFAULTS.refSpeedMps;
  const exp = Number.isFinite(exponent) && exponent > 0 ? exponent : SPEED_RADIUS_DEFAULTS.exponent;
  const v = Number.isFinite(speedMps) && speedMps > 0 ? speedMps : 0;
  const norm = Math.max(0, Math.min(1, v / refV));
  return minR + (maxR - minR) * Math.pow(norm, exp);
}

export const RIBBON_SPEED_DEFAULTS = Object.freeze({
  minHalfWidthM: 0.0025, // 5 mm total width at standstill / turning points so twist stays crisp
  maxHalfWidthM: 0.0175, // 35 mm total width (7x swell) at >= 550 mm/s
  refSpeedMps: 0.55,
  exponent: 1.0,
});

const HEAT_STOPS = Object.freeze([
  Object.freeze([0.00, 0.12, 0.68, 0.98]), // Cyan-Blue
  Object.freeze([0.35, 0.13, 0.82, 0.36]), // Emerald Green
  Object.freeze([0.70, 0.98, 0.80, 0.08]), // Bright Yellow
  Object.freeze([1.00, 0.96, 0.22, 0.18]), // Vivid Red-Orange
]);

/**
 * Map instantaneous tangential path speed (m/s) to a smooth ribbon half-width (m).
 */
export function speedToRibbonHalfWidth(speedMps, {
  minHalfWidthM = RIBBON_SPEED_DEFAULTS.minHalfWidthM,
  maxHalfWidthM = RIBBON_SPEED_DEFAULTS.maxHalfWidthM,
  refSpeedMps = RIBBON_SPEED_DEFAULTS.refSpeedMps,
  exponent = RIBBON_SPEED_DEFAULTS.exponent,
} = {}) {
  const minHW = Number.isFinite(minHalfWidthM) && minHalfWidthM > 0 ? minHalfWidthM : RIBBON_SPEED_DEFAULTS.minHalfWidthM;
  const maxHW = Number.isFinite(maxHalfWidthM) && maxHalfWidthM >= minHW ? maxHalfWidthM : minHW;
  const refV = Number.isFinite(refSpeedMps) && refSpeedMps > 1e-4 ? refSpeedMps : RIBBON_SPEED_DEFAULTS.refSpeedMps;
  const exp = Number.isFinite(exponent) && exponent > 0 ? exponent : RIBBON_SPEED_DEFAULTS.exponent;
  const v = Number.isFinite(speedMps) && speedMps > 0 ? speedMps : 0;
  const norm = Math.max(0, Math.min(1, v / refV));
  return minHW + (maxHW - minHW) * Math.pow(norm, exp);
}

/**
 * Map instantaneous tangential path speed (m/s) to a 4-stop [r, g, b] heatmap in [0..1].
 */
export function speedToHeatColor(speedMps, {
  refSpeedMps = RIBBON_SPEED_DEFAULTS.refSpeedMps,
  out = null,
} = {}) {
  const refV = Number.isFinite(refSpeedMps) && refSpeedMps > 1e-4 ? refSpeedMps : RIBBON_SPEED_DEFAULTS.refSpeedMps;
  const v = Number.isFinite(speedMps) && speedMps > 0 ? speedMps : 0;
  const t = Math.max(0, Math.min(1, v / refV));

  let s0 = HEAT_STOPS[0];
  let s1 = HEAT_STOPS[HEAT_STOPS.length - 1];
  for (let i = 0; i < HEAT_STOPS.length - 1; i++) {
    if (t <= HEAT_STOPS[i + 1][0]) {
      s0 = HEAT_STOPS[i];
      s1 = HEAT_STOPS[i + 1];
      break;
    }
  }
  const span = s1[0] - s0[0];
  const f = span > 1e-6 ? Math.max(0, Math.min(1, (t - s0[0]) / span)) : 0;
  const r = s0[1] + (s1[1] - s0[1]) * f;
  const g = s0[2] + (s1[2] - s0[2]) * f;
  const b = s0[3] + (s1[3] - s0[3]) * f;

  if (out) {
    out[0] = r;
    out[1] = g;
    out[2] = b;
    return out;
  }
  return [r, g, b];
}

export const TURBO_SPEED_DEFAULTS = Object.freeze({
  refSpeedMps: 0.60, // 0–600 mm/s default display envelope
});

// Official Google Turbo sRGB LUT (17 stops at t = 0/16 .. 16/16; Apache-2.0, Anton Mikhailov).
const TURBO_STOPS = Object.freeze([
  Object.freeze([0.18995, 0.07176, 0.23217]), // 0/16  #30123b
  Object.freeze([0.25107, 0.25237, 0.63374]), // 1/16
  Object.freeze([0.27628, 0.42118, 0.89123]), // 2/16
  Object.freeze([0.25862, 0.57958, 0.99876]), // 3/16
  Object.freeze([0.15844, 0.73551, 0.92305]), // 4/16
  Object.freeze([0.09267, 0.86554, 0.76230]), // 5/16
  Object.freeze([0.19659, 0.94901, 0.59466]), // 6/16
  Object.freeze([0.42778, 0.99419, 0.38575]), // 7/16
  Object.freeze([0.64362, 0.98999, 0.23356]), // 8/16
  Object.freeze([0.80473, 0.92452, 0.20459]), // 9/16
  Object.freeze([0.93301, 0.81236, 0.22667]), // 10/16
  Object.freeze([0.99314, 0.67408, 0.20348]), // 11/16
  Object.freeze([0.98360, 0.49291, 0.12849]), // 12/16
  Object.freeze([0.92105, 0.31489, 0.05475]), // 13/16
  Object.freeze([0.81608, 0.18462, 0.01809]), // 14/16
  Object.freeze([0.66449, 0.08436, 0.00424]), // 15/16
  Object.freeze([0.47960, 0.01583, 0.01055]), // 16/16 #7a0403
]);

/**
 * Sample Google's Turbo perceptual colormap at normalized scalar t in [0..1] (sRGB [r, g, b] in [0..1]).
 */
export function turboColormap(t, out = null) {
  const x = Number.isFinite(t) ? Math.max(0, Math.min(1, t)) : 0;
  const scaled = x * 16;
  const i0 = Math.min(15, scaled | 0);
  const f = scaled - i0;
  const s0 = TURBO_STOPS[i0];
  const s1 = TURBO_STOPS[i0 + 1];
  const r = s0[0] + (s1[0] - s0[0]) * f;
  const g = s0[1] + (s1[1] - s0[1]) * f;
  const b = s0[2] + (s1[2] - s0[2]) * f;
  if (out) {
    out[0] = r;
    out[1] = g;
    out[2] = b;
    return out;
  }
  return [r, g, b];
}

/**
 * Map instantaneous tangential path speed (m/s) to a Google Turbo sRGB color [r, g, b] in [0..1].
 */
export function speedToTurboColor(speedMps, {
  refSpeedMps = TURBO_SPEED_DEFAULTS.refSpeedMps,
  out = null,
} = {}) {
  const refV = Number.isFinite(refSpeedMps) && refSpeedMps > 1e-4 ? refSpeedMps : TURBO_SPEED_DEFAULTS.refSpeedMps;
  const v = Number.isFinite(speedMps) && speedMps > 0 ? speedMps : 0;
  return turboColormap(Math.max(0, Math.min(1, v / refV)), out);
}

function writeNormalizedVec3(out, off, x, y, z) {
  const len = Math.hypot(x, y, z);
  if (len > 1e-6) {
    const inv = 1 / len;
    out[off] = x * inv;
    out[off + 1] = y * inv;
    out[off + 2] = z * inv;
  } else {
    out[off] = 1;
    out[off + 1] = 0;
    out[off + 2] = 0;
  }
}

/**
 * Zero-phase 3-tap binomial tail smoother ([1,2,1]/4, re-normalized) for unit 3D twist axes
 * at index count - 2, pinning endpoints and skipping across rapid > 90 deg reversals.
 */
export function smoothAxisTail(rawAxes, outAxes, count) {
  if (!rawAxes || !outAxes || count <= 0) return;

  writeNormalizedVec3(outAxes, 0, rawAxes[0], rawAxes[1], rawAxes[2]);
  if (count === 1) return;

  const lastOff = (count - 1) * 3;
  writeNormalizedVec3(outAxes, lastOff, rawAxes[lastOff], rawAxes[lastOff + 1], rawAxes[lastOff + 2]);
  if (count < 3) return;

  const i = count - 2;
  const o = i * 3;
  const m1 = o - 3;
  const p1 = o + 3;
  const cx = rawAxes[o], cy = rawAxes[o + 1], cz = rawAxes[o + 2];
  const px = rawAxes[m1], py = rawAxes[m1 + 1], pz = rawAxes[m1 + 2];
  const nx = rawAxes[p1], ny = rawAxes[p1 + 1], nz = rawAxes[p1 + 2];

  const dotPrev = px * cx + py * cy + pz * cz;
  const dotNext = nx * cx + ny * cy + nz * cz;
  if (dotPrev > 0 && dotNext > 0) {
    writeNormalizedVec3(
      outAxes,
      o,
      px + 2 * cx + nx,
      py + 2 * cy + ny,
      pz + 2 * cz + nz,
    );
  } else {
    writeNormalizedVec3(outAxes, o, cx, cy, cz);
  }
}

export const MOTION_WAKE_DEFAULTS = Object.freeze({
  durationMs: 1600,
  maxPoints: 72,
  minStepM: 0.005,
  minIntervalMs: 20,
  maxJumpM: 0.18,
  headFilterTauSec: 0.035,
  smoothPasses: 2,
  maxSmoothDeviationM: 0.014,
  minVisibleArcM: 0.014,
  minRadiusM: 0.00035,
  maxRadiusM: 0.00130,
  radiusExponent: 0.65,
  fadeExponent: 1.40,
  maxOpacity: 0.74,
});

function isValidVec3(p) {
  return Boolean(
    p &&
    p.length >= 3 &&
    Number.isFinite(p[0]) &&
    Number.isFinite(p[1]) &&
    Number.isFinite(p[2]),
  );
}

/**
 * Pre-allocated, zero-GC ephemeral motion-wake ring buffer for moving tags in default
 * non-recording mode. Combines exponential head low-pass filtering, multi-pass zero-phase
 * binomial smoothing, continuous cutoff interpolation, and life-tapered radius/fade.
 */
export function createMotionWakeBuffer(options = {}) {
  const cfg = { ...MOTION_WAKE_DEFAULTS, ...options };
  const cap = Math.max(4, Math.floor(cfg.maxPoints));
  const outCap = cap + 2;

  const rawX = new Float32Array(cap);
  const rawY = new Float32Array(cap);
  const rawZ = new Float32Array(cap);
  const rawSpd = new Float32Array(cap);
  const rawT = new Float64Array(cap);

  const outPositions = new Float32Array(outCap * 3);
  const basePositions = new Float32Array(outCap * 3);
  const workPositions = new Float32Array(outCap * 3);
  const outTimes = new Float64Array(outCap);
  const outRadii = new Float32Array(outCap);
  const outFades = new Float32Array(outCap);
  const outSpeeds = new Float32Array(outCap);

  let activeLength = 0;
  let hasHead = false;
  let fx = 0;
  let fy = 0;
  let fz = 0;
  let fTimeMs = 0;
  let fSpeed = 0;
  let hasShownStroke = false;
  let peakArcM = 0;

  function emptySample() {
    return {
      count: 0,
      positions: outPositions,
      radii: outRadii,
      fades: outFades,
      speeds: outSpeeds,
      arcLengthM: 0,
    };
  }

  function clear() {
    activeLength = 0;
    hasHead = false;
    fx = 0;
    fy = 0;
    fz = 0;
    fTimeMs = 0;
    fSpeed = 0;
    hasShownStroke = false;
    peakArcM = 0;
  }

  function seed(pos3, nowMs) {
    if (!isValidVec3(pos3) || !Number.isFinite(nowMs)) return false;
    fx = pos3[0];
    fy = pos3[1];
    fz = pos3[2];
    fTimeMs = nowMs;
    fSpeed = 0;
    hasHead = true;
    rawX[0] = fx;
    rawY[0] = fy;
    rawZ[0] = fz;
    rawT[0] = nowMs;
    rawSpd[0] = 0;
    activeLength = 1;
    hasShownStroke = false;
    peakArcM = 0;
    return true;
  }

  function push(pos3, nowMs, { speedMps = null } = {}) {
    if (!isValidVec3(pos3) || !Number.isFinite(nowMs)) return false;
    if (!hasHead || activeLength === 0 || nowMs < fTimeMs - 500) {
      return seed(pos3, nowMs);
    }

    const px = pos3[0];
    const py = pos3[1];
    const pz = pos3[2];
    const dHead = Math.hypot(px - fx, py - fy, pz - fz);
    if (dHead > cfg.maxJumpM) {
      return seed(pos3, nowMs);
    }

    const dtMs = Math.max(0, nowMs - fTimeMs);
    const dtSec = dtMs * 0.001;
    if (dtSec > 0) {
      const alpha = cfg.headFilterTauSec > 0 ? 1 - Math.exp(-dtSec / cfg.headFilterTauSec) : 1;
      const prevFx = fx;
      const prevFy = fy;
      const prevFz = fz;
      fx += (px - fx) * alpha;
      fy += (py - fy) * alpha;
      fz += (pz - fz) * alpha;
      const instSpd = Math.min(2.5, Math.hypot(fx - prevFx, fy - prevFy, fz - prevFz) / Math.max(0.004, dtSec));
      const targetSpd = Number.isFinite(speedMps) && speedMps >= 0 ? speedMps : instSpd;
      fSpeed = fSpeed * 0.65 + targetSpd * 0.35;
      fTimeMs = nowMs;
    } else if (Number.isFinite(speedMps) && speedMps >= 0) {
      fSpeed = fSpeed * 0.65 + speedMps * 0.35;
    }

    const last = activeLength - 1;
    const dKnot = Math.hypot(fx - rawX[last], fy - rawY[last], fz - rawZ[last]);
    const dtKnotMs = nowMs - rawT[last];
    if ((dKnot >= cfg.minStepM && dtKnotMs >= cfg.minIntervalMs) || dKnot >= cfg.minStepM * 2) {
      if (activeLength >= cap) {
        rawX.copyWithin(0, 1, cap);
        rawY.copyWithin(0, 1, cap);
        rawZ.copyWithin(0, 1, cap);
        rawT.copyWithin(0, 1, cap);
        rawSpd.copyWithin(0, 1, cap);
        activeLength = cap - 1;
      }
      const idx = activeLength++;
      rawX[idx] = fx;
      rawY[idx] = fy;
      rawZ[idx] = fz;
      rawT[idx] = nowMs;
      rawSpd[idx] = fSpeed;
    }
    return true;
  }

  function sample(nowMs, { includeLiveHead = true } = {}) {
    if (!Number.isFinite(nowMs) || activeLength === 0) {
      return emptySample();
    }

    const cutoffMs = nowMs - cfg.durationMs;
    let drop = 0;
    while (drop + 1 < activeLength && rawT[drop + 1] <= cutoffMs) {
      drop++;
    }
    if (drop > 0) {
      rawX.copyWithin(0, drop, activeLength);
      rawY.copyWithin(0, drop, activeLength);
      rawZ.copyWithin(0, drop, activeLength);
      rawT.copyWithin(0, drop, activeLength);
      rawSpd.copyWithin(0, drop, activeLength);
      activeLength -= drop;
    }

    if (activeLength === 0 || rawT[activeLength - 1] <= cutoffMs) {
      activeLength = 0;
      hasShownStroke = false;
      peakArcM = 0;
      return emptySample();
    }

    let count = 0;
    let startIdx = 0;
    if (rawT[0] < cutoffMs && activeLength >= 2) {
      const spanT = rawT[1] - rawT[0];
      const frac = spanT > 1e-6 ? Math.max(0, Math.min(1, (cutoffMs - rawT[0]) / spanT)) : 1;
      outPositions[0] = rawX[0] + (rawX[1] - rawX[0]) * frac;
      outPositions[1] = rawY[0] + (rawY[1] - rawY[0]) * frac;
      outPositions[2] = rawZ[0] + (rawZ[1] - rawZ[0]) * frac;
      outTimes[0] = cutoffMs;
      outSpeeds[0] = rawSpd[0] + (rawSpd[1] - rawSpd[0]) * frac;
      count = 1;
      startIdx = 1;
    }

    for (let k = startIdx; k < activeLength; k++) {
      const o = count * 3;
      outPositions[o] = rawX[k];
      outPositions[o + 1] = rawY[k];
      outPositions[o + 2] = rawZ[k];
      outTimes[count] = rawT[k];
      outSpeeds[count] = rawSpd[k];
      count++;
    }

    if (includeLiveHead && hasHead && count < outCap) {
      const lastO = (count - 1) * 3;
      const dHeadEnd = Math.hypot(
        fx - outPositions[lastO],
        fy - outPositions[lastO + 1],
        fz - outPositions[lastO + 2],
      );
      if (dHeadEnd >= 0.0005) {
        const o = count * 3;
        outPositions[o] = fx;
        outPositions[o + 1] = fy;
        outPositions[o + 2] = fz;
        outTimes[count] = Math.min(nowMs, Math.max(outTimes[count - 1], fTimeMs));
        outSpeeds[count] = fSpeed;
        count++;
      }
    }

    let arcLengthM = 0;
    for (let i = 1; i < count; i++) {
      const o = i * 3;
      arcLengthM += Math.hypot(
        outPositions[o] - outPositions[o - 3],
        outPositions[o + 1] - outPositions[o - 2],
        outPositions[o + 2] - outPositions[o - 1],
      );
    }

    if (arcLengthM > peakArcM) peakArcM = arcLengthM;
    if (!hasShownStroke) {
      if (count < 2 || arcLengthM < cfg.minVisibleArcM) {
        return emptySample();
      }
      hasShownStroke = true;
    }
    if (count < 2) {
      hasShownStroke = false;
      peakArcM = 0;
      return emptySample();
    }

    const len3 = count * 3;
    basePositions.set(outPositions.subarray(0, len3));
    const passes = Math.max(0, cfg.smoothPasses | 0);
    if (passes > 0 && count >= 3) {
      let src = outPositions;
      let dst = workPositions;
      for (let p = 0; p < passes; p++) {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        const lastO = (count - 1) * 3;
        dst[lastO] = src[lastO];
        dst[lastO + 1] = src[lastO + 1];
        dst[lastO + 2] = src[lastO + 2];
        for (let i = 1; i < count - 1; i++) {
          const o = i * 3;
          if (i >= 2 && i <= count - 3) {
            const m2 = o - 6, m1 = o - 3, p1 = o + 3, p2 = o + 6;
            dst[o] = (src[m2] + 4 * src[m1] + 6 * src[o] + 4 * src[p1] + src[p2]) * 0.0625;
            dst[o + 1] = (src[m2 + 1] + 4 * src[m1 + 1] + 6 * src[o + 1] + 4 * src[p1 + 1] + src[p2 + 1]) * 0.0625;
            dst[o + 2] = (src[m2 + 2] + 4 * src[m1 + 2] + 6 * src[o + 2] + 4 * src[p1 + 2] + src[p2 + 2]) * 0.0625;
          } else {
            const m1 = o - 3, p1 = o + 3;
            dst[o] = (src[m1] + 2 * src[o] + src[p1]) * 0.25;
            dst[o + 1] = (src[m1 + 1] + 2 * src[o + 1] + src[p1 + 1]) * 0.25;
            dst[o + 2] = (src[m1 + 2] + 2 * src[o + 2] + src[p1 + 2]) * 0.25;
          }
        }
        const tmp = src;
        src = dst;
        dst = tmp;
      }
      if (src !== outPositions) {
        outPositions.set(src.subarray(0, len3));
      }
      const maxDev = Number.isFinite(cfg.maxSmoothDeviationM) && cfg.maxSmoothDeviationM > 0
        ? cfg.maxSmoothDeviationM
        : Infinity;
      if (Number.isFinite(maxDev)) {
        for (let i = 1; i < count - 1; i++) {
          const o = i * 3;
          const dx = outPositions[o] - basePositions[o];
          const dy = outPositions[o + 1] - basePositions[o + 1];
          const dz = outPositions[o + 2] - basePositions[o + 2];
          const dev = Math.hypot(dx, dy, dz);
          if (dev > maxDev) {
            const sc = maxDev / dev;
            outPositions[o] = basePositions[o] + dx * sc;
            outPositions[o + 1] = basePositions[o + 1] + dy * sc;
            outPositions[o + 2] = basePositions[o + 2] + dz * sc;
          }
        }
      }
    }

    const strokeGain = Math.min(1, peakArcM / 0.035);
    for (let i = 0; i < count; i++) {
      const life = Math.max(0, Math.min(1, 1 - (nowMs - outTimes[i]) / cfg.durationMs));
      outRadii[i] = cfg.minRadiusM + (cfg.maxRadiusM - cfg.minRadiusM) * Math.pow(life, cfg.radiusExponent);
      outFades[i] = Math.pow(life, cfg.fadeExponent) * strokeGain;
    }

    return {
      count,
      positions: outPositions,
      radii: outRadii,
      fades: outFades,
      speeds: outSpeeds,
      arcLengthM,
    };
  }

  return {
    clear,
    seed,
    push,
    sample,
    hasPoints: () => activeLength > 0,
  };
}

export const TRAIL_VERTEX_ROLE = Object.freeze({
  NORMAL: 0,
  UNRELATED: 1,
  PINNED_OUTER: 2,
  PINNED_SINGLE: 3,
  HOVERED: 4,
  PINNED_PREVIEW: 5,
  PINNED_SPAN: 6,
});

export const TRAIL_MEASURE_STYLE_DEFAULTS = Object.freeze({
  normalRadiusScale: 1.0,
  unrelatedRadiusScale: 0.77,
  hoveredRadiusScale: 1.17,
  pinnedSingleRadiusScale: 0.98,
  pinnedOuterRadiusScale: 0.84,
  pinnedPreviewRadiusScale: 1.38,
  pinnedSpanRadiusScale: 1.64,
  normalFade: 0.94,
  unrelatedFade: 0.22,
  hoveredFade: 0.98,
  pinnedSingleFade: 0.72,
  pinnedOuterFade: 0.48,
  pinnedPreviewFade: 0.95,
  pinnedSpanFade: 1.0,
  unrelatedGrayLinearRgb: Object.freeze([0.28, 0.30, 0.34]),
  pinnedSingleDesat: 0.16,
  pinnedOuterDesat: 0.34,
  hoveredLiftId: 0.12,
  pinnedPreviewLiftId: 0.14,
  pinnedSpanLiftId: 0.18,
});

function isValidTrailPointRef(pt) {
  return Boolean(
    pt &&
    Number.isFinite(pt.tagId) &&
    Number.isFinite(pt.segmentIndex) &&
    pt.segmentIndex >= 1,
  );
}

/**
 * Group active MEASURE-mode pinned and hovered trail points by continuous strand
 * (`${tagId}:${segmentIndex}`) so renderer code can distinguish unrelated runs/IDs,
 * hovered strands, 1-pin active strands, and bold between-pins spans.
 */
export function buildTrailMeasureFocus({
  active = false,
  pinnedPoints = [],
  hoveredPoints = [],
} = {}) {
  const strands = new Map();
  const pinnedStrands = new Set();
  const hoveredStrands = new Set();

  if (!active) {
    return { hasFocus: false, strands, pinnedStrands, hoveredStrands };
  }

  const ensureStrand = (tagId, segmentIndex) => {
    const key = `${tagId}:${segmentIndex}`;
    let strand = strands.get(key);
    if (!strand) {
      strand = {
        key,
        tagId,
        segmentIndex,
        pinSeqs: [],
        hoverSeqs: [],
        pinTotal: 0,
        pinCount: 0,
        isHovered: false,
        state: 'hovered',
        _pinSeqSet: new Set(),
        _hoverSeqSet: new Set(),
      };
      strands.set(key, strand);
    }
    return strand;
  };

  if (pinnedPoints) {
    for (const pt of pinnedPoints) {
      if (!isValidTrailPointRef(pt)) continue;
      const strand = ensureStrand(pt.tagId, pt.segmentIndex);
      strand.pinTotal++;
      if (Number.isFinite(pt.lineSeq)) {
        strand._pinSeqSet.add(pt.lineSeq);
      }
      pinnedStrands.add(strand.key);
    }
  }

  if (hoveredPoints) {
    for (const pt of hoveredPoints) {
      if (!isValidTrailPointRef(pt)) continue;
      const strand = ensureStrand(pt.tagId, pt.segmentIndex);
      strand.isHovered = true;
      if (Number.isFinite(pt.lineSeq)) {
        strand._hoverSeqSet.add(pt.lineSeq);
      }
      hoveredStrands.add(strand.key);
    }
  }

  if (strands.size === 0) {
    return { hasFocus: false, strands, pinnedStrands, hoveredStrands };
  }

  for (const strand of strands.values()) {
    strand.pinSeqs = Array.from(strand._pinSeqSet).sort((a, b) => a - b);
    strand.hoverSeqs = Array.from(strand._hoverSeqSet).sort((a, b) => a - b);
    strand.pinCount = strand.pinTotal;
    strand.state = strand.pinCount >= 2
      ? 'pinned-span'
      : (strand.pinCount === 1 ? 'pinned-single' : 'hovered');
    delete strand._pinSeqSet;
    delete strand._hoverSeqSet;
  }

  return { hasFocus: true, strands, pinnedStrands, hoveredStrands };
}

function resolveSegmentSeqToIndex(lineSeqs, segStart, segEnd, targetSeq) {
  if (!lineSeqs || !Number.isFinite(targetSeq)) return segStart;
  let bestIdx = segStart;
  let bestDiff = Infinity;
  for (let i = segStart; i <= segEnd; i++) {
    const seq = lineSeqs[i];
    if (!Number.isFinite(seq)) continue;
    const diff = Math.abs(seq - targetSeq);
    if (diff === 0) return i;
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIdx = i;
    }
  }
  return bestIdx;
}

/**
 * Evaluate per-vertex roles, tube radii, fades, and linear RGB colors for a single
 * tag's recorded trail under the current MEASURE-mode focus state.
 */
export function evaluateEntryTrailMeasureStyle({
  tagId,
  lineCount = 0,
  lineSegmentIds = null,
  lineSeqs = null,
  baseColors = null,
  focus = null,
  baseRadiusM = 0.0015,
  preserveChroma = false,
  outRoles = null,
  outRadii = null,
  outFades = null,
  outColors = null,
  style = null,
} = {}) {
  const n = Math.max(0, lineCount | 0);
  const segmentStates = new Map();
  const hasFocus = Boolean(focus?.hasFocus);
  if (n === 0) {
    return {
      hasFocus,
      hasActiveVertex: false,
      hasDimVertex: false,
      hasMixedLayers: false,
      segmentStates,
    };
  }

  const cfg = style ? { ...TRAIL_MEASURE_STYLE_DEFAULTS, ...style } : TRAIL_MEASURE_STYLE_DEFAULTS;
  const baseR = Number.isFinite(baseRadiusM) && baseRadiusM > 1e-5 ? baseRadiusM : 0.0015;
  const gray = cfg.unrelatedGrayLinearRgb || TRAIL_MEASURE_STYLE_DEFAULTS.unrelatedGrayLinearRgb;

  let hasActiveVertex = false;
  let hasDimVertex = false;
  let hasLowFade = false;
  let hasHighFade = false;
  let hasUnrelated = false;
  let hasNonUnrelated = false;

  let segStart = 0;
  while (segStart < n) {
    const segId = lineSegmentIds ? (lineSegmentIds[segStart] || 1) : 1;
    let segEnd = segStart;
    while (segEnd + 1 < n && (lineSegmentIds ? (lineSegmentIds[segEnd + 1] || 1) : 1) === segId) {
      segEnd++;
    }

    let strand = null;
    if (hasFocus && focus.strands) {
      strand = focus.strands.get(`${tagId}:${segId}`) || null;
    }

    let segState = 'normal';
    let hasPinnedSpan = false;
    let spanMin = -1;
    let spanMax = -1;
    let hasPreviewSpan = false;
    let previewMin = -1;
    let previewMax = -1;

    if (!hasFocus) {
      segState = 'normal';
    } else if (!strand) {
      segState = 'unrelated';
    } else {
      segState = strand.state;
      const pinIndices = [];
      for (let k = 0; k < strand.pinSeqs.length; k++) {
        pinIndices.push(resolveSegmentSeqToIndex(lineSeqs, segStart, segEnd, strand.pinSeqs[k]));
      }
      pinIndices.sort((a, b) => a - b);

      const hoverIndices = [];
      for (let k = 0; k < strand.hoverSeqs.length; k++) {
        hoverIndices.push(resolveSegmentSeqToIndex(lineSeqs, segStart, segEnd, strand.hoverSeqs[k]));
      }
      hoverIndices.sort((a, b) => a - b);

      if (pinIndices.length >= 2) {
        hasPinnedSpan = true;
        spanMin = pinIndices[0];
        spanMax = pinIndices[pinIndices.length - 1];
      }

      if (pinIndices.length >= 1 && hoverIndices.length >= 1) {
        const candMin = Math.min(pinIndices[0], hoverIndices[0]);
        const candMax = Math.max(pinIndices[pinIndices.length - 1], hoverIndices[hoverIndices.length - 1]);
        if (candMax > candMin) {
          hasPreviewSpan = true;
          previewMin = candMin;
          previewMax = candMax;
        }
      } else if (pinIndices.length === 0 && hoverIndices.length >= 2) {
        const candMin = hoverIndices[0];
        const candMax = hoverIndices[hoverIndices.length - 1];
        if (candMax > candMin) {
          hasPreviewSpan = true;
          previewMin = candMin;
          previewMax = candMax;
        }
      }
    }

    segmentStates.set(segId, segState);

    for (let i = segStart; i <= segEnd; i++) {
      let role;
      if (!hasFocus) {
        role = TRAIL_VERTEX_ROLE.NORMAL;
      } else if (!strand) {
        role = TRAIL_VERTEX_ROLE.UNRELATED;
      } else if (hasPinnedSpan && i >= spanMin && i <= spanMax) {
        role = TRAIL_VERTEX_ROLE.PINNED_SPAN;
      } else if (hasPreviewSpan && i >= previewMin && i <= previewMax) {
        role = TRAIL_VERTEX_ROLE.PINNED_PREVIEW;
      } else if (hasPinnedSpan || hasPreviewSpan) {
        role = TRAIL_VERTEX_ROLE.PINNED_OUTER;
      } else if (strand.pinCount === 1) {
        role = TRAIL_VERTEX_ROLE.PINNED_SINGLE;
      } else {
        role = TRAIL_VERTEX_ROLE.HOVERED;
      }

      const cOff = i * 3;
      const br = baseColors && Number.isFinite(baseColors[cOff]) ? baseColors[cOff] : 1;
      const bg = baseColors && Number.isFinite(baseColors[cOff + 1]) ? baseColors[cOff + 1] : 1;
      const bb = baseColors && Number.isFinite(baseColors[cOff + 2]) ? baseColors[cOff + 2] : 1;

      let rScale = cfg.normalRadiusScale;
      let fade = cfg.normalFade;
      let cr = br;
      let cg = bg;
      let cb = bb;

      if (role === TRAIL_VERTEX_ROLE.UNRELATED) {
        rScale = cfg.unrelatedRadiusScale;
        fade = cfg.unrelatedFade;
        cr = gray[0];
        cg = gray[1];
        cb = gray[2];
      } else if (role === TRAIL_VERTEX_ROLE.PINNED_OUTER) {
        rScale = cfg.pinnedOuterRadiusScale;
        fade = cfg.pinnedOuterFade;
        const t = cfg.pinnedOuterDesat;
        cr = br + (gray[0] - br) * t;
        cg = bg + (gray[1] - bg) * t;
        cb = bb + (gray[2] - bb) * t;
      } else if (role === TRAIL_VERTEX_ROLE.PINNED_SINGLE) {
        rScale = cfg.pinnedSingleRadiusScale;
        fade = cfg.pinnedSingleFade;
        const t = cfg.pinnedSingleDesat;
        cr = br + (gray[0] - br) * t;
        cg = bg + (gray[1] - bg) * t;
        cb = bb + (gray[2] - bb) * t;
      } else if (role === TRAIL_VERTEX_ROLE.HOVERED) {
        rScale = cfg.hoveredRadiusScale;
        fade = cfg.hoveredFade;
        if (!preserveChroma) {
          const t = cfg.hoveredLiftId;
          cr = br + (1 - br) * t;
          cg = bg + (1 - bg) * t;
          cb = bb + (1 - bb) * t;
        }
      } else if (role === TRAIL_VERTEX_ROLE.PINNED_PREVIEW) {
        rScale = cfg.pinnedPreviewRadiusScale;
        fade = cfg.pinnedPreviewFade;
        if (!preserveChroma) {
          const t = cfg.pinnedPreviewLiftId;
          cr = br + (1 - br) * t;
          cg = bg + (1 - bg) * t;
          cb = bb + (1 - bb) * t;
        }
      } else if (role === TRAIL_VERTEX_ROLE.PINNED_SPAN) {
        rScale = cfg.pinnedSpanRadiusScale;
        fade = cfg.pinnedSpanFade;
        if (!preserveChroma) {
          const t = cfg.pinnedSpanLiftId;
          cr = br + (1 - br) * t;
          cg = bg + (1 - bg) * t;
          cb = bb + (1 - bb) * t;
        }
      }

      if (outRoles) outRoles[i] = role;
      if (outRadii) outRadii[i] = baseR * rScale;
      if (outFades) outFades[i] = fade;
      if (outColors) {
        outColors[cOff] = cr;
        outColors[cOff + 1] = cg;
        outColors[cOff + 2] = cb;
      }

      if (role === TRAIL_VERTEX_ROLE.UNRELATED) {
        hasUnrelated = true;
      } else {
        hasActiveVertex = true;
        hasNonUnrelated = true;
      }
      if (fade < 0.90) {
        hasDimVertex = true;
        hasLowFade = true;
      } else {
        hasHighFade = true;
      }
    }

    segStart = segEnd + 1;
  }

  return {
    hasFocus,
    hasActiveVertex,
    hasDimVertex,
    hasMixedLayers: (hasLowFade && hasHighFade) || (hasUnrelated && hasNonUnrelated),
    segmentStates,
  };
}

/**
 * Compute exact curved 3D path arc length (m) along a single continuous recorded
 * trail segment between sequence numbers `seqA` and `seqB`. Returns `null` if the
 * segment is unavailable, interrupted by a break, or partially evicted from the ring buffer.
 */
export function measureTrailArcLength({
  linePositions = null,
  lineCount = 0,
  lineSegmentIds = null,
  lineSeqs = null,
  lineBreaks = null,
  segmentIndex = 0,
  seqA = null,
  seqB = null,
} = {}) {
  const n = Math.max(0, lineCount | 0);
  if (
    !linePositions ||
    !lineSegmentIds ||
    !lineSeqs ||
    n < 2 ||
    !Number.isFinite(segmentIndex) ||
    segmentIndex < 1 ||
    !Number.isFinite(seqA) ||
    !Number.isFinite(seqB)
  ) {
    return null;
  }

  let segStart = -1;
  let segEnd = -1;
  for (let i = 0; i < n; i++) {
    if (lineSegmentIds[i] === segmentIndex) {
      if (segStart < 0) segStart = i;
      segEnd = i;
    } else if (segStart >= 0) {
      break;
    }
  }
  if (segStart < 0 || segEnd < segStart) return null;

  const startSeq = lineSeqs[segStart];
  const endSeq = lineSeqs[segEnd];
  if (!Number.isFinite(startSeq) || !Number.isFinite(endSeq)) return null;
  const minSeq = Math.min(seqA, seqB);
  const maxSeq = Math.max(seqA, seqB);
  if (minSeq < startSeq || maxSeq > endSeq) return null;

  const idxA = resolveSegmentSeqToIndex(lineSeqs, segStart, segEnd, seqA);
  const idxB = resolveSegmentSeqToIndex(lineSeqs, segStart, segEnd, seqB);
  const i0 = Math.min(idxA, idxB);
  const i1 = Math.max(idxA, idxB);
  if (i0 === i1) return 0;

  let arcLengthM = 0;
  for (let i = i0; i < i1; i++) {
    if (lineBreaks && lineBreaks[i + 1]) return null;
    const o0 = i * 3;
    const o1 = o0 + 3;
    const dx = linePositions[o1] - linePositions[o0];
    const dy = linePositions[o1 + 1] - linePositions[o0 + 1];
    const dz = linePositions[o1 + 2] - linePositions[o0 + 2];
    const step = Math.hypot(dx, dy, dz);
    if (!Number.isFinite(step)) return null;
    arcLengthM += step;
  }
  return arcLengthM;
}
