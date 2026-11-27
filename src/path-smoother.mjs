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

/**
 * Zero-phase binomial FIR tail smoother ([1,4,6,4,1]/16 interior, [1,2,1]/4 near
 * endpoints) reading raw xyz points into out xyz points while pinning endpoints and
 * bounding maximum corner deviation to maxDeviationM.
 */
export function smoothLineTail(rawPositions, outPositions, count, {
  maxDeviationM = 0.012,
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
    let sx, sy, sz;

    if (i >= 2 && i <= count - 3) {
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
