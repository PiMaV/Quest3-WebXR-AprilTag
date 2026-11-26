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
