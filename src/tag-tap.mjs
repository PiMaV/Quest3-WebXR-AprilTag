/**
 * Local tag-plane projection and full-face surface distance metrics
 * for surface-lying and mid-air AprilTag waypoints / static tags in WebXR.
 */

/**
 * Project a 3D point in L-frame into a tag's local frame defined by column-major 4x4 lFromTag.
 * +u/+v lie in the tag plane; +n points outward along the printed face normal (+Z).
 */
export function pointInTagFrame(tagMatrix, pointL) {
  const dx = pointL[0] - tagMatrix[12];
  const dy = pointL[1] - tagMatrix[13];
  const dz = pointL[2] - tagMatrix[14];
  const u = tagMatrix[0] * dx + tagMatrix[1] * dy + tagMatrix[2] * dz;
  const v = tagMatrix[4] * dx + tagMatrix[5] * dy + tagMatrix[6] * dz;
  const n = tagMatrix[8] * dx + tagMatrix[9] * dy + tagMatrix[10] * dz;
  return { u, v, n, centerDist: Math.hypot(dx, dy, dz) };
}

/**
 * Compute tag-plane local coordinates and 3D distance to the nearest point on the tag square.
 * This ensures large static tags (80 mm / 112 mm) can be hovered and pinched anywhere on their face.
 */
export function tagSurfaceMetrics(tagMatrix, edgeMm, pointL) {
  const { u, v, n, centerDist } = pointInTagFrame(tagMatrix, pointL);
  const half = (edgeMm / 1000) / 2;
  const du = Math.max(0, Math.abs(u) - half);
  const dv = Math.max(0, Math.abs(v) - half);
  const lateralOverflow = Math.hypot(du, dv);
  const surfaceDist = Math.hypot(du, dv, n);
  return { u, v, n, half, du, dv, lateralOverflow, surfaceDist, centerDist };
}

/**
 * Determine whether a bare-hand probe is eligible to start or continue a
 * MEASURE-mode Index+Thumb pinch-to-pin hold.
 * Both left and right hands are supported; left-hand pinch is suppressed only
 * when looking directly at the left palm (Meta Quest OS menu / wrist menu).
 */
export function isPinchHandEligible({
  kind,
  handedness,
  pinHoldHand = null,
  leftPalmFacingViewer = false,
  leftMenuOpen = false,
} = {}) {
  if (kind !== 'hand') return false;
  if (handedness !== 'right' && handedness !== 'left') return false;
  if (pinHoldHand && handedness !== pinHoldHand) return false;
  if (handedness === 'left' && (leftPalmFacingViewer || leftMenuOpen)) return false;
  return true;
}

/**
 * Classify a fingertip/controller cursor's proximity to its nearest interactive target.
 * Each probe maintains its own independent state ('hot' | 'near' | 'idle').
 */
export function classifyCursorProximity(minDistM, { hotDistM = 0.048, nearDistM = 0.14 } = {}) {
  if (!Number.isFinite(minDistM) || minDistM > nearDistM) return 'idle';
  if (minDistM <= hotDistM) return 'hot';
  return 'near';
}

/**
 * Forward hover offset (m) past the WebXR `index-finger-tip` joint along the
 * distal-phalanx axis so the selector hovers ~10-12 mm in front of the physical fingertip.
 */
export const INDEX_TIP_HOVER_OFFSET_M = 0.018;

const isFiniteVec3 = (v) =>
  Array.isArray(v) &&
  v.length >= 3 &&
  Number.isFinite(v[0]) &&
  Number.isFinite(v[1]) &&
  Number.isFinite(v[2]);

/**
 * Project the pointing/poking selector forward along the distal-phalanx-to-tip
 * ray so users aim slightly ahead of their fingertip rather than jamming into targets.
 */
export function projectIndexSelectorPos(tipPos, refProximalPos, offsetM = INDEX_TIP_HOVER_OFFSET_M) {
  if (!isFiniteVec3(tipPos)) return null;
  if (isFiniteVec3(refProximalPos) && Number.isFinite(offsetM)) {
    const dx = tipPos[0] - refProximalPos[0];
    const dy = tipPos[1] - refProximalPos[1];
    const dz = tipPos[2] - refProximalPos[2];
    const len = Math.hypot(dx, dy, dz);
    if (len > 1e-4) {
      const scale = offsetM / len;
      return [tipPos[0] + dx * scale, tipPos[1] + dy * scale, tipPos[2] + dz * scale];
    }
  }
  return [tipPos[0], tipPos[1], tipPos[2]];
}

/**
 * Defaults for minimalist live-tag action pill visibility ([● REC / ■ STOP] & [⏱ MARK]).
 * Pills stay hidden while moving and until a non-carrying finger approaches a resting tag.
 * Hand-coupled tags allow bimanual activation once held steady within hand-tremor tolerance.
 */
export const TAG_ACTION_PILL_DEFAULTS = {
  minRestMs: 500,
  maxRestPosDispM: 0.012,
  minRestAxisDot: 0.99, // ~8.1 deg orientation tolerance when resting freely on table
  midAirMinZ: 0.05, // > 5 cm above tabletop plane automatically uses hand-tremor tolerance
  coupledMaxRestPosDispM: 0.024, // ~24 mm hand-tremor tolerance when held steady in one hand / mid-air
  coupledMinRestAxisDot: 0.96, // ~16 deg tilt/yaw tolerance when held steady in one hand / mid-air
  holdMaxRestPosDispM: 0.038, // ~38 mm hysteresis once pills are open or a hold is in progress
  holdMinRestAxisDot: 0.92, // ~23 deg orientation hysteresis while pills are open / held
  tagNearM: 0.14,
  zoneNearM: 0.14,
  pillNearM: 0.12,
  exitNearM: 0.20, // wider exit proximity hysteresis once pills are open
  belowOffsetM: 0.056,
  sideHalfSpanM: 0.043,
  minTableZ: 0.018,
};

export function createTagRestState() {
  return { anchorPos: null, anchorX: null, anchorZ: null, sinceMs: null };
}

export function resetTagRestState(state) {
  if (!state) return;
  state.anchorPos = null;
  state.anchorX = null;
  state.anchorZ = null;
  state.sinceMs = null;
}

/**
 * Track whether a movable AprilTag has remained stationary within tight 6DOF displacement
 * tolerances for at least `minRestMs` (default 500 ms).
 * Supports both free/placed tags (12 mm / ~8 deg) and steadily held hand-coupled tags
 * (22 mm / ~15 deg) for bimanual interaction with the opposite hand.
 */
export function updateTagRestState(
  state,
  tagMatrix,
  nowMs,
  {
    handCoupled = false,
    pillsOpen = false,
    minRestMs = TAG_ACTION_PILL_DEFAULTS.minRestMs,
    maxRestPosDispM = TAG_ACTION_PILL_DEFAULTS.maxRestPosDispM,
    minRestAxisDot = TAG_ACTION_PILL_DEFAULTS.minRestAxisDot,
    midAirMinZ = TAG_ACTION_PILL_DEFAULTS.midAirMinZ,
    coupledMaxRestPosDispM = TAG_ACTION_PILL_DEFAULTS.coupledMaxRestPosDispM,
    coupledMinRestAxisDot = TAG_ACTION_PILL_DEFAULTS.coupledMinRestAxisDot,
    holdMaxRestPosDispM = TAG_ACTION_PILL_DEFAULTS.holdMaxRestPosDispM,
    holdMinRestAxisDot = TAG_ACTION_PILL_DEFAULTS.holdMinRestAxisDot,
  } = {},
) {
  if (!state) return { isResting: false, restingMs: 0 };
  if (
    !tagMatrix ||
    tagMatrix.length < 15 ||
    !Number.isFinite(nowMs) ||
    !Number.isFinite(tagMatrix[12]) ||
    !Number.isFinite(tagMatrix[13]) ||
    !Number.isFinite(tagMatrix[14])
  ) {
    resetTagRestState(state);
    return { isResting: false, restingMs: 0 };
  }

  const tx = tagMatrix[12];
  const ty = tagMatrix[13];
  const tz = tagMatrix[14];
  const xx = tagMatrix[0];
  const xy = tagMatrix[1];
  const xz = tagMatrix[2];
  const zx = tagMatrix[8];
  const zy = tagMatrix[9];
  const zz = tagMatrix[10];

  const isMidAir = Math.abs(tz) > midAirMinZ;
  const posLimitM = pillsOpen
    ? holdMaxRestPosDispM
    : (handCoupled || isMidAir ? coupledMaxRestPosDispM : maxRestPosDispM);
  const axisDotLimit = pillsOpen
    ? holdMinRestAxisDot
    : (handCoupled || isMidAir ? coupledMinRestAxisDot : minRestAxisDot);

  if (!state.anchorPos || state.sinceMs == null) {
    state.anchorPos = [tx, ty, tz];
    state.anchorX = [xx, xy, xz];
    state.anchorZ = [zx, zy, zz];
    state.sinceMs = nowMs;
    return { isResting: false, restingMs: 0 };
  }

  const posDisp = Math.hypot(
    tx - state.anchorPos[0],
    ty - state.anchorPos[1],
    tz - state.anchorPos[2],
  );
  const xDot =
    xx * state.anchorX[0] +
    xy * state.anchorX[1] +
    xz * state.anchorX[2];
  const zDot =
    zx * state.anchorZ[0] +
    zy * state.anchorZ[1] +
    zz * state.anchorZ[2];

  if (posDisp > posLimitM || xDot < axisDotLimit || zDot < axisDotLimit) {
    state.anchorPos = [tx, ty, tz];
    state.anchorX = [xx, xy, xz];
    state.anchorZ = [zx, zy, zz];
    state.sinceMs = nowMs;
    return { isResting: false, restingMs: 0 };
  }

  const restingMs = Math.max(0, nowMs - state.sinceMs);
  return { isResting: restingMs >= minRestMs, restingMs };
}

function normVec3(v, fallback) {
  const len = Math.hypot(v[0], v[1], v[2]);
  if (len < 1e-6) return fallback.slice();
  return [v[0] / len, v[1] / len, v[2] / len];
}

/**
 * Compute side-by-side bottom-of-tag 3D positions for [● REC / ■ STOP] and [⏱ MARK].
 * Placing pills below the tag (or toward the viewer in front of flat tabletop tags) keeps
 * the approaching hand completely out of the camera-to-tag line of sight and works
 * symmetrically for both left- and right-handed users.
 */
export function computeTagActionPillLayout({
  tagMatrix,
  edgeMm = 20,
  viewerPosL = null,
  viewerRightL = null,
  viewerUpL = null,
  recording = false,
  belowOffsetM = TAG_ACTION_PILL_DEFAULTS.belowOffsetM,
  sideHalfSpanM = TAG_ACTION_PILL_DEFAULTS.sideHalfSpanM,
  minTableZ = TAG_ACTION_PILL_DEFAULTS.minTableZ,
}) {
  const tx = tagMatrix[12];
  const ty = tagMatrix[13];
  const tz = tagMatrix[14];
  const halfTag = (edgeMm / 1000) / 2;
  const distBelow = Math.max(belowOffsetM, halfTag + 0.036);

  const up = viewerUpL ? normVec3(viewerUpL, [0, 0, 1]) : [0, 0, 1];
  const right = viewerRightL ? normVec3(viewerRightL, [1, 0, 0]) : [1, 0, 0];

  let bx = tx - up[0] * distBelow;
  let by = ty - up[1] * distBelow;
  let bz = tz - up[2] * distBelow;
  let rx = right[0];
  let ry = right[1];
  let rz = right[2];

  const flatOnTable = Math.abs(tz) < 0.032 && Math.abs(tagMatrix[10]) > 0.85;
  if (flatOnTable || bz < minTableZ) {
    // Project "down" onto the horizontal table plane (pointing toward the viewer).
    let hx = -up[0];
    let hy = -up[1];
    let hLen = Math.hypot(hx, hy);
    if (hLen < 0.25 && viewerPosL) {
      hx = viewerPosL[0] - tx;
      hy = viewerPosL[1] - ty;
      hLen = Math.hypot(hx, hy);
    }
    if (hLen > 1e-4) {
      hx /= hLen;
      hy /= hLen;
    } else {
      hx = 0;
      hy = -1;
    }
    bx = tx + hx * distBelow;
    by = ty + hy * distBelow;
    bz = Math.max(minTableZ, tz + 0.012);

    // Keep the side-by-side axis horizontal above the tabletop.
    const rLen = Math.hypot(right[0], right[1]);
    if (rLen > 1e-4) {
      rx = right[0] / rLen;
      ry = right[1] / rLen;
      rz = 0;
    } else {
      rx = hy;
      ry = -hx;
      rz = 0;
    }
  }

  const zoneCenterL = [bx, by, bz];
  if (recording) {
    return {
      zoneCenterL,
      recPosL: [bx, by, bz],
      markPosL: [bx, by, bz],
    };
  }

  const recPosL = [
    bx - rx * sideHalfSpanM,
    by - ry * sideHalfSpanM,
    Math.max(flatOnTable ? minTableZ : -Infinity, bz - rz * sideHalfSpanM),
  ];
  const markPosL = [
    bx + rx * sideHalfSpanM,
    by + ry * sideHalfSpanM,
    Math.max(flatOnTable ? minTableZ : -Infinity, bz + rz * sideHalfSpanM),
  ];

  return { zoneCenterL, recPosL, markPosL };
}

/**
 * Decide whether a movable tag's [● REC / ■ STOP] and [⏱ MARK] action pills should be visible.
 * Distances must be measured from non-carrying probes only (so a hand carrying a tag never
 * opens pills for itself, while the opposite hand can open them once the tag is held steady).
 */
export function evaluateTagActionPills({
  isResting = false,
  recording = false,
  recordAllActive = false,
  hasPose = false,
  wasOpen = false,
  holding = false,
  minTagDist = Infinity,
  minZoneDist = Infinity,
  minRecDist = Infinity,
  minMarkDist = Infinity,
  tagNearM = TAG_ACTION_PILL_DEFAULTS.tagNearM,
  zoneNearM = TAG_ACTION_PILL_DEFAULTS.zoneNearM,
  pillNearM = TAG_ACTION_PILL_DEFAULTS.pillNearM,
  exitNearM = TAG_ACTION_PILL_DEFAULTS.exitNearM,
} = {}) {
  if (!hasPose || !isResting) {
    return { showRec: false, showMark: false };
  }
  const sticky = Boolean(wasOpen || holding);
  const tThresh = sticky ? Math.max(tagNearM, exitNearM) : tagNearM;
  const zThresh = sticky ? Math.max(zoneNearM, exitNearM) : zoneNearM;
  const pThresh = sticky ? Math.max(pillNearM, exitNearM) : pillNearM;
  const nearZone = minZoneDist < zThresh || minTagDist < tThresh;
  const nearRec = nearZone || minRecDist < pThresh;
  const nearMark = nearZone || minMarkDist < pThresh;
  return {
    showRec: !recordAllActive && nearRec,
    showMark: !recording && nearMark,
  };
}
