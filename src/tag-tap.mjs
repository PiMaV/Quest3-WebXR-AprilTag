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
