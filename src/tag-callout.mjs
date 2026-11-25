/**
 * View-aware 3D callout placement for AprilTag hover cards and compact pinned pills.
 * Keeps cards strictly clear of the viewer-to-tag line of sight and the probing hand
 * (especially when looking down at surface-lying AprilTags on a table).
 */

export const HOVER_CALLOUT_DEFAULTS = {
  cardWidthM: 0.124,
  cardHeightM: 0.056,
  pillWidthM: 0.104,
  pillHeightM: 0.026,
  minTableZ: 0.036,
  minPillTableZ: 0.036,
};

function dot3(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross3(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function norm3(v, fallback = [0, 0, 1]) {
  const len = Math.hypot(v[0], v[1], v[2]);
  if (len < 1e-6) return fallback.slice();
  return [v[0] / len, v[1] / len, v[2] / len];
}

/**
 * Build an orthonormal screen basis (screenRight, screenUp, toEye) at a target point in L-frame,
 * where both screenRight and screenUp are strictly orthogonal to the line of sight toEye.
 */
export function viewBasisAtPointL(targetL, viewerPosL, viewerRightL, viewerUpL) {
  const eyeVec = viewerPosL
    ? [viewerPosL[0] - targetL[0], viewerPosL[1] - targetL[1], viewerPosL[2] - targetL[2]]
    : [0, -0.5, 0.5];
  const toEye = norm3(eyeVec, [0, -0.7071, 0.7071]);

  let rightHint = viewerRightL ? norm3(viewerRightL, [1, 0, 0]) : [1, 0, 0];
  const rDot = dot3(rightHint, toEye);
  let rProj = [
    rightHint[0] - rDot * toEye[0],
    rightHint[1] - rDot * toEye[1],
    rightHint[2] - rDot * toEye[2],
  ];
  if (Math.hypot(rProj[0], rProj[1], rProj[2]) < 1e-4) {
    const ref = Math.abs(toEye[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0];
    rProj = cross3(ref, toEye);
  }
  let screenRight = norm3(rProj, [1, 0, 0]);
  let screenUp = norm3(cross3(toEye, screenRight), [0, 0, 1]);

  if (viewerUpL && dot3(screenUp, viewerUpL) < 0) {
    screenUp = [-screenUp[0], -screenUp[1], -screenUp[2]];
    screenRight = [-screenRight[0], -screenRight[1], -screenRight[2]];
  }
  return { toEye, screenRight, screenUp };
}

/**
 * Compute an unobstructed 3D hover-card center and leader-line anchor in L-frame.
 * Shifts the card laterally away from the probing hand and upward in screen space
 * so it never lies between the viewer's eyes and the tag or pinching fingers.
 */
export function computeHoverCardLayout({
  tagCenterL,
  edgeMm = 20,
  viewerPosL = null,
  viewerRightL = null,
  viewerUpL = null,
  handedness = null,
  probePosL = null,
  avoidPosL = null,
  cardWidthM = HOVER_CALLOUT_DEFAULTS.cardWidthM,
  cardHeightM = HOVER_CALLOUT_DEFAULTS.cardHeightM,
  minTableZ = HOVER_CALLOUT_DEFAULTS.minTableZ,
}) {
  const { toEye, screenRight, screenUp } = viewBasisAtPointL(
    tagCenterL,
    viewerPosL,
    viewerRightL,
    viewerUpL,
  );

  // Move card away from the active hand: right hand -> left (-1), left hand -> right (+1).
  let sideSign = -1;
  if (handedness === 'left') {
    sideSign = 1;
  } else if (handedness === 'right') {
    sideSign = -1;
  } else if (probePosL) {
    const dp = [
      probePosL[0] - tagCenterL[0],
      probePosL[1] - tagCenterL[1],
      probePosL[2] - tagCenterL[2],
    ];
    sideSign = dot3(dp, screenRight) >= 0 ? -1 : 1;
  }

  const halfTag = (edgeMm / 1000) / 2;
  const sideDist = Math.max(0.088, halfTag + cardWidthM * 0.5 + 0.018);
  const upDist = Math.max(0.074, halfTag + cardHeightM * 0.5 + 0.022);

  const cardPos = [
    tagCenterL[0] + screenRight[0] * (sideSign * sideDist) + screenUp[0] * upDist,
    tagCenterL[1] + screenRight[1] * (sideSign * sideDist) + screenUp[1] * upDist,
    tagCenterL[2] + screenRight[2] * (sideSign * sideDist) + screenUp[2] * upDist,
  ];

  // Keep the card comfortably above the physical table plane (z = 0 in L-frame).
  if (cardPos[2] < minTableZ) {
    cardPos[2] = minTableZ;
  }

  // If hovering an already-pinned tag, lift clear of its compact pinned pill.
  if (avoidPosL) {
    const dAvoid = Math.hypot(
      cardPos[0] - avoidPosL[0],
      cardPos[1] - avoidPosL[1],
      cardPos[2] - avoidPosL[2],
    );
    if (dAvoid < 0.072) {
      const lift = 0.052;
      cardPos[0] += screenUp[0] * lift;
      cardPos[1] += screenUp[1] * lift;
      cardPos[2] = Math.max(minTableZ, cardPos[2] + screenUp[2] * lift);
    }
  }

  // Terminate the leader line cleanly at the card corner closest to the tag.
  const anchorPos = [
    cardPos[0] - screenRight[0] * (sideSign * cardWidthM * 0.44) - screenUp[0] * (cardHeightM * 0.44),
    cardPos[1] - screenRight[1] * (sideSign * cardWidthM * 0.44) - screenUp[1] * (cardHeightM * 0.44),
    cardPos[2] - screenRight[2] * (sideSign * cardWidthM * 0.44) - screenUp[2] * (cardHeightM * 0.44),
  ];

  return {
    cardPos,
    anchorPos,
    sideSign,
    screenRight,
    screenUp,
    toEye,
  };
}

/**
 * Compute compact 1-line pinned pill positions in L-frame with screen-space anti-overlap.
 */
export function computePinPillPositions(pins, {
  viewerPosL = null,
  viewerRightL = null,
  viewerUpL = null,
  pillWidthM = HOVER_CALLOUT_DEFAULTS.pillWidthM,
  pillHeightM = HOVER_CALLOUT_DEFAULTS.pillHeightM,
  minPillTableZ = HOVER_CALLOUT_DEFAULTS.minPillTableZ,
} = {}) {
  if (!pins || !pins.length) return new Map();

  const items = pins.map((pin) => {
    const basis = viewBasisAtPointL(pin.centerL, viewerPosL, viewerRightL, viewerUpL);
    const halfTag = ((pin.edgeMm || 20) / 1000) / 2;
    const upDist = Math.max(0.058, halfTag + pillHeightM * 0.5 + 0.026);
    const pos = [
      pin.centerL[0] + basis.screenUp[0] * upDist,
      pin.centerL[1] + basis.screenUp[1] * upDist,
      Math.max(minPillTableZ, pin.centerL[2] + basis.screenUp[2] * upDist),
    ];
    return { key: pin.key, centerL: pin.centerL, pos, basis };
  });

  // Gentle screen-space vertical/horizontal separation so nearby pinned pills never overlap.
  const minSepX = pillWidthM * 0.94;
  const minSepY = pillHeightM * 1.45;
  for (let iter = 0; iter < 4; iter++) {
    for (let i = 0; i < items.length; i++) {
      for (let j = i + 1; j < items.length; j++) {
        const a = items[i];
        const b = items[j];
        const diff = [b.pos[0] - a.pos[0], b.pos[1] - a.pos[1], b.pos[2] - a.pos[2]];
        const depth = Math.abs(dot3(diff, a.basis.toEye));
        if (depth > 0.32) continue;
        const dx = dot3(diff, a.basis.screenRight);
        const dy = dot3(diff, a.basis.screenUp);
        if (Math.abs(dx) < minSepX && Math.abs(dy) < minSepY) {
          const pushY = (minSepY - Math.abs(dy)) * 0.55;
          const dirY = dy >= 0 ? 1 : -1;
          a.pos[0] -= a.basis.screenUp[0] * dirY * pushY;
          a.pos[1] -= a.basis.screenUp[1] * dirY * pushY;
          a.pos[2] = Math.max(minPillTableZ, a.pos[2] - a.basis.screenUp[2] * dirY * pushY);
          b.pos[0] += a.basis.screenUp[0] * dirY * pushY;
          b.pos[1] += a.basis.screenUp[1] * dirY * pushY;
          b.pos[2] = Math.max(minPillTableZ, b.pos[2] + a.basis.screenUp[2] * dirY * pushY);
        }
      }
    }
  }

  const out = new Map();
  for (const it of items) {
    const { pos, basis } = it;
    const anchorPos = [
      pos[0] - basis.screenUp[0] * (pillHeightM * 0.46),
      pos[1] - basis.screenUp[1] * (pillHeightM * 0.46),
      pos[2] - basis.screenUp[2] * (pillHeightM * 0.46),
    ];
    const closePosL = [
      pos[0] + basis.screenRight[0] * (pillWidthM * 0.40),
      pos[1] + basis.screenRight[1] * (pillWidthM * 0.40),
      pos[2] + basis.screenRight[2] * (pillWidthM * 0.40),
    ];
    out.set(it.key, {
      pillPos: pos,
      anchorPos,
      closePosL,
      screenRight: basis.screenRight,
      screenUp: basis.screenUp,
    });
  }
  return out;
}
