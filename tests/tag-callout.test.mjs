import test from 'node:test';
import assert from 'node:assert/strict';
import {
  viewBasisAtPointL,
  computeHoverCardLayout,
  computePinPillPositions,
  HOVER_CALLOUT_DEFAULTS,
} from '../src/tag-callout.mjs';

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

test('looking down at a table AprilTag places hover card strictly clear of line of sight and pinching hand', () => {
  // User standing in front of table looking down at ~60 deg pitch at tag on table (z=0).
  const tagCenterL = [0.20, 0.15, 0.0];
  const viewerPosL = [0.20, -0.22, 0.55];
  const viewerRightL = [1, 0, 0];
  const viewerUpL = [0, 0.83, 0.55];

  const rightHand = computeHoverCardLayout({
    tagCenterL,
    edgeMm: 80,
    viewerPosL,
    viewerRightL,
    viewerUpL,
    handedness: 'right',
  });

  // Right hand -> card shifts LEFT (-screenRight) and screen-UP (away across the table, NOT straight toward eyes).
  assert.equal(rightHand.sideSign, -1);
  const offset = sub(rightHand.cardPos, tagCenterL);
  const alongRight = dot(offset, rightHand.screenRight);
  const alongUp = dot(offset, rightHand.screenUp);
  const alongEye = dot(offset, rightHand.toEye);

  // Card inner edge stays clear of the 80 mm tag border (half=40 mm) + card half-width/height.
  assert.ok(alongRight <= -(0.040 + HOVER_CALLOUT_DEFAULTS.cardWidthM * 0.5), 'card should clear tag horizontally to the left');
  assert.ok(alongUp >= (0.040 + HOVER_CALLOUT_DEFAULTS.cardHeightM * 0.5), 'card should clear tag vertically in screen space');
  assert.ok(Math.abs(alongEye) < 0.02, 'card should stay nearly orthogonal to line of sight instead of floating toward eyes');
  assert.ok(rightHand.cardPos[2] >= HOVER_CALLOUT_DEFAULTS.minTableZ, 'card must stay above the table plane');

  // Leader anchor sits closer to the tag than the card center.
  const dCenter = Math.hypot(...sub(rightHand.cardPos, tagCenterL));
  const dAnchor = Math.hypot(...sub(rightHand.anchorPos, tagCenterL));
  assert.ok(dAnchor < dCenter, 'leader line must terminate at the inner corner closest to the tag');

  // Left hand -> card shifts RIGHT (+screenRight).
  const leftHand = computeHoverCardLayout({
    tagCenterL,
    edgeMm: 20,
    viewerPosL,
    viewerRightL,
    viewerUpL,
    handedness: 'left',
  });
  assert.equal(leftHand.sideSign, 1);
  assert.ok(dot(sub(leftHand.cardPos, tagCenterL), leftHand.screenRight) > 0.08);
});

test('hovering an already-pinned tag keeps hover detail card clear of its compact pinned pill', () => {
  const tagCenterL = [0.10, 0.10, 0.05];
  const viewerPosL = [0.10, -0.40, 0.55];
  const viewerRightL = [1, 0, 0];
  const viewerUpL = [0, 0.7, 0.7];

  const base = computeHoverCardLayout({
    tagCenterL,
    edgeMm: 20,
    viewerPosL,
    viewerRightL,
    viewerUpL,
    handedness: 'right',
  });
  // Force an obstacle right at the base card position.
  const avoided = computeHoverCardLayout({
    tagCenterL,
    edgeMm: 20,
    viewerPosL,
    viewerRightL,
    viewerUpL,
    handedness: 'right',
    avoidPosL: base.cardPos,
  });
  const sep = Math.hypot(...sub(avoided.cardPos, base.cardPos));
  assert.ok(sep >= 0.045, 'hover card should lift cleanly away from a nearby pinned pill');
});

test('computePinPillPositions separates closely spaced pinned tags so pills do not overlap', () => {
  const viewerPosL = [0.2, -0.45, 0.55];
  const viewerRightL = [1, 0, 0];
  const viewerUpL = [0, 0.75, 0.66];
  const pins = [
    { key: '301:1', centerL: [0.20, 0.10, 0.00], edgeMm: 20, isStatic: false },
    { key: '301:2', centerL: [0.21, 0.105, 0.00], edgeMm: 20, isStatic: false },
  ];

  const res = computePinPillPositions(pins, { viewerPosL, viewerRightL, viewerUpL });
  const a = res.get('301:1');
  const b = res.get('301:2');
  assert.ok(a && b);

  const basis = viewBasisAtPointL(pins[0].centerL, viewerPosL, viewerRightL, viewerUpL);
  const sepY = Math.abs(dot(sub(b.pillPos, a.pillPos), basis.screenUp));
  assert.ok(sepY >= HOVER_CALLOUT_DEFAULTS.pillHeightM * 1.1, 'nearby pinned pills should separate in screen space');

  // Close orb sits on the right end of each pill.
  const closeOffsetA = dot(sub(a.closePosL, a.pillPos), a.screenRight);
  assert.ok(Math.abs(closeOffsetA - HOVER_CALLOUT_DEFAULTS.pillWidthM * 0.40) < 1e-6);
});
