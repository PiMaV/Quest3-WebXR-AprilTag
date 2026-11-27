// Dynamic clockwise table-perimeter fence driven by the fixed L-angle (350-352)
// and movable perimeter markers (358 YL, 353 TL, 357 XT, 354 TR, 359 YR, 355 BR, 356 XB).
// Starts idle until TL (353) is discovered ('offered'), then activates on user confirmation ('active').

export const FENCE_HEIGHT_M = 0.20;
export const PERIMETER_TRIGGER_TAG_ID = 353;
export const PERIMETER_OUTSIDE_HYSTERESIS_M = Object.freeze({
  enterOutsideM: 0.008,
  exitOutsideM: 0.002,
});

export const PERIMETER_MARKERS = Object.freeze([
  Object.freeze({ id: 358, role: 'YL' }),
  Object.freeze({ id: 353, role: 'TL' }),
  Object.freeze({ id: 357, role: 'XT' }),
  Object.freeze({ id: 354, role: 'TR' }),
  Object.freeze({ id: 359, role: 'YR' }),
  Object.freeze({ id: 355, role: 'BR' }),
  Object.freeze({ id: 356, role: 'XB' }),
]);

const PERIMETER_BY_ID = new Map(PERIMETER_MARKERS.map((m) => [m.id, m]));

export function isPerimeterTag(id) {
  return PERIMETER_BY_ID.has(id);
}

export function perimeterRole(id) {
  return PERIMETER_BY_ID.get(id)?.role ?? null;
}

function tagCenterXY(tag) {
  const c = tag.corners;
  let sx = 0;
  let sy = 0;
  for (let i = 0; i < c.length; i++) {
    sx += c[i][0];
    sy += c[i][1];
  }
  return [sx / c.length, sy / c.length, 0];
}

export function extractRigAnchors(rig) {
  const tags = rig?.tools?.[0]?.tags;
  if (!Array.isArray(tags) || tags.length < 3) {
    return {
      origin: { id: 350, role: 'BL', fixed: true, pos: [0.0175, 0.015, 0] },
      xArm: { id: 351, role: '+X', fixed: true, pos: [0.2475, 0.015, 0] },
      yArm: { id: 352, role: '+Y', fixed: true, pos: [0.0175, 0.185, 0] },
    };
  }
  return {
    origin: { id: tags[0].id, role: 'BL', fixed: true, pos: tagCenterXY(tags[0]) },
    xArm: { id: tags[1].id, role: '+X', fixed: true, pos: tagCenterXY(tags[1]) },
    yArm: { id: tags[2].id, role: '+Y', fixed: true, pos: tagCenterXY(tags[2]) },
  };
}

// Intersects the camera-to-tag ray with the L-angle table plane (z = 0) when the
// tag sits close to the table. Eliminates horizontal XY shift caused by single-tag
// depth noise while safely falling back to orthographic projection on grazing views.
export function projectPointToTablePlane(tagPosL, cameraPosL = null, {
  maxTableOffsetM = 0.12,
  minCameraHeightM = 0.12,
  minRayDropM = 0.08,
  maxHorizontalShiftM = 0.20,
} = {}) {
  const [tx, ty, tz] = tagPosL;
  if (
    Array.isArray(cameraPosL) &&
    cameraPosL.length >= 3 &&
    Number.isFinite(cameraPosL[0]) &&
    Number.isFinite(cameraPosL[1]) &&
    Number.isFinite(cameraPosL[2]) &&
    Math.abs(tz) <= maxTableOffsetM &&
    cameraPosL[2] >= minCameraHeightM &&
    cameraPosL[2] - tz >= minRayDropM
  ) {
    const t = cameraPosL[2] / (cameraPosL[2] - tz);
    const px = cameraPosL[0] + t * (tx - cameraPosL[0]);
    const py = cameraPosL[1] + t * (ty - cameraPosL[1]);
    if (Number.isFinite(px) && Number.isFinite(py) && Math.hypot(px - tx, py - ty) <= maxHorizontalShiftM) {
      return [px, py, 0];
    }
  }
  return [tx, ty, 0];
}

function extractTableYaw(lFromTag) {
  const x = lFromTag[0];
  const y = lFromTag[1];
  if (Number.isFinite(x) && Number.isFinite(y) && Math.hypot(x, y) > 1e-4) {
    return Math.atan2(y, x);
  }
  return 0;
}

function shortestAngleDelta(from, to) {
  let d = (to - from) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d <= -Math.PI) d += Math.PI * 2;
  return d;
}

function composeTableTagMatrix(pos, yaw) {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return [
    c, s, 0, 0,
    -s, c, 0, 0,
    0, 0, 1, 0,
    pos[0], pos[1], pos[2], 1,
  ];
}

export function polygonMetricsXY(vertices) {
  const n = vertices.length;
  if (n < 2) return { perimeterM: 0, areaM2: 0 };
  let perimeterM = 0;
  let signedArea2 = 0;
  for (let i = 0; i < n; i++) {
    const a = vertices[i].pos;
    const b = vertices[(i + 1) % n].pos;
    perimeterM += Math.hypot(b[0] - a[0], b[1] - a[1]);
    signedArea2 += a[0] * b[1] - b[0] * a[1];
  }
  return {
    perimeterM,
    areaM2: n >= 3 ? Math.abs(signedArea2) * 0.5 : 0,
  };
}

/**
 * Evaluate exact 2D XY point-in-polygon membership and signed distance to the polygon
 * boundary (negative inside, positive outside) using ray-casting + segment distance.
 */
export function signedDistanceToPolygonXY(x, y, vertices) {
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Array.isArray(vertices) || vertices.length < 3) {
    return { inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  const n = vertices.length;
  let inside = false;
  let minDistSq = Infinity;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const va = vertices[j]?.pos ?? vertices[j];
    const vb = vertices[i]?.pos ?? vertices[i];
    if (!va || !vb) continue;
    const ax = va[0], ay = va[1];
    const bx = vb[0], by = vb[1];
    if (!Number.isFinite(ax) || !Number.isFinite(ay) || !Number.isFinite(bx) || !Number.isFinite(by)) continue;

    if (((ay > y) !== (by > y)) && (x < ((bx - ax) * (y - ay)) / (by - ay) + ax)) {
      inside = !inside;
    }

    const abx = bx - ax;
    const aby = by - ay;
    const abLenSq = abx * abx + aby * aby;
    let px = ax;
    let py = ay;
    if (abLenSq > 1e-12) {
      const t = Math.max(0, Math.min(1, ((x - ax) * abx + (y - ay) * aby) / abLenSq));
      px = ax + t * abx;
      py = ay + t * aby;
    }
    const dx = x - px;
    const dy = y - py;
    const dSq = dx * dx + dy * dy;
    if (dSq < minDistSq) minDistSq = dSq;
  }
  const edgeDistanceM = Number.isFinite(minDistSq) ? Math.sqrt(minDistSq) : 0;
  if (edgeDistanceM <= 1e-9) {
    return { inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  return {
    inside,
    signedDistanceM: inside ? -edgeDistanceM : edgeDistanceM,
    edgeDistanceM,
  };
}

/**
 * Evaluate whether a 3D L-frame tag center lies outside the XY perimeter polygon,
 * applying hysteresis (enterOutsideM / exitOutsideM) to prevent boundary flicker.
 */
export function evaluatePerimeterOutside(posL, vertices, {
  wasOutside = false,
  enterOutsideM = PERIMETER_OUTSIDE_HYSTERESIS_M.enterOutsideM,
  exitOutsideM = PERIMETER_OUTSIDE_HYSTERESIS_M.exitOutsideM,
} = {}) {
  const x = posL?.[0];
  const y = posL?.[1];
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Array.isArray(vertices) || vertices.length < 3) {
    return { outside: false, inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  const res = signedDistanceToPolygonXY(x, y, vertices);
  const threshold = wasOutside ? exitOutsideM : enterOutsideM;
  const outside = res.signedDistanceM > threshold;
  return {
    outside,
    inside: !outside,
    signedDistanceM: res.signedDistanceM,
    edgeDistanceM: res.edgeDistanceM,
  };
}

export function isTablePointObservableInCamera(posL, lFromCamera, K, {
  minDistM = 0.22,
  maxDistM = 1.30,
  minCosView = 0.50,
  borderMarginFrac = 0.10,
  minBorderMarginPx = 72,
  deadZoneM = 0.04,
  minExpectedMinorPx = 11,
  decimate = 1,
  tagEdgeM = 0.020,
} = {}) {
  if (
    !Array.isArray(posL) || posL.length < 3 ||
    !lFromCamera || lFromCamera.length < 15 ||
    !K || !(K.fx > 0) || !(K.fy > 0) ||
    !Number.isFinite(K.cx) || !Number.isFinite(K.cy) ||
    !(K.width >= 32) || !(K.height >= 32)
  ) {
    return false;
  }
  const px = posL[0], py = posL[1], pz = posL[2];
  const tx = lFromCamera[12], ty = lFromCamera[13], tz = lFromCamera[14];
  if (
    !Number.isFinite(px) || !Number.isFinite(py) || !Number.isFinite(pz) ||
    !Number.isFinite(tx) || !Number.isFinite(ty) || !Number.isFinite(tz)
  ) {
    return false;
  }
  const dx = px - tx;
  const dy = py - ty;
  const dz = pz - tz;
  const dist = Math.hypot(dx, dy, dz);
  if (!Number.isFinite(dist) || dist < minDistM || dist > maxDistM) return false;

  const cosView = (tz - pz) / dist;
  if (!Number.isFinite(cosView) || cosView < minCosView) return false;

  // Rigid transpose inverse: p_GL = R^T * (p_L - t), then GL -> OpenCV (x, -y, -z).
  const xCV = lFromCamera[0] * dx + lFromCamera[1] * dy + lFromCamera[2] * dz;
  const yCV = -(lFromCamera[4] * dx + lFromCamera[5] * dy + lFromCamera[6] * dz);
  const zCV = -(lFromCamera[8] * dx + lFromCamera[9] * dy + lFromCamera[10] * dz);
  if (!Number.isFinite(zCV) || zCV < minDistM || zCV > maxDistM) return false;

  // Check foreshortened minor axis AFTER detector decimation so far or oblique tags
  // that AprilTag cannot reliably decode on this frame are never falsely pruned.
  const effDecimate = Number.isFinite(decimate) && decimate >= 1 ? decimate : 1;
  const expectedEdgePx = (tagEdgeM * Math.min(K.fx, K.fy)) / zCV;
  const expectedMinorPx = (expectedEdgePx * cosView) / effDecimate;
  if (!Number.isFinite(expectedMinorPx) || expectedMinorPx < minExpectedMinorPx) return false;

  const u = K.fx * (xCV / zCV) + K.cx;
  const v = K.fy * (yCV / zCV) + K.cy;
  if (!Number.isFinite(u) || !Number.isFinite(v)) return false;

  // Include a spatial dead-zone around the expected marker location on the table
  // so small XR drift or edge clipping cannot trigger false removal.
  const deadZonePx = Number.isFinite(deadZoneM) && deadZoneM > 0
    ? (deadZoneM * Math.max(K.fx, K.fy)) / zCV
    : 0;
  const marginX = Math.max(minBorderMarginPx, K.width * borderMarginFrac) + deadZonePx;
  const marginY = Math.max(minBorderMarginPx, K.height * borderMarginFrac) + deadZonePx;
  return u >= marginX && u <= K.width - marginX && v >= marginY && v <= K.height - marginY;
}

export function createPerimeterFence(rig, {
  heightM = FENCE_HEIGHT_M,
  triggerTagId = PERIMETER_TRIGGER_TAG_ID,
  smoothTauSec = 0.065,
  jumpDistanceM = 0.15,
  jumpWindowMs = 500,
  jumpConfirmM = 0.06,
  missingFramesToRemove = 10,
  missingMsToRemove = 3500,
  minGraceAfterSeenMs = 500,
  minRemoveConfidence = 0.65,
  inViewMinDistM = 0.22,
  inViewMaxDistM = 1.30,
  inViewMinCosView = 0.50,
  inViewBorderMarginFrac = 0.10,
  inViewMinBorderMarginPx = 72,
  inViewDeadZoneM = 0.04,
  inViewMinExpectedMinorPx = 11,
  tagEdgeM = 0.020,
} = {}) {
  let anchors = extractRigAnchors(rig);
  const markers = new Map();
  let active = false;
  let dirty = true;
  let stateVersion = 0;
  const cachedPolyXY = [];
  let cachedPolyVer = -1;

  function isActive() {
    return active;
  }

  function isOffered() {
    return !active && markers.has(triggerTagId);
  }

  function status() {
    if (active) return 'active';
    if (markers.has(triggerTagId)) return 'offered';
    return 'idle';
  }

  function triggerMarker() {
    const m = markers.get(triggerTagId);
    if (!m) return null;
    return {
      id: m.id,
      role: m.role,
      pos: m.currentPos.slice(),
      yaw: m.currentYaw,
      matrix: composeTableTagMatrix(m.currentPos, m.currentYaw),
      lastAt: m.lastAt,
    };
  }

  function activate({ force = false } = {}) {
    if (active) return false;
    if (!force && !markers.has(triggerTagId)) return false;
    active = true;
    dirty = true;
    stateVersion++;
    return true;
  }

  function deactivate() {
    if (!active) return false;
    active = false;
    dirty = true;
    stateVersion++;
    return true;
  }

  function setRig(nextRig) {
    anchors = extractRigAnchors(nextRig);
    dirty = true;
    stateVersion++;
  }

  function reset() {
    markers.clear();
    active = false;
    dirty = true;
    stateVersion++;
  }

  function observe(tagId, lFromTag, cameraPosL, atMs, { weight = 1 } = {}) {
    const spec = PERIMETER_BY_ID.get(tagId);
    if (!spec || !lFromTag || lFromTag.length < 15) return null;
    const rawPos = [lFromTag[12], lFromTag[13], lFromTag[14]];
    if (!rawPos.every(Number.isFinite)) return null;
    const pos = projectPointToTablePlane(rawPos, cameraPosL);
    const yaw = extractTableYaw(lFromTag);
    const w = Number.isFinite(weight) ? Math.max(0, Math.min(1, weight)) : 1;
    const existing = markers.get(tagId);
    if (existing) {
      existing.missCount = 0;
      existing.missSinceMs = null;
    }
    if (w <= 0) {
      return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-frozen' };
    }
    const wasOffered = isOffered();

    if (!existing) {
      if (w < 0.45) {
        return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-low-confidence' };
      }
      markers.set(tagId, {
        id: tagId,
        role: spec.role,
        fixed: false,
        targetPos: pos.slice(),
        currentPos: pos.slice(),
        targetYaw: yaw,
        currentYaw: yaw,
        lastAt: atMs,
        pendingJump: null,
        updates: 1,
        missCount: 0,
        missSinceMs: null,
      });
      dirty = true;
      stateVersion++;
      return {
        accepted: true,
        isNew: true,
        justOffered: !wasOffered && isOffered(),
        status: status(),
        id: tagId,
        role: spec.role,
        pos,
      };
    }

    const dist = Math.hypot(pos[0] - existing.targetPos[0], pos[1] - existing.targetPos[1]);
    if (dist > jumpDistanceM && atMs - existing.lastAt < jumpWindowMs) {
      if (
        !existing.pendingJump ||
        Math.hypot(pos[0] - existing.pendingJump[0], pos[1] - existing.pendingJump[1]) > jumpConfirmM
      ) {
        existing.pendingJump = pos.slice();
        return {
          accepted: false,
          isNew: false,
          justOffered: false,
          status: status(),
          id: tagId,
          role: spec.role,
          pos,
        };
      }
    }

    if (w < 0.6 && dist < 0.008) {
      existing.lastAt = atMs;
      return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-deadband' };
    }

    existing.pendingJump = null;
    if (w >= 0.999) {
      existing.targetPos[0] = pos[0];
      existing.targetPos[1] = pos[1];
      existing.targetYaw = yaw;
    } else {
      existing.targetPos[0] += (pos[0] - existing.targetPos[0]) * w;
      existing.targetPos[1] += (pos[1] - existing.targetPos[1]) * w;
      existing.targetYaw = shortestAngleDelta(0, existing.targetYaw + shortestAngleDelta(existing.targetYaw, yaw) * w);
    }
    existing.targetPos[2] = 0;
    existing.lastAt = atMs;
    existing.updates++;
    dirty = true;
    stateVersion++;
    return {
      accepted: true,
      isNew: false,
      justOffered: false,
      status: status(),
      id: tagId,
      role: spec.role,
      pos,
    };
  }

  function remove(tagId) {
    const m = markers.get(tagId);
    if (!m) return null;
    markers.delete(tagId);
    let deactivated = false;
    if (active && markers.size === 0) {
      active = false;
      deactivated = true;
    }
    dirty = true;
    stateVersion++;
    return {
      removed: true,
      id: m.id,
      role: m.role,
      pos: m.currentPos.slice(),
      deactivated,
      status: status(),
    };
  }

  const inViewOpts = {
    minDistM: inViewMinDistM,
    maxDistM: inViewMaxDistM,
    minCosView: inViewMinCosView,
    borderMarginFrac: inViewBorderMarginFrac,
    minBorderMarginPx: inViewMinBorderMarginPx,
    deadZoneM: inViewDeadZoneM,
    minExpectedMinorPx: inViewMinExpectedMinorPx,
    decimate: 1,
    tagEdgeM,
  };

  function reconcileView({
    seenIds = [],
    lFromCamera = null,
    K = null,
    atMs = NaN,
    confidence = 1,
    occludedIds = null,
    decimate = 1,
  } = {}) {
    if (markers.size === 0) {
      return { removed: [], deactivated: false, status: status() };
    }
    const seenSet = seenIds instanceof Set ? seenIds : new Set(seenIds);
    const occludedSet = occludedIds
      ? (occludedIds instanceof Set ? occludedIds : new Set(occludedIds))
      : null;

    if (
      !(confidence >= minRemoveConfidence) ||
      !lFromCamera ||
      !K ||
      !Number.isFinite(atMs)
    ) {
      for (const id of seenSet) {
        const m = markers.get(id);
        if (m) {
          m.missCount = 0;
          m.missSinceMs = null;
        }
      }
      return { removed: [], deactivated: false, status: status() };
    }

    inViewOpts.decimate = Number.isFinite(decimate) && decimate >= 1 ? decimate : 1;
    const toRemove = [];
    for (const [id, m] of markers) {
      if (seenSet.has(id) || occludedSet?.has(id)) {
        m.missCount = 0;
        m.missSinceMs = null;
        continue;
      }
      if (!isTablePointObservableInCamera(m.currentPos, lFromCamera, K, inViewOpts)) {
        m.missCount = 0;
        m.missSinceMs = null;
        continue;
      }
      if (m.lastAt > 0 && atMs - m.lastAt < minGraceAfterSeenMs) {
        continue;
      }
      if (m.missSinceMs === null) {
        m.missSinceMs = atMs;
      }
      m.missCount++;
      if (m.missCount >= missingFramesToRemove && atMs - m.missSinceMs >= missingMsToRemove) {
        toRemove.push(id);
      }
    }

    if (toRemove.length === 0) {
      return { removed: [], deactivated: false, status: status() };
    }

    const removed = [];
    for (const id of toRemove) {
      const m = markers.get(id);
      if (!m) continue;
      markers.delete(id);
      removed.push({ id: m.id, role: m.role, pos: m.currentPos.slice() });
    }
    let deactivated = false;
    if (active && markers.size === 0) {
      active = false;
      deactivated = true;
    }
    dirty = true;
    stateVersion++;
    return { removed, deactivated, status: status() };
  }

  function step(dtSec) {
    const wasDirty = dirty;
    dirty = false;
    let moved = false;
    const dt = Number.isFinite(dtSec) && dtSec > 0 ? Math.min(dtSec, 0.25) : 0;
    const alpha = smoothTauSec > 0 && dt > 0 ? 1 - Math.exp(-dt / smoothTauSec) : 1;

    for (const m of markers.values()) {
      const dx = m.targetPos[0] - m.currentPos[0];
      const dy = m.targetPos[1] - m.currentPos[1];
      const dist = Math.hypot(dx, dy);
      if (dist <= 0.0004) {
        if (dx !== 0 || dy !== 0) {
          m.currentPos[0] = m.targetPos[0];
          m.currentPos[1] = m.targetPos[1];
          moved = true;
        }
      } else {
        m.currentPos[0] += dx * alpha;
        m.currentPos[1] += dy * alpha;
        moved = true;
      }

      const dYaw = shortestAngleDelta(m.currentYaw, m.targetYaw);
      if (Math.abs(dYaw) <= 0.002) {
        if (dYaw !== 0) {
          m.currentYaw = m.targetYaw;
          moved = true;
        }
      } else {
        m.currentYaw = shortestAngleDelta(0, m.currentYaw + dYaw * alpha);
        moved = true;
      }
    }
    if (moved) stateVersion++;
    return wasDirty || moved;
  }

  function serializeMarker(m) {
    return {
      id: m.id,
      role: m.role,
      fixed: false,
      pos: m.currentPos.slice(),
      yaw: m.currentYaw,
      matrix: composeTableTagMatrix(m.currentPos, m.currentYaw),
      lastAt: m.lastAt,
      updates: m.updates,
    };
  }

  // Strictly clockwise around the table viewed from +Z:
  // 350 (BL) -> 352 (+Y) -> 358 (YL) -> 353 (TL) -> 357 (XT) -> 354 (TR)
  // -> 359 (YR) -> 355 (BR) -> 356 (XB) -> 351 (+X) -> back to 350 (BL).
  function orderedVertices() {
    const verts = [
      { id: anchors.origin.id, role: anchors.origin.role, fixed: true, pos: anchors.origin.pos, lastAt: 0, updates: 0 },
      { id: anchors.yArm.id, role: anchors.yArm.role, fixed: true, pos: anchors.yArm.pos, lastAt: 0, updates: 0 },
    ];
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) verts.push(serializeMarker(m));
    }
    verts.push({
      id: anchors.xArm.id,
      role: anchors.xArm.role,
      fixed: true,
      pos: anchors.xArm.pos,
      lastAt: 0,
      updates: 0,
    });
    return verts;
  }

  function discoveredMarkers() {
    const list = [];
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) list.push(serializeMarker(m));
    }
    return list;
  }

  function dynamicCount() {
    return markers.size;
  }

  function syncCachedPolyXY() {
    if (cachedPolyVer === stateVersion) return cachedPolyXY;
    cachedPolyXY.length = 0;
    cachedPolyXY.push(anchors.origin.pos, anchors.yArm.pos);
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) cachedPolyXY.push(m.currentPos);
    }
    cachedPolyXY.push(anchors.xArm.pos);
    cachedPolyVer = stateVersion;
    return cachedPolyXY;
  }

  function isOutsideXY(posL, opts = {}) {
    if (!active) {
      return { active: false, outside: false, inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
    }
    const res = evaluatePerimeterOutside(posL, syncCachedPolyXY(), opts);
    return { active: true, ...res };
  }

  function snapshot() {
    const verts = orderedVertices();
    const { perimeterM, areaM2 } = polygonMetricsXY(verts);
    return {
      status: status(),
      active,
      offered: isOffered(),
      triggerTagId,
      heightM,
      dynamicCount: markers.size,
      totalDynamicMarkers: PERIMETER_MARKERS.length,
      perimeterM,
      areaM2,
      vertices: verts.map((v) => ({
        id: v.id,
        role: v.role,
        fixed: v.fixed,
        posL: v.pos.slice(),
        updates: v.updates,
      })),
    };
  }

  return {
    heightM,
    triggerTagId,
    isActive,
    isOffered,
    status,
    triggerMarker,
    activate,
    deactivate,
    setRig,
    reset,
    observe,
    remove,
    reconcileView,
    step,
    orderedVertices,
    discoveredMarkers,
    dynamicCount,
    isOutsideXY,
    version: () => stateVersion,
    snapshot,
  };
}
