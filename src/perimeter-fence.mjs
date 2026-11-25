// Dynamic clockwise table-perimeter fence driven by the fixed L-angle (350-352)
// and movable perimeter markers (358 YL, 353 TL, 357 XT, 354 TR, 359 YR, 355 BR, 356 XB).
// Starts idle until TL (353) is discovered ('offered'), then activates on user confirmation ('active').

export const FENCE_HEIGHT_M = 0.20;
export const PERIMETER_TRIGGER_TAG_ID = 353;

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

export function createPerimeterFence(rig, {
  heightM = FENCE_HEIGHT_M,
  triggerTagId = PERIMETER_TRIGGER_TAG_ID,
  smoothTauSec = 0.065,
  jumpDistanceM = 0.15,
  jumpWindowMs = 500,
  jumpConfirmM = 0.06,
} = {}) {
  let anchors = extractRigAnchors(rig);
  const markers = new Map();
  let active = false;
  let dirty = true;
  let stateVersion = 0;

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
    if (w <= 0) {
      return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-frozen' };
    }
    const existing = markers.get(tagId);
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
    step,
    orderedVertices,
    discoveredMarkers,
    dynamicCount,
    version: () => stateVersion,
    snapshot,
  };
}
