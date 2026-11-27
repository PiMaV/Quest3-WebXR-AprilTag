// Gaze-cone 3-class Focus-ROI scheduler (Single-Focus / Multi-Focus / Wide) with
// anti-thrashing hysteresis and adaptive 1-3 Hz background sweeps.
import { projectPriorRois } from './detector.mjs';
import { inverse, product } from './runtime.mjs';

export const FOCUS_SCHEDULER_DEFAULTS = Object.freeze({
  coneEnterDeg: 26,
  coneExitDeg: 32,
  coneMaxDistM: 1.85,
  coneFarMaxDistM: 2.50,
  maxForegroundTags: 3,
  lostInConeHoldMs: 1800,
  unstableLostHoldMs: 700,
  stablePromoteMs: 450,
  outOfConeHoldMs: 500,
  restingDemoteMs: 2500,
  freshPromoteAgeMs: 650,
  backgroundSweepMinMs: 360,
  backgroundSweepNomMs: 550,
  backgroundSweepFastMs: 950,
  fastSpeedThresholdMps: 0.25,
  headroomBudgetMs: 22,
  partialMissRecoveryLimit: 3,
  primaryStickyBonusDeg: 3,
});

const isFiniteMat4 = m => Boolean(
  m && typeof m.length === 'number' && m.length === 16 && Array.prototype.every.call(m, Number.isFinite),
);

/**
 * Project an L-frame 3D position into camera pixel coordinates, depth, and angular
 * eccentricity (deg) relative to the optical axis.
 */
export function projectTagGazeMetrics(posL, camGLFromL, K, width, height) {
  if (
    !Array.isArray(posL) || posL.length < 3 ||
    !Number.isFinite(posL[0]) || !Number.isFinite(posL[1]) || !Number.isFinite(posL[2]) ||
    !isFiniteMat4(camGLFromL) || !K || !(K.fx > 0) || !(K.fy > 0) ||
    !Number.isFinite(K.cx) || !Number.isFinite(K.cy) || !(width >= 32) || !(height >= 32)
  ) {
    return { inFrustum: false, angleDeg: Infinity, distM: Infinity, u: NaN, v: NaN, zCV: NaN };
  }
  const px = posL[0], py = posL[1], pz = posL[2];
  const xGL = camGLFromL[0] * px + camGLFromL[4] * py + camGLFromL[8] * pz + camGLFromL[12];
  const yGL = camGLFromL[1] * px + camGLFromL[5] * py + camGLFromL[9] * pz + camGLFromL[13];
  const zGL = camGLFromL[2] * px + camGLFromL[6] * py + camGLFromL[10] * pz + camGLFromL[14];
  const xCV = xGL;
  const yCV = -yGL;
  const zCV = -zGL;
  if (!Number.isFinite(zCV) || zCV <= 0.05) {
    return { inFrustum: false, angleDeg: Infinity, distM: Infinity, u: NaN, v: NaN, zCV: Number.isFinite(zCV) ? zCV : NaN };
  }
  const k1 = Number.isFinite(K.k1) ? K.k1 : 0;
  const xn = xCV / zCV, yn = yCV / zCV;
  const sc = k1 ? 1 + k1 * (xn * xn + yn * yn) : 1;
  const u = K.fx * xn * sc + K.cx;
  const v = K.fy * yn * sc + K.cy;
  const distM = Math.hypot(xCV, yCV, zCV);
  const angleDeg = (Math.atan2(Math.hypot(xCV, yCV), zCV) * 180) / Math.PI;
  const inFrustum = Number.isFinite(u) && Number.isFinite(v) &&
    u >= -48 && u <= width + 48 && v >= -48 && v <= height + 48;
  return { inFrustum, angleDeg, distM, u, v, zCV };
}

/**
 * Stateful gaze-cone foreground/background classifier and Focus-ROI scheduler.
 * Distinguishes 1st-class foreground tags (inside the central gaze cone or active)
 * from 2nd-class background tags while preventing single-frame dropout thrashing.
 */
export function createFocusScheduler(options = {}) {
  const cfg = { ...FOCUS_SCHEDULER_DEFAULTS, ...options };
  const slots = new Map();
  let lastSweepAtMs = 0;
  let primaryTagId = null;
  let lastMode = 'wide';
  let partialMissStreak = 0;

  function reset() {
    slots.clear();
    lastSweepAtMs = 0;
    primaryTagId = null;
    lastMode = 'wide';
    partialMissStreak = 0;
  }

  function wideResult(skipReason, nowMs) {
    lastMode = 'wide';
    primaryTagId = null;
    if (Number.isFinite(nowMs)) lastSweepAtMs = nowMs;
    return {
      mode: 'wide',
      rois: undefined,
      tagIds: [],
      activeTagIds: [],
      searchingTagIds: [],
      speedFirst: false,
      isBackgroundSweep: false,
      sweepDecimate: 2,
      skipReason,
    };
  }

  function plan({
    candidates,
    worldFromAnchor,
    worldFromViewer,
    viewerFromCameraGL,
    K,
    width,
    height,
    nowMs = performance.now(),
    farMode = false,
    recentDetectMs = 15,
    maxForegroundTags = cfg.maxForegroundTags,
  } = {}) {
    if (!K || !(K.fx > 0) || !(K.fy > 0) || !(width >= 32) || !(height >= 32)) {
      return wideResult('no-camera', nowMs);
    }
    if (!isFiniteMat4(worldFromAnchor) || !isFiniteMat4(worldFromViewer) || !isFiniteMat4(viewerFromCameraGL)) {
      return wideResult('no-xr-pose', nowMs);
    }
    if (!Array.isArray(candidates) || candidates.length === 0) {
      slots.clear();
      return wideResult('no-candidates', nowMs);
    }

    const maxFg = Number.isInteger(maxForegroundTags) && maxForegroundTags >= 1
      ? maxForegroundTags
      : cfg.maxForegroundTags;

    const lFromViewer = product(inverse(worldFromAnchor), worldFromViewer);
    const viewerGLFromL = inverse(lFromViewer);
    const lFromCameraGL = product(lFromViewer, viewerFromCameraGL);
    const camGLFromL = inverse(lFromCameraGL);
    if (!isFiniteMat4(camGLFromL) || !isFiniteMat4(viewerGLFromL)) {
      return wideResult('no-xr-pose', nowMs);
    }

    const maxDistM = farMode ? cfg.coneFarMaxDistM : cfg.coneMaxDistM;
    const candidateById = new Map();
    const metricsById = new Map();
    let hasActive = false;

    for (const c of candidates) {
      if (!c || !Number.isInteger(c.tagId) || !Array.isArray(c.pos) || c.pos.length < 3) continue;
      candidateById.set(c.tagId, c);
      const m = projectTagGazeMetrics(c.pos, camGLFromL, K, width, height);
      // Measure gaze-cone angle symmetrically around the viewer's head axis (worldFromViewer)
      // rather than the laterally offset Quest 3 RGB camera (~3.3 cm right), while keeping
      // frustum/pixel projection strictly in the physical camera frame.
      if (m.inFrustum) {
        const px = c.pos[0], py = c.pos[1], pz = c.pos[2];
        const vx = viewerGLFromL[0] * px + viewerGLFromL[4] * py + viewerGLFromL[8] * pz + viewerGLFromL[12];
        const vy = viewerGLFromL[1] * px + viewerGLFromL[5] * py + viewerGLFromL[9] * pz + viewerGLFromL[13];
        const vz = -(viewerGLFromL[2] * px + viewerGLFromL[6] * py + viewerGLFromL[10] * pz + viewerGLFromL[14]);
        if (Number.isFinite(vz) && vz > 0.05) {
          m.angleDeg = (Math.atan2(Math.hypot(vx, vy), vz) * 180) / Math.PI;
          m.distM = Math.hypot(vx, vy, vz);
        }
      }
      metricsById.set(c.tagId, m);
      if (c.recording || c.handCoupled || c.moving || c.nearHand) hasActive = true;
    }

    for (const tagId of slots.keys()) {
      if (!candidateById.has(tagId)) slots.delete(tagId);
    }

    const hasOtherActiveInCenter = (excludeTagId) => {
      for (const [id, c] of candidateById) {
        if (id === excludeTagId) continue;
        if (!(c.recording || c.handCoupled || c.moving || c.nearHand)) continue;
        const m = metricsById.get(id);
        if (!m || !m.inFrustum || m.distM > maxDistM) continue;
        if (m.angleDeg <= cfg.coneEnterDeg || (slots.has(id) && m.angleDeg <= cfg.coneExitDeg)) {
          return true;
        }
      }
      return false;
    };

    for (const [tagId, c] of candidateById) {
      const m = metricsById.get(tagId);
      const isPriorityActive = Boolean(c.recording || c.handCoupled);
      const isActiveCandidate = Boolean(isPriorityActive || c.moving || c.settling || c.nearHand);
      const inEnterCone = m.inFrustum && m.distM <= maxDistM && m.angleDeg <= cfg.coneEnterDeg;
      const inExitCone = m.inFrustum && m.distM <= maxDistM * 1.15 && m.angleDeg <= cfg.coneExitDeg;
      const isRestingBg = Boolean(
        c.locked && !isActiveCandidate && (c.stillDurationMs || 0) >= cfg.restingDemoteMs,
      );

      const slot = slots.get(tagId);
      if (slot) {
        const lastObsMs = Math.max(slot.lastSeenAtMs, Number.isFinite(c.lastOpticalAtMs) ? c.lastOpticalAtMs : 0);
        slot.lastSeenAtMs = lastObsMs;
        const opticalAgeMs = lastObsMs > 0 ? Math.max(0, nowMs - lastObsMs) : Infinity;
        slot.stable = slot.stable || isPriorityActive || (nowMs - slot.enteredAtMs >= cfg.stablePromoteMs);

        if (inExitCone) {
          slot.outsideConeSinceMs = null;
          slot.lastInConeAtMs = nowMs;
        } else if (slot.outsideConeSinceMs == null) {
          slot.outsideConeSinceMs = nowMs;
        }

        const holdMs = slot.stable ? cfg.lostInConeHoldMs : cfg.unstableLostHoldMs;
        const expiredOptical = (!isPriorityActive && opticalAgeMs > holdMs) ||
          (isPriorityActive && !c.handCoupled && opticalAgeMs > cfg.lostInConeHoldMs);
        const exitedCone = !inExitCone && !isPriorityActive &&
          slot.outsideConeSinceMs != null && (nowMs - slot.outsideConeSinceMs) >= cfg.outOfConeHoldMs;
        const yieldedResting = hasActive && isRestingBg && (!inEnterCone || hasOtherActiveInCenter(tagId));

        if (expiredOptical || exitedCone || yieldedResting) {
          slots.delete(tagId);
        }
      } else {
        const optAgeMs = Number.isFinite(c.lastOpticalAtMs) && c.lastOpticalAtMs > 0
          ? Math.max(0, nowMs - c.lastOpticalAtMs)
          : Infinity;
        const freshForPromote = optAgeMs <= cfg.freshPromoteAgeMs ||
          (isPriorityActive && optAgeMs <= cfg.lostInConeHoldMs);
        if (!freshForPromote || !m.inFrustum || m.distM > maxDistM) continue;

        const promote = isPriorityActive ||
          (inEnterCone && !(hasActive && isRestingBg)) ||
          ((c.moving || c.settling || c.nearHand) && m.angleDeg <= cfg.coneExitDeg);
        if (promote) {
          slots.set(tagId, {
            tagId,
            enteredAtMs: nowMs,
            lastSeenAtMs: c.lastOpticalAtMs || nowMs,
            lastInConeAtMs: nowMs,
            outsideConeSinceMs: null,
            missStreak: 0,
            stable: isPriorityActive,
          });
        }
      }
    }

    const ranked = [];
    let activeForegroundCount = 0;
    for (const [tagId, slot] of slots) {
      const c = candidateById.get(tagId);
      const m = metricsById.get(tagId);
      if (!c || !m) continue;
      const tier = (c.recording || c.handCoupled) ? 0 : ((c.moving || c.settling || c.nearHand) ? 1 : 2);
      if (tier <= 1) activeForegroundCount++;
      const effAngleDeg = Math.max(0, m.angleDeg - (tagId === primaryTagId ? cfg.primaryStickyBonusDeg : 0));
      ranked.push({ tagId, c, m, slot, tier, effAngleDeg });
    }

    if (ranked.length === 0) {
      return wideResult('no-candidates', nowMs);
    }
    if (activeForegroundCount > maxFg) {
      return wideResult('crowd-wide', nowMs);
    }

    ranked.sort((a, b) => {
      if (a.tier !== b.tier) return a.tier - b.tier;
      if (Math.abs(a.effAngleDeg - b.effAngleDeg) > 0.25) return a.effAngleDeg - b.effAngleDeg;
      if (a.m.distM !== b.m.distM) return a.m.distM - b.m.distM;
      return (b.c.apparentEdgePx || 0) - (a.c.apparentEdgePx || 0);
    });

    const chosen = ranked.slice(0, maxFg);
    const mode = chosen.length === 1 ? 'single-focus' : 'multi-focus';
    lastMode = mode;
    primaryTagId = chosen[0].tagId;

    const tagIds = chosen.map(x => x.tagId);
    const activeTagIds = chosen.filter(x => x.tier <= 1).map(x => x.tagId);
    const searchingTagIds = chosen
      .filter(x => x.slot.missStreak > 0 || (nowMs - (x.c.lastOpticalAtMs || 0)) > 140)
      .map(x => x.tagId);
    const allLocked = chosen.every(
      x => x.c.locked && !x.c.recording && !x.c.handCoupled && !x.c.moving && !x.c.settling && !x.c.nearHand,
    );
    const speedFirst = !allLocked;
    const sweepDecimate = allLocked ? 2 : 3;

    if (lastSweepAtMs === 0) lastSweepAtMs = nowMs;

    if (partialMissStreak >= cfg.partialMissRecoveryLimit) {
      return {
        mode,
        rois: undefined,
        tagIds,
        activeTagIds,
        searchingTagIds,
        speedFirst,
        isBackgroundSweep: true,
        sweepDecimate: 3,
        skipReason: 'partial-miss-recovery',
      };
    }

    const maxSpeed = Math.max(0, ...chosen.map(x => (Number.isFinite(x.c.speedMps) ? x.c.speedMps : 0)));
    const bgIntervalMs = (maxSpeed >= cfg.fastSpeedThresholdMps || recentDetectMs > cfg.headroomBudgetMs || mode === 'multi-focus')
      ? (maxSpeed >= cfg.fastSpeedThresholdMps ? cfg.backgroundSweepFastMs : cfg.backgroundSweepNomMs)
      : (allLocked ? cfg.backgroundSweepMinMs : cfg.backgroundSweepNomMs);

    if ((nowMs - lastSweepAtMs) >= bgIntervalMs) {
      return {
        mode,
        rois: undefined,
        tagIds,
        activeTagIds,
        searchingTagIds,
        speedFirst,
        isBackgroundSweep: true,
        sweepDecimate,
        skipReason: 'bg-sweep',
      };
    }

    const pointsL = chosen.map((x) => {
      const baseHalfPx = Number.isFinite(x.c.halfPx) && x.c.halfPx >= 32 ? x.c.halfPx : 120;
      const missScale = x.slot.missStreak >= 2 ? 1.75 : (x.slot.missStreak === 1 ? 1.35 : 1.0);
      const halfPx = Math.min(260, Math.max(90, Math.round(baseHalfPx * missScale)));
      return {
        pos: x.c.pos,
        halfPx,
        minDistM: Number.isFinite(x.c.minDistM) ? x.c.minDistM : 0.10,
        maxDistM: Number.isFinite(x.c.maxDistM) ? x.c.maxDistM : maxDistM * 1.15,
      };
    });

    const rois = projectPriorRois({
      pointsL,
      worldFromAnchor,
      worldFromViewer,
      viewerFromCameraGL,
      K,
      width,
      height,
      maxRois: chosen.length,
    });

    if (!rois.length) {
      return wideResult('out-of-frustum', nowMs);
    }

    return {
      mode,
      rois,
      tagIds,
      activeTagIds,
      searchingTagIds,
      speedFirst,
      isBackgroundSweep: false,
      sweepDecimate,
      skipReason: null,
    };
  }

  function reportObservation({
    seenIds = [],
    focusTagIds = [],
    isFocusFrame = false,
    isBackgroundSweep = false,
    focusHit = false,
    nowMs = performance.now(),
  } = {}) {
    const seenSet = seenIds instanceof Set ? seenIds : new Set(seenIds);
    const focusList = Array.isArray(focusTagIds) ? focusTagIds : [];
    const focusSet = new Set(focusList);

    if (!isFocusFrame || isBackgroundSweep || !focusHit) {
      lastSweepAtMs = nowMs;
      partialMissStreak = 0;
    }

    for (const [tagId, slot] of slots) {
      if (seenSet.has(tagId)) {
        slot.lastSeenAtMs = nowMs;
        slot.missStreak = 0;
      } else if (isFocusFrame && focusSet.has(tagId)) {
        slot.missStreak++;
      }
    }

    if (isFocusFrame && focusHit) {
      const anyMissed = focusList.some(id => !seenSet.has(id));
      partialMissStreak = anyMissed ? partialMissStreak + 1 : 0;
    }
  }

  function state() {
    return {
      mode: lastMode,
      primaryTagId,
      slotIds: [...slots.keys()],
      lastSweepAtMs,
      partialMissStreak,
    };
  }

  return { plan, reportObservation, reset, state };
}
