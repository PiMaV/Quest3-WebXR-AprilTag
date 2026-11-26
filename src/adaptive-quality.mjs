/**
 * Motion-adaptive quality controller ('auto' | 'fluid' | 'crisp').
 * Drops immediately to responsive 'fluid' settings on head motion and iteratively
 * refines foveation, AprilTag resolution, and static lock refinement while still.
 */
export const ADAPTIVE_QUALITY_MODES = Object.freeze(['auto', 'fluid', 'crisp']);

export const ADAPTIVE_QUALITY_DEFAULTS = Object.freeze({
  moveAngDegPerSec: 18,
  moveLinMPerSec: 0.14,
  steadyAngDegPerSec: 8,
  steadyLinMPerSec: 0.055,
  refineDelayMs: 450,
  crispDelayMs: 1200,
  fluidFoveation: 1.0,
  refinedFoveation: 0.45,
  crispFoveation: 0.35,
  fluidScanDecimate: 2,
  refinedScanDecimate: 1.5,
  crispScanDecimate: 1.5,
  fluidScanMaxHz: 12,
  refinedScanMaxHz: 10,
  crispScanMaxHz: 8,
  fluidDetectMode: 'two-stage',
  refinedDetectMode: 'two-stage',
  crispDetectMode: 'two-stage',
  longRangeFluidScanDecimate: 2,
  longRangeFluidScanMaxHz: 12,
  longRangeRefinedScanDecimate: 1,
  longRangeRefinedScanMaxHz: 6,
  longRangeCrispScanDecimate: 1,
  longRangeCrispScanMaxHz: 6,
  longRangeMinFps: 78,
  longRangeBackoffFps: 72,
});

function tierConfig(cfg, tier, activeMode = 'auto', longRangeActive = false) {
  if (tier === 'crisp') {
    return {
      foveation: activeMode === 'crisp' ? 0.0 : cfg.crispFoveation,
      scanDecimate: longRangeActive ? cfg.longRangeCrispScanDecimate : cfg.crispScanDecimate,
      scanMaxHz: longRangeActive ? cfg.longRangeCrispScanMaxHz : cfg.crispScanMaxHz,
      detectMode: cfg.crispDetectMode,
      allowRefine: true,
    };
  }
  if (tier === 'refined') {
    return {
      foveation: cfg.refinedFoveation,
      scanDecimate: longRangeActive ? cfg.longRangeRefinedScanDecimate : cfg.refinedScanDecimate,
      scanMaxHz: longRangeActive ? cfg.longRangeRefinedScanMaxHz : cfg.refinedScanMaxHz,
      detectMode: cfg.refinedDetectMode,
      allowRefine: true,
    };
  }
  return {
    foveation: cfg.fluidFoveation,
    scanDecimate: longRangeActive ? cfg.longRangeFluidScanDecimate : cfg.fluidScanDecimate,
    scanMaxHz: longRangeActive ? cfg.longRangeFluidScanMaxHz : cfg.fluidScanMaxHz,
    detectMode: cfg.fluidDetectMode,
    allowRefine: false,
  };
}

export function createAdaptiveQualityController({ mode = 'auto', longRange = false, ...opts } = {}) {
  const cfg = { ...ADAPTIVE_QUALITY_DEFAULTS, ...opts };
  let activeMode = ADAPTIVE_QUALITY_MODES.includes(mode) ? mode : 'auto';
  let longRangeEnabled = Boolean(longRange);
  let perfAllowsLongRange = true;
  let tier = activeMode === 'crisp' ? 'crisp' : 'fluid';
  let stillSinceMs = null;
  let transitions = 0;
  const tierFrames = { fluid: 0, refined: 0, crisp: 0 };

  const current = {
    mode: activeMode,
    tier,
    foveation: cfg.fluidFoveation,
    scanDecimate: cfg.fluidScanDecimate,
    scanMaxHz: cfg.fluidScanMaxHz,
    detectMode: cfg.fluidDetectMode,
    allowRefine: false,
    longRange: longRangeEnabled,
    longRangeActive: longRangeEnabled && perfAllowsLongRange,
    changed: false,
  };

  function applyTier(nextTier) {
    const prevTier = tier;
    const prevLongRangeActive = current.longRangeActive;
    const longRangeActive = longRangeEnabled && perfAllowsLongRange && nextTier !== 'fluid';
    tier = nextTier;
    const tc = tierConfig(cfg, tier, activeMode, longRangeActive);
    current.mode = activeMode;
    current.tier = tier;
    current.foveation = tc.foveation;
    current.scanDecimate = tc.scanDecimate;
    current.scanMaxHz = tc.scanMaxHz;
    current.detectMode = tc.detectMode;
    current.allowRefine = tc.allowRefine;
    current.longRange = longRangeEnabled;
    current.longRangeActive = longRangeActive;
    current.changed = prevTier !== tier || prevLongRangeActive !== longRangeActive;
    if (prevTier !== tier) transitions++;
  }

  applyTier(activeMode === 'crisp' ? 'crisp' : 'fluid');
  current.changed = false;
  transitions = 0;

  function setMode(nextMode) {
    if (!ADAPTIVE_QUALITY_MODES.includes(nextMode)) {
      throw new TypeError(`Invalid adaptive quality mode: ${nextMode}`);
    }
    const modeChanged = activeMode !== nextMode;
    activeMode = nextMode;
    stillSinceMs = null;
    if (activeMode === 'fluid') applyTier('fluid');
    else if (activeMode === 'crisp') applyTier('crisp');
    else applyTier('fluid');
    if (modeChanged) current.changed = true;
    return current;
  }

  function setLongRange(enabled) {
    const next = Boolean(enabled);
    const lrChanged = longRangeEnabled !== next;
    longRangeEnabled = next;
    if (next) perfAllowsLongRange = true;
    applyTier(tier);
    if (lrChanged) current.changed = true;
    return current;
  }

  function reset() {
    stillSinceMs = null;
    perfAllowsLongRange = true;
    transitions = 0;
    tierFrames.fluid = 0;
    tierFrames.refined = 0;
    tierFrames.crisp = 0;
    applyTier(activeMode === 'crisp' ? 'crisp' : 'fluid');
    current.changed = false;
    transitions = 0;
  }

  function update({ angDegPerSec = 0, linMPerSec = 0, nowMs = 0, xrFps = null } = {}) {
    if (longRangeEnabled && Number.isFinite(xrFps) && xrFps > 0) {
      if (perfAllowsLongRange && xrFps < cfg.longRangeBackoffFps) perfAllowsLongRange = false;
      else if (!perfAllowsLongRange && xrFps >= cfg.longRangeMinFps) perfAllowsLongRange = true;
    }

    if (activeMode === 'fluid') {
      applyTier('fluid');
      tierFrames[tier]++;
      return current;
    }
    if (activeMode === 'crisp') {
      applyTier('crisp');
      tierFrames[tier]++;
      return current;
    }

    const ang = Number.isFinite(angDegPerSec) ? Math.abs(angDegPerSec) : 0;
    const lin = Number.isFinite(linMPerSec) ? Math.abs(linMPerSec) : 0;
    let nextTier = tier;

    if (ang > cfg.moveAngDegPerSec || lin > cfg.moveLinMPerSec) {
      stillSinceMs = null;
      nextTier = 'fluid';
    } else if (ang < cfg.steadyAngDegPerSec && lin < cfg.steadyLinMPerSec) {
      if (stillSinceMs == null) stillSinceMs = nowMs;
      const elapsed = nowMs - stillSinceMs;
      if (elapsed >= cfg.crispDelayMs) nextTier = 'crisp';
      else if (elapsed >= cfg.refineDelayMs && tier === 'fluid') nextTier = 'refined';
    } else {
      if (tier === 'fluid') stillSinceMs = null;
    }

    applyTier(nextTier);
    tierFrames[tier]++;
    return current;
  }

  return {
    setMode,
    getMode: () => activeMode,
    setLongRange,
    getLongRange: () => longRangeEnabled,
    reset,
    update,
    state: () => ({ ...current }),
    stats: () => ({
      mode: activeMode,
      tier,
      longRange: longRangeEnabled,
      longRangeActive: longRangeEnabled && perfAllowsLongRange && tier !== 'fluid',
      transitions,
      tierFrames: { ...tierFrames },
    }),
  };
}
