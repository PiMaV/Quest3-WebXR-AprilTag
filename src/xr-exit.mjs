/** Quest Browser XR controller exit handling. Follows the proven DONNER left-stick pattern. */
export const XR_EXIT_HOLD_MS = 1400;
export const XR_EXIT_STICK_BUTTON = 3; // xr-standard thumbstick
export const XR_EXIT_SECONDARY_BUTTON = 5; // Quest B/Y (fallback)

function pressed(gamepad, index, allowAnalog = false) {
  const button = gamepad?.buttons?.[index];
  return Boolean(button && (
    button.pressed === true ||
    (allowAnalog && Number.isFinite(Number(button.value)) && Number(button.value) >= .90)
  ));
}

/**
 * Left thumbstick click or either controller B/Y long-press.
 * Never use analog thumbstick deflection, which can falsely fire.
 */
export function xrExitPressed(inputSources) {
  const sources = Array.from(inputSources || []).filter(s => s?.gamepad);
  const left = sources.filter(s => s.handedness === "left");
  const stickSources = left.length ? left : sources.length === 1 ? sources : [];
  const stick = stickSources.some(s => pressed(s.gamepad, XR_EXIT_STICK_BUTTON, true));
  const secondary = sources.some(s => pressed(s.gamepad, XR_EXIT_SECONDARY_BUTTON));
  return { stick, secondary, held: stick || secondary };
}

export function nextExitHold(state, held, nowMs, holdMs = XR_EXIT_HOLD_MS) {
  const now = Number(nowMs);
  if (!held || !Number.isFinite(now)) return { sinceMs: null, progress: 0, fire: false };
  const sinceMs = Number.isFinite(state?.sinceMs) ? state.sinceMs : now;
  const progress = Math.max(0, Math.min(1, (now - sinceMs) / holdMs));
  return { sinceMs, progress, fire: progress >= 1 };
}
