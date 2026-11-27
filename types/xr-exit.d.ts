import type { XRInputSourceLike } from './common.js';

export const XR_EXIT_HOLD_MS: number;
export const XR_EXIT_STICK_BUTTON: number;
export const XR_EXIT_SECONDARY_BUTTON: number;

export interface ExitHoldState {
  sinceMs: number | null;
  progress: number;
  fire: boolean;
}

export function xrExitPressed(
  inputSources: Iterable<XRInputSourceLike> | null | undefined,
): { stick: boolean; secondary: boolean; held: boolean };
export function nextExitHold(
  state: Partial<ExitHoldState> | null | undefined,
  held: boolean,
  nowMs: number,
  holdMs?: number,
): ExitHoldState;
