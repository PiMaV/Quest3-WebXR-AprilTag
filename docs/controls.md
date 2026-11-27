# Controls & Interaction (`examples/quest.html`)

**Hand tracking first. Controllers optional.**

All primary workflows in [`examples/quest.html`](../examples/quest.html) work directly with bare Meta Quest 3 hand tracking; Touch controllers remain supported as an optional alternative.

---

## Startup & Pre-AR Flow

1. **Confirm L-angle:** First-time visitors confirm they have a flat 1:1 L-angle target ready (see [Markers & 1:1 Setup](./markers.md)). A4 (`350–352`) and Tablet (`370–372`) targets are detected automatically.
2. **Enter AR:** Tap **Enter AR →** in Meta Quest Browser (`immersive-ar`). Non-AR browsers stay on a static info screen without loading the WASM detector.
3. **Calibrate or Verify:** Sweep your head around the world-anchored **3D Spatial Halo** floating over the stationary L-angle until all 8 arcs turn green to calibrate camera intrinsics, hand-eye offset, and capture delay once, or sweep left/right to verify your cached `localStorage` profile.
4. **Lock Native `XRAnchor`:** Hold the 3-tag L-angle steady for 5 frames to create the native `XRAnchor` at physical `(0, 0, 0)`. Optional debug HUD panels start `OFF` (`HUD ON/OFF` in the Wrist Menu or Controller `A` / `X`).

---

## `SCAN` Mode (Camera ON · Red Left / Sky-Blue–Red Right Hand Tint)

Used for discovering markers, tracking generic tagged objects, recording 3D motion paths, and placing Time-Copies.

- **M.E.S.S. ID Convention (`tag36h11`):** The hundreds digit selects the size class and tag edge. `x00–x49` are generic (no assigned role, e.g. `200–249` at 12 mm, `300–349` at 20 mm); `x50–x99` are dedicated (fixed purpose with a geometry profile, e.g. `350–359`, `370–372`). See [Markers & ID Convention](./markers.md).
- **Generic Tags (`x00–x49`, e.g. 20 mm `300–349`):**
  - Auto-lock and follow live while optically fresh (`~0.95 s` idle / `~1.2 s` recording dropout window, with no mid-air "last seen" ghosts after removal).
  - **Default Smooth Motion Wake (Non-Recording):** Whenever a generic tag moves in default `SCAN` mode, it renders a tapered `~1.6 s` smoothed wake (`2.6 mm` head, `0.74` max opacity via `createMotionWakeBuffer` + per-vertex `aFade` `createPathTubeMesh`, respecting `Unique Tag Color` / `Turbo Speed`) that visually calms micro-jitter and fades completely to `0` draw calls at rest.
  - **Multi-tag rigid ruler 90 Hz hand coupling:** Supports gripping two rigidly mounted tags simultaneously under rotation up to `~0.42 m` lever arm, snapping smoothly back to the optical pose on timeout or drift.
  - **Proximity-prioritized multi-tag Focus-ROI tracking:** Runs up to `28 Hz` on up to 3 active/sibling targets (keeping nearby tags within `45 cm` even during active recording) via a two-stage ROI pipeline (`decimate = 2` search on large boxes → tight `decimate = 1` corner refinement, plus `decimate = 1.5` motion-blur retry), instant partial-miss recovery sweeps, live 90 Hz hand-centered ROIs, and damped 6DOF optical lead prediction inside thicker shimmering frames.
- **Dedicated IDs without a profile (`x50–x99`, e.g. `250–299`):** Shown as static locked markers scaled by ID class (`requiredTagEdgeMm`) with their ID rendered flat on the tag matrix, plus a one-time warning.
- **Table Perimeter Fence (`353–359`):** Reserved perimeter markers (activated via `TL` `353`, labeled `YL` `358`, `TL` `353`, `XT` `357`, `TR` `354`, `YR` `359`, `BR` `355`, `XB` `356`) build a 20 cm clockwise table fence.
- **Time-Copy Snapshots (`[⏱ MARK]`):** Hold an index fingertip `0.40 s` on the bottom **`[⏱ MARK]`** pill below any tag (in mid-air or flat on the table, positioned clear of left/right grabbing fingers) to freeze a **Time-Copy** point marker at its live 3D pose.
- **3D Path Recording (`[● REC]` & `[● REC ALL]`):** Hold **`[● REC]`** (`0.65 s` start / `0.35 s` stop) above a tag, or toggle **`[● REC ALL]`** on the 2-row Origin Bar / **Right Frontal Fist Hold (`0.80 s`)** / Left Wrist Middle **`● REC ALL`** to record all moving generic tags seamlessly while resting tags stay in the `Ruhezone` (`0` GPU trail uploads, re-entering rest after `450 ms` of stillness). Trails render as a calm, uniform **`3.0 mm` 3D line** (`1` draw call via `createPathTubeMesh`) with waypoint dots; segments outside an active Table Perimeter Fence render cleanly **dashed**.
- **Mode Switch, Trail Color & Reset:** On the compact 2-row anchored L-angle dock (top: `[■ MEASURE / ● SCAN]`, `[● REC ALL]`, `[?]`; bottom: `[◈ FENCE]`, `[TRAIL: ID / SPEED]`, `[↺ CLEAR ALL]`), poke any pill (with anchored **Turbo Color Bar** below in speed mode), or use **Left Fist Hold (`0.85 s`)** (`SCAN ↔ MEASURE`) / **Right Fist Hold (`0.80 s`)** (`REC ALL`) / Controller Grip.

---

## `MEASURE` Mode (Camera OFF · 90 FPS · Green Left / Sky-Blue–Red Right Hand Tint)

Pure native WebXR anchor tracking — low power and free of optical stutter.

- **Fingertip Hover Inspection:** Hover any locked tag, Time-Copy, or path waypoint with your index fingertip to reveal the full 3D tag square (with a red `LL` origin corner dot showing tag orientation), orientation plane, tangential speed (`mm/s`), and metric coordinates (`X / Y / Z / Floor`).
- **Pinning & Distance Rails:** **Index + Thumb Pinch** (left or right hand, or Controller Trigger) exclusively pins/unpins hovered points for live pairwise 3D distance rails (`mm`, `ΔZ`, `Δt`), preventing accidental free-hand pill brushing.
- **Gated Wrist Menu** (`0.65 s` steady frontal palm gaze or `90 ms` tap on the inner Wrist-Dot):
  - `HUD ON/OFF` (Left Index) — toggles floating diagnostic HUD panels.
  - `● REC ALL` (Left Middle) — switches into `SCAN` and arms Ruhezone-coupled recording for all moving generic tags.
  - `TRAIL: ID / TURBO` (Left Ring) — toggles recorded trails between Unique Tag Color and Google Turbo Speed (`0–600 mm/s`).
  - `↺ CLEAR ALL` (Left Pinky) — clears user-scanned tags, trails, Time-Copies, and pins while preserving the Perimeter Fence.
  - `Q: AUTO / FLUID / CRISP` — switches motion-adaptive rendering quality.
  - `RANGE: AUTO 1.5 / EXP 1.0` — `AUTO 1.5` caps at `decimate = 1.5` when steady and locks to `decimate = 2` whenever moving/active; `EXP 1.0` enables idle-only `decimate = 1` above `78 FPS`.
  - `↺ RECALIBRATE` — clears active profile check and enters fresh calibration.

---

## Re-anchor & Exit

- **Re-anchor:** `1× Trigger` on tracking loss (`lost` / `failed`), or `2× Trigger (≤ 1.5 s)` while `tracking` (controller).
- **Exit AR:** Right-hand Meta gesture (palm facing toward you + pinch), or hold **Left Thumbstick** or **`B` / `Y`** for `1.4 s` (`src/xr-exit.mjs`).
