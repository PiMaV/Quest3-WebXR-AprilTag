# Controls & Interaction (`examples/quest.html`)

**Hand tracking first. Controllers optional.**

All primary workflows in [`examples/quest.html`](../examples/quest.html) work directly with bare Meta Quest 3 hand tracking; Touch controllers remain supported as an optional alternative.

---

## Startup & Pre-AR Flow

1. **Confirm L-angle:** First-time visitors confirm they have a flat 1:1 L-angle target ready (see [Markers & 1:1 Setup](./markers.md)). A4 (`350–352`) and Tablet (`370–372`) targets are detected automatically.
2. **Enter AR:** Tap **Enter AR →** in Meta Quest Browser (`immersive-ar`). Non-AR browsers stay on a static info screen without loading the WASM detector.
3. **Calibrate or Verify:** Follow the orbital reticle around the stationary L-angle to calibrate camera intrinsics, hand-eye offset, and capture delay once, or perform a quick head shift to verify your cached `localStorage` profile.
4. **Lock Native `XRAnchor`:** Hold the 3-tag L-angle steady for 5 frames to create the native `XRAnchor` at physical `(0, 0, 0)`. Optional debug HUD panels start `OFF` (`HUD ON/OFF` in the Wrist Menu or Controller `A` / `X`).

---

## `SCAN` Mode (Camera ON · Red Hand Tint)

Used for discovering markers, tracking movable objects, recording 3D motion paths, and placing Time-Copies.

- **M.E.S.S. ID Convention (`tag36h11`):** Only the first 50 IDs per 100-block (`x00–x49`) are free user-assignable AprilTags (e.g., `200–249` at 12 mm, `300–349` at 20 mm). IDs from `x50` upward (`x50–x99`, such as `250–299` and `350–399`) are reserved M.E.S.S. system/reference ranges. See [Markers & ID Convention](./markers.md).
- **Movable 20 mm Tags (`300–349`):**
  - Auto-lock and follow live while optically fresh (`~0.95 s` idle / `~1.2 s` recording dropout window, with no mid-air "last seen" ghosts after removal).
  - **Multi-tag rigid ruler 90 Hz hand coupling:** Supports gripping two rigidly mounted tags simultaneously under rotation up to `~0.42 m` lever arm, snapping smoothly back to the optical pose on timeout or drift.
  - **Proximity-prioritized multi-tag Focus-ROI tracking:** Runs up to `28 Hz` on up to 3 active/sibling targets (keeping nearby tags within `45 cm` even during active recording) via a two-stage ROI pipeline (`decimate = 2` search on large boxes → tight `decimate = 1` corner refinement, plus `decimate = 1.5` motion-blur retry), instant partial-miss recovery sweeps, live 90 Hz hand-centered ROIs, and damped 6DOF optical lead prediction inside thicker shimmering frames.
- **Static & Other Edge Classes (`200–249`, etc.):** Detected tags with defined edge classes (such as free 12 mm tags `200–249`) place static locked markers scaled by ID class (`requiredTagEdgeMm`) with their ID rendered flat on the tag matrix.
- **Table Perimeter Fence (`353–359`):** Reserved perimeter markers (activated via `TL` `353`, labeled `YL` `358`, `TL` `353`, `XT` `357`, `TR` `354`, `YR` `359`, `BR` `355`, `XB` `356`) build a 20 cm clockwise table fence.
- **Time-Copy Snapshots (`[⏱ MARK]`):** Hold an index fingertip `0.40 s` on the bottom **`[⏱ MARK]`** pill below any tag (in mid-air or flat on the table, positioned clear of left/right grabbing fingers) to freeze a **Time-Copy** point marker at its live 3D pose.
- **3D Twist-Ribbon Path Recording (`[● REC]`):** Hold **`[● REC]`** (`0.65 s` start / `0.35 s` stop) above a tag to record a **tag-colored 2-edge Twist-Ribbon** (retains per-tag object color while modulating width `5–35 mm` (`7×`) across `0–550 mm/s` to show 3D roll/twist and speed in 2 draw calls) with waypoint dots.
- **Mode Switch & Reset:** On the anchored L-angle bar, tap **`[■ MEASURE / ● SCAN]`**, **`[◈ FENCE]`**, or **`[↺ CLEAR ALL]`** (clears user-scanned tags, trails, Time-Copies, and pins while keeping the Perimeter Fence active), or use a frontal **Left Fist Hold (`0.85 s`)** / Controller Grip.

---

## `MEASURE` Mode (Camera OFF · 90 FPS · Green Hand Tint)

Pure native WebXR anchor tracking — low power and free of optical stutter.

- **Fingertip Hover Inspection:** Hover any locked tag, Time-Copy, or path waypoint with your index fingertip to reveal the full 3D tag square (with a red `LL` origin corner dot showing tag orientation), orientation plane, tangential speed (`mm/s`), and metric coordinates (`X / Y / Z / Floor`).
- **Pinning & Distance Rails:** **Index + Thumb Pinch** (left or right hand, or Controller Trigger) exclusively pins/unpins hovered points for live pairwise 3D distance rails (`mm`, `ΔZ`, `Δt`), preventing accidental free-hand pill brushing.
- **Gated Wrist Menu** (`1.2 s` steady frontal palm gaze or `120 ms` tap on the inner Wrist-Dot):
  - `↺ CLEAR ALL` — clears user-scanned tags, trails, Time-Copies, and pins while preserving the Perimeter Fence.
  - `HUD ON/OFF` — toggles floating diagnostic HUD panels.
  - `Q: AUTO / FLUID / CRISP` — switches motion-adaptive rendering quality.
  - `RANGE: AUTO 1.5 / EXP 1.0` — `AUTO 1.5` caps at `decimate = 1.5` when steady and locks to `decimate = 2` whenever moving/active; `EXP 1.0` enables idle-only `decimate = 1` above `78 FPS`.
  - `↺ RECALIBRATE` — clears active profile check and enters fresh calibration.

---

## Re-anchor & Exit

- **Re-anchor:** `1× Trigger` on tracking loss (`lost` / `failed`), or `2× Trigger (≤ 1.5 s)` while `tracking` (controller).
- **Exit AR:** Right-hand Meta gesture (palm facing toward you + pinch), or hold **Left Thumbstick** or **`B` / `Y`** for `1.4 s` (`src/xr-exit.mjs`).
