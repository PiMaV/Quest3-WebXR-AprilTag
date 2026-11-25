# Quest3-WebXR-AprilTag

Browser-native **AprilTag → WebXR Spatial Anchor** registration for **Meta Quest 3** — no Unity, no native app, and no continuous marker tracking after anchoring.

> **Release:** `v1.0.0` · **License:** `Apache-2.0` · Hardware-tested on Meta Quest 3 (`OculusBrowser 157 / Chrome 157`).
> **Live Soft Launch (GitHub Pages):**
> - **Landing Page:** https://pimav.github.io/Quest3-WebXR-AprilTag/
> - **Quest 3 AR Demo:** https://pimav.github.io/Quest3-WebXR-AprilTag/examples/quest.html
> - **1:1 Screen Target (A4):** https://pimav.github.io/Quest3-WebXR-AprilTag/examples/target.html
> - **1:1 Tablet / e-Ink Target (`370 / 371 / 372`):** https://pimav.github.io/Quest3-WebXR-AprilTag/examples/target.html?mode=tablet

## Core Idea

1. **Calibrate once per device/camera mode:** Capture multi-view observations of a printed planar AprilTag L-angle (`350 / 351 / 352`, `tag36h11`) while moving your head in `immersive-ar`. A background Web Worker solves camera intrinsics (`K`), camera-to-headset hand-eye transform (`viewerFromCameraGL`), and capture-to-XR pose delay (`timeOffsetMs`), then validates on held-out and fresh views and caches the profile in `localStorage`.
2. **Lock onto the physical frame:** Collect 5 consistent 3-tag views of the stationary L-angle.
3. **Hand off to native WebXR:** Call `frame.createAnchor(rigid, referenceSpace)` once and **stop the camera**. From that point on, `frame.getPose(anchor.anchorSpace, referenceSpace)` drives the 3D scene natively at 90 FPS with zero optical fallback.

```text
worldFromL = worldFromViewer(captureTime + timeOffsetMs) × viewerFromCameraGL × cameraFromL(captureTime)
```

## Hardware Measurements (Meta Quest 3)

Recorded on real Meta Quest 3 hardware (`OculusBrowser 157.0.0.22.15`, `Chrome 157`, `1280×1080`, `resizeMode: "none"`, `centered-square` zero-distortion model):

| Metric | Measured Result |
|---|---|
| Intrinsics fit (`fx = fy`) | `866–892 px` (`trainingRmsPx: 0.37–0.48 px`, `heldOutRmsPx: 0.27–0.37 px`) |
| Capture-to-XR time offset | `-50.2` to `-53.4 ms` (`3/3` seeds converged, spread `< 1 µs`) |
| Holdout / verification consistency | `3.8–5.3 mm` / `0.63°` holdout; `0.7–5.5 mm` / `0.06–0.3°` fresh verification |
| Off-thread camera readback (`worker-bitmap`) | `0.8–0.9 ms` median main-thread (`max 2.4 ms`), `89–90 FPS` XR loop |
| Native `XRAnchor` creation & tracking | Created in `~15 ms`; `18,777` / `11,462` / `6,109` consecutive tracked frames (`0` lost) |
| Cross-session repeatability (1 m span) | `2–4 mm` on 12 mm (`200`-series) & 20 mm (`300`-series) tags; read-only audit `6–8 mm` median |

*Note: Reprojection and self-consistency metrics are **not** an independent physical metrology certification (`independentlyValidated: false`). Anchors are currently session-scoped (`persistent: false`); calibration profiles persist in `localStorage` across sessions.*

## Quickstart

### Option A — Direct HTTPS via GitHub Pages (Meta Quest 3 Browser)
Open **https://pimav.github.io/Quest3-WebXR-AprilTag/** directly in the Meta Quest 3 Browser (choose **Quest 3 AR Demo**), and open **1:1 Screen / Tablet Target** on a monitor/tablet or print the A4 PDF at 100% scale.

### Option B — Local Standalone (No Build Step)

```bash
# 1. Run automated tests (Node >= 22)
npm test

# 2. Start the zero-dependency static server (serves root index.html at /)
npm run serve

# Optional: test the exact GitHub Pages subpath locally
node scripts/serve.mjs --base /Quest3-WebXR-AprilTag

# 3a. Meta Quest 3 over USB (no TLS certificate needed; localhost is a Secure Context):
adb reverse tcp:8443 tcp:8443
# Open in Meta Quest Browser: http://localhost:8443/examples/quest.html

# 3b. Or serve over LAN HTTPS with your own certificate:
node scripts/serve.mjs --cert cert.pem --key key.pem --host 0.0.0.0 --port 8443
```

### Target Sheet
- **Print 1:1 (recommended for true metric scale):** `print/sheet_A4_eb_mess_icu_en_full_pieces_print.pdf` (M.E.S.S. A4 landscape print sheet — currently print v3.0, see `print/SOURCE.json`; 20 mm L-angle tags `350 / 351 / 352`, 230 × 170 mm L-span, plus stationary corner/edge pieces `353–359`, movable tags `300–301`, and 12 mm micro-tags `200–201`).
- **Tablet / e-Ink Reader (20 × 15 cm, `370 / 371 / 372`):** Open [`examples/target.html?mode=tablet`](https://pimav.github.io/Quest3-WebXR-AprilTag/examples/target.html?mode=tablet) (or copy `print/tablet_200x150mm_370_371_372.pdf` directly onto an e-Ink reader / iPad — 20 mm tags, 150 × 110 mm L-span, `rigs/ms-qr-tablet-l.json`).
- **Desktop Monitor (no printer handy):** Open [`examples/target.html`](https://pimav.github.io/Quest3-WebXR-AprilTag/examples/target.html) — displays the full A4 sheet with an ISO credit/ID card (`85.6 × 54.0 mm`) 1:1 calibration overlay at `X 50%, Y 75%` (white quiet-zone plate), smooth zoom, and display PPI auto-guess.

### Quest 3 Demo Controls & Two-Mode Workflow (`examples/quest.html`)
- **Step 1–3 (Calibrate → Verify → Lock L-angle):** Guided automatically by the center reticle orbital guide and optional split HUD (`HUD ON/OFF`, default OFF).
- **Step 4 — Two Main Modes (`SCAN` vs. `MEASURE`):**
  - **`SCAN` Mode (active right after L-lock · camera ON · pulsing red dot on Left Middle knuckle):** Enables live optical AprilTag detection, **0.55 s Long-Press Time-Copies**, and continuous 3D path recording via **`[● REC]`**. Static tags (`>= 350`, `100–299`) place locked objects scaled by ID class (including perimeter markers `353–359`); movable tags (`300–349`) **auto-lock** and follow smoothly via 90 Hz hand-tracking coupling. Toggle back to `MEASURE` via the floating **`[■ MEASURE]`** button on the L-angle, **Left Fist Hold (`0.85 s`)**, or Controller Grip.
  - **`MEASURE` Mode (camera OFF · 90 FPS):** Pure native WebXR anchor tracking — rock-solid, low power, and free of optical stutter. Hover any locked tag, Time-Copy, or path point with your index fingertip to inspect its `X / Y / Z / Floor` axes; pin/unpin via **Index+Thumb Pinch (left or right hand)** or the hover card's **`[📌]`** orb to draw live pairwise 3D distance rails (`mm`, `ΔZ`, `Δt`).
  - **Gated Wrist Menu (active in `MEASURE` mode):** Open via **1.2 s steady frontal palm gaze** or **tapping the inner Wrist-Dot** (`120 ms`) for `● SCAN`, `✕ PINS`, `↺ CLEAR ALL`, `Q: AUTO/FLUID/CRISP`, `HUD ON/OFF`, and `↺ RECALIBRATE`.
  - **Re-anchor & Exit:** **1× Trigger** on tracking loss (`lost` / `failed`), **2× Trigger (≤ 1.5 s)** while `tracking` (controller), **Left Thumbstick or B/Y (hold 1.4 s)** to Exit AR.

## Architecture

Core calibration, pose, anchor, hand-coupling, and runtime modules under `src/` are pure ES modules with **zero Three.js or framework dependencies**. Three.js (`vendor/three`) is only imported by `examples/quest.mjs` and the optional rendering helpers `src/ui-atlas.mjs` and `src/render-batch.mjs`:

| Module | Responsibility |
|---|---|
| `src/camera.mjs` + `src/detector-worker.js` | `getUserMedia`, atomic `requestVideoFrameCallback` capture, off-thread `VideoFrame` / `ImageBitmap` readback, AprilTag 36h11 WASM |
| `src/rig.mjs` | Planar rig validation, homography + Gauss-Newton PnP (`fitPlanarPose`), strict-majority subset consensus, `requiredTagEdgeMm` |
| `src/solver.mjs` + `src/calibration-worker.mjs` | Pseudo-Huber Schur-complement intrinsic BA, multi-seed hand-eye + `timeOffsetMs` solver, 80/20 holdout validation |
| `src/assistant.mjs` + `src/profile.mjs` | Guided cold-start / cached-profile verification state machine, SHA-256 `localStorage` cache, `createObservationBuffer` |
| `src/anchor.mjs` | 5-view stability collector, synchronous `frame.createAnchor`, explicit `reanchor()`, zero optical fallback |
| `src/runtime.mjs` | Zero-dependency column-major 4×4 rigid-transform & quaternion math, SLERP pose interpolation |
| `src/hand-coupling.mjs` + `src/hand-gesture.mjs` | 90 Hz rigid-palm hand-tracking coupling for movable tags (`300–349`) and scale-invariant fist/wrist gestures |
| `src/anchor-audit.mjs` | Strictly read-only optical comparison against native anchor pose (`usedToCorrectAnchor: false`) |
| `src/tool-scan.mjs` + `src/perimeter-fence.mjs` | Multi-view 6DOF registration for loose plates and clockwise 20 cm perimeter fence (`353–359`) |
| `src/ui-atlas.mjs` + `src/render-batch.mjs` | Optional Three.js instanced/atlas rendering helpers for `examples/quest.mjs` |
| `src/xr-exit.mjs` | Quest controller long-press exit helper |

## Known Scope & Limits

- Tested on **Meta Quest 3** (`OculusBrowser 157 / Chrome 157`). Android smartphone AR (`camera-access`) is intentionally deferred to keep the core focused on Quest 3.
- Single-anchor lever arm: small Quest SLAM corrections can appear as gentle float at `> 1 m` distance from the L-origin; multi-anchor / multi-corner perimeter localization is planned for a later phase.
- Licensed under **Apache-2.0** (`LICENSE`). See `THIRD_PARTY_NOTICES.md` for vendored AprilTag WASM (BSD-2-Clause / BSD-3-Clause), Three.js (MIT), and print-asset provenance (`print/SOURCE.json`).
