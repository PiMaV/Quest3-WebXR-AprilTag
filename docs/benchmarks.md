# Hardware Benchmarks & Scope

## Measured on Meta Quest 3

Recorded on real Meta Quest 3 hardware (`OculusBrowser 157.0.0.22.15`, `Chrome 157`, `1280×1080`, `resizeMode: "none"`, `centered-square` zero-distortion model):

| Metric | Measured Result |
|---|---|
| Native `XRAnchor` loop & readback | `89–90 FPS`; `0.8–0.9 ms` median main-thread readback (`max 2.4 ms`, `worker-bitmap`) |
| Anchor creation & tracking stability | `~15 ms` creation; `18,777` / `11,462` / `6,109` consecutive tracked frames (`0` lost) |
| Intrinsics fit (`fx = fy`) | `866–892 px` (`trainingRmsPx: 0.37–0.48 px`, `heldOutRmsPx: 0.27–0.37 px`) |
| Capture-to-XR time offset | `-50.2` to `-53.4 ms` (`3/3` seeds converged, spread `< 1 µs`) |
| Holdout / verification consistency | `3.8–5.3 mm` / `0.63°` holdout; `0.7–5.5 mm` / `0.06–0.3°` fresh verification |
| Cross-session repeatability (`1 m` span) | `2–4 mm` on 12 mm (`200`-series) & 20 mm (`300`-series) tags; read-only audit `6–8 mm` median |
| Core & automated tests | `0` external runtime dependencies in pure ES core modules (`src/`); vendored AprilTag WASM & Three.js only; `123` passing unit tests |

> **Honesty & Metrology Note:** Reprojection and self-consistency metrics are not an independent physical metrology certification (`independentlyValidated: false`). Anchors are session-scoped (`persistent: false`); calibrated camera profiles persist in `localStorage` across sessions.

---

## Calibration & Tracking Quality Gates

Recent calibration and Focus-ROI additions enforce strict physical sanity bounds before accepting or caching a calibration profile:

- **Joint `fx` + hand-eye BA (`refineJointCalibration`):** Jointly optimizes focal length, `viewerFromCameraGL`, `worldFromRig`, and `timeOffsetMs` in pixel reprojection space, preventing planar-only `f ↔ t_z` scale/depth degeneracy on shallow head tilts.
- **3D translation diversity:** Calibration capture requires `>= 14 cm` of 3D head translation span in addition to multi-axis rotation (`diversity`).
- **Validation & verification bounds:** Holdout `positionP95Mm < 5.0 mm`, `angleP95Deg < 1.0°`, `jointHeldOutRmsPx < 2.2 px`, `|camT| < 0.16 m`, `|camT_z| < 0.09 m`; post-fit verification `< 5.5 mm / 1.0°`; cached session verification `< 8.0 mm / 1.2°`.
- **Fast Focus-ROI near-tracking:** Crops close active movable tags (`300–349`) for `28 Hz` off-thread WASM detection with `decimate = 1.5` (`refineEdges = 0`) motion-blur retry while keeping rendering at `90 FPS`.

---

## Scope & Limits

- **Target Hardware:** Tested on **Meta Quest 3** (`OculusBrowser 157 / Chrome 157`). Android smartphone AR (`camera-access`) is deferred to keep the core focused on Quest 3.
- **Single-Anchor Lever Arm:** Small Quest SLAM corrections show up as gentle drift beyond `~1 m` from the physical L-origin; multi-tag perimeter spatial localization is planned for a future release.
- **Read-Only Optical Audit:** [`src/anchor-audit.mjs`](../src/anchor-audit.mjs) compares live optical poses against the native `XRAnchor` strictly read-only (`usedToCorrectAnchor: false`) and never steers the anchor.
