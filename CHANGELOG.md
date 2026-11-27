# Changelog

All notable changes to **Quest3-WebXR-AprilTag** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.1.4] - 2026-11-26

### Added
- **World-Anchored 3D Spatial Calibration Halo:** Replaced the head-locked chasing ring/dashed line and mid-air trail dots with a native Face-ID-style 8-sector 3D halo anchored directly around the physical L-angle in world space ([`examples/quest.mjs`](examples/quest.mjs)). Sweep your head in any order to turn all 8 sectors Emerald Green, then shift left/right during verification and watch the halo lock clockwise on anchor creation.
- **Diagnostic Calibration Retry Feedback:** Calibration retries now surface the exact failing validation gate (e.g. `pos`, `ang`, `rms`, `reproj`, `camera offset`) instead of a generic diversity message ([`src/assistant.mjs`](src/assistant.mjs)).

### Changed
- **Quest 3 Real-World Calibration Gates & Solver Speed:** Subsampled `refineJointCalibration` to `<= 48` evenly-spaced training frames at `30` LM iterations (`~4×` faster), relaxed `diversity` translation span to `>= 10 cm`, and calibrated Quest 3 validation/verification gates to `< 8.5 mm / 1.5°` and `jointHeldOutRmsPx < 10.0 px` (`< 10.0 mm / 1.5°` cached session check, `minMotion = 0.04 m`) while preserving `|camT| < 0.16 m` and `|camT_z| < 0.09 m` sanity bounds ([`src/solver.mjs`](src/solver.mjs), [`src/assistant.mjs`](src/assistant.mjs), [`src/profile.mjs`](src/profile.mjs)).

---

## [1.1.3] - 2026-11-26

### Added
- **Joint Calibration Refinement (`refineJointCalibration`):** Jointly optimizes focal length (`K.fx`), camera-to-headset transform (`viewerFromCameraGL`), rig pose (`worldFromRig`), and capture delay (`timeOffsetMs`) directly in pixel reprojection space ([`src/solver.mjs`](src/solver.mjs)).
- **Tag-Colored 2-Edge Twist-Ribbon:** Added `RIBBON_SPEED_DEFAULTS`, `smoothAxisTail`, `speedToRibbonHalfWidth`, `speedToHeatColor`, and `createPathRibbonMesh` (`5–35 mm` / `7×` width across `0–550 mm/s` with per-tag color identity and subtle right-edge tint for 3D twist readability in 2 draw calls) for `[● REC]` 3D paths ([`src/path-smoother.mjs`](src/path-smoother.mjs), [`src/render-batch.mjs`](src/render-batch.mjs), [`examples/quest.mjs`](examples/quest.mjs)).
- **Two-Stage Multi-Tag Focus-ROI & Hand Coupling:** Added `90 Hz` multi-tag rigid ruler hand coupling up to `~0.42 m` lever arm, two-stage `28 Hz` Focus-ROI tracking (`dec=2` fast search on large boxes → tight `dec=1` refinement, `dec=1.5` motion-blur retry, `focusMissedBoxes` 2-frame partial-miss recovery sweeps, `0.85`-weighted live hand-centered ROIs, `siblingRadiusM = 0.45`), and damped 6DOF optical lead prediction ([`src/hand-coupling.mjs`](src/hand-coupling.mjs), [`src/detector.mjs`](src/detector.mjs), [`src/detector-worker.js`](src/detector-worker.js), [`src/camera.mjs`](src/camera.mjs), [`src/tag-predictor.mjs`](src/tag-predictor.mjs)).
- **Hover Origin Corner Dot (`LL`):** Added a red `LL` origin corner marker on hover highlight squares (single- and dual-hand inspection) so tag orientation is immediately visible ([`examples/quest.mjs`](examples/quest.mjs)).
- **Mid-Air `[⏱ MARK]` & `[↺ CLEAR ALL]`:** Added `0.40 s` bottom-pill Time-Copy snapshots (mid-air or tabletop) and user-data reset that preserves the active Perimeter Fence ([`examples/quest.mjs`](examples/quest.mjs)).
- **Pre-AR L-Angle Confirmation & Auto-Detection:** Added first-visit L-angle gate, static non-AR fallback view without loading the WASM detector, and automatic recognition of A4 (`350–352`) vs. Tablet (`370–372`) targets ([`examples/quest.html`](examples/quest.html), [`examples/quest.mjs`](examples/quest.mjs)).
- **Shared Version Label:** Added [`src/version.mjs`](src/version.mjs) (`APP_VERSION`, `APP_BUILD`, `APP_VERSION_LABEL`) surfaced across the AR page, top/right HUD, and diagnostics JSON.
- **Public Release Documentation & Landing Page:** Relocated technical reference docs to [`docs/`](docs/architecture.md), restructured [`README.md`](README.md) for fast onboarding, updated the M.E.S.S. landing page in [`index.html`](index.html) with prepared video integration (`docs/media.md`), and created this changelog.

### Changed
- **Stricter Calibration & Verification Gates:** Enforced `>= 14 cm` 3D translation diversity during capture (`diversity`) and tightened validation (`< 5.0 mm / 1.0°`, `jointHeldOutRmsPx < 2.2`, `|camT| < 0.16 m`, `|camT_z| < 0.09 m`), post-fit verification (`< 5.5 mm / 1.0°`, motion `> 6 cm`), and session check (`< 8.0 mm / 1.2°`) bounds ([`src/solver.mjs`](src/solver.mjs), [`src/assistant.mjs`](src/assistant.mjs), [`src/profile.mjs`](src/profile.mjs)).
- **Pinning & Optical Freshness UX:** Switched `MEASURE` mode pinning exclusively to Index + Thumb Pinch (or Controller Trigger) to prevent accidental pill brushing; shortened optical dropout window to `~0.95 s` (`1.2 s` while recording) with no mid-air "last seen" ghosts after tag removal ([`examples/quest.mjs`](examples/quest.mjs)).
- **Wrist Menu & Visual Polish:** Replaced left-wrist fence pill with `HUD ON/OFF`, simplified L-angle selection to Screen/Tablet inside `Advanced & Diagnostics`, and softened mode-tinted hands (`SCAN` red / `MEASURE` green).
- **M.E.S.S. AprilTag ID Convention:** Aligned code and docs with the shared convention: `x00–x49` generic (no assigned role), `x50–x99` dedicated (geometry profile); `isMovableTag` replaced by `isGenericTag` (`100–499`), all generic tags tracked uniformly (pinning is a user action), dedicated IDs without a profile warn once; `370–372` documented as dedicated tablet L block; `200–299` confirmed at `12.0 mm` ([`examples/quest.mjs`](examples/quest.mjs), [`print/SOURCE.json`](print/SOURCE.json), [`docs/markers.md`](docs/markers.md)).

### Fixed
- **Focal-Length / Depth Degeneracy:** Eliminated planar-only `f ↔ t_z` scale/depth coupling on shallow head tilts and ensured `validateCalibration` rejects degraded legacy cached profiles and malformed `viewerFromCameraGL` inputs ([`src/solver.mjs`](src/solver.mjs), [`src/profile.mjs`](src/profile.mjs)).
- **Verification Motion Prompt Consistency:** Aligned `verifying` phase head-shift prompt threshold with `minMotion` (`6 cm`) in [`src/assistant.mjs`](src/assistant.mjs).

---

## [1.0.0] - 2026-11-25

### Added
- Initial public release of **Quest3-WebXR-AprilTag** for Meta Quest 3 (`OculusBrowser` / WebXR `immersive-ar`):
  - Browser-native AprilTag `tag36h11` calibration (`K`, `viewerFromCameraGL`, `timeOffsetMs`) with SHA-256 `localStorage` caching.
  - Synchronous native `XRAnchor` handoff (`5` consistent views) and `90 FPS` camera-off `MEASURE` loop.
  - M.E.S.S. branded landing page, 1:1 Screen (`350–352`) and Tablet (`370–372`) target viewer with ISO/IEC 7810 ID-1 card scaling, printable PDFs, and zero-dependency static server (`scripts/serve.mjs`).
