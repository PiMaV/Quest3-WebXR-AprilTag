# Architecture & Modules

Technical overview of **Quest3-WebXR-AprilTag** — how camera-to-XR calibration, native WebXR `XRAnchor` handoff, and 90 Hz object tracking work in the browser without a build step.

---

## How It Works

WebXR starts `(0, 0, 0)` in an arbitrary room frame, while Meta Quest Browser's `getUserMedia()` passthrough stream exposes no calibrated intrinsics, no camera-to-headset offset, and ~50 ms of capture delay. Calibrating all three once lets native WebXR SLAM take over completely:

1. **Calibrate once:** Orbit your head around a flat, stationary 3-tag L-angle (`350 / 351 / 352` on A4/screen, `tag36h11`, or `370 / 371 / 372` on tablet/e-Ink). A background Web Worker solves camera intrinsics (`K`), `viewerFromCameraGL`, and `timeOffsetMs`, verifies held-out and fresh views, and caches the validated profile in `localStorage`.
2. **Lock native `XRAnchor`:** Collect 5 consistent 3-tag views, call `frame.createAnchor(rigid, referenceSpace)` once, and **stop the camera**. `frame.getPose(anchor.anchorSpace, referenceSpace)` drives the scene at 90 FPS with zero optical fallback.
3. **Work in `SCAN` or `MEASURE`:** Stay in camera-off 90 FPS `MEASURE` mode for probing and pinning, or switch to `SCAN` when locating perimeter markers, tracking movable tags, or recording 3D paths.

```text
worldFromL = worldFromViewer(captureTime + timeOffsetMs) × viewerFromCameraGL × cameraFromL(captureTime)
```

---

## Calibration & Joint Bundle Adjustment

The calibration pipeline runs off-thread in [`src/calibration-worker.mjs`](../src/calibration-worker.mjs) via `solveCalibration` in [`src/solver.mjs`](../src/solver.mjs), orchestrated by [`src/assistant.mjs`](../src/assistant.mjs) and persisted via [`src/profile.mjs`](../src/profile.mjs):

- **Capture diversity gate (`diversity`):** Requires `>= 30` multi-tag views, two non-collinear head rotation axes (`maxAngle > 0.2 rad`, `rotationArea > 0.015`), and `>= 10 cm` 3D translation span (`translationSpanM >= 0.10`) so focal length `fx` and camera-headset depth `t_z` stay cleanly decoupled even on shallow head tilts.
- **Stage 1 — Planar Intrinsics (`fitIntrinsics`):** Pseudo-Huber (`huberDelta = 3`) Schur-complement bundle adjustment over `centered-square` (`fx = fy`, centered `(cx, cy)`) or full `pinhole` models, restarting planar homographies across `K` updates.
- **Stage 2 — Multi-Seed Hand-Eye + Time Offset (`fitHandEye`):** Solves `viewerFromCameraGL`, `worldFromRig`, and `timeOffsetMs` across multiple delay seeds (`-120 ms`, `0 ms`, `+120 ms`), verifying continuous `< 100 ms` XR pose history around every observation.
- **Stage 3 — Joint Pixel-Space Refinement (`refineJointCalibration`):** Jointly optimizes `K`, `viewerFromCameraGL`, `worldFromRig`, and `timeOffsetMs` directly in pixel reprojection space over up to `48` evenly-spaced training frames (`30` LM iterations) against interpolated WebXR viewer poses to eliminate planar-only `f ↔ t_z` scale/depth degeneracy.
- **Holdout & Verification Gates:**
  - **80/20 holdout validation (`solveCalibration`):** Last 20% of views are never fitted; requires `trainingRmsPx < 1.0`, `heldOutRmsPx < 1.2`, `jointHeldOutRmsPx < 10.0`, `positionP95Mm < 8.5`, `angleP95Deg < 1.5`, `|camT| < 0.16 m`, `|camT_z| < 0.09 m`, `|timeOffsetMs| < 230 ms`, `>= 2` converged delay seeds (`delaySpreadMs < 15`), and tight parameter uncertainty (`fxStd < 0.05`).
  - **Post-fit verification (`verifying`):** Fresh capture after fit (`positionP95Mm < 8.5`, `angleP95Deg < 1.4`, `reprojectionRmsPx < 1.2`, head motion `> 0.04 m`).
  - **Cached session check (`checking`):** New-session check against `localStorage` SHA-256 profile (`positionP95Mm < 10.0`, `angleP95Deg < 1.5`, `reprojectionRmsPx < 1.2`, head motion `> 0.04 m`). `validateCalibration` rejects degraded or out-of-bounds profiles automatically without overwriting valid caches on failure.

---

## Core Architecture & Dependencies

**Zero-dependency ES core vs. vendored runtime dependencies:**

1. **Zero npm dependencies (`package.json`):** No bundler, no transpiler, and no external npm runtime or dev packages (`node >= 22` built-in `node:test` and `scripts/serve.mjs`).
2. **Pure ES core modules (`src/`):** Calibration, pose estimation, anchor lifecycle, hand-coupling, motion prediction, path smoothing, auditing, and rigid-transform math under `src/index.mjs` (and subpath exports `./solver`, `./scan`, `./audit`, `./xr-exit`, `./perimeter`, `./camera`, `./assistant`, `./anchor`, `./profile`, `./rig`, `./runtime`, `./detector`) have **zero Three.js or framework dependencies**.
3. **Zero-dependency TypeScript types (`types/`):** Full `.d.ts` declarations for `.` and every subpath export are included via `"types"` in [`package.json`](../package.json).
4. **Vendored AprilTag WASM (`vendor/apriltag/`):** ARENA/AprilRobotics `tag36h11` WebAssembly detector (`BSD-2-Clause` / `BSD-3-Clause`) loaded inside [`src/detector-worker.js`](../src/detector-worker.js).
5. **Vendored Three.js `0.187.0` (`vendor/three/`):** `MIT`-licensed WebGL engine imported only by [`examples/quest.mjs`](../examples/quest.mjs), [`examples/minimal.mjs`](../examples/minimal.mjs), and the optional rendering helpers exported separately via [`src/three.mjs`](../src/three.mjs) (`@pimav/quest3-webxr-apriltag/three`, wrapping [`src/ui-atlas.mjs`](../src/ui-atlas.mjs) and [`src/render-batch.mjs`](../src/render-batch.mjs)).

See [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) for full license attribution and [`print/SOURCE.json`](../print/SOURCE.json) for physical asset provenance.

---

## Quickstart (Minimal Integration)

For a complete runnable starter (~240 lines, no showcase HUD or portals), see [`examples/minimal.html`](../examples/minimal.html) and [`examples/minimal.mjs`](../examples/minimal.mjs).

### Core Calibration → `XRAnchor` → `worldFromL` Loop

```js
import {
  createReferenceCamera,
  createCalibrationAssistant,
  createLAnchorController,
  createObservationBuffer,
  deviceIdentity,
  product,
  cvToGL,
} from '@pimav/quest3-webxr-apriltag';

const obsBuffer = createObservationBuffer({ maxAgeMs: 650, maxItems: 16 });
let profile = null;

const anchor = createLAnchorController({
  quality: { softReject: true, maxRmsPx: 1.15 },
  onCreated: () => capture.stop(), // switch to pure 90 FPS native XRAnchor mode
});

const capture = createReferenceCamera({
  video,
  geometry: rig,
  getContext: () => ({ sessionId: 'app-1', spaceEpoch }),
  getSettings: () => ({
    horizontalFovDeg: 65,
    printScale: 1,
    maxLongEdge: 1280,
    experimentalCalibration: profile,
    decimate: 1,
    maxHz: 8,
    detectMode: 'two-stage',
  }),
  onObservation: (observation, sample) => {
    assistant.observe(observation, sample, history, performance.now());
    if (profile) obsBuffer.push({ observation, sample });
  },
});

// Inside XRSession startup after capture.start():
const assistant = createCalibrationAssistant({
  rig,
  device: await deviceIdentity(capture.state(), rig),
  imageSize: capture.state().sourceSize,
  onProfile: (p) => { profile = p; },
});
await assistant.begin();

// Inside frame callback:
const worldFromL = anchor.update(frame, now, { viewerTracked });
if (profile && (anchor.phase() === 'collecting' || anchor.phase() === 'ready')) {
  for (const item of obsBuffer.drain(history, profile, now)) {
    const tool = item.observation.tools[0];
    anchor.observe({
      sample: item.sample,
      tool,
      candidate: item.candidate,
      worldFromL: tool
        ? product(item.candidate.worldFromViewer, profile.viewerFromCameraGL, cvToGL(tool.cameraFromTool))
        : null,
    }, now);
  }
}
```

### Custom Physical Tag Sizes

`requiredTagEdgeMm(id)` maps the M.E.S.S. `tag36h11` ranges (`100–499`). For standard tags with custom printed edge lengths (e.g. tags `0, 1, 2` at `50 mm`), pass `edgeMm` or `getTagEdgeMm` directly to `solveSingleTagPose`, or pass `unassignedTagEdgeMm` in `getSettings()` for the detector worker:

```js
import { solveSingleTagPose } from '@pimav/quest3-webxr-apriltag';

const pose = solveSingleTagPose(rawMarker, K, { edgeMm: 50 });
// Or per-ID lookup:
const mapped = solveSingleTagPose(rawMarker, K, {
  getTagEdgeMm: (id) => (id === 0 ? 50 : 30),
});

// In createReferenceCamera getSettings():
// { solveUnassigned: true, unassignedTagEdgeMm: { 0: 50, default: 30 } }
```

### Bundler Setups (Vite / Webpack / Rollup)

`createReferenceCamera` accepts `workerFactory` and `wasmBaseUrl` so bundlers can serve `apriltag_wasm.js` / `apriltag_wasm.wasm` from `public/` or an asset URL without relying on relative `../vendor/apriltag/` resolution:

```js
const capture = createReferenceCamera({
  video,
  geometry: rig,
  wasmBaseUrl: '/vendor/apriltag/',
  workerFactory: ({ workerUrl }) =>
    new Worker(new URL('./my-detector-worker-entry.js' + workerUrl.search, import.meta.url)),
  getSettings,
  getContext,
  onObservation,
});
```

---

## Library Modules (`src/`)

Physical rig and print-sheet definitions live in [`rigs/ms-qr-main-l.json`](../rigs/ms-qr-main-l.json), [`rigs/ms-qr-tablet-l.json`](../rigs/ms-qr-tablet-l.json), [`rigs/ms-qr-independent-tools.json`](../rigs/ms-qr-independent-tools.json), and [`print/SOURCE.json`](../print/SOURCE.json).

| Module | Responsibility |
|---|---|
| [`src/camera.mjs`](../src/camera.mjs) + [`src/detector-worker.js`](../src/detector-worker.js) | `getUserMedia`, atomic `requestVideoFrameCallback` capture, off-thread `VideoFrame` / `ImageBitmap` readback, AprilTag 36h11 WASM, Focus-ROI near-tracking |
| [`src/detector.mjs`](../src/detector.mjs) | AprilTag WASM ABI adapter, two-stage & Focus-ROI scheduling (`detectFocusRois`, `projectPriorRois`, `prioritizeFocusCandidates`) |
| [`src/rig.mjs`](../src/rig.mjs) | Planar rig validation, homography + analytic-Jacobian Gauss-Newton PnP (`fitPlanarPose`, `solveSingleTagPose`), strict-majority subset consensus, `requiredTagEdgeMm` |
| [`src/solver.mjs`](../src/solver.mjs) + [`src/calibration-worker.mjs`](../src/calibration-worker.mjs) | Pseudo-Huber Schur-complement intrinsic BA, multi-seed hand-eye + `timeOffsetMs` solver, joint `fx` + hand-eye pixel BA (`refineJointCalibration`), 80/20 holdout validation |
| [`src/assistant.mjs`](../src/assistant.mjs) + [`src/profile.mjs`](../src/profile.mjs) | Guided cold-start / cached-profile verification state machine, `validateCalibration`, SHA-256 `localStorage` cache, `createObservationBuffer` |
| [`src/anchor.mjs`](../src/anchor.mjs) | 5-view stability collector (`ANCHOR_QUALITY`), synchronous `frame.createAnchor`, explicit `reanchor()`, zero optical fallback |
| [`src/runtime.mjs`](../src/runtime.mjs) | Zero-dependency column-major 4×4 rigid-transform & quaternion math, SLERP pose interpolation |
| [`src/motion-guard.mjs`](../src/motion-guard.mjs) | Head-motion velocity gating (`evaluateMotionGate`, `evaluateCarriedJumpGuard`) and same-frame optical L-angle bridge (`resolveCameraInL`) |
| [`src/hand-coupling.mjs`](../src/hand-coupling.mjs) + [`src/hand-gesture.mjs`](../src/hand-gesture.mjs) | 90 Hz rigid-palm & multi-tag ruler hand-tracking coupling for movable tags (`300–349`) and scale-invariant fist/wrist gestures |
| [`src/tag-predictor.mjs`](../src/tag-predictor.mjs) | Damped constant-velocity 6DOF optical velocity/lead prediction for uncoupled moving AprilTags |
| [`src/path-smoother.mjs`](../src/path-smoother.mjs) | Trail segment fusion (`fuseCommittedTrailSegment`), position/axis tail smoothing, velocity tube & Twist-Ribbon width/heatmap helpers (`speedToRibbonHalfWidth`, `speedToHeatColor`) |
| [`src/perf-stats.mjs`](../src/perf-stats.mjs) + [`src/adaptive-quality.mjs`](../src/adaptive-quality.mjs) | Zero-alloc frame timing, `opticalLagMs` & `mm/s` telemetry (`trackingBenchmark`), and motion-adaptive quality controller (`auto` / `fluid` / `crisp`) |
| [`src/anchor-audit.mjs`](../src/anchor-audit.mjs) | Strictly read-only optical comparison against native anchor pose (`usedToCorrectAnchor: false`) |
| [`src/tool-scan.mjs`](../src/tool-scan.mjs) + [`src/perimeter-fence.mjs`](../src/perimeter-fence.mjs) | Multi-view 6DOF registration for loose plates and clockwise 20 cm perimeter fence (`353–359`) |
| [`src/tag-callout.mjs`](../src/tag-callout.mjs) + [`src/tag-tap.mjs`](../src/tag-tap.mjs) | View-aware 3D callout layout (`computeHoverCardLayout`, `computePinPillPositions`) and full-face tag surface / pinch proximity metrics |
| [`src/three.mjs`](../src/three.mjs) + [`src/ui-atlas.mjs`](../src/ui-atlas.mjs) + [`src/render-batch.mjs`](../src/render-batch.mjs) | Optional Three.js subpath (`./three`) with instanced pools, badge atlases, 2-edge Twist-Ribbon (`createPathRibbonMesh`), and variable-radius tube (`createPathTubeMesh`) helpers |
| [`src/frame-evidence.mjs`](../src/frame-evidence.mjs) + [`src/xr-exit.mjs`](../src/xr-exit.mjs) + [`src/version.mjs`](../src/version.mjs) | Frozen-frame pixel change fingerprinting, Quest controller long-press exit helper, and shared `APP_VERSION_LABEL` |

---

## Run Locally

Zero build step required (Node `>= 22` for automated tests and the zero-dependency static server in [`scripts/serve.mjs`](../scripts/serve.mjs)):

```bash
# 1. Run automated tests
npm test

# 2. Start the zero-dependency static server (serves index.html at /)
npm run serve

# Optional: serve under a subpath (production serves at the domain root q3.mess.engineering)
node scripts/serve.mjs --base /Quest3-WebXR-AprilTag

# 3a. Meta Quest 3 over USB (localhost is a Secure Context, no TLS cert needed)
adb reverse tcp:8443 tcp:8443
# Open in Meta Quest Browser: http://localhost:8443/examples/quest.html
# Or minimal starter:         http://localhost:8443/examples/minimal.html

# 3b. Or serve over LAN HTTPS with your own certificate
node scripts/serve.mjs --cert cert.pem --key key.pem --host 0.0.0.0 --port 8443
```
