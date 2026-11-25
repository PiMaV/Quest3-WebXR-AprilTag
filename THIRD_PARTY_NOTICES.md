# Third-Party Notices and Asset Provenance

Project-specific source code (`src/`, `examples/`, `rigs/`, `scripts/`, `tests/`)
and supplied M.E.S.S. print, preview, and showcase branding assets (`print/`,
`examples/assets/`) are licensed under the **Apache License, Version 2.0**
(`v1.0.0`; see `LICENSE`).

Third-party components vendored in this repository remain under their own
respective licenses:

* **`vendor/apriltag/apriltag_wasm.js` and `apriltag_wasm.wasm`**
  ARENA/CONIX standalone AprilTag JavaScript/WASM wrapper — **BSD-3-Clause**.
  License: `vendor/apriltag/LICENSE`
  Upstream: https://github.com/arenaxr/apriltag-js-standalone
  Build provenance and SHA-256 checksums: `vendor/apriltag/README.md`

* **Underlying AprilTag library**
  AprilRobotics / University of Michigan — **BSD-2-Clause**.
  License: `vendor/apriltag/APRILTAG_LICENSE.md`
  Upstream: https://github.com/AprilRobotics/apriltag

* **`vendor/three/three.module.js` and `three.core.js`**
  Three.js `0.187.0` — **MIT License**.
  License: `vendor/three/LICENSE`
  Upstream: https://github.com/mrdoob/three.js
  Used by the Quest 3 demo (`examples/quest.mjs`) and optional WebGL rendering
  helpers (`src/ui-atlas.mjs`, `src/render-batch.mjs`). The core calibration,
  pose solver, anchor lifecycle, and runtime math modules have zero Three.js
  dependencies.

* **Reference Print (`print/`) & Showcase / UI Branding Assets (`examples/assets/`)**
  Authored by PiMaV / Mattern Engineering & Software Solutions (`mess.engineering`),
  licensed under **Apache-2.0** (`print/SOURCE.json`). Only the fixed primary
  L-angle tags (`350 / 351 / 352` on the A4 target, or `370 / 371 / 372` on the
  200 × 150 mm tablet target) form the calibrated reference assembly; loose
  stationary/movable/micro pieces (`353–359`, `300–301`, `200–201`) remain
  independent targets.
