# Markers, 1:1 Setup & ID Convention

Every physical coordinate measurement depends on an exact **1:1 millimeter-accurate AprilTag reference L-angle** (`tag36h11`).

---

## Choose a Calibration Target

| Target | Tag IDs | Physical Size | Live 1:1 Viewer | Printable PDF | Rig Definition |
|---|---|---|---|---|---|
| **SCREEN / A4** | `350 / 351 / 352` | `297 × 210 mm` sheet · `230 × 170 mm` L-span | [Open 1:1 Screen Target →](https://q3.mess.engineering/examples/target.html) | [`sheet_A4_eb_mess_icu_en_full_pieces_print.pdf`](../print/sheet_A4_eb_mess_icu_en_full_pieces_print.pdf) | [`rigs/ms-qr-main-l.json`](../rigs/ms-qr-main-l.json) |
| **TABLET / e-Ink** | `370 / 371 / 372` | `200 × 150 mm` sheet · `150 × 110 mm` L-span | [Open 1:1 Tablet Target →](https://q3.mess.engineering/examples/target.html?mode=tablet) | [`tablet_200x150mm_370_371_372.pdf`](../print/tablet_200x150mm_370_371_372.pdf) | [`rigs/ms-qr-tablet-l.json`](../rigs/ms-qr-tablet-l.json) |

### Ensuring Exact 1:1 Scale

- **Printing:** Print the A4 or `200 × 150 mm` PDF at **100% scale** (disable "Fit to Page" / "Shrink to Fit"). Lay the printed sheet flat and stationary on your workbench.
- **Screen / iPad / e-Ink Display:** Open [`examples/target.html`](https://q3.mess.engineering/examples/target.html) (or `?mode=tablet`), hold a standard **ISO/IEC 7810 ID-1 bank or ID card (`85.6 × 54.0 mm`)** against the dashed overlay box, and fine-tune the slider until the physical card matches 1:1. For e-Ink readers, use the **Download e-Ink PNG** button (`300 DPI`, `2362 × 1772 px`) or the `200 × 150 mm` PDF.
- **Automatic Recognition:** `examples/quest.html` automatically detects whether you are using the A4 (`350–352`) or Tablet (`370–372`) L-angle.

---

## M.E.S.S. AprilTag ID Convention (`tag36h11`)

Per 100-block in the M.E.S.S. AprilTag convention (defined in [`print/SOURCE.json`](../print/SOURCE.json) and enforced in [`src/rig.mjs`](../src/rig.mjs) via `requiredTagEdgeMm`), **only the first 50 IDs (`x00–x49`) are free user-assignable tags**; IDs from **`x50` upward (`x50–x99`) are reserved M.E.S.S. system and reference IDs**:

| ID Range | Tag Edge Size | Role |
|---|---|---|
| `200–249` | `12 mm` | Free user-assignable static markers (sample tags `200`, `201` on A4 sheet) |
| `250–299` | — | Reserved M.E.S.S. 200-series range |
| `300–349` | `20 mm` | Free user-assignable movable 3D object / tool tags (sample tags `300`, `301` on A4 sheet) |
| `350–352` | `20 mm` | Reserved A4 L-angle origin rig (`350` origin, `351` +X 230 mm, `352` +Y 170 mm) |
| `353–359` | `20 mm` | Reserved clockwise table perimeter markers (`YL 358`, `TL 353`, `XT 357`, `TR 354`, `YR 359`, `BR 355`, `XB 356`) |
| `370–372` | `20 mm` | Reserved Tablet / e-Ink L-angle origin rig (`370` origin, `371` +X 150 mm, `372` +Y 110 mm) |
| `360–369`, `373–399` | — | Reserved M.E.S.S. 300-series range |

Loose independent tool/plate definitions are described in [`rigs/ms-qr-independent-tools.json`](../rigs/ms-qr-independent-tools.json), and exact millimeter cut-piece geometry plus SHA-256 asset checksums live in [`print/SOURCE.json`](../print/SOURCE.json).
