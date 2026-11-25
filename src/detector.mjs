// ARENA/AprilRobotics WASM ABI adapter; physical tag sizes come explicitly from rig definitions.
import { inverse, product } from './runtime.mjs';
const VALID_DECIMATES = [1, 1.5, 2];

function validateGrayFrame(gray, width, height) {
  if (!(gray instanceof Uint8Array) || !Number.isInteger(width) || !Number.isInteger(height) ||
      width <= 0 || height <= 0 || width > 4096 || height > 4096 || gray.length !== width * height) {
    throw new TypeError('Invalid grayscale frame');
  }
}

function markerQuadArea(c) {
  if (!Array.isArray(c) || c.length !== 4) return 0;
  let a = 0;
  for (let i = 0; i < 4; i++) {
    const p = c[i], q = c[(i + 1) & 3];
    a += p.x * q.y - p.y * q.x;
  }
  return Math.abs(a) * 0.5;
}

function mergeRoiBoxes(boxes, gapPx = 16, { maxMergedArea = 90000, areaSlackPx2 = 4096 } = {}) {
  let merged = true;
  while (merged && boxes.length > 1) {
    merged = false;
    for (let i = 0; i < boxes.length; i++) {
      const a = boxes[i];
      if (!a) continue;
      for (let j = i + 1; j < boxes.length; j++) {
        const b = boxes[j];
        if (!b) continue;
        if (a[2] + gapPx < b[0] || b[2] + gapPx < a[0] || a[3] + gapPx < b[1] || b[3] + gapPx < a[1]) continue;
        const ux0 = Math.min(a[0], b[0]);
        const uy0 = Math.min(a[1], b[1]);
        const ux1 = Math.max(a[2], b[2]);
        const uy1 = Math.max(a[3], b[3]);
        const unionArea = (ux1 - ux0) * (uy1 - uy0);
        const areaA = (a[2] - a[0]) * (a[3] - a[1]);
        const areaB = (b[2] - b[0]) * (b[3] - b[1]);
        if (unionArea > maxMergedArea || unionArea > areaA + areaB + areaSlackPx2) continue;
        a[0] = ux0;
        a[1] = uy0;
        a[2] = ux1;
        a[3] = uy1;
        boxes[j] = null;
        merged = true;
      }
    }
    if (merged) boxes = boxes.filter(Boolean);
  }
  return boxes;
}

function markerCenterXY(m) {
  if (m?.center && Number.isFinite(m.center.x) && Number.isFinite(m.center.y)) {
    return [m.center.x, m.center.y];
  }
  const c = m?.corners;
  if (!Array.isArray(c) || c.length !== 4) return null;
  const cx = (c[0].x + c[1].x + c[2].x + c[3].x) * 0.25;
  const cy = (c[0].y + c[1].y + c[2].y + c[3].y) * 0.25;
  return Number.isFinite(cx) && Number.isFinite(cy) ? [cx, cy] : null;
}

function sanitizePriorRois(priorRois, width, height, maxCount = 6) {
  if (!Array.isArray(priorRois) || priorRois.length === 0) return [];
  const maxArea = 0.40 * width * height;
  const out = [];
  for (const box of priorRois) {
    if (!Array.isArray(box) || box.length !== 4) continue;
    const [bx0, by0, bx1, by1] = box;
    if (!Number.isFinite(bx0) || !Number.isFinite(by0) || !Number.isFinite(bx1) || !Number.isFinite(by1)) continue;
    const x0 = Math.max(0, Math.floor(bx0));
    const y0 = Math.max(0, Math.floor(by0));
    const x1 = Math.min(width, Math.ceil(bx1));
    const y1 = Math.min(height, Math.ceil(by1));
    const rw = x1 - x0;
    const rh = y1 - y0;
    if (rw < 16 || rh < 16 || (rw * rh) > maxArea) continue;
    out.push([x0, y0, x1, y1]);
    if (out.length >= maxCount) break;
  }
  return out;
}

const isFiniteMat4 = m => Boolean(m && typeof m.length === 'number' && m.length === 16 && Array.prototype.every.call(m, Number.isFinite));

/**
 * Project known 3D L-frame anchor/hand/tag coordinates into clamped 2D camera ROI boxes [x0, y0, x1, y1].
 * Preserves input priority order while merging overlapping crops and capping to maxRois.
 */
export function projectPriorRois({
  pointsL,
  worldFromAnchor,
  worldFromViewer,
  viewerFromCameraGL,
  K,
  width,
  height,
  minDistM = 0.45,
  maxDistM = 1.85,
  defaultHalfPx = 68,
  maxRois = 4,
} = {}) {
  if (
    !Array.isArray(pointsL) || pointsL.length === 0 ||
    !isFiniteMat4(worldFromAnchor) || !isFiniteMat4(worldFromViewer) || !isFiniteMat4(viewerFromCameraGL) ||
    !K || !(K.fx > 0) || !(K.fy > 0) || !Number.isFinite(K.cx) || !Number.isFinite(K.cy) ||
    !(width >= 32) || !(height >= 32) || !(maxRois >= 1)
  ) {
    return [];
  }
  const lFromCameraGL = product(inverse(worldFromAnchor), worldFromViewer, viewerFromCameraGL);
  const camGLFromL = inverse(lFromCameraGL);
  if (!isFiniteMat4(camGLFromL)) return [];

  const boxes = [];
  for (const item of pointsL) {
    const pos = Array.isArray(item) ? item : item?.pos;
    if (!Array.isArray(pos) || pos.length < 3) continue;
    const px = pos[0], py = pos[1], pz = pos[2];
    if (!Number.isFinite(px) || !Number.isFinite(py) || !Number.isFinite(pz)) continue;

    const itemMinDist = Number.isFinite(item?.minDistM) ? item.minDistM : minDistM;
    const itemMaxDist = Number.isFinite(item?.maxDistM) ? item.maxDistM : maxDistM;
    const halfPx = Number.isFinite(item?.halfPx) && item.halfPx >= 16 ? item.halfPx : defaultHalfPx;

    const xGL = camGLFromL[0] * px + camGLFromL[4] * py + camGLFromL[8] * pz + camGLFromL[12];
    const yGL = camGLFromL[1] * px + camGLFromL[5] * py + camGLFromL[9] * pz + camGLFromL[13];
    const zGL = camGLFromL[2] * px + camGLFromL[6] * py + camGLFromL[10] * pz + camGLFromL[14];
    const xCV = xGL, yCV = -yGL, zCV = -zGL;
    if (!Number.isFinite(zCV) || zCV < itemMinDist || zCV > itemMaxDist) continue;

    const u = K.fx * (xCV / zCV) + K.cx;
    const v = K.fy * (yCV / zCV) + K.cy;
    if (!Number.isFinite(u) || !Number.isFinite(v)) continue;
    if (u < -halfPx || u > width + halfPx || v < -halfPx || v > height + halfPx) continue;

    const x0 = Math.max(0, Math.floor(u - halfPx));
    const y0 = Math.max(0, Math.floor(v - halfPx));
    const x1 = Math.min(width, Math.ceil(u + halfPx));
    const y1 = Math.min(height, Math.ceil(v + halfPx));
    if (x1 - x0 < 32 || y1 - y0 < 32) continue;
    boxes.push([x0, y0, x1, y1]);
  }
  if (boxes.length === 0) return [];
  const merged = mergeRoiBoxes(boxes, 16);
  if (merged.length > maxRois) merged.length = maxRois;
  return merged;
}

export function createAprilTagDetector(wasm, decimate = 1) {
  if (!wasm?.cwrap || !VALID_DECIMATES.includes(decimate)) throw new TypeError('Invalid AprilTag runtime/options');
  const init = wasm.cwrap('atagjs_init', 'number', []);
  const config = wasm.cwrap('atagjs_set_detector_options', 'number', Array(7).fill('number'));
  const setBuffer = wasm.cwrap('atagjs_set_img_buffer', 'number', ['number', 'number', 'number']);
  const detect = wasm.cwrap('atagjs_detect', 'number', []);
  const decoder = new TextDecoder();

  let currentDecimate = decimate;
  let currentRefineEdges = 1;
  let roiScratch = new Uint8Array(0);

  init();
  config(currentDecimate, 0, 1, currentRefineEdges, 0, 0, 0);

  function setOptions({ decimate: nextDec, refineEdges: nextRef } = {}) {
    let dec = currentDecimate;
    let ref = currentRefineEdges;
    if (nextDec !== undefined) {
      if (!VALID_DECIMATES.includes(nextDec)) throw new TypeError('Invalid decimate option');
      dec = nextDec;
    }
    if (nextRef !== undefined) {
      ref = nextRef ? 1 : 0;
    }
    if (dec === currentDecimate && ref === currentRefineEdges) return;
    currentDecimate = dec;
    currentRefineEdges = ref;
    config(currentDecimate, 0, 1, currentRefineEdges, 0, 0, 0);
  }

  function rawDetect(buf, w, h) {
    const buffer = setBuffer(w, h, w);
    wasm.HEAPU8.set(buf, buffer);
    const ptr = detect();
    const len = wasm.getValue(ptr, 'i32');
    if (!len) return [];
    const strPtr = wasm.getValue(ptr + 4, 'i32');
    const markers = JSON.parse(decoder.decode(new Uint8Array(wasm.HEAPU8.buffer, strPtr, len)));
    if (!Array.isArray(markers)) throw new Error('Invalid WASM detector result');
    return markers;
  }

  function copyRoi(gray, frameW, x0, y0, rw, rh) {
    const needed = rw * rh;
    if (roiScratch.length < needed) roiScratch = new Uint8Array(needed);
    for (let r = 0; r < rh; r++) {
      const srcOff = (y0 + r) * frameW + x0;
      roiScratch.set(gray.subarray(srcOff, srcOff + rw), r * rw);
    }
    return roiScratch.subarray(0, needed);
  }

  function offsetMarkers(markers, x0, y0) {
    if (x0 === 0 && y0 === 0) return markers;
    for (const m of markers) {
      if (Array.isArray(m.corners)) {
        for (const p of m.corners) {
          p.x += x0;
          p.y += y0;
        }
      }
      if (m.center && Number.isFinite(m.center.x) && Number.isFinite(m.center.y)) {
        m.center.x += x0;
        m.center.y += y0;
      }
    }
    return markers;
  }

  function deduplicateMarkers(markers) {
    if (markers.length <= 1) return markers;
    const kept = [];
    for (const m of markers) {
      const c = m.corners;
      if (!Array.isArray(c) || c.length !== 4) continue;
      const cx = (c[0].x + c[1].x + c[2].x + c[3].x) * 0.25;
      const cy = (c[0].y + c[1].y + c[2].y + c[3].y) * 0.25;
      const baseScore = Number.isFinite(m.decision_margin) ? m.decision_margin : markerQuadArea(c);
      const score = (m.softFallback ? 0 : 1e6) + baseScore;
      let dupIdx = -1;
      for (let i = 0; i < kept.length; i++) {
        const k = kept[i];
        if (k.id === m.id && Math.hypot(k.cx - cx, k.cy - cy) <= 12) {
          dupIdx = i;
          break;
        }
      }
      if (dupIdx === -1) {
        kept.push({ id: m.id, cx, cy, score, marker: m });
      } else if (score > kept[dupIdx].score) {
        kept[dupIdx] = { id: m.id, cx, cy, score, marker: m };
      }
    }
    return kept.map(k => k.marker);
  }

  function detectTwoStage(gray, width, height, {
    coarseDecimate = 2,
    padFrac = 0.65,
    minPadPx = 24,
    maxRois = 16,
    refine = true,
    priorRois = null,
  } = {}) {
    validateGrayFrame(gray, width, height);
    const prevDec = currentDecimate;
    const prevRef = currentRefineEdges;
    try {
      setOptions({ decimate: coarseDecimate, refineEdges: 0 });
      const tCoarse = performance.now();
      const coarseMarkers = rawDetect(gray, width, height);
      const coarseMs = performance.now() - tCoarse;
      const validPriors = refine ? sanitizePriorRois(priorRois, width, height, 6) : [];
      if (!refine || (coarseMarkers.length === 0 && validPriors.length === 0)) {
        return { markers: coarseMarkers, coarseMs, roiMs: 0, roiCount: 0 };
      }

      let boxes = [];
      const coarseCenters = [];
      for (const m of coarseMarkers) {
        const c = m.corners;
        if (!Array.isArray(c) || c.length !== 4) continue;
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        let ok = true;
        for (const p of c) {
          if (!Number.isFinite(p?.x) || !Number.isFinite(p?.y)) { ok = false; break; }
          if (p.x < minX) minX = p.x;
          if (p.x > maxX) maxX = p.x;
          if (p.y < minY) minY = p.y;
          if (p.y > maxY) maxY = p.y;
        }
        if (!ok) continue;
        const ctr = markerCenterXY(m);
        if (ctr) coarseCenters.push(ctr);
        const span = Math.max(maxX - minX, maxY - minY);
        const pad = Math.max(minPadPx, Math.round(span * padFrac));
        const x0 = Math.max(0, Math.floor(minX) - pad);
        const y0 = Math.max(0, Math.floor(minY) - pad);
        const x1 = Math.min(width, Math.ceil(maxX) + pad);
        const y1 = Math.min(height, Math.ceil(maxY) + pad);
        if (x1 - x0 < 16 || y1 - y0 < 16) continue;
        boxes.push([x0, y0, x1, y1]);
      }
      const uncoveredPriors = [];
      for (const pBox of validPriors) {
        const area = (pBox[2] - pBox[0]) * (pBox[3] - pBox[1]);
        if (area <= 55000 && coarseCenters.length > 0) {
          const pcx = (pBox[0] + pBox[2]) * 0.5;
          const pcy = (pBox[1] + pBox[3]) * 0.5;
          if (coarseCenters.some(([cx, cy]) => Math.hypot(cx - pcx, cy - pcy) <= 48)) continue;
        }
        uncoveredPriors.push(pBox);
        boxes.push(pBox);
      }

      boxes = mergeRoiBoxes(boxes, 16);
      if (boxes.length > maxRois) boxes.length = maxRois;
      if (boxes.length === 0) {
        return { markers: coarseMarkers, coarseMs, roiMs: 0, roiCount: 0 };
      }

      setOptions({ decimate: 1, refineEdges: 1 });
      const tRoi = performance.now();
      const refined = [];
      for (const [x0, y0, x1, y1] of boxes) {
        const rw = x1 - x0;
        const rh = y1 - y0;
        const buf = copyRoi(gray, width, x0, y0, rw, rh);
        const roiMarkers = offsetMarkers(rawDetect(buf, rw, rh), x0, y0);
        for (const m of roiMarkers) refined.push(m);
      }
      if (uncoveredPriors.length > 0) {
        const softFallbackBoxes = [];
        for (const pBox of uncoveredPriors) {
          const [px0, py0, px1, py1] = pBox;
          if ((px1 - px0) * (py1 - py0) > 28000) continue;
          const covered = refined.some((m) => {
            const ctr = markerCenterXY(m);
            return ctr && ctr[0] >= px0 && ctr[0] <= px1 && ctr[1] >= py0 && ctr[1] <= py1;
          });
          if (!covered) {
            softFallbackBoxes.push(pBox);
            if (softFallbackBoxes.length >= 2) break;
          }
        }
        if (softFallbackBoxes.length > 0) {
          setOptions({ decimate: 1, refineEdges: 0 });
          for (const [x0, y0, x1, y1] of softFallbackBoxes) {
            const rw = x1 - x0;
            const rh = y1 - y0;
            const buf = copyRoi(gray, width, x0, y0, rw, rh);
            const roiMarkers = offsetMarkers(rawDetect(buf, rw, rh), x0, y0);
            for (const m of roiMarkers) {
              m.softFallback = true;
              refined.push(m);
            }
          }
        }
      }
      const roiMs = performance.now() - tRoi;
      return {
        markers: deduplicateMarkers(refined),
        coarseMs,
        roiMs,
        roiCount: boxes.length,
      };
    } finally {
      setOptions({ decimate: prevDec, refineEdges: prevRef });
    }
  }

  function detectTrackedRoi(gray, width, height, hintBox, { requiredIds = [], fallbackOpts = {} } = {}) {
    validateGrayFrame(gray, width, height);
    if (Array.isArray(hintBox) && hintBox.length === 4) {
      const x0 = Math.max(0, Math.floor(hintBox[0]));
      const y0 = Math.max(0, Math.floor(hintBox[1]));
      const x1 = Math.min(width, Math.ceil(hintBox[2]));
      const y1 = Math.min(height, Math.ceil(hintBox[3]));
      const rw = x1 - x0;
      const rh = y1 - y0;
      if (rw >= 32 && rh >= 32 && (rw * rh) < 0.55 * width * height) {
        const prevDec = currentDecimate;
        const prevRef = currentRefineEdges;
        let roiMarkers = [];
        let roiMs = 0;
        try {
          setOptions({ decimate: 1, refineEdges: 1 });
          const t0 = performance.now();
          const buf = copyRoi(gray, width, x0, y0, rw, rh);
          roiMarkers = offsetMarkers(rawDetect(buf, rw, rh), x0, y0);
          roiMs = performance.now() - t0;
        } finally {
          setOptions({ decimate: prevDec, refineEdges: prevRef });
        }
        if (requiredIds.length > 0) {
          const seen = new Set(roiMarkers.map(m => m.id));
          if (requiredIds.every(id => seen.has(id))) {
            return { markers: roiMarkers, coarseMs: 0, roiMs, roiCount: 1, trackedHit: true };
          }
        }
        const fb = detectTwoStage(gray, width, height, fallbackOpts);
        return { ...fb, roiMs: fb.roiMs + roiMs, trackedHit: false };
      }
    }
    const fb = detectTwoStage(gray, width, height, fallbackOpts);
    return { ...fb, trackedHit: false };
  }

  return {
    setDecimate(next) {
      setOptions({ decimate: next });
    },
    setOptions,
    detect(gray, width, height) {
      validateGrayFrame(gray, width, height);
      return rawDetect(gray, width, height);
    },
    detectTwoStage,
    detectTrackedRoi,
  };
}
