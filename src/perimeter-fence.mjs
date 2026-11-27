// Dynamic clockwise table-perimeter fence driven by the fixed L-angle (350-352)
// and movable perimeter markers (358 YL, 353 TL, 357 XT, 354 TR, 359 YR, 355 BR, 356 XB).
// Supports multi-view z = 0 triangulation to promote stationary markers into optical landmarks.
import { undistortPoint } from './rig.mjs';
import { inverse } from './runtime.mjs';

export const FENCE_HEIGHT_M = 0.20;
export const PERIMETER_TRIGGER_TAG_ID = 353;
export const PERIMETER_OUTSIDE_HYSTERESIS_M = Object.freeze({
  enterOutsideM: 0.008,
  exitOutsideM: 0.002,
});
export const PERIMETER_LANDMARK_DEFAULTS = Object.freeze({
  minKeyframes: 6,
  maxKeyframes: 16,
  minKeyframeDistM: 0.04,
  minKeyframeAngleDeg: 4.0,
  minBaselineM: 0.12,
  minParallaxDeg: 8.0,
  minSpanMs: 1000,
  minConfidence: 0.65,
  minEdgePx: 11,
  minCosView: 0.35,
  maxSquareFitRmsMm: 3.5,
  maxPosSpreadMm: 8.0,
  maxYawSpreadDeg: 3.0,
  maxReprojRmsPxXr: 3.0,
  maxReprojRmsPxOptical: 1.6,
  outlierDemoteStreak: 3,
  severeOutlierPx: 6.0,
  severeOutlierStreak: 2,
  lockedJumpDemoteM: 0.025,
});

export const PERIMETER_MARKERS = Object.freeze([
  Object.freeze({ id: 358, role: 'YL' }),
  Object.freeze({ id: 353, role: 'TL' }),
  Object.freeze({ id: 357, role: 'XT' }),
  Object.freeze({ id: 354, role: 'TR' }),
  Object.freeze({ id: 359, role: 'YR' }),
  Object.freeze({ id: 355, role: 'BR' }),
  Object.freeze({ id: 356, role: 'XB' }),
]);

const PERIMETER_BY_ID = new Map(PERIMETER_MARKERS.map((m) => [m.id, m]));

export function isPerimeterTag(id) {
  return PERIMETER_BY_ID.has(id);
}

export function perimeterRole(id) {
  return PERIMETER_BY_ID.get(id)?.role ?? null;
}

function tagCenterXY(tag) {
  const c = tag.corners;
  let sx = 0;
  let sy = 0;
  for (let i = 0; i < c.length; i++) {
    sx += c[i][0];
    sy += c[i][1];
  }
  return [sx / c.length, sy / c.length, 0];
}

export function extractRigAnchors(rig) {
  const tags = rig?.tools?.[0]?.tags;
  if (!Array.isArray(tags) || tags.length < 3) {
    return {
      origin: { id: 350, role: 'BL', fixed: true, pos: [0.0175, 0.015, 0] },
      xArm: { id: 351, role: '+X', fixed: true, pos: [0.2475, 0.015, 0] },
      yArm: { id: 352, role: '+Y', fixed: true, pos: [0.0175, 0.185, 0] },
    };
  }
  return {
    origin: { id: tags[0].id, role: 'BL', fixed: true, pos: tagCenterXY(tags[0]) },
    xArm: { id: tags[1].id, role: '+X', fixed: true, pos: tagCenterXY(tags[1]) },
    yArm: { id: tags[2].id, role: '+Y', fixed: true, pos: tagCenterXY(tags[2]) },
  };
}

// Intersects the camera-to-tag ray with the L-angle table plane (z = 0) when the
// tag sits close to the table. Eliminates horizontal XY shift caused by single-tag
// depth noise while safely falling back to orthographic projection on grazing views.
export function projectPointToTablePlane(tagPosL, cameraPosL = null, {
  maxTableOffsetM = 0.12,
  minCameraHeightM = 0.12,
  minRayDropM = 0.08,
  maxHorizontalShiftM = 0.20,
} = {}) {
  const [tx, ty, tz] = tagPosL;
  if (
    Array.isArray(cameraPosL) &&
    cameraPosL.length >= 3 &&
    Number.isFinite(cameraPosL[0]) &&
    Number.isFinite(cameraPosL[1]) &&
    Number.isFinite(cameraPosL[2]) &&
    Math.abs(tz) <= maxTableOffsetM &&
    cameraPosL[2] >= minCameraHeightM &&
    cameraPosL[2] - tz >= minRayDropM
  ) {
    const t = cameraPosL[2] / (cameraPosL[2] - tz);
    const px = cameraPosL[0] + t * (tx - cameraPosL[0]);
    const py = cameraPosL[1] + t * (ty - cameraPosL[1]);
    if (Number.isFinite(px) && Number.isFinite(py) && Math.hypot(px - tx, py - ty) <= maxHorizontalShiftM) {
      return [px, py, 0];
    }
  }
  return [tx, ty, 0];
}

function extractTableYaw(lFromTag) {
  const x = lFromTag[0];
  const y = lFromTag[1];
  if (Number.isFinite(x) && Number.isFinite(y) && Math.hypot(x, y) > 1e-4) {
    return Math.atan2(y, x);
  }
  return 0;
}

function shortestAngleDelta(from, to) {
  let d = (to - from) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d <= -Math.PI) d += Math.PI * 2;
  return d;
}

function composeTableTagMatrix(pos, yaw) {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return [
    c, s, 0, 0,
    -s, c, 0, 0,
    0, 0, 1, 0,
    pos[0], pos[1], pos[2], 1,
  ];
}

/** Return the 4 L-frame `[x, y]` corners of a `z = 0` square tag in WASM LL, LR, UR, UL order. */
export function tagCornersOnZ0(x, y, yaw, edgeM = 0.020) {
  const h = edgeM * 0.5;
  const dx = Math.cos(yaw) * h;
  const dy = Math.sin(yaw) * h;
  return [
    [x - dx + dy, y - dy - dx],
    [x + dx + dy, y + dy - dx],
    [x + dx - dy, y + dy + dx],
    [x - dx - dy, y - dy + dx],
  ];
}

/**
 * Intersect all 4 (undistorted) corner rays with the L-frame table plane (`z = 0`)
 * and solve a closed-form 2D rigid square `(x, y, yaw)` via Procrustes.
 */
export function fitTagSquareOnZ0(cornersPx, lFromCamera, K, {
  edgeM = 0.020,
  minCameraHeightM = 0.10,
  minRayDrop = 0.08,
  maxDistM = 3.5,
} = {}) {
  if (
    !Array.isArray(cornersPx) || cornersPx.length !== 4 ||
    !lFromCamera || lFromCamera.length < 16 ||
    !K || !(K.fx > 0) || !(K.fy > 0) || !Number.isFinite(K.cx) || !Number.isFinite(K.cy) ||
    !(edgeM > 0)
  ) {
    return null;
  }
  const ox = lFromCamera[12], oy = lFromCamera[13], oz = lFromCamera[14];
  if (!Number.isFinite(ox) || !Number.isFinite(oy) || !Number.isFinite(oz) || oz < minCameraHeightM) {
    return null;
  }
  const k1 = Number.isFinite(K.k1) ? K.k1 : 0;
  const pts = [];
  for (let i = 0; i < 4; i++) {
    const p = cornersPx[i];
    if (!Number.isFinite(p?.x) || !Number.isFinite(p?.y)) return null;
    const up = k1 ? undistortPoint(p.x, p.y, K) : p;
    const xn = (up.x - K.cx) / K.fx;
    const yn = (up.y - K.cy) / K.fy;
    const dx = lFromCamera[0] * xn - lFromCamera[4] * yn - lFromCamera[8];
    const dy = lFromCamera[1] * xn - lFromCamera[5] * yn - lFromCamera[9];
    const dz = lFromCamera[2] * xn - lFromCamera[6] * yn - lFromCamera[10];
    if (!Number.isFinite(dz) || dz > -minRayDrop) return null;
    const t = -oz / dz;
    const rayDist = t * Math.hypot(dx, dy, dz);
    if (!Number.isFinite(rayDist) || t <= 0.08 || rayDist > maxDistM) return null;
    pts.push([ox + t * dx, oy + t * dy]);
  }
  const mx = (pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) * 0.25;
  const my = (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) * 0.25;
  const h = edgeM * 0.5;
  const local = [[-h, -h], [h, -h], [h, h], [-h, h]];
  let A = 0, B = 0;
  for (let i = 0; i < 4; i++) {
    const dx = pts[i][0] - mx, dy = pts[i][1] - my;
    A += local[i][0] * dx + local[i][1] * dy;
    B += local[i][0] * dy - local[i][1] * dx;
  }
  if (Math.hypot(A, B) < 1e-12) return null;
  const yaw = Math.atan2(B, A);
  const cornersL = tagCornersOnZ0(mx, my, yaw, edgeM);
  let sqErr = 0;
  for (let i = 0; i < 4; i++) {
    const ex = cornersL[i][0] - pts[i][0], ey = cornersL[i][1] - pts[i][1];
    sqErr += ex * ex + ey * ey;
  }
  const squareRmsMm = Math.sqrt(sqErr * 0.25) * 1000;
  const vx = ox - mx, vy = oy - my, vz = oz;
  const distM = Math.hypot(vx, vy, vz);
  if (!(distM > 1e-4)) return null;
  return {
    x: mx,
    y: my,
    yaw,
    squareRmsMm,
    cornersL,
    rayDir: [vx / distM, vy / distM, vz / distM],
    distM,
    cosView: vz / distM,
  };
}

function solve3x3(A, b) {
  const m = [
    [A[0][0], A[0][1], A[0][2], b[0]],
    [A[1][0], A[1][1], A[1][2], b[1]],
    [A[2][0], A[2][1], A[2][2], b[2]],
  ];
  for (let c = 0; c < 3; c++) {
    let piv = c;
    for (let r = c + 1; r < 3; r++) {
      if (Math.abs(m[r][c]) > Math.abs(m[piv][c])) piv = r;
    }
    if (Math.abs(m[piv][c]) < 1e-12) return null;
    if (piv !== c) [m[c], m[piv]] = [m[piv], m[c]];
    const invD = 1 / m[c][c];
    for (let j = c; j < 4; j++) m[c][j] *= invD;
    for (let r = 0; r < 3; r++) {
      if (r === c) continue;
      const f = m[r][c];
      for (let j = c; j < 4; j++) m[r][j] -= f * m[c][j];
    }
  }
  const sol = [m[0][3], m[1][3], m[2][3]];
  return sol.every(Number.isFinite) ? sol : null;
}

/**
 * Multi-view 3-DOF `(x, y, yaw)` Pseudo-Huber bundle adjustment constraining a marker
 * to the `z = 0` L-angle table plane directly in distorted pixel space.
 */
export function triangulateTagOnZ0(keyframes, K, {
  edgeM = 0.020,
  maxIterations = 20,
  huberDeltaPx = 2.0,
} = {}) {
  if (!Array.isArray(keyframes) || keyframes.length < 3 || !K || !(K.fx > 0) || !(K.fy > 0)) return null;
  const views = [];
  for (const kf of keyframes) {
    if (!kf?.lFromCamera || !Array.isArray(kf.pixels) || kf.pixels.length !== 4) continue;
    const sq = fitTagSquareOnZ0(kf.pixels, kf.lFromCamera, K, { edgeM });
    const camFromL = inverse(kf.lFromCamera);
    if (!sq || !camFromL || camFromL.length < 16) continue;
    views.push({
      camFromL,
      pixels: kf.pixels,
      sqrtW: Math.sqrt(Math.max(0.1, Number.isFinite(kf.weight) ? kf.weight : 1)),
      sq,
    });
  }
  if (views.length < 3) return null;

  const xs = views.map((v) => v.sq.x).sort((a, b) => a - b);
  const ys = views.map((v) => v.sq.y).sort((a, b) => a - b);
  let x = xs[xs.length >> 1];
  let y = ys[ys.length >> 1];
  let cSum = 0, sSum = 0;
  for (const v of views) {
    cSum += Math.cos(v.sq.yaw);
    sSum += Math.sin(v.sq.yaw);
  }
  let yaw = Math.atan2(sSum, cSum);
  const k1 = Number.isFinite(K.k1) ? K.k1 : 0;

  const evalResiduals = (tx, ty, tyaw) => {
    const corners = tagCornersOnZ0(tx, ty, tyaw, edgeM);
    const weighted = [];
    let rawSq = 0;
    for (let vi = 0; vi < views.length; vi++) {
      const { camFromL: C, pixels: pix, sqrtW } = views[vi];
      for (let i = 0; i < 4; i++) {
        const cx0 = corners[i][0], cy0 = corners[i][1];
        const gx = C[12] + C[0] * cx0 + C[4] * cy0;
        const gy = C[13] + C[1] * cx0 + C[5] * cy0;
        const gz = C[14] + C[2] * cx0 + C[6] * cy0;
        const zcv = -gz;
        if (!(zcv > 0.05)) return null;
        const xn = gx / zcv, yn = -gy / zcv;
        const sc = k1 ? 1 + k1 * (xn * xn + yn * yn) : 1;
        const dx = K.fx * xn * sc + K.cx - pix[i].x;
        const dy = K.fy * yn * sc + K.cy - pix[i].y;
        if (!Number.isFinite(dx) || !Number.isFinite(dy)) return null;
        rawSq += dx * dx + dy * dy;
        weighted.push(sqrtW * dx, sqrtW * dy);
      }
    }
    return { weighted, rawSq };
  };

  let cur = evalResiduals(x, y, yaw);
  if (!cur) return null;
  const huberCost = (arr) => {
    let s = 0;
    for (let i = 0; i < arr.length; i++) {
      const u = arr[i] / huberDeltaPx;
      s += 2 * huberDeltaPx * huberDeltaPx * (Math.sqrt(1 + u * u) - 1);
    }
    return s;
  };
  let cost = huberCost(cur.weighted);
  let lam = 1e-3;
  const eps = 1e-6;
  let lastA = null;

  for (let it = 0; it < maxIterations; it++) {
    const r = cur.weighted;
    const rx = evalResiduals(x + eps, y, yaw)?.weighted;
    const ry = evalResiduals(x, y + eps, yaw)?.weighted;
    const rw = evalResiduals(x, y, yaw + eps)?.weighted;
    if (!rx || !ry || !rw) break;
    const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const b = [0, 0, 0];
    for (let k = 0; k < r.length; k++) {
      const u = r[k] / huberDeltaPx;
      const w = 1 / Math.sqrt(1 + u * u);
      const j0 = (rx[k] - r[k]) / eps;
      const j1 = (ry[k] - r[k]) / eps;
      const j2 = (rw[k] - r[k]) / eps;
      b[0] -= w * j0 * r[k];
      b[1] -= w * j1 * r[k];
      b[2] -= w * j2 * r[k];
      A[0][0] += w * j0 * j0; A[0][1] += w * j0 * j1; A[0][2] += w * j0 * j2;
      A[1][1] += w * j1 * j1; A[1][2] += w * j1 * j2;
      A[2][2] += w * j2 * j2;
    }
    A[1][0] = A[0][1]; A[2][0] = A[0][2]; A[2][1] = A[1][2];
    lastA = A.map((row) => row.slice());
    for (let d = 0; d < 3; d++) A[d][d] += lam * Math.max(1, A[d][d]);
    const step = solve3x3(A, b);
    if (!step) break;
    const next = evalResiduals(x + step[0], y + step[1], yaw + step[2]);
    const nextCost = next ? huberCost(next.weighted) : Infinity;
    if (nextCost < cost) {
      x += step[0];
      y += step[1];
      yaw = shortestAngleDelta(0, yaw + step[2]);
      cur = next;
      cost = nextCost;
      lam = Math.max(1e-9, lam / 3);
      if (Math.hypot(step[0], step[1], step[2]) < 1e-8) break;
    } else {
      lam *= 10;
      if (lam > 1e10) break;
    }
  }

  const rmsPx = Math.sqrt(cur.rawSq / (views.length * 4));
  let stdXYMm = Infinity, stdYawDeg = Infinity;
  if (lastA) {
    const dof = Math.max(1, 2 * 4 * views.length - 3);
    const varW = cur.weighted.reduce((s, v) => s + v * v, 0) / dof;
    const cx = solve3x3(lastA, [1, 0, 0]);
    const cy = solve3x3(lastA, [0, 1, 0]);
    const cw = solve3x3(lastA, [0, 0, 1]);
    if (cx && cy && cx[0] > 0 && cy[1] > 0) stdXYMm = Math.sqrt((cx[0] + cy[1]) * varW) * 1000;
    if (cw && cw[2] > 0) stdYawDeg = Math.sqrt(cw[2] * varW) * (180 / Math.PI);
  }
  return {
    x,
    y,
    yaw,
    rmsPx,
    corners: tagCornersOnZ0(x, y, yaw, edgeM),
    views: views.length,
    stdXYMm,
    stdYawDeg,
  };
}

export function polygonMetricsXY(vertices) {
  const n = vertices.length;
  if (n < 2) return { perimeterM: 0, areaM2: 0 };
  let perimeterM = 0;
  let signedArea2 = 0;
  for (let i = 0; i < n; i++) {
    const a = vertices[i].pos;
    const b = vertices[(i + 1) % n].pos;
    perimeterM += Math.hypot(b[0] - a[0], b[1] - a[1]);
    signedArea2 += a[0] * b[1] - b[0] * a[1];
  }
  return {
    perimeterM,
    areaM2: n >= 3 ? Math.abs(signedArea2) * 0.5 : 0,
  };
}

/**
 * Evaluate exact 2D XY point-in-polygon membership and signed distance to the polygon
 * boundary (negative inside, positive outside) using ray-casting + segment distance.
 */
export function signedDistanceToPolygonXY(x, y, vertices) {
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Array.isArray(vertices) || vertices.length < 3) {
    return { inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  const n = vertices.length;
  let inside = false;
  let minDistSq = Infinity;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const va = vertices[j]?.pos ?? vertices[j];
    const vb = vertices[i]?.pos ?? vertices[i];
    if (!va || !vb) continue;
    const ax = va[0], ay = va[1];
    const bx = vb[0], by = vb[1];
    if (!Number.isFinite(ax) || !Number.isFinite(ay) || !Number.isFinite(bx) || !Number.isFinite(by)) continue;

    if (((ay > y) !== (by > y)) && (x < ((bx - ax) * (y - ay)) / (by - ay) + ax)) {
      inside = !inside;
    }

    const abx = bx - ax;
    const aby = by - ay;
    const abLenSq = abx * abx + aby * aby;
    let px = ax;
    let py = ay;
    if (abLenSq > 1e-12) {
      const t = Math.max(0, Math.min(1, ((x - ax) * abx + (y - ay) * aby) / abLenSq));
      px = ax + t * abx;
      py = ay + t * aby;
    }
    const dx = x - px;
    const dy = y - py;
    const dSq = dx * dx + dy * dy;
    if (dSq < minDistSq) minDistSq = dSq;
  }
  const edgeDistanceM = Number.isFinite(minDistSq) ? Math.sqrt(minDistSq) : 0;
  if (edgeDistanceM <= 1e-9) {
    return { inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  return {
    inside,
    signedDistanceM: inside ? -edgeDistanceM : edgeDistanceM,
    edgeDistanceM,
  };
}

/**
 * Evaluate whether a 3D L-frame tag center lies outside the XY perimeter polygon,
 * applying hysteresis (enterOutsideM / exitOutsideM) to prevent boundary flicker.
 */
export function evaluatePerimeterOutside(posL, vertices, {
  wasOutside = false,
  enterOutsideM = PERIMETER_OUTSIDE_HYSTERESIS_M.enterOutsideM,
  exitOutsideM = PERIMETER_OUTSIDE_HYSTERESIS_M.exitOutsideM,
} = {}) {
  const x = posL?.[0];
  const y = posL?.[1];
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Array.isArray(vertices) || vertices.length < 3) {
    return { outside: false, inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
  }
  const res = signedDistanceToPolygonXY(x, y, vertices);
  const threshold = wasOutside ? exitOutsideM : enterOutsideM;
  const outside = res.signedDistanceM > threshold;
  return {
    outside,
    inside: !outside,
    signedDistanceM: res.signedDistanceM,
    edgeDistanceM: res.edgeDistanceM,
  };
}

/**
 * Locate where a 3D L-frame path (prevPosL -> currPosL) intersects the 2D XY
 * perimeter boundary, falling back to the closest perimeter point when no
 * segment intersection exists. Returns winding-agnostic outward normal, arc
 * length along the perimeter, and clamped height along the fence wall.
 */
export function locatePerimeterCrossing(prevPosL, currPosL, vertices, {
  heightM = FENCE_HEIGHT_M,
  minZ = 0.015,
} = {}) {
  const x1 = currPosL?.[0];
  const y1 = currPosL?.[1];
  const z1 = currPosL?.[2];
  if (!Number.isFinite(x1) || !Number.isFinite(y1) || !Array.isArray(vertices) || vertices.length < 3) {
    return null;
  }

  const pts = [];
  for (let i = 0; i < vertices.length; i++) {
    const v = vertices[i]?.pos ?? vertices[i];
    if (v && Number.isFinite(v[0]) && Number.isFinite(v[1])) {
      pts.push(v);
    }
  }
  const n = pts.length;
  if (n < 3) return null;

  const edgeLen = new Array(n);
  const cumS = new Array(n);
  let signedArea2 = 0;
  let perimeterM = 0;
  for (let i = 0; i < n; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % n];
    const ax = a[0], ay = a[1];
    const bx = b[0], by = b[1];
    signedArea2 += ax * by - bx * ay;
    const len = Math.hypot(bx - ax, by - ay);
    edgeLen[i] = len;
    cumS[i] = perimeterM;
    perimeterM += len;
  }
  if (perimeterM <= 1e-9) return null;

  const windingSign = signedArea2 <= 0 ? 1 : -1;

  let hit = null;
  let bestU = -1;
  let hitX = x1;
  let hitY = y1;
  let rawZ = z1;
  let bestEdge = 0;
  let bestT = 0;
  let bestArcS = 0;
  let bestNx = 1;
  let bestNy = 0;
  let bestDist = 0;

  const x0 = prevPosL?.[0];
  const y0 = prevPosL?.[1];
  const z0 = prevPosL?.[2];
  if (Number.isFinite(x0) && Number.isFinite(y0)) {
    const dx = x1 - x0;
    const dy = y1 - y0;
    if (Math.hypot(dx, dy) > 1e-6) {
      const z0Finite = Number.isFinite(z0);
      const z1Finite = Number.isFinite(z1);
      for (let i = 0; i < n; i++) {
        const L = edgeLen[i];
        if (L <= 1e-9) continue;
        const a = pts[i];
        const b = pts[(i + 1) % n];
        const ax = a[0], ay = a[1];
        const ex = b[0] - ax;
        const ey = b[1] - ay;
        const denom = dx * ey - dy * ex;
        if (Math.abs(denom) <= 1e-12) continue;
        const u = ((ax - x0) * ey - (ay - y0) * ex) / denom;
        const t = ((ax - x0) * dy - (ay - y0) * dx) / denom;
        if (u >= -1e-6 && u <= 1 + 1e-6 && t >= -1e-6 && t <= 1 + 1e-6) {
          const uc = Math.max(0, Math.min(1, u));
          if (uc > bestU) {
            const tc = Math.max(0, Math.min(1, t));
            bestU = uc;
            hitX = ax + tc * ex;
            hitY = ay + tc * ey;
            rawZ = z0Finite && z1Finite ? z0 + uc * (z1 - z0) : z1;
            bestEdge = i;
            bestT = tc;
            bestArcS = cumS[i] + tc * L;
            bestNx = (-ey / L) * windingSign;
            bestNy = (ex / L) * windingSign;
            bestDist = 0;
            hit = 'intersection';
          }
        }
      }
    }
  }

  if (!hit) {
    let minDistSq = Infinity;
    for (let i = 0; i < n; i++) {
      const L = edgeLen[i];
      if (L <= 1e-9) continue;
      const a = pts[i];
      const b = pts[(i + 1) % n];
      const ax = a[0], ay = a[1];
      const ex = b[0] - ax;
      const ey = b[1] - ay;
      const t = Math.max(0, Math.min(1, ((x1 - ax) * ex + (y1 - ay) * ey) / (L * L)));
      const px = ax + t * ex;
      const py = ay + t * ey;
      const dSq = (x1 - px) * (x1 - px) + (y1 - py) * (y1 - py);
      if (dSq < minDistSq) {
        minDistSq = dSq;
        hitX = px;
        hitY = py;
        rawZ = z1;
        bestEdge = i;
        bestT = t;
        bestArcS = cumS[i] + t * L;
        bestNx = (-ey / L) * windingSign;
        bestNy = (ex / L) * windingSign;
      }
    }
    bestDist = Number.isFinite(minDistSq) ? Math.sqrt(minDistSq) : 0;
    hit = 'closest';
  }

  const zRaw = Number.isFinite(rawZ) ? rawZ : heightM * 0.5;
  const overTop = zRaw > heightM;
  const zClamped = Math.max(Math.min(minZ, heightM), Math.min(heightM, zRaw));

  return {
    pos: [hitX, hitY, zClamped],
    rawZ: zRaw,
    overTop,
    normal: [bestNx, bestNy],
    edgeIndex: bestEdge,
    edgeT: bestT,
    arcS: bestArcS,
    perimeterM,
    distanceM: bestDist,
    hit,
  };
}

export function isTablePointObservableInCamera(posL, lFromCamera, K, {
  minDistM = 0.22,
  maxDistM = 1.30,
  minCosView = 0.50,
  borderMarginFrac = 0.08,
  minBorderMarginPx = 64,
  deadZoneM = 0.04,
  minExpectedMinorPx = 7.0,
  decimate = 1,
  priorRois = null,
  tagEdgeM = 0.020,
} = {}) {
  if (
    !Array.isArray(posL) || posL.length < 3 ||
    !lFromCamera || lFromCamera.length < 15 ||
    !K || !(K.fx > 0) || !(K.fy > 0) ||
    !Number.isFinite(K.cx) || !Number.isFinite(K.cy) ||
    !(K.width >= 32) || !(K.height >= 32)
  ) {
    return false;
  }
  const px = posL[0], py = posL[1], pz = posL[2];
  const tx = lFromCamera[12], ty = lFromCamera[13], tz = lFromCamera[14];
  if (
    !Number.isFinite(px) || !Number.isFinite(py) || !Number.isFinite(pz) ||
    !Number.isFinite(tx) || !Number.isFinite(ty) || !Number.isFinite(tz)
  ) {
    return false;
  }
  const dx = px - tx;
  const dy = py - ty;
  const dz = pz - tz;
  const dist = Math.hypot(dx, dy, dz);
  if (!Number.isFinite(dist) || dist < minDistM || dist > maxDistM) return false;

  const cosView = (tz - pz) / dist;
  if (!Number.isFinite(cosView) || cosView < minCosView) return false;

  // Rigid transpose inverse: p_GL = R^T * (p_L - t), then GL -> OpenCV (x, -y, -z).
  const xCV = lFromCamera[0] * dx + lFromCamera[1] * dy + lFromCamera[2] * dz;
  const yCV = -(lFromCamera[4] * dx + lFromCamera[5] * dy + lFromCamera[6] * dz);
  const zCV = -(lFromCamera[8] * dx + lFromCamera[9] * dy + lFromCamera[10] * dz);
  if (!Number.isFinite(zCV) || zCV < minDistM || zCV > maxDistM) return false;

  const k1 = Number.isFinite(K.k1) ? K.k1 : 0;
  const xn = xCV / zCV, yn = yCV / zCV;
  const sc = k1 ? 1 + k1 * (xn * xn + yn * yn) : 1;
  const u = K.fx * xn * sc + K.cx;
  const v = K.fy * yn * sc + K.cy;
  if (!Number.isFinite(u) || !Number.isFinite(v)) return false;

  // Include a spatial dead-zone around the expected marker location on the table
  // so small XR drift or edge clipping cannot trigger false removal.
  const deadZonePx = Number.isFinite(deadZoneM) && deadZoneM > 0
    ? (deadZoneM * Math.max(K.fx, K.fy)) / zCV
    : 0;
  const marginX = Math.max(minBorderMarginPx, K.width * borderMarginFrac) + deadZonePx;
  const marginY = Math.max(minBorderMarginPx, K.height * borderMarginFrac) + deadZonePx;
  if (u < marginX || u > K.width - marginX || v < marginY || v > K.height - marginY) return false;

  // Check foreshortened minor axis AFTER detector decimation so far or oblique tags
  // that AprilTag cannot reliably decode on this frame are never falsely pruned.
  // If a two-stage priorRoi covered this marker's pixel position, it was refined at decimate = 1.
  const inPriorRoi = Array.isArray(priorRois) && priorRois.some(
    (b) => Array.isArray(b) && b.length >= 4 && u >= b[0] + 12 && u <= b[2] - 12 && v >= b[1] + 12 && v <= b[3] - 12,
  );
  const effDecimate = inPriorRoi ? 1 : (Number.isFinite(decimate) && decimate >= 1 ? decimate : 1);
  const expectedEdgePx = (tagEdgeM * Math.min(K.fx, K.fy)) / zCV;
  const expectedMinorPx = (expectedEdgePx * cosView) / effDecimate;
  return Number.isFinite(expectedMinorPx) && expectedMinorPx >= minExpectedMinorPx;
}

export function createPerimeterFence(rig, {
  heightM = FENCE_HEIGHT_M,
  triggerTagId = PERIMETER_TRIGGER_TAG_ID,
  smoothTauSec = 0.065,
  jumpDistanceM = 0.15,
  jumpWindowMs = 500,
  jumpConfirmM = 0.06,
  missingFramesToRemove = 8,
  missingMsToRemove = 2500,
  minGraceAfterSeenMs = 500,
  minRemoveConfidence = 0.55,
  inViewMinDistM = 0.22,
  inViewMaxDistM = 1.30,
  inViewMinCosView = 0.50,
  inViewBorderMarginFrac = 0.08,
  inViewMinBorderMarginPx = 64,
  inViewDeadZoneM = 0.04,
  inViewMinExpectedMinorPx = 7.0,
  tagEdgeM = 0.020,
  coordinateMode: initialCoordinateMode = 'planar-z0',
  landmark = {},
} = {}) {
  const lmCfg = { ...PERIMETER_LANDMARK_DEFAULTS, ...landmark };
  let coordinateMode = initialCoordinateMode === 'free-6dof' ? 'free-6dof' : 'planar-z0';
  let anchors = extractRigAnchors(rig);
  const markers = new Map();
  let active = false;
  let dirty = true;
  let stateVersion = 0;
  const cachedPolyXY = [];
  let cachedPolyVer = -1;

  function getCoordinateMode() {
    return coordinateMode;
  }

  function setCoordinateMode(nextMode) {
    const resolved = nextMode === 'free-6dof' ? 'free-6dof' : 'planar-z0';
    if (resolved === coordinateMode) return coordinateMode;
    coordinateMode = resolved;
    for (const m of markers.values()) {
      if (resolved === 'free-6dof') {
        m.landmarkState = 'tracking';
        m.lockedPose = null;
        m.keyframes.length = 0;
        m.outlierStreak = 0;
        m.severeOutlierStreak = 0;
        m.pendingDemotePos = null;
      } else {
        m.targetPos[2] = 0;
        m.currentPos[2] = 0;
      }
    }
    dirty = true;
    stateVersion++;
    return coordinateMode;
  }

  function isActive() {
    return active;
  }

  function isOffered() {
    return !active && markers.has(triggerTagId);
  }

  function status() {
    if (active) return 'active';
    if (markers.has(triggerTagId)) return 'offered';
    return 'idle';
  }

  function triggerMarker() {
    const m = markers.get(triggerTagId);
    if (!m) return null;
    return {
      id: m.id,
      role: m.role,
      pos: m.currentPos.slice(),
      yaw: m.currentYaw,
      matrix: composeTableTagMatrix(m.currentPos, m.currentYaw),
      lastAt: m.lastAt,
    };
  }

  function activate({ force = false } = {}) {
    if (active) return false;
    if (!force && !markers.has(triggerTagId)) return false;
    active = true;
    dirty = true;
    stateVersion++;
    return true;
  }

  function deactivate() {
    if (!active) return false;
    active = false;
    dirty = true;
    stateVersion++;
    return true;
  }

  function setRig(nextRig) {
    anchors = extractRigAnchors(nextRig);
    dirty = true;
    stateVersion++;
  }

  function reset() {
    markers.clear();
    active = false;
    dirty = true;
    stateVersion++;
  }

  function maybePromoteLandmark(m, K, atMs) {
    if (coordinateMode !== 'planar-z0' || m.keyframes.length < lmCfg.minKeyframes) return false;
    const kfs = m.keyframes;
    let minT = Infinity, maxT = -Infinity;
    let maxBase = 0, minDot = 1, opticalCount = 0;
    for (let i = 0; i < kfs.length; i++) {
      const a = kfs[i];
      if (a.atMs < minT) minT = a.atMs;
      if (a.atMs > maxT) maxT = a.atMs;
      if (a.optical) opticalCount++;
      for (let j = i + 1; j < kfs.length; j++) {
        const b = kfs[j];
        const bd = Math.hypot(a.camPosL[0] - b.camPosL[0], a.camPosL[1] - b.camPosL[1], a.camPosL[2] - b.camPosL[2]);
        if (bd > maxBase) maxBase = bd;
        const dt = a.rayDir[0] * b.rayDir[0] + a.rayDir[1] * b.rayDir[1] + a.rayDir[2] * b.rayDir[2];
        if (dt < minDot) minDot = dt;
      }
    }
    const maxParallaxDeg = Math.acos(Math.max(-1, Math.min(1, minDot))) * (180 / Math.PI);
    if (maxT - minT < lmCfg.minSpanMs || maxBase < lmCfg.minBaselineM || maxParallaxDeg < lmCfg.minParallaxDeg) {
      return false;
    }
    const xs = kfs.map((k) => k.x).sort((a, b) => a - b);
    const ys = kfs.map((k) => k.y).sort((a, b) => a - b);
    const mx = xs[xs.length >> 1], my = ys[ys.length >> 1];
    let cSum = 0, sSum = 0;
    for (const k of kfs) { cSum += Math.cos(k.yaw); sSum += Math.sin(k.yaw); }
    const mYaw = Math.atan2(sSum, cSum);
    let maxPosM = 0, maxYawDeg = 0;
    for (const k of kfs) {
      const d = Math.hypot(k.x - mx, k.y - my);
      if (d > maxPosM) maxPosM = d;
      const dy = Math.abs(shortestAngleDelta(mYaw, k.yaw)) * (180 / Math.PI);
      if (dy > maxYawDeg) maxYawDeg = dy;
    }
    if (maxPosM * 1000 > lmCfg.maxPosSpreadMm || maxYawDeg > lmCfg.maxYawSpreadDeg) return false;

    const tri = triangulateTagOnZ0(kfs, K, { edgeM: tagEdgeM });
    if (!tri) return false;
    const rmsCap = (opticalCount / kfs.length) >= 0.5 ? lmCfg.maxReprojRmsPxOptical : lmCfg.maxReprojRmsPxXr;
    if (!Number.isFinite(tri.rmsPx) || tri.rmsPx > rmsCap) return false;

    m.landmarkState = 'locked';
    m.lockedPose = {
      x: tri.x,
      y: tri.y,
      yaw: tri.yaw,
      corners: tri.corners,
      rmsPx: tri.rmsPx,
      baselineM: maxBase,
      parallaxDeg: maxParallaxDeg,
      views: tri.views,
      lockedAt: atMs,
    };
    m.targetPos[0] = tri.x;
    m.targetPos[1] = tri.y;
    m.targetPos[2] = 0;
    m.targetYaw = tri.yaw;
    m.outlierStreak = 0;
    m.severeOutlierStreak = 0;
    m.pendingDemotePos = null;
    dirty = true;
    stateVersion++;
    return true;
  }

  function recordKeyframe(m, sqFit, corners, lFromCamera, K, atMs, { weight = 1, minEdgePx = 12, poseSource = 'xr-history' } = {}) {
    if (
      coordinateMode !== 'planar-z0' ||
      m.landmarkState === 'locked' ||
      !sqFit || !lFromCamera || !K ||
      !Array.isArray(corners) || corners.length !== 4 ||
      weight < lmCfg.minConfidence ||
      minEdgePx < lmCfg.minEdgePx ||
      sqFit.cosView < lmCfg.minCosView ||
      sqFit.squareRmsMm > lmCfg.maxSquareFitRmsMm
    ) {
      return false;
    }
    const camPosL = [lFromCamera[12], lFromCamera[13], lFromCamera[14]];
    const minAngleCos = Math.cos((lmCfg.minKeyframeAngleDeg * Math.PI) / 180);
    for (const kf of m.keyframes) {
      const dist = Math.hypot(camPosL[0] - kf.camPosL[0], camPosL[1] - kf.camPosL[1], camPosL[2] - kf.camPosL[2]);
      const dotR = sqFit.rayDir[0] * kf.rayDir[0] + sqFit.rayDir[1] * kf.rayDir[1] + sqFit.rayDir[2] * kf.rayDir[2];
      if (dist < lmCfg.minKeyframeDistM && dotR > minAngleCos) {
        return false;
      }
    }
    const optical = poseSource === 'optical-l' || poseSource === 'optical-perimeter';
    const entry = {
      lFromCamera: Array.from(lFromCamera),
      camPosL,
      rayDir: sqFit.rayDir,
      pixels: corners.map((p) => ({ x: p.x, y: p.y })),
      x: sqFit.x,
      y: sqFit.y,
      yaw: sqFit.yaw,
      atMs,
      weight: optical ? 2 : 1,
      optical,
    };
    if (m.keyframes.length >= lmCfg.maxKeyframes) {
      const dropIdx = m.keyframes.findIndex((k) => !k.optical);
      m.keyframes.splice(dropIdx >= 0 ? dropIdx : 0, 1);
    }
    m.keyframes.push(entry);
    return maybePromoteLandmark(m, K, atMs);
  }

  function observe(tagId, lFromTag, cameraPosL, atMs, {
    weight = 1,
    corners = null,
    lFromCamera = null,
    K = null,
    minEdgePx = 12,
    poseSource = 'xr-history',
  } = {}) {
    const spec = PERIMETER_BY_ID.get(tagId);
    if (!spec || !lFromTag || lFromTag.length < 15) return null;
    const rawPos = [lFromTag[12], lFromTag[13], lFromTag[14]];
    if (!rawPos.every(Number.isFinite)) return null;
    const sqFit = (coordinateMode === 'planar-z0' && Array.isArray(corners) && lFromCamera && K)
      ? fitTagSquareOnZ0(corners, lFromCamera, K, { edgeM: tagEdgeM })
      : null;
    const useSq = Boolean(sqFit && sqFit.squareRmsMm <= lmCfg.maxSquareFitRmsMm);
    const pos = coordinateMode === 'free-6dof'
      ? rawPos.slice()
      : (useSq ? [sqFit.x, sqFit.y, 0] : projectPointToTablePlane(rawPos, cameraPosL));
    const yaw = useSq ? sqFit.yaw : extractTableYaw(lFromTag);
    const w = Number.isFinite(weight) ? Math.max(0, Math.min(1, weight)) : 1;
    const existing = markers.get(tagId);
    if (existing) {
      existing.missCount = 0;
      existing.missSinceMs = null;
    }
    if (w <= 0) {
      return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-frozen' };
    }
    const wasOffered = isOffered();

    if (!existing) {
      if (w < 0.45) {
        return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-low-confidence' };
      }
      const entry = {
        id: tagId,
        role: spec.role,
        fixed: false,
        targetPos: pos.slice(),
        currentPos: pos.slice(),
        targetYaw: yaw,
        currentYaw: yaw,
        lastAt: atMs,
        pendingJump: null,
        updates: 1,
        missCount: 0,
        missSinceMs: null,
        landmarkState: 'tracking',
        keyframes: [],
        lockedPose: null,
        outlierStreak: 0,
        severeOutlierStreak: 0,
        pendingDemotePos: null,
      };
      markers.set(tagId, entry);
      if (sqFit) recordKeyframe(entry, sqFit, corners, lFromCamera, K, atMs, { weight: w, minEdgePx, poseSource });
      dirty = true;
      stateVersion++;
      return {
        accepted: true,
        isNew: true,
        justOffered: !wasOffered && isOffered(),
        justLocked: entry.landmarkState === 'locked',
        landmarkState: entry.landmarkState,
        status: status(),
        id: tagId,
        role: spec.role,
        pos: entry.targetPos.slice(),
      };
    }

    if (existing.landmarkState === 'locked' && existing.lockedPose) {
      const distLocked = Math.hypot(pos[0] - existing.lockedPose.x, pos[1] - existing.lockedPose.y);
      if (w >= lmCfg.minConfidence && distLocked > lmCfg.lockedJumpDemoteM) {
        if (
          existing.pendingDemotePos &&
          Math.hypot(pos[0] - existing.pendingDemotePos[0], pos[1] - existing.pendingDemotePos[1]) <= 0.03
        ) {
          existing.landmarkState = 'demoted';
          existing.lockedPose = null;
          existing.keyframes.length = 0;
          existing.outlierStreak = 0;
          existing.severeOutlierStreak = 0;
          existing.pendingDemotePos = null;
          existing.targetPos[0] = pos[0];
          existing.targetPos[1] = pos[1];
          existing.targetPos[2] = coordinateMode === 'free-6dof' ? pos[2] : 0;
          existing.targetYaw = yaw;
          existing.lastAt = atMs;
          existing.updates++;
          dirty = true;
          stateVersion++;
          return {
            accepted: true,
            isNew: false,
            demoted: true,
            landmarkState: 'demoted',
            status: status(),
            id: tagId,
            role: spec.role,
            pos,
          };
        }
        existing.pendingDemotePos = pos.slice();
        existing.lastAt = atMs;
        return {
          accepted: false,
          isNew: false,
          locked: true,
          landmarkState: 'locked',
          status: status(),
          id: tagId,
          role: spec.role,
          pos: existing.targetPos.slice(),
          reason: 'locked-jump-pending',
        };
      }
      existing.pendingDemotePos = null;
      existing.lastAt = atMs;
      return {
        accepted: true,
        isNew: false,
        locked: true,
        landmarkState: 'locked',
        status: status(),
        id: tagId,
        role: spec.role,
        pos: existing.targetPos.slice(),
      };
    }

    const dist = Math.hypot(pos[0] - existing.targetPos[0], pos[1] - existing.targetPos[1]);
    if (dist > jumpDistanceM && atMs - existing.lastAt < jumpWindowMs) {
      if (
        !existing.pendingJump ||
        Math.hypot(pos[0] - existing.pendingJump[0], pos[1] - existing.pendingJump[1]) > jumpConfirmM
      ) {
        existing.pendingJump = pos.slice();
        return {
          accepted: false,
          isNew: false,
          justOffered: false,
          status: status(),
          id: tagId,
          role: spec.role,
          pos,
        };
      }
      existing.keyframes.length = 0;
      if (existing.landmarkState === 'demoted') existing.landmarkState = 'tracking';
    }

    if (w < 0.6 && dist < 0.008) {
      existing.lastAt = atMs;
      return { accepted: false, isNew: false, id: tagId, role: spec.role, pos, reason: 'motion-deadband' };
    }

    existing.pendingJump = null;
    if (w >= 0.999) {
      existing.targetPos[0] = pos[0];
      existing.targetPos[1] = pos[1];
      existing.targetYaw = yaw;
    } else {
      existing.targetPos[0] += (pos[0] - existing.targetPos[0]) * w;
      existing.targetPos[1] += (pos[1] - existing.targetPos[1]) * w;
      existing.targetYaw = shortestAngleDelta(0, existing.targetYaw + shortestAngleDelta(existing.targetYaw, yaw) * w);
    }
    existing.targetPos[2] = coordinateMode === 'free-6dof' ? pos[2] : 0;
    existing.lastAt = atMs;
    existing.updates++;
    const justLocked = sqFit
      ? recordKeyframe(existing, sqFit, corners, lFromCamera, K, atMs, { weight: w, minEdgePx, poseSource })
      : false;
    dirty = true;
    stateVersion++;
    return {
      accepted: true,
      isNew: false,
      justOffered: false,
      justLocked,
      landmarkState: existing.landmarkState,
      status: status(),
      id: tagId,
      role: spec.role,
      pos: existing.targetPos.slice(),
    };
  }

  function lockedLandmarks() {
    if (coordinateMode !== 'planar-z0') return [];
    const list = [];
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m?.landmarkState === 'locked' && m.lockedPose) {
        list.push({
          id: m.id,
          role: m.role,
          x: m.lockedPose.x,
          y: m.lockedPose.y,
          yaw: m.lockedPose.yaw,
          points: m.lockedPose.corners.map((c) => c.slice()),
          rmsPx: m.lockedPose.rmsPx,
          baselineM: m.lockedPose.baselineM,
          parallaxDeg: m.lockedPose.parallaxDeg,
          views: m.lockedPose.views,
        });
      }
    }
    return list;
  }

  function lockedCount() {
    if (coordinateMode !== 'planar-z0') return 0;
    let count = 0;
    for (const m of markers.values()) {
      if (m.landmarkState === 'locked' && m.lockedPose) count++;
    }
    return count;
  }

  function reportLandmarkInlier(tagId) {
    const m = markers.get(tagId);
    if (!m) return false;
    m.outlierStreak = 0;
    m.severeOutlierStreak = 0;
    return true;
  }

  function reportLandmarkOutlier(tagId, { residualPx = Infinity } = {}) {
    const m = markers.get(tagId);
    if (!m || m.landmarkState !== 'locked') return null;
    m.outlierStreak++;
    if (residualPx >= lmCfg.severeOutlierPx) m.severeOutlierStreak++;
    else m.severeOutlierStreak = 0;
    if (m.outlierStreak >= lmCfg.outlierDemoteStreak || m.severeOutlierStreak >= lmCfg.severeOutlierStreak) {
      m.landmarkState = 'demoted';
      m.lockedPose = null;
      m.keyframes.length = 0;
      m.outlierStreak = 0;
      m.severeOutlierStreak = 0;
      m.pendingDemotePos = null;
      dirty = true;
      stateVersion++;
      return { demoted: true, id: m.id, role: m.role, residualPx };
    }
    return { demoted: false, id: m.id, streak: m.outlierStreak };
  }

  function remove(tagId) {
    const m = markers.get(tagId);
    if (!m) return null;
    markers.delete(tagId);
    let deactivated = false;
    if (active && markers.size === 0) {
      active = false;
      deactivated = true;
    }
    dirty = true;
    stateVersion++;
    return {
      removed: true,
      id: m.id,
      role: m.role,
      pos: m.currentPos.slice(),
      deactivated,
      status: status(),
    };
  }

  const inViewOpts = {
    minDistM: inViewMinDistM,
    maxDistM: inViewMaxDistM,
    minCosView: inViewMinCosView,
    borderMarginFrac: inViewBorderMarginFrac,
    minBorderMarginPx: inViewMinBorderMarginPx,
    deadZoneM: inViewDeadZoneM,
    minExpectedMinorPx: inViewMinExpectedMinorPx,
    decimate: 1,
    priorRois: null,
    tagEdgeM,
  };

  function reconcileView({
    seenIds = [],
    lFromCamera = null,
    K = null,
    atMs = NaN,
    confidence = 1,
    occludedIds = null,
    decimate = 1,
    priorRois = null,
  } = {}) {
    if (markers.size === 0) {
      return { removed: [], deactivated: false, status: status() };
    }
    const seenSet = seenIds instanceof Set ? seenIds : new Set(seenIds);
    const occludedSet = occludedIds
      ? (occludedIds instanceof Set ? occludedIds : new Set(occludedIds))
      : null;

    const effDec = Number.isFinite(decimate) && decimate >= 1 ? decimate : 1;
    const hasPriors = Array.isArray(priorRois) && priorRois.length > 0;
    if (
      !(confidence >= minRemoveConfidence) ||
      (effDec > 2 && !hasPriors) ||
      !lFromCamera ||
      !K ||
      !Number.isFinite(atMs)
    ) {
      for (const id of seenSet) {
        const m = markers.get(id);
        if (m) {
          m.missCount = 0;
          m.missSinceMs = null;
        }
      }
      return { removed: [], deactivated: false, status: status() };
    }

    inViewOpts.decimate = effDec;
    inViewOpts.priorRois = hasPriors ? priorRois : null;
    const toRemove = [];
    for (const [id, m] of markers) {
      if (seenSet.has(id) || occludedSet?.has(id)) {
        m.missCount = 0;
        m.missSinceMs = null;
        continue;
      }
      if (!isTablePointObservableInCamera(m.currentPos, lFromCamera, K, inViewOpts)) {
        m.missCount = 0;
        m.missSinceMs = null;
        continue;
      }
      if (m.lastAt > 0 && atMs - m.lastAt < minGraceAfterSeenMs) {
        continue;
      }
      if (m.missSinceMs === null) {
        m.missSinceMs = atMs;
      }
      m.missCount++;
      if (m.missCount >= missingFramesToRemove && atMs - m.missSinceMs >= missingMsToRemove) {
        toRemove.push(id);
      }
    }

    if (toRemove.length === 0) {
      return { removed: [], deactivated: false, status: status() };
    }

    const removed = [];
    for (const id of toRemove) {
      const m = markers.get(id);
      if (!m) continue;
      markers.delete(id);
      removed.push({ id: m.id, role: m.role, pos: m.currentPos.slice() });
    }
    let deactivated = false;
    if (active && markers.size === 0) {
      active = false;
      deactivated = true;
    }
    dirty = true;
    stateVersion++;
    return { removed, deactivated, status: status() };
  }

  function step(dtSec) {
    const wasDirty = dirty;
    dirty = false;
    let moved = false;
    const dt = Number.isFinite(dtSec) && dtSec > 0 ? Math.min(dtSec, 0.25) : 0;
    const alpha = smoothTauSec > 0 && dt > 0 ? 1 - Math.exp(-dt / smoothTauSec) : 1;

    for (const m of markers.values()) {
      const dx = m.targetPos[0] - m.currentPos[0];
      const dy = m.targetPos[1] - m.currentPos[1];
      const dz = m.targetPos[2] - m.currentPos[2];
      const dist = Math.hypot(dx, dy, dz);
      if (dist <= 0.0004) {
        if (dx !== 0 || dy !== 0 || dz !== 0) {
          m.currentPos[0] = m.targetPos[0];
          m.currentPos[1] = m.targetPos[1];
          m.currentPos[2] = m.targetPos[2];
          moved = true;
        }
      } else {
        m.currentPos[0] += dx * alpha;
        m.currentPos[1] += dy * alpha;
        m.currentPos[2] += dz * alpha;
        moved = true;
      }

      const dYaw = shortestAngleDelta(m.currentYaw, m.targetYaw);
      if (Math.abs(dYaw) <= 0.002) {
        if (dYaw !== 0) {
          m.currentYaw = m.targetYaw;
          moved = true;
        }
      } else {
        m.currentYaw = shortestAngleDelta(0, m.currentYaw + dYaw * alpha);
        moved = true;
      }
    }
    if (moved) stateVersion++;
    return wasDirty || moved;
  }

  function serializeMarker(m) {
    return {
      id: m.id,
      role: m.role,
      fixed: false,
      pos: m.currentPos.slice(),
      yaw: m.currentYaw,
      matrix: composeTableTagMatrix(m.currentPos, m.currentYaw),
      lastAt: m.lastAt,
      updates: m.updates,
      landmarkState: m.landmarkState ?? 'tracking',
      landmarkRmsPx: m.lockedPose?.rmsPx ?? null,
    };
  }

  // Strictly clockwise around the table viewed from +Z:
  // 350 (BL) -> 352 (+Y) -> 358 (YL) -> 353 (TL) -> 357 (XT) -> 354 (TR)
  // -> 359 (YR) -> 355 (BR) -> 356 (XB) -> 351 (+X) -> back to 350 (BL).
  function orderedVertices() {
    const verts = [
      { id: anchors.origin.id, role: anchors.origin.role, fixed: true, pos: anchors.origin.pos, lastAt: 0, updates: 0, landmarkState: 'fixed', landmarkRmsPx: null },
      { id: anchors.yArm.id, role: anchors.yArm.role, fixed: true, pos: anchors.yArm.pos, lastAt: 0, updates: 0, landmarkState: 'fixed', landmarkRmsPx: null },
    ];
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) verts.push(serializeMarker(m));
    }
    verts.push({
      id: anchors.xArm.id,
      role: anchors.xArm.role,
      fixed: true,
      pos: anchors.xArm.pos,
      lastAt: 0,
      updates: 0,
      landmarkState: 'fixed',
      landmarkRmsPx: null,
    });
    return verts;
  }

  function discoveredMarkers() {
    const list = [];
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) list.push(serializeMarker(m));
    }
    return list;
  }

  function dynamicCount() {
    return markers.size;
  }

  function syncCachedPolyXY() {
    if (cachedPolyVer === stateVersion) return cachedPolyXY;
    cachedPolyXY.length = 0;
    cachedPolyXY.push(anchors.origin.pos, anchors.yArm.pos);
    for (const spec of PERIMETER_MARKERS) {
      const m = markers.get(spec.id);
      if (m) cachedPolyXY.push(m.currentPos);
    }
    cachedPolyXY.push(anchors.xArm.pos);
    cachedPolyVer = stateVersion;
    return cachedPolyXY;
  }

  function isOutsideXY(posL, opts = {}) {
    if (!active) {
      return { active: false, outside: false, inside: true, signedDistanceM: 0, edgeDistanceM: 0 };
    }
    const res = evaluatePerimeterOutside(posL, syncCachedPolyXY(), opts);
    return { active: true, ...res };
  }

  function locateCrossing(prevPosL, currPosL, opts = {}) {
    if (!active) return null;
    return locatePerimeterCrossing(prevPosL, currPosL, syncCachedPolyXY(), { heightM, ...opts });
  }

  function snapshot() {
    const verts = orderedVertices();
    const { perimeterM, areaM2 } = polygonMetricsXY(verts);
    return {
      status: status(),
      active,
      offered: isOffered(),
      coordinateMode,
      triggerTagId,
      heightM,
      dynamicCount: markers.size,
      lockedCount: lockedCount(),
      totalDynamicMarkers: PERIMETER_MARKERS.length,
      perimeterM,
      areaM2,
      vertices: verts.map((v) => ({
        id: v.id,
        role: v.role,
        fixed: v.fixed,
        posL: v.pos.slice(),
        updates: v.updates,
        landmarkState: v.landmarkState ?? (v.fixed ? 'fixed' : 'tracking'),
        landmarkRmsPx: v.landmarkRmsPx ?? null,
      })),
    };
  }

  return {
    heightM,
    triggerTagId,
    isActive,
    isOffered,
    status,
    getCoordinateMode,
    setCoordinateMode,
    triggerMarker,
    activate,
    deactivate,
    setRig,
    reset,
    observe,
    remove,
    reconcileView,
    step,
    orderedVertices,
    discoveredMarkers,
    lockedLandmarks,
    lockedCount,
    reportLandmarkInlier,
    reportLandmarkOutlier,
    dynamicCount,
    isOutsideXY,
    locateCrossing,
    version: () => stateVersion,
    snapshot,
  };
}
