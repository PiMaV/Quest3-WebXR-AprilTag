const quadAreaPx=c=>Math.abs(c.reduce((v,p,i)=>v+p.x*c[(i+1)%4].y-p.y*c[(i+1)%4].x,0))/2;

// ARENA's atagjs_detect returns AprilTag p[0..3] in physical LL, LR, UR, UL order
// (even after rotation). OpenCV's corner order differs.
export const WASM_PRINT_ORDER = [0, 1, 2, 3];
/** Return the physical edge length in mm defined by the M.E.S.S. tag36h11 ID classes (100–499). */
export function requiredTagEdgeMm(markerId) {
  if (!Number.isInteger(markerId)) throw new RangeError(`No standard tag edge defined for ID ${markerId}`);
  if (markerId >= 100 && markerId <= 199) return 6.0;
  if (markerId >= 200 && markerId <= 299) return 12.0;
  if (markerId >= 300 && markerId <= 399) return 20.0;
  if (markerId >= 400 && markerId <= 449) return 80.0;
  if (markerId >= 450 && markerId <= 499) return 112.0;
  throw new RangeError(`No standard tag edge defined for ID ${markerId}`);
}
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const mul = (a, s) => a.map(v => v * s);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const finite = a => a.every(Number.isFinite);

/** Validate and scale a `webxr-apriltag-rig-v1` planar target definition. */
export function referenceTools(g, printScale=1) {
  if(g?.schema!=='webxr-apriltag-rig-v1'||g.family!=='apriltag-36h11'||g.units!=='m'||!g.id||
     !Array.isArray(g.tools)||g.tools.length!==1||!(printScale>0)) throw new TypeError('Invalid planar rigid target');
  const ids=new Set();
  return g.tools.map(tool=>{
    if(!tool.id||!Array.isArray(tool.tags)||tool.tags.length<3||tool.tags.length>8) throw new TypeError('Need 3–8 fixed tags');
    return {id:tool.id,tags:tool.tags.map(tag=>{
      if(!Number.isInteger(tag.id)||tag.id<0||tag.id>586||ids.has(tag.id)||!Array.isArray(tag.corners)||tag.corners.length!==4||
        tag.corners.some(p=>p.length!==2||!finite(p))) throw new TypeError('Invalid/duplicate planar tag');
      ids.add(tag.id);const c=tag.corners,edges=c.map((v,i)=>Math.hypot(...v.map((x,j)=>x-c[(i+1)%4][j])));
      const a=c[1].map((v,j)=>v-c[0][j]),b=c[3].map((v,j)=>v-c[0][j]);
      if(edges[0]<.001||edges.some(x=>Math.abs(x-edges[0])>1e-6)||Math.abs(dot(a,b))>1e-8||a[0]*b[1]-a[1]*b[0]<=0)
        throw new TypeError('Invalid square/corner order');
      return {id:tag.id,corners:c.map(v=>v.map(x=>x*printScale))};
    })};
  });
}

function solve(rows, rhs) {
  const n = rows[0].length, a = Array.from({length:n}, () => Array(n+1).fill(0));
  for (let k=0;k<rows.length;k++) for (let i=0;i<n;i++) {
    a[i][n] += rows[k][i]*rhs[k];
    for (let j=0;j<n;j++) a[i][j] += rows[k][i]*rows[k][j];
  }
  for (let c=0;c<n;c++) {
    let p=c;
    for (let r=c+1;r<n;r++) if (Math.abs(a[r][c])>Math.abs(a[p][c])) p=r;
    if (Math.abs(a[p][c])<1e-12) return null;
    [a[c],a[p]]=[a[p],a[c]];
    const d=a[c][c]; for (let j=c;j<=n;j++) a[c][j]/=d;
    for (let r=0;r<n;r++) if (r!==c) {
      const f=a[r][c]; for (let j=c;j<=n;j++) a[r][j]-=f*a[c][j];
    }
  }
  return a.map(r=>r[n]);
}

/**
 * Joint least-squares planar pose fit (normalized homography + analytic-Jacobian Gauss-Newton).
 * K must describe these image pixels; distortion must be removed before calling.
 */
export function fitPlanarPose(tags, K) {
  const points=tags.flatMap(t=>t.points), pixels=tags.flatMap(t=>t.pixels);
  const centre=[0,1].map(j=>points.reduce((s,p)=>s+p[j],0)/points.length);
  const spread=Math.sqrt(points.reduce((s,p)=>s+(p[0]-centre[0])**2+(p[1]-centre[1])**2,0)/points.length);
  if (!(spread>1e-9)) return null;
  const rows=[],rhs=[];
  points.forEach((p,i)=>{
    const [x,y]=p.map((v,j)=>(v-centre[j])/spread);
    const u=(pixels[i].x-K.cx)/K.fx,v=(pixels[i].y-K.cy)/K.fy;
    rows.push([x,y,1,0,0,0,-u*x,-u*y],[0,0,0,x,y,1,-v*x,-v*y]);rhs.push(u,v);
  });
  const h=solve(rows,rhs); if (!h) return null;
  const b1=[h[0],h[3],h[6]].map(v=>v/spread),b2=[h[1],h[4],h[7]].map(v=>v/spread);
  const b3=[h[2],h[5],1];
  const scale=2/(Math.hypot(...b1)+Math.hypot(...b2)) * (b3[2]<0?-1:1);
  const r1=mul(b1,scale/Math.hypot(...mul(b1,scale)));
  const v2=mul(b2,scale).map((v,i)=>v-r1[i]*dot(r1,mul(b2,scale)));
  const r2=mul(v2,1/Math.hypot(...v2)),r3=cross(r1,r2);
  // Decompose at the observed centroid before moving the rigid pose to the
  // outer origin. Orthogonalization must not amplify noise by the 230 mm arm.
  const t=mul(b3,scale).map((v,i)=>v-r1[i]*centre[0]-r2[i]*centre[1]);
  if (![...r1,...r2,...r3,...t].every(Number.isFinite) || t[2]<=0) return null;
  return refinePose({R:[r1,r2,r3],t},tags,K);
}

function pixelResiduals(pose,tags,K) {
  const out=[];
  for(const tag of tags) for(let i=0;i<4;i++) {
    const [x,y]=tag.points[i],p=pose.t.map((v,j)=>v+pose.R[0][j]*x+pose.R[1][j]*y);
    if(p[2]<=0) return null;
    out.push(K.fx*p[0]/p[2]+K.cx-tag.pixels[i].x,K.fy*p[1]/p[2]+K.cy-tag.pixels[i].y);
  }
  return out.every(Number.isFinite)?out:null;
}
export function incrementPose(pose,step) {
  const angle=Math.hypot(...step.slice(0,3)),axis=angle?mul(step.slice(0,3),1/angle):[0,0,0];
  const c=Math.cos(angle),s=Math.sin(angle);
  return {R:pose.R.map(r=>r.map((v,i)=>v*c+cross(axis,r)[i]*s+axis[i]*dot(axis,r)*(1-c))),
    t:pose.t.map((v,i)=>v+step[i+3])};
}

const _refineA = new Float64Array(42);
const _refineStep = new Float64Array(6);
const _refineJu = new Float64Array(6);
const _refineJv = new Float64Array(6);
const _candR = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
const _candT = [0, 0, 0];

function solve6x6InPlace(A, out) {
  for (let c = 0; c < 6; c++) {
    let piv = c;
    let maxV = Math.abs(A[c * 7 + c]);
    for (let r = c + 1; r < 6; r++) {
      const av = Math.abs(A[r * 7 + c]);
      if (av > maxV) { maxV = av; piv = r; }
    }
    if (maxV < 1e-12) return false;
    if (piv !== c) {
      const o1 = c * 7, o2 = piv * 7;
      for (let j = c; j < 7; j++) {
        const tmp = A[o1 + j];
        A[o1 + j] = A[o2 + j];
        A[o2 + j] = tmp;
      }
    }
    const oc = c * 7;
    const invD = 1 / A[oc + c];
    for (let j = c; j < 7; j++) A[oc + j] *= invD;
    for (let r = 0; r < 6; r++) {
      if (r === c) continue;
      const or = r * 7;
      const f = A[or + c];
      if (f === 0) continue;
      for (let j = c; j < 7; j++) A[or + j] -= f * A[oc + j];
    }
  }
  for (let i = 0; i < 6; i++) {
    const v = A[i * 7 + 6];
    if (!Number.isFinite(v)) return false;
    out[i] = v;
  }
  return true;
}

function applyScaledIncrement(r1, r2, r3, t, step, factor, dstR, dstT) {
  const s0 = step[0] * factor, s1 = step[1] * factor, s2 = step[2] * factor;
  dstT[0] = t[0] + step[3] * factor;
  dstT[1] = t[1] + step[4] * factor;
  dstT[2] = t[2] + step[5] * factor;
  const angle = Math.hypot(s0, s1, s2);
  const src = [r1, r2, r3];
  if (angle === 0) {
    for (let k = 0; k < 3; k++) {
      dstR[k][0] = src[k][0];
      dstR[k][1] = src[k][1];
      dstR[k][2] = src[k][2];
    }
    return;
  }
  const invA = 1 / angle;
  const ax = s0 * invA, ay = s1 * invA, az = s2 * invA;
  const c = Math.cos(angle), s = Math.sin(angle), omc = 1 - c;
  for (let k = 0; k < 3; k++) {
    const rx = src[k][0], ry = src[k][1], rz = src[k][2];
    const d = ax * rx + ay * ry + az * rz;
    const cx = ay * rz - az * ry;
    const cy = az * rx - ax * rz;
    const cz = ax * ry - ay * rx;
    dstR[k][0] = rx * c + cx * s + ax * d * omc;
    dstR[k][1] = ry * c + cy * s + ay * d * omc;
    dstR[k][2] = rz * c + cz * s + az * d * omc;
  }
}

function residualCost(r1, r2, t, tags, K) {
  const fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const tx = t[0], ty = t[1], tz = t[2];
  const r10 = r1[0], r11 = r1[1], r12 = r1[2];
  const r20 = r2[0], r21 = r2[1], r22 = r2[2];
  let sum = 0;
  for (let ti = 0; ti < tags.length; ti++) {
    const pts = tags[ti].points, pix = tags[ti].pixels;
    for (let i = 0; i < 4; i++) {
      const x = pts[i][0], y = pts[i][1];
      const px = tx + r10 * x + r20 * y;
      const py = ty + r11 * x + r21 * y;
      const pz = tz + r12 * x + r22 * y;
      if (!(pz > 0)) return Infinity;
      const invZ = 1 / pz;
      const eu = fx * px * invZ + cx - pix[i].x;
      const ev = fy * py * invZ + cy - pix[i].y;
      sum += eu * eu + ev * ev;
    }
  }
  return Number.isFinite(sum) ? sum : Infinity;
}

// Optimize the rigid reprojection after homography initialization using the exact
// analytic 2x6 Jacobian of incrementPose and pre-allocated normal-equation buffers.
function refinePose(initial,tags,K) {
  let r1 = [initial.R[0][0], initial.R[0][1], initial.R[0][2]];
  let r2 = [initial.R[1][0], initial.R[1][1], initial.R[1][2]];
  let r3 = [initial.R[2][0], initial.R[2][1], initial.R[2][2]];
  let t = [initial.t[0], initial.t[1], initial.t[2]];
  const fx = K.fx, fy = K.fy, cx = K.cx, cy = K.cy;
  const Ju = _refineJu, Jv = _refineJv, A = _refineA, step = _refineStep;
  const lineFactors = [1, 0.5, 0.25, 0.125, 0.0625];

  for (let iteration = 0; iteration < 15; iteration++) {
    A.fill(0);
    let cost = 0;
    const tx = t[0], ty = t[1], tz = t[2];
    const r10 = r1[0], r11 = r1[1], r12 = r1[2];
    const r20 = r2[0], r21 = r2[1], r22 = r2[2];
    for (let ti = 0; ti < tags.length; ti++) {
      const pts = tags[ti].points, pix = tags[ti].pixels;
      for (let i = 0; i < 4; i++) {
        const x = pts[i][0], y = pts[i][1];
        const qx = r10 * x + r20 * y;
        const qy = r11 * x + r21 * y;
        const qz = r12 * x + r22 * y;
        const px = qx + tx;
        const py = qy + ty;
        const pz = qz + tz;
        if (!(pz > 0)) return null;
        const invZ = 1 / pz;
        const fxZ = fx * invZ;
        const fyZ = fy * invZ;
        const fxXZ = fxZ * px * invZ;
        const fyYZ = fyZ * py * invZ;
        const eu = fxZ * px + cx - pix[i].x;
        const ev = fyZ * py + cy - pix[i].y;
        if (!Number.isFinite(eu) || !Number.isFinite(ev)) return null;
        cost += eu * eu + ev * ev;

        Ju[0] = -fxXZ * qy;
        Ju[1] = fxZ * qz + fxXZ * qx;
        Ju[2] = -fxZ * qy;
        Ju[3] = fxZ;
        Ju[4] = 0;
        Ju[5] = -fxXZ;

        Jv[0] = -fyZ * qz - fyYZ * qy;
        Jv[1] = fyYZ * qx;
        Jv[2] = fyZ * qx;
        Jv[3] = 0;
        Jv[4] = fyZ;
        Jv[5] = -fyYZ;

        for (let r = 0; r < 6; r++) {
          const ro = r * 7;
          const jur = Ju[r], jvr = Jv[r];
          A[ro + 6] -= jur * eu + jvr * ev;
          for (let c = r; c < 6; c++) {
            A[ro + c] += jur * Ju[c] + jvr * Jv[c];
          }
        }
      }
    }
    for (let r = 0; r < 6; r++) {
      for (let c = 0; c < r; c++) {
        A[r * 7 + c] = A[c * 7 + r];
      }
    }
    if (!solve6x6InPlace(A, step)) break;

    let improved = false;
    for (let fIdx = 0; fIdx < 5; fIdx++) {
      applyScaledIncrement(r1, r2, r3, t, step, lineFactors[fIdx], _candR, _candT);
      const nextCost = residualCost(_candR[0], _candR[1], _candT, tags, K);
      if (nextCost < cost) {
        r1[0] = _candR[0][0]; r1[1] = _candR[0][1]; r1[2] = _candR[0][2];
        r2[0] = _candR[1][0]; r2[1] = _candR[1][1]; r2[2] = _candR[1][2];
        r3[0] = _candR[2][0]; r3[1] = _candR[2][1]; r3[2] = _candR[2][2];
        t[0] = _candT[0]; t[1] = _candT[1]; t[2] = _candT[2];
        improved = true;
        break;
      }
    }
    const stepNorm = Math.hypot(step[0], step[1], step[2], step[3], step[4], step[5]);
    if (!improved || stepNorm < 1e-9) break;
  }
  return { R: [r1, r2, r3], t };
}

/**
 * Solve a 6DOF OpenCV-frame pose for a single detected AprilTag square.
 * Accepts an explicit `edgeMm` or `getTagEdgeMm(id)` override; defaults to `requiredTagEdgeMm(id)`.
 */
export function solveSingleTagPose(raw, K, { edgeMm = null, getTagEdgeMm = null, minEdgePx = 8, minAreaPx = 64, maxRmsPx = 2.5, minCosView = 0, maxDepthM = Infinity } = {}) {
  let resolvedEdgeMm;
  if (edgeMm !== null && edgeMm !== undefined) {
    if (typeof edgeMm !== 'number' || !Number.isFinite(edgeMm) || edgeMm <= 0) return null;
    resolvedEdgeMm = edgeMm;
  } else if (typeof getTagEdgeMm === 'function') {
    try { resolvedEdgeMm = getTagEdgeMm(raw?.id); } catch { return null; }
    if (typeof resolvedEdgeMm !== 'number' || !Number.isFinite(resolvedEdgeMm) || resolvedEdgeMm <= 0) return null;
  } else {
    try { resolvedEdgeMm = requiredTagEdgeMm(raw?.id); } catch { return null; }
  }
  const half = (resolvedEdgeMm / 1000) * 0.5;
  const corners = [[-half, -half], [half, -half], [half, half], [-half, half]];
  const c = raw.corners;
  if (!Array.isArray(c) || c.length !== 4) return null;
  for (let i = 0; i < 4; i++) {
    const p = c[i];
    if (!Number.isFinite(p?.x) || !Number.isFinite(p?.y) || p.x < 0 || p.y < 0 || p.x >= K.width || p.y >= K.height) {
      return null;
    }
  }
  let minE = Infinity, area2 = 0;
  for (let i = 0; i < 4; i++) {
    const p = c[i], q = c[(i + 1) & 3], r = c[(i + 2) & 3];
    if ((q.x - p.x) * (r.y - q.y) - (q.y - p.y) * (r.x - q.x) >= 0) return null;
    const e = Math.hypot(p.x - q.x, p.y - q.y);
    if (e < minE) minE = e;
    area2 += p.x * q.y - p.y * q.x;
  }
  const area = Math.abs(area2) * 0.5;
  if (minE < minEdgePx || area < minAreaPx) return null;
  const pose = fitPlanarPose([{ id: raw.id, points: corners, pixels: c }], K);
  if (!pose) return null;
  const [r1, r2, r3] = pose.R;
  const t = pose.t;
  let sqErr = 0;
  for (let i = 0; i < 4; i++) {
    const x = corners[i][0], y = corners[i][1];
    const px = t[0] + r1[0] * x + r2[0] * y;
    const py = t[1] + r1[1] * x + r2[1] * y;
    const pz = t[2] + r1[2] * x + r2[2] * y;
    if (!(pz > 0.01)) return null;
    const dx = K.fx * px / pz + K.cx - c[i].x;
    const dy = K.fy * py / pz + K.cy - c[i].y;
    sqErr += dx * dx + dy * dy;
  }
  const rmsPx = Math.sqrt(sqErr * 0.25);
  if (!Number.isFinite(rmsPx) || rmsPx > maxRmsPx) return null;
  const tNorm = Math.hypot(t[0], t[1], t[2]);
  const cosView = tNorm > 1e-6 ? Math.abs(r3[0] * t[0] + r3[1] * t[1] + r3[2] * t[2]) / tNorm : 0;
  const depthM = t[2];
  if (!Number.isFinite(depthM) || depthM > maxDepthM || cosView < minCosView) return null;
  return { edgeMm: resolvedEdgeMm, pose, corners: c, minEdgePx: minE, areaPx: area, rmsPx, cosView, depthM };
}

function residual(pose, tag, K) {
  const [r1,r2]=pose.R;
  let sum=0;
  for (let i=0;i<4;i++) {
    const [x,y]=tag.points[i],p=pose.t.map((v,j)=>v+r1[j]*x+r2[j]*y);
    if (p[2]<=0) return Infinity;
    sum+=(K.fx*p[0]/p[2]+K.cx-tag.pixels[i].x)**2+(K.fy*p[1]/p[2]+K.cy-tag.pixels[i].y)**2;
  }
  return Math.sqrt(sum/4);
}

function consensus(tags, K, maxRmsPx) {
  // Fast path: try all tags jointly first; return immediately when every tag is an inlier.
  const fullMask=(1<<tags.length)-1;
  const allPose=fitPlanarPose(tags,K);
  if (allPose) {
    let sumSq=0,allInliers=true;
    for (let i=0;i<tags.length;i++) {
      const r=residual(allPose,tags[i],K);
      if (r>maxRmsPx) {allInliers=false;break;}
      sumSq+=r*r;
    }
    if (allInliers) return {pose:allPose,inliers:tags,rms:Math.sqrt(sumSq/tags.length)};
  }
  // At most three rigid tags: enumerate remaining subsets, score entire tags, refit.
  // A bad tag must not contribute three apparently good corners to a fit.
  let best=null;
  const seenInlierMasks=new Set();
  for (let mask=1;mask<fullMask;mask++) {
    const selected=tags.filter((_,i)=>mask&(1<<i));
    let pose=fitPlanarPose(selected,K);
    if (!pose) continue;
    const inliers=[];let inliersMask=0;
    for (let i=0;i<tags.length;i++) {
      if (residual(pose,tags[i],K)<=maxRmsPx) {inliers.push(tags[i]);inliersMask|=1<<i;}
    }
    // With >1 tag seen, require a strict majority. Two disagreeing tags are
    // ambiguous; a lone surviving tag out of three is not a consensus.
    if (!inliers.length || (tags.length>1 && inliers.length<=tags.length/2)) continue;
    if (seenInlierMasks.has(inliersMask)) continue;
    seenInlierMasks.add(inliersMask);
    if (inliersMask!==mask) {
      pose=fitPlanarPose(inliers,K); if (!pose) continue;
    }
    let sumSq=0,valid=true;
    for (let i=0;i<inliers.length;i++) {
      const r=residual(pose,inliers[i],K);
      if (r>maxRmsPx) {valid=false;break;}
      sumSq+=r*r;
    }
    if (!valid) continue;
    const rms=Math.sqrt(sumSq/inliers.length);
    if (!best || inliers.length>best.inliers.length || (inliers.length===best.inliers.length && rms<best.rms))
      best={pose,inliers,rms};
  }
  return best;
}

/** Match raw AprilTag detections against a planar reference rig and run strict-majority consensus. */
export function observeReference(markers, geometry, K, {printScale=1,maxRmsPx=3,minAreaPx=100}={}) {
  if (![K?.fx,K?.fy,K?.cx,K?.cy,K?.width,K?.height].every(Number.isFinite) ||
      K.fx<=0 || K.fy<=0 || K.width<=0 || K.height<=0 || !(maxRmsPx>0) || !(minAreaPx>0))
    throw new TypeError('Invalid camera model/gates');
  const tools=referenceTools(geometry,printScale),known=new Map(tools.flatMap(t=>t.tags.map(m=>[m.id,{...m,tool:t.id}])));
  const counts=new Map(), rejected=[], accepted=[];
  for (const m of markers) counts.set(m.id,(counts.get(m.id)||0)+1);
  for (const m of markers) {
    const spec=known.get(m.id), c=m.corners;
    let reason=!spec?'unassigned_id':counts.get(m.id)>1?'duplicate_id':null;
    if (!reason && (!Array.isArray(c) || c.length!==4 || c.some(p=>!finite([p?.x,p?.y]) || p.x<0 || p.y<0 || p.x>=K.width || p.y>=K.height))) reason='invalid_corners';
    if (!reason) {
      const turns=c.map((p,i)=>{
        const q=c[(i+1)%4],r=c[(i+2)%4];return (q.x-p.x)*(r.y-q.y)-(q.y-p.y)*(r.x-q.x);
      });
      if (!turns.every(v=>v<0) || c.some((p,i)=>Math.hypot(p.x-c[(i+1)%4].x,p.y-c[(i+1)%4].y)<8) || quadAreaPx(c)<minAreaPx) reason='small_or_invalid_quad';
    }
    if (reason) {rejected.push({id:m.id,reason});continue;}
    accepted.push({id:m.id,toolId:spec.tool,points:spec.corners,pixels:WASM_PRINT_ORDER.map(i=>({...c[i]}))});
  }
  const result={layoutId:geometry.id,printScale,imageSize:[K.width,K.height],cameraModel:K,
    calibrated:false,scenePose:null,tags:accepted,tools:[],rejected};
  for (const tool of tools) {
    const tags=accepted.filter(t=>t.toolId===tool.id); if (!tags.length) continue;
    const model=consensus(tags,K,maxRmsPx);
    if (!model) {
      rejected.push(...tags.map(t=>({id:t.id,reason:'inconsistent_rigid_geometry'})));continue;
    }
    const ids=model.inliers.map(t=>t.id);
    rejected.push(...tags.filter(t=>!ids.includes(t.id)).map(t=>({id:t.id,reason:'rigid_tag_outlier'})));
    result.tools.push({id:tool.id,scenePose:null,tagIds:ids,pointCount:ids.length*4,
      mode:ids.length>1?'joint-rigid-tags':'single-tag',cameraFromTool:model.pose,
      reprojectionRmsPx:model.rms,perTagRmsPx:Object.fromEntries(model.inliers.map(t=>[t.id,residual(model.pose,t,K)]))});
  }
  return result;
}
