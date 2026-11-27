// Offline calibration model comparison & radial-residual analysis CLI.
// Usage:
//   node scripts/analyze-calibration.mjs --synthetic-demo
//   node scripts/analyze-calibration.mjs [debug_jsons/*.json ...]

import fs from 'node:fs';
import path from 'node:path';
import { Matrix4, Vector3 } from '../vendor/three/three.module.js';
import { solveCalibration, summarizeRadialResiduals } from '../src/solver.mjs';
import { solveSingleTagPose } from '../src/rig.mjs';
import { inverse, product, vectorRigid } from '../src/runtime.mjs';

const MODELS = ['centered-square', 'centered-radial', 'pinhole', 'pinhole-radial'];
const fmt = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '—');
const pad = (s, n) => String(s).padEnd(n);

function printBucketLine(label, b) {
  if (!b || !b.count) {
    console.log(`    ${pad(label, 14)} n=  0`);
    return;
  }
  console.log(
    `    ${pad(label, 14)} n=${String(b.count).padStart(3)}  rms=${fmt(b.rmsPx, 3)}px  ` +
      `rad=${fmt(b.meanRadialPx, 3)}px  dx=${fmt(b.meanDxPx, 3)}px  dy=${fmt(b.meanDyPx, 3)}px`,
  );
}

function printResidualBreakdown(title, summary) {
  if (!summary || !summary.totalCorners) return;
  console.log(`  ${title} (corners=${summary.totalCorners}, rP95=${fmt(summary.rP95, 2)}, rMax=${fmt(summary.rMax, 2)}, rms=${fmt(summary.rmsPx, 3)}px):`);
  printBucketLine('inner r<0.25', summary.inner);
  printBucketLine('mid   0.25..45', summary.mid);
  printBucketLine('outer r>=0.45', summary.outer);
  if (summary.quadrants) {
    const q = summary.quadrants;
    console.log(
      `    quadrants: TL(${q.tl.count})=${fmt(q.tl.rmsPx, 2)}px  TR(${q.tr.count})=${fmt(q.tr.rmsPx, 2)}px  ` +
        `BL(${q.bl.count})=${fmt(q.bl.rmsPx, 2)}px  BR(${q.br.count})=${fmt(q.br.rmsPx, 2)}px`,
    );
  }
}

function runModelComparison(rows, history, imageSize) {
  console.log('  Model comparison (80/20 holdout):');
  console.log(
    '    ' +
      [
        pad('model', 16),
        pad('pass', 5),
        pad('fx', 6),
        pad('fy', 6),
        pad('cx', 6),
        pad('cy', 6),
        pad('k1', 8),
        pad('tzMm', 6),
        pad('trnPx', 6),
        pad('hldPx', 6),
        pad('jntPx', 6),
        pad('posMm', 6),
        pad('angDeg', 6),
        pad('inPx', 6),
        pad('midPx', 6),
        pad('outPx', 6),
        pad('outRad', 7),
      ].join(' '),
  );
  const results = [];
  for (const model of MODELS) {
    try {
      const r = solveCalibration({ rows, history, imageSize, model });
      results.push({ model, result: r });
      const v = r.validation;
      const rr = v.radialResiduals;
      console.log(
        '    ' +
          [
            pad(model, 16),
            pad(v.passed ? 'yes' : 'NO', 5),
            pad(fmt(r.K.fx, 1), 6),
            pad(fmt(r.K.fy, 1), 6),
            pad(fmt(r.K.cx, 1), 6),
            pad(fmt(r.K.cy, 1), 6),
            pad(fmt(r.distCoeffs[0], 4), 8),
            pad(fmt(r.viewerFromCameraGL[14] * 1000, 1), 6),
            pad(fmt(v.trainingRmsPx, 3), 6),
            pad(fmt(v.heldOutRmsPx, 3), 6),
            pad(fmt(v.jointHeldOutRmsPx, 2), 6),
            pad(fmt(v.positionP95Mm, 2), 6),
            pad(fmt(v.angleP95Deg, 2), 6),
            pad(fmt(rr?.inner?.rmsPx, 3), 6),
            pad(fmt(rr?.mid?.rmsPx, 3), 6),
            pad(fmt(rr?.outer?.rmsPx, 3), 6),
            pad(fmt(rr?.outer?.meanRadialPx, 3), 7),
          ].join(' '),
      );
    } catch (err) {
      console.log(`    ${pad(model, 16)} ERROR: ${err.message}`);
    }
  }
  return results;
}

function summarizeAuditSamples(auditSamples, K) {
  const mk = () => ({ count: 0, sumSq: 0, sumRad: 0, sumDx: 0, sumDy: 0 });
  const fin = a => ({
    count: a.count,
    rmsPx: a.count ? Math.sqrt(a.sumSq / a.count) : 0,
    meanRadialPx: a.count ? a.sumRad / a.count : 0,
    meanDxPx: a.count ? a.sumDx / a.count : 0,
    meanDyPx: a.count ? a.sumDy / a.count : 0,
  });
  const inner = mk(), mid = mk(), outer = mk(), all = mk();
  const qAcc = { tl: mk(), tr: mk(), bl: mk(), br: mk() };
  const radii = [];
  const push = (acc, dx, dy, rad) => {
    acc.count++;
    acc.sumSq += dx * dx + dy * dy;
    acc.sumRad += rad;
    acc.sumDx += dx;
    acc.sumDy += dy;
  };
  for (const row of auditSamples) {
    for (const proj of row.projections || []) {
      const obs = proj.observed || [], pred = proj.predicted || [];
      for (let i = 0; i < Math.min(obs.length, pred.length); i++) {
        const dx = pred[i].x - obs[i].x;
        const dy = pred[i].y - obs[i].y;
        const rx = (obs[i].x - K.cx) / K.fx;
        const ry = (obs[i].y - K.cy) / K.fy;
        const r = Math.hypot(rx, ry);
        const rPx = Math.hypot(obs[i].x - K.cx, obs[i].y - K.cy);
        const rad = rPx > 1e-9 ? (dx * (obs[i].x - K.cx) + dy * (obs[i].y - K.cy)) / rPx : 0;
        radii.push(r);
        push(all, dx, dy, rad);
        if (r < 0.25) push(inner, dx, dy, rad);
        else if (r < 0.45) push(mid, dx, dy, rad);
        else push(outer, dx, dy, rad);
        const qKey = (obs[i].y < K.cy ? 't' : 'b') + (obs[i].x < K.cx ? 'l' : 'r');
        push(qAcc[qKey], dx, dy, rad);
      }
    }
  }
  radii.sort((a, b) => a - b);
  return {
    totalCorners: all.count,
    rmsPx: fin(all).rmsPx,
    rMax: radii.length ? radii.at(-1) : 0,
    rP95: radii.length ? radii[Math.min(radii.length - 1, Math.floor(radii.length * 0.95))] : 0,
    inner: fin(inner),
    mid: fin(mid),
    outer: fin(outer),
    quadrants: { tl: fin(qAcc.tl), tr: fin(qAcc.tr), bl: fin(qAcc.bl), br: fin(qAcc.br) },
  };
}

function buildSyntheticQuestDemo() {
  const rig = JSON.parse(fs.readFileSync(new URL('../rigs/ms-qr-main-l.json', import.meta.url), 'utf8'));
  const lTags = rig.tools[0].tags;
  const Ktrue = { width: 1280, height: 1080, fx: 886.3, fy: 886.3, cx: 640, cy: 540, k1: -0.08 };
  const Xtrue = vectorRigid([0.01, 0.015, -0.008, 0.0333, -0.0043, -0.0454]);
  const L = vectorRigid([-Math.PI / 2, 0, 0, -0.10, -0.65, -0.35]);
  const delay = -45.2;
  const lCenterW = [
    L[12] + L[0] * 0.10 + L[4] * 0.08,
    L[13] + L[1] * 0.10 + L[5] * 0.08,
    L[14] + L[2] * 0.10 + L[6] * 0.08,
  ];
  const head = t => {
    const s = t / 1000;
    const az = 0.72 * Math.sin(s * 0.9);
    const el = 0.95 + 0.25 * Math.cos(s * 1.1);
    const dist = 0.56 + 0.12 * Math.sin(s * 1.3);
    const eye = new Vector3(
      lCenterW[0] + dist * Math.cos(el) * Math.sin(az),
      lCenterW[1] + dist * Math.sin(el),
      lCenterW[2] + dist * Math.cos(el) * Math.cos(az),
    );
    const look = new Vector3(
      lCenterW[0] + 0.20 * Math.sin(s * 1.7),
      lCenterW[1],
      lCenterW[2] + 0.15 * Math.cos(s * 1.4),
    );
    const m = new Matrix4().lookAt(eye, look, new Vector3(0.12 * Math.sin(s * 1.5), 1, 0).normalize());
    m.setPosition(eye);
    return m.toArray();
  };
  const history = [];
  for (let t = 0; t <= 12000; t += 10) {
    history.push({ xrRafTimeMs: t, worldFromViewer: head(t), scope: 's', sessionId: 's', spaceEpoch: 0 });
  }
  const rows = [];
  for (let i = 0; i < 48; i++) {
    const captureTime = 500 + i * 200;
    const C = product(inverse(Xtrue), inverse(head(captureTime + delay)), L);
    let visible = true;
    const tags = lTags.map(tag => {
      const pixels = tag.corners.map(([x, y], j) => {
        const px = C[12] + C[0] * x + C[4] * y;
        const py = C[13] + C[1] * x + C[5] * y;
        const pz = C[14] + C[2] * x + C[6] * y;
        const zcv = -pz;
        const xn = px / zcv, yn = (-py) / zcv;
        const sc = 1 + Ktrue.k1 * (xn * xn + yn * yn);
        const u = Ktrue.fx * xn * sc + Ktrue.cx + 0.25 * Math.sin(i * 17 + j * 11 + tag.id);
        const v = Ktrue.fy * yn * sc + Ktrue.cy + 0.25 * Math.cos(i * 13 + j * 7 + tag.id);
        if (u < 6 || u > 1274 || v < 6 || v > 1074) visible = false;
        return { x: u, y: v };
      });
      return { id: tag.id, points: tag.corners, pixels };
    });
    if (visible) rows.push({ scope: 's', captureTime, tags, serial: i + 1 });
  }
  return { rows, history, imageSize: [1280, 1080], Ktrue };
}

function runSyntheticDemo() {
  const { rows, history, imageSize, Ktrue } = buildSyntheticQuestDemo();
  console.log('=== SYNTHETIC QUEST-3 DEMO (fx=886.3, k1=-0.0800, 0.25px noise, edge-sweep orbit) ===');
  console.log(`Views: ${rows.length} (80/20 split)`);
  const results = runModelComparison(rows, history, imageSize);
  console.log('\n  Outer-field 20mm tag reconstruction error at Z=0.50m (45° oblique table):');
  const half = 0.010, c45 = Math.SQRT1_2, s45 = Math.SQRT1_2;
  const tagLocal = [[-half, -half], [half, -half], [half, half], [-half, half]];
  for (const { model, result } of results) {
    const errs = [0.125, 0.25].map(X => {
      const Z = 0.50;
      const corners = tagLocal.map(([lx, ly]) => {
        const px = X + lx, py = -ly * c45, pz = Z + ly * s45;
        const xn = px / pz, yn = py / pz, sc = 1 + Ktrue.k1 * (xn * xn + yn * yn);
        return { x: Ktrue.fx * xn * sc + Ktrue.cx, y: Ktrue.fy * yn * sc + Ktrue.cy };
      });
      const solved = solveSingleTagPose({ id: 300, corners }, result.K, { maxRmsPx: 20 });
      return solved ? Math.hypot(solved.pose.t[0] - X, solved.pose.t[1], solved.pose.t[2] - Z) * 1000 : Infinity;
    });
    console.log(`    ${pad(model, 16)} @12.5cm: ${fmt(errs[0], 2)} mm   @25.0cm: ${fmt(errs[1], 2)} mm`);
  }
}

function analyzeFile(filePath) {
  const j = JSON.parse(fs.readFileSync(filePath, 'utf8'));
  const p = j.profile;
  const samples = j.samples || [];
  const optL = samples.filter(s => s.poseSource === 'optical-l').length;
  const optP = samples.filter(s => s.poseSource === 'optical-perimeter').length;
  const xrH = samples.filter(s => s.poseSource === 'xr-history').length;
  const active = optL + optP + xrH;
  console.log(`\n=== ${path.basename(filePath)} (${j.appVersion ?? 'pre-v1.1.0'}) ===`);
  if (p) {
    const k1 = p.distCoeffs?.[0] ?? p.K?.k1 ?? 0;
    const tMm = (p.viewerFromCameraGL?.slice(12, 15) ?? []).map(v => fmt(v * 1000, 2)).join(', ');
    console.log(
      `  Profile: ${p.id} | solver=v${p.solverVersion ?? 1} model=${p.model} | fx=${fmt(p.K?.fx, 2)} fy=${fmt(p.K?.fy, 2)} ` +
        `cx=${fmt(p.K?.cx, 1)} cy=${fmt(p.K?.cy, 1)} k1=${fmt(k1, 4)} | t=[${tMm}] mm | dt=${fmt(p.timeOffsetMs, 2)} ms`,
    );
    console.log(
      `  Val/Ver: train=${fmt(p.validation?.trainingRmsPx, 3)}px held=${fmt(p.validation?.heldOutRmsPx, 3)}px ` +
        `posP95=${fmt(p.validation?.positionP95Mm, 2)}mm | verRms=${fmt(p.verification?.reprojectionRmsPx, 3)}px (${p.verification?.kind ?? '—'})`,
    );
  }
  const optAll = optL + optP;
  console.log(
    `  Session: optical=${optAll}/${active} (L=${optL}, P=${optP}; ${active ? fmt((100 * optAll) / active, 1) : '0.0'}%) | ` +
      `auditMed=${fmt(j.audit?.positionDifferenceMm?.median, 2)}mm predRms=${fmt(j.audit?.nativePredictionRmsPx?.median, 2)}px | ` +
      `anchorMaxDrift=${fmt(j.xr?.anchorStability?.maxDriftMm, 2)}mm`,
  );
  if (j.perimeterFence || j.anchorCorrection) {
    const pf = j.perimeterFence;
    const ac = j.anchorCorrection;
    console.log(
      `  Perimeter/Anchor: mode=${pf?.coordinateMode ?? '—'} landmarksLocked=${pf?.lockedCount ?? 0}/${pf?.dynamicCount ?? 0} | ` +
        `corrPos=${fmt(ac?.posMm, 2)}mm corrAng=${fmt(ac?.angDeg, 2)}deg updates=${ac?.updates ?? 0}`,
    );
  }

  const fit = j.calibrationData?.fit;
  const ver = j.calibrationData?.verification;
  const auditSamples = j.auditSamples;
  if (fit?.rows?.length >= 30 && fit?.history?.length >= 2) {
    runModelComparison(fit.rows, fit.history, fit.imageSize ?? p?.imageSize ?? [1280, 1080]);
  }
  if (ver?.rows?.length && p?.K) {
    const activeK = { ...p.K, k1: p.distCoeffs?.[0] ?? p.K?.k1 ?? 0 };
    printResidualBreakdown(`Verification optical residuals (${ver.kind ?? 'check'})`, summarizeRadialResiduals(ver.rows, activeK));
  }
  if (Array.isArray(auditSamples) && auditSamples.length && p?.K) {
    const activeK = { ...p.K, k1: p.distCoeffs?.[0] ?? p.K?.k1 ?? 0 };
    printResidualBreakdown(`Native-vs-optical audit projections (${auditSamples.length} frames)`, summarizeAuditSamples(auditSamples, activeK));
  }
  if (!fit?.rows?.length && !ver?.rows?.length && !auditSamples?.length) {
    console.log('  Note: legacy diagnostic log contains no raw corner pixels / calibrationData / auditSamples.');
  }
}

const args = process.argv.slice(2);
if (args.includes('--synthetic-demo')) {
  runSyntheticDemo();
} else {
  let targets = args.filter(a => !a.startsWith('-'));
  if (!targets.length) {
    const dir = 'debug_jsons';
    if (fs.existsSync(dir)) {
      targets = fs.readdirSync(dir).filter(f => f.endsWith('.json')).sort().map(f => path.join(dir, f));
    }
  }
  if (!targets.length) {
    console.error('No diagnostic JSON files found. Pass paths or --synthetic-demo.');
    process.exit(1);
  }
  for (const file of targets) analyzeFile(file);
}
