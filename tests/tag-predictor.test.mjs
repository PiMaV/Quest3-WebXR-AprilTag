import test from 'node:test';
import assert from 'node:assert/strict';
import { createTagMotionPredictor, TAG_PREDICTOR_DEFAULTS } from '../src/tag-predictor.mjs';
import { isRigidPose } from '../src/anchor.mjs';
import { vectorRigid } from '../src/runtime.mjs';

test('createTagMotionPredictor extrapolates rigid 6DOF constant velocity + rotation lead and decays smoothly', () => {
  const pred = createTagMotionPredictor();

  // Stationary frames produce near-zero velocity and exact pose
  const p0 = vectorRigid([0, 0, 0, 0.10, 0.20, 0.30]);
  pred.observe(301, p0, 1000);
  const res1 = pred.observe(301, vectorRigid([0, 0, 0.5, 0.16, 0.20, 0.30]), 1100); // 0.6 m/s + 5 rad/s
  assert.ok(res1.speedMps > 0.3 && res1.speedMps < 0.7);

  // Feed consistent velocity frames to converge EMA cleanly
  const p2 = vectorRigid([0, 0, 1.0, 0.22, 0.20, 0.30]);
  const res2 = pred.observe(301, p2, 1200);
  assert.ok(res2.speedMps > 0.45 && res2.angDegPerSec > 180);

  // Predict 80 ms ahead of t=1200 -> lead pose strictly rigid and advances along +X & yaw
  const lead = pred.predict(301, 1280);
  assert.ok(lead && isRigidPose(lead));
  assert.ok(lead[12] > p2[12] + 0.02, `expected forward X lead > 20 mm, got ${((lead[12] - p2[12]) * 1000).toFixed(1)} mm`);
  assert.ok(Math.abs(lead[13] - p2[13]) < 1e-4);

  // Stale prediction (age >> decayTauSec) decays lead rather than shooting away infinitely
  const stale = pred.predict(301, 1800);
  assert.ok(stale && isRigidPose(stale));
  assert.ok(Math.hypot(stale[12] - p2[12], stale[13] - p2[13], stale[14] - p2[14]) < 0.01);

  // Sharp direction reversal immediately drops old velocity sign
  const rev = pred.observe(301, vectorRigid([0, 0, 0.8, 0.16, 0.20, 0.30]), 1300);
  assert.ok(rev.speedMps > 0.4);
  const revPred = pred.predict(301, 1360);
  assert.ok(revPred[12] < 0.16, `expected negative X reversal lead, got ${revPred[12]}`);

  // Speed clamping protects against outlier jump spikes
  const spike = pred.observe(301, vectorRigid([0, 0, 0.8, 10.0, 0.20, 0.30]), 1350);
  assert.ok(spike.speedMps <= TAG_PREDICTOR_DEFAULTS.maxLinearMps + 1e-6);

  pred.reset(301);
  assert.equal(pred.predict(301, 1400), null);
  assert.equal(pred.speedOf(301), null);

  // seed() pre-warms velocity immediately without cold-start frame delay
  const seeded = pred.seed(302, vectorRigid([0, 0, 0, 0.20, 0.10, 0.05]), 2000, {
    velL: [0.80, 0, 0],
    angLocal: [0, 0, 2.0],
  });
  assert.ok(seeded && Math.abs(seeded.speedMps - 0.80) < 1e-6);
  const seededLead = pred.predict(302, 2060);
  assert.ok(seededLead && isRigidPose(seededLead));
  assert.ok(seededLead[12] > 0.20 + 0.035, `expected immediate seeded lead > 35 mm, got ${((seededLead[12] - 0.20) * 1000).toFixed(1)} mm`);

  // seed() clamps extreme velocity cleanly
  const clampedSeed = pred.seed(303, vectorRigid([0, 0, 0, 0, 0, 0]), 2100, {
    velL: [10.0, 0, 0],
    angLocal: [0, 0, 50.0],
  });
  assert.ok(clampedSeed.speedMps <= TAG_PREDICTOR_DEFAULTS.maxLinearMps + 1e-6);
  assert.ok(clampedSeed.angDegPerSec <= (TAG_PREDICTOR_DEFAULTS.maxAngularRadPerSec * 180 / Math.PI) + 1e-4);
});

test('trackingFade: stays opaque in 0–600 mm/s even close-up, fades in 600–1000 mm/s lateral, spares axial and brief lag', () => {
  const viewer = [0, 0, 0]; // tag 0.35 m ahead along +z (close arm's reach)
  const mk = (dx, dz, opts) => {
    const p = createTagMotionPredictor(opts);
    for (let i = 0; i < 4; i++) {
      p.observe(1, vectorRigid([0, 0, 0, i * dx, 0, 0.35 + i * dz]), 1000 + i * 33);
    }
    return p;
  };
  const run = (p, endMs = 1255) => {
    let f = 1;
    for (let t = 1099; t <= endMs; t += 11) f = p.trackingFade(1, t, viewer);
    return f;
  };

  // ~0.545 m/s lateral (< 0.60 m/s) at close 0.35 m distance: stays opaque
  const stableLat = run(mk(0.018, 0));
  assert.ok(stableLat > 0.95, `0.545 m/s lateral should stay opaque, got ${stableLat}`);

  // ~0.909 m/s lateral (inside 600–1000 mm/s red zone): fades clearly, never below fadeMinAlpha
  const fastLat = run(mk(0.030, 0));
  assert.ok(
    fastLat < 0.55 && fastLat >= TAG_PREDICTOR_DEFAULTS.fadeMinAlpha - 1e-9,
    `0.909 m/s lateral should fade between fadeMinAlpha and 0.55, got ${fastLat}`,
  );

  // ~0.909 m/s axial along view ray: weighted at 0.30 (~0.27 m/s effective) -> stays opaque
  const fastAxial = run(mk(0, 0.030));
  assert.ok(fastAxial > 0.95, `0.909 m/s axial should stay opaque, got ${fastAxial}`);

  // Active tracking with brief lag (age = 0.25 s < fadeStaleStartSec 0.35 s): stays opaque
  const briefLag = run(mk(0.0005, 0), 1349);
  assert.ok(briefLag > 0.95, `brief 250 ms lag should stay opaque, got ${briefLag}`);

  // True dropout (age = 1.0 s > fadeStaleEndSec 0.90 s): fades out smoothly
  const dropout = run(mk(0.0005, 0), 2100);
  assert.ok(dropout < 0.10, `1.0 s stale tag should fade out, got ${dropout}`);

  // Default fadeLearnLoss: false -> noteLoss() does not mutate limitMps
  const fixed = mk(0.030, 0);
  run(fixed);
  assert.equal(fixed.noteLoss(1), TAG_PREDICTOR_DEFAULTS.fadeInitialLimitMps);
  assert.equal(fixed.limitMps(), TAG_PREDICTOR_DEFAULTS.fadeInitialLimitMps);

  // Opt-in fadeLearnLoss: true -> noteLoss() updates limitMps
  const adaptive = mk(0.030, 0, { fadeLearnLoss: true });
  run(adaptive);
  const before = adaptive.limitMps();
  assert.notEqual(adaptive.noteLoss(1), before);

  // Unknown tag -> 1
  assert.equal(fixed.trackingFade(404, 1000, viewer), 1);
});

test('predict supports optional bounded negative minLeadSec for Passthrough display lag without affecting default callers', () => {
  const pred = createTagMotionPredictor();
  const basePose = vectorRigid([0, 0, 0, 0.20, 0.10, 0.05]);
  pred.seed(320, basePose, 2000, {
    velL: [0.80, 0, 0],
    angLocal: [0, 0, 2.0],
  });

  // Default caller (no opts) never returns negative lead
  const defPast = pred.predict(320, 1980);
  assert.ok(defPast && isRigidPose(defPast));
  assert.equal(defPast[12], 0.20);

  // Display caller with minLeadSec: -0.05 moves smoothly behind observation timestamp
  const opts = { minLeadSec: -0.05 };
  const neg = pred.predict(320, 1980, opts);
  assert.ok(neg && isRigidPose(neg));
  const expectedNegX = 0.20 - 0.80 * 0.020 * TAG_PREDICTOR_DEFAULTS.leadGain;
  assert.ok(Math.abs(neg[12] - expectedNegX) < 1e-5, `expected x=${expectedNegX}, got ${neg[12]}`);

  // Smooth continuity across rawAgeSec = 0 (t = 1995..2005)
  const beforeZero = pred.predict(320, 1995, opts);
  const atZero = pred.predict(320, 2000, opts);
  const afterZero = pred.predict(320, 2005, opts);
  assert.ok(beforeZero[12] < atZero[12] && atZero[12] < afterZero[12]);
  assert.ok(Math.abs(atZero[12] - 0.20) < 1e-6);

  // Clamped at minLeadSec = -0.05 for large negative offset
  const clamped = pred.predict(320, 1850, opts);
  assert.ok(clamped && isRigidPose(clamped));
  assert.ok(Math.abs(clamped[12] - (0.20 - 0.80 * 0.05)) < 1e-5, `expected clamped x=0.16, got ${clamped[12]}`);

  // Stationary tag remains at exact pose even with minLeadSec
  pred.seed(321, basePose, 2000, { velL: [0, 0, 0], angLocal: [0, 0, 0] });
  const stillNeg = pred.predict(321, 1980, opts);
  assert.equal(stillNeg[12], 0.20);
});
