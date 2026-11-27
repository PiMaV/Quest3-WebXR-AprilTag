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
