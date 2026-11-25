import test from 'node:test';
import assert from 'node:assert/strict';
import {
  buildHandSample,
  createCoupledTrailBuffer,
  createHandHistory,
  createTagHandCoupler,
} from '../src/hand-coupling.mjs';
import {isRigidPose} from '../src/anchor.mjs';
import {product,vectorRigid} from '../src/runtime.mjs';

function transformPt(m,p){
  return [
    m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],
    m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],
    m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14],
  ];
}

// Local hand skeleton around palm origin (meters).
function makeHandAt(lFromPalm,xrTimeMs,handedness='right',{pinchGapM=0.018,fingerOffsetZ=0}={}){
  const half=pinchGapM/2;
  const joints={
    wrist:transformPt(lFromPalm,[0,-0.06,0]),
    indexMcp:transformPt(lFromPalm,[0.028,0.03,0]),
    middleMcp:transformPt(lFromPalm,[0,0.035,0]),
    pinkyMcp:transformPt(lFromPalm,[-0.028,0.025,0]),
    thumbTip:transformPt(lFromPalm,[0.012-half,0.065,fingerOffsetZ]),
    indexTip:transformPt(lFromPalm,[0.012+half,0.065,fingerOffsetZ]),
    middleTip:transformPt(lFromPalm,[-0.005,0.075,fingerOffsetZ]),
  };
  return buildHandSample(joints,xrTimeMs,handedness);
}

test('buildHandSample creates a valid right-handed 6DOF palm frame and handHistory interpolates at delayed t_cap',()=>{
  const pose0=vectorRigid([0,0,0,0.10,0.20,0.30]);
  const pose1=vectorRigid([0,0,0.4,0.20,0.20,0.30]);
  const s0=makeHandAt(pose0,1000,'right');
  const s1=makeHandAt(pose1,1050,'right');
  assert.ok(s0&&isRigidPose(s0.lFromHand));
  assert.ok(s1&&isRigidPose(s1.lFromHand));

  const hh=createHandHistory();
  hh.push(s0);
  hh.push(s1);
  const mid=hh.sampleAt('right',1025);
  assert.ok(mid&&isRigidPose(mid.lFromHand));
  assert.ok(Math.abs(mid.palmCenter[0]-0.15)<1e-3);

  // Fallback to wristMatrix when metacarpals are unavailable.
  const fb=buildHandSample({wristMatrix:pose0},1100,'left');
  assert.ok(fb&&isRigidPose(fb.lFromHand));
});

test('resting tag with nearby stationary hand and fast hand pass-over never false-couple',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const staticTag=vectorRigid([0,0,0,0.30,0.20,0.01]);

  // 1. Hand rests 3 cm next to stationary tag for 5 frames -> never couples.
  for(let t=1000;t<=1400;t+=100){
    const h=makeHandAt(vectorRigid([0,0,0,0.27,0.14,0.01]),t,'right');
    hh.push(h);
    const res=coupler.observeOptical({tagId:300,lFromTag:staticTag,targetXrTimeMs:t,confidence:1,handHistory:hh});
    assert.notEqual(res.mode,'coupled');
  }
  assert.equal(coupler.predict(300,hh,1410).coupled,false);

  // 2. Hand swipes 90 mm quickly right over the stationary tag -> still rejected!
  for(let t=1420;t<=1620;t+=20){
    const frac=(t-1420)/200;
    hh.push(makeHandAt(vectorRigid([0,0,0,0.27+frac*0.09,0.14,0.03]),t,'right'));
  }
  const swipe1=coupler.observeOptical({tagId:300,lFromTag:staticTag,targetXrTimeMs:1520,confidence:1,handHistory:hh});
  const swipe2=coupler.observeOptical({tagId:300,lFromTag:staticTag,targetXrTimeMs:1620,confidence:1,handHistory:hh});
  assert.notEqual(swipe1.mode,'coupled');
  assert.notEqual(swipe2.mode,'coupled');
  assert.equal(coupler.predict(300,hh,1630).coupled,false);
});

test('externally pushed tag rejects non-carrying nearby hand and locks onto the hand actually moving it',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();

  // Left hand stays still near the tag; right hand carries the tag 70 mm along +X.
  const leftIdle=vectorRigid([0,0,0,0.20,0.14,0.02]);
  const right0=vectorRigid([0,0,0,0.22,0.14,0.02]);
  const right1=vectorRigid([0,0,0,0.29,0.14,0.02]);
  const sL0=makeHandAt(leftIdle,1000,'left');
  const sL1=makeHandAt(leftIdle,1120,'left');
  const sR0=makeHandAt(right0,1000,'right');
  const sR1=makeHandAt(right1,1120,'right');
  hh.push(sL0);hh.push(sL1);
  hh.push(sR0);hh.push(sR1);

  const offsetInRight=vectorRigid([0,0,0,0.01,0.065,0]);
  const tag0=product(sR0.lFromHand,offsetInRight);
  const tag1=product(sR1.lFromHand,offsetInRight);

  // First test: only left hand exists while tag is pushed externally -> left hand must NOT couple.
  const hhLeftOnly=createHandHistory();
  hhLeftOnly.push(sL0);hhLeftOnly.push(sL1);
  const cLeftOnly=createTagHandCoupler();
  cLeftOnly.observeOptical({tagId:301,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hhLeftOnly});
  const extRes=cLeftOnly.observeOptical({tagId:301,lFromTag:tag1,targetXrTimeMs:1120,confidence:1,handHistory:hhLeftOnly});
  assert.notEqual(extRes.mode,'coupled');

  // Second test: both hands tracked -> right hand (coherent motion) wins and locks.
  coupler.observeOptical({tagId:301,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh});
  const r2=coupler.observeOptical({tagId:301,lFromTag:tag1,targetXrTimeMs:1120,confidence:1,handHistory:hh});
  assert.equal(r2.mode,'coupled');
  assert.equal(r2.handedness,'right');
  assert.equal(coupler.isHandCarrying('right',301),true);
  assert.equal(coupler.isHandCarrying('left',301),false);
});

test('pinched 20 mm card couples accurately despite 140 ms camera lag and releases at 0 ms on pinch open',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const offset=vectorRigid([0,0,0,0.012,0.065,0]);

  // Simulate 90 Hz hand carrying the card smoothly at 0.4 m/s from t=1000..1500 ms.
  const handPoseAt=t=>vectorRigid([0,0,0.0004*(t-1000),0.10+0.0004*(t-1000),0.10,0.20]);
  const samples=new Map();
  for(let t=1000;t<=1500;t+=11){
    const s=makeHandAt(handPoseAt(t),t,'right',{pinchGapM:0.018});
    hh.push(s);
    samples.set(t,s);
  }

  // Optical observations arrive 140 ms late, stamped with targetXrTimeMs = 1044 and 1154.
  const tagAt1044=product(hh.sampleAt('right',1044).lFromHand,offset);
  const tagAt1154=product(hh.sampleAt('right',1154).lFromHand,offset);

  const r1=coupler.observeOptical({tagId:300,lFromTag:tagAt1044,targetXrTimeMs:1044,confidence:1,handHistory:hh});
  assert.equal(r1.mode,'candidate');
  const r2=coupler.observeOptical({tagId:300,lFromTag:tagAt1154,targetXrTimeMs:1154,confidence:1,handHistory:hh});
  assert.equal(r2.mode,'coupled');
  assert.equal(r2.gripKind,'pinch');
  assert.equal(r2.handedness,'right');

  // Predict at t_now = 1294 (140 ms ahead of latest camera frame!): must match true tag pose to < 1 mm!
  const pred=coupler.predict(300,hh,1294);
  assert.equal(pred.coupled,true);
  const truth1294=product(hh.latest('right',1294).lFromHand,offset);
  const errMm=Math.hypot(pred.lFromTag[12]-truth1294[12],pred.lFromTag[13]-truth1294[13],pred.lFromTag[14]-truth1294[14])*1000;
  assert.ok(errMm<1.0,`expected < 1 mm lag-free prediction error, got ${errMm.toFixed(2)} mm`);

  // User opens fingers (pinch gap jumps from 18 mm -> 58 mm) and pulls fingertips back -> instant 0-ms release!
  hh.push(makeHandAt(handPoseAt(1511),1511,'right',{pinchGapM:0.058,fingerOffsetZ:-0.03}));
  const rel=coupler.predict(300,hh,1511);
  assert.equal(rel.coupled,false);
  assert.equal(rel.released,true);
  assert.equal(rel.reason,'pinch-opened');
});

test('18 cm wand/stick couples in wand mode, ignores thumb wiggle, adapts softly, and releases via optical watchdog',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  // Tag sits on the tip of an 18 cm wand extending along +Y from palm.
  const wandOffset=vectorRigid([0.1,0,0,0,0.18,0]);

  const handPoseAt=t=>vectorRigid([0,0,0.002*(t-1000),0.20,0.10,0.25]);
  for(let t=1000;t<=1400;t+=15){
    hh.push(makeHandAt(handPoseAt(t),t,'right',{pinchGapM:0.055}));
  }

  const tag0=product(hh.sampleAt('right',1030).lFromHand,wandOffset);
  const tag1=product(hh.sampleAt('right',1110).lFromHand,wandOffset);
  const tag2=product(hh.sampleAt('right',1190).lFromHand,wandOffset);
  coupler.observeOptical({tagId:305,lFromTag:tag0,targetXrTimeMs:1030,confidence:1,handHistory:hh});
  const firstDelta=coupler.observeOptical({tagId:305,lFromTag:tag1,targetXrTimeMs:1110,confidence:1,handHistory:hh});
  assert.equal(firstDelta.mode,'candidate');
  const locked=coupler.observeOptical({tagId:305,lFromTag:tag2,targetXrTimeMs:1190,confidence:1,handHistory:hh});
  assert.equal(locked.mode,'coupled');
  assert.equal(locked.gripKind,'wand');

  // Opening thumb/index slightly on the wand handle must NOT trigger false pinch-open release.
  hh.push(makeHandAt(handPoseAt(1415),1415,'right',{pinchGapM:0.095,fingerOffsetZ:-0.03}));
  assert.equal(coupler.predict(305,hh,1415).coupled,true);

  // Small 8 mm grip shift adapts smoothly without releasing.
  const shiftedOffset=vectorRigid([0.1,0,0,0.008,0.18,0]);
  const tagShifted=product(hh.sampleAt('right',1300).lFromHand,shiftedOffset);
  const adapted=coupler.observeOptical({tagId:305,lFromTag:tagShifted,targetXrTimeMs:1300,confidence:1,handHistory:hh});
  assert.equal(adapted.mode,'coupled');
  assert.equal(adapted.event,'adapted');

  // User sets wand down (tag stays at tagShifted while hand pulls 80 mm away) -> optical watchdog releases immediately.
  const pulledAwayHand=makeHandAt(vectorRigid([0,0,0.8,0.28,0.10,0.35]),1450,'right',{pinchGapM:0.07});
  hh.push(pulledAwayHand);
  const released=coupler.observeOptical({tagId:305,lFromTag:tagShifted,targetXrTimeMs:1450,confidence:1,handHistory:hh});
  assert.equal(released.mode,'free');
  assert.equal(released.event,'released');
});

test('hand-tracking dropout (>120 ms) and optical stale timeout (>900 ms) release cleanly',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const offset=vectorRigid([0,0,0,0.01,0.065,0]);

  for(let t=1000;t<=1200;t+=20){
    hh.push(makeHandAt(vectorRigid([0,0,0,0.10+0.0005*(t-1000),0.10,0.20]),t,'left',{pinchGapM:0.016}));
  }
  coupler.observeOptical({tagId:310,lFromTag:product(hh.sampleAt('left',1020).lFromHand,offset),targetXrTimeMs:1020,confidence:1,handHistory:hh});
  const lock=coupler.observeOptical({tagId:310,lFromTag:product(hh.sampleAt('left',1140).lFromHand,offset),targetXrTimeMs:1140,confidence:1,handHistory:hh});
  assert.equal(lock.mode,'coupled');

  // Hand stops reporting after t=1200 -> at t=1350 (>120 ms stale), must release cleanly.
  const lost=coupler.predict(310,hh,1350);
  assert.equal(lost.coupled,false);
  assert.equal(lost.released,true);
  assert.equal(lost.reason,'hand-tracking-lost');

  // Re-lock and test optical stale timeout (>900 ms without optical confirmation while hand keeps moving).
  for(let t=2000;t<=3100;t+=20){
    hh.push(makeHandAt(vectorRigid([0,0,0,0.10+0.0003*(t-2000),0.10,0.20]),t,'left',{pinchGapM:0.016}));
  }
  coupler.observeOptical({tagId:311,lFromTag:product(hh.sampleAt('left',2020).lFromHand,offset),targetXrTimeMs:2020,confidence:1,handHistory:hh});
  assert.equal(coupler.observeOptical({tagId:311,lFromTag:product(hh.sampleAt('left',2140).lFromHand,offset),targetXrTimeMs:2140,confidence:1,handHistory:hh}).mode,'coupled');
  const stale=coupler.predict(311,hh,3080);
  assert.equal(stale.coupled,false);
  assert.equal(stale.released,true);
  assert.equal(stale.reason,'optical-timeout');
});

test('hand moving 50 mm near a barely moving tag (6-8 mm or 6 deg PnP jitter) never couples',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();

  hh.push(makeHandAt(vectorRigid([0,0,0,0.20,0.14,0.02]),1000,'right',{pinchGapM:0.06}));
  hh.push(makeHandAt(vectorRigid([0,0,0.05,0.25,0.14,0.02]),1100,'right',{pinchGapM:0.06}));
  hh.push(makeHandAt(vectorRigid([0,0,0.10,0.30,0.14,0.02]),1200,'right',{pinchGapM:0.06}));

  const tag0=vectorRigid([0,0,0,0.21,0.21,0.02]);
  const tag1=vectorRigid([0,0,0.105,0.217,0.212,0.02]); // +7.3 mm, +6 deg PnP noise
  const tag2=vectorRigid([0,0,0.105,0.218,0.211,0.02]); // +8.1 mm total

  assert.equal(coupler.observeOptical({tagId:302,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh}).mode,'candidate');
  assert.notEqual(coupler.observeOptical({tagId:302,lFromTag:tag1,targetXrTimeMs:1100,confidence:1,handHistory:hh}).mode,'coupled');
  assert.notEqual(coupler.observeOptical({tagId:302,lFromTag:tag2,targetXrTimeMs:1200,confidence:1,handHistory:hh}).mode,'coupled');
  assert.equal(coupler.isHandCarrying('right',302),false);
});

test('one-tag-per-hand exclusivity prevents two nearby tags from coupling to the same hand',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const off311=vectorRigid([0,0,0,0.012,0.065,0]);
  const off310=vectorRigid([0,0,0,0.012,0.145,0]);

  for(let t=1000;t<=1240;t+=40){
    const x=0.10+0.0005*(t-1000);
    hh.push(makeHandAt(vectorRigid([0,0,0,x,0.10,0.20]),t,'right',{pinchGapM:0.016}));
  }

  // First delta: #311 (pinch) locks onto right hand.
  coupler.observeOptical({tagId:311,lFromTag:product(hh.sampleAt('right',1000).lFromHand,off311),targetXrTimeMs:1000,confidence:1,handHistory:hh});
  const lock311=coupler.observeOptical({tagId:311,lFromTag:product(hh.sampleAt('right',1080).lFromHand,off311),targetXrTimeMs:1080,confidence:1,handHistory:hh});
  assert.equal(lock311.mode,'coupled');
  assert.equal(lock311.handedness,'right');

  // Nearby #310 also moves identically in the same hand's radius -> must be blocked while right carries #311.
  coupler.observeOptical({tagId:310,lFromTag:product(hh.sampleAt('right',1000).lFromHand,off310),targetXrTimeMs:1000,confidence:1,handHistory:hh});
  const try310=coupler.observeOptical({tagId:310,lFromTag:product(hh.sampleAt('right',1080).lFromHand,off310),targetXrTimeMs:1080,confidence:1,handHistory:hh});
  assert.notEqual(try310.mode,'coupled');
  assert.equal(coupler.isHandCarrying('right',310),false);
  assert.equal(coupler.isHandCarrying('right',311),true);
});

test('single nudge then pull-away in open-hand wand mode resets confirmation and never locks',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const wandOffset=vectorRigid([0,0,0,0.0,0.15,0]);

  const h0=makeHandAt(vectorRigid([0,0,0,0.20,0.10,0.20]),1000,'right',{pinchGapM:0.06});
  const h1=makeHandAt(vectorRigid([0,0,0,0.235,0.10,0.20]),1080,'right',{pinchGapM:0.06});
  const h2=makeHandAt(vectorRigid([0,0,0,0.285,0.10,0.20]),1160,'right',{pinchGapM:0.06});
  hh.push(h0);hh.push(h1);hh.push(h2);

  const tag0=product(h0.lFromHand,wandOffset);
  const tag1=product(h1.lFromHand,wandOffset); // 35 mm nudge together -> 1st coherent delta only ('candidate')
  const tag2=tag1.slice(); // tag stops while hand continues another 50 mm

  coupler.observeOptical({tagId:306,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh});
  const r1=coupler.observeOptical({tagId:306,lFromTag:tag1,targetXrTimeMs:1080,confidence:1,handHistory:hh});
  assert.equal(r1.mode,'candidate');
  const r2=coupler.observeOptical({tagId:306,lFromTag:tag2,targetXrTimeMs:1160,confidence:1,handHistory:hh});
  assert.equal(r2.mode,'candidate');
  assert.equal(coupler.isHandCarrying('right',306),false);
});

test('coupled tag releases in 1 frame with tag-stationary and returns true lastOpticalPose when hand moves away',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const offset=vectorRigid([0,0,0,0.012,0.065,0]);

  const h0=makeHandAt(vectorRigid([0,0,0,0.10,0.10,0.20]),1000,'right',{pinchGapM:0.018});
  const h1=makeHandAt(vectorRigid([0,0,0,0.14,0.10,0.20]),1080,'right',{pinchGapM:0.018});
  const h2=makeHandAt(vectorRigid([0,0,0,0.175,0.10,0.20]),1160,'right',{pinchGapM:0.018}); // +35 mm hand move
  hh.push(h0);hh.push(h1);hh.push(h2);

  const tag0=product(h0.lFromHand,offset);
  const tag1=product(h1.lFromHand,offset);
  coupler.observeOptical({tagId:307,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh});
  const coupled=coupler.observeOptical({tagId:307,lFromTag:tag1,targetXrTimeMs:1080,confidence:1,handHistory:hh});
  assert.equal(coupled.mode,'coupled');

  // Optical tag barely moves (+3 mm) while hand moved 35 mm -> instant 1-frame release!
  const tagAlmostStill=tag1.slice();
  tagAlmostStill[12]+=0.003;
  const rel=coupler.observeOptical({tagId:307,lFromTag:tagAlmostStill,targetXrTimeMs:1160,confidence:1,handHistory:hh});
  assert.equal(rel.event,'released');
  assert.equal(rel.mode,'free');
  assert.equal(rel.reason,'tag-stationary');
  assert.ok(rel.lastOpticalPose);
  const snapErr=Math.hypot(
    rel.lastOpticalPose[12]-tagAlmostStill[12],
    rel.lastOpticalPose[13]-tagAlmostStill[13],
    rel.lastOpticalPose[14]-tagAlmostStill[14],
  );
  assert.ok(snapErr<1e-9);
});

test('basis-flip protection refuses palm/wrist interpolation and releases on hand-frame-changed',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const offset=vectorRigid([0,0,0,0.012,0.065,0]);

  const s0=makeHandAt(vectorRigid([0,0,0,0.10,0.10,0.20]),1000,'left',{pinchGapM:0.016});
  const s1=makeHandAt(vectorRigid([0,0,0,0.14,0.10,0.20]),1080,'left',{pinchGapM:0.016});
  assert.equal(s0.frameKind,'palm');
  assert.equal(s1.frameKind,'palm');
  hh.push(s0);hh.push(s1);

  const tag0=product(s0.lFromHand,offset);
  const tag1=product(s1.lFromHand,offset);
  coupler.observeOptical({tagId:308,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh});
  assert.equal(coupler.observeOptical({tagId:308,lFromTag:tag1,targetXrTimeMs:1080,confidence:1,handHistory:hh}).mode,'coupled');

  // Tracking degrades to wristMatrix fallback at t=1120 -> sampleAt between 1080 (palm) and 1120 (wrist) returns null.
  const sWrist=buildHandSample({wristMatrix:vectorRigid([0,0,0.5,0.15,0.10,0.20])},1120,'left');
  assert.equal(sWrist.frameKind,'wrist');
  hh.push(sWrist);
  assert.equal(hh.sampleAt('left',1100),null);

  // predict() sees frameKind changed from 'palm' to 'wrist' -> immediate release with lastOpticalPose = tag1.
  const rel=coupler.predict(308,hh,1120);
  assert.equal(rel.coupled,false);
  assert.equal(rel.released,true);
  assert.equal(rel.reason,'hand-frame-changed');
  assert.ok(rel.lastOpticalPose);
  assert.ok(Math.hypot(rel.lastOpticalPose[12]-tag1[12],rel.lastOpticalPose[13]-tag1[13],rel.lastOpticalPose[14]-tag1[14])<1e-9);
});

test('unconfirmed drift cap releases when hand shoots > 22 cm away without optical confirmation',()=>{
  const hh=createHandHistory();
  const coupler=createTagHandCoupler();
  const offset=vectorRigid([0,0,0,0.012,0.065,0]);

  const s0=makeHandAt(vectorRigid([0,0,0,0.10,0.10,0.20]),1000,'right',{pinchGapM:0.018});
  const s1=makeHandAt(vectorRigid([0,0,0,0.14,0.10,0.20]),1080,'right',{pinchGapM:0.018});
  hh.push(s0);hh.push(s1);

  const tag0=product(s0.lFromHand,offset);
  const tag1=product(s1.lFromHand,offset);
  coupler.observeOptical({tagId:309,lFromTag:tag0,targetXrTimeMs:1000,confidence:1,handHistory:hh});
  assert.equal(coupler.observeOptical({tagId:309,lFromTag:tag1,targetXrTimeMs:1080,confidence:1,handHistory:hh}).mode,'coupled');

  // Hand flies 24 cm in 100 ms before optical camera catches up -> unconfirmed-drift release + lastOpticalPose = tag1.
  const sFar=makeHandAt(vectorRigid([0,0,0,0.38,0.10,0.20]),1180,'right',{pinchGapM:0.018});
  hh.push(sFar);
  const rel=coupler.predict(309,hh,1180);
  assert.equal(rel.coupled,false);
  assert.equal(rel.released,true);
  assert.equal(rel.reason,'unconfirmed-drift');
  assert.ok(rel.lastOpticalPose);
  assert.ok(Math.hypot(rel.lastOpticalPose[12]-tag1[12],rel.lastOpticalPose[13]-tag1[13],rel.lastOpticalPose[14]-tag1[14])<1e-9);
});

test('createCoupledTrailBuffer deduplicates tiny steps, commits only up to optical xrTimeMs, and discards unconfirmed motion',()=>{
  const buf=createCoupledTrailBuffer({minStepM:0.01,minStepDeg:5,maxPoints:10});
  const p0=vectorRigid([0,0,0,0.100,0.0,0.0]);
  const pTiny=vectorRigid([0,0,0,0.104,0.0,0.0]); // 4 mm < 10 mm -> skipped
  const p1=vectorRigid([0,0,0,0.120,0.0,0.0]);
  const p2=vectorRigid([0,0,0,0.150,0.0,0.0]);
  const p3=vectorRigid([0,0,0,0.200,0.0,0.0]);

  assert.equal(buf.push({xrTimeMs:100,atMs:200,pose:p0}),true);
  assert.equal(buf.push({xrTimeMs:110,atMs:210,pose:pTiny}),false);
  assert.equal(buf.push({xrTimeMs:110,atMs:210,pose:p1}),true);
  assert.equal(buf.push({xrTimeMs:120,atMs:220,pose:p2}),true);
  assert.equal(buf.push({xrTimeMs:150,atMs:250,pose:p3}),true);
  assert.equal(buf.size(),4);

  // Confirm up to xrTimeMs=120 -> flushes p0, p1, p2; keeps speculative p3.
  const committed=buf.commitUpTo(120);
  assert.equal(committed.length,3);
  assert.deepEqual(committed.map(x=>x.xrTimeMs),[100,110,120]);
  assert.equal(buf.size(),1);

  // False escape discard wipes remaining speculative p3.
  assert.equal(buf.discard(),1);
  assert.equal(buf.size(),0);
});
