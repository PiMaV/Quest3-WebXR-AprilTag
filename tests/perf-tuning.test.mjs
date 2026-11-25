import test from 'node:test';
import assert from 'node:assert/strict';
import {Matrix4} from '../vendor/three/three.module.js';
import {createLAnchorController,isRigidPose} from '../src/anchor.mjs';
import {createCalibrationAssistant} from '../src/assistant.mjs';
import {viewerPoseHorizon} from '../src/profile.mjs';
import {associateAnchorCapture} from '../src/anchor-audit.mjs';
import {observeReference} from '../src/rig.mjs';
import {createPerfTracker} from '../src/perf-stats.mjs';
import {createAdaptiveQualityController} from '../src/adaptive-quality.mjs';
import {createPerimeterFence} from '../src/perimeter-fence.mjs';
import {vectorRigid} from '../src/runtime.mjs';
import {synthetic,rig} from './synthetic.mjs';

test('isRigidPose accepts valid rigid matrices and rejects malformed/non-rigid ones without allocating',()=>{
  const rot=new Matrix4().makeRotationFromEuler({x:0.3,y:-0.7,z:1.1,isEuler:true,order:'XYZ'});
  rot.setPosition(0.25,-0.1,1.4);
  const arr=rot.toArray();
  assert.equal(isRigidPose(arr),true);
  assert.equal(isRigidPose(new Float32Array(arr)),true);

  assert.equal(isRigidPose(null),false);
  assert.equal(isRigidPose(arr.slice(0,15)),false);

  const withNaN=arr.slice();withNaN[5]=NaN;
  assert.equal(isRigidPose(withNaN),false);

  const scaled=arr.slice();scaled[0]*=1.02;scaled[1]*=1.02;scaled[2]*=1.02;
  assert.equal(isRigidPose(scaled),false);

  const sheared=arr.slice();sheared[4]+=0.05;
  assert.equal(isRigidPose(sheared),false);

  const reflected=arr.slice();reflected[0]=-reflected[0];reflected[1]=-reflected[1];reflected[2]=-reflected[2];
  assert.equal(isRigidPose(reflected),false);

  const proj=arr.slice();proj[3]=0.01;
  assert.equal(isRigidPose(proj),false);
});

test('anchor.phase() and anchor.summary() match anchor.state() across lifecycle transitions',async()=>{
  const ctx={sessionId:'perf-1',spaceEpoch:0};
  const base=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,-0.8,1];
  const session={enabledFeatures:['anchors']};
  const reference={};
  const native={anchorSpace:{},deleted:0,delete(){this.deleted++;}};
  let hasPose=true;
  const controller=createLAnchorController({
    makeRigidTransform:(position,orientation)=>({position,orientation}),
    clock:()=>2600,
  });
  const frame={
    session,
    createAnchor:()=>Promise.resolve(native),
    getPose:()=>hasPose?{transform:{matrix:base},emulatedPosition:false}:null,
  };

  assert.equal(controller.phase(),'idle');
  assert.equal(controller.summary().phase,'idle');

  controller.begin(session,reference,ctx,900);
  assert.equal(controller.phase(),'collecting');
  assert.equal(controller.summary().phase,controller.state().phase);

  for(let i=0;i<5;i++){
    const at=1000+i*350;
    controller.observe({
      sample:{frameEvidence:{fresh:true},serial:i,generation:1,context:{...ctx},submittedAtPerformanceMs:at,videoFrameMetadata:{captureTime:at-20}},
      tool:{tagIds:[350,351,352],mode:'joint-rigid-tags',pointCount:12,reprojectionRmsPx:0.3},
      candidate:{...ctx,association:'validated-profile-at-capture-time',calibrationProfileId:'prof-1'},
      worldFromL:base.slice(),
    },at+200);
  }
  assert.equal(controller.phase(),'ready');
  {
    const sum=controller.summary(),st=controller.state();
    assert.equal(sum.phase,st.phase);
    assert.equal(sum.quality.count,st.quality.count);
    assert.equal(sum.quality.required,st.quality.required);
    assert.equal(sum.quality.ready,st.quality.ready);
  }

  controller.update(frame,2600);
  await new Promise(r=>setImmediate(r));
  assert.equal(controller.phase(),'created');

  controller.update(frame,2650);
  assert.equal(controller.phase(),'tracking');
  assert.equal(controller.summary().posesTracked,controller.state().posesTracked);

  hasPose=false;
  controller.update(frame,2700);
  assert.equal(controller.phase(),'lost');
  assert.equal(controller.summary().posesMissing,controller.state().posesMissing);
  controller.end(2800);
});

test('assistant.phase() and assistant.summary() match assistant.state()',async()=>{
  const memoryStore=new Map();
  const store={
    async load(d){return memoryStore.get(d.key)??null;},
    async save(p){memoryStore.set(p.device.key,p);return p;},
  };
  const a=createCalibrationAssistant({
    rig,
    device:{key:'dev-1',mode:{sourceSize:[1280,1080]}},
    imageSize:[1280,1080],
    store,
    workerFactory:()=>({terminate(){},postMessage(){}}),
  });
  assert.equal(a.phase(),'idle');
  assert.deepEqual(
    {phase:a.summary().phase,count:a.summary().count,required:a.summary().required,message:a.summary().message},
    {phase:a.state().phase,count:a.state().count,required:a.state().required,message:a.state().message},
  );
  await a.begin();
  assert.equal(a.phase(),'scanning');
  assert.deepEqual(
    {phase:a.summary().phase,count:a.summary().count,required:a.summary().required,message:a.summary().message},
    {phase:a.state().phase,count:a.state().count,required:a.state().required,message:a.state().message},
  );
  a.dispose();
});

test('windowed O(log N) viewerPoseHorizon and associateAnchorCapture handle 24,000-entry history rapidly and accurately',()=>{
  const ctx={sessionId:'s-big',spaceEpoch:0};
  const L=[1,0,0,0,0,1,0,0,0,0,1,0,0.2,0.1,-0.9,1];
  const X=[1,0,0,0,0,1,0,0,0,0,1,0,0.03,0,0,1];
  const profile={id:'p-big',viewerFromCameraGL:X,timeOffsetMs:-50};

  const N=24000;
  const history=new Array(N);
  for(let i=0;i<N;i++){
    const t=1000+i*11;
    const x=i*0.0001;
    const isOldEpoch=i<100;
    const isLost=i>=18000&&i<18050;
    const isGapped=i>=20000&&i<20020;
    history[i]={
      sessionId:ctx.sessionId,
      spaceEpoch:isOldEpoch?99:0,
      xrRafTimeMs:isGapped?t+i*0.05:t,
      emulatedPosition:false,
      worldFromViewer:[1,0,0,0,0,1,0,0,0,0,1,0,x,0,0,1],
      worldFromAnchor:L,
      anchorTrackingState:isLost?'lost':'tracking',
    };
  }
  // Keep xrRafTimeMs strictly monotonic across the artificial gap at i=20000.
  for(let i=20000;i<N;i++)history[i].xrRafTimeMs=1000+i*11+250;

  const makeSample=(serial,cap)=>({
    serial,
    generation:1,
    context:ctx,
    submittedAtPerformanceMs:cap+15,
    videoFrameMetadata:{captureTime:cap},
    frameEvidence:{fresh:true},
    settings:{experimentalCalibration:profile},
  });

  // Valid lookup deep inside 24k history.
  const midTargetIdx=15000;
  const midCap=history[midTargetIdx].xrRafTimeMs+55; // target = xrRafTimeMs + 5
  const midSample=makeSample(1,midCap);
  const t0=performance.now();
  for(let k=0;k<100;k++){
    const assoc=associateAnchorCapture(history,midSample,profile,midCap+20);
    assert.ok(assoc);
  }
  const elapsedMs=performance.now()-t0;
  assert.ok(elapsedMs<50,`100 lookups over 24k entries should be fast, took ${elapsedMs.toFixed(2)}ms`);

  const assoc=associateAnchorCapture(history,midSample,profile,midCap+20);
  assert.ok(assoc);
  const expectedX=(midTargetIdx+5/11)*0.0001+X[12];
  assert.ok(Math.abs(assoc.worldFromCamera[12]-expectedX)<1e-6);

  // Mixed epoch entry is excluded.
  const epochCap=history[50].xrRafTimeMs+50;
  assert.equal(viewerPoseHorizon(history,makeSample(2,epochCap),profile).status,'invalid');

  // Lost anchor tracking state refuses association.
  const lostCap=history[18020].xrRafTimeMs+50;
  assert.equal(associateAnchorCapture(history,makeSample(3,lostCap),profile,lostCap+20),null);

  // >100 ms XR history gap between index 19999 and 20000 refuses interpolation.
  const gapTime=(history[19999].xrRafTimeMs+history[20000].xrRafTimeMs)/2;
  const gapCap=gapTime+50;
  assert.equal(viewerPoseHorizon(history,makeSample(4,gapCap),profile).status,'invalid');

  // Future target returns 'future'.
  const futCap=history.at(-1).xrRafTimeMs+80;
  assert.equal(viewerPoseHorizon(history,makeSample(5,futCap),profile).status,'future');
});

test('consensus fast path matches full-fit and preserves outlier/disagreement rejection',()=>{
  const d=synthetic({noise:0.15,count:5}),K=d.truth.K,row=d.rows[1];
  const markers=row.tags.map(t=>({id:t.id,corners:structuredClone(t.pixels)}));
  const clean=observeReference(markers,rig,K);
  assert.equal(clean.tools.length,1);
  assert.deepEqual(clean.tools[0].tagIds,[350,351,352]);
  assert.ok(clean.tools[0].reprojectionRmsPx<0.5);

  const oneBad=structuredClone(markers);
  oneBad[1].corners.forEach(p=>{p.y+=45;p.x-=35;});
  const filtered=observeReference(oneBad,rig,K);
  assert.equal(filtered.tools.length,1);
  assert.deepEqual(filtered.tools[0].tagIds,[350,352]);
  assert.ok(filtered.rejected.some(r=>r.id===351&&r.reason==='rigid_tag_outlier'));

  const twoDisagree=structuredClone(markers.slice(0,2));
  twoDisagree[0].corners.forEach(p=>{p.y-=60;});
  const rejected=observeReference(twoDisagree,rig,K);
  assert.equal(rejected.tools.length,0);
  assert.ok(rejected.rejected.every(r=>r.reason==='inconsistent_rigid_geometry'));
});

test('createPerfTracker records per-mode percentiles, drops, HUD counts, anchor steps and resets cleanly',()=>{
  const tracker=createPerfTracker({capacity:64});
  // Empty snapshot check
  let snap=tracker.snapshot();
  assert.equal(snap.frameTiming.overall.frames,0);
  assert.equal(snap.frameTiming.measure.dtMs,null);
  assert.equal(snap.anchorStability.stepSamples,0);
  assert.equal(snap.anchorStability.stepMm,null);

  // Measure frames: 10 fast frames + 2 slow frames (>14 ms, >20 ms)
  for(let i=0;i<10;i++)tracker.recordFrame('measure',11.1,1.8,0.6);
  tracker.recordFrame('measure',16.5,4.2,1.2);
  tracker.recordFrame('measure',24.0,6.5,2.0);
  // First frame with null dt should still count frames & cpuMs
  tracker.recordFrame('calibrating',null,3.0,1.0);
  tracker.recordFrame('scan',38.0,9.5,2.5);

  tracker.recordHudFrame(true);
  tracker.recordHudFrame(false);
  tracker.recordHudFrame(true);
  tracker.recordHudUpload('top',1.4);
  tracker.recordHudUpload('left',5.8);
  tracker.recordHudUpload('right',3.2);

  tracker.recordAnchorStep(0.15,0.02,0.4,0.8);
  tracker.recordAnchorStep(1.25,0.18,1.5,1.1);

  snap=tracker.snapshot();
  assert.equal(snap.frameTiming.overall.frames,14);
  assert.equal(snap.frameTiming.overall.dropped14ms,3);
  assert.equal(snap.frameTiming.overall.dropped20ms,2);
  assert.equal(snap.frameTiming.overall.dropped33ms,1);
  assert.equal(snap.frameTiming.overall.dtMs.max,38);
  assert.equal(snap.frameTiming.measure.frames,12);
  assert.equal(snap.frameTiming.measure.dropped14ms,2);
  assert.equal(snap.frameTiming.measure.dropped20ms,1);
  assert.equal(snap.frameTiming.measure.dropped33ms,0);
  assert.equal(snap.frameTiming.measure.dtMs.p50,11.1);
  assert.equal(snap.frameTiming.measure.dtMs.max,24);
  assert.equal(snap.frameTiming.scan.frames,1);
  assert.equal(snap.frameTiming.scan.dropped33ms,1);
  assert.equal(snap.frameTiming.calibrating.frames,1);
  assert.equal(snap.frameTiming.calibrating.dtMs,null);
  assert.equal(snap.frameTiming.calibrating.cpuMs.p50,3);

  assert.equal(snap.hud.visibleFrames,2);
  assert.equal(snap.hud.uploadsTop,1);
  assert.equal(snap.hud.uploadsLeft,1);
  assert.equal(snap.hud.uploadsRight,1);
  assert.equal(snap.hud.maxHudDrawMs,5.8);

  assert.equal(snap.anchorStability.stepSamples,2);
  assert.equal(snap.anchorStability.stepMm.max,1.25);
  assert.equal(snap.anchorStability.maxStepDeg,0.18);
  assert.equal(snap.anchorStability.maxJitterMm,1.5);
  assert.equal(snap.anchorStability.maxDriftMm,1.1);

  tracker.reset();
  snap=tracker.snapshot();
  assert.equal(snap.frameTiming.overall.frames,0);
  assert.equal(snap.hud.visibleFrames,0);
  assert.equal(snap.hud.uploadsLeft,0);
  assert.equal(snap.anchorStability.stepSamples,0);
});

test('createAdaptiveQualityController transitions smoothly fluid -> refined -> crisp when still and drops immediately on movement',()=>{
  const aq=createAdaptiveQualityController({mode:'auto'});
  assert.equal(aq.getMode(),'auto');

  // Moving at t=0 -> fluid
  let s=aq.update({angDegPerSec:22,linMPerSec:0.15,nowMs:0});
  assert.equal(s.tier,'fluid');
  assert.equal(s.scanDecimate,2);
  assert.equal(s.scanMaxHz,12);
  assert.equal(s.detectMode,'two-stage');
  assert.equal(s.allowRefine,false);

  // Hold steady from t=100..500 -> still fluid until >=450ms steady
  s=aq.update({angDegPerSec:2,linMPerSec:0.01,nowMs:100});
  assert.equal(s.tier,'fluid');
  s=aq.update({angDegPerSec:2,linMPerSec:0.01,nowMs:500});
  assert.equal(s.tier,'fluid');

  // At t=560 (elapsed=460ms >= 450ms) -> refined
  s=aq.update({angDegPerSec:2,linMPerSec:0.01,nowMs:560});
  assert.equal(s.tier,'refined');
  assert.equal(s.changed,true);
  assert.equal(s.scanDecimate,1.5);
  assert.equal(s.scanMaxHz,10);
  assert.equal(s.detectMode,'two-stage');
  assert.equal(s.foveation,0.45);
  assert.equal(s.allowRefine,true);

  // At t=1320 (elapsed=1220ms >= 1200ms) -> auto crisp caps foveation at 0.35 to preserve 90 FPS
  s=aq.update({angDegPerSec:1.5,linMPerSec:0.01,nowMs:1320});
  assert.equal(s.tier,'crisp');
  assert.equal(s.changed,true);
  assert.equal(s.foveation,0.35);
  assert.equal(s.scanDecimate,1.5);
  assert.equal(s.scanMaxHz,8);
  assert.equal(s.detectMode,'two-stage');
  assert.equal(s.allowRefine,true);

  // Middle hysteresis band (9 deg/s) keeps crisp tier
  s=aq.update({angDegPerSec:9,linMPerSec:0.06,nowMs:1400});
  assert.equal(s.tier,'crisp');
  assert.equal(s.changed,false);

  // Linear or angular motion immediately drops to fluid
  s=aq.update({angDegPerSec:3,linMPerSec:0.15,nowMs:1450});
  assert.equal(s.tier,'fluid');
  assert.equal(s.changed,true);

  // Explicit mode overrides ('crisp' uses 0.0 foveation)
  aq.setMode('crisp');
  s=aq.update({angDegPerSec:60,linMPerSec:0.6,nowMs:1500});
  assert.equal(s.mode,'crisp');
  assert.equal(s.tier,'crisp');
  assert.equal(s.foveation,0);
  assert.equal(s.allowRefine,true);

  aq.setMode('fluid');
  s=aq.update({angDegPerSec:0,linMPerSec:0,nowMs:5000});
  assert.equal(s.mode,'fluid');
  assert.equal(s.tier,'fluid');
  assert.equal(s.allowRefine,false);

  assert.throws(()=>aq.setMode('bogus'),/Invalid adaptive quality mode/);
  const st=aq.stats();
  assert.ok(st.transitions>=3);
  assert.ok(st.tierFrames.fluid>=4&&st.tierFrames.refined>=1&&st.tierFrames.crisp>=2);
  aq.reset();
  assert.equal(aq.stats().transitions,0);
});

test('perimeterFence.version tracks mutations and stays constant during idle MEASURE frames',()=>{
  const fence=createPerimeterFence(rig);
  const v0=fence.version();
  fence.observe(353,vectorRigid([0,0,0,0.02,0.45,0]),[0.1,-0.2,0.6],1000);
  const v1=fence.version();
  assert.ok(v1>v0);

  // First step flushes dirty flag without incrementing version (new marker starts converged).
  assert.equal(fence.step(0.011),true);
  const vIdle=fence.version();
  assert.equal(vIdle,v1);
  // Subsequent stationary steps return false and keep version unchanged.
  assert.equal(fence.step(0.011),false);
  assert.equal(fence.version(),vIdle);

  // Activating and resetting mutate version.
  assert.equal(fence.activate(),true);
  assert.ok(fence.version()>vIdle);
  const vActive=fence.version();
  fence.reset();
  assert.ok(fence.version()>vActive);
});
