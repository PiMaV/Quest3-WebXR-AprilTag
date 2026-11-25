import test from 'node:test';
import assert from 'node:assert/strict';
import {
  viewerVelocityAt,
  resolveCameraInL,
  evaluateMotionGate,
  evaluateCarriedJumpGuard,
  CARRIED_MOTION_GATE_DEFAULTS,
} from '../src/motion-guard.mjs';
import {cvToGL,inverse,product,vectorRigid} from '../src/runtime.mjs';

function makeHistory(steps,{sessionId='s1',spaceEpoch=0}={}){
  return steps.map(([t,rx,ry,rz,tx,ty,tz])=>({
    sessionId,
    spaceEpoch,
    xrRafTimeMs:t,
    emulatedPosition:false,
    worldFromViewer:vectorRigid([rx,ry,rz,tx,ty,tz]),
  }));
}

function glToCvPose(gl){
  // Inverse of cvToGL: columns 1 and 2 negated, translation y/z negated.
  return {
    R:[
      [gl[0],-gl[1],-gl[2]],
      [gl[4],-gl[5],-gl[6]],
      [gl[8],-gl[9],-gl[10]],
    ],
    t:[gl[12],-gl[13],-gl[14]],
  };
}

test('viewerVelocityAt measures stationary, rotating, and translating head motion accurately',()=>{
  const ctx={sessionId:'s1',spaceEpoch:0};
  const still=makeHistory([0,20,40,60,80,100].map(t=>[t,0,0,0,0.2,1.5,-0.3]));
  const vStill=viewerVelocityAt(still,50,ctx);
  assert.ok(vStill);
  assert.ok(vStill.angDegPerSec<1e-6);
  assert.ok(vStill.linMPerSec<1e-6);

  // Pure yaw rotation at 60 deg/s = pi/3 rad/s.
  const radPerMs=(Math.PI/3)/1000;
  const rot=makeHistory([0,15,30,45,60,75,90,105,120].map(t=>[t,0,t*radPerMs,0,0,1.5,0]));
  const vRot=viewerVelocityAt(rot,60,ctx);
  assert.ok(vRot);
  assert.ok(Math.abs(vRot.angDegPerSec-60)<0.5,`expected ~60 deg/s, got ${vRot.angDegPerSec}`);
  assert.ok(vRot.linMPerSec<1e-6);

  // Pure linear translation at 0.5 m/s along X.
  const trans=makeHistory([0,15,30,45,60,75,90,105,120].map(t=>[t,0,0,0,t*0.0005,1.5,0]));
  const vTrans=viewerVelocityAt(trans,60,ctx);
  assert.ok(vTrans);
  assert.ok(vTrans.angDegPerSec<1e-6);
  assert.ok(Math.abs(vTrans.linMPerSec-0.5)<1e-3,`expected ~0.5 m/s, got ${vTrans.linMPerSec}`);

  // Symmetric envelope catches a sudden head turn starting right at targetTimeMs.
  const sudden=makeHistory([
    [0,0,0,0,0,0,0],
    [25,0,0,0,0,0,0],
    [50,0,0,0,0,0,0],
    [75,0,0.035,0,0,0,0], // ~80 deg/s in [50..75]
    [100,0,0.070,0,0,0,0],
  ]);
  const vSudden=viewerVelocityAt(sudden,50,ctx);
  assert.ok(vSudden&&vSudden.angDegPerSec>75,`expected symmetric envelope to catch turn, got ${vSudden?.angDegPerSec}`);

  // Ignores entries from another spaceEpoch.
  assert.equal(viewerVelocityAt(rot,60,{sessionId:'s1',spaceEpoch:1}),null);
});

test('resolveCameraInL prefers clean same-frame optical L-pose over lagged XR estimate and guards divergence',()=>{
  const worldFromAnchor=vectorRigid([0,0,0,0,0,0]);
  const trueLFromCam=vectorRigid([0.2,-0.1,0,0.1,0.05,0.55]);
  const camGLFromL=inverse(trueLFromCam);
  // Simulated 25 mm XR lag offset on the camera pose.
  const laggedWorldFromCam=vectorRigid([0.2,-0.1,0,0.125,0.05,0.55]);
  const assoc={worldFromAnchor,worldFromCamera:laggedWorldFromCam};

  const goodObs={
    tools:[{
      id:'origin-l',
      mode:'joint-rigid-tags',
      tagIds:[350,351,352],
      reprojectionRmsPx:0.65,
      cameraFromTool:glToCvPose(camGLFromL),
    }],
  };
  const resolved=resolveCameraInL(goodObs,assoc);
  assert.equal(resolved.source,'optical-l');
  assert.equal(resolved.opticalTagCount,3);
  assert.ok(Math.abs(resolved.xrDeltaMm-25)<0.1);
  // Returned pose must match the zero-lag optical pose, not the 25 mm lagged XR pose.
  assert.ok(Math.abs(resolved.lFromCamera[12]-0.1)<1e-6);

  // Single-tag or 2-tag collinear L observation or noisy RMS (>1.0 px) falls back to XR history.
  const noisyObs={tools:[{...goodObs.tools[0],reprojectionRmsPx:1.3}]};
  assert.equal(resolveCameraInL(noisyObs,assoc).source,'xr-history');

  const twoTagObs={tools:[{...goodObs.tools[0],tagIds:[350,351]}]};
  assert.equal(resolveCameraInL(twoTagObs,assoc).source,'xr-history');

  const singleTagObs={tools:[{...goodObs.tools[0],mode:'single-tag',tagIds:[350]}]};
  assert.equal(resolveCameraInL(singleTagObs,assoc).source,'xr-history');

  // If the optical L-pose diverges >4 cm from the native XR anchor (e.g. shifted 6 cm), fall back safely.
  const movedCamGLFromL=inverse(vectorRigid([0.2,-0.1,0,0.185,0.05,0.55]));
  const movedObs={tools:[{...goodObs.tools[0],cameraFromTool:glToCvPose(movedCamGLFromL)}]};
  assert.equal(resolveCameraInL(movedObs,assoc).source,'xr-history');
});

test('evaluateMotionGate freezes fast head turns, damps moderate turns, and grants wider envelope to optical-l',()=>{
  const steadyXr=evaluateMotionGate({angDegPerSec:5,linMPerSec:0.04},{source:'xr-history'});
  assert.equal(steadyXr.frozen,false);
  assert.equal(steadyXr.confidence,1);
  assert.equal(steadyXr.maxTagRmsPx,1.5);

  const midXr=evaluateMotionGate({angDegPerSec:21,linMPerSec:0.05},{source:'xr-history'});
  assert.equal(midXr.frozen,false);
  assert.ok(midXr.confidence>0.1&&midXr.confidence<0.5);
  assert.ok(midXr.maxTagRmsPx<1.3);

  const fastXr=evaluateMotionGate({angDegPerSec:40,linMPerSec:0.05},{source:'xr-history'});
  assert.equal(fastXr.frozen,true);
  assert.equal(fastXr.confidence,0);

  // At 40 deg/s, same-frame optical-l (0 ms lag) stays active while XR-history is frozen.
  const midOpt=evaluateMotionGate({angDegPerSec:40,linMPerSec:0.05},{source:'optical-l'});
  assert.equal(midOpt.frozen,false);
  assert.ok(midOpt.confidence>0.4);

  // Extreme blur speed (90 deg/s) freezes even optical-l.
  const extremeOpt=evaluateMotionGate({angDegPerSec:90,linMPerSec:0.05},{source:'optical-l'});
  assert.equal(extremeOpt.frozen,true);
  assert.equal(extremeOpt.confidence,0);
});

test('CARRIED_MOTION_GATE_DEFAULTS keeps walking handheld tags active while freezing extreme whips',()=>{
  const walk=evaluateMotionGate({angDegPerSec:15,linMPerSec:0.60},{source:'xr-history'},CARRIED_MOTION_GATE_DEFAULTS);
  assert.equal(walk.frozen,false);
  assert.ok(walk.confidence>0.75,`expected carried walk confidence > 0.75, got ${walk.confidence}`);

  const whip=evaluateMotionGate({angDegPerSec:110,linMPerSec:0.40},{source:'xr-history'},CARRIED_MOTION_GATE_DEFAULTS);
  assert.equal(whip.frozen,true);
  assert.equal(whip.confidence,0);
});

test('evaluateCarriedJumpGuard accepts continuous walking without deadlock while blocking spikes and mirror flips',()=>{
  const p0=vectorRigid([0,0,0,0,0,0]);
  // 0.60 m/s walk at 10 Hz = 60 mm/frame
  const p1=vectorRigid([0,0,0,0.06,0,0]);
  const gWalk1=evaluateCarriedJumpGuard({prevMatrix:p0,nextMatrix:p1,dtSinceLastMs:100,confidence:0.85,atMs:1100});
  assert.equal(gWalk1.accept,true);
  assert.equal(gWalk1.nextPendingJump,null);

  // Even lower-confidence faster 0.90 m/s step (90 mm in 100 ms) accepts without stalling.
  const pFast=vectorRigid([0,0,0,0.15,0,0]);
  const gWalk2=evaluateCarriedJumpGuard({prevMatrix:p1,nextMatrix:pFast,dtSinceLastMs:100,confidence:0.50,atMs:1200});
  assert.equal(gWalk2.accept,true);

  // Single-frame 300 mm PnP spike in 100 ms is rejected on frame 1 and sets pendingJump.
  const pSpike=vectorRigid([0,0,0,0.45,0,0]);
  const gSpike=evaluateCarriedJumpGuard({prevMatrix:pFast,nextMatrix:pSpike,dtSinceLastMs:100,confidence:0.85,atMs:1300});
  assert.equal(gSpike.accept,false);
  assert.equal(gSpike.reason,'jump');
  assert.ok(gSpike.nextPendingJump);

  // If frame 2 returns to normal trajectory, pendingJump clears; if frame 2 continues at spike+60 mm in 100 ms, it confirms.
  const pSpikeCont=vectorRigid([0,0,0,0.51,0,0]);
  const gConfirm=evaluateCarriedJumpGuard({
    prevMatrix:pFast,
    nextMatrix:pSpikeCont,
    dtSinceLastMs:200,
    confidence:0.85,
    pendingJump:gSpike.nextPendingJump,
    atMs:1400,
  });
  assert.equal(gConfirm.accept,true);
  assert.equal(gConfirm.reason,'confirmed');
  assert.equal(gConfirm.nextPendingJump,null);

  // Single-frame 25 deg IPPE mirror flip at small position change (< 22 mm) is rejected on frame 1.
  const pFlip=vectorRigid([0,25*Math.PI/180,0,0.155,0,0]);
  const gFlip=evaluateCarriedJumpGuard({prevMatrix:pFast,nextMatrix:pFlip,dtSinceLastMs:100,confidence:0.9,atMs:1500});
  assert.equal(gFlip.accept,false);
  assert.equal(gFlip.reason,'mirror-flip');

  // After long occlusion gap (>= 500 ms), returning pose is accepted immediately without stale-delta rejection.
  const gReappear=evaluateCarriedJumpGuard({prevMatrix:p0,nextMatrix:vectorRigid([0,0,0,0.80,0,0]),dtSinceLastMs:900,confidence:0.8,atMs:2000});
  assert.equal(gReappear.accept,true);
});
