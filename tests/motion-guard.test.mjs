import test from 'node:test';
import assert from 'node:assert/strict';
import {
  viewerVelocityAt,
  resolveCameraInL,
  evaluateMotionGate,
  evaluateCarriedJumpGuard,
  CARRIED_MOTION_GATE_DEFAULTS,
  ANCHOR_CORRECTION_DEFAULTS,
  landmarkSpan2D,
  solveLandmarkConsensus,
  createDampedAnchorCorrection,
} from '../src/motion-guard.mjs';
import {cvToGL,inverse,product,rigidVector,vectorRigid} from '../src/runtime.mjs';
import {tagCornersOnZ0} from '../src/perimeter-fence.mjs';
import {distortNormalized} from '../src/rig.mjs';

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

  // Disabling the optical bridge keeps camera-in-L strictly locked to the native XR anchor (no 0..4 cm jump when moving the L-target).
  const fixedOnly=resolveCameraInL(goodObs,assoc,{allowOpticalBridge:false});
  assert.equal(fixedOnly.source,'xr-history');
  assert.ok(Math.abs(fixedOnly.lFromCamera[12]-0.125)<1e-6);
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

  // Following a rolling object with a natural 60-65 deg/s head pan must stay above the 0.30 optical drop floor.
  const panFollow=evaluateMotionGate({angDegPerSec:65,linMPerSec:0.15},{source:'xr-history'},CARRIED_MOTION_GATE_DEFAULTS);
  assert.equal(panFollow.frozen,false);
  assert.ok(panFollow.confidence>=0.35,`expected pan-follow confidence >= 0.35, got ${panFollow.confidence}`);

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

  // Fast 1.30 m/s optical step (130 mm in 100 ms) also accepts directly without stalling above 500 mm/s.
  const pSprint=vectorRigid([0,0,0,0.28,0,0]);
  const gSprint=evaluateCarriedJumpGuard({prevMatrix:pFast,nextMatrix:pSprint,dtSinceLastMs:100,confidence:0.85,atMs:1250});
  assert.equal(gSprint.accept,true);
  assert.equal(gSprint.nextPendingJump,null);

  // Single-frame 470 mm PnP spike in 100 ms (> 220 mm guard) is rejected on frame 1 and sets pendingJump.
  const pSpike=vectorRigid([0,0,0,0.62,0,0]);
  const gSpike=evaluateCarriedJumpGuard({prevMatrix:pFast,nextMatrix:pSpike,dtSinceLastMs:100,confidence:0.85,atMs:1300});
  assert.equal(gSpike.accept,false);
  assert.equal(gSpike.reason,'jump');
  assert.ok(gSpike.nextPendingJump);

  // If frame 2 returns to normal trajectory, pendingJump clears; if frame 2 continues at spike+60 mm in 100 ms, it confirms.
  const pSpikeCont=vectorRigid([0,0,0,0.68,0,0]);
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

function lookAtGL(camPos,targetPos){
  const fx=targetPos[0]-camPos[0],fy=targetPos[1]-camPos[1],fz=targetPos[2]-camPos[2];
  const fl=Math.hypot(fx,fy,fz);
  const fwd=[fx/fl,fy/fl,fz/fl],zc=[-fwd[0],-fwd[1],-fwd[2]];
  let xx=fwd[1],xy=-fwd[0],xl=Math.hypot(xx,xy);
  if(xl<1e-6){xx=1;xy=0;xl=1;}
  const xc=[xx/xl,xy/xl,0];
  const yc=[zc[1]*xc[2]-zc[2]*xc[1],zc[2]*xc[0]-zc[0]*xc[2],zc[0]*xc[1]-zc[1]*xc[0]];
  return [xc[0],xc[1],xc[2],0,yc[0],yc[1],yc[2],0,zc[0],zc[1],zc[2],0,camPos[0],camPos[1],camPos[2],1];
}

function projectTagFromGL(corners2D,lFromCamera,K){
  const C=inverse(lFromCamera);
  return corners2D.map(([x,y])=>{
    const gx=C[12]+C[0]*x+C[4]*y,gy=C[13]+C[1]*x+C[5]*y,gz=C[14]+C[2]*x+C[6]*y;
    const [dx,dy]=distortNormalized(gx/-gz,-gy/-gz,K.k1??0);
    return {x:K.fx*dx+K.cx,y:K.fy*dy+K.cy};
  });
}

test('landmarkSpan2D and solveLandmarkConsensus localize over wide perimeter baseline and isolate bumped outlier markers',()=>{
  const K={width:1280,height:1080,fx:860,fy:860,cx:640,cy:540,k1:-0.03};
  // Collinear X-axis points yield ~0 minor span, 2D triangle yields >= 0.30 m minor span.
  const colTags=[0,0.4,0.8].map((x,i)=>({id:353+i,points:tagCornersOnZ0(x,0,0,0.02)}));
  assert.ok(landmarkSpan2D(colTags).minorM<1e-4);

  const lFromCam=lookAtGL([0.35,0.20,0.95],[0.35,0.65,0]);
  const wideDefs=[
    {id:350,x:0.0,y:0.0,yaw:0},
    {id:351,x:0.185,y:0.0,yaw:0},
    {id:358,x:0.0,y:0.75,yaw:0.2},
    {id:359,x:0.70,y:0.75,yaw:-0.1},
  ];
  const wideTags=wideDefs.map(d=>{
    const pts=tagCornersOnZ0(d.x,d.y,d.yaw,0.02);
    return {id:d.id,points:pts,pixels:projectTagFromGL(pts,lFromCam,K)};
  });
  assert.ok(landmarkSpan2D(wideTags).minorM>0.30);

  // Rejects pure collinear subset of only 2 tags.
  assert.equal(solveLandmarkConsensus(wideTags.slice(0,2),K),null);

  // Clean 4-tag wide consensus recovers exact camera pose.
  const con=solveLandmarkConsensus(wideTags,K,{fixedIds:[350,351]});
  assert.ok(con);
  assert.equal(con.inliers.length,4);
  assert.equal(con.outliers.length,0);
  const solvedLFromCam=inverse(cvToGL(con.pose));
  const poseErr=rigidVector(product(inverse(lFromCam),solvedLFromCam));
  assert.ok(Math.hypot(...poseErr.slice(3))*1000<0.1);

  // Bump marker 359 by 3 cm -> consensus keeps [350, 351, 358] and isolates 359 as an outlier!
  const bumpedPts=tagCornersOnZ0(0.73,0.75,-0.1,0.02);
  const bumpedTags=wideTags.map(t=>t.id===359?{id:359,points:t.points,pixels:projectTagFromGL(bumpedPts,lFromCam,K)}:t);
  const bumpCon=solveLandmarkConsensus(bumpedTags,K,{fixedIds:[350,351]});
  assert.ok(bumpCon);
  assert.equal(bumpCon.inliers.length,3);
  assert.equal(bumpCon.outliers.length,1);
  assert.equal(bumpCon.outliers[0].id,359);
  assert.ok(bumpCon.outliers[0].rmsPx>5);
});

test('resolveCameraInL uses converged perimeter landmarks optically, guards divergence vs raw XRAnchor, and applies anchorCorrection on XR fallback',()=>{
  const K={width:1280,height:1080,fx:860,fy:860,cx:640,cy:540,k1:-0.02};
  const lFromCam=lookAtGL([0.35,-0.15,1.00],[0.35,0.40,0]);
  const worldFromAnchor=vectorRigid([0.05,-0.02,0.01,0.1,-0.2,0.4]);
  const xrDrift=vectorRigid([0,0,0,0.008,-0.004,0.005]); // ~10 mm XRAnchor drift in L-frame
  const worldFromCamera=product(worldFromAnchor,xrDrift,lFromCam);

  const lms=[
    {id:353,role:'TL',x:0.05,y:0.70,yaw:0,points:tagCornersOnZ0(0.05,0.70,0,0.02)},
    {id:354,role:'TR',x:0.65,y:0.70,yaw:0,points:tagCornersOnZ0(0.65,0.70,0,0.02)},
    {id:355,role:'BR',x:0.65,y:0.05,yaw:0,points:tagCornersOnZ0(0.65,0.05,0,0.02)},
  ];
  const obs={
    cameraModel:K,
    tags:[],
    tools:[],
    unassignedDetections:lms.map(lm=>({id:lm.id,corners:projectTagFromGL(lm.points,lFromCam,K)})),
  };

  const resPerim=resolveCameraInL(obs,{worldFromAnchor,worldFromCamera},{
    allowOpticalBridge:true,
    perimeterLandmarks:lms,
  });
  assert.equal(resPerim.source,'optical-perimeter');
  assert.equal(resPerim.perimeterTagCount,3);
  const errMm=Math.hypot(...rigidVector(product(inverse(lFromCam),resPerim.lFromCamera)).slice(3))*1000;
  assert.ok(errMm<0.1);

  // If raw XRAnchor diverges > 4 cm, even with non-identity anchorCorrection, bridge must NOT bypass maxDivergenceM.
  const farDriftWorldCam=product(worldFromAnchor,vectorRigid([0,0,0,0.07,0,0]),lFromCam);
  const corr7cm=vectorRigid([0,0,0,0.07,0,0]);
  const resBlocked=resolveCameraInL(obs,{worldFromAnchor,worldFromCamera:farDriftWorldCam},{
    allowOpticalBridge:true,
    perimeterLandmarks:lms,
    anchorCorrection:corr7cm,
  });
  assert.equal(resBlocked.source,'xr-history');

  // When falling back to xr-history, anchorCorrection cleanly aligns returned lFromCamera.
  const resCorr=resolveCameraInL({cameraModel:K,tags:[],tools:[],unassignedDetections:[]},{worldFromAnchor,worldFromCamera},{
    allowOpticalBridge:true,
    perimeterLandmarks:[],
    anchorCorrection:vectorRigid([0,0,0,0.008,-0.004,0.005]),
  });
  assert.equal(resCorr.source,'xr-history');
  const xrErrMm=Math.hypot(...rigidVector(product(inverse(lFromCam),resCorr.lFromCamera)).slice(3))*1000;
  assert.ok(xrErrMm<0.5);

  // evaluateMotionGate gives optical-perimeter the same relaxed blur thresholds as optical-l.
  const mg=evaluateMotionGate({angDegPerSec:40,linMPerSec:0.05},{source:'optical-perimeter'});
  assert.equal(mg.frozen,false);
  assert.ok(mg.confidence>0.4);
});

test('createDampedAnchorCorrection filters deadband, enforces caps, rate-limits smoothly without snapping, and resets cleanly',()=>{
  const ctrl=createDampedAnchorCorrection();
  const lFromCam=lookAtGL([0.3,0.3,0.8],[0.3,0.7,0]);

  // Sub-deadband (0.2 mm) should not advance target.
  const tinyXr=product(lFromCam,vectorRigid([0,0,0,0.0002,0,0]));
  assert.equal(ctrl.observeOptical({lFromCameraXrRaw:tinyXr,lFromCameraOpt:lFromCam,confidence:1}).updates,0);

  // Exceeding cap (> maxPosM = 35 mm) must be ignored.
  const hugeXr=product(lFromCam,vectorRigid([0,0,0,0.060,0,0]));
  assert.equal(ctrl.observeOptical({lFromCameraXrRaw:hugeXr,lFromCameraOpt:lFromCam,confidence:1}).updates,0);

  // Real 15 mm drift updates target smoothly (converging within 0.5 mm deadband in ~8 updates) and rate-limits visual step().
  const xr15=product(vectorRigid([0,0,0,0.015,0,0]),lFromCam);
  for(let i=0;i<15;i++)ctrl.observeOptical({lFromCameraXrRaw:xr15,lFromCameraOpt:lFromCam,confidence:1});
  assert.ok(ctrl.state().updates>=6&&ctrl.state().updates<=12,`unexpected updates=${ctrl.state().updates}`);
  const s1=ctrl.step(0.1);
  // Bounded by maxSpeedMps (15 mm/s) * 0.1 s = 1.5 mm in a single frame.
  assert.ok(s1.active&&s1.posMm<=ANCHOR_CORRECTION_DEFAULTS.maxSpeedMps*1000*0.1+1e-6);

  for(let i=0;i<80;i++)ctrl.step(0.05);
  assert.ok(ctrl.state().posMm>12&&ctrl.state().posMm<=16);

  ctrl.reset();
  assert.equal(ctrl.state().active,false);
  assert.equal(ctrl.state().updates,0);
});
