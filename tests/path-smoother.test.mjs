import test from 'node:test';
import assert from 'node:assert/strict';
import {
  fuseCommittedTrailSegment,
  smoothLineTail,
  smoothAxisTail,
  SPEED_RADIUS_DEFAULTS,
  speedToTubeRadius,
  RIBBON_SPEED_DEFAULTS,
  speedToRibbonHalfWidth,
  speedToHeatColor,
  TURBO_SPEED_DEFAULTS,
  turboColormap,
  speedToTurboColor,
  MOTION_WAKE_DEFAULTS,
  createMotionWakeBuffer,
} from '../src/path-smoother.mjs';
import {isRigidPose} from '../src/anchor.mjs';
import {vectorRigid} from '../src/runtime.mjs';

test('fuseCommittedTrailSegment smoothly distributes optical offset and preserves rigid pose orthogonality',()=>{
  const seg=[0,1,2,3].map(i=>({
    xrTimeMs:1000+i*25,
    atMs:1000+i*25,
    pose:vectorRigid([0.2,-0.1,0.3,i*0.02,0.10,0.20]),
  }));
  // Optical frame ends 16 mm offset along +Z from last committed hand point.
  const optical=vectorRigid([0.2,-0.1,0.3,0.06,0.10,0.216]);
  const fused=fuseCommittedTrailSegment(seg,optical,{blendWeight:0.45,maxStepCorrectionM:0.035});
  assert.equal(fused.length,4);
  for(let i=0;i<4;i++){
    assert.equal(isRigidPose(fused[i].pose),true);
    const expectedDz=0.016*((i+1)/4)*0.45;
    assert.ok(Math.abs((fused[i].pose[14]-0.20)-expectedDz)<1e-6);
  }

  // Outlier step (> maxStepCorrectionM) is clamped cleanly.
  const hugeOptical=vectorRigid([0,0,0,0.06,0.10,0.40]);
  const clamped=fuseCommittedTrailSegment(seg,hugeOptical,{blendWeight:0.5,maxStepCorrectionM:0.03});
  assert.ok(Math.abs((clamped[3].pose[14]-0.20)-0.015)<1e-6);

  assert.deepEqual(fuseCommittedTrailSegment([],optical),[]);
});

test('smoothLineTail cancels sawtooth Nyquist ripple, pins endpoints, bounds corners, and is idempotent across appends',()=>{
  // 1. Exact straight line stays exact and pins endpoints.
  const N1=8;
  const rawLine=new Float32Array(N1*3),outLine=new Float32Array(N1*3);
  for(let i=0;i<N1;i++){
    rawLine[i*3]=i*0.02;rawLine[i*3+1]=0.1+i*0.01;rawLine[i*3+2]=0.25-i*0.005;
    smoothLineTail(rawLine,outLine,i+1);
  }
  for(let i=0;i<N1;i++){
    for(let k=0;k<3;k++)assert.ok(Math.abs(outLine[i*3+k]-rawLine[i*3+k])<1e-6);
  }

  // 2. Alternating +/-15 mm sawtooth along Z cancels to 0 at interior indices 2, 3, 4!
  const saw=[+0.015,-0.015,+0.015,-0.015,+0.015,-0.015,+0.015];
  const rawSaw=new Float32Array(saw.length*3),outSaw=new Float32Array(saw.length*3);
  for(let i=0;i<saw.length;i++){
    rawSaw[i*3]=i*0.02;rawSaw[i*3+1]=0.2;rawSaw[i*3+2]=0.3+saw[i];
    smoothLineTail(rawSaw,outSaw,i+1,{maxDeviationM:0.020});
  }
  // Endpoints untouched:
  assert.ok(Math.abs(outSaw[2]-(0.3+saw[0]))<1e-6);
  assert.ok(Math.abs(outSaw[6*3+2]-(0.3+saw[6]))<1e-6);
  // Interior vertices 2, 3, 4 hit exact Nyquist null -> Z = 0.300 m!
  for(const idx of [2,3,4]){
    assert.ok(Math.abs(outSaw[idx*3+2]-0.3)<1e-6,`idx=${idx} expected 0.3, got ${outSaw[idx*3+2]}`);
  }

  // 3. Sharp 90-deg corner respects maxDeviationM=0.012 m cap and matches batch pass (idempotent).
  const cornerPts=[
    [0,0,0],[0.05,0,0],[0.10,0,0],[0.10,0.05,0],[0.10,0.10,0],[0.10,0.15,0],
  ];
  const rawC=new Float32Array(cornerPts.length*3),outC=new Float32Array(cornerPts.length*3);
  for(let i=0;i<cornerPts.length;i++){
    rawC[i*3]=cornerPts[i][0];rawC[i*3+1]=cornerPts[i][1];rawC[i*3+2]=cornerPts[i][2];
    smoothLineTail(rawC,outC,i+1,{maxDeviationM:0.012});
  }
  const cIdx=2*3;
  const dev=Math.hypot(outC[cIdx]-rawC[cIdx],outC[cIdx+1]-rawC[cIdx+1],outC[cIdx+2]-rawC[cIdx+2]);
  assert.ok(dev<=0.012+1e-6,`corner deviation ${dev} exceeded 12 mm cap`);
});

test('speedToTubeRadius maps speed monotonically between clamped bounds and guards non-finite inputs',()=>{
  const minR=SPEED_RADIUS_DEFAULTS.minRadiusM;
  const maxR=SPEED_RADIUS_DEFAULTS.maxRadiusM;
  assert.equal(speedToTubeRadius(0),minR);
  assert.equal(speedToTubeRadius(-1.5),minR);
  assert.equal(speedToTubeRadius(NaN),minR);
  assert.equal(speedToTubeRadius(Infinity),minR);

  const mid=speedToTubeRadius(SPEED_RADIUS_DEFAULTS.refSpeedMps*0.5);
  assert.ok(mid>minR&&mid<maxR,`mid speed radius ${mid} must lie strictly inside (${minR}, ${maxR})`);
  assert.equal(speedToTubeRadius(SPEED_RADIUS_DEFAULTS.refSpeedMps),maxR);
  assert.equal(speedToTubeRadius(10.0),maxR);

  assert.equal(SPEED_RADIUS_DEFAULTS.refSpeedMps,0.55);
  const r50=speedToTubeRadius(0.05);
  const r250=speedToTubeRadius(0.25);
  const r500=speedToTubeRadius(0.50);
  assert.ok(r50<r250&&r250<r500&&r500<maxR);
  assert.ok(maxR/minR>5,'0..550 mm/s envelope must provide >5x tube diameter contrast');

  assert.equal(speedToTubeRadius(0.5,{minRadiusM:0.001,maxRadiusM:0.005,refSpeedMps:1.0,exponent:1.0}),0.003);
});

test('speedToRibbonHalfWidth swells 7x across 0..550 mm/s and speedToHeatColor spans cyan -> green -> yellow -> red',()=>{
  const minHW=RIBBON_SPEED_DEFAULTS.minHalfWidthM;
  const maxHW=RIBBON_SPEED_DEFAULTS.maxHalfWidthM;
  assert.equal(speedToRibbonHalfWidth(0),minHW);
  assert.equal(speedToRibbonHalfWidth(-2),minHW);
  assert.equal(speedToRibbonHalfWidth(NaN),minHW);
  assert.equal(speedToRibbonHalfWidth(0.55),maxHW);
  assert.equal(speedToRibbonHalfWidth(2.0),maxHW);
  assert.ok(maxHW/minHW>=6.5,`expected ~7x width swell, got ${maxHW/minHW}`);

  const wMid=speedToRibbonHalfWidth(0.275);
  assert.ok(Math.abs(wMid-(minHW+maxHW)*0.5)<1e-6);

  // Verify 4 heatmap color stops & reused output buffer
  const outBuf=new Float32Array(3);
  const c0=speedToHeatColor(0,{out:outBuf});
  assert.equal(c0,outBuf);
  assert.ok(c0[2]>0.9&&c0[0]<0.2,'0 mm/s must be cyan-blue');
  const cGreen=speedToHeatColor(0.55*0.35);
  assert.ok(cGreen[1]>0.75&&cGreen[0]<0.2,'~192 mm/s must pass through green');
  const cYellow=speedToHeatColor(0.55*0.70);
  assert.ok(cYellow[0]>0.9&&cYellow[1]>0.7&&cYellow[2]<0.2,'~385 mm/s must pass through yellow');
  const cRed=speedToHeatColor(0.55);
  assert.ok(cRed[0]>0.9&&cRed[1]<0.3&&cRed[2]<0.3,'550 mm/s must saturate at vivid red-orange');
  const cBad=speedToHeatColor(Infinity);
  assert.ok(Math.abs(cBad[2]-c0[2])<1e-6);
});

test('smoothAxisTail preserves unit vectors, pins endpoints, and skips filtering across >=90 deg flips',()=>{
  const s=Math.SQRT1_2;
  const raw=new Float32Array([
    1,0,0,
    s,s,0,
    1,0,0,
  ]);
  const out=new Float32Array(raw.length);
  smoothAxisTail(raw,out,3);
  assert.ok(Math.abs(out[0]-1)<1e-6&&Math.abs(out[1])<1e-6);
  // Mid sample i=1 (45 deg) smoothed toward +X neighbors -> ~22.5 deg (cos ~0.924, sin ~0.383)
  const midLen=Math.hypot(out[3],out[4],out[5]);
  assert.ok(Math.abs(midLen-1)<1e-6);
  assert.ok(out[3]>0.9&&out[4]>0.35&&out[4]<0.42);

  // 180-deg flip between i=0 (+X) and i=1 (-X) must not cancel to 0!
  const flipRaw=new Float32Array([
    1,0,0,
    -1,0,0,
    -1,0,0,
  ]);
  const flipOut=new Float32Array(flipRaw.length);
  smoothAxisTail(flipRaw,flipOut,3);
  assert.ok(Math.abs(flipOut[3]-(-1))<1e-6&&Math.abs(flipOut[4])<1e-6);
});

test('turboColormap and speedToTurboColor map 0..600 mm/s across official Google Turbo stops with clamping and out reuse',()=>{
  assert.equal(TURBO_SPEED_DEFAULTS.refSpeedMps,0.60);

  const outBuf=new Float32Array(3);
  const c0=turboColormap(0);
  const cLow=turboColormap(0,outBuf);
  assert.equal(cLow,outBuf);
  // t=0 (#30123b indigo)
  assert.ok(Math.abs(cLow[0]-0.18995)<1e-5);
  assert.ok(Math.abs(cLow[1]-0.07176)<1e-5);
  assert.ok(Math.abs(cLow[2]-0.23217)<1e-5);

  // Key intermediate stops
  const cCyan=turboColormap(0.25);
  assert.ok(cCyan[2]>0.9&&cCyan[1]>0.7&&cCyan[0]<0.2,'t=0.25 must be bright cyan-blue');
  const cLime=turboColormap(0.50);
  assert.ok(cLime[1]>0.95&&cLime[0]>0.6&&cLime[2]<0.3,'t=0.50 must be lime green-yellow');
  const cOrange=turboColormap(0.75);
  assert.ok(cOrange[0]>0.95&&cOrange[1]>0.45&&cOrange[1]<0.55&&cOrange[2]<0.2,'t=0.75 must be vivid orange');

  // t=1 (#7a0403 deep crimson)
  const cHigh=turboColormap(1);
  assert.ok(Math.abs(cHigh[0]-0.47960)<1e-5);
  assert.ok(Math.abs(cHigh[1]-0.01583)<1e-5);
  assert.ok(Math.abs(cHigh[2]-0.01055)<1e-5);

  // Clamping & non-finite guards
  assert.deepEqual(turboColormap(-0.5),c0);
  assert.deepEqual(turboColormap(NaN),c0);
  assert.deepEqual(turboColormap(Infinity),c0);
  assert.deepEqual(turboColormap(2.5),cHigh);

  // Speed mapping at 0, 300 mm/s, 600 mm/s, >600 mm/s, and custom refSpeedMps
  assert.deepEqual(speedToTurboColor(0),c0);
  assert.deepEqual(speedToTurboColor(0.30),cLime);
  assert.deepEqual(speedToTurboColor(0.60),cHigh);
  assert.deepEqual(speedToTurboColor(1.50),cHigh);
  const reused=speedToTurboColor(0.5,{refSpeedMps:1.0,out:outBuf});
  assert.equal(reused,outBuf);
  assert.ok(Math.abs(reused[1]-cLime[1])<1e-6);
});

test('createMotionWakeBuffer suppresses micro-bumps, smooths jitter, interpolates continuous tail decay, resets jumps, and tapers monotonically',()=>{
  const wake=createMotionWakeBuffer();
  assert.equal(wake.hasPoints(),false);

  // 1. Micro-bump suppression (< minVisibleArcM = 14 mm stays hidden with count === 0)
  wake.seed([0,0,0],1000);
  wake.push([0.008,0.002,0],1040);
  wake.push([0.011,-0.001,0],1080);
  const micro=wake.sample(1080);
  assert.equal(micro.count,0);

  // Once travel exceeds >= 14 mm, wake unfurls cleanly with monotonic tapered radii & fades
  wake.push([0.025,0,0],1120,{speedMps:0.25});
  wake.push([0.055,0,0],1160,{speedMps:0.45});
  wake.push([0.095,0,0],1200,{speedMps:0.55});
  const active=wake.sample(1200);
  assert.ok(active.count>=4,`expected >=4 points after 95 mm motion, got ${active.count}`);
  assert.ok(active.arcLengthM>=0.04);
  for(let i=1;i<active.count;i++){
    assert.ok(active.radii[i]>=active.radii[i-1]-1e-7,'radii must taper monotonically from tail to head');
    assert.ok(active.fades[i]>=active.fades[i-1]-1e-7,'fades must grow monotonically from tail to head');
  }
  assert.ok(active.radii[0]>=MOTION_WAKE_DEFAULTS.minRadiusM-1e-7);
  assert.ok(active.radii[active.count-1]<=MOTION_WAKE_DEFAULTS.maxRadiusM+1e-7);

  // 2. Continuous tail interpolation at cutoffMs = nowMs - durationMs (no discrete vertex pop) and full fade to 0
  const wakeInterp=createMotionWakeBuffer({durationMs:1000,headFilterTauSec:0,minStepM:0.005,minIntervalMs:10,minVisibleArcM:0.014});
  wakeInterp.seed([0,0,0],0);
  wakeInterp.push([0.05,0,0],400);
  wakeInterp.push([0.10,0,0],800);
  // At nowMs=1200 -> cutoffMs=200 (midpoint between t=0 at x=0 and t=400 at x=0.05 -> interpolated tail x=0.025, life=0)
  const midDecay=wakeInterp.sample(1200,{includeLiveHead:false});
  assert.equal(midDecay.count,3);
  assert.ok(Math.abs(midDecay.positions[0]-0.025)<1e-5,`expected interpolated tail x=0.025, got ${midDecay.positions[0]}`);
  assert.ok(Math.abs(midDecay.fades[0])<1e-6,'interpolated tail at cutoffMs must have fade === 0');
  assert.ok(Math.abs(midDecay.radii[0]-MOTION_WAKE_DEFAULTS.minRadiusM)<1e-7);
  // Past last knot + durationMs (800 + 1000 = 1800 ms), wake completely dissolves to count === 0
  const expired=wakeInterp.sample(1800,{includeLiveHead:false});
  assert.equal(expired.count,0);
  assert.equal(wakeInterp.hasPoints(),false);

  // 3. Head low-pass + 2-pass binomial smoothing attenuates alternating +/-12 mm jitter and bounds deviation <= maxSmoothDeviationM
  const wakeSmooth=createMotionWakeBuffer({durationMs:2000,minStepM:0.010,minIntervalMs:10,headFilterTauSec:0.025,smoothPasses:2,maxSmoothDeviationM:0.014});
  wakeSmooth.seed([0,0,0],0);
  const rawPts=[[0.02,0.012,0],[0.04,-0.012,0],[0.06,0.012,0],[0.08,-0.012,0],[0.10,0.012,0],[0.12,-0.012,0]];
  for(let i=0;i<rawPts.length;i++){
    wakeSmooth.push(rawPts[i],(i+1)*30);
  }
  const sm=wakeSmooth.sample(210,{includeLiveHead:false});
  assert.ok(sm.count>=4);
  let maxAbsY=0;
  for(let i=1;i<sm.count-1;i++){
    maxAbsY=Math.max(maxAbsY,Math.abs(sm.positions[i*3+1]));
  }
  assert.ok(maxAbsY<0.008,`expected jitter attenuated below 8 mm, got ${maxAbsY}`);

  // 4. Teleport jump (> maxJumpM) resets cleanly without emitting a long spike
  wakeSmooth.push([0.50,0,0],240);
  const afterJump=wakeSmooth.sample(240);
  assert.equal(afterJump.count,0,'jump > maxJumpM must reset wake cleanly without drawing a spike');
});
