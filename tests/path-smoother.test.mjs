import test from 'node:test';
import assert from 'node:assert/strict';
import {
  fuseCommittedTrailSegment,
  TRAIL_GAP_DEFAULTS,
  shouldStartNewTrailSegment,
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
  TRAIL_VERTEX_ROLE,
  TRAIL_MEASURE_STYLE_DEFAULTS,
  buildTrailMeasureFocus,
  evaluateEntryTrailMeasureStyle,
  measureTrailArcLength,
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

test('shouldStartNewTrailSegment separates true occlusion/pause relocations from brief drops and stationary occlusions',()=>{
  const pOrigin=vectorRigid([0,0,0,0.10,0.20,0.05]);
  const pMove10=vectorRigid([0,0,0,0.11,0.20,0.05]);
  const pMove60=vectorRigid([0,0,0,0.16,0.20,0.05]);
  const pMove80=vectorRigid([0,0,0,0.18,0.20,0.05]);
  const pMove200=vectorRigid([0,0,0,0.30,0.20,0.05]);

  assert.equal(TRAIL_GAP_DEFAULTS.minGapMs,550);
  assert.equal(TRAIL_GAP_DEFAULTS.minGapDistM,0.030);

  // Brief optical blur drop (200 ms, 80 mm) stays continuous
  const briefDrop=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove80,dtMs:200});
  assert.equal(briefDrop.isBreak,false);
  assert.equal(briefDrop.reason,null);

  // Stationary occlusion (900 ms, 10 mm) stays continuous
  const statOccl=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove10,dtMs:900});
  assert.equal(statOccl.isBreak,false);

  // Occlusion + relocation (650 ms, 60 mm) triggers occlusion-gap
  const occlGap=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove60,dtMs:650});
  assert.equal(occlGap.isBreak,true);
  assert.equal(occlGap.reason,'occlusion-gap');
  assert.ok(Math.abs(occlGap.distM-0.06)<1e-6);

  // Fast occluded jump (350 ms, 200 mm) triggers occluded-jump
  const jumpGap=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove200,dtMs:350});
  assert.equal(jumpGap.isBreak,true);
  assert.equal(jumpGap.reason,'occluded-jump');

  // Explicit break (e.g., manual stop/restart or live timeout) breaks >=30 mm and stays continuous <30 mm
  const expBreak=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove60,dtMs:0,explicitBreak:true});
  assert.equal(expBreak.isBreak,true);
  assert.equal(expBreak.reason,'explicit-break');

  const expSameSpot=shouldStartNewTrailSegment({prevPose:pOrigin,nextPose:pMove10,dtMs:1200,explicitBreak:true});
  assert.equal(expSameSpot.isBreak,false);

  // Invalid / missing poses never break
  const bad1=shouldStartNewTrailSegment({prevPose:null,nextPose:pMove60,dtMs:800});
  assert.deepEqual(bad1,{isBreak:false,distM:0,dtMs:0,reason:null});
  const badPose=pOrigin.slice();
  badPose[12]=NaN;
  const bad2=shouldStartNewTrailSegment({prevPose:badPose,nextPose:pMove60,dtMs:800});
  assert.deepEqual(bad2,{isBreak:false,distM:0,dtMs:0,reason:null});
});

test('smoothLineTail respects breakFlags: pins segment endpoints, prevents cross-gap bleed, handles 1- and 2-pt segments, and smooths interiors',()=>{
  // Segment 1: indices 0..6 (sawtooth in Z), Segment 2: indices 7..13 (sawtooth shifted far away in X/Y/Z)
  const saw=[+0.015,-0.015,+0.015,-0.015,+0.015,-0.015,+0.015];
  const n=14;
  const raw=new Float32Array(n*3);
  const out=new Float32Array(n*3);
  const breaks=new Uint8Array(n);
  breaks[7]=1;

  for(let i=0;i<7;i++){
    raw[i*3]=i*0.02;
    raw[i*3+1]=0.1;
    raw[i*3+2]=0.2+saw[i];
    smoothLineTail(raw,out,i+1,{maxDeviationM:0.020,breakFlags:breaks});
  }
  for(let j=0;j<7;j++){
    const idx=7+j;
    raw[idx*3]=0.80+j*0.02;
    raw[idx*3+1]=0.60;
    raw[idx*3+2]=0.50+saw[j];
    smoothLineTail(raw,out,idx+1,{maxDeviationM:0.020,breakFlags:breaks});
  }

  // Segment 1 endpoints (0 and 6) and Segment 2 endpoints (7 and 13) stay pinned exactly
  for(const endIdx of [0,6,7,13]){
    assert.ok(Math.abs(out[endIdx*3]-raw[endIdx*3])<1e-6,`x at ${endIdx} drifted across break`);
    assert.ok(Math.abs(out[endIdx*3+1]-raw[endIdx*3+1])<1e-6,`y at ${endIdx} drifted across break`);
    assert.ok(Math.abs(out[endIdx*3+2]-raw[endIdx*3+2])<1e-6,`z at ${endIdx} drifted across break`);
  }
  // Interiors of both segments still cancel Nyquist sawtooth cleanly
  for(const midIdx of [2,3,4,9,10,11]){
    const expectedZ=midIdx<7?0.2:0.5;
    assert.ok(Math.abs(out[midIdx*3+2]-expectedZ)<1e-6,`midIdx=${midIdx} expected ${expectedZ}, got ${out[midIdx*3+2]}`);
  }

  // 1-point and 2-point segments stay exact without blending neighbors across breaks
  const rawShort=new Float32Array([
    0,0,0,  0.05,0,0,          // seg 1 (2 pts: 0,1)
    0.40,0.40,0.40,            // seg 2 (1 pt: 2)
    0.80,0.80,0.80, 0.85,0.80,0.80, // seg 3 (2 pts: 3,4)
  ]);
  const outShort=new Float32Array(rawShort.length);
  const brkShort=new Uint8Array([0,0,1,1,0]);
  for(let k=1;k<=5;k++){
    smoothLineTail(rawShort,outShort,k,{maxDeviationM:0.02,breakFlags:brkShort});
  }
  for(let k=0;k<rawShort.length;k++){
    assert.ok(Math.abs(outShort[k]-rawShort[k])<1e-6,`short segment coord ${k} must remain exact`);
  }
});

test('buildTrailMeasureFocus and evaluateEntryTrailMeasureStyle distinguish hovered, 1-pin, between-pins span, outer tails, and unrelated gray strands',()=>{
  // Build two tags: tag 301 has segment 1 (vertices 0..5, seqs 1..6) and segment 2 (vertices 6..9, seqs 7..10).
  // Tag 302 has segment 1 (vertices 0..3, seqs 1..4).
  const n301=10;
  const segIds301=new Uint32Array([1,1,1,1,1,1,2,2,2,2]);
  const seqs301=new Uint32Array([1,2,3,4,5,6,7,8,9,10]);
  const base301=new Float32Array(n301*3);
  for(let i=0;i<n301;i++){
    base301[i*3]=0.10;
    base301[i*3+1]=0.85;
    base301[i*3+2]=0.95;
  }

  const n302=4;
  const segIds302=new Uint32Array([1,1,1,1]);
  const seqs302=new Uint32Array([1,2,3,4]);
  const base302=new Float32Array(n302*3);
  for(let i=0;i<n302;i++){
    base302[i*3]=0.95;
    base302[i*3+1]=0.40;
    base302[i*3+2]=0.15;
  }

  const roles301=new Uint8Array(n301),radii301=new Float32Array(n301),fades301=new Float32Array(n301),cols301=new Float32Array(n301*3);
  const roles302=new Uint8Array(n302),radii302=new Float32Array(n302),fades302=new Float32Array(n302),cols302=new Float32Array(n302*3);
  const baseR=0.0015;
  const gray=TRAIL_MEASURE_STYLE_DEFAULTS.unrelatedGrayLinearRgb;

  // 1. Inactive or empty -> NORMAL with exact base colors and normalFade.
  const inactive=buildTrailMeasureFocus({active:false,pinnedPoints:[{tagId:301,segmentIndex:1,lineSeq:2}],hoveredPoints:[{tagId:301,segmentIndex:1,lineSeq:4}]});
  assert.equal(inactive.hasFocus,false);
  const sInactive=evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:inactive,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  assert.equal(sInactive.hasFocus,false);
  assert.equal(sInactive.hasMixedLayers,false);
  for(let i=0;i<n301;i++){
    assert.equal(roles301[i],TRAIL_VERTEX_ROLE.NORMAL);
    assert.ok(Math.abs(fades301[i]-TRAIL_MEASURE_STYLE_DEFAULTS.normalFade)<1e-6);
    assert.ok(Math.abs(cols301[i*3]-0.10)<1e-6);
  }
  const emptyFocus=buildTrailMeasureFocus({active:true,pinnedPoints:[],hoveredPoints:[{tagId:301,segmentIndex:0}]});
  assert.equal(emptyFocus.hasFocus,false);

  // 2. Hovering 1 point on (301, 1): seg 1 -> HOVERED; seg 2 of 301 & seg 1 of 302 -> UNRELATED transparent gray.
  const hoverOnly=buildTrailMeasureFocus({active:true,hoveredPoints:[{tagId:301,segmentIndex:1,lineSeq:3}]});
  assert.equal(hoverOnly.hasFocus,true);
  assert.equal(hoverOnly.strands.get('301:1')?.state,'hovered');
  const sumHover301=evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:hoverOnly,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  const sumHover302=evaluateEntryTrailMeasureStyle({
    tagId:302,lineCount:n302,lineSegmentIds:segIds302,lineSeqs:seqs302,baseColors:base302,
    focus:hoverOnly,baseRadiusM:baseR,outRoles:roles302,outRadii:radii302,outFades:fades302,outColors:cols302,
  });
  assert.equal(sumHover301.hasActiveVertex,true);
  assert.equal(sumHover301.hasDimVertex,true);
  assert.equal(sumHover301.hasMixedLayers,true);
  assert.equal(sumHover302.hasActiveVertex,false);
  assert.equal(sumHover302.hasDimVertex,true);
  for(let i=0;i<6;i++){
    assert.equal(roles301[i],TRAIL_VERTEX_ROLE.HOVERED);
    assert.ok(Math.abs(fades301[i]-TRAIL_MEASURE_STYLE_DEFAULTS.hoveredFade)<1e-6);
    assert.ok(radii301[i]>baseR);
  }
  for(let i=6;i<n301;i++){
    assert.equal(roles301[i],TRAIL_VERTEX_ROLE.UNRELATED);
    assert.ok(Math.abs(fades301[i]-TRAIL_MEASURE_STYLE_DEFAULTS.unrelatedFade)<1e-6);
    assert.ok(Math.abs(cols301[i*3]-gray[0])<1e-6&&Math.abs(cols301[i*3+1]-gray[1])<1e-6&&Math.abs(cols301[i*3+2]-gray[2])<1e-6);
  }
  for(let i=0;i<n302;i++){
    assert.equal(roles302[i],TRAIL_VERTEX_ROLE.UNRELATED);
    assert.ok(Math.abs(cols302[i*3]-gray[0])<1e-6);
  }

  // 3. Pinning 1 point on (301, 1) with no hover -> PINNED_SINGLE (colored, fade 0.72), other runs/IDs -> UNRELATED gray.
  const pin1Only=buildTrailMeasureFocus({active:true,pinnedPoints:[{tagId:301,segmentIndex:1,lineSeq:2}]});
  assert.equal(pin1Only.strands.get('301:1')?.state,'pinned-single');
  evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:pin1Only,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  for(let i=0;i<6;i++){
    assert.equal(roles301[i],TRAIL_VERTEX_ROLE.PINNED_SINGLE);
    assert.ok(Math.abs(fades301[i]-TRAIL_MEASURE_STYLE_DEFAULTS.pinnedSingleFade)<1e-6);
    assert.ok(cols301[i*3+2]>0.75,'PINNED_SINGLE must keep tag color rather than neutral gray');
  }
  for(let i=6;i<n301;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.UNRELATED);

  // 4. Pinning 1 point (seq 2) + hovering another point (seq 5) on (301, 1) -> indices 1..4 PINNED_PREVIEW, 0 & 5 PINNED_OUTER.
  const pin1Hover=buildTrailMeasureFocus({
    active:true,
    pinnedPoints:[{tagId:301,segmentIndex:1,lineSeq:2}],
    hoveredPoints:[{tagId:301,segmentIndex:1,lineSeq:5}],
  });
  evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:pin1Hover,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  assert.equal(roles301[0],TRAIL_VERTEX_ROLE.PINNED_OUTER);
  for(let i=1;i<=4;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.PINNED_PREVIEW);
  assert.equal(roles301[5],TRAIL_VERTEX_ROLE.PINNED_OUTER);
  assert.ok(Math.abs(fades301[0]-TRAIL_MEASURE_STYLE_DEFAULTS.pinnedOuterFade)<1e-6);
  assert.ok(Math.abs(fades301[2]-TRAIL_MEASURE_STYLE_DEFAULTS.pinnedPreviewFade)<1e-6);

  // 5. Pinning 2 points (seq 2 and 5) and 3 points (seq 2, 4, 5) -> [1..4] PINNED_SPAN (1.64x, fade 1.0), outer [0,5] PINNED_OUTER (colored, fade 0.48), seg 2 UNRELATED gray.
  for(const pinSeqList of [[2,5],[2,4,5]]){
    const pinFocus=buildTrailMeasureFocus({
      active:true,
      pinnedPoints:pinSeqList.map(seq=>({tagId:301,segmentIndex:1,lineSeq:seq})),
    });
    assert.equal(pinFocus.strands.get('301:1')?.state,'pinned-span');
    evaluateEntryTrailMeasureStyle({
      tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
      focus:pinFocus,baseRadiusM:baseR,preserveChroma:false,
      outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
    });
    assert.equal(roles301[0],TRAIL_VERTEX_ROLE.PINNED_OUTER);
    assert.equal(roles301[5],TRAIL_VERTEX_ROLE.PINNED_OUTER);
    assert.ok(cols301[2]>cols301[0]&&cols301[2]>gray[2]+0.2,'PINNED_OUTER must remain colored (not gray)');
    for(let i=1;i<=4;i++){
      assert.equal(roles301[i],TRAIL_VERTEX_ROLE.PINNED_SPAN);
      assert.ok(Math.abs(fades301[i]-1.0)<1e-6);
      assert.ok(Math.abs(radii301[i]-baseR*TRAIL_MEASURE_STYLE_DEFAULTS.pinnedSpanRadiusScale)<1e-7);
      assert.ok(cols301[i*3]>0.10,'ID mode lifts span color toward white');
    }
    for(let i=6;i<n301;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.UNRELATED);

    // preserveChroma: true (Turbo Speed mode) keeps exact unmodified base color on PINNED_SPAN.
    evaluateEntryTrailMeasureStyle({
      tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
      focus:pinFocus,baseRadiusM:baseR,preserveChroma:true,
      outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
    });
    assert.ok(Math.abs(cols301[1*3]-0.10)<1e-6&&Math.abs(cols301[1*3+1]-0.85)<1e-6&&Math.abs(cols301[1*3+2]-0.95)<1e-6);
  }

  // 6. Dual-hand hover on same unpinned strand previews span; dual-hand hover on different strands highlights both while graying out third.
  const dualSame=buildTrailMeasureFocus({
    active:true,
    hoveredPoints:[{tagId:301,segmentIndex:1,lineSeq:2},{tagId:301,segmentIndex:1,lineSeq:5}],
  });
  evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:dualSame,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  assert.equal(roles301[0],TRAIL_VERTEX_ROLE.PINNED_OUTER);
  for(let i=1;i<=4;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.PINNED_PREVIEW);
  assert.equal(roles301[5],TRAIL_VERTEX_ROLE.PINNED_OUTER);

  const dualDiff=buildTrailMeasureFocus({
    active:true,
    hoveredPoints:[{tagId:301,segmentIndex:2,lineSeq:8},{tagId:302,segmentIndex:1,lineSeq:2}],
  });
  evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:n301,lineSegmentIds:segIds301,lineSeqs:seqs301,baseColors:base301,
    focus:dualDiff,baseRadiusM:baseR,outRoles:roles301,outRadii:radii301,outFades:fades301,outColors:cols301,
  });
  evaluateEntryTrailMeasureStyle({
    tagId:302,lineCount:n302,lineSegmentIds:segIds302,lineSeqs:seqs302,baseColors:base302,
    focus:dualDiff,baseRadiusM:baseR,outRoles:roles302,outRadii:radii302,outFades:fades302,outColors:cols302,
  });
  for(let i=0;i<6;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.UNRELATED);
  for(let i=6;i<n301;i++)assert.equal(roles301[i],TRAIL_VERTEX_ROLE.HOVERED);
  for(let i=0;i<n302;i++)assert.equal(roles302[i],TRAIL_VERTEX_ROLE.HOVERED);

  // 7. Ring-buffer wraparound: scrolled-out seq (e.g. seq 1 when buffer starts at seq 4) clamps gracefully without NaN.
  const wrappedSeqs=new Uint32Array([4,5,6,7,8,9]);
  const wrappedSegs=new Uint32Array([1,1,1,1,1,1]);
  const wrappedRoles=new Uint8Array(6),wrappedRadii=new Float32Array(6),wrappedFades=new Float32Array(6),wrappedCols=new Float32Array(18);
  const wrapFocus=buildTrailMeasureFocus({
    active:true,
    pinnedPoints:[{tagId:301,segmentIndex:1,lineSeq:1},{tagId:301,segmentIndex:1,lineSeq:7}],
  });
  evaluateEntryTrailMeasureStyle({
    tagId:301,lineCount:6,lineSegmentIds:wrappedSegs,lineSeqs:wrappedSeqs,baseColors:base301,
    focus:wrapFocus,baseRadiusM:baseR,outRoles:wrappedRoles,outRadii:wrappedRadii,outFades:wrappedFades,outColors:wrappedCols,
  });
  assert.equal(wrappedRoles[0],TRAIL_VERTEX_ROLE.PINNED_SPAN);
  assert.equal(wrappedRoles[3],TRAIL_VERTEX_ROLE.PINNED_SPAN);
  assert.equal(wrappedRoles[4],TRAIL_VERTEX_ROLE.PINNED_OUTER);
  for(let i=0;i<6;i++)assert.ok(Number.isFinite(wrappedRadii[i])&&Number.isFinite(wrappedFades[i]));
});

test('measureTrailArcLength computes exact curved path length along a continuous segment and guards breaks/eviction',()=>{
  // Segment 1 (indices 0..4, seqs 10..14): 90-deg L-shape:
  // (0,0,0) -> (0.10,0,0) -> (0.20,0,0) -> (0.20,0.10,0) -> (0.20,0.20,0) => arc = 0.40 m vs ~0.283 m straight line.
  // Segment 2 (indices 5..8, seqs 15..18): U-turn loop:
  // (0.50,0,0) -> (0.70,0,0) -> (0.70,0.02,0) -> (0.50,0.02,0) => arc = 0.42 m vs 0.02 m straight line.
  const coords=[
    [0.00,0.00,0.00],[0.10,0.00,0.00],[0.20,0.00,0.00],[0.20,0.10,0.00],[0.20,0.20,0.00],
    [0.50,0.00,0.00],[0.70,0.00,0.00],[0.70,0.02,0.00],[0.50,0.02,0.00],
  ];
  const n=coords.length;
  const pos=new Float32Array(n*3);
  for(let i=0;i<n;i++)pos.set(coords[i],i*3);
  const segIds=new Uint32Array([1,1,1,1,1,2,2,2,2]);
  const seqs=new Uint32Array([10,11,12,13,14,15,16,17,18]);
  const breaks=new Uint8Array([0,0,0,0,0,1,0,0,0]);

  // L-shape forward & reverse symmetry
  const lForward=measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:10,seqB:14});
  const lReverse=measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:14,seqB:10});
  assert.ok(Math.abs(lForward-0.40)<1e-6,`expected 0.40 m, got ${lForward}`);
  assert.ok(Math.abs(lReverse-0.40)<1e-6);

  // Sub-span (seq 11 -> seq 13 = 0.20 m) and identical seq -> 0
  const lSub=measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:11,seqB:13});
  assert.ok(Math.abs(lSub-0.20)<1e-6);
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:12,seqB:12}),0);

  // Segment 2 U-turn loop: start and end are 20 mm apart straight-line, arc length is 0.42 m
  const uLoop=measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:2,seqA:15,seqB:18});
  assert.ok(Math.abs(uLoop-0.42)<1e-6,`expected 0.42 m, got ${uLoop}`);

  // Missing segment, break inside span, evicted seq, or invalid args return null
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:3,seqA:10,seqB:14}),null);
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:8,seqB:14}),null);
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:breaks,segmentIndex:1,seqA:10,seqB:15}),null);

  const badBreaks=new Uint8Array([0,0,1,0,0,1,0,0,0]);
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:n,lineSegmentIds:segIds,lineSeqs:seqs,lineBreaks:badBreaks,segmentIndex:1,seqA:10,seqB:14}),null);
  assert.equal(measureTrailArcLength({linePositions:pos,lineCount:1,lineSegmentIds:segIds,lineSeqs:seqs,segmentIndex:1,seqA:10,seqB:10}),null);
});
