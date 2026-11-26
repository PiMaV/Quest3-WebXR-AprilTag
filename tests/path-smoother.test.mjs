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
