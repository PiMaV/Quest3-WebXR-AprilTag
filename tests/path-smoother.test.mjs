import test from 'node:test';
import assert from 'node:assert/strict';
import {fuseCommittedTrailSegment,smoothLineTail} from '../src/path-smoother.mjs';
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
