import test from 'node:test';
import assert from 'node:assert/strict';
import {observeReference,referenceTools,fitPlanarPose,solveSingleTagPose,incrementPose,requiredTagEdgeMm,distortNormalized,undistortPoint,projectCameraPoint} from '../src/rig.mjs';
import {cvToGL,product,inverse,poseAt,meanRigid,rigidVector,vectorRigid,observationIsFresh} from '../src/runtime.mjs';
import {prepareRgbaFrame,prepareGrayFrame,grayFingerprint,frameEvidence} from '../src/frame-evidence.mjs';
import {xrExitPressed,nextExitHold,XR_EXIT_HOLD_MS} from '../src/xr-exit.mjs';
import {synthetic,rig} from './synthetic.mjs';

test('requiredTagEdgeMm returns exact standard edge lengths per 36h11 ID class and rejects others',()=>{
  assert.equal(requiredTagEdgeMm(100),6.0);
  assert.equal(requiredTagEdgeMm(199),6.0);
  assert.equal(requiredTagEdgeMm(200),12.0);
  assert.equal(requiredTagEdgeMm(299),12.0);
  assert.equal(requiredTagEdgeMm(300),20.0);
  assert.equal(requiredTagEdgeMm(350),20.0);
  assert.equal(requiredTagEdgeMm(399),20.0);
  assert.equal(requiredTagEdgeMm(400),80.0);
  assert.equal(requiredTagEdgeMm(449),80.0);
  assert.equal(requiredTagEdgeMm(450),112.0);
  assert.equal(requiredTagEdgeMm(499),112.0);
  for(const bad of [-1,0,99,500,586,200.5,NaN,'350'])
    assert.throws(()=>requiredTagEdgeMm(bad),/No standard tag edge/);
});

test('referenceTools validates schema, tag count, square geometry and winding',()=>{
  assert.equal(referenceTools(rig).length,1);
  assert.throws(()=>referenceTools({...rig,schema:'wrong'}));
  assert.throws(()=>referenceTools({...rig,tools:[{id:'x',tags:rig.tools[0].tags.slice(0,2)}]}));
  const flipped=structuredClone(rig);flipped.tools[0].tags[0].corners.reverse();
  assert.throws(()=>referenceTools(flipped));
  const nonSquare=structuredClone(rig);nonSquare.tools[0].tags[0].corners[1][0]+=0.01;
  assert.throws(()=>referenceTools(nonSquare));
});

test('observeReference filters outliers, duplicates, mirrored winding, tiny quads and unassigned IDs',()=>{
  const d=synthetic({noise:0,count:5}),K=d.truth.K,row=d.rows[2];
  const markers=row.tags.map(t=>({id:t.id,corners:structuredClone(t.pixels)}));
  const clean=observeReference(markers,rig,K);
  assert.equal(clean.tools.length,1);
  assert.deepEqual(clean.tools[0].tagIds,[350,351,352]);
  assert.ok(clean.tools[0].reprojectionRmsPx<0.2);

  // Corrupt one tag severely -> 2-of-3 consensus keeps the two consistent tags and rejects the outlier.
  const oneBad=structuredClone(markers);
  oneBad[2].corners.forEach(p=>{p.x+=45;});
  const partial=observeReference(oneBad,rig,K);
  assert.equal(partial.tools.length,1);
  assert.deepEqual(partial.tools[0].tagIds,[350,351]);
  assert.ok(partial.rejected.some(r=>r.id===352&&r.reason==='rigid_tag_outlier'));

  // Two disagreeing tags (no strict majority) -> rejected as inconsistent.
  const twoDisagree=structuredClone(markers.slice(0,2));
  twoDisagree[1].corners.forEach(p=>{p.y+=80;});
  const noConsensus=observeReference(twoDisagree,rig,K);
  assert.equal(noConsensus.tools.length,0);
  assert.ok(noConsensus.rejected.every(r=>r.reason==='inconsistent_rigid_geometry'));

  // Mirrored winding, duplicates, tiny quad, and unassigned IDs are all categorized cleanly.
  const mixed=[
    {id:350,corners:structuredClone(markers[0].corners).reverse()},
    {id:351,corners:structuredClone(markers[1].corners)},
    {id:351,corners:structuredClone(markers[1].corners)},
    {id:352,corners:[{x:10,y:10},{x:12,y:10},{x:12,y:12},{x:10,y:12}]},
    {id:400,corners:structuredClone(markers[0].corners)},
  ];
  const res=observeReference(mixed,rig,K);
  assert.equal(res.tools.length,0);
  assert.ok(res.rejected.some(r=>r.id===350&&r.reason==='small_or_invalid_quad'));
  assert.ok(res.rejected.some(r=>r.id===351&&r.reason==='duplicate_id'));
  assert.ok(res.rejected.some(r=>r.id===352&&r.reason==='small_or_invalid_quad'));
  assert.ok(res.rejected.some(r=>r.id===400&&r.reason==='unassigned_id'));
});

test('runtime cvToGL, poseAt gap rejection, meanRigid and observationIsFresh behave consistently',()=>{
  const d=synthetic({noise:0,count:5}),K=d.truth.K,row=d.rows[1];
  const pose=fitPlanarPose(row.tags,K);assert.ok(pose);
  const gl=cvToGL(pose);
  const expected=product(inverse(d.truth.X),inverse(poseAt(d.history,row.captureTime+d.truth.delay)),d.truth.L);
  const err=rigidVector(product(inverse(expected),gl));
  assert.ok(Math.hypot(...err)<1e-4);

  // poseAt must refuse >100 ms gaps and out-of-range times.
  const gapped=[{xrRafTimeMs:0,worldFromViewer:vectorRigid([0,0,0,0,0,0])},{xrRafTimeMs:150,worldFromViewer:vectorRigid([0,0,0,1,0,0])}];
  assert.equal(poseAt(gapped,75),null);
  assert.equal(poseAt(d.history,-10),null);
  assert.equal(poseAt(d.history,999999),null);

  // meanRigid handles antipodal rotations near 180 deg.
  const a=vectorRigid([0,3.13,0,0,0,-1]),b=vectorRigid([0,-3.13,0,0,0,-1]);
  const m=meanRigid([a,b]);
  assert.ok(Math.abs(rigidVector(m)[1])>3.1);

  assert.equal(observationIsFresh({context:{sessionId:'a',spaceEpoch:0},submittedAtPerformanceMs:500,frameEvidence:{fresh:true}},{sessionId:'a',spaceEpoch:0},400),false);
});

test('frame-evidence detects identical vs changed pixels and validates RGBA length',()=>{
  const a=new Uint8ClampedArray(4*4*4).fill(100),b=new Uint8ClampedArray(a);b[0]=200;
  const first=prepareRgbaFrame(a,4,4,null,1);
  assert.equal(first.evidence.fresh,true);
  const frozen=prepareRgbaFrame(a,4,4,first.evidence,1);
  assert.equal(frozen.evidence.fresh,false);
  assert.equal(frozen.evidence.pixelsChanged,false);
  const changed=prepareRgbaFrame(b,4,4,first.evidence,1);
  assert.equal(changed.evidence.fresh,true);
  assert.equal(changed.evidence.pixelsChanged,true);
  assert.equal(grayFingerprint(first.gray,4,4),first.evidence.fingerprint);
  assert.equal(frameEvidence(first.evidence,first.evidence.fingerprint,2).pixelsChanged,false);
  assert.throws(()=>prepareRgbaFrame(new Uint8ClampedArray(5),4,4,null,1),/Invalid RGBA/);

  // Direct grayscale path (WebCodecs Y-plane) produces identical fingerprints and validates dimensions.
  const grayOnly=prepareGrayFrame(first.gray,4,4,null,1);
  assert.equal(grayOnly.evidence.fingerprint,first.evidence.fingerprint);
  assert.equal(prepareGrayFrame(first.gray,4,4,grayOnly.evidence,1).evidence.fresh,false);
  assert.throws(()=>prepareGrayFrame(new Uint8Array(5),4,4,null,1),/Invalid grayscale/);

  // Large-frame strided hashing remains consistent between prepareRgbaFrame and grayFingerprint.
  const bigRgba=new Uint8ClampedArray(128*108*4).fill(90);
  const bigRes=prepareRgbaFrame(bigRgba,128,108,null,1);
  assert.equal(grayFingerprint(bigRes.gray,128,108),bigRes.evidence.fingerprint);
});

test('xr-exit handles left thumbstick, secondary button and hold timer cleanly',()=>{
  const left={handedness:'left',gamepad:{buttons:[{},{},{},{pressed:true,value:1},{},{pressed:false,value:0}]}};
  const right={handedness:'right',gamepad:{buttons:[{},{},{},{pressed:true,value:1},{},{pressed:true,value:1}]}};
  assert.equal(xrExitPressed([left]).stick,true);
  assert.equal(xrExitPressed([left,right]).secondary,true);
  // Right thumbstick alone should NOT trigger exit when both controllers are present.
  const rightStickOnly={handedness:'right',gamepad:{buttons:[{},{},{},{pressed:true,value:1},{},{pressed:false,value:0}]}};
  const leftIdle={handedness:'left',gamepad:{buttons:[{},{},{},{pressed:false,value:0},{},{pressed:false,value:0}]}};
  assert.equal(xrExitPressed([leftIdle,rightStickOnly]).held,false);

  let h={sinceMs:null,progress:0,fire:false};
  h=nextExitHold(h,true,1000,XR_EXIT_HOLD_MS);
  assert.equal(h.fire,false);
  h=nextExitHold(h,true,1000+XR_EXIT_HOLD_MS/2,XR_EXIT_HOLD_MS);
  assert.ok(h.progress>0.49&&h.progress<0.51&&!h.fire);
  h=nextExitHold(h,true,1000+XR_EXIT_HOLD_MS,XR_EXIT_HOLD_MS);
  assert.equal(h.fire,true);
  assert.equal(nextExitHold(h,false,3000).sinceMs,null);
});

test('zero-dependency runtime math matches Three.js Matrix4/Quaternion to 1e-12 across arbitrary rigid transforms',async()=>{
  const {Matrix4,Quaternion,Vector3}=await import('../vendor/three/three.module.js');
  const a=vectorRigid([0.4,-0.7,1.1,0.3,-0.2,0.8]),b=vectorRigid([-0.9,0.5,-0.3,-0.4,0.6,-0.5]);
  const ma=new Matrix4().fromArray(a),mb=new Matrix4().fromArray(b);
  const prodExpected=new Matrix4().multiplyMatrices(ma,mb).toArray();
  const prodActual=product(a,b);
  for(let i=0;i<16;i++)assert.ok(Math.abs(prodActual[i]-prodExpected[i])<1e-12);
  const invExpected=ma.clone().invert().toArray();
  const invActual=inverse(a);
  for(let i=0;i<16;i++)assert.ok(Math.abs(invActual[i]-invExpected[i])<1e-12);
  const qa=new Quaternion().setFromRotationMatrix(ma),qb=new Quaternion().setFromRotationMatrix(mb);
  const slerpExpected=new Matrix4().compose(new Vector3(0.3,-0.2,0.8).lerp(new Vector3(-0.4,0.6,-0.5),0.35),qa.clone().slerp(qb,0.35),new Vector3(1,1,1)).toArray();
  const slerpActual=poseAt([{xrRafTimeMs:0,worldFromViewer:a},{xrRafTimeMs:100,worldFromViewer:b}],35);
  for(let i=0;i<16;i++)assert.ok(Math.abs(slerpActual[i]-slerpExpected[i])<1e-12);
});

test('analytic-Jacobian fitPlanarPose and solveSingleTagPose recover exact 6DOF poses and enforce quality gates',()=>{
  const d=synthetic({noise:0,count:8}),K=d.truth.K;
  // Multi-view 3-tag rig accuracy against synthetic trajectory (reprojection RMS < 1e-10 px;
  // history poseAt compares within < 1e-4 due to 10 ms SLERP chord interpolation).
  for(const row of d.rows){
    const pose=fitPlanarPose(row.tags,K);
    assert.ok(pose);
    const gl=cvToGL(pose);
    const expected=product(inverse(d.truth.X),inverse(poseAt(d.history,row.captureTime+d.truth.delay)),d.truth.L);
    const err=rigidVector(product(inverse(expected),gl));
    assert.ok(Math.hypot(...err)<1e-4,`poseAt delta err ${Math.hypot(...err)}`);
    let sq=0;
    for(const tag of row.tags){
      for(let i=0;i<4;i++){
        const [x,y]=tag.points[i];
        const px=pose.t[0]+pose.R[0][0]*x+pose.R[1][0]*y;
        const py=pose.t[1]+pose.R[0][1]*x+pose.R[1][1]*y;
        const pz=pose.t[2]+pose.R[0][2]*x+pose.R[1][2]*y;
        sq+=(K.fx*px/pz+K.cx-tag.pixels[i].x)**2+(K.fy*py/pz+K.cy-tag.pixels[i].y)**2;
      }
    }
    const rms=Math.sqrt(sq/(row.tags.length*4));
    assert.ok(rms<1e-10,`reprojection RMS ${rms}`);
  }

  // Exact direct 3-tag rig projections from oblique 6DOF poses (no chord approximation).
  for(const vec of [
    [0.35,-0.45,0.25,-0.05,0.04,-0.68],
    [-0.5,0.3,-0.7,0.12,-0.08,-0.55],
  ]){
    const gtGL=vectorRigid(vec);
    const rigTags=rig.tools[0].tags.map(tag=>({
      id:tag.id,
      points:tag.corners,
      pixels:tag.corners.map(([x,y])=>{
        const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
        const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
        const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
        return {x:K.fx*(wx/(-wz))+K.cx,y:K.fy*((-wy)/(-wz))+K.cy};
      }),
    }));
    const pose=fitPlanarPose(rigTags,K);
    assert.ok(pose);
    const delta=rigidVector(product(inverse(gtGL),cvToGL(pose)));
    assert.ok(Math.hypot(...delta.slice(0,3))<1e-10,`exact 3-tag rot err ${Math.hypot(...delta.slice(0,3))}`);
    assert.ok(Math.hypot(...delta.slice(3,6))<1e-10,`exact 3-tag pos err ${Math.hypot(...delta.slice(3,6))}`);
    let sq=0;
    for(const tag of rigTags){
      for(let i=0;i<4;i++){
        const [x,y]=tag.points[i];
        const px=pose.t[0]+pose.R[0][0]*x+pose.R[1][0]*y;
        const py=pose.t[1]+pose.R[0][1]*x+pose.R[1][1]*y;
        const pz=pose.t[2]+pose.R[0][2]*x+pose.R[1][2]*y;
        sq+=(K.fx*px/pz+K.cx-tag.pixels[i].x)**2+(K.fy*py/pz+K.cy-tag.pixels[i].y)**2;
      }
    }
    assert.ok(Math.sqrt(sq/(rigTags.length*4))<1e-10);
  }

  // Oblique single-tag solve via solveSingleTagPose (< 1e-10).
  const gtGL=vectorRigid([0.45,-0.55,0.65,0.06,-0.04,-0.52]);
  const half=0.010; // 20 mm tag (#305)
  const cornersLocal=[[-half,-half],[half,-half],[half,half],[-half,half]];
  const corners=cornersLocal.map(([x,y])=>{
    const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
    const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
    const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
    return {x:K.fx*(wx/(-wz))+K.cx,y:K.fy*((-wy)/(-wz))+K.cy};
  });
  const solved=solveSingleTagPose({id:305,corners},K,{minEdgePx:8,minAreaPx:64,maxRmsPx:0.5});
  assert.ok(solved);
  assert.equal(solved.edgeMm,20);
  assert.ok(solved.minEdgePx>15);
  assert.ok(solved.rmsPx<1e-10);
  const recoveredGL=cvToGL(solved.pose);
  const delta=rigidVector(product(inverse(gtGL),recoveredGL));
  assert.ok(Math.hypot(...delta.slice(0,3))<1e-10);
  assert.ok(Math.hypot(...delta.slice(3,6))<1e-10);

  // Noisy single-tag corners (+/-0.4 px): stationary point matches finite-difference optimality.
  const noisyCorners=corners.map((p,i)=>({x:p.x+(i%2===0?0.38:-0.35),y:p.y+(i<2?-0.32:0.36)}));
  const noisySolved=solveSingleTagPose({id:305,corners:noisyCorners},K,{minEdgePx:8,minAreaPx:64,maxRmsPx:1.0});
  assert.ok(noisySolved&&noisySolved.rmsPx<0.5);
  const eps=1e-6;
  const costAt=pose=>{
    let s=0;
    for(let i=0;i<4;i++){
      const [x,y]=cornersLocal[i];
      const px=pose.t[0]+pose.R[0][0]*x+pose.R[1][0]*y;
      const py=pose.t[1]+pose.R[0][1]*x+pose.R[1][1]*y;
      const pz=pose.t[2]+pose.R[0][2]*x+pose.R[1][2]*y;
      s+=(K.fx*px/pz+K.cx-noisyCorners[i].x)**2+(K.fy*py/pz+K.cy-noisyCorners[i].y)**2;
    }
    return s;
  };
  const baseCost=costAt(noisySolved.pose);
  for(let j=0;j<6;j++){
    const pStep=[0,0,0,0,0,0];pStep[j]=eps;
    const mStep=[0,0,0,0,0,0];mStep[j]=-eps;
    const cPlus=costAt(incrementPose(noisySolved.pose,pStep));
    const cMinus=costAt(incrementPose(noisySolved.pose,mStep));
    const grad=Math.abs(cPlus-cMinus)/(2*eps);
    assert.ok(cPlus>=baseCost-1e-12&&cMinus>=baseCost-1e-12,`expected local minimum along dim ${j}`);
    assert.ok(grad<1e-4,`non-stationary gradient along dim ${j}: ${grad}`);
  }

  // Quality gate rejections.
  assert.ok(Number.isFinite(solved.cosView)&&solved.cosView>0.5&&solved.cosView<=1);
  assert.ok(Number.isFinite(solved.depthM)&&Math.abs(solved.depthM-0.52)<0.05);
  assert.equal(solveSingleTagPose({id:305,corners},K,{minCosView:0.99}),null,'grazing cosView gate must reject');
  assert.equal(solveSingleTagPose({id:305,corners},K,{maxDepthM:0.25}),null,'maxDepthM gate must reject');
  assert.equal(solveSingleTagPose({id:10,corners},K),null);
  assert.equal(solveSingleTagPose({id:305,corners:corners.slice().reverse()},K),null);
  const tiny=corners.map(p=>({x:100+(p.x-corners[0].x)*0.05,y:100+(p.y-corners[0].y)*0.05}));
  assert.equal(solveSingleTagPose({id:305,corners:tiny},K,{minEdgePx:8,minAreaPx:64}),null);
  const distorted=structuredClone(corners);distorted[0].x+=12;
  assert.equal(solveSingleTagPose({id:305,corners:distorted},K,{maxRmsPx:1.5}),null);
});

test('solveSingleTagPose supports custom edgeMm and getTagEdgeMm overrides without breaking defaults',()=>{
  const K={width:1280,height:1080,fx:900,fy:900,cx:640,cy:540};
  const gtGL=vectorRigid([0.2,-0.3,0.1,0.03,-0.02,-0.50]);
  const half20=0.010;
  const cornersLocal=[[-half20,-half20],[half20,-half20],[half20,half20],[-half20,half20]];
  const corners=cornersLocal.map(([x,y])=>{
    const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
    const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
    const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
    return {x:K.fx*(wx/(-wz))+K.cx,y:K.fy*((-wy)/(-wz))+K.cy};
  });

  // Default behavior unchanged: non-M.E.S.S. ID 0 returns null, ID 300 resolves to 20 mm.
  assert.equal(solveSingleTagPose({id:0,corners},K),null);
  const def300=solveSingleTagPose({id:300,corners},K,{maxRmsPx:0.5});
  assert.ok(def300);
  assert.equal(def300.edgeMm,20);

  // Explicit edgeMm=50 scales depth by 2.5x relative to 20 mm corners.
  const custom50=solveSingleTagPose({id:0,corners},K,{edgeMm:50,maxRmsPx:0.5});
  assert.ok(custom50);
  assert.equal(custom50.edgeMm,50);
  assert.ok(Math.abs(custom50.depthM-def300.depthM*2.5)<1e-6);

  // Callback lookup getTagEdgeMm(id).
  const lookup=id=>(id===7?35:null);
  const custom7=solveSingleTagPose({id:7,corners},K,{getTagEdgeMm:lookup,maxRmsPx:0.5});
  assert.ok(custom7);
  assert.equal(custom7.edgeMm,35);
  assert.ok(Math.abs(custom7.depthM-def300.depthM*(35/20))<1e-6);
  assert.equal(solveSingleTagPose({id:8,corners},K,{getTagEdgeMm:lookup}),null);

  // Invalid custom sizes fail cleanly with null.
  assert.equal(solveSingleTagPose({id:0,corners},K,{edgeMm:0}),null);
  assert.equal(solveSingleTagPose({id:0,corners},K,{edgeMm:-10}),null);
  assert.equal(solveSingleTagPose({id:0,corners},K,{edgeMm:NaN}),null);
  assert.equal(solveSingleTagPose({id:0,corners},K,{getTagEdgeMm:()=>{throw new Error('boom');}}),null);
});

test('distortNormalized, undistortPoint, fitPlanarPose, and solveSingleTagPose handle radial distortion k1 accurately',()=>{
  const Kbase={width:1280,height:1080,fx:886.3,fy:886.3,cx:640,cy:540};
  for(const k1 of [-0.15,-0.08,0,0.08]){
    const K={...Kbase,k1};
    for(const [xn,yn] of [[0,0],[0.25,0.15],[-0.45,0.35],[0.62,-0.52]]){
      const [xd,yd]=distortNormalized(xn,yn,k1);
      const u=K.fx*xd+K.cx,v=K.fy*yd+K.cy;
      const back=undistortPoint(u,v,K);
      const ux=K.fx*xn+K.cx,uy=K.fy*yn+K.cy;
      assert.ok(Math.hypot(back.x-ux,back.y-uy)<1e-6,`roundtrip failed for k1=${k1}`);
    }
  }
  // Fold-safety guard for extreme corner under strong barrel distortion.
  const extreme=undistortPoint(2400,1900,{...Kbase,k1:-0.25});
  assert.ok(Number.isFinite(extreme.x)&&Number.isFinite(extreme.y));

  // Oblique 45 deg outer-field tag at X=0.25m, Z=0.50m under k1=-0.08 and k1=+0.06.
  const half=0.010,local=[[-half,-half],[half,-half],[half,half],[-half,half]];
  const gtGL=vectorRigid([0.65,-0.25,0.15,0.22,-0.12,-0.52]);
  for(const k1 of [-0.08,0.06]){
    const K={...Kbase,k1};
    const corners=local.map(([x,y])=>{
      const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
      const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
      const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
      return projectCameraPoint(wx,-wy,-wz,K);
    });
    const solved=solveSingleTagPose({id:305,corners},K,{maxRmsPx:0.5});
    assert.ok(solved);
    assert.ok(solved.rmsPx<1e-9,`expected <1e-9 rmsPx, got ${solved.rmsPx}`);
    const d=rigidVector(product(inverse(gtGL),cvToGL(solved.pose)));
    assert.ok(Math.hypot(...d.slice(0,3))<1e-9);
    assert.ok(Math.hypot(...d.slice(3,6))<1e-9);

    // Multi-tag planar rig recovery with radial distortion.
    const rigTags=rig.tools[0].tags.map(tag=>({
      id:tag.id,
      points:tag.corners,
      pixels:tag.corners.map(([x,y])=>{
        const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
        const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
        const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
        return projectCameraPoint(wx,-wy,-wz,K);
      }),
    }));
    const rigPose=fitPlanarPose(rigTags,K);
    assert.ok(rigPose);
    const rd=rigidVector(product(inverse(gtGL),cvToGL(rigPose)));
    assert.ok(Math.hypot(...rd)<1e-9);
  }

  // Analytic distorted 2x6 Jacobian optimality check on noisy outer-field tag corners.
  const Krad={...Kbase,k1:-0.08};
  const noisyCorners=local.map(([x,y],i)=>{
    const wx=gtGL[0]*x+gtGL[4]*y+gtGL[12];
    const wy=gtGL[1]*x+gtGL[5]*y+gtGL[13];
    const wz=gtGL[2]*x+gtGL[6]*y+gtGL[14];
    const p=projectCameraPoint(wx,-wy,-wz,Krad);
    return {x:p.x+(i%2===0?0.35:-0.32),y:p.y+(i<2?-0.30:0.34)};
  });
  const noisySolved=solveSingleTagPose({id:305,corners:noisyCorners},Krad,{maxRmsPx:1.0});
  assert.ok(noisySolved&&noisySolved.rmsPx<0.5);
  const costDist=pose=>{
    let s=0;
    for(let i=0;i<4;i++){
      const [x,y]=local[i];
      const px=pose.t[0]+pose.R[0][0]*x+pose.R[1][0]*y;
      const py=pose.t[1]+pose.R[0][1]*x+pose.R[1][1]*y;
      const pz=pose.t[2]+pose.R[0][2]*x+pose.R[1][2]*y;
      const p=projectCameraPoint(px,py,pz,Krad);
      s+=(p.x-noisyCorners[i].x)**2+(p.y-noisyCorners[i].y)**2;
    }
    return s;
  };
  const baseC=costDist(noisySolved.pose),eps=1e-6;
  for(let j=0;j<6;j++){
    const sp=[0,0,0,0,0,0];sp[j]=eps;
    const sm=[0,0,0,0,0,0];sm[j]=-eps;
    const grad=Math.abs(costDist(incrementPose(noisySolved.pose,sp))-costDist(incrementPose(noisySolved.pose,sm)))/(2*eps);
    assert.ok(grad<1e-4,`distorted Jacobian gradient along dim ${j} too high: ${grad}`);
  }

  // Border-tag pitfall check: raw corners inside [0..width, 0..height] whose undistorted corners exceed sensor edge must still succeed.
  const borderGL=vectorRigid([0.05,-0.05,0,0.365,-0.28,-0.50]);
  const borderCorners=local.map(([x,y])=>{
    const wx=borderGL[0]*x+borderGL[4]*y+borderGL[12];
    const wy=borderGL[1]*x+borderGL[5]*y+borderGL[13];
    const wz=borderGL[2]*x+borderGL[6]*y+borderGL[14];
    return projectCameraPoint(wx,-wy,-wz,Krad);
  });
  assert.ok(borderCorners.every(p=>p.x>=0&&p.x<Krad.width&&p.y>=0&&p.y<Krad.height));
  assert.ok(borderCorners.some(p=>{const u=undistortPoint(p.x,p.y,Krad);return u.x>=Krad.width||u.y>=Krad.height;}));
  const borderSolved=solveSingleTagPose({id:305,corners:borderCorners},Krad,{maxRmsPx:0.5});
  assert.ok(borderSolved);
  assert.ok(borderSolved.rmsPx<1e-9);
});
