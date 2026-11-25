import test from 'node:test';
import assert from 'node:assert/strict';
import {solveCalibration,fitIntrinsics} from '../src/solver.mjs';
import {product,inverse,rigidVector} from '../src/runtime.mjs';
import {deviceIdentity,createProfileStore,PROFILE_SCHEMA,PIPELINE,calibratedCameraModel,extractCalibrationProfile,importCalibrationProfile,viewerPoseHorizon,createObservationBuffer} from '../src/profile.mjs';
import {synthetic,rig} from './synthetic.mjs';
for(const model of ['centered-square','pinhole'])test(`${model}: recovers camera, hand/eye and delay from diverse unseen poses`,()=>{
  const d=synthetic({pinhole:model==='pinhole'}),r=solveCalibration({...d,model});
  assert.equal(r.validation.passed,true);
  assert.ok(Math.abs(r.K.fx-d.truth.K.fx)<4);assert.ok(Math.abs(r.K.fy-d.truth.K.fy)<4);
  assert.ok(Math.abs(r.K.cx-d.truth.K.cx)<4);assert.ok(Math.abs(r.K.cy-d.truth.K.cy)<4);
  assert.ok(Math.abs(r.timeOffsetMs-d.truth.delay)<3);
  const e=rigidVector(product(inverse(d.truth.X),r.viewerFromCameraGL));
  assert.ok(Math.hypot(...e.slice(3))<.004);assert.ok(Math.hypot(...e.slice(0,3))<.006);
  assert.equal(r.independentlyValidated,false);
  assert.ok(r.convergedSeeds>=2);
});
test('rejects static-head, mixed-epoch and insufficient captures',()=>{
  const d=synthetic();assert.throws(()=>solveCalibration({...d,rows:d.rows.slice(0,15)}),/30/);
  assert.throws(()=>solveCalibration({...d,history:d.history.map(p=>({...p,worldFromViewer:d.history[0].worldFromViewer}))}),/axes/i);
  assert.throws(()=>solveCalibration({...d,rows:d.rows.map((p,i)=>({...p,scope:i===5?'other':'s'}))}),/changed/i);
});
test('different mode, physical camera or browser never silently shares a profile',async()=>{
  const c={trackLabel:'camera',trackSettings:{deviceId:'physical-A',resizeMode:'none'},sourceSize:[1280,1080]};
  const a=await deviceIdentity(c,rig,'Quest test');
  for(const changed of [{...c,sourceSize:[1080,1728]},{...c,trackSettings:{...c.trackSettings,deviceId:'physical-B'}}])
    assert.notEqual((await deviceIdentity(changed,rig,'Quest test')).key,a.key);
  assert.notEqual((await deviceIdentity(c,rig,'Quest updated')).key,a.key);
  const data=new Map(),storage={getItem:k=>data.get(k)??null,setItem:(k,v)=>data.set(k,v)},store=createProfileStore(storage);
  const fit=solveCalibration(synthetic()),p={...fit,schema:PROFILE_SCHEMA,id:'synthetic',device:a,imageSize:[1280,1080],pipeline:PIPELINE,verification:{passed:true}};
  await store.save(p);assert.equal((await store.load(a)).id,'synthetic');
  await assert.rejects(store.save({...p,verification:{passed:false}}));assert.equal((await store.load(a)).id,'synthetic');
  const entry=[...data.keys()][0];data.set(entry,data.get(entry).replace('synthetic','altered'));assert.equal(await store.load(a),null);
  assert.throws(()=>calibratedCameraModel(p,[1280,1080],[1080,1728],'camera','Quest test',{resizeMode:'none'}),/changed/i);
});
test('imports a standalone profile or recording wrapper only into the exact device/mode and preserves a good cache on rejection',async()=>{
  const device=await deviceIdentity({trackLabel:'camera',trackSettings:{deviceId:'A',resizeMode:'none'},sourceSize:[1280,1080]},rig,'Test');
  const profile={...solveCalibration(synthetic()),schema:PROFILE_SCHEMA,id:'good-profile',device,imageSize:[1280,1080],verification:{passed:true}};
  const memory=new Map(),store=createProfileStore({getItem:k=>memory.get(k)??null,setItem:(k,v)=>memory.set(k,v)});
  await importCalibrationProfile(JSON.stringify(profile),device,store);
  const recording={schema:'webxr-apriltag-recording-v1',calibrationAssistant:{profile}};
  assert.equal(extractCalibrationProfile(recording).id,profile.id);await importCalibrationProfile(recording,device,store);
  for(const change of [{...device,key:'different'},{...device,mode:{...device.mode,sourceSize:[1080,1728]}},{...device,userAgent:'different'}])
    await assert.rejects(importCalibrationProfile(recording,change,store),/different/i);
  assert.throws(()=>extractCalibrationProfile({schema:PROFILE_SCHEMA}),/validated/i);
  assert.throws(()=>extractCalibrationProfile({schema:'unknown-recording-v1',calibrationAssistant:{profile}}));
  assert.throws(()=>extractCalibrationProfile({...profile,verification:{passed:false}}));
  assert.equal((await store.load(device)).id,'good-profile');
});
test('fitHandEye survives an internal >100 ms XR history gap without crashing',()=>{
  const d=synthetic({count:72});
  // Drop a 150 ms chunk right in the middle of training history.
  const gapped=d.history.filter(p=>p.xrRafTimeMs<3000||p.xrRafTimeMs>3150);
  const r=solveCalibration({...d,history:gapped});
  assert.equal(r.validation.passed,true);
  assert.ok(r.usedViews<57&&r.usedViews>=24);
  assert.ok(Math.abs(r.timeOffsetMs-d.truth.delay)<3);
});
test('fitIntrinsics stays accurate under a single severe corner outlier via Pseudo-Huber IRLS',()=>{
  const d=synthetic({count:50});
  const corrupted=structuredClone(d.rows);
  corrupted[10].tags[0].pixels[0].x+=40;
  const fit=fitIntrinsics(corrupted,d.imageSize,{model:'centered-square'});
  assert.ok(Math.abs(fit.K.fx-d.truth.K.fx)<6,`fx drifted too far: ${fit.K.fx}`);
  assert.ok(Number.isFinite(fit.std[0])&&fit.std[0]>0);
});
test('viewerPoseHorizon and createObservationBuffer handle positive timeOffsetMs and bounded clock skew',()=>{
  const d=synthetic({count:10});
  const profile={id:'pos-offset',timeOffsetMs:60};
  // Sample captured at t=1000 with slight negative skew (submittedAt = 990) and +60 ms offset -> target 1060 ms.
  const sample={context:{sessionId:'synthetic',spaceEpoch:0},submittedAtPerformanceMs:990,videoFrameMetadata:{captureTime:1000}};
  const earlyHistory=d.history.filter(p=>p.xrRafTimeMs<=1030);
  assert.equal(viewerPoseHorizon(earlyHistory,sample,profile).status,'future');
  const buf=createObservationBuffer({maxAgeMs:650});
  buf.push({sample,tag:'ok'});
  assert.equal(buf.drain(earlyHistory,profile,1030).length,0);
  assert.equal(buf.size(),1);
  const readyHistory=d.history.filter(p=>p.xrRafTimeMs<=1080);
  const drained=buf.drain(readyHistory,profile,1080);
  assert.equal(drained.length,1);
  assert.equal(drained[0].candidate.targetXrTimeMs,1060);
  assert.equal(buf.size(),0);
});
