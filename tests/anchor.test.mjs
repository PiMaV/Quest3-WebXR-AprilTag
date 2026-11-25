import test from 'node:test';
import assert from 'node:assert/strict';
import {Matrix4,Quaternion,Vector3} from '../vendor/three/three.module.js';
import {createStableLCollector,createLAnchorController,isRigidPose} from '../src/anchor.mjs';
const ctx={sessionId:'G-1',spaceEpoch:0};
const I=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,-.8,1];
function input(i,{ids=[350,351,352],offset=0,angle=0,context=ctx}={}) {
  const at=1000+350*i,m=new Matrix4().makeRotationY(angle);m.setPosition(.001*Math.sin(i)+offset,0,-.8);
  return {sample:{frameEvidence:{fresh:true},serial:i,generation:1,context:{...context},submittedAtPerformanceMs:at,videoFrameMetadata:{captureTime:at-20}},
    tool:{tagIds:ids,mode:ids.length>1?'joint-rigid-tags':'single-tag',pointCount:ids.length*4,reprojectionRmsPx:.3},
    candidate:{...context,association:'validated-profile-at-capture-time',calibrationProfileId:'frozen-profile'},worldFromL:m.toArray()};
}
const at=i=>1000+350*i+200;
function fixture(options={}) {
  const events=[],session={enabledFeatures:['anchors']},reference={},native={anchorSpace:{},deleted:0,delete(){this.deleted++;}};
  let n=0,hasPose=true,nativeMatrix=I.slice(),created=0;
  const controller=createLAnchorController({onEvent:(type,detail)=>events.push({type,detail}),onCreated:()=>created++,makeRigidTransform:(position,orientation)=>({position,orientation}),clock:()=>at(4),...options});
  const frame={session,createAnchor(rigid,space){assert.equal(space,reference);n++;return Promise.resolve(native);},getPose(space,ref){assert.equal(space,native.anchorSpace);assert.equal(ref,reference);return hasPose?{transform:{matrix:nativeMatrix},emulatedPosition:false}:null;}};
  controller.begin(session,reference,ctx,900);
  return {controller,events,session,reference,native,frame,get attempts(){return n;},get created(){return created;},
    set hasPose(x){hasPose=x;},set matrix(x){nativeMatrix=x;}};
}
function seed(f,start=0,context=ctx) {for(let i=start;i<start+5;i++) f.controller.observe(input(i,{context}),at(i));}
const flush=()=>new Promise(resolve=>setImmediate(resolve));

test('generic fixed rig IDs override the default 350/351/352 IDs',()=>{
  const c=createStableLCollector({tagIds:[10,20,30]});let state;
  for(let i=0;i<5;i++)state=c.observe(input(i,{ids:[10,20,30]}),ctx,at(i));
  assert.equal(state.ready,true);
  const wrong=createStableLCollector({tagIds:[10,20,30]});
  assert.equal(wrong.observe(input(0),ctx,at(0)).count,0);
});

test('five distinct, consistent three-tag views yield a rigid seed; quaternion signs stay continuous',()=>{
  const c=createStableLCollector();let state;
  for(let i=0;i<5;i++)state=c.observe(input(i,{angle:(i%2?179.8:180.2)*Math.PI/180}),ctx,at(i));
  assert.equal(state.ready,true);assert.equal(state.count,5);assert.ok(isRigidPose(state.quality.worldFromL));
  const q=new Quaternion().setFromRotationMatrix(new Matrix4().fromArray(state.quality.worldFromL));
  assert.ok(q.angleTo(new Quaternion().setFromAxisAngle(new Vector3(0,1,0),Math.PI))<.01);
  assert.ok(state.quality.positionSpreadMm<2);assert.deepEqual(state.quality.serials,[0,1,2,3,4]);
});
test('single tags, pairs, blur, duplicates, old poses and a new stream cannot bypass the quality gate',()=>{
  const c=createStableLCollector();
  for(const ids of [[351],[350,351]]) for(let i=0;i<6;i++) assert.equal(c.observe(input(i,{ids}),ctx,at(i)).ready,false);
  const x=input(0);c.observe(x,ctx,at(0));assert.equal(c.observe(x,ctx,at(0)).count,1);
  const blur=input(1);blur.tool.reprojectionRmsPx=2;assert.equal(c.observe(blur,ctx,at(1)).count,0);
  assert.equal(c.observe(input(0),ctx,at(0)+1000).count,0);
  for(let i=0;i<4;i++)c.observe(input(i),ctx,at(i));
  const next=input(4);next.sample.generation=2;assert.equal(c.observe(next,ctx,at(4)).count,1);
  const wrong=input(5,{context:{sessionId:'old',spaceEpoch:0}});assert.equal(c.observe(wrong,ctx,at(5)).count,0);
});
test('a displaced or rotated observation prevents a seed until a fresh consistent window replaces it',()=>{
  for(const bad of [{offset:.08},{angle:.15}]) {
    const c=createStableLCollector();for(let i=0;i<4;i++)c.observe(input(i),ctx,at(i));
    assert.equal(c.observe(input(4,bad),ctx,at(4)).ready,false);
    for(let i=5;i<9;i++)assert.equal(c.observe(input(i),ctx,at(i)).ready,false);
    assert.equal(c.observe(input(9),ctx,at(9)).ready,true);
  }
});
test('capability rejection never pretends that a real anchor exists',()=>{
  for(const unavailable of ['feature','method']) {
    const f=fixture();seed(f);
    if(unavailable==='feature')f.session.enabledFeatures=['local'];else delete f.frame.createAnchor;
    assert.equal(f.controller.update(f.frame,at(4)),null);
    assert.equal(f.controller.state().phase,'unsupported');assert.equal(f.controller.state().created,false);
    assert.equal(f.created,0);assert.equal(f.attempts,0);
  }
});
test('native anchor is created once, owns the world pose, hides on loss and recovers',async()=>{
  const f=fixture();seed(f);assert.equal(f.controller.update(f.frame,at(4)),null);await flush();
  assert.equal(f.attempts,1);assert.equal(f.created,1);assert.equal(f.controller.state().created,true);assert.equal(f.controller.state().createdAtPerformanceMs,at(4));
  const moved=I.slice();moved[12]=.12;f.matrix=moved;
  assert.deepEqual(f.controller.update(f.frame,at(5)),moved,'must use native pose rather than the seed');
  f.hasPose=false;assert.equal(f.controller.update(f.frame,at(6)),null);assert.equal(f.controller.state().phase,'lost');
  f.hasPose=true;assert.deepEqual(f.controller.update(f.frame,at(7)),moved);
  assert.equal(f.controller.state().phase,'tracking');assert.equal(f.attempts,1);
  assert.equal(f.controller.update(f.frame,at(8),{viewerTracked:false}),null);
  f.controller.end(at(9));assert.equal(f.native.deleted,1);assert.equal(f.controller.state().created,true);
  f.controller.end(at(10));assert.equal(f.native.deleted,1);
});
test('late completion after exit or space reset is deleted and cannot revive an old candidate',async()=>{
  for(const mode of ['end','reset']) {
    let finish;const f=fixture();f.frame.createAnchor=()=>new Promise(resolve=>{finish=resolve;});seed(f);f.controller.update(f.frame,at(4));
    if(mode==='end')f.controller.end(at(5));else f.controller.resetReference(f.reference,{...ctx,spaceEpoch:1},at(5));
    finish(f.native);await flush();assert.equal(f.native.deleted,1);assert.equal(f.created,0);
    assert.equal(f.controller.state().active,false);
  }
});
test('an already created native anchor survives a reference reset and is located in the current space',async()=>{
  const f=fixture();seed(f);f.controller.update(f.frame,at(4));await flush();
  f.controller.resetReference(f.reference,{...ctx,spaceEpoch:1},at(5));
  assert.deepEqual(f.controller.update(f.frame,at(6)),I);assert.equal(f.native.deleted,0);assert.equal(f.attempts,1);
});
test('rejected or stalled creation remains a reported failure, with no automatic retry or fake success',async()=>{
  for(const mode of ['reject','throw','timeout']) {
    let finish;const f=fixture({creationTimeoutMs:500});
    f.frame.createAnchor=()=>{if(mode==='throw')throw new Error('synthetic failure');if(mode==='reject')return Promise.reject(new Error('synthetic failure'));return new Promise(resolve=>{finish=resolve;});};
    seed(f);f.controller.update(f.frame,at(4));await flush();
    if(mode==='timeout'){f.controller.update(f.frame,at(4)+600);finish(f.native);await flush();assert.equal(f.native.deleted,1);}
    assert.equal(f.controller.state().phase,'failed');assert.equal(f.controller.state().created,false);assert.equal(f.created,0);
    assert.equal(f.controller.update(f.frame,at(8)),null);
  }
});
test('freshness and rigid-matrix checks prevent stale or malformed placement and tracking',async()=>{
  const f=fixture();seed(f);f.controller.update(f.frame,at(4)+1000);assert.equal(f.controller.state().phase,'collecting');assert.equal(f.attempts,0);
  const shear=I.slice();shear[4]=.2;assert.equal(isRigidPose(shear),false);
  const reflected=I.slice();reflected[0]=-1;assert.equal(isRigidPose(reflected),false);
  const x=input(0);x.worldFromL=shear;assert.equal(createStableLCollector().observe(x,ctx,at(0)).ready,false);
  const g=fixture();seed(g);g.controller.update(g.frame,at(4));await flush();g.matrix=shear;assert.equal(g.controller.update(g.frame,at(5)),null);
});
test('explicit reanchor deletes old native anchor, collects fresh views with no fallback and creates a new anchor',async()=>{
  const f=fixture();seed(f);f.controller.update(f.frame,at(4));await flush();
  assert.equal(f.attempts,1);assert.equal(f.native.deleted,0);
  assert.deepEqual(f.controller.update(f.frame,at(5)),I);
  assert.equal(f.controller.reanchor(at(6),'trigger'),true);
  assert.equal(f.native.deleted,1);
  assert.equal(f.controller.state().phase,'collecting');
  assert.equal(f.controller.state().created,false);
  assert.equal(f.controller.update(f.frame,at(7)),null,'must never fall back to old pose during reanchor');
  seed(f,10);
  assert.equal(f.controller.update(f.frame,at(14)),null);await flush();
  assert.equal(f.attempts,2);assert.equal(f.created,2);
  assert.deepEqual(f.controller.update(f.frame,at(15)),I);
});
test('collector rejects explicitly frozen pixels even when frameEvidence.fresh is true',()=>{
  const c=createStableLCollector();
  const frozen=input(0);frozen.sample.frameEvidence={fresh:true,pixelsChanged:false};
  assert.equal(c.observe(frozen,ctx,at(0)).count,0);
});
test('setTagIds switches anchor collector from 350/351/352 to tablet/e-Ink 370/371/372 cleanly',()=>{
  const f=fixture();
  f.controller.setTagIds([370,371,372]);
  assert.deepEqual(f.controller.state().quality.tagIds,[370,371,372]);
  // Default 350/351/352 frames are rejected once switched to 370/371/372.
  assert.equal(f.controller.observe(input(0),at(0)).quality.count,0);
  // 5 consistent 370/371/372 frames reach ready.
  for(let i=1;i<=5;i++){
    const obs=input(i);obs.tool.tagIds=[370,371,372];
    f.controller.observe(obs,at(i));
  }
  assert.equal(f.controller.state().phase,'ready');
  assert.equal(f.controller.state().quality.ready,true);
});
test('softReject skips momentary blur or partial-tag frames without wiping collected samples',()=>{
  const c=createStableLCollector({softReject:true});
  for(let i=0;i<3;i++) c.observe(input(i),ctx,at(i));
  assert.equal(c.state().count,3);
  const blur=input(3);blur.tool.reprojectionRmsPx=2.2;
  const afterBlur=c.observe(blur,ctx,at(3));
  assert.equal(afterBlur.count,3,'transient blur must not wipe prior valid samples');
  assert.equal(afterBlur.ready,false);
  const partial=input(5,{ids:[350,351]});
  const afterPartial=c.observe(partial,ctx,at(5));
  assert.equal(afterPartial.seenTagCount,2);
  // Real gap beyond maxGapMs (1050 ms > 700 ms) still expires old samples.
  assert.equal(afterPartial.count,0);
  // Fresh consistent sequence with one skipped blur reaches ready cleanly.
  for(let i=10;i<13;i++) c.observe(input(i),ctx,at(i));
  const briefBlur=input(13);briefBlur.tool.reprojectionRmsPx=1.9;
  c.observe(briefBlur,ctx,at(13));
  assert.equal(c.state().count,3);
  c.observe(input(14),ctx,at(14));
  const fin=c.observe(input(15),ctx,at(15));
  assert.equal(fin.ready,true);
});

test('fast 8 Hz observations satisfy minSpanMs without deadlocking at minSamples and drop stale outliers',()=>{
  const c=createStableLCollector();
  const makeAt=(idx,atMs,{offset=0}={})=>{
    const m=new Matrix4();m.setPosition(offset,0,-0.8);
    return {
      sample:{frameEvidence:{fresh:true},serial:idx,generation:1,context:{...ctx},submittedAtPerformanceMs:atMs,videoFrameMetadata:{captureTime:atMs-20}},
      tool:{tagIds:[350,351,352],mode:'joint-rigid-tags',pointCount:12,reprojectionRmsPx:0.25},
      candidate:{...ctx,association:'validated-profile-at-capture-time',calibrationProfileId:'fast-prof'},
      worldFromL:m.toArray(),
    };
  };
  let st;
  for(let i=0;i<=4;i++) st=c.observe(makeAt(i,1000+i*125),ctx,1000+i*125+15);
  assert.equal(st.count,5);
  assert.equal(st.spanMs,500);
  assert.equal(st.ready,false);
  for(let i=5;i<=8;i++) st=c.observe(makeAt(i,1000+i*125),ctx,1000+i*125+15);
  assert.equal(st.ready,true);
  assert.equal(st.quality.spanMs,1000);
  assert.ok(st.count>=5);

  // Displaced sample at t=0 followed by steady 8 Hz stream until trailing >=1000 ms clears the outlier.
  c.reset();
  st=c.observe(makeAt(100,2000,{offset:0.08}),ctx,2015);
  assert.equal(st.ready,false);
  for(let i=1;i<=8;i++) st=c.observe(makeAt(100+i,2000+i*125),ctx,2000+i*125+15);
  assert.equal(st.ready,false);
  st=c.observe(makeAt(109,3125),ctx,3140);
  assert.equal(st.ready,true);
  assert.ok(st.quality.positionSpreadMm<1);
});
