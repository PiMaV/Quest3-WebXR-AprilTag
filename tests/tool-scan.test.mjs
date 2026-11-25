import test from 'node:test';import assert from 'node:assert/strict';
import {createToolScan,validateToolSpecs,validateToolLayout,createToolLayoutStore,TOOL_SCAN_LIMITS,VISUAL_TOOL_SCAN_LIMITS} from '../src/tool-scan.mjs';
import {product,inverse,rigidVector} from '../src/runtime.mjs';
import {fixture,toolSpec} from './tool-fixture.mjs';
const make=()=>createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools'});
function observe(scan,f,id=353){return scan.observe({...f.observation,unassignedDetections:f.unassignedDetections.filter(t=>t.id===id)},f.sample,f.association,f.now);}
test('a tilted elevated plate needs fresh multi-view evidence and explicit acceptance; retains full 6DOF',()=>{
  const f=fixture(),s=make();s.begin();let state,last;
  for(let i=0;i<20;i++){last=f.frame(i);state=observe(s,last);if(state.tools[0].candidate.ready)break;}
  assert.equal(state.tools[0].candidate.ready,true);assert.equal(s.layout().tools.length,0);
  const accepted=s.accept(last.now,f.profile.id);assert.equal(accepted.tagId,353);
  const e=rigidVector(product(inverse(f.transforms[0]),accepted.lFromTool));assert.ok(Math.hypot(...e)<1e-5);
  assert.ok(accepted.lFromTool[14]>.09,'must not force the loose plate onto the L plane');
  assert.equal(s.layout().tools.length,1);assert.equal(s.layout().worldFromAnchor,undefined);
  assert.equal(s.accept(last.now+1000),null,'stale placement cannot be confirmed');
});
test('stationary views and frozen pixels cannot pretend to give viewpoint diversity',()=>{
  const f=fixture();
  for(const kind of ['stationary','frozen']){const s=make();s.begin();
    for(let i=0;i<20;i++){const a=f.frame(i,{stationary:kind==='stationary',noise:.002});if(kind==='frozen')a.observation.unassignedDetections=a.unassignedDetections=f.frame(0).unassignedDetections;observe(s,a);}
    assert.equal(s.state(f.frame(19).now).tools[0].candidate.ready,false);assert.equal(s.layout().tools.length,0);
  }
});
test('scope changes, duplicate IDs and invalid pixels cannot bypass capture gates',()=>{
  const f=fixture(),s=make();s.begin();for(let i=0;i<8;i++)observe(s,f.frame(i));
  const next=f.frame(8,{generation:2});observe(s,next);assert.equal(s.state(next.now).tools[0].candidate.count,1);
  const duplicate=f.frame(9,{generation:2});duplicate.unassignedDetections.push(duplicate.unassignedDetections[0]);observe(s,duplicate);
  assert.equal(s.state(duplicate.now).selectedTagId,null);
  const wrong=f.frame(10,{generation:2});wrong.association.candidate.sessionId='old';observe(s,wrong);assert.equal(s.state(wrong.now).selectedTagId,null);
  const tiny=f.frame(11,{generation:2});tiny.unassignedDetections[0].corners=[{x:1,y:1},{x:2,y:1},{x:2,y:2},{x:1,y:2}];observe(s,tiny);assert.equal(s.state(tiny.now).selectedTagId,null);
});
test('one displaced view is rejected instead of dragging an otherwise consistent registration',()=>{
  const f=fixture(),s=make();s.begin();let last,state;
  for(let i=0;i<22;i++){last=f.frame(i,{shift:i===5?80:0});state=observe(s,last);if(state.tools[0].candidate.ready)break;}
  assert.equal(state.tools[0].candidate.ready,true);assert.ok(state.tools[0].candidate.quality.rejectedSamples>=1);
  const result=s.accept(last.now);assert.ok(Math.hypot(...rigidVector(product(inverse(f.transforms[0]),result.lFromTool)))<1e-5);
  s.resetPending();assert.equal(s.layout().tools.length,1);assert.equal(s.state(last.now).selectedTagId,null);
});
test('layout geometry, units, identity and rigid poses are checked before cache replacement',async()=>{
  const f=fixture(),s=make();s.begin();let last;
  for(let i=0;i<20;i++){last=f.frame(i);if(observe(s,last).tools[0].candidate.ready)break;}s.accept(last.now);
  const layout=s.layout(),restored=make();restored.loadLayout(layout);assert.deepEqual(restored.layout().tools,layout.tools);
  const ctx={spec:toolSpec,rigHash:'rig',toolsHash:'tools'},data=new Map(),store=createToolLayoutStore(ctx,{getItem:k=>data.get(k)??null,setItem:(k,v)=>data.set(k,v)});
  await store.save(layout);assert.equal((await store.load()).tools.length,1);
  for(const bad of [{...layout,units:'mm'},{...layout,rigHash:'other'},{...layout,tools:[...layout.tools,...layout.tools]}]){
    assert.throws(()=>validateToolLayout(bad,ctx));await assert.rejects(store.save(bad));
  }
  assert.equal((await store.load()).tools.length,1);
  const shear=structuredClone(layout);shear.tools[0].lFromTool[4]+=.3;assert.throws(()=>validateToolLayout(shear,ctx));
  const duplicate=structuredClone(toolSpec);duplicate.tools[1].tagId=duplicate.tools[0].tagId;assert.throws(()=>validateToolSpecs(duplicate));
});
test('visual policy is explicit and cannot register without user intent',()=>{
  assert.equal(TOOL_SCAN_LIMITS.maxPositionSpreadM,.006);assert.equal(TOOL_SCAN_LIMITS.maxOrientationSpreadDeg,2);
  const f=fixture(),s=createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools',limits:VISUAL_TOOL_SCAN_LIMITS});s.begin();
  let last;for(let i=0;i<18;i++){last=f.frame(i);if(observe(s,last).tools[0].candidate.ready)break;}
  assert.equal(s.state(last.now).tools[0].candidate.ready,true);assert.equal(s.layout().tools.length,0);
  const saved=s.accept(last.now,f.profile.id);assert.equal(saved.quality.registrationMode,'visual');
});
test('early explicit intent completes on fresh ready views once; repeated events preserve the pose',()=>{
  const f=fixture(),accepted=[],s=createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools',limits:VISUAL_TOOL_SCAN_LIMITS,onAccepted:e=>accepted.push(e)});s.begin();
  const first=f.frame(0);observe(s,first);assert.equal(s.requestCapture(first.now,f.profile.id).status,'pending');
  for(let i=1;i<18&&!s.layout().tools.length;i++)observe(s,f.frame(i));
  assert.equal(accepted.length,1);const original=s.layout().tools[0];assert.equal(original.tagId,353);
  assert.equal(s.requestCapture(first.now+3000,f.profile.id).status,'already-accepted');
  assert.equal(s.accept(first.now+3000,f.profile.id),null);
  for(let i=18;i<25;i++)observe(s,f.frame(i,{shift:2}));
  assert.deepEqual(s.layout().tools[0],original);assert.equal(accepted.length,1);
  s.resetPending();assert.equal(s.state(first.now+8500).captureRequest,null);assert.equal(s.layout().tools.length,1);
  assert.equal(s.reopen(353),true);assert.equal(s.layout().tools.length,0);assert.equal(s.reopen(999),false);
});
test('pending intent never registers a different visible ID and expires without an automatic placement',()=>{
  const f=fixture(),s=createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools',limits:{...VISUAL_TOOL_SCAN_LIMITS,captureRequestMs:1000}});s.begin();
  const first=f.frame(0);observe(s,first);s.requestCapture(first.now,f.profile.id);
  for(let i=1;i<18;i++)observe(s,f.frame(i),354);
  assert.equal(s.state(f.frame(17).now).captureRequest,null);assert.equal(s.layout().tools.length,0);
  for(let i=18;i<30;i++)observe(s,f.frame(i));
  assert.equal(s.layout().tools.length,0,'expired intent must not revive when requested tag returns');
});
test('scope changes, frozen images and missing tracking cancel or block buffered capture',()=>{
  const f=fixture();
  for(const mode of ['generation','epoch','frozen','tracking']){
    const s=createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools',limits:VISUAL_TOOL_SCAN_LIMITS});s.begin();
    const first=f.frame(0);observe(s,first);s.requestCapture(first.now,f.profile.id);
    for(let i=1;i<18;i++){
      const a=f.frame(i,{generation:mode==='generation'?2:1,epoch:mode==='epoch'?1:0});
      if(mode==='frozen')a.sample.frameEvidence.fresh=false;
      if(mode==='tracking'){s.resetPending();a.association=null;}
      observe(s,a);
    }
    assert.equal(s.layout().tools.length,0,mode);
  }
});
test('loading a partial layout keeps its accepted entry and only adds missing plates',()=>{
  const f=fixture(),s=make();s.begin();
  const saved={id:toolSpec.tools[2].id,tagId:355,role:toolSpec.tools[2].role,lFromTool:f.transforms[2].slice(),capturedWithProfileId:f.profile.id};
  const restored=createToolScan({spec:toolSpec,rigHash:'rig',toolsHash:'tools',limits:VISUAL_TOOL_SCAN_LIMITS});
  restored.loadLayout({...s.layout(),tools:[saved]});
  for(const [generation,id] of [[1,353],[2,354]]){
    let requested=false;
    for(let i=0;i<30;i++){
      const a=f.frame(i,{generation}),state=observe(restored,a,id);
      if(!requested&&state.selectedTagId===id){restored.requestCapture(a.now,f.profile.id);requested=true;}
    }
  }
  assert.deepEqual(restored.layout().tools.find(t=>t.tagId===355),saved);assert.equal(restored.layout().tools.length,3);
});
