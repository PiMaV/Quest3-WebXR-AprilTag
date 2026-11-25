import test from 'node:test';
import assert from 'node:assert/strict';
import {createCalibrationAssistant} from '../src/assistant.mjs';
import {solveCalibration} from '../src/solver.mjs';
import {deviceIdentity,createProfileStore} from '../src/profile.mjs';
import {synthetic,rig} from './synthetic.mjs';
const wait=()=>new Promise(r=>setTimeout(r,0));
function worker(){return {terminate(){this.closed=true;},postMessage(data){queueMicrotask(()=>{
  const result=solveCalibration(data.payload);if(!this.closed)this.onmessage({data:{id:data.id,type:'result',result}});
});}};}
async function fixture(){
  const data=synthetic({count:80}),memory=new Map(),store=createProfileStore({getItem:k=>memory.get(k)??null,setItem:(k,v)=>memory.set(k,v)});
  const device=await deviceIdentity({trackLabel:'Synthetic',sourceSize:[1280,1080],trackSettings:{deviceId:'A',resizeMode:'none'}},rig,'Test');
  const a=createCalibrationAssistant({rig,device,imageSize:[1280,1080],store,workerFactory:worker});
  const feed=async(row,{generation=1,epoch=0,duplicate=false,latency=350}={})=>{
    const sample={serial:row.serial,generation,context:{sessionId:'synthetic',spaceEpoch:epoch},imageSize:[1280,1080],sourceSize:[1280,1080],
      submittedAtPerformanceMs:row.captureTime+Math.min(80,latency/2),videoFrameMetadata:{captureTime:row.captureTime},frameEvidence:{fresh:true}};
    const now=row.captureTime+latency,history=data.history.filter(p=>p.xrRafTimeMs<=now);
    a.observe({tags:row.tags},sample,history,now);await wait();return sample;
  };
  return {data,store,device,a,feed,memory};
}
test('cold start fits, verifies newly captured views, persists, then checks cache next session',async()=>{
  const {a,feed,data,store,device}=await fixture();await a.begin();
  for(const row of data.rows){await feed(row);if(a.state().phase==='ready')break;}
  assert.equal(a.state().phase,'ready');assert.equal(a.state().persisted,true);
  assert.equal(a.profile().verification.kind,'new-capture-after-fit');assert.ok(await store.load(device));
  assert.equal(a.profile().validation.reference,undefined);assert.equal(a.profile().verification.reference,undefined);
  assert.equal(a.profile().worldFromRig,undefined);
  await a.begin();assert.equal(a.state().phase,'checking');
  for(const row of data.rows.slice(0,30)){await feed(row);if(a.state().phase==='ready')break;}
  assert.equal(a.state().phase,'ready');assert.equal(a.profile().verification.kind,'new-session-check');
});
test('frozen pixels do not progress; failed cache verification preserves the saved profile',async()=>{
  const {a,feed,data,store,device}=await fixture();await a.begin();
  for(let i=0;i<35;i++)await feed({...data.rows[i],tags:data.rows[0].tags});assert.equal(a.state().count,1);
  a.reset();await a.begin();for(const row of data.rows){await feed(row);if(a.state().phase==='ready')break;}
  const good=await store.load(device);await a.begin();
  for(const row of data.rows.slice(0,30)){
    const tags=structuredClone(row.tags);for(const t of tags)for(const p of t.pixels)p.x+=Math.sin(row.serial)*80;
    await feed({...row,tags});
  }
  assert.equal(a.state().phase,'check-failed');assert.equal((await store.load(device)).id,good.id);assert.equal(a.profile(),null);
  assert.ok(a.state().checkFailure);assert.match(a.state().message,/Profile loaded/);
  assert.equal(a.retryCheck(),true);assert.equal(a.state().phase,'checking');
  for(const row of data.rows.slice(0,30)){await feed(row);if(a.state().phase==='ready')break;}
  assert.equal(a.state().phase,'ready');a.retry();assert.equal(a.state().phase,'scanning');
});
test('fast camera results are retained until their future XR search horizon is available',async()=>{
  const {a,feed,data}=await fixture();await a.begin();
  for(const row of data.rows){await feed(row,{latency:45});if(a.state().phase==='ready')break;}
  assert.equal(a.state().phase,'ready');assert.equal(a.state().persisted,true);
});
test('reset cancels a pending fit and ignores late worker completion',async()=>{
  const {data,device,store}=await fixture();let pending;
  const a=createCalibrationAssistant({rig,device,imageSize:[1280,1080],store,workerFactory:()=>pending={postMessage(){},terminate(){this.closed=true;}}});
  await a.begin();for(const r of data.rows.slice(0,60))a.observe({tags:r.tags},{serial:r.serial,generation:1,context:{sessionId:'synthetic',spaceEpoch:0},
    imageSize:[1280,1080],sourceSize:[1280,1080],submittedAtPerformanceMs:r.captureTime+80,videoFrameMetadata:{captureTime:r.captureTime},frameEvidence:{fresh:true}},data.history,r.captureTime+350);
  assert.equal(a.state().phase,'solving');a.reset();assert.equal(pending.closed,true);
  pending.onmessage({data:{id:2,type:'result',result:solveCalibration(data)}});assert.equal(a.state().phase,'idle');assert.equal(await store.load(device),null);
});
test('verification tolerates minor negative clock skew, rejects rigid outliers and accepts out-and-back motion',async()=>{
  const {a,feed,data,store,device}=await fixture();await a.begin();
  for(const row of data.rows){await feed(row);if(a.state().phase==='ready')break;}
  assert.equal(a.state().phase,'ready');
  await a.begin();assert.equal(a.state().phase,'checking');
  // Outlier flagged by observeReference must not be accepted during calibrated checking.
  const r0=data.rows[0];
  a.observe({tags:r0.tags,rejected:[{id:350,reason:'rigid_tag_outlier'}]},{
    serial:900,generation:1,context:{sessionId:'synthetic',spaceEpoch:0},imageSize:[1280,1080],sourceSize:[1280,1080],
    submittedAtPerformanceMs:r0.captureTime+40,videoFrameMetadata:{captureTime:r0.captureTime},frameEvidence:{fresh:true},
  },data.history.filter(p=>p.xrRafTimeMs<=r0.captureTime+350),r0.captureTime+350);
  assert.equal(a.state().count,0);

  // Feed 15 views with minor -10 ms clock skew (captureTime 10 ms ahead of submittedAtPerformanceMs).
  for(const row of data.rows.slice(0,20)){
    const sample={serial:row.serial+1000,generation:1,context:{sessionId:'synthetic',spaceEpoch:0},imageSize:[1280,1080],sourceSize:[1280,1080],
      submittedAtPerformanceMs:row.captureTime-10,videoFrameMetadata:{captureTime:row.captureTime},frameEvidence:{fresh:true}};
    const now=row.captureTime+350;
    a.observe({tags:row.tags,rejected:[]},sample,data.history.filter(p=>p.xrRafTimeMs<=now),now);
    await wait();
    if(a.state().phase==='ready')break;
  }
  assert.equal(a.state().phase,'ready');
  assert.ok(await store.load(device));
});
