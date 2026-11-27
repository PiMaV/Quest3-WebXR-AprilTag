import test from 'node:test';import assert from 'node:assert/strict';
import {associateAnchorCapture,auditAnchorObservation,createAnchorAudit} from '../src/anchor-audit.mjs';
import {fitPlanarPose} from '../src/rig.mjs';
import {fixture,rig} from './tool-fixture.mjs';
function observation(f){const tags=rig.tools[0].tags.map(t=>({id:t.id,points:t.corners,pixels:f.project(t.corners,f.association.worldFromAnchor)}));
  return {tags,cameraModel:f.K,tools:[{id:'origin-l',tagIds:tags.map(t=>t.id),cameraFromTool:fitPlanarPose(tags,f.K),reprojectionRmsPx:0}]};}
test('optical audit compares capture-time native pose without altering it',()=>{
  const data=fixture(),f=data.frame(5),assoc=associateAnchorCapture(f.history,f.sample,data.profile,f.now);assert.ok(assoc);
  const original=structuredClone(assoc),obs=observation(f),r=auditAnchorObservation(obs,f.sample,assoc,rig);
  assert.ok(r.positionDifferenceMm<1e-6);assert.ok(r.nativePredictionRmsPx<1e-6);assert.equal(r.usedToCorrectAnchor,false);assert.deepEqual(assoc,original);
  const moved=structuredClone(assoc);moved.worldFromAnchor[12]+=.01;const bad=auditAnchorObservation(obs,f.sample,moved,rig);
  assert.ok(Math.abs(bad.positionDifferenceMm-10)<1e-5);assert.ok(bad.nativePredictionRmsPx>1);
  const audit=createAnchorAudit();assert.equal(audit.observe(r),true);assert.equal(audit.observe(r),false);assert.equal(audit.state().samples,1);assert.equal(audit.state().usedToCorrectAnchor,false);
});
test('capture association refuses missing native history, lost tracking and a different reference-space epoch',()=>{
  const d=fixture(),f=d.frame(5);assert.equal(associateAnchorCapture(f.history.map(p=>({...p,spaceEpoch:1})),f.sample,d.profile,f.now),null);
  assert.equal(associateAnchorCapture(f.history.map(p=>({...p,anchorTrackingState:'lost'})),f.sample,d.profile,f.now),null);
  assert.equal(associateAnchorCapture(f.history,{...f.sample,settings:{experimentalCalibration:{id:'other'}}},d.profile,f.now),null);
  assert.equal(associateAnchorCapture(f.history,f.sample,d.profile,f.now+1000),null);
});

test('optical audit supports radial distortion k1 and retains recentRows for offline analysis',()=>{
  const data=fixture(),f=data.frame(5),assoc=associateAnchorCapture(f.history,f.sample,data.profile,f.now);
  assert.ok(assoc);
  const k1=-0.08;
  const K={...f.K,k1};
  const distort=p=>{
    const xn=(p.x-K.cx)/K.fx,yn=(p.y-K.cy)/K.fy,sc=1+k1*(xn*xn+yn*yn);
    return {x:K.fx*xn*sc+K.cx,y:K.fy*yn*sc+K.cy};
  };
  const tags=rig.tools[0].tags.map(t=>({
    id:t.id,points:t.corners,
    pixels:f.project(t.corners,f.association.worldFromAnchor).map(distort),
  }));
  const obs={tags,cameraModel:K,tools:[{id:'origin-l',tagIds:tags.map(t=>t.id),cameraFromTool:fitPlanarPose(tags,K),reprojectionRmsPx:0}]};
  const row=auditAnchorObservation(obs,f.sample,assoc,rig);
  assert.ok(row&&row.nativePredictionRmsPx<1e-6,`expected <1e-6, got ${row?.nativePredictionRmsPx}`);
  const audit=createAnchorAudit();
  assert.equal(audit.observe(row),true);
  const recent=audit.recentRows(5);
  assert.equal(recent.length,1);
  assert.equal(recent[0].projections.length,3);
});
