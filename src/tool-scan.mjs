import {fitPlanarPose} from './rig.mjs';
import {isRigidPose} from './anchor.mjs';
import {product,inverse,cvToGL,rigidVector,meanRigid,observationIsFresh} from './runtime.mjs';
import {digest} from './profile.mjs';

export const TOOL_SCAN_LIMITS=Object.freeze({minSamples:12,minSpanMs:3000,maxWindowMs:10000,maxGapMs:1000,maxAgeMs:650,
  minEdgePx:20,minAreaPx:400,maxRmsPx:1,maxPositionSpreadM:.006,maxOrientationSpreadDeg:2,
  minCameraBaselineM:.05,minViewAngleDeg:6,maxIncidenceDeg:70,outlierPositionM:.010,outlierOrientationDeg:3,minInlierRatio:.8,
  mode:'strict',captureRequestMs:6000});
// Explicitly a visual-registration policy. Report actual scatter; no metrology claim.
export const VISUAL_TOOL_SCAN_LIMITS=Object.freeze({...TOOL_SCAN_LIMITS,minSamples:8,minSpanMs:2000,maxWindowMs:6000,
  maxAgeMs:1000,maxPositionSpreadM:.015,maxOrientationSpreadDeg:4,minCameraBaselineM:.03,minViewAngleDeg:4,
  outlierPositionM:.020,outlierOrientationDeg:6,minInlierRatio:.65,captureRequestMs:12000,mode:'visual'});
const clone=v=>structuredClone(v);
const median=a=>{const s=a.slice().sort((x,y)=>x-y);return s[Math.floor(s.length/2)];};
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
function delta(a,b){const d=rigidVector(product(inverse(a),b));return {position:Math.hypot(...d.slice(3)),angle:Math.hypot(...d.slice(0,3))*180/Math.PI};}
export function validateToolSpecs(spec,{referenceTagIds=[350,351,352]}={}){
  if(spec?.schema!=='webxr-apriltag-independent-tools-v1'||spec.units!=='m'||spec.family!=='apriltag-36h11'||
    !Array.isArray(spec.tools)||!spec.tools.length||spec.tools.length>16)throw new Error('Invalid independent tool geometry');
  const ids=new Set(),names=new Set();
  for(const t of spec.tools){
    if(typeof t.id!=='string'||!t.id||names.has(t.id)||!Number.isInteger(t.tagId)||t.tagId<0||t.tagId>586||
      ids.has(t.tagId)||referenceTagIds.includes(t.tagId)||!Array.isArray(t.tagCorners)||t.tagCorners.length!==4||
      t.tagCorners.some(p=>p.length!==2||!p.every(Number.isFinite)))throw new Error('Invalid/duplicate tool identity');
    const c=t.tagCorners,e=c.map((p,i)=>distance(p,c[(i+1)%4])),a=c[1].map((v,i)=>v-c[0][i]),b=c[3].map((v,i)=>v-c[0][i]);
    if(e[0]<.001||e.some(v=>Math.abs(v-e[0])>1e-6)||Math.abs(a[0]*b[0]+a[1]*b[1])>1e-8||a[0]*b[1]-a[1]*b[0]<=0)
      throw new Error('Invalid tool square/corner order');
    names.add(t.id);ids.add(t.tagId);
  }
  return spec;
}
export function observeIndependentTool(raw,spec,K,association,limits=TOOL_SCAN_LIMITS){
  if(!K||![K.fx,K.fy,K.cx,K.cy,K.width,K.height].every(Number.isFinite)||K.fx<=0||K.fy<=0||
    !isRigidPose(association?.worldFromAnchor)||!isRigidPose(association?.worldFromCamera))return null;
  const c=raw?.corners;
  if(raw?.id!==spec.tagId||raw.family!=='apriltag-36h11'||!Array.isArray(c)||c.length!==4||
    c.some(p=>![p?.x,p?.y].every(Number.isFinite)||p.x<0||p.y<0||p.x>=K.width||p.y>=K.height))return null;
  const turns=c.map((p,i)=>{const q=c[(i+1)%4],r=c[(i+2)%4];return (q.x-p.x)*(r.y-q.y)-(q.y-p.y)*(r.x-q.x);});
  const edges=c.map((p,i)=>Math.hypot(p.x-c[(i+1)%4].x,p.y-c[(i+1)%4].y));
  const area=Math.abs(c.reduce((s,p,i)=>s+p.x*c[(i+1)%4].y-p.y*c[(i+1)%4].x,0))/2;
  if(!turns.every(v=>v<0)||Math.min(...edges)<limits.minEdgePx||area<limits.minAreaPx)return null;
  const tag={id:raw.id,points:spec.tagCorners,pixels:c},pose=fitPlanarPose([tag],K);if(!pose)return null;
  const incidence=-pose.R[2].reduce((s,v,i)=>s+v*pose.t[i],0)/Math.hypot(...pose.t);
  if(incidence<Math.cos(limits.maxIncidenceDeg*Math.PI/180))return null;
  const k1=Number.isFinite(K.k1)?K.k1:0;
  let sum=0;
  for(let i=0;i<4;i++){const [x,y]=spec.tagCorners[i],p=pose.t.map((v,j)=>v+pose.R[0][j]*x+pose.R[1][j]*y);
    if(p[2]<=.01)return null;
    const xn=p[0]/p[2],yn=p[1]/p[2],sc=k1?1+k1*(xn*xn+yn*yn):1;
    sum+=(K.fx*xn*sc+K.cx-c[i].x)**2+(K.fy*yn*sc+K.cy-c[i].y)**2;}
  const rms=Math.sqrt(sum/4);if(!Number.isFinite(rms)||rms>limits.maxRmsPx)return null;
  const lFromCamera=product(inverse(association.worldFromAnchor),association.worldFromCamera);
  const lFromTool=product(lFromCamera,cvToGL(pose));if(!isRigidPose(lFromTool))return null;
  return {id:spec.id,tagId:spec.tagId,lFromTool,cameraPositionL:lFromCamera.slice(12,15),reprojectionRmsPx:rms,areaPx:area,
    minEdgePx:Math.min(...edges),incidenceDeg:Math.acos(Math.max(-1,Math.min(1,incidence)))*180/Math.PI,
    pixels:clone(c)};
}
function summarize(rows,limits,now){
  if(!rows.length)return {count:0,required:limits.minSamples,ready:false,reason:'Look closer and keep the full tag in view'};
  if(now-rows.at(-1).at>limits.maxAgeMs)return {count:rows.length,required:limits.minSamples,ready:false,reason:'Fresh observation required'};
  const medoid=rows.reduce((best,r)=>{
    const cost=median(rows.map(s=>delta(r.lFromTool,s.lFromTool).angle));return !best||cost<best.cost?{r,cost}:best;
  },null).r.lFromTool.slice();
  for(const i of [12,13,14])medoid[i]=median(rows.map(r=>r.lFromTool[i]));
  const inliers=rows.filter(r=>{const d=delta(medoid,r.lFromTool);return d.position<=limits.outlierPositionM&&d.angle<=limits.outlierOrientationDeg;});
  if(!inliers.length)return {count:0,total:rows.length,required:limits.minSamples,ready:false,reason:'Views disagree · look again'};
  const pose=meanRigid(inliers.map(r=>r.lFromTool)),errors=inliers.map(r=>delta(pose,r.lFromTool));
  const spread=Math.max(...errors.map(d=>d.position)),angle=Math.max(...errors.map(d=>d.angle));
  let baseline=0,viewAngle=0;
  const centre=pose.slice(12,15),rays=inliers.map(r=>{const v=centre.map((x,i)=>x-r.cameraPositionL[i]),n=Math.hypot(...v);return v.map(x=>x/n);});
  for(let i=0;i<inliers.length;i++)for(let j=0;j<i;j++){
    baseline=Math.max(baseline,distance(inliers[i].cameraPositionL,inliers[j].cameraPositionL));
    viewAngle=Math.max(viewAngle,Math.acos(Math.min(1,Math.max(-1,rays[i].reduce((s,v,k)=>s+v*rays[j][k],0))))*180/Math.PI);
  }
  const span=inliers.at(-1).at-inliers[0].at;
  const enough=inliers.length>=limits.minSamples&&inliers.length>=Math.ceil(rows.length*limits.minInlierRatio)&&span>=limits.minSpanMs;
  const steady=spread<=limits.maxPositionSpreadM&&angle<=limits.maxOrientationSpreadDeg;
  const motion=baseline>=limits.minCameraBaselineM&&viewAngle>=limits.minViewAngleDeg;
  const ready=enough&&steady&&motion;
  return {count:inliers.length,total:rows.length,required:limits.minSamples,ready,
    reason:inliers.length<limits.minSamples||span<limits.minSpanMs?'Collecting fresh views':inliers.length<Math.ceil(rows.length*limits.minInlierRatio)?'Views disagree · start again':!motion?'Move head sideways to vary viewing angle':!steady?'Not steady yet · look closer':'Ready · press trigger to accept',
    lFromTool:pose,quality:{samples:inliers.length,rejectedSamples:rows.length-inliers.length,spanMs:span,
      positionSpreadMm:spread*1000,orientationSpreadDeg:angle,cameraBaselineMm:baseline*1000,viewAngleDeg:viewAngle,
      maxReprojectionRmsPx:Math.max(...inliers.map(r=>r.reprojectionRmsPx)),registrationMode:limits.mode,independentlyValidated:false}};
}
export function createToolScan({spec,rigHash,toolsHash,limits={},onAccepted=()=>{}}){
  validateToolSpecs(spec);const gates={...TOOL_SCAN_LIMITS,...limits},known=new Map(spec.tools.map(t=>[t.tagId,t]));
  let rows=new Map(),summaries=new Map(),registered=new Map(),focus=null,lastSeen=-Infinity,scope=null,seen=new Set(),captureRequest=null;
  function resetPending(){rows=new Map();summaries=new Map();focus=null;lastSeen=-Infinity;scope=null;seen=new Set();captureRequest=null;}
  function begin(){resetPending();registered=new Map();}
  function observe(observation,sample,association,now){
    focus=null;if(!association||!observationIsFresh(sample,sample.context,now)||
      association.candidate?.sessionId!==sample.context.sessionId||association.candidate.spaceEpoch!==sample.context.spaceEpoch||
      association.candidate.calibrationProfileId!==sample.settings.experimentalCalibration?.id)return state(now);
    const key=`${sample.context.sessionId}/${sample.context.spaceEpoch}/${sample.generation}/${sample.settings.experimentalCalibration?.id}`;
    if(scope&&scope!==key)resetPending();scope=key;
    const identity=`${key}/${sample.serial}`;if(seen.has(identity))return state(now);seen.add(identity);
    const all=observation.unassignedDetections??[],counts=new Map();for(const t of all)counts.set(t.id,(counts.get(t.id)||0)+1);
    const visible=[];
    for(const raw of all){const tool=known.get(raw.id);if(!tool||counts.get(raw.id)!==1)continue;
      const value=observeIndependentTool(raw,tool,observation.cameraModel,association,gates);if(!value)continue;
      let list=rows.get(raw.id)??[];const cap=sample.videoFrameMetadata?.captureTime;
      if(!Number.isFinite(cap)||list.length&&cap<=list.at(-1).captureTime)continue;
      if(list.length&&now-list.at(-1).at>gates.maxGapMs)list=[];
      const fingerprint=JSON.stringify(raw.corners);if(list.some(r=>r.fingerprint===fingerprint))continue;
      list.push({...value,at:now,captureTime:cap,serial:sample.serial,fingerprint});list=list.filter(r=>now-r.at<=gates.maxWindowMs);rows.set(raw.id,list);
      const summary=summarize(list,gates,now);summaries.set(raw.id,summary);visible.push({id:raw.id,ready:summary.ready,registered:registered.has(raw.id),area:value.areaPx});
    }
    visible.sort((a,b)=>Number(a.registered)-Number(b.registered)||Number(b.ready)-Number(a.ready)||b.area-a.area);
    focus=visible[0]?.id??null;if(focus!==null)lastSeen=now;
    if(captureRequest&&now>captureRequest.until)captureRequest=null;
    if(captureRequest&&captureRequest.scope===scope&&visible.some(t=>t.id===captureRequest.tagId)){
      const requested=captureRequest;
      commit(requested.tagId,now,requested.profileId);
    }
    return state(now);
  }
  function state(now=performance.now()){
    const selected=focus!==null&&now-lastSeen<=gates.maxAgeMs?focus:null;
    const tools=spec.tools.map(t=>{let candidate=summaries.get(t.tagId)??{count:0,required:gates.minSamples,ready:false,reason:'Look closer and keep the full tag in view'};
      if(candidate.ready&&now-(rows.get(t.tagId)?.at(-1)?.at??-Infinity)>gates.maxAgeMs)candidate={...candidate,ready:false,reason:'Fresh observation required'};
      return {id:t.id,tagId:t.tagId,role:t.role,registered:registered.has(t.tagId),candidate:clone(candidate)};});
    return {selectedTagId:selected,acceptedCount:registered.size,total:spec.tools.length,tools,limits:{...gates},
      captureRequest:captureRequest&&now<=captureRequest.until?{tagId:captureRequest.tagId,remainingMs:captureRequest.until-now}:null};
  }
  function commit(tagId,now,profileId){
    if(registered.has(tagId))return null;
    const s=state(now),tool=s.tools.find(t=>t.tagId===tagId);
    if(s.selectedTagId!==tagId||!tool?.candidate.ready)return null;
    const entry={id:tool.id,tagId:tool.tagId,role:tool.role,lFromTool:tool.candidate.lFromTool.slice(),quality:clone(tool.candidate.quality),
      capturedWithProfileId:profileId,acceptedAt:new Date().toISOString()};registered.set(tool.tagId,entry);captureRequest=null;onAccepted(clone(entry));return clone(entry);
  }
  function accept(now=performance.now(),profileId=null){return commit(state(now).selectedTagId,now,profileId);}
  function requestCapture(now=performance.now(),profileId=null){
    const id=state(now).selectedTagId;
    if(id==null)return {status:'not-visible',tagId:null};
    if(registered.has(id))return {status:'already-accepted',tagId:id};
    const result=commit(id,now,profileId);if(result)return {status:'accepted',tagId:id};
    captureRequest={tagId:id,profileId,scope,until:now+gates.captureRequestMs};return {status:'pending',tagId:id};
  }
  function reopen(tagId){
    if(!known.has(tagId))return false;
    registered.delete(tagId);rows.delete(tagId);summaries.delete(tagId);if(captureRequest?.tagId===tagId)captureRequest=null;
    if(focus===tagId)focus=null;return true;
  }
  function layout(){return {schema:'webxr-apriltag-layout-v1',units:'m',rigHash,toolsHash,referenceToolId:'origin-l',
    createdAt:new Date().toISOString(),assumption:'Physical tool layout unchanged relative to the fixed L',tools:clone([...registered.values()])};}
  function loadLayout(value){const p=validateToolLayout(value,{spec,rigHash,toolsHash});resetPending();registered=new Map(p.tools.map(t=>[t.tagId,clone(t)]));return layout();}
  return {begin,observe,state,accept,requestCapture,reopen,layout,loadLayout,resetPending};
}
export function validateToolLayout(value,{spec,rigHash,toolsHash}){
  const p=typeof value==='string'?JSON.parse(value):value;
  if(p?.schema!=='webxr-apriltag-layout-v1'||p.units!=='m'||p.rigHash!==rigHash||p.toolsHash!==toolsHash||
    p.referenceToolId!=='origin-l'||!Array.isArray(p.tools)||p.tools.length>spec.tools.length)throw new Error('Layout does not match reference geometry');
  const ids=new Set();for(const t of p.tools){const known=spec.tools.find(s=>s.tagId===t.tagId);
    if(!known||known.id!==t.id||ids.has(t.tagId)||!isRigidPose(t.lFromTool)||Math.hypot(...t.lFromTool.slice(12,15))>20)
      throw new Error('Invalid or duplicate tool pose');ids.add(t.tagId);}
  return clone(p);
}
export function createToolLayoutStore({spec,rigHash,toolsHash},storage=globalThis.localStorage){
  const key=`webxr-apriltag/layout/v1/${rigHash}/${toolsHash}`;
  return {async load(){try{const e=JSON.parse(storage.getItem(key));return e&&await digest(e.layout)===e.checksum?validateToolLayout(e.layout,{spec,rigHash,toolsHash}):null;}catch{return null;}},
    async save(value){const layout=validateToolLayout(value,{spec,rigHash,toolsHash});storage.setItem(key,JSON.stringify({layout,checksum:await digest(layout)}));return layout;}};
}
