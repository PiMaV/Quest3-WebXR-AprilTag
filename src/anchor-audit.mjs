import {calibratedViewerPose} from './profile.mjs';
import {isRigidPose} from './anchor.mjs';
import {product,inverse,cvToGL,poseAt,rigidVector,observationIsFresh} from './runtime.mjs';

// Interpolate recorded native poses; never call an XRFrame asynchronously.
export function associateAnchorCapture(history,sample,profile,now){
  if(!observationIsFresh(sample,sample?.context,now)||!sample.context.sessionId||
    sample.settings?.experimentalCalibration?.id!==profile?.id)return null;
  const candidate=calibratedViewerPose(history,sample,profile);if(!candidate)return null;
  if(!Array.isArray(history)||history.length<2)return null;
  const c=sample.context,target=candidate.targetXrTimeMs,tMin=target-110,tMax=target+110;
  let lo=0,hi=history.length-1;
  while(lo<hi){const mid=(lo+hi)>>1;if(history[mid].xrRafTimeMs<tMin)lo=mid+1;else hi=mid;}
  const local=[];
  for(let i=Math.max(0,lo-1);i<history.length;i++){
    const p=history[i];
    if(p.xrRafTimeMs>tMax)break;
    if(p.sessionId===c.sessionId&&p.spaceEpoch===c.spaceEpoch&&
      !p.emulatedPosition&&p.anchorTrackingState==='tracking'&&isRigidPose(p.worldFromAnchor)){
      local.push({xrRafTimeMs:p.xrRafTimeMs,worldFromViewer:p.worldFromAnchor});
    }
  }
  const worldFromAnchor=poseAt(local,target);if(!worldFromAnchor)return null;
  return {candidate,worldFromAnchor,worldFromCamera:product(candidate.worldFromViewer,profile.viewerFromCameraGL),
    targetXrTimeMs:target};
}
export function auditAnchorObservation(observation,sample,association,rig){
  if(!association)return null;
  const ids=rig.tools[0].tags.map(t=>t.id).sort((a,b)=>a-b),tool=observation.tools.find(t=>t.id===rig.tools[0].id);
  if(!tool||tool.tagIds.length!==ids.length||tool.tagIds.slice().sort((a,b)=>a-b).some((v,i)=>v!==ids[i]))return null;
  const tags=observation.tags.filter(t=>ids.includes(t.id)),K=observation.cameraModel;
  if(tags.length!==ids.length||!K||!Number.isFinite(tool.reprojectionRmsPx))return null;
  const worldFromOpticalL=product(association.worldFromCamera,cvToGL(tool.cameraFromTool));
  const delta=rigidVector(product(inverse(association.worldFromAnchor),worldFromOpticalL));
  const flip=[1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1];
  const cameraCVFromNativeL=product(flip,inverse(association.worldFromCamera),association.worldFromAnchor);
  let total=0,n=0;const projections=[],k1=Number.isFinite(K?.k1)?K.k1:0;
  for(const tag of tags){const predicted=[];let sum=0;
    for(let i=0;i<4;i++){
      const [x,y]=tag.points[i],p=[0,1,2].map(j=>cameraCVFromNativeL[j]*x+cameraCVFromNativeL[4+j]*y+cameraCVFromNativeL[12+j]);
      if(p[2]<=.01)return null;
      const xn=p[0]/p[2],yn=p[1]/p[2],sc=k1?1+k1*(xn*xn+yn*yn):1;
      const pixel={x:K.fx*xn*sc+K.cx,y:K.fy*yn*sc+K.cy};predicted.push(pixel);
      sum+=(pixel.x-tag.pixels[i].x)**2+(pixel.y-tag.pixels[i].y)**2;
    }
    total+=sum;n+=4;projections.push({id:tag.id,observed:structuredClone(tag.pixels),predicted,rmsPx:Math.sqrt(sum/4)});
  }
  if(!Number.isFinite(total))return null;
  return {serial:sample.serial,generation:sample.generation,context:structuredClone(sample.context),
    captureTime:sample.videoFrameMetadata.captureTime,targetXrTimeMs:association.targetXrTimeMs,
    positionDifferenceMm:Math.hypot(...delta.slice(3))*1000,orientationDifferenceDeg:Math.hypot(...delta.slice(0,3))*180/Math.PI,
    nativePredictionRmsPx:Math.sqrt(total/n),opticalFitRmsPx:tool.reprojectionRmsPx,
    worldFromNativeL:association.worldFromAnchor.slice(),worldFromOpticalL,projections,
    independentPhysicalAccuracy:false,usedToCorrectAnchor:false};
}
const stats=a=>{const s=a.slice().sort((x,y)=>x-y);return s.length?{median:s[Math.floor(s.length/2)],p95:s[Math.min(s.length-1,Math.floor(s.length*.95))],max:s.at(-1)}:null;};
export function createAnchorAudit({maxSamples=120}={}){
  let rows=[],scope=null,seen=new Set(),lastCapture=-Infinity;
  function reset(){rows=[];scope=null;seen=new Set();lastCapture=-Infinity;}
  function observe(row){
    if(!row)return false;const next=`${row.context.sessionId}/${row.context.spaceEpoch}/${row.generation}`;
    if(scope&&scope!==next)reset();scope=next;
    const id=`${next}/${row.serial}`;
    if(seen.has(id)||row.captureTime<=lastCapture)return false;
    seen.add(id);lastCapture=row.captureTime;rows.push(structuredClone(row));if(rows.length>maxSamples)rows.shift();return true;
  }
  function recentRows(limit=24){
    const n=Number.isFinite(limit)?Math.max(0,Math.floor(limit)):24;
    return n?structuredClone(rows.slice(-n)):[];
  }
  function state(){return {samples:rows.length,positionDifferenceMm:stats(rows.map(r=>r.positionDifferenceMm)),
    orientationDifferenceDeg:stats(rows.map(r=>r.orientationDifferenceDeg)),nativePredictionRmsPx:stats(rows.map(r=>r.nativePredictionRmsPx)),
    independentPhysicalAccuracy:false,usedToCorrectAnchor:false};}
  return {reset,observe,recentRows,state};
}
