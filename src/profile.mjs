import {poseAt} from './runtime.mjs';
import {isRigidPose} from './anchor.mjs';
export const PROFILE_SCHEMA='webxr-apriltag-profile-v1';
export const PIPELINE='rgba-gray-arena36h11-planar-capture-xrraf-v1';
export const CALIBRATION_SOLVER_VERSION=2;
const stable=v=>JSON.stringify(v,(_,x)=>x&&typeof x==='object'&&!Array.isArray(x)?Object.fromEntries(Object.entries(x).sort(([a],[b])=>a.localeCompare(b))):x);
export async function digest(v){return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(stable(v)))),b=>b.toString(16).padStart(2,'0')).join('');}
/** Compute a SHA-256 device + camera-mode + rig identity key for calibration profile scoping. */
export async function deviceIdentity(camera,rig,userAgent=navigator.userAgent){
  const s=camera.trackSettings??{},mode={sourceSize:camera.sourceSize,resizeMode:s.resizeMode??null,facingMode:s.facingMode??null,
    zoom:s.zoom??null,focusMode:s.focusMode??null};
  const cacheable=Boolean(s.deviceId);
  const rigHash=await digest(rig),cameraHash=await digest({deviceId:s.deviceId||crypto.randomUUID(),label:camera.trackLabel,userAgent});
  return {key:await digest({cameraHash,mode,rigHash,pipeline:PIPELINE}),cameraHash,mode,rigHash,pipeline:PIPELINE,
    trackLabel:camera.trackLabel,userAgent,cacheable};
}
/** Validate a calibration profile against schema, physical camera bounds, and verification gates. */
export function validateCalibration(p){
  const camT=Array.isArray(p?.viewerFromCameraGL)?p.viewerFromCameraGL.slice(12,15):null;
  if(p?.schema!==PROFILE_SCHEMA||!p.id||!p.device?.key||p.device.pipeline!==PIPELINE||!p.device.rigHash||
    !Array.isArray(p.imageSize)||p.imageSize.length!==2||p.imageSize.some(v=>!Number.isInteger(v)||v<64||v>4096)||
    !p.K||![p.K.fx,p.K.fy,p.K.cx,p.K.cy].every(Number.isFinite)||p.K.fx<=0||p.K.fy<=0||
    p.K.width!==p.imageSize[0]||p.K.height!==p.imageSize[1]||
    !isRigidPose(p.viewerFromCameraGL)||!Number.isFinite(p.timeOffsetMs)||Math.abs(p.timeOffsetMs)>=230||
    !camT||Math.hypot(camT[0],camT[1],camT[2])>=0.16||Math.abs(camT[2])>=0.09||
    !Array.isArray(p.distCoeffs)||p.distCoeffs.length!==5||!p.distCoeffs.every(Number.isFinite)||
    Math.abs(p.distCoeffs[0])>=0.45||p.distCoeffs.slice(1).some(x=>x!==0)||
    (p.K.k1!==undefined&&(!Number.isFinite(p.K.k1)||Math.abs(p.K.k1-p.distCoeffs[0])>=1e-6))||
    (p.solverVersion!==undefined&&(!Number.isInteger(p.solverVersion)||p.solverVersion<1))||
    p.validation?.passed!==true||p.verification?.passed!==true||p.independentlyValidated!==false||
    (p.validation.positionP95Mm!=null&&p.validation.positionP95Mm>=8.5)||
    (p.validation.angleP95Deg!=null&&p.validation.angleP95Deg>=1.5)||
    (p.verification.positionP95Mm!=null&&p.verification.positionP95Mm>=10.0)||
    (p.verification.angleP95Deg!=null&&p.verification.angleP95Deg>=1.5))
    throw new Error('Not a validated, supported calibration profile');
  return p;
}
export function extractCalibrationProfile(payload){
  const value=typeof payload==='string'?JSON.parse(payload):payload;
  const profile=value?.schema===PROFILE_SCHEMA?value:
    value?.schema==='webxr-apriltag-recording-v1'?value.calibrationAssistant?.profile:null;
  return structuredClone(validateCalibration(profile));
}
export function assertProfileDevice(profile,device){
  validateCalibration(profile);
  const fields=['key','cameraHash','mode','rigHash','pipeline','trackLabel','userAgent'];
  if(fields.some(k=>stable(profile.device[k])!==stable(device[k])))
    throw new Error('Profile belongs to a different camera, image mode, or browser version and was not imported.');
  return profile;
}
export async function importCalibrationProfile(payload,device,store=createProfileStore()){
  const profile=assertProfileDevice(extractCalibrationProfile(payload),device);
  await store.save(profile);return profile;
}
/** Rescale and validate a calibration profile's intrinsics for the active camera stream mode. */
export function calibratedCameraModel(p,imageSize,sourceSize,label,userAgent,settings={}){
  // Candidates may be used for verification, but cannot be persisted before it.
  if(p.schema!==PROFILE_SCHEMA||!isRigidPose(p.viewerFromCameraGL))throw new Error('Invalid calibration profile');
  if(sourceSize.some((v,i)=>v!==p.device.mode.sourceSize[i])||label!==p.device.trackLabel||userAgent!==p.device.userAgent||
    (settings.resizeMode??null)!==p.device.mode.resizeMode||(settings.zoom??null)!==p.device.mode.zoom||
    (settings.focusMode??null)!==p.device.mode.focusMode)
    throw new Error('Camera or image mode changed. Please re-verify the profile.');
  const [w,h]=imageSize,[sw,sh]=p.imageSize;if(Math.abs(w/h-sw/sh)>.001)throw new Error('Image aspect ratio changed');
  const k1=Number.isFinite(p.distCoeffs?.[0])?p.distCoeffs[0]:(Number.isFinite(p.K?.k1)?p.K.k1:0);
  return {width:w,height:h,fx:p.K.fx*w/sw,fy:p.K.fy*h/sh,cx:p.K.cx*w/sw,cy:p.K.cy*h/sh,k1};
}
export function profileCameraConstraints(p,devices=[],supported={}){
  const [width,height]=p.device.mode.sourceSize,device=devices.find(d=>d.label===p.device.trackLabel&&d.kind==='videoinput');
  return {audio:false,video:{width:{exact:width},height:{exact:height},facingMode:{ideal:'environment'},
    ...(supported.resizeMode&&p.device.mode.resizeMode?{resizeMode:{exact:p.device.mode.resizeMode}}:{}),
    ...(device?.deviceId?{deviceId:{exact:device.deviceId}}:{})}};
}
/** Interpolate the calibrated XR viewer pose at `captureTime + profile.timeOffsetMs`. */
export function viewerPoseHorizon(history,sample,p){
  const c=sample?.context,cap=sample?.videoFrameMetadata?.captureTime;
  if(!Number.isFinite(cap)||!c?.sessionId||!Number.isFinite(p?.timeOffsetMs)||
    sample.submittedAtPerformanceMs-cap<-50||sample.submittedAtPerformanceMs-cap>1000)
    return {status:'invalid',targetXrTimeMs:null,pose:null};
  const target=cap+p.timeOffsetMs;
  if(!Array.isArray(history)||!history.length)return {status:'future',targetXrTimeMs:target,pose:null};
  let latest=null;
  for(let i=history.length-1;i>=0;i--){
    const v=history[i];
    if(v.sessionId===c.sessionId&&v.spaceEpoch===c.spaceEpoch&&!v.emulatedPosition&&v.worldFromViewer){latest=v;break;}
  }
  if(!latest||target>latest.xrRafTimeMs)return {status:'future',targetXrTimeMs:target,pose:null};
  const tMin=target-110,tMax=target+110;
  let lo=0,hi=history.length-1;
  while(lo<hi){const mid=(lo+hi)>>1;if(history[mid].xrRafTimeMs<tMin)lo=mid+1;else hi=mid;}
  const local=[];
  for(let i=Math.max(0,lo-1);i<history.length;i++){
    const v=history[i];
    if(v.xrRafTimeMs>tMax)break;
    if(v.sessionId===c.sessionId&&v.spaceEpoch===c.spaceEpoch&&!v.emulatedPosition&&v.worldFromViewer)local.push(v);
  }
  const worldFromViewer=poseAt(local,target);
  if(!worldFromViewer)return {status:'invalid',targetXrTimeMs:target,pose:null};
  return {status:'ready',targetXrTimeMs:target,pose:{...c,worldFromViewer,calibrationProfileId:p.id,
    association:'validated-profile-at-capture-time',targetXrTimeMs:target,timeAligned:false}};
}
export function calibratedViewerPose(history,sample,p){
  return viewerPoseHorizon(history,sample,p).pose;
}
/** Buffer fast optical observations until their future XR viewer-pose horizon is available. */
export function createObservationBuffer({maxAgeMs=650,maxItems=12}={}){
  let queue=[];
  function push(entry){
    queue.push(entry);
    if(queue.length>maxItems)queue.splice(0,queue.length-maxItems);
  }
  function drain(history,profile,now){
    if(!profile||!queue.length)return [];
    const ready=[],pending=[];
    for(const item of queue){
      const age=now-(item.sample?.submittedAtPerformanceMs??-Infinity);
      if(!Number.isFinite(age)||age>maxAgeMs)continue;
      const h=viewerPoseHorizon(history,item.sample,profile);
      if(h.status==='ready')ready.push({...item,candidate:h.pose});
      else if(h.status==='future')pending.push(item);
    }
    queue=pending;
    return ready;
  }
  return {push,drain,clear(){queue=[];},size:()=>queue.length};
}
/** Create a SHA-256 checksummed persistence store (defaults to `localStorage`) for calibration profiles. */
export function createProfileStore(storage=globalThis.localStorage){
  const key=id=>`webxr-apriltag/v1/${id}`;
  return {
    async load(device){
      if(device.cacheable===false)return null;
      try{const entry=JSON.parse(storage.getItem(key(device.key)));if(!entry||await digest(entry.profile)!==entry.checksum)return null;
        const p=validateCalibration(entry.profile);return p.device.key===device.key?p:null;}catch{return null;}
    },
    async save(profile){validateCalibration(profile);if(profile.device.cacheable===false)throw new Error('Session-only camera identity');
      const value=JSON.stringify({profile,checksum:await digest(profile)});
      // One atomic replacement; a failed candidate never overwrites a good profile.
      storage.setItem(key(profile.device.key),value);return profile;},
  };
}
