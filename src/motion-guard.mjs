// Head-motion velocity gating and same-frame optical L-angle bridge to eliminate
// lag-induced tag jumping while the Quest camera is running during movement.
import {isRigidPose} from './anchor.mjs';
import {cvToGL,inverse,poseAt,product,rigidVector} from './runtime.mjs';

export const MOTION_GATE_DEFAULTS=Object.freeze({
  xrSteadyAngDegPerSec:10,
  xrMaxAngDegPerSec:32,
  xrSteadyLinMPerSec:0.10,
  xrMaxLinMPerSec:0.35,
  opticalSteadyAngDegPerSec:25,
  opticalMaxAngDegPerSec:75,
  opticalSteadyLinMPerSec:0.25,
  opticalMaxLinMPerSec:0.70,
});

export const OPTICAL_BRIDGE_DEFAULTS=Object.freeze({
  allowOpticalBridge:true,
  minTags:3,
  maxOpticalRmsPx:1.0,
  maxDivergenceM:0.04,
  maxDivergenceDeg:3.5,
});

export const CARRIED_MOTION_GATE_DEFAULTS=Object.freeze({
  xrSteadyAngDegPerSec:40,
  xrMaxAngDegPerSec:108,
  xrSteadyLinMPerSec:0.55,
  xrMaxLinMPerSec:1.50,
  opticalSteadyAngDegPerSec:46,
  opticalMaxAngDegPerSec:125,
  opticalSteadyLinMPerSec:0.70,
  opticalMaxLinMPerSec:1.80,
});

const clamp01=v=>Math.max(0,Math.min(1,v));

function pairSpeed(poseA,poseB,dtMs){
  if(!poseA||!poseB||!(dtMs>=8))return null;
  const dtSec=dtMs/1000;
  const rel=rigidVector(product(inverse(poseA),poseB));
  const angDegPerSec=(Math.hypot(rel[0],rel[1],rel[2])*180/Math.PI)/dtSec;
  const linMPerSec=Math.hypot(
    poseB[12]-poseA[12],
    poseB[13]-poseA[13],
    poseB[14]-poseA[14],
  )/dtSec;
  if(!Number.isFinite(angDegPerSec)||!Number.isFinite(linMPerSec))return null;
  return {angDegPerSec,linMPerSec,dtMs};
}

/**
 * Estimate peak viewer angular (deg/s) and linear (m/s) speed in a symmetric
 * window around targetTimeMs using binary search over the XR pose history.
 */
export function viewerVelocityAt(history,targetTimeMs,context,{windowMs=35,searchPadMs=160}={}){
  if(!Array.isArray(history)||history.length<2||!Number.isFinite(targetTimeMs))return null;
  const tLo=targetTimeMs-searchPadMs,tHi=targetTimeMs+searchPadMs;
  // Binary search first index with xrRafTimeMs >= tLo.
  let lo=0,hi=history.length-1;
  while(lo<hi){
    const mid=(lo+hi)>>1;
    if(history[mid].xrRafTimeMs<tLo)lo=mid+1;
    else hi=mid;
  }
  const start=Math.max(0,lo-1);
  const local=[];
  for(let i=start;i<history.length;i++){
    const entry=history[i];
    if(entry.xrRafTimeMs>tHi&&local.length>=2)break;
    if(context){
      if(entry.sessionId!==context.sessionId||entry.spaceEpoch!==context.spaceEpoch)continue;
    }
    if(entry.emulatedPosition||!entry.worldFromViewer)continue;
    local.push(entry);
  }
  if(local.length<2)return null;

  const tA=targetTimeMs-windowMs;
  const tM=targetTimeMs;
  const tB=targetTimeMs+windowMs;
  const pA=poseAt(local,tA);
  const pM=poseAt(local,tM);
  const pB=poseAt(local,tB);

  const candidates=[
    pairSpeed(pA,pM,windowMs),
    pairSpeed(pM,pB,windowMs),
    pairSpeed(pA,pB,windowMs*2),
  ].filter(Boolean);

  if(!candidates.length){
    // Fall back to direct bracketing samples around targetTimeMs when near history boundaries.
    let idx=0;
    while(idx<local.length-2&&local[idx+1].xrRafTimeMs<targetTimeMs)idx++;
    const a=local[idx],b=local[idx+1];
    const direct=pairSpeed(a.worldFromViewer,b.worldFromViewer,b.xrRafTimeMs-a.xrRafTimeMs);
    if(!direct)return null;
    return direct;
  }

  let angDegPerSec=0,linMPerSec=0,dtMs=0;
  for(const c of candidates){
    if(c.angDegPerSec>angDegPerSec)angDegPerSec=c.angDegPerSec;
    if(c.linMPerSec>linMPerSec)linMPerSec=c.linMPerSec;
    if(c.dtMs>dtMs)dtMs=c.dtMs;
  }
  return {angDegPerSec,linMPerSec,dtMs};
}

/**
 * Resolve lFromCamera for a camera frame. Whenever >=2 reference L-angle tags
 * are jointly observed in the same frame with low reprojection RMS and agree
 * with the anchored XR estimate, prefer the optical same-frame L pose (0 ms
 * relative camera-to-L lag). Falls back cleanly to XR history otherwise.
 */
export function resolveCameraInL(observation,assoc,options={}){
  if(!assoc?.worldFromAnchor||!assoc?.worldFromCamera)return null;
  const cfg={...OPTICAL_BRIDGE_DEFAULTS,...options};
  const lFromCameraXr=product(inverse(assoc.worldFromAnchor),assoc.worldFromCamera);
  const lTool=observation?.tools?.[0]??null;
  const tagCount=Array.isArray(lTool?.tagIds)?lTool.tagIds.length:0;
  const rms=lTool?.reprojectionRmsPx;

  if(
    cfg.allowOpticalBridge!==false&&
    lTool&&
    lTool.mode==='joint-rigid-tags'&&
    tagCount>=cfg.minTags&&
    lTool.cameraFromTool&&
    Number.isFinite(rms)&&
    rms<=cfg.maxOpticalRmsPx
  ){
    const lFromCameraOpt=inverse(cvToGL(lTool.cameraFromTool));
    if(isRigidPose(lFromCameraOpt)){
      const d=rigidVector(product(inverse(lFromCameraXr),lFromCameraOpt));
      const posDiffM=Math.hypot(d[3],d[4],d[5]);
      const angDiffDeg=Math.hypot(d[0],d[1],d[2])*180/Math.PI;
      if(posDiffM<=cfg.maxDivergenceM&&angDiffDeg<=cfg.maxDivergenceDeg){
        return {
          lFromCamera:lFromCameraOpt,
          source:'optical-l',
          opticalTagCount:tagCount,
          opticalRmsPx:rms,
          xrDeltaMm:posDiffM*1000,
          xrDeltaDeg:angDiffDeg,
        };
      }
    }
  }

  return {
    lFromCamera:lFromCameraXr,
    source:'xr-history',
    opticalTagCount:tagCount,
    opticalRmsPx:Number.isFinite(rms)?rms:null,
    xrDeltaMm:null,
    xrDeltaDeg:null,
  };
}

/**
 * Convert viewer motion speed and pose source into an update confidence in [0, 1].
 * Fast head motion freezes camera-driven tag updates (confidence = 0) so tags stay
 * rock-solid on their 90 Hz WebXR anchor instead of jumping with camera lag.
 */
export function evaluateMotionGate(velocity,{source='xr-history'}={},limits={}){
  const cfg={...MOTION_GATE_DEFAULTS,...limits};
  if(!velocity||!Number.isFinite(velocity.angDegPerSec)||!Number.isFinite(velocity.linMPerSec)){
    return {
      confidence:1,
      frozen:false,
      angDegPerSec:0,
      linMPerSec:0,
      source,
      maxTagRmsPx:1.5,
    };
  }
  const optical=source==='optical-l';
  const steadyAng=optical?cfg.opticalSteadyAngDegPerSec:cfg.xrSteadyAngDegPerSec;
  const maxAng=optical?cfg.opticalMaxAngDegPerSec:cfg.xrMaxAngDegPerSec;
  const steadyLin=optical?cfg.opticalSteadyLinMPerSec:cfg.xrSteadyLinMPerSec;
  const maxLin=optical?cfg.opticalMaxLinMPerSec:cfg.xrMaxLinMPerSec;

  const angStress=maxAng>steadyAng?clamp01((velocity.angDegPerSec-steadyAng)/(maxAng-steadyAng)):(velocity.angDegPerSec>maxAng?1:0);
  const linStress=maxLin>steadyLin?clamp01((velocity.linMPerSec-steadyLin)/(maxLin-steadyLin)):(velocity.linMPerSec>maxLin?1:0);
  const stress=Math.max(angStress,linStress);
  const confidence=stress>=1?0:(1-stress)*(1-stress);

  return {
    confidence,
    frozen:confidence<=0,
    angDegPerSec:velocity.angDegPerSec,
    linMPerSec:velocity.linMPerSec,
    source,
    maxTagRmsPx:1.0+0.5*confidence,
  };
}

/**
 * Velocity-scaled 2-frame jump & IPPE mirror-flip guard for moving/carried movable tags.
 * Rejects single-frame PnP spikes and mirror flips while allowing continuous 0.5-1.2 m/s
 * handheld walking without deadlocking.
 */
export function evaluateCarriedJumpGuard({
  prevMatrix,
  nextMatrix,
  dtSinceLastMs,
  confidence=1,
  pendingJump=null,
  atMs=0,
}={}){
  if(!prevMatrix||!nextMatrix){
    return {accept:true,nextPendingJump:null,reason:null};
  }
  const d=rigidVector(product(inverse(prevMatrix),nextMatrix));
  const posMm=Math.hypot(d[3],d[4],d[5])*1000;
  const angDeg=Math.hypot(d[0],d[1],d[2])*180/Math.PI;
  const dtMs=Number.isFinite(dtSinceLastMs)?Math.max(0,dtSinceLastMs):Infinity;
  const dtSec=Number.isFinite(dtMs)?Math.min(0.6,Math.max(0.05,dtMs/1000)):0.2;
  const jumpPosLimit=Math.max(confidence<0.75?110:180,dtSec*2200);
  const jumpAngLimit=Math.max(confidence<0.75?24:36,dtSec*180);
  const isJump=dtMs<500&&(posMm>jumpPosLimit||angDeg>jumpAngLimit);
  const isMirrorFlip=dtMs<650&&posMm<22&&angDeg>18;
  if(!isJump&&!isMirrorFlip){
    return {accept:true,nextPendingJump:null,reason:null};
  }
  const validPending=
    pendingJump?.matrix&&
    Number.isFinite(pendingJump.atMs)&&
    atMs>=pendingJump.atMs&&
    atMs-pendingJump.atMs<=650;
  if(validPending){
    const prevD=rigidVector(product(inverse(pendingJump.matrix),nextMatrix));
    const pPosMm=Math.hypot(prevD[3],prevD[4],prevD[5])*1000;
    const pAngDeg=Math.hypot(prevD[0],prevD[1],prevD[2])*180/Math.PI;
    const dtPendingSec=Math.min(0.6,Math.max(0.05,(atMs-pendingJump.atMs)/1000));
    if(pPosMm<=Math.max(85,dtPendingSec*2000)&&pAngDeg<=Math.max(18,dtPendingSec*160)){
      return {accept:true,nextPendingJump:null,reason:'confirmed'};
    }
  }
  return {
    accept:false,
    nextPendingJump:{atMs,matrix:nextMatrix.slice()},
    reason:isMirrorFlip?'mirror-flip':'jump',
  };
}
