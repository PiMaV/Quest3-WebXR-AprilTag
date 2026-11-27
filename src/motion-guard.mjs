// Head-motion velocity gating, same-frame optical L/perimeter bridge, outlier consensus,
// and damped visual-continuity drift correction.
import {isRigidPose} from './anchor.mjs';
import {fitPlanarPose} from './rig.mjs';
import {composeRigid,cvToGL,identity,inverse,matToQuat,poseAt,product,quatAngle,quatSlerp,rigidVector} from './runtime.mjs';

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
  maxPerimeterRmsPx:1.4,
  minPerimeterSpanMinorM:0.08,
  maxTagOutlierPx:1.6,
  maxDivergenceM:0.04,
  maxDivergenceDeg:3.5,
});

export const ANCHOR_CORRECTION_DEFAULTS=Object.freeze({
  maxPosM:0.035,
  maxAngDeg:2.5,
  tauSec:0.40,
  maxSpeedMps:0.015,
  maxAngSpeedDegPerSec:2.0,
  deadbandPosM:0.0005,
  deadbandAngDeg:0.05,
  minConfidence:0.55,
  measAlpha:0.35,
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

function tagCenter2D(tag){
  const pts=tag?.points;
  if(!Array.isArray(pts)||!pts.length)return null;
  let sx=0,sy=0;
  for(const p of pts){
    if(!p||!Number.isFinite(p[0])||!Number.isFinite(p[1]))return null;
    sx+=p[0];sy+=p[1];
  }
  return [sx/pts.length,sy/pts.length];
}

/** Measure 2D XY spread (`majorM`, `minorM`) across planar tag centers to guard collinear sets. */
export function landmarkSpan2D(tags){
  if(!Array.isArray(tags)||tags.length<2)return {majorM:0,minorM:0,count:tags?.length??0};
  const centers=tags.map(tagCenter2D).filter(Boolean);
  if(centers.length<2)return {majorM:0,minorM:0,count:centers.length};
  let mx=0,my=0;
  for(const [x,y] of centers){mx+=x;my+=y;}
  mx/=centers.length;my/=centers.length;
  let cxx=0,cyy=0,cxy=0;
  for(const [x,y] of centers){
    const dx=x-mx,dy=y-my;
    cxx+=dx*dx;cyy+=dy*dy;cxy+=dx*dy;
  }
  cxx/=centers.length;cyy/=centers.length;cxy/=centers.length;
  const trace=cxx+cyy;
  const disc=Math.sqrt(Math.max(0,(cxx-cyy)*(cxx-cyy)+4*cxy*cxy));
  const lam1=(trace+disc)*0.5;
  const lam2=Math.max(0,(trace-disc)*0.5);
  return {majorM:2*Math.sqrt(lam1),minorM:2*Math.sqrt(lam2),count:centers.length};
}

function tagReprojRmsPx(pose,tag,K){
  const [r1,r2]=pose.R,k1=Number.isFinite(K?.k1)?K.k1:0;
  let sum=0;
  for(let i=0;i<tag.points.length;i++){
    const [x,y]=tag.points[i];
    const px=pose.t[0]+r1[0]*x+r2[0]*y;
    const py=pose.t[1]+r1[1]*x+r2[1]*y;
    const pz=pose.t[2]+r1[2]*x+r2[2]*y;
    if(!(pz>0.01))return Infinity;
    const xn=px/pz,yn=py/pz,sc=k1?1+k1*(xn*xn+yn*yn):1;
    const dx=K.fx*xn*sc+K.cx-tag.pixels[i].x;
    const dy=K.fy*yn*sc+K.cy-tag.pixels[i].y;
    sum+=dx*dx+dy*dy;
  }
  return Math.sqrt(sum/tag.points.length);
}

/**
 * Jointly solve a planar 6DOF camera pose across fixed L-tags and converged `z = 0`
 * perimeter landmarks with Leave-One-Out / triplet RANSAC outlier rejection and 2D span gating.
 */
export function solveLandmarkConsensus(tags,K,{
  fixedIds=[],
  maxTagRmsPx=OPTICAL_BRIDGE_DEFAULTS.maxTagOutlierPx,
  minTags=OPTICAL_BRIDGE_DEFAULTS.minTags,
  minMinorSpanM=OPTICAL_BRIDGE_DEFAULTS.minPerimeterSpanMinorM,
}={}){
  if(!Array.isArray(tags)||tags.length<minTags||!K||!(K.fx>0)||!(K.fy>0))return null;
  const fixedSet=new Set(fixedIds);
  const fixedTotal=tags.filter(t=>fixedSet.has(t.id)).length;

  const allPose=fitPlanarPose(tags,K);
  if(allPose){
    let sumSq=0,allOk=true;
    for(const t of tags){
      const r=tagReprojRmsPx(allPose,t,K);
      if(!Number.isFinite(r)||r>maxTagRmsPx){allOk=false;break;}
      sumSq+=r*r;
    }
    if(allOk){
      const span=landmarkSpan2D(tags);
      if(span.minorM>=minMinorSpanM){
        return {pose:allPose,inliers:tags.slice(),outliers:[],rmsPx:Math.sqrt(sumSq/tags.length),
          minorSpanM:span.minorM,majorSpanM:span.majorM};
      }
    }
  }

  let best=null;
  const seenKeys=new Set();
  const consider=seedTags=>{
    if(seedTags.length<minTags)return;
    let pose=fitPlanarPose(seedTags,K);
    if(!pose)return;
    const inliers=[];
    for(const t of tags){
      if(tagReprojRmsPx(pose,t,K)<=maxTagRmsPx)inliers.push(t);
    }
    if(inliers.length<minTags)return;
    const fixedIn=inliers.filter(t=>fixedSet.has(t.id)).length;
    const majority=inliers.length>tags.length/2||(fixedTotal>=3&&fixedIn===fixedTotal);
    if(!majority)return;
    const key=inliers.map(t=>t.id).sort((a,b)=>a-b).join(',');
    if(seenKeys.has(key))return;
    seenKeys.add(key);
    if(inliers.length!==seedTags.length){
      pose=fitPlanarPose(inliers,K);
      if(!pose)return;
    }
    let sumSq=0;
    for(const t of inliers){
      const r=tagReprojRmsPx(pose,t,K);
      if(!Number.isFinite(r)||r>maxTagRmsPx)return;
      sumSq+=r*r;
    }
    const span=landmarkSpan2D(inliers);
    if(span.minorM<minMinorSpanM)return;
    const rmsPx=Math.sqrt(sumSq/inliers.length);
    if(
      !best||
      fixedIn>best.fixedIn||
      (fixedIn===best.fixedIn&&inliers.length>best.inliers.length)||
      (fixedIn===best.fixedIn&&inliers.length===best.inliers.length&&rmsPx<best.rmsPx)
    ){
      best={pose,inliers,fixedIn,rmsPx,minorSpanM:span.minorM,majorSpanM:span.majorM};
    }
  };

  if(tags.length<=4){
    const full=(1<<tags.length)-1;
    for(let mask=1;mask<full;mask++){
      const sub=tags.filter((_,i)=>mask&(1<<i));
      if(sub.length>=minTags)consider(sub);
    }
  }else{
    for(let skip=0;skip<tags.length;skip++){
      consider(tags.filter((_,i)=>i!==skip));
    }
    if(!best){
      const ordered=tags.slice().sort((a,b)=>Number(fixedSet.has(b.id))-Number(fixedSet.has(a.id)));
      let tested=0;
      for(let i=0;i<ordered.length&&tested<24;i++){
        for(let j=i+1;j<ordered.length&&tested<24;j++){
          for(let k=j+1;k<ordered.length&&tested<24;k++){
            const tri=[ordered[i],ordered[j],ordered[k]];
            if(landmarkSpan2D(tri).minorM<minMinorSpanM*0.5)continue;
            tested++;
            consider(tri);
          }
        }
      }
    }
  }

  if(!best)return null;
  const inlierSet=new Set(best.inliers.map(t=>t.id));
  const outliers=tags
    .filter(t=>!inlierSet.has(t.id))
    .map(t=>({id:t.id,rmsPx:tagReprojRmsPx(best.pose,t,K)}));
  return {
    pose:best.pose,
    inliers:best.inliers,
    outliers,
    rmsPx:best.rmsPx,
    minorSpanM:best.minorSpanM,
    majorSpanM:best.majorSpanM,
  };
}

function isValidQuadForK(c,K){
  if(!Array.isArray(c)||c.length!==4)return false;
  for(let i=0;i<4;i++){
    const p=c[i];
    if(!Number.isFinite(p?.x)||!Number.isFinite(p?.y)||p.x<0||p.y<0||p.x>=K.width||p.y>=K.height)return false;
  }
  let minE=Infinity,area2=0;
  for(let i=0;i<4;i++){
    const p=c[i],q=c[(i+1)&3],r=c[(i+2)&3];
    if((q.x-p.x)*(r.y-q.y)-(q.y-p.y)*(r.x-q.x)>=0)return false;
    const e=Math.hypot(p.x-q.x,p.y-q.y);
    if(e<minE)minE=e;
    area2+=p.x*q.y-p.y*q.x;
  }
  return minE>=8&&Math.abs(area2)*0.5>=64;
}

/**
 * Resolve lFromCamera for a camera frame. Prefers joint same-frame optical localization over
 * fixed L-tags + converged `z = 0` perimeter landmarks (`optical-perimeter`), then L-only (`optical-l`),
 * falling back cleanly to anchored XR history (`xr-history`) when optical gates are not met.
 */
export function resolveCameraInL(observation,assoc,options={}){
  if(!assoc?.worldFromAnchor||!assoc?.worldFromCamera)return null;
  const cfg={...OPTICAL_BRIDGE_DEFAULTS,...options};
  const lFromCameraXrRaw=product(inverse(assoc.worldFromAnchor),assoc.worldFromCamera);
  const hasCorr=isRigidPose(options.anchorCorrection);
  const lFromCameraXr=hasCorr
    ?product(inverse(product(assoc.worldFromAnchor,options.anchorCorrection)),assoc.worldFromCamera)
    :lFromCameraXrRaw;

  const landmarks=Array.isArray(options.perimeterLandmarks)?options.perimeterLandmarks:[];
  const K=observation?.cameraModel;
  if(cfg.allowOpticalBridge!==false&&landmarks.length>0&&K){
    const fixedTags=[];
    const fixedIds=[];
    const obsTags=Array.isArray(observation?.tags)?observation.tags:[];
    for(const t of obsTags){
      if(t&&Array.isArray(t.points)&&t.points.length===4&&Array.isArray(t.pixels)&&t.pixels.length===4){
        fixedTags.push({id:t.id,points:t.points,pixels:t.pixels});
        fixedIds.push(t.id);
      }
    }
    const lmMap=new Map();
    for(const lm of landmarks){
      if(lm&&Number.isInteger(lm.id)&&Array.isArray(lm.points)&&lm.points.length===4)lmMap.set(lm.id,lm);
    }
    const unassigned=Array.isArray(observation?.unassignedDetections)?observation.unassignedDetections:[];
    const counts=new Map();
    for(const d of unassigned)counts.set(d.id,(counts.get(d.id)||0)+1);
    const perimTags=[];
    for(const d of unassigned){
      if(counts.get(d.id)!==1)continue;
      const lm=lmMap.get(d.id);
      if(!lm||!isValidQuadForK(d.corners,K))continue;
      perimTags.push({id:d.id,points:lm.points,pixels:d.corners});
    }

    const combined=[...fixedTags,...perimTags];
    if(perimTags.length>=1&&combined.length>=cfg.minTags){
      const consensus=solveLandmarkConsensus(combined,K,{
        fixedIds,
        maxTagRmsPx:cfg.maxTagOutlierPx,
        minTags:cfg.minTags,
        minMinorSpanM:cfg.minPerimeterSpanMinorM,
      });
      if(consensus&&Number.isFinite(consensus.rmsPx)&&consensus.rmsPx<=cfg.maxPerimeterRmsPx){
        const lFromCameraOpt=inverse(cvToGL(consensus.pose));
        if(isRigidPose(lFromCameraOpt)){
          const d=rigidVector(product(inverse(lFromCameraXrRaw),lFromCameraOpt));
          const posDiffM=Math.hypot(d[3],d[4],d[5]);
          const angDiffDeg=Math.hypot(d[0],d[1],d[2])*180/Math.PI;
          if(posDiffM<=cfg.maxDivergenceM&&angDiffDeg<=cfg.maxDivergenceDeg){
            const fixedSet=new Set(fixedIds);
            const inlierIds=consensus.inliers.map(t=>t.id);
            const fixedIn=inlierIds.filter(id=>fixedSet.has(id)).length;
            const perimIn=inlierIds.length-fixedIn;
            return {
              lFromCamera:lFromCameraOpt,
              source:perimIn>0?'optical-perimeter':'optical-l',
              opticalTagCount:inlierIds.length,
              fixedTagCount:fixedIn,
              perimeterTagCount:perimIn,
              opticalRmsPx:consensus.rmsPx,
              xrDeltaMm:posDiffM*1000,
              xrDeltaDeg:angDiffDeg,
              inlierIds,
              outliers:consensus.outliers,
              baselineMinorM:consensus.minorSpanM,
              baselineMajorM:consensus.majorSpanM,
            };
          }
        }
      }
    }
  }

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
      const d=rigidVector(product(inverse(lFromCameraXrRaw),lFromCameraOpt));
      const posDiffM=Math.hypot(d[3],d[4],d[5]);
      const angDiffDeg=Math.hypot(d[0],d[1],d[2])*180/Math.PI;
      if(posDiffM<=cfg.maxDivergenceM&&angDiffDeg<=cfg.maxDivergenceDeg){
        return {
          lFromCamera:lFromCameraOpt,
          source:'optical-l',
          opticalTagCount:tagCount,
          fixedTagCount:tagCount,
          perimeterTagCount:0,
          opticalRmsPx:rms,
          xrDeltaMm:posDiffM*1000,
          xrDeltaDeg:angDiffDeg,
          inlierIds:lTool.tagIds.slice(),
          outliers:[],
        };
      }
    }
  }

  return {
    lFromCamera:lFromCameraXr,
    source:'xr-history',
    opticalTagCount:tagCount,
    fixedTagCount:0,
    perimeterTagCount:0,
    opticalRmsPx:Number.isFinite(rms)?rms:null,
    xrDeltaMm:null,
    xrDeltaDeg:null,
    inlierIds:[],
    outliers:[],
  };
}

/**
 * Critically damped, bounded scene drift correction `anchorCorrection` (`anchorFromCorrectedL`).
 * Blends native WebXR `XRAnchor` drift smoothly without visual snapping or overwriting `XRAnchor`.
 */
export function createDampedAnchorCorrection(options={}){
  const cfg={...ANCHOR_CORRECTION_DEFAULTS,...options};
  let currentMatrix=identity();
  let targetMatrix=identity();
  let updates=0;

  function metrics(mat){
    const v=rigidVector(mat);
    return {
      posMm:Math.hypot(v[3],v[4],v[5])*1000,
      angDeg:Math.hypot(v[0],v[1],v[2])*180/Math.PI,
    };
  }

  function reset(){
    currentMatrix=identity();
    targetMatrix=identity();
    updates=0;
  }

  function state(){
    const m=metrics(currentMatrix);
    const active=m.posMm>0.01||m.angDeg>1e-4;
    return {
      matrix:currentMatrix.slice(),
      targetMatrix:targetMatrix.slice(),
      posMm:m.posMm,
      angDeg:m.angDeg,
      updates,
      active,
    };
  }

  function observeOptical({lFromCameraXrRaw,lFromCameraOpt,confidence=1}={}){
    if(!isRigidPose(lFromCameraXrRaw)||!isRigidPose(lFromCameraOpt))return state();
    const conf=Number.isFinite(confidence)?clamp01(confidence):0;
    if(conf<cfg.minConfidence)return state();
    const meas=product(lFromCameraXrRaw,inverse(lFromCameraOpt));
    if(!isRigidPose(meas))return state();
    const mv=rigidVector(meas);
    const posM=Math.hypot(mv[3],mv[4],mv[5]);
    const angDeg=Math.hypot(mv[0],mv[1],mv[2])*180/Math.PI;
    if(posM>cfg.maxPosM||angDeg>cfg.maxAngDeg)return state();

    const dt=rigidVector(product(inverse(targetMatrix),meas));
    const dPos=Math.hypot(dt[3],dt[4],dt[5]);
    const dAngDeg=Math.hypot(dt[0],dt[1],dt[2])*180/Math.PI;
    if(dPos<cfg.deadbandPosM&&dAngDeg<cfg.deadbandAngDeg)return state();

    const alpha=clamp01(cfg.measAlpha*conf);
    const nextPos=[
      targetMatrix[12]+(meas[12]-targetMatrix[12])*alpha,
      targetMatrix[13]+(meas[13]-targetMatrix[13])*alpha,
      targetMatrix[14]+(meas[14]-targetMatrix[14])*alpha,
    ];
    const nextQ=quatSlerp(matToQuat(targetMatrix),matToQuat(meas),alpha);
    targetMatrix=composeRigid(nextPos,nextQ);
    updates++;
    return state();
  }

  function step(dtSec){
    const dt=Number.isFinite(dtSec)&&dtSec>0?Math.min(dtSec,0.25):0;
    const d=rigidVector(product(inverse(currentMatrix),targetMatrix));
    const posErr=Math.hypot(d[3],d[4],d[5]);
    const angErrDeg=Math.hypot(d[0],d[1],d[2])*180/Math.PI;
    if(posErr<1e-5&&angErrDeg<1e-4){
      currentMatrix=targetMatrix.slice();
      return state();
    }
    if(!(dt>0))return state();
    const expAlpha=cfg.tauSec>0?1-Math.exp(-dt/cfg.tauSec):1;
    const posFrac=posErr>1e-9?Math.min(expAlpha,(cfg.maxSpeedMps*dt)/posErr):expAlpha;
    const angFrac=angErrDeg>1e-6?Math.min(expAlpha,(cfg.maxAngSpeedDegPerSec*dt)/angErrDeg):expAlpha;
    const nextPos=[
      currentMatrix[12]+(targetMatrix[12]-currentMatrix[12])*posFrac,
      currentMatrix[13]+(targetMatrix[13]-currentMatrix[13])*posFrac,
      currentMatrix[14]+(targetMatrix[14]-currentMatrix[14])*posFrac,
    ];
    const nextQ=quatSlerp(matToQuat(currentMatrix),matToQuat(targetMatrix),clamp01(angFrac));
    currentMatrix=composeRigid(nextPos,nextQ);
    return state();
  }

  return {observeOptical,step,reset,state};
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
  const optical=source==='optical-l'||source==='optical-perimeter';
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
