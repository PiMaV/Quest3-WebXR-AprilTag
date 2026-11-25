// 90 Hz WebXR hand-tracking coupling for actively carried movable AprilTags (300-349).
// Uses time-aligned rigid-body kinematic coherence between handHistory(t_cap) and optical
// tag poses to distinguish real carrying (both pinched cards and wands/sticks) from
// resting tags, hand pass-overs, and externally moved tags.
import {isRigidPose} from './anchor.mjs';
import {composeRigid,inverse,matToQuat,product,quatSlerp,rigidVector} from './runtime.mjs';

export const HAND_COUPLING_DEFAULTS=Object.freeze({
  attachRadiusM:0.16,
  maxLeverArmM:0.25,
  pinchGripDistM:0.065,
  pinchApertureMaxM:0.045,
  pinchConfirmDeltas:1,
  wandConfirmDeltas:2,
  minExcitePosM:0.022,
  minExciteAngDeg:12,
  lockPosBaseM:0.015,
  lockPosLeverSlope:0.03,
  lockAngDeg:12,
  lockStaticRatio:0.35,
  minMotionRatio:0.55,
  maxMotionRatio:1.65,
  minOpticalConfidence:0.35,
  adaptAlpha:0.25,
  watchdogPosBaseM:0.022,
  watchdogPosLeverSlope:0.04,
  watchdogAngDeg:18,
  watchdogStationaryMinHandM:0.025,
  watchdogStationaryMaxRatio:0.35,
  watchdogPullMarginM:0.06,
  pinchReleaseDeltaM:0.025,
  pinchReleasePullM:0.022,
  handLostTimeoutMs:120,
  opticalStaleTimeoutMs:520,
  maxUnconfirmedDriftM:0.22,
  candidateMaxAgeMs:900,
});

const HANDS=['left','right'];
const dist3=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
const sub3=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const cross3=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm3=v=>{const l=Math.hypot(v[0],v[1],v[2]);return l>1e-6?[v[0]/l,v[1]/l,v[2]/l]:null;};
const isVec3=v=>Array.isArray(v)&&v.length>=3&&Number.isFinite(v[0])&&Number.isFinite(v[1])&&Number.isFinite(v[2]);
const lerp3=(a,b,t)=>[a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,a[2]+(b[2]-a[2])*t];

function blendRigid(a,b,alpha){
  const t=Math.max(0,Math.min(1,alpha));
  const p=[
    a[12]+(b[12]-a[12])*t,
    a[13]+(b[13]-a[13])*t,
    a[14]+(b[14]-a[14])*t,
  ];
  return composeRigid(p,quatSlerp(matToQuat(a),matToQuat(b),t));
}

function poseDelta(a,b){
  const d=rigidVector(product(inverse(a),b));
  return {
    posM:Math.hypot(b[12]-a[12],b[13]-a[13],b[14]-a[14]),
    angDeg:Math.hypot(d[0],d[1],d[2])*180/Math.PI,
  };
}

function minAnchorDistance(sample,pointL){
  let best=Infinity;
  for(const a of sample.anchors){
    const d=dist3(a,pointL);
    if(d<best)best=d;
  }
  return best;
}

function minPinchFingerDistance(sample,pointL){
  let best=Infinity;
  if(sample.thumbTip)best=Math.min(best,dist3(sample.thumbTip,pointL));
  if(sample.indexTip)best=Math.min(best,dist3(sample.indexTip,pointL));
  if(sample.pinchMidL)best=Math.min(best,dist3(sample.pinchMidL,pointL));
  return best;
}

/**
 * Build a rigid 6DOF palm-based hand sample in L-frame from WebXR joint positions.
 * Uses the non-flexing metacarpal triangle (wrist, index-mcp, pinky-mcp) for orientation
 * stability, falling back to wristMatrix when metacarpals are unavailable.
 */
export function buildHandSample(jointsInL,xrTimeMs,handedness){
  if(!jointsInL||!Number.isFinite(xrTimeMs)||(handedness!=='left'&&handedness!=='right'))return null;
  const wrist=isVec3(jointsInL.wrist)?jointsInL.wrist.slice(0,3):null;
  const indexMcp=isVec3(jointsInL.indexMcp)?jointsInL.indexMcp.slice(0,3):null;
  const middleMcp=isVec3(jointsInL.middleMcp)?jointsInL.middleMcp.slice(0,3):null;
  const pinkyMcp=isVec3(jointsInL.pinkyMcp)?jointsInL.pinkyMcp.slice(0,3):null;
  const thumbTip=isVec3(jointsInL.thumbTip)?jointsInL.thumbTip.slice(0,3):null;
  const indexTip=isVec3(jointsInL.indexTip)?jointsInL.indexTip.slice(0,3):null;
  const middleTip=isVec3(jointsInL.middleTip)?jointsInL.middleTip.slice(0,3):null;

  let lFromHand=null;
  let palmCenter=null;
  let frameKind=null;

  if(wrist&&indexMcp&&pinkyMcp){
    const refMcp=middleMcp||lerp3(indexMcp,pinkyMcp,0.5);
    const fwdRaw=sub3(refMcp,wrist);
    const latRaw=sub3(indexMcp,pinkyMcp);
    const fwd=Math.hypot(...fwdRaw)>0.025?norm3(fwdRaw):null;
    const crossRaw=(fwd&&Math.hypot(...latRaw)>0.02)?cross3(latRaw,fwd):null;
    const normal=(crossRaw&&Math.hypot(...crossRaw)>1e-3)?norm3(crossRaw):null;
    const lat=normal?norm3(cross3(fwd,normal)):null;
    if(fwd&&normal&&lat){
      palmCenter=[
        (wrist[0]+indexMcp[0]+pinkyMcp[0])/3,
        (wrist[1]+indexMcp[1]+pinkyMcp[1])/3,
        (wrist[2]+indexMcp[2]+pinkyMcp[2])/3,
      ];
      lFromHand=[
        lat[0],lat[1],lat[2],0,
        fwd[0],fwd[1],fwd[2],0,
        normal[0],normal[1],normal[2],0,
        palmCenter[0],palmCenter[1],palmCenter[2],1,
      ];
      frameKind='palm';
    }
  }

  if(!lFromHand&&isRigidPose(jointsInL.wristMatrix)){
    lFromHand=Array.from(jointsInL.wristMatrix);
    palmCenter=[lFromHand[12],lFromHand[13],lFromHand[14]];
    frameKind='wrist';
  }
  if(!lFromHand)return null;

  const pinchMidL=(thumbTip&&indexTip)?lerp3(thumbTip,indexTip,0.5):null;
  const pinchDistM=(thumbTip&&indexTip)?dist3(thumbTip,indexTip):null;
  const anchors=[palmCenter];
  if(wrist)anchors.push(wrist);
  if(thumbTip)anchors.push(thumbTip);
  if(indexTip)anchors.push(indexTip);
  if(middleTip)anchors.push(middleTip);
  if(pinchMidL)anchors.push(pinchMidL);

  return {
    handedness,
    frameKind,
    xrTimeMs,
    lFromHand,
    palmCenter,
    wrist,
    anchors,
    thumbTip,
    indexTip,
    middleTip,
    pinchMidL,
    pinchDistM,
  };
}

function interpolateHandSample(a,b,t,xrTimeMs){
  if(a.frameKind!==b.frameKind)return null;
  const q=quatSlerp(matToQuat(a.lFromHand),matToQuat(b.lFromHand),t);
  const p=[
    a.lFromHand[12]+(b.lFromHand[12]-a.lFromHand[12])*t,
    a.lFromHand[13]+(b.lFromHand[13]-a.lFromHand[13])*t,
    a.lFromHand[14]+(b.lFromHand[14]-a.lFromHand[14])*t,
  ];
  const lFromHand=composeRigid(p,q);
  const palmCenter=lerp3(a.palmCenter,b.palmCenter,t);
  const wrist=(a.wrist&&b.wrist)?lerp3(a.wrist,b.wrist,t):(t<0.5?a.wrist:b.wrist);
  const thumbTip=(a.thumbTip&&b.thumbTip)?lerp3(a.thumbTip,b.thumbTip,t):(t<0.5?a.thumbTip:b.thumbTip);
  const indexTip=(a.indexTip&&b.indexTip)?lerp3(a.indexTip,b.indexTip,t):(t<0.5?a.indexTip:b.indexTip);
  const middleTip=(a.middleTip&&b.middleTip)?lerp3(a.middleTip,b.middleTip,t):(t<0.5?a.middleTip:b.middleTip);
  const pinchMidL=(thumbTip&&indexTip)?lerp3(thumbTip,indexTip,0.5):null;
  const pinchDistM=(thumbTip&&indexTip)?dist3(thumbTip,indexTip):null;
  const anchors=[palmCenter];
  if(wrist)anchors.push(wrist);
  if(thumbTip)anchors.push(thumbTip);
  if(indexTip)anchors.push(indexTip);
  if(middleTip)anchors.push(middleTip);
  if(pinchMidL)anchors.push(pinchMidL);
  return {
    handedness:a.handedness,
    frameKind:a.frameKind,
    xrTimeMs,
    lFromHand,
    palmCenter,
    wrist,
    anchors,
    thumbTip,
    indexTip,
    middleTip,
    pinchMidL,
    pinchDistM,
  };
}

/**
 * Time-indexed 90 Hz hand pose ring buffer per handedness ('left' | 'right'),
 * enabling exact time alignment against delayed optical camera frames at targetXrTimeMs.
 */
export function createHandHistory({maxMs=1800,maxGapMs=100,futureClampMs=25}={}){
  const buffers={left:[],right:[]};

  function push(sample){
    if(!sample||!buffers[sample.handedness]||!Number.isFinite(sample.xrTimeMs))return;
    const arr=buffers[sample.handedness];
    if(arr.length&&sample.xrTimeMs<=arr.at(-1).xrTimeMs)return;
    arr.push(sample);
    const cutoff=sample.xrTimeMs-maxMs;
    while(arr.length>2&&arr[0].xrTimeMs<cutoff)arr.shift();
  }

  function sampleAt(handedness,targetTimeMs){
    const arr=buffers[handedness];
    if(!arr||!arr.length||!Number.isFinite(targetTimeMs))return null;
    if(targetTimeMs<arr[0].xrTimeMs)return null;
    const last=arr.at(-1);
    if(targetTimeMs>=last.xrTimeMs){
      return targetTimeMs-last.xrTimeMs<=futureClampMs?last:null;
    }
    let lo=0,hi=arr.length-1;
    while(hi-lo>1){
      const mid=(lo+hi)>>1;
      if(arr[mid].xrTimeMs<=targetTimeMs)lo=mid;
      else hi=mid;
    }
    const a=arr[lo],b=arr[hi],gap=b.xrTimeMs-a.xrTimeMs;
    if(a.xrTimeMs===targetTimeMs)return a;
    if(gap<=0||gap>maxGapMs)return null;
    return interpolateHandSample(a,b,(targetTimeMs-a.xrTimeMs)/gap,targetTimeMs);
  }

  function latest(handedness,nowXrTimeMs,maxStaleMs=120){
    const arr=buffers[handedness];
    if(!arr||!arr.length)return null;
    const last=arr.at(-1);
    if(Number.isFinite(nowXrTimeMs)&&nowXrTimeMs-last.xrTimeMs>maxStaleMs)return null;
    return last;
  }

  function clear(){
    buffers.left.length=0;
    buffers.right.length=0;
  }

  return {push,sampleAt,latest,clear,size:h=>buffers[h]?.length??0};
}

/**
 * Speculative trail queue for 90 Hz hand-predicted poses during recording.
 * Buffered points are only committed up to the timestamp confirmed by a valid
 * optical observation; unconfirmed points are discarded on watchdog/timeout release.
 */
export function createCoupledTrailBuffer({minStepM=0.008,minStepDeg=5,maxPoints=90}={}){
  const queue=[];

  function push({xrTimeMs,atMs,pose}){
    if(!Number.isFinite(xrTimeMs)||!isRigidPose(pose))return false;
    if(queue.length){
      const last=queue.at(-1);
      if(xrTimeMs<=last.xrTimeMs)return false;
      const d=poseDelta(last.pose,pose);
      if(d.posM<minStepM&&d.angDeg<minStepDeg)return false;
    }
    queue.push({
      xrTimeMs,
      atMs:Number.isFinite(atMs)?atMs:xrTimeMs,
      pose:pose.slice(),
    });
    while(queue.length>maxPoints)queue.shift();
    return true;
  }

  function commitUpTo(targetXrTimeMs){
    if(!Number.isFinite(targetXrTimeMs)||!queue.length)return [];
    let count=0;
    while(count<queue.length&&queue[count].xrTimeMs<=targetXrTimeMs)count++;
    return count?queue.splice(0,count):[];
  }

  function discard(){
    const count=queue.length;
    queue.length=0;
    return count;
  }

  return {push,commitUpTo,discard,size:()=>queue.length};
}

/**
 * Per-tag finite state machine (FREE -> CANDIDATE -> COUPLED) enforcing rigid-body
 * kinematic coherence between hand motion and optical tag motion.
 */
export function createTagHandCoupler(options={}){
  const cfg={...HAND_COUPLING_DEFAULTS,...options};
  const states=new Map();

  function getOrCreate(tagId){
    let st=states.get(tagId);
    if(!st){
      st={
        tagId,
        mode:'free', // 'free' | 'candidate' | 'coupled'
        candidates:{left:null,right:null},
        handedness:null,
        gripKind:null, // 'pinch' | 'wand'
        frameKind:null, // 'palm' | 'wrist'
        handFromTag:null,
        leverArmM:0,
        lockGripDistM:null,
        lockPinchDistM:null,
        lockFingerDistM:null,
        lastOpticalXrMs:0,
        lastOpticalTagPose:null,
        disagreeCount:0,
      };
      states.set(tagId,st);
    }
    return st;
  }

  function isHandOccupiedByOther(handedness,tagId){
    for(const other of states.values()){
      if(other.tagId!==tagId&&other.mode==='coupled'&&other.handedness===handedness)return true;
    }
    return false;
  }

  function release(st,reason,opticalPose=null){
    const prevHand=st.handedness;
    const lastOpticalPose=(opticalPose||st.lastOpticalTagPose)?.slice()??null;
    st.mode='free';
    st.handedness=null;
    st.gripKind=null;
    st.frameKind=null;
    st.handFromTag=null;
    st.leverArmM=0;
    st.lockGripDistM=null;
    st.lockPinchDistM=null;
    st.lockFingerDistM=null;
    st.lastOpticalTagPose=null;
    st.disagreeCount=0;
    st.candidates.left=null;
    st.candidates.right=null;
    return {changed:true,event:'released',handedness:prevHand,reason,mode:'free',lastOpticalPose};
  }

  function observeOptical({tagId,lFromTag,targetXrTimeMs,confidence=1,handHistory}){
    if(!isRigidPose(lFromTag)||!Number.isFinite(targetXrTimeMs)||!handHistory){
      return {changed:false,event:null,mode:states.get(tagId)?.mode??'free'};
    }
    const st=getOrCreate(tagId);
    if(confidence<cfg.minOpticalConfidence){
      return {changed:false,event:null,mode:st.mode,handedness:st.handedness};
    }

    const tagPos=[lFromTag[12],lFromTag[13],lFromTag[14]];

    // Active COUPLED watchdog & soft offset adaptation.
    if(st.mode==='coupled'){
      const hSample=handHistory.sampleAt(st.handedness,targetXrTimeMs);
      if(!hSample){
        if(targetXrTimeMs-st.lastOpticalXrMs>cfg.opticalStaleTimeoutMs){
          return release(st,'hand-history-missing',lFromTag);
        }
        return {changed:false,event:null,mode:'coupled',handedness:st.handedness};
      }
      if(hSample.frameKind!==st.frameKind){
        return release(st,'hand-frame-changed',lFromTag);
      }
      const predicted=product(hSample.lFromHand,st.handFromTag);
      const err=poseDelta(predicted,lFromTag);
      const posLimit=cfg.watchdogPosBaseM+cfg.watchdogPosLeverSlope*st.leverArmM;
      const curDist=minAnchorDistance(hSample,tagPos);
      const pullLimit=st.lockGripDistM!=null
        ?Math.min(cfg.attachRadiusM+0.03,st.lockGripDistM+cfg.watchdogPullMarginM)
        :Math.max(cfg.attachRadiusM+0.03,st.leverArmM+0.05);
      const handPulledAway=curDist>pullLimit;
      let tagStationary=false;
      if(st.lastOpticalTagPose){
        const optPredMove=poseDelta(st.lastOpticalTagPose,predicted);
        const optTagMove=poseDelta(st.lastOpticalTagPose,lFromTag);
        tagStationary=optPredMove.posM>=cfg.watchdogStationaryMinHandM&&
          optTagMove.posM<=cfg.watchdogStationaryMaxRatio*optPredMove.posM;
      }

      if(tagStationary||handPulledAway||err.posM>posLimit||err.angDeg>cfg.watchdogAngDeg){
        st.disagreeCount++;
        const pinchOpened=st.gripKind==='pinch'&&hSample.pinchDistM!=null&&st.lockPinchDistM!=null&&
          hSample.pinchDistM>st.lockPinchDistM+cfg.pinchReleaseDeltaM*0.75;
        // Immediate 1-frame release on stationary tag, pull-away, severe disagreement, or open fingers; 2-frame confirm on borderline PnP jitter.
        if(
          tagStationary||
          handPulledAway||
          pinchOpened||
          st.disagreeCount>=2||
          err.posM>posLimit*1.35||
          err.angDeg>cfg.watchdogAngDeg*1.35
        ){
          const reason=tagStationary?'tag-stationary':(handPulledAway?'hand-pulled-away':'optical-disagreement');
          return release(st,reason,lFromTag);
        }
        return {changed:false,event:null,mode:'coupled',handedness:st.handedness,residualMm:err.posM*1000};
      }

      // Consistent observation: softly adapt handFromTag to cancel finger flex & slow hand drift.
      st.disagreeCount=0;
      st.lastOpticalXrMs=targetXrTimeMs;
      st.lastOpticalTagPose=lFromTag.slice();
      const measuredOffset=product(inverse(hSample.lFromHand),lFromTag);
      const alpha=Math.max(0.05,Math.min(0.6,cfg.adaptAlpha*confidence));
      st.handFromTag=blendRigid(st.handFromTag,measuredOffset,alpha);
      st.leverArmM=Math.hypot(st.handFromTag[12],st.handFromTag[13],st.handFromTag[14]);
      if(st.lockGripDistM!=null)st.lockGripDistM=st.lockGripDistM*(1-alpha)+curDist*alpha;
      return {
        changed:false,
        event:'adapted',
        mode:'coupled',
        handedness:st.handedness,
        gripKind:st.gripKind,
        residualMm:err.posM*1000,
      };
    }

    // FREE or CANDIDATE: evaluate each hand for rigid-body kinematic coherence.
    let bestLock=null;
    let hasAnyCandidate=false;

    for(const handedness of HANDS){
      if(isHandOccupiedByOther(handedness,tagId)){
        st.candidates[handedness]=null;
        continue;
      }
      const hSample=handHistory.sampleAt(handedness,targetXrTimeMs);
      if(!hSample){
        st.candidates[handedness]=null;
        continue;
      }
      const gripDist=minAnchorDistance(hSample,tagPos);
      if(gripDist>cfg.attachRadiusM){
        st.candidates[handedness]=null;
        continue;
      }
      hasAnyCandidate=true;
      const prev=st.candidates[handedness];
      if(!prev||prev.frameKind!==hSample.frameKind||targetXrTimeMs<=prev.xrTimeMs||targetXrTimeMs-prev.xrTimeMs>cfg.candidateMaxAgeMs){
        st.candidates[handedness]={
          xrTimeMs:targetXrTimeMs,
          frameKind:hSample.frameKind,
          lFromHand:hSample.lFromHand.slice(),
          lFromTag:lFromTag.slice(),
          handFromTag:product(inverse(hSample.lFromHand),lFromTag),
          gripDist,
          pinchDistM:hSample.pinchDistM,
          fingerDistM:minPinchFingerDistance(hSample,tagPos),
          confirmCount:0,
        };
        continue;
      }

      const handMove=poseDelta(prev.lFromHand,hSample.lFromHand);
      const tagMove=poseDelta(prev.lFromTag,lFromTag);
      const predicted=product(hSample.lFromHand,prev.handFromTag);
      const expectedMove=poseDelta(prev.lFromTag,predicted);
      const rigidErr=poseDelta(predicted,lFromTag);
      const leverArmM=Math.hypot(prev.handFromTag[12],prev.handFromTag[13],prev.handFromTag[14]);
      const lockPosLimit=cfg.lockPosBaseM+cfg.lockPosLeverSlope*leverArmM;

      const posExcited=expectedMove.posM>=cfg.minExcitePosM&&tagMove.posM>=cfg.minExcitePosM*0.72;
      const angExcited=expectedMove.angDeg>=cfg.minExciteAngDeg&&tagMove.angDeg>=cfg.minExciteAngDeg*0.72;
      const excited=posExcited||angExcited;

      const posRatioOk=!posExcited||(
        tagMove.posM>=expectedMove.posM*cfg.minMotionRatio&&
        tagMove.posM<=expectedMove.posM*cfg.maxMotionRatio&&
        rigidErr.posM<=cfg.lockStaticRatio*Math.min(tagMove.posM,expectedMove.posM)
      );
      const pureRotPosOk=posExcited||(
        expectedMove.posM<cfg.minExcitePosM&&
        tagMove.posM<cfg.minExcitePosM&&
        rigidErr.posM<=Math.min(0.012,lockPosLimit)
      );
      const angRatioOk=!angExcited||(
        rigidErr.angDeg<=cfg.lockStaticRatio*Math.min(tagMove.angDeg,expectedMove.angDeg)
      );

      const coherent=
        excited&&
        leverArmM<=cfg.maxLeverArmM&&
        rigidErr.posM<=lockPosLimit&&
        rigidErr.angDeg<=cfg.lockAngDeg&&
        posRatioOk&&
        pureRotPosOk&&
        angRatioOk;

      if(coherent){
        const isPinch=gripDist<=cfg.pinchGripDistM&&hSample.pinchDistM!=null&&hSample.pinchDistM<=cfg.pinchApertureMaxM;
        const needed=isPinch?cfg.pinchConfirmDeltas:cfg.wandConfirmDeltas;
        const nextConfirm=(prev.confirmCount||0)+1;
        if(nextConfirm>=needed){
          const score=rigidErr.posM+rigidErr.angDeg*0.001+gripDist*0.05;
          if(!bestLock||score<bestLock.score){
            bestLock={
              handedness,
              score,
              gripDist,
              leverArmM,
              hSample,
              rigidErr,
              isPinch,
            };
          }
        }else{
          st.candidates[handedness]={
            xrTimeMs:targetXrTimeMs,
            frameKind:hSample.frameKind,
            lFromHand:hSample.lFromHand.slice(),
            lFromTag:lFromTag.slice(),
            handFromTag:product(inverse(hSample.lFromHand),lFromTag),
            gripDist,
            pinchDistM:hSample.pinchDistM,
            fingerDistM:minPinchFingerDistance(hSample,tagPos),
            confirmCount:nextConfirm,
          };
        }
      }else if(
        handMove.posM>=cfg.minExcitePosM||tagMove.posM>=cfg.minExcitePosM||
        expectedMove.posM>=cfg.minExcitePosM||handMove.angDeg>=cfg.minExciteAngDeg||
        tagMove.angDeg>=cfg.minExciteAngDeg
      ){
        // Motion occurred without rigid coherence (pass-over or external push): reset baseline to current frame.
        st.candidates[handedness]={
          xrTimeMs:targetXrTimeMs,
          frameKind:hSample.frameKind,
          lFromHand:hSample.lFromHand.slice(),
          lFromTag:lFromTag.slice(),
          handFromTag:product(inverse(hSample.lFromHand),lFromTag),
          gripDist,
          pinchDistM:hSample.pinchDistM,
          fingerDistM:minPinchFingerDistance(hSample,tagPos),
          confirmCount:0,
        };
      }
      // If neither moved yet, keep the original baseline so slow smooth pickup accumulates cleanly.
    }

    if(bestLock){
      const {handedness,gripDist,hSample,rigidErr,isPinch}=bestLock;
      const offset=product(inverse(hSample.lFromHand),lFromTag);
      st.mode='coupled';
      st.handedness=handedness;
      st.gripKind=isPinch?'pinch':'wand';
      st.frameKind=hSample.frameKind;
      st.handFromTag=offset;
      st.leverArmM=Math.hypot(offset[12],offset[13],offset[14]);
      st.lockGripDistM=gripDist;
      st.lockPinchDistM=hSample.pinchDistM;
      st.lockFingerDistM=minPinchFingerDistance(hSample,tagPos);
      st.lastOpticalXrMs=targetXrTimeMs;
      st.lastOpticalTagPose=lFromTag.slice();
      st.disagreeCount=0;
      st.candidates.left=null;
      st.candidates.right=null;
      return {
        changed:true,
        event:'coupled',
        mode:'coupled',
        handedness,
        gripKind:st.gripKind,
        residualMm:rigidErr.posM*1000,
      };
    }

    st.mode=hasAnyCandidate?'candidate':'free';
    return {changed:false,event:null,mode:st.mode,handedness:null};
  }

  function predict(tagId,handHistory,nowXrTimeMs){
    const st=states.get(tagId);
    if(!st||st.mode!=='coupled'||!handHistory||!Number.isFinite(nowXrTimeMs)){
      return {coupled:false,mode:st?.mode??'free',handedness:null,gripKind:null,lFromTag:null,released:false,reason:null};
    }

    if(nowXrTimeMs-st.lastOpticalXrMs>cfg.opticalStaleTimeoutMs){
      const rel=release(st,'optical-timeout');
      return {coupled:false,mode:'free',handedness:null,gripKind:null,lFromTag:null,released:true,reason:rel.reason,prevHand:rel.handedness,lastOpticalPose:rel.lastOpticalPose};
    }

    const latest=handHistory.latest(st.handedness,nowXrTimeMs,cfg.handLostTimeoutMs);
    if(!latest){
      const rel=release(st,'hand-tracking-lost');
      return {coupled:false,mode:'free',handedness:null,gripKind:null,lFromTag:null,released:true,reason:rel.reason,prevHand:rel.handedness,lastOpticalPose:rel.lastOpticalPose};
    }

    if(latest.frameKind!==st.frameKind){
      const rel=release(st,'hand-frame-changed');
      return {coupled:false,mode:'free',handedness:null,gripKind:null,lFromTag:null,released:true,reason:rel.reason,prevHand:rel.handedness,lastOpticalPose:rel.lastOpticalPose};
    }

    const predicted=product(latest.lFromHand,st.handFromTag);

    if(st.lastOpticalTagPose){
      const driftM=Math.hypot(
        predicted[12]-st.lastOpticalTagPose[12],
        predicted[13]-st.lastOpticalTagPose[13],
        predicted[14]-st.lastOpticalTagPose[14],
      );
      if(driftM>cfg.maxUnconfirmedDriftM){
        const rel=release(st,'unconfirmed-drift');
        return {coupled:false,mode:'free',handedness:null,gripKind:null,lFromTag:null,released:true,reason:rel.reason,prevHand:rel.handedness,lastOpticalPose:rel.lastOpticalPose};
      }
    }

    // 0-ms 90 Hz pinch-open instant release (armed only for direct pinched cards, never false-triggers on wands/sticks).
    if(st.gripKind==='pinch'&&latest.pinchDistM!=null&&st.lockPinchDistM!=null){
      const opened=latest.pinchDistM>st.lockPinchDistM+cfg.pinchReleaseDeltaM;
      if(opened){
        const predPos=[predicted[12],predicted[13],predicted[14]];
        const fingerDist=minPinchFingerDistance(latest,predPos);
        const pulled=st.lockFingerDistM==null||fingerDist>st.lockFingerDistM+cfg.pinchReleasePullM;
        if(pulled||latest.pinchDistM>st.lockPinchDistM+cfg.pinchReleaseDeltaM*1.8){
          const rel=release(st,'pinch-opened');
          return {coupled:false,mode:'free',handedness:null,gripKind:null,lFromTag:null,released:true,reason:rel.reason,prevHand:rel.handedness,lastOpticalPose:rel.lastOpticalPose};
        }
      }
    }

    return {
      coupled:true,
      mode:'coupled',
      handedness:st.handedness,
      gripKind:st.gripKind,
      lFromTag:predicted,
      opticalAgeMs:Math.max(0,nowXrTimeMs-st.lastOpticalXrMs),
      released:false,
      reason:null,
    };
  }

  function isHandCarrying(handedness,tagId=null){
    if(tagId!=null){
      const st=states.get(tagId);
      return Boolean(st&&st.mode==='coupled'&&st.handedness===handedness);
    }
    for(const st of states.values()){
      if(st.mode==='coupled'&&st.handedness===handedness)return true;
    }
    return false;
  }

  function stateOf(tagId){
    const st=states.get(tagId);
    if(!st)return {mode:'free',handedness:null,gripKind:null};
    return {mode:st.mode,handedness:st.handedness,gripKind:st.gripKind,leverArmM:st.leverArmM};
  }

  function reset(tagId=null){
    if(tagId!=null)states.delete(tagId);
    else states.clear();
  }

  return {observeOptical,predict,isHandCarrying,stateOf,reset};
}
