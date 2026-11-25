import {anchorFeatureState,poseToRigidParts,observationIsFresh,det4,matToQuat,composeRigid,quatDot,quatNormalize,quatAngle} from './runtime.mjs';

export const ANCHOR_QUALITY=Object.freeze({minSamples:5,minSpanMs:1000,maxWindowMs:2500,
  maxGapMs:700,maxAgeMs:650,maxRmsPx:1,maxPositionSpreadM:.005,maxOrientationSpreadDeg:1.5,softReject:false});
const copy=value=>JSON.parse(JSON.stringify(value));
const median=a=>{const s=a.slice().sort((x,y)=>x-y),n=s.length;return n%2?s[(n-1)/2]:(s[n/2-1]+s[n/2])/2;};
const dot3=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const dist3=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);

export function isRigidPose(array) {
  if(!array||array.length!==16) return false;
  for(let i=0;i<16;i++) if(!Number.isFinite(array[i])) return false;
  if(Math.abs(array[3])+Math.abs(array[7])+Math.abs(array[11])+Math.abs(array[15]-1)>=1e-6) return false;
  const a0x=array[0],a0y=array[1],a0z=array[2];
  const a1x=array[4],a1y=array[5],a1z=array[6];
  const a2x=array[8],a2y=array[9],a2z=array[10];
  if(Math.abs(Math.hypot(a0x,a0y,a0z)-1)>=1e-3||
     Math.abs(Math.hypot(a1x,a1y,a1z)-1)>=1e-3||
     Math.abs(Math.hypot(a2x,a2y,a2z)-1)>=1e-3) return false;
  if(Math.abs(a0x*a1x+a0y*a1y+a0z*a1z)>=1e-3||
     Math.abs(a0x*a2x+a0y*a2y+a0z*a2z)>=1e-3||
     Math.abs(a1x*a2x+a1y*a2y+a1z*a2z)>=1e-3) return false;
  return Math.abs(det4(array)-1)<1e-3;
}

export function createStableLCollector(options={}) {
  const limits={...ANCHOR_QUALITY,...options};
  let requiredIds=(options.tagIds??[350,351,352]).slice().sort((a,b)=>a-b);
  let rows=[],lastIdentity=null,lastCapture=null,scope=null,reason='Hold all three reference tags steady in view',ready=null;
  let lastSpreadMm=null,lastSpreadDeg=null,lastRmsPx=null,lastSeenTagCount=0;
  function reset(why='Hold all three reference tags steady in view') {rows=[];lastIdentity=null;lastCapture=null;scope=null;ready=null;lastSpreadMm=null;lastSpreadDeg=null;lastRmsPx=null;reason=why;}
  function setTagIds(nextIds) {
    if(!Array.isArray(nextIds)||nextIds.length<3||nextIds.some(v=>!Number.isInteger(v))) throw new TypeError('Invalid reference tag IDs');
    requiredIds=nextIds.slice().sort((a,b)=>a-b);
    reset();
  }
  function reject(why) {reset(why);return state();}
  function softSkip(why,atMs) {
    if(!limits.softReject) return reject(why);
    ready=null;
    if(Number.isFinite(atMs)) {
      rows=rows.filter(r=>atMs-r.at<=limits.maxWindowMs);
      if(rows.length&&atMs-rows.at(-1).at>limits.maxGapMs) rows=[];
    }
    reason=why;
    return state();
  }
  function state() {const spanMs=rows.length>=2?rows.at(-1).at-rows[0].at:0;
    return {count:rows.length,required:limits.minSamples,spanMs,ready:Boolean(ready),reason,
      tagIds:requiredIds.slice(),seenTagCount:lastSeenTagCount,spreadMm:lastSpreadMm,spreadDeg:lastSpreadDeg,lastRmsPx,
      limits:{...limits},quality:ready?copy(ready):null};}
  function summary() {const spanMs=rows.length>=2?rows.at(-1).at-rows[0].at:0;
    return {count:rows.length,required:limits.minSamples,spanMs,ready:Boolean(ready),reason,
      seenTagCount:lastSeenTagCount,spreadMm:lastSpreadMm,spreadDeg:lastSpreadDeg,lastRmsPx,limits};}
  function observe({sample,tool,candidate,worldFromL},context,now) {
    if(!observationIsFresh(sample,context,now,limits.maxAgeMs)||sample?.frameEvidence?.pixelsChanged===false) return reject('Fresh marker observation required');
    const ids=tool?.tagIds?.slice().sort((a,b)=>a-b);
    lastSeenTagCount=ids?ids.filter(id=>requiredIds.includes(id)).length:0;
    if(Number.isFinite(tool?.reprojectionRmsPx)) lastRmsPx=tool.reprojectionRmsPx;
    const atMs=sample.submittedAtPerformanceMs;
    if(!ids||ids.length!==requiredIds.length||ids.some((v,i)=>v!==requiredIds[i])||tool.mode!=='joint-rigid-tags'||tool.pointCount!==requiredIds.length*4)
      return softSkip(`All reference tags ${requiredIds.join(' / ')} required`,atMs);
    if(!Number.isFinite(tool.reprojectionRmsPx)||tool.reprojectionRmsPx>limits.maxRmsPx)
      return softSkip('Marker image not sharp enough yet',atMs);
    if(candidate?.association!=='validated-profile-at-capture-time'||!candidate.calibrationProfileId||
      candidate.sessionId!==context.sessionId||candidate.spaceEpoch!==context.spaceEpoch||!isRigidPose(worldFromL))
      return reject('Calibrated spatial association missing');
    const capture=sample.videoFrameMetadata?.captureTime;
    if(!Number.isFinite(capture)) return reject('Capture timestamp missing');
    const newScope=`${context.sessionId}/${context.spaceEpoch}/${sample.generation}/${candidate.calibrationProfileId}`;
    if(scope&&scope!==newScope) reset('New camera session');
    const identity=`${newScope}/${sample.serial}`;
    if(identity===lastIdentity) return state();
    if(lastCapture!=null&&capture<=lastCapture) return reject('New camera frame required');
    const at=sample.submittedAtPerformanceMs;
    if(rows.length&&(at-rows.at(-1).at>limits.maxGapMs||at<=rows.at(-1).at)) reset('Detection interrupted');
    scope=newScope;lastIdentity=identity;lastCapture=capture;ready=null;lastRmsPx=tool.reprojectionRmsPx;
    rows.push({at,capture,serial:sample.serial,generation:sample.generation,matrix:Array.from(worldFromL),rms:tool.reprojectionRmsPx});
    rows=rows.filter(r=>at-r.at<=limits.maxWindowMs);
    while(rows.length>limits.minSamples&&at-rows[1].at>=limits.minSpanMs) rows.shift();
    if(rows.length>=2) {
      const pos=[12,13,14].map(i=>median(rows.map(r=>r.matrix[i])));
      const qs=rows.map(r=>matToQuat(r.matrix));
      const qsum=[0,0,0,0],q0=qs[0];
      for(const q of qs) {const s=quatDot(q0,q)<0?-1:1;qsum[0]+=s*q[0];qsum[1]+=s*q[1];qsum[2]+=s*q[2];qsum[3]+=s*q[3];}
      const qmean=quatNormalize(qsum);
      lastSpreadMm=Math.max(...rows.map(r=>dist3([r.matrix[12],r.matrix[13],r.matrix[14]],pos)))*1000;
      lastSpreadDeg=Math.max(...qs.map(q=>quatAngle(qmean,q)*180/Math.PI));
    }
    if(rows.length<limits.minSamples||at-rows[0].at<limits.minSpanMs) {reason='Collecting steady views';return state();}
    const position=[12,13,14].map(i=>median(rows.map(r=>r.matrix[i])));
    const quats=rows.map(r=>matToQuat(r.matrix));
    const sum=[0,0,0,0],first=quats[0];
    for(const q of quats) {const sign=quatDot(first,q)<0?-1:1;sum[0]+=sign*q[0];sum[1]+=sign*q[1];sum[2]+=sign*q[2];sum[3]+=sign*q[3];}
    const qmean=quatNormalize(sum);
    const positionSpreadM=Math.max(...rows.map(r=>dist3([r.matrix[12],r.matrix[13],r.matrix[14]],position)));
    const orientationSpreadDeg=Math.max(...quats.map(q=>quatAngle(qmean,q)*180/Math.PI));
    lastSpreadMm=positionSpreadM*1000;lastSpreadDeg=orientationSpreadDeg;
    if(positionSpreadM>limits.maxPositionSpreadM||orientationSpreadDeg>limits.maxOrientationSpreadDeg) {
      reason='Hold L-angle steady · views do not agree yet';return state();
    }
    ready={worldFromL:composeRigid(position,qmean),
      sessionId:context.sessionId,spaceEpoch:context.spaceEpoch,calibrationProfileId:candidate.calibrationProfileId,
      generation:sample.generation,serials:rows.map(r=>r.serial),firstAtPerformanceMs:rows[0].at,
      lastAtPerformanceMs:at,spanMs:at-rows[0].at,positionSpreadMm:positionSpreadM*1000,
      orientationSpreadDeg,maxReprojectionRmsPx:Math.max(...rows.map(r=>r.rms))};
    reason='L-pose ready';return state();
  }
  return {observe,reset,setTagIds,state,summary};
}

// All anchor creation calls happen synchronously inside update(frame) while the XRFrame is active.
export function createLAnchorController({onEvent=()=>{},onCreated=()=>{},
  makeRigidTransform=(position,orientation)=>new XRRigidTransform(position,orientation),
  quality={},creationTimeoutMs=8000,clock=()=>performance.now()}={}) {
  const collector=createStableLCollector(quality);
  let session=null,reference=null,context=null,token=0,anchor=null,phase='idle',seed=null,matrix=null;
  let created=false,attempts=0,posesTracked=0,posesMissing=0,createdAt=null,creatingAt=null,error=null,capability=null;
  const emit=(type,detail,now)=>onEvent(type,{...copy(detail),...context,atPerformanceMs:now});
  const dispose=a=>{try {a?.delete?.();} catch { /* Session cleanup must continue. */ }};
  function state() {return {phase,created,active:Boolean(anchor),pending:phase==='creating',createAttempts:attempts,
    posesTracked,posesMissing,createdAtPerformanceMs:createdAt,creationStartedAtPerformanceMs:creatingAt,
    error,capability:capability?{...capability}:null,quality:collector.state(),seed:seed?copy(seed):null,
    persistent:false};}
  function summary() {return {phase,created,active:Boolean(anchor),pending:phase==='creating',
    posesTracked,posesMissing,quality:collector.summary()};}
  function end(now,reason='session-ended') {
    ++token;const active=Boolean(anchor),wasPending=phase==='creating';dispose(anchor);anchor=null;matrix=null;collector.reset();
    if(session) {phase='ended';emit('anchor-session-ended',{reason,deleted:active,pendingDiscarded:wasPending},now);}
    session=null;reference=null;context=null;
  }
  function begin(newSession,newReference,newContext,now) {
    if(session) end(now,'session-replaced');
    session=newSession;reference=newReference;context={...newContext};phase='collecting';seed=null;matrix=null;anchor=null;
    created=false;attempts=0;posesTracked=0;posesMissing=0;createdAt=null;creatingAt=null;error=null;capability=null;
    collector.reset();emit('anchor-collection-start',{limits:collector.state().limits},now);
  }
  function reanchor(now,reason='user-requested') {
    if(!session||phase==='unsupported') return false;
    ++token;const hadAnchor=Boolean(anchor),wasPending=phase==='creating';
    dispose(anchor);anchor=null;matrix=null;seed=null;created=false;creatingAt=null;error=null;
    collector.reset('Re-anchoring: hold all three reference tags steady in view');
    phase='collecting';
    emit('anchor-reanchor-requested',{reason,deleted:hadAnchor,pendingDiscarded:wasPending},now);
    return true;
  }
  function observe(input,now) {
    if(!session||!['collecting','ready'].includes(phase)) return state();
    const result=collector.observe(input,context,now);
    if(result.ready) {seed=result.quality;phase='ready';emit('anchor-seed-ready',seed,now);}
    else {seed=null;phase='collecting';}
    return state();
  }
  function resetReference(newReference,newContext,now) {
    if(!session) return;
    ++token;reference=newReference;context={...newContext};collector.reset();matrix=null;
    if(!anchor) {seed=null;creatingAt=null;phase='collecting';error=null;}
    else phase='lost';
    emit('anchor-reference-reset',{keptNativeAnchor:Boolean(anchor)},now);
  }
  function update(frame,now,{viewerTracked=true}={}) {
    matrix=null;
    if(!session||frame?.session!==session) return null;
    if(phase==='collecting'||phase==='ready') {
      capability=anchorFeatureState(session,frame);
      if(!capability.canAttempt) {phase='unsupported';emit('anchor-unavailable',{capability},now);return null;}
    }
    if(phase==='creating'&&now-creatingAt>creationTimeoutMs) {
      ++token;phase='failed';error='Anchor creation timed out';emit('anchor-create-failed',{error,timeout:true},now);
    }
    if(phase==='ready'&&viewerTracked) {
      if(now-seed.lastAtPerformanceMs>collector.state().limits.maxAgeMs||now<seed.lastAtPerformanceMs) {
        collector.reset('Marker pose expired');seed=null;phase='collecting';return null;
      }
      const parts=poseToRigidParts(seed.worldFromL),currentSession=session,request=++token;
      phase='creating';attempts++;creatingAt=now;
      emit('anchor-create-request',{seed,capability},now);
      try {
        if(!parts) throw new Error('Invalid L-pose');
        const rigid=makeRigidTransform(parts.position,parts.orientation);
        const pending=frame.createAnchor(rigid,reference);
        Promise.resolve(pending).then(result=>{
          if(request!==token||session!==currentSession) {dispose(result);return;}
          if(!result?.anchorSpace) {dispose(result);throw new Error('No valid XRAnchor instance returned');}
          anchor=result;created=true;createdAt=clock();phase='created';
          emit('anchor-created',{seed,createAttempts:attempts},createdAt);
          onCreated(state());
        }).catch(failure=>{
          if(request!==token||session!==currentSession) return;
          phase='failed';error=String(failure?.message??failure);emit('anchor-create-failed',{error},clock());
        });
      } catch(failure) {phase='failed';error=String(failure?.message??failure);emit('anchor-create-failed',{error},now);}
    }
    if(!anchor) return null;
    let pose=null;
    try {pose=viewerTracked?frame.getPose(anchor.anchorSpace,reference):null;} catch(failure) {error=String(failure?.message??failure);}
    if(pose&&!pose.emulatedPosition&&isRigidPose(pose.transform?.matrix)) {
      matrix=Array.from(pose.transform.matrix);posesTracked++;
      if(phase!=='tracking') {phase='tracking';error=null;emit('anchor-tracking',{restored:posesMissing>0},now);}
    } else {
      posesMissing++;
      if(phase!=='lost') {phase='lost';emit('anchor-tracking-lost',{},now);}
    }
    return matrix?.slice()??null;
  }
  return {begin,reanchor,observe,update,resetReference,setTagIds:collector.setTagIds,end,state,phase:()=>phase,summary};
}
