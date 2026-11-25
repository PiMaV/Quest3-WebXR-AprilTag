import {referenceTools} from './rig.mjs';
import {diversity,consistency,projectResidual} from './solver.mjs';
import {fitPlanarPose} from './rig.mjs';
import {cvToGL,observationIsFresh,poseAt,product} from './runtime.mjs';
import {PROFILE_SCHEMA,PIPELINE,createProfileStore} from './profile.mjs';
const clone=v=>structuredClone(v);
export function createCalibrationAssistant({rig,device,imageSize,store=createProfileStore(),onChange=()=>{},
  onProfile=()=>{},workerFactory=()=>new Worker(new URL('./calibration-worker.mjs',import.meta.url),{type:'module'}),model='centered-square',
  scanViews=36,verificationViews=10}={}){
  const ids=referenceTools(rig)[0].tags.map(t=>t.id);
  let phase='idle',message='Prepare camera and enter AR',rows=[],seen=new Set(),lastCapture=-Infinity,scope=null;
  let candidate=null,profile=null,job=null,token=0,worker=null,timeout=null,verification=null,persisted=false,disposed=false,nextFitAt=scanViews,fitRows=[],checkFailure=null;
  function state(){return {phase,message,count:rows.length,required:phase==='scanning'?nextFitAt:phase==='solving'?rows.length:verificationViews,
    profileId:profile?.id??candidate?.id??null,persisted,verification:clone(verification),checkFailure:clone(checkFailure)};}
  function notify(){onChange(state());}
  function resetRows(){rows=[];seen=new Set();lastCapture=-Infinity;nextFitAt=scanViews;}
  function cancel(){++token;clearTimeout(timeout);timeout=null;worker?.terminate();worker=null;job=null;}
  function beginScan(why='Keep the L-angle still. Move head closer/farther, left/right, and up/down.'){
    cancel();candidate=null;profile=null;verification=null;checkFailure=null;persisted=false;resetRows();phase='scanning';message=why;onProfile(null);notify();
  }
  async function begin({force=false}={}){
    cancel();const request=token;let cached=force?null:await store.load(device);if(disposed||request!==token)return;
    if(cached&&cached.imageSize.some((v,i)=>v!==imageSize[i])){
      const [w,h]=imageSize,[sw,sh]=cached.imageSize;
      if(Math.abs(w/h-sw/sh)>.001)cached=null;
      else{cached=clone(cached);cached.K={width:w,height:h,fx:cached.K.fx*w/sw,fy:cached.K.fy*h/sh,cx:cached.K.cx*w/sw,cy:cached.K.cy*h/sh};cached.imageSize=clone(imageSize);}
    }
    resetRows();scope=null;candidate=cached;profile=null;verification=null;checkFailure=null;persisted=Boolean(cached);onProfile(null);
    phase=cached?'checking':'scanning';message=cached?'Verifying saved profile: look at the L-angle and shift your head slightly.':'Keep the L-angle still. Move head around multiple axes and vary distance.';notify();
  }
  function failed(why){cancel();phase='failed';message=why;candidate=null;profile=null;onProfile(null);notify();}
  function fit(history){
    // Fast detectors can return before the +250 ms search horizon exists.
    // Keep those observations, but fit only rows already bracketed by XR history.
    const captured=clone(rows.filter(r=>r.captureTime-251>=history[0].xrRafTimeMs&&r.captureTime+251<=history.at(-1).xrRafTimeMs));
    if(captured.length<30)return;
    const request=++token;phase='solving';message='Solving calibration…';notify();
    worker=workerFactory();timeout=setTimeout(()=>{if(request===token)failed('Calibration solver timed out. Please try again.');},60000);
    worker.onerror=e=>{if(request===token)failed(`Calibration solver failed: ${e.message}`);};
    worker.onmessage=({data})=>{
      if(disposed||request!==token||data.id!==request)return;
      if(data.type==='progress'){message=data.message;notify();return;}
      clearTimeout(timeout);worker?.terminate();worker=null;
      if(data.type==='error'){
        if(rows.length<180){nextFitAt=rows.length+15;phase='scanning';message=`${data.message}. Collect additional viewpoints.`;notify();}
        else failed(data.message);return;
      }
      const result=data.result;
      if(!result?.validation?.passed){
        if(rows.length<180){nextFitAt=rows.length+15;phase='scanning';message='Validation needs more diverse viewpoints. Keep L-angle still and rotate head around multiple axes.';notify();}
        else failed('Calibration did not meet quality gates. Restart with more diverse viewpoints.');return;
      }
      const {worldFromRig,cost,...portable}=result;
      const {reference,...metrics}=portable.validation;portable.validation=metrics;
      candidate={schema:PROFILE_SCHEMA,id:`cal-${device.key.slice(0,12)}-${Date.now()}`,device:clone(device),imageSize:clone(imageSize),
        ...portable,createdAt:new Date().toISOString(),pipeline:PIPELINE};
      fitRows=captured;resetRows();phase='verifying';message='Capture fresh verification views: shift head position and distance while keeping the L-angle still.';notify();
    };
    worker.postMessage({id:request,payload:{rows:captured,history:clone(history),imageSize,model}});job=request;
  }
  async function accept(report){
    const request=token,wasCached=phase==='checking',{reference,...metrics}=report;candidate.verification={...metrics,passed:true,views:rows.length,
      checkedAt:new Date().toISOString(),kind:wasCached?'new-session-check':'new-capture-after-fit'};
    profile=clone(candidate);phase='saving';notify();
    try{await store.save(profile);persisted=true;}catch{persisted=false;}
    if(disposed||request!==token)return;
    phase='ready';message=persisted?'Profile verified and saved. Hold all tags steady to create anchor.':'Profile verified (browser storage unavailable; active for this session only).';
    onProfile(clone(profile));notify();
  }
  function observe(observation,sample,history,now){
    if(!['scanning','checking','verifying'].includes(phase))return state();
    if(!observationIsFresh(sample,sample.context,now)||!sample.context.sessionId||sample.imageSize.some((v,i)=>v!==imageSize[i])||
      sample.sourceSize.some((v,i)=>v!==device.mode.sourceSize[i]))return state();
    const newScope=`${sample.context.sessionId}/${sample.context.spaceEpoch}/${sample.generation}`;
    if(scope&&scope!==newScope){scope=newScope;beginScan('Camera or XR space changed. Collect fresh calibration views.');}
    scope=newScope;
    const cap=sample.videoFrameMetadata?.captureTime;
    const tags=observation.tags.filter(t=>ids.includes(t.id));
    if(!Number.isFinite(cap)||cap<=lastCapture||sample.submittedAtPerformanceMs-cap<-50||sample.submittedAtPerformanceMs-cap>1000){message='Fresh camera frames with captureTime required.';notify();return state();}
    if(tags.length<2){message='Keep at least two (ideally all three) reference tags in view.';notify();return state();}
    const hasOutlier=Array.isArray(observation.rejected)&&observation.rejected.some(r=>ids.includes(r.id)&&(r.reason==='rigid_tag_outlier'||r.reason==='inconsistent_rigid_geometry'));
    if(phase!=='scanning'&&(tags.length!==ids.length||hasOutlier)){message='Keep all reference tags cleanly visible for profile verification.';notify();return state();}
    let lo=0;
    if(history.length>0){
      const minTargetT=(rows.length?Math.min(rows[0].captureTime,cap):cap)-260;
      let hi=history.length-1;
      if(history[0].xrRafTimeMs<=minTargetT){
        while(hi-lo>1){const mid=(lo+hi)>>1;if(history[mid].xrRafTimeMs<=minTargetT)lo=mid;else hi=mid;}
      }
    }
    const valid=[];
    for(let i=lo;i<history.length;i++){
      const p=history[i];
      if(p.sessionId===sample.context.sessionId&&p.spaceEpoch===sample.context.spaceEpoch&&p.worldFromViewer&&!p.emulatedPosition)
        valid.push({xrRafTimeMs:p.xrRafTimeMs,worldFromViewer:p.worldFromViewer,scope});
    }
    if(valid.length<2||cap-251<valid[0].xrRafTimeMs)return state();
    const signature=JSON.stringify(tags.map(t=>[t.id,t.pixels]));if(seen.has(signature))return state();
    seen.add(signature);lastCapture=cap;rows.push({scope,captureTime:cap,tags:clone(tags),serial:sample.serial});
    if(phase==='scanning'){
      const diverse=diversity(rows,valid);
      if(rows.length>=nextFitAt&&diverse.ok){fit(valid);return state();}
      if(rows.length>180){failed('More distinct perspectives needed instead of repeated views. Rotate head around multiple axes.');return state();}
      message=diverse.ok?'Keep varying viewpoints while the L-angle stays stationary.':diverse.reason;notify();return state();
    }
    const paired=[];
    for(const r of rows){const h=poseAt(valid,r.captureTime+candidate.timeOffsetMs);if(h)paired.push({row:r,h});}
    if(paired.length>=verificationViews){
      const poses=[],errors=[];let motion=0;
      for(let i=0;i<paired.length;i++){
        const {row:r,h}=paired[i];
        for(let j=0;j<i;j++){
          const hj=paired[j].h,d=Math.hypot(h[12]-hj[12],h[13]-hj[13],h[14]-hj[14]);
          if(d>motion)motion=d;
        }
        const camPose=fitPlanarPose(r.tags,candidate.K);
        poses.push(camPose?product(h,candidate.viewerFromCameraGL,cvToGL(camPose)):null);
        const res=camPose?projectResidual(camPose,r.tags,candidate.K):null;
        if(res)for(let k=0;k<res.length;k++)errors.push(res[k]);
        else errors.push(1e6);
      }
      const report=consistency(poses.slice(4),consistency(poses.slice(0,4)).reference);
      verification=report;
      let errSq=0;for(let k=0;k<errors.length;k++)errSq+=errors[k]*errors[k];
      report.reprojectionRmsPx=Math.sqrt(errSq/(errors.length/2));
      if(report.ok&&report.positionP95Mm<12&&report.angleP95Deg<1.5&&report.reprojectionRmsPx<1.2&&motion>.04){
        verification=report;accept(report);return state();
      }
      if(rows.length>=30){
        if(phase==='checking'){
          checkFailure={positionP95Mm:report.positionP95Mm,angleP95Deg:report.angleP95Deg,reprojectionRmsPx:report.reprojectionRmsPx,
            motionM:motion,limits:{positionP95Mm:12,angleP95Deg:1.5,reprojectionRmsPx:1.2,minMotionM:.04}};
          const why=report.angleP95Deg>=1.5?`orientation error ${report.angleP95Deg.toFixed(1)}° exceeds 1.5°`:
            report.positionP95Mm>=12?`position error ${report.positionP95Mm.toFixed(1)} mm exceeds 12 mm`:
            report.reprojectionRmsPx>=1.2?'marker image not sharp enough':motion<=.04?'head motion too small for verification':'spatial association incomplete';
          phase='check-failed';message=`Profile loaded; verification failed: ${why}. Retry check or explicitly recalibrate.`;
          onProfile(null);notify();return state();
        }
        const previous=phase==='verifying'?fitRows.concat(rows):[];
        beginScan('Verification requires a new fit. Saved profile remains intact; collect more viewpoints.');
        if(previous.length&&previous.length<180){
          rows=previous;seen=new Set(rows.map(r=>JSON.stringify(r.tags.map(t=>[t.id,t.pixels]))));lastCapture=rows.at(-1).captureTime;
          nextFitAt=rows.length+1;notify();
        }
        return state();
      }
      message=motion<=.04?'Shift your head side-to-side and up/down for verification.':'Collecting verification views; keep L-angle stationary.';
    }
    notify();return state();
  }
  function retryCheck(){
    if(!candidate||phase!=='check-failed')return false;
    cancel();resetRows();scope=null;profile=null;verification=null;checkFailure=null;phase='checking';
    message='Retrying saved profile verification: look steadily at the L-angle, then shift your head slightly.';onProfile(null);notify();return true;
  }
  return {begin,observe,state,phase:()=>phase,
    summary:()=>({phase,count:rows.length,required:phase==='scanning'?nextFitAt:phase==='solving'?rows.length:verificationViews,message}),
    retryCheck,retry:()=>beginScan(),profile:()=>profile&&clone(profile),
    reset(){cancel();scope=null;resetRows();phase='idle';candidate=null;profile=null;onProfile(null);notify();},
    dispose(){disposed=true;cancel();profile=null;candidate=null;phase='disposed';}};
}
