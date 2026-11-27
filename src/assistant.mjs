import {referenceTools} from './rig.mjs';
import {diversity,consistency,projectResidual} from './solver.mjs';
import {fitPlanarPose} from './rig.mjs';
import {cvToGL,observationIsFresh,poseAt,product} from './runtime.mjs';
import {PROFILE_SCHEMA,PIPELINE,CALIBRATION_SOLVER_VERSION,createProfileStore} from './profile.mjs';
const clone=v=>structuredClone(v);
/**
 * Create the guided calibration and cached-profile verification state machine
 * (`idle` -> `checking` / `scanning` -> `solving` -> `verifying` -> `ready`).
 */
export function createCalibrationAssistant({rig,device,imageSize,store=createProfileStore(),onChange=()=>{},
  onProfile=()=>{},workerFactory=()=>new Worker(new URL('./calibration-worker.mjs',import.meta.url),{type:'module'}),model='centered-square',
  scanViews=36,verificationViews=10,minSolverVersion=null}={}){
  const isRadialModel=model==='centered-radial'||model==='pinhole-radial';
  const effMinSolverVersion=minSolverVersion!==null?minSolverVersion:(isRadialModel?CALIBRATION_SOLVER_VERSION:null);
  const ids=referenceTools(rig)[0].tags.map(t=>t.id);
  let phase='idle',message='Prepare camera and enter AR',rows=[],seen=new Set(),lastCapture=-Infinity,scope=null;
  let candidate=null,profile=null,job=null,token=0,worker=null,timeout=null,verification=null,persisted=false,disposed=false,nextFitAt=scanViews,fitRows=[],checkFailure=null;
  let lastFitCapture=null,lastVerificationCapture=null,upgradeReason=null;
  function sliceHistoryForRows(rowList,histList,padMs=300){
    if(!Array.isArray(rowList)||!rowList.length||!Array.isArray(histList)||!histList.length)return [];
    let minT=Infinity,maxT=-Infinity;
    for(const r of rowList){if(r.captureTime<minT)minT=r.captureTime;if(r.captureTime>maxT)maxT=r.captureTime;}
    const t0=minT-padMs,t1=maxT+padMs,out=[];
    for(const p of histList){
      if(p.xrRafTimeMs<t0||p.xrRafTimeMs>t1||!p.worldFromViewer)continue;
      out.push({xrRafTimeMs:p.xrRafTimeMs,worldFromViewer:Array.from(p.worldFromViewer),scope:p.scope??scope});
    }
    return out;
  }
  function state(){return {phase,message,count:rows.length,required:phase==='scanning'?nextFitAt:phase==='solving'?rows.length:verificationViews,
    profileId:profile?.id??candidate?.id??null,persisted,verification:clone(verification),checkFailure:clone(checkFailure),upgradeReason};}
  function notify(){onChange(state());}
  function resetRows(){rows=[];seen=new Set();lastCapture=-Infinity;nextFitAt=scanViews;}
  function cancel(){++token;clearTimeout(timeout);timeout=null;worker?.terminate();worker=null;job=null;}
  function beginScan(why='Keep the L-angle still. Move head closer/farther, left/right, and up/down.'){
    cancel();candidate=null;profile=null;verification=null;checkFailure=null;persisted=false;upgradeReason=null;lastFitCapture=null;lastVerificationCapture=null;resetRows();phase='scanning';message=why;onProfile(null);notify();
  }
  async function begin({force=false}={}){
    cancel();const request=token;let cached=force?null:await store.load(device);if(disposed||request!==token)return;
    upgradeReason=null;
    if(cached){
      if(cached.model&&cached.model!==model){
        cached=null;upgradeReason='model-changed';
      }else if(effMinSolverVersion&&(cached.solverVersion??1)<effMinSolverVersion){
        cached=null;upgradeReason='solver-upgraded';
      }else if(effMinSolverVersion!=null){
        const val=cached.validation;
        const pFx=val?.planarFx,jFx=val?.jointFx;
        const fxDrift=Number.isFinite(pFx)&&pFx>0&&Number.isFinite(jFx)?Math.abs(pFx-jFx)/pFx:1;
        if(!Number.isFinite(jFx)||fxDrift>0.08||(val?.positionP95Mm!=null&&val.positionP95Mm>=6.5)){
          cached=null;upgradeReason='sanity-check';
        }
      }
    }
    if(cached&&cached.imageSize.some((v,i)=>v!==imageSize[i])){
      const [w,h]=imageSize,[sw,sh]=cached.imageSize;
      if(Math.abs(w/h-sw/sh)>.001)cached=null;
      else{
        cached=clone(cached);
        const k1=Number.isFinite(cached.distCoeffs?.[0])?cached.distCoeffs[0]:(Number.isFinite(cached.K?.k1)?cached.K.k1:0);
        cached.K={width:w,height:h,fx:cached.K.fx*w/sw,fy:cached.K.fy*h/sh,cx:cached.K.cx*w/sw,cy:cached.K.cy*h/sh,k1};
        cached.imageSize=clone(imageSize);
      }
    }
    resetRows();scope=null;candidate=cached;profile=null;verification=null;checkFailure=null;persisted=Boolean(cached);lastFitCapture=null;lastVerificationCapture=null;onProfile(null);
    phase=cached?'checking':'scanning';
    message=cached?'Verifying saved profile: look at the L-angle and shift your head slightly.'
      :(upgradeReason==='solver-upgraded'?'Solver upgraded (v2 radial+optical BA): sweep around L-angle & image edges to recalibrate.'
        :(upgradeReason==='sanity-check'?'Cached profile flagged by sanity check: recalibrate around L-angle & image edges.'
          :(upgradeReason==='model-changed'?'Camera model updated: keep L-angle still, sweep around halo and toward image edges.'
            :'Keep the L-angle still. Move head around multiple axes and vary distance.')));
    notify();
  }
  function failed(why){cancel();phase='failed';message=why;candidate=null;profile=null;onProfile(null);notify();}
  function fit(history){
    // Fast detectors can return before the +250 ms search horizon exists.
    // Keep those observations, but fit only rows already bracketed by XR history.
    const captured=clone(rows.filter(r=>r.captureTime-251>=history[0].xrRafTimeMs&&r.captureTime+251<=history.at(-1).xrRafTimeMs));
    if(captured.length<30)return;
    lastFitCapture={model,imageSize:clone(imageSize),rigId:rig?.id??null,rows:clone(captured),history:sliceHistoryForRows(captured,history)};
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
        const v=result?.validation||{};
        const why=v.translationSafe===false?'camera offset out of range'
          :v.offsetSafe===false?'clock offset out of range'
          :v.delayIdentified===false?'sync delay unstable'
          :v.uncertainty===false?'focal uncertainty too high'
          :Number.isFinite(v.positionP95Mm)&&v.positionP95Mm>=8.5?`pos ${v.positionP95Mm.toFixed(1)} mm`
          :Number.isFinite(v.angleP95Deg)&&v.angleP95Deg>=1.5?`ang ${v.angleP95Deg.toFixed(2)} deg`
          :Number.isFinite(v.heldOutRmsPx)&&v.heldOutRmsPx>=1.2?`rms ${v.heldOutRmsPx.toFixed(2)} px`
          :Number.isFinite(v.jointHeldOutRmsPx)&&v.jointHeldOutRmsPx>=10.0?`reproj ${v.jointHeldOutRmsPx.toFixed(1)} px`
          :'more diverse viewpoints needed';
        if(rows.length<180){nextFitAt=rows.length+15;phase='scanning';message=`Retry calibration (${why}). Keep L-angle still and sweep around halo.`;notify();}
        else failed(`Calibration did not meet quality gates (${why}). Restart with wider viewpoints.`);return;
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
      const diverse=diversity(rows,valid,imageSize);
      const isRadial=model==='centered-radial'||model==='pinhole-radial';
      const needEdge=isRadial&&rows.length<Math.max(nextFitAt+12,48)&&(diverse.maxCornerRadiusFrac??1)<0.55;
      if(rows.length>=nextFitAt&&diverse.ok&&!needEdge){fit(valid);return state();}
      if(rows.length>180){failed('More distinct perspectives needed instead of repeated views. Rotate head around multiple axes.');return state();}
      message=!diverse.ok?diverse.reason:(needEdge?'Sweep head so the L-angle also visits image edges/corners.':'Keep varying viewpoints while the L-angle stays stationary.');
      notify();return state();
    }
    const paired=[];
    for(const r of rows){const h=poseAt(valid,r.captureTime+candidate.timeOffsetMs);if(h)paired.push({row:r,h});}
    if(paired.length>=verificationViews){
      const verRows=paired.slice(0,30).map(p=>p.row);
      lastVerificationCapture={kind:phase,profileId:candidate?.id??null,imageSize:clone(imageSize),
        rows:clone(verRows),history:sliceHistoryForRows(verRows,valid)};
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
      const isChecking=phase==='checking';
      const posMax=isChecking?10.0:8.5;
      const angMax=isChecking?1.5:1.4;
      const rmsMax=isChecking?1.2:1.2;
      const minMotion=isChecking?0.04:0.04;
      if(report.ok&&report.positionP95Mm<posMax&&report.angleP95Deg<angMax&&report.reprojectionRmsPx<rmsMax&&motion>minMotion){
        verification=report;accept(report);return state();
      }
      if(rows.length>=30){
        if(isChecking){
          checkFailure={positionP95Mm:report.positionP95Mm,angleP95Deg:report.angleP95Deg,reprojectionRmsPx:report.reprojectionRmsPx,
            motionM:motion,limits:{positionP95Mm:posMax,angleP95Deg:angMax,reprojectionRmsPx:rmsMax,minMotionM:minMotion}};
          const why=report.angleP95Deg>=angMax?`orientation error ${report.angleP95Deg.toFixed(1)}° exceeds ${angMax}°`:
            report.positionP95Mm>=posMax?`position error ${report.positionP95Mm.toFixed(1)} mm exceeds ${posMax} mm`:
            report.reprojectionRmsPx>=rmsMax?'marker image not sharp enough':motion<=minMotion?'head motion too small for verification':'spatial association incomplete';
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
      message=motion<=minMotion?'Shift your head side-to-side and up/down for verification.':'Collecting verification views; keep L-angle stationary.';
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
    diagnosticsData:()=>(lastFitCapture||lastVerificationCapture)?clone({fit:lastFitCapture,verification:lastVerificationCapture}):null,
    retryCheck,retry:()=>beginScan(),profile:()=>profile&&clone(profile),
    reset(){cancel();scope=null;resetRows();phase='idle';candidate=null;profile=null;upgradeReason=null;lastFitCapture=null;lastVerificationCapture=null;onProfile(null);notify();},
    dispose(){disposed=true;cancel();profile=null;candidate=null;upgradeReason=null;lastFitCapture=null;lastVerificationCapture=null;phase='disposed';}};
}
