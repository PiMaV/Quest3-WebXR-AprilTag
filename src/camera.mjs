import {intrinsics} from './runtime.mjs';
import {observeReference} from './rig.mjs';
import {calibratedCameraModel,profileCameraConstraints} from './profile.mjs';



// Reference camera capture adapter using atomic requestVideoFrameCallback frames and off-thread AprilTag detection.
export function createReferenceCamera({video,geometry,getSettings,getContext,onObservation,onState,onError}) {
  const canvas=document.createElement('canvas'),ctx=canvas.getContext('2d',{willReadFrequently:true});
  let worker=null,ready=null,readyResolve,readyReject,workerTimer;
  let stream=null,running=false,starting=false,generation=0,serial=0,busy=null;
  let videoCallback=null,metadata=null,lastMediaTime=-1,lastPresentedFrames=-1,lastSubmitted=-Infinity,presentedCallbacks=0;
  let previousEvidence=null,lastEvidenceAt=null,lastRvfcAt=null,startedAt=null,sourceFrames=0,stalled=false,rvfcStalled=false;
  let requestedConstraints=null,resizeModeFallback=false,lastError=null,activeDecimate=1,activeDetectMode=undefined,activeTrackRefKey=undefined;
  const state=()=>({running,starting,generation,presentedCallbacks,
    sourceFrames,requestedConstraints,resizeModeFallback,lastError,rvfcStalled,decimate:activeDecimate,detectMode:activeDetectMode??'full',
    frameStalled:running&&lastEvidenceAt!=null&&performance.now()-lastEvidenceAt>1500,
    evidenceAgeMs:lastEvidenceAt==null?null:performance.now()-lastEvidenceAt,
    trackState:stream?.getVideoTracks()[0]?.readyState??'none',
    trackMuted:stream?.getVideoTracks()[0]?.muted??null,
    trackLabel:stream?.getVideoTracks()[0]?.label??null,
    trackSettings:stream?.getVideoTracks()[0]?.getSettings()??null,
    trackCapabilities:stream?.getVideoTracks()[0]?.getCapabilities?.()??null,
    sourceSize:[video.videoWidth,video.videoHeight]});
  const notify=()=>onState?.(state());
  function fail(error) {
    clearTimeout(workerTimer);readyReject?.(error);readyResolve=null;readyReject=null;
    stop();worker?.terminate();worker=null;ready=null;activeDecimate=1;activeDetectMode=undefined;activeTrackRefKey=undefined;onError?.(error);
  }
  function prepare() {
    if(ready) return ready;
    ready=new Promise((resolve,reject)=>{readyResolve=resolve;readyReject=reject;});
    worker=new Worker(new URL('./detector-worker.js',import.meta.url));activeDecimate=1;activeDetectMode=undefined;activeTrackRefKey=undefined;
    workerTimer=setTimeout(()=>fail(new Error('Detector failed to load (15 s).')),15000);
    worker.onerror=event=>fail(new Error(event.message||'Detector error'));
    worker.onmessage=({data})=>{
      if(data.type==='ready') {clearTimeout(workerTimer);readyResolve?.();readyResolve=null;readyReject=null;return;}
      if(data.type==='error') {fail(new Error(data.message));return;}
      if(!['detections','frame-skipped'].includes(data.type)||data.serial!==busy?.serial) return;
      const sample=busy;clearTimeout(sample.timeout);busy=null;
      if(sample.generation!==generation||!running) return;
      if(data.type==='frame-skipped') {
        if(!stalled&&performance.now()-lastEvidenceAt>1500) {stalled=true;notify();}
        return;
      }
      previousEvidence=data.frameEvidence;sourceFrames++;lastEvidenceAt=sample.submittedAtPerformanceMs;
      if(stalled) {stalled=false;notify();}
      sample.frameEvidence=data.frameEvidence;sample.workerPreprocessMs=data.preprocessMs;
      if(data.readbackMode) sample.readbackMode=data.readbackMode;
      if(data.detectMode) sample.detectMode=data.detectMode;
      if(Number.isFinite(data.coarseMs)) sample.coarseMs=data.coarseMs;
      if(Number.isFinite(data.roiMs)) sample.roiMs=data.roiMs;
      if(Number.isFinite(data.roiCount)) sample.roiCount=data.roiCount;
      if('focusHit' in data) sample.focusHit=Boolean(data.focusHit);
      sample.receivedAtPerformanceMs=performance.now();sample.detectMs=data.detectMs;delete sample.timeout;
      try {
        let observation;
        if(data.observation) {
          observation=data.observation;
          sample.poseFitMs=data.poseFitMs??0;
        } else {
          const track=stream?.getVideoTracks()[0];
          const K=sample.cameraModel??(sample.settings.experimentalCalibration
            ?calibratedCameraModel(sample.settings.experimentalCalibration,sample.imageSize,sample.sourceSize,track?.label??null,navigator.userAgent,track?.getSettings?.()??null)
            :intrinsics(...sample.imageSize,sample.settings.horizontalFovDeg));
          const g=sample.geometry??geometry;
          const poseStarted=performance.now();
          observation=observeReference(data.markers,g,K,{printScale:sample.settings.printScale});
          // Preserve measured pixels for future independent-tool registration.
          // These detections have no known metric placement and never seed the L anchor.
          const unassigned=new Set(observation.rejected.filter(t=>t.reason==='unassigned_id').map(t=>t.id));
          observation.unassignedDetections=data.markers.filter(t=>unassigned.has(t.id)).map(t=>({
            id:t.id,family:g.family,corners:structuredClone(t.corners),registered:false,pose:null}));
          sample.poseFitMs=performance.now()-poseStarted;
        }
        onObservation?.(observation,sample,canvas);
      } catch(error) {if(sample.settings.experimentalCalibration) stop();onError?.(error);}
    };
    return ready;
  }
  function submitFrame(now,fromRvfc) {
    if(!running||busy||video.readyState<2||!video.videoWidth) return;
    const settings=getSettings();
    const maxHz=Number.isFinite(settings.maxHz)?Math.max(1,Math.min(30,settings.maxHz)):3;
    if(now-lastSubmitted<1000/maxHz) return;
    if(fromRvfc) {
      if(metadata&&metadata.mediaTime===lastMediaTime&&metadata.presentedFrames===lastPresentedFrames) return;
    } else if(video.currentTime===lastMediaTime) return;
    const desiredDecimate=settings.decimate===2?2:(settings.decimate===1.5?1.5:1);
    const desiredDetectMode=(settings.detectMode==='full'||settings.detectMode==='two-stage'||settings.detectMode==='fast')?settings.detectMode:undefined;
    const desiredTrackRefIds=Array.isArray(settings.trackRefIds)?settings.trackRefIds:(settings.trackRefIds===null?null:undefined);
    const desiredTrackRefKey=desiredTrackRefIds===undefined?undefined:(desiredTrackRefIds?desiredTrackRefIds.join(','):'null');
    const cfg={};let cfgChanged=false;
    if(desiredDecimate!==activeDecimate) {
      activeDecimate=desiredDecimate;
      cfg.decimate=activeDecimate;
      cfgChanged=true;
    }
    if(desiredDetectMode!==undefined&&desiredDetectMode!==activeDetectMode) {
      activeDetectMode=desiredDetectMode;
      cfg.detectMode=activeDetectMode;
      cfgChanged=true;
    }
    if(desiredTrackRefKey!==undefined&&desiredTrackRefKey!==activeTrackRefKey) {
      activeTrackRefKey=desiredTrackRefKey;
      cfg.trackRefIds=desiredTrackRefIds?desiredTrackRefIds.slice():null;
      cfgChanged=true;
    }
    if(cfgChanged&&worker) {
      if(!('decimate' in cfg)) cfg.decimate=activeDecimate;
      worker.postMessage({type:'config',...cfg});
    }
    lastSubmitted=now;
    lastMediaTime=fromRvfc&&metadata?metadata.mediaTime:video.currentTime;
    if(metadata&&Number.isFinite(metadata.presentedFrames)) lastPresentedFrames=metadata.presentedFrames;
    const [w,h]=[video.videoWidth,video.videoHeight],scale=Math.min(1,settings.maxLongEdge/Math.max(w,h));
    const submittedAtPerformanceMs=performance.now();
    const width=Math.round(w*scale),height=Math.round(h*scale);
    let K;
    try {
      const track=stream?.getVideoTracks()[0];
      K=settings.experimentalCalibration
        ?calibratedCameraModel(settings.experimentalCalibration,[width,height],[w,h],track?.label??null,navigator.userAgent,track?.getSettings?.()??null)
        :intrinsics(width,height,settings.horizontalFovDeg);
    } catch(error) {
      if(settings.experimentalCalibration) stop();
      onError?.(error);
      return;
    }
    const canVideoFrame=typeof globalThis.VideoFrame==='function';
    const canOffload=typeof globalThis.createImageBitmap==='function'&&typeof globalThis.OffscreenCanvas==='function';
    const sample={serial:++serial,generation,context:structuredClone(getContext()),
      mainThreadReadbackMs:0,decimate:activeDecimate,cameraModel:K,geometry,
      submittedAtPerformanceMs,videoTimeAtDraw:video.currentTime,
      videoFrameMetadata:metadata?{...metadata}:null,presentedCallbacks,
      metadataMediaTimeDeltaS:metadata?video.currentTime-metadata.mediaTime:null,
      capturedInRvfc:fromRvfc,readbackMode:canVideoFrame?'worker-videoframe':(canOffload?'worker-bitmap':'main-canvas'),
      sourceSize:[w,h],imageSize:[width,height],settings,
      screenOrientation:{angle:globalThis.screen?.orientation?.angle??null,type:globalThis.screen?.orientation?.type??null},
      captureTimeKnown:Number.isFinite(metadata?.captureTime),frameSynchronizationValidated:false};
    sample.timeout=setTimeout(()=>fail(new Error('Detector did not respond. Please restart camera.')),10000);
    busy=sample;
    const printScale=settings.printScale??1;
    const solveUnassigned=Boolean(settings.solveUnassigned);
    const priorRois=Array.isArray(settings.priorRois)&&settings.priorRois.length?settings.priorRois:undefined;
    const focusRois=Array.isArray(settings.focusRois)&&settings.focusRois.length?settings.focusRois:undefined;
    sample.priorRoiCount=priorRois?priorRois.length:0;
    sample.focusRoiCount=focusRois?focusRois.length:0;
    const extraMsg=(priorRois||focusRois)?{...(priorRois?{priorRois}:null),...(focusRois?{focusRois}:null)}:null;
    const sendMainCanvas=(mode='main-canvas')=>{
      if(canvas.width!==width||canvas.height!==height) {canvas.width=width;canvas.height=height;}
      ctx.drawImage(video,0,0,width,height);
      const rgba=ctx.getImageData(0,0,width,height).data;
      sample.readbackMode=mode;
      sample.mainThreadReadbackMs=performance.now()-submittedAtPerformanceMs;
      worker.postMessage({serial:sample.serial,rgba:rgba.buffer,width,height,previousEvidence,callbackCount:presentedCallbacks,K,geometry,printScale,solveUnassigned,...extraMsg},[rgba.buffer]);
    };
    const sendBitmap=()=>{
      sample.readbackMode='worker-bitmap';
      globalThis.createImageBitmap(video).then(bitmap=>{
        if(busy!==sample||sample.generation!==generation||!running||!worker) {bitmap?.close?.();return;}
        sample.mainThreadReadbackMs=performance.now()-submittedAtPerformanceMs;
        worker.postMessage({serial:sample.serial,bitmap,width,height,previousEvidence,callbackCount:presentedCallbacks,K,geometry,printScale,solveUnassigned,...extraMsg},[bitmap]);
      }).catch(()=>{
        if(busy!==sample||sample.generation!==generation||!running||!worker) return;
        sendMainCanvas('main-canvas-fallback');
      });
    };
    if(canVideoFrame) {
      let vf=null;
      try {
        vf=new globalThis.VideoFrame(video);
        sample.mainThreadReadbackMs=performance.now()-submittedAtPerformanceMs;
        worker.postMessage({serial:sample.serial,videoFrame:vf,width,height,previousEvidence,callbackCount:presentedCallbacks,K,geometry,printScale,solveUnassigned,...extraMsg},[vf]);
        return;
      } catch {
        try {vf?.close?.();} catch { /* ignore */ }
      }
    }
    if(canOffload) sendBitmap();
    else sendMainCanvas('main-canvas');
  }
  function watch(token) {
    if(!video.requestVideoFrameCallback) return;
    videoCallback=video.requestVideoFrameCallback((now,m)=>{
      if(token!==generation||!running) return;
      presentedCallbacks++;lastRvfcAt=performance.now();
      if(rvfcStalled) {rvfcStalled=false;notify();}
      metadata={callbackAtPerformanceMs:now,mediaTime:m.mediaTime,presentedFrames:m.presentedFrames,
        captureTime:m.captureTime??null,presentationTime:m.presentationTime??null,
        expectedDisplayTime:m.expectedDisplayTime??null,processingDuration:m.processingDuration??null};
      submitFrame(now,true);
      watch(token);
    });
  }
  async function start() {
    if(running) return true;
    if(starting) return false;
    const token=++generation;starting=true;lastError=null;resizeModeFallback=false;rvfcStalled=false;notify();
    let timer;
    try {
      if(!isSecureContext||!navigator.mediaDevices?.getUserMedia) throw new Error('Camera requires HTTPS and a browser with camera access.');
      await prepare();if(token!==generation) return false;
      timer=setTimeout(()=>{
        if(token!==generation||running) return;
        stop();onError?.(new Error('Camera permission timed out. Please allow camera access in the browser.'));
      },15000);
      const profile=getSettings().experimentalCalibration;
      const devices=profile?await navigator.mediaDevices.enumerateDevices?.().catch(()=>[])??[]:[];
      if(token!==generation) return false;
      const supported=navigator.mediaDevices.getSupportedConstraints?.()??{};
      requestedConstraints=profile?profileCameraConstraints(profile,devices,supported):
        {audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:1080},...(supported.resizeMode?{resizeMode:{exact:'none'}}:{})}};
      let acquired;
      try {
        acquired=await navigator.mediaDevices.getUserMedia(requestedConstraints);
      } catch(error) {
        if(!profile&&error?.name==='OverconstrainedError'&&requestedConstraints.video?.resizeMode) {
          const relaxed={audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:1280},height:{ideal:1080}}};
          requestedConstraints=relaxed;resizeModeFallback=true;
          acquired=await navigator.mediaDevices.getUserMedia(relaxed);
        } else throw error;
      }
      if(token!==generation) {acquired.getTracks().forEach(t=>t.stop());return false;}
      stream=acquired;video.srcObject=stream;await video.play();
      if(token!==generation) return false;
      if(getSettings().experimentalCalibration) calibratedCameraModel(getSettings().experimentalCalibration,
        [video.videoWidth,video.videoHeight],[video.videoWidth,video.videoHeight],acquired.getVideoTracks()[0]?.label,navigator.userAgent,acquired.getVideoTracks()[0]?.getSettings()??{});
      acquired.getVideoTracks()[0]?.addEventListener?.('ended',()=>{
        if(token!==generation) return;
        stop();onError?.(new Error('Camera stream ended. Please restart the camera.'));
      },{once:true});
      starting=false;running=true;lastMediaTime=-1;lastPresentedFrames=-1;lastSubmitted=-Infinity;metadata=null;presentedCallbacks=0;
      previousEvidence=null;startedAt=performance.now();lastEvidenceAt=startedAt;lastRvfcAt=null;sourceFrames=0;stalled=false;rvfcStalled=false;
      watch(token);notify();return true;
    } catch(error) {
      if(token===generation) {
        const profile=getSettings().experimentalCalibration,original=error;
        if(profile&&error.name==='OverconstrainedError') error=new Error(`Camera cannot provide ${profile.imageSize.join(' × ')} without cropping (${error.constraint??'camera mode'}). Calibration profile remains locked; standard capture is separate.`);
        lastError={name:original.name,message:error.message,constraint:original.constraint??null};stop();onError?.(error);
      }return false;
    } finally {clearTimeout(timer);}
  }
  function stop() {
    generation++;running=false;starting=false;
    if(busy) clearTimeout(busy.timeout);busy=null;
    if(videoCallback!=null) video.cancelVideoFrameCallback?.(videoCallback);
    videoCallback=null;metadata=null;rvfcStalled=false;stream?.getTracks().forEach(t=>t.stop());stream=null;video.srcObject=null;notify();
  }
  function tick(now) {
    if(!running) return;
    if(video.requestVideoFrameCallback) {
      const ref=lastRvfcAt??startedAt;
      const isStalled=ref!=null&&performance.now()-ref>1500&&video.readyState>=2;
      if(isStalled!==rvfcStalled) {rvfcStalled=isStalled;notify();}
      return;
    }
    submitFrame(now,false);
  }
  return {prepare,start,stop,tick,state,isRunning:()=>running,presentedCallbacks:()=>presentedCallbacks,canvas,setGeometry(next){geometry=next;},dispose(){stop();clearTimeout(workerTimer);worker?.terminate();worker=null;ready=null;}};
}
