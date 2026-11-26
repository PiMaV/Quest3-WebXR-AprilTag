// Classic worker: the existing public Emscripten build uses importScripts.
// Supports VideoFrame (WebCodecs Y-plane), ImageBitmap, RGBA, and raw grayscale inputs.
const base=new URL('../vendor/apriltag/',self.location.href).href;
const YUV_FORMATS=new Set(['I420','I420A','I422','I444','NV12']);
let detector,prepareRgbaFrame,prepareGrayFrame,observeReference,solveSingleTagPose,offscreen=null,offCtx=null,copyBuf=null,grayBuf=null;
let detectMode='full',activeDecimate=1,trackRefIds=null,lastRefHintBox=null;
(async()=>{
  importScripts(base+'apriltag_wasm.js');
  const wasm=await AprilTagWasm({locateFile:path=>base+path});
  const [{createAprilTagDetector},pixels,rig]=await Promise.all([import('./detector.mjs'),import('./frame-evidence.mjs'),import('./rig.mjs')]);
  prepareRgbaFrame=pixels.prepareRgbaFrame;prepareGrayFrame=pixels.prepareGrayFrame;
  observeReference=rig.observeReference;solveSingleTagPose=rig.solveSingleTagPose;
  detector=createAprilTagDetector(wasm,1);postMessage({type:'ready'});
})().catch(error=>postMessage({type:'error',message:String(error)}));

function ensureGrayBuf(size){
  if(!grayBuf||grayBuf.length!==size) grayBuf=new Uint8Array(size);
  return grayBuf;
}

function drawToGray(source,width,height,previousEvidence,callbackCount){
  if(!offscreen||offscreen.width!==width||offscreen.height!==height) {
    offscreen=new OffscreenCanvas(width,height);
    offCtx=offscreen.getContext('2d',{willReadFrequently:true});
  }
  offCtx.drawImage(source,0,0,width,height);
  const rgba=offCtx.getImageData(0,0,width,height).data;
  return prepareRgbaFrame(rgba,width,height,previousEvidence,callbackCount,ensureGrayBuf(width*height));
}

async function extractVideoFrameGray(frame,width,height,previousEvidence,callbackCount){
  const vr=frame.visibleRect||{x:0,y:0,width:frame.codedWidth,height:frame.codedHeight};
  if(YUV_FORMATS.has(frame.format)&&vr.width===width&&vr.height===height&&typeof frame.copyTo==='function'&&typeof frame.allocationSize==='function') {
    try {
      const opts={rect:{x:vr.x,y:vr.y,width:vr.width,height:vr.height}};
      const needed=frame.allocationSize(opts);
      if(!copyBuf||copyBuf.byteLength<needed) copyBuf=new Uint8Array(needed);
      const target=copyBuf.subarray(0,needed);
      const layouts=await frame.copyTo(target,opts);
      const yPlane=layouts?.[0];
      if(yPlane&&Number.isFinite(yPlane.stride)&&yPlane.stride>=width) {
        let gray;
        if(yPlane.offset===0&&yPlane.stride===width) {
          gray=target.subarray(0,width*height);
        } else {
          gray=ensureGrayBuf(width*height);
          for(let row=0;row<height;row++) {
            const srcOff=yPlane.offset+row*yPlane.stride;
            gray.set(target.subarray(srcOff,srcOff+width),row*width);
          }
        }
        return {...prepareGrayFrame(gray,width,height,previousEvidence,callbackCount),readbackMode:'worker-videoframe-y'};
      }
    } catch { /* Fall back to OffscreenCanvas drawImage below. */ }
  }
  return {...drawToGray(frame,width,height,previousEvidence,callbackCount),readbackMode:'worker-videoframe-canvas'};
}

self.onmessage=async({data})=>{
  if(!detector) {data.videoFrame?.close?.();data.bitmap?.close?.();return;}
  try {
    if(data.type==='config') {
      if(data.decimate===1||data.decimate===1.5||data.decimate===2) {activeDecimate=data.decimate;detector.setDecimate(data.decimate);}
      if(data.detectMode==='full'||data.detectMode==='two-stage'||data.detectMode==='fast') detectMode=data.detectMode;
      if('trackRefIds' in data) {
        const nextIds=Array.isArray(data.trackRefIds)&&data.trackRefIds.length?data.trackRefIds.slice():null;
        const prevKey=trackRefIds?trackRefIds.join(','):'';
        const nextKey=nextIds?nextIds.join(','):'';
        if(prevKey!==nextKey) lastRefHintBox=null;
        trackRefIds=nextIds;
      }
      return;
    }
    const started=performance.now();let gray,evidence,readbackMode;
    if(data.videoFrame) {
      try {
        const prepared=await extractVideoFrameGray(data.videoFrame,data.width,data.height,data.previousEvidence,data.callbackCount);
        gray=prepared.gray;evidence=prepared.evidence;readbackMode=prepared.readbackMode;
      } finally {data.videoFrame.close?.();}
      if(!evidence.fresh) {postMessage({type:'frame-skipped',serial:data.serial,frameEvidence:evidence,preprocessMs:performance.now()-started,readbackMode});return;}
    } else if(data.bitmap) {
      let prepared;
      try {
        prepared=drawToGray(data.bitmap,data.width,data.height,data.previousEvidence,data.callbackCount);
      } finally {data.bitmap.close?.();}
      gray=prepared.gray;evidence=prepared.evidence;readbackMode='worker-bitmap';
      if(!evidence.fresh) {postMessage({type:'frame-skipped',serial:data.serial,frameEvidence:evidence,preprocessMs:performance.now()-started,readbackMode});return;}
    } else if(data.rgba) {
      const prepared=prepareRgbaFrame(new Uint8ClampedArray(data.rgba),data.width,data.height,data.previousEvidence,data.callbackCount,ensureGrayBuf(data.width*data.height));
      gray=prepared.gray;evidence=prepared.evidence;
      if(!evidence.fresh) {postMessage({type:'frame-skipped',serial:data.serial,frameEvidence:evidence,preprocessMs:performance.now()-started});return;}
    } else gray=new Uint8Array(data.gray);
    const preprocessMs=performance.now()-started,start=performance.now();
    let markers,coarseMs=0,roiMs=0,roiCount=0,focusHit=false,focusMissedBoxes=0;
    if(detectMode==='two-stage') {
      const coarseDecimate=activeDecimate===1?1:(activeDecimate===1.5?1.5:2);
      const priorRois=Array.isArray(data.priorRois)&&data.priorRois.length?data.priorRois:undefined;
      const focusRois=Array.isArray(data.focusRois)&&data.focusRois.length?data.focusRois:null;
      const res=(trackRefIds?.length&&lastRefHintBox)
        ?detector.detectTrackedRoi(gray,data.width,data.height,lastRefHintBox,{requiredIds:trackRefIds,fallbackOpts:{coarseDecimate,priorRois}})
        :(focusRois
          ?detector.detectFocusRois(gray,data.width,data.height,focusRois,{fallbackOpts:{coarseDecimate,priorRois}})
          :detector.detectTwoStage(gray,data.width,data.height,{coarseDecimate,priorRois}));
      markers=res.markers;coarseMs=res.coarseMs;roiMs=res.roiMs;roiCount=res.roiCount;focusHit=Boolean(res.focusHit);
      focusMissedBoxes=Number.isFinite(res.focusMissedBoxes)?res.focusMissedBoxes:0;
      if(trackRefIds?.length) {
        const idSet=new Set(trackRefIds);
        const found=new Set();
        let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
        for(const m of markers) {
          if(!idSet.has(m.id)||!Array.isArray(m.corners)) continue;
          found.add(m.id);
          for(const p of m.corners) {
            if(p.x<minX) minX=p.x;
            if(p.x>maxX) maxX=p.x;
            if(p.y<minY) minY=p.y;
            if(p.y>maxY) maxY=p.y;
          }
        }
        if(found.size===trackRefIds.length&&Number.isFinite(minX)) {
          lastRefHintBox=[
            Math.max(0,Math.floor(minX)-96),
            Math.max(0,Math.floor(minY)-96),
            Math.min(data.width,Math.ceil(maxX)+96),
            Math.min(data.height,Math.ceil(maxY)+96),
          ];
        } else {
          lastRefHintBox=null;
        }
      }
    } else if(detectMode==='fast') {
      const res=detector.detectTwoStage(gray,data.width,data.height,{coarseDecimate:2,refine:false});
      markers=res.markers;coarseMs=res.coarseMs;roiMs=0;roiCount=0;
    } else {
      markers=detector.detect(gray,data.width,data.height);
    }
    const detectMs=performance.now()-start;
    let observation=null,poseFitMs=0;
    if(data.K&&data.geometry) {
      const poseStarted=performance.now();
      observation=observeReference(markers,data.geometry,data.K,{printScale:data.printScale??1});
      const unassigned=new Set(observation.rejected.filter(t=>t.reason==='unassigned_id').map(t=>t.id));
      const counts=new Map();
      for(const m of markers) if(unassigned.has(m.id)) counts.set(m.id,(counts.get(m.id)||0)+1);
      observation.unassignedDetections=[];
      for(const m of markers) {
        if(!unassigned.has(m.id)) continue;
        const entry={id:m.id,family:data.geometry.family,corners:m.corners,registered:false,pose:null,softFallback:Boolean(m.softFallback)};
        if(data.solveUnassigned&&counts.get(m.id)===1) {
          entry.workerSolved=true;
          const solved=solveSingleTagPose(m,data.K,{minEdgePx:8,minAreaPx:64,maxRmsPx:2.5});
          if(solved) {
            entry.pose=solved.pose;
            entry.edgeMm=solved.edgeMm;
            entry.minEdgePx=solved.minEdgePx;
            entry.areaPx=solved.areaPx;
            entry.rmsPx=solved.rmsPx;
            entry.cosView=solved.cosView;
            entry.depthM=solved.depthM;
          }
        }
        observation.unassignedDetections.push(entry);
      }
      poseFitMs=performance.now()-poseStarted;
    }
    postMessage({type:'detections',serial:data.serial,markers,observation,poseFitMs,frameEvidence:evidence,preprocessMs,detectMs,readbackMode,detectMode,coarseMs,roiMs,roiCount,focusHit,focusMissedBoxes});
  } catch(error) {postMessage({type:'error',message:String(error)});}
};
