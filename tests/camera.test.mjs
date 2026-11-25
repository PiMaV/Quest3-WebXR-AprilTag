import test from 'node:test';
import assert from 'node:assert/strict';
import {createReferenceCamera} from '../src/camera.mjs';
import {rig} from './synthetic.mjs';

function withBrowserMocks(fn){
  const saved={
    document:globalThis.document,
    Worker:globalThis.Worker,
    navigator:globalThis.navigator,
    isSecureContext:globalThis.isSecureContext,
  };
  let workerInstance=null;
  class MockWorker{
    constructor(){workerInstance=this;this.messages=[];this.terminated=false;queueMicrotask(()=>this.onmessage?.({data:{type:'ready'}}));}
    postMessage(data){this.messages.push(data);}
    terminate(){this.terminated=true;}
  }
  const fakeCanvas={width:0,height:0,getContext:()=>({drawImage(){},getImageData:(_,__,w,h)=>({data:new Uint8ClampedArray(w*h*4).fill(128)})})};
  Object.defineProperty(globalThis,'document',{value:{createElement:()=>fakeCanvas},configurable:true,writable:true});
  Object.defineProperty(globalThis,'Worker',{value:MockWorker,configurable:true,writable:true});
  Object.defineProperty(globalThis,'isSecureContext',{value:true,configurable:true,writable:true});
  return Promise.resolve().then(()=>fn({getWorker:()=>workerInstance})).finally(()=>{
    for(const [k,v] of Object.entries(saved)){
      if(v===undefined)delete globalThis[k];
      else Object.defineProperty(globalThis,k,{value:v,configurable:true,writable:true});
    }
  });
}

function makeMockVideo({withRvfc=true}={}){
  let nextCbId=1,cb=null;
  const video={
    videoWidth:1280,videoHeight:1080,readyState:4,currentTime:0,srcObject:null,
    async play(){},
    ...(withRvfc?{
      requestVideoFrameCallback(fn){cb=fn;return nextCbId++;},
      cancelVideoFrameCallback(){cb=null;},
    }:{}),
    fireRvfc(now,meta){const fn=cb;cb=null;fn?.(now,meta);},
  };
  return video;
}

test('createReferenceCamera captures atomically inside rVFC and falls back on cold-start resizeMode OverconstrainedError',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const calls=[];
    const track={readyState:'live',muted:false,label:'Quest RGB',stop(){this.readyState='ended';},getSettings:()=>({deviceId:'cam1',resizeMode:'crop-and-scale'}),addEventListener(){}};
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        async getUserMedia(constraints){
          calls.push(constraints);
          if(constraints.video?.resizeMode){
            const err=new Error('resizeMode unsupported');err.name='OverconstrainedError';err.constraint='resizeMode';throw err;
          }
          return {getVideoTracks:()=>[track],getTracks:()=>[track]};
        },
      },
    },configurable:true,writable:true});

    const video=makeMockVideo({withRvfc:true});
    const observations=[],errors=[];
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
      onObservation:(obs,sample)=>observations.push({obs,sample}),
      onError:e=>errors.push(e),
    });

    const ok=await cam.start();
    assert.equal(ok,true);
    assert.equal(errors.length,0);
    assert.equal(calls.length,2,'must retry without resizeMode on cold-start OverconstrainedError');
    assert.equal(cam.state().resizeModeFallback,true);

    // Trigger an rVFC frame -> must capture synchronously inside rVFC, not wait for tick().
    video.currentTime=1.5;
    video.fireRvfc(1000,{mediaTime:1.5,presentedFrames:1,captureTime:985});
    const worker=getWorker();
    assert.equal(worker.messages.length,1);
    worker.onmessage({data:{type:'detections',serial:worker.messages[0].serial,markers:[],frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:2,detectMs:15}});
    assert.equal(observations.length,1);
    assert.equal(observations[0].sample.capturedInRvfc,true);
    assert.equal(observations[0].sample.metadataMediaTimeDeltaS,0);
    assert.equal(observations[0].sample.videoFrameMetadata.captureTime,985);

    // Throttled: second rVFC within < 333 ms must not enqueue another worker frame.
    video.currentTime=1.53;
    video.fireRvfc(1033,{mediaTime:1.53,presentedFrames:2,captureTime:1018});
    assert.equal(worker.messages.length,1);
    cam.dispose();
  });
});

test('createReferenceCamera never falls back on resizeMode when a calibrated profile is active',async()=>{
  await withBrowserMocks(async()=>{
    const calls=[];
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        async getUserMedia(constraints){
          calls.push(constraints);
          const err=new Error('cannot match exact');err.name='OverconstrainedError';err.constraint='resizeMode';throw err;
        },
      },
    },configurable:true,writable:true});

    const video=makeMockVideo({withRvfc:true});
    const errors=[];
    const fakeProfile={
      schema:'webxr-apriltag-profile-v1',id:'p1',imageSize:[1280,1080],
      device:{mode:{sourceSize:[1280,1080],resizeMode:'none'},trackLabel:'Quest RGB',userAgent:'QuestMock/1.0'},
    };
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:fakeProfile}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
      onError:e=>errors.push(e),
    });

    const ok=await cam.start();
    assert.equal(ok,false);
    assert.equal(calls.length,1,'must NOT relax constraints when a calibrated profile is active');
    assert.equal(errors.length,1);
    assert.match(errors[0].message,/locked/i);
    cam.dispose();
  });
});

test('createReferenceCamera offloads readback via ImageBitmap when OffscreenCanvas is available',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const savedBmp=globalThis.createImageBitmap,savedOsc=globalThis.OffscreenCanvas;
    const fakeBitmap={closed:false,close(){this.closed=true;}};
    Object.defineProperty(globalThis,'createImageBitmap',{value:async()=>fakeBitmap,configurable:true,writable:true});
    Object.defineProperty(globalThis,'OffscreenCanvas',{value:class MockOffscreen{},configurable:true,writable:true});
    try{
      const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
      Object.defineProperty(globalThis,'navigator',{value:{
        userAgent:'QuestMock/1.0',
        mediaDevices:{
          getSupportedConstraints:()=>({resizeMode:true}),
          enumerateDevices:async()=>[],
          getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
        },
      },configurable:true,writable:true});
      const video=makeMockVideo({withRvfc:true});
      const observations=[];
      const cam=createReferenceCamera({
        video,geometry:rig,
        getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null}),
        getContext:()=>({sessionId:'s1',spaceEpoch:0}),
        onObservation:(obs,sample)=>observations.push({obs,sample}),
      });
      await cam.start();
      video.currentTime=2.0;
      video.fireRvfc(2000,{mediaTime:2.0,presentedFrames:1,captureTime:1994});
      await new Promise(r=>setTimeout(r,0));
      const worker=getWorker();
      assert.equal(worker.messages.length,1);
      assert.equal(worker.messages[0].bitmap,fakeBitmap);
      assert.equal(worker.messages[0].rgba,undefined);
      worker.onmessage({data:{type:'detections',serial:worker.messages[0].serial,markers:[],frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:6,detectMs:250}});
      assert.equal(observations.length,1);
      assert.equal(observations[0].sample.readbackMode,'worker-bitmap');
      cam.dispose();
    }finally{
      if(savedBmp===undefined)delete globalThis.createImageBitmap;else Object.defineProperty(globalThis,'createImageBitmap',{value:savedBmp,configurable:true,writable:true});
      if(savedOsc===undefined)delete globalThis.OffscreenCanvas;else Object.defineProperty(globalThis,'OffscreenCanvas',{value:savedOsc,configurable:true,writable:true});
    }
  });
});

test('createReferenceCamera switches detector decimate and maxHz dynamically',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
      },
    },configurable:true,writable:true});
    const video=makeMockVideo({withRvfc:true});
    let decimateSetting=1,maxHzSetting=3;
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null,decimate:decimateSetting,maxHz:maxHzSetting}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
    });
    await cam.start();
    video.currentTime=1.0;
    video.fireRvfc(1000,{mediaTime:1.0,presentedFrames:1,captureTime:995});
    const worker=getWorker();
    assert.equal(worker.messages.length,1);
    worker.onmessage({data:{type:'detections',serial:worker.messages[0].serial,markers:[],frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:5,detectMs:250}});
    // Switch to decimate=1.5 at 7 Hz (~143 ms interval) and fire after 160 ms.
    decimateSetting=1.5;maxHzSetting=7;
    video.currentTime=1.16;
    video.fireRvfc(1160,{mediaTime:1.16,presentedFrames:2,captureTime:1155});
    assert.equal(worker.messages.length,3,'must send config{decimate:1.5} followed by frame');
    assert.deepEqual(worker.messages[1],{type:'config',decimate:1.5});
    assert.equal(cam.state().decimate,1.5);
    worker.onmessage({data:{type:'detections',serial:worker.messages[2].serial,markers:[],frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:4,detectMs:90}});
    // Switch to fast mode (decimate=2, 10 Hz -> 100 ms interval) and fire after 120 ms.
    decimateSetting=2;maxHzSetting=10;
    video.currentTime=1.28;
    video.fireRvfc(1280,{mediaTime:1.28,presentedFrames:3,captureTime:1275});
    assert.equal(worker.messages.length,5,'must send config{decimate:2} followed by frame');
    assert.deepEqual(worker.messages[3],{type:'config',decimate:2});
    assert.equal(cam.state().decimate,2);
    cam.dispose();
  });
});

test('createReferenceCamera transfers synchronous WebCodecs VideoFrame when available and falls back cleanly on error',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const savedVf=globalThis.VideoFrame;
    let shouldThrow=false;
    const createdFrames=[];
    class MockVideoFrame{
      constructor(src){
        if(shouldThrow)throw new Error('VideoFrame unsupported');
        this.src=src;this.closed=false;createdFrames.push(this);
      }
      close(){this.closed=true;}
    }
    Object.defineProperty(globalThis,'VideoFrame',{value:MockVideoFrame,configurable:true,writable:true});
    try{
      const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
      Object.defineProperty(globalThis,'navigator',{value:{
        userAgent:'QuestMock/1.0',
        mediaDevices:{
          getSupportedConstraints:()=>({resizeMode:true}),
          enumerateDevices:async()=>[],
          getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
        },
      },configurable:true,writable:true});
      const video=makeMockVideo({withRvfc:true});
      const observations=[];
      const cam=createReferenceCamera({
        video,geometry:rig,
        getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null,maxHz:10}),
        getContext:()=>({sessionId:'s1',spaceEpoch:0}),
        onObservation:(obs,sample)=>observations.push({obs,sample}),
      });
      await cam.start();
      video.currentTime=1.0;
      video.fireRvfc(1000,{mediaTime:1.0,presentedFrames:1,captureTime:992});
      const worker=getWorker();
      // Synchronous transfer: no microtask delay needed.
      assert.equal(worker.messages.length,1);
      assert.equal(worker.messages[0].videoFrame,createdFrames[0]);
      assert.equal(worker.messages[0].rgba,undefined);
      worker.onmessage({data:{type:'detections',serial:worker.messages[0].serial,markers:[],frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:2.1,detectMs:82,readbackMode:'worker-videoframe-y'}});
      assert.equal(observations.length,1);
      assert.equal(observations[0].sample.readbackMode,'worker-videoframe-y');

      // If VideoFrame construction throws, seamlessly fall back to main canvas without erroring.
      shouldThrow=true;
      video.currentTime=1.2;
      video.fireRvfc(1200,{mediaTime:1.2,presentedFrames:2,captureTime:1192});
      assert.equal(worker.messages.length,2);
      assert.ok(worker.messages[1].rgba instanceof ArrayBuffer);
      cam.dispose();
    }finally{
      if(savedVf===undefined)delete globalThis.VideoFrame;
      else Object.defineProperty(globalThis,'VideoFrame',{value:savedVf,configurable:true,writable:true});
    }
  });
});

test('createReferenceCamera forwards detectMode/trackRefIds only when provided and records ROI telemetry',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
      },
    },configurable:true,writable:true});
    const video=makeMockVideo({withRvfc:true});
    let detectMode='two-stage',trackRefIds=[350,351,352];
    const observations=[];
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null,maxHz:12,detectMode,trackRefIds}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
      onObservation:(obs,sample)=>observations.push({obs,sample}),
    });
    await cam.start();
    video.currentTime=1.0;
    video.fireRvfc(1000,{mediaTime:1.0,presentedFrames:1,captureTime:992});
    const worker=getWorker();
    assert.equal(worker.messages.length,2);
    assert.deepEqual(worker.messages[0],{type:'config',detectMode:'two-stage',trackRefIds:[350,351,352],decimate:1});
    assert.equal(cam.state().detectMode,'two-stage');
    worker.onmessage({data:{
      type:'detections',serial:worker.messages[1].serial,markers:[],
      frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:3,detectMs:28,
      detectMode:'two-stage',coarseMs:14,roiMs:13,roiCount:2,
    }});
    assert.equal(observations.length,1);
    assert.equal(observations[0].sample.detectMode,'two-stage');
    assert.equal(observations[0].sample.coarseMs,14);
    assert.equal(observations[0].sample.roiMs,13);
    assert.equal(observations[0].sample.roiCount,2);

    trackRefIds=null;
    video.currentTime=1.1;
    video.fireRvfc(1100,{mediaTime:1.1,presentedFrames:2,captureTime:1092});
    assert.equal(worker.messages.length,4);
    assert.deepEqual(worker.messages[2],{type:'config',trackRefIds:null,decimate:1});
    cam.dispose();
  });
});

test('createReferenceCamera forwards priorRois when provided and records priorRoiCount on sample',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
      },
    },configurable:true,writable:true});
    const video=makeMockVideo({withRvfc:true});
    const observations=[];
    const expectedRois=[[100,100,240,240],[500,400,640,540]];
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null,maxHz:12,detectMode:'two-stage',priorRois:expectedRois}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
      onObservation:(obs,sample)=>observations.push({obs,sample}),
    });
    await cam.start();
    video.currentTime=1.0;
    video.fireRvfc(1000,{mediaTime:1.0,presentedFrames:1,captureTime:992});
    const worker=getWorker();
    const frameMsg=worker.messages[1];
    assert.deepEqual(frameMsg.priorRois,expectedRois);
    worker.onmessage({data:{
      type:'detections',serial:frameMsg.serial,markers:[],
      frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:3,detectMs:35,roiCount:2,
    }});
    assert.equal(observations.length,1);
    assert.equal(observations[0].sample.priorRoiCount,2);
    cam.dispose();
  });
});

test('createReferenceCamera accepts pre-solved worker observations and passes K/geometry/solveUnassigned inline',async()=>{
  await withBrowserMocks(async({getWorker})=>{
    const track={readyState:'live',muted:false,label:'Quest RGB',stop(){},getSettings:()=>({deviceId:'cam1',resizeMode:'none'}),addEventListener(){}};
    Object.defineProperty(globalThis,'navigator',{value:{
      userAgent:'QuestMock/1.0',
      mediaDevices:{
        getSupportedConstraints:()=>({resizeMode:true}),
        enumerateDevices:async()=>[],
        getUserMedia:async()=>({getVideoTracks:()=>[track],getTracks:()=>[track]}),
      },
    },configurable:true,writable:true});
    const video=makeMockVideo({withRvfc:true});
    const observations=[];
    const cam=createReferenceCamera({
      video,geometry:rig,
      getSettings:()=>({horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:null,maxHz:12,solveUnassigned:true}),
      getContext:()=>({sessionId:'s1',spaceEpoch:0}),
      onObservation:(obs,sample)=>observations.push({obs,sample}),
    });
    await cam.start();
    video.currentTime=1.0;
    video.fireRvfc(1000,{mediaTime:1.0,presentedFrames:1,captureTime:992});
    const worker=getWorker();
    assert.equal(worker.messages.length,1);
    const frameMsg=worker.messages[0];
    assert.ok(frameMsg.K&&frameMsg.K.width===1280&&frameMsg.K.height===1080);
    assert.equal(frameMsg.geometry,rig);
    assert.equal(frameMsg.printScale,1);
    assert.equal(frameMsg.solveUnassigned,true);

    const fakePose={R:[[1,0,0],[0,1,0],[0,0,1]],t:[0,0,0.5]};
    const preSolvedObs={
      layoutId:rig.id,printScale:1,imageSize:[1280,1080],cameraModel:frameMsg.K,
      calibrated:false,scenePose:null,tags:[],tools:[],rejected:[{id:305,reason:'unassigned_id'}],
      unassignedDetections:[{id:305,family:'apriltag-36h11',corners:[],registered:false,pose:fakePose,edgeMm:20,minEdgePx:24,areaPx:576,rmsPx:0.25,workerSolved:true}],
    };
    worker.onmessage({data:{
      type:'detections',serial:frameMsg.serial,markers:[],observation:preSolvedObs,poseFitMs:0.42,
      frameEvidence:{fresh:true,pixelsChanged:true},preprocessMs:2,detectMs:25,
    }});
    assert.equal(observations.length,1);
    assert.equal(observations[0].obs,preSolvedObs);
    assert.equal(observations[0].obs.unassignedDetections[0].pose,fakePose);
    assert.equal(observations[0].sample.poseFitMs,0.42);
    cam.dispose();
  });
});
