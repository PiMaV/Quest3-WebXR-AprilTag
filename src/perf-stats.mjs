// Zero-allocation frame-timing, HUD upload, and anchor-step stability tracker.

const round2=v=>Math.round(v*100)/100;

function percentiles(buf,count,maxVal){
  if(!count)return null;
  const sorted=buf.slice(0,count).sort();
  const at=p=>sorted[Math.min(count-1,Math.floor(count*p))];
  return {
    p50:round2(at(0.50)),
    p95:round2(at(0.95)),
    p99:round2(at(0.99)),
    max:round2(maxVal),
  };
}

function makeBucket(capacity){
  return {
    dt:new Float32Array(capacity),
    cpu:new Float32Array(capacity),
    renderCpu:new Float32Array(capacity),
    dtHead:0,
    dtCount:0,
    cpuHead:0,
    cpuCount:0,
    frames:0,
    dropped14ms:0,
    dropped20ms:0,
    dropped33ms:0,
    maxDtMs:0,
    maxCpuMs:0,
    maxRenderCpuMs:0,
  };
}

function resetBucket(b){
  b.dtHead=0;b.dtCount=0;
  b.cpuHead=0;b.cpuCount=0;
  b.frames=0;b.dropped14ms=0;b.dropped20ms=0;b.dropped33ms=0;
  b.maxDtMs=0;b.maxCpuMs=0;b.maxRenderCpuMs=0;
}

function snapshotBucket(b){
  if(!b.frames){
    return {frames:0,dropped14ms:0,dropped20ms:0,dropped33ms:0,dtMs:null,cpuMs:null,renderCpuMs:null};
  }
  return {
    frames:b.frames,
    dropped14ms:b.dropped14ms,
    dropped20ms:b.dropped20ms,
    dropped33ms:b.dropped33ms,
    dtMs:percentiles(b.dt,b.dtCount,b.maxDtMs),
    cpuMs:percentiles(b.cpu,b.cpuCount,b.maxCpuMs),
    renderCpuMs:percentiles(b.renderCpu,b.cpuCount,b.maxRenderCpuMs),
  };
}

export function createPerfTracker({capacity=2048}={}){
  const cap=Math.max(16,capacity|0);
  const buckets={
    overall:makeBucket(cap),
    calibrating:makeBucket(cap),
    measure:makeBucket(cap),
    scan:makeBucket(cap),
  };
  const stepBuf=new Float32Array(cap);
  let stepHead=0,stepCount=0,stepSamples=0;
  let maxStepMm=0,maxStepDeg=0,maxJitterMm=null,maxDriftMm=null;
  let hudVisibleFrames=0,uploadsLeft=0,uploadsRight=0,uploadsTop=0,maxHudDrawMs=0;

  function pushBucket(b,dtMs,cpuMs,renderCpuMs){
    b.frames++;
    if(Number.isFinite(dtMs)&&dtMs>0&&dtMs<2000){
      b.dt[b.dtHead]=dtMs;
      b.dtHead=(b.dtHead+1)%cap;
      if(b.dtCount<cap)b.dtCount++;
      if(dtMs>b.maxDtMs)b.maxDtMs=dtMs;
      if(dtMs>14)b.dropped14ms++;
      if(dtMs>20)b.dropped20ms++;
      if(dtMs>33)b.dropped33ms++;
    }
    if(Number.isFinite(cpuMs)&&cpuMs>=0){
      const rMs=Number.isFinite(renderCpuMs)&&renderCpuMs>=0?renderCpuMs:0;
      b.cpu[b.cpuHead]=cpuMs;
      b.renderCpu[b.cpuHead]=rMs;
      b.cpuHead=(b.cpuHead+1)%cap;
      if(b.cpuCount<cap)b.cpuCount++;
      if(cpuMs>b.maxCpuMs)b.maxCpuMs=cpuMs;
      if(rMs>b.maxRenderCpuMs)b.maxRenderCpuMs=rMs;
    }
  }

  function recordFrame(mode,dtMs,cpuMs,renderCpuMs){
    pushBucket(buckets.overall,dtMs,cpuMs,renderCpuMs);
    const target=buckets[mode];
    if(target&&target!==buckets.overall)pushBucket(target,dtMs,cpuMs,renderCpuMs);
  }

  function recordHudFrame(visible){
    if(visible)hudVisibleFrames++;
  }

  function recordHudUpload(slot,drawMs){
    if(slot==='left')uploadsLeft++;
    else if(slot==='right')uploadsRight++;
    else if(slot==='top')uploadsTop++;
    if(Number.isFinite(drawMs)&&drawMs>maxHudDrawMs)maxHudDrawMs=drawMs;
  }

  function recordAnchorStep(stepMm,stepDeg,jitterMm,driftMm){
    if(Number.isFinite(stepMm)&&stepMm>=0){
      stepBuf[stepHead]=stepMm;
      stepHead=(stepHead+1)%cap;
      if(stepCount<cap)stepCount++;
      stepSamples++;
      if(stepMm>maxStepMm)maxStepMm=stepMm;
    }
    if(Number.isFinite(stepDeg)&&stepDeg>maxStepDeg)maxStepDeg=stepDeg;
    if(Number.isFinite(jitterMm)&&(maxJitterMm==null||jitterMm>maxJitterMm))maxJitterMm=jitterMm;
    if(Number.isFinite(driftMm)&&(maxDriftMm==null||driftMm>maxDriftMm))maxDriftMm=driftMm;
  }

  function reset(){
    resetBucket(buckets.overall);
    resetBucket(buckets.calibrating);
    resetBucket(buckets.measure);
    resetBucket(buckets.scan);
    stepHead=0;stepCount=0;stepSamples=0;
    maxStepMm=0;maxStepDeg=0;maxJitterMm=null;maxDriftMm=null;
    hudVisibleFrames=0;uploadsLeft=0;uploadsRight=0;uploadsTop=0;maxHudDrawMs=0;
  }

  function snapshot(){
    return {
      frameTiming:{
        overall:snapshotBucket(buckets.overall),
        calibrating:snapshotBucket(buckets.calibrating),
        measure:snapshotBucket(buckets.measure),
        scan:snapshotBucket(buckets.scan),
      },
      hud:{
        visibleFrames:hudVisibleFrames,
        uploadsLeft,
        uploadsRight,
        uploadsTop,
        maxHudDrawMs:round2(maxHudDrawMs),
      },
      anchorStability:{
        stepSamples,
        stepMm:percentiles(stepBuf,stepCount,maxStepMm),
        maxStepDeg:stepSamples?round2(maxStepDeg):null,
        maxJitterMm:maxJitterMm!=null?round2(maxJitterMm):null,
        maxDriftMm:maxDriftMm!=null?round2(maxDriftMm):null,
      },
    };
  }

  return {recordFrame,recordHudFrame,recordHudUpload,recordAnchorStep,reset,snapshot};
}
