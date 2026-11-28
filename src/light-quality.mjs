// Lighting-quality estimator (diagnostic, development-only for now).
//
// Motivation: the Quest passthrough camera is rolling-shutter, ~30 FPS and has an
// uncontrollable auto-exposure. Backlight (window behind the tag) makes AE expose for
// the bright background -> tags are dark/low-contrast; clipped highlights destroy the
// black/white border the detector needs. We cannot fix that in software, but we can
// MEASURE it and show it to the user (traffic light + sparkline) to find out whether
// lighting is really the cause of poor tracking.
//
// Cost control: runs in the detector worker on the gray frame that already exists, at
// most every `LIGHT_INTERVAL_MS` (~5 Hz) and on a strided subsample (<0.5 ms at 1280x1080).
// Pure functions, no allocation in the hot loop except one reused 256-bin histogram.

export const LIGHT_INTERVAL_MS=200;   // ~5 Hz analysis rate
const GLOBAL_STRIDE=4;                // sample every 4th pixel in x and y (1/16 of pixels)
const MAX_TAGS=8;                     // cap per-tag work
const CLIP_HI=250;                    // >= this counts as blown-out highlight
const CLIP_LO=8;                      // <= this counts as crushed shadow

const hist=new Uint32Array(256);

// Percentile (0..1) from the shared histogram holding `total` samples.
function percentile(total,p) {
  const target=total*p;let acc=0;
  for(let i=0;i<256;i++) {acc+=hist[i];if(acc>=target) return i;}
  return 255;
}
const clamp01=v=>v<0?0:v>1?1:v;

// Histogram statistics of a rectangular region (inclusive-exclusive), strided.
function regionStats(gray,width,x0,y0,x1,y1,stride) {
  hist.fill(0);
  let n=0,sum=0;
  for(let y=y0;y<y1;y+=stride) {
    const row=y*width;
    for(let x=x0;x<x1;x+=stride) {const v=gray[row+x];hist[v]++;sum+=v;n++;}
  }
  if(!n) return null;
  let hi=0,lo=0;
  for(let i=CLIP_HI;i<256;i++) hi+=hist[i];
  for(let i=0;i<=CLIP_LO;i++) lo+=hist[i];
  const p5=percentile(n,0.05),p50=percentile(n,0.5),p95=percentile(n,0.95);
  return {n,mean:sum/n,p5,p50,p95,contrast:p95-p5,clipHi:hi/n,clipLo:lo/n};
}

/**
 * Analyze one gray frame.
 * @param {Uint8Array} gray  width*height luminance
 * @param {number} width
 * @param {number} height
 * @param {Array<{corners:{x:number,y:number}[]}>} markers  detections in the SAME pixel space as `gray`
 * @param {{meanLuma:number}|null} previous  previous result, used for auto-exposure drift
 * @returns {object} metrics + `score` (0..100) + `level` ('good'|'warn'|'bad') + `flags`
 */
export function analyzeLight(gray,width,height,markers=[],previous=null) {
  const g=regionStats(gray,width,0,0,width,height,GLOBAL_STRIDE);
  if(!g) return null;

  // Per-tag stats: contrast/clipping INSIDE the tag's bounding box is what the detector sees.
  let tagCount=0,tagContrastMin=Infinity,tagClipMax=0,tagMeanMin=255;
  for(const m of markers) {
    if(tagCount>=MAX_TAGS) break;
    if(!Array.isArray(m?.corners)||m.corners.length<4) continue;
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
    for(const p of m.corners) {
      if(p.x<minX) minX=p.x;if(p.x>maxX) maxX=p.x;
      if(p.y<minY) minY=p.y;if(p.y>maxY) maxY=p.y;
    }
    const x0=Math.max(0,Math.floor(minX)),y0=Math.max(0,Math.floor(minY));
    const x1=Math.min(width,Math.ceil(maxX)),y1=Math.min(height,Math.ceil(maxY));
    if(x1-x0<4||y1-y0<4) continue;
    const s=regionStats(gray,width,x0,y0,x1,y1,Math.max(1,Math.min(x1-x0,y1-y0)>>5));
    if(!s) continue;
    tagCount++;
    if(s.contrast<tagContrastMin) tagContrastMin=s.contrast;
    if(s.clipHi>tagClipMax) tagClipMax=s.clipHi;
    if(s.mean<tagMeanMin) tagMeanMin=s.mean;
  }

  // Auto-exposure hunting: mean luminance jumping between analyses (~200 ms apart).
  const aeDrift=previous?Math.abs(g.mean-previous.meanLuma):0;

  // Sub-scores in 0..1 — the final score is the WEAKEST link (one bad factor is enough to fail tracking).
  const sExposure=1-clamp01(Math.abs(g.mean-120)/100);               // too dark or too bright overall
  const sClip=1-clamp01((g.clipHi+g.clipLo*0.5-0.02)/0.18);           // blown highlights / crushed shadows
  const sTag=tagCount?clamp01((tagContrastMin-30)/70)*(1-clamp01(tagClipMax/0.25)):null; // tag border contrast
  const sAe=1-clamp01((aeDrift-6)/30);                               // AE settling

  const flags=[];
  // Backlight signature: a noticeable blown-out area AND a dark median (AE exposed for the bright part).
  if(g.clipHi>0.04&&g.p50<95) flags.push('backlight');
  if(g.mean<55) flags.push('dark');
  if(g.mean>195||g.clipHi>0.25) flags.push('overexposed');
  if(aeDrift>18) flags.push('ae-hunting');
  if(tagCount&&tagContrastMin<45) flags.push('tag-low-contrast');
  if(tagCount&&tagClipMax>0.15) flags.push('tag-clipped');

  const parts=[sExposure,sClip,sAe];
  if(sTag!=null) parts.push(sTag);
  const score=Math.round(100*Math.min(...parts));
  const level=score>=70?'good':score>=40?'warn':'bad';
  return {
    score,level,flags,
    meanLuma:g.mean,p5:g.p5,p50:g.p50,p95:g.p95,dynamicRange:g.contrast,
    clipHi:g.clipHi,clipLo:g.clipLo,aeDrift,
    tagCount,tagContrastMin:tagCount?tagContrastMin:null,tagClipMax:tagCount?tagClipMax:null,tagMeanMin:tagCount?tagMeanMin:null,
  };
}
