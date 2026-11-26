import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {createRequire} from 'node:module';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
import {createAprilTagDetector,prioritizeFocusCandidates,projectPriorRois} from '../src/detector.mjs';
import {prepareRgbaFrame} from '../src/frame-evidence.mjs';
import {vectorRigid} from '../src/runtime.mjs';

function renderTag36h11(code,{width=200,height=200,cell=14,ox=44,oy=44}={}){
  const rgba=new Uint8ClampedArray(width*height*4).fill(255);
  for(let r=0;r<8;r++){
    for(let c=0;c<8;c++){
      let val=0;
      if(r>0&&r<7&&c>0&&c<7){
        const bit=(r-1)*6+(c-1);
        val=Number((code>>BigInt(35-bit))&1n)?255:0;
      }
      for(let dy=0;dy<cell;dy++){
        for(let dx=0;dx<cell;dx++){
          const idx=((oy+r*cell+dy)*width+(ox+c*cell+dx))*4;
          rgba[idx]=rgba[idx+1]=rgba[idx+2]=val;rgba[idx+3]=255;
        }
      }
    }
  }
  return rgba;
}

test('vendored AprilTag WASM loads in Node and detects tag36h11 with LL,LR,UR,UL corner order',async()=>{
  const vendorDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../vendor/apriltag');
  const src=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.js'),'utf8');
  const AprilTagWasm=new Function('require','__dirname','__filename',`${src}; return AprilTagWasm;`)(
    createRequire(import.meta.url),vendorDir,path.join(vendorDir,'apriltag_wasm.js'),
  );
  const wasmBinary=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.wasm'));
  const wasm=await AprilTagWasm({wasmBinary});
  const detector=createAprilTagDetector(wasm,1);

  // Official tag36h11 row-major bitmaps (ID 0 + A4 IDs 350/351/352 + Tablet/e-Ink IDs 370/371/372).
  for(const [expectedId,code] of [
    [0,0x000000d5d628584n],
    [350,0x89ded4ea1n],[351,0x256a0ba34n],[352,0xb6948e627n],
    [370,0xeafb4ae59n],[371,0x7ca7fe2bdn],[372,0x1490ca8f6n],
  ]){
    const rgba=renderTag36h11(code,{width:200,height:200,cell:14,ox:44,oy:44});
    const {gray,evidence}=prepareRgbaFrame(rgba,200,200,null,1);
    assert.equal(evidence.fresh,true);
    const markers=detector.detect(gray,200,200);
    assert.equal(markers.length,1,`expected 1 marker for ID ${expectedId}`);
    assert.equal(markers[0].id,expectedId);
    const c=markers[0].corners;
    // Verify LL, LR, UR, UL order in Y-down image coordinates and negative cross-product winding.
    assert.ok(c[0].x<60&&c[0].y>140,'p[0] must be LL');
    assert.ok(c[1].x>140&&c[1].y>140,'p[1] must be LR');
    assert.ok(c[2].x>140&&c[2].y<60,'p[2] must be UR');
    assert.ok(c[3].x<60&&c[3].y<60,'p[3] must be UL');
    const turns=c.map((p,i)=>{const q=c[(i+1)%4],r=c[(i+2)%4];return (q.x-p.x)*(r.y-q.y)-(q.y-p.y)*(r.x-q.x);});
    assert.ok(turns.every(v=>v<0),'quad winding must match observeReference convention');
  }
  detector.setDecimate(1.5);
  const midMarkers=detector.detect(prepareRgbaFrame(renderTag36h11(0x89ded4ea1n,{width:200,height:200,cell:14,ox:44,oy:44}),200,200,null,2).gray,200,200);
  assert.equal(midMarkers.length,1);
  assert.equal(midMarkers[0].id,350);
  detector.setDecimate(2);
  const fastMarkers=detector.detect(prepareRgbaFrame(renderTag36h11(0x89ded4ea1n,{width:200,height:200,cell:14,ox:44,oy:44}),200,200,null,3).gray,200,200);
  assert.equal(fastMarkers.length,1);
  assert.equal(fastMarkers[0].id,350);
  assert.throws(()=>detector.setDecimate(3),/Invalid decimate/);
  assert.throws(()=>detector.detect(new Uint8Array(10),200,200),/Invalid grayscale frame/);
});

function paintTagGray(gray,width,height,code,{cell,ox,oy}){
  const size=cell*8,qPad=Math.max(4,cell*2);
  for(let y=Math.max(0,oy-qPad);y<Math.min(height,oy+size+qPad);y++){
    for(let x=Math.max(0,ox-qPad);x<Math.min(width,ox+size+qPad);x++){
      gray[y*width+x]=245;
    }
  }
  for(let r=0;r<8;r++){
    for(let c=0;c<8;c++){
      let val=15;
      if(r>0&&r<7&&c>0&&c<7){
        const bit=(r-1)*6+(c-1);
        val=Number((code>>BigInt(35-bit))&1n)?245:15;
      }
      for(let dy=0;dy<cell;dy++){
        for(let dx=0;dx<cell;dx++){
          gray[(oy+r*cell+dy)*width+(ox+c*cell+dx)]=val;
        }
      }
    }
  }
}

test('detectTwoStage and detectTrackedRoi match full-frame dec=1 corner accuracy on textured 1280x1080 frames',async()=>{
  const vendorDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../vendor/apriltag');
  const src=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.js'),'utf8');
  const AprilTagWasm=new Function('require','__dirname','__filename',`${src}; return AprilTagWasm;`)(
    createRequire(import.meta.url),vendorDir,path.join(vendorDir,'apriltag_wasm.js'),
  );
  const wasmBinary=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.wasm'));
  const wasm=await AprilTagWasm({wasmBinary});
  const detector=createAprilTagDetector(wasm,1);

  const W=1280,H=1080;
  const gray=new Uint8Array(W*H);
  for(let y=0;y<H;y++){
    for(let x=0;x<W;x++){
      gray[y*W+x]=140+((x*7+y*13)%90);
    }
  }
  const specs=[
    {id:350,code:0x89ded4ea1n,cell:6,ox:260,oy:260}, // 48 px
    {id:351,code:0x256a0ba34n,cell:5,ox:430,oy:270}, // 40 px
    {id:352,code:0xb6948e627n,cell:4,ox:280,oy:430}, // 32 px
    {id:370,code:0xeafb4ae59n,cell:3,ox:820,oy:340}, // 24 px
    {id:371,code:0x7ca7fe2bdn,cell:2,ox:980,oy:760}, // 16 px tiny tag
  ];
  for(const sp of specs)paintTagGray(gray,W,H,sp.code,sp);

  const baseline=detector.detect(gray,W,H);
  assert.equal(baseline.length,5,'baseline dec=1 should detect all 5 tags');
  const baseById=new Map(baseline.map(m=>[m.id,m]));

  const two=detector.detectTwoStage(gray,W,H);
  assert.equal(two.markers.length,5,'detectTwoStage must recall all 5/5 tags including 16px tag');
  assert.ok(two.roiCount>=2&&two.roiCount<=5);
  let maxErrPx=0;
  for(const m of two.markers){
    const ref=baseById.get(m.id);
    assert.ok(ref);
    for(let i=0;i<4;i++){
      const err=Math.hypot(m.corners[i].x-ref.corners[i].x,m.corners[i].y-ref.corners[i].y);
      if(err>maxErrPx)maxErrPx=err;
    }
  }
  assert.ok(maxErrPx<0.15,`expected <0.15px corner delta vs dec=1, got ${maxErrPx.toFixed(4)}px`);

  // Tracked ROI covering L-bracket [350, 351, 352] hits in 1 ROI without coarse pass.
  const hit=detector.detectTrackedRoi(gray,W,H,[200,200,560,540],{requiredIds:[350,351,352]});
  assert.equal(hit.trackedHit,true);
  assert.equal(hit.roiCount,1);
  assert.equal(hit.coarseMs,0);
  const hitIds=new Set(hit.markers.map(m=>m.id));
  assert.ok([350,351,352].every(id=>hitIds.has(id)));

  // Hint missing ID 352 automatically falls back to two-stage discovery and returns all 5 tags.
  const miss=detector.detectTrackedRoi(gray,W,H,[200,200,520,380],{requiredIds:[350,351,352]});
  assert.equal(miss.trackedHit,false);
  assert.equal(miss.markers.length,5);

  // coarseDecimate=1.5 matches full-frame accuracy as well.
  const mid=detector.detectTwoStage(gray,W,H,{coarseDecimate:1.5});
  assert.equal(mid.markers.length,5);
  for(const m of mid.markers){
    const ref=baseById.get(m.id);
    assert.ok(ref);
    for(let i=0;i<4;i++){
      assert.ok(Math.hypot(m.corners[i].x-ref.corners[i].x,m.corners[i].y-ref.corners[i].y)<0.15);
    }
  }

  // Focus-ROI fast near-tracking hits 2 tags directly without coarse scan and falls back cleanly on miss.
  const focusHit=detector.detectFocusRois(gray,W,H,[[210,210,510,510]]);
  assert.equal(focusHit.focusHit,true);
  assert.equal(focusHit.coarseMs,0);
  assert.equal(focusHit.roiCount,1);
  const fIds=new Set(focusHit.markers.map(m=>m.id));
  assert.ok(fIds.has(350)&&fIds.has(351)&&fIds.has(352));

  const focusMiss=detector.detectFocusRois(gray,W,H,[[20,20,120,120]],{fallbackOpts:{coarseDecimate:2}});
  assert.equal(focusMiss.focusHit,false);
  assert.equal(focusMiss.markers.length,5);
});

function paintFractionalTagGray(gray,width,height,code,{sizePx=14,ox=632,oy=532,blur=false}={}){
  const qPad=Math.max(4,Math.round(sizePx*0.5));
  const yLo=Math.max(1,oy-qPad),yHi=Math.min(height-1,Math.ceil(oy+sizePx+qPad));
  const xLo=Math.max(1,ox-qPad),xHi=Math.min(width-1,Math.ceil(ox+sizePx+qPad));
  for(let y=yLo;y<yHi;y++){
    for(let x=xLo;x<xHi;x++){
      gray[y*width+x]=195;
    }
  }
  const bitVal=(r,c)=>{
    if(r===0||r===7||c===0||c===7)return 55;
    const bit=(r-1)*6+(c-1);
    return Number((code>>BigInt(35-bit))&1n)?195:55;
  };
  const cell=sizePx/8;
  const xStart=Math.floor(ox),yStart=Math.floor(oy);
  const xEnd=Math.ceil(ox+sizePx),yEnd=Math.ceil(oy+sizePx);
  for(let py=yStart;py<yEnd;py++){
    for(let px=xStart;px<xEnd;px++){
      let acc=0;
      for(let sy=0;sy<3;sy++){
        for(let sx=0;sx<3;sx++){
          const fx=((px+(sx+0.5)/3)-ox)/cell;
          const fy=((py+(sy+0.5)/3)-oy)/cell;
          const rr=Math.floor(fy),cc=Math.floor(fx);
          acc+=(rr>=0&&rr<8&&cc>=0&&cc<8)?bitVal(rr,cc):195;
        }
      }
      gray[py*width+px]=Math.round(acc/9);
    }
  }
  if(blur){
    const tmp=gray.slice();
    for(let y=yLo;y<yHi;y++){
      for(let x=xLo;x<xHi;x++){
        let s=0;
        for(let dy=-1;dy<=1;dy++){
          for(let dx=-1;dx<=1;dx++){
            s+=tmp[(y+dy)*width+(x+dx)]*((dx===0?2:1)*(dy===0?2:1));
          }
        }
        gray[y*width+x]=Math.round(s/16);
      }
    }
  }
}

test('detectTwoStage extends detection range via coarseDecimate=1.5 (3A) and priorRois (3B)',async()=>{
  const vendorDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../vendor/apriltag');
  const src=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.js'),'utf8');
  const AprilTagWasm=new Function('require','__dirname','__filename',`${src}; return AprilTagWasm;`)(
    createRequire(import.meta.url),vendorDir,path.join(vendorDir,'apriltag_wasm.js'),
  );
  const wasmBinary=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.wasm'));
  const wasm=await AprilTagWasm({wasmBinary});
  const detector=createAprilTagDetector(wasm,1);

  const W=1280,H=1080;
  const makeBg=()=>{
    const g=new Uint8Array(W*H);
    for(let y=0;y<H;y++)for(let x=0;x<W;x++)g[y*W+x]=135+((x*5+y*11)%70);
    return g;
  };

  // Case 1 (Step 3A ~1.05 m equivalent): 16 px camera-softened tag is missed at dec=2 but caught by coarseDecimate=1.5.
  const gray16=makeBg();
  paintFractionalTagGray(gray16,W,H,0x89ded4ea1n,{sizePx:16,ox:632,oy:532,blur:true});
  assert.equal(detector.detectTwoStage(gray16,W,H,{coarseDecimate:2}).markers.length,0,'coarseDecimate=2 misses 16 px soft tag');
  const step3A=detector.detectTwoStage(gray16,W,H,{coarseDecimate:1.5});
  assert.equal(step3A.markers.length,1,'coarseDecimate=1.5 detects 16 px soft tag');
  assert.equal(step3A.markers[0].id,350);

  // Case 2 (Step 3B ~1.21 m equivalent): 14 px camera-softened tag is missed by dec=2 and dec=1.5, recovered by priorRois (dec=1).
  const gray14=makeBg();
  paintFractionalTagGray(gray14,W,H,0x89ded4ea1n,{sizePx:14,ox:632,oy:532,blur:true});
  assert.equal(detector.detectTwoStage(gray14,W,H,{coarseDecimate:2}).markers.length,0,'coarseDecimate=2 misses 14 px soft tag');
  assert.equal(detector.detectTwoStage(gray14,W,H,{coarseDecimate:1.5}).markers.length,0,'coarseDecimate=1.5 misses 14 px soft tag');

  const guided=detector.detectTwoStage(gray14,W,H,{coarseDecimate:2,priorRois:[[570,470,710,610]]});
  assert.equal(guided.markers.length,1,'priorRois must recover 14 px soft tag at full dec=1 resolution');
  assert.equal(guided.markers[0].id,350);
  assert.equal(guided.markers[0].softFallback,true,'14 px soft fallback marker must be tagged softFallback=true');
  assert.equal(guided.roiCount,1);

  // Overlapping aligned small priorRois (104x104) merge only locally; diagonal pairs within 16 px gap never chain into a mega-box.
  const fourSpread=[
    [80,80,184,184],
    [188,80,292,184],     // 4 px gap horizontally, SAME y -> merges cleanly with [80..184] (212x104 = 22048 px2)
    [600,480,704,584],    // diagonal pair within 6 px gap: unionArea 45796 > areaA+areaB+slack -> stays 2 tight boxes
    [710,590,814,694],
  ];
  const spread=detector.detectTwoStage(gray14,W,H,{coarseDecimate:2,priorRois:fourSpread});
  assert.equal(spread.roiCount,3,'collinear adjacent boxes merge locally while diagonal boxes remain separate');

  // Case 3 (Step 3C): 480x400 center-gaze discovery pulse recovers unmapped 16 px soft tag missed at coarseDecimate=2.
  const pulse=detector.detectTwoStage(gray16,W,H,{coarseDecimate:2,priorRois:[[400,340,880,740]]});
  assert.equal(pulse.markers.length,1,'center-gaze 480x400 pulse crop recovers unmapped 16 px soft tag');
  assert.equal(pulse.markers[0].id,350);
  assert.ok(!pulse.markers[0].softFallback,'refined 16 px pulse marker should not be flagged softFallback');
  assert.equal(pulse.roiCount,1);

  // Case 4 (Long-Range coarseDecimate=1): rescues blurred 14 px tag from coarse pass even if refineEdges=1 ROI drops it.
  const far=detector.detectTwoStage(gray14,W,H,{coarseDecimate:1});
  assert.equal(far.markers.length,1,'coarseDecimate=1 must rescue 14 px blurred tag');
  assert.equal(far.markers[0].id,350);
  assert.equal(far.markers[0].softFallback,true);
});

test('projectPriorRois projects L-frame 3D anchors into priority-capped CV image ROIs with GL->CV sign handling',()=>{
  const I=vectorRigid([0,0,0,0,0,0]);
  const K={width:1280,height:1080,fx:850,fy:850,cx:640,cy:540};
  const rois=projectPriorRois({
    pointsL:[
      {pos:[0,0,-1.0],halfPx:68},          // dead ahead at 1.0 m -> (640, 540)
      {pos:[0.2,0.1,-1.0],halfPx:50},      // right & up in GL -> u > 640, v < 540
      {pos:[0,0,-0.20],halfPx:68},         // too close (< minDistM=0.45) -> skipped
      {pos:[0,0,1.0],halfPx:68},           // behind camera (zGL > 0) -> skipped
      {pos:[0,0,-3.0],halfPx:68},          // too far (> maxDistM=1.85) -> skipped
      {pos:[-0.25,0.15,-1.0],halfPx:48},   // ROI #3
      {pos:[0.3,-0.2,-1.1],halfPx:48},     // ROI #4
      {pos:[-0.35,-0.25,-1.1],halfPx:48},  // ROI #5 -> truncated by maxRois=4
    ],
    worldFromAnchor:I,
    worldFromViewer:I,
    viewerFromCameraGL:I,
    K,
    width:1280,
    height:1080,
    maxRois:4,
  });
  assert.equal(rois.length,4);
  assert.deepEqual(rois[0],[640-68,540-68,640+68,540+68]);
  // Right & up in GL projects to u = 640 + 170 = 810, v = 540 - 85 = 455
  assert.deepEqual(rois[1],[810-50,455-50,810+50,455+50]);

  // Nearby overlapping points merge cleanly into 1 box.
  const merged=projectPriorRois({
    pointsL:[{pos:[0,0,-1.0],halfPx:60},{pos:[0.03,0,-1.0],halfPx:60}],
    worldFromAnchor:I,worldFromViewer:I,viewerFromCameraGL:I,K,width:1280,height:1080,
  });
  assert.equal(merged.length,1);
});

test('detectFocusRois recovers motion-blurred tags via unrefined ROI retry inside fast Focus-ROI path',async()=>{
  const vendorDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../vendor/apriltag');
  const src=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.js'),'utf8');
  const AprilTagWasm=new Function('require','__dirname','__filename',`${src}; return AprilTagWasm;`)(
    createRequire(import.meta.url),vendorDir,path.join(vendorDir,'apriltag_wasm.js'),
  );
  const wasmBinary=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.wasm'));
  const wasm=await AprilTagWasm({wasmBinary});
  const detector=createAprilTagDetector(wasm,1);

  const W=1280,H=1080,cell=5,ox=500,oy=400,blurPx=9;
  const sharp=new Float32Array(W*H).fill(180);
  const size=cell*8,qPad=cell*2;
  for(let y=oy-qPad;y<oy+size+qPad;y++)for(let x=ox-qPad;x<ox+size+qPad;x++)sharp[y*W+x]=240;
  for(let r=0;r<8;r++){
    for(let c=0;c<8;c++){
      let val=20;
      if(r>0&&r<7&&c>0&&c<7){
        const bit=(r-1)*6+(c-1);
        val=Number((0x89ded4ea1n>>BigInt(35-bit))&1n)?240:20;
      }
      for(let dy=0;dy<cell;dy++)for(let dx=0;dx<cell;dx++)sharp[(oy+r*cell+dy)*W+(ox+c*cell+dx)]=val;
    }
  }
  const blurred=new Uint8Array(W*H).fill(180);
  const half=Math.floor(blurPx/2);
  for(let y=oy-qPad;y<oy+size+qPad;y++){
    for(let x=ox-qPad-blurPx;x<ox+size+qPad+blurPx;x++){
      let sum=0;
      for(let dx=0;dx<blurPx;dx++)sum+=sharp[y*W+Math.min(W-1,Math.max(0,x-half+dx))];
      blurred[y*W+x]=Math.round(sum/blurPx);
    }
  }

  const res=detector.detectFocusRois(blurred,W,H,[[ox-60,oy-60,ox+size+60,oy+size+60]]);
  assert.equal(res.focusHit,true);
  assert.equal(res.markers.length,1);
  assert.equal(res.markers[0].id,350);
  assert.equal(res.markers[0].softFallback,true);
});

test('prioritizeFocusCandidates prioritizes active targets over static tags and picks closest-to-camera first',()=>{
  const candidates=[
    {tagId:309,recording:false,handCoupled:false,moving:false,distToCameraM:0.35,apparentEdgePx:60},
    {tagId:310,recording:false,handCoupled:false,moving:true,distToCameraM:1.10,apparentEdgePx:24},
    {tagId:305,recording:true,handCoupled:false,moving:true,distToCameraM:0.55,apparentEdgePx:42},
    {tagId:306,recording:false,handCoupled:false,moving:true,distToCameraM:0.45,apparentEdgePx:50},
    {tagId:311,recording:false,handCoupled:true,moving:true,distToCameraM:0.40,apparentEdgePx:48},
  ];
  // Active targets exclude resting static tag #309; recording/coupled #311 (0.40m) & #305 (0.55m) outrank uncoupled #306/#310.
  const two=prioritizeFocusCandidates(candidates,{maxCount:2});
  assert.deepEqual(two.map(c=>c.tagId),[311,305]);

  // Single recording pendulum + resting nearby table tags -> picks ONLY the recording pendulum so it gets 1 tight ~9-20ms ROI.
  const singlePendulum=prioritizeFocusCandidates([
    {tagId:309,recording:false,handCoupled:false,moving:false,distToCameraM:0.35,apparentEdgePx:60},
    {tagId:310,recording:false,handCoupled:false,moving:false,distToCameraM:0.40,apparentEdgePx:54},
    {tagId:305,recording:true,handCoupled:false,moving:true,distToCameraM:0.70,apparentEdgePx:36},
  ],{maxCount:2});
  assert.deepEqual(singlePendulum.map(c=>c.tagId),[305]);

  // When only close static tags are visible, sorts nearest-to-camera first (with apparentEdgePx tie-break).
  const staticOnly=prioritizeFocusCandidates([
    {tagId:301,distToCameraM:0.95,apparentEdgePx:28},
    {tagId:302,distToCameraM:0.50,apparentEdgePx:45},
    {tagId:303,distToCameraM:0.50,apparentEdgePx:52},
  ],{maxCount:2});
  assert.deepEqual(staticOnly.map(c=>c.tagId),[303,302]);

  // Two-tag ruler: recording #311 keeps sibling #310 (15 cm away) even if resting, while dropping distant #309 (70 cm away).
  const rulerPair=prioritizeFocusCandidates([
    {tagId:311,recording:true,handCoupled:false,moving:true,pos:[0,0,0],distToCameraM:0.32,apparentEdgePx:62},
    {tagId:310,recording:false,handCoupled:false,moving:false,settling:false,pos:[0.15,0,0],distToCameraM:0.34,apparentEdgePx:58},
    {tagId:309,recording:false,handCoupled:false,moving:false,settling:false,pos:[0.70,0,0],distToCameraM:0.85,apparentEdgePx:24},
  ],{maxCount:3});
  assert.deepEqual(rulerPair.map(c=>c.tagId),[311,310]);

  // Moving/coupled (not recording) never starves nearby resting or newly settling sibling tags.
  const movingPlusSettling=prioritizeFocusCandidates([
    {tagId:311,recording:false,handCoupled:true,moving:true,pos:[0,0,0],distToCameraM:0.30,apparentEdgePx:64},
    {tagId:310,recording:false,handCoupled:false,moving:false,settling:true,pos:[0.12,0,0],distToCameraM:0.31,apparentEdgePx:60},
  ],{maxCount:3});
  assert.deepEqual(movingPlusSettling.map(c=>c.tagId),[311,310]);
});

test('two-stage large Focus-ROI matches dec=1 corner accuracy, reports partial-miss counts, and rescues blur',async()=>{
  const vendorDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../vendor/apriltag');
  const src=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.js'),'utf8');
  const AprilTagWasm=new Function('require','__dirname','__filename',`${src}; return AprilTagWasm;`)(
    createRequire(import.meta.url),vendorDir,path.join(vendorDir,'apriltag_wasm.js'),
  );
  const wasmBinary=fs.readFileSync(path.join(vendorDir,'apriltag_wasm.wasm'));
  const wasm=await AprilTagWasm({wasmBinary});
  const detector=createAprilTagDetector(wasm,1);

  const W=1280,H=1080;
  const gray=new Uint8Array(W*H);
  for(let y=0;y<H;y++){
    for(let x=0;x<W;x++)gray[y*W+x]=140+((x*7+y*13)%90);
  }
  const tagSpecs=[
    {id:350,code:0x89ded4ea1n,cell:8,ox:260,oy:260}, // 64 px tag inside Box 1
    {id:351,code:0x256a0ba34n,cell:8,ox:760,oy:260}, // 64 px tag inside Box 2
  ];
  for(const sp of tagSpecs)paintTagGray(gray,W,H,sp.code,sp);

  const baseline=detector.detect(gray,W,H);
  assert.equal(baseline.length,2);
  const baseById=new Map(baseline.map(m=>[m.id,m]));

  // Two 460x460 search ROIs (>= 52k px² triggers Stage-1 dec=2 search + Stage-3 tight dec=1 refinement).
  const resBoth=detector.detectFocusRois(gray,W,H,[
    [100,100,560,560],
    [600,100,1060,560],
  ]);
  assert.equal(resBoth.focusHit,true);
  assert.equal(resBoth.focusHitBoxes,2);
  assert.equal(resBoth.focusMissedBoxes,0);
  assert.equal(resBoth.markers.length,2);
  let maxErrPx=0;
  for(const m of resBoth.markers){
    const ref=baseById.get(m.id);
    assert.ok(ref);
    for(let i=0;i<4;i++){
      const err=Math.hypot(m.corners[i].x-ref.corners[i].x,m.corners[i].y-ref.corners[i].y);
      if(err>maxErrPx)maxErrPx=err;
    }
  }
  assert.ok(maxErrPx<=0.35,`expected <=0.35px corner delta vs dec=1, got ${maxErrPx.toFixed(4)}px`);

  // Partial miss telemetry: Box 1 hits #350 while Box 2 (bottom-right) is empty texture.
  const partial=detector.detectFocusRois(gray,W,H,[
    [100,100,560,560],
    [700,600,1160,1060],
  ]);
  assert.equal(partial.focusHit,true);
  assert.equal(partial.focusHitBoxes,1);
  assert.equal(partial.focusMissedBoxes,1);
  assert.equal(partial.markers.length,1);
  assert.equal(partial.markers[0].id,350);

  // Large-box (320x320) Stage-2 dec=1.5 ref=0 motion-blur rescue on a 9 px horizontally smeared cell=5 tag.
  const cell=5,ox=500,oy=400,blurPx=9,size=cell*8,qPad=cell*2;
  const sharp=new Float32Array(W*H).fill(180);
  for(let y=oy-qPad;y<oy+size+qPad;y++)for(let x=ox-qPad;x<ox+size+qPad;x++)sharp[y*W+x]=240;
  for(let r=0;r<8;r++){
    for(let c=0;c<8;c++){
      let val=20;
      if(r>0&&r<7&&c>0&&c<7){
        const bit=(r-1)*6+(c-1);
        val=Number((0x89ded4ea1n>>BigInt(35-bit))&1n)?240:20;
      }
      for(let dy=0;dy<cell;dy++)for(let dx=0;dx<cell;dx++)sharp[(oy+r*cell+dy)*W+(ox+c*cell+dx)]=val;
    }
  }
  const blurred=new Uint8Array(W*H).fill(180);
  const half=Math.floor(blurPx/2);
  for(let y=oy-qPad;y<oy+size+qPad;y++){
    for(let x=ox-qPad-blurPx;x<ox+size+qPad+blurPx;x++){
      let sum=0;
      for(let dx=0;dx<blurPx;dx++)sum+=sharp[y*W+Math.min(W-1,Math.max(0,x-half+dx))];
      blurred[y*W+x]=Math.round(sum/blurPx);
    }
  }
  // Confirm single-pass 320x320 dec=1, ref=1 misses blurPx=9 while two-stage Focus-ROI rescues #350!
  detector.setOptions({decimate:1,refineEdges:1});
  const cropBlur=new Uint8Array(320*320);
  for(let r=0;r<320;r++)cropBlur.set(blurred.subarray((oy-140+r)*W+(ox-140),(oy-140+r)*W+(ox+size+140)),r*320);
  assert.equal(detector.detect(cropBlur,320,320).length,0);
  const resBlur=detector.detectFocusRois(blurred,W,H,[[ox-140,oy-140,ox+size+140,oy+size+140]]);
  assert.equal(resBlur.focusHit,true);
  assert.equal(resBlur.focusHitBoxes,1);
  assert.equal(resBlur.focusMissedBoxes,0);
  assert.equal(resBlur.markers.length,1);
  assert.equal(resBlur.markers[0].id,350);
});
