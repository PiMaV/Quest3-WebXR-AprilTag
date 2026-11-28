import * as THREE from '../vendor/three/three.module.js';
import {
  createReferenceCamera,createCalibrationAssistant,createLAnchorController,deviceIdentity,
  createObservationBuffer,fitPlanarPose,solveSingleTagPose,requiredTagEdgeMm,product,inverse,cvToGL,meanRigid,rigidVector,
  xrExitPressed,nextExitHold,tagSurfaceMetrics,isPinchHandEligible,classifyCursorProximity,INDEX_TIP_HOVER_OFFSET_M,projectIndexSelectorPos,TAG_ACTION_PILL_DEFAULTS,createTagRestState,resetTagRestState,updateTagRestState,computeTagActionPillLayout,evaluateTagActionPills,computeHoverCardLayout,computePinPillPositions,HOVER_CALLOUT_DEFAULTS,
  viewerVelocityAt,resolveCameraInL,evaluateMotionGate,evaluateCarriedJumpGuard,CARRIED_MOTION_GATE_DEFAULTS,createDampedAnchorCorrection,
  buildHandSample,createHandHistory,createTagHandCoupler,createCoupledTrailBuffer,
  classifyFistPose,createFistToggleController,
  createPerfTracker,ADAPTIVE_QUALITY_MODES,createAdaptiveQualityController,
  projectPriorRois,fuseCommittedTrailSegment,TRAIL_GAP_DEFAULTS,shouldStartNewTrailSegment,smoothLineTail,
  TURBO_SPEED_DEFAULTS,turboColormap,speedToTurboColor,
  MOTION_WAKE_DEFAULTS,createMotionWakeBuffer,
  TRAIL_VERTEX_ROLE,TRAIL_MEASURE_STYLE_DEFAULTS,buildTrailMeasureFocus,evaluateEntryTrailMeasureStyle,analyzeTrailSegmentPasses,measureTrailSpan,measureTrailArcLength,pickPreferredMeasureCandidate,
  createTagMotionPredictor,createFocusScheduler,APP_VERSION_LABEL,
} from '../src/index.mjs';
import {
  createInstancedPool,createAxesBatch,createPathTubeMesh,createBadgeAtlas,createBillboardAtlasMesh,createOrientedAtlasMesh,
} from '../src/three.mjs';
import {associateAnchorCapture,auditAnchorObservation,createAnchorAudit} from '../src/anchor-audit.mjs';
import {createPerimeterFence,isPerimeterTag,perimeterRole,FENCE_HEIGHT_M} from '../src/perimeter-fence.mjs';
import {matToQuat,quatNormalize,quatSlerp,composeRigid,poseAt} from '../src/runtime.mjs';

const $=id=>document.getElementById(id),status=text=>{const el=$('status');if(el)el.textContent=`${APP_VERSION_LABEL} · ${text}`;};
if($('appVersion'))$('appVersion').textContent=APP_VERSION_LABEL;
const PERIMETER_ENABLED=true;
const EXTRA_COLORS=[0x38bdf8,0xf59e0b,0xa78bfa,0x34d399,0xfb7185,0xfacc15];
const urlParams=new URLSearchParams(location.search);
const aaToggleEl=$('aaToggle');
const devModeEl=$('devMode');
const qualityModeEl=$('qualityMode');
const longRangeEl=$('longRange');
const trailColorModeEl=$('trailColorMode');
const TRAIL_COLOR_MODES=Object.freeze(['id','speed']);
if(urlParams.has('aa')&&aaToggleEl)aaToggleEl.checked=urlParams.get('aa')!=='0';
if(urlParams.has('dev')&&devModeEl)devModeEl.checked=urlParams.get('dev')==='1';
let devModeEnabled=urlParams.has('dev')?urlParams.get('dev')==='1':Boolean(devModeEl?.checked);
let devHandCouplingEnabled=urlParams.get('handCouple')!=='0';
let devDisplayPredEnabled=urlParams.get('dispPred')!=='0';
const reqQuality=urlParams.get('quality');
if(reqQuality&&ADAPTIVE_QUALITY_MODES.includes(reqQuality)&&qualityModeEl)qualityModeEl.value=reqQuality;
if(urlParams.has('range')&&longRangeEl)longRangeEl.checked=urlParams.get('range')==='far'||urlParams.get('range')==='1';
const reqTrailColor=urlParams.get('trailColor');
if(reqTrailColor&&TRAIL_COLOR_MODES.includes(reqTrailColor)&&trailColorModeEl)trailColorModeEl.value=reqTrailColor;
const currentTrailColorMode=()=>{
  if(urlParams.has('trailColor')){
    const qv=urlParams.get('trailColor');
    if(TRAIL_COLOR_MODES.includes(qv))return qv;
  }
  const v=trailColorModeEl?.value;
  return TRAIL_COLOR_MODES.includes(v)?v:'id';
};
let trailColorMode=currentTrailColorMode();
const currentAaEnabled=()=>urlParams.has('aa')?urlParams.get('aa')!=='0':Boolean($('aaToggle')?.checked??true);
const currentQualityMode=()=>{
  const v=$('qualityMode')?.value;
  return ADAPTIVE_QUALITY_MODES.includes(v)?v:'auto';
};
const currentLongRangeEnabled=()=>urlParams.has('range')?(urlParams.get('range')==='far'||urlParams.get('range')==='1'):Boolean($('longRange')?.checked??false);
const adaptiveQuality=createAdaptiveQualityController({mode:currentQualityMode(),longRange:currentLongRangeEnabled()});
let appliedFoveation=null,lastQualityState=adaptiveQuality.state();
const _prevViewerPoseBuf=new Float32Array(16);
let hasPrevViewerPose=false,prevViewerPoseMs=0,smoothedHeadAngDegS=0,smoothedHeadLinMS=0;
let xrFrameHandler=null,renderer=null,rendererAa=null,rendererFbScale=null,shadersWarmed=false;
function currentFramebufferScale(){
  if(urlParams.has('fbScale')){
    const parsed=Number(urlParams.get('fbScale'));
    if(Number.isFinite(parsed))return Math.max(0.6,Math.min(1.25,parsed));
  }
  return currentQualityMode()==='crisp'?1.0:0.85;
}
function ensureRenderer(){
  const aa=currentAaEnabled();
  const fbScale=currentFramebufferScale();
  if(renderer&&rendererAa===aa&&rendererFbScale===fbScale)return renderer;
  let canvasEl=$('canvas');
  if(renderer){
    try{renderer.setAnimationLoop(null);renderer.dispose();}catch{}
    const nextCanvas=canvasEl.cloneNode(false);
    canvasEl.parentNode.replaceChild(nextCanvas,canvasEl);
    canvasEl=nextCanvas;
  }
  renderer=new THREE.WebGLRenderer({canvas:canvasEl,alpha:true,antialias:aa,powerPreference:'high-performance'});
  renderer.debug.checkShaderErrors=false;
  renderer.xr.enabled=true;
  renderer.xr.setReferenceSpaceType('local');
  renderer.xr.setFramebufferScaleFactor(fbScale);
  renderer.setSize(innerWidth,innerHeight);
  rendererAa=aa;rendererFbScale=fbScale;shadersWarmed=false;
  if(xrFrameHandler)renderer.setAnimationLoop(xrFrameHandler);
  return renderer;
}
function warmupSceneShaders(r,scn,cam){
  if(!r||shadersWarmed)return;
  const targets=[root,hudRig,leaveConfirmMesh,fenceGroup,inspectRoot,portalRoot,trailGroup,handLabelsGroup,toastSprite,warpBannerSprite];
  const prevVis=targets.map(o=>o.visible);
  for(const o of targets)o.visible=true;
  const tmpGeo=new THREE.BufferGeometry();
  tmpGeo.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0.01,0,0,0,0.01,0],3));
  const tmpInstMat=new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.72,side:THREE.DoubleSide,depthWrite:false});
  const tmpInst=new THREE.InstancedMesh(tmpGeo,tmpInstMat,1);
  tmpInst.setColorAt(0,new THREE.Color(0xffffff));
  tmpInst.count=1;
  const tmpAxes=new THREE.AxesHelper(0.05);
  const tmpLineGeo=new THREE.BufferGeometry();
  tmpLineGeo.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0.01,0,0],3));
  const tmpLineMat=new THREE.LineBasicMaterial({color:0xffffff,transparent:true,opacity:0.65});
  const tmpLine=new THREE.Line(tmpLineGeo,tmpLineMat);
  const tmpTrailGeo=new THREE.BufferGeometry();
  tmpTrailGeo.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0.01,0,0,0,0.01,0],3));
  tmpTrailGeo.setAttribute('color',new THREE.Float32BufferAttribute([1,1,1,1,1,1,1,1,1],3));
  tmpTrailGeo.setAttribute('aDash',new THREE.Float32BufferAttribute([0,1,0.01,1,0.02,1],2));
  tmpTrailGeo.setAttribute('aFade',new THREE.Float32BufferAttribute([1,1,1],1));
  const tmpTrail=new THREE.Mesh(tmpTrailGeo,trailLineMat);
  const tmpTrailDim=new THREE.Mesh(tmpTrailGeo,trailLineDimMat);
  const tmpWakeGeo=new THREE.BufferGeometry();
  tmpWakeGeo.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0.01,0,0,0,0.01,0],3));
  tmpWakeGeo.setAttribute('color',new THREE.Float32BufferAttribute([1,1,1,1,1,1,1,1,1],3));
  tmpWakeGeo.setAttribute('aFade',new THREE.Float32BufferAttribute([0,0.5,1],1));
  const tmpWake=new THREE.Mesh(tmpWakeGeo,wakeLineMat);
  const tmpSpriteMat=new THREE.SpriteMaterial({color:0xffffff,transparent:true,depthTest:false,depthWrite:false});
  const tmpSprite=new THREE.Sprite(tmpSpriteMat);
  scn.add(tmpInst,tmpAxes,tmpLine,tmpTrail,tmpTrailDim,tmpWake,tmpSprite);
  const prevFrameCount=frameBatch.mesh.count;
  const prevAlertGlowCount=alertGlowBatch.mesh.count;
  if(prevFrameCount===0)frameBatch.mesh.count=1;
  if(prevAlertGlowCount===0)alertGlowBatch.mesh.count=1;
  try{
    r.compile(scn,cam);
    shadersWarmed=true;
  }catch{}
  frameBatch.mesh.count=prevFrameCount;
  alertGlowBatch.mesh.count=prevAlertGlowCount;
  scn.remove(tmpInst,tmpAxes,tmpLine,tmpTrail,tmpTrailDim,tmpWake,tmpSprite);
  tmpInst.dispose();tmpGeo.dispose();tmpInstMat.dispose();
  tmpAxes.geometry.dispose();tmpAxes.material.dispose();
  tmpLineGeo.dispose();tmpLineMat.dispose();
  tmpTrailGeo.dispose();
  tmpWakeGeo.dispose();
  tmpSpriteMat.dispose();
  for(let i=0;i<targets.length;i++)targets[i].visible=prevVis[i];
}
ensureRenderer();
if(qualityModeEl){
  qualityModeEl.onchange=()=>{
    const m=currentQualityMode();
    adaptiveQuality.setMode(m);
    lastQualityState=adaptiveQuality.state();
  };
}
if(longRangeEl){
  longRangeEl.onchange=()=>{
    adaptiveQuality.setLongRange(currentLongRangeEnabled());
    lastQualityState=adaptiveQuality.state();
  };
}
if(devModeEl){
  devModeEl.onchange=()=>{
    devModeEnabled=Boolean(devModeEl.checked);
  };
}
const perfTracker=createPerfTracker();

const scene=new THREE.Scene(),camera3d=new THREE.PerspectiveCamera();
camera3d.near=0.05;camera3d.far=40;camera3d.updateProjectionMatrix();

const root=new THREE.Group();root.matrixAutoUpdate=false;root.visible=false;scene.add(root);
const extraRoot=new THREE.Group();root.add(extraRoot);

// World-space calibration breadcrumb trail (head poses where views were accepted).
const trailGroup=new THREE.Group();trailGroup.visible=false;scene.add(trailGroup);
const trailGeo=new THREE.SphereGeometry(0.011,10,10);
const trailMatScan=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.85});
const trailMatVerify=new THREE.MeshBasicMaterial({color:0x4ade80,transparent:true,opacity:0.85});

// Split upper HUD panels (sharper 1.5x DPR, larger physical size, Quest-style bottom grab-bars with localStorage persistence)
// + centered upper info bar + subtle calibration reticle.
const HUD_DPR=1.5;
const HUD_LAYOUT_KEY='webxr-apriltag.hudLayout.v1';
const HUD_DEFAULT_LAYOUT={
  left:{pos:[-0.42,0.16,-1.02]},
  right:{pos:[0.42,0.16,-1.02]},
  top:{pos:[0.0,0.22,-0.98]},
};
function loadHudLayout(){
  try{
    const raw=window.localStorage?.getItem(HUD_LAYOUT_KEY);
    if(!raw)return structuredClone(HUD_DEFAULT_LAYOUT);
    const parsed=JSON.parse(raw);
    const out=structuredClone(HUD_DEFAULT_LAYOUT);
    for(const k of ['left','right','top']){
      const p=parsed?.[k]?.pos;
      if(Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)){
        out[k].pos=[
          Math.max(-0.85,Math.min(0.85,p[0])),
          Math.max(-0.45,Math.min(0.55,p[1])),
          Math.max(-1.60,Math.min(-0.45,p[2])),
        ];
      }
    }
    return out;
  }catch{return structuredClone(HUD_DEFAULT_LAYOUT);}
}
function saveHudLayout(layout){
  try{window.localStorage?.setItem(HUD_LAYOUT_KEY,JSON.stringify(layout));}catch{}
  console.info('[webxr-apriltag] saved HUD layout:',JSON.stringify(layout));
}
let hudLayout=loadHudLayout();

function makeUiTexture(canvas){
  const tex=new THREE.CanvasTexture(canvas);
  tex.generateMipmaps=false;
  tex.minFilter=THREE.LinearFilter;
  tex.magFilter=THREE.LinearFilter;
  return tex;
}

const hudRig=new THREE.Group();hudRig.visible=false;scene.add(hudRig);

function makeHudGrabHandle(panelH){
  const mat=new THREE.MeshBasicMaterial({color:0x94a3b8,transparent:true,opacity:0.82,depthTest:false,depthWrite:false});
  const mesh=new THREE.Mesh(new THREE.PlaneGeometry(0.14,0.015),mat);
  mesh.position.set(0,-panelH/2-0.018,0.002);
  mesh.renderOrder=1000;
  // Generous invisible raycast hit plane so far ray-pinch grabbing from 1 m away feels effortless.
  const hitMesh=new THREE.Mesh(
    new THREE.PlaneGeometry(0.24,0.065),
    new THREE.MeshBasicMaterial({transparent:true,opacity:0,depthWrite:false,depthTest:false}),
  );
  mesh.add(hitMesh);
  return {mesh,hitMesh,mat};
}
const hudRayDotMat=new THREE.MeshBasicMaterial({color:0xfde047,transparent:true,opacity:0.95,depthTest:false,depthWrite:false});
const hudRayDot=new THREE.Mesh(new THREE.SphereGeometry(0.007,12,12),hudRayDotMat);
hudRayDot.visible=false;hudRayDot.renderOrder=1003;scene.add(hudRayDot);
const _hudRaycaster=new THREE.Raycaster();
const _hudRayOrigin=new THREE.Vector3();
const _hudRayDir=new THREE.Vector3();
const _hudRayQuat=new THREE.Quaternion();
const _hudRayMat4=new THREE.Matrix4();
function orientHudPanel(mesh){
  mesh.lookAt(0,0,0);
}

const leftCanvas=document.createElement('canvas');leftCanvas.width=Math.round(512*HUD_DPR);leftCanvas.height=Math.round(420*HUD_DPR);
const leftTex=makeUiTexture(leftCanvas);
const hudLeft=new THREE.Mesh(
  new THREE.PlaneGeometry(0.38,0.312),
  new THREE.MeshBasicMaterial({map:leftTex,transparent:true,depthTest:false,depthWrite:false}),
);
hudLeft.position.set(...hudLayout.left.pos);orientHudPanel(hudLeft);hudLeft.renderOrder=999;
const hudLeftGrab=makeHudGrabHandle(0.312);hudLeft.add(hudLeftGrab.mesh);
hudRig.add(hudLeft);

const rightCanvas=document.createElement('canvas');rightCanvas.width=Math.round(512*HUD_DPR);rightCanvas.height=Math.round(420*HUD_DPR);
const rightTex=makeUiTexture(rightCanvas);
const hudRight=new THREE.Mesh(
  new THREE.PlaneGeometry(0.38,0.312),
  new THREE.MeshBasicMaterial({map:rightTex,transparent:true,depthTest:false,depthWrite:false}),
);
hudRight.position.set(...hudLayout.right.pos);orientHudPanel(hudRight);hudRight.renderOrder=999;
const hudRightGrab=makeHudGrabHandle(0.312);hudRight.add(hudRightGrab.mesh);
hudRig.add(hudRight);

// Centered upper info bar (sits gently above the center sweet-spot, crisp & always readable when HUD is visible).
const topBarCanvas=document.createElement('canvas');topBarCanvas.width=840;topBarCanvas.height=120;
const topBarTex=makeUiTexture(topBarCanvas);
const hudTopBar=new THREE.Mesh(
  new THREE.PlaneGeometry(0.42,0.060),
  new THREE.MeshBasicMaterial({map:topBarTex,transparent:true,depthTest:false,depthWrite:false}),
);
hudTopBar.position.set(...hudLayout.top.pos);orientHudPanel(hudTopBar);hudTopBar.renderOrder=999;
const hudTopGrab=makeHudGrabHandle(0.060);hudTopBar.add(hudTopGrab.mesh);
hudRig.add(hudTopBar);


// ── Lighting-quality telemetry (rendered inside right HUD panel when HUD is visible) ─────────
// Metrics are computed in the detector worker at ~5 Hz (src/light-quality.mjs) and arrive via
// sample.light. Recording samples here touches zero DOM/GPU resources while the HUD is hidden.
const LIGHT_HIST_LEN=60;                       // 60 samples * 200 ms ≈ last 12 s
const LIGHT_COLORS={good:'#22c55e',warn:'#facc15',bad:'#ef4444'};
const LIGHT_BADGE_COLORS={good:'#16a34a',warn:'#ca8a04',bad:'#dc2626'};
const LIGHT_LEVEL_LABELS={good:'GOOD',warn:'WARN',bad:'POOR'};
const LIGHT_FLAG_TEXT={
  backlight:'BACKLIGHT',
  dark:'TOO DARK',
  overexposed:'OVEREXPOSED',
  'ae-hunting':'AE HUNTING',
  'tag-low-contrast':'LOW TAG CONTRAST',
  'tag-clipped':'TAG CLIPPED',
};
const lightScoreHistory=[];                    // {score,level}
let lightLast=null,lightLastAt=0;

function pushLightSample(light,now){
  lightLast=light;lightLastAt=now;
  lightScoreHistory.push({score:light.score,level:light.level});
  if(lightScoreHistory.length>LIGHT_HIST_LEN)lightScoreHistory.shift();
}

const hudDraggablePanels=[
  {id:'left',mesh:hudLeft,grab:hudLeftGrab},
  {id:'right',mesh:hudRight,grab:hudRightGrab},
  {id:'top',mesh:hudTopBar,grab:hudTopGrab},
];
let hudDragState=null; // {panel, probeId, offsetLocal:THREE.Vector3}

const reticle=new THREE.Mesh(
  new THREE.RingGeometry(0.0055,0.009,24),
  new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.85,side:THREE.DoubleSide,depthTest:false,depthWrite:false}),
);
reticle.position.set(0,0,-1.0);reticle.renderOrder=998;
hudRig.add(reticle);

// Bottom calibration cue strip on HUD + World-Anchored 3D Spatial Halo around the physical L-angle.
const orbitGuideGroup=new THREE.Group();
orbitGuideGroup.visible=false;
hudRig.add(orbitGuideGroup);
const orbitCueCanvas=document.createElement('canvas');orbitCueCanvas.width=440;orbitCueCanvas.height=124;
const orbitCueTex=makeUiTexture(orbitCueCanvas);
const orbitCueSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:orbitCueTex,transparent:true,depthTest:false,depthWrite:false}));
orbitCueSprite.scale.set(0.30,0.084,1);
orbitCueSprite.position.set(0,-0.115,-1.0);
orbitCueSprite.renderOrder=998;
orbitGuideGroup.add(orbitCueSprite);

const HALO_RING_R=0.137;
const HALO_CW_ORDER=[2,1,0,7,6,5,4,3];
const spatialHaloGroup=new THREE.Group();
spatialHaloGroup.visible=false;
scene.add(spatialHaloGroup);
const haloInnerMat=new THREE.MeshBasicMaterial({color:0x94a3b8,transparent:true,opacity:0.28,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const haloInnerRing=new THREE.Mesh(new THREE.RingGeometry(0.118,0.122,64),haloInnerMat);
haloInnerRing.renderOrder=990;
spatialHaloGroup.add(haloInnerRing);
const haloSectorMats=[];
for(let i=0;i<8;i++){
  const mat=new THREE.MeshBasicMaterial({color:0x64748b,transparent:true,opacity:0.45,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
  const seg=new THREE.Mesh(new THREE.RingGeometry(0.126,0.148,10,1,i*(Math.PI/4)-(Math.PI/8-0.035),Math.PI/4-0.07),mat);
  seg.renderOrder=991;
  spatialHaloGroup.add(seg);
  haloSectorMats.push(mat);
}
const haloCursorGroup=new THREE.Group();
haloCursorGroup.position.set(0,0,0.003);
spatialHaloGroup.add(haloCursorGroup);
const haloCursorDotMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.95,depthTest:false,depthWrite:false});
const haloCursorDot=new THREE.Mesh(new THREE.SphereGeometry(0.0105,16,12),haloCursorDotMat);
haloCursorDot.renderOrder=993;
haloCursorGroup.add(haloCursorDot);
const haloCursorRingMat=new THREE.MeshBasicMaterial({color:0xe0f2fe,transparent:true,opacity:0.9,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const haloCursorRing=new THREE.Mesh(new THREE.RingGeometry(0.014,0.019,24),haloCursorRingMat);
haloCursorRing.renderOrder=994;
haloCursorGroup.add(haloCursorRing);

// Small 3D visual confirmation ring at left fingertips for hand-only Scan / HUD / Clear pinch holds.
const handGestureMat=new THREE.MeshBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.95,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const handGestureRing=new THREE.Mesh(new THREE.RingGeometry(0.009,0.016,28),handGestureMat);
handGestureRing.visible=false;handGestureRing.renderOrder=999;
scene.add(handGestureRing);

// Palm-up floating fingertip shortcut pills on Left (Scan / Pins / Clear All) and Right (HUD) hands.
const handLabelsGroup=new THREE.Group();
handLabelsGroup.visible=false;
scene.add(handLabelsGroup);
function makeFingerPill(parent=handLabelsGroup){
  const canvas=document.createElement('canvas');canvas.width=210;canvas.height=64;
  const tex=makeUiTexture(canvas);
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true,depthTest:false,depthWrite:false}));
  sprite.scale.set(0.047,0.0145,1);sprite.renderOrder=998;
  parent.add(sprite);
  return {sprite,canvas,tex,lastKey:''};
}
function drawFingerPill(pill,text,accent,hot=false){
  const key=`${text}:${accent}:${hot?1:0}`;
  if(key===pill.lastKey)return;
  pill.lastKey=key;
  const ctx=pill.canvas.getContext('2d');
  ctx.clearRect(0,0,210,64);
  ctx.fillStyle=hot?'rgba(15,23,42,0.96)':'rgba(8,14,24,0.86)';
  ctx.beginPath();ctx.roundRect(4,4,202,56,28);ctx.fill();
  ctx.strokeStyle=hot?'#fef08a':accent;ctx.lineWidth=hot?5:3.5;ctx.stroke();
  ctx.fillStyle=hot?'#ffffff':accent;ctx.font=text.length>10?'bold 19px system-ui,sans-serif':'bold 24px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(text,105,text.length>10?39:41,188);ctx.textAlign='left';
  pill.tex.needsUpdate=true;
}
const fingerBadgeIdx=makeFingerPill();
const fingerBadgeMid=makeFingerPill();
const fingerBadgeRing=makeFingerPill();
const fingerBadgePinky=makeFingerPill();
const fingerBadgeRightMid=makeFingerPill();
const fingerBadgeRightRing=makeFingerPill();
const fingerBadgeRightPinky=makeFingerPill();

// Subtle inner-wrist dots (two-handed instant open for the gated wrist menu) + 1.2 s gaze progress circle + left-middle SCAN recording dot.
const _wristDotGeo=new THREE.SphereGeometry(0.008,14,14);
const leftWristDotMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.55,depthTest:false,depthWrite:false});
const leftWristDot=new THREE.Mesh(_wristDotGeo,leftWristDotMat);
leftWristDot.visible=false;leftWristDot.renderOrder=997;scene.add(leftWristDot);
const rightWristDotMat=new THREE.MeshBasicMaterial({color:0xc084fc,transparent:true,opacity:0.55,depthTest:false,depthWrite:false});
const rightWristDot=new THREE.Mesh(_wristDotGeo,rightWristDotMat);
rightWristDot.visible=false;rightWristDot.renderOrder=997;scene.add(rightWristDot);

const wristGazeCanvas=document.createElement('canvas');wristGazeCanvas.width=96;wristGazeCanvas.height=96;
const wristGazeTex=makeUiTexture(wristGazeCanvas);
const wristGazeSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:wristGazeTex,transparent:true,depthTest:false,depthWrite:false}));
wristGazeSprite.scale.set(0.038,0.038,1);wristGazeSprite.visible=false;wristGazeSprite.renderOrder=998;
scene.add(wristGazeSprite);
let _wristGazeKey='';
function drawWristGazeCircle(progress,accent){
  const q=Math.max(0,Math.min(36,Math.round(progress*36)));
  const key=`${q}:${accent}`;
  if(key===_wristGazeKey)return;
  _wristGazeKey=key;
  const ctx=wristGazeCanvas.getContext('2d');
  ctx.clearRect(0,0,96,96);
  ctx.strokeStyle='rgba(15,23,42,0.72)';ctx.lineWidth=10;
  ctx.beginPath();ctx.arc(48,48,34,0,Math.PI*2);ctx.stroke();
  ctx.strokeStyle=accent;ctx.lineWidth=8;ctx.lineCap='round';
  ctx.beginPath();ctx.arc(48,48,34,-Math.PI/2,-Math.PI/2+Math.max(0.02,progress)*Math.PI*2);ctx.stroke();
  wristGazeTex.needsUpdate=true;
}

const _fingerCueGeo=new THREE.SphereGeometry(0.0058,14,14);
const leftScanRecDotMat=new THREE.MeshBasicMaterial({color:0xef4444,transparent:true,opacity:0.9,depthTest:false,depthWrite:false});
const leftScanRecDot=new THREE.Mesh(_fingerCueGeo,leftScanRecDotMat);
leftScanRecDot.visible=false;leftScanRecDot.renderOrder=997;scene.add(leftScanRecDot);

// Clear 3D progress/confirmation badge floating above the left fist during the 0.55 s mode-switch hold.
const fistProgressCanvas=document.createElement('canvas');fistProgressCanvas.width=240;fistProgressCanvas.height=128;
const fistProgressTex=makeUiTexture(fistProgressCanvas);
const fistProgressSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:fistProgressTex,transparent:true,depthTest:false,depthWrite:false}));
fistProgressSprite.scale.set(0.095,0.051,1);
fistProgressSprite.visible=false;fistProgressSprite.renderOrder=999;
scene.add(fistProgressSprite);
let _fistProgressKey='';
function drawFistProgressBadge(progress,targetOrOpts,done=false){
  const opts=(typeof targetOrOpts==='object'&&targetOrOpts!==null)
    ?targetOrOpts
    :{
      label:targetOrOpts?'SCAN':'MEASURE',
      accent:targetOrOpts?'#ef4444':'#4ade80',
      symbol:targetOrOpts?'●':'■',
      done,
    };
  const isDone=Boolean(opts.done??done);
  const label=opts.label||'SCAN';
  const accent=opts.accent||'#ef4444';
  const symbol=opts.symbol||'●';
  const bucket=isDone?'done':Math.round(Math.max(0,Math.min(1,progress))*36);
  const key=`${bucket}:${label}:${accent}:${symbol}:${isDone?1:0}`;
  if(key===_fistProgressKey)return;
  _fistProgressKey=key;
  const ctx=fistProgressCanvas.getContext('2d');
  ctx.clearRect(0,0,240,128);
  ctx.fillStyle=isDone?'rgba(15,23,42,0.96)':'rgba(8,14,24,0.90)';
  ctx.beginPath();ctx.roundRect(4,8,232,112,24);ctx.fill();
  ctx.strokeStyle=isDone?'#ffffff':accent;ctx.lineWidth=isDone?5:4;ctx.stroke();
  ctx.strokeStyle='rgba(148,163,184,0.32)';ctx.lineWidth=8;
  ctx.beginPath();ctx.arc(56,64,32,0,Math.PI*2);ctx.stroke();
  ctx.strokeStyle=accent;ctx.lineWidth=8;ctx.lineCap='round';
  ctx.beginPath();ctx.arc(56,64,32,-Math.PI/2,-Math.PI/2+Math.max(0.04,isDone?1:progress)*Math.PI*2);ctx.stroke();
  ctx.fillStyle=isDone?'#ffffff':accent;
  ctx.font='bold 30px system-ui,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillText(isDone?'✓':symbol,56,65);
  ctx.fillStyle=isDone?'#ffffff':accent;
  ctx.font=label.length>6?'bold 24px system-ui,sans-serif':'bold 28px system-ui,sans-serif';
  ctx.fillText(label,160,66,126);
  ctx.textAlign='left';ctx.textBaseline='alphabetic';
  fistProgressTex.needsUpdate=true;
}

// Lightweight WebAudio confirmation ticks for mode switches, time-copies, and pins (best-effort, zero dependencies).
let _audioCtx=null;
function playModeTick(kind='scan'){
  try{
    const AC=window.AudioContext||window.webkitAudioContext;
    if(!AC)return;
    if(!_audioCtx)_audioCtx=new AC();
    if(_audioCtx.state==='suspended')_audioCtx.resume().catch(()=>{});
    const t=_audioCtx.currentTime;
    const osc=_audioCtx.createOscillator(),gain=_audioCtx.createGain();
    osc.type='sine';
    if(kind==='breach-out'){
      osc.type='triangle';
      osc.frequency.setValueAtTime(210,t);
      osc.frequency.exponentialRampToValueAtTime(460,t+0.045);
      osc.frequency.exponentialRampToValueAtTime(240,t+0.18);
      gain.gain.setValueAtTime(0.0001,t);
      gain.gain.exponentialRampToValueAtTime(0.085,t+0.012);
      gain.gain.exponentialRampToValueAtTime(0.0001,t+0.19);
      osc.connect(gain);gain.connect(_audioCtx.destination);
      osc.start(t);osc.stop(t+0.20);
      const osc2=_audioCtx.createOscillator(),gain2=_audioCtx.createGain();
      osc2.type='sine';
      osc2.frequency.setValueAtTime(880,t);
      osc2.frequency.exponentialRampToValueAtTime(620,t+0.13);
      gain2.gain.setValueAtTime(0.0001,t);
      gain2.gain.exponentialRampToValueAtTime(0.025,t+0.010);
      gain2.gain.exponentialRampToValueAtTime(0.0001,t+0.13);
      osc2.connect(gain2);gain2.connect(_audioCtx.destination);
      osc2.start(t);osc2.stop(t+0.14);
      return;
    }
    if(kind==='breach-in'){
      osc.frequency.setValueAtTime(420,t);
      osc.frequency.exponentialRampToValueAtTime(660,t+0.11);
      gain.gain.setValueAtTime(0.0001,t);
      gain.gain.exponentialRampToValueAtTime(0.05,t+0.012);
      gain.gain.exponentialRampToValueAtTime(0.0001,t+0.11);
      osc.connect(gain);gain.connect(_audioCtx.destination);
      osc.start(t);osc.stop(t+0.12);
      return;
    }
    if(kind==='scan'){osc.frequency.setValueAtTime(620,t);osc.frequency.exponentialRampToValueAtTime(880,t+0.075);}
    else if(kind==='measure'){osc.frequency.setValueAtTime(560,t);osc.frequency.exponentialRampToValueAtTime(390,t+0.085);}
    else if(kind==='timecopy'){osc.frequency.setValueAtTime(740,t);osc.frequency.exponentialRampToValueAtTime(1040,t+0.07);}
    else if(kind==='pin'){osc.frequency.setValueAtTime(780,t);osc.frequency.exponentialRampToValueAtTime(920,t+0.055);}
    else{osc.frequency.setValueAtTime(480,t);osc.frequency.exponentialRampToValueAtTime(360,t+0.055);}
    gain.gain.setValueAtTime(0.0001,t);
    gain.gain.exponentialRampToValueAtTime(0.09,t+0.012);
    gain.gain.exponentialRampToValueAtTime(0.0001,t+0.11);
    osc.connect(gain);gain.connect(_audioCtx.destination);
    osc.start(t);osc.stop(t+0.12);
  }catch{}
}

// Mini, soft looping siren while any tag is in alarm (outside perimeter). Quiet sine wobble, fades in/out.
let _sirenOsc=null,_sirenGain=null,_sirenLfo=null;
function updateAlarmSiren(on){
  try{
    if(on){
      if(_sirenOsc)return;
      const AC=window.AudioContext||window.webkitAudioContext;
      if(!AC)return;
      if(!_audioCtx)_audioCtx=new AC();
      if(_audioCtx.state==='suspended')_audioCtx.resume().catch(()=>{});
      const t=_audioCtx.currentTime;
      const osc=_audioCtx.createOscillator(),gain=_audioCtx.createGain();
      const lfo=_audioCtx.createOscillator(),lfoGain=_audioCtx.createGain();
      osc.type='sine';osc.frequency.value=720;
      lfo.type='sine';lfo.frequency.value=1.6;lfoGain.gain.value=180;
      lfo.connect(lfoGain);lfoGain.connect(osc.frequency);
      gain.gain.setValueAtTime(0.0001,t);
      gain.gain.exponentialRampToValueAtTime(0.018,t+0.25);
      osc.connect(gain);gain.connect(_audioCtx.destination);
      osc.start(t);lfo.start(t);
      _sirenOsc=osc;_sirenGain=gain;_sirenLfo=lfo;
    }else if(_sirenOsc){
      const t=_audioCtx.currentTime,o=_sirenOsc,l=_sirenLfo;
      _sirenGain.gain.cancelScheduledValues(t);
      _sirenGain.gain.setValueAtTime(Math.max(0.0001,_sirenGain.gain.value),t);
      _sirenGain.gain.exponentialRampToValueAtTime(0.0001,t+0.2);
      o.stop(t+0.25);l.stop(t+0.25);
      _sirenOsc=_sirenGain=_sirenLfo=null;
    }
  }catch{}
}

// Radial loading ring + checkmark burst shown when pinching a waypoint to pin/unpin its box.
const pinLoadCanvas=document.createElement('canvas');pinLoadCanvas.width=128;pinLoadCanvas.height=128;
const pinLoadTex=makeUiTexture(pinLoadCanvas);
const pinLoadMat=new THREE.SpriteMaterial({map:pinLoadTex,transparent:true,depthTest:false,depthWrite:false});
const pinLoadSprite=new THREE.Sprite(pinLoadMat);
pinLoadSprite.scale.set(0.044,0.044,1);
pinLoadSprite.visible=false;pinLoadSprite.renderOrder=999;
scene.add(pinLoadSprite);
function drawPinIconPath(ctx,cx,cy,size,fillStyle,holeFillStyle){
  const r=size*0.31;
  const headY=cy-size*0.14;
  const tipY=cy+size*0.50;
  const d=tipY-headY;
  const phi=Math.asin(Math.min(0.95,r/d));
  const aL=Math.PI-phi;
  const aR=Math.PI*2+phi;
  ctx.beginPath();
  ctx.moveTo(cx,tipY);
  ctx.lineTo(cx+r*Math.cos(aL),headY+r*Math.sin(aL));
  ctx.arc(cx,headY,r,aL,aR,false);
  ctx.closePath();
  ctx.fillStyle=fillStyle;
  ctx.fill();
  ctx.beginPath();
  ctx.arc(cx,headY,r*0.40,0,Math.PI*2);
  ctx.fillStyle=holeFillStyle;
  ctx.fill();
}

let lastPinLoadKey='';
function drawPinLoader(progress,color,symbol,done=false){
  const bucket=done?'done':Math.round(progress*36);
  const key=`${bucket}:${color}:${symbol}`;
  if(key===lastPinLoadKey)return;
  lastPinLoadKey=key;
  const ctx=pinLoadCanvas.getContext('2d');
  ctx.clearRect(0,0,128,128);
  ctx.fillStyle=done?'rgba(15,23,42,0.88)':'rgba(8,14,24,0.48)';
  ctx.beginPath();ctx.arc(64,64,54,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='rgba(148,163,184,0.35)';ctx.lineWidth=10;
  ctx.beginPath();ctx.arc(64,64,44,0,Math.PI*2);ctx.stroke();
  ctx.strokeStyle=color;ctx.lineWidth=11;ctx.lineCap='round';
  ctx.beginPath();
  ctx.arc(64,64,44,-Math.PI/2,-Math.PI/2+Math.max(0.04,done?1:progress)*Math.PI*2);
  ctx.stroke();
  if(symbol==='pin'){
    drawPinIconPath(ctx,64,65,42,done?'#ffffff':color,done?'rgba(15,23,42,0.92)':'rgba(8,14,24,0.75)');
  }else{
    ctx.fillStyle=done?'#ffffff':color;
    ctx.font=done?'bold 48px system-ui,sans-serif':'bold 38px system-ui,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(symbol,64,66);
    ctx.textAlign='left';ctx.textBaseline='alphabetic';
  }
  pinLoadTex.needsUpdate=true;
}

// Top-center status toast (always visible for 2.2 s on Scan / HUD / Clear actions, even if HUD panels are hidden).
const toastCanvas=document.createElement('canvas');toastCanvas.width=420;toastCanvas.height=72;
const toastTex=makeUiTexture(toastCanvas);
const toastSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:toastTex,transparent:true,depthTest:false,depthWrite:false}));
toastSprite.scale.set(0.28,0.048,1);
toastSprite.position.set(0,0.22,-1.05);
toastSprite.visible=false;toastSprite.renderOrder=1000;
hudRig.add(toastSprite);
let toastUntilMs=0;
let centerConfirmUntilMs=0;
function showCenterConfirm(title,subtitle,accent='#4ade80',durationMs=1900){
  drawWarpBanner(title,subtitle,accent,1);
  centerConfirmUntilMs=performance.now()+durationMs;
}

function showHudToast(text,accent='#22d3ee',durationMs=2200){
  const ctx=toastCanvas.getContext('2d');
  ctx.clearRect(0,0,420,72);
  ctx.fillStyle='rgba(8,14,24,0.92)';
  ctx.beginPath();ctx.roundRect(4,4,412,64,14);ctx.fill();
  ctx.strokeStyle=accent;ctx.lineWidth=3.5;ctx.stroke();
  ctx.fillStyle=accent;ctx.font='bold 24px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(text,210,45);ctx.textAlign='left';
  toastTex.needsUpdate=true;
  toastUntilMs=performance.now()+durationMs;
  toastSprite.visible=true;
}

// Center-view warp transition banner (shown during spatial portal jumps, even when HUD panels are hidden).
const warpCanvas=document.createElement('canvas');warpCanvas.width=560;warpCanvas.height=148;
const warpTex=makeUiTexture(warpCanvas);
const warpBannerSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:warpTex,transparent:true,depthTest:false,depthWrite:false}));
warpBannerSprite.scale.set(0.42,0.111,1);
warpBannerSprite.position.set(0,0.03,-0.92);
warpBannerSprite.visible=false;warpBannerSprite.renderOrder=1001;
hudRig.add(warpBannerSprite);

function drawWarpBanner(title,subtitle,accent,progress){
  const ctx=warpCanvas.getContext('2d');
  ctx.clearRect(0,0,560,148);
  ctx.fillStyle='rgba(7,11,22,0.94)';
  ctx.beginPath();ctx.roundRect(6,6,548,136,18);ctx.fill();
  ctx.strokeStyle=accent;ctx.lineWidth=4.5;ctx.stroke();
  ctx.fillStyle=accent;ctx.font='bold 34px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(title,280,58,512);
  ctx.fillStyle='#e2e8f0';ctx.font='bold 20px ui-monospace,monospace';
  ctx.fillText(subtitle,280,94,512);
  ctx.fillStyle='rgba(30,41,59,0.95)';
  ctx.beginPath();ctx.roundRect(46,112,468,14,7);ctx.fill();
  ctx.fillStyle=accent;
  ctx.beginPath();ctx.roundRect(46,112,Math.max(14,Math.round(468*Math.max(0,Math.min(1,progress)))),14,7);ctx.fill();
  ctx.textAlign='left';
  warpTex.needsUpdate=true;
  warpBannerSprite.visible=true;
}

let session=null,space=null,floorSpace=null,epoch=0,sessionCount=0,sessionId=null,history=[],assistant=null,profile=null,pending=false;
let hold={sinceMs:null,progress:0,fire:false},lastHud=0,lastStabilitySample=0,extraScanActive=false,recordAllActive=false,reanchorConfirmUntil=0,hudPanelsVisible=false;
const isEntryRecording=entry=>Boolean(entry&&(entry.recording||entry.autoRecording));
let hudLeftKey='',hudTopKey='',hudSlot=0,lastRightHud=0,hudGrabResetDone=true;
function resetHudCache(){hudLeftKey='';hudTopKey='';lastHud=0;lastRightHud=0;hudSlot=0;}
const anchorDriftCorrection=createDampedAnchorCorrection();
let lastShownUpgradeReason='';
function handleCalibrationStatus(s){
  status(`${s.phase} · ${s.count}/${s.required} · ${s.message}`);
  if(s.phase==='scanning'&&s.upgradeReason&&s.upgradeReason!==lastShownUpgradeReason){
    lastShownUpgradeReason=s.upgradeReason;
    const msg=s.upgradeReason==='solver-upgraded'
      ?'↺ KALIBRIERUNG AKTUALISIERT (v2) · BITTE KURZ SCANNEN'
      :(s.upgradeReason==='sanity-check'
        ?'↺ PROFIL-CHECK FEHLGESCHLAGEN · NEU KALIBRIEREN'
        :'↺ KAMERAMODELL AKTUALISIERT · BITTE KURZ SCANNEN');
    showHudToast(msg,'#38bdf8',3200);
    pushDiagEvent({type:'calibration-upgrade-rescan',reason:s.upgradeReason,atPerformanceMs:performance.now()});
  }
}
let axHoldSince=null,axLongFired=false;
const leftFistToggle=createFistToggleController({holdMs:850,deadzoneMs:220,cooldownMs:1400,maxSpeedMps:0.14});
const rightFistToggle=createFistToggleController({holdMs:800,deadzoneMs:220,cooldownMs:1400,maxSpeedMps:0.14});
let leftFistFlashUntil=0,leftFistFlashPos=null,leftFistFlashToScan=false;
let rightFistFlashUntil=0,rightFistFlashPos=null,rightFistFlashToRecAll=false;
let leftMidPinchSince=null,leftRingPinchSince=null,leftPinkyPinchSince=null,leftHandCooldownUntil=0;
let rightMidPinchSince=null,rightRingPinchSince=null,rightPinkyPinchSince=null,rightHandCooldownUntil=0;
function cycleAdaptiveQualityMode(){
  const order=ADAPTIVE_QUALITY_MODES;
  const next=order[(order.indexOf(adaptiveQuality.getMode())+1)%order.length];
  adaptiveQuality.setMode(next);
  lastQualityState=adaptiveQuality.state();
  if(qualityModeEl)qualityModeEl.value=next;
  playModeTick('pin');
  showHudToast(`QUALITY: ${next.toUpperCase()}`,next==='crisp'?'#38bdf8':(next==='fluid'?'#f59e0b':'#4ade80'),1500);
}
function cycleHandGlow(){
  handGlowLevel=(handGlowLevel+1)%HAND_GLOW_LEVELS.length;
  playModeTick('pin');
  showHudToast(`HAND GLOW: ${HAND_GLOW_LEVELS[handGlowLevel]}`,['#94a3b8','#4ade80','#38bdf8','#f59e0b'][handGlowLevel],1400);
}
function toggleLongRangeMode(){
  const next=!adaptiveQuality.getLongRange();
  adaptiveQuality.setLongRange(next);
  lastQualityState=adaptiveQuality.state();
  if(longRangeEl)longRangeEl.checked=next;
  playModeTick(next?'scan':'pin');
  showHudToast(next?'RANGE: EXP FAR (DEC 1 IDLE)':'RANGE: AUTO (MAX DEC 1.5)',next?'#22d3ee':'#94a3b8',1600);
}
function toggleDevMode(){
  devModeEnabled=!devModeEnabled;
  if(devModeEl)devModeEl.checked=devModeEnabled;
  playModeTick(devModeEnabled?'scan':'pin');
  showHudToast(devModeEnabled?'DEV MODE: ON (L Y-AXIS)':'DEV MODE: OFF',devModeEnabled?'#c084fc':'#94a3b8',1600);
}
let leftPalmDwellSince=null,leftMenuOpenUntil=0,leftWristPrev=null,leftWristSpeed=0,leftDotTapSince=null;
let leftPalmFacingViewer=false,leftMenuOpenNow=false;
let rightPalmDwellSince=null,rightMenuOpenUntil=0,rightWristPrev=null,rightWristSpeed=0,rightDotTapSince=null;
let wristPillPokeId=null,wristPillPokeSince=0;
let liveTouchLoaderActive=false;
let pinHoldWp=null,pinHoldKey=null,pinHoldHand=null,pinHoldFloorMm=null,pinHoldSince=null,pinHoldConsumed=false;
let hoverCardPosL=null,hoverCardForWpKey=null,lastInspectorTimeMs=null,lastViewerBasis=null;
let pinFlashPos=null,pinFlashUntilMs=0,pinFlashColor='#e879f9',pinFlashSymbol='✓';
let lastXrTime=null,prevXrTime=null,fpsHistory=[],stabilityHistory=[],rmsHistory=[],maxFrameGapMs=0;
let anchorBaseline=null,liveJitterMm=null,liveDriftMm=null,liveDriftDeg=null;
const ANCHOR_WIN_CAP=60;
const anchorWinBuf=new Float32Array(ANCHOR_WIN_CAP*3);
let anchorWinHead=0,anchorWinCount=0,hasPrevAnchorWorld=false;
const _prevAnchorWorldBuf=new Float32Array(16);
function resetAnchorStability(){
  anchorBaseline=null;hasPrevAnchorWorld=false;anchorWinHead=0;anchorWinCount=0;
  liveJitterMm=null;liveDriftMm=null;liveDriftDeg=null;
}
let lastSampleStats=null,lastObs=null,lastObsAtMs=0,lastAuditRow=null,lastAuditAt=0,lastMovedTargetAt=0,lastMotionState=null,rvfcWindowStart=null,rvfcWindowCount=0,rvfcHz=0,scanPulseCounter=0,recentFocusDetectMs=15;
let activeFocusTagIds=new Set();
let calViewerPoses=[],lastCalCount=0,lastCalPhase='idle';
let placedExtras=new Map(),extraPending=new Map(),movableTools=new Map(),lastDiagnostics=null,diagnostics=null;
function pushDiagEvent(ev){
  if(!diagnostics)return;
  if(diagnostics.events.length<500)diagnostics.events.push(ev);
  else{diagnostics.events.splice(50,1);diagnostics.events.push(ev);}
}
const obsBuffer=createObservationBuffer({maxAgeMs:650,maxItems:16});
const audit=createAnchorAudit({maxSamples:120});
const handHistory=createHandHistory({maxMs:1800});
const tagHandCoupler=createTagHandCoupler({opticalStaleTimeoutMs:380,maxUnconfirmedDriftM:0.26});
const tagPredictor=createTagMotionPredictor();
// Quest 3 Passthrough shows moving (non-head-reprojected) objects behind WebXR scanout (xrTimeMs).
// Subtract devOptLagMs only for visual display so Focus-ROI stays at full physical lead (0.90 / 120 ms)
// while the drawn overlay aligns with the Passthrough car (live-tunable in Dev Dock or via ?ptLag=<ms>, 0..120).
let devOptLagMs=(()=>{
  if(urlParams.has('ptLag')){
    const v=Number(urlParams.get('ptLag'));
    if(Number.isFinite(v))return Math.max(0,Math.min(120,Math.round(v/2)*2));
  }
  return 28;
})();
let devHandLagMs=(()=>{
  if(urlParams.has('handLag')){
    const v=Number(urlParams.get('handLag'));
    if(Number.isFinite(v))return Math.max(0,Math.min(60,Math.round(v/2)*2));
  }
  return 22;
})();
const DISPLAY_PREDICT_OPTS=Object.freeze({minLeadSec:-0.05});
function predictDisplayTagPose(tagId,xrTimeMs){
  if(!devDisplayPredEnabled||!Number.isFinite(xrTimeMs))return null;
  return tagPredictor.predict(tagId,xrTimeMs-devOptLagMs,DISPLAY_PREDICT_OPTS);
}
const focusScheduler=createFocusScheduler();
const context=()=>({sessionId,spaceEpoch:epoch});
// M.E.S.S. convention: x00-x49 = generic (no assigned role, user decides how to use it); x50-x99 = dedicated (role from a geometry profile).
// Hundreds digit only selects the size class. 500+ (special) and 0-99 (debug) are neither.
const isGenericTag=id=>Number.isInteger(id)&&id>=100&&id<500&&(id%100)<50;
const warnedUnassignedDedicated=new Set();
const refGroup=new THREE.Group();
root.add(refGroup);
const portalRoot=new THREE.Group();
root.add(portalRoot);

// Interactive spatial buttons anchored directly to the printed L-bracket (A4 & Tablet/e-Ink):
// • Y-arm QR code  -> https://eb.mess.icu/?from=webxr-apriltag (no warp; explicit "Visit eb.mess.icu -> leaving this demo")
// • X-arm SETUP    -> prominent "★ mess.engineering" showcase panel (mess.engineering & wetter.mess.engineering previews)
const PORTAL_SPECS_BY_RIG={
  'mess-ms-qr-v2.18-en-main-l':[
    {
      id:'eb-qr',x:0.025,y:0.075,w:0.036,h:0.036,
      title:'eb.mess.icu',subtitle:'Visit eb.mess.icu -> leaving this demo',
      ctaHand:'PINCH OR HOLD TO VISIT',ctaCtrl:'TRIGGER OR HOLD TO VISIT',
      url:'https://eb.mess.icu/?from=webxr-apriltag',
      warpTitle:'Visit eb.mess.icu',warpSub:'Visit eb.mess.icu -> leaving this demo',
      accentHex:0x38bdf8,accentCss:'#38bdf8',secondaryHex:0x22d3ee,
      idleHint:false,warp:false,panel:false,
    },
    {
      id:'mess-showcase',x:0.125,y:0.015,w:0.044,h:0.028,
      title:'mess.engineering',subtitle:'Homepage & WETTER Image Framework',
      ctaHand:'PINCH OR HOLD TO OPEN',ctaCtrl:'TRIGGER OR HOLD TO OPEN',
      accentHex:0x38bdf8,accentCss:'#38bdf8',secondaryHex:0x34d399,
      idleHint:true,sparkle:true,warp:false,panel:true,
    },
  ],
  'mess-tablet-200x150-370-371-372':[
    {
      id:'eb-qr',x:0.020,y:0.075,w:0.026,h:0.0263,
      title:'eb.mess.icu',subtitle:'Visit eb.mess.icu -> leaving this demo',
      ctaHand:'PINCH OR HOLD TO VISIT',ctaCtrl:'TRIGGER OR HOLD TO VISIT',
      url:'https://eb.mess.icu/?from=webxr-apriltag',
      warpTitle:'Visit eb.mess.icu',warpSub:'Visit eb.mess.icu -> leaving this demo',
      accentHex:0x38bdf8,accentCss:'#38bdf8',secondaryHex:0x22d3ee,
      idleHint:false,warp:false,panel:false,
    },
    {
      id:'mess-showcase',x:0.095,y:0.0165,w:0.042,h:0.022,
      title:'mess.engineering',subtitle:'Homepage & WETTER Image Framework',
      ctaHand:'PINCH OR HOLD TO OPEN',ctaCtrl:'TRIGGER OR HOLD TO OPEN',
      accentHex:0x38bdf8,accentCss:'#38bdf8',secondaryHex:0x34d399,
      idleHint:true,sparkle:true,warp:false,panel:true,
    },
  ],
};

// Minimalist In-XR Showcase Panel (mess.engineering + wetter.mess.engineering clickable cards with real WebP previews).
// Uses THREE.Sprite so it always faces the viewer upright and cleanly, with attached canvas [×] & card links (no detached orb).
const SHOWCASE_W_M=0.58,SHOWCASE_H_M=0.28;
const showcaseGroup=new THREE.Group();
showcaseGroup.visible=false;
root.add(showcaseGroup);
const showcaseCanvas=document.createElement('canvas');
showcaseCanvas.width=960;showcaseCanvas.height=460;
const showcaseTex=makeUiTexture(showcaseCanvas);
showcaseTex.colorSpace=THREE.SRGBColorSpace;
const showcaseSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:showcaseTex,transparent:true,depthTest:false,depthWrite:false,toneMapped:false}));
showcaseSprite.scale.set(SHOWCASE_W_M,SHOWCASE_H_M,1);
showcaseSprite.renderOrder=15;
showcaseGroup.add(showcaseSprite);

const showcaseImages={mess:null,wetter:null};
let showcaseOpen=false,showcaseHoverZone=null,showcasePokeZone=null,showcasePokeSince=0;
let showcaseNavState=null; // {url, title, subtitle, accentCss, startedAt, durationMs, navigated}

// Explicit 2-step "Leave Demo?" confirmation modal in world space (near-field reach for direct finger poke + far raycast).
const LEAVE_W_M=0.36,LEAVE_H_M=0.176;
const LEAVE_CANVAS_W=840,LEAVE_CANVAS_H=410;
const LEAVE_SPAWN_DIST_M=0.42,LEAVE_MIN_TABLE_H_M=0.14,LEAVE_AUTO_CLOSE_DIST_M=1.5;
const LEAVE_SELECT_GRACE_MS=250,LEAVE_POKE_GRACE_MS=450;
const LEAVE_CANCEL_POKE_MS=220,LEAVE_CONFIRM_POKE_MS=450;
const leaveConfirmCanvas=document.createElement('canvas');
leaveConfirmCanvas.width=LEAVE_CANVAS_W;leaveConfirmCanvas.height=LEAVE_CANVAS_H;
const leaveConfirmTex=makeUiTexture(leaveConfirmCanvas);
leaveConfirmTex.colorSpace=THREE.SRGBColorSpace;
const leaveConfirmMesh=new THREE.Mesh(
  new THREE.PlaneGeometry(LEAVE_W_M,LEAVE_H_M),
  new THREE.MeshBasicMaterial({map:leaveConfirmTex,transparent:true,side:THREE.DoubleSide,depthTest:false,depthWrite:false,toneMapped:false}),
);
leaveConfirmMesh.visible=false;
leaveConfirmMesh.renderOrder=1002;
scene.add(leaveConfirmMesh);
const _lcEye=new THREE.Vector3();
const _lcFwd=new THREE.Vector3();
const _lcPos=new THREE.Vector3();
const _lcTableO=new THREE.Vector3();
const _lcTableN=new THREE.Vector3();
const _lcProbeW=new THREE.Vector3();
const _lcProbeL=new THREE.Vector3();
const _lcSurfaceW=new THREE.Vector3();
let leaveConfirmState=null; // {host, url, accentCss, openedAt}
let leaveConfirmHover=null; // 'confirm' | 'cancel' | null
let leaveConfirmPokeZone=null; // 'confirm' | 'cancel' | null
let leaveConfirmPokeSince=0;
let leaveConfirmNeedsPlacement=false;
let lastLeaveConfirmKey='';

function placeLeaveConfirm(pose,world){
  const vm=pose?.transform?.matrix;
  if(!vm)return;
  _lcEye.set(vm[12],vm[13],vm[14]);
  _lcFwd.set(-vm[8],-vm[9],-vm[10]);
  if(_lcFwd.lengthSq()<1e-6)_lcFwd.set(0,0,-1);
  else _lcFwd.normalize();

  const horiz=Math.hypot(_lcFwd.x,_lcFwd.z);
  if(horiz<0.05){
    let hx=vm[4],hz=vm[6],hl=Math.hypot(hx,hz);
    if(hl<1e-4){hx=0;hz=-1;hl=1;}
    const clampedY=-0.82;
    const xzScale=Math.sqrt(1-clampedY*clampedY)/hl;
    _lcFwd.set(hx*xzScale,clampedY,hz*xzScale);
  }else{
    const clampedY=Math.max(-0.82,Math.min(0.42,_lcFwd.y));
    const xzScale=Math.sqrt(1-clampedY*clampedY)/horiz;
    _lcFwd.set(_lcFwd.x*xzScale,clampedY,_lcFwd.z*xzScale);
  }

  const levelFactor=Math.max(0,Math.min(1,(_lcFwd.y+0.50)/0.35));
  _lcPos.copy(_lcEye).addScaledVector(_lcFwd,LEAVE_SPAWN_DIST_M);
  _lcPos.y-=levelFactor*0.035;

  if(world){
    _lcTableO.set(world[12],world[13],world[14]);
    _lcTableN.set(world[8],world[9],world[10]);
    if(_lcTableN.lengthSq()>1e-6){
      _lcTableN.normalize();
      if(_lcTableN.dot(_lcProbeW.copy(_lcEye).sub(_lcTableO))<0)_lcTableN.negate();
      const hAbove=_lcProbeW.copy(_lcPos).sub(_lcTableO).dot(_lcTableN);
      if(hAbove<LEAVE_MIN_TABLE_H_M){
        _lcPos.addScaledVector(_lcTableN,LEAVE_MIN_TABLE_H_M-hAbove);
      }
    }
  }

  const dist=_lcPos.distanceTo(_lcEye);
  if(dist>1e-4){
    const clampedDist=Math.max(0.34,Math.min(0.48,dist));
    if(clampedDist!==dist){
      _lcPos.sub(_lcEye).multiplyScalar(clampedDist/dist).add(_lcEye);
    }
  }else{
    _lcPos.copy(_lcEye).addScaledVector(_lcFwd,0.38);
  }

  leaveConfirmMesh.position.copy(_lcPos);
  leaveConfirmMesh.up.set(0,1,0);
  leaveConfirmMesh.lookAt(_lcEye);
  leaveConfirmMesh.updateMatrixWorld(true);
}

function drawLeaveConfirm(pokeProg=0){
  if(!leaveConfirmState)return;
  const {host}=leaveConfirmState;
  const prog=Math.max(0,Math.min(1,pokeProg));
  const key=`${host}|${leaveConfirmHover||''}|${leaveConfirmPokeZone||''}|${Math.round(prog*20)}`;
  if(key===lastLeaveConfirmKey)return;
  lastLeaveConfirmKey=key;
  const ctx=leaveConfirmCanvas.getContext('2d');
  ctx.clearRect(0,0,LEAVE_CANVAS_W,LEAVE_CANVAS_H);
  ctx.fillStyle='rgba(7,11,22,0.97)';
  ctx.beginPath();ctx.roundRect(8,8,LEAVE_CANVAS_W-16,LEAVE_CANVAS_H-16,22);ctx.fill();
  ctx.strokeStyle='#f59e0b';ctx.lineWidth=4.5;ctx.stroke();

  ctx.fillStyle='#fbbf24';ctx.font='bold 34px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText('⚠ LEAVE THIS DEMO?',420,60,780);
  ctx.fillStyle='#f8fafc';ctx.font='600 23px system-ui,sans-serif';
  ctx.fillText(`Visit ${host} -> you will leave this XR demo`,420,108,780);
  ctx.fillStyle='#94a3b8';ctx.font='500 20px system-ui,sans-serif';
  ctx.fillText('The XR session will end and you will not return automatically.',420,144,780);

  // Confirm button (left)
  const cHot=leaveConfirmHover==='confirm';
  const cProg=leaveConfirmPokeZone==='confirm'?prog:0;
  ctx.fillStyle=cHot?(cProg>0?'rgba(72,42,8,0.96)':'#f59e0b'):'rgba(245,158,11,0.22)';
  ctx.beginPath();ctx.roundRect(38,192,416,132,18);ctx.fill();
  if(cProg>0){
    ctx.save();
    ctx.beginPath();ctx.roundRect(38,192,416,132,18);ctx.clip();
    ctx.fillStyle='rgba(245,158,11,0.34)';
    ctx.fillRect(38,192,416*cProg,132);
    ctx.fillStyle='#f59e0b';
    ctx.fillRect(38,310,416*cProg,14);
    ctx.restore();
  }
  ctx.beginPath();ctx.roundRect(38,192,416,132,18);
  ctx.strokeStyle=cHot?'#fef08a':'#f59e0b';ctx.lineWidth=cHot?4.5:3;ctx.stroke();
  ctx.fillStyle=(cHot&&cProg===0)?'#0f172a':'#fef3c7';ctx.font='bold 25px system-ui,sans-serif';
  ctx.fillText('↗ LEAVE & OPEN',246,246,384);
  ctx.fillStyle=(cHot&&cProg===0)?'#1e293b':'#fde68a';ctx.font='600 20px ui-monospace,monospace';
  ctx.fillText(host,246,284,384);

  // Cancel / Stay button (right)
  const sHot=leaveConfirmHover==='cancel';
  const sProg=leaveConfirmPokeZone==='cancel'?prog:0;
  ctx.fillStyle=sHot?(sProg>0?'rgba(14,58,34,0.96)':'#4ade80'):'rgba(34,197,94,0.22)';
  ctx.beginPath();ctx.roundRect(486,192,316,132,18);ctx.fill();
  if(sProg>0){
    ctx.save();
    ctx.beginPath();ctx.roundRect(486,192,316,132,18);ctx.clip();
    ctx.fillStyle='rgba(34,197,94,0.36)';
    ctx.fillRect(486,192,316*sProg,132);
    ctx.fillStyle='#22c55e';
    ctx.fillRect(486,310,316*sProg,14);
    ctx.restore();
  }
  ctx.beginPath();ctx.roundRect(486,192,316,132,18);
  ctx.strokeStyle=sHot?'#bbf7d0':'#4ade80';ctx.lineWidth=sHot?4.5:3;ctx.stroke();
  ctx.fillStyle=(sHot&&sProg===0)?'#0f172a':'#f0fdf4';ctx.font='bold 26px system-ui,sans-serif';
  ctx.fillText('✕ STAY IN DEMO',644,246,288);
  ctx.fillStyle=(sHot&&sProg===0)?'#14532d':'#86efac';ctx.font='500 19px system-ui,sans-serif';
  ctx.fillText('Return to AR',644,284,288);

  ctx.fillStyle='#94a3b8';ctx.font='500 18px system-ui,sans-serif';
  ctx.fillText('Touch a button with your finger, or point & pinch / trigger',420,368,780);
  ctx.textAlign='left';
  leaveConfirmTex.needsUpdate=true;
}
function openLeaveConfirm(host,url,accentCss='#38bdf8'){
  leaveConfirmState={host,url,accentCss,openedAt:performance.now()};
  leaveConfirmHover=null;
  leaveConfirmPokeZone=null;
  leaveConfirmPokeSince=0;
  leaveConfirmNeedsPlacement=true;
  lastLeaveConfirmKey='';
  leaveConfirmMesh.visible=false;
  drawLeaveConfirm(0);
  playModeTick('scan');
}
function closeLeaveConfirm(){
  leaveConfirmState=null;
  leaveConfirmHover=null;
  leaveConfirmPokeZone=null;
  leaveConfirmPokeSince=0;
  leaveConfirmNeedsPlacement=false;
  lastLeaveConfirmKey='';
  leaveConfirmMesh.visible=false;
}
function classifyLeaveConfirmUv(u,v){
  const cx=u*LEAVE_CANVAS_W,cy=(1-v)*LEAVE_CANVAS_H;
  if(cy>=180&&cy<=338){
    if(cx>=26&&cx<=462)return 'confirm';
    if(cx>=478&&cx<=814)return 'cancel';
  }
  return null;
}
function executeLeaveConfirm(action,now){
  if(!leaveConfirmState)return;
  if(action==='cancel'){
    closeLeaveConfirm();
    playModeTick('unpin');
    return;
  }
  if(action==='confirm'){
    const {host,url,accentCss}=leaveConfirmState;
    closeLeaveConfirm();
    showcaseOpen=false;showcaseGroup.visible=false;showcaseHoverZone=null;showcasePokeZone=null;showcasePokeSince=0;
    showcaseNavState={
      url,
      title:`Visit ${host}`,
      subtitle:`Visit ${host} -> leaving this demo`,
      accentCss,
      startedAt:now,
      durationMs:500,
      navigated:false,
    };
    status(showcaseNavState.subtitle);
    drawWarpBanner(showcaseNavState.title,showcaseNavState.subtitle,accentCss,0.12);
  }
}

function drawShowcasePanel(){
  const ctx=showcaseCanvas.getContext('2d');
  ctx.clearRect(0,0,960,460);
  ctx.fillStyle='rgba(7,12,24,0.96)';
  ctx.beginPath();ctx.roundRect(8,8,944,444,22);ctx.fill();
  ctx.strokeStyle='#38bdf8';ctx.lineWidth=4;ctx.stroke();

  ctx.fillStyle='#f8fafc';ctx.font='bold 30px system-ui,sans-serif';
  ctx.fillText('M.E.S.S. · Homepage & WETTER Image Framework',32,54);
  // Attached [×] close button inside the panel frame (top-right)
  const closeHot=showcaseHoverZone==='close';
  ctx.fillStyle=closeHot?'#fef08a':'#dc2626';
  ctx.beginPath();ctx.roundRect(872,22,56,44,12);ctx.fill();
  ctx.fillStyle=closeHot?'#0f172a':'#ffffff';
  ctx.font='bold 28px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText('×',900,53);ctx.textAlign='left';

  const cards=[
    {id:'url-mess',x:28,title:'mess.engineering',sub:'Mattern Engineering & Software Solutions',img:showcaseImages.mess,accent:'#38bdf8'},
    {id:'url-wetter',x:492,title:'wetter.mess.engineering',sub:'WETTER · Scientific Image Framework',img:showcaseImages.wetter,accent:'#34d399'},
  ];
  for(const card of cards){
    const urlHot=showcaseHoverZone===card.id;
    ctx.fillStyle='rgba(15,23,42,0.94)';
    ctx.beginPath();ctx.roundRect(card.x,78,440,322,16);ctx.fill();
    ctx.strokeStyle='#334155';ctx.lineWidth=2.5;ctx.stroke();
    // Top-aligned screenshot preview area (info only, not a link)
    const ix=card.x+14,iy=92,iw=412,ih=212;
    ctx.save();
    ctx.beginPath();ctx.roundRect(ix,iy,iw,ih,10);ctx.clip();
    ctx.fillStyle='#070c18';ctx.fillRect(ix,iy,iw,ih);
    if(card.img&&card.img.complete&&card.img.naturalWidth>0){
      const scale=iw/card.img.naturalWidth;
      const dw=iw,dh=card.img.naturalHeight*scale;
      ctx.imageSmoothingEnabled=true;
      ctx.imageSmoothingQuality='high';
      ctx.drawImage(card.img,ix,iy,dw,dh);
    }else{
      ctx.fillStyle='#1e293b';ctx.fillRect(ix,iy,iw,ih);
      ctx.fillStyle='#94a3b8';ctx.font='600 20px system-ui,sans-serif';ctx.textAlign='center';
      ctx.fillText('Loading preview…',ix+iw/2,iy+ih/2+6);ctx.textAlign='left';
    }
    ctx.restore();
    ctx.fillStyle='#cbd5e1';ctx.font='500 18px system-ui,sans-serif';
    ctx.fillText(card.sub,card.x+18,332);
    // Optional clickable URL pill (triggers 2-step "Leave Demo?" confirmation)
    ctx.fillStyle=urlHot?'rgba(254,240,138,0.18)':'rgba(30,41,59,0.9)';
    ctx.beginPath();ctx.roundRect(card.x+14,348,412,40,10);ctx.fill();
    ctx.strokeStyle=urlHot?'#fef08a':card.accent;ctx.lineWidth=urlHot?3:2;ctx.stroke();
    ctx.fillStyle=urlHot?'#fef08a':card.accent;ctx.font='bold 22px ui-monospace,monospace';ctx.textAlign='center';
    ctx.fillText(`↗ ${card.title}`,card.x+220,375,392);ctx.textAlign='left';
  }
  ctx.fillStyle='#94a3b8';ctx.font='500 17px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText('Info preview · Click a URL below to visit (asks before leaving) · [×] to close',480,432);
  ctx.textAlign='left';
  showcaseTex.needsUpdate=true;
}
function ensureShowcaseImages(){
  if(!showcaseImages.mess){
    const im=new Image();im.onload=()=>drawShowcasePanel();
    im.src=new URL('./assets/mess_icu.webp',import.meta.url).href;
    showcaseImages.mess=im;
  }
  if(!showcaseImages.wetter){
    const im=new Image();im.onload=()=>drawShowcasePanel();
    im.src=new URL('./assets/wetter_mess_icu.webp',import.meta.url).href;
    showcaseImages.wetter=im;
  }
}
function toggleShowcasePanel(open){
  showcaseOpen=Boolean(open);
  showcaseGroup.visible=showcaseOpen;
  if(showcaseOpen){
    ensureShowcaseImages();
    showcaseHoverZone=null;showcasePokeZone=null;showcasePokeSince=0;
    drawShowcasePanel();
    playModeTick('pin');
  }else{
    showcaseHoverZone=null;showcasePokeZone=null;showcasePokeSince=0;
    playModeTick('unpin');
  }
}
function triggerShowcaseAction(zone,now){
  if(zone==='close'){toggleShowcasePanel(false);return;}
  if(zone==='url-mess'){
    openLeaveConfirm('mess.engineering','https://mess.engineering/?from=webxr-apriltag','#38bdf8');
  }else if(zone==='url-wetter'){
    openLeaveConfirm('wetter.mess.engineering','https://wetter.mess.engineering/?from=webxr-apriltag','#34d399');
  }
}

let spatialPortals=[];
let hoveredPortal=null;
let portalHoldId=null;
let portalHoldSinceMs=0;
let portalWarpState=null;

function makeRectFrameGeo(w,h,border=0.0022){
  const hw=w/2,hh=h/2,iw=Math.max(0.001,hw-border),ih=Math.max(0.001,hh-border);
  const pos=new Float32Array([
    -hw,-hh,0,  hw,-hh,0,  hw,-ih,0,  -hw,-hh,0,  hw,-ih,0, -hw,-ih,0,
    -hw, ih,0,  hw, ih,0,  hw, hh,0,  -hw, ih,0,  hw, hh,0, -hw, hh,0,
    -hw,-ih,0, -iw,-ih,0, -iw, ih,0,  -hw,-ih,0, -iw, ih,0, -hw, ih,0,
     iw,-ih,0,  hw,-ih,0,  hw, ih,0,   iw,-ih,0,  hw, ih,0,  iw, ih,0,
  ]);
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  return geo;
}

function drawPortalCard(portal,probeKind,holdProg=0){
  const ctx=portal.cardCanvas.getContext('2d');
  ctx.clearRect(0,0,420,148);
  ctx.fillStyle=portal.spec.warp?'rgba(16,10,28,0.95)':'rgba(8,16,28,0.94)';
  ctx.beginPath();ctx.roundRect(5,5,410,138,14);ctx.fill();
  ctx.strokeStyle=portal.spec.accentCss;ctx.lineWidth=3.5;ctx.stroke();
  ctx.fillStyle=portal.spec.accentCss;ctx.font='bold 30px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(portal.spec.title,210,46,380);
  ctx.fillStyle='#e2e8f0';ctx.font='bold 18px ui-monospace,monospace';
  ctx.fillText(portal.spec.subtitle,210,80);
  if(holdProg>0.01){
    ctx.fillStyle='rgba(30,41,59,0.95)';
    ctx.beginPath();ctx.roundRect(38,100,344,18,9);ctx.fill();
    ctx.fillStyle=portal.spec.accentCss;
    ctx.beginPath();ctx.roundRect(38,100,Math.max(16,Math.round(344*Math.min(1,holdProg))),18,9);ctx.fill();
  }else{
    ctx.fillStyle='#94a3b8';ctx.font='bold 16px system-ui,sans-serif';
    ctx.fillText(probeKind==='hand'?portal.spec.ctaHand:portal.spec.ctaCtrl,210,118);
  }
  ctx.textAlign='left';
  portal.cardTex.needsUpdate=true;
}

function makePortalIdleGlyph(text,accent){
  const c=document.createElement('canvas');c.width=96;c.height=52;
  const ctx=c.getContext('2d');
  ctx.fillStyle='rgba(15,10,28,0.78)';
  ctx.beginPath();ctx.roundRect(4,4,88,44,12);ctx.fill();
  ctx.strokeStyle=accent;ctx.lineWidth=3;ctx.stroke();
  ctx.fillStyle=accent;ctx.font='bold 28px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(text,48,36);ctx.textAlign='left';
  const tex=makeUiTexture(c);
  const mat=new THREE.SpriteMaterial({map:tex,transparent:true,opacity:0.78,depthTest:false,depthWrite:false});
  const spr=new THREE.Sprite(mat);
  spr.scale.set(0.020,0.011,1);
  spr.renderOrder=8;
  return {spr,tex,mat};
}

function makeSparkleTexture(accent='#7dd3fc'){
  const c=document.createElement('canvas');c.width=64;c.height=64;
  const ctx=c.getContext('2d');
  const g=ctx.createRadialGradient(32,32,1,32,32,30);
  g.addColorStop(0,'rgba(255,255,255,0.98)');
  g.addColorStop(0.28,'rgba(125,211,252,0.72)');
  g.addColorStop(1,'rgba(56,189,248,0)');
  ctx.fillStyle=g;
  ctx.fillRect(0,0,64,64);
  ctx.fillStyle='#ffffff';
  ctx.beginPath();
  ctx.moveTo(32,4);ctx.quadraticCurveTo(34,30,60,32);
  ctx.quadraticCurveTo(34,34,32,60);ctx.quadraticCurveTo(30,34,4,32);
  ctx.quadraticCurveTo(30,30,32,4);ctx.closePath();ctx.fill();
  ctx.strokeStyle=accent;ctx.lineWidth=1.5;ctx.stroke();
  return makeUiTexture(c);
}

function disposeSpatialPortals(){
  for(const p of spatialPortals){
    portalRoot.remove(p.group);
    p.baseGeo.dispose();p.baseMat.dispose();
    if(p.glowGeo)p.glowGeo.dispose();
    if(p.glowMat)p.glowMat.dispose();
    if(p.sparkleTex)p.sparkleTex.dispose();
    for(const sp of p.sparkles||[])sp.mat.dispose();
    p.frameGeo.dispose();p.frameMat.dispose();
    p.pillarGeo.dispose();p.pillarMat.dispose();
    p.ringGeo?.dispose();p.ringMat?.dispose();
    p.holdRingGeo.dispose();p.holdRingMat.dispose();
    p.cardTex.dispose();p.cardSprite.material.dispose();
    if(p.idleGlyph){p.idleGlyph.tex.dispose();p.idleGlyph.mat.dispose();}
    for(const sw of p.shockwaves){sw.mesh.geometry.dispose();sw.mat.dispose();}
  }
  spatialPortals=[];
  hoveredPortal=null;portalHoldId=null;portalHoldSinceMs=0;
}

function buildSpatialPortals(rig){
  disposeSpatialPortals();
  const specs=PORTAL_SPECS_BY_RIG[rig?.id]||[];
  for(const spec of specs){
    const group=new THREE.Group();
    group.position.set(spec.x,spec.y,0);
    const baseGeo=makeRectFrameGeo(spec.w,spec.h,0.0014);
    const baseMat=new THREE.MeshBasicMaterial({color:spec.accentHex,transparent:true,opacity:spec.idleHint?0.42:0,side:THREE.DoubleSide,depthWrite:false});
    const baseMesh=new THREE.Mesh(baseGeo,baseMat);
    baseMesh.position.z=0.0014;
    group.add(baseMesh);

    let glowGeo=null,glowMat=null,sparkleTex=null;
    const sparkles=[];
    if(spec.sparkle){
      glowGeo=new THREE.PlaneGeometry(spec.w*0.94,spec.h*0.94);
      glowMat=new THREE.MeshBasicMaterial({color:spec.secondaryHex??spec.accentHex,transparent:true,opacity:0.14,side:THREE.DoubleSide,depthWrite:false});
      const glowMesh=new THREE.Mesh(glowGeo,glowMat);
      glowMesh.position.z=0.0008;
      glowMesh.renderOrder=5;
      group.add(glowMesh);

      sparkleTex=makeSparkleTexture(spec.accentCss||'#7dd3fc');
      const hwS=spec.w*0.44,hhS=spec.h*0.42;
      const anchors=[[-hwS,-hhS],[hwS,-hhS],[hwS,hhS],[-hwS,hhS]];
      for(let i=0;i<anchors.length;i++){
        const mat=new THREE.SpriteMaterial({map:sparkleTex,transparent:true,opacity:0.8,depthTest:false,depthWrite:false});
        const spr=new THREE.Sprite(mat);
        spr.position.set(anchors[i][0],anchors[i][1],0.005);
        spr.scale.set(0.010,0.010,1);
        spr.renderOrder=9;
        group.add(spr);
        sparkles.push({spr,mat,ax:anchors[i][0],ay:anchors[i][1],phase:i*1.75});
      }
    }

    let idleGlyph=null;
    if(spec.idleHint){
      idleGlyph=makePortalIdleGlyph(spec.sparkle?'✦':'⚡',spec.accentCss);
      idleGlyph.spr.position.set(0,0,spec.sparkle?0.008:0.005);
      group.add(idleGlyph.spr);
    }

    // Four rising corner light pillars connecting printed surface (z=0) to lifted 3D portal frame.
    const hw=spec.w/2,hh=spec.h/2;
    const pillarPositions=new Float32Array([
      -hw,-hh,0, -hw,-hh,1,
       hw,-hh,0,  hw,-hh,1,
       hw, hh,0,  hw, hh,1,
      -hw, hh,0, -hw, hh,1,
    ]);
    const pillarGeo=new THREE.BufferGeometry();
    pillarGeo.setAttribute('position',new THREE.BufferAttribute(pillarPositions,3));
    const pillarMat=new THREE.LineBasicMaterial({color:spec.accentHex,transparent:true,opacity:0});
    const pillars=new THREE.LineSegments(pillarGeo,pillarMat);
    pillars.scale.set(1,1,0.001);
    pillars.visible=false;
    group.add(pillars);

    const liftGroup=new THREE.Group();
    liftGroup.visible=false;
    group.add(liftGroup);

    const frameGeo=makeRectFrameGeo(spec.w,spec.h,0.0024);
    const frameMat=new THREE.MeshBasicMaterial({color:spec.accentHex,transparent:true,opacity:0.92,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
    const frameMesh=new THREE.Mesh(frameGeo,frameMat);
    frameMesh.renderOrder=10;
    liftGroup.add(frameMesh);

    let ringGeo=null,ringMat=null,ringMesh=null;
    if(spec.warp){
      const r=Math.min(spec.w,spec.h)*0.42;
      ringGeo=new THREE.RingGeometry(r*0.70,r,28);
      ringMat=new THREE.MeshBasicMaterial({color:spec.secondaryHex,transparent:true,opacity:0.85,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
      ringMesh=new THREE.Mesh(ringGeo,ringMat);
      ringMesh.position.z=0.001;
      ringMesh.renderOrder=10;
      liftGroup.add(ringMesh);
    }

    const holdR=Math.max(spec.w,spec.h)*0.62;
    const holdRingGeo=new THREE.RingGeometry(holdR*0.86,holdR,36);
    const holdRingMat=new THREE.MeshBasicMaterial({color:spec.secondaryHex,transparent:true,opacity:0.95,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
    const holdRing=new THREE.Mesh(holdRingGeo,holdRingMat);
    holdRing.position.z=0.002;
    holdRing.visible=false;
    holdRing.renderOrder=11;
    liftGroup.add(holdRing);

    const cardCanvas=document.createElement('canvas');cardCanvas.width=420;cardCanvas.height=148;
    const cardTex=makeUiTexture(cardCanvas);
    const cardSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:cardTex,transparent:true,opacity:0,depthTest:false,depthWrite:false}));
    cardSprite.scale.set(0.132,0.0465,1);
    cardSprite.position.set(0,0,0.042);
    cardSprite.renderOrder=12;
    liftGroup.add(cardSprite);

    // Three concentric shockwave rings launched upward during the warp / open transition.
    const shockwaves=[];
    const swColors=[spec.accentHex,spec.secondaryHex,0xffffff];
    for(let i=0;i<3;i++){
      const swGeo=new THREE.RingGeometry(0.012,0.018,36);
      const swMat=new THREE.MeshBasicMaterial({color:swColors[i],transparent:true,opacity:0,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
      const swMesh=new THREE.Mesh(swGeo,swMat);
      swMesh.visible=false;
      swMesh.renderOrder=14;
      group.add(swMesh);
      shockwaves.push({mesh:swMesh,mat:swMat,delay:i*0.14});
    }

    portalRoot.add(group);
    const portal={
      spec,group,baseGeo,baseMat,baseMesh,glowGeo,glowMat,sparkleTex,sparkles,idleGlyph,
      pillarGeo,pillarMat,pillars,liftGroup,
      frameGeo,frameMat,frameMesh,
      ringGeo,ringMat,ringMesh,
      holdRingGeo,holdRingMat,holdRing,
      cardCanvas,cardTex,cardSprite,shockwaves,
      lift:0,cardKey:'',
    };
    drawPortalCard(portal,'hand',0);
    spatialPortals.push(portal);
  }
}

const thickFrameGeoCache=new Map();
function getThickTagFrameGeo(edgeMm){
  if(thickFrameGeoCache.has(edgeMm))return thickFrameGeoCache.get(edgeMm);
  const h=(edgeMm/1000)/2;
  const i=Math.max(0.001,h-0.0004);
  const o=h+Math.max(0.0024,(edgeMm/1000)*0.12);
  // Four flat border strips (8 triangles) framing outward so the printed tag border stays clear.
  const pos=new Float32Array([
    // bottom strip
    -o,-o,0,  o,-o,0,  o,-i,0,   -o,-o,0,  o,-i,0, -o,-i,0,
    // top strip
    -o, i,0,  o, i,0,  o, o,0,   -o, i,0,  o, o,0, -o, o,0,
    // left strip
    -o,-i,0, -i,-i,0, -i, i,0,   -o,-i,0, -i, i,0, -o, i,0,
    // right strip
     i,-i,0,  o,-i,0,  o, i,0,    i,-i,0,  o, i,0,  i, i,0,
  ]);
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  thickFrameGeoCache.set(edgeMm,geo);
  return geo;
}

const MAX_TRAIL_WAYPOINTS=600;
const PATHLINE_DEFAULTS=Object.freeze({
  lineRadiusM:0.0015, // 3.0 mm calm uniform 3D line
  lineSides:5,
  lineOpacity:0.94,
  dashPeriodM:0.024,  // ~13.9 mm dash / ~10.1 mm gap outside perimeter
  dashDuty:0.58,
  turboRefSpeedMps:TURBO_SPEED_DEFAULTS.refSpeedMps, // 0.60 m/s = 600 mm/s
  waypointDotRadiusM:0.0030,
  timeCopyDotRadiusM:0.0046,
  gapBridgeRadiusM:0.00045, // 0.9 mm thin neutral-grey transfer bridge
  gapBridgeColorHex:0x94a3b8,
  gapBridgeOpacity:0.40,
  gapCapRadiusM:0.0036,
  gapEndCapColorHex:0xcbd5e1,
  gapStartCapColorHex:0xf8fafc,
});
function applyTrailLineShader(mat,cacheKey){
  mat.customProgramCacheKey=()=>cacheKey;
  mat.onBeforeCompile=shader=>{
    shader.vertexShader=shader.vertexShader
      .replace('#include <common>','#include <common>\nattribute vec2 aDash;\nattribute float aFade;\nvarying vec2 vDash;\nvarying float vFade;')
      .replace('#include <begin_vertex>','#include <begin_vertex>\nvDash = aDash;\nvFade = aFade;');
    shader.fragmentShader=shader.fragmentShader
      .replace('#include <common>','#include <common>\nvarying vec2 vDash;\nvarying float vFade;')
      .replace(
        '#include <color_fragment>',
        `#include <color_fragment>\nif (vDash.y > 0.5) {\n  float dashPhase = fract(vDash.x / ${PATHLINE_DEFAULTS.dashPeriodM.toFixed(4)});\n  if (dashPhase > ${PATHLINE_DEFAULTS.dashDuty.toFixed(2)}) discard;\n}\ndiffuseColor.a *= clamp(vFade, 0.0, 1.0);`,
      );
  };
  return mat;
}
const trailLineMat=applyTrailLineShader(new THREE.MeshBasicMaterial({
  color:0xffffff,
  vertexColors:true,
  transparent:true,
  opacity:1.0,
  depthWrite:true,
}),'trail-line-dash-fade-v2');
const trailLineDimMat=applyTrailLineShader(new THREE.MeshBasicMaterial({
  color:0xffffff,
  vertexColors:true,
  transparent:true,
  opacity:1.0,
  depthWrite:false,
}),'trail-line-dash-fade-dim-v2');
const wakeLineMat=new THREE.MeshBasicMaterial({
  color:0xffffff,
  vertexColors:true,
  transparent:true,
  opacity:MOTION_WAKE_DEFAULTS.maxOpacity,
  depthWrite:false,
});
wakeLineMat.customProgramCacheKey=()=>'wake-line-fade-v1';
wakeLineMat.onBeforeCompile=shader=>{
  shader.vertexShader=shader.vertexShader
    .replace('#include <common>','#include <common>\nattribute float aFade;\nvarying float vFade;')
    .replace('#include <begin_vertex>','#include <begin_vertex>\nvFade = aFade;');
  shader.fragmentShader=shader.fragmentShader
    .replace('#include <common>','#include <common>\nvarying float vFade;')
    .replace(
      '#include <color_fragment>',
      '#include <color_fragment>\ndiffuseColor.a *= clamp(vFade, 0.0, 1.0);',
    );
};
const _tmpMat4=new THREE.Matrix4();
const _tmpScale=new THREE.Vector3(1.12,1.12,1);
const _tmpPlaneScale=new THREE.Vector3(0.0224,0.0224,1);
const _rodGeo=new THREE.CylinderGeometry(1,1,1,6);
const _nodeGeo=new THREE.SphereGeometry(0.0045,8,6);
const _waypointDotGeo=new THREE.SphereGeometry(1,8,6);
const _originDotGeo=new THREE.SphereGeometry(1,10,8);
const _closeOrbGeo=new THREE.SphereGeometry(0.0085,10,8);
const _recOrbGeo=new THREE.SphereGeometry(0.0115,10,8);
const _upVec=new THREE.Vector3(0,1,0);
const _vA=new THREE.Vector3(),_vB=new THREE.Vector3(),_vDir=new THREE.Vector3();
function setTagOriginDot(dotMesh,matrix,edgeMm){
  const edgeM=edgeMm/1000;
  const h=edgeM*0.5*1.12;
  dotMesh.position.set(
    matrix[12]-matrix[0]*h-matrix[4]*h+matrix[8]*0.0006,
    matrix[13]-matrix[1]*h-matrix[5]*h+matrix[9]*0.0006,
    matrix[14]-matrix[2]*h-matrix[6]*h+matrix[10]*0.0006,
  );
  dotMesh.scale.setScalar(Math.max(0.0020,Math.min(0.0055,edgeM*0.13)));
}

const frameShimmerUniforms={uTime:{value:0}};
const frameBatchMat=new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.92,side:THREE.DoubleSide});
frameBatchMat.customProgramCacheKey=()=>'tag-frame-shimmer-v3';
frameBatchMat.onBeforeCompile=shader=>{
  shader.uniforms.uTime=frameShimmerUniforms.uTime;
  shader.vertexShader=shader.vertexShader
    .replace('#include <common>','#include <common>\nattribute float aAlert;\nattribute float aFade;\nvarying float vFrameFade;\nvarying vec2 vFrameUv;\nvarying float vFramePhase;\nvarying float vFrameAlert;')
    .replace(
      '#include <begin_vertex>',
      '#include <begin_vertex>\nvFrameUv = position.xy / 0.0124;\n#ifdef USE_INSTANCING\nvFramePhase = dot(instanceMatrix[3].xyz, vec3(19.37, 43.13, 27.91));\nvFrameAlert = aAlert;\nvFrameFade = aFade;\n#else\nvFramePhase = 0.0;\nvFrameAlert = 0.0;\nvFrameFade = 1.0;\n#endif',
    );
  shader.fragmentShader=shader.fragmentShader
    .replace('#include <common>','#include <common>\nuniform float uTime;\nvarying float vFrameFade;\nvarying vec2 vFrameUv;\nvarying float vFramePhase;\nvarying float vFrameAlert;')
    .replace(
      '#include <color_fragment>',
      '#include <color_fragment>\nif (vFrameAlert > 0.5) {\n  float alarmWave = 0.5 + 0.5 * sin(uTime * 11.5);\n  float alarmFlash = pow(alarmWave, 1.8);\n  diffuseColor.rgb = mix(vec3(0.88, 0.05, 0.05), vec3(1.0, 0.36, 0.30), alarmFlash);\n  diffuseColor.a = mix(0.88, 1.0, alarmFlash);\n} else {\n  float ang = atan(vFrameUv.y, vFrameUv.x);\n  float sweep = pow(0.5 + 0.5 * sin(ang * 2.0 - uTime * 2.4 + vFramePhase), 6.0);\n  float corner = smoothstep(0.60, 0.96, min(abs(vFrameUv.x), abs(vFrameUv.y)));\n  float twinkle = corner * pow(0.5 + 0.5 * sin(uTime * 4.0 + vFramePhase * 2.3 + ang * 4.0), 4.0);\n  float glint = clamp(sweep * 0.40 + twinkle * 0.60, 0.0, 0.80);\n  diffuseColor.rgb = mix(diffuseColor.rgb, vec3(1.0), glint * 0.68);\n  diffuseColor.a = min(1.0, diffuseColor.a + glint * 0.14);\n}\ndiffuseColor.a *= vFrameFade;',
    );
};

// Shared L-frame render batches & single billboarded/oriented badge atlases for all tags, pins, rails & fence posts.
const frameBatch=createInstancedPool(
  getThickTagFrameGeo(20),
  frameBatchMat,
  64,
  {name:'frameBatch',alertAttribute:true},
);
const alertGlowGeo=new THREE.PlaneGeometry(0.044,0.044);
const alertGlowMat=new THREE.MeshBasicMaterial({
  color:0xff2222,
  transparent:true,
  opacity:1.0,
  side:THREE.DoubleSide,
  depthTest:true,
  depthWrite:false,
  blending:THREE.AdditiveBlending,
});
alertGlowMat.customProgramCacheKey=()=>'tag-alert-glow-v1';
alertGlowMat.onBeforeCompile=shader=>{
  shader.uniforms.uTime=frameShimmerUniforms.uTime;
  shader.vertexShader=shader.vertexShader
    .replace('#include <common>','#include <common>\nvarying vec2 vGlowUv;')
    .replace('#include <begin_vertex>','#include <begin_vertex>\nvGlowUv = position.xy / 0.022;');
  shader.fragmentShader=shader.fragmentShader
    .replace('#include <common>','#include <common>\nuniform float uTime;\nvarying vec2 vGlowUv;')
    .replace(
      '#include <color_fragment>',
      '#include <color_fragment>\nfloat sqDist = max(abs(vGlowUv.x), abs(vGlowUv.y));\nfloat rad = length(vGlowUv);\nfloat pulse = 0.5 + 0.5 * sin(uTime * 11.5);\nfloat flash = pow(pulse, 1.6);\nfloat ringPhase = fract(uTime * 1.65);\nfloat ringR = 0.42 + ringPhase * 0.56;\nfloat ring = smoothstep(0.13, 0.0, abs(rad - ringR)) * (1.0 - ringPhase);\nfloat innerFlash = smoothstep(0.56, 0.42, sqDist) * (0.14 + 0.24 * flash);\nfloat outerHalo = smoothstep(1.0, 0.40, sqDist) * (0.24 + 0.44 * flash);\nfloat alpha = clamp(outerHalo + innerFlash + ring * 0.85, 0.0, 0.92);\ndiffuseColor.rgb = mix(vec3(1.0, 0.07, 0.07), vec3(1.0, 0.52, 0.36), clamp(ring * 0.65 + flash * 0.32, 0.0, 1.0));\ndiffuseColor.a = alpha;',
    );
};
const alertGlowBatch=createInstancedPool(alertGlowGeo,alertGlowMat,48,{renderOrder:7,name:'alertGlowBatch'});
const overlayFrameBatch=createInstancedPool(
  getThickTagFrameGeo(20),
  new THREE.MeshBasicMaterial({color:0xffffff,side:THREE.DoubleSide,depthTest:false,depthWrite:false}),
  16,
  {renderOrder:9,name:'overlayFrameBatch'},
);
const rodBatch=createInstancedPool(
  _rodGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.92}),
  96,
  {name:'rodBatch'},
);
const overlayRodBatch=createInstancedPool(
  _rodGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.60,depthTest:false,depthWrite:false}),
  24,
  {renderOrder:10,name:'overlayRodBatch'},
);
const nodeBatch=createInstancedPool(
  _nodeGeo,
  new THREE.MeshBasicMaterial({color:0xffffff}),
  48,
  {name:'nodeBatch'},
);
const MAX_TRAIL_GAPS=192;
const _gapCapGeo=new THREE.OctahedronGeometry(1,0);
const trailGapBridgeBatch=createInstancedPool(
  _rodGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:PATHLINE_DEFAULTS.gapBridgeOpacity,depthWrite:false}),
  MAX_TRAIL_GAPS,
  {renderOrder:1,name:'trailGapBridgeBatch'},
);
const trailGapCapBatch=createInstancedPool(
  _gapCapGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.92}),
  MAX_TRAIL_GAPS*2,
  {renderOrder:3,name:'trailGapCapBatch'},
);
const zRingBatch=createInstancedPool(
  new THREE.RingGeometry(0.003,0.0065,16),
  new THREE.MeshBasicMaterial({color:0x38bdf8,side:THREE.DoubleSide,transparent:true,opacity:0.80,depthWrite:false}),
  16,
  {name:'zRingBatch'},
);
const recOrbBatch=createInstancedPool(
  _recOrbGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.35,depthTest:true,depthWrite:false}),
  64,
  {renderOrder:11,name:'recOrbBatch'},
);
const closeOrbBatch=createInstancedPool(
  _closeOrbGeo,
  new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.90,depthTest:true,depthWrite:false}),
  16,
  {renderOrder:13,name:'closeOrbBatch'},
);
const axesBatch=createAxesBatch(64);
const badgeAtlas=createBadgeAtlas({width:1024,height:2304,cellW:256,cellH:96});
const badgeMesh=createBillboardAtlasMesh(badgeAtlas,96,{renderOrder:12,depthTest:true});
const tagFaceAtlas=createBadgeAtlas({width:512,height:1024,cellW:128,cellH:128});
const tagFaceMesh=createOrientedAtlasMesh(tagFaceAtlas,32,{renderOrder:6,depthTest:true,depthWrite:false});
root.add(
  frameBatch.mesh,alertGlowBatch.mesh,overlayFrameBatch.mesh,rodBatch.mesh,overlayRodBatch.mesh,
  nodeBatch.mesh,trailGapBridgeBatch.mesh,trailGapCapBatch.mesh,zRingBatch.mesh,recOrbBatch.mesh,closeOrbBatch.mesh,
  axesBatch.lines,badgeMesh.mesh,tagFaceMesh.mesh,
);

const _framePoseBuf=new Float32Array(16);
function setTagFramePose(batch,slot,mat16,zOff=0,scaleXY=1){
  if(slot<0)return;
  const o=_framePoseBuf;
  o[0]=mat16[0]*scaleXY;o[1]=mat16[1]*scaleXY;o[2]=mat16[2]*scaleXY;o[3]=0;
  o[4]=mat16[4]*scaleXY;o[5]=mat16[5]*scaleXY;o[6]=mat16[6]*scaleXY;o[7]=0;
  o[8]=mat16[8];        o[9]=mat16[9];        o[10]=mat16[10];      o[11]=0;
  o[12]=mat16[12]+mat16[8]*zOff;
  o[13]=mat16[13]+mat16[9]*zOff;
  o[14]=mat16[14]+mat16[10]*zOff;
  o[15]=1;
  batch.setMatrix(slot,o);
}
const _localZBuf=[0,0,0];
function localZOffset(mat16,zOff){
  _localZBuf[0]=mat16[12]+mat16[8]*zOff;
  _localZBuf[1]=mat16[13]+mat16[9]*zOff;
  _localZBuf[2]=mat16[14]+mat16[10]*zOff;
  return _localZBuf;
}

function drawStaticBadgeAtlas(slot,cacheKey,text,border='#38bdf8'){
  if(slot<0)return;
  badgeAtlas.draw(slot,cacheKey,(ctx,w,h)=>{
    ctx.scale(w/180,h/64);
    ctx.fillStyle='rgba(8,14,24,0.74)';
    ctx.beginPath();ctx.roundRect(4,4,172,56,18);ctx.fill();
    ctx.strokeStyle=border;ctx.lineWidth=3;ctx.stroke();
    ctx.fillStyle='#f8fafc';ctx.font='bold 28px ui-monospace,monospace';ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(text,90,33,156);
    ctx.textAlign='left';ctx.textBaseline='alphabetic';
  });
}

function drawTagMatrixFace(slot,cacheKey,idText,accentCss,{fill='rgba(6,11,20,0.88)',statusSymbol='',statusColor=null,alarmBell=false}={}){
  if(slot<0)return;
  tagFaceAtlas.draw(slot,cacheKey,(ctx,w,h)=>{
    ctx.scale(w/128,h/128);
    ctx.fillStyle=fill;
    ctx.fillRect(0,0,128,128);
    ctx.strokeStyle=accentCss;
    ctx.lineWidth=alarmBell?7:4;
    ctx.strokeRect(4,4,120,120);
    if(alarmBell){
      ctx.fillStyle='#fef08a';
      ctx.beginPath();
      ctx.arc(64,15,3.2,0,Math.PI*2);
      ctx.fill();
      ctx.beginPath();
      ctx.arc(64,29,12.5,Math.PI,0,false);
      ctx.lineTo(80,43);
      ctx.lineTo(84,46);
      ctx.lineTo(44,46);
      ctx.lineTo(48,43);
      ctx.closePath();
      ctx.fill();
      ctx.beginPath();
      ctx.arc(64,49.5,3.8,0,Math.PI*2);
      ctx.fill();
      ctx.strokeStyle='#fde047';
      ctx.lineWidth=3;
      ctx.lineCap='round';
      for(const r of [19,25]){
        ctx.beginPath();
        ctx.arc(64,30,r,-0.44,0.44);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(64,30,r,Math.PI-0.44,Math.PI+0.44);
        ctx.stroke();
      }
      ctx.fillStyle='#ffffff';
      ctx.font='bold 44px ui-monospace,monospace';
      ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillText(idText,64,88,108);
    }else if(statusSymbol){
      ctx.fillStyle=statusColor||accentCss;
      ctx.font='bold 22px system-ui,sans-serif';
      ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillText(statusSymbol,64,28);
      ctx.fillStyle='#f8fafc';
      ctx.font='bold 42px ui-monospace,monospace';
      ctx.fillText(idText,64,78,108);
    }else{
      ctx.fillStyle='#f8fafc';
      ctx.font='bold 46px ui-monospace,monospace';
      ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillText(idText,64,66,108);
    }
    ctx.textAlign='left';ctx.textBaseline='alphabetic';
  });
}

let refRodSlots=[],refFrameSlots=[];
function buildReferenceMesh(rig){
  for(const s of refRodSlots)rodBatch.free(s);
  for(const s of refFrameSlots)frameBatch.free(s);
  refRodSlots=[];refFrameSlots=[];
  while(refGroup.children.length){
    const ch=refGroup.children[0];
    refGroup.remove(ch);
    ch.geometry?.dispose?.();
    ch.material?.dispose?.();
  }
  // Permanent, clearly visible 3D room coordinate axes (X red, Y green, Z cyan) anchored right at the L-origin.
  const axes=[
    [0xef4444,0.65,0,0.002],
    [0x22c55e,0,0.65,0.002],
    [0x38bdf8,0,0,0.55],
  ];
  for(const [hex,bx,by,bz] of axes){
    const s=rodBatch.alloc();
    if(s>=0){
      rodBatch.setRod(s,0,0,0.002,bx,by,bz,0.0022);
      rodBatch.setColor(s,hex);
      refRodSlots.push(s);
    }
  }

  const tags=rig.tools[0].tags;
  const maxX=Math.max(...tags.flatMap(t=>t.corners.map(c=>c[0])))+0.008;
  const maxY=Math.max(...tags.flatMap(t=>t.corners.map(c=>c[1])))+0.005;
  const armW=Math.max(...tags[0].corners.map(c=>c[0]))+0.008;
  const armH=Math.max(...tags[0].corners.map(c=>c[1]))+0.005;
  const lMat=new THREE.LineBasicMaterial({color:0x22d3ee});
  const lPts=[[0,0,0.0006],[maxX,0,0.0006],[maxX,armH,0.0006],[armW,armH,0.0006],[armW,maxY,0.0006],[0,maxY,0.0006],[0,0,0.0006]]
    .map(([x,y,z])=>new THREE.Vector3(x,y,z));
  refGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(lPts),lMat));
  for(const tag of tags){
    const cx=tag.corners.reduce((s,c)=>s+c[0],0)/4;
    const cy=tag.corners.reduce((s,c)=>s+c[1],0)/4;
    const edgeMm=Math.round(Math.hypot(tag.corners[1][0]-tag.corners[0][0],tag.corners[1][1]-tag.corners[0][1])*1000);
    const sc=edgeMm/20;
    const fs=frameBatch.alloc();
    if(fs>=0){
      frameBatch.setMatrix(fs,[sc,0,0,0, 0,sc,0,0, 0,0,1,0, cx,cy,0,1]);
      frameBatch.setColor(fs,0x4ade80);
      refFrameSlots.push(fs);
    }
  }
  buildSpatialPortals(rig);
}

function createExtraVisual(tagId,edgeMm,lFromTag){
  const s=Math.max(0.55,Math.min(2.2,edgeMm/20));
  const color=EXTRA_COLORS[tagId%EXTRA_COLORS.length];
  const colorCss=`#${color.toString(16).padStart(6,'0')}`;
  const frameSlot=frameBatch.alloc();
  if(frameSlot>=0){
    setTagFramePose(frameBatch,frameSlot,lFromTag,0,edgeMm/20);
    frameBatch.setColor(frameSlot,color);
  }
  const alertSlot=alertGlowBatch.alloc();
  if(alertSlot>=0)alertGlowBatch.setVisible(alertSlot,false);
  const axesSlot=axesBatch.alloc();
  if(axesSlot>=0){
    axesBatch.setPose(axesSlot,lFromTag,0.025*s);
  }
  const faceSlot=tagFaceAtlas.acquire();
  if(faceSlot>=0){
    const faceEdge=Math.max(0.008,edgeMm/1000-0.0006);
    drawTagMatrixFace(faceSlot,`extra:${tagId}:${edgeMm}:${colorCss}`,`${tagId}`,colorCss);
    tagFaceMesh.setOrientedQuad(faceSlot,lFromTag,faceEdge,faceEdge,0,true);
  }
  return {frameSlot,alertSlot,axesSlot,faceSlot,colorCss,outsidePerimeter:false};
}

function setRodSegment(mesh,ax,ay,az,bx,by,bz,radius=0.0018){
  _vA.set(ax,ay,az);_vB.set(bx,by,bz);
  _vDir.subVectors(_vB,_vA);
  const len=_vDir.length();
  if(len<1e-4){mesh.visible=false;return;}
  mesh.visible=true;
  mesh.position.addVectors(_vA,_vB).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(_upVec,_vDir.divideScalar(len));
  mesh.scale.set(radius,len,radius);
}

// Dynamic clockwise 20 cm translucent perimeter fence (L-angle 350-352 + movable markers 353-359).
// Stays hidden until TL (353) is found ('offered') and the user presses TL to activate ('active').
// Pre-allocated geometry buffers guarantee zero GC churn during live scanning and 90 FPS idle anchoring.
const MAX_FENCE_SEGS=12;
const fenceGroup=new THREE.Group();
fenceGroup.visible=false;
root.add(fenceGroup);
const perimeterTagVisuals=new Map();
let perimeterFence=null;
let perimeterOfferSuppressed=false;
let fenceRiseStartMs=0;
let perimeterCompleteAnnounced=false;

function makeFenceBandTexture(){
  const c=document.createElement('canvas');c.width=64;c.height=256;
  const ctx=c.getContext('2d');
  // Canvas y=0 corresponds to UV v=1 (top rim at z=20 cm), y=256 corresponds to UV v=0 (table z=0).
  const grad=ctx.createLinearGradient(0,0,0,256);
  grad.addColorStop(0.00,'rgba(186,230,253,0.72)');
  grad.addColorStop(0.04,'rgba(56,189,248,0.46)');
  grad.addColorStop(0.14,'rgba(34,211,238,0.14)');
  grad.addColorStop(0.50,'rgba(34,211,238,0.09)');
  grad.addColorStop(0.88,'rgba(34,211,238,0.15)');
  grad.addColorStop(0.96,'rgba(56,189,248,0.38)');
  grad.addColorStop(1.00,'rgba(125,211,252,0.58)');
  ctx.fillStyle=grad;
  ctx.fillRect(0,0,64,256);
  // Subtle 5 cm horizontal elevation guide lines so the 20 cm band reads clearly in passthrough.
  ctx.fillStyle='rgba(125,211,252,0.26)';
  for(const frac of [0.25,0.50,0.75])ctx.fillRect(0,Math.round(256*frac),64,2);
  // Light vertical mesh hairlines repeating every ~25 cm along the perimeter.
  ctx.fillStyle='rgba(125,211,252,0.16)';
  ctx.fillRect(0,0,1,256);
  ctx.fillRect(32,0,1,256);
  const tex=new THREE.CanvasTexture(c);
  tex.wrapS=THREE.RepeatWrapping;
  tex.wrapT=THREE.ClampToEdgeWrapping;
  tex.needsUpdate=true;
  return tex;
}

const MAX_FENCE_COLS=224;
const FENCE_WALL_ROWS=8;
const FENCE_WALL_VROWS=FENCE_WALL_ROWS+1;
const MAX_FENCE_WALL_VERTS=(MAX_FENCE_COLS+1)*FENCE_WALL_VROWS;
const MAX_FENCE_WALL_IDX=MAX_FENCE_COLS*FENCE_WALL_ROWS*6;
const MAX_FENCE_BREACHES=6;
const MAX_FENCE_PROX=2;
const FENCE_BREACH_OUT_LIFE_S=3.4;
const FENCE_BREACH_IN_LIFE_S=1.1;

const fenceFxUniforms={
  uTime:frameShimmerUniforms.uTime,
  uPerimeterM:{value:1.0},
  uFenceHeight:{value:FENCE_HEIGHT_M},
  uFxActive:{value:0.0},
  uBreachA:{value:Array.from({length:MAX_FENCE_BREACHES},()=>new THREE.Vector4(0,0,-999,0))},
  uBreachB:{value:Array.from({length:MAX_FENCE_BREACHES},()=>new THREE.Vector4(1,1,0,0))},
  uProx:{value:Array.from({length:MAX_FENCE_PROX},()=>new THREE.Vector4(0,0,0,0))},
};

function attachFenceFxUniforms(shader){
  shader.uniforms.uTime=fenceFxUniforms.uTime;
  shader.uniforms.uPerimeterM=fenceFxUniforms.uPerimeterM;
  shader.uniforms.uFenceHeight=fenceFxUniforms.uFenceHeight;
  shader.uniforms.uFxActive=fenceFxUniforms.uFxActive;
  shader.uniforms.uBreachA=fenceFxUniforms.uBreachA;
  shader.uniforms.uBreachB=fenceFxUniforms.uBreachB;
  shader.uniforms.uProx=fenceFxUniforms.uProx;
}

const FENCE_VERT_PARS=`#include <common>
uniform float uTime;
uniform float uPerimeterM;
uniform float uFenceHeight;
uniform float uFxActive;
uniform vec4 uBreachA[6];
uniform vec4 uBreachB[6];
uniform vec4 uProx[2];
attribute vec2 aWallNormal;
attribute float aArcS;
varying float vArcS;
varying float vLocalZ;
varying float vDisp;`;

const FENCE_VERT_BODY=`#include <begin_vertex>
vArcS = aArcS;
vLocalZ = position.z;
vDisp = 0.0;
if (uFxActive > 0.5) {
  float hFrac = clamp(position.z / max(uFenceHeight, 0.01), 0.0, 1.0);
  float pinMask = smoothstep(0.015, 0.16, hFrac);
  float whip = 1.0 + 0.30 * hFrac;
  float disp = 0.0;
  for (int j = 0; j < 2; j++) {
    vec4 pr = uProx[j];
    if (pr.z > 0.005) {
      float absDsP = abs(aArcS - pr.x);
      float dsP = min(absDsP, max(0.0, uPerimeterM - absDsP));
      float dzP = position.z - pr.y;
      float d2P = dsP * dsP + dzP * dzP;
      disp += 0.015 * pr.z * exp(-d2P / (2.0 * 0.065 * 0.065));
    }
  }
  for (int i = 0; i < 6; i++) {
    vec4 bA = uBreachA[i];
    vec4 bB = uBreachB[i];
    float life = bA.w;
    float age = uTime - bA.z;
    if (life > 0.0 && age >= 0.0 && age < life) {
      float dir = bB.x;
      float ampScale = bB.y;
      float absDs = abs(aArcS - bA.x);
      float ds = min(absDs, max(0.0, uPerimeterM - absDs));
      float dz = position.z - bA.y;
      float d2 = ds * ds + dz * dz;
      float dist = sqrt(d2);
      float envOn = smoothstep(0.0, 0.035, age);
      float wobble = exp(-age * 3.1) * sin(age * 27.0) + exp(-age * 9.5) * envOn * 0.45;
      disp += 0.040 * dir * ampScale * exp(-d2 / (2.0 * 0.095 * 0.095)) * wobble;
      float waveDelta = dist - age * 0.62;
      float waveEnv = exp(-(waveDelta * waveDelta) / (2.0 * 0.042 * 0.042)) * exp(-age * 2.9) * smoothstep(0.0, 0.04, age);
      disp += 0.014 * dir * ampScale * waveEnv * sin(waveDelta * 52.0);
      if (dir > 0.0 && age > life - 0.35) {
        float sealAge = age - (life - 0.35);
        float sealEnv = smoothstep(0.0, 0.04, sealAge) * (1.0 - smoothstep(0.22, 0.35, sealAge));
        disp += -0.008 * sealEnv * exp(-d2 / (2.0 * 0.075 * 0.075)) * exp(-sealAge * 10.0) * sin(sealAge * 36.0);
      }
    }
  }
  float finalDisp = clamp(disp * pinMask * whip, -0.065, 0.065);
  transformed.xy += aWallNormal * finalDisp;
  vDisp = finalDisp;
}`;

const fenceBreachSlots=Array.from({length:MAX_FENCE_BREACHES},()=>({
  active:false,tagId:-1,startSec:-999,lifeSec:0,arcS:0,z:0,dir:1,intensity:1,overTop:0,
}));
const fenceProxTargets=Array.from({length:MAX_FENCE_PROX},()=>({arcS:0,z:0,strength:0,overTop:0}));
const fenceProxSmooth=Array.from({length:MAX_FENCE_PROX},()=>({arcS:0,z:0,strength:0,overTop:0}));
let fenceProxTargetCount=0;
let lastBreachSoundMs=0;

function clearFenceBreaches(){
  for(let i=0;i<MAX_FENCE_BREACHES;i++){
    const s=fenceBreachSlots[i];
    s.active=false;s.tagId=-1;s.startSec=-999;s.lifeSec=0;
    fenceFxUniforms.uBreachA.value[i].set(0,0,-999,0);
    fenceFxUniforms.uBreachB.value[i].set(1,1,0,0);
  }
  fenceProxTargetCount=0;
  for(let j=0;j<MAX_FENCE_PROX;j++){
    fenceProxTargets[j].strength=0;
    fenceProxSmooth[j].strength=0;
    fenceFxUniforms.uProx.value[j].set(0,0,0,0);
  }
  fenceFxUniforms.uFxActive.value=0.0;
}

function triggerFenceBreach({tagId=-1,arcS=0,z=0.10,dir=1,intensity=1,overTop=false},nowMs=performance.now()){
  const perim=Math.max(0.05,fenceFxUniforms.uPerimeterM.value);
  const startSec=nowMs*0.001;
  const lifeSec=dir>0?FENCE_BREACH_OUT_LIFE_S:FENCE_BREACH_IN_LIFE_S;
  const clampedInt=Math.max(0.8,Math.min(1.35,Number.isFinite(intensity)?intensity:1));
  const overTopF=overTop?1:0;

  let slotIdx=-1;
  for(let i=0;i<MAX_FENCE_BREACHES;i++){
    const s=fenceBreachSlots[i];
    if(!s.active)continue;
    const age=startSec-s.startSec;
    if(age>=s.lifeSec){s.active=false;continue;}
    if(s.tagId===tagId&&age<0.80){
      let dArc=Math.abs(s.arcS-arcS);
      if(dArc>perim*0.5)dArc=Math.max(0,perim-dArc);
      if(dArc<0.10){slotIdx=i;break;}
    }
  }
  if(slotIdx<0){
    for(let i=0;i<MAX_FENCE_BREACHES;i++){
      if(!fenceBreachSlots[i].active){slotIdx=i;break;}
    }
  }
  if(slotIdx<0){
    let oldestStart=Infinity;
    for(let i=0;i<MAX_FENCE_BREACHES;i++){
      if(fenceBreachSlots[i].startSec<oldestStart){
        oldestStart=fenceBreachSlots[i].startSec;
        slotIdx=i;
      }
    }
  }
  if(slotIdx<0)slotIdx=0;

  const s=fenceBreachSlots[slotIdx];
  s.active=true;s.tagId=tagId;s.startSec=startSec;s.lifeSec=lifeSec;
  s.arcS=arcS;s.z=z;s.dir=dir>0?1:-1;s.intensity=clampedInt;s.overTop=overTopF;
  fenceFxUniforms.uBreachA.value[slotIdx].set(arcS,z,startSec,lifeSec);
  fenceFxUniforms.uBreachB.value[slotIdx].set(s.dir,clampedInt,overTopF,0);
  fenceFxUniforms.uFxActive.value=1.0;

  if(nowMs-lastBreachSoundMs>=220){
    lastBreachSoundMs=nowMs;
    playModeTick(s.dir>0?'breach-out':'breach-in');
  }
}

function beginFenceProximityFrame(){
  fenceProxTargetCount=0;
}

function recordFenceProximity(arcS,z,strength,overTop=false){
  if(!(strength>0.01))return;
  const ot=overTop?1:0;
  if(fenceProxTargetCount<MAX_FENCE_PROX){
    const t=fenceProxTargets[fenceProxTargetCount++];
    t.arcS=arcS;t.z=z;t.strength=strength;t.overTop=ot;
    return;
  }
  let weakIdx=0;
  for(let j=1;j<MAX_FENCE_PROX;j++){
    if(fenceProxTargets[j].strength<fenceProxTargets[weakIdx].strength)weakIdx=j;
  }
  if(strength>fenceProxTargets[weakIdx].strength){
    const t=fenceProxTargets[weakIdx];
    t.arcS=arcS;t.z=z;t.strength=strength;t.overTop=ot;
  }
}

function flushFenceFxUniforms(nowMs,dtSec){
  const nowSec=nowMs*0.001;
  const dt=Math.min(0.05,Math.max(0.004,dtSec||1/90));
  const alphaIn=1-Math.exp(-dt*18);
  const alphaOut=1-Math.exp(-dt*10);
  const perim=Math.max(0.05,fenceFxUniforms.uPerimeterM.value);

  let anyActive=false;
  for(let i=0;i<MAX_FENCE_BREACHES;i++){
    const s=fenceBreachSlots[i];
    if(!s.active)continue;
    if(nowSec-s.startSec>=s.lifeSec){
      s.active=false;
      fenceFxUniforms.uBreachA.value[i].w=0;
    }else{
      anyActive=true;
    }
  }
  for(let j=0;j<MAX_FENCE_PROX;j++){
    const cur=fenceProxSmooth[j];
    if(j<fenceProxTargetCount){
      const tgt=fenceProxTargets[j];
      if(cur.strength<=0.005){
        cur.arcS=tgt.arcS;cur.z=tgt.z;
      }else{
        let dArc=tgt.arcS-cur.arcS;
        if(dArc>perim*0.5)dArc-=perim;
        if(dArc<-perim*0.5)dArc+=perim;
        if(Math.abs(dArc)>0.25){
          cur.arcS=tgt.arcS;cur.z=tgt.z;
        }else{
          cur.arcS=((cur.arcS+dArc*alphaIn)%perim+perim)%perim;
          cur.z+=(tgt.z-cur.z)*alphaIn;
        }
      }
      cur.strength+=(tgt.strength-cur.strength)*alphaIn;
      cur.overTop=tgt.overTop;
    }else{
      cur.strength+=(0-cur.strength)*alphaOut;
      if(cur.strength<0.005)cur.strength=0;
    }
    fenceFxUniforms.uProx.value[j].set(cur.arcS,cur.z,cur.strength,cur.overTop);
    if(cur.strength>0.005)anyActive=true;
  }
  fenceFxUniforms.uFxActive.value=anyActive?1.0:0.0;
}

const fenceWallPos=new Float32Array(MAX_FENCE_WALL_VERTS*3);
const fenceWallUv=new Float32Array(MAX_FENCE_WALL_VERTS*2);
const fenceWallNorm=new Float32Array(MAX_FENCE_WALL_VERTS*2);
const fenceWallArcS=new Float32Array(MAX_FENCE_WALL_VERTS);
const fenceWallIdx=new Uint16Array(MAX_FENCE_WALL_IDX);
const fenceWallGeo=new THREE.BufferGeometry();
const fenceWallPosAttr=new THREE.BufferAttribute(fenceWallPos,3);
const fenceWallUvAttr=new THREE.BufferAttribute(fenceWallUv,2);
const fenceWallNormAttr=new THREE.BufferAttribute(fenceWallNorm,2);
const fenceWallArcAttr=new THREE.BufferAttribute(fenceWallArcS,1);
const fenceWallIdxAttr=new THREE.BufferAttribute(fenceWallIdx,1);
fenceWallGeo.setAttribute('position',fenceWallPosAttr);
fenceWallGeo.setAttribute('uv',fenceWallUvAttr);
fenceWallGeo.setAttribute('aWallNormal',fenceWallNormAttr);
fenceWallGeo.setAttribute('aArcS',fenceWallArcAttr);
fenceWallGeo.setIndex(fenceWallIdxAttr);
fenceWallGeo.setDrawRange(0,0);

const fenceWallMat=new THREE.MeshBasicMaterial({
  map:makeFenceBandTexture(),
  transparent:true,
  opacity:0.94,
  side:THREE.DoubleSide,
  depthWrite:false,
});
fenceWallMat.customProgramCacheKey=()=>'fence-wall-membrane-v1';
fenceWallMat.onBeforeCompile=shader=>{
  attachFenceFxUniforms(shader);
  shader.vertexShader=shader.vertexShader
    .replace('#include <common>',FENCE_VERT_PARS)
    .replace('#include <begin_vertex>',FENCE_VERT_BODY);
  shader.fragmentShader=shader.fragmentShader
    .replace(
      '#include <common>',
      `#include <common>
uniform float uTime;
uniform float uPerimeterM;
uniform float uFenceHeight;
uniform float uFxActive;
uniform vec4 uBreachA[6];
uniform vec4 uBreachB[6];
uniform vec4 uProx[2];
varying float vArcS;
varying float vLocalZ;
varying float vDisp;
float fenceHexEdge(vec2 uv) {
  vec2 r = vec2(1.0, 1.7320508);
  vec2 h = r * 0.5;
  vec2 a = mod(uv, r) - h;
  vec2 b = mod(uv - h, r) - h;
  vec2 gv = dot(a, a) < dot(b, b) ? a : b;
  vec2 p = abs(gv);
  return 0.5 - max(dot(p, vec2(0.5, 0.8660254)), p.x);
}`,
    )
    .replace(
      '#include <color_fragment>',
      `#include <color_fragment>
if (uFxActive > 0.5) {
  float darkMask = 0.0;
  vec3 darkRgb = vec3(0.07, 0.01, 0.02);
  vec3 addRgb = vec3(0.0);
  float addAlpha = 0.0;
  float hexReveal = 0.0;
  vec3 hexTint = vec3(0.20, 0.90, 1.0);
  float hexWeight = 0.0;

  for (int j = 0; j < 2; j++) {
    vec4 pr = uProx[j];
    if (pr.z > 0.01) {
      float rawDsP = vArcS - pr.x;
      if (rawDsP > 0.5 * uPerimeterM) rawDsP -= uPerimeterM;
      if (rawDsP < -0.5 * uPerimeterM) rawDsP += uPerimeterM;
      float dzP = vLocalZ - pr.y;
      float radP = sqrt(rawDsP * rawDsP + dzP * dzP);
      float haloP = exp(-(radP * radP) / (2.0 * 0.055 * 0.055)) * pr.z;
      float coreP = exp(-(radP * radP) / (2.0 * 0.018 * 0.018)) * pr.z;
      vec3 proxCol = mix(vec3(0.15, 0.92, 1.0), vec3(1.0, 0.65, 0.15), smoothstep(0.25, 0.95, pr.z));
      addRgb += proxCol * (haloP * 0.42 + coreP * 0.55);
      addAlpha += haloP * 0.34 + coreP * 0.36;
      float hP = haloP * 0.90;
      hexReveal += hP;
      if (hP > hexWeight) { hexWeight = hP; hexTint = proxCol; }
    }
  }

  for (int i = 0; i < 6; i++) {
    vec4 bA = uBreachA[i];
    vec4 bB = uBreachB[i];
    float life = bA.w;
    float age = uTime - bA.z;
    if (life > 0.0 && age >= 0.0 && age < life) {
      float dir = bB.x;
      float ampScale = bB.y;
      float rawDs = vArcS - bA.x;
      if (rawDs > 0.5 * uPerimeterM) rawDs -= uPerimeterM;
      if (rawDs < -0.5 * uPerimeterM) rawDs += uPerimeterM;
      float dz = vLocalZ - bA.y;
      float rad = sqrt(rawDs * rawDs + dz * dz);
      float ang = (rad > 1e-4) ? atan(dz, rawDs) : 0.0;

      if (dir > 0.0) {
        float shockR = 0.014 + age * 0.60;
        float shockBand = smoothstep(0.026, 0.0, abs(rad - shockR)) * exp(-age * 4.5);
        float shockHex = smoothstep(0.055, 0.0, abs(rad - shockR)) * exp(-age * 3.8);
        addRgb += vec3(1.0, 0.72, 0.32) * shockBand * 1.05;
        addAlpha += shockBand * 0.70;
        float hShock = shockHex * 1.20 * ampScale;
        hexReveal += hShock;

        float flash = exp(-(rad * rad) / (2.0 * 0.030 * 0.030)) * exp(-age * 11.5);
        addRgb += vec3(1.0, 0.95, 0.82) * flash * 1.15;
        addAlpha += flash * 0.80;

        float openT = smoothstep(0.0, 0.14, age);
        float elasticPop = 1.0 + 0.24 * exp(-age * 7.5) * sin(age * 18.0);
        float healT = smoothstep(life - 0.95, life - 0.10, age);
        float closeCurve = 1.0 - healT;
        float radiusScale = openT * elasticPop * closeCurve;
        float portalVis = 1.0 - smoothstep(life - 0.14, life - 0.02, age);
        float rimNoise = 1.0 + 0.11 * sin(ang * 5.0 - uTime * 7.0) + 0.06 * cos(ang * 9.0 + uTime * 11.0);
        float baseR = 0.027 * clamp(ampScale, 0.85, 1.25);
        float portalR = max(0.002, baseR * radiusScale * rimNoise);

        float voidMask = (1.0 - smoothstep(portalR * 0.65, portalR * 1.02, rad)) * portalVis;
        float swirl = 0.5 + 0.5 * sin(ang * 3.0 + rad * 120.0 - uTime * 9.0);
        vec3 voidCol = mix(vec3(0.07, 0.01, 0.02), vec3(0.02, 0.11, 0.13), healT);
        voidCol += mix(vec3(0.16, 0.02, 0.01), vec3(0.02, 0.20, 0.22), healT) * swirl * 0.55;
        if (voidMask > 0.005) {
          float wMask = voidMask * 0.90;
          if (wMask > darkMask) { darkMask = wMask; darkRgb = voidCol; }
        }

        float contrastOutline = smoothstep(0.022, 0.008, abs(rad - portalR)) * portalVis * 0.42;
        if (contrastOutline > darkMask) {
          darkMask = contrastOutline;
          darkRgb = mix(vec3(0.10, 0.01, 0.01), vec3(0.01, 0.08, 0.10), healT);
        }

        float rimPulse = 0.82 + 0.18 * sin(uTime * 10.5 - rad * 40.0);
        float rimOuter = smoothstep(0.018, 0.002, abs(rad - portalR)) * portalVis;
        float rimCore = smoothstep(0.006, 0.0, abs(rad - portalR)) * portalVis;
        vec3 warnRim = vec3(1.0, 0.24, 0.08);
        vec3 healRim = vec3(0.10, 0.96, 0.72);
        vec3 rimCol = mix(warnRim, healRim, healT);
        vec3 rimHot = mix(vec3(1.0, 0.84, 0.40), vec3(0.84, 1.0, 0.96), healT);
        addRgb += (rimCol * rimOuter * 0.95 + rimHot * rimCore * 0.95) * rimPulse;
        addAlpha += (rimOuter * 0.82 + rimCore * 0.55) * rimPulse;

        float aura = exp(-(rad * rad) / (2.0 * 0.065 * 0.065)) * (1.0 - smoothstep(life - 0.35, life, age));
        float hAura = aura * (0.95 + 0.45 * healT) + hShock;
        hexReveal += aura * (0.95 + 0.45 * healT);
        if (hAura > hexWeight) { hexWeight = hAura; hexTint = mix(vec3(1.0, 0.34, 0.12), healRim, healT); }

        if (age > life - 0.34) {
          float sealWindow = smoothstep(life - 0.34, life - 0.20, age) * (1.0 - smoothstep(life - 0.14, life, age));
          float sealRingR = 0.004 + max(0.0, age - (life - 0.30)) * 0.22;
          float sealRing = smoothstep(0.012, 0.0, abs(rad - sealRingR)) * sealWindow;
          float sealDot = exp(-(rad * rad) / (2.0 * 0.015 * 0.015)) * sealWindow;
          addRgb += vec3(0.45, 1.0, 0.92) * (sealRing * 0.95 + sealDot * 1.05);
          addAlpha += sealRing * 0.65 + sealDot * 0.75;
        }
      } else {
        float rippleR = 0.010 + age * 0.48;
        float ripple = smoothstep(0.022, 0.0, abs(rad - rippleR)) * exp(-age * 4.2);
        float glowIn = exp(-(rad * rad) / (2.0 * 0.036 * 0.036)) * exp(-age * 4.8);
        vec3 inCol = vec3(0.15, 0.96, 0.72);
        addRgb += inCol * (ripple * 0.85 + glowIn * 0.65) + vec3(0.75, 1.0, 0.95) * glowIn * 0.35;
        addAlpha += ripple * 0.60 + glowIn * 0.48;
        float hIn = ripple * 0.95 + glowIn * 0.70;
        hexReveal += hIn;
        if (hIn > hexWeight) { hexWeight = hIn; hexTint = inCol; }
      }
    }
  }

  if (hexReveal > 0.01) {
    float hexDist = fenceHexEdge(vec2(vArcS, vLocalZ) / 0.022);
    float hexLine = 1.0 - smoothstep(0.02, 0.11, hexDist);
    float hexFill = (1.0 - hexLine) * 0.14;
    float hexEnergy = clamp(hexReveal, 0.0, 1.35) * (hexLine * 0.95 + hexFill);
    addRgb += hexTint * hexEnergy * 0.78;
    addAlpha += hexEnergy * 0.42;
  }

  float stretch = clamp(abs(vDisp) / 0.040, 0.0, 1.0);
  if (stretch > 0.02) {
    addRgb += vec3(0.45, 0.92, 1.0) * stretch * 0.24;
    addAlpha += stretch * 0.18;
  }

  if (darkMask > 0.001) {
    diffuseColor.rgb = mix(diffuseColor.rgb, darkRgb, clamp(darkMask, 0.0, 0.92));
    diffuseColor.a = max(diffuseColor.a, clamp(darkMask * 0.80, 0.0, 0.88));
  }
  diffuseColor.rgb = clamp(diffuseColor.rgb + addRgb, 0.0, 1.0);
  diffuseColor.a = clamp(diffuseColor.a + addAlpha, 0.0, 0.98);
}`,
    );
};

const fenceWallMesh=new THREE.Mesh(fenceWallGeo,fenceWallMat);
fenceWallMesh.frustumCulled=false;
fenceWallMesh.renderOrder=4;
fenceGroup.add(fenceWallMesh);

const fenceTopPos=new Float32Array(MAX_FENCE_COLS*3);
const fenceTopNorm=new Float32Array(MAX_FENCE_COLS*2);
const fenceTopArcS=new Float32Array(MAX_FENCE_COLS);
const fenceTopGeo=new THREE.BufferGeometry();
const fenceTopAttr=new THREE.BufferAttribute(fenceTopPos,3);
const fenceTopNormAttr=new THREE.BufferAttribute(fenceTopNorm,2);
const fenceTopArcAttr=new THREE.BufferAttribute(fenceTopArcS,1);
fenceTopGeo.setAttribute('position',fenceTopAttr);
fenceTopGeo.setAttribute('aWallNormal',fenceTopNormAttr);
fenceTopGeo.setAttribute('aArcS',fenceTopArcAttr);
fenceTopGeo.setDrawRange(0,0);

const fenceTopMat=new THREE.LineBasicMaterial({color:0x7dd3fc,transparent:true,opacity:0.92});
fenceTopMat.customProgramCacheKey=()=>'fence-top-membrane-v1';
fenceTopMat.onBeforeCompile=shader=>{
  attachFenceFxUniforms(shader);
  shader.vertexShader=shader.vertexShader
    .replace('#include <common>',FENCE_VERT_PARS)
    .replace('#include <begin_vertex>',FENCE_VERT_BODY);
  shader.fragmentShader=shader.fragmentShader
    .replace(
      '#include <common>',
      `#include <common>
uniform float uTime;
uniform float uPerimeterM;
uniform float uFenceHeight;
uniform float uFxActive;
uniform vec4 uBreachA[6];
uniform vec4 uBreachB[6];
uniform vec4 uProx[2];
varying float vArcS;
varying float vLocalZ;
varying float vDisp;`,
    )
    .replace(
      '#include <color_fragment>',
      `#include <color_fragment>
if (uFxActive > 0.5) {
  vec3 topAdd = vec3(0.0);
  float topAlpha = 0.0;
  for (int j = 0; j < 2; j++) {
    vec4 pr = uProx[j];
    if (pr.z > 0.01) {
      float absDsP = abs(vArcS - pr.x);
      float dsP = min(absDsP, max(0.0, uPerimeterM - absDsP));
      float proxGlow = exp(-(dsP * dsP) / (2.0 * 0.065 * 0.065)) * pr.z * (0.45 + 0.55 * pr.w);
      vec3 pCol = mix(vec3(0.40, 0.95, 1.0), vec3(1.0, 0.72, 0.25), pr.z);
      topAdd += pCol * proxGlow * 0.85;
      topAlpha += proxGlow * 0.15;
    }
  }
  for (int i = 0; i < 6; i++) {
    vec4 bA = uBreachA[i];
    vec4 bB = uBreachB[i];
    float life = bA.w;
    float age = uTime - bA.z;
    if (life > 0.0 && age >= 0.0 && age < life) {
      float dir = bB.x;
      float overTop = bB.z;
      float absDs = abs(vArcS - bA.x);
      float ds = min(absDs, max(0.0, uPerimeterM - absDs));
      float dz = uFenceHeight - bA.y;
      float radTop = sqrt(ds * ds + dz * dz);
      if (dir > 0.0) {
        float healT = smoothstep(life - 0.95, life - 0.10, age);
        float vis = 1.0 - smoothstep(life - 0.28, life, age);
        float topReach = mix(exp(-(dz * dz) / (2.0 * 0.085 * 0.085)), 1.0, overTop);
        float topSig = 0.075 + 0.035 * overTop;
        float arcGlow = exp(-(ds * ds) / (2.0 * topSig * topSig)) * topReach * vis;
        float shockR = 0.014 + age * 0.60;
        float shockHit = smoothstep(0.032, 0.0, abs(radTop - shockR)) * exp(-age * 4.2);
        vec3 baseHot = mix(vec3(1.0, 0.35, 0.10), vec3(1.0, 0.92, 0.72), overTop * 0.45);
        vec3 col = mix(baseHot, vec3(0.18, 1.0, 0.80), healT);
        topAdd += col * arcGlow * 1.15 + vec3(1.0, 0.85, 0.48) * shockHit * 0.95;
        topAlpha += arcGlow * 0.10 + shockHit * 0.08;
      } else {
        float rippleR = 0.010 + age * 0.48;
        float rip = smoothstep(0.028, 0.0, abs(radTop - rippleR)) * exp(-age * 4.2);
        topAdd += vec3(0.20, 1.0, 0.76) * rip * 0.85;
        topAlpha += rip * 0.08;
      }
    }
  }
  float stretch = clamp(abs(vDisp) / 0.035, 0.0, 1.0);
  topAdd += vec3(0.45, 0.95, 1.0) * stretch * 0.35;
  diffuseColor.rgb = clamp(diffuseColor.rgb + topAdd, 0.0, 1.0);
  diffuseColor.a = clamp(diffuseColor.a + topAlpha, 0.0, 1.0);
}`,
    );
};

const fenceTopLoop=new THREE.LineLoop(fenceTopGeo,fenceTopMat);
fenceTopLoop.frustumCulled=false;
fenceTopLoop.renderOrder=5;
fenceGroup.add(fenceTopLoop);

const _segLens=new Float32Array(MAX_FENCE_SEGS);
const _edgeNx=new Float32Array(MAX_FENCE_SEGS);
const _edgeNy=new Float32Array(MAX_FENCE_SEGS);
const _cornerNx=new Float32Array(MAX_FENCE_SEGS);
const _cornerNy=new Float32Array(MAX_FENCE_SEGS);
const _segCols=new Int16Array(MAX_FENCE_SEGS);

const fenceBotPos=new Float32Array(MAX_FENCE_SEGS*3);
const fenceBotGeo=new THREE.BufferGeometry();
const fenceBotAttr=new THREE.BufferAttribute(fenceBotPos,3);
fenceBotGeo.setAttribute('position',fenceBotAttr);
fenceBotGeo.setDrawRange(0,0);
const fenceBotLoop=new THREE.LineLoop(
  fenceBotGeo,
  new THREE.LineBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.78}),
);
fenceBotLoop.frustumCulled=false;
fenceBotLoop.renderOrder=5;
fenceGroup.add(fenceBotLoop);

const fencePostCache=new Map();

function getOrCreatePerimeterTagVisual(m){
  let vis=perimeterTagVisuals.get(m.id);
  if(vis)return vis;
  const frameSlot=frameBatch.alloc();
  if(frameSlot>=0)frameBatch.setColor(frameSlot,0x22d3ee);
  const axesSlot=axesBatch.alloc();
  const badgeSlot=badgeAtlas.acquire();
  if(badgeSlot>=0){
    drawStaticBadgeAtlas(badgeSlot,`perim:${m.role}`,m.role,'#22d3ee');
  }
  const wp={
    isStatic:true,isPerimeter:true,perimeterRole:m.role,tagId:m.id,edgeMm:20,index:0,
    matrix:m.matrix.slice(),centerL:[m.pos[0],m.pos[1],0],
    xMm:m.pos[0]*1000,yMm:m.pos[1]*1000,zMm:0,tiltDeg:0,atMs:null,elapsedS:0,
    clock:new Date().toTimeString().slice(0,8),
  };
  vis={id:m.id,role:m.role,frameSlot,axesSlot,badgeSlot,visible:false,hot:false,wp};
  perimeterTagVisuals.set(m.id,vis);
  return vis;
}

function isPerimeterBadgeVisible(vis){
  const k=waypointIdKey(vis.wp);
  const tlBlocked=Boolean(perimeterFence?.isOffered()&&!perimeterOfferSuppressed&&vis.id===perimeterFence.triggerTagId);
  return !(hoveredWaypoint===vis.wp||hoveredWaypoint2===vis.wp||pinnedPoints.has(k)||tlBlocked);
}

let lastSyncedFenceVer=-1,lastSyncedFenceScan=false;
function syncPerimeterTagVisuals(now){
  const curVer=perimeterFence?.version?.()??-1;
  if(curVer===lastSyncedFenceVer&&!extraScanActive&&!lastSyncedFenceScan)return;
  lastSyncedFenceVer=curVer;
  lastSyncedFenceScan=Boolean(extraScanActive);
  const discovered=perimeterFence?.discoveredMarkers?.()??[];
  const activeIds=new Set();
  for(const m of discovered){
    activeIds.add(m.id);
    const vis=getOrCreatePerimeterTagVisual(m);
    vis.visible=true;
    setTagFramePose(frameBatch,vis.frameSlot,m.matrix,0,1);
    frameBatch.setVisible(vis.frameSlot,true);
    axesBatch.setPose(vis.axesSlot,m.matrix,0.025);
    axesBatch.setVisible(vis.axesSlot,true);
    const isHot=Boolean(extraScanActive&&m.lastAt>0&&now-m.lastAt<480);
    if(vis.hot!==isHot){
      vis.hot=isHot;
      frameBatch.setColor(vis.frameSlot,isHot?0xfbbf24:0x22d3ee);
    }
    const prev=vis.wp.matrix;
    const matMoved=prev.some((v,i)=>Math.abs(v-m.matrix[i])>0.0005);
    vis.wp.matrix=m.matrix.slice();
    vis.wp.centerL=[m.pos[0],m.pos[1],0];
    vis.wp.xMm=m.pos[0]*1000;
    vis.wp.yMm=m.pos[1]*1000;
    if(matMoved)syncPinnedWaypointPose(vis.wp);
    badgeMesh.setQuad(vis.badgeSlot,localZOffset(m.matrix,0.018),0.026,0.012,isPerimeterBadgeVisible(vis));
  }
  for(const [id,vis] of perimeterTagVisuals){
    if(!activeIds.has(id)){
      vis.visible=false;
      frameBatch.setVisible(vis.frameSlot,false);
      axesBatch.setVisible(vis.axesSlot,false);
      badgeMesh.setQuadVisible(vis.badgeSlot,false);
    }
  }
}

function getOrCreateFencePost(v){
  let post=fencePostCache.get(v.id);
  if(post)return post;
  const baseHex=v.fixed?0x4ade80:0x22d3ee;
  const rodSlot=rodBatch.alloc();
  if(rodSlot>=0)rodBatch.setColor(rodSlot,baseHex);
  const capSlot=nodeBatch.alloc();
  if(capSlot>=0)nodeBatch.setColor(capSlot,baseHex);
  const badgeSlot=badgeAtlas.acquire();
  if(badgeSlot>=0){
    drawStaticBadgeAtlas(badgeSlot,`fpost:${v.id}:${v.role}:${v.fixed?1:0}`,v.role,v.fixed?'#4ade80':'#22d3ee');
  }
  post={id:v.id,fixed:v.fixed,rodSlot,capSlot,badgeSlot,hot:false};
  fencePostCache.set(v.id,post);
  return post;
}

const _alertPosBuf=[0,0,0];
let lastExtraAlertFenceVer=-1;
function syncStaticExtrasPerimeterAlerts(force=false){
  const curVer=perimeterFence?.version?.()??-1;
  if(!force&&curVer===lastExtraAlertFenceVer)return;
  lastExtraAlertFenceVer=curVer;
  const fenceActive=Boolean(perimeterFence?.isActive?.());
  for(const extra of placedExtras.values()){
    let nextOutside=false;
    if(fenceActive&&extra.lFromTag){
      _alertPosBuf[0]=extra.lFromTag[12];
      _alertPosBuf[1]=extra.lFromTag[13];
      _alertPosBuf[2]=extra.lFromTag[14];
      nextOutside=perimeterFence.isOutsideXY(_alertPosBuf,{wasOutside:Boolean(extra.outsidePerimeter)}).outside;
    }
    if(nextOutside!==Boolean(extra.outsidePerimeter)){
      extra.outsidePerimeter=nextOutside;
      frameBatch.setAlert(extra.frameSlot,nextOutside);
      if(nextOutside&&extra.lFromTag){
        setTagFramePose(alertGlowBatch,extra.alertSlot,extra.lFromTag,0.0008,extra.edgeMm/20);
        alertGlowBatch.setVisible(extra.alertSlot,true);
        drawTagMatrixFace(extra.faceSlot,`extra:${extra.tagId}:${extra.edgeMm}:alarm`,`${extra.tagId}`,'#ff3b30',{fill:'rgba(138,12,16,0.95)',alarmBell:true});
      }else{
        alertGlowBatch.setVisible(extra.alertSlot,false);
        drawTagMatrixFace(extra.faceSlot,`extra:${extra.tagId}:${extra.edgeMm}:${extra.colorCss}`,`${extra.tagId}`,extra.colorCss);
      }
    }else if(nextOutside&&extra.lFromTag){
      setTagFramePose(alertGlowBatch,extra.alertSlot,extra.lFromTag,0.0008,extra.edgeMm/20);
      alertGlowBatch.setVisible(extra.alertSlot,true);
    }
  }
}

function syncFenceVisuals(now,dtSec=0,force=false){
  if(!perimeterFence)return;
  const moved=perimeterFence.step(dtSec);
  syncPerimeterTagVisuals(now);
  syncStaticExtrasPerimeterAlerts(force);
  if(!perimeterFence.isActive()){
    fenceGroup.visible=false;
    clearFenceBreaches();
    for(const post of fencePostCache.values()){
      rodBatch.setVisible(post.rodSlot,false);
      nodeBatch.setVisible(post.capSlot,false);
      badgeMesh.setQuadVisible(post.badgeSlot,false);
    }
    return;
  }
  fenceGroup.visible=true;

  const rawRise=fenceRiseStartMs>0?Math.min(1,Math.max(0,(now-fenceRiseStartMs)/360)):1;
  const easedRise=1-(1-rawRise)**3;
  const animatingRise=rawRise<1;
  if(!animatingRise)fenceRiseStartMs=0;

  const verts=perimeterFence.orderedVertices();
  let hotChanged=false;
  for(const v of verts){
    if(v.fixed)continue;
    const isHot=Boolean(extraScanActive&&v.lastAt>0&&now-v.lastAt<480);
    const cached=fencePostCache.get(v.id);
    if(!cached||cached.hot!==isHot)hotChanged=true;
  }
  if(!moved&&!hotChanged&&!animatingRise&&!force)return;

  const n=Math.min(verts.length,MAX_FENCE_SEGS);
  const h=Math.max(0.004,perimeterFence.heightM*easedRise);
  const activeIds=new Set();

  let totalPerim=0;
  let signedArea2=0;
  for(let i=0;i<n;i++){
    const a=verts[i],b=verts[(i+1)%n];
    const [ax,ay]=a.pos,[bx,by]=b.pos;
    activeIds.add(a.id);

    fenceBotPos[i*3]=ax;fenceBotPos[i*3+1]=ay;fenceBotPos[i*3+2]=0.002;
    signedArea2+=ax*by-bx*ay;
    const segLen=Math.hypot(bx-ax,by-ay);
    _segLens[i]=segLen;
    totalPerim+=segLen;

    const post=getOrCreateFencePost(a);
    rodBatch.setRod(post.rodSlot,ax,ay,0.001,ax,ay,h,a.fixed?0.0018:0.0022);
    rodBatch.setVisible(post.rodSlot,true);
    nodeBatch.setPoint(post.capSlot,ax,ay,h,0.93);
    nodeBatch.setVisible(post.capSlot,true);
    badgeMesh.setQuad(post.badgeSlot,[ax,ay,h+0.016],0.028,0.0125,true);
    if(!a.fixed){
      const isHot=Boolean(extraScanActive&&a.lastAt>0&&now-a.lastAt<480);
      if(post.hot!==isHot){
        post.hot=isHot;
        const hex=isHot?0xfbbf24:0x22d3ee;
        rodBatch.setColor(post.rodSlot,hex);
        nodeBatch.setColor(post.capSlot,hex);
      }
    }
  }

  for(const [id,post] of fencePostCache){
    if(!activeIds.has(id)){
      rodBatch.setVisible(post.rodSlot,false);
      nodeBatch.setVisible(post.capSlot,false);
      badgeMesh.setQuadVisible(post.badgeSlot,false);
    }
  }

  if(n<2||totalPerim<=1e-6){
    fenceWallGeo.setDrawRange(0,0);
    fenceTopGeo.setDrawRange(0,0);
    fenceBotGeo.setDrawRange(0,0);
    return;
  }

  const windingSign=signedArea2<=0?1:-1;
  for(let i=0;i<n;i++){
    const len=_segLens[i];
    if(len>1e-6){
      const a=verts[i].pos,b=verts[(i+1)%n].pos;
      const tx=(b[0]-a[0])/len,ty=(b[1]-a[1])/len;
      _edgeNx[i]=-ty*windingSign;
      _edgeNy[i]=tx*windingSign;
    }else{
      _edgeNx[i]=1;
      _edgeNy[i]=0;
    }
  }
  for(let i=0;i<n;i++){
    const prev=(i-1+n)%n;
    const nx=_edgeNx[prev]+_edgeNx[i];
    const ny=_edgeNy[prev]+_edgeNy[i];
    const nLen=Math.hypot(nx,ny);
    if(nLen>1e-5){
      _cornerNx[i]=nx/nLen;
      _cornerNy[i]=ny/nLen;
    }else{
      _cornerNx[i]=_edgeNx[i];
      _cornerNy[i]=_edgeNy[i];
    }
  }

  const stepM=Math.max(0.024,totalPerim/Math.max(1,MAX_FENCE_COLS-n));
  let totalCols=0;
  for(let i=0;i<n;i++){
    const remSegs=n-1-i;
    const maxHere=Math.max(1,MAX_FENCE_COLS-totalCols-remSegs);
    const want=Math.max(1,Math.round(_segLens[i]/stepM));
    const cols=Math.min(maxHere,want);
    _segCols[i]=cols;
    totalCols+=cols;
  }

  let col=0;
  let cumDist=0;
  for(let i=0;i<n;i++){
    const a=verts[i].pos,b=verts[(i+1)%n].pos;
    const ax=a[0],ay=a[1],dx=b[0]-ax,dy=b[1]-ay;
    const cols=_segCols[i],segLen=_segLens[i];
    for(let k=0;k<cols;k++){
      const t=k/cols;
      const x=ax+dx*t;
      const y=ay+dy*t;
      const arcS=cumDist+segLen*t;
      const u=arcS*4.0;
      const nx=k===0?_cornerNx[i]:_edgeNx[i];
      const ny=k===0?_cornerNy[i]:_edgeNy[i];
      for(let r=0;r<FENCE_WALL_VROWS;r++){
        const vIdx=col*FENCE_WALL_VROWS+r;
        const vFrac=r/FENCE_WALL_ROWS;
        const z=0.001+(h-0.001)*vFrac;
        fenceWallPos[vIdx*3]=x;fenceWallPos[vIdx*3+1]=y;fenceWallPos[vIdx*3+2]=z;
        fenceWallUv[vIdx*2]=u;fenceWallUv[vIdx*2+1]=vFrac;
        fenceWallNorm[vIdx*2]=nx;fenceWallNorm[vIdx*2+1]=ny;
        fenceWallArcS[vIdx]=arcS;
      }
      fenceTopPos[col*3]=x;fenceTopPos[col*3+1]=y;fenceTopPos[col*3+2]=h;
      fenceTopNorm[col*2]=nx;fenceTopNorm[col*2+1]=ny;
      fenceTopArcS[col]=arcS;
      col++;
    }
    cumDist+=segLen;
  }

  // Closing seam column duplicates vertex 0 with arcS = totalPerim for seamless UV & shader wrapping.
  const closeX=verts[0].pos[0],closeY=verts[0].pos[1];
  const closeNx=_cornerNx[0],closeNy=_cornerNy[0];
  const closeU=totalPerim*4.0;
  for(let r=0;r<FENCE_WALL_VROWS;r++){
    const vIdx=totalCols*FENCE_WALL_VROWS+r;
    const vFrac=r/FENCE_WALL_ROWS;
    const z=0.001+(h-0.001)*vFrac;
    fenceWallPos[vIdx*3]=closeX;fenceWallPos[vIdx*3+1]=closeY;fenceWallPos[vIdx*3+2]=z;
    fenceWallUv[vIdx*2]=closeU;fenceWallUv[vIdx*2+1]=vFrac;
    fenceWallNorm[vIdx*2]=closeNx;fenceWallNorm[vIdx*2+1]=closeNy;
    fenceWallArcS[vIdx]=totalPerim;
  }

  let ii=0;
  for(let c=0;c<totalCols;c++){
    for(let r=0;r<FENCE_WALL_ROWS;r++){
      const b0=c*FENCE_WALL_VROWS+r;
      const b1=(c+1)*FENCE_WALL_VROWS+r;
      fenceWallIdx[ii++]=b0;fenceWallIdx[ii++]=b1;fenceWallIdx[ii++]=b1+1;
      fenceWallIdx[ii++]=b0;fenceWallIdx[ii++]=b1+1;fenceWallIdx[ii++]=b0+1;
    }
  }

  fenceFxUniforms.uPerimeterM.value=Math.max(0.05,totalPerim);
  fenceFxUniforms.uFenceHeight.value=h;

  fenceWallGeo.setDrawRange(0,ii);
  fenceWallPosAttr.needsUpdate=true;
  fenceWallUvAttr.needsUpdate=true;
  fenceWallNormAttr.needsUpdate=true;
  fenceWallArcAttr.needsUpdate=true;
  fenceWallIdxAttr.needsUpdate=true;
  fenceTopGeo.setDrawRange(0,totalCols);
  fenceTopAttr.needsUpdate=true;
  fenceTopNormAttr.needsUpdate=true;
  fenceTopArcAttr.needsUpdate=true;
  fenceBotGeo.setDrawRange(0,n);
  fenceBotAttr.needsUpdate=true;
}

// Spatial activation prompt anchored right over TL (#353) once spotted: press TL to activate the perimeter & scan around the table.
const tlOfferGroup=new THREE.Group();
tlOfferGroup.visible=false;
root.add(tlOfferGroup);
const tlOfferFrameMat=new THREE.MeshBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.95,side:THREE.DoubleSide});
const tlOfferBaseFrame=new THREE.Mesh(getThickTagFrameGeo(20),tlOfferFrameMat);
tlOfferBaseFrame.position.z=0.0022;
tlOfferGroup.add(tlOfferBaseFrame);
const tlOfferPulseMat=new THREE.MeshBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.75,side:THREE.DoubleSide,depthWrite:false});
const tlOfferPulseRing=new THREE.Mesh(new THREE.RingGeometry(0.015,0.021,32),tlOfferPulseMat);
tlOfferPulseRing.position.z=0.003;
tlOfferGroup.add(tlOfferPulseRing);
const tlOfferPressMat=new THREE.MeshBasicMaterial({color:0x4ade80,transparent:true,opacity:0.95,side:THREE.DoubleSide,depthWrite:false});
const tlOfferPressRing=new THREE.Mesh(new THREE.RingGeometry(0.023,0.030,32),tlOfferPressMat);
tlOfferPressRing.position.z=0.010;
tlOfferPressRing.visible=false;
tlOfferGroup.add(tlOfferPressRing);
const tlOfferStemMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.78});
const tlOfferStem=new THREE.Mesh(_rodGeo,tlOfferStemMat);
setRodSegment(tlOfferStem,0,0,0.002,0,0,0.046,0.0014);
tlOfferGroup.add(tlOfferStem);
const tlOfferCanvas=document.createElement('canvas');tlOfferCanvas.width=440;tlOfferCanvas.height=188;
const tlOfferTex=makeUiTexture(tlOfferCanvas);
const tlOfferCard=new THREE.Sprite(new THREE.SpriteMaterial({map:tlOfferTex,transparent:true,depthTest:true,depthWrite:false}));
tlOfferCard.scale.set(0.168,0.072,1);
tlOfferCard.position.set(0,0,0.076);
tlOfferCard.renderOrder=14;
tlOfferGroup.add(tlOfferCard);

let hoveredTlOffer=false;
let tlOfferPressSince=null;
let lastTlOfferCardKey='';

function drawTlOfferCard(probeKind,pressProg=0,hot=false){
  const bucket=Math.round(pressProg*20);
  const key=`${probeKind}:${bucket}:${hot?1:0}`;
  if(key===lastTlOfferCardKey)return;
  lastTlOfferCardKey=key;
  const ctx=tlOfferCanvas.getContext('2d');
  ctx.clearRect(0,0,440,188);
  ctx.fillStyle='rgba(8,16,28,0.95)';
  ctx.beginPath();ctx.roundRect(4,4,432,180,16);ctx.fill();
  ctx.strokeStyle=pressProg>0.02?'#4ade80':(hot?'#fde047':'#22d3ee');
  ctx.lineWidth=4;ctx.stroke();
  ctx.fillStyle='#22d3ee';ctx.font='bold 24px ui-monospace,monospace';ctx.textAlign='center';
  ctx.fillText('◈ TL #353 FOUND',220,42);
  ctx.fillStyle='#f8fafc';ctx.font='bold 26px system-ui,sans-serif';
  ctx.fillText('Activate Table Perimeter?',220,82);
  // Tactile progress bar / CTA pill at bottom of card.
  ctx.fillStyle='rgba(30,41,59,0.95)';
  ctx.beginPath();ctx.roundRect(28,108,384,54,12);ctx.fill();
  if(pressProg>0.02){
    ctx.fillStyle='#22c55e';
    ctx.beginPath();ctx.roundRect(28,108,Math.max(24,384*pressProg),54,12);ctx.fill();
  }
  ctx.strokeStyle=pressProg>0.02?'#86efac':(hot?'#fde047':'#38bdf8');
  ctx.lineWidth=2.5;
  ctx.beginPath();ctx.roundRect(28,108,384,54,12);ctx.stroke();
  ctx.fillStyle=pressProg>0.4?'#052e16':'#f8fafc';
  ctx.font='bold 22px system-ui,sans-serif';
  const cta=pressProg>0.02?'ACTIVATING PERIMETER…':(probeKind==='ctrl'?'TRIGGER TO ACTIVATE':'PRESS TL TO ACTIVATE');
  ctx.fillText(cta,220,143);
  ctx.textAlign='left';
  tlOfferTex.needsUpdate=true;
}

async function activatePerimeterFromTL(now=performance.now()){
  if(!perimeterFence?.activate())return;
  perimeterOfferSuppressed=false;
  lastModeFenceKey='';
  const tl=perimeterFence.triggerMarker();
  hoveredTlOffer=false;
  tlOfferPressSince=null;
  tlOfferGroup.visible=false;
  fenceRiseStartMs=now;
  perimeterCompleteAnnounced=perimeterFence.dynamicCount()>=7;
  syncFenceVisuals(now,0,true);
  if(tl&&root.visible){
    const wpWorld=lPointToWorld(root.matrix.elements,tl.pos[0],tl.pos[1],0.035);
    triggerPinFlash(wpWorld,'#22c55e','✓');
  }
  pushDiagEvent({
    type:'perimeter-activated',triggerTagId:perimeterFence.triggerTagId,
    atPerformanceMs:now,dynamicCount:perimeterFence.dynamicCount(),
  });
  if(!extraScanActive){
    extraScanActive=true;obsBuffer.clear();
    status('Perimeter active · walk around table to scan remaining boundary markers');
    showHudToast('✓ PERIMETER ACTIVE · WALK AROUND TABLE','#22c55e',2600);
    if(!await capture.start()){
      extraScanActive=false;
      status('Perimeter active (camera failed to auto-start — toggle Viewfinder to scan)');
    }
  }else{
    status('Perimeter active · walk around table to scan remaining boundary markers');
    showHudToast('✓ PERIMETER ACTIVE · WALK AROUND TABLE','#22c55e',2400);
  }
}

function togglePerimeterFence(now=performance.now()){
  if(!perimeterFence)return;
  if(perimeterFence.isActive()){
    perimeterFence.deactivate();
    clearFenceBreaches();
    perimeterOfferSuppressed=true;
    hoveredTlOffer=false;tlOfferPressSince=null;tlOfferGroup.visible=false;
    lastModeFenceKey='';
    syncFenceVisuals(now,0,true);
    playModeTick('unpin');
    showHudToast('◈ PERIMETER OFF','#94a3b8',1500);
    status('Perimeter fence OFF');
  }else{
    perimeterFence.activate({force:true});
    perimeterOfferSuppressed=false;
    hoveredTlOffer=false;tlOfferPressSince=null;tlOfferGroup.visible=false;
    fenceRiseStartMs=now;
    lastModeFenceKey='';
    syncFenceVisuals(now,0,true);
    const cnt=perimeterFence.dynamicCount();
    playModeTick('pin');
    showHudToast(cnt>0?`◈ PERIMETER ON (${cnt}/7)`:'◈ PERIMETER ON · SCAN 353–359','#22d3ee',1800);
    status(cnt>0?`Perimeter fence ON (${cnt}/7)`:'Perimeter fence ON · scan 353–359');
  }
}

function updateTlPerimeterOffer(frameInputs,world,anPhase,asPhase,now){
  const anchored=Boolean(world)&&asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created');
  if(!anchored||!perimeterFence?.isOffered()||perimeterOfferSuppressed){
    tlOfferGroup.visible=false;
    hoveredTlOffer=false;
    tlOfferPressSince=null;
    return;
  }
  const tl=perimeterFence.triggerMarker();
  if(!tl){tlOfferGroup.visible=false;hoveredTlOffer=false;tlOfferPressSince=null;return;}
  tlOfferGroup.visible=true;
  tlOfferGroup.position.set(tl.pos[0],tl.pos[1],0);
  tlOfferBaseFrame.rotation.z=tl.yaw??0;

  const pulse=0.5+0.5*Math.sin(now*0.0055);
  tlOfferPulseRing.scale.setScalar(0.92+0.24*pulse);
  tlOfferPulseMat.opacity=0.45+0.45*(1-pulse);

  const probes=frameInputs.getProbesL();
  let bestDist=Infinity,bestProbe=null;
  if(!hoveredClosePinKey){
    for(const pr of probes){
      const dx=Math.max(0,Math.abs(pr.pos[0]-tl.pos[0])-0.018);
      const dy=Math.max(0,Math.abs(pr.pos[1]-tl.pos[1])-0.018);
      const dz=Math.max(0,pr.pos[2]-0.065,-pr.pos[2]-0.025);
      const d=Math.hypot(dx,dy,dz);
      recordProbeProximity(pr.probeId,d);
      if(d<bestDist){bestDist=d;bestProbe=pr;}
    }
  }

  const hoverThresh=hoveredTlOffer?0.048:0.036;
  hoveredTlOffer=Boolean(bestProbe&&bestDist<=hoverThresh);

  // Tactile finger press directly onto TL (<= 2.4 cm for 350 ms) activates perimeter & starts table-walk scan.
  let pressProg=0;
  if(hoveredTlOffer&&bestDist<=0.024){
    if(tlOfferPressSince==null)tlOfferPressSince=now;
    pressProg=Math.min(1,(now-tlOfferPressSince)/350);
    if(pressProg>=1){
      activatePerimeterFromTL(now);
      return;
    }
  }else{
    tlOfferPressSince=null;
  }

  tlOfferPressRing.visible=pressProg>0.02;
  if(pressProg>0.02){
    tlOfferPressRing.scale.setScalar(0.75+0.45*pressProg);
    tlOfferPressMat.opacity=0.55+0.45*pressProg;
  }
  tlOfferFrameMat.color.setHex(pressProg>0.02?0x4ade80:(hoveredTlOffer?0xfde047:0x22d3ee));
  drawTlOfferCard(bestProbe?.kind??'hand',pressProg,hoveredTlOffer);
}

// Compact floating 2-row L-angle dock ([■ MEASURE / ● SCAN], [● REC ALL], [?], [◈ FENCE], [TRAIL], [↺ CLEAR ALL]).
const modePillGroup=new THREE.Group();
modePillGroup.visible=false;
modePillGroup.position.set(0.11,-0.048,0.032);
root.add(modePillGroup);
const modePillCanvas=document.createElement('canvas');modePillCanvas.width=280;modePillCanvas.height=96;
const modePillTex=makeUiTexture(modePillCanvas);
const modePillSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modePillTex,transparent:true,depthTest:true,depthWrite:false}));
modePillSprite.position.set(-0.064,0.002,0.012);
modePillSprite.scale.set(0.086,0.030,1);
modePillSprite.renderOrder=14;
modePillGroup.add(modePillSprite);

const modeRecAllCanvas=document.createElement('canvas');modeRecAllCanvas.width=264;modeRecAllCanvas.height=96;
const modeRecAllTex=makeUiTexture(modeRecAllCanvas);
const modeRecAllSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeRecAllTex,transparent:true,depthTest:true,depthWrite:false}));
modeRecAllSprite.position.set(0.028,0.002,0.012);
modeRecAllSprite.scale.set(0.084,0.030,1);
modeRecAllSprite.renderOrder=14;
modePillGroup.add(modeRecAllSprite);

const modeInfoCanvas=document.createElement('canvas');modeInfoCanvas.width=96;modeInfoCanvas.height=96;
const modeInfoTex=makeUiTexture(modeInfoCanvas);
const modeInfoSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeInfoTex,transparent:true,depthTest:true,depthWrite:false}));
modeInfoSprite.position.set(0.092,0.002,0.012);
modeInfoSprite.scale.set(0.030,0.030,1);
modeInfoSprite.renderOrder=14;
modePillGroup.add(modeInfoSprite);

const modeFenceCanvas=document.createElement('canvas');modeFenceCanvas.width=240;modeFenceCanvas.height=96;
const modeFenceTex=makeUiTexture(modeFenceCanvas);
const modeFenceSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeFenceTex,transparent:true,depthTest:true,depthWrite:false}));
modeFenceSprite.position.set(-0.082,-0.036,-0.010);
modeFenceSprite.scale.set(0.074,0.030,1);
modeFenceSprite.renderOrder=14;
modePillGroup.add(modeFenceSprite);

const modeColorCanvas=document.createElement('canvas');modeColorCanvas.width=256;modeColorCanvas.height=96;
const modeColorTex=makeUiTexture(modeColorCanvas);
const modeColorSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeColorTex,transparent:true,depthTest:true,depthWrite:false}));
modeColorSprite.position.set(0,-0.036,-0.010);
modeColorSprite.scale.set(0.076,0.030,1);
modeColorSprite.renderOrder=14;
modePillGroup.add(modeColorSprite);

const modeClearCanvas=document.createElement('canvas');modeClearCanvas.width=240;modeClearCanvas.height=96;
const modeClearTex=makeUiTexture(modeClearCanvas);
const modeClearSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeClearTex,transparent:true,depthTest:true,depthWrite:false}));
modeClearSprite.position.set(0.082,-0.036,-0.010);
modeClearSprite.scale.set(0.074,0.030,1);
modeClearSprite.renderOrder=14;
modePillGroup.add(modeClearSprite);

const turboBarCanvas=document.createElement('canvas');turboBarCanvas.width=560;turboBarCanvas.height=92;
const turboBarTex=makeUiTexture(turboBarCanvas);
turboBarTex.colorSpace=THREE.SRGBColorSpace;
const turboBarSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:turboBarTex,transparent:true,depthTest:true,depthWrite:false}));
turboBarSprite.position.set(0,-0.078,-0.016);
turboBarSprite.scale.set(0.240,0.039,1);
turboBarSprite.visible=false;
turboBarSprite.renderOrder=14;
modePillGroup.add(turboBarSprite);

const modeHelpCanvas=document.createElement('canvas');modeHelpCanvas.width=560;modeHelpCanvas.height=318;
const modeHelpTex=makeUiTexture(modeHelpCanvas);
const modeHelpSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:modeHelpTex,transparent:true,depthTest:true,depthWrite:false}));
modeHelpSprite.position.set(0.010,0.110,0.034);
modeHelpSprite.scale.set(0.255,0.145,1);
modeHelpSprite.visible=false;
modeHelpSprite.renderOrder=15;
modePillGroup.add(modeHelpSprite);

let hoveredModePill=false,hoveredModeMain=false,hoveredModeRecAll=false,hoveredModeInfo=false,hoveredModeFence=false,hoveredModeClear=false,hoveredModeColor=false;
let modePillPokeSince=null,modePillPokeConsumed=false,modePillCooldownUntil=0,lastModePillKey='';
let modeRecAllPokeSince=null,modeRecAllPokeConsumed=false,modeRecAllCooldownUntil=0,lastModeRecAllKey='';
let modeFencePokeSince=null,modeFencePokeConsumed=false,modeFenceCooldownUntil=0,lastModeFenceKey='';
let modeClearPokeSince=null,modeClearPokeConsumed=false,modeClearCooldownUntil=0,lastModeClearKey='';
let modeColorPokeSince=null,modeColorPokeConsumed=false,modeColorCooldownUntil=0,lastModeColorKey='';
let modeHelpOpen=false,modeInfoPokeSince=null,modeInfoPokeConsumed=false,modeInfoCooldownUntil=0,lastModeInfoKey='',lastModeHelpKey='',lastTurboBarKey='';
const userItemCount=()=>movableTools.size+placedExtras.size+pinnedPoints.size;
function drawModeRecAllBtn(armed,activeCount,hot,prog=0){
  const key=`${armed?1:0}:${activeCount}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModeRecAllKey)return;
  lastModeRecAllKey=key;
  const ctx=modeRecAllCanvas.getContext('2d');
  ctx.clearRect(0,0,264,96);
  ctx.fillStyle=armed?(hot?'rgba(90,14,22,0.96)':'rgba(56,10,16,0.94)'):(hot?'rgba(16,32,52,0.96)':'rgba(12,20,34,0.92)');
  ctx.beginPath();ctx.roundRect(4,4,256,88,24);ctx.fill();
  ctx.strokeStyle=hot?'#fde047':(armed?'#ef4444':'#38bdf8');
  ctx.lineWidth=hot?5:3.5;ctx.stroke();
  if(prog>0.02){
    ctx.fillStyle=armed?'rgba(74,222,128,0.32)':'rgba(239,68,68,0.34)';
    ctx.beginPath();ctx.roundRect(6,6,Math.max(16,252*Math.min(1,prog)),84,22);ctx.fill();
  }
  ctx.fillStyle=hot?'#ffffff':(armed?'#fca5a5':'#7dd3fc');
  ctx.font='bold 26px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(armed?'■ REC ALL':'● REC ALL',132,45);
  ctx.fillStyle=armed?'#fecaca':'#94a3b8';
  ctx.font='bold 16px ui-monospace,monospace';
  ctx.fillText(armed?(activeCount>0?`REC · ${activeCount} MOV`:'ARMED · IDLE'):'ALL GENERIC',132,74);
  ctx.textAlign='left';
  modeRecAllTex.needsUpdate=true;
}
function drawModeClearBtn(itemCount,hot,prog=0){
  const hasItems=itemCount>0;
  const key=`${itemCount}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModeClearKey)return;
  lastModeClearKey=key;
  const ctx=modeClearCanvas.getContext('2d');
  ctx.clearRect(0,0,240,96);
  ctx.fillStyle=hasItems?(hot?'rgba(62,28,8,0.96)':'rgba(32,20,10,0.92)'):'rgba(15,23,42,0.78)';
  ctx.beginPath();ctx.roundRect(4,4,232,88,24);ctx.fill();
  ctx.strokeStyle=hot&&hasItems?'#fde047':(hasItems?'#f59e0b':'#475569');
  ctx.lineWidth=hot&&hasItems?5:3.5;ctx.stroke();
  if(prog>0.02&&hasItems){
    ctx.fillStyle='rgba(245,158,11,0.34)';
    ctx.beginPath();ctx.roundRect(6,6,Math.max(16,228*Math.min(1,prog)),84,22);ctx.fill();
  }
  ctx.fillStyle=hot&&hasItems?'#ffffff':(hasItems?'#fbbf24':'#64748b');
  ctx.font='bold 24px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText('↺ CLEAR ALL',120,44);
  ctx.fillStyle=hasItems?'#fde68a':'#64748b';
  ctx.font='bold 16px ui-monospace,monospace';
  ctx.fillText(hasItems?`${itemCount} ITEMS`:'EMPTY',120,74);
  ctx.textAlign='left';
  modeClearTex.needsUpdate=true;
}
function drawModeFenceBtn(fStatus,dynCount,hot,prog=0){
  const eff=(fStatus==='offered'&&perimeterOfferSuppressed)?'idle':fStatus;
  const key=`${eff}:${dynCount}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModeFenceKey)return;
  lastModeFenceKey=key;
  const ctx=modeFenceCanvas.getContext('2d');
  ctx.clearRect(0,0,240,96);
  const active=eff==='active',offered=eff==='offered';
  ctx.fillStyle=active?'rgba(8,36,44,0.94)':(offered?'rgba(42,28,8,0.94)':'rgba(15,23,42,0.92)');
  ctx.beginPath();ctx.roundRect(4,4,232,88,24);ctx.fill();
  ctx.strokeStyle=hot?'#fde047':(active?'#22d3ee':(offered?'#fbbf24':'#64748b'));
  ctx.lineWidth=hot?5:3.5;ctx.stroke();
  if(prog>0.02){
    ctx.fillStyle=active?'rgba(148,163,184,0.30)':'rgba(34,211,238,0.32)';
    ctx.beginPath();ctx.roundRect(6,6,Math.max(16,228*Math.min(1,prog)),84,22);ctx.fill();
  }
  ctx.fillStyle=hot?'#ffffff':(active?'#22d3ee':(offered?'#fbbf24':'#cbd5e1'));
  ctx.font='bold 26px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText('◈ FENCE',120,44);
  ctx.fillStyle=active?'#a5f3fc':(offered?'#fde68a':'#94a3b8');
  ctx.font='bold 17px ui-monospace,monospace';
  ctx.fillText(active?`ON · ${dynCount}/7`:(offered?'TL READY':'OFF'),120,74);
  ctx.textAlign='left';
  modeFenceTex.needsUpdate=true;
}
function drawModePillCard(scanActive,hot,prog=0){
  const qMode=adaptiveQuality.getMode();
  const key=`${scanActive?1:0}:${qMode}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModePillKey)return;
  lastModePillKey=key;
  const ctx=modePillCanvas.getContext('2d');
  ctx.clearRect(0,0,280,96);
  ctx.fillStyle=scanActive?'rgba(42,12,18,0.94)':'rgba(10,24,20,0.94)';
  ctx.beginPath();ctx.roundRect(4,4,272,88,24);ctx.fill();
  ctx.strokeStyle=hot?'#fde047':(scanActive?'#4ade80':'#22d3ee');
  ctx.lineWidth=hot?5:3.5;ctx.stroke();
  if(prog>0.02){
    ctx.fillStyle=scanActive?'rgba(74,222,128,0.32)':'rgba(34,211,238,0.32)';
    ctx.beginPath();ctx.roundRect(6,6,Math.max(16,268*Math.min(1,prog)),84,22);ctx.fill();
  }
  ctx.fillStyle=hot?'#ffffff':(scanActive?'#4ade80':'#22d3ee');
  ctx.font='bold 30px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(scanActive?'■ MEASURE':'● SCAN',140,48);
  ctx.fillStyle='#94a3b8';ctx.font='bold 16px ui-monospace,monospace';
  ctx.fillText(`POKE · ${qMode.toUpperCase()}`,140,76);
  ctx.textAlign='left';
  modePillTex.needsUpdate=true;
}
function drawModeColorBtn(colorMode,hot,prog=0){
  const isSpeed=colorMode==='speed';
  const key=`${isSpeed?1:0}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModeColorKey)return;
  lastModeColorKey=key;
  const ctx=modeColorCanvas.getContext('2d');
  ctx.clearRect(0,0,256,96);
  ctx.fillStyle=isSpeed?(hot?'rgba(54,24,10,0.96)':'rgba(34,16,8,0.92)'):(hot?'rgba(14,32,52,0.96)':'rgba(12,22,38,0.92)');
  ctx.beginPath();ctx.roundRect(4,4,248,88,24);ctx.fill();
  ctx.strokeStyle=hot?'#fde047':(isSpeed?'#f97316':'#38bdf8');
  ctx.lineWidth=hot?5:3.5;ctx.stroke();
  if(prog>0.02){
    ctx.fillStyle=isSpeed?'rgba(249,115,22,0.32)':'rgba(56,189,248,0.30)';
    ctx.beginPath();ctx.roundRect(6,6,Math.max(16,244*Math.min(1,prog)),84,22);ctx.fill();
  }
  ctx.fillStyle=hot?'#ffffff':(isSpeed?'#fdba74':'#7dd3fc');
  ctx.font='bold 24px system-ui,sans-serif';ctx.textAlign='center';
  ctx.fillText(isSpeed?'TRAIL: SPEED':'TRAIL: ID',128,42);
  if(isSpeed){
    const grad=ctx.createLinearGradient(34,0,136,0);
    for(let s=0;s<=16;s++){
      const t=s/16;
      const [r,g,b]=turboColormap(t);
      grad.addColorStop(t,`rgb(${Math.round(r*255)},${Math.round(g*255)},${Math.round(b*255)})`);
    }
    ctx.fillStyle=grad;
    ctx.beginPath();ctx.roundRect(34,60,102,14,7);ctx.fill();
    ctx.fillStyle='#fed7aa';ctx.font='bold 16px ui-monospace,monospace';ctx.textAlign='left';
    ctx.fillText('TURBO',146,73);
  }else{
    const swatches=['#22d3ee','#fb923c','#34d399','#f472b6'];
    for(let i=0;i<swatches.length;i++){
      ctx.fillStyle=swatches[i];
      ctx.beginPath();ctx.arc(46+i*20,67,6.5,0,Math.PI*2);ctx.fill();
    }
    ctx.fillStyle='#bae6fd';ctx.font='bold 16px ui-monospace,monospace';ctx.textAlign='left';
    ctx.fillText('UNIQUE',132,73);
  }
  ctx.textAlign='left';
  modeColorTex.needsUpdate=true;
}
function drawTurboColorBar(refSpeedMps){
  const maxMm=Math.max(100,Math.round((refSpeedMps||0.60)*1000));
  const key=`${maxMm}`;
  if(key===lastTurboBarKey)return;
  lastTurboBarKey=key;
  const ctx=turboBarCanvas.getContext('2d');
  ctx.clearRect(0,0,560,92);
  ctx.fillStyle='rgba(7,12,24,0.92)';
  ctx.beginPath();ctx.roundRect(4,4,552,84,18);ctx.fill();
  ctx.strokeStyle='rgba(148,163,184,0.42)';ctx.lineWidth=2.5;ctx.stroke();

  ctx.fillStyle='#e2e8f0';ctx.font='bold 16px system-ui,sans-serif';ctx.textAlign='left';
  ctx.fillText('SPEED · TURBO',18,27);
  ctx.fillStyle='#94a3b8';ctx.font='bold 15px ui-monospace,monospace';ctx.textAlign='right';
  ctx.fillText('mm/s',542,27);

  const barX=20,barY=36,barW=520,barH=18;
  const grad=ctx.createLinearGradient(barX,0,barX+barW,0);
  for(let s=0;s<=32;s++){
    const t=s/32;
    const [r,g,b]=turboColormap(t);
    grad.addColorStop(t,`rgb(${Math.round(r*255)},${Math.round(g*255)},${Math.round(b*255)})`);
  }
  ctx.fillStyle=grad;
  ctx.beginPath();ctx.roundRect(barX,barY,barW,barH,9);ctx.fill();
  ctx.strokeStyle='rgba(255,255,255,0.28)';ctx.lineWidth=1.5;ctx.stroke();

  ctx.strokeStyle='rgba(248,250,252,0.72)';ctx.lineWidth=1.8;
  ctx.fillStyle='#e2e8f0';ctx.font='bold 16px ui-monospace,monospace';
  const ticks=[0,1/3,2/3,1];
  for(let i=0;i<ticks.length;i++){
    const f=ticks[i];
    const tx=barX+f*barW;
    ctx.beginPath();ctx.moveTo(tx,barY+barH-2);ctx.lineTo(tx,barY+barH+5);ctx.stroke();
    const val=Math.round(f*maxMm);
    const label=i===ticks.length-1?`${val}+`:`${val}`;
    ctx.textAlign=i===0?'left':(i===ticks.length-1?'right':'center');
    ctx.fillText(label,tx,barY+barH+24);
  }
  ctx.textAlign='left';
  turboBarTex.needsUpdate=true;
}
function drawModeInfoBtn(open,hot,prog=0){
  const key=`${open?1:0}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===lastModeInfoKey)return;
  lastModeInfoKey=key;
  const ctx=modeInfoCanvas.getContext('2d');
  ctx.clearRect(0,0,96,96);
  ctx.fillStyle=open?'rgba(12,42,66,0.96)':'rgba(12,18,32,0.94)';
  ctx.beginPath();ctx.arc(48,48,40,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=hot?'#fde047':(open?'#38bdf8':'#94a3b8');
  ctx.lineWidth=hot?6:4;ctx.stroke();
  if(prog>0.02){
    ctx.beginPath();
    ctx.arc(48,48,33,-Math.PI/2,-Math.PI/2+Math.PI*2*Math.min(1,prog));
    ctx.strokeStyle='#38bdf8';ctx.lineWidth=6;ctx.stroke();
  }
  ctx.fillStyle=hot?'#ffffff':(open?'#7dd3fc':'#e2e8f0');
  ctx.font='bold 42px system-ui,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillText(open?'×':'?',48,50);
  ctx.textAlign='left';ctx.textBaseline='alphabetic';
  modeInfoTex.needsUpdate=true;
}
function drawModeHelpCard(scanActive){
  const key=scanActive?'scan':'measure';
  if(key===lastModeHelpKey)return;
  lastModeHelpKey=key;
  const ctx=modeHelpCanvas.getContext('2d');
  ctx.clearRect(0,0,560,318);
  const accent=scanActive?'#22d3ee':'#4ade80';
  ctx.fillStyle='rgba(7,12,24,0.96)';
  ctx.beginPath();ctx.roundRect(6,6,548,306,20);ctx.fill();
  ctx.strokeStyle=accent;ctx.lineWidth=4;ctx.stroke();

  ctx.fillStyle=accent;ctx.font='bold 24px system-ui,sans-serif';
  ctx.fillText(scanActive?'● SCAN MODE · CONTROLS':'■ MEASURE MODE · 90 FPS',24,42,512);
  ctx.strokeStyle='rgba(148,163,184,0.28)';ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(22,56);ctx.lineTo(538,56);ctx.stroke();

  const lines=scanActive?[
    '• Look at AprilTags (300–359) to auto-lock (moving wake)',
    '• Grab & move tag/ruler → 90 Hz hand tracking',
    '• Poke [● REC ALL] or Right Fist → record all moving tags',
    '• Hold ● below tag → path · Hold Pin → Time-Copy',
    '• Poke [■ MEASURE] or Left Fist → switch to 90 FPS inspect',
  ]:[
    '• Hover any tag/point → X/Y/Z, speed & 3D square',
    '• Pinch / Trigger point → pin & measure distance',
    '• Poke [● REC ALL] or Right Fist → start SCAN + REC ALL',
    '• Poke [TRAIL: ID/SPEED] or Left Wrist → switch trail color',
    '• Poke [↺ CLEAR ALL] on table → reset tags & trails',
  ];
  ctx.fillStyle='#e2e8f0';ctx.font='19px system-ui,sans-serif';
  for(let i=0;i<lines.length;i++){
    ctx.fillText(lines[i],24,92+i*38,512);
  }
  ctx.fillStyle='#94a3b8';ctx.font='bold 15px ui-monospace,monospace';ctx.textAlign='center';
  ctx.fillText('Poke [ × ] again to close guide',280,294,512);
  ctx.textAlign='left';
  modeHelpTex.needsUpdate=true;
}
function toggleModeHelp(now=performance.now()){
  modeHelpOpen=!modeHelpOpen;
  modeHelpSprite.visible=modeHelpOpen;
  if(modeHelpOpen)drawModeHelpCard(extraScanActive);
  modeInfoCooldownUntil=now+600;
  playModeTick('pin');
}
function updateModeAnchorPill(frameInputs,world,anPhase,asPhase,now){
  const anchored=Boolean(world)&&asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created');
  if(!anchored){
    modePillGroup.visible=false;hoveredModePill=false;hoveredModeMain=false;hoveredModeRecAll=false;hoveredModeInfo=false;hoveredModeFence=false;hoveredModeClear=false;hoveredModeColor=false;
    modePillPokeSince=null;modePillPokeConsumed=false;
    modeRecAllPokeSince=null;modeRecAllPokeConsumed=false;
    modeFencePokeSince=null;modeFencePokeConsumed=false;
    modeClearPokeSince=null;modeClearPokeConsumed=false;
    modeColorPokeSince=null;modeColorPokeConsumed=false;
    modeInfoPokeSince=null;modeInfoPokeConsumed=false;
    modeHelpSprite.visible=false;
    turboBarSprite.visible=false;
    return;
  }
  modePillGroup.visible=true;
  if(leaveConfirmState){
    hoveredModePill=false;hoveredModeMain=false;hoveredModeRecAll=false;hoveredModeInfo=false;hoveredModeFence=false;hoveredModeClear=false;hoveredModeColor=false;
    modePillPokeSince=null;modePillPokeConsumed=false;
    modeRecAllPokeSince=null;modeRecAllPokeConsumed=false;
    modeFencePokeSince=null;modeFencePokeConsumed=false;
    modeClearPokeSince=null;modeClearPokeConsumed=false;
    modeColorPokeSince=null;modeColorPokeConsumed=false;
    modeInfoPokeSince=null;modeInfoPokeConsumed=false;
    return;
  }
  const itemCount=userItemCount();
  const probes=frameInputs.getProbesL();
  const gx=modePillGroup.position.x,gy=modePillGroup.position.y,gz=modePillGroup.position.z;
  const distTo=(spr,px,py,pz)=>Math.hypot(px-(gx+spr.position.x),py-(gy+spr.position.y),pz-(gz+spr.position.z));
  let bestDist=Infinity,bestRecAllDist=Infinity,bestInfoDist=Infinity,bestFenceDist=Infinity,bestClearDist=Infinity,bestColorDist=Infinity;
  for(const pr of probes){
    const px=pr.pos[0],py=pr.pos[1],pz=pr.pos[2];
    const d=distTo(modePillSprite,px,py,pz);
    const dRec=distTo(modeRecAllSprite,px,py,pz);
    const di=distTo(modeInfoSprite,px,py,pz);
    const df=distTo(modeFenceSprite,px,py,pz);
    const dc=distTo(modeClearSprite,px,py,pz);
    const dCol=distTo(modeColorSprite,px,py,pz);
    recordProbeProximity(pr.probeId,Math.min(d,dRec,di,df,dc,dCol));
    if(d<bestDist)bestDist=d;
    if(dRec<bestRecAllDist)bestRecAllDist=dRec;
    if(di<bestInfoDist)bestInfoDist=di;
    if(df<bestFenceDist)bestFenceDist=df;
    if(dc<bestClearDist)bestClearDist=dc;
    if(dCol<bestColorDist)bestColorDist=dCol;
  }

  // Exclusive closest-button arbitration so adjacent pills never double-trigger.
  let closestBtn=null,closestDist=Infinity;
  const consider=(id,dist,maxDist)=>{if(dist<maxDist&&dist<closestDist){closestDist=dist;closestBtn=id;}};
  consider('main',bestDist,0.042);
  consider('recAll',bestRecAllDist,0.040);
  consider('info',bestInfoDist,0.032);
  if(perimeterFence)consider('fence',bestFenceDist,0.038);
  consider('clear',bestClearDist,0.038);
  consider('color',bestColorDist,0.038);

  hoveredModeMain=closestBtn==='main';
  hoveredModeRecAll=closestBtn==='recAll';
  hoveredModeInfo=closestBtn==='info';
  hoveredModeFence=closestBtn==='fence';
  hoveredModeClear=closestBtn==='clear';
  hoveredModeColor=closestBtn==='color';
  hoveredModePill=Boolean(closestBtn);

  modePillSprite.scale.set(hoveredModeMain?0.094:0.086,hoveredModeMain?0.034:0.030,1);
  modeRecAllSprite.scale.set(hoveredModeRecAll?0.092:0.084,hoveredModeRecAll?0.034:0.030,1);
  modeInfoSprite.scale.set(hoveredModeInfo?0.035:0.030,hoveredModeInfo?0.035:0.030,1);
  modeFenceSprite.visible=Boolean(perimeterFence);
  modeFenceSprite.scale.set(hoveredModeFence?0.082:0.074,hoveredModeFence?0.034:0.030,1);
  modeClearSprite.scale.set(hoveredModeClear&&itemCount>0?0.082:0.074,hoveredModeClear&&itemCount>0?0.034:0.030,1);
  modeColorSprite.scale.set(hoveredModeColor?0.084:0.076,hoveredModeColor?0.034:0.030,1);

  let prog=0;
  if(closestBtn==='main'&&bestDist<0.036&&now>=modePillCooldownUntil){
    if(!modePillPokeConsumed){
      if(modePillPokeSince==null)modePillPokeSince=now;
      prog=Math.min(1,(now-modePillPokeSince)/220);
      if(prog>=1){
        modePillPokeConsumed=true;modePillPokeSince=null;modePillCooldownUntil=now+900;
        toggleExtraScan();
      }
    }
  }else{
    modePillPokeSince=null;
    if(bestDist>0.052)modePillPokeConsumed=false;
  }

  let recAllProg=0;
  if(closestBtn==='recAll'&&bestRecAllDist<0.035&&now>=modeRecAllCooldownUntil){
    if(!modeRecAllPokeConsumed){
      if(modeRecAllPokeSince==null)modeRecAllPokeSince=now;
      recAllProg=Math.min(1,(now-modeRecAllPokeSince)/220);
      if(recAllProg>=1){
        modeRecAllPokeConsumed=true;modeRecAllPokeSince=null;modeRecAllCooldownUntil=now+800;
        toggleRecordAll();
      }
    }
  }else{
    modeRecAllPokeSince=null;
    if(bestRecAllDist>0.050)modeRecAllPokeConsumed=false;
  }

  let fenceProg=0;
  if(closestBtn==='fence'&&perimeterFence&&bestFenceDist<0.034&&now>=modeFenceCooldownUntil){
    if(!modeFencePokeConsumed){
      if(modeFencePokeSince==null)modeFencePokeSince=now;
      fenceProg=Math.min(1,(now-modeFencePokeSince)/220);
      if(fenceProg>=1){
        modeFencePokeConsumed=true;modeFencePokeSince=null;modeFenceCooldownUntil=now+700;
        togglePerimeterFence(now);
      }
    }
  }else{
    modeFencePokeSince=null;
    if(bestFenceDist>0.048)modeFencePokeConsumed=false;
  }

  let clearProg=0;
  if(closestBtn==='clear'&&itemCount>0&&bestClearDist<0.034&&now>=modeClearCooldownUntil){
    if(!modeClearPokeConsumed){
      if(modeClearPokeSince==null)modeClearPokeSince=now;
      clearProg=Math.min(1,(now-modeClearPokeSince)/350);
      if(clearProg>=1){
        modeClearPokeConsumed=true;modeClearPokeSince=null;modeClearCooldownUntil=now+900;
        clearAllUserItems();
      }
    }
  }else{
    modeClearPokeSince=null;
    if(bestClearDist>0.048)modeClearPokeConsumed=false;
  }

  let colorProg=0;
  if(closestBtn==='color'&&bestColorDist<0.034&&now>=modeColorCooldownUntil){
    if(!modeColorPokeConsumed){
      if(modeColorPokeSince==null)modeColorPokeSince=now;
      colorProg=Math.min(1,(now-modeColorPokeSince)/220);
      if(colorProg>=1){
        modeColorPokeConsumed=true;modeColorPokeSince=null;modeColorCooldownUntil=now+650;
        toggleTrailColorMode();
      }
    }
  }else{
    modeColorPokeSince=null;
    if(bestColorDist>0.048)modeColorPokeConsumed=false;
  }

  let infoProg=0;
  if(closestBtn==='info'&&bestInfoDist<0.027&&now>=modeInfoCooldownUntil){
    if(!modeInfoPokeConsumed){
      if(modeInfoPokeSince==null)modeInfoPokeSince=now;
      infoProg=Math.min(1,(now-modeInfoPokeSince)/220);
      if(infoProg>=1){
        modeInfoPokeConsumed=true;modeInfoPokeSince=null;
        toggleModeHelp(now);
      }
    }
  }else{
    modeInfoPokeSince=null;
    if(bestInfoDist>0.042)modeInfoPokeConsumed=false;
  }

  let activeRecCount=0;
  for(const e of movableTools.values())if(isEntryRecording(e))activeRecCount++;
  drawModePillCard(extraScanActive,hoveredModeMain,prog);
  drawModeRecAllBtn(recordAllActive,activeRecCount,hoveredModeRecAll,recAllProg);
  if(perimeterFence){
    drawModeFenceBtn(perimeterFence.status(),perimeterFence.dynamicCount(),hoveredModeFence,fenceProg);
  }
  drawModeClearBtn(userItemCount(),hoveredModeClear,clearProg);
  drawModeColorBtn(trailColorMode,hoveredModeColor,colorProg);
  drawModeInfoBtn(modeHelpOpen,hoveredModeInfo,infoProg);
  turboBarSprite.visible=trailColorMode==='speed';
  if(turboBarSprite.visible)drawTurboColorBar(PATHLINE_DEFAULTS.turboRefSpeedMps);
  modeHelpSprite.visible=modeHelpOpen;
  if(modeHelpOpen)drawModeHelpCard(extraScanActive);
}

// Extensible Developer Dock along the outside of the L-angle Y-axis (x < 0, clear of tags & workspace).
function setDevHandCoupling(enabled){
  devHandCouplingEnabled=Boolean(enabled);
  if(!devHandCouplingEnabled){
    tagHandCoupler.reset();
    for(const entry of movableTools.values()){
      if(entry.handCoupled){
        entry.handCoupled=false;
        entry.coupledHand=null;
        entry.coupledStepCount=0;
        resetDisplayErrorOffset(entry);
        drawLiveBadge(entry,isEntryRecording(entry)?'rec':entry.lockState,false);
      }
    }
  }
  playModeTick(devHandCouplingEnabled?'scan':'pin');
  showHudToast(devHandCouplingEnabled?'DEV · HAND COUPLE: ON':'DEV · HAND COUPLE: OFF (OPTICAL)',devHandCouplingEnabled?'#4ade80':'#f59e0b',1500);
}
function setDevDisplayPred(enabled){
  devDisplayPredEnabled=Boolean(enabled);
  for(const entry of movableTools.values())resetDisplayErrorOffset(entry);
  playModeTick(devDisplayPredEnabled?'scan':'pin');
  showHudToast(devDisplayPredEnabled?'DEV · DISP PRED: ON':'DEV · DISP PRED: RAW (OFF)',devDisplayPredEnabled?'#38bdf8':'#f43f5e',1500);
}

const devDockGroup=new THREE.Group();
devDockGroup.visible=false;
devDockGroup.position.set(-0.092,0.080,0.036);
root.add(devDockGroup);
const devDockStem=new THREE.Mesh(
  _rodGeo,
  new THREE.MeshBasicMaterial({color:0xa855f7,transparent:true,opacity:0.40,depthWrite:false}),
);
devDockStem.rotation.x=Math.PI/2;
devDockStem.scale.set(0.0007,0.228,0.0007);
devDockStem.position.set(0,0,0.078);
devDockGroup.add(devDockStem);
let hoveredDevDock=false;
let devActiveSliderId=null,devActiveSliderProbeId=null,devSliderCooldownUntil=0;

const devDockSpecs=[
  {id:'hand',kind:'toggle',y:0,z:0.000,label:'HAND',onText:'ON',offText:'OFF',onColor:'#4ade80',offColor:'#f59e0b',get:()=>devHandCouplingEnabled,onToggle:()=>setDevHandCoupling(!devHandCouplingEnabled)},
  {id:'pred',kind:'toggle',y:0,z:0.056,label:'DISP PRED',onText:'ON',offText:'RAW',onColor:'#38bdf8',offColor:'#f43f5e',get:()=>devDisplayPredEnabled,onToggle:()=>setDevDisplayPred(!devDisplayPredEnabled)},
  {id:'optLag',kind:'slider',y:0,z:0.122,label:'OPT LAG',min:0,max:120,step:2,unit:'ms',accent:'#38bdf8',get:()=>devOptLagMs,set:v=>{devOptLagMs=v;},dimmed:()=>!devDisplayPredEnabled},
  {id:'handLag',kind:'slider',y:0,z:0.192,label:'HAND LAG',min:0,max:60,step:2,unit:'ms',accent:'#c084fc',get:()=>devHandLagMs,set:v=>{devHandLagMs=v;},dimmed:()=>!devHandCouplingEnabled},
];

const devDockItems=devDockSpecs.map(spec=>{
  const isSlider=spec.kind==='slider';
  const canvas=document.createElement('canvas');
  canvas.width=isSlider?320:280;
  canvas.height=isSlider?96:84;
  const tex=makeUiTexture(canvas);
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true,depthTest:true,depthWrite:false}));
  sprite.position.set(0,spec.y,spec.z);
  sprite.scale.set(isSlider?0.114:0.094,isSlider?0.034:0.028,1);
  sprite.renderOrder=14;
  devDockGroup.add(sprite);
  return {
    spec,canvas,tex,sprite,
    trackHalfW:isSlider?0.048:0,
    lastKey:'',
    pokeSince:null,pokeConsumed:false,cooldownUntil:0,lastTickAt:0,
    bestDist:Infinity,bestU:0,bestProbeId:null,
  };
});

function drawDevToggle(item,hot,prog=0){
  const on=Boolean(item.spec.get());
  const key=`${on?1:0}:${hot?1:0}:${Math.round(prog*20)}`;
  if(key===item.lastKey)return;
  item.lastKey=key;
  const ctx=item.canvas.getContext('2d');
  ctx.clearRect(0,0,280,84);
  const col=on?item.spec.onColor:item.spec.offColor;
  ctx.fillStyle=on?'rgba(14,10,28,0.92)':'rgba(18,12,24,0.88)';
  ctx.beginPath();ctx.roundRect(4,4,272,76,20);ctx.fill();
  if(prog>0.01){
    ctx.save();
    ctx.beginPath();ctx.roundRect(4,4,272,76,20);ctx.clip();
    ctx.fillStyle='rgba(192,132,252,0.28)';
    ctx.fillRect(4,4,272*Math.min(1,prog),76);
    ctx.restore();
  }
  ctx.strokeStyle=hot?'#fef08a':(on?'#a855f7':col);
  ctx.lineWidth=hot?5:3;
  ctx.beginPath();ctx.roundRect(4,4,272,76,20);ctx.stroke();
  ctx.fillStyle=col;
  ctx.beginPath();ctx.arc(30,42,9,0,Math.PI*2);ctx.fill();
  ctx.fillStyle=hot?'#ffffff':'#f8fafc';
  ctx.font='bold 21px ui-monospace,monospace';
  ctx.textAlign='left';
  ctx.textBaseline='middle';
  ctx.fillText(`${item.spec.label}: ${on?item.spec.onText:item.spec.offText}`,50,43,214);
  ctx.textBaseline='alphabetic';
  item.tex.needsUpdate=true;
}

function drawDevSlider(item,hot,active){
  const val=item.spec.get();
  const dim=Boolean(item.spec.dimmed?.());
  const key=`${val}:${dim?1:0}:${hot?1:0}:${active?1:0}`;
  if(key===item.lastKey)return;
  item.lastKey=key;
  const ctx=item.canvas.getContext('2d');
  ctx.clearRect(0,0,320,96);
  const {min,max,unit,label,accent}=item.spec;
  const frac=Math.max(0,Math.min(1,(val-min)/Math.max(1e-6,max-min)));
  ctx.fillStyle=dim?'rgba(14,12,22,0.78)':'rgba(14,10,28,0.92)';
  ctx.beginPath();ctx.roundRect(4,4,312,88,20);ctx.fill();
  ctx.strokeStyle=(hot||active)?'#fef08a':(dim?'#475569':accent);
  ctx.lineWidth=(hot||active)?5:3;
  ctx.beginPath();ctx.roundRect(4,4,312,88,20);ctx.stroke();
  ctx.fillStyle=dim?'#94a3b8':'#e2e8f0';
  ctx.font='bold 19px ui-monospace,monospace';
  ctx.textAlign='left';
  ctx.fillText(label,20,33,168);
  ctx.fillStyle=dim?'#64748b':((hot||active)?'#fef08a':accent);
  ctx.textAlign='right';
  const suffix=dim?(item.spec.id==='optLag'?' (RAW)':' (OFF)'):'';
  ctx.fillText(`${val} ${unit}${suffix}`,300,33,128);
  ctx.textAlign='left';
  const tx=24,ty=64,tw=272,th=12;
  ctx.fillStyle='rgba(51,65,85,0.85)';
  ctx.beginPath();ctx.roundRect(tx,ty-th/2,tw,th,6);ctx.fill();
  const barCol=active?'#fef08a':(dim?'#64748b':accent);
  if(frac>0.005){
    ctx.fillStyle=barCol;
    ctx.beginPath();ctx.roundRect(tx,ty-th/2,Math.max(th,tw*frac),th,6);ctx.fill();
  }
  const kx=tx+tw*frac;
  ctx.fillStyle=dim?'#cbd5e1':'#ffffff';
  ctx.beginPath();ctx.arc(kx,ty,active?15:12,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=barCol;
  ctx.lineWidth=3.5;
  ctx.stroke();
  item.tex.needsUpdate=true;
}

const _devSliderHit=[0,0];
function probeDevSlider(item,px,py,pz,gx,gy,gz,rx,ry,rz){
  const sp=item.sprite.position;
  const dx=px-(gx+sp.x),dy=py-(gy+sp.y),dz=pz-(gz+sp.z);
  const along=dx*rx+dy*ry+dz*rz;
  const clamped=Math.max(-item.trackHalfW,Math.min(item.trackHalfW,along));
  _devSliderHit[0]=Math.hypot(dx-rx*clamped,dy-ry*clamped,dz-rz*clamped);
  _devSliderHit[1]=(clamped+item.trackHalfW)/(2*item.trackHalfW);
}

function applyDevSliderValue(item,u,now){
  const {min,max,step}=item.spec;
  const raw=min+u*(max-min);
  const val=Math.max(min,Math.min(max,Math.round(raw/step)*step));
  if(val!==item.spec.get()){
    item.spec.set(val);
    if(now-item.lastTickAt>=90){
      item.lastTickAt=now;
      playModeTick('pin');
    }
  }
}

function resetDevDockPoke(){
  devActiveSliderId=null;
  devActiveSliderProbeId=null;
  devSliderCooldownUntil=0;
  for(const item of devDockItems){
    item.pokeSince=null;
    item.pokeConsumed=false;
    item.bestDist=Infinity;
    item.bestProbeId=null;
  }
}

function updateDevDock(frameInputs,world,anPhase,asPhase,now){
  const anchored=Boolean(world)&&asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created');
  if(!devModeEnabled||!anchored){
    devDockGroup.visible=false;
    hoveredDevDock=false;
    resetDevDockPoke();
    return;
  }
  devDockGroup.visible=true;
  if(leaveConfirmState){
    hoveredDevDock=false;
    resetDevDockPoke();
    return;
  }

  const probes=frameInputs.getProbesL();
  const rightL=frameInputs.viewerRightL;
  const rx=rightL[0],ry=rightL[1],rz=rightL[2];
  const gx=devDockGroup.position.x,gy=devDockGroup.position.y,gz=devDockGroup.position.z;

  if(devActiveSliderId!=null){
    const locked=devDockItems.find(it=>it.spec.id===devActiveSliderId);
    let pr=null;
    if(locked&&devActiveSliderProbeId){
      for(const cand of probes){if(cand.probeId===devActiveSliderProbeId){pr=cand;break;}}
    }
    if(locked&&pr){
      probeDevSlider(locked,pr.pos[0],pr.pos[1],pr.pos[2],gx,gy,gz,rx,ry,rz);
      const perp=_devSliderHit[0],u=_devSliderHit[1];
      if(perp<0.046){
        hoveredDevDock=true;
        recordProbeProximity(pr.probeId,perp);
        applyDevSliderValue(locked,u,now);
        for(const item of devDockItems){
          if(item===locked){
            item.sprite.scale.set(0.118,0.036,1);
            drawDevSlider(item,true,true);
          }else{
            item.pokeSince=null;
            if(item.spec.kind==='slider'){
              item.sprite.scale.set(0.114,0.034,1);
              drawDevSlider(item,false,false);
            }else{
              item.sprite.scale.set(0.094,0.028,1);
              drawDevToggle(item,false,0);
            }
          }
        }
        return;
      }
    }
    devActiveSliderId=null;
    devActiveSliderProbeId=null;
    devSliderCooldownUntil=now+400;
  }

  for(const item of devDockItems){
    item.bestDist=Infinity;
    item.bestU=0;
    item.bestProbeId=null;
  }
  for(const pr of probes){
    const px=pr.pos[0],py=pr.pos[1],pz=pr.pos[2];
    let probeMin=Infinity;
    for(const item of devDockItems){
      if(item.spec.kind==='slider'){
        probeDevSlider(item,px,py,pz,gx,gy,gz,rx,ry,rz);
        const perp=_devSliderHit[0];
        if(perp<probeMin)probeMin=perp;
        if(perp<item.bestDist){
          item.bestDist=perp;
          item.bestU=_devSliderHit[1];
          item.bestProbeId=pr.probeId;
        }
      }else{
        const sp=item.sprite.position;
        const d=Math.hypot(px-(gx+sp.x),py-(gy+sp.y),pz-(gz+sp.z));
        if(d<probeMin)probeMin=d;
        if(d<item.bestDist){
          item.bestDist=d;
          item.bestProbeId=pr.probeId;
        }
      }
    }
    recordProbeProximity(pr.probeId,probeMin);
  }

  let closestItem=null,closestDist=Infinity;
  for(const item of devDockItems){
    const maxHover=item.spec.kind==='slider'?0.036:0.038;
    if(item.bestDist<maxHover&&item.bestDist<closestDist){
      closestDist=item.bestDist;
      closestItem=item;
    }
  }
  hoveredDevDock=Boolean(closestItem);

  for(const item of devDockItems){
    const hot=closestItem===item;
    if(item.spec.kind==='slider'){
      const grab=hot&&now>=devSliderCooldownUntil&&item.bestDist<0.020;
      if(grab){
        devActiveSliderId=item.spec.id;
        devActiveSliderProbeId=item.bestProbeId;
        applyDevSliderValue(item,item.bestU,now);
      }
      item.sprite.scale.set((hot||grab)?0.118:0.114,(hot||grab)?0.036:0.034,1);
      drawDevSlider(item,hot,grab);
    }else{
      item.sprite.scale.set(hot?0.100:0.094,hot?0.031:0.028,1);
      let prog=0;
      if(hot&&now>=devSliderCooldownUntil&&item.bestDist<0.032&&now>=item.cooldownUntil){
        if(!item.pokeConsumed){
          if(item.pokeSince==null)item.pokeSince=now;
          prog=Math.min(1,(now-item.pokeSince)/220);
          if(prog>=1){
            item.pokeConsumed=true;item.pokeSince=null;item.cooldownUntil=now+650;
            item.spec.onToggle();
          }
        }
      }else{
        item.pokeSince=null;
        if(item.bestDist>0.048)item.pokeConsumed=false;
      }
      drawDevToggle(item,hot,prog);
    }
  }
}

// Interactive ribbon waypoint inspector (activated when controller or index fingertip comes within 5 cm of a gray tag frame).
const inspectRoot=new THREE.Group();
inspectRoot.visible=false;
root.add(inspectRoot);
const highlightMat=new THREE.MeshBasicMaterial({color:0xfbbf24,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const highlightFrame=new THREE.Mesh(getThickTagFrameGeo(20),highlightMat);
highlightFrame.matrixAutoUpdate=false;
highlightFrame.renderOrder=10;
inspectRoot.add(highlightFrame);
const highlightPlaneMat=new THREE.MeshBasicMaterial({color:0xfbbf24,transparent:true,opacity:0.16,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const highlightPlane=new THREE.Mesh(new THREE.PlaneGeometry(1,1),highlightPlaneMat);
highlightPlane.matrixAutoUpdate=false;
highlightPlane.renderOrder=9;
inspectRoot.add(highlightPlane);
const highlightOriginDotMat=new THREE.MeshBasicMaterial({color:0xff3b30,depthTest:false,depthWrite:false});
const highlightOriginDot=new THREE.Mesh(_originDotGeo,highlightOriginDotMat);
highlightOriginDot.renderOrder=11;
inspectRoot.add(highlightOriginDot);
// Thick volumetric 3D rods (3.6 mm diameter) all meeting at the table footprint (cx, cy, 0):
// Cyan Z: (cx,cy,cz) -> (cx,cy,0), Red X (at y=cy): (cx,cy,0) -> (0,cy,0), Green Y (down): (cx,cy,0) -> (cx,0,0).
const zRodMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.95});
const xRodMat=new THREE.MeshBasicMaterial({color:0xef4444,transparent:true,opacity:0.95});
const yRodMat=new THREE.MeshBasicMaterial({color:0x22c55e,transparent:true,opacity:0.95});
const floorRodMat=new THREE.MeshBasicMaterial({color:0xf97316,transparent:true,opacity:0.95});

const dropRod=new THREE.Mesh(_rodGeo,zRodMat);
inspectRoot.add(dropRod);
const dropRing=new THREE.Mesh(
  new THREE.RingGeometry(0.0045,0.0095,20),
  new THREE.MeshBasicMaterial({color:0x38bdf8,side:THREE.DoubleSide,transparent:true,opacity:0.95}),
);
inspectRoot.add(dropRing);
const xAxisRod=new THREE.Mesh(_rodGeo,xRodMat);
inspectRoot.add(xAxisRod);
const yAxisRod=new THREE.Mesh(_rodGeo,yRodMat);
inspectRoot.add(yAxisRod);
const xHitRing=new THREE.Mesh(
  new THREE.RingGeometry(0.003,0.0065,16),
  new THREE.MeshBasicMaterial({color:0xf87171,side:THREE.DoubleSide,transparent:true,opacity:0.95}),
);
inspectRoot.add(xHitRing);
const yHitRing=new THREE.Mesh(
  new THREE.RingGeometry(0.003,0.0065,16),
  new THREE.MeshBasicMaterial({color:0x4ade80,side:THREE.DoubleSide,transparent:true,opacity:0.95}),
);
inspectRoot.add(yHitRing);
// Thin full-room L-frame X/Y/Z guide axes (-5 m .. +5 m, 1.6 mm dia) shown while hovering so distant projections terminate cleanly on the room axes.
const roomXMat=new THREE.MeshBasicMaterial({color:0xef4444,transparent:true,opacity:0.48,depthWrite:false});
const roomYMat=new THREE.MeshBasicMaterial({color:0x22c55e,transparent:true,opacity:0.48,depthWrite:false});
const roomZMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.48,depthWrite:false});
const roomXAxisRod=new THREE.Mesh(_rodGeo,roomXMat);
setRodSegment(roomXAxisRod,-5,0,0.0018,5,0,0.0018,0.0008);
inspectRoot.add(roomXAxisRod);
const roomYAxisRod=new THREE.Mesh(_rodGeo,roomYMat);
setRodSegment(roomYAxisRod,0,-5,0.0018,0,5,0.0018,0.0008);
inspectRoot.add(roomYAxisRod);
const roomZAxisRod=new THREE.Mesh(_rodGeo,roomZMat);
setRodSegment(roomZAxisRod,0,0,-5,0,0,5,0.0008);
inspectRoot.add(roomZAxisRod);

// Orange floor extension 3D rod in world space (extends straight down along gravity from L-plane to Quest floor).
const floorInspectGroup=new THREE.Group();
floorInspectGroup.visible=false;
scene.add(floorInspectGroup);
const floorDropRod=new THREE.Mesh(_rodGeo,floorRodMat);
floorInspectGroup.add(floorDropRod);
const floorDropRing=new THREE.Mesh(
  new THREE.RingGeometry(0.009,0.018,24),
  new THREE.MeshBasicMaterial({color:0xf97316,side:THREE.DoubleSide,transparent:true,opacity:0.95}),
);
floorDropRing.rotation.x=-Math.PI/2;
floorInspectGroup.add(floorDropRing);

// Independent per-hand / per-controller world-space fingertip cursors (zero L-frame round-trip, never occluded behind UI sprites).
const _cursorDotGeo=new THREE.SphereGeometry(0.0032,12,12);
const _cursorRingGeo=new THREE.RingGeometry(0.0052,0.0068,24);
const _cursorMat4=new THREE.Matrix4();
const _cursorViewerQuat=new THREE.Quaternion();
const _xrVmMat4=new THREE.Matrix4();
const _gestureMat4=new THREE.Matrix4();
const _scPlaneN=new THREE.Vector3();
const _scPlaneR=new THREE.Vector3();
const _scPlaneU=new THREE.Vector3();
const _scPlaneC=new THREE.Vector3();
const _scRel=new THREE.Vector3();
const probeCursors=new Map();
const probeProximity=new Map();

function recordProbeProximity(probeId,distM){
  if(!probeId||!Number.isFinite(distM))return;
  const prev=probeProximity.get(probeId);
  if(prev==null||distM<prev)probeProximity.set(probeId,distM);
}

function getOrCreateProbeCursor(probeId){
  let c=probeCursors.get(probeId);
  if(c)return c;
  const group=new THREE.Group();
  group.visible=false;
  const dotMat=new THREE.MeshBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.85,depthTest:false,depthWrite:false});
  const dot=new THREE.Mesh(_cursorDotGeo,dotMat);
  dot.renderOrder=1005;
  const ringMat=new THREE.MeshBasicMaterial({color:0x22d3ee,transparent:true,opacity:0.55,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
  const ring=new THREE.Mesh(_cursorRingGeo,ringMat);
  ring.renderOrder=1004;
  group.add(dot,ring);
  scene.add(group);
  c={group,dot,dotMat,ring,ringMat};
  probeCursors.set(probeId,c);
  return c;
}

function hideAllProbeCursors(){
  for(const c of probeCursors.values())c.group.visible=false;
}

// ---- Bilateral hand mode/REC-ALL sphere-joint tint in ONE shared InstancedMesh (0 extra draw calls). ----
const LEFT_HAND_TINT_MEASURE=0x6fcf97,LEFT_HAND_TINT_SCAN=0x7dd3fc,LEFT_HAND_TINT_SCAN_REC=0xef4444;
const RIGHT_HAND_TINT_IDLE=0x7dd3fc,RIGHT_HAND_TINT_REC_ALL=0xef4444;
// Joint order of XRHand (WebXR spec): wrist, thumb(4), index(5), middle(5), ring(5), pinky(5) = 25.
const HAND_TINT_CHAINS=[[0,1,2,3,4],[0,5,6,7,8,9],[0,10,11,12,13,14],[0,15,16,17,18,19],[0,20,21,22,23,24]];
const HAND_PALM_LOOP=[0,1,2,6,11,16,21,20];
// Wrist + metacarpal (palm) joint balls drawn small; bones/palm hull keep the full-size occluder.
const HAND_SPHERE_SCALE=new Float32Array(25).fill(1);for(const j of [0,1,5,10,15,20])HAND_SPHERE_SCALE[j]=0.35;
const HAND_TINT_MAX=64,HAND_BONE_MAX=88,HAND_PALM_VERTS_MAX=2*32*3;
const _tintMats=new Float32Array(25*16),_tintRadii=new Float32Array(25);
const _tintColLeftMeasure=new THREE.Color(LEFT_HAND_TINT_MEASURE),_tintColLeftScan=new THREE.Color(LEFT_HAND_TINT_SCAN),_tintColLeftScanRec=new THREE.Color(LEFT_HAND_TINT_SCAN_REC);
const _tintColRightIdle=new THREE.Color(RIGHT_HAND_TINT_IDLE),_tintColRightRecAll=new THREE.Color(RIGHT_HAND_TINT_REC_ALL);
let handOcclusionEnabled=true;
// Hand glow: 0=OFF, 1=LOW, 2=MID, 3=HIGH (solid joint spheres only; bones/palm stay invisible depth occluders).
const HAND_GLOW_LEVELS=['OFF','LOW','MID','HIGH'],HAND_GLOW_OPACITY=[0,0.10,0.20,0.32];
let handGlowLevel=1;
const handOccluderMat=new THREE.MeshBasicMaterial({colorWrite:false,depthWrite:true,depthTest:true,side:THREE.DoubleSide});
const handTintMat=new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.22,depthTest:false,depthWrite:false,side:THREE.DoubleSide});
const handTintMesh=new THREE.InstancedMesh(new THREE.SphereGeometry(1,12,10),handOccluderMat,HAND_TINT_MAX);
handTintMesh.frustumCulled=false;handTintMesh.renderOrder=-10;handTintMesh.visible=false;handTintMesh.count=0;
handTintMesh.setColorAt(0,_tintColLeftMeasure);scene.add(handTintMesh);
const handBoneMesh=new THREE.InstancedMesh(new THREE.CylinderGeometry(1,1,1,12,1,true),handOccluderMat,HAND_BONE_MAX);
handBoneMesh.frustumCulled=false;handBoneMesh.renderOrder=-10;handBoneMesh.visible=false;handBoneMesh.count=0;
handBoneMesh.setColorAt(0,_tintColLeftMeasure);scene.add(handBoneMesh);
// Visible glow spheres share matrix/color buffers with the occluder sphere mesh (no extra per-frame work).
const handGlowMesh=new THREE.InstancedMesh(handTintMesh.geometry,handTintMat,HAND_TINT_MAX);
handGlowMesh.instanceMatrix=handTintMesh.instanceMatrix;handGlowMesh.instanceColor=handTintMesh.instanceColor;
handGlowMesh.frustumCulled=false;handGlowMesh.renderOrder=990;handGlowMesh.visible=false;handGlowMesh.count=0;scene.add(handGlowMesh);
const handPalmPosArr=new Float32Array(HAND_PALM_VERTS_MAX*3);
const handPalmGeo=new THREE.BufferGeometry();
handPalmGeo.setAttribute('position',new THREE.BufferAttribute(handPalmPosArr,3));
handPalmGeo.setDrawRange(0,0);
const handPalmMesh=new THREE.Mesh(handPalmGeo,handOccluderMat);
handPalmMesh.frustumCulled=false;handPalmMesh.renderOrder=-10;handPalmMesh.visible=false;
scene.add(handPalmMesh);
const _handUpY=new THREE.Vector3(0,1,0),_handDir=new THREE.Vector3(),_handPos=new THREE.Vector3(),_handScale=new THREE.Vector3(),_handQuat=new THREE.Quaternion(),_handMat4=new THREE.Matrix4();
function fallbackJointRadius(idx){return idx===0?0.020:(idx%5===4?0.0095:(idx%5===1?0.0155:0.0120));}
function appendHandOccluder(handInput,col,sphereMatArr,sphereColArr,boneMatArr,boneColArr,palmArr,counts){
  if(!handInput?.fillJointMatrices||!handInput.fillJointMatrices(_tintMats))return;
  const hasRadii=Boolean(handInput.fillJointRadii&&handInput.fillJointRadii(_tintRadii));
  const cr=col.r,cg=col.g,cb=col.b;
  const jR=idx=>{
    const r=hasRadii?_tintRadii[idx]:NaN;
    return (Number.isFinite(r)&&r>0.004&&r<0.045)?r*1.20:fallbackJointRadius(idx);
  };
  const pushBone=(ia,ib,scaleR=1)=>{
    if(counts.bones>=HAND_BONE_MAX)return;
    const oa=ia*16,ob=ib*16;
    const ax=_tintMats[oa+12],ay=_tintMats[oa+13],az=_tintMats[oa+14];
    const bx=_tintMats[ob+12],by=_tintMats[ob+13],bz=_tintMats[ob+14];
    if(!Number.isFinite(ax)||!Number.isFinite(bx))return;
    const dx=bx-ax,dy=by-ay,dz=bz-az,len=Math.hypot(dx,dy,dz);
    if(len<1e-4)return;
    const r=Math.max(jR(ia),jR(ib))*scaleR;
    _handDir.set(dx/len,dy/len,dz/len);
    if(_handDir.y<-0.9999)_handQuat.set(1,0,0,0);
    else _handQuat.setFromUnitVectors(_handUpY,_handDir);
    _handPos.set((ax+bx)*0.5,(ay+by)*0.5,(az+bz)*0.5);
    _handScale.set(r,len,r);
    _handMat4.compose(_handPos,_handQuat,_handScale);
    const bIdx=counts.bones++;
    _handMat4.toArray(boneMatArr,bIdx*16);
    const bc=bIdx*3;boneColArr[bc]=cr;boneColArr[bc+1]=cg;boneColArr[bc+2]=cb;
  };
  // 1. Joint spheres (25 joints per hand)
  for(let idx=0;idx<25;idx++){
    if(counts.spheres>=HAND_TINT_MAX)break;
    const o=idx*16,x=_tintMats[o+12],y=_tintMats[o+13],z=_tintMats[o+14];
    if(!Number.isFinite(x))continue;
    const r=jR(idx)*HAND_SPHERE_SCALE[idx];
    const m=counts.spheres*16,c=counts.spheres*3;counts.spheres++;
    sphereMatArr[m]=r;sphereMatArr[m+1]=0;sphereMatArr[m+2]=0;sphereMatArr[m+3]=0;
    sphereMatArr[m+4]=0;sphereMatArr[m+5]=r;sphereMatArr[m+6]=0;sphereMatArr[m+7]=0;
    sphereMatArr[m+8]=0;sphereMatArr[m+9]=0;sphereMatArr[m+10]=r;sphereMatArr[m+11]=0;
    sphereMatArr[m+12]=x;sphereMatArr[m+13]=y;sphereMatArr[m+14]=z;sphereMatArr[m+15]=1;
    sphereColArr[c]=cr;sphereColArr[c+1]=cg;sphereColArr[c+2]=cb;
  }
  // 2. Continuous bone cylinders + rounded palm perimeter capsules (zero gaps, smooth outer hand edges)
  for(const chain of HAND_TINT_CHAINS){
    for(let k=1;k<chain.length;k++)pushBone(chain[k-1],chain[k],1.0);
  }
  for(let i=0;i<HAND_PALM_LOOP.length;i++){
    pushBone(HAND_PALM_LOOP[i],HAND_PALM_LOOP[(i+1)%HAND_PALM_LOOP.length],1.05);
  }
  // 3. Solid extruded palm hull (eliminates gaps across the palm and thumb-index web)
  let cx=0,cy=0,cz=0;
  for(let i=0;i<8;i++){
    const o=HAND_PALM_LOOP[i]*16;
    if(!Number.isFinite(_tintMats[o+12]))return;
    cx+=_tintMats[o+12];cy+=_tintMats[o+13];cz+=_tintMats[o+14];
  }
  cx/=8;cy/=8;cz/=8;
  const wx=_tintMats[12],wy=_tintMats[13],wz=_tintMats[14];
  const mpx=_tintMats[11*16+12]-wx,mpy=_tintMats[11*16+13]-wy,mpz=_tintMats[11*16+14]-wz;
  const ipx=_tintMats[6*16+12]-_tintMats[21*16+12],ipy=_tintMats[6*16+13]-_tintMats[21*16+13],ipz=_tintMats[6*16+14]-_tintMats[21*16+14];
  let nx=mpy*ipz-mpz*ipy,ny=mpz*ipx-mpx*ipz,nz=mpx*ipy-mpy*ipx;
  const nLen=Math.hypot(nx,ny,nz);
  if(nLen<1e-5)return;
  nx/=nLen;ny/=nLen;nz/=nLen;
  const halfT=Math.max(0.014,Math.min(0.024,jR(10)*1.18));
  const pushTri=(ax,ay,az,bx,by,bz,px,py,pz)=>{
    let v=counts.palmVerts*3;
    if(counts.palmVerts+3>HAND_PALM_VERTS_MAX)return;
    palmArr[v]=ax;palmArr[v+1]=ay;palmArr[v+2]=az;
    palmArr[v+3]=bx;palmArr[v+4]=by;palmArr[v+5]=bz;
    palmArr[v+6]=px;palmArr[v+7]=py;palmArr[v+8]=pz;
    counts.palmVerts+=3;
  };
  const ctx=cx+nx*halfT,cty=cy+ny*halfT,ctz=cz+nz*halfT;
  const cbx=cx-nx*halfT,cby=cy-ny*halfT,cbz=cz-nz*halfT;
  for(let i=0;i<8;i++){
    const oa=HAND_PALM_LOOP[i]*16,ob=HAND_PALM_LOOP[(i+1)%8]*16;
    const ax=_tintMats[oa+12],ay=_tintMats[oa+13],az=_tintMats[oa+14];
    const bx=_tintMats[ob+12],by=_tintMats[ob+13],bz=_tintMats[ob+14];
    const atx=ax+nx*halfT,aty=ay+ny*halfT,atz=az+nz*halfT;
    const btx=bx+nx*halfT,bty=by+ny*halfT,btz=bz+nz*halfT;
    const abx=ax-nx*halfT,aby=ay-ny*halfT,abz=az-nz*halfT;
    const bbx=bx-nx*halfT,bby=by-ny*halfT,bbz=bz-nz*halfT;
    pushTri(ctx,cty,ctz,atx,aty,atz,btx,bty,btz);
    pushTri(cbx,cby,cbz,bbx,bby,bbz,abx,aby,abz);
    pushTri(atx,aty,atz,abx,aby,abz,btx,bty,btz);
    pushTri(btx,bty,btz,abx,aby,abz,bbx,bby,bbz);
  }
}
function hideHandOccluders(){
  handTintMesh.visible=false;handGlowMesh.visible=false;handBoneMesh.visible=false;handPalmMesh.visible=false;
}
function updateHandTints(leftHandInput,rightHandInput,scanMode,recAllMode,show){
  // Solid watertight hand depth-occluder (joint spheres + continuous bone cylinders + extruded palm hull):
  // defaults to invisible depth-only occlusion so 3D lines, tags, and pills behind the user's real Passthrough hands
  // are cleanly occluded without gaps; optional color tint remains toggleable.
  if(!show||!handOcclusionEnabled){hideHandOccluders();return;}
  const glowOn=handGlowLevel>0;
  if(glowOn)handTintMat.opacity=HAND_GLOW_OPACITY[handGlowLevel];
  const counts={spheres:0,bones:0,palmVerts:0};
  appendHandOccluder(leftHandInput,scanMode?(recAllMode?_tintColLeftScanRec:_tintColLeftScan):_tintColLeftMeasure,handTintMesh.instanceMatrix.array,handTintMesh.instanceColor.array,handBoneMesh.instanceMatrix.array,handBoneMesh.instanceColor.array,handPalmPosArr,counts);
  appendHandOccluder(rightHandInput,recAllMode?_tintColRightRecAll:_tintColRightIdle,handTintMesh.instanceMatrix.array,handTintMesh.instanceColor.array,handBoneMesh.instanceMatrix.array,handBoneMesh.instanceColor.array,handPalmPosArr,counts);
  if(counts.spheres===0&&counts.bones===0&&counts.palmVerts===0){hideHandOccluders();return;}
  handTintMesh.count=counts.spheres;handTintMesh.instanceMatrix.needsUpdate=true;
  if(glowOn)handTintMesh.instanceColor.needsUpdate=true;
  handTintMesh.visible=counts.spheres>0;
  handGlowMesh.count=counts.spheres;handGlowMesh.visible=glowOn&&counts.spheres>0;
  handBoneMesh.count=counts.bones;handBoneMesh.instanceMatrix.needsUpdate=true;
  handBoneMesh.visible=counts.bones>0;
  handPalmGeo.attributes.position.needsUpdate=true;
  handPalmGeo.setDrawRange(0,counts.palmVerts);
  handPalmMesh.visible=counts.palmVerts>0;
}

function updateFingertipCursors(frameInputs,pose,{hideLeft=false,hideRight=false}={}){
  if(!pose||!session||!space){hideAllProbeCursors();return;}
  const activeIds=new Set();
  const hasViewer=Boolean(pose?.transform?.matrix);
  if(hasViewer)_cursorViewerQuat.setFromRotationMatrix(_cursorMat4.fromArray(pose.transform.matrix));
  for(const inp of frameInputs.inputs){
    let wx,wy,wz;
    if(inp.kind==='hand'){
      if((hideLeft&&inp.handedness==='left')||(hideRight&&inp.handedness==='right'))continue;
      const tipPos=inp.getIndexTipWorld();
      if(!tipPos)continue;
      [wx,wy,wz]=tipPos;
    }else{
      const tipPos=inp.getCtrlTipWorld();
      if(!tipPos)continue;
      [wx,wy,wz]=tipPos;
    }
    const probeId=inp.probeId;
    activeIds.add(probeId);
    const cur=getOrCreateProbeCursor(probeId);
    cur.group.visible=true;
    cur.group.position.set(wx,wy,wz);
    if(hasViewer)cur.ring.quaternion.copy(_cursorViewerQuat);
    const state=classifyCursorProximity(probeProximity.get(probeId)??Infinity);
    if(state==='hot'){
      cur.dotMat.color.setHex(0xfde047);cur.dotMat.opacity=1.0;cur.dot.scale.setScalar(1.22);
      cur.ringMat.color.setHex(0xfde047);cur.ringMat.opacity=1.0;cur.ring.scale.setScalar(0.88);
    }else if(state==='near'){
      cur.dotMat.color.setHex(0x38bdf8);cur.dotMat.opacity=0.96;cur.dot.scale.setScalar(1.08);
      cur.ringMat.color.setHex(0x38bdf8);cur.ringMat.opacity=0.82;cur.ring.scale.setScalar(1.0);
    }else{
      cur.dotMat.color.setHex(0x38bdf8);cur.dotMat.opacity=0.72;cur.dot.scale.setScalar(0.95);
      cur.ringMat.color.setHex(0xe0f2fe);cur.ringMat.opacity=0.48;cur.ring.scale.setScalar(1.05);
    }
  }
  for(const [id,cur] of probeCursors.entries()){
    if(!activeIds.has(id))cur.group.visible=false;
  }
}

const HOVER_CARD_W_M=HOVER_CALLOUT_DEFAULTS.cardWidthM;
const HOVER_CARD_H_M=HOVER_CALLOUT_DEFAULTS.cardHeightM;
const PIN_PILL_W_M=HOVER_CALLOUT_DEFAULTS.pillWidthM;
const PIN_PILL_H_M=HOVER_CALLOUT_DEFAULTS.pillHeightM;
const inspectCanvas=document.createElement('canvas');
inspectCanvas.width=420;inspectCanvas.height=188;
const inspectTex=makeUiTexture(inspectCanvas);
const inspectMat=new THREE.SpriteMaterial({map:inspectTex,transparent:true,depthTest:false,depthWrite:false});
const inspectSprite=new THREE.Sprite(inspectMat);
inspectSprite.scale.set(HOVER_CARD_W_M,HOVER_CARD_H_M,1);
inspectSprite.renderOrder=11;
inspectRoot.add(inspectSprite);
const inspectLeaderMat=new THREE.MeshBasicMaterial({color:0xfbbf24,transparent:true,opacity:0.72,depthTest:false,depthWrite:false});
const inspectLeaderRod=new THREE.Mesh(_rodGeo,inspectLeaderMat);
inspectLeaderRod.renderOrder=10;
inspectRoot.add(inspectLeaderRod);

// Persistent pinned waypoints (max 6) + pairwise 3D distance measurement rails + live preview rails (all batched).
const MAX_PINNED_POINTS=6;
const pinnedPoints=new Map();
let pinnedRailSlots=[];
let previewRailSlots=[];
let dualHandRailSlots=[];

// Secondary simultaneous hover slot (Dual-Hand Hover: left + right hands hovering 2 different tags at once).
const inspect2Root=new THREE.Group();
inspect2Root.visible=false;
root.add(inspect2Root);
const highlight2Mat=new THREE.MeshBasicMaterial({color:0x38bdf8,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const highlightFrame2=new THREE.Mesh(getThickTagFrameGeo(20),highlight2Mat);
highlightFrame2.matrixAutoUpdate=false;
highlightFrame2.renderOrder=10;
inspect2Root.add(highlightFrame2);
const highlightPlane2Mat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.16,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const highlightPlane2=new THREE.Mesh(new THREE.PlaneGeometry(1,1),highlightPlane2Mat);
highlightPlane2.matrixAutoUpdate=false;
highlightPlane2.renderOrder=9;
inspect2Root.add(highlightPlane2);
const highlightOriginDot2=new THREE.Mesh(_originDotGeo,highlightOriginDotMat);
highlightOriginDot2.renderOrder=11;
inspect2Root.add(highlightOriginDot2);
const dropRod2=new THREE.Mesh(_rodGeo,zRodMat);
inspect2Root.add(dropRod2);
const dropRing2=new THREE.Mesh(
  new THREE.RingGeometry(0.0045,0.0095,20),
  new THREE.MeshBasicMaterial({color:0x38bdf8,side:THREE.DoubleSide,transparent:true,opacity:0.95}),
);
inspect2Root.add(dropRing2);
const xAxisRod2=new THREE.Mesh(_rodGeo,xRodMat);
inspect2Root.add(xAxisRod2);
const yAxisRod2=new THREE.Mesh(_rodGeo,yRodMat);
inspect2Root.add(yAxisRod2);
const inspect2Canvas=document.createElement('canvas');
inspect2Canvas.width=420;inspect2Canvas.height=188;
const inspect2Tex=makeUiTexture(inspect2Canvas);
const inspect2Mat=new THREE.SpriteMaterial({map:inspect2Tex,transparent:true,depthTest:false,depthWrite:false});
const inspect2Sprite=new THREE.Sprite(inspect2Mat);
inspect2Sprite.scale.set(HOVER_CARD_W_M,HOVER_CARD_H_M,1);
inspect2Sprite.renderOrder=11;
inspect2Root.add(inspect2Sprite);
const inspect2LeaderMat=new THREE.MeshBasicMaterial({color:0x38bdf8,transparent:true,opacity:0.72,depthTest:false,depthWrite:false});
const inspect2LeaderRod=new THREE.Mesh(_rodGeo,inspect2LeaderMat);
inspect2LeaderRod.renderOrder=10;
inspect2Root.add(inspect2LeaderRod);

let hoveredWaypointKey=null;
let hoveredWaypoint=null;
let hoveredWaypoint2=null;
let hoveredWaypoint2Key=null;
let hoverCard2PosL=null;
let hoverCard2ForWpKey=null;
let inspect2Visible=false;
let dualHandRailKey=null;
let hoveredFloorHeightMm=null;
let hoveredClosePinKey=null;
let closePokeKey=null;
let closePokeSinceMs=0;
let previewKey=null;
let inspectVisible=false;

function waypointIdKey(wp){
  if(wp.isStatic)return `static:${wp.tagId}`;
  if(wp.isCurrent)return `current:${wp.tagId}`;
  if(wp.isTimeCopy)return `tc:${wp.tagId}:${wp.timeCopyIndex}`;
  return `${wp.tagId}:${wp.index}`;
}
function waypointTitle(wp){
  if(wp.isPerimeter)return `${wp.perimeterRole} · PERIMETER`;
  if(wp.isStatic)return `TAG #${wp.tagId} · STATIC`;
  if(wp.isCurrent)return `TAG #${wp.tagId} · LOCKED`;
  if(wp.isTimeCopy)return `TAG #${wp.tagId} · T${wp.timeCopyIndex}`;
  return `TAG #${wp.tagId} · PT #${wp.index}`;
}
function waypointShortLabel(wp){
  if(wp.isPerimeter)return wp.perimeterRole;
  if(wp.isStatic)return `#${wp.tagId} · STATIC`;
  if(wp.isCurrent)return `#${wp.tagId}`;
  if(wp.isTimeCopy)return `#${wp.tagId} · T${wp.timeCopyIndex}`;
  return `#${wp.tagId} · PT #${wp.index}`;
}

function syncStaticTagBadges(){
  for(const vis of perimeterTagVisuals.values()){
    if(!vis.wp||vis.badgeSlot<0)continue;
    badgeMesh.setQuadVisible(vis.badgeSlot,vis.visible&&isPerimeterBadgeVisible(vis));
  }
}

function getEntryPassAnalysis(entry,segmentIndex){
  if(!entry?.linePositions||entry.lineCount<2||!(segmentIndex>=1))return null;
  if(entry.passAnalysisSeq!==entry.lineSeqCounter||!entry.passAnalysisMap){
    entry.passAnalysisMap=new Map();
    entry.passAnalysisSeq=entry.lineSeqCounter;
  }
  if(entry.passAnalysisMap.has(segmentIndex))return entry.passAnalysisMap.get(segmentIndex);
  const analysis=analyzeTrailSegmentPasses({
    linePositions:entry.linePositions,
    lineCount:entry.lineCount,
    lineSegmentIds:entry.lineSegmentIds,
    lineBreaks:entry.lineBreaks,
    segmentIndex,
  });
  entry.passAnalysisMap.set(segmentIndex,analysis);
  return analysis;
}

function getWaypointPassMeta(wp){
  if(!wp?.isTrailPoint||!(wp.segmentIndex>=1)||!Number.isFinite(wp.lineSeq))return null;
  const entry=movableTools.get(wp.tagId);
  const analysis=getEntryPassAnalysis(entry,wp.segmentIndex);
  if(!analysis)return null;
  const lineIdx=analysis.resolveSeqIndex(entry.lineSeqs,wp.lineSeq);
  if(lineIdx<0)return null;
  return {
    strandKey:`${wp.tagId}:${wp.segmentIndex}`,
    lineIdx,
    lineSeq:wp.lineSeq,
    passIndex:analysis.passIndexAt(lineIdx),
    passCount:analysis.passCount,
    analysis,
  };
}

function drawWaypointCard(canvas,tex,wp,probeKind,floorHeightMm,{alreadyPinned=false,passInfo=null}={}){
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,420,188);
  ctx.fillStyle=alreadyPinned?'rgba(16,16,32,0.86)':'rgba(8,14,24,0.84)';
  ctx.beginPath();ctx.roundRect(4,4,412,180,14);ctx.fill();
  ctx.strokeStyle=alreadyPinned?'#e879f9':'#fbbf24';ctx.lineWidth=3;ctx.stroke();
  ctx.fillStyle=alreadyPinned?'#f0abfc':'#fbbf24';ctx.font='bold 19px ui-monospace,monospace';
  ctx.fillText(waypointTitle(wp),16,32);
  ctx.fillStyle=alreadyPinned?'#f0abfc':'#94a3b8';
  ctx.font='bold 13px system-ui,sans-serif';
  ctx.textAlign='right';
  const cue=alreadyPinned
    ?(probeKind==='hand'?'PINCH TO UNPIN':'TRIGGER TO UNPIN')
    :(probeKind==='hand'?'PINCH TO PIN':'TRIGGER TO PIN');
  ctx.fillText(cue,404,31);
  ctx.textAlign='left';

  const sx=wp.xMm>=0?'+':'',sy=wp.yMm>=0?'+':'',sz=wp.zMm>=0?'+':'';
  ctx.font='bold 22px ui-monospace,monospace';
  ctx.fillStyle='#f87171';ctx.fillText(`X ${sx}${wp.xMm.toFixed(0)}`,16,66);
  ctx.fillStyle='#4ade80';ctx.fillText(`Y ${sy}${wp.yMm.toFixed(0)} mm`,152,66);
  ctx.fillStyle='#38bdf8';ctx.font='bold 25px ui-monospace,monospace';
  ctx.fillText(`Z ${sz}${wp.zMm.toFixed(0)} mm`,16,102);
  ctx.fillStyle='#94a3b8';ctx.font='15px system-ui,sans-serif';
  ctx.fillText('above table',230,100);
  if(passInfo&&passInfo.passCount>=2){
    ctx.fillStyle='rgba(8,32,46,0.92)';
    ctx.beginPath();ctx.roundRect(320,78,84,28,14);ctx.fill();
    ctx.strokeStyle='#22d3ee';ctx.lineWidth=2;ctx.stroke();
    ctx.fillStyle='#67e8f9';ctx.font='bold 13px ui-monospace,monospace';ctx.textAlign='center';
    ctx.fillText(`PASS ${passInfo.passIndex}/${passInfo.passCount}`,362,97,74);
    ctx.textAlign='left';
  }
  ctx.fillStyle='#fb923c';ctx.font='bold 21px ui-monospace,monospace';
  ctx.fillText(floorHeightMm!=null?`Floor ${floorHeightMm.toFixed(0)} mm`:'Floor —',16,136);
  if(floorHeightMm!=null){
    ctx.fillStyle='#94a3b8';ctx.font='15px system-ui,sans-serif';
    ctx.fillText('above floor',230,134);
  }
  ctx.fillStyle='#cbd5e1';ctx.font='bold 16px ui-monospace,monospace';
  const speedText=Number.isFinite(wp.speedMps)
    ?(wp.speedMps<1.0?` · v ${Math.round(wp.speedMps*1000)}mm/s`:` · v ${wp.speedMps.toFixed(2)}m/s`)
    :'';
  const meta=wp.isPerimeter
    ?`PERIMETER ${wp.perimeterRole} · ${wp.edgeMm}mm · ${wp.clock}`
    :(wp.isStatic
      ?`STATIC · ${wp.edgeMm}mm · ${wp.clock} · tilt ${wp.tiltDeg.toFixed(0)}°`
      :(wp.isCurrent
        ?`LOCKED · ${wp.edgeMm}mm · ${wp.clock} · tilt ${wp.tiltDeg.toFixed(0)}°`
        :(wp.isTimeCopy
          ?`TIME-COPY · ${wp.clock} · tilt ${wp.tiltDeg.toFixed(0)}°`
          :`t +${wp.elapsedS.toFixed(1)}s${speedText} · tilt ${wp.tiltDeg.toFixed(0)}°`)));
  ctx.fillText(meta,16,168,388);
  tex.needsUpdate=true;
}

function drawPinPillAtlas(slot,wp,{closeHot=false}={}){
  if(slot<0)return;
  const idLabel=wp.isPerimeter?wp.perimeterRole:(wp.isStatic||wp.isCurrent?`#${wp.tagId}`:(wp.isTimeCopy?`#${wp.tagId}·T${wp.timeCopyIndex}`:`#${wp.tagId}·${wp.index}`));
  const sz=wp.zMm>=0?'+':'';
  const zText=`Z ${sz}${wp.zMm.toFixed(0)}mm`;
  const key=`pin:${idLabel}:${zText}:${closeHot?1:0}`;
  badgeAtlas.draw(slot,key,(ctx,w,h)=>{
    ctx.scale(w/340,h/84);
    ctx.fillStyle='rgba(14,18,34,0.86)';
    ctx.beginPath();ctx.roundRect(3,3,334,78,39);ctx.fill();
    ctx.strokeStyle='#e879f9';ctx.lineWidth=3.5;ctx.stroke();
    ctx.fillStyle='#f0abfc';ctx.font='bold 23px ui-monospace,monospace';
    ctx.fillText(idLabel,20,50);
    ctx.fillStyle='#38bdf8';ctx.font='bold 24px ui-monospace,monospace';
    ctx.fillText(zText,130,50);
    // Crisp red circular [×] close button at the right end of the pill.
    ctx.fillStyle=closeHot?'#ef4444':'#b91c1c';
    ctx.beginPath();ctx.arc(304,42,23,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=closeHot?'#fef08a':'#fca5a5';ctx.lineWidth=2.5;ctx.stroke();
    ctx.fillStyle='#ffffff';ctx.font='bold 28px system-ui,sans-serif';
    ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText('×',304,43);
    ctx.textAlign='left';ctx.textBaseline='alphabetic';
  });
}

function computeSameTrailSpanInfo(wpA,wpB){
  if(
    !wpA?.isTrailPoint||!wpB?.isTrailPoint||
    wpA.tagId!==wpB.tagId||
    !(wpA.segmentIndex>=1)||wpA.segmentIndex!==wpB.segmentIndex||
    !Number.isFinite(wpA.lineSeq)||!Number.isFinite(wpB.lineSeq)
  ){
    return null;
  }
  const entry=movableTools.get(wpA.tagId);
  if(!entry?.linePositions||entry.lineCount<2)return null;
  const passAnalysis=getEntryPassAnalysis(entry,wpA.segmentIndex);
  const span=measureTrailSpan({
    linePositions:entry.linePositions,
    lineCount:entry.lineCount,
    lineSegmentIds:entry.lineSegmentIds,
    lineSeqs:entry.lineSeqs,
    lineBreaks:entry.lineBreaks,
    segmentIndex:wpA.segmentIndex,
    seqA:wpA.lineSeq,
    seqB:wpB.lineSeq,
    passAnalysis,
  });
  return span?{pathLenMm:span.arcLengthM*1000,reversalCount:span.reversalCount,hasReversal:span.hasReversal}:null;
}

function drawDistanceBadgeAtlas(slot,distMm,dzMm,dtSec,preview=false,pathLenMm=null,reversalCount=0){
  if(slot<0)return;
  const dzSign=dzMm>=0?'+':'';
  const sub=dtSec!=null?`ΔZ ${dzSign}${dzMm.toFixed(0)} mm · Δt ${dtSec.toFixed(1)}s`:`ΔZ ${dzSign}${dzMm.toFixed(0)} mm`;
  if(Number.isFinite(pathLenMm)){
    const resolvedPathMm=Math.max(distMm,pathLenMm);
    const directText=`${distMm.toFixed(0)} mm`;
    const pathText=`${resolvedPathMm.toFixed(0)} mm`;
    const revCount=reversalCount>0?(reversalCount|0):0;
    const key=`dist2:${directText}:${pathText}:${sub}:${preview?1:0}:${revCount}`;
    badgeAtlas.draw(slot,key,(ctx,w,h)=>{
      ctx.scale(w/256,h/96);
      ctx.fillStyle=preview?'rgba(24,14,42,0.78)':'rgba(14,10,30,0.90)';
      ctx.beginPath();ctx.roundRect(3,3,250,90,12);ctx.fill();
      ctx.strokeStyle=preview?'#a855f7':'#e879f9';ctx.lineWidth=2.5;ctx.stroke();

      ctx.strokeStyle='#f0abfc';ctx.lineWidth=2.5;ctx.lineCap='round';
      ctx.beginPath();ctx.moveTo(16,21);ctx.lineTo(42,21);ctx.stroke();
      ctx.fillStyle='#f0abfc';
      ctx.beginPath();ctx.arc(16,21,3,0,Math.PI*2);ctx.arc(42,21,3,0,Math.PI*2);ctx.fill();
      ctx.fillStyle='#ffffff';ctx.font='bold 23px ui-monospace,monospace';ctx.textAlign='left';
      ctx.fillText(directText,50,29);
      ctx.fillStyle='#d8b4fe';ctx.font='bold 12px system-ui,sans-serif';ctx.textAlign='right';
      ctx.fillText('LINE',242,26);

      if(revCount>0){
        ctx.strokeStyle='#fbbf24';ctx.lineWidth=2.5;ctx.lineCap='round';
        ctx.beginPath();ctx.moveTo(16,43);ctx.lineTo(35,43);ctx.arc(35,48,5,-Math.PI/2,Math.PI/2,false);ctx.lineTo(16,53);ctx.stroke();
        ctx.fillStyle='#fbbf24';
        ctx.beginPath();ctx.arc(16,43,2.8,0,Math.PI*2);ctx.arc(16,53,2.8,0,Math.PI*2);ctx.fill();
        ctx.fillStyle='#fde047';ctx.font='bold 23px ui-monospace,monospace';ctx.textAlign='left';
        ctx.fillText(pathText,50,56);
        ctx.fillStyle='#fbbf24';ctx.font='bold 12px system-ui,sans-serif';ctx.textAlign='right';
        ctx.fillText(revCount>1?`↩${revCount} PATH`:'↩ PATH',242,53);
      }else{
        ctx.strokeStyle='#22d3ee';ctx.lineWidth=2.5;ctx.lineCap='round';
        ctx.beginPath();ctx.moveTo(16,52);ctx.bezierCurveTo(23,39,35,57,42,44);ctx.stroke();
        ctx.fillStyle='#22d3ee';
        ctx.beginPath();ctx.arc(16,52,3,0,Math.PI*2);ctx.arc(42,44,3,0,Math.PI*2);ctx.fill();
        ctx.fillStyle='#67e8f9';ctx.font='bold 23px ui-monospace,monospace';ctx.textAlign='left';
        ctx.fillText(pathText,50,56);
        ctx.fillStyle='#67e8f9';ctx.font='bold 12px system-ui,sans-serif';ctx.textAlign='right';
        ctx.fillText('PATH',242,53);
      }

      ctx.strokeStyle='rgba(148,163,184,0.25)';ctx.lineWidth=1;
      ctx.beginPath();ctx.moveTo(14,64);ctx.lineTo(242,64);ctx.stroke();
      ctx.fillStyle='#e9d5ff';ctx.font='bold 14px ui-monospace,monospace';ctx.textAlign='center';
      ctx.fillText(sub,128,83);
      ctx.textAlign='left';
    });
    return;
  }
  const main=`${distMm.toFixed(0)} mm`;
  const key=`dist:${main}:${sub}:${preview?1:0}`;
  badgeAtlas.draw(slot,key,(ctx,w,h)=>{
    ctx.scale(w/280,h/92);
    ctx.fillStyle=preview?'rgba(24,14,42,0.72)':'rgba(15,10,30,0.86)';
    ctx.beginPath();ctx.roundRect(3,3,274,86,12);ctx.fill();
    ctx.strokeStyle=preview?'#a855f7':'#e879f9';ctx.lineWidth=3;ctx.stroke();
    ctx.fillStyle='#ffffff';ctx.font='bold 34px ui-monospace,monospace';ctx.textAlign='center';
    ctx.fillText(main,140,44);
    ctx.fillStyle='#e9d5ff';ctx.font='bold 18px ui-monospace,monospace';
    ctx.fillText(sub,140,74);
    ctx.textAlign='left';
  });
}

function releaseRailSlots(list){
  for(const item of list){
    if(item.rodSlot>=0)rodBatch.free(item.rodSlot);
    if(item.nodeASlot>=0)nodeBatch.free(item.nodeASlot);
    if(item.nodeBSlot>=0)nodeBatch.free(item.nodeBSlot);
    if(item.badgeSlot>=0){badgeMesh.clearSlot(item.badgeSlot);badgeAtlas.release(item.badgeSlot);}
  }
  list.length=0;
}

function addRailSegment(slotList,wpA,wpB,preview=false,includeEndpoints=false){
  const [ax,ay,az]=wpA.centerL,[bx,by,bz]=wpB.centerL;
  const distMm=Math.hypot(bx-ax,by-ay,bz-az)*1000;
  const dzMm=(bz-az)*1000;
  const dtSec=(wpA.atMs!=null&&wpB.atMs!=null)?Math.abs(wpB.atMs-wpA.atMs)/1000:null;
  const spanInfo=computeSameTrailSpanInfo(wpA,wpB);
  const pathLenMm=spanInfo?spanInfo.pathLenMm:null;
  const reversalCount=spanInfo?spanInfo.reversalCount:0;
  const rodSlot=rodBatch.alloc();
  if(rodSlot>=0){
    rodBatch.setRod(rodSlot,ax,ay,az,bx,by,bz,preview?0.0011:0.0014);
    rodBatch.setColor(rodSlot,preview?0xc084fc:0xe879f9);
  }
  let nodeASlot=-1,nodeBSlot=-1;
  if(includeEndpoints){
    nodeASlot=nodeBatch.alloc();
    if(nodeASlot>=0){nodeBatch.setPoint(nodeASlot,ax,ay,az,1);nodeBatch.setColor(nodeASlot,0xf0abfc);}
    nodeBSlot=nodeBatch.alloc();
    if(nodeBSlot>=0){nodeBatch.setPoint(nodeBSlot,bx,by,bz,1);nodeBatch.setColor(nodeBSlot,0xf0abfc);}
  }
  const badgeSlot=badgeAtlas.acquire();
  if(badgeSlot>=0){
    drawDistanceBadgeAtlas(badgeSlot,distMm,dzMm,dtSec,preview,pathLenMm,reversalCount);
    const hasPath=Number.isFinite(pathLenMm);
    const badgeW=hasPath?(preview?0.092:0.098):(preview?0.082:0.088);
    const badgeH=hasPath?(preview?0.035:0.037):(preview?0.026:0.029);
    badgeMesh.setQuad(badgeSlot,[(ax+bx)/2,(ay+by)/2,(az+bz)/2+0.020],badgeW,badgeH,true);
  }
  slotList.push({rodSlot,nodeASlot,nodeBSlot,badgeSlot});
}

function recomputePinnedPillLayout(){
  if(pinnedPoints.size===0)return;
  const pillInputs=[];
  for(const pin of pinnedPoints.values()){
    pillInputs.push({key:pin.key,centerL:pin.wp.centerL,edgeMm:pin.wp.edgeMm,isStatic:pin.wp.isStatic||pin.wp.isCurrent});
  }
  const layouts=computePinPillPositions(pillInputs,lastViewerBasis||{});
  for(const pin of pinnedPoints.values()){
    const layout=layouts.get(pin.key);
    if(!layout)continue;
    pin.pillPosL=layout.pillPos.slice();
    const [px,py,pz]=pin.pillPosL;
    if(pin.pillSlot>=0)badgeMesh.setQuad(pin.pillSlot,[px,py,pz],PIN_PILL_W_M,PIN_PILL_H_M,true);
    const [cx,cy,cz]=pin.wp.centerL;
    const ax=px-layout.screenUp[0]*(PIN_PILL_H_M*0.46);
    const ay=py-layout.screenUp[1]*(PIN_PILL_H_M*0.46);
    const az=pz-layout.screenUp[2]*(PIN_PILL_H_M*0.46);
    overlayRodBatch.setRod(pin.leaderRodSlot,cx,cy,cz,ax,ay,az,0.0007);
    const rx=px+layout.screenRight[0]*(PIN_PILL_W_M*0.40);
    const ry=py+layout.screenRight[1]*(PIN_PILL_W_M*0.40);
    const rz=pz+layout.screenRight[2]*(PIN_PILL_W_M*0.40);
    pin.closePosL=[rx,ry,rz];
    closeOrbBatch.setPoint(pin.closeOrbSlot,rx,ry,rz,pin.closeHot?1.35:1.0);
  }
}

function rebuildPinnedRails(){
  releaseRailSlots(pinnedRailSlots);
  const list=[...pinnedPoints.values()];
  for(let i=0;i<list.length;i++){
    for(let j=i+1;j<list.length;j++){
      addRailSegment(pinnedRailSlots,list[i].wp,list[j].wp,false,false);
    }
  }
  previewKey=null;
  recomputePinnedPillLayout();
}

function pinWaypoint(wp,floorHeightMm){
  const key=waypointIdKey(wp);
  if(pinnedPoints.has(key))return;
  if(pinnedPoints.size>=MAX_PINNED_POINTS){
    const oldestKey=pinnedPoints.keys().next().value;
    unpinWaypoint(oldestKey);
  }
  const [cx,cy,cz]=wp.centerL;
  const frameSlot=overlayFrameBatch.alloc();
  if(frameSlot>=0){
    setTagFramePose(overlayFrameBatch,frameSlot,wp.matrix,0,(wp.edgeMm/20)*1.12);
    overlayFrameBatch.setColor(frameSlot,0xfbbf24);
  }
  const pinNodeSlot=nodeBatch.alloc();
  if(pinNodeSlot>=0){nodeBatch.setPoint(pinNodeSlot,cx,cy,cz,1);nodeBatch.setColor(pinNodeSlot,0xf0abfc);}
  const zRodSlot=rodBatch.alloc();
  if(zRodSlot>=0){rodBatch.setRod(zRodSlot,cx,cy,cz,cx,cy,0,0.0010);rodBatch.setColor(zRodSlot,0x38bdf8);}
  const zRingSlot=zRingBatch.alloc();
  if(zRingSlot>=0){zRingBatch.setPoint(zRingSlot,cx,cy,0.0014,1);}
  const leaderRodSlot=overlayRodBatch.alloc();
  if(leaderRodSlot>=0){overlayRodBatch.setColor(leaderRodSlot,0xe879f9);}

  const initLayout=computePinPillPositions([{key,centerL:wp.centerL,edgeMm:wp.edgeMm,isStatic:wp.isStatic}],lastViewerBasis||{}).get(key);
  const pillPosL=initLayout?initLayout.pillPos.slice():[cx,cy,Math.max(HOVER_CALLOUT_DEFAULTS.minPillTableZ,cz+0.048)];
  const anchorPosL=initLayout?initLayout.anchorPos.slice():[cx,cy,pillPosL[2]-PIN_PILL_H_M*0.46];
  const closePosL=initLayout?initLayout.closePosL.slice():[pillPosL[0]+PIN_PILL_W_M*0.40,pillPosL[1],pillPosL[2]];
  if(leaderRodSlot>=0)overlayRodBatch.setRod(leaderRodSlot,cx,cy,cz,anchorPosL[0],anchorPosL[1],anchorPosL[2],0.0007);

  const pillSlot=badgeAtlas.acquire();
  if(pillSlot>=0){
    drawPinPillAtlas(pillSlot,wp,{closeHot:false});
    badgeMesh.setQuad(pillSlot,pillPosL,PIN_PILL_W_M,PIN_PILL_H_M,true);
  }
  const closeOrbSlot=closeOrbBatch.alloc();
  if(closeOrbSlot>=0){
    closeOrbBatch.setPoint(closeOrbSlot,closePosL[0],closePosL[1],closePosL[2],1);
    closeOrbBatch.setColor(closeOrbSlot,0xdc2626);
  }
  pinnedPoints.set(key,{
    key,wp,frameSlot,pinNodeSlot,zRodSlot,zRingSlot,leaderRodSlot,pillSlot,closeOrbSlot,
    pillPosL,closePosL,
    floorHeightMm,closeHot:false,pinnedAtMs:performance.now(),settled:false,
  });
  syncStaticTagBadges();
  rebuildPinnedRails();
}

function unpinWaypoint(key){
  const pin=pinnedPoints.get(key);
  if(!pin)return;
  if(pin.frameSlot>=0)overlayFrameBatch.free(pin.frameSlot);
  if(pin.pinNodeSlot>=0)nodeBatch.free(pin.pinNodeSlot);
  if(pin.zRodSlot>=0)rodBatch.free(pin.zRodSlot);
  if(pin.zRingSlot>=0)zRingBatch.free(pin.zRingSlot);
  if(pin.leaderRodSlot>=0)overlayRodBatch.free(pin.leaderRodSlot);
  if(pin.pillSlot>=0){badgeMesh.clearSlot(pin.pillSlot);badgeAtlas.release(pin.pillSlot);}
  if(pin.closeOrbSlot>=0)closeOrbBatch.free(pin.closeOrbSlot);
  pinnedPoints.delete(key);
  if(hoveredClosePinKey===key)hoveredClosePinKey=null;
  if(closePokeKey===key){closePokeKey=null;closePokeSinceMs=0;}
  syncStaticTagBadges();
  rebuildPinnedRails();
}

function syncPinnedWaypointPose(wp){
  const pin=pinnedPoints.get(waypointIdKey(wp));
  if(!pin)return;
  const [cx,cy,cz]=wp.centerL;
  pin.wp=wp;
  setTagFramePose(overlayFrameBatch,pin.frameSlot,wp.matrix,0,(wp.edgeMm/20)*1.12);
  nodeBatch.setPoint(pin.pinNodeSlot,cx,cy,cz,1);
  rodBatch.setRod(pin.zRodSlot,cx,cy,cz,cx,cy,0,0.0010);
  zRingBatch.setPoint(pin.zRingSlot,cx,cy,0.0014,1);
  drawPinPillAtlas(pin.pillSlot,wp,{closeHot:pin.closeHot});
  rebuildPinnedRails();
}

function triggerPinFlash(worldPos,color,symbol){
  pinFlashPos=worldPos;
  pinFlashUntilMs=performance.now()+380;
  pinFlashColor=color;
  pinFlashSymbol=symbol;
}

function renderPinFlash(now){
  if(liveTouchLoaderActive)return true;
  if(pinFlashPos&&now<pinFlashUntilMs){
    const t=1-(pinFlashUntilMs-now)/380;
    pinLoadSprite.visible=true;
    pinLoadSprite.position.set(pinFlashPos[0],pinFlashPos[1],pinFlashPos[2]);
    const sc=0.040*(1+t*0.36);
    pinLoadSprite.scale.set(sc,sc,1);
    pinLoadMat.opacity=Math.max(0,1-t*t);
    drawPinLoader(1,pinFlashColor,pinFlashSymbol,true);
    return true;
  }
  pinLoadSprite.visible=false;
  pinLoadMat.opacity=1;
  return false;
}

function togglePinWaypoint(wp,floorHeightMm,flashWorldPos=null){
  const key=waypointIdKey(wp);
  const wasPinned=pinnedPoints.has(key);
  if(wasPinned){
    unpinWaypoint(key);
    playModeTick('unpin');
    showHudToast(`✕ UNPINNED ${waypointShortLabel(wp)}`,'#f87171',1200);
    if(flashWorldPos)triggerPinFlash(flashWorldPos,'#ef4444','×');
    return false;
  }
  pinWaypoint(wp,floorHeightMm);
  playModeTick('pin');
  showHudToast(`✓ PINNED ${waypointShortLabel(wp)}`,'#e879f9',1300);
  if(flashWorldPos)triggerPinFlash(flashWorldPos,'#22c55e','✓');
  return true;
}

function clearPins(){
  for(const key of [...pinnedPoints.keys()])unpinWaypoint(key);
  releaseRailSlots(pinnedRailSlots);
  releaseRailSlots(previewRailSlots);
  previewKey=null;hoveredClosePinKey=null;closePokeKey=null;closePokeSinceMs=0;
}

function resetPinHold(){
  pinHoldWp=null;pinHoldKey=null;pinHoldHand=null;pinHoldFloorMm=null;pinHoldSince=null;pinHoldConsumed=false;
}

function clearHover2(){
  inspect2Root.visible=false;
  if(dualHandRailKey!==null){releaseRailSlots(dualHandRailSlots);dualHandRailKey=null;}
  hoveredWaypoint2=null;hoveredWaypoint2Key=null;
  hoverCard2PosL=null;hoverCard2ForWpKey=null;inspect2Visible=false;
}

function hideWaypointInspector(){
  inspectRoot.visible=false;floorInspectGroup.visible=false;
  releaseRailSlots(previewRailSlots);
  clearHover2();
  hoveredWaypointKey=null;hoveredWaypoint=null;hoveredFloorHeightMm=null;
  hoveredClosePinKey=null;closePokeKey=null;closePokeSinceMs=0;
  previewKey=null;inspectVisible=false;
  hoverCardPosL=null;hoverCardForWpKey=null;
  inspectMat.opacity=1;inspectLeaderMat.opacity=0.72;
  resetPinHold();
  syncStaticTagBadges();
}

function worldPointToL(invWorld,x,y,z){
  return [
    invWorld[0]*x+invWorld[4]*y+invWorld[8]*z+invWorld[12],
    invWorld[1]*x+invWorld[5]*y+invWorld[9]*z+invWorld[13],
    invWorld[2]*x+invWorld[6]*y+invWorld[10]*z+invWorld[14],
  ];
}

function worldVecToL(invWorld,x,y,z){
  return [
    invWorld[0]*x+invWorld[4]*y+invWorld[8]*z,
    invWorld[1]*x+invWorld[5]*y+invWorld[9]*z,
    invWorld[2]*x+invWorld[6]*y+invWorld[10]*z,
  ];
}

function lPointToWorld(world,x,y,z){
  return [
    world[0]*x+world[4]*y+world[8]*z+world[12],
    world[1]*x+world[5]*y+world[9]*z+world[13],
    world[2]*x+world[6]*y+world[10]*z+world[14],
  ];
}

function collectFrameInputs(frame,world,pose){
  const invWorld=world?inverse(world):null;
  let viewerPosL=null,viewerRightL=[1,0,0],viewerUpL=[0,0,1];
  if(invWorld&&pose?.transform?.matrix){
    const vm=pose.transform.matrix;
    viewerPosL=worldPointToL(invWorld,vm[12],vm[13],vm[14]);
    viewerRightL=worldVecToL(invWorld,vm[0],vm[1],vm[2]);
    viewerUpL=worldVecToL(invWorld,vm[4],vm[5],vm[6]);
  }
  const inputs=[];
  const hands={left:null,right:null};
  let axPressed=false;
  let probesLCache=null;
  if(session&&space){
    for(const src of session.inputSources){
      if(!axPressed&&src?.gamepad?.buttons?.[4]?.pressed===true)axPressed=true;
      const handedness=src.handedness==='left'||src.handedness==='right'?src.handedness:null;
      let rayPoseLoaded=false,rayPoseVal=null;
      const getTargetRayPose=()=>{
        if(!rayPoseLoaded){
          rayPoseLoaded=true;
          rayPoseVal=src.targetRaySpace?(frame.getPose?.(src.targetRaySpace,space)??null):null;
        }
        return rayPoseVal;
      };
      if(src.hand){
        const jointCache=Object.create(null);
        let idxTipLoaded=false,idxTipVal=null;
        const getJoint=name=>{
          if(name in jointCache)return jointCache[name];
          const j=src.hand.get(name);
          const jp=j?(frame.getJointPose?.(j,space)??null):null;
          jointCache[name]=jp;
          return jp;
        };
        const getIndexTipWorld=()=>{
          if(idxTipLoaded)return idxTipVal;
          idxTipLoaded=true;
          const jp=getJoint('index-finger-tip');
          if(!jp){idxTipVal=null;return null;}
          const p=jp.transform.position;
          const dp=getJoint('index-finger-phalanx-distal')||getJoint('index-finger-phalanx-intermediate');
          const refPos=dp?[dp.transform.position.x,dp.transform.position.y,dp.transform.position.z]:null;
          idxTipVal=projectIndexSelectorPos([p.x,p.y,p.z],refPos,INDEX_TIP_HOVER_OFFSET_M);
          return idxTipVal;
        };
        const item={
          src,kind:'hand',handedness,probeId:`hand:${src.handedness||'unknown'}`,
          hand:src.hand,getJoint,getIndexTipWorld,getTargetRayPose,
          fillJointMatrices:out=>{try{return Boolean(frame.fillPoses?.(src.hand.values(),space,out));}catch{return false;}},
          fillJointRadii:out=>{try{return Boolean(frame.fillJointRadii?.(src.hand.values(),out));}catch{return false;}},
          getCtrlPose:()=>null,getCtrlTipWorld:()=>null,
        };
        inputs.push(item);
        if(handedness==='left'&&!hands.left)hands.left=item;
        else if(handedness==='right'&&!hands.right)hands.right=item;
      }else{
        let ctrlPoseLoaded=false,ctrlPoseVal=null,ctrlTipLoaded=false,ctrlTipVal=null;
        const getCtrlPose=()=>{
          if(!ctrlPoseLoaded){
            ctrlPoseLoaded=true;
            const sp=src.gripSpace||src.targetRaySpace;
            ctrlPoseVal=sp?(frame.getPose?.(sp,space)??null):null;
          }
          return ctrlPoseVal;
        };
        const getCtrlTipWorld=()=>{
          if(ctrlTipLoaded)return ctrlTipVal;
          ctrlTipLoaded=true;
          const cp=getCtrlPose();
          if(!cp){ctrlTipVal=null;return null;}
          const m=cp.transform.matrix;
          // Probe sits 4 cm forward along controller nose (-Z) for intuitive pointing.
          ctrlTipVal=[m[12]-m[8]*0.04,m[13]-m[9]*0.04,m[14]-m[10]*0.04];
          return ctrlTipVal;
        };
        inputs.push({
          src,kind:'ctrl',handedness,probeId:`ctrl:${src.handedness||'unknown'}`,
          hand:null,getJoint:()=>null,getIndexTipWorld:()=>null,getTargetRayPose,
          getCtrlPose,getCtrlTipWorld,
        });
      }
    }
  }
  const getProbesL=()=>{
    if(probesLCache!==null)return probesLCache;
    if(!invWorld){probesLCache=[];return probesLCache;}
    probesLCache=[];
    for(const inp of inputs){
      const posW=inp.kind==='hand'?inp.getIndexTipWorld():inp.getCtrlTipWorld();
      if(posW){
        probesLCache.push({
          pos:worldPointToL(invWorld,posW[0],posW[1],posW[2]),
          kind:inp.kind,handedness:inp.handedness,probeId:inp.probeId,src:inp.src,input:inp,
        });
      }
    }
    return probesLCache;
  };
  return {frame,invWorld,viewerPosL,viewerRightL,viewerUpL,inputs,hands,axPressed,getProbesL};
}

function resetSpatialPortals(){
  hoveredPortal=null;portalHoldId=null;portalHoldSinceMs=0;portalWarpState=null;showcaseNavState=null;
  closeLeaveConfirm();
  hudRayDot.visible=false;
  centerConfirmUntilMs=0;
  warpBannerSprite.visible=false;
  showcaseOpen=false;showcaseGroup.visible=false;showcaseHoverZone=null;showcasePokeZone=null;showcasePokeSince=0;
  for(const p of spatialPortals){
    p.lift=0;p.liftGroup.visible=false;p.pillars.visible=false;p.holdRing.visible=false;
    p.baseMat.opacity=p.spec.idleHint?0.55:0;
    if(p.glowMat)p.glowMat.opacity=0.14;
    for(const sp of p.sparkles||[]){sp.spr.visible=true;sp.mat.opacity=0.75;}
    if(p.idleGlyph)p.idleGlyph.spr.visible=true;
    for(const sw of p.shockwaves){sw.mesh.visible=false;sw.mat.opacity=0;}
  }
}

function triggerSpatialPortal(portal,now){
  if(!portal||portalWarpState||showcaseNavState)return;
  portalHoldId=null;portalHoldSinceMs=0;
  if(portal.spec.panel){
    toggleShowcasePanel(!showcaseOpen);
    return;
  }
  // Extra safety confirmation step before leaving the XR demo for external links (e.g. eb.mess.icu).
  openLeaveConfirm(portal.spec.title,portal.spec.url,portal.spec.accentCss||'#38bdf8');
}

function classifyShowcaseUv(u,v){
  // Canvas is 960 x 460; UV (0,0) is bottom-left, (1,1) is top-right.
  // Cards themselves are pure info; only [×] and the two bottom URL pills are clickable.
  const cx=u*960,cy=(1-v)*460;
  if(cx>=854&&cx<=946&&cy>=10&&cy<=80)return 'close';
  if(cy>=344&&cy<=394){
    if(cx>=38&&cx<=458)return 'url-mess';
    if(cx>=502&&cx<=922)return 'url-wetter';
  }
  return null;
}

function updateLeaveConfirm(frameInputs,pose,now,world=null){
  if(!leaveConfirmState||!pose||!session||!space){
    leaveConfirmHover=null;
    leaveConfirmPokeZone=null;
    leaveConfirmPokeSince=0;
    return;
  }
  if(leaveConfirmNeedsPlacement){
    placeLeaveConfirm(pose,world);
    leaveConfirmNeedsPlacement=false;
    leaveConfirmMesh.visible=true;
  }
  const vm=pose.transform.matrix;
  _lcEye.set(vm[12],vm[13],vm[14]);
  if(_lcEye.distanceTo(leaveConfirmMesh.position)>LEAVE_AUTO_CLOSE_DIST_M){
    closeLeaveConfirm();
    playModeTick('unpin');
    return;
  }
  leaveConfirmMesh.updateMatrixWorld(true);

  let nextHover=null,directPoke=false,pointerOnPanel=false,bestZDist=Infinity;
  // Pass 1: Direct near-field fingertip / controller-tip touch.
  for(const inp of frameInputs.inputs){
    const tipW=inp.kind==='hand'?inp.getIndexTipWorld():inp.getCtrlTipWorld();
    if(!tipW)continue;
    _lcProbeL.set(tipW[0],tipW[1],tipW[2]);
    leaveConfirmMesh.worldToLocal(_lcProbeL);
    const u=_lcProbeL.x/LEAVE_W_M+0.5;
    const v=_lcProbeL.y/LEAVE_H_M+0.5;
    const zDist=Math.abs(_lcProbeL.z);
    if(u>=-0.04&&u<=1.04&&v>=-0.06&&v<=1.06&&zDist<0.095){
      recordProbeProximity(inp.probeId,zDist);
      const z=classifyLeaveConfirmUv(u,v);
      const better=(z&&!nextHover)||(Boolean(z)===Boolean(nextHover)&&zDist<bestZDist);
      if(better){
        bestZDist=zDist;
        pointerOnPanel=true;
        nextHover=z;
        directPoke=Boolean(z&&zDist<0.040);
        const clampedX=Math.max(-LEAVE_W_M/2,Math.min(LEAVE_W_M/2,_lcProbeL.x));
        const clampedY=Math.max(-LEAVE_H_M/2,Math.min(LEAVE_H_M/2,_lcProbeL.y));
        _lcSurfaceW.set(clampedX,clampedY,0.002);
        leaveConfirmMesh.localToWorld(_lcSurfaceW);
        hudRayDot.visible=true;
        hudRayDot.position.copy(_lcSurfaceW);
        hudRayDotMat.color.setHex(z?(z==='cancel'?0x4ade80:0xfde047):0x7dd3fc);
        hudRayDot.scale.setScalar(z?(directPoke?1.15:0.95):0.65);
      }
    }
  }

  // Pass 2: Far ray-aim fallback whenever direct touch is not hovering a button.
  if(!nextHover){
    for(const inp of frameInputs.inputs){
      const rp=inp.getTargetRayPose();
      if(!rp)continue;
      const rm=rp.transform.matrix;
      _hudRayOrigin.set(rm[12],rm[13],rm[14]);
      _hudRayDir.set(-rm[8],-rm[9],-rm[10]).normalize();
      _hudRaycaster.set(_hudRayOrigin,_hudRayDir);
      const hits=_hudRaycaster.intersectObject(leaveConfirmMesh,false);
      if(hits.length&&hits[0].uv){
        const z=classifyLeaveConfirmUv(hits[0].uv.x,hits[0].uv.y);
        if(z||!pointerOnPanel){
          pointerOnPanel=true;
          nextHover=z;
          hudRayDot.visible=true;
          hudRayDot.position.copy(hits[0].point);
          hudRayDotMat.color.setHex(z?(z==='cancel'?0x4ade80:0xfde047):0x7dd3fc);
          hudRayDot.scale.setScalar(z?1.0:0.65);
          if(z)break;
        }
      }
    }
  }

  // Pass 3: Direct-poke dwell with spawn grace period.
  let pokeProg=0;
  const pokeArmed=(now-leaveConfirmState.openedAt)>=LEAVE_POKE_GRACE_MS;
  if(directPoke&&nextHover&&pokeArmed){
    if(leaveConfirmPokeZone!==nextHover){
      leaveConfirmPokeZone=nextHover;
      leaveConfirmPokeSince=now;
    }else{
      const dur=nextHover==='cancel'?LEAVE_CANCEL_POKE_MS:LEAVE_CONFIRM_POKE_MS;
      pokeProg=Math.min(1,(now-leaveConfirmPokeSince)/dur);
      if(pokeProg>=1){
        const act=nextHover;
        leaveConfirmPokeZone=null;leaveConfirmPokeSince=0;
        executeLeaveConfirm(act,now);
        return;
      }
    }
  }else{
    leaveConfirmPokeZone=null;
    leaveConfirmPokeSince=0;
  }

  leaveConfirmHover=nextHover;
  drawLeaveConfirm(pokeProg);
}

const _scViewerL=new THREE.Vector3();
const _scLocalP=new THREE.Vector3();
function updateSpatialPortals(frameInputs,world,anPhase,asPhase,now,pose=null){
  const anchored=Boolean(world)&&asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created');
  portalRoot.visible=anchored;
  if(!anchored||!spatialPortals.length){
    hoveredPortal=null;portalHoldId=null;portalHoldSinceMs=0;
    if(!portalWarpState&&!showcaseNavState&&centerConfirmUntilMs<=now)warpBannerSprite.visible=false;
    if(showcaseOpen)showcaseGroup.visible=false;
    return;
  }

  // Clean external navigation from showcase cards (mess.engineering / wetter.mess.engineering) -> banner -> leave XR -> open URL.
  if(showcaseNavState){
    const st=showcaseNavState;
    const t=Math.min(1,(now-st.startedAt)/st.durationMs);
    drawWarpBanner(st.title,st.subtitle,st.accentCss,t);
    if(t>=1&&!st.navigated){
      st.navigated=true;
      const targetUrl=st.url;
      const curSession=session;
      if(curSession){
        curSession.end().catch(()=>{}).finally(()=>{window.location.assign(targetUrl);});
      }else{
        window.location.assign(targetUrl);
      }
    }
    return;
  }

  // Clean external navigation (no warp / shockwaves): show explicit "Visit eb.mess.icu -> leaving this demo" banner, then navigate.
  if(portalWarpState){
    const st=portalWarpState,p=st.portal;
    const t=Math.min(1,(now-st.startedAt)/st.durationMs);
    drawWarpBanner(p.spec.warpTitle,p.spec.warpSub,p.spec.accentCss,t);
    p.lift=1;
    p.liftGroup.visible=true;
    p.liftGroup.position.z=0.028;
    p.pillars.visible=true;p.pillars.scale.set(1,1,0.028);p.pillarMat.opacity=0.65;
    p.holdRing.visible=false;
    if(t>=1&&!st.navigated){
      st.navigated=true;
      const targetUrl=p.spec.url;
      const curSession=session;
      if(curSession){
        curSession.end().catch(()=>{}).finally(()=>{window.location.assign(targetUrl);});
      }else{
        window.location.assign(targetUrl);
      }
    }
    return;
  }

  if(leaveConfirmState){
    if(showcaseOpen)showcaseGroup.visible=false;
    showcaseHoverZone=null;showcasePokeZone=null;showcasePokeSince=0;
    hoveredPortal=null;portalHoldId=null;portalHoldSinceMs=0;
    return;
  }

  const invWorld=frameInputs.invWorld;
  const probes=frameInputs.getProbesL();

  // Update floating mess.engineering showcase panel (upright THREE.Sprite; [×] + both links live inside the sprite canvas).
  showcaseGroup.visible=showcaseOpen;
  if(showcaseOpen){
    const spx=0.11,spy=0.07,spz=0.25;
    showcaseSprite.position.set(spx,spy,spz);

    // Derive viewer-aligned right/up/normal axes in L-frame directly from cached viewer basis so direct touch & raycast match the sprite 1:1.
    let rL=[1,0,0],uL=[0,0,1],nL=[0,-1,0];
    if(pose?.transform?.matrix){
      const vm=pose.transform.matrix;
      rL=frameInputs.viewerRightL;
      uL=frameInputs.viewerUpL;
      nL=worldVecToL(invWorld,vm[8],vm[9],vm[10]);
      const rl=Math.hypot(rL[0],rL[1],rL[2])||1,ul=Math.hypot(uL[0],uL[1],uL[2])||1,nl=Math.hypot(nL[0],nL[1],nL[2])||1;
      rL=[rL[0]/rl,rL[1]/rl,rL[2]/rl];
      uL=[uL[0]/ul,uL[1]/ul,uL[2]/ul];
      nL=[nL[0]/nl,nL[1]/nl,nL[2]/nl];
    }

    let nextHoverZone=null,directPoke=false;
    // 1. Direct fingertip / controller touch projected onto the upright sprite plane
    for(const pr of probes){
      const dx=pr.pos[0]-spx,dy=pr.pos[1]-spy,dz=pr.pos[2]-spz;
      const lx=dx*rL[0]+dy*rL[1]+dz*rL[2];
      const ly=dx*uL[0]+dy*uL[1]+dz*uL[2];
      const lz=dx*nL[0]+dy*nL[1]+dz*nL[2];
      const u=lx/SHOWCASE_W_M+0.5;
      const v=ly/SHOWCASE_H_M+0.5;
      const zDist=Math.abs(lz);
      if(u>=0&&u<=1&&v>=0&&v<=1&&zDist<0.060){
        recordProbeProximity(pr.probeId,zDist);
        const z=classifyShowcaseUv(u,v);
        if(z){
          nextHoverZone=z;
          if(zDist<0.030)directPoke=true;
          break;
        }
      }
    }
    // 2. Far ray-aim from hand/controller targetRaySpace intersected against the sprite plane in world space
    if(!nextHoverZone&&session&&pose?.transform?.matrix){
      const vm=pose.transform.matrix;
      const spWorld=lPointToWorld(world,spx,spy,spz);
      _scPlaneN.set(vm[8],vm[9],vm[10]).normalize();
      _scPlaneR.set(vm[0],vm[1],vm[2]).normalize();
      _scPlaneU.set(vm[4],vm[5],vm[6]).normalize();
      _scPlaneC.set(spWorld[0],spWorld[1],spWorld[2]);
      for(const inp of frameInputs.inputs){
        const rp=inp.getTargetRayPose();
        if(!rp)continue;
        const rm=rp.transform.matrix;
        _hudRayOrigin.set(rm[12],rm[13],rm[14]);
        _hudRayDir.set(-rm[8],-rm[9],-rm[10]).normalize();
        const denom=_hudRayDir.dot(_scPlaneN);
        if(Math.abs(denom)<1e-4)continue;
        const tHit=_scLocalP.copy(_scPlaneC).sub(_hudRayOrigin).dot(_scPlaneN)/denom;
        if(tHit<=0.05||tHit>3.0)continue;
        const hitPt=_scViewerL.copy(_hudRayOrigin).addScaledVector(_hudRayDir,tHit);
        _scRel.copy(hitPt).sub(_scPlaneC);
        const u=_scRel.dot(_scPlaneR)/SHOWCASE_W_M+0.5;
        const v=_scRel.dot(_scPlaneU)/SHOWCASE_H_M+0.5;
        if(u>=0&&u<=1&&v>=0&&v<=1){
          const z=classifyShowcaseUv(u,v);
          if(z){
            nextHoverZone=z;
            hudRayDot.visible=true;
            hudRayDot.position.copy(hitPt);
            break;
          }
        }
      }
    }
    if(nextHoverZone!==showcaseHoverZone){
      showcaseHoverZone=nextHoverZone;
      drawShowcasePanel();
    }
    if(directPoke&&showcaseHoverZone){
      if(showcasePokeZone!==showcaseHoverZone){showcasePokeZone=showcaseHoverZone;showcasePokeSince=now;}
      else if(now-showcasePokeSince>=180){
        const z=showcaseHoverZone;
        showcasePokeZone=null;showcasePokeSince=0;
        triggerShowcaseAction(z,now);
        return;
      }
    }else{
      showcasePokeZone=null;showcasePokeSince=0;
    }
  }
  let best=null,bestDist=Infinity,bestProbe=null;
  // Never steal interaction while inspecting a waypoint, closing a pin, pressing TL offer, or pressing mode/dev controls.
  if(!hoveredWaypoint&&!hoveredClosePinKey&&!hoveredTlOffer&&!hoveredModePill&&!hoveredDevDock){
    for(const p of spatialPortals){
      if(!p.spec.panel&&extraScanActive)continue;
      const s=p.spec;
      for(const pr of probes){
        const dx=Math.max(0,Math.abs(pr.pos[0]-s.x)-s.w/2);
        const dy=Math.max(0,Math.abs(pr.pos[1]-s.y)-s.h/2);
        const dz=Math.max(0,pr.pos[2]-0.036,-pr.pos[2]-0.016);
        const d=Math.hypot(dx,dy,dz);
        recordProbeProximity(pr.probeId,d);
        if(d<bestDist){bestDist=d;best=p;bestProbe=pr;}
      }
    }
  }

  const threshold=hoveredPortal?0.042:0.028;
  hoveredPortal=(best&&bestDist<=threshold)?best:null;

  // Intentional hold (< 1.6 cm from hotspot footprint) triggers portal cleanly without needing pinch (same UX as QR code).
  let holdProg=0;
  if(hoveredPortal&&bestDist<0.016){
    const holdDur=hoveredPortal.spec.panel?550:950;
    if(portalHoldId!==hoveredPortal.spec.id){portalHoldId=hoveredPortal.spec.id;portalHoldSinceMs=now;}
    holdProg=Math.min(1,(now-portalHoldSinceMs)/holdDur);
    if(holdProg>=1){
      portalHoldId=null;portalHoldSinceMs=0;
      triggerSpatialPortal(hoveredPortal,now);
      return;
    }
  }else{
    portalHoldId=null;portalHoldSinceMs=0;
  }

  const pulse=0.5+0.5*Math.sin(now*0.0032);
  for(const p of spatialPortals){
    const active=p===hoveredPortal;
    const targetLift=active?1:0;
    p.lift+=(targetLift-p.lift)*0.22;
    if(p.lift<0.01&&!active)p.lift=0;

    if(p.spec.sparkle){
      p.baseMat.opacity=active?0.95:(0.42+0.44*pulse);
      if(p.glowMat)p.glowMat.opacity=active?0.34:(0.08+0.14*pulse);
      const fade=Math.max(0,1-p.lift*3);
      for(const sp of p.sparkles||[]){
        sp.spr.visible=fade>0.03;
        if(sp.spr.visible){
          const tw=Math.pow(0.5+0.5*Math.sin(now*0.0055+sp.phase),2.4);
          sp.mat.opacity=fade*(0.15+0.85*tw);
          const sc=0.006+0.007*tw;
          sp.spr.scale.set(sc,sc,1);
          sp.spr.position.set(
            sp.ax+0.0018*Math.cos(now*0.0022+sp.phase),
            sp.ay+0.0014*Math.sin(now*0.0026+sp.phase),
            0.005+0.004*Math.sin(now*0.0032+sp.phase),
          );
        }
      }
      if(p.idleGlyph){
        p.idleGlyph.spr.visible=p.lift<0.25;
        p.idleGlyph.mat.opacity=(1-p.lift*4)*(0.58+0.36*pulse);
        p.idleGlyph.spr.position.z=0.008+0.003*pulse;
      }
    }else if(p.spec.idleHint){
      p.baseMat.opacity=active?0.92:(0.26+0.24*pulse);
      if(p.idleGlyph){
        p.idleGlyph.spr.visible=p.lift<0.25;
        p.idleGlyph.mat.opacity=(1-p.lift*4)*(0.55+0.35*pulse);
      }
    }else{
      p.baseMat.opacity=p.lift*0.85;
    }

    const raisedZ=p.lift*(p.spec.warp?0.030:0.026);
    const showLift=p.lift>0.02;
    p.liftGroup.visible=showLift;
    p.pillars.visible=showLift;
    if(showLift){
      p.liftGroup.position.z=raisedZ;
      p.frameMat.opacity=0.25+0.70*p.lift;
      p.pillars.scale.set(1,1,Math.max(0.001,raisedZ));
      p.pillarMat.opacity=0.65*p.lift;
      p.cardSprite.material.opacity=Math.min(1,p.lift*1.35);
      p.cardSprite.position.z=0.038+p.lift*0.008;
      if(p.ringMesh){
        p.ringMesh.rotation.z=now*0.0028;
        p.ringMesh.scale.setScalar(0.88+0.12*Math.sin(now*0.006));
        p.ringMat.opacity=0.82*p.lift;
      }
      const pHold=active?holdProg:0;
      p.holdRing.visible=pHold>0.02;
      if(pHold>0.02){
        p.holdRing.scale.setScalar(0.55+pHold*0.75);
        p.holdRingMat.opacity=0.5+0.5*pHold;
      }
      const kind=active&&bestProbe?bestProbe.kind:'hand';
      const bucket=Math.round(pHold*20);
      const ck=`${kind}:${bucket}`;
      if(ck!==p.cardKey){p.cardKey=ck;drawPortalCard(p,kind,pHold);}
    }
  }
}

function updateWaypointInspector(frameInputs,world,pose,now){
  if(!world||!session||!space||(movableTools.size===0&&placedExtras.size===0&&perimeterTagVisuals.size===0&&pinnedPoints.size===0)){hideWaypointInspector();renderPinFlash(now);return;}
  const dt=lastInspectorTimeMs!=null?Math.min(0.05,Math.max(0.004,(now-lastInspectorTimeMs)/1000)):1/90;
  lastInspectorTimeMs=now;
  const viewerPosL=frameInputs.viewerPosL,rightL=frameInputs.viewerRightL,upL=frameInputs.viewerUpL;
  lastViewerBasis={viewerPosL,viewerRightL:rightL,viewerUpL:upL};

  const probes=extraScanActive?[]:frameInputs.getProbesL();

  // Keep pinned pills 100% world-locked in L-frame (no per-frame view-dependent jostling on head motion).
  // Only animate the initial 300 ms pop-in scale and keep the [×] close orb aligned to the billboarded sprite's right edge.
  if(pinnedPoints.size>0){
    const rLen=Math.hypot(rightL[0],rightL[1],rightL[2])||1;
    const sRx=rightL[0]/rLen,sRy=rightL[1]/rLen,sRz=rightL[2]/rLen;
    for(const pin of pinnedPoints.values()){
      if(!pin.settled){
        const age=now-pin.pinnedAtMs;
        if(age<300){
          const t=Math.max(0,age)/300;
          const s=t<0.65?(0.74+(t/0.65)*0.34):(1.08-((t-0.65)/0.35)*0.08);
          badgeMesh.setQuad(pin.pillSlot,pin.pillPosL,PIN_PILL_W_M*s,PIN_PILL_H_M*s,true);
        }else{
          badgeMesh.setQuad(pin.pillSlot,pin.pillPosL,PIN_PILL_W_M,PIN_PILL_H_M,true);
          pin.settled=true;
        }
      }
      // Keep the [×] close orb rock-steady while the user is reaching for it; otherwise align to the sprite's right edge.
      let freezeForClose=pin.key===hoveredClosePinKey;
      if(!freezeForClose&&probes.length){
        const [rx,ry,rz]=pin.closePosL;
        for(const pr of probes){
          if(Math.hypot(rx-pr.pos[0],ry-pr.pos[1],rz-pr.pos[2])<0.075){freezeForClose=true;break;}
        }
      }
      if(!freezeForClose){
        const px=pin.pillPosL[0],py=pin.pillPosL[1],pz=pin.pillPosL[2];
        const rx=px+sRx*(PIN_PILL_W_M*0.40);
        const ry=py+sRy*(PIN_PILL_W_M*0.40);
        const rz=pz+sRz*(PIN_PILL_W_M*0.40);
        pin.closePosL=[rx,ry,rz];
        closeOrbBatch.setPoint(pin.closeOrbSlot,rx,ry,rz,pin.closeHot?1.35:1.0);
      }
    }
  }

  // Pause hover/selection/pinning while SCAN is active so moving physical tags never pops up cards or accidental pins.
  if(extraScanActive){
    for(const pin of pinnedPoints.values()){
      if(pin.closeHot){
        pin.closeHot=false;
        closeOrbBatch.setPoint(pin.closeOrbSlot,pin.closePosL[0],pin.closePosL[1],pin.closePosL[2],1.0);
        closeOrbBatch.setColor(pin.closeOrbSlot,0xdc2626);
        drawPinPillAtlas(pin.pillSlot,pin.wp,{closeHot:false});
      }
    }
    hideWaypointInspector();
    renderPinFlash(now);
    return;
  }

  let nearCloseKey=null,nearCloseDist=Infinity,nearCloseProbe=null;
  for(const pin of pinnedPoints.values()){
    const [rx,ry,rz]=pin.closePosL;
    for(const pr of probes){
      const d=Math.hypot(rx-pr.pos[0],ry-pr.pos[1],rz-pr.pos[2]);
      recordProbeProximity(pr.probeId,d);
      if(d<nearCloseDist){nearCloseDist=d;nearCloseKey=pin.key;nearCloseProbe=pr;}
    }
  }

  // Always evaluate point/tag candidates FIRST so pinned [×] close orbs never hijack closer targets.
  const enterDist=0.048,exitDist=0.060;
  const threshold=pinHoldSince!=null?0.11:(inspectVisible?exitDist:enterDist);
  let best=null,bestDist=Infinity,bestCenterDist=Infinity,bestProbe=null,bestPassMeta=null;
  let bestDistForCloseProbe=Infinity;
  const closeProbeId=nearCloseProbe?.probeId??null;
  const candidates=[];
  const pinnedAnchorRefs=[];
  const collectCandidate=(wp,pr,maxDist)=>{
    const dx=pr.pos[0]-wp.centerL[0],dy=pr.pos[1]-wp.centerL[1],dz=pr.pos[2]-wp.centerL[2];
    const cutoff=0.16+wp.edgeMm*0.00072;
    if(dx*dx+dy*dy+dz*dz>cutoff*cutoff)return;
    const m=tagSurfaceMetrics(wp.matrix,wp.edgeMm,pr.pos);
    recordProbeProximity(pr.probeId,m.surfaceDist);
    if(closeProbeId&&pr.probeId===closeProbeId&&m.surfaceDist<bestDistForCloseProbe)bestDistForCloseProbe=m.surfaceDist;
    if(m.surfaceDist>=maxDist||m.surfaceDist>threshold)return;
    const pm=wp.isTrailPoint?getWaypointPassMeta(wp):null;
    candidates.push({
      wp,
      probe:pr,
      surfaceDist:m.surfaceDist,
      centerDist:m.centerDist,
      strandKey:pm?pm.strandKey:null,
      lineIdx:pm?pm.lineIdx:null,
      lineSeq:pm?pm.lineSeq:null,
      passIndex:pm?pm.passIndex:null,
      passMeta:pm,
    });
  };

  if(pinHoldWp&&pinHoldSince!=null){
    for(const pr of probes){
      if(!isPinchHandEligible({kind:pr.kind,handedness:pr.handedness,pinHoldHand,leftPalmFacingViewer,leftMenuOpen:leftMenuOpenNow}))continue;
      collectCandidate(pinHoldWp,pr,0.11);
    }
  }
  if(candidates.length===0){
    for(const entry of movableTools.values()){
      if(entry.currentPose){
        for(const pr of probes)collectCandidate(entry.currentPose,pr,Infinity);
      }
      for(const wp of entry.waypoints){
        for(const pr of probes)collectCandidate(wp,pr,Infinity);
      }
    }
    for(const extra of placedExtras.values()){
      if(!extra.wp)continue;
      for(const pr of probes)collectCandidate(extra.wp,pr,Infinity);
    }
    for(const vis of perimeterTagVisuals.values()){
      if(!vis.visible||!vis.wp)continue;
      if(perimeterFence?.isOffered()&&!perimeterOfferSuppressed&&vis.id===perimeterFence.triggerTagId)continue;
      for(const pr of probes)collectCandidate(vis.wp,pr,Infinity);
    }
  }
  if(candidates.length>0){
    if(pinnedPoints.size>0){
      for(const pin of pinnedPoints.values()){
        const pm=getWaypointPassMeta(pin.wp);
        if(pm){
          pinnedAnchorRefs.push({
            strandKey:pm.strandKey,
            lineIdx:pm.lineIdx,
            lineSeq:pm.lineSeq,
            countReversalsBetween:(a,b)=>pm.analysis.countReversalsBetween(a,b),
          });
        }
      }
    }
    const prevHoveredMeta=hoveredWaypoint?getWaypointPassMeta(hoveredWaypoint):null;
    const picked=pickPreferredMeasureCandidate(candidates,{
      anchorTrailRefs:pinnedAnchorRefs,
      prevHoveredRef:prevHoveredMeta,
      maxReachM:threshold,
    });
    if(picked){
      best=picked.wp;
      bestDist=picked.surfaceDist;
      bestCenterDist=picked.centerDist;
      bestProbe=picked.probe;
      bestPassMeta=picked.passMeta;
    }
  }

  hoveredClosePinKey=(nearCloseDist<0.028&&nearCloseDist<bestDistForCloseProbe)?nearCloseKey:null;
  for(const pin of pinnedPoints.values()){
    const hot=pin.key===hoveredClosePinKey;
    if(hot!==pin.closeHot){
      pin.closeHot=hot;
      closeOrbBatch.setPoint(pin.closeOrbSlot,pin.closePosL[0],pin.closePosL[1],pin.closePosL[2],hot?1.35:1.0);
      closeOrbBatch.setColor(pin.closeOrbSlot,hot?0xfef08a:0xdc2626);
      drawPinPillAtlas(pin.pillSlot,pin.wp,{closeHot:hot});
    }
  }
  // Direct fingertip/controller poke (< 2.5 cm for 180 ms) on the red [×] orb closes the pin effortlessly.
  if(hoveredClosePinKey&&nearCloseDist<0.025){
    const pin=pinnedPoints.get(hoveredClosePinKey);
    if(pin&&now-pin.pinnedAtMs>400){
      if(closePokeKey!==hoveredClosePinKey){closePokeKey=hoveredClosePinKey;closePokeSinceMs=now;}
      else if(now-closePokeSinceMs>=180){unpinWaypoint(hoveredClosePinKey);renderPinFlash(now);return;}
    }
  }else{
    closePokeKey=null;closePokeSinceMs=0;
  }

  const clearHover=()=>{
    inspectRoot.visible=false;floorInspectGroup.visible=false;
    if(previewKey!==null){releaseRailSlots(previewRailSlots);previewKey=null;}
    clearHover2();
    hoveredWaypoint=null;hoveredWaypointKey=null;hoveredFloorHeightMm=null;inspectVisible=false;
    hoverCardPosL=null;hoverCardForWpKey=null;
    inspectMat.opacity=1;inspectLeaderMat.opacity=0.72;
    resetPinHold();
    syncStaticTagBadges();
    renderPinFlash(now);
  };

  if(!probes.length){
    clearHover();
    return;
  }

  const hasTagHover=Boolean(best&&bestDist<=threshold);
  const closeBlocksActiveHover=Boolean(hoveredClosePinKey&&(!hasTagHover||bestProbe?.probeId===nearCloseProbe?.probeId));

  if(!best||bestDist>threshold||closeBlocksActiveHover){
    clearHover();
    return;
  }
  inspectVisible=true;inspectRoot.visible=true;hoveredWaypoint=best;
  syncStaticTagBadges();

  // Query real Quest floor Y coordinate in current 'local' reference space via secondary 'local-floor' space.
  const floorPose=floorSpace?frameInputs.frame.getPose?.(floorSpace,space):null;
  const floorY=floorPose?floorPose.transform.position.y:null;
  const [cx,cy,cz]=best.centerL;
  const wpWorld=lPointToWorld(world,cx,cy,cz);
  const planeWorld=lPointToWorld(world,cx,cy,0);
  const floorHeightMm=floorY!=null?(wpWorld[1]-floorY)*1000:null;
  hoveredFloorHeightMm=floorHeightMm;

  const wpKey=waypointIdKey(best);
  let alreadyPinned=pinnedPoints.has(wpKey);

  if(floorY!=null){
    floorInspectGroup.visible=true;
    // Thick orange 3D rod extending straight down along gravity from table footprint to physical Quest floor.
    setRodSegment(floorDropRod,planeWorld[0],planeWorld[1],planeWorld[2],planeWorld[0],floorY,planeWorld[2],0.0018);
    floorDropRing.position.set(planeWorld[0],floorY+0.002,planeWorld[2]);
  }else{
    floorInspectGroup.visible=false;
  }

  // Bare-hand Left or Right Thumb+Index pinch (350 ms hold) to pin/unpin the hovered tag or waypoint.
  // Left-hand pinch is suppressed only when looking directly at the left palm (Meta Quest OS menu / wrist menu).
  let loaderShown=false;
  const pinchEligible=isPinchHandEligible({
    kind:bestProbe.kind,
    handedness:bestProbe.handedness,
    pinHoldHand,
    leftPalmFacingViewer,
    leftMenuOpen:leftMenuOpenNow,
  });
  if(pinchEligible&&bestProbe.input?.hand){
    const jt=bestProbe.input.getJoint('thumb-tip');
    const ji=bestProbe.input.getJoint('index-finger-tip');
    const pDist=(jt&&ji)?Math.hypot(
      jt.transform.position.x-ji.transform.position.x,
      jt.transform.position.y-ji.transform.position.y,
      jt.transform.position.z-ji.transform.position.z,
    ):Infinity;
    const PIN_HOLD_MS=350;
    const pinchThresh=pinHoldSince!=null?0.032:0.024;
    if(pDist>0.034){
      pinHoldConsumed=false;
      pinHoldSince=null;pinHoldWp=null;pinHoldKey=null;pinHoldHand=null;
    }else if(pDist<pinchThresh&&!pinHoldConsumed){
      if(pinHoldKey!==wpKey||pinHoldHand!==bestProbe.handedness){
        pinHoldKey=wpKey;pinHoldWp=best;pinHoldHand=bestProbe.handedness;pinHoldFloorMm=floorHeightMm;pinHoldSince=now;
      }
      const prog=Math.min(1,(now-pinHoldSince)/PIN_HOLD_MS);
      const mx=(jt.transform.position.x+ji.transform.position.x)/2;
      const my=(jt.transform.position.y+ji.transform.position.y)/2;
      const mz=(jt.transform.position.z+ji.transform.position.z)/2;
      pinLoadSprite.visible=true;
      pinLoadSprite.position.set(mx,my+0.022,mz);
      const sc=0.032+prog*0.010;
      pinLoadSprite.scale.set(sc,sc,1);
      pinLoadMat.opacity=1;
      drawPinLoader(prog,alreadyPinned?'#ef4444':'#e879f9',alreadyPinned?'×':'+',false);
      loaderShown=true;
      if(prog>=1){
        pinHoldConsumed=true;
        pinHoldSince=null;pinHoldWp=null;pinHoldKey=null;pinHoldHand=null;
        togglePinWaypoint(best,floorHeightMm,[mx,my+0.022,mz]);
        alreadyPinned=pinnedPoints.has(wpKey);
        hoveredWaypointKey=null;
        loaderShown=renderPinFlash(now);
      }
    }else if(pinHoldSince!=null){
      pinHoldSince=null;pinHoldWp=null;pinHoldKey=null;pinHoldHand=null;
    }
  }else if(!loaderShown){
    resetPinHold();
  }

  if(!loaderShown){
    renderPinFlash(now);
  }

  // Show live translucent preview distance rail(s) from pinned points to a newly hovered unpinned waypoint.
  const nextPreviewKey=(!alreadyPinned&&pinnedPoints.size>0)?`${wpKey}:${[...pinnedPoints.keys()].join(',')}`:null;
  if(nextPreviewKey!==previewKey){
    previewKey=nextPreviewKey;
    releaseRailSlots(previewRailSlots);
    if(nextPreviewKey){
      for(const pin of pinnedPoints.values())addRailSegment(previewRailSlots,pin.wp,best,true,false);
    }
  }

  // Position the compact hover card laterally away from the probing hand & viewer line of sight, connected by a thin leader line.
  const avoidPillPos=alreadyPinned?pinnedPoints.get(wpKey)?.pillPosL||null:null;
  const cardLayout=computeHoverCardLayout({
    tagCenterL:best.centerL,
    edgeMm:best.edgeMm,
    viewerPosL,
    viewerRightL:rightL,
    viewerUpL:upL,
    handedness:bestProbe.handedness,
    probePosL:bestProbe.pos,
    avoidPosL:avoidPillPos,
    cardWidthM:HOVER_CARD_W_M,
    cardHeightM:HOVER_CARD_H_M,
  });
  if(!hoverCardPosL||hoverCardForWpKey!==wpKey){
    hoverCardPosL=cardLayout.cardPos.slice();
    hoverCardForWpKey=wpKey;
  }else{
    const cardAlpha=1-Math.exp(-dt*14);
    hoverCardPosL[0]+=(cardLayout.cardPos[0]-hoverCardPosL[0])*cardAlpha;
    hoverCardPosL[1]+=(cardLayout.cardPos[1]-hoverCardPosL[1])*cardAlpha;
    hoverCardPosL[2]+=(cardLayout.cardPos[2]-hoverCardPosL[2])*cardAlpha;
  }
  inspectSprite.visible=true;
  inspectSprite.position.set(hoverCardPosL[0],hoverCardPosL[1],hoverCardPosL[2]);

  const lAx=hoverCardPosL[0]-cardLayout.screenRight[0]*(cardLayout.sideSign*HOVER_CARD_W_M*0.44)-cardLayout.screenUp[0]*(HOVER_CARD_H_M*0.44);
  const lAy=hoverCardPosL[1]-cardLayout.screenRight[1]*(cardLayout.sideSign*HOVER_CARD_W_M*0.44)-cardLayout.screenUp[1]*(HOVER_CARD_H_M*0.44);
  const lAz=hoverCardPosL[2]-cardLayout.screenRight[2]*(cardLayout.sideSign*HOVER_CARD_W_M*0.44)-cardLayout.screenUp[2]*(HOVER_CARD_H_M*0.44);
  setRodSegment(inspectLeaderRod,cx,cy,cz,lAx,lAy,lAz,0.00075);
  inspectLeaderMat.color.setHex(alreadyPinned?0xe879f9:0xfbbf24);

  // Gently dim the hover card while actively pinching so fingers and the pinch ring stay 100% unobstructed.
  const pinching=pinHoldSince!=null;
  const fadeAlpha=1-Math.exp(-dt*16);
  inspectMat.opacity+=((pinching?0.28:1.0)-inspectMat.opacity)*fadeAlpha;
  inspectLeaderMat.opacity+=((pinching?0.24:0.72)-inspectLeaderMat.opacity)*fadeAlpha;

  const floorBucket=floorHeightMm!=null?Math.round(floorHeightMm/2):'na';
  const passBucket=bestPassMeta?`${bestPassMeta.passIndex}/${bestPassMeta.passCount}`:'na';
  const key=`${wpKey}:${bestProbe.kind}:${floorBucket}:${alreadyPinned?1:0}:${passBucket}`;
  if(key!==hoveredWaypointKey){
    hoveredWaypointKey=key;
    highlightFrame.geometry=getThickTagFrameGeo(best.edgeMm);
    _tmpMat4.fromArray(best.matrix).scale(_tmpScale);
    highlightFrame.matrix.copy(_tmpMat4);
    highlightFrame.matrixWorldNeedsUpdate=true;
    _tmpPlaneScale.set((best.edgeMm/1000)*1.12,(best.edgeMm/1000)*1.12,1);
    _tmpMat4.fromArray(best.matrix).scale(_tmpPlaneScale);
    highlightPlane.matrix.copy(_tmpMat4);
    highlightPlane.matrixWorldNeedsUpdate=true;
    setTagOriginDot(highlightOriginDot,best.matrix,best.edgeMm);
    // Thick 3D rods all meeting at the table footprint (cx, cy, 0):
    setRodSegment(dropRod,cx,cy,cz,cx,cy,0,0.0018);
    dropRing.position.set(cx,cy,0.0018);
    // Red X directly at y = y_measured: (cx, cy) -> (0, cy) on Y-arm.
    setRodSegment(xAxisRod,cx,cy,0.0018,0,cy,0.0018,0.0018);
    xHitRing.position.set(0,cy,0.0018);
    // Green Y straight down to X-arm: (cx, cy) -> (cx, 0).
    setRodSegment(yAxisRod,cx,cy,0.0018,cx,0,0.0018,0.0018);
    yHitRing.position.set(cx,0,0.0018);
    drawWaypointCard(inspectCanvas,inspectTex,best,bestProbe.kind,floorHeightMm,{alreadyPinned,passInfo:bestPassMeta});
  }

  // --- Dual-Hand Hover: if the other hand/controller simultaneously hovers a second tag, show its own Floating Infobar + live Δmm distance rail ---
  const otherProbes=probes.filter(pr=>pr.probeId!==bestProbe.probeId);
  const thresh2=inspect2Visible?exitDist:enterDist;
  let best2=null,best2Dist=Infinity,best2CenterDist=Infinity,best2Probe=null,best2PassMeta=null;
  if(otherProbes.length){
    const candidates2=[];
    const collectCandidate2=(wp,pr)=>{
      if(wp===best)return;
      const dx=pr.pos[0]-wp.centerL[0],dy=pr.pos[1]-wp.centerL[1],dz=pr.pos[2]-wp.centerL[2];
      const cutoff=0.16+wp.edgeMm*0.00072;
      if(dx*dx+dy*dy+dz*dz>cutoff*cutoff)return;
      const m=tagSurfaceMetrics(wp.matrix,wp.edgeMm,pr.pos);
      recordProbeProximity(pr.probeId,m.surfaceDist);
      if(m.surfaceDist>thresh2)return;
      const pm=wp.isTrailPoint?getWaypointPassMeta(wp):null;
      candidates2.push({
        wp,
        probe:pr,
        surfaceDist:m.surfaceDist,
        centerDist:m.centerDist,
        strandKey:pm?pm.strandKey:null,
        lineIdx:pm?pm.lineIdx:null,
        lineSeq:pm?pm.lineSeq:null,
        passIndex:pm?pm.passIndex:null,
        passMeta:pm,
      });
    };
    for(const entry of movableTools.values()){
      if(entry.currentPose){
        for(const pr of otherProbes)collectCandidate2(entry.currentPose,pr);
      }
      for(const wp of entry.waypoints){
        for(const pr of otherProbes)collectCandidate2(wp,pr);
      }
    }
    for(const extra of placedExtras.values()){
      if(!extra.wp)continue;
      for(const pr of otherProbes)collectCandidate2(extra.wp,pr);
    }
    for(const vis of perimeterTagVisuals.values()){
      if(!vis.visible||!vis.wp)continue;
      if(perimeterFence?.isOffered()&&!perimeterOfferSuppressed&&vis.id===perimeterFence.triggerTagId)continue;
      for(const pr of otherProbes)collectCandidate2(vis.wp,pr);
    }
    if(candidates2.length>0){
      const dualAnchors=pinnedAnchorRefs.length>0
        ?pinnedAnchorRefs
        :(bestPassMeta?[{
          strandKey:bestPassMeta.strandKey,
          lineIdx:bestPassMeta.lineIdx,
          lineSeq:bestPassMeta.lineSeq,
          countReversalsBetween:(a,b)=>bestPassMeta.analysis.countReversalsBetween(a,b),
        }]:[]);
      const prevHovered2Meta=hoveredWaypoint2?getWaypointPassMeta(hoveredWaypoint2):null;
      const picked2=pickPreferredMeasureCandidate(candidates2,{
        anchorTrailRefs:dualAnchors,
        prevHoveredRef:prevHovered2Meta,
        maxReachM:thresh2,
      });
      if(picked2){
        best2=picked2.wp;
        best2Dist=picked2.surfaceDist;
        best2CenterDist=picked2.centerDist;
        best2Probe=picked2.probe;
        best2PassMeta=picked2.passMeta;
      }
    }
  }
  if(!best2||best2Dist>thresh2){
    if(inspect2Visible){clearHover2();syncStaticTagBadges();}
  }else{
    inspect2Visible=true;
    inspect2Root.visible=true;
    hoveredWaypoint2=best2;
    syncStaticTagBadges();
    const wp2Key=waypointIdKey(best2);
    const alreadyPinned2=pinnedPoints.has(wp2Key);
    const [cx2,cy2,cz2]=best2.centerL;
    const wp2World=lPointToWorld(world,cx2,cy2,cz2);
    const floorHeight2Mm=floorY!=null?(wp2World[1]-floorY)*1000:null;
    const card2Layout=computeHoverCardLayout({
      tagCenterL:best2.centerL,
      edgeMm:best2.edgeMm,
      viewerPosL,
      viewerRightL:rightL,
      viewerUpL:upL,
      handedness:best2Probe.handedness,
      probePosL:best2Probe.pos,
      avoidPosL:hoverCardPosL,
      cardWidthM:HOVER_CARD_W_M,
      cardHeightM:HOVER_CARD_H_M,
    });
    if(!hoverCard2PosL||hoverCard2ForWpKey!==wp2Key){
      hoverCard2PosL=card2Layout.cardPos.slice();
      hoverCard2ForWpKey=wp2Key;
    }else{
      const cardAlpha2=1-Math.exp(-dt*14);
      hoverCard2PosL[0]+=(card2Layout.cardPos[0]-hoverCard2PosL[0])*cardAlpha2;
      hoverCard2PosL[1]+=(card2Layout.cardPos[1]-hoverCard2PosL[1])*cardAlpha2;
      hoverCard2PosL[2]+=(card2Layout.cardPos[2]-hoverCard2PosL[2])*cardAlpha2;
    }
    inspect2Sprite.position.set(hoverCard2PosL[0],hoverCard2PosL[1],hoverCard2PosL[2]);
    const l2Ax=hoverCard2PosL[0]-card2Layout.screenRight[0]*(card2Layout.sideSign*HOVER_CARD_W_M*0.44)-card2Layout.screenUp[0]*(HOVER_CARD_H_M*0.44);
    const l2Ay=hoverCard2PosL[1]-card2Layout.screenRight[1]*(card2Layout.sideSign*HOVER_CARD_W_M*0.44)-card2Layout.screenUp[1]*(HOVER_CARD_H_M*0.44);
    const l2Az=hoverCard2PosL[2]-card2Layout.screenRight[2]*(card2Layout.sideSign*HOVER_CARD_W_M*0.44)-card2Layout.screenUp[2]*(HOVER_CARD_H_M*0.44);
    setRodSegment(inspect2LeaderRod,cx2,cy2,cz2,l2Ax,l2Ay,l2Az,0.00075);

    const floor2Bucket=floorHeight2Mm!=null?Math.round(floorHeight2Mm/2):'na';
    const pass2Bucket=best2PassMeta?`${best2PassMeta.passIndex}/${best2PassMeta.passCount}`:'na';
    const key2=`${wp2Key}:${best2Probe.kind}:${floor2Bucket}:${alreadyPinned2?1:0}:${pass2Bucket}`;
    if(key2!==hoveredWaypoint2Key){
      hoveredWaypoint2Key=key2;
      highlightFrame2.geometry=getThickTagFrameGeo(best2.edgeMm);
      _tmpMat4.fromArray(best2.matrix).scale(_tmpScale);
      highlightFrame2.matrix.copy(_tmpMat4);
      highlightFrame2.matrixWorldNeedsUpdate=true;
      _tmpPlaneScale.set((best2.edgeMm/1000)*1.12,(best2.edgeMm/1000)*1.12,1);
      _tmpMat4.fromArray(best2.matrix).scale(_tmpPlaneScale);
      highlightPlane2.matrix.copy(_tmpMat4);
      highlightPlane2.matrixWorldNeedsUpdate=true;
      setTagOriginDot(highlightOriginDot2,best2.matrix,best2.edgeMm);
      setRodSegment(dropRod2,cx2,cy2,cz2,cx2,cy2,0,0.0018);
      dropRing2.position.set(cx2,cy2,0.0018);
      setRodSegment(xAxisRod2,cx2,cy2,0.0018,0,cy2,0.0018,0.0018);
      setRodSegment(yAxisRod2,cx2,cy2,0.0018,cx2,0,0.0018,0.0018);
      drawWaypointCard(inspect2Canvas,inspect2Tex,best2,best2Probe.kind,floorHeight2Mm,{alreadyPinned:alreadyPinned2,passInfo:best2PassMeta});
    }

    // Live 3D distance rail (Δmm / ΔZ) directly between the two simultaneously hovered tags.
    const nextDualKey=wpKey<wp2Key?`${wpKey}|${wp2Key}`:`${wp2Key}|${wpKey}`;
    if(nextDualKey!==dualHandRailKey){
      dualHandRailKey=nextDualKey;
      releaseRailSlots(dualHandRailSlots);
      addRailSegment(dualHandRailSlots,best,best2,false,true);
    }
  }
}

const RIBBON_ACCENTS=[
  {hex:0x22d3ee,css:'#22d3ee'}, // cyan
  {hex:0xfb923c,css:'#fb923c'}, // coral orange
  {hex:0x34d399,css:'#34d399'}, // mint emerald
  {hex:0xf472b6,css:'#f472b6'}, // pink rose
  {hex:0x60a5fa,css:'#60a5fa'}, // sky blue
  {hex:0xa3e635,css:'#a3e635'}, // lime
  {hex:0xfbbf24,css:'#fbbf24'}, // amber gold
  {hex:0xc084fc,css:'#c084fc'}, // violet
];
function ribbonThemeFor(tagId){return RIBBON_ACCENTS[Math.abs(tagId)%RIBBON_ACCENTS.length];}

const MAX_LINE_PTS=1200;

function blendRigid(a,b,alpha){
  const t=Math.max(0,Math.min(1,alpha));
  const p=[
    a[12]+(b[12]-a[12])*t,
    a[13]+(b[13]-a[13])*t,
    a[14]+(b[14]-a[14])*t,
  ];
  const q=quatSlerp(matToQuat(a),matToQuat(b),t);
  return composeRigid(p,q);
}

const QUAT_ID=[0,0,0,1];
function quatMul(a,b){
  return [
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
    a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
    a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2],
  ];
}
const quatConj=q=>[-q[0],-q[1],-q[2],q[3]];
function resetDisplayErrorOffset(entry){
  entry.errOffsetPos[0]=0;entry.errOffsetPos[1]=0;entry.errOffsetPos[2]=0;
  entry.errOffsetRot[0]=0;entry.errOffsetRot[1]=0;entry.errOffsetRot[2]=0;entry.errOffsetRot[3]=1;
}
function captureDisplayErrorOffset(entry,nextPredictedMatrix,tauSec=0.055){
  if(!entry.displayMatrix||!nextPredictedMatrix){resetDisplayErrorOffset(entry);return;}
  let dx=entry.displayMatrix[12]-nextPredictedMatrix[12];
  let dy=entry.displayMatrix[13]-nextPredictedMatrix[13];
  let dz=entry.displayMatrix[14]-nextPredictedMatrix[14];
  const errM=Math.hypot(dx,dy,dz);
  if(!Number.isFinite(errM)){resetDisplayErrorOffset(entry);return;}
  if(errM>0.18){const s=0.18/errM;dx*=s;dy*=s;dz*=s;}
  entry.errOffsetPos[0]=dx;entry.errOffsetPos[1]=dy;entry.errOffsetPos[2]=dz;
  let qErr=quatNormalize(quatMul(matToQuat(entry.displayMatrix),quatConj(matToQuat(nextPredictedMatrix))));
  if(qErr[3]<0)qErr=[-qErr[0],-qErr[1],-qErr[2],-qErr[3]];
  entry.errOffsetRot[0]=qErr[0];entry.errOffsetRot[1]=qErr[1];entry.errOffsetRot[2]=qErr[2];entry.errOffsetRot[3]=qErr[3];
  entry.errDecayTauSec=tauSec;
}
function applyAndDecayErrorOffset(entry,basePose,dt){
  const tau=Math.max(0.02,entry.errDecayTauSec||0.055);
  const decay=Math.exp(-dt/tau);
  const ep=entry.errOffsetPos;
  ep[0]*=decay;ep[1]*=decay;ep[2]*=decay;
  if(Math.hypot(ep[0],ep[1],ep[2])<0.0002){ep[0]=0;ep[1]=0;ep[2]=0;}
  const er=entry.errOffsetRot;
  if(er[3]<0){er[0]=-er[0];er[1]=-er[1];er[2]=-er[2];er[3]=-er[3];}
  const decayedRot=er[3]>0.999999?QUAT_ID:quatSlerp(er,QUAT_ID,1-decay);
  er[0]=decayedRot[0];er[1]=decayedRot[1];er[2]=decayedRot[2];er[3]=decayedRot[3];
  return composeRigid([basePose[12]+ep[0],basePose[13]+ep[1],basePose[14]+ep[2]],quatMul(decayedRot,matToQuat(basePose)));
}

function drawLiveBadge(entry,mode,coupled=false){
  const out=Boolean(entry.outsidePerimeter);
  const measure=mode==='measure';
  const rec=mode==='rec';
  const armedIdle=!rec&&!measure&&recordAllActive&&!entry.recording;
  const fastTracked=!measure&&!coupled&&activeFocusTagIds.has(entry.tagId);
  const key=`${entry.tagId}:${mode}:${coupled?1:0}:${fastTracked?1:0}:${out?1:0}:${armedIdle?1:0}`;
  if(key===entry.faceKey||entry.faceSlot<0)return;
  entry.faceKey=key;
  if(out){
    drawTagMatrixFace(entry.faceSlot,`live:${key}`,`${entry.tagId}`,'#ff3b30',{fill:'rgba(138,12,16,0.95)',alarmBell:true});
    return;
  }
  const near=mode==='near',locked=mode==='locked';
  const stroke=rec?'#ef4444':(coupled?'#4ade80':(fastTracked?'#fbbf24':(near?'#22d3ee':(locked?'#4ade80':entry.theme.css))));
  const fill=rec?'rgba(45,8,14,0.92)':(coupled?'rgba(8,30,22,0.92)':(fastTracked?'rgba(34,24,6,0.92)':(near?'rgba(8,28,36,0.92)':'rgba(6,11,20,0.86)')));
  const statusSymbol=rec
    ?(coupled?'● ✋':(fastTracked?'● ⚡':'●'))
    :(coupled?(armedIdle?'○ ✋':'✋'):(near?'◉':(fastTracked?(armedIdle?'○ ⚡':'⚡'):(armedIdle?'○':''))));
  const statusColor=rec?'#fca5a5':(armedIdle?'#f87171':(coupled?'#86efac':(fastTracked?'#fde047':(near?'#67e8f9':entry.theme.css))));
  drawTagMatrixFace(entry.faceSlot,`live:${key}`,`${entry.tagId}`,stroke,{fill,statusSymbol,statusColor});
}

function drawRecOrbPill(entry,recording,hot){
  const key=`${recording?1:0}:${hot?1:0}`;
  if(key===entry.recPillKey||entry.recPillSlot<0)return;
  entry.recPillKey=key;
  badgeAtlas.draw(entry.recPillSlot,`rec:${key}`,(ctx,w,h)=>{
    ctx.scale(w/128,h/128);
    ctx.fillStyle=recording?'rgba(58,12,18,0.94)':'rgba(10,18,30,0.88)';
    ctx.beginPath();ctx.arc(64,64,56,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=hot?'#fef08a':(recording?'#ef4444':'rgba(248,113,113,0.72)');
    ctx.lineWidth=hot?7:(recording?5:4.5);ctx.stroke();
    if(recording){
      ctx.fillStyle=hot?'#ffffff':'#fca5a5';
      ctx.beginPath();ctx.roundRect(42,42,44,44,9);ctx.fill();
    }else{
      ctx.strokeStyle=hot?'rgba(255,255,255,0.85)':'rgba(248,113,113,0.48)';
      ctx.lineWidth=3.5;
      ctx.beginPath();ctx.arc(64,64,34,0,Math.PI*2);ctx.stroke();
      ctx.fillStyle=hot?'#ff5c5c':'#ef4444';
      ctx.beginPath();ctx.arc(64,64,23,0,Math.PI*2);ctx.fill();
    }
  });
}

function drawMarkOrbPill(entry,hot){
  const key=`${hot?1:0}`;
  if(key===entry.markPillKey||entry.markPillSlot<0)return;
  entry.markPillKey=key;
  badgeAtlas.draw(entry.markPillSlot,`mark:${key}`,(ctx,w,h)=>{
    ctx.scale(w/128,h/128);
    ctx.fillStyle='rgba(8,22,34,0.88)';
    ctx.beginPath();ctx.arc(64,64,56,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=hot?'#fef08a':'rgba(34,211,238,0.72)';
    ctx.lineWidth=hot?7:4.5;ctx.stroke();
    drawPinIconPath(ctx,64,65,62,hot?'#ffffff':'#67e8f9','rgba(8,22,34,0.96)');
  });
}

const _trailColorTmp=new THREE.Color();
const _turboSrgbBuf=new Float32Array(3);
const _trailOutsidePosBuf=[0,0,0];

function writeSampleLinearColor(entry,speedMps,outArr,offset){
  if(trailColorMode==='speed'){
    speedToTurboColor(speedMps,{refSpeedMps:PATHLINE_DEFAULTS.turboRefSpeedMps,out:_turboSrgbBuf});
    _trailColorTmp.setRGB(_turboSrgbBuf[0],_turboSrgbBuf[1],_turboSrgbBuf[2],THREE.SRGBColorSpace);
    outArr[offset]=_trailColorTmp.r;
    outArr[offset+1]=_trailColorTmp.g;
    outArr[offset+2]=_trailColorTmp.b;
    return;
  }
  const rgb=entry.uniqueLinearRgb;
  outArr[offset]=rgb[0];
  outArr[offset+1]=rgb[1];
  outArr[offset+2]=rgb[2];
}

function waypointDotColorFor(entry,wp){
  if(wp?.isTimeCopy)return entry.timeCopyDotColor;
  if(trailColorMode==='speed'){
    const spd=Number.isFinite(wp?.speedMps)?wp.speedMps:0;
    speedToTurboColor(spd,{refSpeedMps:PATHLINE_DEFAULTS.turboRefSpeedMps,out:_turboSrgbBuf});
    _trailColorTmp.setRGB(_turboSrgbBuf[0],_turboSrgbBuf[1],_turboSrgbBuf[2],THREE.SRGBColorSpace);
    return _trailColorTmp;
  }
  return entry.dotColor;
}

function recolorMovableTrail(entry){
  if(entry.trailTube&&entry.lineCount>0&&entry.lineColors&&entry.lineSpeeds){
    const n=entry.lineCount;
    const s=entry.lineSpeeds;
    const b=entry.lineBreaks;
    for(let i=0;i<n;i++){
      const isStart=i===0||Boolean(b&&b[i]);
      const isEnd=i===n-1||Boolean(b&&b[i+1]);
      const spd=(!isStart&&!isEnd)?(s[i-1]*0.25+s[i]*0.50+s[i+1]*0.25):s[i];
      writeSampleLinearColor(entry,spd,entry.lineColors,i*3);
    }
    entry.trailTube.syncColors(entry.lineColors,n);
  }
  if(entry.dotsMesh&&entry.waypoints.length>0){
    for(let i=0;i<entry.waypoints.length;i++){
      entry.dotsMesh.setColorAt(i,waypointDotColorFor(entry,entry.waypoints[i]));
    }
    if(entry.dotsMesh.instanceColor)entry.dotsMesh.instanceColor.needsUpdate=true;
  }
}

function syncMovableTrailOutsideFlags(entry){
  if(!entry.trailTube||!entry.lineOutside||!entry.linePositions||entry.lineCount<=0)return;
  const n=entry.lineCount;
  const fenceActive=Boolean(perimeterFence?.isActive?.());
  let changed=false;
  let prevOut=false;
  for(let i=0;i<n;i++){
    if(entry.lineBreaks?.[i])prevOut=false;
    let next=0;
    if(fenceActive){
      const o=i*3;
      _trailOutsidePosBuf[0]=entry.linePositions[o];
      _trailOutsidePosBuf[1]=entry.linePositions[o+1];
      _trailOutsidePosBuf[2]=entry.linePositions[o+2];
      next=perimeterFence.isOutsideXY(_trailOutsidePosBuf,{wasOutside:prevOut}).outside?1:0;
      prevOut=next===1;
    }
    if(entry.lineOutside[i]!==next){
      entry.lineOutside[i]=next;
      changed=true;
    }
  }
  if(changed){
    entry.trailTube.syncOutsideFlags(entry.lineOutside,n);
  }
}

function freeTrailGapSlot(gap){
  if(!gap)return;
  if(gap.bridgeSlot>=0)trailGapBridgeBatch.free(gap.bridgeSlot);
  if(gap.endCapSlot>=0)trailGapCapBatch.free(gap.endCapSlot);
  if(gap.startCapSlot>=0)trailGapCapBatch.free(gap.startCapSlot);
}

function freeEntryGapVisuals(entry){
  if(!entry?.trailGaps?.length)return;
  for(const g of entry.trailGaps)freeTrailGapSlot(g);
  entry.trailGaps.length=0;
}

function addTrailGapVisual(entry,startIndex){
  if(!entry?.linePositions||startIndex<1||startIndex>=entry.lineCount)return;
  const p0=(startIndex-1)*3;
  const p1=startIndex*3;
  const ax=entry.linePositions[p0],ay=entry.linePositions[p0+1],az=entry.linePositions[p0+2];
  const bx=entry.linePositions[p1],by=entry.linePositions[p1+1],bz=entry.linePositions[p1+2];
  const bridgeSlot=trailGapBridgeBatch.alloc();
  if(bridgeSlot>=0){
    trailGapBridgeBatch.setRod(bridgeSlot,ax,ay,az,bx,by,bz,PATHLINE_DEFAULTS.gapBridgeRadiusM);
    trailGapBridgeBatch.setColor(bridgeSlot,PATHLINE_DEFAULTS.gapBridgeColorHex);
  }
  const capR=PATHLINE_DEFAULTS.gapCapRadiusM;
  const endCapSlot=trailGapCapBatch.alloc();
  if(endCapSlot>=0){
    trailGapCapBatch.setPoint(endCapSlot,ax,ay,az,capR);
    trailGapCapBatch.setColor(endCapSlot,PATHLINE_DEFAULTS.gapEndCapColorHex);
  }
  const startCapSlot=trailGapCapBatch.alloc();
  if(startCapSlot>=0){
    trailGapCapBatch.setPoint(startCapSlot,bx,by,bz,capR);
    trailGapCapBatch.setColor(startCapSlot,PATHLINE_DEFAULTS.gapStartCapColorHex);
  }
  entry.trailGaps.push({startIndex,bridgeSlot,endCapSlot,startCapSlot});
}

function setTrailColorMode(nextMode,{silent=false}={}){
  const resolved=nextMode==='speed'?'speed':'id';
  if(trailColorMode===resolved)return;
  trailColorMode=resolved;
  if(trailColorModeEl)trailColorModeEl.value=resolved;
  for(const entry of movableTools.values())recolorMovableTrail(entry);
  lastMeasureTrailFocusKey='';
  if(!extraScanActive)syncMeasurementTrailFocus({force:true});
  lastModeColorKey='';
  turboBarSprite.visible=Boolean(modePillGroup.visible&&resolved==='speed');
  if(turboBarSprite.visible)drawTurboColorBar(PATHLINE_DEFAULTS.turboRefSpeedMps);
  pushDiagEvent({type:'trail-color-mode',mode:resolved,atPerformanceMs:performance.now()});
  if(!silent){
    playModeTick('pin');
    showHudToast(
      resolved==='speed'?'TRAIL: SPEED (TURBO 0–600 mm/s)':'TRAIL: UNIQUE TAG COLOR',
      resolved==='speed'?'#f97316':'#38bdf8',
      1600,
    );
  }
}
function toggleTrailColorMode(){
  setTrailColorMode(trailColorMode==='speed'?'id':'speed');
}
if(trailColorModeEl){
  trailColorModeEl.onchange=()=>{
    if(urlParams.has('trailColor'))urlParams.delete('trailColor');
    setTrailColorMode(trailColorModeEl.value,{silent:true});
  };
}

const _dotGrayColor=new THREE.Color(0x8c96a6);
const _dotDimTimeCopyColor=new THREE.Color(0x475569).lerp(new THREE.Color(0x67e8f9),0.35);
const _dotStyleColor=new THREE.Color();
const _dotWhiteColor=new THREE.Color(0xffffff);
let lastMeasureTrailFocusKey='';

function findWaypointLineVertexIndex(entry,wp,cursorRef){
  const n=entry.lineCount|0;
  if(!wp?.isTrailPoint||n<=0||!entry.lineSegmentIds||!entry.lineSeqs)return -1;
  const targetSeg=wp.segmentIndex|0;
  const targetSeq=wp.lineSeq;
  if(targetSeq!=null&&Number.isFinite(targetSeq)){
    let k=Math.max(0,Math.min(n-1,cursorRef.idx));
    while(k+1<n&&entry.lineSeqs[k]<targetSeq)k++;
    cursorRef.idx=k;
    if(entry.lineSegmentIds[k]===targetSeg){
      if(k>0&&entry.lineSegmentIds[k-1]===targetSeg&&Math.abs(entry.lineSeqs[k-1]-targetSeq)<Math.abs(entry.lineSeqs[k]-targetSeq)){
        return k-1;
      }
      return k;
    }
    if(k>0&&entry.lineSegmentIds[k-1]===targetSeg)return k-1;
  }
  for(let i=0;i<n;i++){
    if(entry.lineSegmentIds[i]===targetSeg)return i;
  }
  return -1;
}

function syncMeasurementTrailFocus({force=false}={}){
  let focusKey;
  const pinnedTrailWps=[];
  const hoveredTrailWps=[];
  const pinnedTimeCopyKeys=[];
  const hoveredTimeCopyKeys=[];

  if(extraScanActive){
    focusKey='scan';
  }else{
    const pinnedStrandCounts=new Map();
    for(const [k,pin] of pinnedPoints.entries()){
      const wp=pin.wp;
      if(wp?.isTrailPoint&&wp.segmentIndex>=1){
        pinnedTrailWps.push(wp);
        const sk=`${wp.tagId}:${wp.segmentIndex}`;
        pinnedStrandCounts.set(sk,(pinnedStrandCounts.get(sk)||0)+1);
      }else if(wp?.isTimeCopy){
        pinnedTimeCopyKeys.push(k);
      }
    }
    if(hoveredWaypoint){
      if(hoveredWaypoint.isTrailPoint&&hoveredWaypoint.segmentIndex>=1){
        hoveredTrailWps.push(hoveredWaypoint);
      }else if(hoveredWaypoint.isTimeCopy){
        hoveredTimeCopyKeys.push(waypointIdKey(hoveredWaypoint));
      }
    }
    if(hoveredWaypoint2){
      if(hoveredWaypoint2.isTrailPoint&&hoveredWaypoint2.segmentIndex>=1){
        hoveredTrailWps.push(hoveredWaypoint2);
      }else if(hoveredWaypoint2.isTimeCopy){
        hoveredTimeCopyKeys.push(waypointIdKey(hoveredWaypoint2));
      }
    }

    const dualSameStrand=
      hoveredTrailWps.length>=2&&
      hoveredTrailWps[0].tagId===hoveredTrailWps[1].tagId&&
      hoveredTrailWps[0].segmentIndex===hoveredTrailWps[1].segmentIndex;

    const pinParts=pinnedTrailWps.map(wp=>`${wp.tagId}:${wp.segmentIndex}@${wp.lineSeq??'x'}`);
    const hoverParts=hoveredTrailWps.map(wp=>{
      const sk=`${wp.tagId}:${wp.segmentIndex}`;
      const needSeq=dualSameStrand||(pinnedStrandCounts.get(sk)||0)>=1;
      return needSeq?`${sk}@${wp.lineSeq??'x'}`:sk;
    });
    const tcPart=(pinnedTrailWps.length>0||hoveredTrailWps.length>0)
      ?`${pinnedTimeCopyKeys.join(',')}|${hoveredTimeCopyKeys.join(',')}`
      :'';
    focusKey=`measure|${trailColorMode}|${pinParts.join(',')}|${hoverParts.join(',')}|${tcPart}`;
  }

  if(!force&&focusKey===lastMeasureTrailFocusKey)return;
  lastMeasureTrailFocusKey=focusKey;

  const focus=buildTrailMeasureFocus({
    active:!extraScanActive,
    pinnedPoints:pinnedTrailWps,
    hoveredPoints:hoveredTrailWps,
  });

  const preserveChroma=trailColorMode==='speed';
  const cursorRef={idx:0};

  for(const entry of movableTools.values()){
    let summary=null;
    if(entry.trailTube&&entry.lineCount>0){
      summary=evaluateEntryTrailMeasureStyle({
        tagId:entry.tagId,
        lineCount:entry.lineCount,
        lineSegmentIds:entry.lineSegmentIds,
        lineSeqs:entry.lineSeqs,
        baseColors:entry.lineColors,
        focus,
        baseRadiusM:PATHLINE_DEFAULTS.lineRadiusM,
        preserveChroma,
        outRoles:entry.lineRoles,
        outRadii:entry.lineRadii,
        outFades:entry.lineFades,
        outColors:entry.lineRenderColors,
      });

      let targetMat=trailLineMat;
      let targetOrder=2;
      let sortByFade=false;
      if(summary.hasFocus){
        if(!summary.hasActiveVertex){
          targetMat=trailLineDimMat;
          targetOrder=1;
        }else if(summary.hasDimVertex){
          targetMat=trailLineDimMat;
          targetOrder=3;
          sortByFade=summary.hasMixedLayers;
        }else{
          targetMat=trailLineMat;
          targetOrder=3;
        }
      }
      entry.trailTube.mesh.material=targetMat;
      entry.trailTube.mesh.renderOrder=targetOrder;
      entry.trailTube.syncFromPositions(entry.linePositions,entry.lineCount,{
        radii:entry.lineRadii,
        colors:entry.lineRenderColors,
        outsideFlags:entry.lineOutside,
        fades:entry.lineFades,
        breakFlags:entry.lineBreaks,
        fromIndex:0,
        sortByFade,
      });
    }

    if(entry.dotsMesh&&entry.waypoints.length>0){
      cursorRef.idx=0;
      let hasHighlightedTimeCopy=false;
      for(let i=0;i<entry.waypoints.length;i++){
        const wp=entry.waypoints[i];
        let dotR=0;
        let dotCol=waypointDotColorFor(entry,wp);
        if(wp.isSegmentStart){
          dotR=0;
        }else if(wp.isTimeCopy){
          const tcFocused=!focus.hasFocus||pinnedPoints.has(waypointIdKey(wp))||hoveredWaypoint===wp||hoveredWaypoint2===wp;
          if(focus.hasFocus&&tcFocused)hasHighlightedTimeCopy=true;
          dotR=PATHLINE_DEFAULTS.timeCopyDotRadiusM*(tcFocused?1.0:0.85);
          dotCol=tcFocused?entry.timeCopyDotColor:_dotDimTimeCopyColor;
        }else if(!summary||!summary.hasFocus){
          dotR=PATHLINE_DEFAULTS.waypointDotRadiusM;
        }else{
          const vIdx=findWaypointLineVertexIndex(entry,wp,cursorRef);
          const role=vIdx>=0
            ?entry.lineRoles[vIdx]
            :(focus.strands.has(`${entry.tagId}:${wp.segmentIndex}`)?TRAIL_VERTEX_ROLE.HOVERED:TRAIL_VERTEX_ROLE.UNRELATED);
          if(role===TRAIL_VERTEX_ROLE.UNRELATED){
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM*0.70;
            dotCol=_dotGrayColor;
          }else if(role===TRAIL_VERTEX_ROLE.PINNED_SPAN){
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM*1.24;
            if(vIdx>=0&&entry.lineRenderColors){
              _dotStyleColor.setRGB(entry.lineRenderColors[vIdx*3],entry.lineRenderColors[vIdx*3+1],entry.lineRenderColors[vIdx*3+2]).lerp(_dotWhiteColor,0.25);
              dotCol=_dotStyleColor;
            }
          }else if(role===TRAIL_VERTEX_ROLE.PINNED_PREVIEW||role===TRAIL_VERTEX_ROLE.HOVERED){
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM*1.12;
            if(vIdx>=0&&entry.lineRenderColors){
              _dotStyleColor.setRGB(entry.lineRenderColors[vIdx*3],entry.lineRenderColors[vIdx*3+1],entry.lineRenderColors[vIdx*3+2]).lerp(_dotWhiteColor,0.20);
              dotCol=_dotStyleColor;
            }
          }else if(role===TRAIL_VERTEX_ROLE.PINNED_SINGLE){
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM*0.96;
            if(vIdx>=0&&entry.lineRenderColors){
              _dotStyleColor.setRGB(entry.lineRenderColors[vIdx*3],entry.lineRenderColors[vIdx*3+1],entry.lineRenderColors[vIdx*3+2]);
              dotCol=_dotStyleColor;
            }
          }else if(role===TRAIL_VERTEX_ROLE.PINNED_OUTER){
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM*0.84;
            if(vIdx>=0&&entry.lineRenderColors){
              _dotStyleColor.setRGB(entry.lineRenderColors[vIdx*3],entry.lineRenderColors[vIdx*3+1],entry.lineRenderColors[vIdx*3+2]);
              dotCol=_dotStyleColor;
            }
          }else{
            dotR=PATHLINE_DEFAULTS.waypointDotRadiusM;
          }
        }
        _tmpMat4.makeScale(dotR,dotR,dotR).setPosition(wp.centerL[0],wp.centerL[1],wp.centerL[2]);
        entry.dotsMesh.setMatrixAt(i,_tmpMat4);
        entry.dotsMesh.setColorAt(i,dotCol);
      }
      entry.dotsMesh.instanceMatrix.needsUpdate=true;
      if(entry.dotsMesh.instanceColor)entry.dotsMesh.instanceColor.needsUpdate=true;

      if(entry.dotMat){
        if(!focus.hasFocus){
          entry.dotMat.opacity=0.92;
          entry.dotMat.depthWrite=true;
          entry.dotsMesh.renderOrder=3;
        }else if((summary?.hasActiveVertex??false)||hasHighlightedTimeCopy){
          entry.dotMat.opacity=0.94;
          entry.dotMat.depthWrite=true;
          entry.dotsMesh.renderOrder=4;
        }else{
          entry.dotMat.opacity=0.24;
          entry.dotMat.depthWrite=false;
          entry.dotsMesh.renderOrder=1;
        }
      }
    }

    if(entry.trailGaps?.length&&entry.linePositions&&entry.lineCount>0){
      const capR=PATHLINE_DEFAULTS.gapCapRadiusM;
      const bridgeR=PATHLINE_DEFAULTS.gapBridgeRadiusM;
      for(const g of entry.trailGaps){
        if(g.startIndex<1||g.startIndex>=entry.lineCount)continue;
        const p0=(g.startIndex-1)*3;
        const p1=g.startIndex*3;
        const ax=entry.linePositions[p0],ay=entry.linePositions[p0+1],az=entry.linePositions[p0+2];
        const bx=entry.linePositions[p1],by=entry.linePositions[p1+1],bz=entry.linePositions[p1+2];
        if(!summary||!summary.hasFocus){
          if(g.bridgeSlot>=0){
            trailGapBridgeBatch.setRod(g.bridgeSlot,ax,ay,az,bx,by,bz,bridgeR);
            trailGapBridgeBatch.setColor(g.bridgeSlot,PATHLINE_DEFAULTS.gapBridgeColorHex);
          }
          if(g.endCapSlot>=0){
            trailGapCapBatch.setPoint(g.endCapSlot,ax,ay,az,capR);
            trailGapCapBatch.setColor(g.endCapSlot,PATHLINE_DEFAULTS.gapEndCapColorHex);
          }
          if(g.startCapSlot>=0){
            trailGapCapBatch.setPoint(g.startCapSlot,bx,by,bz,capR);
            trailGapCapBatch.setColor(g.startCapSlot,PATHLINE_DEFAULTS.gapStartCapColorHex);
          }
        }else{
          const endRole=entry.lineRoles[g.startIndex-1];
          const startRole=entry.lineRoles[g.startIndex];
          const endActive=endRole!==TRAIL_VERTEX_ROLE.UNRELATED;
          const startActive=startRole!==TRAIL_VERTEX_ROLE.UNRELATED;
          const anyActive=endActive||startActive;
          if(g.bridgeSlot>=0){
            trailGapBridgeBatch.setRod(g.bridgeSlot,ax,ay,az,bx,by,bz,bridgeR*(anyActive?0.78:0.60));
            trailGapBridgeBatch.setColor(g.bridgeSlot,anyActive?PATHLINE_DEFAULTS.gapBridgeColorHex:0x64748b);
          }
          if(g.endCapSlot>=0){
            trailGapCapBatch.setPoint(g.endCapSlot,ax,ay,az,capR*(endActive?1.05:0.68));
            trailGapCapBatch.setColor(g.endCapSlot,endActive?PATHLINE_DEFAULTS.gapEndCapColorHex:0x64748b);
          }
          if(g.startCapSlot>=0){
            trailGapCapBatch.setPoint(g.startCapSlot,bx,by,bz,capR*(startActive?1.05:0.68));
            trailGapCapBatch.setColor(g.startCapSlot,startActive?PATHLINE_DEFAULTS.gapStartCapColorHex:0x64748b);
          }
        }
      }
    }
  }
}

function ensureMovableTrailVisuals(entry){
  const accentCol=new THREE.Color(entry.theme.hex);
  if(!entry.uniqueLinearRgb){
    entry.uniqueLinearRgb=[accentCol.r,accentCol.g,accentCol.b];
  }
  if(!entry.trailTube){
    const rawLinePositions=new Float32Array(MAX_LINE_PTS*3);
    const linePositions=new Float32Array(MAX_LINE_PTS*3);
    const lineSpeeds=new Float32Array(MAX_LINE_PTS);
    const lineColors=new Float32Array(MAX_LINE_PTS*3);
    const lineRenderColors=new Float32Array(MAX_LINE_PTS*3);
    const lineRadii=new Float32Array(MAX_LINE_PTS);
    lineRadii.fill(PATHLINE_DEFAULTS.lineRadiusM);
    const lineFades=new Float32Array(MAX_LINE_PTS);
    lineFades.fill(PATHLINE_DEFAULTS.lineOpacity);
    const lineRoles=new Uint8Array(MAX_LINE_PTS);
    const lineOutside=new Uint8Array(MAX_LINE_PTS);
    const lineBreaks=new Uint8Array(MAX_LINE_PTS);
    const lineSegmentIds=new Uint32Array(MAX_LINE_PTS);
    const lineSeqs=new Uint32Array(MAX_LINE_PTS);
    const trailTube=createPathTubeMesh(MAX_LINE_PTS,trailLineMat,{
      radius:PATHLINE_DEFAULTS.lineRadiusM,
      sides:PATHLINE_DEFAULTS.lineSides,
      vertexColors:true,
      dashAttribute:true,
      fadeAttribute:true,
      renderOrder:2,
      name:`trail-${entry.tagId}`,
    });
    extraRoot.add(trailTube.mesh);
    entry.trailTube=trailTube;
    entry.rawLinePositions=rawLinePositions;
    entry.linePositions=linePositions;
    entry.lineSpeeds=lineSpeeds;
    entry.lineColors=lineColors;
    entry.lineRenderColors=lineRenderColors;
    entry.lineRadii=lineRadii;
    entry.lineFades=lineFades;
    entry.lineRoles=lineRoles;
    entry.lineOutside=lineOutside;
    entry.lineBreaks=lineBreaks;
    entry.lineSegmentIds=lineSegmentIds;
    entry.lineSeqs=lineSeqs;
    entry.lastTrailFenceVer=perimeterFence?.version?.()??-1;
  }
  if(!entry.dotsMesh){
    const dotColor=new THREE.Color(0xf8fafc).lerp(accentCol,0.65);
    const timeCopyDotColor=new THREE.Color(0x67e8f9);
    const dotMat=new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:0.92});
    const dotsMesh=new THREE.InstancedMesh(_waypointDotGeo,dotMat,MAX_TRAIL_WAYPOINTS);
    dotsMesh.setColorAt(0,dotColor);
    dotsMesh.count=0;
    dotsMesh.frustumCulled=false;
    extraRoot.add(dotsMesh);
    entry.dotsMesh=dotsMesh;
    entry.dotMat=dotMat;
    entry.dotColor=dotColor;
    entry.timeCopyDotColor=timeCopyDotColor;
  }
}

const MAX_WAKE_OUT_PTS=MOTION_WAKE_DEFAULTS.maxPoints+2;
function ensureMovableWakeVisuals(entry){
  if(!entry.wakeBuffer){
    entry.wakeBuffer=createMotionWakeBuffer();
  }
  if(!entry.wakeLinearRgb){
    const baseCol=new THREE.Color(entry.theme.hex).lerp(new THREE.Color(0xffffff),0.18);
    entry.wakeLinearRgb=[baseCol.r,baseCol.g,baseCol.b];
  }
  if(!entry.wakeTube){
    const wakeTube=createPathTubeMesh(MAX_WAKE_OUT_PTS,wakeLineMat,{
      radius:MOTION_WAKE_DEFAULTS.maxRadiusM,
      sides:5,
      vertexColors:true,
      fadeAttribute:true,
      renderOrder:1,
      name:`wake-${entry.tagId}`,
    });
    wakeTube.mesh.visible=false;
    extraRoot.add(wakeTube.mesh);
    entry.wakeTube=wakeTube;
    entry.wakeColors=new Float32Array(MAX_WAKE_OUT_PTS*3);
  }
}

function seedMovableWake(entry,pose,atMs){
  if(!pose||isEntryRecording(entry)||!extraScanActive)return;
  if(!entry.wakeBuffer)entry.wakeBuffer=createMotionWakeBuffer();
  entry.wakeBuffer.seed([pose[12],pose[13],pose[14]],atMs);
}

function clearMovableWake(entry){
  if(entry.wakeBuffer)entry.wakeBuffer.clear();
  if(entry.wakeTube){
    entry.wakeTube.clear();
    entry.wakeTube.mesh.visible=false;
  }
}

function updateMovableWake(entry,nowMs,{moving=false,pose=null,speedMps=0}={}){
  if(!extraScanActive||isEntryRecording(entry)){
    clearMovableWake(entry);
    return;
  }
  if(moving&&pose&&pose.length>=16){
    if(!entry.wakeBuffer)entry.wakeBuffer=createMotionWakeBuffer();
    _alertPosBuf[0]=pose[12];
    _alertPosBuf[1]=pose[13];
    _alertPosBuf[2]=pose[14];
    entry.wakeBuffer.push(_alertPosBuf,nowMs,{speedMps});
  }
  if(!entry.wakeBuffer||!entry.wakeBuffer.hasPoints()){
    if(entry.wakeTube&&entry.wakeTube.mesh.visible){
      entry.wakeTube.clear();
      entry.wakeTube.mesh.visible=false;
    }
    return;
  }
  const sample=entry.wakeBuffer.sample(nowMs,{includeLiveHead:moving});
  if(sample.count<2){
    if(entry.wakeTube&&entry.wakeTube.mesh.visible){
      entry.wakeTube.clear();
      entry.wakeTube.mesh.visible=false;
    }
    return;
  }
  ensureMovableWakeVisuals(entry);
  const n=sample.count;
  const cols=entry.wakeColors;
  if(trailColorMode==='speed'){
    for(let i=0;i<n;i++){
      writeSampleLinearColor(entry,sample.speeds[i],cols,i*3);
    }
  }else{
    const rgb=entry.wakeLinearRgb;
    for(let i=0;i<n;i++){
      const o=i*3;
      cols[o]=rgb[0];
      cols[o+1]=rgb[1];
      cols[o+2]=rgb[2];
    }
  }
  entry.wakeTube.mesh.visible=true;
  entry.wakeTube.syncFromPositions(sample.positions,n,{
    radii:sample.radii,
    colors:cols,
    fades:sample.fades,
    fromIndex:0,
  });
}

function createMovableTool(tagId,edgeMm){
  const theme=ribbonThemeFor(tagId);
  const frameSlot=frameBatch.alloc();
  if(frameSlot>=0){
    frameBatch.setColor(frameSlot,theme.hex);
    frameBatch.setVisible(frameSlot,false);
  }
  const alertSlot=alertGlowBatch.alloc();
  if(alertSlot>=0)alertGlowBatch.setVisible(alertSlot,false);
  const axesSlot=axesBatch.alloc();
  if(axesSlot>=0)axesBatch.setVisible(axesSlot,false);
  const faceSlot=tagFaceAtlas.acquire();
  if(faceSlot>=0)tagFaceMesh.setQuadVisible(faceSlot,false);
  const recOrbSlot=recOrbBatch.alloc();
  if(recOrbSlot>=0){
    recOrbBatch.setColor(recOrbSlot,0x0ea5e9);
    recOrbBatch.setVisible(recOrbSlot,false);
  }
  const recPillSlot=badgeAtlas.acquire();
  if(recPillSlot>=0)badgeMesh.setQuadVisible(recPillSlot,false);
  const markOrbSlot=recOrbBatch.alloc();
  if(markOrbSlot>=0){
    recOrbBatch.setColor(markOrbSlot,0x22d3ee);
    recOrbBatch.setVisible(markOrbSlot,false);
  }
  const markPillSlot=badgeAtlas.acquire();
  if(markPillSlot>=0)badgeMesh.setQuadVisible(markPillSlot,false);

  const entry={
    tagId,edgeMm,theme,
    frameSlot,alertSlot,axesSlot,faceSlot,faceKey:'',
    recOrbSlot,recPillSlot,recPillKey:'',
    markOrbSlot,markPillSlot,markPillKey:'',
    visible:false,faceVisible:false,recOrbVisible:false,markOrbVisible:false,outsidePerimeter:false,lastAlertFenceVer:-1,
    frameColorHex:theme.hex,recOrbColorHex:0x0ea5e9,markOrbColorHex:0x22d3ee,recOrbScale:1,
    trailTube:null,uniqueLinearRgb:null,rawLinePositions:null,linePositions:null,lineSpeeds:null,lineColors:null,lineRenderColors:null,lineRadii:null,lineFades:null,lineRoles:null,lineOutside:null,lineBreaks:null,lineSegmentIds:null,lineSeqs:null,lineSeqCounter:0,passAnalysisSeq:-1,passAnalysisMap:null,trailGaps:[],trailSegmentCount:0,pendingTrailBreak:false,lastTrailFenceVer:-1,lineCount:0,
    wakeBuffer:null,wakeTube:null,wakeColors:null,wakeLinearRgb:null,
    dotsMesh:null,dotMat:null,dotColor:null,timeCopyDotColor:null,
    waypoints:[],waypointCount:0,timeCopyCount:0,firstAtMs:null,smoothedSpeedMps:0,hasEverAnnouncedLock:false,
    lockState:'settling',settleSamples:[],lockedMatrix:null,lockedSinceMs:0,lockQuality:null,refineSamples:[],relocPending:null,
    stillAnchor:null,stillSinceMs:null,stillSamples:[],lockFlashUntilMs:0,currentPose:null,
    lastMatrix:null,lastOpticalMatrix:null,targetMatrix:null,displayMatrix:null,targetConfidence:1,lastAt:0,lastOpticalAtMs:0,lastAcceptedOpticalAtMs:0,lastOpticalXrTimeMs:0,lastSpeedMps:0,lastApparentEdgePx:0,lastOpticalRmsPx:null,pendingJump:null,skipNextJumpGuardUntilMs:0,
    errOffsetPos:[0,0,0],errOffsetRot:[0,0,0,1],errDecayTauSec:0.055,
    handCoupled:false,coupledHand:null,coupledStepCount:0,measureModeApplied:false,measureSyncedPose:null,lastCoupledStepAtMs:0,postReleaseBoostUntilMs:0,releaseSpeedMps:0,pendingTrail:createCoupledTrailBuffer(),
    recentPoses:[],recording:false,autoRecording:false,lastRecLineMatrix:null,lastRecFrameMatrix:null,lastRecAtMs:0,lastCommittedPose:null,lastCommittedAtMs:0,
    markHoldSince:null,markHoldConsumed:false,recHoldSince:null,recHoldConsumed:false,recOrbPosL:[0,0,0],markOrbPosL:[0,0,0],
    hasFenceSample:false,lastFencePosL:[0,0,0],lastInsidePosL:null,lastBreachAtMs:0,
    pillRestState:createTagRestState(),
  };
  drawLiveBadge(entry,'settling',false);
  drawRecOrbPill(entry,false,false);
  drawMarkOrbPill(entry,false);
  return entry;
}

function syncCurrentPose(entry,matrix,atMs){
  const cx=matrix[12],cy=matrix[13],cz=matrix[14];
  const nz=Math.max(-1,Math.min(1,matrix[10]));
  const tiltDeg=Math.acos(Math.abs(nz))*180/Math.PI;
  const clock=new Date().toTimeString().slice(0,8);
  if(entry.firstAtMs==null)entry.firstAtMs=atMs;
  const wp={
    isCurrent:true,tagId:entry.tagId,edgeMm:entry.edgeMm,index:0,
    matrix:matrix.slice(),centerL:[cx,cy,cz],
    xMm:cx*1000,yMm:cy*1000,zMm:cz*1000,tiltDeg,atMs,
    elapsedS:Math.max(0,(atMs-entry.firstAtMs)/1000),clock,
  };
  entry.currentPose=wp;
  // If this locked tag is already pinned, keep its pinned 3D frame and pairwise distance rails in sync with the new locked pose.
  syncPinnedWaypointPose(wp);
  return wp;
}

function appendLinePoint(entry,pose,speedMps=0,{isBreak=false,atMs=null}={}){
  if(!pose||pose.length<16)return;
  ensureMovableTrailVisuals(entry);
  const cx=pose[12],cy=pose[13],cz=pose[14];
  const fullRebuild=entry.lineCount>=MAX_LINE_PTS;
  let idx;
  if(fullRebuild){
    entry.rawLinePositions.copyWithin(0,3,MAX_LINE_PTS*3);
    entry.linePositions.copyWithin(0,3,MAX_LINE_PTS*3);
    entry.lineSpeeds.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineColors.copyWithin(0,3,MAX_LINE_PTS*3);
    entry.lineRenderColors.copyWithin(0,3,MAX_LINE_PTS*3);
    entry.lineRadii.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineFades.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineRoles.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineOutside.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineBreaks.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineSegmentIds.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineSeqs.copyWithin(0,1,MAX_LINE_PTS);
    entry.lineBreaks[0]=0;
    for(let gi=entry.trailGaps.length-1;gi>=0;gi--){
      const g=entry.trailGaps[gi];
      g.startIndex--;
      if(g.startIndex<=0){
        freeTrailGapSlot(g);
        entry.trailGaps.splice(gi,1);
      }
    }
    idx=MAX_LINE_PTS-1;
  }else{
    idx=entry.lineCount++;
  }
  const resolvedBreak=Boolean(isBreak&&idx>0);
  entry.lineBreaks[idx]=resolvedBreak?1:0;
  if(idx===0)entry.trailSegmentCount=1;
  else if(resolvedBreak)entry.trailSegmentCount=Math.max(1,entry.trailSegmentCount)+1;
  entry.lineSegmentIds[idx]=entry.trailSegmentCount;
  entry.lineSeqs[idx]=++entry.lineSeqCounter;
  entry.lineRadii[idx]=PATHLINE_DEFAULTS.lineRadiusM;
  entry.lineFades[idx]=PATHLINE_DEFAULTS.lineOpacity;
  const ptSpeed=resolvedBreak?0:speedMps;
  const i=idx*3;
  entry.rawLinePositions[i]=cx;entry.rawLinePositions[i+1]=cy;entry.rawLinePositions[i+2]=cz;
  entry.linePositions[i]=cx;entry.linePositions[i+1]=cy;entry.linePositions[i+2]=cz;
  entry.lineSpeeds[idx]=ptSpeed;
  writeSampleLinearColor(entry,ptSpeed,entry.lineColors,i);
  if(!resolvedBreak&&idx>=2&&!entry.lineBreaks[idx-1]){
    const avgSpd=entry.lineSpeeds[idx-2]*0.25+entry.lineSpeeds[idx-1]*0.50+ptSpeed*0.25;
    writeSampleLinearColor(entry,avgSpd,entry.lineColors,(idx-1)*3);
  }
  smoothLineTail(entry.rawLinePositions,entry.linePositions,entry.lineCount,{maxDeviationM:0.012,breakFlags:entry.lineBreaks});
  const fenceActive=Boolean(perimeterFence?.isActive?.());
  const outStart=fullRebuild?0:Math.max(0,entry.lineCount-4);
  let prevOut=(outStart>0&&!entry.lineBreaks[outStart])?entry.lineOutside[outStart-1]===1:false;
  for(let k=outStart;k<entry.lineCount;k++){
    if(entry.lineBreaks[k])prevOut=false;
    if(!fenceActive){
      entry.lineOutside[k]=0;
      continue;
    }
    const ko=k*3;
    _trailOutsidePosBuf[0]=entry.linePositions[ko];
    _trailOutsidePosBuf[1]=entry.linePositions[ko+1];
    _trailOutsidePosBuf[2]=entry.linePositions[ko+2];
    const out=perimeterFence.isOutsideXY(_trailOutsidePosBuf,{wasOutside:prevOut}).outside;
    entry.lineOutside[k]=out?1:0;
    prevOut=out;
  }
  entry.lastTrailFenceVer=perimeterFence?.version?.()??-1;
  if(entry.trailTube.mesh.material!==trailLineMat){
    entry.trailTube.mesh.material=trailLineMat;
    entry.trailTube.mesh.renderOrder=2;
  }
  entry.trailTube.syncFromPositions(entry.linePositions,entry.lineCount,{
    radii:entry.lineRadii,
    colors:entry.lineColors,
    outsideFlags:entry.lineOutside,
    fades:entry.lineFades,
    breakFlags:entry.lineBreaks,
    fromIndex:fullRebuild?0:Math.max(0,entry.lineCount-6),
  });
  if(resolvedBreak)addTrailGapVisual(entry,idx);
  entry.lastCommittedPose=pose.slice();
  if(Number.isFinite(atMs))entry.lastCommittedAtMs=atMs;
}

function recordMovablePathPoint(entry,pose,atMs,{opticalGapMs=null}={}){
  if(!isEntryRecording(entry)||!pose)return;
  if(entry.lineCount>0&&entry.lastCommittedPose){
    const gap=shouldStartNewTrailSegment({
      prevPose:entry.lastCommittedPose,
      nextPose:pose,
      dtMs:opticalGapMs,
      explicitBreak:entry.pendingTrailBreak,
    });
    if(gap.isBreak){
      entry.pendingTrailBreak=false;
      entry.smoothedSpeedMps=0;
      entry.lastRecLineMatrix=pose.slice();
      entry.lastRecFrameMatrix=pose.slice();
      entry.lastRecAtMs=atMs;
      appendLinePoint(entry,pose,0,{isBreak:true,atMs});
      stampMovableWaypoint(entry.tagId,entry.edgeMm,pose,atMs,{silent:true,skipLine:true,speedMps:0,isSegmentStart:true});
      pushDiagEvent({
        type:'trail-gap',tagId:entry.tagId,reason:gap.reason,
        distMm:Math.round(gap.distM*1000),gapMs:Math.round(gap.dtMs||0),
        segment:entry.trailSegmentCount,atPerformanceMs:performance.now(),
      });
      return;
    }
    if(entry.pendingTrailBreak){
      entry.pendingTrailBreak=false;
      if(!entry.lastRecLineMatrix){
        entry.lastRecLineMatrix=entry.lastCommittedPose.slice();
        entry.lastRecAtMs=atMs;
      }
    }
  }
  let linePosMm=Infinity;
  if(entry.lastRecLineMatrix){
    const dl=rigidVector(product(inverse(entry.lastRecLineMatrix),pose));
    linePosMm=Math.hypot(...dl.slice(3))*1000;
  }
  // Guard against multi-frame synthetic interpolation bursts across a tracking jump.
  const dtRecSec=entry.lastRecAtMs>0?Math.max(0.05,(atMs-entry.lastRecAtMs)/1000):0.2;
  if(entry.lastRecLineMatrix&&linePosMm>Math.max(150,dtRecSec*2000)&&entry.lastRecAtMs>0&&atMs-entry.lastRecAtMs<250){
    entry.lastRecLineMatrix=pose.slice();
    entry.lastRecFrameMatrix=pose.slice();
    entry.lastRecAtMs=atMs;
    entry.smoothedSpeedMps=0;
    return;
  }
  if(!entry.lastRecLineMatrix||linePosMm>=14){
    let speedMps=0;
    if(entry.lastRecLineMatrix){
      const dtSec=(entry.lastRecAtMs>0&&atMs>entry.lastRecAtMs)?(atMs-entry.lastRecAtMs)/1000:0;
      if(dtSec>=0.008){
        const rawSpeed=Math.min(2.5,(linePosMm/1000)/dtSec);
        entry.smoothedSpeedMps=entry.smoothedSpeedMps>0?(entry.smoothedSpeedMps*0.38+rawSpeed*0.62):rawSpeed;
      }
      speedMps=entry.smoothedSpeedMps;
    }else{
      entry.smoothedSpeedMps=0;
    }
    entry.lastRecLineMatrix=pose.slice();
    entry.lastRecAtMs=atMs;
    appendLinePoint(entry,pose,speedMps,{atMs});
    let framePosMm=Infinity,frameAngDeg=Infinity;
    if(entry.lastRecFrameMatrix){
      const df=rigidVector(product(inverse(entry.lastRecFrameMatrix),pose));
      framePosMm=Math.hypot(...df.slice(3))*1000;
      frameAngDeg=Math.hypot(...df.slice(0,3))*180/Math.PI;
    }
    if(!entry.lastRecFrameMatrix||framePosMm>=48||(framePosMm>=18&&frameAngDeg>=28)){
      entry.lastRecFrameMatrix=pose.slice();
      stampMovableWaypoint(entry.tagId,entry.edgeMm,pose,atMs,{silent:true,skipLine:true,speedMps});
    }
  }
}

function recordHandsInL(frameInputs,world,xrTimeMs){
  if(!extraScanActive||!world||!session||!space||!Number.isFinite(xrTimeMs))return;
  const invWorld=frameInputs.invWorld;
  if(!invWorld)return;
  for(const inp of frameInputs.inputs){
    if(inp.kind!=='hand'||!inp.handedness)continue;
    const getPos=name=>{
      const jp=inp.getJoint(name);
      if(!jp)return null;
      const p=jp.transform.position;
      return worldPointToL(invWorld,p.x,p.y,p.z);
    };
    const jpWrist=inp.getJoint('wrist');
    const wrist=jpWrist?worldPointToL(invWorld,jpWrist.transform.position.x,jpWrist.transform.position.y,jpWrist.transform.position.z):null;
    const wristMatrix=jpWrist?product(invWorld,jpWrist.transform.matrix):null;
    const sample=buildHandSample({
      wrist,
      wristMatrix,
      indexMcp:getPos('index-finger-phalanx-proximal'),
      middleMcp:getPos('middle-finger-phalanx-proximal'),
      pinkyMcp:getPos('pinky-finger-phalanx-proximal'),
      thumbTip:getPos('thumb-tip'),
      indexTip:getPos('index-finger-tip'),
      middleTip:getPos('middle-finger-tip'),
    },xrTimeMs,inp.handedness);
    if(sample)handHistory.push(sample);
  }
}

function updateLiveMovablePreview(tagId,edgeMm,lFromTag,atMs,{confidence=1,targetXrTimeMs=null,decimate=1,apparentEdgePx=0,rmsPx=null,nearHand=false,allowRefine=false,softFallback=false,inFocus=false}={}){
  const conf=Number.isFinite(confidence)?Math.max(0,Math.min(1,confidence)):1;
  let entry=movableTools.get(tagId);
  if(conf<=0){
    // Keep already-visible tag anchored without moving its pose while head is turning fast.
    if(entry?.displayMatrix){entry.lastAt=atMs;entry.lastOpticalAtMs=atMs;}
    return;
  }
  const farMode=Boolean(lastQualityState.longRangeActive);
  if(!entry){
    const minPx=farMode?(softFallback?10:9):12;
    if(conf<0.25||(!farMode&&softFallback)||apparentEdgePx<minPx)return;
    entry=createMovableTool(tagId,edgeMm);
    movableTools.set(tagId,entry);
  }
  const prevOpticalAt=entry.lastOpticalAtMs??0;
  const prevAcceptedOpticalAt=entry.lastAcceptedOpticalAtMs??0;
  const dtSinceLastMs=prevOpticalAt>0?Math.max(0,atMs-prevOpticalAt):Infinity;
  const opticalGapMs=prevAcceptedOpticalAt>0?Math.max(0,atMs-prevAcceptedOpticalAt):0;
  entry.lastAt=atMs;
  entry.lastOpticalAtMs=atMs;
  if(Number.isFinite(targetXrTimeMs))entry.lastOpticalXrTimeMs=targetXrTimeMs;
  entry.lastApparentEdgePx=apparentEdgePx;
  entry.lastOpticalRmsPx=rmsPx;
  entry.targetConfidence=conf;
  if(Number.isFinite(lastXrTime)&&Number.isFinite(targetXrTimeMs)){
    perfTracker.recordOpticalLag(Math.max(0,lastXrTime-targetXrTimeMs));
  }

  // Check 90 Hz hand-tracking coupling first so picking up a locked tag immediately follows the hand at ~0 ms latency.
  let coupRes={mode:'free',event:null};
  if(devHandCouplingEnabled&&Number.isFinite(targetXrTimeMs)){
    coupRes=tagHandCoupler.observeOptical({
      tagId,
      lFromTag,
      targetXrTimeMs,
      confidence:conf,
      handHistory,
    });
    if(coupRes.event==='coupled'){
      pushDiagEvent({
        type:'hand-coupled',tagId,handedness:coupRes.handedness,gripKind:coupRes.gripKind,
        residualMm:coupRes.residualMm,atPerformanceMs:performance.now(),
      });
    }else if(coupRes.event==='released'){
      pushDiagEvent({
        type:'hand-released',tagId,handedness:coupRes.handedness,reason:coupRes.reason,
        atPerformanceMs:performance.now(),
      });
      entry.handCoupled=false;
      entry.coupledHand=null;
      entry.lockState='moving';
      entry.lockedSinceMs=0;
      entry.releaseSpeedMps=Math.max(entry.lastSpeedMps||0,0.35);
      entry.postReleaseBoostUntilMs=atMs+500;
      entry.pendingTrail.discard();
      entry.pendingJump=null;
      entry.skipNextJumpGuardUntilMs=atMs+900;
      if(Number.isFinite(targetXrTimeMs))tagPredictor.observe(tagId,lFromTag,targetXrTimeMs);
      const handoverRefT=entry.displayMatrix?(prevXrTime??lastXrTime):lastXrTime;
      const handoverBase=(coupRes.reason!=='tag-stationary'&&predictDisplayTagPose(tagId,handoverRefT))||lFromTag;
      if(entry.displayMatrix)captureDisplayErrorOffset(entry,handoverBase,coupRes.reason==='tag-stationary'?0.045:0.090);
      else{entry.displayMatrix=handoverBase.slice();resetDisplayErrorOffset(entry);}
      entry.targetMatrix=lFromTag.slice();
      entry.lastMatrix=lFromTag.slice();
      entry.lastOpticalMatrix=lFromTag.slice();
      entry.stillAnchor=lFromTag.slice();
      entry.stillSinceMs=atMs;
      entry.stillSamples=[lFromTag.slice()];
      drawLiveBadge(entry,isEntryRecording(entry)?'rec':entry.lockState,false);
    }
  }

  // While hand-coupled, 90 Hz hand prediction drives displayMatrix directly; keep tagPredictor warm in the shadow for seamless release.
  if(coupRes.mode==='coupled'){
    entry.lastAcceptedOpticalAtMs=atMs;
    if(Number.isFinite(targetXrTimeMs)){
      const vel=tagPredictor.observe(tagId,lFromTag,targetXrTimeMs);
      if(vel&&vel.speedMps>0.01)entry.lastSpeedMps=vel.speedMps;
    }
    resetDisplayErrorOffset(entry);
    if(entry.lockState==='locked'&&!recordAllActive&&!entry.recording&&entry.lockedMatrix){
      seedMovableWake(entry,entry.lockedMatrix,atMs);
    }
    entry.handCoupled=true;
    entry.coupledHand=coupRes.handedness;
    entry.lockState='moving';
    entry.lockedSinceMs=0;
    entry.relocPending=null;
    entry.refineSamples=[];
    entry.stillAnchor=null;entry.stillSinceMs=null;entry.stillSamples=[];
    entry.lastOpticalMatrix=lFromTag.slice();
    if(!entry.displayMatrix)entry.displayMatrix=lFromTag.slice();
    if(recordAllActive&&!entry.recording&&!entry.autoRecording){
      const occludedCouple=Boolean(
        entry.lockedMatrix&&
        opticalGapMs>=TRAIL_GAP_DEFAULTS.minGapMs&&
        Math.hypot(lFromTag[12]-entry.lockedMatrix[12],lFromTag[13]-entry.lockedMatrix[13],lFromTag[14]-entry.lockedMatrix[14])>=TRAIL_GAP_DEFAULTS.minGapDistM
      );
      const startBase=occludedCouple?lFromTag:(entry.lockedMatrix||entry.displayMatrix||lFromTag);
      activateAutoRecording(entry,startBase,atMs,{opticalGapMs});
    }
    if(isEntryRecording(entry)){
      if(coupRes.event==='adapted'){
        const confirmed=entry.pendingTrail.commitUpTo(targetXrTimeMs);
        if(confirmed.length){
          const fused=fuseCommittedTrailSegment(confirmed,lFromTag,{blendWeight:0.45,maxStepCorrectionM:0.035});
          for(let i=0;i<fused.length;i++){
            const pt=fused[i];
            recordMovablePathPoint(entry,pt.pose,Math.max(entry.lastRecAtMs||0,pt.atMs),{opticalGapMs:i===0?opticalGapMs:0});
          }
        }else{
          const blended=(entry.lastRecLineMatrix&&Math.hypot(lFromTag[12]-entry.lastRecLineMatrix[12],lFromTag[13]-entry.lastRecLineMatrix[13],lFromTag[14]-entry.lastRecLineMatrix[14])<=0.045)
            ?blendRigid(entry.lastRecLineMatrix,lFromTag,0.55)
            :lFromTag;
          recordMovablePathPoint(entry,blended,Math.max(entry.lastRecAtMs||0,atMs),{opticalGapMs});
        }
      }else if(coupRes.event==='coupled'){
        entry.pendingTrail.discard();
        const distM=entry.lastRecLineMatrix
          ?Math.hypot(lFromTag[12]-entry.lastRecLineMatrix[12],lFromTag[13]-entry.lastRecLineMatrix[13],lFromTag[14]-entry.lastRecLineMatrix[14])
          :Infinity;
        if(distM>=0.018){
          recordMovablePathPoint(entry,lFromTag,Math.max(entry.lastRecAtMs||0,atMs),{opticalGapMs});
        }else if(entry.pendingTrailBreak){
          entry.pendingTrailBreak=false;
        }
      }
    }else{
      entry.pendingTrail.discard();
    }
    if(conf>=0.30){
      entry.recentPoses.push({atMs,matrix:lFromTag.slice()});
      while(entry.recentPoses.length>5||(entry.recentPoses.length>1&&atMs-entry.recentPoses[0].atMs>1200)){
        entry.recentPoses.shift();
      }
    }
    return;
  }
  if(conf>=0.30)entry.lastOpticalMatrix=lFromTag.slice();

  // --- STATE 1: SETTLING -> auto-anchor after 2-3 consistent frames (posSpread <= 18 mm, angSpread <= 10 deg) ---
  if(entry.lockState==='settling'){
    if(isEntryRecording(entry)&&conf>=0.35){
      entry.lockState='moving';
      entry.lockedSinceMs=0;
      entry.settleSamples=[];
      entry.relocPending=null;
      entry.pendingJump=null;
      entry.stillAnchor=lFromTag.slice();
      entry.stillSinceMs=atMs;
      entry.stillSamples=[lFromTag.slice()];
      resetDisplayErrorOffset(entry);
      drawLiveBadge(entry,'rec',entry.handCoupled);
    }else{
      entry.settleSamples.push({atMs,matrix:lFromTag.slice()});
      entry.settleSamples=entry.settleSamples.filter(s=>atMs-s.atMs<=1500).slice(-3);
      entry.lastMatrix=lFromTag.slice();
      entry.targetMatrix=lFromTag.slice();
      if(!entry.displayMatrix)entry.displayMatrix=lFromTag.slice();
      const needed=(conf<0.75||apparentEdgePx<18||softFallback)?3:2;
      if(entry.settleSamples.length>=needed){
        const mean=meanRigid(entry.settleSamples.map(s=>s.matrix));
        const invM=inverse(mean);
        const maxPos=Math.max(...entry.settleSamples.map(s=>Math.hypot(...rigidVector(product(invM,s.matrix)).slice(3))*1000));
        const maxAng=Math.max(...entry.settleSamples.map(s=>Math.hypot(...rigidVector(product(invM,s.matrix)).slice(0,3))*180/Math.PI));
        if(maxPos<=18&&maxAng<=10){
          entry.lastAcceptedOpticalAtMs=atMs;
          const wasNeverLocked=!entry.hasEverAnnouncedLock;
          entry.hasEverAnnouncedLock=true;
          entry.lockState='locked';
          entry.lockedSinceMs=atMs;
          entry.lockedMatrix=mean.slice();
          entry.lockQuality={decimate,apparentEdgePx,confidence:conf,refineCount:0};
          entry.refineSamples=[];
          entry.displayMatrix=mean.slice();
          entry.targetMatrix=mean.slice();
          entry.lastMatrix=mean.slice();
          resetDisplayErrorOffset(entry);
          entry.settleSamples=[];
          entry.relocPending=null;
          entry.lockFlashUntilMs=atMs+450;
          tagPredictor.reset(tagId);
          syncCurrentPose(entry,mean,atMs);
          if(!isEntryRecording(entry))drawLiveBadge(entry,'locked',false);
          if(wasNeverLocked){
            showHudToast(`✓ LOCKED #${tagId}`,'#4ade80',1300);
            pushDiagEvent({type:'tag-auto-locked',tagId,edgeMm,atPerformanceMs:performance.now(),xMm:mean[12]*1000,yMm:mean[13]*1000,zMm:mean[14]*1000});
          }
          return;
        }
        if(!softFallback&&conf>=0.55&&maxPos>18){
          const startPose=entry.settleSamples[0]?.matrix||lFromTag;
          entry.lockState='moving';
          entry.lockedSinceMs=0;
          entry.settleSamples=[];
          entry.relocPending=null;
          entry.pendingJump=null;
          entry.stillAnchor=lFromTag.slice();
          entry.stillSinceMs=atMs;
          entry.stillSamples=[lFromTag.slice()];
          resetDisplayErrorOffset(entry);
          if(recordAllActive&&!entry.recording){
            activateAutoRecording(entry,startPose,atMs,{opticalGapMs});
            recordMovablePathPoint(entry,lFromTag,atMs,{opticalGapMs:0});
          }else{
            seedMovableWake(entry,startPose,atMs);
            drawLiveBadge(entry,entry.recording?'rec':'moving',entry.handCoupled);
          }
        }else{
          return;
        }
      }else{
        return;
      }
    }
  }

  // --- STATE 2: LOCKED -> rigid when resting, iteratively refines when viewed cleaner/closer still, unlocks cleanly on move ---
  if(entry.lockState==='locked'){
    const ref=entry.lockedMatrix||entry.displayMatrix;
    const d=rigidVector(product(inverse(ref),lFromTag));
    const posMm=Math.hypot(...d.slice(3))*1000;
    const angDeg=Math.hypot(...d.slice(0,3))*180/Math.PI;
    const stillPosMm=(inFocus&&conf>=0.45)?12:18;
    const stillAngDeg=(inFocus&&conf>=0.45)?8:12;

    // While actively recording, prioritize responsiveness immediately on motion.
    if(isEntryRecording(entry)&&(posMm>=8||angDeg>=6)&&conf>=0.40){
      entry.lockState='moving';
      entry.lockedSinceMs=0;
      entry.relocPending=null;
      entry.refineSamples=[];
      entry.pendingJump=null;
      entry.lastMatrix=lFromTag.slice();
      entry.targetMatrix=lFromTag.slice();
      entry.stillAnchor=null;entry.stillSinceMs=null;entry.stillSamples=[];
    }else if(posMm<stillPosMm&&angDeg<stillAngDeg){
      entry.lastAcceptedOpticalAtMs=atMs;
      entry.relocPending=null;
      // Iterative stillness precision refinement when cleaner/closer view confirms a 2..stillPosMm offset.
      const prevQual=entry.lockQuality;
      const qualityMatch=!prevQual||decimate<=prevQual.decimate||apparentEdgePx>=prevQual.apparentEdgePx*0.92;
      if(allowRefine&&!softFallback&&!isEntryRecording(entry)&&conf>=0.72&&qualityMatch&&posMm>=2.0){
        entry.refineSamples.push({atMs,matrix:lFromTag.slice()});
        entry.refineSamples=entry.refineSamples.filter(s=>atMs-s.atMs<=1800).slice(-3);
        if(entry.refineSamples.length>=3){
          const mean=meanRigid(entry.refineSamples.map(s=>s.matrix));
          const invM=inverse(mean);
          const maxPos=Math.max(...entry.refineSamples.map(s=>Math.hypot(...rigidVector(product(invM,s.matrix)).slice(3))*1000));
          const maxAng=Math.max(...entry.refineSamples.map(s=>Math.hypot(...rigidVector(product(invM,s.matrix)).slice(0,3))*180/Math.PI));
          if(maxPos<=8&&maxAng<=5){
            const refined=blendRigid(ref,mean,0.35);
            entry.lockedMatrix=refined.slice();
            entry.displayMatrix=refined.slice();
            entry.targetMatrix=refined.slice();
            entry.lastMatrix=refined.slice();
            resetDisplayErrorOffset(entry);
            entry.lockQuality={decimate,apparentEdgePx,confidence:conf,refineCount:(prevQual?.refineCount??0)+1};
            entry.refineSamples=[];
            syncCurrentPose(entry,refined,atMs);
          }
        }
      }else if(posMm<1.8){
        entry.refineSamples=[];
      }
      return;
    }else{
      if(conf<0.35){
        entry.relocPending=null;
        return;
      }
      const staleLocked=dtSinceLastMs>=700;
      const unlockMaxPos=inFocus?320:180;
      const cleanFastUnlock=
        (nearHand&&posMm>=stillPosMm&&posMm<=unlockMaxPos&&conf>=0.45&&(!softFallback||(Number.isFinite(rmsPx)&&rmsPx<=1.6)))||
        (inFocus&&posMm>=stillPosMm&&posMm<=unlockMaxPos&&conf>=0.45&&(!softFallback||(Number.isFinite(rmsPx)&&rmsPx<=2.2)))||
        (!softFallback&&Number.isFinite(rmsPx)&&rmsPx<=1.25&&posMm>=18&&posMm<=180&&conf>=0.65)||
        (staleLocked&&!softFallback&&Number.isFinite(rmsPx)&&rmsPx<=1.10&&apparentEdgePx>=14&&conf>=0.65);
      const mirrorFlipLikely=posMm<24&&angDeg>20;
      if(cleanFastUnlock&&!mirrorFlipLikely){
        entry.lastAcceptedOpticalAtMs=atMs;
        entry.lockState='moving';
        entry.lockedSinceMs=0;
        entry.relocPending=null;
        entry.refineSamples=[];
        entry.pendingJump=null;
        entry.lastMatrix=lFromTag.slice();
        entry.targetMatrix=lFromTag.slice();
        if(staleLocked&&posMm>180){
          entry.displayMatrix=lFromTag.slice();
          resetDisplayErrorOffset(entry);
        }
        entry.stillAnchor=lFromTag.slice();
        entry.stillSinceMs=atMs;
        entry.stillSamples=[lFromTag.slice()];
        if(Number.isFinite(targetXrTimeMs))tagPredictor.observe(tagId,lFromTag,targetXrTimeMs);
        const unlockRefT=entry.displayMatrix?(prevXrTime??lastXrTime):lastXrTime;
        const predTarget=predictDisplayTagPose(tagId,unlockRefT)||lFromTag;
        if(entry.displayMatrix)captureDisplayErrorOffset(entry,predTarget,0.055);
        const occludedReloc=opticalGapMs>=TRAIL_GAP_DEFAULTS.minGapMs&&posMm>=TRAIL_GAP_DEFAULTS.minGapDistM*1000;
        if(recordAllActive&&!entry.recording){
          if(occludedReloc){
            activateAutoRecording(entry,lFromTag,atMs,{opticalGapMs});
          }else{
            activateAutoRecording(entry,ref,atMs,{opticalGapMs:0});
            recordMovablePathPoint(entry,lFromTag,atMs,{opticalGapMs:0});
          }
        }else{
          if(!occludedReloc)seedMovableWake(entry,ref,atMs);
          if(!entry.recording)drawLiveBadge(entry,'moving',entry.handCoupled);
        }
        return;
      }
      // Single-frame outlier / IPPE mirror-flip guard: require a 2nd consecutive frame within velocity-scaled tolerance of relocPending.
      if(entry.relocPending&&atMs-entry.relocPending.atMs<=1500){
        const dp=rigidVector(product(inverse(entry.relocPending.matrix),lFromTag));
        const pPosMm=Math.hypot(...dp.slice(3))*1000;
        const pAngDeg=Math.hypot(...dp.slice(0,3))*180/Math.PI;
        const relocDtSec=Math.min(0.65,Math.max(0.05,(atMs-entry.relocPending.atMs)/1000));
        if(pPosMm<=Math.max(28,relocDtSec*1400)&&pAngDeg<=Math.max(15,relocDtSec*160)){
          entry.lastAcceptedOpticalAtMs=atMs;
          entry.lockState='moving';
          entry.lockedSinceMs=0;
          entry.relocPending=null;
          entry.refineSamples=[];
          entry.pendingJump=null;
          entry.lastMatrix=lFromTag.slice();
          entry.targetMatrix=lFromTag.slice();
          entry.stillAnchor=lFromTag.slice();
          entry.stillSinceMs=atMs;
          entry.stillSamples=[lFromTag.slice()];
          if(Number.isFinite(targetXrTimeMs))tagPredictor.observe(tagId,lFromTag,targetXrTimeMs);
          const relocRefT=entry.displayMatrix?(prevXrTime??lastXrTime):lastXrTime;
          const predTarget=predictDisplayTagPose(tagId,relocRefT)||lFromTag;
          if(entry.displayMatrix)captureDisplayErrorOffset(entry,predTarget,0.055);
          const occludedReloc=opticalGapMs>=TRAIL_GAP_DEFAULTS.minGapMs&&posMm>=TRAIL_GAP_DEFAULTS.minGapDistM*1000;
          if(recordAllActive&&!entry.recording){
            if(occludedReloc){
              activateAutoRecording(entry,lFromTag,atMs,{opticalGapMs});
            }else{
              activateAutoRecording(entry,ref,atMs,{opticalGapMs:0});
              recordMovablePathPoint(entry,lFromTag,atMs,{opticalGapMs:0});
            }
          }else{
            if(!occludedReloc)seedMovableWake(entry,ref,atMs);
            if(!entry.recording)drawLiveBadge(entry,'moving',entry.handCoupled);
          }
        }else{
          entry.relocPending={atMs,matrix:lFromTag.slice()};
          return;
        }
      }else{
        entry.relocPending={atMs,matrix:lFromTag.slice()};
        return;
      }
    }
  }

  // --- STATE 3: MOVING -> 90 Hz error-decayed predictive follow + velocity-scaled spike/flip guard + ~450 ms stillness re-lock ---
  if(!entry.handCoupled&&conf<0.30)return;
  const skipJumpGuard=entry.skipNextJumpGuardUntilMs>0&&atMs<=entry.skipNextJumpGuardUntilMs;
  if(entry.lastMatrix&&!entry.handCoupled&&!skipJumpGuard){
    const guard=evaluateCarriedJumpGuard({
      prevMatrix:entry.lastMatrix,
      nextMatrix:lFromTag,
      dtSinceLastMs,
      confidence:conf,
      pendingJump:entry.pendingJump,
      atMs,
    });
    entry.pendingJump=guard.nextPendingJump;
    if(!guard.accept)return;
  }else{
    entry.pendingJump=null;
  }
  entry.skipNextJumpGuardUntilMs=0;
  entry.lastAcceptedOpticalAtMs=atMs;

  const nextPose=(conf>=0.35||!entry.targetMatrix)?lFromTag.slice():blendRigid(entry.targetMatrix,lFromTag,conf);
  entry.lastMatrix=nextPose.slice();
  entry.targetMatrix=nextPose.slice();
  if(!entry.handCoupled&&Number.isFinite(targetXrTimeMs)){
    const vel=tagPredictor.observe(tagId,lFromTag,targetXrTimeMs);
    if(vel){
      entry.lastSpeedMps=vel.speedMps;
      if(vel.speedMps>0.01)perfTracker.recordTagSpeed(vel.speedMps,false);
    }
  }
  if(!devDisplayPredEnabled){
    entry.displayMatrix=nextPose.slice();
    resetDisplayErrorOffset(entry);
  }else{
    const dispRefT=entry.displayMatrix?(prevXrTime??lastXrTime):lastXrTime;
    const newPred=predictDisplayTagPose(tagId,dispRefT)||nextPose;
    if(entry.displayMatrix)captureDisplayErrorOffset(entry,newPred,0.048);
    else{entry.displayMatrix=newPred.slice();resetDisplayErrorOffset(entry);}
  }

  // Rolling pre-touch pose ring buffer.
  if(conf>=0.30){
    entry.recentPoses.push({atMs,matrix:nextPose.slice()});
    while(entry.recentPoses.length>5||(entry.recentPoses.length>1&&atMs-entry.recentPoses[0].atMs>1200)){
      entry.recentPoses.shift();
    }
    recordMovablePathPoint(entry,nextPose,atMs,{opticalGapMs});
  }

  // Stillness check: once tag rests within < 10 mm / < 6 deg for >= 450 ms (2+ frames), re-lock cleanly into LOCKED (never auto-lock while recording).
  if(!entry.recording&&conf>=0.45){
    if(!entry.stillAnchor){
      entry.stillAnchor=nextPose.slice();
      entry.stillSinceMs=atMs;
      entry.stillSamples=[nextPose.slice()];
    }else{
      const ds=rigidVector(product(inverse(entry.stillAnchor),nextPose));
      const sPosMm=Math.hypot(...ds.slice(3))*1000;
      const sAngDeg=Math.hypot(...ds.slice(0,3))*180/Math.PI;
      if(sPosMm<10&&sAngDeg<6){
        entry.stillSamples.push(nextPose.slice());
        if(entry.stillSamples.length>4)entry.stillSamples.shift();
        if(entry.stillSamples.length>=2&&atMs-entry.stillSinceMs>=450){
          const locked=meanRigid(entry.stillSamples);
          entry.lockState='locked';
          entry.lockedSinceMs=atMs;
          entry.lockedMatrix=locked.slice();
          entry.lockQuality={decimate,apparentEdgePx,confidence:conf,refineCount:0};
          entry.refineSamples=[];
          entry.displayMatrix=locked.slice();
          entry.targetMatrix=locked.slice();
          entry.lastMatrix=locked.slice();
          resetDisplayErrorOffset(entry);
          entry.stillAnchor=null;entry.stillSinceMs=null;entry.stillSamples=[];
          entry.lockFlashUntilMs=atMs+400;
          tagPredictor.reset(tagId);
          syncCurrentPose(entry,locked,atMs);
          if(entry.autoRecording){
            deactivateAutoRecording(entry,locked,atMs);
          }else{
            drawLiveBadge(entry,'locked',false);
          }
        }
      }else{
        entry.stillAnchor=nextPose.slice();
        entry.stillSinceMs=atMs;
        entry.stillSamples=[nextPose.slice()];
      }
    }
  }
}

function getPreTouchPose(entry,now){
  if(entry.lockState==='locked'&&entry.lockedMatrix)return entry.lockedMatrix.slice();
  const fresh=entry.recentPoses.filter(p=>now-p.atMs<=1200);
  if(!fresh.length)return entry.displayMatrix||entry.lastMatrix;
  const preOccl=fresh.filter(p=>now-p.atMs>=80);
  const pool=(preOccl.length>=2?preOccl:fresh).slice(-3);
  if(pool.length===1)return pool[0].matrix.slice();
  const ref=pool[pool.length-1].matrix;
  const invRef=inverse(ref);
  const consistent=pool.filter(p=>{
    const d=rigidVector(product(invRef,p.matrix));
    return Math.hypot(...d.slice(3))*1000<=28&&Math.hypot(...d.slice(0,3))*180/Math.PI<=12;
  });
  return consistent.length>=2?meanRigid(consistent.map(p=>p.matrix)):ref.slice();
}

function seedTrailRecordingStart(entry,startPose,now,{explicitBreak=false,opticalGapMs=null}={}){
  if(!startPose)return;
  if(entry.lineCount>0&&entry.lastCommittedPose){
    const gap=shouldStartNewTrailSegment({
      prevPose:entry.lastCommittedPose,
      nextPose:startPose,
      dtMs:opticalGapMs,
      explicitBreak:explicitBreak||entry.pendingTrailBreak,
    });
    entry.pendingTrailBreak=false;
    if(gap.isBreak){
      entry.lastRecLineMatrix=startPose.slice();
      entry.lastRecFrameMatrix=startPose.slice();
      entry.lastRecAtMs=now;
      stampMovableWaypoint(entry.tagId,entry.edgeMm,startPose,now,{silent:true,isBreak:true,isSegmentStart:true,speedMps:0});
      pushDiagEvent({
        type:'trail-gap',tagId:entry.tagId,reason:gap.reason,
        distMm:Math.round(gap.distM*1000),gapMs:Math.round(gap.dtMs||0),
        segment:entry.trailSegmentCount,atPerformanceMs:performance.now(),
      });
      return;
    }
    if(gap.distM>=0.014){
      entry.lastRecLineMatrix=startPose.slice();
      entry.lastRecFrameMatrix=startPose.slice();
      entry.lastRecAtMs=now;
      const lastWp=entry.waypoints.at(-1);
      if(!lastWp||now-lastWp.atMs>750){
        stampMovableWaypoint(entry.tagId,entry.edgeMm,startPose,now,{silent:true});
      }else{
        appendLinePoint(entry,startPose,0,{atMs:now});
      }
    }else{
      entry.lastRecLineMatrix=entry.lastCommittedPose.slice();
      entry.lastRecAtMs=now;
    }
    return;
  }
  entry.pendingTrailBreak=false;
  const prevDist=entry.lastRecLineMatrix
    ?Math.hypot(startPose[12]-entry.lastRecLineMatrix[12],startPose[13]-entry.lastRecLineMatrix[13],startPose[14]-entry.lastRecLineMatrix[14])
    :Infinity;
  if(!entry.lastRecLineMatrix||prevDist>=0.014){
    entry.lastRecLineMatrix=startPose.slice();
    entry.lastRecFrameMatrix=startPose.slice();
    entry.lastRecAtMs=now;
    const lastWp=entry.waypoints.at(-1);
    if(!lastWp||now-lastWp.atMs>750){
      stampMovableWaypoint(entry.tagId,entry.edgeMm,startPose,now,{silent:true});
    }else{
      appendLinePoint(entry,startPose,0,{atMs:now});
    }
  }else{
    entry.lastRecAtMs=now;
  }
}

function setTagRecording(entry,active,now){
  if(entry.recording===active)return;
  entry.recording=active;
  if(active){
    entry.autoRecording=false;
    clearMovableWake(entry);
  }
  entry.pendingTrail.discard();
  entry.frameColorHex=active?0xef4444:entry.theme.hex;
  frameBatch.setColor(entry.frameSlot,entry.frameColorHex);
  drawLiveBadge(entry,active?'rec':entry.lockState,entry.handCoupled);
  drawRecOrbPill(entry,active,false);
  entry.recOrbColorHex=active?0xef4444:0x0ea5e9;
  recOrbBatch.setColor(entry.recOrbSlot,entry.recOrbColorHex);
  if(active){
    entry.smoothedSpeedMps=0;
    const startPose=(entry.handCoupled&&entry.lastOpticalMatrix)?entry.lastOpticalMatrix.slice():getPreTouchPose(entry,now);
    if(startPose){
      seedTrailRecordingStart(entry,startPose,now,{explicitBreak:true});
      const flashPos=root.visible?lPointToWorld(root.matrix.elements,startPose[12],startPose[13],startPose[14]):null;
      if(flashPos)triggerPinFlash(flashPos,'#ef4444','●');
    }
    playModeTick('scan');
    showHudToast(`● RECORDING #${entry.tagId} PATH`,'#ef4444',1800);
  }else{
    entry.smoothedSpeedMps=0;
    entry.lastRecLineMatrix=null;
    entry.lastRecFrameMatrix=null;
    entry.lastRecAtMs=0;
    if(entry.lineCount>0)entry.pendingTrailBreak=true;
    if(entry.displayMatrix){
      const flashPos=root.visible?lPointToWorld(root.matrix.elements,entry.displayMatrix[12],entry.displayMatrix[13],entry.displayMatrix[14]):null;
      if(flashPos)triggerPinFlash(flashPos,'#22c55e','■');
    }
    playModeTick('measure');
    showHudToast(`■ STOPPED #${entry.tagId} (${entry.waypoints.length} PTS)`,'#22c55e',1600);
  }
}

function activateAutoRecording(entry,startPose,now,{opticalGapMs=null}={}){
  if(!entry||entry.recording||entry.autoRecording)return;
  entry.autoRecording=true;
  clearMovableWake(entry);
  entry.pendingTrail.discard();
  entry.smoothedSpeedMps=0;
  entry.frameColorHex=0xef4444;
  frameBatch.setColor(entry.frameSlot,entry.frameColorHex);
  drawLiveBadge(entry,'rec',entry.handCoupled);
  seedTrailRecordingStart(entry,startPose,now,{opticalGapMs});
}

function deactivateAutoRecording(entry,finalPose,now){
  if(!entry||!entry.autoRecording)return;
  if(finalPose&&entry.lastRecLineMatrix){
    const distMm=Math.hypot(
      finalPose[12]-entry.lastRecLineMatrix[12],
      finalPose[13]-entry.lastRecLineMatrix[13],
      finalPose[14]-entry.lastRecLineMatrix[14],
    )*1000;
    if(distMm>=6&&distMm<=150){
      appendLinePoint(entry,finalPose,0,{atMs:now});
      entry.lastRecLineMatrix=finalPose.slice();
    }
  }
  entry.autoRecording=false;
  entry.pendingTrail.discard();
  entry.smoothedSpeedMps=0;
  entry.lastRecAtMs=0;
  if(!entry.recording){
    entry.frameColorHex=entry.theme.hex;
    frameBatch.setColor(entry.frameSlot,entry.frameColorHex);
    drawLiveBadge(entry,!extraScanActive?'measure':entry.lockState,entry.handCoupled);
  }
}

async function setRecordAllActive(active,{silent=false}={}){
  const next=Boolean(active);
  if(next&&(anchor.phase()!=='tracking'&&anchor.phase()!=='created'))return;
  if(next&&!extraScanActive){
    await toggleExtraScan();
    if(!extraScanActive)return;
  }
  const now=performance.now();
  if(recordAllActive===next){
    if(next&&!silent)showHudToast('● REC ALL ALREADY ARMED','#ef4444',1400);
    return;
  }
  recordAllActive=next;
  lastModeRecAllKey='';
  if(next){
    let activeMoves=0;
    for(const e of movableTools.values()){
      if(!e.visible||e.recording)continue;
      if(e.handCoupled||e.lockState==='moving'){
        const startPose=e.displayMatrix||e.lastOpticalMatrix||e.lastMatrix;
        activateAutoRecording(e,startPose,now);
        activeMoves++;
      }else{
        drawLiveBadge(e,e.lockState,e.handCoupled);
      }
    }
    if(!silent){
      playModeTick('scan');
      showHudToast(activeMoves>0?`● REC ALL ON (${activeMoves} MOVING)`:'● REC ALL ARMED (RUHEZONE)','#ef4444',1800);
    }
    pushDiagEvent({type:'rec-all-enabled',activeMoves,atPerformanceMs:now});
  }else{
    for(const e of movableTools.values()){
      if(e.autoRecording){
        const finalPose=e.displayMatrix||e.lockedMatrix||e.lastOpticalMatrix;
        deactivateAutoRecording(e,finalPose,now);
      }
      if(!e.recording){
        e.lastRecLineMatrix=null;
        e.lastRecFrameMatrix=null;
        if(e.lineCount>0)e.pendingTrailBreak=true;
      }
      if(e.visible){
        drawLiveBadge(e,!extraScanActive?'measure':(e.recording?'rec':e.lockState),e.handCoupled);
      }
    }
    if(!silent){
      playModeTick('measure');
      showHudToast('■ REC ALL OFF','#22c55e',1500);
    }
    pushDiagEvent({type:'rec-all-disabled',atPerformanceMs:now});
  }
}

function toggleRecordAll(){
  return setRecordAllActive(!recordAllActive);
}

function syncMovablePerimeterAlert(entry,poseMat,visible,now){
  const fenceActive=Boolean(perimeterFence?.isActive?.());
  if(!visible||!poseMat||!fenceActive){
    entry.hasFenceSample=false;
    entry.lastInsidePosL=null;
    if(entry.outsidePerimeter){
      entry.outsidePerimeter=false;
      frameBatch.setAlert(entry.frameSlot,false);
      alertGlowBatch.setVisible(entry.alertSlot,false);
      if(visible){
        drawLiveBadge(entry,!extraScanActive?'measure':(isEntryRecording(entry)?'rec':entry.lockState),entry.handCoupled);
      }
    }else{
      alertGlowBatch.setVisible(entry.alertSlot,false);
    }
    return;
  }
  _alertPosBuf[0]=poseMat[12];
  _alertPosBuf[1]=poseMat[13];
  _alertPosBuf[2]=poseMat[14];
  const res=perimeterFence.isOutsideXY(_alertPosBuf,{wasOutside:Boolean(entry.outsidePerimeter)});
  if(res.outside!==Boolean(entry.outsidePerimeter)){
    let cross=null;
    if(entry.hasFenceSample&&extraScanActive&&now-entry.lastBreachAtMs>=280){
      const prevRef=res.outside?(entry.lastInsidePosL??entry.lastFencePosL):entry.lastFencePosL;
      cross=perimeterFence.locateCrossing(prevRef,_alertPosBuf);
      if(cross){
        const spd=Number.isFinite(entry.lastSpeedMps)?entry.lastSpeedMps:0.25;
        const intensity=Math.max(0.85,Math.min(1.35,0.85+spd*0.65));
        triggerFenceBreach({
          tagId:entry.tagId,
          arcS:cross.arcS,
          z:cross.pos[2],
          dir:res.outside?1:-1,
          intensity,
          overTop:cross.overTop,
        },now);
        entry.lastBreachAtMs=now;
      }
    }
    entry.outsidePerimeter=res.outside;
    frameBatch.setAlert(entry.frameSlot,res.outside);
    drawLiveBadge(entry,!extraScanActive?'measure':(isEntryRecording(entry)?'rec':entry.lockState),entry.handCoupled);
    if(res.outside&&extraScanActive){
      pushDiagEvent({
        type:'perimeter-tag-outside',
        tagId:entry.tagId,
        edgeDistanceMm:Math.round(res.edgeDistanceM*1000),
        breachPosMm:cross?cross.pos.map(v=>Math.round(v*1000)):null,
        overTop:cross?cross.overTop:false,
        atPerformanceMs:now,
      });
    }
  }
  if(extraScanActive&&!res.outside&&res.signedDistanceM<=0&&res.edgeDistanceM<0.070){
    const near=perimeterFence.locateCrossing(null,_alertPosBuf);
    if(near){
      const t=Math.max(0,Math.min(1,(0.070-res.edgeDistanceM)/(0.070-0.004)));
      const strength=t*t*(3-2*t);
      recordFenceProximity(near.arcS,near.pos[2],strength,near.overTop);
    }
  }
  entry.lastFencePosL[0]=_alertPosBuf[0];
  entry.lastFencePosL[1]=_alertPosBuf[1];
  entry.lastFencePosL[2]=_alertPosBuf[2];
  if(res.signedDistanceM<=0){
    if(!entry.lastInsidePosL)entry.lastInsidePosL=[0,0,0];
    entry.lastInsidePosL[0]=_alertPosBuf[0];
    entry.lastInsidePosL[1]=_alertPosBuf[1];
    entry.lastInsidePosL[2]=_alertPosBuf[2];
  }
  entry.hasFenceSample=true;
  if(entry.outsidePerimeter){
    setTagFramePose(alertGlowBatch,entry.alertSlot,poseMat,0.0008,entry.edgeMm/20);
    alertGlowBatch.setVisible(entry.alertSlot,true);
  }else{
    alertGlowBatch.setVisible(entry.alertSlot,false);
  }
}

function stepLiveTagSmoothing(now,dtSec,xrTimeMs=null){
  const dt=Math.min(0.05,Math.max(0.004,dtSec||1/90));
  const curFenceVer=perimeterFence?.version?.()??-1;
  beginFenceProximityFrame();
  for(const entry of movableTools.values()){
    if(entry.trailTube&&entry.lineCount>0&&entry.lastTrailFenceVer!==curFenceVer){
      entry.lastTrailFenceVer=curFenceVer;
      syncMovableTrailOutsideFlags(entry);
    }
    if(!extraScanActive){
      // MEASURE mode (90 FPS): keep every locked tag anchored & visible at its static currentPose so it can be hovered/pinned immediately.
      const needsModeSync=!entry.measureModeApplied||entry.measureSyncedPose!==entry.currentPose;
      if(needsModeSync){
        entry.handCoupled=false;entry.coupledHand=null;
        clearMovableWake(entry);
        if(entry.currentPose){
          entry.visible=true;
          setTagFramePose(frameBatch,entry.frameSlot,entry.currentPose.matrix,0,entry.edgeMm/20);
          if(entry.frameColorHex!==entry.theme.hex){
            entry.frameColorHex=entry.theme.hex;
            frameBatch.setColor(entry.frameSlot,entry.theme.hex);
          }
          frameBatch.setVisible(entry.frameSlot,true);
          axesBatch.setPose(entry.axesSlot,entry.currentPose.matrix,0.055);
          axesBatch.setVisible(entry.axesSlot,true);
          drawLiveBadge(entry,'measure',false);
          const faceEdge=Math.max(0.008,entry.edgeMm/1000-0.0006);
          entry.faceVisible=true;
          tagFaceMesh.setOrientedQuad(entry.faceSlot,entry.currentPose.matrix,faceEdge,faceEdge,0,true);
        }else{
          entry.visible=false;
          entry.faceVisible=false;
          frameBatch.setVisible(entry.frameSlot,false);
          axesBatch.setVisible(entry.axesSlot,false);
          tagFaceMesh.setQuadVisible(entry.faceSlot,false);
        }
        entry.recOrbVisible=false;
        recOrbBatch.setVisible(entry.recOrbSlot,false);
        badgeMesh.setQuadVisible(entry.recPillSlot,false);
        entry.markOrbVisible=false;
        recOrbBatch.setVisible(entry.markOrbSlot,false);
        badgeMesh.setQuadVisible(entry.markPillSlot,false);
        entry.markHoldSince=null;entry.markHoldConsumed=false;
        entry.recHoldSince=null;entry.recHoldConsumed=false;
        entry.measureSyncedPose=entry.currentPose;
        entry.measureModeApplied=true;
      }
      if(needsModeSync||entry.lastAlertFenceVer!==curFenceVer){
        entry.lastAlertFenceVer=curFenceVer;
        syncMovablePerimeterAlert(entry,entry.currentPose?.matrix??null,Boolean(entry.currentPose),now);
      }
      continue;
    }
    entry.measureModeApplied=false;

    // SCAN mode: check 90 Hz hand coupling prediction first.
    const pred=(devHandCouplingEnabled&&Number.isFinite(xrTimeMs))?tagHandCoupler.predict(entry.tagId,handHistory,xrTimeMs):null;
    if(pred?.released){
      pushDiagEvent({
        type:'hand-released',tagId:entry.tagId,handedness:pred.prevHand,reason:pred.reason,
        atPerformanceMs:performance.now(),
      });
      entry.lockState='moving';
      entry.lockedSinceMs=0;
      entry.releaseSpeedMps=Math.max(entry.lastSpeedMps||0,0.35);
      entry.postReleaseBoostUntilMs=now+500;
      entry.pendingTrail.discard();
      entry.pendingJump=null;
      entry.skipNextJumpGuardUntilMs=now+900;
      if(pred.reason==='optical-timeout'||pred.reason==='unconfirmed-drift'){
        if(entry.lineCount>0)entry.pendingTrailBreak=true;
      }
      const smoothRelease=pred.reason==='pinch-opened'||pred.reason==='hand-pulled-away'||pred.reason==='optical-disagreement';
      if(smoothRelease){
        const relRefT=entry.displayMatrix?(prevXrTime??xrTimeMs):xrTimeMs;
        const handoverBase=predictDisplayTagPose(entry.tagId,relRefT)||entry.targetMatrix||entry.displayMatrix;
        if(handoverBase){
          if(entry.displayMatrix)captureDisplayErrorOffset(entry,handoverBase,0.095);
          else{entry.displayMatrix=handoverBase.slice();resetDisplayErrorOffset(entry);}
          entry.targetMatrix=handoverBase.slice();
          entry.lastMatrix=handoverBase.slice();
          entry.stillAnchor=handoverBase.slice();
          entry.stillSinceMs=now;
          entry.stillSamples=[handoverBase.slice()];
        }
      }else{
        const snapPose=pred.lastOpticalPose||entry.lastOpticalMatrix||entry.lockedMatrix||entry.displayMatrix;
        tagPredictor.reset(entry.tagId);
        resetDisplayErrorOffset(entry);
        // Snap back to grounded optical pose on timeout/watchdog release so tags never float off in mid-air.
        if(snapPose){
          entry.displayMatrix=snapPose.slice();
          entry.targetMatrix=snapPose.slice();
          entry.lastMatrix=snapPose.slice();
          entry.stillAnchor=snapPose.slice();
          entry.stillSinceMs=now;
          entry.stillSamples=[snapPose.slice()];
        }
      }
    }
    const wasCoupled=entry.handCoupled;
    const isCoupled=Boolean(pred?.coupled&&pred.lFromTag);
    entry.handCoupled=isCoupled;
    entry.coupledHand=isCoupled?pred.handedness:null;
    if(!isCoupled)entry.coupledStepCount=0;
    if(isCoupled){
      const coupledPose=(devHandLagMs>0&&Number.isFinite(xrTimeMs))
        ?(tagHandCoupler.predictAt(entry.tagId,handHistory,xrTimeMs-devHandLagMs)||pred.lFromTag)
        :pred.lFromTag;
      if(entry.lockState==='locked'&&!recordAllActive&&!entry.recording&&entry.lockedMatrix){
        seedMovableWake(entry,entry.lockedMatrix,now);
      }
      entry.lockState='moving';
      entry.lastAt=now;
      const dtGap=entry.lastCoupledStepAtMs>0?(now-entry.lastCoupledStepAtMs):0;
      const jumpM=entry.displayMatrix?Math.hypot(
        coupledPose[12]-entry.displayMatrix[12],
        coupledPose[13]-entry.displayMatrix[13],
        coupledPose[14]-entry.displayMatrix[14],
      ):0;
      if(!entry.displayMatrix||!wasCoupled||dtGap>280||jumpM>0.065){
        entry.displayMatrix=coupledPose.slice();
        entry.coupledStepCount=1;
      }else{
        // Ultra-snappy 90 Hz hand following (~0 ms trailing while suppressing sub-mm jitter).
        const alpha=1-Math.exp(-dt*95);
        entry.displayMatrix=blendRigid(entry.displayMatrix,coupledPose,alpha);
        entry.coupledStepCount=(entry.coupledStepCount||0)+1;
        if(dt>0.004&&jumpM<0.065){
          const rawSpd=Math.min(2.5,jumpM/dt);
          entry.lastSpeedMps=entry.coupledStepCount<=2?rawSpd:(entry.lastSpeedMps*0.65+rawSpd*0.35);
          if(entry.coupledStepCount>=3)perfTracker.recordTagSpeed(entry.lastSpeedMps,true);
        }
      }
      entry.lastCoupledStepAtMs=now;
      entry.targetMatrix=entry.displayMatrix.slice();
      if(recordAllActive&&!entry.recording&&!entry.autoRecording){
        activateAutoRecording(entry,entry.lockedMatrix||entry.displayMatrix||coupledPose,now);
      }
      if(isEntryRecording(entry)&&pred.opticalAgeMs<=450&&Number.isFinite(xrTimeMs)){
        entry.pendingTrail.push({xrTimeMs,atMs:now,pose:pred.lFromTag});
      }else if(!isEntryRecording(entry)){
        entry.pendingTrail.discard();
      }
    }

    // Live SCAN visuals render ONLY while physically tracked (optically fresh or actively hand-coupled) — no "last seen" ghosts in mid-air.
    const liveDropoutMs=isEntryRecording(entry)?1200:950;
    const opticallyFresh=entry.lastOpticalAtMs>0&&(now-entry.lastOpticalAtMs)<liveDropoutMs;
    const alive=Boolean(entry.displayMatrix)&&(isCoupled||opticallyFresh);
    entry.visible=alive;
    entry.faceVisible=alive;
    if(!alive){
      if(entry.autoRecording){
        deactivateAutoRecording(entry,entry.lastOpticalMatrix||entry.displayMatrix,now);
        entry.lastRecLineMatrix=null;
        entry.lastRecFrameMatrix=null;
      }
      if(entry.lineCount>0){
        entry.pendingTrailBreak=true;
        if(entry.recording){
          entry.lastRecLineMatrix=null;
          entry.lastRecFrameMatrix=null;
          entry.lastRecAtMs=0;
          entry.smoothedSpeedMps=0;
        }
      }
      clearMovableWake(entry);
      syncMovablePerimeterAlert(entry,null,false,now);
      entry.lastAlertFenceVer=curFenceVer;
      frameBatch.setVisible(entry.frameSlot,false);
      axesBatch.setVisible(entry.axesSlot,false);
      tagFaceMesh.setQuadVisible(entry.faceSlot,false);
      entry.recOrbVisible=false;
      recOrbBatch.setVisible(entry.recOrbSlot,false);
      badgeMesh.setQuadVisible(entry.recPillSlot,false);
      entry.markOrbVisible=false;
      recOrbBatch.setVisible(entry.markOrbSlot,false);
      badgeMesh.setQuadVisible(entry.markPillSlot,false);
      entry.markHoldSince=null;entry.markHoldConsumed=false;
      entry.recHoldSince=null;entry.recHoldConsumed=false;
      if(entry.lockState==='moving')tagPredictor.noteLoss(entry.tagId);
      if(entry.lockState!=='settling'||entry.lockedMatrix||entry.displayMatrix||entry.currentPose){
        tagPredictor.reset(entry.tagId);
        resetDisplayErrorOffset(entry);
        entry.lockState='settling';
        entry.lockedSinceMs=0;
        entry.lastOpticalXrTimeMs=0;
        entry.lockedMatrix=null;
        entry.displayMatrix=null;
        entry.targetMatrix=null;
        entry.lastMatrix=null;
        entry.settleSamples=[];
        entry.refineSamples=[];
        entry.relocPending=null;
        entry.pendingJump=null;
        entry.stillAnchor=null;
        entry.stillSinceMs=null;
        entry.stillSamples=[];
        entry.currentPose=null;
        drawLiveBadge(entry,isEntryRecording(entry)?'rec':'settling',false);
      }
      continue;
    }
    if(wasCoupled!==isCoupled){
      drawLiveBadge(entry,isEntryRecording(entry)?'rec':entry.lockState,isCoupled);
    }

    if(!isCoupled){
      if(entry.lockState==='locked'&&entry.lockedMatrix){
        // 100% rigid — zero wobble, zero drift.
        entry.displayMatrix=entry.lockedMatrix;
      }else if(entry.lockState==='moving'){
        if(!devDisplayPredEnabled){
          if(entry.targetMatrix)entry.displayMatrix=entry.targetMatrix.slice();
        }else{
          const predTarget=predictDisplayTagPose(entry.tagId,xrTimeMs)||entry.targetMatrix;
          if(predTarget){
            entry.displayMatrix=entry.displayMatrix?applyAndDecayErrorOffset(entry,predTarget,dt):predTarget.slice();
          }
        }
      }else{
        const predTarget=entry.targetMatrix;
        if(predTarget){
          const alpha=1-Math.exp(-dt*24.0);
          entry.displayMatrix=entry.displayMatrix?blendRigid(entry.displayMatrix,predTarget,alpha):predTarget.slice();
        }
      }
    }

    // Latency-offset fade: only optically-driven (moving, uncoupled) tags can lag; everything else stays opaque.
    const trackFade=(!isCoupled&&entry.lockState==='moving'&&Number.isFinite(xrTimeMs))?tagPredictor.trackingFade(entry.tagId,xrTimeMs,lastViewerBasis?.viewerPosL):1;
    setTagFramePose(frameBatch,entry.frameSlot,entry.displayMatrix,0,entry.edgeMm/20);
    frameBatch.setVisible(entry.frameSlot,true);
    frameBatch.setFade(entry.frameSlot,trackFade);
    axesBatch.setPose(entry.axesSlot,entry.displayMatrix,0.055);
    axesBatch.setVisible(entry.axesSlot,trackFade>0.6);
    const faceEdge=Math.max(0.008,entry.edgeMm/1000-0.0006);
    tagFaceMesh.setOrientedQuad(entry.faceSlot,entry.displayMatrix,faceEdge,faceEdge,0,true);
    tagFaceMesh.setQuadAlpha(entry.faceSlot,trackFade);
    syncMovablePerimeterAlert(entry,entry.displayMatrix,true,now);
    entry.lastAlertFenceVer=curFenceVer;
    updateMovableWake(entry,now,{
      moving:isCoupled||entry.lockState==='moving',
      pose:entry.displayMatrix,
      speedMps:entry.lastSpeedMps,
    });

    // Brief green confirmation flash right after locking into place.
    const flashing=now<entry.lockFlashUntilMs;
    const desiredHex=isEntryRecording(entry)?0xef4444:(flashing?0x4ade80:entry.theme.hex);
    if(entry.frameColorHex!==desiredHex){
      entry.frameColorHex=desiredHex;
      frameBatch.setColor(entry.frameSlot,desiredHex);
    }

    if(entry.recording){
      entry.recOrbScale=1+0.10*Math.sin(now*0.012);
    }
  }
  flushFenceFxUniforms(now,dt);
}

function updateLiveTagTouch(frameInputs,world,pose,now){
  liveTouchLoaderActive=false;
  if(!extraScanActive||!world||!session||!space||movableTools.size===0)return;
  const upL=frameInputs.viewerUpL;
  const rightL=frameInputs.viewerRightL;
  const viewerPosL=frameInputs.viewerPosL;
  const probes=frameInputs.getProbesL();

  const REC_START_HOLD_MS=650;
  const REC_STOP_HOLD_MS=350;
  const MARK_HOLD_MS=400;

  for(const entry of movableTools.values()){
    if(!entry.visible||!entry.displayMatrix){
      resetTagRestState(entry.pillRestState);
      continue;
    }
    const wasOpen=Boolean(entry.recOrbVisible||entry.markOrbVisible);
    const holding=entry.recHoldSince!=null||entry.markHoldSince!=null;
    const rest=updateTagRestState(entry.pillRestState,entry.displayMatrix,now,{
      handCoupled:entry.handCoupled,
      pillsOpen:wasOpen||holding,
    });
    const tx=entry.displayMatrix[12],ty=entry.displayMatrix[13],tz=entry.displayMatrix[14];
    // Both [● REC] and [⏱ MARK] sit side-by-side below the tag (or in front of flat tabletop tags toward the viewer)
    // so the approaching hand never occludes the camera-to-tag line of sight. Active recording centers [■ STOP] below.
    const {zoneCenterL,recPosL,markPosL}=computeTagActionPillLayout({
      tagMatrix:entry.displayMatrix,
      edgeMm:entry.edgeMm,
      viewerPosL,
      viewerRightL:rightL,
      viewerUpL:upL,
      recording:entry.recording,
    });
    const [zx,zy,zz]=zoneCenterL;
    const [ox,oy,oz]=recPosL;
    const [mx,my,mz]=markPosL;
    entry.recOrbPosL=recPosL;
    entry.markOrbPosL=markPosL;

    let minTagDist=Infinity,minZoneDist=Infinity,minOrbDist=Infinity,minMarkDist=Infinity;
    if(rest.isResting){
      for(const pr of probes){
        // Never let the hand currently carrying this tag open or trigger pills for itself;
        // the opposite hand can open pills once the carrying hand holds the tag steady for >= 500 ms.
        const carryingThis=pr.kind==='hand'&&(
          tagHandCoupler.isHandCarrying(pr.handedness,entry.tagId)||
          (entry.handCoupled&&entry.coupledHand===pr.handedness)
        );
        if(carryingThis)continue;
        const dt=Math.hypot(tx-pr.pos[0],ty-pr.pos[1],tz-pr.pos[2]);
        if(dt<minTagDist)minTagDist=dt;
        const dz=Math.hypot(zx-pr.pos[0],zy-pr.pos[1],zz-pr.pos[2]);
        if(dz<minZoneDist)minZoneDist=dz;
        const doOrb=Math.hypot(ox-pr.pos[0],oy-pr.pos[1],oz-pr.pos[2]);
        if(doOrb<minOrbDist)minOrbDist=doOrb;
        const doMark=Math.hypot(mx-pr.pos[0],my-pr.pos[1],mz-pr.pos[2]);
        if(doMark<minMarkDist)minMarkDist=doMark;
        const nearest=recordAllActive
          ?(entry.recording?Infinity:Math.min(dz,doMark))
          :(entry.recording?Math.min(dz,doOrb):Math.min(dz,doOrb,doMark));
        recordProbeProximity(pr.probeId,nearest);
      }
    }

    const {showRec:showOrb,showMark}=evaluateTagActionPills({
      isResting:rest.isResting,
      recording:entry.recording,
      recordAllActive,
      hasPose:Boolean(entry.displayMatrix),
      wasOpen,
      holding,
      minTagDist,
      minZoneDist,
      minRecDist:minOrbDist,
      minMarkDist,
    });
    const pillSize=TAG_ACTION_PILL_DEFAULTS.pillSizeM;
    const hotDistM=TAG_ACTION_PILL_DEFAULTS.pillHotDistM;
    const pokeDistM=TAG_ACTION_PILL_DEFAULTS.pillPokeDistM;
    const releaseDistM=TAG_ACTION_PILL_DEFAULTS.pillReleaseDistM;
    const recCloser=!showMark||minOrbDist<=minMarkDist;
    const markCloser=!showOrb||minMarkDist<minOrbDist;

    entry.recOrbVisible=showOrb;
    const orbHot=showOrb&&recCloser&&minOrbDist<hotDistM;
    const orbScale=entry.recording?entry.recOrbScale:(orbHot?1.22:0.92);
    const orbHex=orbHot?0xfef08a:(entry.recording?0xef4444:0x0ea5e9);
    if(showOrb){
      recOrbBatch.setPoint(entry.recOrbSlot,ox,oy,oz,orbScale);
      if(entry.recOrbColorHex!==orbHex){
        entry.recOrbColorHex=orbHex;
        recOrbBatch.setColor(entry.recOrbSlot,orbHex);
      }
      recOrbBatch.setVisible(entry.recOrbSlot,true);
      drawRecOrbPill(entry,entry.recording,orbHot);
      badgeMesh.setQuad(entry.recPillSlot,entry.recOrbPosL,pillSize,pillSize,true);
    }else{
      recOrbBatch.setVisible(entry.recOrbSlot,false);
      badgeMesh.setQuadVisible(entry.recPillSlot,false);
    }

    entry.markOrbVisible=showMark;
    const markHot=showMark&&markCloser&&minMarkDist<hotDistM;
    const markScale=markHot?1.22:0.92;
    const markHex=markHot?0xfef08a:0x22d3ee;
    if(showMark){
      recOrbBatch.setPoint(entry.markOrbSlot,mx,my,mz,markScale);
      if(entry.markOrbColorHex!==markHex){
        entry.markOrbColorHex=markHex;
        recOrbBatch.setColor(entry.markOrbSlot,markHex);
      }
      recOrbBatch.setVisible(entry.markOrbSlot,true);
      drawMarkOrbPill(entry,markHot);
      badgeMesh.setQuad(entry.markPillSlot,entry.markOrbPosL,pillSize,pillSize,true);
    }else{
      recOrbBatch.setVisible(entry.markOrbSlot,false);
      badgeMesh.setQuadVisible(entry.markPillSlot,false);
    }

    // 1. Left icon-only Record / Stop button poke (2.6 cm hit zone, 650 ms start / 350 ms stop, > 4.2 cm release).
    if(showOrb&&recCloser&&minOrbDist<pokeDistM){
      entry.markHoldSince=null;
      if(!entry.recHoldConsumed){
        if(entry.recHoldSince==null)entry.recHoldSince=now;
        const holdTargetMs=entry.recording?REC_STOP_HOLD_MS:REC_START_HOLD_MS;
        const prog=Math.min(1,(now-entry.recHoldSince)/holdTargetMs);
        const orbW=lPointToWorld(world,ox,oy,oz);
        pinLoadSprite.visible=true;
        pinLoadSprite.position.set(orbW[0],orbW[1]+0.026,orbW[2]);
        const sc=0.032+prog*0.010;
        pinLoadSprite.scale.set(sc,sc,1);
        pinLoadMat.opacity=1;
        drawPinLoader(prog,entry.recording?'#22c55e':'#ef4444',entry.recording?'■':'●',false);
        liveTouchLoaderActive=true;
        if(prog>=1){
          entry.recHoldConsumed=true;
          entry.recHoldSince=null;
          liveTouchLoaderActive=false;
          setTagRecording(entry,!entry.recording,now);
        }
      }
      continue;
    }else{
      entry.recHoldSince=null;
      if(minOrbDist>releaseDistM)entry.recHoldConsumed=false;
    }

    // 2. Right icon-only Spatial Pin button poke (2.6 cm hit zone, 400 ms hold, > 4.2 cm release).
    if(showMark&&markCloser&&minMarkDist<pokeDistM){
      if(!entry.markHoldConsumed){
        drawLiveBadge(entry,'near',entry.handCoupled);
        if(entry.markHoldSince==null)entry.markHoldSince=now;
        const prog=Math.min(1,(now-entry.markHoldSince)/MARK_HOLD_MS);
        const markW=lPointToWorld(world,mx,my,mz);
        pinLoadSprite.visible=true;
        pinLoadSprite.position.set(markW[0],markW[1]+0.026,markW[2]);
        const sc=0.032+prog*0.010;
        pinLoadSprite.scale.set(sc,sc,1);
        pinLoadMat.opacity=1;
        drawPinLoader(prog,'#22d3ee','pin',false);
        liveTouchLoaderActive=true;
        if(prog>=1){
          entry.markHoldConsumed=true;
          entry.markHoldSince=null;
          liveTouchLoaderActive=false;
          const snapshotPose=((entry.lockState==='locked'&&entry.lockedMatrix)?entry.lockedMatrix:entry.displayMatrix).slice();
          stampMovableWaypoint(entry.tagId,entry.edgeMm,snapshotPose,now,{skipLine:true,isTimeCopy:true});
          drawLiveBadge(entry,entry.lockState,entry.handCoupled);
        }
      }
    }else{
      entry.markHoldSince=null;
      if(minMarkDist>releaseDistM)entry.markHoldConsumed=false;
      if(!isEntryRecording(entry))drawLiveBadge(entry,entry.lockState,entry.handCoupled);
    }
  }
}

function stampMovableWaypoint(tagId,edgeMm,lFromTag,atMs,{silent=false,skipLine=false,isTimeCopy=false,speedMps=null,isBreak=false,isSegmentStart=false}={}){
  let entry=movableTools.get(tagId);
  if(!entry){entry=createMovableTool(tagId,edgeMm);movableTools.set(tagId,entry);}
  ensureMovableTrailVisuals(entry);
  if(entry.firstAtMs==null)entry.firstAtMs=atMs;
  entry.lastMatrix=lFromTag.slice();entry.lastAt=atMs;
  if(!entry.targetMatrix)entry.targetMatrix=lFromTag.slice();
  if(!entry.displayMatrix)entry.displayMatrix=lFromTag.slice();
  if(!skipLine)appendLinePoint(entry,lFromTag,Number.isFinite(speedMps)?speedMps:0,{isBreak,atMs});
  entry.waypointCount++;
  const timeCopyIndex=isTimeCopy?(++entry.timeCopyCount):0;
  const cx=lFromTag[12],cy=lFromTag[13],cz=lFromTag[14];
  const nz=Math.max(-1,Math.min(1,lFromTag[10]));
  const tiltDeg=Math.acos(Math.abs(nz))*180/Math.PI;
  const clock=new Date().toTimeString().slice(0,8);
  const wp={
    tagId,edgeMm,index:entry.waypointCount,isTimeCopy,timeCopyIndex,
    isTrailPoint:!isTimeCopy,
    isSegmentStart:Boolean(isSegmentStart),
    segmentIndex:isTimeCopy?0:(entry.trailSegmentCount||1),
    lineSeq:isTimeCopy?null:(entry.lineSeqCounter||null),
    speedMps:Number.isFinite(speedMps)?speedMps:null,
    matrix:lFromTag.slice(),centerL:[cx,cy,cz],
    xMm:cx*1000,yMm:cy*1000,zMm:cz*1000,tiltDeg,atMs,
    elapsedS:Math.max(0,(atMs-entry.firstAtMs)/1000),clock,
  };
  const dotR=wp.isSegmentStart?0:(isTimeCopy?PATHLINE_DEFAULTS.timeCopyDotRadiusM:PATHLINE_DEFAULTS.waypointDotRadiusM);
  const dotCol=waypointDotColorFor(entry,wp);
  if(entry.waypoints.length>=MAX_TRAIL_WAYPOINTS){
    entry.waypoints.shift();
    entry.waypoints.push(wp);
    for(let i=0;i<entry.waypoints.length;i++){
      const w=entry.waypoints[i];
      const r=w.isSegmentStart?0:(w.isTimeCopy?PATHLINE_DEFAULTS.timeCopyDotRadiusM:PATHLINE_DEFAULTS.waypointDotRadiusM);
      _tmpMat4.makeScale(r,r,r).setPosition(w.centerL[0],w.centerL[1],w.centerL[2]);
      entry.dotsMesh.setMatrixAt(i,_tmpMat4);
      entry.dotsMesh.setColorAt(i,waypointDotColorFor(entry,w));
    }
  }else{
    const idx=entry.waypoints.length;
    entry.waypoints.push(wp);
    _tmpMat4.makeScale(dotR,dotR,dotR).setPosition(cx,cy,cz);
    entry.dotsMesh.setMatrixAt(idx,_tmpMat4);
    entry.dotsMesh.setColorAt(idx,dotCol);
  }
  entry.dotsMesh.count=entry.waypoints.length;
  entry.dotsMesh.instanceMatrix.needsUpdate=true;
  if(entry.dotsMesh.instanceColor)entry.dotsMesh.instanceColor.needsUpdate=true;
  if(!silent){
    const flashPos=root.visible?lPointToWorld(root.matrix.elements,cx,cy,cz):null;
    if(flashPos)triggerPinFlash(flashPos,isTimeCopy?'#22d3ee':'#22c55e','✓');
    if(isTimeCopy){
      playModeTick('timecopy');
      showHudToast(`✓ TIME-COPY #${tagId} · T${timeCopyIndex}`,'#22d3ee',1700);
    }else{
      showHudToast(`✓ LOCKED #${tagId} · PT #${wp.index}`,entry.theme.css,1500);
    }
  }
  pushDiagEvent({type:isTimeCopy?'tag-time-copy':'tag-stamped',tagId,edgeMm,index:wp.index,timeCopyIndex,speedMps:wp.speedMps,atPerformanceMs:performance.now(),xMm:wp.xMm,yMm:wp.yMm,zMm:wp.zMm});
  return wp;
}

function disposeMovableEntry(e){
  freeEntryGapVisuals(e);
  if(e.frameSlot>=0)frameBatch.free(e.frameSlot);
  if(e.alertSlot>=0)alertGlowBatch.free(e.alertSlot);
  if(e.axesSlot>=0)axesBatch.free(e.axesSlot);
  if(e.faceSlot>=0){tagFaceMesh.clearSlot(e.faceSlot);tagFaceAtlas.release(e.faceSlot);}
  if(e.recOrbSlot>=0)recOrbBatch.free(e.recOrbSlot);
  if(e.recPillSlot>=0){badgeMesh.clearSlot(e.recPillSlot);badgeAtlas.release(e.recPillSlot);}
  if(e.markOrbSlot>=0)recOrbBatch.free(e.markOrbSlot);
  if(e.markPillSlot>=0){badgeMesh.clearSlot(e.markPillSlot);badgeAtlas.release(e.markPillSlot);}
  if(e.trailTube){
    extraRoot.remove(e.trailTube.mesh);
    e.trailTube.dispose();
    e.rawLinePositions=null;
    e.linePositions=null;
    e.lineSpeeds=null;
    e.lineColors=null;
    e.lineRenderColors=null;
    e.lineRadii=null;
    e.lineFades=null;
    e.lineRoles=null;
    e.lineOutside=null;
    e.lineBreaks=null;
    e.lineSegmentIds=null;
    e.lineSeqs=null;
  }
  e.passAnalysisMap?.clear?.();
  e.passAnalysisMap=null;
  e.passAnalysisSeq=-1;
  if(e.wakeTube){
    extraRoot.remove(e.wakeTube.mesh);
    e.wakeTube.dispose();
    e.wakeTube=null;
    e.wakeColors=null;
  }
  if(e.wakeBuffer){
    e.wakeBuffer.clear();
    e.wakeBuffer=null;
  }
  if(e.dotsMesh){
    extraRoot.remove(e.dotsMesh);
    e.dotMat?.dispose?.();
    e.dotsMesh.dispose?.();
  }
}

function clearAllUserItems(){
  const hadItems=userItemCount()>0||extraPending.size>0;
  lastMeasureTrailFocusKey='';
  hideWaypointInspector();
  clearPins();
  tagHandCoupler.reset();
  tagPredictor.reset();
  focusScheduler.reset();
  liveTouchLoaderActive=false;
  pinLoadSprite.visible=false;
  for(const e of movableTools.values())disposeMovableEntry(e);
  trailGapBridgeBatch.clear();
  trailGapCapBatch.clear();
  for(const extra of placedExtras.values()){
    if(extra.frameSlot>=0)frameBatch.free(extra.frameSlot);
    if(extra.alertSlot>=0)alertGlowBatch.free(extra.alertSlot);
    if(extra.axesSlot>=0)axesBatch.free(extra.axesSlot);
    if(extra.faceSlot>=0){tagFaceMesh.clearSlot(extra.faceSlot);tagFaceAtlas.release(extra.faceSlot);}
  }
  movableTools=new Map();
  placedExtras=new Map();
  extraPending=new Map();
  lastExtraAlertFenceVer=-1;
  clearFenceBreaches();
  if(hadItems)playModeTick('unpin');
  showHudToast(hadItems?'↺ CLEARED ALL TAGS & TRAILS':'NOTHING TO CLEAR','#f59e0b');
}

function clearExtras(){
  lastMeasureTrailFocusKey='';
  hideWaypointInspector();
  clearPins();
  tagHandCoupler.reset();
  tagPredictor.reset();
  focusScheduler.reset();
  handHistory.clear();
  for(const e of movableTools.values())disposeMovableEntry(e);
  trailGapBridgeBatch.clear();
  trailGapCapBatch.clear();
  for(const extra of placedExtras.values()){
    if(extra.frameSlot>=0)frameBatch.free(extra.frameSlot);
    if(extra.alertSlot>=0)alertGlowBatch.free(extra.alertSlot);
    if(extra.axesSlot>=0)axesBatch.free(extra.axesSlot);
    if(extra.faceSlot>=0){tagFaceMesh.clearSlot(extra.faceSlot);tagFaceAtlas.release(extra.faceSlot);}
  }
  for(const vis of perimeterTagVisuals.values()){
    if(vis.frameSlot>=0)frameBatch.free(vis.frameSlot);
    if(vis.axesSlot>=0)axesBatch.free(vis.axesSlot);
    if(vis.badgeSlot>=0){badgeMesh.clearSlot(vis.badgeSlot);badgeAtlas.release(vis.badgeSlot);}
  }
  for(const post of fencePostCache.values()){
    if(post.rodSlot>=0)rodBatch.free(post.rodSlot);
    if(post.capSlot>=0)nodeBatch.free(post.capSlot);
    if(post.badgeSlot>=0){badgeMesh.clearSlot(post.badgeSlot);badgeAtlas.release(post.badgeSlot);}
  }
  fencePostCache.clear();
  while(extraRoot.children.length){
    const ch=extraRoot.children[0];
    extraRoot.remove(ch);
    ch.geometry?.dispose?.();
    ch.material?.dispose?.();
  }
  placedExtras=new Map();extraPending=new Map();movableTools=new Map();
  recordAllActive=false;lastModeRecAllKey='';
  lastExtraAlertFenceVer=-1;
  perimeterTagVisuals.clear();
  perimeterFence?.reset();
  clearFenceBreaches();
  perimeterOfferSuppressed=false;lastModeFenceKey='';
  lastSyncedFenceVer=-1;lastSyncedFenceScan=false;
  fenceRiseStartMs=0;perimeterCompleteAnnounced=false;
  hoveredTlOffer=false;tlOfferPressSince=null;tlOfferGroup.visible=false;fenceGroup.visible=false;
  syncFenceVisuals(performance.now(),0,true);
}

let orbitMode=null,orbitCueKey=null;
const _haloAnchor=new THREE.Vector3();
const _haloHomePos=new THREE.Vector3();
const _haloRight=new THREE.Vector3();
const _haloUp=new THREE.Vector3();
const _haloNormal=new THREE.Vector3();
const _haloMat4=new THREE.Matrix4();
const _haloWorldUp=new THREE.Vector3(0,1,0);
const _haloSectors=new Uint8Array(8);
let _haloAnchored=false,_haloCursorX=0,_haloCursorY=0,_haloDoneCount=0,_haloVerifyMask=0,_haloMinDz=0,_haloMaxDz=0;

function orientHaloToward(anchor,vx,vy,vz,vm){
  _haloNormal.set(vx-anchor.x,vy-anchor.y,vz-anchor.z);
  if(_haloNormal.lengthSq()<1e-5)_haloNormal.set(vm[8],vm[9],vm[10]);
  _haloNormal.normalize();
  _haloRight.crossVectors(_haloWorldUp,_haloNormal);
  if(_haloRight.lengthSq()<1e-4)_haloRight.set(vm[0],vm[1],vm[2]);
  _haloRight.normalize();
  _haloUp.crossVectors(_haloNormal,_haloRight).normalize();
  _haloMat4.makeBasis(_haloRight,_haloUp,_haloNormal);
  spatialHaloGroup.position.copy(anchor);
  spatialHaloGroup.quaternion.setFromRotationMatrix(_haloMat4);
}

function resetOrbitGuide(){
  orbitMode=null;orbitCueKey=null;
  _haloAnchored=false;_haloCursorX=0;_haloCursorY=0;_haloDoneCount=0;_haloVerifyMask=0;_haloMinDz=0;_haloMaxDz=0;
  _haloSectors.fill(0);
  orbitGuideGroup.visible=false;
  spatialHaloGroup.visible=false;
}

function buildLiveTagBadgeState(now){
  const expected=rig?rig.tools[0].tags.map(t=>t.id):[350,351,352];
  const fresh=Boolean(lastObs)&&lastObsAtMs>0&&(now-lastObsAtMs)<1500;
  const acceptedIds=new Set((fresh?lastObs.tags:[])?.map(t=>t.id)??[]);
  const rejectedMap=new Map();
  if(fresh&&Array.isArray(lastObs.rejected)){
    for(const r of lastObs.rejected){
      if(r?.reason&&r.reason!=='unassigned_id')rejectedMap.set(r.id,r.reason);
    }
  }
  const items=expected.map(id=>{
    const status=!fresh?'miss':(acceptedIds.has(id)&&!rejectedMap.has(id))?'ok':rejectedMap.has(id)?'warn':'miss';
    return {id,status};
  });
  const extraSet=new Set();
  if(fresh&&Array.isArray(lastObs.unassignedDetections)){
    for(const u of lastObs.unassignedDetections)if(Number.isInteger(u?.id))extraSet.add(u.id);
  }
  const extraIds=[...extraSet].sort((a,b)=>a-b);
  const rms=fresh?lastObs?.tools?.[0]?.reprojectionRmsPx:null;
  const detMs=fresh?lastSampleStats?.detectMs:null;
  const rmsText=Number.isFinite(rms)?`${rms.toFixed(1)}px`:(Number.isFinite(detMs)?`${Math.round(detMs/10)*10}ms`:'WAIT');
  const key=`${items.map(it=>`${it.id}:${it.status}`).join(',')}|${extraIds.slice(0,4).join(',')}|${rmsText}`;
  return {items,extraIds,rmsText,key};
}

function drawOrbitCue(label,done,total,depthHint,goodFraming,{counterText='',tagState=null}={}){
  const ctx=orbitCueCanvas.getContext('2d');
  ctx.clearRect(0,0,440,124);
  ctx.fillStyle='rgba(8,14,24,0.88)';
  ctx.beginPath();ctx.roundRect(4,4,432,116,14);ctx.fill();
  ctx.strokeStyle=goodFraming?'#38bdf8':'#fbbf24';ctx.lineWidth=2.5;ctx.stroke();

  const text=depthHint?`${label} · ${depthHint}`:label;
  if(counterText){
    ctx.fillStyle=goodFraming?'#f8fafc':'#fde047';
    ctx.font='bold 19px system-ui,sans-serif';ctx.textAlign='left';
    ctx.fillText(text,18,31,316);
    ctx.fillStyle='rgba(30,41,59,0.92)';
    ctx.beginPath();ctx.roundRect(342,10,82,28,8);ctx.fill();
    ctx.strokeStyle=goodFraming?'#38bdf8':'#fbbf24';ctx.lineWidth=1.5;ctx.stroke();
    ctx.fillStyle=goodFraming?'#7dd3fc':'#fde68a';
    ctx.font='bold 14px ui-monospace,monospace';ctx.textAlign='center';
    ctx.fillText(counterText,383,29,74);
  }else{
    ctx.fillStyle=goodFraming?'#f8fafc':'#fde047';
    ctx.font='bold 19px system-ui,sans-serif';ctx.textAlign='center';
    ctx.fillText(text,220,31,404);
  }

  // Progress dots for orbital stations / lock timer.
  if(total>0){
    const n=Math.max(1,total),gap=Math.min(22,Math.floor(380/n)),startX=220-((n-1)*gap)/2;
    for(let i=0;i<n;i++){
      ctx.beginPath();ctx.arc(startX+i*gap,54,i===done?6.5:5,0,Math.PI*2);
      ctx.fillStyle=i<done?'#4ade80':i===done?'#fbbf24':'#475569';
      ctx.fill();
    }
  }

  // Row 3: Live detected tags strip.
  if(tagState){
    ctx.fillStyle='#94a3b8';ctx.font='bold 13px ui-monospace,monospace';ctx.textAlign='left';
    ctx.fillText('TAGS',18,97);
    const items=tagState.items.slice(0,3);
    items.forEach((it,idx)=>{
      const px=68+idx*80,py=78,pw=72,ph=28;
      const ok=it.status==='ok',warn=it.status==='warn';
      ctx.fillStyle=ok?'rgba(22,163,74,0.34)':warn?'rgba(217,119,6,0.34)':'rgba(127,29,29,0.28)';
      ctx.beginPath();ctx.roundRect(px,py,pw,ph,8);ctx.fill();
      ctx.strokeStyle=ok?'#4ade80':warn?'#fbbf24':'#f87171';ctx.lineWidth=1.8;ctx.stroke();
      ctx.fillStyle=ok?'#bbf7d0':warn?'#fde68a':'#fecaca';
      ctx.font='bold 14px ui-monospace,monospace';ctx.textAlign='center';
      const sym=ok?'✓':warn?'!':'✗';
      ctx.fillText(`${sym}${it.id}`,px+pw/2,py+19,pw-6);
    });
    if(tagState.extraIds.length>0){
      const extraLabel=`+${tagState.extraIds.slice(0,3).join(',')}${tagState.extraIds.length>3?'…':''}`;
      ctx.fillStyle='rgba(8,51,68,0.65)';
      ctx.beginPath();ctx.roundRect(312,78,112,28,8);ctx.fill();
      ctx.strokeStyle='#22d3ee';ctx.lineWidth=1.5;ctx.stroke();
      ctx.fillStyle='#67e8f9';ctx.font='bold 13px ui-monospace,monospace';ctx.textAlign='center';
      ctx.fillText(extraLabel,368,97,104);
    }else{
      ctx.fillStyle='#94a3b8';ctx.font='bold 13px ui-monospace,monospace';ctx.textAlign='center';
      ctx.fillText(tagState.rmsText,368,97,104);
    }
  }
  ctx.textAlign='left';
  orbitCueTex.needsUpdate=true;
}

// ---- First-seen L-angle world hint: rough 3D marker on the table where the tags were first spotted. ----
let rigIdsByKey={a4:[],tablet:[]};
let latestViewMat=null,tagHintViewMat=null,tagHintLastObsAt=0,tagHintLastSeenMs=0;
const tagHintGroup=new THREE.Group();tagHintGroup.visible=false;scene.add(tagHintGroup);
const tagHintTarget=new THREE.Vector3(),tagHintHasPos={v:false};
const tagHintRingMat=new THREE.MeshBasicMaterial({color:0x4ade80,transparent:true,opacity:0.9,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const tagHintRing=new THREE.Mesh(new THREE.RingGeometry(0.045,0.055,48),tagHintRingMat);
tagHintRing.rotation.x=-Math.PI/2;tagHintRing.renderOrder=996;tagHintGroup.add(tagHintRing);
const tagHintRing2=new THREE.Mesh(new THREE.RingGeometry(0.045,0.05,48),tagHintRingMat);
tagHintRing2.rotation.x=-Math.PI/2;tagHintRing2.renderOrder=996;tagHintGroup.add(tagHintRing2);
const tagHintArrow=new THREE.Mesh(new THREE.ConeGeometry(0.018,0.04,16),tagHintRingMat);
tagHintArrow.rotation.x=Math.PI;tagHintArrow.renderOrder=997;tagHintGroup.add(tagHintArrow);
const tagHintCanvas=document.createElement('canvas');tagHintCanvas.width=256;tagHintCanvas.height=72;
{const c=tagHintCanvas.getContext('2d');
 c.fillStyle='rgba(8,24,18,0.9)';c.beginPath();c.roundRect(3,3,250,66,14);c.fill();
 c.strokeStyle='#4ade80';c.lineWidth=3;c.stroke();
 c.fillStyle='#bbf7d0';c.font='bold 30px system-ui,sans-serif';c.textAlign='center';c.textBaseline='middle';
 c.fillText('L-ANGLE HERE',128,37,236);}
const tagHintSprite=new THREE.Sprite(new THREE.SpriteMaterial({map:makeUiTexture(tagHintCanvas),transparent:true,depthTest:false,depthWrite:false}));
tagHintSprite.scale.set(0.2,0.056,1);tagHintSprite.position.set(0,0.2,0);tagHintSprite.renderOrder=998;tagHintGroup.add(tagHintSprite);
function updateTagFirstSeenHint(pose,asPhase,anPhase,now){
  const calibrating=asPhase!=='ready'||anPhase==='collecting';
  if(!calibrating||!capture?.isRunning?.()){tagHintGroup.visible=false;tagHintHasPos.v=false;return;}
  // New fresh observation containing at least one expected L tag -> update rough world target.
  if(lastObs&&lastObsAtMs>tagHintLastObsAt&&tagHintViewMat){
    tagHintLastObsAt=lastObsAtMs;
    // Any known angle counts (IDs identify the angle uniquely) – selected rig first, other rig via unassigned detections.
    const allIds=new Set([...rigIdsByKey.a4,...rigIdsByKey.tablet]);
    const seen=[
      ...(lastObs.tags??[]).filter(t=>allIds.has(t.id)&&Array.isArray(t.pixels)&&t.pixels.length>=4),
      ...(lastObs.unassignedDetections??[]).filter(t=>allIds.has(t.id)&&Array.isArray(t.corners)&&t.corners.length>=4).map(t=>({id:t.id,pixels:t.corners})),
    ];
    if(seen.length){
      const [iw,ih]=lastObs.imageSize??[1280,1080];
      let cx=0,cy=0,n=0;
      for(const t of seen)for(const q of t.pixels){cx+=q.x;cy+=q.y;n++;}
      cx/=n;cy/=n;
      const fx=(iw/2)/Math.tan(65*Math.PI/360);
      const dir=new THREE.Vector3((cx-iw/2)/fx,-(cy-ih/2)/fx,-1).normalize();
      const m=new THREE.Matrix4().fromArray(tagHintViewMat);
      dir.transformDirection(m);
      const o=new THREE.Vector3().setFromMatrixPosition(m);
      const fr=analyzeFraming();
      const dist=Number.isFinite(fr.distCm)?THREE.MathUtils.clamp(fr.distCm/100,0.25,1.4):0.55;
      const pt=o.clone().addScaledVector(dir,dist);
      if(!tagHintHasPos.v){tagHintTarget.copy(pt);tagHintGroup.position.copy(pt);tagHintHasPos.v=true;}
      else tagHintTarget.lerp(pt,0.35);
      tagHintLastSeenMs=now;
    }
  }
  if(!tagHintHasPos.v||now-tagHintLastSeenMs>4000){tagHintGroup.visible=false;return;}
  const fresh=now-tagHintLastSeenMs<800;
  tagHintGroup.position.lerp(tagHintTarget,0.25);
  tagHintGroup.visible=!spatialHaloGroup.visible;
  const pulse=0.5+0.5*Math.sin(now*0.006);
  tagHintRing2.scale.setScalar(1+0.5*pulse);
  tagHintRingMat.opacity=(fresh?0.9:0.45)*(0.6+0.4*pulse);
  tagHintArrow.position.set(0,0.07+0.015*pulse,0);
  tagHintSprite.material.opacity=fresh?1:0.55;
}

function updateOrbitGuide(pose,asPhase,now,anState=null,asSummary=null){
  const activeMode=asPhase==='scanning'?'scan'
    :asPhase==='solving'?'solving'
    :(asPhase==='verifying'||asPhase==='checking')?'verify'
    :asPhase==='check-failed'?'check-failed'
    :(asPhase==='ready'&&(anState?.phase==='collecting'||anState?.phase==='ready'||anState?.phase==='creating'))?'lock'
    :null;
  if(!pose||!activeMode){orbitGuideGroup.visible=false;spatialHaloGroup.visible=false;return;}
  const tagState=buildLiveTagBadgeState(now);
  if(orbitMode!==activeMode){
    orbitMode=activeMode;orbitCueKey=null;
    if(activeMode==='scan'||activeMode==='verify'){
      _haloAnchored=false;_haloCursorX=0;_haloCursorY=0;_haloDoneCount=0;_haloVerifyMask=0;_haloMinDz=0;_haloMaxDz=0;
      _haloSectors.fill(0);
    }
  }
  if(activeMode==='solving'){
    orbitGuideGroup.visible=true;
    if(_haloAnchored&&tagHintHasPos.v){
      spatialHaloGroup.visible=true;haloCursorGroup.visible=false;tagHintGroup.visible=false;
      const sweep=Math.floor((now*0.008)%8);
      haloInnerMat.color.setHex(0x38bdf8);haloInnerMat.opacity=0.35;
      for(let i=0;i<8;i++){
        const d=(i-sweep+8)%8;
        haloSectorMats[i].color.setHex(d<=2?0x38bdf8:0x475569);
        haloSectorMats[i].opacity=d===0?0.95:d===1?0.72:d===2?0.50:0.25;
      }
    }else{spatialHaloGroup.visible=false;}
    const ck=`solving:${tagState.key}`;
    if(ck!==orbitCueKey){orbitCueKey=ck;drawOrbitCue('SOLVING CALIBRATION…',0,0,'',true,{counterText:'SOLVE',tagState});}
    return;
  }
  if(activeMode==='check-failed'){
    orbitGuideGroup.visible=true;spatialHaloGroup.visible=false;
    const ck=`check-failed:${tagState.key}`;
    if(ck!==orbitCueKey){orbitCueKey=ck;drawOrbitCue('CHECK FAILED · TRIGGER=RETRY',0,0,'',false,{counterText:'RETRY',tagState});}
    return;
  }
  const vm=pose.transform.matrix;
  if(activeMode==='lock'){
    orbitGuideGroup.visible=true;
    const q=anState?.quality||{};
    const count=q.count||0,req=q.required||5;
    const seen=q.seenTagCount??0;
    const lim=q.limits||{};
    const minSpanMs=lim.minSpanMs||1000;
    const spanMs=q.spanMs??0;
    const creating=anState?.phase==='ready'||anState?.phase==='creating';
    const lockFrac=creating?1:Math.min(req?count/req:0,minSpanMs?spanMs/minSpanMs:1);
    const shownDone=lockFrac>=1?req:Math.min(req-1,Math.floor(lockFrac*req));
    const spanSec=(Math.min(spanMs,minSpanMs)/1000).toFixed(1);
    const reqSec=(minSpanMs/1000).toFixed(1);
    let label='LOCK L · HOLD STEADY',hint='',good=true,counterText=`${spanSec}/${reqSec}s`;
    if(creating){
      label='CREATING ANCHOR…';counterText='OK';good=true;
    }else if(seen<3){
      label=`SHOW ALL 3 L-TAGS (${seen}/3)`;good=false;
    }else if(Number.isFinite(q.lastRmsPx)&&Number.isFinite(lim.maxRmsPx)&&q.lastRmsPx>lim.maxRmsPx){
      label=`SHARPEN · RMS ${q.lastRmsPx.toFixed(1)}/${lim.maxRmsPx.toFixed(1)}px`;hint='CLOSER / STILL';good=false;
    }else if(Number.isFinite(q.spreadMm)&&Number.isFinite(lim.maxPositionSpreadM)&&q.spreadMm>lim.maxPositionSpreadM*1000){
      label=`HOLD STILL · ${q.spreadMm.toFixed(1)}/${(lim.maxPositionSpreadM*1000).toFixed(0)}mm`;good=false;
    }else if(Number.isFinite(q.spreadDeg)&&Number.isFinite(lim.maxOrientationSpreadDeg)&&q.spreadDeg>lim.maxOrientationSpreadDeg){
      label=`HOLD STILL · ${q.spreadDeg.toFixed(1)}°`;good=false;
    }
    if(tagHintHasPos.v&&now-tagHintLastSeenMs<=4000){
      if(!_haloAnchored){_haloAnchor.copy(tagHintTarget);_haloAnchored=true;}
      else _haloAnchor.lerp(tagHintTarget,0.25);
      orientHaloToward(_haloAnchor,vm[12],vm[13],vm[14],vm);
      spatialHaloGroup.visible=true;haloCursorGroup.visible=false;tagHintGroup.visible=false;
      const filled=creating?8:Math.min(8,Math.floor(lockFrac*8+1e-4));
      haloInnerMat.color.setHex(creating?0x22c55e:good?0x38bdf8:0xf87171);
      haloInnerMat.opacity=0.32;
      for(let k=0;k<8;k++){
        const mat=haloSectorMats[HALO_CW_ORDER[k]];
        if(k<filled){mat.color.setHex(creating?0x22c55e:good?0x38bdf8:0xf87171);mat.opacity=0.95;}
        else{mat.color.setHex(good?0x475569:0x7f1d1d);mat.opacity=0.35;}
      }
    }else{
      spatialHaloGroup.visible=false;
    }
    const ck=`lock:${creating?1:0}:${shownDone}:${req}:${label}:${hint}:${counterText}:${good?1:0}:${tagState.key}`;
    if(ck!==orbitCueKey){orbitCueKey=ck;drawOrbitCue(label,shownDone,req,hint,good,{counterText,tagState});}
    return;
  }

  const isScan=activeMode==='scan';
  const counterText=asSummary?`${asSummary.count}/${asSummary.required}`:'';
  const framing=analyzeFraming();
  if(!_haloAnchored){
    if(framing.seenCount<2||!tagHintHasPos.v){
      orbitGuideGroup.visible=true;spatialHaloGroup.visible=false;
      const k=`find:${framing.cue}:${counterText}:${tagState.key}`;
      if(k!==orbitCueKey){orbitCueKey=k;drawOrbitCue(framing.cue,0,isScan?8:2,'',false,{counterText,tagState});}
      return;
    }
    _haloAnchor.copy(tagHintTarget);
    _haloHomePos.set(vm[12],vm[13],vm[14]);
    orientHaloToward(_haloAnchor,vm[12],vm[13],vm[14],vm);
    _haloAnchored=true;_haloMinDz=0;_haloMaxDz=0;_haloCursorX=0;_haloCursorY=0;
  }else if(framing.seenCount>=2&&tagHintHasPos.v){
    _haloAnchor.lerp(tagHintTarget,0.12);
    spatialHaloGroup.position.copy(_haloAnchor);
  }

  const wx=vm[12]-_haloHomePos.x,wy=vm[13]-_haloHomePos.y,wz=vm[14]-_haloHomePos.z;
  const dx=wx*_haloRight.x+wy*_haloRight.y+wz*_haloRight.z;
  const dy=wx*_haloUp.x+wy*_haloUp.y+wz*_haloUp.z;
  const dz=wx*_haloNormal.x+wy*_haloNormal.y+wz*_haloNormal.z;
  if(framing.seenCount>=2){
    if(dz<_haloMinDz)_haloMinDz=dz;
    if(dz>_haloMaxDz)_haloMaxDz=dz;
  }
  const radial=Math.hypot(dx,dy);
  const fullR=isScan?0.095:0.065;
  const rNorm=radial>1e-4?Math.min(1,radial/fullR):0;
  const targetCx=radial>1e-4?(dx/radial)*rNorm*HALO_RING_R:0;
  const targetCy=radial>1e-4?(dy/radial)*rNorm*HALO_RING_R:0;
  _haloCursorX+=(targetCx-_haloCursorX)*0.25;
  _haloCursorY+=(targetCy-_haloCursorY)*0.25;

  spatialHaloGroup.visible=true;
  haloCursorGroup.visible=true;
  tagHintGroup.visible=false;
  haloCursorGroup.position.set(_haloCursorX,_haloCursorY,0.003);
  const depthScale=Math.max(0.75,Math.min(1.45,1-dz*1.8));
  haloCursorRing.scale.setScalar(depthScale);
  const seenOk=framing.seenCount>=2;
  haloCursorDotMat.color.setHex(seenOk?0x38bdf8:0xf87171);
  haloCursorRingMat.color.setHex(seenOk?0xe0f2fe:0xfca5a5);
  haloInnerMat.color.setHex(seenOk?0x94a3b8:0xf87171);
  haloInnerMat.opacity=seenOk?0.28:0.42;

  let hoveredSec=-1;
  if(radial>=0.025){
    let a=Math.atan2(dy,dx);
    if(a<0)a+=Math.PI*2;
    hoveredSec=Math.floor(((a+Math.PI/8)%(Math.PI*2))/(Math.PI/4))&7;
  }

  let doneCount=0,totalCount=8,cueLabel='',depthHint='';
  const pulse=0.5+0.5*Math.sin(now*0.006);
  if(isScan){
    if(seenOk&&radial>=0.045&&hoveredSec>=0&&!_haloSectors[hoveredSec]){
      _haloSectors[hoveredSec]=1;
      _haloDoneCount++;
    }
    doneCount=_haloDoneCount;totalCount=8;
    for(let i=0;i<8;i++){
      const mat=haloSectorMats[i];
      if(_haloSectors[i]){mat.color.setHex(0x22c55e);mat.opacity=0.95;}
      else if(i===hoveredSec&&radial>=0.045){mat.color.setHex(seenOk?0x38bdf8:0xf87171);mat.opacity=0.90;}
      else{mat.color.setHex(seenOk?0x94a3b8:0xf87171);mat.opacity=(seenOk?0.36:0.28)+0.16*pulse;}
    }
    const depthSpan=_haloMaxDz-_haloMinDz;
    if(doneCount===0)cueLabel='SWEEP HEAD AROUND HALO';
    else if(doneCount<8)cueLabel=`SWEEP REMAINING ARCS (${doneCount}/8)`;
    else if(depthSpan<0.05)cueLabel='LEAN CLOSER & BACK';
    else cueLabel='SMOOTH CIRCLES · ALMOST READY';
    if(doneCount>=4&&depthSpan<0.05)depthHint=_haloMaxDz<0.03?'CLOSER / BACK':'';
  }else{
    if(seenOk){
      if(dx<-0.032)_haloVerifyMask|=1;
      if(dx>0.032)_haloVerifyMask|=2;
    }
    const leftDone=(_haloVerifyMask&1)!==0,rightDone=(_haloVerifyMask&2)!==0;
    doneCount=(leftDone?1:0)+(rightDone?1:0);totalCount=2;
    for(let i=0;i<8;i++){
      const mat=haloSectorMats[i];
      const isLeft=(i===3||i===4||i===5),isRight=(i===7||i===0||i===1);
      if((isLeft&&leftDone)||(isRight&&rightDone)){mat.color.setHex(0x22c55e);mat.opacity=0.95;}
      else if(isLeft||isRight){mat.color.setHex(seenOk?0x38bdf8:0xf87171);mat.opacity=(seenOk?0.42:0.30)+0.18*pulse;}
      else{mat.color.setHex(0x475569);mat.opacity=0.16;}
    }
    cueLabel=!leftDone?'SWEEP LEFT':!rightDone?'SWEEP RIGHT':'HOLD STEADY';
  }

  orbitGuideGroup.visible=true;
  const shownLabel=framing.status!=='good'?framing.cue:cueLabel;
  const shownHint=framing.status==='good'?depthHint:'';
  const ck=`${orbitMode}:${doneCount}:${totalCount}:${shownLabel}:${shownHint}:${framing.status}:${counterText}:${tagState.key}`;
  if(ck!==orbitCueKey){
    orbitCueKey=ck;
    drawOrbitCue(shownLabel,doneCount,totalCount,shownHint,framing.status==='good',{counterText,tagState});
  }
}

function clearTrail(){
  while(trailGroup.children.length)trailGroup.remove(trailGroup.children[0]);
  calViewerPoses=[];trailGroup.visible=false;
  resetOrbitGuide();
}

function addTrailDot(){}

function calibrationMotionChecklist(){
  if(calViewerPoses.length<2)return {lr:false,ud:false,io:false,tilt:false,spans:[0,0,0],angleDeg:0};
  const xs=calViewerPoses.map(m=>m[12]),ys=calViewerPoses.map(m=>m[13]),zs=calViewerPoses.map(m=>m[14]);
  const spanX=Math.max(...xs)-Math.min(...xs),spanY=Math.max(...ys)-Math.min(...ys),spanZ=Math.max(...zs)-Math.min(...zs);
  const inv0=inverse(calViewerPoses[0]);
  const rvecs=calViewerPoses.map(p=>rigidVector(product(inv0,p)).slice(0,3));
  const maxAng=Math.max(...rvecs.map(v=>Math.hypot(...v)));
  let area=0;
  for(const a of rvecs)for(const b of rvecs)
    area=Math.max(area,Math.hypot(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]));
  return {
    lr:spanX>=0.14,ud:spanY>=0.09,io:spanZ>=0.14,tilt:maxAng>0.2&&area>0.015,
    spans:[spanX,spanY,spanZ],angleDeg:maxAng*180/Math.PI,
  };
}

function resolveUnassignedTagPose(raw,K,{minEdgePx=10,minAreaPx=100,maxRmsPx=1.5,minCosView=0,maxDepthM=Infinity,scaleRmsFactor=0,softFallbackMaxRms=0.55}={}){
  const capRms=scaleRmsFactor>0?Math.min(maxRmsPx,Math.max(0.50,minEdgePx*scaleRmsFactor)):maxRmsPx;
  const effMaxRms=raw.softFallback?Math.min(capRms,softFallbackMaxRms):capRms;
  if(raw.pose&&Number.isFinite(raw.rmsPx)&&Number.isFinite(raw.minEdgePx)&&Number.isFinite(raw.edgeMm)){
    const effRmsLimit=scaleRmsFactor>0
      ?Math.min(maxRmsPx,Math.max(0.50,raw.minEdgePx*scaleRmsFactor),raw.softFallback?softFallbackMaxRms:Infinity)
      :effMaxRms;
    const cosView=Number.isFinite(raw.cosView)?raw.cosView:1;
    const depthM=Number.isFinite(raw.depthM)?raw.depthM:(raw.pose.t?.[2]??0);
    if(
      raw.minEdgePx<minEdgePx||
      (Number.isFinite(raw.areaPx)&&raw.areaPx<minAreaPx)||
      raw.rmsPx>effRmsLimit||
      cosView<minCosView||
      depthM>maxDepthM
    ){
      return null;
    }
    return {edgeMm:raw.edgeMm,pose:raw.pose,corners:raw.corners,minEdgePx:raw.minEdgePx,rmsPx:raw.rmsPx,cosView,depthM,softFallback:Boolean(raw.softFallback)};
  }
  if(raw.workerSolved)return null;
  const solved=solveSingleTagPose(raw,K,{minEdgePx,minAreaPx,maxRmsPx,minCosView,maxDepthM});
  if(!solved)return null;
  const solvedRmsLimit=scaleRmsFactor>0
    ?Math.min(maxRmsPx,Math.max(0.50,solved.minEdgePx*scaleRmsFactor),raw.softFallback?softFallbackMaxRms:Infinity)
    :effMaxRms;
  if(solved.rmsPx>solvedRmsLimit)return null;
  return {...solved,softFallback:Boolean(raw.softFallback)};
}

// Seed any visible 353-359 perimeter markers directly in L-frame during L-angle lock
// so the 20 cm enclosure immediately spans all visible boundary markers upon anchor creation.
function observePerimeterFromL(observation,lFromCamera,atMs,poseSource='optical-l'){
  if(!perimeterFence||!lFromCamera)return;
  const K=observation.cameraModel,all=observation.unassignedDetections??[];
  if(!K||!all.length)return;
  const counts=new Map();for(const d of all)counts.set(d.id,(counts.get(d.id)||0)+1);
  const camPosL=[lFromCamera[12],lFromCamera[13],lFromCamera[14]];
  for(const raw of all){
    if(!isPerimeterTag(raw.id)||counts.get(raw.id)!==1)continue;
    const solved=resolveUnassignedTagPose(raw,K,{minEdgePx:8,minAreaPx:64,maxRmsPx:1.8});
    if(!solved)continue;
    const lFromTag=product(lFromCamera,cvToGL(solved.pose));
    const res=perimeterFence.observe(raw.id,lFromTag,camPosL,atMs,{
      corners:raw.corners,lFromCamera,K,minEdgePx:solved.minEdgePx,poseSource,
    });
    if(res?.justLocked){
      showHudToast(`◈ LANDMARK #${raw.id} ${res.role} FIXED (z=0)`,'#4ade80',1800);
      pushDiagEvent({type:'perimeter-landmark-locked',tagId:raw.id,role:res.role,atPerformanceMs:performance.now(),posL:res.pos});
    }
  }
}

function isTagNearAnyHand(tagMatrix,targetXrTimeMs,maxDistM=0.22){
  if(!tagMatrix||!Number.isFinite(targetXrTimeMs))return false;
  const tx=tagMatrix[12],ty=tagMatrix[13],tz=tagMatrix[14];
  for(const h of ['left','right']){
    const hs=handHistory.sampleAt(h,targetXrTimeMs)||handHistory.latest(h,targetXrTimeMs,160);
    if(!hs?.anchors)continue;
    for(const a of hs.anchors){
      if(Math.hypot(a[0]-tx,a[1]-ty,a[2]-tz)<=maxDistM)return true;
    }
  }
  return false;
}

function observeExtraTags(observation,sample,assoc){
  if(!extraScanActive||!assoc)return;
  const K=observation.cameraModel,all=observation.unassignedDetections??[];
  const atMs=sample.submittedAtPerformanceMs;
  const allowBridge=selectedRigKey==='a4';
  const perimeterLandmarks=allowBridge?(perimeterFence?.lockedLandmarks?.()??[]):[];
  const camInL=resolveCameraInL(observation,assoc,{
    allowOpticalBridge:allowBridge,
    perimeterLandmarks,
    anchorCorrection:allowBridge?anchorDriftCorrection.state().matrix:null,
  });
  if(!camInL)return;
  if(perimeterFence){
    if(camInL.source==='optical-perimeter'&&Array.isArray(camInL.inlierIds)){
      for(const id of camInL.inlierIds){
        if(isPerimeterTag(id))perimeterFence.reportLandmarkInlier(id);
      }
    }
    if(Array.isArray(camInL.outliers)){
      for(const out of camInL.outliers){
        const dr=perimeterFence.reportLandmarkOutlier(out.id,{residualPx:out.rmsPx});
        if(dr?.demoted){
          showHudToast(`↺ LANDMARK #${out.id} ${dr.role} DEMOTED`,'#f59e0b',1800);
          pushDiagEvent({type:'perimeter-landmark-demoted',tagId:out.id,role:dr.role,residualPx:out.rmsPx,atPerformanceMs:performance.now()});
        }
      }
    }
  }
  const velocity=viewerVelocityAt(history,assoc.targetXrTimeMs,sample.context);
  const staticGate=evaluateMotionGate(velocity,{source:camInL.source},{
    xrSteadyAngDegPerSec:15,
    xrMaxAngDegPerSec:48,
    xrSteadyLinMPerSec:0.14,
    xrMaxLinMPerSec:0.48,
    opticalSteadyAngDegPerSec:18,
    opticalMaxAngDegPerSec:50,
    opticalSteadyLinMPerSec:0.18,
    opticalMaxLinMPerSec:0.50,
  });
  if(allowBridge&&!staticGate.frozen&&(camInL.source==='optical-l'||camInL.source==='optical-perimeter')){
    anchorDriftCorrection.observeOptical({
      lFromCameraXrRaw:product(inverse(assoc.worldFromAnchor),assoc.worldFromCamera),
      lFromCameraOpt:camInL.lFromCamera,
      confidence:staticGate.confidence,
    });
  }
  const carriedGate=evaluateMotionGate(velocity,{source:camInL.source},CARRIED_MOTION_GATE_DEFAULTS);
  lastMotionState={
    ...staticGate,atMs,
    opticalTagCount:camInL.opticalTagCount,
    perimeterTagCount:camInL.perimeterTagCount??0,
    xrDeltaMm:camInL.xrDeltaMm,
  };
  if(lastSampleStats&&lastSampleStats.serial===sample.serial){
    lastSampleStats.poseSource=camInL.source;
    lastSampleStats.perimeterTagCount=camInL.perimeterTagCount??0;
    lastSampleStats.headAngDegPerSec=staticGate.angDegPerSec;
    lastSampleStats.headLinMPerSec=staticGate.linMPerSec;
    lastSampleStats.motionConfidence=staticGate.confidence;
    lastSampleStats.motionFrozen=staticGate.frozen;
    lastSampleStats.carriedConfidence=carriedGate.confidence;
    lastSampleStats.rawGenericIds=all.filter(d=>isGenericTag(d.id)).map(d=>d.id);
  }
  if(!K)return;
  const counts=new Map();for(const d of all)counts.set(d.id,(counts.get(d.id)||0)+1);

  if(staticGate.frozen&&carriedGate.frozen){
    // Keep already-visible generic tags alive at their anchored 90 Hz pose while head is turning fast.
    for(const raw of all){
      if(counts.get(raw.id)!==1||!isGenericTag(raw.id))continue;
      const existing=movableTools.get(raw.id);
      if(existing?.displayMatrix){existing.lastAt=atMs;existing.lastOpticalAtMs=atMs;}
    }
    return;
  }

  const lFromCamera=camInL.lFromCamera;
  const camPosL=[lFromCamera[12],lFromCamera[13],lFromCamera[14]];
  const staticMaxRms=1.45+0.55*staticGate.confidence;
  const carriedMaxRms=1.45+0.55*Math.max(staticGate.confidence,carriedGate.confidence);
  const perimeterMaxRms=Math.min(2.2,staticMaxRms+0.25);
  const farMode=Boolean(lastQualityState.longRangeActive);
  const steadyBoost=lastQualityState.tier!=='fluid'&&!hasActiveMovableTag(atMs);
  const focusIdsSet=new Set(activeFocusTagIds);
  if(Array.isArray(sample.settings?.focusTagIds)){
    for(const id of sample.settings.focusTagIds)focusIdsSet.add(id);
  }
  const valid=[];
  for(const raw of all){
    if(counts.get(raw.id)!==1)continue;
    const perimeter=isPerimeterTag(raw.id);
    const generic=isGenericTag(raw.id);
    if(!generic&&staticGate.frozen)continue;
    const existingMov=generic?movableTools.get(raw.id):null;
    const isFocusedTag=generic&&focusIdsSet.has(raw.id);
    const isTrackedMoving=Boolean(existingMov&&(existingMov.handCoupled||existingMov.recording||existingMov.lockState==='moving'));
    const relaxFocusGates=isTrackedMoving||isFocusedTag;
    const solved=resolveUnassignedTagPose(
      raw,
      K,
      perimeter
        ?{minEdgePx:farMode?8:(steadyBoost?8.5:9),minAreaPx:farMode?64:72,maxRmsPx:perimeterMaxRms,minCosView:0.22,maxDepthM:farMode?3.2:(steadyBoost?2.6:2.2),scaleRmsFactor:0.08}
        :{
          minEdgePx:farMode?9:(steadyBoost?10:11),minAreaPx:farMode?80:(steadyBoost?92:110),
          maxRmsPx:relaxFocusGates?Math.max(carriedMaxRms,2.8):(generic?carriedMaxRms:staticMaxRms),
          minCosView:relaxFocusGates?0.22:0.30,
          maxDepthM:farMode?3.5:(steadyBoost?2.6:1.95),
          scaleRmsFactor:relaxFocusGates?0.11:0.055,
          softFallbackMaxRms:relaxFocusGates?2.4:0.55,
        },
    );
    if(!solved)continue;
    const {edgeMm,pose,minEdgePx,rmsPx,softFallback}=solved;
    const lFromTag=product(lFromCamera,cvToGL(pose));
    valid.push({tagId:raw.id,edgeMm,lFromTag,apparentEdgePx:minEdgePx,rmsPx,generic,perimeter,softFallback,corners:raw.corners});
  }
  if(lastSampleStats&&lastSampleStats.serial===sample.serial){
    lastSampleStats.seenGenericIds=valid.filter(v=>v.generic).map(v=>v.tagId);
  }

  // Always observe perimeter markers (353-359) while SCAN is active and staticGate allows.
  if(!staticGate.frozen){
    for(const v of valid){
      if(!v.perimeter)continue;
      const res=perimeterFence?.observe(v.tagId,v.lFromTag,camPosL,atMs,{
        weight:staticGate.confidence,
        corners:v.corners,
        lFromCamera,
        K,
        minEdgePx:v.apparentEdgePx,
        poseSource:camInL.source,
      });
      if(res?.justOffered){
        showHudToast('◈ TL #353 FOUND · PRESS TL FOR PERIMETER','#22d3ee',2800);
        pushDiagEvent({type:'perimeter-offered',tagId:v.tagId,role:res.role,atPerformanceMs:performance.now(),posL:res.pos});
      }else if(res?.isNew){
        if(perimeterFence.isActive()){
          if(perimeterFence.dynamicCount()>=7&&!perimeterCompleteAnnounced){
            perimeterCompleteAnnounced=true;
            const fSnap=perimeterFence.snapshot();
            showHudToast(`✓ PERIMETER 7/7 (${fSnap.areaM2.toFixed(2)} m²) · SWITCH TO MEASURE`,'#4ade80',3000);
          }else{
            showHudToast(`◈ FENCE +${v.tagId} ${res.role} (${perimeterFence.dynamicCount()}/7)`,'#22d3ee',1400);
          }
        }else{
          showHudToast(`◈ TAG #${v.tagId} ${res.role} FOUND`,'#22d3ee',1400);
        }
        pushDiagEvent({type:'perimeter-marker-detected',tagId:v.tagId,role:res.role,atPerformanceMs:performance.now(),posL:res.pos});
      }
      if(res?.justLocked){
        showHudToast(`◈ LANDMARK #${v.tagId} ${res.role} FIXED (z=0)`,'#4ade80',1800);
        pushDiagEvent({type:'perimeter-landmark-locked',tagId:v.tagId,role:res.role,atPerformanceMs:performance.now(),posL:res.pos});
      }else if(res?.demoted){
        showHudToast(`↺ LANDMARK #${v.tagId} ${res.role} DEMOTED`,'#f59e0b',1800);
        pushDiagEvent({type:'perimeter-landmark-demoted',tagId:v.tagId,role:res.role,reason:'moved',atPerformanceMs:performance.now()});
      }
    }
    const isFocusOnly=(sample.focusRoiCount??0)>0&&Boolean(sample.focusHit);
    const isFrozenFrame=sample.frameEvidence?.fresh===false||sample.frameEvidence?.pixelsChanged===false;
    if(perimeterFence&&!isFocusOnly&&!isFrozenFrame&&perimeterFence.dynamicCount()>0){
      const rawPerimeterSeen=[];
      for(const raw of all){if(isPerimeterTag(raw.id))rawPerimeterSeen.push(raw.id);}
      const occludedIds=[];
      for(const m of perimeterFence.discoveredMarkers()){
        if(isTagNearAnyHand(m.matrix,assoc.targetXrTimeMs,0.22))occludedIds.push(m.id);
      }
      const effDec=(sample.focusRoiCount??0)>0?(sample.settings?.focusFallbackDecimate??3):(sample.decimate??2);
      const pruned=perimeterFence.reconcileView({
        seenIds:rawPerimeterSeen,
        lFromCamera,
        K,
        atMs,
        confidence:staticGate.confidence,
        occludedIds,
        decimate:effDec,
        priorRois:sample.settings?.priorRois??null,
      });
      if(pruned?.removed?.length){
        if(perimeterFence.dynamicCount()<7)perimeterCompleteAnnounced=false;
        for(const rm of pruned.removed){
          const vis=perimeterTagVisuals.get(rm.id);
          if(vis?.wp){
            const pinKey=waypointIdKey(vis.wp);
            if(pinnedPoints.has(pinKey))unpinWaypoint(pinKey);
          }
          pushDiagEvent({
            type:'perimeter-marker-removed',
            tagId:rm.id,
            role:rm.role,
            atPerformanceMs:performance.now(),
            posL:rm.pos,
            deactivated:Boolean(pruned.deactivated),
          });
        }
        if(pruned.deactivated){
          showHudToast('◈ FENCE OFF · NO MARKERS LEFT','#f59e0b',1800);
        }else if(pruned.removed.length===1){
          const rm=pruned.removed[0];
          showHudToast(`◈ FENCE -${rm.id} ${rm.role} (${perimeterFence.dynamicCount()}/7)`,'#f59e0b',1400);
        }else{
          showHudToast(`◈ FENCE -${pruned.removed.length} MARKERS (${perimeterFence.dynamicCount()}/7)`,'#f59e0b',1400);
        }
      }
    }
  }

  // SCAN mode: track every generic tag (x00-x49) uniformly; pinning is a user action. Dedicated IDs without a known profile are only shown and warned about once.
  for(const v of valid){
    if(v.perimeter)continue;
    if(v.generic){
      const existing=movableTools.get(v.tagId);
      const nearHand=isTagNearAnyHand(v.lFromTag,assoc.targetXrTimeMs,0.22);
      const lockedDispM=(existing?.lockState==='locked'&&existing.lockedMatrix)
        ?Math.hypot(
          v.lFromTag[12]-existing.lockedMatrix[12],
          v.lFromTag[13]-existing.lockedMatrix[13],
          v.lFromTag[14]-existing.lockedMatrix[14],
        )
        :0;
      const useCarried=!existing||existing.handCoupled||isEntryRecording(existing)||existing.lockState!=='locked'||nearHand||lockedDispM>0.035;
      const effConf=useCarried?carriedGate.confidence:staticGate.confidence;
      if(effConf<=0&&existing?.displayMatrix)existing.lastAt=atMs;
      updateLiveMovablePreview(v.tagId,v.edgeMm,v.lFromTag,atMs,{
        confidence:effConf,
        targetXrTimeMs:assoc.targetXrTimeMs,
        decimate:sample.decimate??1,
        apparentEdgePx:v.apparentEdgePx,
        rmsPx:v.rmsPx,
        nearHand,
        allowRefine:!useCarried&&lastQualityState.allowRefine&&staticGate.confidence>=0.72,
        softFallback:v.softFallback,
        inFocus:focusIdsSet.has(v.tagId),
      });
      continue;
    }
    if(!warnedUnassignedDedicated.has(v.tagId)){
      warnedUnassignedDedicated.add(v.tagId);
      showHudToast(`⚠ ID ${v.tagId}: dedicated range, no profile assigned`,'#facc15',2400);
      pushDiagEvent({type:'unassigned-dedicated-id',tagId:v.tagId,atPerformanceMs:performance.now()});
    }
    if(staticGate.frozen||placedExtras.has(v.tagId)||staticGate.confidence<0.35)continue;
    const list=extraPending.get(v.tagId)??[];
    list.push({at:atMs,matrix:v.lFromTag});
    const recent=list.filter(r=>atMs-r.at<=4000).slice(-3);
    extraPending.set(v.tagId,recent);
    if(recent.length<3)continue;
    const mean=meanRigid(recent.map(r=>r.matrix)),invMean=inverse(mean);
    const maxPos=Math.max(...recent.map(r=>Math.hypot(...rigidVector(product(invMean,r.matrix)).slice(3))*1000));
    const maxAng=Math.max(...recent.map(r=>Math.hypot(...rigidVector(product(invMean,r.matrix)).slice(0,3))*180/Math.PI));
    if(maxPos>15||maxAng>8)continue;
    const vis=createExtraVisual(v.tagId,v.edgeMm,mean);
    const cx=mean[12],cy=mean[13],cz=mean[14];
    const nz=Math.max(-1,Math.min(1,mean[10]));
    const tiltDeg=Math.acos(Math.abs(nz))*180/Math.PI;
    const clock=new Date().toTimeString().slice(0,8);
    const wp={
      isStatic:true,tagId:v.tagId,edgeMm:v.edgeMm,index:0,matrix:mean.slice(),centerL:[cx,cy,cz],
      xMm:cx*1000,yMm:cy*1000,zMm:cz*1000,tiltDeg,atMs:null,elapsedS:0,clock,
    };
    placedExtras.set(v.tagId,{tagId:v.tagId,edgeMm:v.edgeMm,lFromTag:mean,placedAt:new Date().toISOString(),wp,...vis});
    syncStaticExtrasPerimeterAlerts(true);
    extraPending.delete(v.tagId);
    pushDiagEvent({type:'extra-tag-placed',tagId:v.tagId,edgeMm:v.edgeMm,atPerformanceMs:performance.now(),maxPosMm:maxPos,maxAngDeg:maxAng});
  }
}

function rotDeltaDeg(a,b){
  const tr=a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[4]*b[4]+a[5]*b[5]+a[6]*b[6]+a[8]*b[8]+a[9]*b[9]+a[10]*b[10];
  const cosTheta=Math.max(-1,Math.min(1,(tr-1)*0.5));
  return Math.acos(cosTheta)*180/Math.PI;
}

function recordAnchorStability(world){
  if(!world){anchorWinHead=0;anchorWinCount=0;hasPrevAnchorWorld=false;liveJitterMm=null;return;}
  if(!anchorBaseline){anchorBaseline=world.slice();anchorWinHead=0;anchorWinCount=0;}
  const baseIdx=anchorWinHead*3;
  anchorWinBuf[baseIdx]=world[12];
  anchorWinBuf[baseIdx+1]=world[13];
  anchorWinBuf[baseIdx+2]=world[14];
  anchorWinHead=(anchorWinHead+1)%ANCHOR_WIN_CAP;
  if(anchorWinCount<ANCHOR_WIN_CAP)anchorWinCount++;
  let sx=0,sy=0,sz=0;
  for(let i=0;i<anchorWinCount;i++){
    const k=i*3;
    sx+=anchorWinBuf[k];sy+=anchorWinBuf[k+1];sz+=anchorWinBuf[k+2];
  }
  const mx=sx/anchorWinCount,my=sy/anchorWinCount,mz=sz/anchorWinCount;
  let maxDist=0;
  for(let i=0;i<anchorWinCount;i++){
    const k=i*3;
    const d=Math.hypot(anchorWinBuf[k]-mx,anchorWinBuf[k+1]-my,anchorWinBuf[k+2]-mz);
    if(d>maxDist)maxDist=d;
  }
  liveJitterMm=maxDist*1000;
  liveDriftMm=Math.hypot(world[12]-anchorBaseline[12],world[13]-anchorBaseline[13],world[14]-anchorBaseline[14])*1000;
  liveDriftDeg=rotDeltaDeg(anchorBaseline,world);
  if(hasPrevAnchorWorld){
    const stepMm=Math.hypot(world[12]-_prevAnchorWorldBuf[12],world[13]-_prevAnchorWorldBuf[13],world[14]-_prevAnchorWorldBuf[14])*1000;
    const stepDeg=rotDeltaDeg(_prevAnchorWorldBuf,world);
    perfTracker.recordAnchorStep(stepMm,stepDeg,liveJitterMm,liveDriftMm);
  }
  for(let i=0;i<16;i++)_prevAnchorWorldBuf[i]=world[i];
  hasPrevAnchorWorld=true;
}

function wrapText(ctx,text,x,y,maxW,lineH,maxLines=3){
  const words=String(text||'').split(/\s+/);let line='',n=0;
  for(const w of words){
    const test=line?`${line} ${w}`:w;
    if(ctx.measureText(test).width>maxW&&line){
      ctx.fillText(line,x,y+n*lineH);line=w;n++;
      if(n>=maxLines-1)break;
    }else line=test;
  }
  if(line&&n<maxLines)ctx.fillText(line,x,y+n*lineH);
  return y+(n+1)*lineH;
}

function drawSparkline(ctx,x,y,w,h,values,{min=0,max=100,color='#38bdf8',guides=[]}){
  ctx.fillStyle='rgba(15,23,42,0.88)';ctx.fillRect(x,y,w,h);
  ctx.strokeStyle='#334155';ctx.lineWidth=1.5;ctx.strokeRect(x,y,w,h);
  for(const g of guides){
    const gy=y+h-((g.value-min)/(max-min))*h;
    ctx.strokeStyle=g.color;ctx.lineWidth=1;ctx.setLineDash([4,4]);
    ctx.beginPath();ctx.moveTo(x,gy);ctx.lineTo(x+w,gy);ctx.stroke();ctx.setLineDash([]);
  }
  if(values.length<2)return;
  ctx.strokeStyle=color;ctx.lineWidth=2.5;ctx.beginPath();
  for(let i=0;i<values.length;i++){
    const px=x+(i/Math.max(1,values.length-1))*w;
    const clamped=Math.max(min,Math.min(max,values[i]));
    const py=y+h-((clamped-min)/(max-min))*h;
    if(i===0)ctx.moveTo(px,py);else ctx.lineTo(px,py);
  }
  ctx.stroke();
}

function drawLightSparkline(ctx,x,y,w,h,samples,{guides=[]}={}){
  ctx.fillStyle='rgba(15,23,42,0.88)';ctx.fillRect(x,y,w,h);
  ctx.strokeStyle='#334155';ctx.lineWidth=1.5;ctx.strokeRect(x,y,w,h);
  for(const g of guides){
    const gy=y+h-(g.value/100)*h;
    ctx.strokeStyle=g.color;ctx.lineWidth=1;ctx.setLineDash([4,4]);
    ctx.beginPath();ctx.moveTo(x,gy);ctx.lineTo(x+w,gy);ctx.stroke();ctx.setLineDash([]);
  }
  const n=samples.length;
  if(n<2)return;
  ctx.lineWidth=2.5;
  const denom=Math.max(1,n-1);
  for(let i=1;i<n;i++){
    const a=samples[i-1],b=samples[i];
    const ax=x+((i-1)/denom)*w,ay=y+h-(Math.max(0,Math.min(100,a.score))/100)*h;
    const bx=x+(i/denom)*w,by=y+h-(Math.max(0,Math.min(100,b.score))/100)*h;
    ctx.strokeStyle=LIGHT_COLORS[b.level]||'#94a3b8';
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();
  }
}

function analyzeFraming(){
  const expected=rig?rig.tools[0].tags.map(t=>t.id):[350,351,352];
  const seenTags=(lastObs?.tags??[]).filter(t=>expected.includes(t.id));
  const seenSet=new Set(seenTags.map(t=>t.id));
  const missing=expected.filter(id=>!seenSet.has(id));
  const [iw,ih]=lastObs?.imageSize??[1280,1080];
  let distCm=null,nearEdge=false,tooSmall=false,minEdgePx=Infinity;
  if(seenTags.length){
    const edges=[];
    for(const t of seenTags){
      const p=t.pixels;
      for(let i=0;i<4;i++){
        const q=p[(i+1)%4];
        const e=Math.hypot(q.x-p[i].x,q.y-p[i].y);
        edges.push(e);if(e<minEdgePx)minEdgePx=e;
        if(p[i].x<iw*0.05||p[i].x>iw*0.95||p[i].y<ih*0.05||p[i].y>ih*0.95)nearEdge=true;
      }
    }
    edges.sort((a,b)=>a-b);
    const medEdgePx=edges[Math.floor(edges.length/2)];
    const fxEst=profile?.K?.fx??(iw*0.69);
    if(medEdgePx>1)distCm=(0.020*fxEst/medEdgePx)*100;
    if(minEdgePx<22)tooSmall=true;
  }
  let status='lost',cue='POINT AT L-ANGLE',color='#f87171';
  if(seenTags.length<2){
    status='lost';
    cue=missing.length===expected.length?'POINT AT L-ANGLE':`SHOW ${missing.join(' & ')}`;
    color='#f87171';
  }else if(nearEdge||(distCm!=null&&distCm<26)){
    status='warn';cue=`MOVE BACK (${distCm?distCm.toFixed(0)+' cm':'edge'})`;color='#fbbf24';
  }else if(tooSmall||(distCm!=null&&distCm>95)){
    status='warn';cue=`MOVE CLOSER (${distCm?distCm.toFixed(0)+' cm':'small'})`;color='#fbbf24';
  }else if(missing.length){
    status='warn';cue=`SHOW TAG ${missing[0]} (${distCm?distCm.toFixed(0)+' cm':'2/3'})`;color='#fbbf24';
  }else{
    status='good';cue=`SWEET SPOT · ${distCm?distCm.toFixed(0)+' cm':'OK'}`;color='#4ade80';
  }
  return {expected,seenSet,missing,seenCount:seenTags.length,distCm,nearEdge,tooSmall,status,cue,color};
}

function drawCameraInset(ctx,x,y,w,h,framing,{drawVideo=true,motion=null}={}){
  ctx.fillStyle='#020617';ctx.fillRect(x,y,w,h);
  const video=$('video');
  if(capture.state().running&&video.readyState>=2&&video.videoWidth){
    if(drawVideo){
      ctx.drawImage(video,x,y,w,h);
    }else{
      // Schematic quad radar during Step 4 SCAN avoids main-thread GPU video-readback stalls so XR stays near 90 FPS.
      ctx.strokeStyle='rgba(51,65,85,0.55)';ctx.lineWidth=1;
      ctx.beginPath();
      ctx.moveTo(x+w/2,y+6);ctx.lineTo(x+w/2,y+h-6);
      ctx.moveTo(x+6,y+h/2);ctx.lineTo(x+w-6,y+h/2);
      ctx.stroke();
    }
  }else{
    ctx.fillStyle='#64748b';ctx.font='16px system-ui';ctx.textAlign='center';
    ctx.fillText('Camera idle',x+w/2,y+h/2+5);ctx.textAlign='left';
  }
  // Safe-zone framing guide inside camera inset (keep all 3 tags inside dashed box).
  ctx.strokeStyle=framing?.status==='good'?'rgba(74,222,128,0.65)':'rgba(148,163,184,0.45)';
  ctx.lineWidth=1.5;ctx.setLineDash([5,4]);
  ctx.strokeRect(x+w*0.08,y+h*0.08,w*0.84,h*0.84);
  ctx.setLineDash([]);
  if(lastObs&&lastObs.imageSize){
    const [iw,ih]=lastObs.imageSize,sx=w/iw,sy=h/ih;
    const drawQuad=(pts,color,label)=>{
      ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
      pts.forEach((p,i)=>{const px=x+p.x*sx,py=y+p.y*sy;if(i===0)ctx.moveTo(px,py);else ctx.lineTo(px,py);});
      ctx.closePath();ctx.stroke();
      if(label){
        const cx=pts.reduce((s,p)=>s+p.x,0)/4*sx+x,cy=pts.reduce((s,p)=>s+p.y,0)/4*sy+y;
        ctx.fillStyle='rgba(2,6,23,0.75)';ctx.fillRect(cx-18,cy-10,36,16);
        ctx.fillStyle=color;ctx.font='bold 12px ui-monospace,monospace';ctx.textAlign='center';
        ctx.fillText(label,cx,cy+2);ctx.textAlign='left';
      }
    };
    const rejectedIds=new Set((lastObs.rejected??[]).filter(r=>r.reason==='rigid_tag_outlier'||r.reason==='inconsistent_rigid_geometry').map(r=>r.id));
    for(const t of lastObs.tags??[]){
      drawQuad(t.pixels,rejectedIds.has(t.id)?'#f87171':'#4ade80',String(t.id));
    }
    for(const u of lastObs.unassignedDetections??[]){
      const role=perimeterRole(u.id);
      const col=isGenericTag(u.id)?'#facc15':role?'#22d3ee':'#94a3b8';
      drawQuad(u.corners,col,role?`${u.id}${role}`:String(u.id));
    }
  }
  if(!drawVideo&&motion){
    const label=motion.frozen?`HOLD ${Math.round(motion.angDegPerSec)}°/s`:(motion.source==='optical-l'?'L-SYNC 0ms':'XR-SYNC');
    const col=motion.frozen?'#f59e0b':(motion.source==='optical-l'?'#4ade80':'#38bdf8');
    ctx.fillStyle='rgba(2,6,23,0.85)';ctx.fillRect(x+6,y+6,118,20);
    ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.strokeRect(x+6,y+6,118,20);
    ctx.fillStyle=col;ctx.font='bold 12px ui-monospace,monospace';
    ctx.fillText(label,x+12,y+20);
  }
  ctx.strokeStyle=framing?framing.color:(motion?.frozen?'#f59e0b':'#475569');ctx.lineWidth=2.5;ctx.strokeRect(x,y,w,h);
}

function drawDistanceGauge(ctx,x,y,w,framing){
  // Live distance meter: 20 cm .. 120 cm with green 35..80 cm sweet spot.
  ctx.fillStyle='#1e293b';ctx.fillRect(x,y,w,10);
  const toX=cm=>x+Math.max(0,Math.min(1,(cm-20)/100))*w;
  const sw0=toX(35),sw1=toX(80);
  ctx.fillStyle='rgba(74,222,128,0.35)';ctx.fillRect(sw0,y,sw1-sw0,10);
  ctx.strokeStyle='#4ade80';ctx.lineWidth=1;ctx.strokeRect(sw0,y,sw1-sw0,10);
  if(framing.distCm!=null){
    const mx=toX(framing.distCm);
    ctx.fillStyle=framing.color;
    ctx.beginPath();ctx.arc(mx,y+5,6,0,Math.PI*2);ctx.fill();
  }
  ctx.fillStyle='#94a3b8';ctx.font='12px ui-monospace,monospace';
  ctx.fillText('20cm',x,y+23);
  ctx.textAlign='center';ctx.fillStyle=framing.color;ctx.font='bold 13px ui-monospace,monospace';
  ctx.fillText(framing.cue,x+w/2,y+23);
  ctx.textAlign='right';ctx.fillStyle='#94a3b8';ctx.font='12px ui-monospace,monospace';
  ctx.fillText('120cm',x+w,y+23);ctx.textAlign='left';
}

function renderLeftHud(now,as,an,cs){
  const c=leftCanvas.getContext('2d');
  c.setTransform(HUD_DPR,0,0,HUD_DPR,0,0);
  c.clearRect(0,0,512,420);
  c.fillStyle='rgba(10,16,26,0.90)';c.fillRect(0,0,512,420);
  c.strokeStyle='#334155';c.lineWidth=3;c.strokeRect(2,2,508,416);

  const framing=analyzeFraming();
  const isCal=as.phase==='scanning'||as.phase==='verifying'||as.phase==='checking'||as.phase==='solving';
  const isLock=as.phase==='ready'&&(an.phase==='collecting'||an.phase==='ready');
  // Live peripheral reticle feedback during Steps 1-3 so user feels sweet-spot framing without looking up.
  if(isCal||isLock){
    reticle.material.color.set(framing.status==='good'?0x4ade80:framing.status==='warn'?0xfbbf24:0xf87171);
  }

  let title='STEP 1/4 · CALIBRATE',accent='#38bdf8',progress=0,copy='Keep the L-angle flat on the table. Move smoothly around it.';
  let controls='Hold Left Stick or B/Y (1.4s): Exit AR';

  if(as.phase==='scanning'){
    title=`STEP 1/4 · CALIBRATE (${as.count}/${as.required})`;
    progress=as.required?Math.min(1,as.count/as.required):0;
    const chk=calibrationMotionChecklist();
    if(framing.status!=='good'){
      copy=`${framing.cue} — keep all 3 tags inside the green box.`;
    }else if(!chk.lr){
      copy='NEXT: Shift head smoothly LEFT & RIGHT while watching all 3 tags.';
    }else if(!chk.ud){
      copy='NEXT: Move head HIGHER & LOWER (bend knees / stand taller).';
    }else if(!chk.io){
      copy='NEXT: Lean CLOSER (35 cm) and FARTHER (75 cm) smoothly.';
    }else if(!chk.tilt){
      copy='NEXT: Orbit & tilt around the L-angle from diagonal angles.';
    }else{
      copy='Great coverage! Keep moving smoothly until calibration finishes.';
    }
  }else if(as.phase==='solving'){
    title='STEP 1/4 · SOLVING…';accent='#f59e0b';progress=1;
    copy='Computing camera intrinsics, headset offset, and capture delay…';
  }else if(as.phase==='verifying'||as.phase==='checking'){
    const ids=rig?rig.tools[0].tags.map(t=>t.id).join('/'):'350/351/352';
    title=`STEP 2/4 · VERIFY (${as.count}/${as.required})`;accent='#38bdf8';
    progress=as.required?Math.min(1,as.count/as.required):0;
    copy=framing.status!=='good'
      ?`${framing.cue} — keep all 3 tags (${ids}) inside the green box.`
      :`Sweet spot! Shift head slightly side-to-side to verify (${ids}).`;
  }else if(as.phase==='check-failed'){
    title='STEP 2/4 · CHECK FAILED';accent='#f97316';progress=0;
    copy='Saved profile did not match fresh views. Retry the check or start a fresh calibration.';
    controls='Trigger: Retry check · Grip: Recalibrate';
  }else if(as.phase==='ready'){
    if(an.phase==='collecting'||an.phase==='ready'){
      const q=an.quality,ids=rig?rig.tools[0].tags.map(t=>t.id).join(', '):'350, 351, 352';
      const minSpanMs=q.limits?.minSpanMs||1000,spanMs=q.spanMs??0;
      title=`STEP 3/4 · LOCK L-ANGLE (${(Math.min(spanMs,minSpanMs)/1000).toFixed(1)}s)`;accent='#eab308';
      progress=an.phase==='ready'?1:(q.required?Math.min(1,q.count/q.required,spanMs/minSpanMs):0);
      copy=q.spreadMm!=null
        ?`Hold steady on all 3 tags. Spread: ${q.spreadMm.toFixed(1)} mm / ${q.spreadDeg?.toFixed(1)??'–'}° (limit 5 mm).`
        :(framing.status!=='good'?`${framing.cue} (${ids}).`:`${q.reason||'Hold steady on all 3 tags'} (${ids}).`);
    }else if(an.phase==='creating'){
      title='STEP 3/4 · CREATING ANCHOR…';accent='#eab308';progress=1;
      copy='Requesting native WebXR spatial anchor from Quest runtime…';
    }else if(an.phase==='tracking'||an.phase==='created'){
      const rawFStatus=perimeterFence?.status()??'idle';
      const fStatus=(rawFStatus==='offered'&&perimeterOfferSuppressed)?'idle':rawFStatus;
      const fenceCnt=perimeterFence?perimeterFence.dynamicCount():0;
      const recCount=[...movableTools.values()].filter(isEntryRecording).length;
      const fenceBadge=fStatus==='active'?`Fence:${fenceCnt}/7`:(fStatus==='offered'?'TL READY':'SCAN');
      title=recCount?`STEP 4/4 · ● REC PATH (${recCount})`:(recordAllActive?'STEP 4/4 · ● REC ALL (IDLE)':(extraScanActive?`STEP 4/4 · ${fenceBadge} · T:${movableTools.size}`:'STEP 4/4 · MEASURE (90 FPS)'));
      accent=recCount||recordAllActive?'#ef4444':(extraScanActive?'#22d3ee':'#22c55e');progress=1;
      copy=fStatus==='offered'
        ?'TL #353 found! Press TL on the table (or Trigger) to activate the 20 cm perimeter.'
        :(extraScanActive
          ?(fStatus==='active'
            ?'Auto-Lock + Follow · ● = path · Pin = Time-Copy · slide 353–359 for fence.'
            :'Auto-Lock + Follow · ● = path · Pin = Time-Copy · L-Fist = MEASURE.')
          :'Camera OFF (90 FPS). Hover + Pinch to pin/unpin · L-Fist / Grip = SCAN.');
      controls=now<reanchorConfirmUntil
        ?'PRESS TRIGGER AGAIN (1.5s) to re-anchor!'
        :`L-Fist (0.55s) / Palm-Dot: ${extraScanActive?'MEASURE':'SCAN (●)'} · Wrist-Dot: Menu`;
    }else if(an.phase==='lost'){
      title='ANCHOR LOST · RE-ANCHOR';accent='#ef4444';progress=0;
      copy='Native anchor pose unavailable. Look at the L-angle and press Trigger to re-anchor.';
      controls='1× Trigger: Re-anchor now · Stick/B/Y: Exit';
    }else if(an.phase==='failed'||an.phase==='unsupported'){
      title=an.phase==='unsupported'?'ANCHORS UNSUPPORTED':'ANCHOR FAILED';accent='#ef4444';progress=0;
      copy=an.error||'Native WebXR anchor could not be created.';
      controls=an.phase==='failed'?'1× Trigger: Try re-anchor · Stick/B/Y: Exit':'Hold Left Stick or B/Y (1.4s): Exit';
    }
  }

  c.fillStyle=accent;c.fillRect(0,0,512,8);
  c.fillStyle='#f8fafc';c.font='bold 24px system-ui';c.fillText(title,18,40);

  c.fillStyle='#1e293b';c.fillRect(18,52,476,10);
  c.fillStyle=accent;c.fillRect(18,52,Math.round(476*progress),10);

  c.fillStyle='#e2e8f0';c.font='18px system-ui';
  wrapText(c,copy,18,86,476,24,2);

  // Middle section: Live Camera Inset (left) + Distance Gauge underneath + Checklist/Tag Status (right).
  const motionFresh=lastMotionState&&now-lastMotionState.atMs<1200?lastMotionState:null;
  drawCameraInset(c,18,144,224,182,(isCal||isLock)?framing:null,{drawVideo:isCal||isLock,motion:motionFresh});
  if(isCal||isLock){
    drawDistanceGauge(c,18,334,224,framing);
  }

  if(isCal){
    const chk=calibrationMotionChecklist();
    c.fillStyle='#94a3b8';c.font='bold 15px system-ui';c.fillText('MOTION CHECKLIST',256,162);
    const items=[
      [chk.lr,`Left / Right (${(chk.spans[0]*100).toFixed(0)} cm)`],
      [chk.ud,`Up / Down (${(chk.spans[1]*100).toFixed(0)} cm)`],
      [chk.io,`Closer / Farther (${(chk.spans[2]*100).toFixed(0)} cm)`],
      [chk.tilt,`Tilt / Orbit (${chk.angleDeg.toFixed(0)}°)`],
    ];
    items.forEach(([ok,label],idx)=>{
      c.fillStyle=ok?'#4ade80':'#cbd5e1';
      c.font=ok?'bold 17px system-ui':'17px system-ui';
      c.fillText(`${ok?'[✓]':'[ ]'} ${label}`,256,192+idx*30);
    });
    // Per-tag visibility pills so user instantly sees which tag is missing/clipped.
    c.fillStyle='#94a3b8';c.font='bold 14px system-ui';c.fillText('L-TAGS VISIBLE',256,318);
    framing.expected.forEach((id,idx)=>{
      const ok=framing.seenSet.has(id);
      const px=256+idx*78;
      c.fillStyle=ok?'rgba(22,163,74,0.28)':'rgba(220,38,38,0.28)';
      c.fillRect(px,326,72,26);
      c.strokeStyle=ok?'#4ade80':'#f87171';c.lineWidth=1.5;c.strokeRect(px,326,72,26);
      c.fillStyle=ok?'#bbf7d0':'#fecaca';c.font='bold 15px ui-monospace,monospace';c.textAlign='center';
      c.fillText(`${ok?'✓':'✗'}${id}`,px+36,344);c.textAlign='left';
    });
  }else{
    c.fillStyle='#94a3b8';c.font='bold 16px system-ui';c.fillText('TAG STATUS',256,162);
    framing.expected.forEach((id,idx)=>{
      const ok=framing.seenSet.has(id);
      const px=256+idx*78;
      c.fillStyle=ok?'rgba(22,163,74,0.28)':'rgba(51,65,85,0.45)';
      c.fillRect(px,172,72,24);
      c.strokeStyle=ok?'#4ade80':'#475569';c.lineWidth=1.5;c.strokeRect(px,172,72,24);
      c.fillStyle=ok?'#bbf7d0':'#94a3b8';c.font='bold 14px ui-monospace,monospace';c.textAlign='center';
      c.fillText(`${ok?'✓':'·'}${id}`,px+36,189);c.textAlign='left';
    });
    const exSeen=(lastObs?.unassignedDetections??[]).map(u=>u.id).slice(0,5).join(', ')||'–';
    const rawFSnap=perimeterFence?.snapshot();
    const fSnap=(rawFSnap?.status==='offered'&&perimeterOfferSuppressed)?{...rawFSnap,status:'idle',offered:false}:rawFSnap;
    c.fillStyle='#e2e8f0';c.font='16px ui-monospace,monospace';
    c.fillText(`Extra:  ${exSeen}`,256,222);
    if(fSnap?.status==='active'){
      c.fillStyle='#22d3ee';
      c.fillText(`Fence:  ${fSnap.dynamicCount}/7 · ${fSnap.areaM2.toFixed(2)}m²`,256,246);
    }else if(fSnap?.status==='offered'){
      c.fillStyle='#fbbf24';
      c.fillText('Fence:  TL #353 · PRESS TL!',256,246);
    }else{
      c.fillStyle='#94a3b8';
      c.fillText(perimeterFence?'Fence:  show TL #353':'Fence:  off',256,246);
    }
    c.fillStyle='#e2e8f0';
    c.fillText(`Tools:  ${movableTools.size} · Pins: ${pinnedPoints.size}/${MAX_PINNED_POINTS}`,256,270);
    c.fillStyle='#94a3b8';c.font='15px system-ui';
    const fenceHint=perimeterFence
      ?(fSnap?.status==='offered'?'Press TL #353 to raise 20 cm perimeter & walk table.':(fSnap?.status==='active'?'SCAN updates 353–359 fence & locks tags; switch to MEASURE for 90 FPS.':'Show TL #353 in SCAN to offer perimeter; hover + Pinch to pin.'))
      :'SCAN locks tags & records paths; switch to MEASURE for 90 FPS hover + Pinch.';
    wrapText(c,fenceHint,256,296,240,20,3);
  }

  // Footer control bar.
  c.fillStyle='#1e293b';c.fillRect(0,368,512,52);
  c.fillStyle=now<reanchorConfirmUntil?'#fde047':'#f8fafc';c.font='bold 17px system-ui';
  c.fillText(controls,16,400);
  leftTex.needsUpdate=true;
}

function renderRightHud(now,as,an,cs){
  const c=rightCanvas.getContext('2d');
  c.setTransform(HUD_DPR,0,0,HUD_DPR,0,0);
  c.clearRect(0,0,512,420);
  c.fillStyle='rgba(10,16,26,0.90)';c.fillRect(0,0,512,420);
  c.strokeStyle='#334155';c.lineWidth=3;c.strokeRect(2,2,508,416);
  c.textAlign='left';c.textBaseline='alphabetic';

  // Prominent Camera Power / Load Badge (top banner).
  const camActive=cs.running||cs.starting;
  const detText=lastSampleStats?.detectMs!=null?` · det ${Math.round(lastSampleStats.detectMs)}ms`:'';
  const ppText=lastSampleStats?.preprocessMs!=null?` · pp ${Math.round(lastSampleStats.preprocessMs)}ms`:'';
  const decText=`d${cs.decimate??1}${lastQualityState.longRangeActive?'·EXP1':''}`;
  const camLabel=camActive?`● CAM ON (${decText}${ppText}${detText})`:'○ CAM OFF · NATIVE XR ONLY';
  c.fillStyle=camActive?'#dc2626':'#15803d';c.fillRect(0,0,512,40);
  c.fillStyle='#ffffff';c.font='bold 19px system-ui';c.textAlign='center';c.fillText(camLabel,256,27);c.textAlign='left';

  // Sparkline 1: XR FPS (always active from frame 1).
  const curFps=fpsHistory.length?fpsHistory.at(-1):0;
  c.fillStyle='#cbd5e1';c.font='bold 17px system-ui';
  c.fillText(`XR FPS: ${curFps.toFixed(0)}   maxGap: ${maxFrameGapMs.toFixed(0)} ms`,18,59);
  drawSparkline(c,18,65,476,54,fpsHistory,{
    min:0,max:100,color:curFps>=70?'#38bdf8':curFps>=45?'#f59e0b':'#ef4444',
    guides:[{value:72,color:'rgba(74,222,128,0.55)'}],
  });

  // Sparkline 2: Context-adaptive metric (Calibration RMS -> Seed Spread -> Anchor Jitter/Audit).
  const isCal=as.phase!=='ready';
  const auditFresh=lastAuditRow&&now-lastAuditAt<2500;
  let title2='ANCHOR STABILITY',badge='WAIT',badgeColor='#475569',series=stabilityHistory,maxScale=15,guides=[{value:3,color:'rgba(74,222,128,0.55)'},{value:10,color:'rgba(249,115,22,0.55)'}];

  if(isCal){
    const rms=rmsHistory.length?rmsHistory.at(-1):null;
    title2=`DETECTOR L-RMS: ${rms!=null?rms.toFixed(2)+' px':'–'}`;
    series=rmsHistory;maxScale=3;guides=[{value:1,color:'rgba(74,222,128,0.6)'}];
    if(rms!=null){
      if(rms<=1){badge='SHARP';badgeColor='#16a34a';}
      else if(rms<=2){badge='OK';badgeColor='#ca8a04';}
      else{badge='NOISY';badgeColor='#ea580c';}
    }
  }else if(an.phase==='lost'||an.phase==='failed'){
    title2='ANCHOR POSE LOST';badge='LOST';badgeColor='#dc2626';
  }else if(auditFresh){
    title2=`LIVE AUDIT Δ: ${lastAuditRow.positionDifferenceMm.toFixed(1)}mm / ${lastAuditRow.orientationDifferenceDeg.toFixed(1)}°`;
    const mm=lastAuditRow.positionDifferenceMm;
    if(mm<3){badge='ON POINT';badgeColor='#16a34a';}
    else if(mm<10){badge='OK';badgeColor='#ca8a04';}
    else{badge='DRIFT';badgeColor='#ea580c';}
  }else if((an.phase==='tracking'||an.phase==='created')&&now-lastMovedTargetAt<2500){
    title2='ANCHOR FIXED · L-TARGET MOVED';badge='FIXED';badgeColor='#0284c7';
  }else if(an.phase==='tracking'||an.phase==='created'){
    title2=`JITTER: ${liveJitterMm!=null?liveJitterMm.toFixed(2):'–'}mm · DRIFT: ${liveDriftMm!=null?liveDriftMm.toFixed(1):'–'}mm`;
    if(liveJitterMm!=null){
      if(liveJitterMm<3){badge='ON POINT';badgeColor='#16a34a';}
      else if(liveJitterMm<10){badge='OK';badgeColor='#ca8a04';}
      else{badge='DRIFT';badgeColor='#ea580c';}
    }
  }else{
    const sp=an.quality?.spreadMm;
    title2=`SEED SPREAD: ${sp!=null?sp.toFixed(1)+' mm':'–'}`;
    if(sp!=null){
      if(sp<3){badge='ON POINT';badgeColor='#16a34a';}
      else if(sp<=5){badge='READY';badgeColor='#ca8a04';}
      else{badge='STEADY';badgeColor='#ea580c';}
    }
  }

  c.fillStyle='#cbd5e1';c.font='bold 17px system-ui';c.fillText(title2,18,141,364);
  c.fillStyle=badgeColor;c.fillRect(390,124,104,22);
  c.fillStyle='#ffffff';c.font='bold 15px system-ui';c.textAlign='center';c.fillText(badge,442,140);c.textAlign='left';
  drawSparkline(c,18,150,476,54,series,{
    min:0,max:maxScale,
    color:badgeColor==='#16a34a'?'#4ade80':badgeColor==='#ca8a04'?'#facc15':'#f97316',
    guides,
  });

  // Sparkline 3: Light Intensity (traffic light + score + flags + raw telemetry + sparkline).
  const lightFresh=camActive&&Boolean(lightLast)&&now-lightLastAt<3000;
  const lightDotCol=lightFresh?(LIGHT_COLORS[lightLast.level]||'#94a3b8'):'#475569';
  const lightBadgeCol=lightFresh?(LIGHT_BADGE_COLORS[lightLast.level]||'#475569'):'#475569';
  const lightBadgeTxt=lightFresh?(LIGHT_LEVEL_LABELS[lightLast.level]||'OK'):(camActive?'WAIT':'OFF');
  const lightTitle=lightFresh?`LIGHT INTENSITY: ${lightLast.score}`:'LIGHT INTENSITY: –';

  c.fillStyle=lightDotCol;c.beginPath();c.arc(26,221,7.5,0,Math.PI*2);c.fill();
  c.fillStyle='#cbd5e1';c.font='bold 17px system-ui';c.fillText(lightTitle,40,227,342);
  c.fillStyle=lightBadgeCol;c.fillRect(390,210,104,22);
  c.fillStyle='#ffffff';c.font='bold 15px system-ui';c.textAlign='center';c.fillText(lightBadgeTxt,442,226);c.textAlign='left';

  if(lightFresh){
    const flagTxt=lightLast.flags.length?lightLast.flags.map(f=>LIGHT_FLAG_TEXT[f]||f.toUpperCase()).join(' · '):'Lighting OK';
    c.fillStyle=lightLast.flags.length?lightDotCol:'#86efac';c.font='bold 13px system-ui';
    c.fillText(flagTxt,18,249,236);
    const tagTxt=lightLast.tagCount?`Tag C ${Math.round(lightLast.tagContrastMin)}`:'Tag –';
    const rawTxt=`Luma ${Math.round(lightLast.meanLuma)} · Clip ${(lightLast.clipHi*100).toFixed(0)}% · ${tagTxt}`;
    c.fillStyle='#cbd5e1';c.font='13px ui-monospace,monospace';c.textAlign='right';
    c.fillText(rawTxt,494,249,232);c.textAlign='left';
  }else{
    c.fillStyle='#94a3b8';c.font='13px system-ui';
    c.fillText(camActive?'Waiting for camera frame…':'Camera off · switch to SCAN for live light metrics',18,249);
  }
  drawLightSparkline(c,18,255,476,62,lightScoreHistory,{
    guides:[
      {value:70,color:'rgba(74,222,128,0.50)'},
      {value:40,color:'rgba(250,204,21,0.45)'},
    ],
  });

  // Compact technical metrics footer.
  c.fillStyle='#1e293b';c.fillRect(18,327,476,1.5);
  const seed=an.seed;
  const line1=(seed&&profile)
    ?`Seed ±${seed.positionSpreadMm.toFixed(1)}mm · Cal ±${(profile.validation?.positionP95Mm??0).toFixed(1)}mm · fx ${Math.round(profile.K.fx)} k1 ${(profile.distCoeffs?.[0]??0).toFixed(3)}`
    :(seed
      ?`Seed ±${seed.positionSpreadMm.toFixed(2)}mm/${seed.orientationSpreadDeg.toFixed(2)}° · RMS ${seed.maxReprojectionRmsPx.toFixed(2)}px`
      :`Profile: ${profile?profile.id.slice(0,22):'uncalibrated'}`);
  const rbMode=lastSampleStats?.readbackMode==='worker-videoframe-y'?'vf-y'
    :lastSampleStats?.readbackMode==='worker-videoframe-canvas'?'vf-2d'
    :lastSampleStats?.readbackMode==='worker-bitmap'?'bitmap':'main-rb';
  const syncTag=(lastMotionState&&now-lastMotionState.atMs<1200)
    ?(lastMotionState.frozen?'HOLD':(lastMotionState.source==='optical-perimeter'?'P-0ms':(lastMotionState.source==='optical-l'?'L-0ms':'XR')))
    :(profile?`${Math.round(profile.timeOffsetMs)}ms`:'–');
  const line2=`${APP_VERSION_LABEL} · ${cs.sourceSize.join('×')} · ${rvfcHz.toFixed(0)}Hz${cs.rvfcStalled?'!':''} · ${rbMode} · ${syncTag}`;
  const auditState=audit.state();
  const fStat=perimeterFence?.status()??'idle';
  const fText=fStat==='active'?`${perimeterFence.dynamicCount()}/7`:(fStat==='offered'?'TL ready':'off');
  const tb=perfTracker.snapshot().trackingBenchmark;
  const lagText=tb?.opticalLagMs?.p50!=null?` · Lag ${Math.round(tb.opticalLagMs.p50)}ms`:'';
  const spdText=tb?.maxSpeedMmPerSec>0?` · Peak ${Math.round(tb.maxSpeedMmPerSec)}mm/s`:'';
  const line3=auditState.samples
    ?`Audit (${auditState.samples}×, read-only): med ${auditState.positionDifferenceMm.median.toFixed(1)}mm · p95 ${auditState.positionDifferenceMm.p95.toFixed(1)}mm`
    :`Frames ${an.posesTracked}/${an.posesMissing} · Fence ${fText}${lagText}${spdText}`;

  c.fillStyle='#e2e8f0';c.font='15px ui-monospace,monospace';c.fillText(line1,18,346);
  c.fillStyle='#94a3b8';c.font='14px ui-monospace,monospace';c.fillText(line2,18,374);
  c.fillStyle='#94a3b8';c.font='15px system-ui';c.fillText(line3,18,402);
  rightTex.needsUpdate=true;
}

function renderTopHudBar(now,as,an,cs){
  const c=topBarCanvas.getContext('2d');
  c.clearRect(0,0,840,120);
  const anchored=as.phase==='ready'&&(an.phase==='tracking'||an.phase==='created');
  const accent=!anchored?'#38bdf8':(extraScanActive?'#ef4444':'#4ade80');
  c.fillStyle='rgba(8,14,24,0.92)';
  c.beginPath();c.roundRect(6,6,828,108,18);c.fill();
  c.strokeStyle=accent;c.lineWidth=4;c.stroke();

  const fps=fpsHistory.length?Math.round(fpsHistory.reduce((a,b)=>a+b,0)/fpsHistory.length):0;
  let headline='STEP 1 · CALIBRATE';
  let sub='Move head around L-bracket';
  if(as.phase==='verifying'||as.phase==='checking'){
    headline='STEP 2 · VERIFY';sub='Shift head slightly to confirm';
  }else if(as.phase==='ready'&&an.phase==='collecting'){
    const q=an.quality||{};
    headline=`STEP 3 · LOCK L (${q.count||0}/${q.required||5})`;
    sub=q.reason||'Hold all 3 L-tags steady in view';
  }else if(anchored){
    const tagCount=movableTools.size+placedExtras.size+(perimeterFence?.dynamicCount?.()??0);
    headline=extraScanActive?`● SCAN MODE · ${fps} FPS`:`■ MEASURE MODE · ${fps} FPS`;
    sub=`Tags: ${tagCount} · Pins: ${pinnedPoints.size} · Drag bottom bar to reposition`;
  }else if(an.phase==='lost'){
    headline='ANCHOR LOST';sub='Look at L-bracket & press Trigger';
  }
  c.fillStyle=accent;c.font='bold 34px system-ui,sans-serif';c.textAlign='center';
  c.fillText(headline,420,52);
  c.fillStyle='#e2e8f0';c.font='600 22px system-ui,sans-serif';
  c.fillText(sub,420,90);
  c.fillStyle='#94a3b8';c.font='600 14px ui-monospace,monospace';c.textAlign='right';
  c.fillText(APP_VERSION_LABEL,816,28);
  c.textAlign='left';
  topBarTex.needsUpdate=true;
}

const _hudTmpV=new THREE.Vector3();
const _hudTmpG=new THREE.Vector3();
const _hudGrabPool=Array.from({length:4},()=>({id:'',worldPos:new THREE.Vector3(),rayOrigin:new THREE.Vector3(),rayDir:new THREE.Vector3(),hasRay:false,pinching:false,releasing:false}));
function updateHudPanelDragging(frameInputs,pose,now){
  if(!hudPanelsVisible||leaveConfirmState||!pose||!session||!space){
    hudDragState=null;
    if(!hudGrabResetDone){
      hudGrabResetDone=true;
      for(const p of hudDraggablePanels){p.grab.mat.color.setHex(0x94a3b8);p.grab.mat.opacity=0.82;}
    }
    return;
  }
  hudGrabResetDone=false;
  // Collect both near-touch grabbers AND far targetRaySpace rays (like native Quest windows, works at 1 m+ distance).
  let grabCount=0;
  for(const inp of frameInputs.inputs){
    let hasRay=false,rx=0,ry=0,rz=0,dx=0,dy=0,dz=0;
    const rp=inp.getTargetRayPose();
    if(rp){
      const rm=rp.transform.matrix;
      rx=rm[12];ry=rm[13];rz=rm[14];
      const rl=Math.hypot(rm[8],rm[9],rm[10])||1;
      dx=-rm[8]/rl;dy=-rm[9]/rl;dz=-rm[10]/rl;
      hasRay=true;
    }
    let wx=0,wy=0,wz=0,pinching=false,releasing=false,valid=false;
    if(inp.kind==='hand'){
      const jt=inp.getJoint('thumb-tip');
      const ji=inp.getJoint('index-finger-tip');
      if(jt&&ji){
        const d=Math.hypot(jt.transform.position.x-ji.transform.position.x,jt.transform.position.y-ji.transform.position.y,jt.transform.position.z-ji.transform.position.z);
        wx=(jt.transform.position.x+ji.transform.position.x)/2;
        wy=(jt.transform.position.y+ji.transform.position.y)/2;
        wz=(jt.transform.position.z+ji.transform.position.z)/2;
        pinching=d<0.028;releasing=d>0.042;valid=true;
      }
    }else{
      const tipW=inp.getCtrlTipWorld();
      if(tipW){
        [wx,wy,wz]=tipW;
        const btn=Boolean(inp.src.gamepad?.buttons?.[0]?.pressed||inp.src.gamepad?.buttons?.[1]?.pressed);
        pinching=btn;releasing=!btn;valid=true;
      }
    }
    if(!valid)continue;
    if(grabCount>=_hudGrabPool.length){
      _hudGrabPool.push({id:'',worldPos:new THREE.Vector3(),rayOrigin:new THREE.Vector3(),rayDir:new THREE.Vector3(),hasRay:false,pinching:false,releasing:false});
    }
    const g=_hudGrabPool[grabCount++];
    g.id=inp.probeId;
    g.worldPos.set(wx,wy,wz);
    g.hasRay=hasRay;
    if(hasRay){g.rayOrigin.set(rx,ry,rz);g.rayDir.set(dx,dy,dz);}
    g.pinching=pinching;g.releasing=releasing;
  }

  hudRig.updateMatrixWorld(true);
  if(hudDragState){
    let g=null;
    for(let i=0;i<grabCount;i++)if(_hudGrabPool[i].id===hudDragState.probeId){g=_hudGrabPool[i];break;}
    if(!g||g.releasing){
      const p=hudDragState.panel;
      hudLayout[p.id].pos=[Number(p.mesh.position.x.toFixed(3)),Number(p.mesh.position.y.toFixed(3)),Number(p.mesh.position.z.toFixed(3))];
      saveHudLayout(hudLayout);
      showHudToast(`✓ ${p.id.toUpperCase()} PANEL SAVED`,'#4ade80',1500);
      playModeTick('pin');
      hudDragState=null;
    }else{
      if(hudDragState.mode==='ray'&&g.hasRay){
        _hudTmpV.copy(g.rayOrigin).addScaledVector(g.rayDir,hudDragState.rayDist);
      }else{
        _hudTmpV.copy(g.worldPos);
      }
      hudRig.worldToLocal(_hudTmpV);
      const nextX=Math.max(-0.85,Math.min(0.85,_hudTmpV.x+hudDragState.offsetLocal.x));
      const nextY=Math.max(-0.45,Math.min(0.55,_hudTmpV.y+hudDragState.offsetLocal.y));
      const nextZ=Math.max(-1.60,Math.min(-0.45,_hudTmpV.z+hudDragState.offsetLocal.z));
      hudDragState.panel.mesh.position.set(nextX,nextY,nextZ);
      orientHudPanel(hudDragState.panel.mesh);
    }
  }

  for(const p of hudDraggablePanels){
    p.grab.mesh.getWorldPosition(_hudTmpG);
    let hot=hudDragState?.panel===p;
    if(!hudDragState){
      for(let i=0;i<grabCount;i++){
        const g=_hudGrabPool[i];
        // 1. Direct near-touch grab (< 8.5 cm)
        const dist=g.worldPos.distanceTo(_hudTmpG);
        if(dist<0.085){
          hot=true;
          if(g.pinching){
            _hudTmpV.copy(g.worldPos);
            hudRig.worldToLocal(_hudTmpV);
            hudDragState={panel:p,probeId:g.id,mode:'near',rayDist:0,offsetLocal:p.mesh.position.clone().sub(_hudTmpV)};
            playModeTick('pin');
            break;
          }
        }
        // 2. Far ray-pinch / trigger grab (works comfortably at 1.0 m+ HUD distance)
        if(g.hasRay){
          _hudRaycaster.set(g.rayOrigin,g.rayDir);
          const hits=_hudRaycaster.intersectObject(p.grab.hitMesh,false);
          if(hits.length){
            hot=true;
            hudRayDot.visible=true;
            hudRayDot.position.copy(hits[0].point);
            if(g.pinching){
              _hudTmpV.copy(hits[0].point);
              hudRig.worldToLocal(_hudTmpV);
              hudDragState={panel:p,probeId:g.id,mode:'ray',rayDist:hits[0].distance,offsetLocal:p.mesh.position.clone().sub(_hudTmpV)};
              playModeTick('pin');
              break;
            }
          }
        }
      }
    }
    p.grab.mat.color.setHex(hudDragState?.panel===p?0x4ade80:(hot?0xfde047:0x94a3b8));
    p.grab.mat.opacity=hudDragState?.panel===p?1.0:(hot?0.98:0.82);
  }
}

const anchor=createLAnchorController({
  quality:{softReject:true,maxRmsPx:1.15,maxPositionSpreadM:0.005,maxOrientationSpreadDeg:1.5},
  onCreated:s=>{
    extraScanActive=true;history=[];resetAnchorStability();obsBuffer.clear();clearTrail();
    hudPanelsVisible=false;hudDragState=null;resetHudCache();lastModePillKey='';
    if(!capture.isRunning?.())capture.start().catch(()=>{});
    syncFenceVisuals(performance.now(),0,true);
    playModeTick('scan');
    showCenterConfirm('✓ L-ANGLE LOCKED','Now walk around & look at AprilTags to scan','#4ade80',3400);
    if(perimeterFence?.isOffered()&&!perimeterOfferSuppressed){
      showHudToast('◈ TL #353 FOUND · PRESS TL FOR PERIMETER','#22d3ee',2800);
    }
    status(`Anchored (seed ±${s.seed?.positionSpreadMm?.toFixed(2)??'?'} mm) · Walk around & look at AprilTags to scan`);
  },
  onEvent:(type,detail)=>{
    pushDiagEvent({type,...detail});
    if(type==='anchor-tracking-lost')status('Anchor tracking lost · press Trigger to re-anchor');
    else if(type==='anchor-create-failed')status(`Anchor failed: ${detail.error}`);
    else if(type==='anchor-unavailable')status('WebXR Anchors are not supported by this browser.');
  },
});

let capture,rig,arSupported=false,cameraPermState='unknown',selectedRigKey='a4';
const rigButtons=Array.from(document.querySelectorAll('.rig-btn[data-rig]'));
function setRigControlsDisabled(disabled){
  if($('rigSelect'))$('rigSelect').disabled=disabled;
  for(const b of rigButtons)b.disabled=disabled;
}
async function end(){await session?.end();}

function finalizeDiagnostics(){
  if(!diagnostics)return;
  diagnostics.endedAt=new Date().toISOString();
  const perfSnap=perfTracker.snapshot();
  const xrLayer=session?.renderState?.layers?.[0]??session?.renderState?.baseLayer??null;
  diagnostics.xr={
    maxFrameGapMs,
    lastFps:fpsHistory.at(-1)??null,
    historySamples:history.length,
    trailColorMode,
    frameTiming:perfSnap.frameTiming,
    hud:perfSnap.hud,
    anchorStability:perfSnap.anchorStability,
    trackingBenchmark:perfSnap.trackingBenchmark,
    adaptiveQuality:{mode:adaptiveQuality.getMode(),...adaptiveQuality.stats()},
    compositor:{
      frameRate:session?.frameRate??null,
      supportedFrameRates:session?.supportedFrameRates?Array.from(session.supportedFrameRates):null,
      foveation:renderer.xr.getFoveation?.()??appliedFoveation,
      enabledFeatures:session?.enabledFeatures?Array.from(session.enabledFeatures):null,
      layerType:session?.renderState?.layers?.length?'projection-layer':(session?.renderState?.baseLayer?'webgl-layer':'unknown'),
      ignoreDepthValues:xrLayer?.ignoreDepthValues??null,
      antialias:rendererAa??currentAaEnabled(),
    },
    renderer:{
      framebufferScaleFactor:rendererFbScale??currentFramebufferScale(),
      calls:renderer.info.render.calls,
      triangles:renderer.info.render.triangles,
      lines:renderer.info.render.lines,
      textures:renderer.info.memory.textures,
      geometries:renderer.info.memory.geometries,
    },
  };
  diagnostics.camera=capture.state();
  diagnostics.profile=profile;
  diagnostics.anchor=anchor.state();
  diagnostics.audit=audit.state();
  diagnostics.auditSamples=audit?.recentRows?.(30)??[];
  diagnostics.calibrationData=assistant?.diagnosticsData?.()??null;
  diagnostics.extraTags=[...placedExtras.values()].map(({tagId,edgeMm,lFromTag,placedAt,wp})=>({tagId,edgeMm,lFromTag,placedAt,wp}));
  diagnostics.perimeterFence=perimeterFence?.snapshot()??null;
  diagnostics.anchorCorrection=anchorDriftCorrection.state();
  diagnostics.movableTrails=[...movableTools.values()].map(m=>({
    tagId:m.tagId,
    edgeMm:m.edgeMm,
    waypoints:m.waypointCount,
    segments:m.trailSegmentCount||(m.lineCount>0?1:0),
    gaps:m.trailGaps?.length||0,
    lastMatrix:m.lastMatrix,
  }));
  lastDiagnostics=structuredClone(diagnostics);
  $('download').disabled=false;
  console.info('[webxr-apriltag] session diagnostics:',lastDiagnostics);
}

function clean(s){
  if(session!==s)return;
  finalizeDiagnostics();
  space?.removeEventListener?.('reset',reset);
  assistant?.dispose();assistant=null;capture.stop();anchor.end(performance.now());obsBuffer.clear();anchorDriftCorrection.reset();lastShownUpgradeReason='';tagHandCoupler.reset();tagPredictor.reset();focusScheduler.reset();recentFocusDetectMs=15;handHistory.clear();clearTrail();hideWaypointInspector();hideAllProbeCursors();clearPins();resetSpatialPortals();
  hoveredTlOffer=false;tlOfferPressSince=null;tlOfferGroup.visible=false;fenceGroup.visible=false;fenceRiseStartMs=0;perimeterCompleteAnnounced=false;perimeterOfferSuppressed=false;
  clearFenceBreaches();
  modePillGroup.visible=false;hoveredModePill=false;hoveredModeMain=false;hoveredModeRecAll=false;hoveredModeInfo=false;hoveredModeFence=false;hoveredModeClear=false;hoveredModeColor=false;modePillPokeSince=null;modePillPokeConsumed=false;modePillCooldownUntil=0;lastModePillKey='';modeRecAllPokeSince=null;modeRecAllPokeConsumed=false;modeRecAllCooldownUntil=0;lastModeRecAllKey='';modeFencePokeSince=null;modeFencePokeConsumed=false;modeFenceCooldownUntil=0;lastModeFenceKey='';modeClearPokeSince=null;modeClearPokeConsumed=false;modeClearCooldownUntil=0;lastModeClearKey='';modeColorPokeSince=null;modeColorPokeConsumed=false;modeColorCooldownUntil=0;lastModeColorKey='';turboBarSprite.visible=false;lastTurboBarKey='';
  modeHelpOpen=false;modeHelpSprite.visible=false;modeInfoPokeSince=null;modeInfoPokeConsumed=false;modeInfoCooldownUntil=0;lastModeInfoKey='';lastModeHelpKey='';
  devDockGroup.visible=false;hoveredDevDock=false;resetDevDockPoke();
  session=null;space=null;floorSpace=null;profile=null;extraScanActive=false;recordAllActive=false;lastMotionState=null;lastObs=null;lastObsAtMs=0;reanchorConfirmUntil=0;lightScoreHistory.length=0;lightLast=null;lightLastAt=0;resetAnchorStability();
  hasPrevViewerPose=false;smoothedHeadAngDegS=0;smoothedHeadLinMS=0;appliedFoveation=null;
  hudPanelsVisible=false;hudDragState=null;resetHudCache();axHoldSince=null;axLongFired=false;
  leftFistToggle.reset();leftFistFlashUntil=0;leftFistFlashPos=null;leftFistFlashToScan=false;
  rightFistToggle.reset();rightFistFlashUntil=0;rightFistFlashPos=null;rightFistFlashToRecAll=false;
  leftMidPinchSince=null;leftRingPinchSince=null;leftPinkyPinchSince=null;leftHandCooldownUntil=0;
  rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;rightHandCooldownUntil=0;
  leftPalmDwellSince=null;leftMenuOpenUntil=0;leftWristPrev=null;leftWristSpeed=0;leftDotTapSince=null;
  leftPalmFacingViewer=false;leftMenuOpenNow=false;
  rightPalmDwellSince=null;rightMenuOpenUntil=0;rightWristPrev=null;rightWristSpeed=0;rightDotTapSince=null;
  wristPillPokeId=null;wristPillPokeSince=0;liveTouchLoaderActive=false;
  pinFlashUntilMs=0;pinFlashPos=null;lastInspectorTimeMs=null;lastViewerBasis=null;
  handGestureRing.visible=false;handLabelsGroup.visible=false;leftWristDot.visible=false;rightWristDot.visible=false;wristGazeSprite.visible=false;leftScanRecDot.visible=false;hideHandOccluders();fistProgressSprite.visible=false;
  pinLoadSprite.visible=false;toastSprite.visible=false;toastUntilMs=0;
  root.visible=hudRig.visible=false;
  if($('enter'))$('enter').disabled=!arSupported;
  if($('end')){$('end').disabled=true;$('end').hidden=true;}
  setRigControlsDisabled(false);
  if(aaToggleEl)aaToggleEl.disabled=false;
  status('AR ended. Tap Enter AR to start again.');
}

function reset(){
  epoch++;history=[];obsBuffer.clear();anchorDriftCorrection.reset();lastShownUpgradeReason='';tagHandCoupler.reset();tagPredictor.reset();focusScheduler.reset();recentFocusDetectMs=15;handHistory.clear();resetAnchorStability();clearTrail();
  anchor.resetReference(space,context(),performance.now());
  if(!anchor.state().created){profile=null;extraScanActive=false;clearExtras();assistant.reset();assistant.begin();}
}

async function triggerReanchor(){
  const now=performance.now();
  if(!anchor.reanchor(now,'trigger'))return;
  extraScanActive=false;reanchorConfirmUntil=0;anchorDriftCorrection.reset();resetAnchorStability();
  clearExtras();obsBuffer.clear();audit.reset();lastAuditRow=null;lastMovedTargetAt=0;
  status('Re-anchoring · starting camera…');
  if(!await capture.start())status('Failed to restart camera for re-anchoring.');
}

async function triggerRecalibrate(){
  if(!session||!assistant)return;
  const now=performance.now();
  extraScanActive=false;recordAllActive=false;lastModeRecAllKey='';reanchorConfirmUntil=0;anchorDriftCorrection.reset();lastShownUpgradeReason='';resetAnchorStability();
  rightFistToggle.reset();rightFistFlashUntil=0;leftMidPinchSince=null;
  profile=null;clearExtras();clearPins();clearTrail();obsBuffer.clear();audit.reset();lastAuditRow=null;lastMovedTargetAt=0;
  resetOrbitGuide();
  anchor.reanchor(now,'recalibrate');
  assistant.retry();
  resetHudCache();
  playModeTick('scan');
  showHudToast('↺ RECALIBRATING · ORBIT L','#fbbf24',2400);
  status('Recalibrating · move head around L-bracket…');
  if(!await capture.start())status('Failed to restart camera for recalibration.');
}

async function toggleExtraScan(){
  if(anchor.phase()!=='tracking'&&anchor.phase()!=='created')return;
  const now=performance.now();
  if(extraScanActive){
    extraScanActive=false;recordAllActive=false;lastModeRecAllKey='';history=[];obsBuffer.clear();capture.stop();
    tagHandCoupler.reset();tagPredictor.reset();focusScheduler.reset();recentFocusDetectMs=15;activeFocusTagIds.clear();handHistory.clear();
    liveTouchLoaderActive=false;
    for(const e of movableTools.values()){
      e.pendingTrail.discard();
      resetDisplayErrorOffset(e);
      e.lockedSinceMs=0;e.lastOpticalXrTimeMs=0;
      const grounded=(e.handCoupled&&e.lastOpticalMatrix)?e.lastOpticalMatrix:e.displayMatrix;
      if(e.autoRecording){
        deactivateAutoRecording(e,e.visible&&grounded?grounded:null,now);
        e.lastRecLineMatrix=null;e.lastRecFrameMatrix=null;
      }
      if(e.recording){
        e.recording=false;e.lastRecLineMatrix=null;e.lastRecFrameMatrix=null;
        drawRecOrbPill(e,false,false);
      }
      clearMovableWake(e);
      // Keep only currently visible tags anchored in MEASURE; vanished tags leave no ghost frames (their paths & Time-Copies stay inspectable).
      if(e.visible&&e.displayMatrix){
        const finalMat=(e.lockState==='locked'&&e.lockedMatrix)?e.lockedMatrix:grounded;
        e.lockState='locked';
        e.lockedMatrix=finalMat.slice();
        e.displayMatrix=finalMat.slice();
        e.targetMatrix=finalMat.slice();
        e.lastMatrix=finalMat.slice();
        syncCurrentPose(e,finalMat,now);
      }else{
        e.visible=false;
        e.faceVisible=false;
        e.lockState='settling';
        e.lockedMatrix=null;
        e.displayMatrix=null;
        e.targetMatrix=null;
        e.lastMatrix=null;
        e.currentPose=null;
      }
      e.handCoupled=false;e.coupledHand=null;
      e.recOrbVisible=false;
      recOrbBatch.setVisible(e.recOrbSlot,false);
      badgeMesh.setQuadVisible(e.recPillSlot,false);
      e.markOrbVisible=false;
      recOrbBatch.setVisible(e.markOrbSlot,false);
      badgeMesh.setQuadVisible(e.markPillSlot,false);
      e.markHoldSince=null;e.markHoldConsumed=false;
      e.recHoldSince=null;e.recHoldConsumed=false;
    }
    syncStaticTagBadges();
    lastMeasureTrailFocusKey='';
    syncMeasurementTrailFocus({force:true});
    playModeTick('measure');
    status('MEASURE mode · camera OFF (90 FPS)');
    showHudToast('■ MEASURE MODE · 90 FPS','#22c55e');
  }else{
    extraScanActive=true;obsBuffer.clear();tagHandCoupler.reset();tagPredictor.reset();focusScheduler.reset();recentFocusDetectMs=15;activeFocusTagIds.clear();handHistory.clear();
    lastMeasureTrailFocusKey='';
    syncMeasurementTrailFocus({force:true});
    for(const e of movableTools.values()){
      if(e.visible&&e.currentPose){
        e.lastOpticalAtMs=now;
        e.lastAt=now;
        if(e.lockState==='locked')e.lockedSinceMs=now;
        drawLiveBadge(e,isEntryRecording(e)?'rec':e.lockState,e.handCoupled);
      }
    }
    playModeTick('scan');
    status('SCAN mode ON · Auto-Lock + Follow, Pin icon = Time-Copy, ● = path');
    showHudToast('● SCAN MODE · AUTO-LOCK','#22d3ee');
    if(!await capture.start()){
      extraScanActive=false;
      lastMeasureTrailFocusKey='';
      syncMeasurementTrailFocus({force:true});
      status('Failed to start camera for SCAN mode.');
      showHudToast('SCAN FAILED','#ef4444');
    }
  }
}

function hasActiveMovableTag(now=performance.now()){
  for(const entry of movableTools.values()){
    if((entry.handCoupled||isEntryRecording(entry)||entry.lockState==='moving')&&(entry.handCoupled||isEntryRecording(entry)||now-(entry.lastOpticalAtMs||0)<=2500)){
      return true;
    }
  }
  return false;
}

function buildScanPriorRois(){
  if(!extraScanActive||!profile?.viewerFromCameraGL)return undefined;
  const K=lastObs?.cameraModel||profile?.K;
  if(!K)return undefined;
  let latest=null;
  for(let i=history.length-1;i>=Math.max(0,history.length-8);i--){
    const h=history[i];
    if(h&&!h.emulatedPosition&&h.worldFromAnchor&&h.worldFromViewer){latest=h;break;}
  }

  let rois=[];
  if(latest){
    const farMode=Boolean(lastQualityState.longRangeActive);
    const steadyBoost=lastQualityState.tier!=='fluid'&&!hasActiveMovableTag();
    const maxPriorDist=farMode?3.2:(steadyBoost?2.4:1.85);
    const pointsL=[];
    const locked=[];
    for(const entry of movableTools.values()){
      if(!entry.visible)continue;
      const m=entry.displayMatrix||entry.targetMatrix||entry.lockedMatrix;
      if(!m)continue;
      const pt=[m[12],m[13],m[14]];
      if(entry.handCoupled||isEntryRecording(entry)||entry.lockState!=='locked'){
        pointsL.push({pos:pt,halfPx:68,minDistM:0.12,maxDistM:maxPriorDist});
      }else{
        locked.push({pos:pt,halfPx:52,minDistM:0.40,maxDistM:maxPriorDist});
      }
    }
    if(Number.isFinite(lastXrTime)){
      for(const side of ['left','right']){
        const hs=handHistory.latest(side,lastXrTime,180);
        const hp=hs?.pinchMidL||hs?.palmCenter;
        if(hp)pointsL.push({pos:hp,halfPx:84,minDistM:0.12,maxDistM:1.45});
      }
    }
    for(const item of locked)pointsL.push(item);
    const verts=perimeterFence?.orderedVertices?.()??[];
    for(const v of verts){
      if(v?.pos)pointsL.push({pos:v.pos,halfPx:52,minDistM:0.55,maxDistM:maxPriorDist});
    }
    if(pointsL.length){
      rois=projectPriorRois({
        pointsL,
        worldFromAnchor:latest.worldFromAnchor,
        worldFromViewer:latest.worldFromViewer,
        viewerFromCameraGL:profile.viewerFromCameraGL,
        K,
        width:K.width,
        height:K.height,
        maxRois:4,
      });
    }
  }

  if(smoothedHeadAngDegS<22&&(scanPulseCounter%4===3)){
    rois.push([
      Math.max(0,Math.round(K.cx-220)),
      Math.max(0,Math.round(K.cy-180)),
      Math.min(K.width,Math.round(K.cx+220)),
      Math.min(K.height,Math.round(K.cy+180)),
    ]);
  }
  return rois.length?rois:undefined;
}

function syncFocusLiveBadges(){
  if(!extraScanActive)return;
  for(const e of movableTools.values()){
    if(!e.visible)continue;
    drawLiveBadge(e,isEntryRecording(e)?'rec':e.lockState,e.handCoupled);
  }
}

function setsEqual(a,b){
  if(a.size!==b.size)return false;
  for(const v of a)if(!b.has(v))return false;
  return true;
}

function buildScanFocusRois(){
  if(!extraScanActive||!profile?.viewerFromCameraGL){
    return {mode:'wide',rois:undefined,tagIds:[],activeTagIds:[],searchingTagIds:[],speedFirst:false,isBackgroundSweep:false,sweepDecimate:2,skipReason:'inactive'};
  }
  const K=lastObs?.cameraModel||profile?.K;
  if(!K){
    return {mode:'wide',rois:undefined,tagIds:[],activeTagIds:[],searchingTagIds:[],speedFirst:false,isBackgroundSweep:false,sweepDecimate:2,skipReason:'no-camera'};
  }
  let latest=null;
  for(let i=history.length-1;i>=Math.max(0,history.length-8);i--){
    const h=history[i];
    if(h&&!h.emulatedPosition&&h.worldFromAnchor&&h.worldFromViewer){latest=h;break;}
  }
  if(!latest){
    return {mode:'wide',rois:undefined,tagIds:[],activeTagIds:[],searchingTagIds:[],speedFirst:false,isBackgroundSweep:false,sweepDecimate:2,skipReason:'no-xr-pose'};
  }

  const now=performance.now();
  const farMode=Boolean(lastQualityState.longRangeActive);
  const tRoi=Number.isFinite(lastXrTime)?lastXrTime+(profile?.timeOffsetMs??-45):null;
  const viewerAtRoi=(tRoi!=null?poseAt(history,tRoi):null)||latest.worldFromViewer;
  const candidates=[];

  for(const entry of movableTools.values()){
    const freshOptical=entry.lastOpticalAtMs>0&&(now-entry.lastOpticalAtMs)<=1800;
    if(!entry.visible&&!(entry.displayMatrix&&freshOptical))continue;
    const moving=entry.lockState==='moving';
    const settling=entry.lockState==='settling';
    const locked=entry.lockState==='locked';
    const baseMat=entry.lockedMatrix||entry.displayMatrix||entry.targetMatrix;
    if(!baseMat)continue;
    const nearHand=locked&&!isEntryRecording(entry)&&!entry.handCoupled&&Number.isFinite(lastXrTime)&&isTagNearAnyHand(baseMat,lastXrTime,0.25);

    let predMat=null;
    if(entry.handCoupled&&tRoi!=null){
      predMat=tagHandCoupler.predictAt(entry.tagId,handHistory,tRoi);
    }else if(moving&&tRoi!=null){
      predMat=tagPredictor.predict(entry.tagId,tRoi);
    }

    let pos;
    if(predMat){
      pos=[predMat[12],predMat[13],predMat[14]];
    }else if(entry.handCoupled&&entry.lastOpticalMatrix&&entry.displayMatrix){
      const om=entry.lastOpticalMatrix,dm=entry.displayMatrix;
      pos=Math.hypot(dm[12]-om[12],dm[13]-om[13],dm[14]-om[14])<=0.30
        ?[om[12]*0.45+dm[12]*0.55,om[13]*0.45+dm[13]*0.55,om[14]*0.45+dm[14]*0.55]
        :[dm[12],dm[13],dm[14]];
    }else{
      const m=entry.displayMatrix||entry.targetMatrix||entry.lockedMatrix;
      pos=[m[12],m[13],m[14]];
    }

    const releaseBoost=entry.postReleaseBoostUntilMs>0&&now<=entry.postReleaseBoostUntilMs;
    const spd=Math.max(entry.lastSpeedMps||0,tagPredictor.speedOf(entry.tagId)?.speedMps||0,releaseBoost?(entry.releaseSpeedMps||0.35):0);
    const baseHalfPx=Math.round((entry.lastApparentEdgePx||44)*1.9+Math.min(115,spd*130));
    const halfPx=Math.min(220,Math.max(releaseBoost?140:96,baseHalfPx+(releaseBoost?40:0)));
    candidates.push({
      tagId:entry.tagId,
      pos,
      halfPx,
      minDistM:0.10,
      maxDistM:farMode?2.50:1.85,
      recording:isEntryRecording(entry),
      handCoupled:entry.handCoupled,
      moving,
      settling,
      nearHand,
      locked,
      stillDurationMs:locked&&entry.lockedSinceMs>0?Math.max(0,now-entry.lockedSinceMs):0,
      lastOpticalAtMs:entry.lastOpticalAtMs||0,
      speedMps:spd,
      apparentEdgePx:entry.lastApparentEdgePx||0,
    });
  }

  return focusScheduler.plan({
    candidates,
    worldFromAnchor:latest.worldFromAnchor,
    worldFromViewer:viewerAtRoi,
    viewerFromCameraGL:profile.viewerFromCameraGL,
    K,
    width:K.width,
    height:K.height,
    nowMs:now,
    farMode,
    recentDetectMs:recentFocusDetectMs,
    maxForegroundTags:recordAllActive?2:3,
  });
}

try{
  const [rigA4,rigTablet]=await Promise.all([
    fetch(new URL('../rigs/ms-qr-main-l.json',import.meta.url)).then(r=>r.json()),
    fetch(new URL('../rigs/ms-qr-tablet-l.json',import.meta.url)).then(r=>r.json()),
  ]);
  rigIdsByKey={a4:rigA4.tools[0].tags.map(t=>t.id),tablet:rigTablet.tools[0].tags.map(t=>t.id)};
  const rigSelect=$('rigSelect');
  selectedRigKey=urlParams.get('rig')==='tablet'?'tablet':'a4';
  function syncRigUi(){
    if(rigSelect)rigSelect.value=selectedRigKey;
    for(const b of rigButtons){
      const active=b.dataset.rig===selectedRigKey;
      b.classList.toggle('active',active);
      b.setAttribute('aria-pressed',active?'true':'false');
    }
  }
  syncRigUi();
  rig=selectedRigKey==='tablet'?rigTablet:rigA4;
  buildReferenceMesh(rig);
  perimeterFence=PERIMETER_ENABLED?createPerimeterFence(rig,{heightM:FENCE_HEIGHT_M,coordinateMode:selectedRigKey==='tablet'?'free-6dof':'planar-z0'}):null;
  syncFenceVisuals(performance.now(),0,true);
  anchor.setTagIds(rig.tools[0].tags.map(t=>t.id));
  capture=createReferenceCamera({
    video:$('video'),geometry:rig,getContext:context,
    getSettings:()=>{
      const asPhase=assistant?.phase?.()??'idle';
      const anPhase=anchor.phase();
      const calibrating=asPhase!=='ready';
      const locking=!calibrating&&(anPhase==='collecting'||anPhase==='ready');
      const q=lastQualityState;
      if(calibrating){
        return {
          horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:profile,
          decimate:1,maxHz:8,detectMode:'two-stage',trackRefIds:rig.tools[0].tags.map(t=>t.id),
        };
      }
      if(locking){
        return {
          horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:profile,
          decimate:1,maxHz:8,detectMode:'two-stage',trackRefIds:null,solveUnassigned:Boolean(profile),
        };
      }
      const fast=Boolean(extraScanActive&&$('fastScan')?.checked);
      if(!fast){
        scanPulseCounter=0;focusScheduler.reset();
        if(activeFocusTagIds.size){activeFocusTagIds.clear();syncFocusLiveBadges();}
      }
      const activeMoving=fast&&hasActiveMovableTag();
      const focusPlan=fast?buildScanFocusRois():{mode:'wide',rois:undefined,tagIds:[],activeTagIds:[],searchingTagIds:[],speedFirst:false,isBackgroundSweep:false,sweepDecimate:2,skipReason:'fast-off'};
      const focusRois=focusPlan.rois;
      const priorRois=(fast&&!focusRois&&!focusPlan.isBackgroundSweep&&!activeMoving)?buildScanPriorRois():undefined;
      const effDecimate=focusRois?2:(focusPlan.isBackgroundSweep?focusPlan.sweepDecimate:(focusPlan.skipReason==='crowd-wide'?2:(activeMoving?3:(fast?q.scanDecimate:1))));
      const effMaxHz=focusRois?60:(focusPlan.isBackgroundSweep?60:(activeMoving?30:(fast?Math.max(12,q.scanMaxHz):5)));
      return {
        horizontalFovDeg:65,printScale:1,maxLongEdge:1280,experimentalCalibration:profile,
        decimate:effDecimate,maxHz:effMaxHz,detectMode:fast?q.detectMode:'full',trackRefIds:null,solveUnassigned:Boolean(profile),
        priorRois,focusRois,
        ...(focusRois?{focusSpeedFirst:focusPlan.speedFirst,focusFallbackDecimate:3}:null),
        focusMode:focusPlan.mode,
        focusIsBackgroundSweep:focusPlan.isBackgroundSweep,
        focusSearchingTagIds:focusPlan.searchingTagIds,
        focusTagIds:focusPlan.tagIds,
        focusActiveTagIds:focusPlan.activeTagIds,
        focusSkipReason:focusPlan.skipReason,
      };
    },
    onError:e=>{diagnostics?.errors.push({at:performance.now(),message:e.message});status(e.message);},
    onObservation:(observation,sample)=>{
      if(!session)return;
      const now=performance.now();
      lastObs=observation;lastObsAtMs=now;
      if(sample.light)pushLightSample(sample.light,now); // lighting metric (~5 Hz)
      if(rigSwitching)return;
      if(maybeAutoSwitchRig(observation))return;
      if(latestViewMat)tagHintViewMat=latestViewMat.slice();
      const lTool=observation.tools[0];
      if(lTool&&Number.isFinite(lTool.reprojectionRmsPx)){
        rmsHistory.push(lTool.reprojectionRmsPx);
        if(rmsHistory.length>60)rmsHistory.shift();
      }
      const cap=sample.videoFrameMetadata?.captureTime;
      const lag=Number.isFinite(cap)?sample.submittedAtPerformanceMs-cap:null;
      const curPhase=assistant?.phase?.()??'idle';
      const sampleFocusTagIds=Array.isArray(sample.settings?.focusTagIds)?sample.settings.focusTagIds:[];
      const sampleFocusMode=sample.settings?.focusMode??'wide';
      const sampleIsBgSweep=Boolean(sample.settings?.focusIsBackgroundSweep);
      const sampleSearchingIds=Array.isArray(sample.settings?.focusSearchingTagIds)?sample.settings.focusSearchingTagIds:[];
      lastSampleStats={
        serial:sample.serial,decimate:sample.decimate,detectMode:sample.detectMode??null,
        coarseMs:sample.coarseMs??null,roiMs:sample.roiMs??null,roiCount:sample.roiCount??null,priorRoiCount:sample.priorRoiCount??0,
        focusRoiCount:sample.focusRoiCount??0,focusHit:Boolean(sample.focusHit),focusMissedBoxes:sample.focusMissedBoxes??0,
        focusTagIds:sampleFocusTagIds,focusMode:sampleFocusMode,isBackgroundSweep:sampleIsBgSweep,
        searchingTagIds:sampleSearchingIds,focusSpeedFirst:Boolean(sample.settings?.focusSpeedFirst),
        focusSkipReason:sample.settings?.focusSkipReason??null,
        detectMs:sample.detectMs,readbackMs:sample.mainThreadReadbackMs,
        preprocessMs:sample.workerPreprocessMs,poseFitMs:sample.poseFitMs,captureLagMs:lag,capturedInRvfc:sample.capturedInRvfc,
        readbackMode:sample.readbackMode,metadataMediaTimeDeltaS:sample.metadataMediaTimeDeltaS,
        mode:curPhase!=='ready'?'calibrating':(extraScanActive?'scan':'measure'),
      };
      if(extraScanActive){
        const isFocusFrame=(sample.focusRoiCount??0)>0;
        const seenRawIds=new Set((observation.unassignedDetections||[]).map(d=>d.id));
        if(isFocusFrame&&sample.focusHit&&Number.isFinite(sample.detectMs)){
          recentFocusDetectMs=recentFocusDetectMs*0.65+sample.detectMs*0.35;
        }
        focusScheduler.reportObservation({
          seenIds:seenRawIds,
          focusTagIds:sampleFocusTagIds,
          isFocusFrame,
          isBackgroundSweep:sampleIsBgSweep,
          focusHit:Boolean(sample.focusHit),
          nowMs:now,
        });
        let nextFocusIds;
        if(sampleFocusMode==='wide'){
          nextFocusIds=new Set();
        }else if(isFocusFrame&&sample.focusHit){
          nextFocusIds=new Set(sampleFocusTagIds.filter(id=>seenRawIds.has(id)));
        }else if(sampleIsBgSweep){
          nextFocusIds=new Set(sampleFocusTagIds.filter(id=>seenRawIds.has(id)||activeFocusTagIds.has(id)));
        }else{
          nextFocusIds=new Set();
        }
        if(!setsEqual(activeFocusTagIds,nextFocusIds)){
          activeFocusTagIds=nextFocusIds;
          syncFocusLiveBadges();
        }
        if(!isFocusFrame&&smoothedHeadAngDegS<22)scanPulseCounter++;
        else if(smoothedHeadAngDegS>=22)scanPulseCounter=0;
      }else{
        focusScheduler.reset();recentFocusDetectMs=15;scanPulseCounter=0;
        if(activeFocusTagIds.size)activeFocusTagIds.clear();
      }
      if(diagnostics){
        if(diagnostics.samples.length<340)diagnostics.samples.push(lastSampleStats);
        else{diagnostics.samples.splice(40,1);diagnostics.samples.push(lastSampleStats);}
      }
      const prevState=assistant?.summary();
      const nextState=assistant?.observe(observation,sample,history,now);
      if(nextState){
        if(nextState.phase!==lastCalPhase){
          if(nextState.phase==='ready')clearTrail();
          else if(nextState.phase==='scanning')clearTrail();
          lastCalPhase=nextState.phase;lastCalCount=0;
        }
        if((nextState.phase==='scanning'||nextState.phase==='verifying'||nextState.phase==='checking')&&nextState.count>(prevState?.count??0)){
          const latest=history.at(-1)?.worldFromViewer;
          if(latest){calViewerPoses.push(latest.slice());addTrailDot(latest,nextState.phase);}
          lastCalCount=nextState.count;
        }
      }
      if(!profile)return;
      obsBuffer.push({observation,sample});
    },
  });

  // Auto rig detection: the tag IDs identify the angle uniquely (A4: 350-352, small: 370-372),
  // so as soon as the other angle's IDs are seen (and none of the selected one's), switch to it.
  let rigSwitching=false,autoRigVotes=0,autoRigVoteKey=null;
  function maybeAutoSwitchRig(observation){
    if(!session||!assistant||!anchor||anchor.state().created)return false;
    const sum=assistant.summary?.();
    if(!sum||sum.count>0||(sum.phase!=='scanning'&&sum.phase!=='checking'))return false;
    const mine=new Set(rigIdsByKey[selectedRigKey]);
    if((observation.tags??[]).some(t=>mine.has(t.id))){autoRigVotes=0;autoRigVoteKey=null;return false;}
    const otherKey=selectedRigKey==='tablet'?'a4':'tablet';
    const other=new Set(rigIdsByKey[otherKey]);
    const hit=(observation.unassignedDetections??[]).some(t=>other.has(t.id));
    if(!hit){autoRigVotes=0;autoRigVoteKey=null;return false;}
    if(autoRigVoteKey!==otherKey){autoRigVoteKey=otherKey;autoRigVotes=0;}
    if(++autoRigVotes<3)return false;
    autoRigVotes=0;autoRigVoteKey=null;
    switchRigLive(otherKey);
    return true;
  }
  async function switchRigLive(nextKey){
    if(rigSwitching)return;
    rigSwitching=true;
    try{
      selectedRigKey=nextKey;syncRigUi();
      rig=nextKey==='tablet'?rigTablet:rigA4;
      profile=null;anchorDriftCorrection.reset();lastShownUpgradeReason='';
      buildReferenceMesh(rig);
      perimeterFence?.setCoordinateMode?.(nextKey==='tablet'?'free-6dof':'planar-z0');
      perimeterFence?.setRig(rig);
      syncFenceVisuals(performance.now(),0,true);
      anchor.setTagIds(rig.tools[0].tags.map(t=>t.id));
      capture.setGeometry(rig);
      const old=assistant;
      assistant=null;
      old?.dispose();
      const device=await deviceIdentity(capture.state(),rig),[w,h]=capture.state().sourceSize,scale=Math.min(1,1280/Math.max(w,h));
      if(!session)return;
      assistant=createCalibrationAssistant({
        rig,device,imageSize:[Math.round(w*scale),Math.round(h*scale)],
        model:'centered-radial',
        onChange:handleCalibrationStatus,
        onProfile:p=>{profile=p;if(p)clearTrail();},
      });
      resetOrbitGuide();
      await assistant.begin();
      const label=nextKey==='tablet'?'Smaller Angle':'DIN A4 Angle';
      showHudToast(`✓ ${label} erkannt (${rigIdsByKey[nextKey].join('/')})`,'#4ade80',2200);
    }finally{rigSwitching=false;}
  }

  function applyRigSelection(nextKey){
    selectedRigKey=nextKey==='tablet'?'tablet':'a4';
    syncRigUi();
    if(capture.isRunning?.())capture.stop();
    if(assistant){assistant.dispose();assistant=null;}
    profile=null;anchorDriftCorrection.reset();lastShownUpgradeReason='';
    rig=selectedRigKey==='tablet'?rigTablet:rigA4;
    buildReferenceMesh(rig);
    perimeterFence?.setCoordinateMode?.(selectedRigKey==='tablet'?'free-6dof':'planar-z0');
    perimeterFence?.setRig(rig);
    syncFenceVisuals(performance.now(),0,true);
    anchor.setTagIds(rig.tools[0].tags.map(t=>t.id));
    capture.setGeometry(rig);
    const ids=rig.tools[0].tags.map(t=>t.id).join(' / ');
    const label=selectedRigKey==='tablet'?'Smaller Angle':'Printed DIN A4 Angle';
    status(`Target angle: ${label} (IDs ${ids}). Tap Enter AR.`);
  }
  for(const b of rigButtons){
    b.onclick=()=>{if(!session&&!pending)applyRigSelection(b.dataset.rig);};
  }
  if(rigSelect){
    rigSelect.onchange=()=>{if(!session&&!pending)applyRigSelection(rigSelect.value);};
  }

  async function queryCameraPermission(){
    try{
      if(!navigator.permissions?.query)return 'unknown';
      const perm=await navigator.permissions.query({name:'camera'});
      cameraPermState=perm.state;
      perm.onchange=()=>{cameraPermState=perm.state;};
      return perm.state;
    }catch{return 'unknown';}
  }

  async function ensureCameraAndAssistant(){
    if(capture.isRunning?.()&&assistant)return true;
    profile=null;
    if(!await capture.start())return false;
    const device=await deviceIdentity(capture.state(),rig),[w,h]=capture.state().sourceSize,scale=Math.min(1,1280/Math.max(w,h));
    assistant?.dispose();
    assistant=createCalibrationAssistant({
      rig,device,imageSize:[Math.round(w*scale),Math.round(h*scale)],
      model:'centered-radial',
      onChange:handleCalibrationStatus,
      onProfile:p=>{profile=p;if(p)clearTrail();},
    });
    return true;
  }

  await capture.prepare();
  arSupported=Boolean(navigator.xr)&&await navigator.xr.isSessionSupported('immersive-ar').catch(()=>false);
  cameraPermState=await queryCameraPermission();
  if($('enter'))$('enter').disabled=!arSupported;
  if(!arSupported){
    status('Immersive AR is unavailable in this browser. Open in Meta Quest Browser.');
  }else if(cameraPermState==='denied'){
    status('Camera access is blocked. Allow camera permission in browser settings and reload.');
  }else if(cameraPermState==='granted'){
    status('Ready. Tap Enter AR.');
  }else{
    status('Ready. Tap Enter AR (browser will ask for camera permission on first launch).');
  }

  $('download').onclick=()=>{
    if(!lastDiagnostics)return;
    const blob=new Blob([JSON.stringify(lastDiagnostics,null,2)],{type:'application/json'});
    const a=document.createElement('a');
    a.href=URL.createObjectURL(blob);
    a.download=`webxr-apriltag-diag-${new Date().toISOString().replace(/[:.]/g,'-')}.json`;
    a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);
  };

  $('enter').onclick=async()=>{
    if(session||pending||!arSupported)return;pending=true;let acquired=null,earlyEnded=false;
    if($('enter'))$('enter').disabled=true;
    setRigControlsDisabled(true);
    if(aaToggleEl)aaToggleEl.disabled=true;
    try{
      ensureRenderer();
      const canEnterSync=cameraPermState==='granted'||Boolean(capture.isRunning?.()&&assistant);
      if(canEnterSync){
        acquired=await navigator.xr.requestSession('immersive-ar',{optionalFeatures:['anchors','layers','hand-tracking','local-floor']});
        acquired.addEventListener('end',()=>{earlyEnded=true;queueMicrotask(()=>clean(acquired));},{once:true});
        status('Starting camera…');
        if(!await ensureCameraAndAssistant()||earlyEnded){
          capture.stop();assistant?.dispose();assistant=null;
          await acquired.end().catch(()=>{});
          if($('enter'))$('enter').disabled=!arSupported;
          setRigControlsDisabled(false);
          if(aaToggleEl)aaToggleEl.disabled=false;
          if(earlyEnded)status('AR session ended before camera started.');
          return;
        }
      }else{
        status('Allow camera access to continue…');
        if(!await ensureCameraAndAssistant()){
          if($('enter'))$('enter').disabled=!arSupported;
          setRigControlsDisabled(false);
          if(aaToggleEl)aaToggleEl.disabled=false;
          return;
        }
        cameraPermState='granted';
        try{
          acquired=await navigator.xr.requestSession('immersive-ar',{optionalFeatures:['anchors','layers','hand-tracking','local-floor']});
          acquired.addEventListener('end',()=>{earlyEnded=true;queueMicrotask(()=>clean(acquired));},{once:true});
        }catch(xrErr){
          if(xrErr?.name==='SecurityError'||xrErr?.name==='NotAllowedError'){
            if($('enter'))$('enter').disabled=false;
            setRigControlsDisabled(false);
            if(aaToggleEl)aaToggleEl.disabled=false;
            status('Camera ready — tap Enter AR.');
            return;
          }
          throw xrErr;
        }
      }

      session=acquired;sessionId=`quest-${++sessionCount}`;epoch=0;history=[];obsBuffer.clear();anchorDriftCorrection.reset();lastShownUpgradeReason='';clearExtras();clearTrail();resetSpatialPortals();audit.reset();
      hold={sinceMs:null,progress:0,fire:false};extraScanActive=false;recordAllActive=false;reanchorConfirmUntil=0;scanPulseCounter=0;recentFocusDetectMs=15;tagPredictor.reset();focusScheduler.reset();
      adaptiveQuality.setMode(currentQualityMode());
      adaptiveQuality.setLongRange(currentLongRangeEnabled());
      adaptiveQuality.reset();
      appliedFoveation=null;lastQualityState=adaptiveQuality.state();
      hasPrevViewerPose=false;smoothedHeadAngDegS=0;smoothedHeadLinMS=0;leftMidPinchSince=null;rightMidPinchSince=null;perimeterOfferSuppressed=false;
      trailColorMode=currentTrailColorMode();
      lastModePillKey='';modePillGroup.visible=false;hoveredModePill=false;hoveredModeMain=false;hoveredModeRecAll=false;hoveredModeInfo=false;hoveredModeFence=false;hoveredModeClear=false;hoveredModeColor=false;modePillPokeSince=null;modePillPokeConsumed=false;modePillCooldownUntil=0;modeRecAllPokeSince=null;modeRecAllPokeConsumed=false;modeRecAllCooldownUntil=0;lastModeRecAllKey='';modeFencePokeSince=null;modeFencePokeConsumed=false;modeFenceCooldownUntil=0;lastModeFenceKey='';modeClearPokeSince=null;modeClearPokeConsumed=false;modeClearCooldownUntil=0;lastModeClearKey='';modeColorPokeSince=null;modeColorPokeConsumed=false;modeColorCooldownUntil=0;lastModeColorKey='';turboBarSprite.visible=false;lastTurboBarKey='';
      modeHelpOpen=false;modeHelpSprite.visible=false;modeInfoPokeSince=null;modeInfoPokeConsumed=false;modeInfoCooldownUntil=0;lastModeInfoKey='';lastModeHelpKey='';
      devDockGroup.visible=false;hoveredDevDock=false;resetDevDockPoke();
      hudPanelsVisible=false;hudDragState=null;resetHudCache();perfTracker.reset();
      leftFistToggle.reset();leftFistFlashUntil=0;leftFistFlashPos=null;leftFistFlashToScan=false;
      rightFistToggle.reset();rightFistFlashUntil=0;rightFistFlashPos=null;rightFistFlashToRecAll=false;
      lastXrTime=null;prevXrTime=null;fpsHistory=[];stabilityHistory=[];rmsHistory=[];lightScoreHistory.length=0;lightLast=null;lightLastAt=0;maxFrameGapMs=0;resetAnchorStability();
      lastAuditRow=null;lastMovedTargetAt=0;lastMotionState=null;lastSampleStats=null;lastObs=null;lastObsAtMs=0;lastCalPhase='idle';lastCalCount=0;
      rvfcWindowStart=performance.now();rvfcWindowCount=capture.presentedCallbacks?.()??capture.state().presentedCallbacks;rvfcHz=0;
      diagnostics={appVersion:APP_VERSION_LABEL,startedAt:new Date().toISOString(),userAgent:navigator.userAgent,sessionId,rigId:rig.id,samples:[],events:[],errors:[]};

      await renderer.xr.setSession(acquired);
      if(session!==acquired)return;
      warmupSceneShaders(renderer,scene,camera3d);
      try{renderer.xr.setFoveation?.(lastQualityState.foveation);appliedFoveation=lastQualityState.foveation;}catch{}
      space=renderer.xr.getReferenceSpace();space.addEventListener('reset',reset);
      floorSpace=await acquired.requestReferenceSpace('local-floor').catch(()=>null);
      anchor.begin(acquired,space,context(),performance.now());
      await assistant.begin();
      if($('enter'))$('enter').disabled=true;
      if($('end')){$('end').hidden=false;$('end').disabled=false;}

      acquired.addEventListener('select',(ev)=>{
        const asPhase=assistant?.phase?.()??'idle',anPhase=anchor.phase(),now=performance.now();
        // Priority -1: Explicit 2-step "Leave Demo?" confirmation modal (guards eb.mess.icu & showcase URLs).
        if(leaveConfirmState){
          if((now-leaveConfirmState.openedAt)>=LEAVE_SELECT_GRACE_MS&&leaveConfirmHover)executeLeaveConfirm(leaveConfirmHover,now);
          return;
        }
        if(asPhase==='check-failed'){assistant.retryCheck();return;}
        if(asPhase!=='ready')return;
        if(anPhase==='tracking'||anPhase==='created'){
          // Priority 0: Pinch/Trigger on the open mess.engineering showcase panel (close [×] or clickable mess.engineering / wetter.mess.engineering cards).
          if(showcaseOpen&&showcaseHoverZone){triggerShowcaseAction(showcaseHoverZone,now);return;}
          // Priority 1: Pinch/Trigger on a pinned card's red [×] close orb removes that pin.
          if(hoveredClosePinKey){unpinWaypoint(hoveredClosePinKey);return;}
          // Priority 1b: Pinch/Trigger while hovering the anchored Origin Bar 2-row dock buttons.
          if(hoveredModeInfo){toggleModeHelp(now);return;}
          if(hoveredModeRecAll){toggleRecordAll();return;}
          if(hoveredModeFence){togglePerimeterFence(now);return;}
          if(hoveredModeClear&&userItemCount()>0){clearAllUserItems();return;}
          if(hoveredModeColor){toggleTrailColorMode();return;}
          if(hoveredModeMain){toggleExtraScan();return;}
          // Priority 2: Controller Trigger pins/unpins immediately with ✓ confirmation burst; bare-hand pinch uses the 350 ms radial loader.
          if(hoveredWaypoint){
            if(ev?.inputSource?.hand)return;
            const [cx,cy,cz]=hoveredWaypoint.centerL;
            const flashPos=root.visible?lPointToWorld(root.matrix.elements,cx,cy,cz):null;
            togglePinWaypoint(hoveredWaypoint,hoveredFloorHeightMm,flashPos);
            hoveredWaypointKey=null;
            return;
          }
          // Priority 3: Pinch/Trigger while hovering the TL (#353) perimeter activation prompt.
          if(hoveredTlOffer&&perimeterFence?.isOffered()&&!perimeterOfferSuppressed){
            activatePerimeterFromTL(now);
            return;
          }
          // Priority 4: Pinch/Trigger while hovering an anchored L-bracket spatial portal (QR -> eb.mess.icu, SETUP LABEL -> Donner).
          if(hoveredPortal&&!portalWarpState){
            triggerSpatialPortal(hoveredPortal,now);
            return;
          }
          // Ignore bare-hand pinch in open air so picking up a physical AprilTag never triggers re-anchor.
          if(ev?.inputSource?.hand)return;
          if(extraScanActive)return;
          if(now<reanchorConfirmUntil)triggerReanchor();
          else reanchorConfirmUntil=now+1500;
          return;
        }
        if(anPhase==='lost'||anPhase==='failed'){triggerReanchor();return;}
      });
      acquired.addEventListener('squeezestart',()=>{
        if(leaveConfirmState)return;
        const asPhase=assistant?.phase?.()??'idle',anPhase=anchor.phase();
        if(asPhase==='check-failed'){assistant.retry();return;}
        if(asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created'))toggleExtraScan();
      });
    }catch(e){
      if(acquired){
        await acquired.end().catch(()=>{});
        clean(acquired);
      }
      if(!session){
        capture.stop();assistant?.dispose();assistant=null;
        if($('enter'))$('enter').disabled=!arSupported;
        if($('end')){$('end').disabled=true;$('end').hidden=true;}
        setRigControlsDisabled(false);
        if(aaToggleEl)aaToggleEl.disabled=false;
      }
      status(e.message);
    }finally{pending=false;}
  };

  $('end').onclick=end;

  xrFrameHandler=(time,frame)=>{
    const now=performance.now();
    if(!session||!frame||!space)return;
    capture.tick(now);

    let dtSec=0.011;
    let frameDtMs=null;
    if(lastXrTime!=null){
      const dt=time-lastXrTime;
      if(dt>0&&dt<2000){
        frameDtMs=dt;
        dtSec=dt/1000;
        if(dt>maxFrameGapMs)maxFrameGapMs=dt;
        fpsHistory.push(1000/dt);if(fpsHistory.length>60)fpsHistory.shift();
      }
    }
    prevXrTime=lastXrTime;lastXrTime=time;

    const pose=frame.getViewerPose(space);
    const viewerTracked=Boolean(pose&&!pose.emulatedPosition);
    if(viewerTracked&&pose?.transform?.matrix){
      const vm=pose.transform.matrix;
      if(hasPrevViewerPose&&now>prevViewerPoseMs){
        const vDt=Math.max(0.004,(now-prevViewerPoseMs)/1000);
        const instAng=rotDeltaDeg(_prevViewerPoseBuf,vm)/vDt;
        const instLin=Math.hypot(vm[12]-_prevViewerPoseBuf[12],vm[13]-_prevViewerPoseBuf[13],vm[14]-_prevViewerPoseBuf[14])/vDt;
        smoothedHeadAngDegS=smoothedHeadAngDegS*0.65+instAng*0.35;
        smoothedHeadLinMS=smoothedHeadLinMS*0.65+instLin*0.35;
      }else{
        smoothedHeadAngDegS=0;smoothedHeadLinMS=0;
      }
      for(let i=0;i<16;i++)_prevViewerPoseBuf[i]=vm[i];
      hasPrevViewerPose=true;prevViewerPoseMs=now;
      const avgFps=fpsHistory.length>=10?fpsHistory.reduce((a,b)=>a+b,0)/fpsHistory.length:null;
      lastQualityState=adaptiveQuality.update({angDegPerSec:smoothedHeadAngDegS,linMPerSec:smoothedHeadLinMS,nowMs:now,xrFps:avgFps});
      if(lastQualityState.foveation!==appliedFoveation){
        appliedFoveation=lastQualityState.foveation;
        try{renderer.xr.setFoveation?.(appliedFoveation);}catch{}
      }
    }
    const world=anchor.update(frame,now,{viewerTracked});
    const anSummary=anchor.summary();
    const anPhase=anSummary.phase;
    const asPhase=assistant?.phase?.()??'idle';

    const needHistory=asPhase!=='ready'||extraScanActive||anPhase==='collecting'||anPhase==='ready';
    if(needHistory){
      history.push({
        ...context(),xrRafTimeMs:time,emulatedPosition:Boolean(pose?.emulatedPosition),
        worldFromViewer:viewerTracked?Array.from(pose.transform.matrix):null,
        worldFromAnchor:world,
        anchorTrackingState:anPhase,
      });
      if(asPhase!=='ready'){
        if(history.length>12500)history.splice(0,500);
      }else if(history.length>360){
        history.splice(0,history.length-260);
      }
    }

    const corr=(world&&selectedRigKey==='a4')?anchorDriftCorrection.step(dtSec):null;
    const renderWorld=(world&&corr?.active)?product(world,corr.matrix):world;
    const frameInputs=collectFrameInputs(frame,renderWorld,pose);
    if(renderWorld)recordHandsInL(frameInputs,renderWorld,time);

    if(profile){
      const drained=obsBuffer.drain(history,profile,now);
      if(anPhase==='collecting'||anPhase==='ready'){
        for(const item of drained){
          const tool=item.observation.tools[0];
          const lFromCamera=(tool&&tool.tagIds.length>=2&&tool.reprojectionRmsPx<=2.5)?inverse(cvToGL(tool.cameraFromTool)):null;
          if(lFromCamera)observePerimeterFromL(item.observation,lFromCamera,item.sample.submittedAtPerformanceMs);
          anchor.observe({
            sample:item.sample,tool,candidate:item.candidate,
            worldFromL:tool?product(item.candidate.worldFromViewer,profile.viewerFromCameraGL,cvToGL(tool.cameraFromTool)):null,
          },now);
        }
      }else if(extraScanActive&&(anPhase==='tracking'||anPhase==='created')&&drained.length){
        // Cap live-scan backlog drain to latest <= 2 frames so temporary pauses never dump a multi-frame burst at once.
        const recent=drained.length>2?drained.slice(-2):drained;
        for(const item of recent){
          const assoc=associateAnchorCapture(history,item.sample,profile,now);
          if(assoc){
            observeExtraTags(item.observation,item.sample,assoc);
            const row=auditAnchorObservation(item.observation,item.sample,assoc,rig);
            if(row){
              if(row.positionDifferenceMm>50||row.orientationDifferenceDeg>10){
                lastMovedTargetAt=now;
                lastAuditRow=null;
              }else{
                audit.observe(row);lastAuditRow=row;lastAuditAt=now;
              }
            }
          }
        }
      }
    }

    root.visible=Boolean(renderWorld);
    if(renderWorld){
      root.matrix.fromArray(renderWorld);
      root.matrixWorldNeedsUpdate=true;
      syncFenceVisuals(now,dtSec);
    }
    recordAnchorStability(world);
    stepLiveTagSmoothing(now,dtSec,time);
    probeProximity.clear();
    hudRayDot.visible=false;
    hudRayDotMat.color.setHex(0xfde047);
    hudRayDot.scale.setScalar(1);
    updateLiveTagTouch(frameInputs,renderWorld,pose,now);
    updateWaypointInspector(frameInputs,renderWorld,pose,now);
    syncMeasurementTrailFocus();
    updateTlPerimeterOffer(frameInputs,renderWorld,anPhase,asPhase,now);
    updateModeAnchorPill(frameInputs,renderWorld,anPhase,asPhase,now);
    updateDevDock(frameInputs,renderWorld,anPhase,asPhase,now);
    updateSpatialPortals(frameInputs,renderWorld,anPhase,asPhase,now,pose);
    updateLeaveConfirm(frameInputs,pose,now,renderWorld);

    // Controller A/X (button 4): short press toggles HUD panels; hold 0.8 s clears all user tags, trails & pins.
    const axPressed=frameInputs.axPressed;
    if(axPressed){
      if(axHoldSince==null){axHoldSince=now;axLongFired=false;}
      else if(!axLongFired&&now-axHoldSince>=800){
        axLongFired=true;
        clearAllUserItems();
      }
    }else{
      if(axHoldSince!=null&&!axLongFired){
        hudPanelsVisible=!hudPanelsVisible;
        if(hudPanelsVisible)resetHudCache();else hudDragState=null;
        showHudToast(hudPanelsVisible?'HUD SHOWN':'HUD HIDDEN','#c084fc',1400);
      }
      axHoldSince=null;axLongFired=false;
    }

    // Bare-hand Two-Mode Architecture (SCAN <-> MEASURE) + Gated Wrist Menu (MEASURE only, zero Quest-OS conflict):
    // • Left Index+Thumb stays 100% free for the Meta Quest OS menu everywhere.
    // • Direct 1-Handed Shortcuts: Left Fist Hold (0.85 s) toggles SCAN <-> MEASURE; Right Fist Hold (0.80 s) toggles REC ALL.
    // • Wrist Menus (Left: HUD / ● REC ALL / TRAIL / ↺ CLEAR ALL; Right: Q / RANGE / ↺ RECALIBRATE) are active ONLY in MEASURE mode
    //   and completely hidden/disabled in SCAN mode so nothing gets in the way while scanning or recording.
    let ringShown=false,labelsShown=false,gazeShown=false,leftFistActiveForCursor=false,rightFistActiveForCursor=false;
    fingerBadgeIdx.sprite.visible=false;
    fingerBadgeMid.sprite.visible=false;
    fingerBadgeRing.sprite.visible=false;
    fingerBadgePinky.sprite.visible=false;
    fingerBadgeRightMid.sprite.visible=false;
    fingerBadgeRightRing.sprite.visible=false;
    fingerBadgeRightPinky.sprite.visible=false;
    leftWristDot.visible=false;
    rightWristDot.visible=false;
    leftScanRecDot.visible=false;
    hideHandOccluders();
    fistProgressSprite.visible=false;
    const leftHandInput=frameInputs.hands.left;
    const rightHandInput=frameInputs.hands.right;
    const jDist=(a,b)=>(a&&b)?Math.hypot(a.transform.position.x-b.transform.position.x,a.transform.position.y-b.transform.position.y,a.transform.position.z-b.transform.position.z):Infinity;
    const pDist=(a,b)=>(a&&b)?Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]):Infinity;
    const PINCH_DIST=0.018,HOLD_MS=450,PALM_DWELL_MS=650,PALM_GAZE_DEADZONE_MS=90,MENU_GRACE_MS=450,DOT_TAP_MS=90,PILL_POKE_MS=180;
    const canScan=asPhase==='ready'&&(anPhase==='tracking'||anPhase==='created');
    const handViewAndPalm=(handInput,isRight,vm,prevWrist,prevSpeed)=>{
      const getJ=name=>handInput.getJoint(name);
      const toVec=j=>j?[j.transform.position.x,j.transform.position.y,j.transform.position.z]:null;
      const jt=getJ('thumb-tip'),ji=getJ('index-finger-tip'),jm=getJ('middle-finger-tip'),jr=getJ('ring-finger-tip'),jp=getJ('pinky-finger-tip');
      const jw=getJ('wrist'),jim=getJ('index-finger-metacarpal'),jpm=getJ('pinky-finger-metacarpal');
      const jMidProx=getJ('middle-finger-phalanx-proximal');
      let inView=false,fistInView=false,palmDot=-1,innerWristPos=null,fistPose=null,nextPrev=prevWrist,speed=prevSpeed;
      if(jw){
        const wp=jw.transform.position;
        if(prevWrist&&now>prevWrist.atMs){
          const dtW=Math.max(0.004,(now-prevWrist.atMs)/1000);
          const inst=Math.hypot(wp.x-prevWrist.x,wp.y-prevWrist.y,wp.z-prevWrist.z)/dtW;
          speed=prevSpeed*0.72+inst*0.28;
        }
        nextPrev={x:wp.x,y:wp.y,z:wp.z,atMs:now};
      }else{
        nextPrev=null;speed=0;
      }
      const refJoint=jMidProx||jw||jt;
      if(refJoint){
        const tp=refJoint.transform.position;
        const vx=tp.x-vm[12],vy=tp.y-vm[13],vz=tp.z-vm[14];
        const lx=vm[0]*vx+vm[1]*vy+vm[2]*vz,ly=vm[4]*vx+vm[5]*vy+vm[6]*vz,lz=vm[8]*vx+vm[9]*vy+vm[10]*vz;
        inView=lz<-0.15&&lz>-0.65&&Math.abs(lx)<-lz*0.85&&Math.abs(ly)<-lz*0.78;
        fistInView=lz<-0.18&&lz>-0.55&&Math.abs(lx)<-lz*0.55&&Math.abs(ly)<-lz*0.50;
      }
      if(jw){
        const fistJoints={wrist:toVec(jw),thumbTip:toVec(jt)};
        const tips=[['index',ji],['middle',jm],['ring',jr],['pinky',jp]];
        for(const [fn,tipPose] of tips){
          const prox=fn==='middle'?jMidProx:getJ(`${fn}-finger-phalanx-proximal`);
          const inter=getJ(`${fn}-finger-phalanx-intermediate`);
          if(prox&&inter&&tipPose){
            fistJoints[fn]={prox:toVec(prox),inter:toVec(inter),tip:toVec(tipPose)};
          }
        }
        fistPose=classifyFistPose(fistJoints,{
          enterCurlCos:-0.15,
          enterTipPalmRatio:0.74,
          enterTipWristRatio:0.90,
          enterPinkyCurlCos:-0.05,
          enterPinkyTipPalmRatio:0.80,
          thumbMaxRatio:0.72,
        });
      }
      if(jw&&jim&&jpm){
        const w=jw.transform.position,im=jim.transform.position,pm=jpm.transform.position;
        const v1x=im.x-w.x,v1y=im.y-w.y,v1z=im.z-w.z;
        const v2x=pm.x-w.x,v2y=pm.y-w.y,v2z=pm.z-w.z;
        // Left palm normal = v2 x v1; Right palm normal = v1 x v2 (mirror handedness).
        const sgn=isRight?-1:1;
        const nx=sgn*(v2y*v1z-v2z*v1y),ny=sgn*(v2z*v1x-v2x*v1z),nz=sgn*(v2x*v1y-v2y*v1x);
        const nLen=Math.hypot(nx,ny,nz)||1;
        const unx=nx/nLen,uny=ny/nLen,unz=nz/nLen;
        const ex=vm[12]-w.x,ey=vm[13]-w.y,ez=vm[14]-w.z;
        const eLen=Math.hypot(ex,ey,ez)||1;
        palmDot=(unx*ex+uny*ey+unz*ez)/eLen;
        // Inner wrist dot sits 2.2 cm off the inner wrist along palm normal.
        innerWristPos=[w.x+unx*0.022,w.y+uny*0.022,w.z+unz*0.022];
      }
      return {jt,ji,jm,jr,jp,jw,jMidProx,inView,fistInView,palmDot,innerWristPos,fistPose,nextPrev,speed};
    };

    if(pose){
      const vm=pose.transform.matrix;
      const ux=vm[4]*0.022,uy=vm[5]*0.022,uz=vm[6]*0.022;
      const showRingAtPos=(px,py,pz,colorHex,prog)=>{
        handGestureMat.color.setHex(colorHex);
        handGestureRing.position.set(px,py,pz);
        handGestureRing.quaternion.setFromRotationMatrix(_gestureMat4.fromArray(vm));
        handGestureRing.scale.setScalar(0.45+prog*1.35);
        ringShown=true;
      };
      const showRingAt=(a,b,colorHex,prog)=>{
        showRingAtPos((a.transform.position.x+b.transform.position.x)/2,(a.transform.position.y+b.transform.position.y)/2,(a.transform.position.z+b.transform.position.z)/2,colorHex,prog);
      };
      const placePill=(pill,joint,visible,text,accent,hot)=>{
        pill.sprite.visible=Boolean(visible&&joint);
        if(!pill.sprite.visible)return null;
        labelsShown=true;
        const p=joint.transform.position;
        const px=p.x+ux,py=p.y+uy,pz=p.z+uz;
        pill.sprite.position.set(px,py,pz);
        const sc=hot?1.24:1.0;
        pill.sprite.scale.set(0.047*sc,0.0145*sc,1);
        drawFingerPill(pill,text,accent,hot);
        return [px,py,pz];
      };

      const leftInfo=leftHandInput?handViewAndPalm(leftHandInput,false,vm,leftWristPrev,leftWristSpeed):null;
      // Right hand metrics are also sampled in SCAN mode for the Right Frontal Fist REC ALL shortcut (Wrist Menu stays gated to MEASURE).
      const rightInfo=rightHandInput?handViewAndPalm(rightHandInput,true,vm,rightWristPrev,rightWristSpeed):null;
      leftWristPrev=leftInfo?leftInfo.nextPrev:null;leftWristSpeed=leftInfo?leftInfo.speed:0;
      rightWristPrev=rightInfo?rightInfo.nextPrev:null;rightWristSpeed=rightInfo?rightInfo.speed:0;
      updateHandTints(leftHandInput,rightHandInput,extraScanActive,recordAllActive,canScan);
      const leftIdxPos=leftHandInput?leftHandInput.getIndexTipWorld():null;
      const rightIdxPos=rightHandInput?rightHandInput.getIndexTipWorld():null;

      const openPills=[];

      if(extraScanActive){
        leftMenuOpenUntil=0;rightMenuOpenUntil=0;
        leftPalmDwellSince=null;rightPalmDwellSince=null;leftDotTapSince=null;rightDotTapSince=null;
        leftMidPinchSince=null;leftRingPinchSince=null;leftPinkyPinchSince=null;rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;
        leftMenuOpenNow=false;
      }

      if(leftInfo){
        const {jt,ji,jm,jr,jp,jMidProx,inView,fistInView,palmDot,innerWristPos,fistPose,speed}=leftInfo;
        const fistPostureActive=Boolean(fistPose?.isFist||fistPose?.isHeldFist);
        leftFistActiveForCursor=fistPostureActive;
        const dWristDot=pDist(rightIdxPos,innerWristPos);

        // 1. Left Wrist Menu (MEASURE mode ONLY — completely suppressed in SCAN mode).
        if(!extraScanActive){
          const allowWristDot=canScan&&inView&&palmDot>0.48&&!fistPostureActive&&now>=leftMenuOpenUntil;
          if(allowWristDot&&innerWristPos){
            recordProbeProximity('hand:right',dWristDot);
            leftWristDot.visible=true;
            leftWristDot.position.set(innerWristPos[0],innerWristPos[1],innerWristPos[2]);
            const dotHot=dWristDot<0.036;
            leftWristDot.scale.setScalar(dotHot?1.45:1.0);
            leftWristDotMat.opacity=dotHot?0.92:0.52;
            if(dWristDot<0.026){
              if(leftDotTapSince==null)leftDotTapSince=now;
              if(now-leftDotTapSince>=DOT_TAP_MS){
                leftMenuOpenUntil=now+2400;
                leftDotTapSince=null;
                playModeTick('pin');
              }
            }else{
              leftDotTapSince=null;
            }
          }else{
            leftDotTapSince=null;
          }

          if(canScan&&inView&&palmDot>0.72&&speed<0.14&&!fistPostureActive){
            if(leftPalmDwellSince==null)leftPalmDwellSince=now;
            const elapsed=now-leftPalmDwellSince;
            if(elapsed>=PALM_DWELL_MS){
              leftMenuOpenUntil=Math.max(leftMenuOpenUntil,now+MENU_GRACE_MS);
            }else if(now>=leftMenuOpenUntil&&elapsed>=PALM_GAZE_DEADZONE_MS&&innerWristPos){
              const prog=(elapsed-PALM_GAZE_DEADZONE_MS)/(PALM_DWELL_MS-PALM_GAZE_DEADZONE_MS);
              drawWristGazeCircle(prog,'#38bdf8');
              wristGazeSprite.position.set(innerWristPos[0],innerWristPos[1],innerWristPos[2]);
              gazeShown=true;
            }
          }else{
            leftPalmDwellSince=null;
          }
          if(now<leftMenuOpenUntil&&inView&&palmDot>0.45&&!fistPostureActive){
            leftMenuOpenUntil=Math.max(leftMenuOpenUntil,now+MENU_GRACE_MS);
          }
        }
        // (Left-middle SCAN recording dot removed on purpose.)

        const leftMenuOpen=canScan&&!extraScanActive&&now<leftMenuOpenUntil;
        leftPalmFacingViewer=Boolean(inView&&palmDot>0.65);
        leftMenuOpenNow=leftMenuOpen;

        // 2. Hardened Direct 1-Handed SCAN <-> MEASURE Switch: Frontal Left Fist Hold (0.85 s, strict curl + centered view + away from tags).
        let leftNearTag=false;
        if(frameInputs.invWorld&&fistPose?.fistCenter){
          const fcL=worldPointToL(frameInputs.invWorld,fistPose.fistCenter[0],fistPose.fistCenter[1],fistPose.fistCenter[2]);
          for(const entry of movableTools.values()){
            const cL=entry.currentPose?.centerL;
            const d=cL?Math.hypot(fcL[0]-cL[0],fcL[1]-cL[1],fcL[2]-cL[2]):(entry.displayMatrix?Math.hypot(fcL[0]-entry.displayMatrix[12],fcL[1]-entry.displayMatrix[13],fcL[2]-entry.displayMatrix[14]):Infinity);
            if(d<0.12){leftNearTag=true;break;}
          }
          if(!leftNearTag){
            for(const extra of placedExtras.values()){
              const cL=extra.wp?.centerL;
              if(cL&&Math.hypot(fcL[0]-cL[0],fcL[1]-cL[1],fcL[2]-cL[2])<0.12){leftNearTag=true;break;}
            }
          }
        }
        const rightNearLeft=Boolean(rightIdxPos&&innerWristPos&&pDist(rightIdxPos,innerWristPos)<0.075);
        const leftCarrying=tagHandCoupler.isHandCarrying('left');
        const fistSuppressed=leftMenuOpen||leftCarrying||leftNearTag||pinHoldHand==='left'||rightNearLeft||hudDragState?.probeId==='hand:left'||hoveredDevDock;
        const fistRes=leftFistToggle.update({pose:fistPose,inView:fistInView&&palmDot>0.35,speedMps:speed,enabled:canScan,suppressed:fistSuppressed,nowMs:now});
        if(fistRes.triggered){
          leftHandCooldownUntil=now+1200;
          leftMidPinchSince=null;leftRingPinchSince=null;leftPinkyPinchSince=null;
          leftPalmDwellSince=null;leftDotTapSince=null;
          leftFistFlashToScan=!extraScanActive;
          leftFistFlashPos=fistPose?.fistCenter?[fistPose.fistCenter[0]+ux*1.8,fistPose.fistCenter[1]+uy*1.8,fistPose.fistCenter[2]+uz*1.8]:null;
          leftFistFlashUntil=now+320;
          toggleExtraScan();
        }
        if(fistRes.showVisual&&!fistRes.triggered&&fistPose?.fistCenter){
          fistProgressSprite.visible=true;
          fistProgressSprite.position.set(fistPose.fistCenter[0]+ux*1.8,fistPose.fistCenter[1]+uy*1.8,fistPose.fistCenter[2]+uz*1.8);
          drawFistProgressBadge(fistRes.progress,!extraScanActive,false);
        }else if(now<leftFistFlashUntil&&leftFistFlashPos){
          fistProgressSprite.visible=true;
          fistProgressSprite.position.set(leftFistFlashPos[0],leftFistFlashPos[1],leftFistFlashPos[2]);
          drawFistProgressBadge(1,leftFistFlashToScan,true);
        }

        // 3. Left Menu Middle (● REC ALL), Ring (TRAIL: ID / TURBO) & Pinky (↺ CLEAR ALL) ONLY work while in MEASURE mode with Wrist Menu open (keeps Perimeter Fence intact).
        const dIndex=jDist(jt,ji);
        let activeFinger=null;
        const allowMenuPinch=inView&&now>=leftHandCooldownUntil&&dIndex>0.035&&leftMenuOpen;
        const dMid=allowMenuPinch?jDist(jt,jm):Infinity;
        const dRing=allowMenuPinch?jDist(jt,jr):Infinity;
        const dPinky=allowMenuPinch?jDist(jt,jp):Infinity;
        if(dMid<PINCH_DIST&&dMid<=dRing&&dMid<=dPinky){
          leftRingPinchSince=null;leftPinkyPinchSince=null;activeFinger='mid';
          if(leftMidPinchSince==null)leftMidPinchSince=now;
          const p=Math.min(1,(now-leftMidPinchSince)/HOLD_MS);
          showRingAt(jt,jm,0xef4444,p);
          if(p>=1){leftMidPinchSince=null;leftMenuOpenUntil=0;leftHandCooldownUntil=now+900;setRecordAllActive(true);}
        }else if(dRing<PINCH_DIST&&dRing<=dPinky){
          leftMidPinchSince=null;leftPinkyPinchSince=null;activeFinger='ring';
          if(leftRingPinchSince==null)leftRingPinchSince=now;
          const p=Math.min(1,(now-leftRingPinchSince)/HOLD_MS);
          showRingAt(jt,jr,trailColorMode==='speed'?0xf97316:0x38bdf8,p);
          if(p>=1){leftRingPinchSince=null;leftHandCooldownUntil=now+900;toggleTrailColorMode();}
        }else if(dPinky<PINCH_DIST){
          leftMidPinchSince=null;leftRingPinchSince=null;activeFinger='pinky';
          if(leftPinkyPinchSince==null)leftPinkyPinchSince=now;
          const p=Math.min(1,(now-leftPinkyPinchSince)/HOLD_MS);
          showRingAt(jt,jp,0xf59e0b,p);
          if(p>=1){leftPinkyPinchSince=null;leftHandCooldownUntil=now+1200;clearAllUserItems();}
        }else{
          leftMidPinchSince=null;
          leftRingPinchSince=null;
          leftPinkyPinchSince=null;
        }

        if(leftMenuOpen){
          const trailColorLabel=trailColorMode==='speed'?'TRAIL: TURBO':'TRAIL: ID';
          const trailColorAccent=trailColorMode==='speed'?'#f97316':'#38bdf8';
          const pHudL=placePill(fingerBadgeIdx,ji,canScan,hudPanelsVisible?'HUD OFF':'HUD ON','#c084fc',wristPillPokeId==='l-hud');
          const pMidL=placePill(fingerBadgeMid,jm,canScan,'● REC ALL','#ef4444',activeFinger==='mid'||wristPillPokeId==='l-rec-all');
          const pColorL=placePill(fingerBadgeRing,jr,canScan,trailColorLabel,trailColorAccent,activeFinger==='ring'||wristPillPokeId==='l-color');
          const pPinky=placePill(fingerBadgePinky,jp,canScan,'↺ CLEAR ALL','#fbbf24',activeFinger==='pinky'||wristPillPokeId==='l-pinky');
          if(pHudL)openPills.push({id:'l-hud',probeId:'hand:right',pos:pHudL,poker:rightIdxPos,color:0xc084fc,action:()=>{leftMenuOpenUntil=0;leftHandCooldownUntil=now+900;hudPanelsVisible=!hudPanelsVisible;if(hudPanelsVisible)resetHudCache();else hudDragState=null;playModeTick('pin');showHudToast(hudPanelsVisible?'HUD SHOWN':'HUD HIDDEN','#c084fc',1400);}});
          if(pMidL)openPills.push({id:'l-rec-all',probeId:'hand:right',pos:pMidL,poker:rightIdxPos,color:0xef4444,action:()=>{leftMenuOpenUntil=0;leftHandCooldownUntil=now+900;setRecordAllActive(true);}});
          if(pColorL)openPills.push({id:'l-color',probeId:'hand:right',pos:pColorL,poker:rightIdxPos,color:trailColorMode==='speed'?0xf97316:0x38bdf8,action:()=>{leftMenuOpenUntil=0;leftHandCooldownUntil=now+900;toggleTrailColorMode();}});
          if(pPinky)openPills.push({id:'l-pinky',probeId:'hand:right',pos:pPinky,poker:rightIdxPos,color:0xf59e0b,action:()=>{leftMenuOpenUntil=0;leftHandCooldownUntil=now+900;clearAllUserItems();}});
        }
      }else{
        leftFistToggle.update({pose:null,inView:false,enabled:false,nowMs:now});
        leftMidPinchSince=null;leftRingPinchSince=null;leftPinkyPinchSince=null;
        leftPalmDwellSince=null;leftDotTapSince=null;
        leftPalmFacingViewer=false;leftMenuOpenNow=false;
      }

      if(rightInfo){
        const {jt,ji,jm,jr,jp,inView,fistInView,palmDot,innerWristPos,fistPose,speed}=rightInfo;
        if(!extraScanActive){
          const showRightDot=inView&&palmDot>0.45&&now>=rightMenuOpenUntil&&Boolean(innerWristPos);
          if(showRightDot){
            rightWristDot.visible=true;
            rightWristDot.position.set(innerWristPos[0],innerWristPos[1],innerWristPos[2]);
            const dDot=pDist(leftIdxPos,innerWristPos);
            recordProbeProximity('hand:left',dDot);
            const dotHot=dDot<0.036;
            rightWristDot.scale.setScalar(dotHot?1.45:1.0);
            rightWristDotMat.opacity=dotHot?0.92:0.52;
            if(dDot<0.028){
              if(rightDotTapSince==null)rightDotTapSince=now;
              if(now-rightDotTapSince>=DOT_TAP_MS){
                rightMenuOpenUntil=now+2400;
                rightDotTapSince=null;
                playModeTick('pin');
              }
            }else{
              rightDotTapSince=null;
            }
          }else{
            rightDotTapSince=null;
          }

          if(inView&&palmDot>0.72&&speed<0.14){
            if(rightPalmDwellSince==null)rightPalmDwellSince=now;
            const elapsed=now-rightPalmDwellSince;
            if(elapsed>=PALM_DWELL_MS){
              rightMenuOpenUntil=Math.max(rightMenuOpenUntil,now+MENU_GRACE_MS);
            }else if(!gazeShown&&now>=rightMenuOpenUntil&&elapsed>=PALM_GAZE_DEADZONE_MS&&innerWristPos){
              const prog=(elapsed-PALM_GAZE_DEADZONE_MS)/(PALM_DWELL_MS-PALM_GAZE_DEADZONE_MS);
              drawWristGazeCircle(prog,'#c084fc');
              wristGazeSprite.position.set(innerWristPos[0],innerWristPos[1],innerWristPos[2]);
              gazeShown=true;
            }
          }else{
            rightPalmDwellSince=null;
          }
          if(now<rightMenuOpenUntil&&inView&&palmDot>0.45){
            rightMenuOpenUntil=Math.max(rightMenuOpenUntil,now+MENU_GRACE_MS);
          }
          const rightMenuOpen=now<rightMenuOpenUntil;

          const indexOpen=jDist(jt,ji)>0.035;
          const intentional=rightMenuOpen&&inView&&indexOpen&&now>=rightHandCooldownUntil;
          const dRightMid=intentional?jDist(jt,jm):Infinity;
          const dRightRing=intentional?jDist(jt,jr):Infinity;
          const dRightPinky=intentional?jDist(jt,jp):Infinity;
          let rightActive=null;
          if(dRightMid<PINCH_DIST&&dRightMid<=dRightRing&&dRightMid<=dRightPinky){
            rightRingPinchSince=null;rightPinkyPinchSince=null;rightActive='mid';
            if(rightMidPinchSince==null)rightMidPinchSince=now;
            const p=Math.min(1,(now-rightMidPinchSince)/HOLD_MS);
            showRingAt(jt,jm,0x38bdf8,p);
            if(p>=1){
              rightMidPinchSince=null;rightHandCooldownUntil=now+1000;
              cycleHandGlow();
            }
          }else if(dRightRing<PINCH_DIST&&dRightRing<=dRightPinky){
            rightMidPinchSince=null;rightPinkyPinchSince=null;rightActive='ring';
            if(rightRingPinchSince==null)rightRingPinchSince=now;
            const p=Math.min(1,(now-rightRingPinchSince)/HOLD_MS);
            showRingAt(jt,jr,0xc084fc,p);
            if(p>=1){
              rightRingPinchSince=null;rightHandCooldownUntil=now+1000;
              toggleDevMode();
            }
          }else if(dRightPinky<PINCH_DIST){
            rightMidPinchSince=null;rightRingPinchSince=null;rightActive='pinky';
            if(rightPinkyPinchSince==null)rightPinkyPinchSince=now;
            const p=Math.min(1,(now-rightPinkyPinchSince)/HOLD_MS);
            showRingAt(jt,jp,0xf59e0b,p);
            if(p>=1){
              rightPinkyPinchSince=null;rightMenuOpenUntil=0;rightHandCooldownUntil=now+1400;
              triggerRecalibrate();
            }
          }else{
            rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;
          }
          if(rightMenuOpen){
            const qText=`GLOW: ${HAND_GLOW_LEVELS[handGlowLevel]}`;
            const pQual=placePill(fingerBadgeRightMid,jm,true,qText,'#38bdf8',rightActive==='mid'||wristPillPokeId==='r-qual');
            const devText=devModeEnabled?'DEV: ON':'DEV: OFF';
            const devCol=devModeEnabled?'#c084fc':'#94a3b8';
            const pDev=placePill(fingerBadgeRightRing,jr,true,devText,devCol,rightActive==='ring'||wristPillPokeId==='r-dev');
            const pRecal=placePill(fingerBadgeRightPinky,jp,true,'↺ RECALIBRATE','#fbbf24',rightActive==='pinky'||wristPillPokeId==='r-recal');
            if(pQual)openPills.push({id:'r-qual',probeId:'hand:left',pos:pQual,poker:leftIdxPos,color:0x38bdf8,action:()=>{rightMenuOpenUntil=0;rightHandCooldownUntil=now+900;cycleHandGlow();}});
            if(pDev)openPills.push({id:'r-dev',probeId:'hand:left',pos:pDev,poker:leftIdxPos,color:0xc084fc,action:()=>{rightMenuOpenUntil=0;rightHandCooldownUntil=now+900;toggleDevMode();}});
            if(pRecal)openPills.push({id:'r-recal',probeId:'hand:left',pos:pRecal,poker:leftIdxPos,color:0xf59e0b,action:()=>{rightMenuOpenUntil=0;rightHandCooldownUntil=now+1400;triggerRecalibrate();}});
          }
        }else{
          rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;rightPalmDwellSince=null;rightDotTapSince=null;
        }

        // Right Frontal Fist Hold (~800 ms): toggle REC ALL from either SCAN or MEASURE (no Quest-OS conflict).
        const rightFistPostureActive=Boolean(fistPose?.isFist||fistPose?.isHeldFist);
        rightFistActiveForCursor=rightFistPostureActive;
        let rightNearTag=false;
        if(frameInputs.invWorld&&fistPose?.fistCenter){
          const fcL=worldPointToL(frameInputs.invWorld,fistPose.fistCenter[0],fistPose.fistCenter[1],fistPose.fistCenter[2]);
          for(const entry of movableTools.values()){
            const cL=entry.currentPose?.centerL;
            const d=cL?Math.hypot(fcL[0]-cL[0],fcL[1]-cL[1],fcL[2]-cL[2]):(entry.displayMatrix?Math.hypot(fcL[0]-entry.displayMatrix[12],fcL[1]-entry.displayMatrix[13],fcL[2]-entry.displayMatrix[14]):Infinity);
            if(d<0.12){rightNearTag=true;break;}
          }
          if(!rightNearTag){
            for(const extra of placedExtras.values()){
              const cL=extra.wp?.centerL;
              if(cL&&Math.hypot(fcL[0]-cL[0],fcL[1]-cL[1],fcL[2]-cL[2])<0.12){rightNearTag=true;break;}
            }
          }
        }
        const rightMenuOpenNow=!extraScanActive&&now<rightMenuOpenUntil;
        const leftNearRight=Boolean(leftIdxPos&&innerWristPos&&pDist(leftIdxPos,innerWristPos)<0.075);
        const rightCarrying=tagHandCoupler.isHandCarrying('right');
        const rightFistSuppressed=rightMenuOpenNow||rightCarrying||rightNearTag||pinHoldHand==='right'||leftNearRight||hudDragState?.probeId==='hand:right'||leftFistActiveForCursor||hoveredModePill||hoveredDevDock||liveTouchLoaderActive;
        const rFistRes=rightFistToggle.update({pose:fistPose,inView:fistInView&&palmDot>0.35,speedMps:speed,enabled:canScan,suppressed:rightFistSuppressed,nowMs:now});
        if(rFistRes.triggered){
          rightHandCooldownUntil=now+1200;
          rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;
          rightPalmDwellSince=null;rightDotTapSince=null;
          rightFistFlashToRecAll=!recordAllActive;
          rightFistFlashPos=fistPose?.fistCenter?[fistPose.fistCenter[0]+ux*1.8,fistPose.fistCenter[1]+uy*1.8,fistPose.fistCenter[2]+uz*1.8]:null;
          rightFistFlashUntil=now+320;
          toggleRecordAll();
        }
        if(!fistProgressSprite.visible&&rFistRes.showVisual&&!rFistRes.triggered&&fistPose?.fistCenter){
          fistProgressSprite.visible=true;
          fistProgressSprite.position.set(fistPose.fistCenter[0]+ux*1.8,fistPose.fistCenter[1]+uy*1.8,fistPose.fistCenter[2]+uz*1.8);
          drawFistProgressBadge(rFistRes.progress,{
            label:!recordAllActive?'REC ALL':'STOP ALL',
            accent:!recordAllActive?'#ef4444':'#22c55e',
            symbol:!recordAllActive?'●':'■',
            done:false,
          });
        }else if(!fistProgressSprite.visible&&now<rightFistFlashUntil&&rightFistFlashPos){
          fistProgressSprite.visible=true;
          fistProgressSprite.position.set(rightFistFlashPos[0],rightFistFlashPos[1],rightFistFlashPos[2]);
          drawFistProgressBadge(1,{
            label:rightFistFlashToRecAll?'REC ALL':'STOP ALL',
            accent:rightFistFlashToRecAll?'#ef4444':'#22c55e',
            symbol:rightFistFlashToRecAll?'●':'■',
            done:true,
          });
        }
      }else{
        rightFistToggle.update({pose:null,inView:false,enabled:false,nowMs:now});
        rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;rightPalmDwellSince=null;rightDotTapSince=null;
      }

      // Direct other-hand index poke on any open wrist-menu pill (180 ms hold, < 3.2 cm).
      let poked=null,pokedDist=Infinity;
      for(const item of openPills){
        const d=pDist(item.poker,item.pos);
        recordProbeProximity(item.probeId,d);
        if(d<0.032&&d<pokedDist){pokedDist=d;poked=item;}
      }
      if(poked){
        if(wristPillPokeId!==poked.id){wristPillPokeId=poked.id;wristPillPokeSince=now;}
        const prog=Math.min(1,(now-wristPillPokeSince)/PILL_POKE_MS);
        showRingAtPos(poked.pos[0],poked.pos[1],poked.pos[2],poked.color,prog);
        if(prog>=1){
          wristPillPokeId=null;wristPillPokeSince=0;
          poked.action();
        }
      }else{
        wristPillPokeId=null;wristPillPokeSince=0;
      }
    }else{
      leftFistToggle.update({pose:null,inView:false,enabled:false,nowMs:now});
      rightFistToggle.update({pose:null,inView:false,enabled:false,nowMs:now});
      leftMidPinchSince=null;leftRingPinchSince=null;leftPinkyPinchSince=null;rightMidPinchSince=null;rightRingPinchSince=null;rightPinkyPinchSince=null;
      leftPalmDwellSince=null;rightPalmDwellSince=null;leftDotTapSince=null;rightDotTapSince=null;
      wristPillPokeId=null;wristPillPokeSince=0;
    }
    handGestureRing.visible=ringShown;
    handLabelsGroup.visible=labelsShown;
    wristGazeSprite.visible=gazeShown;
    if(toastSprite.visible&&now>toastUntilMs)toastSprite.visible=false;
    updateFingertipCursors(frameInputs,pose,{hideLeft:leftFistActiveForCursor,hideRight:rightFistActiveForCursor});

    hudRig.visible=Boolean(pose);
    hudLeft.visible=hudPanelsVisible;
    hudRight.visible=hudPanelsVisible;
    hudTopBar.visible=hudPanelsVisible;
    perfTracker.recordHudFrame(hudPanelsVisible);
    if(!portalWarpState&&!showcaseNavState&&centerConfirmUntilMs>0&&now>centerConfirmUntilMs){
      centerConfirmUntilMs=0;
      warpBannerSprite.visible=false;
    }
    if(pose){
      _xrVmMat4.fromArray(pose.transform.matrix);
      hudRig.position.setFromMatrixPosition(_xrVmMat4);
      hudRig.quaternion.setFromRotationMatrix(_xrVmMat4);
      updateHudPanelDragging(frameInputs,pose,now);
      const calibratingOrLocking=asPhase!=='ready'||anPhase==='collecting';
      latestViewMat=Array.from(pose.transform.matrix);
      updateTagFirstSeenHint(pose,asPhase,anPhase,now);
      updateOrbitGuide(pose,asPhase,now,anSummary,assistant?.summary?.());
      reticle.visible=Boolean(capture.isRunning?.())&&calibratingOrLocking&&!spatialHaloGroup.visible;
      reticle.material.color.setHex(0x38bdf8);
      reticle.scale.setScalar(1.0);
      if(now-lastStabilitySample>=240){
        lastStabilitySample=now;
        const auditFresh=lastAuditRow&&now-lastAuditAt<2500;
        const jVal=auditFresh?lastAuditRow.positionDifferenceMm:(anPhase==='tracking'?liveJitterMm:(anSummary.quality?.spreadMm??null));
        if(jVal!=null){stabilityHistory.push(jVal);if(stabilityHistory.length>60)stabilityHistory.shift();}
      }
      if(now-lastHud>=120){
        lastHud=now;
        if(rvfcWindowStart!=null&&now-rvfcWindowStart>=1000){
          const cur=capture.presentedCallbacks?.()??capture.state().presentedCallbacks;
          rvfcHz=Math.max(0,(cur-rvfcWindowCount)*1000/(now-rvfcWindowStart));
          rvfcWindowCount=cur;rvfcWindowStart=now;
        }
        // Skip all 2D canvas redraws & GPU texture uploads when HUD panels are hidden; stagger visible panels (max 1 per frame).
        if(hudPanelsVisible){
          hudSlot=(hudSlot+1)%3;
          if(hudSlot===0){
            const avgFps=fpsHistory.length?Math.round(fpsHistory.reduce((a,b)=>a+b,0)/fpsHistory.length):0;
            const q=anSummary.quality||{};
            const tagCount=movableTools.size+placedExtras.size+(perimeterFence?.dynamicCount?.()??0);
            const topKey=`${asPhase}:${anPhase}:${q.count||0}/${q.required||5}:${q.reason||''}:${extraScanActive?1:0}:${avgFps}:${tagCount}:${pinnedPoints.size}`;
            if(topKey!==hudTopKey){
              hudTopKey=topKey;
              const t0=performance.now();
              renderTopHudBar(now,{phase:asPhase},anSummary,null);
              perfTracker.recordHudUpload('top',performance.now()-t0);
            }
          }else if(hudSlot===1){
            const q=anSummary.quality||{};
            const isCalOrLock=asPhase!=='ready'||anPhase==='collecting'||anPhase==='ready';
            let recCount=0;
            for(const e of movableTools.values())if(isEntryRecording(e))recCount++;
            const fStatus=perimeterFence?.status()??'idle';
            const fCnt=perimeterFence?perimeterFence.dynamicCount():0;
            const timeBucket=isCalOrLock?Math.floor(now/260):0;
            const leftKey=`${asPhase}:${assistant?.summary?.().count??0}:${anPhase}:${q.count||0}:${q.seenTagCount??0}:${extraScanActive?1:0}:${recordAllActive?1:0}:${fStatus}:${fCnt}:${movableTools.size}:${pinnedPoints.size}:${recCount}:${lastSampleStats?.serial??0}:${lastMotionState?.frozen?1:0}:${lastMotionState?.source??''}:${now<reanchorConfirmUntil?1:0}:${timeBucket}`;
            if(leftKey!==hudLeftKey){
              hudLeftKey=leftKey;
              const asState=assistant.state(),anState=anchor.state(),csState=capture.state();
              const t0=performance.now();
              renderLeftHud(now,asState,anState,csState);
              perfTracker.recordHudUpload('left',performance.now()-t0);
            }
          }else if(now-lastRightHud>=360){
            lastRightHud=now;
            const asState=assistant.state(),anState=anchor.state(),csState=capture.state();
            const t0=performance.now();
            renderRightHud(now,asState,anState,csState);
            perfTracker.recordHudUpload('right',performance.now()-t0);
          }
        }
      }
    }

    hold=nextExitHold(hold,xrExitPressed(session.inputSources).held,now);
    if(hold.fire)end();
    if(!root.visible)updateAlarmSiren(false);
    if(root.visible){
      if(frameBatch.activeCount()>0||alertGlowBatch.activeCount()>0||fenceGroup.visible)frameShimmerUniforms.uTime.value=now*0.001;
      frameBatch.commit();
      updateAlarmSiren(frameBatch.alertCount()>0);
      alertGlowBatch.commit();
      overlayFrameBatch.commit();
      rodBatch.commit();
      trailGapBridgeBatch.commit();
      trailGapCapBatch.commit();
      overlayRodBatch.commit();
      nodeBatch.commit();
      zRingBatch.commit();
      closeOrbBatch.commit();
      recOrbBatch.commit();
      axesBatch.commit();
      badgeMesh.updateBillboards(frameInputs.viewerRightL,frameInputs.viewerUpL);
      tagFaceMesh.commit();
    }
    const renderStart=performance.now();
    const cpuMs=renderStart-now;
    renderer.render(scene,camera3d);
    const renderCpuMs=performance.now()-renderStart;
    const perfMode=asPhase!=='ready'?'calibrating':(extraScanActive?'scan':'measure');
    perfTracker.recordFrame(perfMode,frameDtMs,cpuMs,renderCpuMs);
  };
  if(renderer)renderer.setAnimationLoop(xrFrameHandler);

  window.addEventListener('pagehide',()=>{assistant?.dispose();anchor.end(performance.now());capture.dispose();session?.end().catch(()=>{});});
}catch(e){status(`Startup failed: ${e.message}`);}
