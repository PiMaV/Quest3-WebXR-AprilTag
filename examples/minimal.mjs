import * as THREE from '../vendor/three/three.module.js';
import {
  APP_VERSION_LABEL,
  associateAnchorCapture,
  createCalibrationAssistant,
  createLAnchorController,
  createObservationBuffer,
  createReferenceCamera,
  cvToGL,
  deviceIdentity,
  inverse,
  nextExitHold,
  product,
  xrExitPressed,
} from '../src/index.mjs';

const $ = (id) => document.getElementById(id);
const setStatus = (msg) => {
  $('status').textContent = `${APP_VERSION_LABEL} · ${msg}`;
};

// 1. Load physical L-angle geometry definitions (units: meters, family: tag36h11).
const [rigA4, rigTablet] = await Promise.all([
  fetch(new URL('../rigs/ms-qr-main-l.json', import.meta.url)).then((r) => r.json()),
  fetch(new URL('../rigs/ms-qr-tablet-l.json', import.meta.url)).then((r) => r.json()),
]);
let rig = $('rigSelect').value === 'tablet' ? rigTablet : rigA4;
const rigTagIds = () => rig.tools[0].tags.map((t) => t.id);

// 2. Three.js WebXR scene anchored at the physical L-angle origin (0, 0, 0).
const renderer = new THREE.WebGLRenderer({ canvas: $('canvas'), alpha: true, antialias: true, powerPreference: 'high-performance' });
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local');
renderer.setSize(innerWidth, innerHeight);
addEventListener('resize', () => renderer.setSize(innerWidth, innerHeight));

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(70, innerWidth / innerHeight, 0.05, 40);
const root = new THREE.Group();
root.matrixAutoUpdate = false;
root.visible = false;
scene.add(root);

root.add(new THREE.AxesHelper(0.15));
const cube = new THREE.Mesh(
  new THREE.BoxGeometry(0.04, 0.04, 0.04),
  new THREE.MeshBasicMaterial({ color: 0x38bdf8, wireframe: true }),
);
cube.position.set(0.06, 0.06, 0.02);
root.add(cube);

const looseGroup = new THREE.Group();
root.add(looseGroup);
const looseMeshes = new Map();

function upsertLooseTag(tagId, lFromTag, edgeMm = 20) {
  let group = looseMeshes.get(tagId);
  if (!group) {
    const size = edgeMm / 1000;
    group = new THREE.Group();
    group.matrixAutoUpdate = false;
    group.add(new THREE.AxesHelper(size * 1.4));
    group.add(new THREE.Mesh(
      new THREE.PlaneGeometry(size, size),
      new THREE.MeshBasicMaterial({ color: 0x4ade80, wireframe: true, side: THREE.DoubleSide }),
    ));
    looseGroup.add(group);
    looseMeshes.set(tagId, group);
  }
  group.matrix.fromArray(lFromTag);
  group.matrixWorldNeedsUpdate = true;
}

function clearLooseTags() {
  for (const g of looseMeshes.values()) looseGroup.remove(g);
  looseMeshes.clear();
}

// Minimal head-locked 3D status card shown until the native XRAnchor locks.
const hudRig = new THREE.Group();
hudRig.visible = false;
scene.add(hudRig);
const _viewerMat4 = new THREE.Matrix4();
const hudCanvas = document.createElement('canvas');
hudCanvas.width = 840;
hudCanvas.height = 184;
const hudCtx = hudCanvas.getContext('2d');
const hudTex = new THREE.CanvasTexture(hudCanvas);
hudTex.generateMipmaps = false;
hudTex.minFilter = THREE.LinearFilter;
hudTex.magFilter = THREE.LinearFilter;
hudTex.colorSpace = THREE.SRGBColorSpace;
const hudCard = new THREE.Mesh(
  new THREE.PlaneGeometry(0.42, 0.092),
  new THREE.MeshBasicMaterial({ map: hudTex, transparent: true, depthTest: false, depthWrite: false }),
);
hudCard.position.set(0, -0.125, -1.0);
hudCard.lookAt(0, 0, 0);
hudCard.renderOrder = 999;
hudRig.add(hudCard);

function roundPath(ctx, x, y, w, h, r) {
  ctx.beginPath();
  if (typeof ctx.roundRect === 'function') ctx.roundRect(x, y, w, h, r);
  else ctx.rect(x, y, w, h);
}

function drawHudCard(title, hint, progress, accent, pulseStep = -1) {
  const p = Number.isFinite(progress) ? Math.max(0, Math.min(1, progress)) : null;
  const key = `${title}|${hint}|${p === null ? '' : Math.round(p * 1000)}|${accent}|${pulseStep}`;
  if (key === hudKey) return;
  hudKey = key;
  const W = hudCanvas.width;
  const H = hudCanvas.height;
  hudCtx.clearRect(0, 0, W, H);
  roundPath(hudCtx, 4, 4, W - 8, H - 8, 24);
  hudCtx.fillStyle = 'rgba(11, 15, 20, 0.86)';
  hudCtx.fill();
  hudCtx.save();
  hudCtx.globalAlpha = 0.55;
  hudCtx.strokeStyle = accent;
  hudCtx.lineWidth = 3;
  hudCtx.stroke();
  hudCtx.restore();

  hudCtx.fillStyle = '#f8fafc';
  hudCtx.font = 'bold 32px system-ui, -apple-system, sans-serif';
  hudCtx.textBaseline = 'top';
  hudCtx.fillText(title, 30, 26, 780);

  hudCtx.fillStyle = '#cbd5e1';
  hudCtx.font = '24px system-ui, -apple-system, sans-serif';
  hudCtx.fillText(hint, 30, 74, 780);

  roundPath(hudCtx, 30, 142, 780, 10, 5);
  hudCtx.fillStyle = 'rgba(255, 255, 255, 0.14)';
  hudCtx.fill();
  if (p !== null && p > 0) {
    roundPath(hudCtx, 30, 142, Math.min(780, Math.max(10, 780 * p)), 10, 5);
    hudCtx.fillStyle = accent;
    hudCtx.fill();
  } else if (pulseStep >= 0) {
    const barW = Math.round(780 * 0.28);
    const barX = 30 + Math.round((780 - barW) * ((((pulseStep % 4) + 4) % 4) / 3));
    roundPath(hudCtx, barX, 142, barW, 10, 5);
    hudCtx.fillStyle = accent;
    hudCtx.fill();
  }
  hudTex.needsUpdate = true;
}

// 3. Session state + reference camera + calibration/anchor controllers.
let session = null;
let space = null;
let sessionId = null;
let sessionCount = 0;
let epoch = 0;
let profile = null;
let assistant = null;
let history = [];
let forceRecalibrate = false;
let exitHold = { sinceMs: null, progress: 0, fire: false };
let cameraPermState = 'unknown';
let seenRefTags = 0;
let lastObsAt = 0;
let lockedBannerUntil = 0;
let lockedSpreadText = '?';
let hudKey = '';

const obsBuffer = createObservationBuffer({ maxAgeMs: 650, maxItems: 16 });
const context = () => ({ sessionId, spaceEpoch: epoch });

const capture = createReferenceCamera({
  video: $('video'),
  geometry: rig,
  getContext: context,
  getSettings: () => {
    const asPhase = assistant?.phase?.() ?? 'idle';
    const anPhase = anchor.phase();
    const calibratingOrLocking = asPhase !== 'ready' || anPhase === 'collecting' || anPhase === 'ready';
    return {
      horizontalFovDeg: 65,
      printScale: 1,
      maxLongEdge: 1280,
      experimentalCalibration: profile,
      decimate: calibratingOrLocking ? 1 : 2,
      maxHz: calibratingOrLocking ? 8 : 10,
      detectMode: 'two-stage',
      trackRefIds: calibratingOrLocking && asPhase !== 'ready' ? rigTagIds() : null,
      solveUnassigned: Boolean(profile && $('trackLoose').checked && !calibratingOrLocking),
    };
  },
  onError: (err) => setStatus(`Camera error: ${err.message}`),
  onObservation: (observation, sample) => {
    if (!session) return;
    const now = performance.now();
    const ids = rigTagIds();
    seenRefTags = (observation.tags ?? []).filter((t) => ids.includes(t.id)).length;
    lastObsAt = now;
    assistant?.observe(observation, sample, history, now);
    if (profile) obsBuffer.push({ observation, sample });
  },
});

const anchor = createLAnchorController({
  quality: { softReject: true, maxRmsPx: 1.15 },
  onCreated: (s) => {
    obsBuffer.clear();
    if (!$('trackLoose').checked) capture.stop();
    lockedSpreadText = s.seed?.positionSpreadMm?.toFixed(1) ?? '?';
    lockedBannerUntil = performance.now() + 1800;
    setStatus(`Anchored (±${lockedSpreadText} mm) · ${$('trackLoose').checked ? 'Scanning loose tags' : '90 FPS native anchor'}`);
  },
  onEvent: (type, detail) => {
    if (type === 'anchor-tracking-lost') setStatus('Anchor tracking lost · press Trigger to re-anchor');
    else if (type === 'anchor-create-failed') setStatus(`Anchor failed: ${detail.error}`);
    else if (type === 'anchor-unavailable') setStatus('WebXR Anchors unsupported in this browser.');
  },
});
anchor.setTagIds(rigTagIds());

// 4. UI controls & WebXR startup.
$('rigSelect').onchange = () => {
  if (session) return;
  rig = $('rigSelect').value === 'tablet' ? rigTablet : rigA4;
  anchor.setTagIds(rigTagIds());
  capture.setGeometry(rig);
  setStatus(`Selected ${rig.id}. Tap Enter AR.`);
};

$('trackLoose').onchange = () => {
  const anPhase = anchor.phase();
  if (!session || (anPhase !== 'tracking' && anPhase !== 'created')) return;
  if ($('trackLoose').checked) capture.start().catch(() => {});
  else { capture.stop(); clearLooseTags(); }
};

$('recalibrate').onclick = async () => {
  if (!session) {
    forceRecalibrate = true;
    setStatus('Next AR session will run a fresh calibration.');
    return;
  }
  profile = null;
  history = [];
  seenRefTags = 0;
  lastObsAt = 0;
  lockedBannerUntil = 0;
  obsBuffer.clear();
  clearLooseTags();
  if (!capture.isRunning()) await capture.start();
  anchor.reanchor(performance.now());
  await assistant?.begin({ force: true });
};

$('end').onclick = () => session?.end();

async function ensureAssistant() {
  if (capture.isRunning() && assistant) return true;
  profile = null;
  if (!(await capture.start())) return false;
  const { sourceSize } = capture.state();
  const scale = Math.min(1, 1280 / Math.max(...sourceSize));
  assistant?.dispose();
  assistant = createCalibrationAssistant({
    rig,
    device: await deviceIdentity(capture.state(), rig),
    imageSize: [Math.round(sourceSize[0] * scale), Math.round(sourceSize[1] * scale)],
    onChange: (s) => setStatus(`${s.phase} · ${s.count}/${s.required} · ${s.message}`),
    onProfile: (p) => { profile = p; },
  });
  return true;
}

function cleanupSession(ended) {
  if (session !== ended) return;
  space?.removeEventListener?.('reset', onSpaceReset);
  assistant?.dispose();
  assistant = null;
  capture.stop();
  anchor.end(performance.now());
  obsBuffer.clear();
  clearLooseTags();
  session = null;
  space = null;
  profile = null;
  history = [];
  seenRefTags = 0;
  lastObsAt = 0;
  lockedBannerUntil = 0;
  hudKey = '';
  hudRig.visible = false;
  root.visible = false;
  $('enter').hidden = false;
  $('enter').disabled = !arSupported;
  $('end').hidden = true;
  $('rigSelect').disabled = false;
  setStatus('Session ended.');
}

function onSpaceReset() {
  epoch++;
  history = [];
  obsBuffer.clear();
  if (space) anchor.resetReference(space, context(), performance.now());
}

$('enter').onclick = async () => {
  if (session || !arSupported) return;
  $('enter').disabled = true;
  $('rigSelect').disabled = true;
  let acquired = null;
  try {
    const xrOpts = { optionalFeatures: ['anchors', 'local-floor'] };
    if (cameraPermState === 'granted') {
      acquired = await navigator.xr.requestSession('immersive-ar', xrOpts);
      if (!(await ensureAssistant())) {
        await acquired.end().catch(() => {});
        $('enter').disabled = !arSupported;
        $('rigSelect').disabled = false;
        return;
      }
    } else {
      if (!(await ensureAssistant())) {
        $('enter').disabled = !arSupported;
        $('rigSelect').disabled = false;
        return;
      }
      cameraPermState = 'granted';
      try {
        acquired = await navigator.xr.requestSession('immersive-ar', xrOpts);
      } catch (xrErr) {
        if (xrErr?.name === 'SecurityError' || xrErr?.name === 'NotAllowedError') {
          setStatus('Camera ready — tap Enter AR.');
          $('enter').disabled = !arSupported;
          $('rigSelect').disabled = false;
          return;
        }
        throw xrErr;
      }
    }
    session = acquired;
    sessionId = `min-${++sessionCount}`;
    epoch = 0;
    history = [];
    seenRefTags = 0;
    lastObsAt = 0;
    lockedBannerUntil = 0;
    obsBuffer.clear();
    clearLooseTags();
    exitHold = { sinceMs: null, progress: 0, fire: false };

    acquired.addEventListener('end', () => cleanupSession(acquired), { once: true });
    acquired.addEventListener('select', () => {
      const asPhase = assistant?.phase?.() ?? 'idle';
      const anPhase = anchor.phase();
      if (asPhase === 'check-failed') { assistant.retryCheck(); return; }
      if (asPhase === 'failed') { assistant.retry(); return; }
      if (asPhase === 'ready' && (anPhase === 'lost' || anPhase === 'failed')) {
        if (!capture.isRunning()) capture.start().catch(() => {});
        anchor.reanchor(performance.now());
      }
    });

    await renderer.xr.setSession(acquired);
    if (session !== acquired) return;
    space = renderer.xr.getReferenceSpace();
    space.addEventListener('reset', onSpaceReset);
    anchor.begin(acquired, space, context(), performance.now());
    const useForce = forceRecalibrate;
    forceRecalibrate = false;
    await assistant.begin({ force: useForce });

    $('enter').hidden = true;
    $('end').hidden = false;
  } catch (err) {
    if (acquired && !session) await acquired.end().catch(() => {});
    setStatus(`Start failed: ${err.message}`);
    $('enter').disabled = !arSupported;
    $('rigSelect').disabled = false;
  }
};

function updateHud(pose, viewerTracked, asPhase, anPhase, hasWorld, now) {
  if (!pose) {
    hudRig.visible = false;
    return;
  }
  _viewerMat4.fromArray(pose.transform.matrix);
  hudRig.position.setFromMatrixPosition(_viewerMat4);
  hudRig.quaternion.setFromRotationMatrix(_viewerMat4);

  const tagsNow = (now - lastObsAt < 1000) ? seenRefTags : 0;
  const idsText = rigTagIds().join(' / ');
  const pulseStep = Math.floor(now / 300) % 4;

  if (exitHold.progress > 0) {
    drawHudCard('EXITING AR…', 'Keep holding to exit', exitHold.progress, '#f59e0b');
  } else if (!viewerTracked) {
    drawHudCard('TRACKING PAUSED', 'Waiting for Quest 6DOF tracking…', null, '#f59e0b');
  } else if (asPhase !== 'ready') {
    const s = assistant?.summary?.() ?? { count: 0, required: 1, message: '' };
    if (asPhase === 'idle') {
      drawHudCard('STARTING…', 'Preparing camera & calibration', null, '#38bdf8', pulseStep);
    } else if (asPhase === 'scanning') {
      const hint = tagsNow < 2
        ? `Aim at L-angle (${idsText}) — keep all 3 tags in view`
        : (s.message || 'Move head around L-angle (all axes & distance)');
      drawHudCard(`CALIBRATING · ${s.count}/${s.required} · ${tagsNow}/3 tags`, hint, s.count / Math.max(1, s.required), '#38bdf8');
    } else if (asPhase === 'solving' || asPhase === 'saving') {
      drawHudCard(asPhase === 'solving' ? 'SOLVING CALIBRATION…' : 'SAVING PROFILE…', s.message || 'Hold on a moment', null, '#38bdf8', pulseStep);
    } else if (asPhase === 'checking' || asPhase === 'verifying') {
      const label = asPhase === 'checking' ? 'VERIFYING PROFILE' : 'VERIFYING FIT';
      const hint = tagsNow < 3
        ? `Aim at L-angle (${idsText}) — all 3 tags required`
        : (s.count >= s.required && s.message ? s.message : 'Look at L-angle & shift head slightly (> 4 cm)');
      drawHudCard(`${label} · ${s.count}/${s.required} · ${tagsNow}/3 tags`, hint, s.count / Math.max(1, s.required), '#38bdf8');
    } else if (asPhase === 'check-failed') {
      drawHudCard('VERIFICATION FAILED', 'Pinch / Trigger to retry · shift head slightly on L', 0, '#f87171');
    } else {
      drawHudCard('CALIBRATION FAILED', 'Pinch / Trigger to retry · sweep wider around L', 0, '#f87171');
    }
  } else if (!hasWorld) {
    const q = anchor.summary().quality;
    if (anPhase === 'collecting' || anPhase === 'ready') {
      const hint = tagsNow < 3
        ? `Aim at L-angle (${idsText}) — all 3 tags required`
        : (q.count >= q.required && q.reason ? q.reason : 'Hold steady on L-angle…');
      drawHudCard(`LOCKING ANCHOR · ${q.count}/${q.required} · ${tagsNow}/3 tags`, hint, q.count / Math.max(1, q.required), '#38bdf8');
    } else if (anPhase === 'creating') {
      drawHudCard('CREATING ANCHOR…', 'Locking native WebXR XRAnchor', null, '#38bdf8', pulseStep);
    } else if (anPhase === 'lost') {
      drawHudCard('ANCHOR TRACKING LOST', 'Pinch / Trigger to re-anchor on L-angle', 0, '#f59e0b');
    } else if (anPhase === 'failed') {
      drawHudCard('ANCHOR FAILED', 'Pinch / Trigger to retry', 0, '#f87171');
    } else if (anPhase === 'unsupported') {
      drawHudCard('XRANCHORS UNSUPPORTED', 'Browser does not expose WebXR Anchors', 0, '#f87171');
    } else {
      hudRig.visible = false;
      return;
    }
  } else if (now < lockedBannerUntil) {
    drawHudCard(`ANCHORED · ±${lockedSpreadText} mm`, $('trackLoose').checked ? 'Tracking loose tags' : '90 FPS native XRAnchor · camera off', 1, '#4ade80');
  } else {
    hudRig.visible = false;
    return;
  }
  hudRig.visible = true;
}

// 5. WebXR per-frame loop: update anchor, associate delayed optical frames, render.
renderer.setAnimationLoop((time, frame) => {
  if (!frame || !session || !space) return;
  const now = performance.now();
  const pose = frame.getViewerPose(space);
  const viewerTracked = Boolean(pose && !pose.emulatedPosition);
  const world = anchor.update(frame, now, { viewerTracked });
  const anPhase = anchor.phase();
  const asPhase = assistant?.phase?.() ?? 'idle';

  if (asPhase !== 'ready' || anPhase === 'collecting' || anPhase === 'ready' || $('trackLoose').checked) {
    history.push({
      ...context(),
      xrRafTimeMs: time,
      emulatedPosition: Boolean(pose?.emulatedPosition),
      worldFromViewer: viewerTracked ? Array.from(pose.transform.matrix) : null,
      worldFromAnchor: world,
      anchorTrackingState: anPhase,
    });
    if (history.length > (asPhase !== 'ready' ? 12000 : 300)) history.splice(0, 200);
  }

  if (profile) {
    const drained = obsBuffer.drain(history, profile, now);
    if (anPhase === 'collecting' || anPhase === 'ready') {
      for (const item of drained) {
        const tool = item.observation.tools[0];
        anchor.observe({
          sample: item.sample,
          tool,
          candidate: item.candidate,
          worldFromL: tool ? product(item.candidate.worldFromViewer, profile.viewerFromCameraGL, cvToGL(tool.cameraFromTool)) : null,
        }, now);
      }
    } else if ($('trackLoose').checked && (anPhase === 'tracking' || anPhase === 'created') && drained.length) {
      const latest = drained.at(-1);
      const assoc = associateAnchorCapture(history, latest.sample, profile, now);
      if (assoc) {
        const lFromCamera = product(inverse(assoc.worldFromAnchor), assoc.worldFromCamera);
        for (const entry of latest.observation.unassignedDetections ?? []) {
          if (entry.pose) upsertLooseTag(entry.id, product(lFromCamera, cvToGL(entry.pose)), entry.edgeMm);
        }
      }
    }
  }

  root.visible = Boolean(world);
  if (world) {
    root.matrix.fromArray(world);
    root.matrixWorldNeedsUpdate = true;
  }

  exitHold = nextExitHold(exitHold, xrExitPressed(session.inputSources).held, now);
  if (exitHold.fire) session.end();
  updateHud(pose, viewerTracked, asPhase, anPhase, Boolean(world), now);
  renderer.render(scene, camera);
});

const arSupported = Boolean(navigator.xr) && await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
try {
  const perm = await navigator.permissions?.query?.({ name: 'camera' });
  if (perm) cameraPermState = perm.state;
} catch { /* ignore unsupported Permissions query */ }
$('enter').disabled = !arSupported;
setStatus(arSupported ? 'Ready. Tap Enter AR.' : 'WebXR immersive-ar unavailable in this browser. Open in Meta Quest Browser.');

addEventListener('pagehide', () => {
  assistant?.dispose();
  anchor.end(performance.now());
  capture.dispose();
  session?.end?.().catch(() => {});
});
