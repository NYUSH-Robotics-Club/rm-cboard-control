'use strict';
const sceneElement = document.querySelector('a-scene');
const enterButton = document.getElementById('enter-immersive');
const entryNotice = document.getElementById('notice');
let entryPending = false;
let appReady = false;
let immersiveSupported;

function entryStatus(stage, message) {
  entryNotice.textContent = message;
  if (!window.Studio) return;
  // Refresh only the diagnostics credential. Do not replay vehicle commands.
  Studio.init().then(() => Studio.post('/api/xr-entry', {stage, message})).catch(() => {});
}

function showEntryError(error) {
  const detail = error?.cause ?? error;
  const name = detail?.name || 'Error';
  const info = detail?.message || String(detail);
  entryStatus('error', `进入 VR 失败（${name}）：${info}`);
}

enterButton.addEventListener('click', async () => {
  if (entryPending) return;
  if (!window.isSecureContext || !navigator.xr || immersiveSupported === false) {
    entryStatus('unsupported', '此窗口无法启动沉浸式 WebXR。请用 Quest 浏览器直接打开此页面。');
    return;
  }
  if (!appReady || !sceneElement.hasLoaded || !sceneElement.renderer) {
    entryStatus('loading', 'VR 场景还在加载，请等几秒后再点一次。');
    return;
  }
  entryPending = true;
  enterButton.disabled = true;
  const timer = setTimeout(() => {
    entryStatus('waiting', '等待 Quest 浏览器进入沉浸模式。若弹出空间追踪许可，请在头显内确认。');
  }, 5000);
  try {
    // Call from the actual click, before awaiting network or permission checks.
    const request = sceneElement.enterVR();
    entryStatus('requested', '正在进入沉浸式 VR…');
    await request;
    if (!sceneElement.renderer.xr.getSession()) {
      throw new Error('浏览器没有建立沉浸式会话');
    }
  } catch (error) {
    showEntryError(error);
  } finally {
    clearTimeout(timer);
    entryPending = false;
    enterButton.disabled = false;
  }
});

(async () => {
  await Studio.init();
  const scene=document.querySelector('a-scene'), canvas=document.getElementById('panel-canvas'), panel=document.getElementById('panel'), notice=document.getElementById('notice');
  const panelRoot = document.getElementById('panel-root').object3D;
  const cameraRig = document.getElementById('camera').object3D;
  const rightPointer = document.getElementById('right-pointer').object3D;
  const previewPanelPosition = panelRoot.position.clone();
  const previewCameraPosition = cameraRig.position.clone();
  let panelNeedsPlacement = false;
  let observedReferenceSpace = null;
  let layout=(await Studio.get('/api/layout')).layout, revision=0, sessionId=null, seq=0, inflight=false, lastSend=0, lastDraw=0, uiEvents=0, connection='', entering=false, rtt=0;
  const texture=new AFRAME.THREE.CanvasTexture(canvas); texture.colorSpace=AFRAME.THREE.SRGBColorSpace;
  const videoCanvas=document.createElement('canvas');videoCanvas.width=960;videoCanvas.height=540;
  const videoTexture=new AFRAME.THREE.CanvasTexture(videoCanvas);videoTexture.colorSpace=AFRAME.THREE.SRGBColorSpace;
  const videoPanel=document.getElementById('video-panel');
  function attachVideo(){const mesh=videoPanel.getObject3D('mesh');if(mesh){mesh.material.map=videoTexture;mesh.material.needsUpdate=true;}}
  videoPanel.addEventListener('object3dset',attachVideo);attachVideo();
  Studio.startVideo(videoCanvas,()=>{videoTexture.needsUpdate=true;});
  const buttonLabels = {};
  for (const [id, text, color] of [
    ['calibration-button', '重新校准', '#345b84'],
    ['arm-button', '启用控车', '#1c6856'],
    ['stop-button', '停止控车', '#853e46']
  ]) {
    const c = document.createElement('canvas');
    c.width = 500; c.height = 156;
    const ctx = c.getContext('2d');
    const t=new AFRAME.THREE.CanvasTexture(c);t.colorSpace=AFRAME.THREE.SRGBColorSpace;const el=document.getElementById(id);
    const bind=()=>{const m=el.getObject3D('mesh');if(m){m.material.map=t;m.material.needsUpdate=true;}};el.addEventListener('object3dset',bind);bind();
    buttonLabels[id] = label => {
      ctx.fillStyle = color;
      ctx.fillRect(0, 0, c.width, c.height);
      ctx.fillStyle = '#ffffff';
      ctx.font = '44px system-ui';
      ctx.textAlign = 'center';
      ctx.fillText(label, 250, 93);
      t.needsUpdate = true;
    };
    buttonLabels[id](text);
  }
  function attach() { const mesh=panel.getObject3D('mesh'); if(mesh){mesh.material.map=texture;mesh.material.needsUpdate=true;} }
  panel.addEventListener('object3dset',attach); attach();
  const state={fresh:false,frame:null,ui_events:0};

  function showCalibration(calibration) {
    if (!calibration) return;
    if (calibration.prompt !== state.calibration?.prompt) state.notice = '';
    if (calibration.stage !== state.calibration?.stage) {
      const collecting = calibration.stage === 'repository';
      buttonLabels['calibration-button'](collecting ? '按右扳机确认' : '重新校准');
      buttonLabels['arm-button'](calibration.stage === 'ready' ? '启用控车' : '校准后启用');
      buttonLabels['stop-button'](collecting ? '取消校准' : '停止控车');
      state.notice = '';
    }
    state.calibration = calibration;
  }
  async function sync(){try{const r=await Studio.get('/api/layout');if(r.revision!==revision){layout=r.layout;revision=r.revision;}const status=await Studio.get('/api/status');state.control=status.control;state.publisher=status.publisher;showCalibration(status.calibration);}catch(e){notice.textContent=e.message;}setTimeout(sync,500);}
  sync();
  if (!window.isSecureContext || !navigator.xr) {
    notice.textContent = '电脑预览。请使用 Quest 浏览器进入 VR。';
  } else {
    navigator.xr.isSessionSupported('immersive-vr').then(supported => {
      immersiveSupported = supported;
      if (!entryPending && !sessionId) {
        notice.textContent = supported ? '平视前方，点击上方按钮进入 VR。' : '当前浏览器不支持沉浸式 VR。';
      }
    }).catch(showEntryError);
  }
  function resetVisual() {
    state.fresh = false;
    state.frame = null;
    rightPointer.visible = false;
    triggerReleased = triggerConsumed = false;
    enableOnRelease = null;
    calibrationEdges.length = 0;
    lastCalibrationEdge = null;
  }
  function onReferenceReset() {
    resetVisual();
    panelNeedsPlacement = true;
    panelRoot.visible = false;
    if (sessionId) {
      Studio.post('/api/calibration', {action:'invalidate', session:sessionId})
        .then(showCalibration).catch(error => { state.notice = error.message; });
    }
  }
  scene.addEventListener('enter-vr',async()=>{
    if(entering)return; entering=true; resetVisual();
    const xrSession = scene.renderer.xr.getSession();
    if (xrSession) {
      // The local-floor pose already carries eye height. The desktop preview's
      // camera rig offset would otherwise translate the rendered XR camera.
      cameraRig.position.set(0, 0, 0);
      panelRoot.visible = false;
      panelNeedsPlacement = true;
    }
    try{
      // A browser tab can outlive a PC service restart. Refresh its token
      // after the immersive request, since the browser needs a direct click.
      await Studio.init();
      if (xrSession && scene.renderer.xr.getSession() !== xrSession) return;
      const {session} = await Studio.post('/api/session', {});
      if (xrSession && scene.renderer.xr.getSession() !== xrSession) {
        await Studio.post('/api/close', {session});
        return;
      }
      sessionId = session;
      seq = 0;
      connection = '';
      entryStatus('entered', '已进入 VR，等待头显和双手柄位姿。');
      xrSession?.addEventListener('visibilitychange',()=>{if(xrSession.visibilityState!=='visible')resetVisual();});
    }catch(e){
      connection=e.message;
      try { await scene.exitVR(); } finally { showEntryError(e); }
    }finally{entering=false;}
  });
  scene.addEventListener('exit-vr',()=>{
    const old = sessionId;
    sessionId = null;
    resetVisual();
    observedReferenceSpace?.removeEventListener('reset', onReferenceReset);
    observedReferenceSpace = null;
    panelNeedsPlacement = false;
    cameraRig.position.copy(previewCameraPosition);
    scene.camera.position.set(0, 0, 0);
    scene.camera.quaternion.identity();
    panelRoot.position.copy(previewPanelPosition);
    panelRoot.rotation.set(0, 0, 0);
    panelRoot.visible = true;
    if(old)Studio.post('/api/close',{session:old}).catch(()=>{});
    notice.textContent='已退出 VR；双手位姿已失效。';
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden)resetVisual();});
  const pointerRay = new AFRAME.THREE.Raycaster(undefined, undefined, .01, 5);
  const rayOrigin = new AFRAME.THREE.Vector3();
  const rayDirection = new AFRAME.THREE.Vector3();
  const rayRotation = new AFRAME.THREE.Quaternion();
  const pointerTargets = ['calibration-button', 'arm-button', 'stop-button', 'panel']
    .map(id => document.getElementById(id));
  let triggerReleased = false;
  let triggerConsumed = false;
  let enableOnRelease = null;
  let lastCalibrationEdge = null;
  const calibrationEdges = [];

  function confirmTarget(id, uv) {
    const session = sessionId;
    const report = error => { state.notice = error.message; };
    if (id === 'calibration-button') {
      Studio.post('/api/repository-calibration', {action:'begin', session})
        .then(() => showCalibration({stage:'repository', prompt:'松开右扳机，再按一次开始采右向'}))
        .catch(report);
      return true;
    }
    if (id === 'arm-button') {
      if (state.calibration?.stage !== 'ready') {
        state.notice = '请先完成本次校准，用右扳机保存后再启用。';
      } else {
        enableOnRelease = {session, released: false};
        state.notice = '已点中启用控车，请松开两只扳机。';
      }
      return true;
    }
    if (id === 'stop-button') {
      enableOnRelease = null;
      if (state.calibration?.stage === 'repository') {
        Studio.post('/api/calibration', {action:'invalidate', session})
          .then(showCalibration).catch(report);
      } else {
        Studio.post('/api/control', {action:'disarm', session}).catch(report);
      }
      return true;
    }
    if (!uv || state.control?.armed) return false;
    const x = uv.x * 960, y = (1 - uv.y) * 600;
    const widget = [...layout.widgets].reverse().find(w => w.type === 'button' &&
      x >= w.x && x <= w.x + w.w && y >= w.y && y <= w.y + w.h);
    if (!widget) return false;
    Studio.post('/api/ui-event', {session, widget:widget.id, hand:'right'})
      .then(result => { uiEvents = result.count; }).catch(report);
    return true;
  }

  function updateRightPointer(targetRay) {
    rightPointer.visible = false;
    if (!targetRay || !panelRoot.visible) return null;
    const {position, orientation} = targetRay;
    rayOrigin.set(position.x, position.y, position.z);
    rayRotation.set(orientation.x, orientation.y, orientation.z, orientation.w);
    rayDirection.set(0, 0, -1).applyQuaternion(rayRotation);
    pointerRay.set(rayOrigin, rayDirection);
    panelRoot.updateWorldMatrix(true, true);
    const meshes = pointerTargets.map(target => target.getObject3D('mesh')).filter(Boolean);
    const hit = pointerRay.intersectObjects(meshes, false)[0];
    if (!hit) return null;
    const target = pointerTargets.find(element => element.getObject3D('mesh') === hit.object);
    rightPointer.position.copy(hit.point).addScaledVector(rayDirection, -.008);
    rightPointer.visible = true;
    return {id: target.id, uv: hit.uv};
  }

  function rightTriggerUI(hand, target) {
    if (!sessionId || !hand.tracked) {
      triggerReleased = triggerConsumed = false;
      enableOnRelease = null;
      return;
    }
    if (hand.trigger <= .20) {
      triggerReleased = true;
      triggerConsumed = false;
      if (enableOnRelease) enableOnRelease.released = true;
      return;
    }
    if (hand.trigger >= .65 && triggerReleased) {
      triggerReleased = false;
      enableOnRelease = null;
      const calibrating = state.calibration?.stage === 'repository';
      if (target && (!calibrating || target.id === 'stop-button')) {
        triggerConsumed = confirmTarget(target.id, target.uv);
      } else if (!calibrating && !state.control?.armed) {
        state.notice = '请用右手柄射线指到按钮，再松开并按一下右扳机。';
      }
    }
    // A UI confirmation can't be interpreted as a simultaneous drive input.
    return triggerConsumed;
  }

  // Calibration taps can occur between periodic uploads. Keep only recent,
  // ordered calibration edges. Never queue a vehicle motion sample.
  function sendInput(time, payload, rawHands, uiPress) {
    const owner = sessionId;
    const right = rawHands.right;
    const collecting = state.calibration?.stage === 'repository' && !state.control?.armed;
    if (!owner || !collecting || !right.tracked || uiPress) {
      calibrationEdges.length = 0;
      lastCalibrationEdge = null;
    } else {
      const edge = right.trigger <= .20 ? 'up' : right.trigger >= .65 ? 'down' : null;
      if (edge && edge !== lastCalibrationEdge) {
        lastCalibrationEdge = edge;
        calibrationEdges.push({time, payload});
      }
      if (calibrationEdges.length > 4 || time - calibrationEdges[0]?.time > 100) {
        calibrationEdges.length = 0;
        lastCalibrationEdge = null;
        state.notice = '校准输入延迟，请松开右扳机再按一次。';
      }
    }
    if (enableOnRelease && (!rawHands.left.tracked || !right.tracked || rawHands.left.trigger > .20)) {
      enableOnRelease = null;
      state.notice = '启用已取消，请保持双手追踪，松开两只扳机后重试。';
    }
    const arm = enableOnRelease?.released && right.trigger <= .20 ? enableOnRelease : null;
    if (!owner || inflight || (!arm && !calibrationEdges.length && time-lastSend <= 33)) return;
    const sample = calibrationEdges.shift()?.payload ?? payload;
    inflight = true;
    lastSend = time;
    const started = performance.now();
    Studio.post('/api/frame', {session:owner, frame:sample}).then(async () => {
      if (owner !== sessionId) return;
      rtt = Math.min(10000, performance.now()-started);
      connection = '';
      if (arm && arm === enableOnRelease && arm.session === owner) {
        enableOnRelease = null;
        if (rtt > 120) {
          state.notice = '松开帧确认超时，请重新点启用控车。';
          return;
        }
        try {
          const result = await Studio.post('/api/control', {action:'arm', session:owner});
          if (owner === sessionId && result.armed) state.notice = '控车已启用。';
        } catch (error) { state.notice = error.message; }
      }
    }).catch(error => {
      if (owner !== sessionId) return;
      connection = error.message;
      notice.textContent = '数据发送异常：' + error.message;
      resetVisual();
    }).finally(() => { inflight = false; });
  }
  AFRAME.registerComponent('studio-sampler',{
    tick(time){
      const xr=scene.renderer?.xr, xrSession=xr?.getSession(), frame=xr?.getFrame(), ref=xr?.getReferenceSpace();
      const visibility=xrSession?.visibilityState||'hidden';
      // The immersive session has its own visibility. A hidden 2D tab is
      // normal on some headset runtimes while immersive frames are visible.
      const {hands, reasons, rightTargetRay} = Studio.controllerTracking(frame, ref, sessionId ? xrSession : null);
      let head=null;
      if(frame&&ref&&visibility==='visible'){
        if (ref !== observedReferenceSpace) {
          observedReferenceSpace?.removeEventListener('reset', onReferenceReset);
          ref.addEventListener('reset', onReferenceReset);
          observedReferenceSpace = ref;
        }
        const viewer=frame.getViewerPose(ref);
        if(viewer&&!viewer.emulatedPosition){
          const {position: p, orientation: q} = viewer.transform;
          head={position:[p.x,p.y,p.z],orientation:[q.x,q.y,q.z,q.w]};
          if (panelNeedsPlacement) {
            const anchor = Studio.panelAnchor(viewer.transform);
            if (anchor) {
              panelRoot.position.set(anchor.x, anchor.y, anchor.z);
              panelRoot.rotation.set(0, anchor.yaw, 0);
              panelRoot.visible = true;
              panelNeedsPlacement = false;
            }
          }
        }
      }
      const uiPress = rightTriggerUI(hands.right, updateRightPointer(rightTargetRay));
      const payload={seq:seq++,space:'webxr-local-floor-rh-m-xyzw',visibility,hands,head,rtt_ms:rtt,tracking:reasons};
      state.fresh=Boolean(sessionId&&!connection);state.frame=payload;state.ui_events=uiEvents;
      const outgoing = uiPress ? {...payload, hands:{...hands, right:{...hands.right, trigger:0}}} : payload;
      sendInput(time, outgoing, hands, uiPress);
      if(time-lastDraw>80){Studio.draw(canvas,layout,state);texture.needsUpdate=true;lastDraw=time;}
    }
  });
  scene.setAttribute('studio-sampler','');
  appReady = true;
})().catch(showEntryError);

