'use strict';
window.Studio = {
  token: '',
  async init() { const r = await this.get('/api/bootstrap'); this.token = r.token; },
  async get(path) { const r = await fetch(path, {cache: 'no-store'}); if (!r.ok) throw Error(`HTTP ${r.status}`); return r.json(); },
  async post(path, value) {
    const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Studio-Token': this.token}, body: JSON.stringify(value), signal: AbortSignal.timeout(1500)});
    const data = await r.json(); if (!r.ok) throw Error(data.error || `HTTP ${r.status}`); return data;
  },
  blank() { return {tracked: false, pose: null, trigger: 0, grip: 0, buttons: [], axes: []}; },
  panelAnchor({position, orientation}) {
    const {x, y, z, w} = orientation;
    // Project the viewer's forward axis onto the floor so the display stays upright.
    const forwardX = -2 * (x * z + w * y);
    const forwardZ = -(1 - 2 * (x * x + y * y));
    const horizontal = Math.hypot(forwardX, forwardZ);
    if (horizontal < .2) return null;
    const distance = 1.6 / horizontal;
    return {
      x: position.x + forwardX * distance,
      y: position.y - .9,
      z: position.z + forwardZ * distance,
      yaw: Math.atan2(-forwardX, -forwardZ),
    };
  },
  trackingMessage(reason) {
    return ({
      'no-source': '网页未收到手柄输入源',
      'session-not-visible': 'VR 会话不在前台',
      'no-frame': '等待 XR 帧或参考坐标',
      'bare-hand': '当前是裸手追踪',
      'no-grip-space': '输入源没有手柄位姿空间',
      'no-trigger-mapping': '没有标准扳机输入',
      'emulated-pose': '当前只有估算位置',
      'no-grip-pose': '当前帧缺少手柄位姿',
    })[reason] ?? '未追踪 / 数据已过期';
  },
  controllerTracking(frame, referenceSpace, session) {
    const hands = {left: this.blank(), right: this.blank()};
    const reasons = {left: 'no-source', right: 'no-source'};
    if (session?.visibilityState !== 'visible') {
      return {hands, reasons: {left: 'session-not-visible', right: 'session-not-visible'}};
    }
    if (!frame || !referenceSpace) {
      return {hands, reasons: {left: 'no-frame', right: 'no-frame'}};
    }
    const input = Array.from(session.inputSources);
    const tracked = session.trackedSources;
    // Quest may move a controller between these lists. Prefer the standard
    // input source, and only accept a real grip pose plus an indexed trigger.
    const sources = new Set([...input, ...(tracked ?? [])]);
    let rightTargetRay = null;
    for (const source of sources) {
      const side = source.handedness;
      if ((side !== 'left' && side !== 'right') || hands[side].tracked) continue;
      if (source.hand) {
        if (reasons[side] === 'no-source') reasons[side] = 'bare-hand';
        continue;
      }
      const gamepad = source.gamepad;
      if (source.targetRayMode !== 'tracked-pointer' || !source.gripSpace) {
        reasons[side] = 'no-grip-space';
        continue;
      }
      if (!gamepad || gamepad.mapping !== 'xr-standard' || !gamepad.buttons.length) {
        reasons[side] = 'no-trigger-mapping';
        continue;
      }
      const pose = frame.getPose(source.gripSpace, referenceSpace);
      if (!pose || pose.emulatedPosition) {
        reasons[side] = pose ? 'emulated-pose' : 'no-grip-pose';
        continue;
      }
      const {position: p, orientation: q} = pose.transform;
      const buttons = Array.from(gamepad.buttons, button => button.value);
      hands[side] = {
        tracked: true,
        pose: {position: [p.x, p.y, p.z], orientation: [q.x, q.y, q.z, q.w]},
        trigger: buttons[0],
        grip: buttons[1] ?? 0,
        buttons,
        axes: Array.from(gamepad.axes),
      };
      reasons[side] = input.includes(source) ? 'input-source' : 'tracked-source';
      if (side === 'right' && source.targetRaySpace) {
        const aimingPose = frame.getPose(source.targetRaySpace, referenceSpace);
        if (aimingPose && !aimingPose.emulatedPosition) rightTargetRay = aimingPose.transform;
      }
    }
    return {hands, reasons, rightTargetRay};
  },
  draw(canvas, layout, status, selected = '') {
    const c = canvas.getContext('2d');
    c.fillStyle = '#0b1220'; c.fillRect(0, 0, 960, 600);
    for (const w of layout.widgets) {
      c.fillStyle = w.color; c.beginPath(); c.roundRect(w.x, w.y, w.w, w.h, 12); c.fill();
      if (w.id === selected) { c.strokeStyle = '#61e2b2'; c.lineWidth = 4; c.stroke(); c.fillStyle = '#61e2b2'; c.fillRect(w.x + w.w - 16, w.y + w.h - 16, 16, 16); }
      c.save(); c.beginPath(); c.rect(w.x + 10, w.y + 6, w.w - 20, w.h - 12); c.clip();
      c.fillStyle = '#e9f1fc'; c.font = `${w.type === 'label' && w.h < 80 ? 30 : 24}px system-ui, sans-serif`;
      c.fillText(w.text, w.x + 16, w.y + 38, w.w - 32);
      if (w.type === 'status') {
        const h = status?.fresh && status.frame?.hands[w.binding];
        const valid = h?.tracked;
        c.font = '20px system-ui, sans-serif'; c.fillStyle = valid ? '#62e8bb' : '#f9bc80';
        const reason = status?.fresh ? status.frame?.tracking?.[w.binding] : undefined;
        c.fillText(valid ? '追踪有效' : this.trackingMessage(reason), w.x + 16, w.y + 74, w.w - 32);
        if (valid) {
          c.fillStyle = '#c7d6eb';
          c.fillText(`位置 m  ${h.pose.position.map(n => n.toFixed(3)).join(' / ')}`, w.x + 16, w.y + 108, w.w - 32);
          c.fillText(`扳机 ${h.trigger.toFixed(2)}   握持 ${h.grip.toFixed(2)}`, w.x + 16, w.y + 140, w.w - 32);
        }
      }
      c.restore();
    }
    c.fillStyle = '#8fa7c3'; c.font = '19px system-ui, sans-serif';
    const out=status?.control,publisher=status?.publisher;
    const collecting = status?.calibration?.stage === 'repository';
    const missingRight = collecting && !status?.frame?.hands?.right?.tracked;
    let prompt = status?.notice || status?.calibration?.prompt;
    if (missingRight) {
      const reason = this.trackingMessage(status?.frame?.tracking?.right);
      prompt = `右手：${reason}。恢复后松开右扳机，再按一次。`;
    }
    if(prompt){c.fillStyle='#f3c68e';c.fillText(prompt,30,535,900);}
    const caption=publisher?.connected?(out?.armed?`已启用：前 ${out.forward} / 右 ${out.right} / 旋转 ${out.rotation}`:'车辆控制已停止'):'车辆指令出口未连接';
    c.fillText(caption, 30, 566);
  },
  startVideo(canvas, changed=()=>{}) {
    const c=canvas.getContext('2d');
    const clear=text=>{c.fillStyle='#08111e';c.fillRect(0,0,960,540);c.fillStyle='#b8cbe1';c.font='26px system-ui';c.fillText(text,35,270);changed();};
    async function next(){
      try{
        const r=await fetch('/api/video/frame.jpg',{cache:'no-store',signal:AbortSignal.timeout(1000)});
        if(r.status===204){clear('等待真实图传画面 / 视频已过期');}
        else if(r.ok){const bitmap=await createImageBitmap(await r.blob());c.drawImage(bitmap,0,0,960,540);bitmap.close();changed();}
        else clear('图传视频服务异常');
      }catch(e){clear('视频连接中断');}
      setTimeout(next,55);
    }
    clear('等待官方 UDP 图传');next();
  }
};
