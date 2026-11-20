import {createRequire} from 'node:module';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const require=createRequire(import.meta.url);
const {chromium}=require('playwright');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
async function waitForServer(path, matches) {
  const until = Date.now() + 5000;
  while (Date.now() < until) {
    const response = await fetch('http://127.0.0.1:18781' + path);
    if (matches(await response.json())) return;
    await new Promise(resolve => setTimeout(resolve, 20));
  }
  throw Error('Timed out waiting for test server ' + path + ' ' + matches);
}
const browser=await chromium.launch({channel:'chrome',headless:true});
const context=await browser.newContext({viewport:{width:1440,height:1100}});
const errors=[],external=[],checks=[];
context.on('page',p=>{p.on('pageerror',e=>errors.push(e.message));p.on('console',m=>{if(m.type()==='error')errors.push(m.text());});});
await context.route('**/*',route=>{const url=route.request().url();if(!url.startsWith('http://127.0.0.1:18781/')&&!url.startsWith('data:')&&!url.startsWith('blob:')){external.push(url);return route.abort();}return route.continue();});
try {
  const editor=await context.newPage();
  await editor.goto('http://127.0.0.1:18781/');
  await editor.waitForFunction(()=>document.querySelector('#widget').options.length===5);
  checks.push('editor loaded with five default widgets');
  const canvas=editor.locator('#preview'),box=await canvas.boundingBox();
  const sx=box.width/960,sy=box.height/600;
  await editor.mouse.move(box.x+100*sx,box.y+140*sy);await editor.mouse.down();
  await editor.mouse.move(box.x+140*sx,box.y+160*sy,{steps:8});await editor.mouse.up();
  assert.equal(await editor.locator('#x').inputValue(),'70');assert.equal(await editor.locator('#y').inputValue(),'128');
  checks.push('drag changes selected widget position');
  await editor.locator('#text').fill('左手测试面板');
  await editor.locator('#save').click();
  await editor.waitForFunction(()=>document.querySelector('#message').textContent.includes('已保存'));
  const saved=await editor.evaluate(()=>fetch('/api/layout').then(r=>r.json()));
  assert.equal(saved.layout.widgets.find(w=>w.id==='left').text,'左手测试面板');
  checks.push('visual property edit saved on server');
  const vr=await context.newPage();
  await vr.addInitScript(() => {
    Object.defineProperty(navigator, 'xr', {value: {isSessionSupported: async () => true}});
  });
  await vr.goto('http://127.0.0.1:18781/vr');
  await vr.waitForFunction(()=>document.querySelector('a-scene').hasLoaded&&document.querySelector('a-scene').components['studio-sampler']);
  await vr.waitForFunction(()=>document.querySelector('#panel').getObject3D('mesh')?.material.map?.isCanvasTexture);
  assert.equal(await vr.locator('[laser-controls]').count(),0);
  assert.equal(await vr.locator('#right-pointer').count(),1);
  checks.push('VR scene loaded with only a right-hand hit indicator and no synthetic beams');
  const anchors = await vr.evaluate(() => {
    const yaw = Math.SQRT1_2;
    return {
      seated: Studio.panelAnchor({
        position: {x: 0, y: 1.1, z: 0},
        orientation: {x: 0, y: 0, z: 0, w: 1},
      }),
      standingAndTurned: Studio.panelAnchor({
        position: {x: 2, y: 1.75, z: 3},
        orientation: {x: 0, y: yaw, z: 0, w: yaw},
      }),
      lookingUp: Studio.panelAnchor({
        position: {x: 0, y: 1.75, z: 0},
        orientation: {x: yaw, y: 0, z: 0, w: yaw},
      }),
    };
  });
  assert.ok(Math.abs(anchors.seated.y + .85 - 1.1) < .06);
  assert.equal(anchors.seated.z, -1.6);
  assert.ok(Math.abs(anchors.standingAndTurned.y + .85 - 1.75) < .06);
  assert.ok(Math.abs(anchors.standingAndTurned.x - .4) < 1e-9);
  assert.ok(Math.abs(anchors.standingAndTurned.z - 3) < 1e-9);
  assert.ok(Math.abs(anchors.standingAndTurned.yaw - Math.PI / 2) < 1e-9);
  assert.equal(anchors.lookingUp, null);
  checks.push('VR panel anchor uses real eye height and heading and waits when looking straight up');
  await vr.evaluate(() => {
    document.querySelector('a-scene').enterVR = () => Promise.reject(
      new Error('session request failed', {cause: new DOMException('test permission denial', 'NotAllowedError')})
    );
  });
  await vr.getByRole('button', {name: '进入沉浸式 VR'}).click();
  await vr.waitForFunction(() => document.querySelector('#notice').textContent.includes('NotAllowedError'));
  await waitForServer('/api/status', s => s.xr_entry?.stage === 'error');
  checks.push('VR entry renders a rejected browser request and reports it to the status API');
  const claimed = vr.waitForResponse(response => response.url().endsWith('/api/session'));
  await vr.evaluate(() => {
    Studio.token = 'expired-token-from-old-service-instance';
    document.querySelector('a-scene').dispatchEvent(new Event('enter-vr'));
  });
  assert.equal((await claimed).status(), 200);
  await waitForServer('/api/status', s => s.xr_entry?.stage === 'entered');
  await vr.evaluate(() => {
    window.originalTracking = Studio.controllerTracking;
    const draw = Studio.draw.bind(Studio);
    Studio.draw = (canvas, layout, status, ...rest) => {
      window.testView = {stage:status.calibration?.stage, notice:status.notice, prompt:status.calibration?.prompt};
      return draw(canvas, layout, status, ...rest);
    };
    const xr = document.querySelector('a-scene').renderer.xr;
    window.originalGetSession = xr.getSession;
    window.originalFrame = xr.getFrame;
    window.originalReference = xr.getReferenceSpace;
    xr.getSession = () => ({visibilityState: 'visible'});
    const ref = new EventTarget();
    xr.getReferenceSpace = () => ref;
    xr.getFrame = () => ({getViewerPose: () => ({transform: {
      position: {x:0,y:1.4,z:0}, orientation:{x:0,y:0,z:0,w:1}
    }})});
    window.testTrigger = 0;
    window.testPosition = [0, 1.1, 0];
    window.aimAt = id => document.getElementById(id).object3D.getWorldPosition(new AFRAME.THREE.Vector3());
    window.aim = window.aimAt('calibration-button');
    Studio.controllerTracking = () => ({
      hands: {left: {...Studio.blank(), tracked:true, pose:{position:[0,1,0],orientation:[0,0,0,1]}}, right: {
        ...Studio.blank(), tracked: true, trigger: window.testTrigger,
        pose: {position: window.testPosition, orientation: [0, 0, 0, 1]},
      }},
      reasons: {left: 'input-source', right: 'input-source'},
      rightTargetRay: {position: {x: window.aim.x, y: window.aim.y, z: window.aim.z + 1},
                       orientation: {x: 0, y: 0, z: 0, w: 1}},
    });
  });
  await vr.waitForFunction(() => document.getElementById('right-pointer').object3D.visible);
  const clicked = vr.waitForResponse(response => response.url().endsWith('/api/repository-calibration'));
  await vr.evaluate(() => { window.testTrigger = 1; });
  assert.equal((await clicked).status(), 200);
  await vr.waitForFunction(() => window.testView?.stage === 'repository');
  // Keep aiming at the calibration button. Each physical tap must advance once.
  for (const [position, next] of [
    [[0,1.1,0], 'right-end'], [[.20,1.1,0], 'forward-start'],
    [[0,1.1,0], 'forward-end'], [[0,1.1,-.20], 'origin'],
    [[0,1.1,0], 'save'], [[0,1.1,0], 'saved']
  ]) {
    await vr.evaluate(position => { window.testTrigger = 0; window.testPosition = position; }, position);
    await waitForServer('/api/status', s => s.frame?.hands.right.trigger === 0);
    await vr.evaluate(async () => {
      // The release must be observed by the local renderer as well.
      await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      window.testTrigger = 1;
    });
    await waitForServer('/snapshot', s => s.save_state === next);
  }
  checks.push('six right-trigger captures save the current XR calibration while aiming at the same control');
  await vr.evaluate(() => { window.testTrigger = 0; window.aim = window.aimAt('arm-button'); });
  await vr.waitForFunction(() => window.testView?.stage === 'ready');
  await vr.evaluate(() => {
    window.publisherHeartbeat = setInterval(() => Studio.post('/api/publisher-status',{connected:true}), 100);
  });
  await waitForServer('/api/status', s => s.publisher?.connected);
  await vr.evaluate(() => { window.testTrigger = 1; });
  await vr.waitForFunction(() => window.testView.notice?.includes('已点中启用控车'));
  assert.equal(await vr.evaluate(async () => (await Studio.get('/api/status')).control.armed),false);
  const enabled = vr.waitForResponse(r => r.url().endsWith('/api/control') && r.request().postDataJSON()?.action === 'arm');
  await vr.evaluate(() => { window.testTrigger = 0; });
  assert.deepEqual(await (await enabled).json(), {armed:true});
  await vr.evaluate(() => clearInterval(window.publisherHeartbeat));
  checks.push('headset arm button waits for a released frame acknowledged by the isolated server');
  await vr.evaluate(() => {
    Studio.controllerTracking = window.originalTracking;
    const xr = document.querySelector('a-scene').renderer.xr;
    xr.getSession = window.originalGetSession;
    xr.getFrame = window.originalFrame;
    xr.getReferenceSpace = window.originalReference;
  });
  checks.push('the real hit indicator and trigger select the same control with an injected XR pointing pose');
  const closed = vr.waitForResponse(response => response.url().endsWith('/api/close'));
  await vr.evaluate(() => document.querySelector('a-scene').dispatchEvent(new Event('exit-vr')));
  assert.equal((await closed).status(), 200);
  checks.push('entry refreshes a stale token before opening its workbench session');
  await vr.evaluate(()=>{window.drawnText=[];const c=document.querySelector('#panel-canvas').getContext('2d'),original=c.fillText.bind(c);c.fillText=(text,...rest)=>{window.drawnText.push(text);if(window.drawnText.length>300)window.drawnText.shift();return original(text,...rest);};});
  await editor.locator('#text').fill('保存后自动同步到头显');await editor.locator('#save').click();
  await editor.waitForFunction(()=>document.querySelector('#message').textContent.includes('已保存'));
  await vr.waitForFunction(()=>window.drawnText.includes('保存后自动同步到头显'));
  checks.push('saved PC text arrived in VR panel renderer without reloading');
  // Simulate only the PC server ingestion contract. This is not a headset acceptance test.
  await editor.bringToFront();
  await editor.evaluate(async()=>{
    const s=await Studio.post('/api/session',{});window.testSession=s.session;
    const hand={tracked:true,pose:{position:[.1,1.2,-.3],orientation:[0,0,0,1]},trigger:.7,grip:.2,buttons:[.7,.2],axes:[0,0]};
    let seq=0,busy=false;
    window.testHeartbeat=setInterval(async()=>{if(busy)return;busy=true;try{await Studio.post('/api/frame',{session:s.session,frame:{seq:seq++,space:'webxr-local-floor-rh-m-xyzw',visibility:'visible',hands:{left:hand,right:hand}}});}finally{busy=false;}},50);
  });
  await editor.waitForFunction(()=>document.querySelector('#left').textContent.includes('追踪有效')&&document.querySelector('#right').textContent.includes('追踪有效'));
  checks.push('both hand state readouts display an explicitly simulated server frame');
  await editor.evaluate(()=>clearInterval(window.testHeartbeat));
  await editor.waitForFunction(()=>document.querySelector('#left').textContent.includes('数据已过期')&&document.querySelector('#right').textContent.includes('数据已过期'));
  checks.push('both readouts invalidate after 300ms lease');
  await editor.evaluate(()=>Studio.post('/api/close',{session:window.testSession}));
  await editor.screenshot({path:path.join(root,'evidence','pc-editor.png'),fullPage:true});
  await vr.screenshot({path:path.join(root,'evidence','vr-page-desktop-preview.png'),fullPage:true});
  assert.deepEqual(external,[],'no external runtime requests');
  // Ignore only the standard missing favicon request, which carries no runtime behavior.
  const actionable=errors.filter(e=>!e.includes('404 (Not Found)'));
  assert.deepEqual(actionable,[],'browser script/console errors');
  checks.push('no external requests or browser script errors');
  fs.writeFileSync(path.join(root,'evidence','browser-checks.json'),JSON.stringify({passed:true,checks,errors,external,real_headset_tested:false},null,2));
  console.log(JSON.stringify({passed:true,checks,errors,external}));
} catch(e) {
  fs.writeFileSync(path.join(root,'evidence','browser-checks-failure.json'),JSON.stringify({passed:false,error:e.stack,checks,errors,external},null,2));
  throw e;
} finally {await context.close();await browser.close();}
