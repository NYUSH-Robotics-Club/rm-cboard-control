// Read the live video only, in an isolated browser; do not claim an XR session.
import {createRequire} from 'node:module';
import fs from 'node:fs';
import assert from 'node:assert/strict';
const require=createRequire(import.meta.url);
const {chromium}=require('playwright');
const browser=await chromium.launch({channel:'chrome',headless:true});
try{
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:18780/vr');
 await page.waitForFunction(()=>document.querySelector('#video-panel')?.getObject3D('mesh')?.material.map?.isCanvasTexture);
 await page.waitForTimeout(1200);
 const status=await page.evaluate(()=>fetch('/api/status').then(r=>r.json()));
 assert.equal(status.video.fresh,true);assert.ok(status.video.decoded>0);
 assert.equal(status.control.armed,false);
 const dimensions=await page.evaluate(()=>{const c=document.querySelector('#video-panel').getObject3D('mesh').material.map.image;return [c.width,c.height];});
 assert.deepEqual(dimensions,[960,540]);assert.deepEqual(errors,[]);
 for(const id of ['arm-button','stop-button'])assert.equal(await page.evaluate(id=>!!document.getElementById(id).getObject3D('mesh').material.map?.isCanvasTexture,id),true);
 await page.screenshot({path:'evidence/live-video-vr-desktop-preview.png'});
 fs.writeFileSync('evidence/live-video-browser.json',JSON.stringify({passed:true,video:status.video,texture:dimensions,errors,real_immersive_headset_tested:false},null,2));
 console.log(JSON.stringify({passed:true,video:status.video,texture:dimensions,errors}));
}finally{await browser.close();}
