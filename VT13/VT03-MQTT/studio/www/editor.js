'use strict';
(async () => {
  await Studio.init();
  const $ = id => document.getElementById(id), canvas = $('preview');
  const controls=document.createElement('section');controls.className='vehicle-controls';
  controls.innerHTML='<div class="toolbar"><button id="arm-vehicle">启用控车</button><button id="disarm-vehicle" class="stop">停止控车</button></div><p>在头显或本地双手柄监控窗口完成校准后启用。右扳机＋位移＝平移；左扳机＋左右位移＝旋转（左移顺时针）。</p><p id="vehicle-state">控制未启用</p><canvas id="video-preview" width="960" height="540" aria-label="真实图传画面"></canvas><p id="video-state">等待视频</p>';
  document.querySelector('.preview').appendChild(controls);
  const calibrationPage = document.createElement('a');
  calibrationPage.href = '/calibration-ui';
  calibrationPage.target = '_blank';
  calibrationPage.rel = 'noopener';
  calibrationPage.textContent = '原版校准（右扳机确认）';
  controls.querySelector('.toolbar').prepend(calibrationPage);
  Studio.startVideo($('video-preview'));
  $('arm-vehicle').onclick=async()=>{try{await Studio.post('/api/control',{action:'arm'});message('已启用；左、右手分别松开扳机后再按住移动。');}catch(e){message(e.message);}};
  $('disarm-vehicle').onclick=async()=>{try{await Studio.post('/api/control',{action:'disarm'});message('已停止车辆控制；界面和监听继续运行。');}catch(e){message(e.message);}};
  let layout, revision, selected = '', status = {}, drag = null, dirty = false;
  const message = text => { $('message').textContent = text; };
  const current = () => layout.widgets.find(w => w.id === selected);
  function clamp(w) { w.w = Math.max(60, Math.min(960, w.w)); w.h = Math.max(40, Math.min(600, w.h)); w.x = Math.max(0, Math.min(960 - w.w, w.x)); w.y = Math.max(0, Math.min(600 - w.h, w.y)); }
  function draw() { Studio.draw(canvas, layout, status, selected); }
  function properties() {
    $('widget').replaceChildren(...layout.widgets.map(w => { const o = document.createElement('option'); o.value = w.id; o.textContent = w.text; return o; }));
    $('widget').value = selected;
    const w = current(); if (!w) return;
    for (const key of ['type','text','color','binding','x','y','w','h']) $(key).value = w[key];
    draw();
  }
  function changed() { dirty = true; message('有未保存的修改；点击“保存并同步到头显”。'); draw(); }
  async function load() { const r = await Studio.get('/api/layout'); layout = r.layout; revision = r.revision; selected = layout.widgets[0].id; dirty = false; properties(); message('布局已载入。拖动组件或修改右侧属性。'); }
  $('widget').onchange = () => { selected = $('widget').value; properties(); };
  for (const key of ['type','text','color','binding','x','y','w','h']) $(key).addEventListener('input', () => {
    const w = current(); if (!w) return;
    w[key] = ['x','y','w','h'].includes(key) ? Number($(key).value) || 0 : $(key).value;
    clamp(w); changed();
  });
  const point = e => { const r = canvas.getBoundingClientRect(); return {x: (e.clientX - r.left) / r.width * 960, y: (e.clientY - r.top) / r.height * 600}; };
  canvas.onpointerdown = e => {
    const p = point(e), w = [...layout.widgets].reverse().find(w => p.x >= w.x && p.x <= w.x+w.w && p.y >= w.y && p.y <= w.y+w.h);
    if (!w) return; selected = w.id;
    drag = {start: p, original: {...w}, resize: p.x > w.x+w.w-24 && p.y > w.y+w.h-24};
    canvas.setPointerCapture(e.pointerId); properties();
  };
  canvas.onpointermove = e => {
    if (!drag) return; const p = point(e), w = current(), dx = p.x-drag.start.x, dy=p.y-drag.start.y;
    if (drag.resize) { w.w = Math.round(Math.min(960-w.x, Math.max(60, drag.original.w+dx))); w.h = Math.round(Math.min(600-w.y, Math.max(40, drag.original.h+dy))); }
    else { w.x = Math.round(drag.original.x+dx); w.y = Math.round(drag.original.y+dy); clamp(w); }
    properties(); changed();
  };
  canvas.onpointerup = canvas.onpointercancel = () => { drag = null; };
  $('add').onclick = () => { if (layout.widgets.length >= 30) return message('最多 30 个组件。'); const id='widget_'+crypto.randomUUID().slice(0,8); layout.widgets.push({id,type:'label',text:'新面板',x:80,y:80,w:360,h:100,color:'#273e57',binding:'none'}); selected=id; properties(); changed(); };
  $('delete').onclick = () => { if (layout.widgets.length <= 1) return message('至少保留一个组件。'); layout.widgets=layout.widgets.filter(w=>w.id!==selected); selected=layout.widgets[0].id; properties(); changed(); };
  $('save').onclick = async () => { try { const r=await Studio.post('/api/layout',{revision,layout}); revision=r.revision; dirty=false; properties(); message(`已保存 · 版本 ${revision}，头显会自动同步。`); } catch(e) { message(e.message); } };
  $('reload').onclick = () => { if (!dirty || confirm('放弃此页面未保存的修改并重新载入？')) load().catch(e=>message(e.message)); };
  await load();
  async function poll() {
    try {
      status=await Studio.get('/api/status'); draw();
      const out=status.control||{},publisher=status.publisher;
      $('output-badge').textContent=publisher?.running&&publisher?.connected?(out.armed?'车辆控制：已启用':'车辆控制：已停止'):'车辆指令出口未启动';
      $('vehicle-state').textContent=`${out.reason||''} · 前 ${out.forward||0} / 右 ${out.right||0} / 逆时针 ${out.rotation||0} RC刻度 · MQTT ${publisher?.connected?'已连接':'未连接'}`;
      const video=status.video||{};$('video-state').textContent=`真实图传：${video.fresh?'画面有效':'等待 / 已失效'} · UDP ${video.packets||0} 包 · 解码 ${video.decoded||0} 帧 · ${video.age_ms??'—'} ms · ${video.error||''}`;
      for (const side of ['left','right']) {
        const h=status.fresh && status.frame?.hands[side];
        $(side).textContent = h?.tracked ? `追踪有效\n位置 m: ${h.pose.position.map(n=>n.toFixed(3)).join(', ')}\n旋转 xyzw: ${h.pose.orientation.map(n=>n.toFixed(3)).join(', ')}\n扳机: ${h.trigger.toFixed(2)}  握持: ${h.grip.toFixed(2)}\n轴: ${h.axes.map(n=>n.toFixed(2)).join(', ')}` : '未追踪 / 数据已过期（不显示旧位姿）';
      }
      $('health').textContent=`已收 ${status.accepted_frames} 帧 · 数据年龄 ${status.age_ms ?? '—'} ms · 上帧请求往返 ${status.fresh ? status.frame.rtt_ms?.toFixed(1) : '—'} ms（不含无线/电机延迟） · UI 点击 ${status.ui_events} 次`;
    } catch(e) { status={}; draw(); $('health').textContent='服务离线：'+e.message; for(const side of ['left','right']) $(side).textContent='服务离线，数据无效'; }
    setTimeout(poll,150);
  }
  poll();
  window.addEventListener('beforeunload', e=>{if(dirty){e.preventDefault();e.returnValue='';}});
})().catch(e=>{document.getElementById('message').textContent=e.message;});
