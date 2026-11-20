"""HTTP UI for live Quest trajectory calibration."""

from __future__ import annotations

import json
import queue
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlparse

from embodied_ops.teleop import atomic_write_json

from .calibration_profiles import calibration_complete, calibration_file, sanitize_profile
from .live_state import LiveState

if TYPE_CHECKING:
    from .calibration_session import CalibrationSession

HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Quest Calibration</title>
  <style>
    :root {
      --bg: #f6f6f6;
      --page: #ffffff;
      --ink: #111111;
      --muted: #666666;
      --soft: #999999;
      --line: #dddddd;
      --line-strong: #bbbbbb;
      --hover: #eeeeee;
      --red: #d44c47;
      --green: #448361;
      --blue: #337ea9;
      --shadow: 0 1px 2px rgba(0,0,0,.04);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 16px;
      height: 100vh;
      overflow: hidden;
      color: var(--ink);
      background: var(--bg);
      font-family: "Helvetica Neue", Helvetica, Arial, sans-serif;
      font-size: 14px;
    }
    .wrap { width: min(1480px, 100%); height: 100%; margin: 0 auto; display: flex; flex-direction: column; }
    header { flex: 0 0 auto; margin-bottom: 12px; }
    h1 { margin: 0; font-size: 24px; line-height: 1.2; font-weight: 600; letter-spacing: -.01em; }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--red); }
    .dot.on { background: var(--green); }
    .layout { display: grid; grid-template-columns: minmax(0, 1fr) 430px; gap: 12px; align-items: stretch; flex: 1 1 auto; min-height: 0; }
    .stage, .side { border: 1px solid var(--line); border-radius: 8px; background: var(--page); box-shadow: var(--shadow); overflow: hidden; }
    .stage { display: flex; flex-direction: column; min-height: 0; }
    .stage-head { display: flex; justify-content: space-between; align-items: center; gap: 12px; flex: 0 0 auto; padding: 10px 12px; border-bottom: 1px solid var(--line); background: #fafafa; }
    .stage-info { display: flex; flex-wrap: wrap; align-items: center; gap: 14px; min-width: 0; }
    .stage-title { font-weight: 600; }
    .legend { display: flex; flex-wrap: wrap; gap: 12px; color: var(--muted); font-size: 12px; }
    .stage-actions { display: flex; gap: 6px; flex: 0 0 auto; }
    .stage-actions button { padding: 5px 9px; font-size: 12px; }
    .swatch { display: inline-flex; align-items: center; gap: 5px; }
    .swatch::before { content: ""; width: 13px; height: 2px; border-radius: 999px; background: currentColor; }
    .right { color: var(--red); } .forward { color: var(--green); } .up { color: var(--blue); } .start { color: var(--green); } .end { color: var(--red); }
    canvas { display: block; width: 100%; height: 100%; min-height: 0; flex: 1 1 auto; background: #fff; }
    .side { height: 100%; padding: 12px; display: flex; flex-direction: column; gap: 10px; overflow-y: auto; overscroll-behavior: contain; scrollbar-gutter: stable; }
    .section-title { margin: 8px 2px 2px; color: var(--soft); font-size: 11px; font-weight: 600; text-transform: uppercase; letter-spacing: .06em; }
    .section-title:first-child { margin-top: 0; }
    .card { padding: 10px; border-radius: 7px; background: #fff; border: 1px solid var(--line); }
    .card.subtle { background: #fafafa; }
    .tool-card { display: grid; gap: 9px; }
    .metric-row { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .metric { min-width: 0; border: 1px solid var(--line); border-radius: 6px; padding: 8px; background: #fff; }
    .metric .name { color: var(--muted); font-size: 11px; font-weight: 600; margin-bottom: 4px; }
    .metric .state { display: flex; align-items: center; gap: 7px; min-height: 22px; font-size: 15px; font-weight: 650; line-height: 1.2; overflow-wrap: anywhere; }
    .metric.trigger-on { background: #111; border-color: #111; color: #fff; }
    .metric.trigger-on .name { color: #ddd; }
    .next-action { border-left: 3px solid #111; padding: 8px 10px; background: #fff; font-weight: 600; line-height: 1.35; }
    .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .profile-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; align-items: center; }
    input, select {
      width: 100%;
      border: 1px solid var(--line-strong);
      background: #fff;
      color: var(--ink);
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
      font-size: 13px;
    }
    input:focus, select:focus { outline: 2px solid #111; outline-offset: 1px; }
    button {
      appearance: none;
      border: 1px solid var(--line-strong);
      background: #fff;
      color: var(--ink);
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
      font-size: 13px;
      font-weight: 500;
      cursor: pointer;
      transition: background .12s ease, color .12s ease, border-color .12s ease;
    }
    button:hover { background: var(--hover); }
    button.primary { background: #111; border-color: #111; color: #fff; }
    button.primary:hover { background: #333; }
    button:disabled { color: #aaa; border-color: #e5e5e5; background: #fafafa; cursor: not-allowed; }
    button.full { grid-column: 1 / -1; }
    .profile-hint { color: var(--muted); font-size: 12px; line-height: 1.45; }
    .action-feedback { position: sticky; top: 0; z-index: 2; padding: 10px; border: 1px solid var(--line); border-radius: 6px; background: #fff; line-height: 1.4; overflow-wrap: anywhere; }
    .action-feedback[data-kind="success"] { border-color: #448361; color: #285c3e; background: #f3faf5; }
    .action-feedback[data-kind="error"] { border-color: #c74740; color: #952d28; background: #fff5f4; }
    @media (max-width: 980px) {
      body { height: auto; min-height: 100vh; padding: 10px; overflow: auto; }
      .wrap { width: min(100%, 760px); margin: 0 auto; }
      .wrap { height: auto; }
      .layout { display: grid; grid-template-columns: 1fr; }
      .stage, .side { height: auto; }
      .side { overflow: visible; }
      canvas { min-height: 420px; height: 58vh; }
      .stage-head { align-items: flex-start; flex-direction: column; }
      .metric-row { grid-template-columns: repeat(3, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <h1>Quest Calibration</h1>
    </header>

    <div class="layout">
      <section class="stage">
        <div class="stage-head">
          <div class="stage-info">
            <div class="stage-title">Calibration View</div>
            <div class="legend">
              <span class="swatch right">right +X</span>
              <span class="swatch forward">forward +Y</span>
              <span class="swatch up">up +Z</span>
              <span class="swatch start">start</span>
              <span class="swatch end">latest</span>
            </div>
          </div>
          <div class="stage-actions">
            <button id="fit">Fit view</button>
            <button id="clear">Clear path</button>
          </div>
        </div>
        <canvas id="canvas"></canvas>
      </section>

      <aside class="side">
        <div id="actionFeedback" class="action-feedback" role="status" aria-live="polite" aria-atomic="true" hidden></div>
        <p class="section-title">Setup</p>
        <div class="card subtle tool-card">
          <div class="metric-row">
            <div class="metric"><div class="name">Quest stream</div><div class="state"><span id="dot" class="dot"></span><span id="streamMetric">--</span></div></div>
            <div id="triggerMetric" class="metric"><div class="name">Trigger</div><div class="state" id="triggerValue">--</div></div>
            <div class="metric"><div class="name">Profile state</div><div class="state" id="profileMetric">Not loaded</div></div>
          </div>
          <div class="profile-row">
            <select id="profileSelect" aria-label="Saved calibration profiles"></select>
            <input id="profileName" value="quest_teleop_frame" spellcheck="false" aria-label="Calibration profile name" />
          </div>
          <div class="profile-row" style="margin-top:8px;">
            <button id="loadProfile">Load profile</button>
            <button id="saveProfile" class="primary" disabled>Save profile</button>
          </div>
          <div id="profileStatus" class="profile-hint" style="margin-top:8px;">Loading profiles...</div>
        </div>

        <p class="section-title">Calibration</p>
        <div id="nextAction" class="next-action">Start a new calibration.</div>
        <div class="actions">
          <button id="calibNext" class="primary full">Start new calibration</button>
          <button id="cancelCalibration" class="full" hidden>Cancel calibration</button>
        </div>
      </aside>
    </div>
  </div>

<script>
const CALIBRATION_VERSION = 5;
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const dot = document.getElementById('dot');
const streamMetric = document.getElementById('streamMetric');
const triggerMetric = document.getElementById('triggerMetric');
const triggerValueEl = document.getElementById('triggerValue');
const profileMetric = document.getElementById('profileMetric');
const nextActionEl = document.getElementById('nextAction');
const calibNextBtn = document.getElementById('calibNext');
const profileSelect = document.getElementById('profileSelect');
const profileNameInput = document.getElementById('profileName');
const loadProfileBtn = document.getElementById('loadProfile');
const saveProfileBtn = document.getElementById('saveProfile');
const profileStatus = document.getElementById('profileStatus');
let rawPoints = [];
let points = [];
let latestRaw = null;
let latest = null;
let gateOpen = false;
let pauseState = null;
let az = 0.72;
let el = 0.42;
let zoom = 1.0;
let dragging = false;
let dragStart = null;
const QUEST_UP = {x:0, y:1, z:0};
const MIN_RIGHT_M = 0.05;
let currentProfile = 'quest_teleop_frame';
let profileInitialized = false;
let calibration = null;
let editorSession;
let commandPending = false;
let pendingButton = null;
let pendingLabel = '';
let calibrationLoadVersion = 0;
let editorPollPending = false;
const actionFeedback = document.getElementById('actionFeedback');
const cancelCalibrationBtn = document.getElementById('cancelCalibration');

function notifyAction(kind, message) {
  actionFeedback.hidden = false;
  actionFeedback.dataset.kind = kind;
  actionFeedback.textContent = message;
}
async function runAction(button, label, action) {
  if (commandPending) return;
  commandPending = true; pendingButton = button; pendingLabel = label;
  calibrationLoadVersion++;
  const originalLabel = button.textContent;
  notifyAction('pending', label); updateStats();
  try {
    const message = await action();
    notifyAction('success', message || 'Done.');
  } catch (error) {
    notifyAction('error', error.message || String(error));
  } finally {
    commandPending = false; pendingButton = null; button.textContent = originalLabel;
    updateStats();
  }
}
async function requestJSON(url, options={}) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 4000);
  try {
    const response = await fetch(url, {...options, signal:controller.signal, cache:'no-store'});
    const text = await response.text();
    let result;
    try { result = JSON.parse(text); }
    catch (_) { throw Object.assign(new Error(!response.ok && text ? text.slice(0,500) : `Server returned an invalid response (HTTP ${response.status}).`), {status:response.status}); }
    if (!response.ok) throw Object.assign(new Error(result.message || `Request failed (HTTP ${response.status}).`), {status:response.status});
    return result;
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('No confirmation within 4 seconds. Check the source status before retrying; your draft is retained.');
    if (error instanceof TypeError) throw new Error('Cannot reach the source. Check the connection; your draft is retained.');
    throw error;
  } finally { clearTimeout(timer); }
}

function applyEditor(next) {
  if (!next || next.schema_version !== 'quest.calibration_editor/v1') return;
  if (editorSession && next.revision !== editorSession.revision) {
    // Never keep an old tab's in-progress geometry after another editor applied a profile.
    if (editorSession.active && !next.active) {
      loadServerCalibration(next.profile).catch(error => { profileStatus.textContent = `Profile refresh failed: ${error.message}`; });
    }
    if (next.active) { calibrationLoadVersion++; rawPoints = []; latestRaw = null; calibration = null; }
  }
  const changed = !editorSession || next.revision !== editorSession.revision;
  editorSession = next;
  if (changed && !commandPending && next.state === 'awaiting_b') {
    notifyAction('success', next.last_action === 'finish'
      ? `Saved and applied: ${next.profile}. Return to your controls, then pause/resume B.`
      : 'Editor closed. Pause/resume B to continue.');
  }
  cancelCalibrationBtn.hidden = !next.active;
  updateStats();
}
async function refreshEditor() {
  if (editorPollPending) return;
  editorPollPending = true;
  try {
    applyEditor(await requestJSON('/editor/status'));
  } catch (error) {
    if (error.status === 404) { editorSession = null; return; }
    if (editorSession) editorSession.tracking_valid = false;
    if (editorSession === undefined) notifyAction('error', 'Source unavailable. Check the service connection; the page will retry.');
  } finally { editorPollPending = false; updateStats(); }
}
async function editorCommand(action, extra={}) {
  const requestId = typeof crypto.randomUUID === 'function' ? crypto.randomUUID()
    : Array.from(crypto.getRandomValues(new Uint8Array(16)), n => n.toString(16).padStart(2,'0')).join('');
  const request = {action, request_id:requestId, revision:editorSession?.revision, ...extra};
  const result = await requestJSON('/editor/command', {method:'POST',
    headers:{'Content-Type':'application/json'}, body:JSON.stringify(request)});
  if (result.editor) applyEditor(result.editor);
  if (!result.applied) throw new Error(result.message || 'Request was not applied.');
  return result;
}

function vec(x=0, y=0, z=0) { return {x:Number(x), y:Number(y), z:Number(z)}; }
function add(a,b) { return vec(a.x+b.x, a.y+b.y, a.z+b.z); }
function sub(a,b) { return vec(a.x-b.x, a.y-b.y, a.z-b.z); }
function mul(a,s) { return vec(a.x*s, a.y*s, a.z*s); }
function dot3(a,b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
function cross(a,b) { return vec(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x); }
function norm(a) { return Math.hypot(a.x, a.y, a.z); }
function normalize(a) { const n = norm(a); return n > 1e-9 ? mul(a, 1/n) : vec(); }
function fmt(value, digits=4) { return Number.isFinite(value) ? Number(value).toFixed(digits) : '--'; }
function clonePoint(p) { return {...p}; }
function rawVec(p) { return vec(p.x, p.y, p.z); }
function profileKey() { return `questTeleopCalibration:${currentProfile}`; }
function hasRightStart() { return calibration && calibration.rightStart; }
function hasRight() { return calibration && calibration.right; }
function hasForwardStart() { return calibration && calibration.forwardStart; }
function hasForward() { return calibration && calibration.forward; }
function hasOrigin() { return calibration && calibration.origin; }
function isCalibrated() { return calibration && calibration.origin && calibration.right && calibration.forward && calibration.up && calibration.version === CALIBRATION_VERSION; }
function sanitizeProfileName(name) {
  const cleaned = String(name || '').replace(/\.json$/i, '').replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^_+|_+$/g, '');
  return cleaned || 'quest_teleop_frame';
}
function localSaveCalibration() {
  if (!calibration) return;
  // Browser storage must never prevent an explicit server save.
  try { localStorage.setItem(profileKey(), JSON.stringify(calibration)); }
  catch (_) { /* The live draft stays in memory; Finish still saves to the source. */ }
}
function loadLocalCalibration(profile=currentProfile) {
  try {
    const parsed = JSON.parse(localStorage.getItem(`questTeleopCalibration:${profile}`) || 'null');
    if (!parsed || parsed.version !== CALIBRATION_VERSION) return null;
    return parsed;
  } catch (_) { return null; }
}
function applyCalibration(data, profile=currentProfile, status='') {
  calibration = data && data.version === CALIBRATION_VERSION ? data : null;
  currentProfile = sanitizeProfileName(profile);
  profileNameInput.value = currentProfile;
  if (calibration) localSaveCalibration();
  refreshDisplayPoints(); updateStats(); draw();
  if (status) profileStatus.textContent = status;
}
function profileStateText() {
  if (!calibration) return 'No profile';
  if (!isCalibrated()) return 'Draft';
  return 'Ready';
}
function nextCalibrationAction() {
  if (!calibration || isCalibrated()) return 'Start a new calibration.';
  if (!latestRaw) return editorSession ? 'Wake the controller and keep it visible to the headset.' : 'Start Quest streaming. Press B until Quest stream shows Streaming.';
  if (!hasRightStart()) return 'Hold the controller at a comfortable start pose, then start collecting right.';
  if (!hasRight()) return 'Move the controller to your physical right, hold still, then finish right.';
  if (!hasForwardStart()) return 'Hold the controller at a comfortable start pose, then start collecting forward.';
  if (!hasForward()) return 'Move the controller forward, hold still, then finish forward.';
  return 'Hold the controller at the neutral teleop pose, then set origin.';
}
function updateToolSummary() {
  streamMetric.textContent = editorSession
    ? (editorSession.active ? (editorSession.tracking_valid ? 'Live' : 'Waiting') : 'Idle')
    : (gateOpen ? 'Streaming' : (pauseState ? `Paused (${pauseState})` : 'Waiting'));
  profileMetric.textContent = profileStateText();
}
async function saveServerCalibration() {
  if (!isCalibrated()) {
    throw new Error('Complete right, forward, and origin before saving.');
  }
  currentProfile = sanitizeProfileName(profileNameInput.value);
  calibration.profile = currentProfile;
  calibration.savedAt = new Date().toISOString();
  localSaveCalibration();
  if (editorSession) {
    const result = await editorCommand('finish', {profile:currentProfile, calibration});
    loadProfiles().catch(() => {});
    return `Saved and applied: ${result.editor.profile}. Return to your controls, then pause/resume B.`;
  }
  await requestJSON(`/calibration?profile=${encodeURIComponent(currentProfile)}`, {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(calibration),
  });
  loadProfiles().catch(() => {});
  return `Saved profile: ${currentProfile}.`;
}
function rawToDisplay(p) {
  const out = clonePoint(p);
  if (!isCalibrated()) {
    out.z = -out.z;
    out.raw = rawVec(p);
    out.teleop = vec(p.x, p.z, p.y);
    return out;
  }
  const delta = sub(rawVec(p), calibration.origin);
  const right = dot3(delta, calibration.right);
  const forward = dot3(delta, calibration.forward);
  const up = dot3(delta, calibration.up);
  out.x = right;
  out.y = up;
  out.z = -forward;
  out.raw = rawVec(p);
  out.teleop = vec(right, forward, up);
  return out;
}
function currentRightMotion() {
  if (!hasRightStart() || !latestRaw) return null;
  const rawDelta = sub(rawVec(latestRaw), calibration.rightStart);
  const horizontal = sub(rawDelta, mul(QUEST_UP, dot3(rawDelta, QUEST_UP)));
  return {horizontal, length:norm(horizontal)};
}
function currentForwardMotion() {
  if (!hasRight() || !hasForwardStart() || !latestRaw) return null;
  const rawDelta = sub(rawVec(latestRaw), calibration.forwardStart);
  const horizontal = sub(rawDelta, mul(QUEST_UP, dot3(rawDelta, QUEST_UP)));
  const orthogonal = sub(horizontal, mul(calibration.right, dot3(horizontal, calibration.right)));
  return {orthogonal, length:norm(orthogonal)};
}
function refreshDisplayPoints() {
  points = rawPoints.map(rawToDisplay);
  latestRaw = rawPoints.length ? rawPoints[rawPoints.length - 1] : null;
  latest = points.length ? points[points.length - 1] : null;
}
function resize() {
  if (window.innerWidth > 980 && window.scrollY !== 0) window.scrollTo(0, 0);
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}
window.addEventListener('resize', resize);
function bounds() {
  const extras = [{x:0,y:0,z:0}];
  const all = points.concat(extras);
  if (!all.length) return {xmin:-.5,xmax:.5,ymin:-.5,ymax:.5,zmin:-.5,zmax:.5,cx:0,cy:0,cz:0,span:1};
  let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity,zmin=Infinity,zmax=-Infinity;
  for (const p of all) { xmin=Math.min(xmin,p.x); xmax=Math.max(xmax,p.x); ymin=Math.min(ymin,p.y); ymax=Math.max(ymax,p.y); zmin=Math.min(zmin,p.z); zmax=Math.max(zmax,p.z); }
  const pad = Math.max((xmax-xmin), (ymax-ymin), (zmax-zmin), .12) * .18;
  xmin-=pad; xmax+=pad; ymin-=pad; ymax+=pad; zmin-=pad; zmax+=pad;
  return {xmin,xmax,ymin,ymax,zmin,zmax,cx:(xmin+xmax)/2,cy:(ymin+ymax)/2,cz:(zmin+zmax)/2,span:Math.max(xmax-xmin,ymax-ymin,zmax-zmin,.12)};
}
function project(p, b) {
  let x = p.x - b.cx, y = p.y - b.cy, z = p.z - b.cz;
  const ca = Math.cos(az), sa = Math.sin(az), ce = Math.cos(el), se = Math.sin(el);
  const xr = ca * x - sa * z;
  const zr = sa * x + ca * z;
  const yr = ce * y - se * zr;
  const depth = se * y + ce * zr;
  const rect = canvas.getBoundingClientRect();
  const scale = Math.min(rect.width, rect.height) * .68 / b.span * zoom;
  return {u: rect.width / 2 + xr * scale, v: rect.height / 2 - yr * scale, d: depth};
}
function line3(a, bpt, b, color, width=1, alpha=1) {
  const A = project(a, b), B = project(bpt, b);
  ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.moveTo(A.u, A.v); ctx.lineTo(B.u, B.v); ctx.stroke(); ctx.restore();
}
function drawArrow3(origin, dir, b, color, axisLen, width, alpha, label) {
  const end = add(origin, mul(normalize(dir), axisLen));
  const A = project(origin, b), B = project(end, b);
  const dx = B.u - A.u, dy = B.v - A.v, screenLen = Math.hypot(dx, dy);
  ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(A.u, A.v); ctx.lineTo(B.u, B.v); ctx.stroke();
  if (screenLen > 2) {
    const angle = Math.atan2(dy, dx), head = width >= 3 ? 9 : 6;
    ctx.beginPath(); ctx.moveTo(B.u, B.v);
    ctx.lineTo(B.u - head * Math.cos(angle - Math.PI / 7), B.v - head * Math.sin(angle - Math.PI / 7));
    ctx.lineTo(B.u - head * Math.cos(angle + Math.PI / 7), B.v - head * Math.sin(angle + Math.PI / 7));
    ctx.closePath(); ctx.fill();
  }
  if (label) { ctx.font = '12px SF Mono, Menlo, monospace'; ctx.fillText(label, B.u + 6, B.v - 4); }
  ctx.restore();
}
function drawGrid(b) {
  for (let i=0; i<=8; i++) {
    const tx = b.xmin + (b.xmax-b.xmin)*i/8;
    const tz = b.zmin + (b.zmax-b.zmin)*i/8;
    line3({x:tx,y:0,z:b.zmin}, {x:tx,y:0,z:b.zmax}, b, '#eeeeee', 1);
    line3({x:b.xmin,y:0,z:tz}, {x:b.xmax,y:0,z:tz}, b, '#eeeeee', 1);
  }
}
function drawWorldAxes(b) {
  const axisLen = Math.max(.18, b.span * .24);
  const origin = vec(0,0,0);
  drawArrow3(origin, vec(1,0,0), b, '#d44c47', axisLen, 2.5, .9, 'right');
  drawArrow3(origin, vec(0,0,-1), b, '#448361', axisLen, 2.5, .9, 'forward');
  drawArrow3(origin, vec(0,1,0), b, '#337ea9', axisLen, 2.5, .9, 'up');
}
function draw() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, rect.width, rect.height);
  const b = bounds();
  ctx.lineCap = 'round'; ctx.lineJoin = 'round';
  drawGrid(b); drawWorldAxes(b);
  if (points.length > 1) {
    for (let i=1; i<points.length; i++) {
      const A = project(points[i-1], b), B = project(points[i], b);
      const t = i / Math.max(points.length-1, 1);
      const gray = Math.round(170 - 110*t);
      ctx.strokeStyle = `rgba(${gray}, ${gray}, ${gray}, .92)`;
      ctx.lineWidth = 1.6 + 1.2*t;
      ctx.beginPath(); ctx.moveTo(A.u,A.v); ctx.lineTo(B.u,B.v); ctx.stroke();
    }
  }
  if (points.length) {
    const S = project(points[0], b), E = project(points[points.length-1], b);
    ctx.fillStyle = '#448361'; ctx.beginPath(); ctx.arc(S.u,S.v,5,0,Math.PI*2); ctx.fill();
    ctx.fillStyle = '#d44c47'; ctx.beginPath(); ctx.arc(E.u,E.v,6,0,Math.PI*2); ctx.fill();
  }
}
function updateCalibrationStatus(message=null) {
  if (message) notifyAction('error', message);
  const rightMotion = currentRightMotion();
  const forwardMotion = currentForwardMotion();
  const rightMotionText = rightMotion ? `${fmt(rightMotion.length,3)} m horizontal movement` : '--';
  const forwardMotionText = forwardMotion ? `${fmt(forwardMotion.length,3)} m forward movement` : '--';
  saveProfileBtn.disabled = commandPending;
  calibNextBtn.disabled = commandPending;
  if (!calibration || isCalibrated()) calibNextBtn.textContent = 'Start new calibration';
  else if (!hasRightStart()) calibNextBtn.textContent = 'Start collecting right';
  else if (!hasRight()) calibNextBtn.textContent = 'Finish right';
  else if (!hasForwardStart()) calibNextBtn.textContent = 'Start collecting forward';
  else if (!hasForward()) calibNextBtn.textContent = 'Finish forward';
  else calibNextBtn.textContent = 'Set origin';
  if (message) nextActionEl.textContent = message;
  else if (hasForwardStart() && !hasForward()) nextActionEl.textContent = `${nextCalibrationAction()} Current movement: ${forwardMotionText}.`;
  else if (hasRightStart() && !hasRight()) nextActionEl.textContent = `${nextCalibrationAction()} Current movement: ${rightMotionText}.`;
  else nextActionEl.textContent = nextCalibrationAction();
  if (editorSession) {
    saveProfileBtn.textContent = !editorSession.active && editorSession.last_action === 'finish' ? 'Saved ✓' : 'Finish Calibration';
    saveProfileBtn.disabled = commandPending || !editorSession.active;
    saveProfileBtn.title = !editorSession.active ? 'Editor is idle. Start a new calibration to edit.' : 'Save and apply this calibration.';
    calibNextBtn.disabled = commandPending;
    if (!editorSession.active) {
      calibNextBtn.textContent = 'Start new calibration';
      nextActionEl.textContent = editorSession.state === 'awaiting_b'
        ? 'Calibration finished. Return to your controls and pause/resume B.'
        : 'Editor idle. Start a new calibration when needed.';
    }
  }
  if (editorSession === undefined) { calibNextBtn.disabled = true; saveProfileBtn.disabled = true; }
  cancelCalibrationBtn.disabled = commandPending;
  loadProfileBtn.disabled = commandPending;
  profileSelect.disabled = commandPending;
  profileNameInput.disabled = commandPending;
  document.getElementById('fit').disabled = commandPending;
  document.getElementById('clear').disabled = commandPending;
  if (pendingButton) { pendingButton.disabled = true; pendingButton.textContent = pendingLabel; }
  document.querySelector('.side').setAttribute('aria-busy', String(commandPending));
}
function updateStats() {
  dot.classList.toggle('on', gateOpen);
  const triggerValue = latestRaw ? (latestRaw.flag ? 'True' : 'False') : '--';
  triggerValueEl.textContent = triggerValue;
  triggerMetric.classList.toggle('trigger-on', !!latestRaw && !!latestRaw.flag);
  updateCalibrationStatus();
  updateToolSummary();
}
function handleEvent(ev) {
  if (ev.type === 'editor') { applyEditor(ev.editor); return; }
  if (ev.type === 'snapshot') {
    rawPoints = (ev.points || []).map(clonePoint);
    gateOpen = !!ev.gate_open; pauseState = ev.pause_state;
  } else if (ev.type === 'reset') {
    rawPoints = [];
  } else if (ev.type === 'pose') {
    if (editorSession && !editorSession.active) return;
    rawPoints.push(clonePoint(ev.point));
  } else if (ev.type === 'status') {
    gateOpen = !!ev.gate_open; pauseState = ev.pause_state;
  }
  refreshDisplayPoints(); updateStats(); draw();
}
function requireLatest() {
  if (editorSession && (!editorSession.active || !editorSession.tracking_valid
      || !latestRaw || Date.now()/1000 - latestRaw.recv_unix > .5)) {
    throw new Error('Controller unavailable. Wake it and keep it visible to the headset, then retry.');
  }
  if (!latestRaw) throw new Error('No controller sample yet. Restore tracking, then retry.');
  return rawVec(latestRaw);
}
function beginNewCalibration() {
  currentProfile = sanitizeProfileName(profileNameInput.value);
  calibration = {version:CALIBRATION_VERSION, profile:currentProfile, up:QUEST_UP, state:'new', createdAt:new Date().toISOString()};
  rawPoints = [];
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
function startRightCollection() {
  const p = requireLatest(); if (!p) return;
  if (!calibration || isCalibrated()) throw new Error('Start a new calibration first.');
  calibration.rightStart = p;
  calibration.state = 'right_started';
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
function saveRightDirection() {
  const p = requireLatest(); if (!p) return;
  if (!hasRightStart()) throw new Error('Start collecting right first.');
  const rawDelta = sub(p, calibration.rightStart);
  const horizontal = sub(rawDelta, mul(QUEST_UP, dot3(rawDelta, QUEST_UP)));
  const length = norm(horizontal);
  if (length < MIN_RIGHT_M) throw new Error(`Right movement too short (${fmt(length,3)} m). Move at least ${MIN_RIGHT_M} m right, then retry.`);
  calibration.rightEnd = p;
  calibration.right = normalize(horizontal);
  calibration.state = 'right_ready';
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
function startForwardSample() {
  const p = requireLatest(); if (!p) return;
  if (!hasRight()) throw new Error('Finish right first.');
  calibration.forwardStart = p;
  calibration.state = 'forward_started';
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
function saveForwardDirection() {
  const p = requireLatest(); if (!p) return;
  if (!hasRight() || !hasForwardStart()) throw new Error('Start collecting forward first.');
  const rawDelta = sub(p, calibration.forwardStart);
  const horizontal = sub(rawDelta, mul(QUEST_UP, dot3(rawDelta, QUEST_UP)));
  const orthogonal = sub(horizontal, mul(calibration.right, dot3(horizontal, calibration.right)));
  const length = norm(orthogonal);
  if (length < MIN_RIGHT_M) throw new Error(`Forward movement too short (${fmt(length,3)} m). Move at least ${MIN_RIGHT_M} m forward, then retry.`);
  let forward = normalize(cross(QUEST_UP, calibration.right));
  if (dot3(forward, orthogonal) < 0) forward = mul(forward, -1);
  calibration.forwardEnd = p;
  calibration.forwardHint = normalize(horizontal);
  calibration.forward = forward;
  calibration.state = 'forward_ready';
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
function saveOrigin() {
  const p = requireLatest(); if (!p) return;
  if (!hasForward()) throw new Error('Finish forward first.');
  calibration.origin = p;
  calibration.state = 'ready';
  calibration.completedAt = new Date().toISOString();
  localSaveCalibration(); refreshDisplayPoints(); updateStats(); draw();
}
calibNextBtn.onclick = () => runAction(calibNextBtn, 'Applying…', async () => {
    if (editorSession && !editorSession.active) await editorCommand('begin');
    if (!calibration || isCalibrated()) { beginNewCalibration(); return 'New calibration started.'; }
    if (!hasRightStart()) { startRightCollection(); return 'Collecting right. Move right, then click Finish right.'; }
    if (!hasRight()) { saveRightDirection(); return 'Right direction captured.'; }
    if (!hasForwardStart()) { startForwardSample(); return 'Collecting forward. Move forward, then click Finish forward.'; }
    if (!hasForward()) { saveForwardDirection(); return 'Forward direction captured.'; }
    if (!hasOrigin()) {
      saveOrigin();
      if (!editorSession) return await saveServerCalibration();
      return 'Origin captured. Click Finish Calibration to save and apply.';
    }
});
cancelCalibrationBtn.onclick = () => runAction(cancelCalibrationBtn, 'Cancelling…', async () => {
  await editorCommand('cancel');
  return 'Cancelled. Previous profile retained. Pause/resume B to continue.';
});
const clearBtn = document.getElementById('clear');
const fitBtn = document.getElementById('fit');
clearBtn.onclick = () => runAction(clearBtn, 'Clearing…', () => { rawPoints = []; refreshDisplayPoints(); updateStats(); draw(); return 'Displayed path cleared. Calibration is unchanged.'; });
fitBtn.onclick = () => runAction(fitBtn, 'Fitting…', () => { zoom = 1; draw(); return 'View fitted.'; });
profileNameInput.addEventListener('change', () => {
  currentProfile = sanitizeProfileName(profileNameInput.value);
  profileNameInput.value = currentProfile;
  // Editing the destination name must not replace freshly captured geometry.
  // Loading another profile is an explicit action through the selector/button.
  if (calibration) { calibration.profile = currentProfile; localSaveCalibration(); }
  profileStatus.textContent = `Profile name set: ${currentProfile}`;
  notifyAction('success', `Profile name: ${currentProfile}. Click Finish Calibration to save.`);
  updateToolSummary();
});
profileSelect.addEventListener('change', () => {
  if (profileSelect.value) {
    runAction(loadProfileBtn, 'Loading…', () => loadServerCalibration(profileSelect.value));
  }
});
loadProfileBtn.onclick = () => runAction(loadProfileBtn, 'Loading…', () => loadServerCalibration(profileNameInput.value));
saveProfileBtn.onclick = () => runAction(saveProfileBtn, 'Saving…', saveServerCalibration);
canvas.addEventListener('mousedown', e => { dragging = true; dragStart = {x:e.clientX, y:e.clientY, az, el}; });
window.addEventListener('mouseup', () => dragging = false);
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  az = dragStart.az + (e.clientX - dragStart.x) * 0.01;
  el = Math.max(-1.2, Math.min(1.2, dragStart.el + (e.clientY - dragStart.y) * 0.01));
  draw();
});
canvas.addEventListener('wheel', e => { e.preventDefault(); zoom *= Math.exp(-e.deltaY * 0.001); zoom = Math.max(.25, Math.min(6, zoom)); draw(); }, {passive:false});
function loadProfiles() {
  return requestJSON('/calibrations').then(info => {
    const activeProfile = sanitizeProfileName(info.active || currentProfile);
    if (!profileInitialized) {
      currentProfile = sanitizeProfileName(new URLSearchParams(location.search).get('profile') || activeProfile);
      profileInitialized = true;
    } else {
      currentProfile = sanitizeProfileName(currentProfile || activeProfile);
    }
    profileNameInput.value = currentProfile;
    profileSelect.innerHTML = '';
    const names = (info.profiles || []).map(p => p.name);
    if (!names.includes(currentProfile)) names.unshift(currentProfile);
    for (const name of names) {
      const opt = document.createElement('option');
      opt.value = name; opt.textContent = name;
      opt.selected = name === currentProfile;
      profileSelect.appendChild(opt);
    }
    profileStatus.textContent = names.length ? `Profiles: ${names.length}` : 'No saved profiles yet.';
    return info;
  }).catch(err => {
    profileStatus.textContent = `Profile list failed: ${err.message}`;
    return {active: currentProfile, profiles: []};
  });
}
function loadServerCalibration(profile=currentProfile) {
  const selected = sanitizeProfileName(profile);
  const version = ++calibrationLoadVersion;
  return requestJSON(`/calibration?profile=${encodeURIComponent(selected)}`).then(saved => {
    if (version !== calibrationLoadVersion) return 'A newer action replaced this load.';
    if (saved && saved.version === CALIBRATION_VERSION) {
      applyCalibration(saved, selected, `Loaded profile: ${selected}`);
      return `Loaded profile: ${selected}.`;
    } else {
      const local = loadLocalCalibration(selected);
      if (local) { applyCalibration(local, selected); return `Loaded browser draft: ${selected}. Not saved to the source yet.`; }
      throw new Error(`No saved profile: ${selected}. Your current draft is unchanged.`);
    }
  });
}
refreshEditor().then(() => loadProfiles()).then(() => loadServerCalibration(currentProfile)).catch(error => notifyAction('error', error.message));
setInterval(refreshEditor, 1000);
fetch('/snapshot').then(r => r.json()).then(handleEvent).catch(() => {});
const es = new EventSource('/events');
es.onmessage = e => handleEvent(JSON.parse(e.data));
es.onerror = () => { streamMetric.textContent = 'Disconnected'; dot.classList.remove('on'); };
updateCalibrationStatus(); resize(); setInterval(draw, 1000);
</script>
</body>
</html>
"""


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def make_handler(
    state: LiveState,
    calibration_path: Path,
    *,
    editor: CalibrationSession | None = None,
) -> type[BaseHTTPRequestHandler]:
    calibration_dir = editor.storage_dir if editor else calibration_path.parent
    default_profile = calibration_path.stem

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def send_text(
            self, body: str, content_type: str, status: HTTPStatus = HTTPStatus.OK
        ) -> None:
            data = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def selected_calibration_path(self) -> Path:
            query = parse_qs(urlparse(self.path).query)
            profile = sanitize_profile(query.get("profile", [default_profile])[0], default_profile)
            return calibration_file(calibration_dir, profile)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/editor/status" and editor is not None:
                self.send_text(json.dumps(editor.snapshot()), "application/json")
                return
            if path == "/":
                self.send_text(HTML, "text/html; charset=utf-8")
                return
            if path == "/snapshot":
                self.send_text(
                    json.dumps(state.snapshot(), separators=(",", ":")), "application/json"
                )
                return
            if path == "/calibrations":
                profiles = []
                calibration_dir.mkdir(parents=True, exist_ok=True)
                for item in sorted(calibration_dir.glob("*.json")):
                    try:
                        data = json.loads(item.read_text(encoding="utf-8"))
                    except (OSError, json.JSONDecodeError):
                        data = None
                    stat = item.stat()
                    profiles.append(
                        {
                            "name": item.stem,
                            "path": str(item),
                            "mtime": stat.st_mtime,
                            "version": data.get("version") if isinstance(data, dict) else None,
                            "complete": calibration_complete(data),
                        }
                    )
                self.send_text(
                    json.dumps(
                        {
                            "active": editor.path.stem if editor else default_profile,
                            "profiles": profiles,
                        },
                        separators=(",", ":"),
                    ),
                    "application/json",
                )
                return
            if path == "/calibration":
                try:
                    selected = self.selected_calibration_path()
                    if editor and not selected.exists() and selected.stem == editor.path.stem:
                        selected = editor.path
                    if selected.exists():
                        self.send_text(selected.read_text(encoding="utf-8"), "application/json")
                    else:
                        self.send_text("null", "application/json")
                except ValueError as exc:
                    self.send_text(str(exc), "text/plain; charset=utf-8", HTTPStatus.BAD_REQUEST)
                return
            if path == "/events":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                client = state.add_client()
                try:
                    while True:
                        try:
                            event = client.get(timeout=20)
                        except queue.Empty:
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                            continue
                        payload = json.dumps(event, separators=(",", ":"))
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    state.remove_client(client)
                return
            self.send_text("Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if editor is not None:
                if path != "/editor/command":
                    self.send_text(
                        "Use Finish Calibration to save and apply a profile",
                        "text/plain",
                        HTTPStatus.CONFLICT,
                    )
                    return
                try:
                    origin = self.headers.get("Origin")
                    if origin and urlparse(origin).netloc != self.headers.get("Host"):
                        raise ValueError("Cross-origin writes are not allowed")
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("application/json required")
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 65536:
                        raise ValueError("Invalid request size")
                    request = json.loads(self.rfile.read(length).decode("utf-8"))
                    if not isinstance(request, dict):
                        raise ValueError("Expected an object")
                    result = editor.submit(request)
                except (OSError, ValueError) as exc:
                    result = {"accepted": False, "applied": False, "message": str(exc)}
                self.send_text(
                    json.dumps(result),
                    "application/json",
                    HTTPStatus.OK if result.get("applied") else HTTPStatus.CONFLICT,
                )
                return
            if path != "/calibration":
                self.send_text("Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(length).decode("utf-8"))
                for key in ("origin", "right", "forward", "up"):
                    if key not in data:
                        raise ValueError(f"missing {key}")
                data.pop("rotation", None)
                if not calibration_complete(data):
                    raise ValueError("profile axes are incomplete or invalid")
                selected = self.selected_calibration_path()
                atomic_write_json(selected, data)
            except (json.JSONDecodeError, OSError, ValueError) as exc:
                self.send_text(
                    f"Invalid calibration: {exc}",
                    "text/plain; charset=utf-8",
                    HTTPStatus.BAD_REQUEST,
                )
                return
            self.send_text(json.dumps({"ok": True}, separators=(",", ":")), "application/json")

        def do_DELETE(self) -> None:  # noqa: N802
            if editor is not None:
                self.send_text(
                    "Profile deletion is disabled during a live source session",
                    "text/plain",
                    HTTPStatus.CONFLICT,
                )
                return
            path = urlparse(self.path).path
            if path != "/calibration":
                self.send_text("Not found", "text/plain; charset=utf-8", HTTPStatus.NOT_FOUND)
                return
            try:
                self.selected_calibration_path().unlink(missing_ok=True)
            except (OSError, ValueError) as exc:
                self.send_text(
                    f"Could not delete calibration: {exc}",
                    "text/plain; charset=utf-8",
                    HTTPStatus.BAD_REQUEST,
                )
                return
            self.send_text(json.dumps({"ok": True}, separators=(",", ":")), "application/json")

    return Handler
