"""Serve the desktop calibration UI with the current WebXR pose stream."""
import ast
from functools import lru_cache
import json
import math
from pathlib import Path
import re
import time


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'firmware/tools/vr/vendor/quest-trajectory-recorder'
    / 'src/quest_trajectory_recorder/live3d_web.py'
)
DEFAULT_PROFILE = 'quest_webxr'
PROFILE_NAME = re.compile(r'[A-Za-z0-9_-][A-Za-z0-9_.-]{0,63}')


@lru_cache(maxsize=1)
def page_assets():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'), filename=str(SOURCE))
    html = next(
        ast.literal_eval(node.value) for node in tree.body
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == 'HTML'
                                               for target in node.targets)
    )
    header, script = html.split('<script>', 1)
    javascript, footer = script.split('</script>', 1)
    header = header.replace('<body>', '<body><p style="margin:12px 20px">保持 Quest 的 VR 工作台运行。'
                            '这里采集右手柄，保存后回监控窗口启用控车。 <a href="/">返回工作台</a></p>', 1)
    replacements = {
        "let editorSession;": "let editorSession = null;\nlet lastSaveState = null;\nsaveProfileBtn.style.display = 'none';",
        "result.message ||": "result.message || result.error ||",
        """await requestJSON(`/calibration?profile=${encodeURIComponent(currentProfile)}`, {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(calibration),
  });
  loadProfiles().catch(() => {});
  return `Saved profile: ${currentProfile}.`;""": """await Studio.init();
  await Studio.post('/api/repository-calibration', {
    action:'begin', profile:currentProfile
  });
  return '已开始。按右手扳机确认每一步。';""",
        "if (!latestRaw) return editorSession ? 'Wake the controller and keep it visible to the headset.' : 'Start Quest streaming. Press B until Quest stream shows Streaming.';": "if (!latestRaw) return '等待 Quest VR 工作台的右手柄，保持追踪后重试。';",
        "if (!latestRaw) throw new Error('No controller sample yet. Restore tracking, then retry.');": "if (!latestRaw || Date.now()/1000 - latestRaw.recv_unix > .5) throw new Error('右手柄数据已过期，请恢复追踪后重试。');",
        "if (!calibration || isCalibrated()) { beginNewCalibration(); return 'New calibration started.'; }": """if (!calibration || isCalibrated()) {
      await Studio.init();
      await Studio.post('/api/repository-calibration', {action:'begin', profile:currentProfile});
      beginNewCalibration();
      return '校准已开始，底盘指令已归零。';
    }""",
        "if (!editorSession) return await saveServerCalibration();": """if (!editorSession) {
        try { return await saveServerCalibration(); }
        catch (error) {
          delete calibration.origin;
          calibration.state = 'forward_ready';
          throw error;
        }
      }""",
        "refreshEditor().then(() => loadProfiles()).then(() => loadServerCalibration(currentProfile)).catch(error => notifyAction('error', error.message));\nsetInterval(refreshEditor, 1000);\nfetch('/snapshot').then(r => r.json()).then(handleEvent).catch(() => {});\nconst es = new EventSource('/events');\nes.onmessage = e => handleEvent(JSON.parse(e.data));\nes.onerror = () => { streamMetric.textContent = 'Disconnected'; dot.classList.remove('on'); };": """loadProfiles().then(info => {
  if (info.profiles.some(profile => profile.name === currentProfile)) return loadServerCalibration(currentProfile);
}).catch(error => notifyAction('error', error.message));
async function pollRightHand() {
  try {
    const sample = await requestJSON('/snapshot');
    handleEvent(sample);
    if (sample.calibration) {
      calibration = sample.calibration;
      refreshDisplayPoints();
      draw();
    }
    if (sample.save_state && sample.save_state !== 'saved') {
      calibNextBtn.textContent = '使用右手扳机确认';
      calibNextBtn.disabled = true;
      profileNameInput.disabled = true;
      profileSelect.disabled = true;
      loadProfileBtn.disabled = true;
    }
    if (sample.prompt) nextActionEl.textContent = sample.prompt;
    if (sample.save_state !== lastSaveState) {
      const previousState = lastSaveState;
      lastSaveState = sample.save_state;
      if (sample.save_state === 'saved') {
        notifyAction('success', '已用右扳机保存。回双手柄监控窗口启用控车。');
        loadProfiles().catch(() => {});
      } else if (sample.save_state === 'error') {
        notifyAction('error', '保存失败，请松开右扳机再按一次。');
      } else if (!sample.save_state && previousState && previousState !== 'saved') {
        notifyAction('error', 'VR 会话已更换，请重新校准。');
      }
    }
  } catch (_) {
    handleEvent({type:'snapshot', points:[], gate_open:false});
  }
  setTimeout(pollRightHand, 100);
}
pollRightHand();""",
    }
    for old, new in replacements.items():
        if old not in javascript:
            raise ValueError('仓库原版校准页面已变化，需要更新接口适配')
        javascript = javascript.replace(old, new, 1)
    return header + '<script src="/shared.js"></script><script src="/calibration-ui.js"></script>' + footer, javascript


def profile_name(name):
    if not isinstance(name, str) or not PROFILE_NAME.fullmatch(name):
        raise ValueError('校准名称只能使用字母、数字、点、下划线或短横线')
    return name


def basis_from_profile(data):
    if not isinstance(data, dict):
        raise ValueError('校准数据无效')
    if data.get('version') != 5 or data.get('state') != 'ready':
        raise ValueError('请先完成右向、前向和原点采集')

    def vector(key):
        value = data.get(key)
        if not isinstance(value, dict):
            raise ValueError(f'缺少校准点 {key}')
        coordinates = tuple(value.get(axis) for axis in 'xyz')
        if any(type(item) not in (int, float) or not math.isfinite(item) or abs(item) > 100
               for item in coordinates):
            raise ValueError(f'校准点 {key} 无效')
        return coordinates

    def direction(start, end):
        x0, _, z0 = vector(start)
        x1, _, z1 = vector(end)
        # The original UI consumes Unity-style coordinates. Mirror Z back to WebXR.
        x, z = x1 - x0, z0 - z1
        length = math.hypot(x, z)
        if not .05 <= length <= .65:
            raise ValueError('每段水平位移请控制在 5–65 cm 内')
        return x / length, z / length

    vector('origin')
    right = direction('rightStart', 'rightEnd')
    forward_hint = direction('forwardStart', 'forwardEnd')
    forward = (right[1], -right[0])
    if sum(a*b for a, b in zip(forward, forward_hint)) < .80:
        raise ValueError('右向与前向应接近直角，请检查方向后重采')
    return {'forward': forward, 'right': right}


class TriggerCalibration:
    prompts = {
        'right-start': '右手放在起点，按右扳机开始采右向',
        'right-end': '右手向正右平移约 20 cm，按右扳机确认',
        'forward-start': '右手回到舒适起点，按右扳机开始采前向',
        'forward-end': '右手向正前平移约 20 cm，按右扳机确认',
        'origin': '右手放在中立位置，按右扳机设置原点',
        'save': '松开右扳机，再按一下保存',
        'saved': '校准已保存。松开两只扳机，再启用控车。',
    }

    def __init__(self, profile):
        self.profile = profile
        self.stage = 'right-start'
        self.data = {'version': 5, 'state': 'new', 'profile': profile, 'up': {'x': 0, 'y': 1, 'z': 0}}
        self.released = False
        self.error = ''
        self.basis = None

    @property
    def prompt(self):
        return self.error or self.prompts[self.stage]

    def feed(self, position, trigger):
        if self.stage == 'saved':
            return False
        if trigger <= .20:
            self.released = True
        if trigger < .65 or not self.released:
            return False
        self.released = False
        self.error = ''
        x, y, z = position
        point = {'x': x, 'y': y, 'z': -z}
        if self.stage in ('right-start', 'forward-start'):
            axis = self.stage.split('-')[0]
            self.data[axis + 'Start'] = point
            self.stage = axis + '-end'
        elif self.stage in ('right-end', 'forward-end'):
            axis = self.stage.split('-')[0]
            start = self.data[axis + 'Start']
            delta = {key: point[key] - start[key] for key in ('x', 'z')}
            length = math.hypot(*delta.values())
            if not .05 <= length <= .65:
                self.error = '请水平平移 5–65 cm，松开右扳机再按一次'
                return False
            self.data[axis + 'End'] = point
            if axis == 'right':
                self.data['right'] = {**{key: val / length for key, val in delta.items()}, 'y': 0}
                self.stage = 'forward-start'
            else:
                candidate = {**self.data, 'origin': point, 'state': 'ready'}
                try:
                    self.basis = basis_from_profile(candidate)
                except ValueError as exc:
                    self.error = str(exc)
                    return False
                fx, fz = self.basis['forward']
                self.data['forward'] = {'x': fx, 'y': 0, 'z': -fz}
                self.stage = 'origin'
        elif self.stage == 'origin':
            self.data.update(origin=point, state='ready')
            self.stage = 'save'
        else:
            return True
        return False

    def persist(self, directory):
        path = directory / (self.profile + '.json')
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.pending')
        payload = dict(self.data, coordinate_space='webxr-local-floor-via-unity', saved_unix=time.time())
        temporary.write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False), 'utf-8')
        temporary.replace(path)
        self.stage = 'saved'
