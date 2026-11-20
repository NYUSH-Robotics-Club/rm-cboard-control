"""Windows status window for the dual-controller VR session."""
import argparse
import json
import queue
import threading
import tkinter as tk
import webbrowser
from urllib.error import HTTPError
from urllib.request import Request, urlopen


BASE = 'http://127.0.0.1:18780'
BG, PANEL, TEXT, MUTED = '#101721', '#1a2533', '#edf3fa', '#a9b8ca'
GREEN, AMBER, RED = '#75dfab', '#ffd183', '#ff9292'
TRACKING_STATUS = {
    'no-source': '网页未收到该手柄的输入源',
    'session-not-visible': '沉浸式会话不在前台',
    'no-frame': '等待 XR 帧或参考坐标',
    'bare-hand': '当前是裸手追踪',
    'no-grip-space': '输入源没有手柄位姿空间',
    'no-trigger-mapping': '没有标准扳机输入',
    'emulated-pose': '当前只有估算位置',
    'no-grip-pose': '当前帧缺少手柄位姿',
}


def request(path, action=None):
    if action is None:
        req = Request(BASE + path)
    else:
        token = request('/api/bootstrap')['token']
        req = Request(
            BASE + path,
            data=json.dumps({'action': action}).encode(),
            headers={'Content-Type': 'application/json', 'X-Studio-Token': token},
        )
    try:
        with urlopen(req, timeout=.5) as response:
            return json.load(response)
    except HTTPError as error:
        raise ValueError(json.load(error).get('error', str(error))) from error


class Monitor:
    def __init__(self, root):
        self.root = root
        self.results = queue.Queue()
        self.latest = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.busy = False
        self.stop_requested = False
        root.title('VR 底盘 · 双手柄实时状态')
        root.geometry('870x490')
        root.minsize(740, 460)
        root.configure(bg=BG)
        root.option_add('*Font', '{Microsoft YaHei UI} 10')
        root.protocol('WM_DELETE_WINDOW', self.close)
        header = tk.Frame(root, bg=BG)
        header.pack(fill='x', padx=20, pady=(14, 8))
        self.label(header, '双手柄实时状态', bg=BG, size=19).pack(side='left')
        topmost = tk.BooleanVar(root, False)
        tk.Checkbutton(header, text='窗口置顶', variable=topmost, bg=BG, fg=TEXT,
                       activebackground=BG, selectcolor=PANEL,
                       command=lambda: root.attributes('-topmost', topmost.get())).pack(side='right')
        self.link = self.label(root, '等待头显', bg=BG)
        self.link.pack(fill='x', padx=20)
        row = tk.Frame(root, bg=BG)
        row.pack(fill='x', padx=16, pady=10)
        self.hands = {}
        for side, title in (('left', '左手柄 · 底盘旋转'), ('right', '右手柄 · 底盘平移')):
            card = tk.Frame(row, bg=PANEL, padx=14, pady=10)
            card.pack(side='left', fill='both', expand=True, padx=4)
            self.label(card, title, size=14).pack(fill='x')
            readout = self.label(card, '等待追踪', color=MUTED)
            readout.configure(height=5)
            readout.pack(fill='x')
            self.hands[side] = readout
        self.output = self.label(root, 'MQTT 指令等待中', size=15)
        self.output.pack(fill='x', padx=20, pady=(0, 10), ipady=7, ipadx=10)
        group = tk.Frame(root, bg=PANEL, padx=12, pady=8)
        group.pack(fill='x', padx=20)
        self.prompt = self.label(group, '等待进入 VR')
        self.prompt.pack(fill='x', pady=(0, 8))
        actions = tk.Frame(group, bg=PANEL)
        actions.pack(fill='x')
        self.buttons = {}
        for action, title in (('start', '原版校准'), ('arm', '启用控车'), ('disarm', '停止控车')):
            button = tk.Button(actions, text=title, command=lambda a=action: self.act(a),
                               padx=10, pady=4, relief='flat')
            button.pack(side='left', padx=(0, 8))
            self.buttons[action] = button
        self.feedback = self.label(root, '', bg=BG, color=AMBER)
        self.feedback.pack(fill='x', padx=20, pady=6)
        self.render({})
        threading.Thread(target=self.poll, daemon=True).start()
        self.root.after(80, self.update)

    @staticmethod
    def label(parent, text, bg=PANEL, color=TEXT, size=10):
        return tk.Label(parent, text=text, bg=bg, fg=color, justify='left', anchor='w',
                        font=('Microsoft YaHei UI', size))

    def close(self):
        self.stop.set()
        self.root.destroy()

    def poll(self):
        while not self.stop.is_set():
            try:
                status = request('/api/status')
            except (OSError, ValueError):
                status = {}
            try:
                self.latest.get_nowait()
            except queue.Empty:
                pass
            self.latest.put_nowait(status)
            self.stop.wait(.10)

    def act(self, action):
        if self.busy:
            if action == 'disarm':
                self.stop_requested = True
            return
        self.busy = True
        self.feedback.configure(text='')
        def work():
            path = '/api/repository-calibration' if action == 'start' else '/api/control'
            try:
                request(path, action)
                if action == 'start':
                    webbrowser.open(BASE + '/calibration-ui')
                self.results.put(('操作已完成', GREEN))
            except (OSError, ValueError) as error:
                self.results.put((str(error), AMBER))
        threading.Thread(target=work, daemon=True).start()

    def update(self):
        while not self.results.empty():
            message, color = self.results.get_nowait()
            self.busy = False
            self.feedback.configure(text=message, fg=color)
            if self.stop_requested:
                self.stop_requested = False
                self.act('disarm')
        try:
            self.render(self.latest.get_nowait())
        except queue.Empty:
            pass
        self.root.after(80, self.update)

    def render(self, status):
        frame = (status.get('frame') or {}) if status.get('fresh') else {}
        publisher = status.get('publisher') or {}
        control = status.get('control') or {}
        calibration = status.get('calibration') or {}
        age = status.get('age_ms')
        age_text = f'{age} ms' if frame and age is not None else '等待追踪'
        link_text = '已连接' if publisher.get('connected') else '未连接'
        self.link.configure(
            text=f"手柄帧 {status.get('accepted_frames', 0)}   ·   {age_text}   ·   官方 MQTT {link_text}",
            fg=TEXT if status else RED,
        )
        for side, label in self.hands.items():
            hand = frame.get('hands', {}).get(side, {})
            if not hand.get('tracked'):
                reason = TRACKING_STATUS.get(frame.get('tracking', {}).get(side), '等待有效追踪')
                label.configure(text=f'{reason}\n扳机 —   握持 —\n位置 —\n按键 / 摇杆 —', fg=MUTED)
                continue
            position = '  '.join(f'{value:+.3f}' for value in hand['pose']['position'])
            orientation = '  '.join(f'{value:+.2f}' for value in hand['pose']['orientation'])
            buttons = ' / '.join(str(i) for i, value in enumerate(hand['buttons']) if value >= .5) or '无'
            axes = ' '.join(f'{value:+.2f}' for value in hand['axes']) or '—'
            label.configure(text=f"追踪有效   扳机 {hand['trigger']:.2f}   握持 {hand['grip']:.2f}\n"
                            f'位置 XYZ  {position} m\n姿态 XYZW  {orientation}\n按键 {buttons}   摇杆 {axes}', fg=GREEN)
        output = publisher.get('output')
        if publisher.get('connected') and publisher.get('publish_ok') is not False and output and len(output) == 4:
            _, forward, right, rotation = output
            mode = '已启用' if control.get('armed') else '已停止'
            label = 'MQTT 已发布' if publisher.get('publish_ok') else 'MQTT 待确认'
            self.output.configure(text=f'{mode}   ·   {label}   前 {forward:+d}   右 {right:+d}   逆时针 {rotation:+d}   ·   序号 {publisher["sequence"]}',
                                  fg=GREEN if control.get('armed') else AMBER)
        else:
            self.output.configure(text='MQTT 暂无可确认已发布的指令', fg=AMBER)
        stage = calibration.get('stage', 'idle')
        self.prompt.configure(text=calibration.get('prompt', '等待打开双手柄工作台'))
        enabled = {
            'start': bool(frame),
            'arm': stage == 'ready' and bool(frame) and publisher.get('connected'),
            'disarm': bool(status),
        }
        for action, button in self.buttons.items():
            available = enabled[action] and (not self.busy or action == 'disarm')
            button.configure(state='normal' if available else 'disabled')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke-test', action='store_true')
    args = parser.parse_args()
    root = tk.Tk()
    if args.smoke_test:
        root.withdraw()
    monitor = Monitor(root)
    if args.smoke_test:
        root.update_idletasks()
        assert set(monitor.hands) == {'left', 'right'}
        monitor.close()
    else:
        root.mainloop()
