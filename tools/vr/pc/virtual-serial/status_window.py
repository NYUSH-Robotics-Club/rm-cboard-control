"""Read-only native dashboard for the active Quest -> virtual COM controller."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import queue
import re
import subprocess
import threading
import time
import tkinter as tk

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vr_config import PROFILE, RUNTIME_ROOT, SETTINGS
HERE = Path(__file__).resolve().parent
CALIBRATION = PROFILE
BG, PANEL, TEXT, MUTED = "#101721", "#1a2533", "#edf3fa", "#a9b8ca"
GREEN, AMBER, RED, BLUE = "#75dfab", "#ffd183", "#ff9292", "#80b9ff"


class StatusWindow:
    def __init__(self, root):
        self.root = root
        root.title("VR 底盘 · 手柄实时状态")
        root.geometry("980x720")
        root.minsize(850, 660)
        root.configure(bg=BG)
        root.option_add("*Font", "{Microsoft YaHei UI} 10")
        self.board_results = queue.Queue()
        self.board_busy = False
        self.last_status = {}
        self.controls = {}
        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", padx=22, pady=(18, 8))
        tk.Label(top, text="VR 底盘实时状态", font=("Microsoft YaHei UI", 21, "bold"), bg=BG, fg=TEXT).pack(side="left")
        self.topmost = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="窗口置顶", variable=self.topmost, bg=BG, fg=TEXT,
                       activebackground=BG, selectcolor=PANEL,
                       command=lambda: root.attributes("-topmost", self.topmost.get())).pack(side="right")
        self.banner = tk.Label(root, text="正在读取手柄状态…", anchor="w", bg=PANEL, fg=AMBER, padx=14, pady=12)
        self.banner.pack(fill="x", padx=22, pady=8)
        row = tk.Frame(root, bg=BG)
        row.pack(fill="x", padx=18, pady=8)
        for key, title in (("tracking", "右手柄追踪"), ("b", "B 状态"),
                           ("trigger", "食指扳机"), ("enabled", "运动输出")):
            card = tk.Frame(row, bg=PANEL, padx=13, pady=10)
            card.pack(side="left", expand=True, fill="both", padx=4)
            tk.Label(card, text=title, bg=PANEL, fg=MUTED).pack(anchor="w")
            label = tk.Label(card, text="等待", bg=PANEL, fg=AMBER, font=("Microsoft YaHei UI", 18, "bold"))
            label.pack(anchor="w", pady=(5, 0))
            self.controls[key] = label
        middle = tk.Frame(root, bg=BG)
        middle.pack(fill="both", expand=True, padx=22, pady=8)
        self.canvas = tk.Canvas(middle, bg=PANEL, highlightthickness=0, width=370, height=215)
        self.canvas.pack(side="left", fill="both", expand=True)
        values = tk.Frame(middle, bg=PANEL, padx=20, pady=14)
        values.pack(side="right", fill="both", padx=(10, 0))
        self.forward = self.metric(values, "前后指令（正＝前，负＝后）")
        self.right = self.metric(values, "左右指令（正＝右，负＝左）")
        self.position = tk.Label(values, text="手柄坐标：等待", bg=PANEL, fg=MUTED, anchor="w", justify="left")
        self.position.pack(anchor="w", pady=(12, 0))
        self.metrics = tk.Label(root, text="", bg=BG, fg=TEXT, anchor="w")
        self.metrics.pack(fill="x", padx=22, pady=6)
        self.freshness = tk.Label(root, text="", bg=BG, fg=MUTED, anchor="w")
        self.freshness.pack(fill="x", padx=22, pady=2)
        board_row = tk.Frame(root, bg=BG)
        board_row.pack(fill="x", padx=22, pady=6)
        self.board = tk.Label(board_row, text="C 板：点击右侧按钮读取接收计数", bg=BG, fg=MUTED, anchor="w")
        self.board.pack(side="left")
        self.board_button = tk.Button(board_row, text="刷新 C 板计数", command=self.refresh_board,
                                      bg=PANEL, fg=TEXT, activebackground=PANEL, relief="flat", padx=12)
        self.board_button.pack(side="right")
        calibration = json.loads(CALIBRATION.read_text(encoding="utf-8"))
        digest = hashlib.sha256(CALIBRATION.read_bytes()).hexdigest()
        saved = calibration.get("completedAt", "")
        try:
            saved = datetime.fromisoformat(saved.replace("Z", "+00:00")).astimezone().strftime("%m-%d %H:%M:%S")
        except ValueError:
            pass
        cal = tk.Frame(root, bg=PANEL, padx=14, pady=10)
        cal.pack(fill="x", padx=22, pady=8)
        tk.Label(cal, text=f"沿用已完成的校准 · {saved} · SHA256 {digest[:12]}", bg=PANEL, fg=GREEN, anchor="w").pack(fill="x")
        tk.Label(cal, text="quest_lab_current.json · 前／右方向和比例保持原设置", bg=PANEL, fg=MUTED, anchor="w").pack(fill="x", pady=(4, 0))
        tk.Label(root, text="操作：B 切换 Low → High；松开扳机取中立位置，再按住扳机移动手柄。\n松开扳机归零，程序继续监听。指令范围 ±100 RC 刻度，中心死区 3 cm。",
                 bg=BG, fg=TEXT, anchor="w", justify="left").pack(fill="x", padx=22, pady=(6, 16))
        self.update()

    def metric(self, parent, caption):
        tk.Label(parent, text=caption, bg=PANEL, fg=MUTED).pack(anchor="w")
        label = tk.Label(parent, text="+0", bg=PANEL, fg=TEXT, font=("Consolas", 27, "bold"))
        label.pack(anchor="w", pady=(0, 8))
        return label

    def refresh_board(self):
        if self.board_busy:
            return
        self.board_busy = True
        self.board_button.configure(state="disabled")
        self.board.configure(text="C 板：正在只读检查…")
        def work():
            try:
                command = ('timeout 10s openocd -f interface/stlink.cfg '
                           '-c "transport select hla_swd" -f target/stm32f4x.cfg '
                           '-c "init; echo [mdw 0x2000f038 3]; shutdown" 2>&1')
                result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", SETTINGS["diagnostic_ssh_host"], command],
                                        capture_output=True, text=True, timeout=15,
                                        creationflags=subprocess.CREATE_NO_WINDOW)
                match = re.search(r"0x2000f038:\s*([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})", result.stdout)
                if not match:
                    raise RuntimeError("未读到计数，请检查 Jetson / ST-Link 连接")
                valid, published, axes = [int(x, 16) for x in match.groups()]
                forward, right = axes & 65535, axes >> 16
                forward = forward - 65536 if forward > 32767 else forward
                right = right - 65536 if right > 32767 else right
                self.board_results.put(f"C 板 {datetime.now():%H:%M:%S}：有效 {valid} · 发布 {published} · 前 {forward:+d} / 右 {right:+d}")
            except Exception as exc:
                self.board_results.put("C 板：" + str(exc))
        threading.Thread(target=work, daemon=True).start()

    def update(self):
        try:
            s = json.loads((RUNTIME_ROOT / "virtual-status.json").read_text(encoding="utf-8"))
            self.last_status = s
        except (OSError, ValueError):
            s = self.last_status
        fresh = bool(s) and time.time() - s.get("host_time", 0) < 1.5
        tracked = fresh and s.get("tracking_valid", False) and (s.get("pose_age_ms") or 0) <= 120
        gate = fresh and s.get("gate_fresh", False)
        trigger = s.get("trigger") if tracked else None
        output = s.get("output") if fresh else None
        enabled = bool(tracked and gate and output and output[0])
        forward, right = (output[1], output[2]) if enabled else (0, 0)
        if not fresh:
            message, color = "监听状态已过期，请启动“VR底盘（虚拟串口）”", RED
        elif not tracked:
            message, color = "手柄追踪无效：保持佩戴头显，确认右手柄模型可跟随", RED
        elif not gate:
            message, color = "等待使能：将 B 状态切到 Low，再切到 High", AMBER
        elif not s.get("hand_released"):
            message, color = "请先松开扳机，记录中立位置", AMBER
        elif not trigger:
            message, color = "已就绪：按住右手扳机并移动手柄", GREEN
        elif forward == 0 and right == 0:
            message, color = "扳机已按下：当前位于中心死区内", GREEN
        else:
            message, color = "正在输出底盘平移指令", GREEN
        self.banner.configure(text=message, fg=color)
        self.controls["tracking"].configure(text="有效" if tracked else "无效 / 等待", fg=GREEN if tracked else RED)
        self.controls["b"].configure(text=s.get("last_b") or "等待", fg=GREEN if gate else AMBER)
        self.controls["trigger"].configure(text="按下" if trigger else ("松开" if trigger is False else "无有效数据"), fg=GREEN if trigger else MUTED)
        self.controls["enabled"].configure(text="允许" if enabled else "零指令 / 停止", fg=GREEN if enabled else AMBER)
        self.forward.configure(text=f"{forward:+d}")
        self.right.configure(text=f"{right:+d}")
        position = s.get("position")
        if tracked and position:
            text = f"校准坐标（米）\n前 {position[1]:+.3f} · 右 {position[0]:+.3f}"
            origin = s.get("origin")
            if origin:
                text += f"\n距中立：前 {(position[1]-origin[0])*100:+.1f} cm / 右 {(position[0]-origin[1])*100:+.1f} cm"
        else:
            text = "手柄坐标：无有效追踪"
        self.position.configure(text=text)
        self.metrics.configure(text=f"位姿帧 {s.get('samples', 0)} · 无效 {s.get('invalid', 0)}    |    串口发送 {s.get('frames_sent', 0)} 帧 · {s.get('transport', 'COM21')} → COM22 → 原版选手端")
        def age(value):
            return "—" if value is None else f"{max(0, value):.0f} ms"
        self.freshness.configure(text=f"位姿距今 {age(s.get('pose_age_ms'))} · B 状态距今 {age(s.get('b_age_ms'))} · 状态更新 {datetime.now():%H:%M:%S}")
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(300, canvas.winfo_width()), max(180, canvas.winfo_height())
        cx, cy, radius = width/2, height/2, min(width, height)*.36
        canvas.create_line(cx-radius, cy, cx+radius, cy, fill=MUTED)
        canvas.create_line(cx, cy-radius, cx, cy+radius, fill=MUTED)
        canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="#34455b")
        canvas.create_text(cx, cy-radius-13, text="前", fill=TEXT)
        canvas.create_text(cx, cy+radius+13, text="后", fill=TEXT)
        canvas.create_text(cx-radius-17, cy, text="左", fill=TEXT)
        canvas.create_text(cx+radius+17, cy, text="右", fill=TEXT)
        x, y = cx+right/100*radius, cy-forward/100*radius
        canvas.create_line(cx, cy, x, y, fill=BLUE, width=3, arrow="last")
        canvas.create_oval(x-6, y-6, x+6, y+6, fill=GREEN if enabled else MUTED, outline="")
        while not self.board_results.empty():
            self.board.configure(text=self.board_results.get_nowait())
            self.board_busy = False
            self.board_button.configure(state="normal")
        self.root.after(100, self.update)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke-test", action="store_true")
    args = parser.parse_args()
    root = tk.Tk()
    window = StatusWindow(root)
    if args.smoke_test:
        root.withdraw()
        root.update_idletasks()
        assert len(window.controls) == 4
        assert CALIBRATION.is_file()
        print(json.dumps({"status_window": "ok", "cards": list(window.controls),
                          "calibration_sha256": hashlib.sha256(CALIBRATION.read_bytes()).hexdigest()}))
        root.destroy()
        return
    root.mainloop()


if __name__ == "__main__":
    main()
