#!/usr/bin/env python3
"""Read coherent yaw/pitch callback snapshots over ST-Link and save CSV.

Only the host writes files. The target stays running; unknown snapshot versions,
image mismatches and read failures stop the monitor without changing motor state.
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time

import firmware as fw

MAGIC = 0x474D4F4E
SIZE = 148
HEADER = struct.Struct('<9I')
AXIS = struct.Struct('<4I3i6fI')
AXIS_NAMES = ('motor_id', 'flags', 'feedback_ms', 'command_unit', 'command_status',
              'command_raw', 'current_actual_raw', 'position_actual_ticks',
              'position_target_ticks', 'speed_actual_rpm', 'speed_target_rpm',
              'speed_loop_actual_rpm', 'speed_loop_dt_s', 'encoder_raw')


def decode_snapshot(data, sequence_after, live_tick):
    """Return a v1 sample, or None if a writer overlapped the read/uninitialized.

    Ages use wrapping MCU milliseconds. Targets keep validity flags; voltage
    commands and failed requests never become a reported current target.
    """
    if len(data) != SIZE:
        raise fw.Failure('Incomplete monitor snapshot')
    head = HEADER.unpack_from(data)
    seq, magic, version, size, count, tick, period, enabled, ready = head
    if seq & 1 or seq != sequence_after:
        return None
    if magic == 0 and count == 0:
        return None
    if (magic, version, size) != (MAGIC, 1, SIZE):
        raise fw.Failure('Unsupported monitor ABI; use matching firmware/tool versions')
    sample = dict(sequence=seq, callback_count=count, tick_ms=tick,
                  callback_dt_ms=period, enabled=enabled, startup_ready=ready,
                  snapshot_age_ms=(live_tick-tick) & 0xffffffff)
    for index, name in enumerate(('yaw', 'pitch')):
        axis = dict(zip(AXIS_NAMES, AXIS.unpack_from(data, HEADER.size+index*AXIS.size)))
        for value in axis.values():
            if isinstance(value, float) and not math.isfinite(value):
                raise fw.Failure('Non-finite monitor value; stopping instead of plotting corrupt data')
        axis['feedback_age_ms'] = ((live_tick-axis['feedback_ms']) & 0xffffffff) if axis['flags'] & 2 else None
        axis['current_target_raw'] = (axis['command_raw'] if axis['command_unit'] == 1
                                      and axis['command_status'] == 0 else None)
        sample[name] = axis
    return sample


def local_image(elf, gcc, directory):
    """Extract symbol addresses and image from this ELF, never from old offsets."""
    suffix = '.exe' if os.name == 'nt' else ''
    bindir = Path(gcc).parent
    nm = bindir / ('arm-none-eabi-nm'+suffix)
    objcopy = bindir / ('arm-none-eabi-objcopy'+suffix)
    output = subprocess.check_output([str(nm), '-S', '--defined-only', str(elf)], text=True)
    symbols = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] in ('g_gimbal_monitor', 'uwTick'):
            symbols[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    if symbols.get('g_gimbal_monitor', (0, 0))[1] != SIZE or symbols.get('uwTick', (0, 0))[1] != 4:
        raise fw.Failure('ELF lacks the v1 monitor. Build and flash the monitor firmware first.')
    for address, size in symbols.values():
        if not 0x20000000 <= address < address+size <= 0x20020000 or address % 4:
            raise fw.Failure('Monitor symbols are outside aligned STM32F407 SRAM')
    binary = directory / 'expected.bin'
    subprocess.run([str(objcopy), '-O', 'binary', str(elf), str(binary)], check=True)
    expected = binary.read_bytes()
    if not 8 <= len(expected) <= 1024*1024:
        raise fw.Failure('ELF image is outside STM32F407 Flash size')
    return symbols, expected


class TclClient:
    """Synchronous OpenOCD Tcl RPC, bounded responses and socket timeout."""
    def __init__(self, sock):
        self.sock = sock

    def call(self, command):
        self.sock.sendall(command.encode('utf-8')+b'\x1a')
        result = bytearray()
        while not result.endswith(b'\x1a'):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise fw.Failure('OpenOCD disconnected')
            result.extend(chunk)
            if len(result) > 1024*1024:
                raise fw.Failure('Oversized OpenOCD reply')
        return result[:-1].decode('utf-8').strip()

    def words(self, address, count):
        reply = self.call(f'read_memory 0x{address:x} 32 {count}')
        try:
            values = [int(word, 0) for word in reply.split()]
        except ValueError:
            raise fw.Failure(f'OpenOCD read failed: {reply}') from None
        if len(values) != count or any(not 0 <= word <= 0xffffffff for word in values):
            raise fw.Failure(f'Incomplete OpenOCD read: {reply}')
        return values


def openocd_command(executable, serial, port):
    """Disable automatic examine hooks/work RAM; start only a local read service."""
    if not serial.isalnum():
        raise fw.Failure('Invalid probe serial')
    return [str(executable), '-c', f'noinit; bindto 127.0.0.1; gdb_port disabled; telnet_port disabled; tcl_port {port}',
            '-f', str(fw.ROOT/'tools/openocd/stm32f407-stlink.cfg'),
            '-c', f'fw_select_serial {serial}; adapter speed 1000; '
                  'stm32f4x.cpu configure -event examine-end {} -work-area-size 0; init']


def verify_image_readonly(client, expected, directory):
    """Copy Flash to a host file and compare locally; no target checksum algorithm."""
    if client.call('stm32f4x.cpu curstate') != 'running':
        raise fw.Failure('MCU is not running; monitor will not resume/reset it')
    if client.words(0xe0042000, 1)[0] & 0xfff != 0x413:
        raise fw.Failure('Target is not the configured STM32F407 family')
    reply = client.call('read_memory 0x1fff7a22 16 1')
    if reply not in ('1024', '0x400', '0x0400'):
        # OpenOCD may render the single value with leading zeros.
        try:
            valid = int(reply, 0) == 1024
        except ValueError:
            valid = False
        if not valid:
            raise fw.Failure('Target Flash size is not 1024 KiB')
    actual_path = directory/'flash.bin'
    client.call(f'dump_image {fw.tcl_word(actual_path.as_posix())} 0x08000000 {len(expected)}')
    if not actual_path.is_file() or actual_path.read_bytes() != expected:
        raise fw.Failure('Board Flash does not match ELF. Flash the chosen build before monitoring.')


def flatten(sample):
    row = {k: v for k, v in sample.items() if k not in ('yaw', 'pitch')}
    for name in ('yaw', 'pitch'):
        row.update({name+'_'+k: v for k, v in sample[name].items()})
    return row


def display(sample, rate, skipped, csv_path):
    lines = [f'云台实时监控 | 采样 {rate:.1f} Hz | 回调 #{sample["callback_count"]} '
             f'间隔 {sample["callback_dt_ms"]} ms | 未采回调 {skipped}',
             f'使能 {sample["enabled"]} / 启动对齐 {sample["startup_ready"]} | '
             f'快照年龄 {sample["snapshot_age_ms"]} ms | 回调停止更新 {sample.get("callback_idle_ms", 0):.0f} ms | Ctrl+C退出',
             '电流/电压均为协议原始刻度；位置为编码刻度（8192/圈）；速度为RPM。']
    if sample['snapshot_age_ms'] > 100 or sample.get('callback_idle_ms', 0) > 100:
        lines.append('*** 快照已过期：以下是历史值，控制回调可能停止 ***')
    for name in ('yaw', 'pitch'):
        a = sample[name]
        if not a['flags'] & 1:
            lines.append(f'{name.upper()}: 未配置/未初始化')
            continue
        age = a['feedback_age_ms']
        target = a['current_target_raw']
        state = '控制有效' if a['flags'] & 8 else '控制未激活（目标无效）'
        lines += [f'\n{name.upper()} ID{a["motor_id"]} | {state} | 反馈年龄 {age} ms | 服务状态 {a["command_status"]}',
                  f'  电流 实际 {a["current_actual_raw"]:8d} / 目标 {target if target is not None else "N/A"}',
                  f'  位置 实际 {a["position_actual_ticks"]:10.2f} / 目标 {a["position_target_ticks"]:10.2f}'
                  + (' [闭环]' if a['flags'] & 16 else ' [未闭合位置环/备忘值]'),
                  f'  速度 实际 {a["speed_actual_rpm"]:10.2f} / 目标 '
                  + (f'{a["speed_target_rpm"]:10.2f}' if a['flags'] & 8 else 'N/A')]
        if age is None or age > 100:
            lines.append('  *** 电机反馈缺失/过期 ***')
        if a['command_unit'] != 1:
            kind = '电压' if a['command_unit'] == 2 else '未知单位'
            lines.append(f'  {kind}命令请求 {a["command_raw"]}（不存在可读取的目标电流）')
        if a['flags'] & 32:
            lines.append(f'  当前内环采用IMU反馈 {a["speed_loop_actual_rpm"]:.2f} RPM，不能与电机相对转速混用')
    lines.append(f'\nCSV: {csv_path}')
    print(('\033[H\033[J' if sys.stdout.isatty() else '')+'\n'.join(lines), flush=True)


def monitor(args):
    cfg = fw.load_config()
    gcc = fw.discover(cfg, 'gcc')
    executable = fw.discover(cfg, 'openocd')
    robot = fw.selected(cfg)
    elf = (args.elf or fw.build_dir(cfg, robot, {'gcc': gcc})/'NYUSH_Infantry.elf').resolve()
    # Local preparation is also useful without an attached board.
    with fw.operation_lock(), tempfile.TemporaryDirectory(prefix='rm-monitor-') as temp:
        directory = Path(temp)
        symbols, expected = local_image(elf, gcc, directory)
        if args.prepare_only:
            print(json.dumps({'elf': str(elf), 'symbols': symbols, 'flash_bytes': len(expected),
                              'hardware_commands_executed': 0}, indent=2))
            return
        settings = fw.flash_settings(cfg).copy()
        if args.serial:
            settings['serial'] = args.serial
        serial = fw.choose_probe(fw.enumerate_probes(), settings)
        csv_path = args.csv or fw.ROOT/'build/monitor'/dt.datetime.now().strftime('gimbal-%Y%m%d-%H%M%S-%f.csv')
        csv_path = csv_path.resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive files preserve older recordings, including a failed connection log.
        with csv_path.open('x', newline='', encoding='utf-8') as csv_file, csv_path.with_suffix('.openocd.log').open('x', encoding='utf-8') as log:
            with socket.socket() as port_socket:
                port_socket.bind(('127.0.0.1', 0))
                port = port_socket.getsockname()[1]
            process = subprocess.Popen(openocd_command(executable, serial, port), stdout=log, stderr=subprocess.STDOUT)
            sock = None
            try:
                deadline = time.monotonic()+10
                while sock is None:
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise fw.Failure(f'OpenOCD failed to start; see {csv_path.with_suffix(".openocd.log")}')
                    try:
                        sock = socket.create_connection(('127.0.0.1', port), timeout=0.2)
                    except OSError:
                        time.sleep(0.05)
                sock.settimeout(10)
                client = TclClient(sock)
                verify_image_readonly(client, expected, directory)
                with csv_path.with_suffix('.json').open('x', encoding='utf-8') as metadata:
                    json.dump({'elf': str(elf), 'elf_sha256': fw.sha256(elf),
                        'probe': serial, 'flash_matches_elf': True, 'requested_hz': args.hz,
                        'symbols': symbols, 'snapshot_version': 1}, metadata, indent=2)
                address = symbols['g_gimbal_monitor'][0]
                tick_address = symbols['uwTick'][0]
                started = time.monotonic()
                writer = None
                previous = None
                previous_time = started
                last_good = started
                last_callback = started
                skipped = 0
                while args.duration == 0 or time.monotonic()-started < args.duration:
                    iteration = time.monotonic()
                    words = client.words(address, SIZE//4)
                    sequence = client.words(address, 1)[0]
                    tick = client.words(tick_address, 1)[0]
                    sample = decode_snapshot(struct.pack('<37I', *words), sequence, tick)
                    if sample is not None:
                        last_good = time.monotonic()
                        sample['host_elapsed_s'] = last_good-started
                        sample['read_ms'] = (last_good-iteration)*1000
                        delta = ((sample['callback_count']-previous) & 0xffffffff) if previous is not None else 0
                        if delta > 0x7fffffff:
                            raise fw.Failure('MCU counter restarted; start a new recording after reset')
                        if previous is None or delta:
                            last_callback = last_good
                        # Host time catches a halted CPU even when uwTick also stops.
                        sample['callback_idle_ms'] = (last_good-last_callback)*1000
                        skipped += max(0, delta-1)
                        row = flatten(sample)
                        if writer is None:
                            writer = csv.DictWriter(csv_file, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                        csv_file.flush()
                        rate = 1/(last_good-previous_time) if previous is not None else 0
                        display(sample, rate, skipped, csv_path)
                        previous, previous_time = sample['callback_count'], last_good
                    elif time.monotonic()-last_good > 2:
                        raise fw.Failure('No coherent callback snapshot for 2 seconds; MCU may not be dispatching')
                    time.sleep(max(0, 1/args.hz-(time.monotonic()-iteration)))
            finally:
                if sock is not None:
                    sock.close()
                # Killing the host server does not issue target halt/reset/resume.
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, help='Exact ELF flashed on the board; defaults to configured build')
    parser.add_argument('--serial', help='ST-Link serial, otherwise use saved probe selection')
    parser.add_argument('--hz', type=float, default=20, help='Host sampling Hz, 1..100 (default 20)')
    parser.add_argument('--duration', type=float, default=0, help='Seconds to record; 0 runs until Ctrl+C')
    parser.add_argument('--csv', type=Path, help='New CSV output path; existing files are never overwritten')
    parser.add_argument('--prepare-only', action='store_true', help='Validate local ELF, without connecting to hardware')
    args = parser.parse_args(argv)
    if not math.isfinite(args.hz) or not 1 <= args.hz <= 100 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--hz must be 1..100; --duration must be finite and nonnegative')
    monitor(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n监控已停止，CSV已保存。')
    except (fw.Failure, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        sys.exit(1)
