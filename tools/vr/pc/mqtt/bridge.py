"""Quest -> official MQTT CustomControl -> VTM; SSH is never a command transport."""
import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import sys
import threading
import time

import paho.mqtt.client as mqtt

PC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PC))
sys.path.insert(0, str(PC.parent / "quest_runtime"))
from frankabot_vehicle import TranslationInput, pack_payload
from vr_config import PROFILE, RUNTIME_ROOT, SETTINGS
from custom_control import encode


@contextmanager
def publisher_lock():
    """Only one process may use this robot's MQTT identity, including diagnostics."""
    file = (RUNTIME_ROOT / "mqtt-publisher.lock").open("a+b")
    try:
        if file.seek(0, 2) == 0:
            file.write(b"0")
            file.flush()
        file.seek(0)
        try:
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(file.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError("An MQTT publisher is already running; stop it before diagnostics") from exc
        yield
    finally:
        file.close()


class Transport:
    def __init__(self, host, port, client_id):
        self.connected = threading.Event()
        self.generation = 0
        self.published = 0
        self.delivered = 0
        self.error = "waiting for MQTT connection"
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                                  clean_session=True, protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish
        self.client.reconnect_delay_set(1, 5)
        self.client.connect_async(host, port, keepalive=10)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason, properties):
        self.generation += 1
        self.error = str(reason)
        self.connected.clear()
        if not reason.is_failure:
            self.connected.set()

    def on_disconnect(self, client, userdata, flags, reason, properties):
        self.connected.clear()
        self.error = str(reason)

    def on_publish(self, client, userdata, mid, reason, properties):
        self.delivered += not reason.is_failure

    def send(self, payload):
        if not self.connected.is_set():
            return False
        # Live setpoints are never retained or queued for offline replay. QoS 0
        # delivery counts confirm a socket write, not radio reception or motion.
        info = self.client.publish("CustomControl", encode(payload), qos=0, retain=False)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.error = mqtt.error_string(info.rc)
            return False
        self.published += 1
        return True

    def close(self):
        self.client.disconnect()
        self.client.loop_stop()


def write_status(path, data):
    try:
        temporary = path.with_suffix(".pending")
        temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        temporary.replace(path)
        return True
    except OSError:
        # A Windows status reader must never interrupt the input listener.
        return False


def usb_worker(stop, state):
    import vr_input
    while not stop.is_set():
        try:
            state.update(vr_input.prepare_usb())
        except Exception as exc:
            state.update(ready=False, message=str(exc))
        stop.wait(5)


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=SETTINGS.get("mqtt_host", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--client-id", default=str(SETTINGS.get("mqtt_client_id", "1")))
    parser.add_argument("--status-file", type=Path, default=RUNTIME_ROOT / "mqtt-status.json")
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--diagnostic", type=int, metavar="FRAMES", help="send only neutral, diagnostic frames")
    parser.add_argument("--neutral-only", action="store_true")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("seconds must be positive")
    if args.diagnostic is not None and not 1 <= args.diagnostic <= 1000:
        parser.error("diagnostic frame count must be 1..1000")
    if args.diagnostic is None:
        from calibration_check import check
        print("Calibration: " + json.dumps(check(), ensure_ascii=False), flush=True)
    args.status_file.parent.mkdir(parents=True, exist_ok=True)
    stop_file = RUNTIME_ROOT / "mqtt-stop.request"
    stop_file.unlink(missing_ok=True)
    source = None
    stop = threading.Event()
    usb = {"ready": False, "message": "waiting for Quest USB"}
    mapping = TranslationInput(1, 1, 0, 1)
    transport = Transport(args.host, args.port, args.client_id)
    sequence = int(time.time_ns() // 1000000) & 65535
    deadline = time.monotonic() + args.seconds if args.seconds else float("inf")
    generation = -1
    sent_this_connection = False
    status = {}
    last_command = None
    if args.diagnostic is None:
        from vr_input import QuestInput
        source = QuestInput(calibration_path=PROFILE)
        source.start()
        threading.Thread(target=usb_worker, args=(stop, usb), daemon=True).start()
    print(f"MQTT {args.host}:{args.port} clientID={args.client_id}; CustomControl -> 0x0311; 20 Hz", flush=True)
    try:
        connection_deadline = time.monotonic() + 12
        while time.monotonic() < deadline and not stop_file.exists():
            tick = time.monotonic()
            now_ns = time.monotonic_ns()
            snap = source.snapshot() if source else {"packet": None, "diagnostics": {}, "count": 0, "invalid": 0, "gate_open": False}
            if source and not source.thread.is_alive():
                raise RuntimeError("Quest listener stopped: " + str(snap.get("reason")))
            if generation != transport.generation or not transport.connected.is_set():
                # A reconnect requires a new trigger release, and a neutral frame
                # before an active setpoint. It cannot replay a previous grip.
                mapping.origin = None
                mapping.released = False
                sent_this_connection = False
                generation = transport.generation
            packet = snap.get("packet")
            diagnostics = snap["diagnostics"]
            gate_ns = diagnostics.get("last_pause_received_monotonic_ns", 0)
            gate = bool(snap["gate_open"] and 0 <= now_ns - gate_ns <= 200_000_000)
            desired = mapping.update(packet, gate, now_ns) if source else None
            output = desired or (False, 0, 0)
            if args.neutral_only or args.diagnostic is not None or not sent_this_connection:
                output = (False, 0, 0)
            sequence = (sequence + 1) & 65535
            success = transport.send(pack_payload(sequence, *output, diagnostic=args.diagnostic is not None))
            sent_this_connection |= success
            command_state = (output, success, gate, (packet or {}).get("tracking_valid", False))
            if args.diagnostic is None and command_state != last_command:
                print(json.dumps(dict(event="command", host_time=time.time(), sequence=sequence,
                    output=output, published=success, gate=gate, tracking=command_state[3])), flush=True)
                last_command = command_state
            stamp = (packet or {}).get("host_received_monotonic_ns", 0)
            status = dict(
                host_time=time.time(), transport="MQTT CustomControl / 0x0311", listener_running=True,
                mqtt_connected=transport.connected.is_set(), mqtt_error=transport.error,
                mqtt_host=args.host, mqtt_port=args.port, mqtt_client_id=args.client_id,
                frames_sent=transport.published, mqtt_socket_delivered=transport.delivered,
                samples=snap["count"], invalid=snap["invalid"], gate_requested=snap["gate_open"],
                gate_fresh=gate, last_b=diagnostics.get("last_pause"),
                b_age_ms=(now_ns-gate_ns)/1e6 if gate_ns else None,
                pose_age_ms=(now_ns-stamp)/1e6 if stamp else None,
                trigger=(packet or {}).get("source_metadata", {}).get("flag"),
                tracking_valid=(packet or {}).get("tracking_valid", False),
                tracking_reason=(packet or {}).get("source_metadata", {}).get("reason"),
                position=(packet or {}).get("position"),
                raw_position=(packet or {}).get("source_metadata", {}).get("raw_position"),
                origin=mapping.origin, hand_released=mapping.released, output=output,
                sequence=sequence, last_publish_ok=success, usb=dict(usb),
                calibration_path=str(PROFILE),
                calibration_sha256=source.calibration_sha256 if source else None,
                neutral_only=args.neutral_only, diagnostic=args.diagnostic is not None,
            )
            write_status(args.status_file, status)
            if args.diagnostic is not None:
                if transport.published >= args.diagnostic:
                    print(json.dumps(status, ensure_ascii=False), flush=True)
                    return 0
                if tick > connection_deadline + args.diagnostic / 20:
                    raise RuntimeError("MQTT diagnostic incomplete: " + transport.error)
            time.sleep(max(0, .05 - (time.monotonic() - tick)))
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        # Shutdown sends a disabled input but leaves the original watchdog in charge.
        if source:
            transport.send(pack_payload((sequence + 1) & 65535, False, 0))
            source.stop_event.set()
            source.thread.join(timeout=2)
        stop.set()
        transport.close()
        final = {**status, "host_time": time.time(), "listener_running": False,
                 "mqtt_connected": False, "output": [False, 0, 0]}
        for _ in range(20):
            if write_status(args.status_file, final):
                break
            time.sleep(.025)


def main():
    with publisher_lock():
        return run()


if __name__ == "__main__":
    raise SystemExit(main())
