#!/usr/bin/env python3
"""Serve RTT telemetry to browser clients and save host-side monitor records.

Probe connection is owned by RTTTransport; connection failures stop startup.
"""
import argparse
import asyncio
import base64
import binascii
import http.server
import json
from http import HTTPStatus
import os
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Set

import websockets
from websockets.datastructures import Headers
from websockets.http11 import Request, Response

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from scripts.rtt_common.monitor_writer import MonitorWriter
from scripts.rtt_common.telemetry import FrameParser
from scripts.rtt_common.rtt_transport import RTTTransport, RTTTransportConfig


STREAM_LOG = 0
STREAM_DASHBOARD = 1
PACKET_MAGIC = 0x5753
PACKET_VERSION = 1
PACKET_HEADER = struct.Struct("<HBBIQ")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge RTT logger/dashboard data to WebSocket clients.",
    )
    parser.add_argument("--backend", choices=["auto", "pyocd", "jlink"], default="auto")
    parser.add_argument("--device", default="STM32F407IG")
    parser.add_argument("--iface", default="SWD")
    parser.add_argument("--speed", type=int, default=4000)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--connect", choices=["normal", "halt", "under-reset"], default="normal")
    parser.add_argument("--elf", default=os.path.join("build", "basic_framework.elf"))
    parser.add_argument("--dashboard-channel", type=int, default=1)
    parser.add_argument("--log-channel", type=int, default=0)
    parser.add_argument("--poll-ms", type=int, default=5)
    parser.add_argument(
        "--monitor-dir",
        default=os.path.join("monitor"),
        help="Directory for exported monitor txt files.",
    )
    parser.add_argument(
        "--monitor-prefix",
        default="logger",
        help="Filename prefix for exported monitor txt files.",
    )
    parser.add_argument(
        "--no-monitor-save",
        action="store_true",
        help="Disable monitor txt export.",
    )
    parser.add_argument("--ws-host", default=None)
    parser.add_argument("--ws-port", type=int, default=None)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument(
        "--viewer-file",
        default=os.path.join("scripts", "dashboard", "rtt_web_viewer.html"),
    )
    return parser.parse_args()


class ViewerHandler(http.server.BaseHTTPRequestHandler):
    viewer_html: bytes = b""

    def load_viewer(self) -> bytes:
        return self.viewer_html

    def do_GET(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] in ("/", "/index.html"):
            try:
                body = self.load_viewer()
            except (OSError, UnicodeError):
                self.send_error(503, "Viewer file unavailable; retry after saving it.")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(b"Not found")

    def log_message(self, _format: str, *_args) -> None:
        return


class RTTWebSocketBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.clients: Set = set()
        self.transport = RTTTransport(
            RTTTransportConfig(
                device=args.device,
                serial=args.serial,
                speed_khz=args.speed,
                connect_mode=args.connect,
                backend=args.backend,
                iface=args.iface,
                elf=args.elf,
            )
        )
        self.stop_event = asyncio.Event()
        self.bytes_dashboard = 0
        self.bytes_log = 0
        self.sent_packets = 0
        self.http_server: http.server.ThreadingHTTPServer | None = None
        self.http_thread: threading.Thread | None = None
        # Fail before opening a probe if the page cannot be loaded.
        _ = self.viewer_html
        self.monitor_parser = FrameParser()
        self.monitor_writer: MonitorWriter | None = None
        self.monitor_error: str | None = None
        if not args.no_monitor_save:
            try:
                self.monitor_writer = MonitorWriter.create(
                    directory=args.monitor_dir,
                    prefix=args.monitor_prefix,
                )
            except Exception as exc:  # pylint: disable=broad-except
                self.monitor_error = str(exc)

    @property
    def viewer_html(self) -> bytes:
        """Load each HTTP request from disk so refresh picks up saved UI changes."""
        viewer = Path(self.args.viewer_file).read_text(encoding="utf-8")
        config = json.dumps({"wsPort": self.ws_port})
        return viewer.replace("</head>", f"<script>window.RTT_CONFIG={config};</script></head>").encode("utf-8")

    @property
    def ws_host(self) -> str:
        return self.args.ws_host if self.args.ws_host is not None else self.args.http_host

    @property
    def ws_port(self) -> int:
        return self.args.ws_port if self.args.ws_port is not None else self.args.http_port

    def _use_single_listener(self) -> bool:
        return self.ws_host == self.args.http_host and self.ws_port == self.args.http_port

    @staticmethod
    def _make_http_response(
        status: HTTPStatus,
        body: bytes,
        content_type: str | None = None,
        extra_headers: dict[str, str] | None = None,
    ) -> Response:
        headers = Headers()
        if content_type:
            headers["Content-Type"] = content_type
        headers["Content-Length"] = str(len(body))
        if extra_headers is not None:
            for name, value in extra_headers.items():
                headers[name] = value
        return Response(status.value, status.phrase, headers, body)

    @staticmethod
    def _header_tokens(headers: Headers, header_name: str) -> set[str]:
        tokens: set[str] = set()
        for raw_value in headers.get_all(header_name):
            for token in raw_value.split(","):
                normalized = token.strip().lower()
                if normalized:
                    tokens.add(normalized)
        return tokens

    def _has_websocket_headers(self, request: Request) -> bool:
        headers = request.headers
        return bool(
            self._header_tokens(headers, "Upgrade")
            or headers.get("Sec-WebSocket-Key") is not None
            or headers.get("Sec-WebSocket-Version") is not None
        )

    def _websocket_handshake_error(self, request: Request) -> str | None:
        headers = request.headers
        upgrade_tokens = self._header_tokens(headers, "Upgrade")
        connection_tokens = self._header_tokens(headers, "Connection")

        if len(upgrade_tokens) != 1 or "websocket" not in upgrade_tokens:
            upgrade_value = ", ".join(headers.get_all("Upgrade")) or "<missing>"
            return f"invalid Upgrade header: {upgrade_value}"
        if "upgrade" not in connection_tokens:
            connection_value = ", ".join(headers.get_all("Connection")) or "<missing>"
            return f"invalid Connection header: {connection_value}"

        key_values = headers.get_all("Sec-WebSocket-Key")
        if len(key_values) != 1:
            return "invalid Sec-WebSocket-Key header"
        key = key_values[0]
        try:
            raw_key = base64.b64decode(key.encode(), validate=True)
        except binascii.Error:
            return f"invalid Sec-WebSocket-Key header: {key}"
        if len(raw_key) != 16:
            return f"invalid Sec-WebSocket-Key header: {key}"

        version_values = headers.get_all("Sec-WebSocket-Version")
        if len(version_values) != 1:
            return "invalid Sec-WebSocket-Version header"
        version = version_values[0]
        if version != "13":
            return f"unsupported Sec-WebSocket-Version: {version}"

        return None

    def _make_ws_guidance_response(self, websocket_error: str) -> Response:
        body = (
            "This endpoint expects a WebSocket client.\n"
            f"Request issue: {websocket_error}\n"
            f"Viewer: http://{self.args.http_host}:{self.args.http_port}/\n"
            f"WebSocket URL: ws://{self.ws_host}:{self.ws_port}/\n"
        ).encode("utf-8")
        return self._make_http_response(
            HTTPStatus.UPGRADE_REQUIRED,
            body,
            "text/plain; charset=utf-8",
            {"Upgrade": "websocket"},
        )

    @staticmethod
    def _make_packet(stream_id: int, payload: bytes) -> bytes:
        host_ts_ms = int(time.time() * 1000)
        header = PACKET_HEADER.pack(
            PACKET_MAGIC,
            PACKET_VERSION,
            stream_id,
            len(payload),
            host_ts_ms,
        )
        return header + payload

    async def _broadcast(self, packet: bytes) -> None:
        if not self.clients:
            return

        clients = list(self.clients)
        results = await asyncio.gather(
            *(client.send(packet) for client in clients),
            return_exceptions=True,
        )
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.clients.discard(client)
        self.sent_packets += 1

    async def _producer_loop(self) -> None:
        while not self.stop_event.is_set():
            dash_data = self.transport.read(self.args.dashboard_channel, 4096)
            if dash_data:
                self.bytes_dashboard += len(dash_data)
                if self.monitor_writer is not None:
                    host_rx_ms = int(time.time() * 1000)
                    frames = self.monitor_parser.feed(dash_data, host_rx_ms)
                    self.monitor_writer.write_frames(frames)
                await self._broadcast(self._make_packet(STREAM_DASHBOARD, dash_data))

            log_data = self.transport.read(self.args.log_channel, 4096)
            if log_data:
                self.bytes_log += len(log_data)
                await self._broadcast(self._make_packet(STREAM_LOG, log_data))

            await asyncio.sleep(max(self.args.poll_ms, 1) / 1000.0)

    async def _stats_loop(self) -> None:
        last_dash = 0
        last_log = 0
        while not self.stop_event.is_set():
            await asyncio.sleep(2.0)
            now_dash = self.bytes_dashboard
            now_log = self.bytes_log
            delta_dash = now_dash - last_dash
            delta_log = now_log - last_log
            last_dash = now_dash
            last_log = now_log
            print(
            f"clients={len(self.clients)} "
                f"dashboard={delta_dash / 2.0:.1f}B/s "
                f"log={delta_log / 2.0:.1f}B/s "
                f"packets={self.sent_packets}"
            )

    async def _ws_handler(self, websocket) -> None:
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    def _serve_http_request(self, request: Request) -> Response:
        path = request.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            try:
                body = self.viewer_html
            except (OSError, UnicodeError):
                return self._make_http_response(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    b"Viewer file unavailable; retry after saving it.\n",
                    "text/plain; charset=utf-8",
                    {"Cache-Control": "no-store"},
                )
            return self._make_http_response(
                HTTPStatus.OK,
                body,
                "text/html; charset=utf-8",
                {"Cache-Control": "no-store"},
            )
        if path == "/favicon.ico":
            return self._make_http_response(HTTPStatus.NO_CONTENT, b"")
        if path == "/healthz":
            return self._make_http_response(
                HTTPStatus.OK,
                b"ok\n",
                "text/plain; charset=utf-8",
            )
        return self._make_http_response(
            HTTPStatus.NOT_FOUND,
            b"Not found\n",
            "text/plain; charset=utf-8",
        )

    def _process_http_request(self, _connection, request: Request) -> Response | None:
        if self._has_websocket_headers(request):
            websocket_error = self._websocket_handshake_error(request)
            if websocket_error is None:
                return None
            return self._make_ws_guidance_response(websocket_error)

        return self._serve_http_request(request)

    def _process_ws_request(self, _connection, request: Request) -> Response | None:
        if request.path.split("?", 1)[0] == "/healthz":
            return self._make_http_response(
                HTTPStatus.OK,
                b"ok\n",
                "text/plain; charset=utf-8",
            )

        websocket_error = self._websocket_handshake_error(request)
        if websocket_error is None:
            return None

        return self._make_ws_guidance_response(websocket_error)

    def _start_http_server(self) -> None:
        bridge = self

        class BridgeViewerHandler(ViewerHandler):
            def load_viewer(self) -> bytes:
                return bridge.viewer_html

        self.http_server = http.server.ThreadingHTTPServer(
            (self.args.http_host, self.args.http_port),
            BridgeViewerHandler,
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.http_thread.start()

    async def run(self) -> int:
        try:
            self.transport.open()
        except Exception as exc:
            print(str(exc))
            print("Check target support and probe access; ST-Link uses --backend pyocd.")
            return 2

        channels = self.transport.available_channels()
        print(
            f"RTT connected backend={self.transport.backend_in_use} "
            f"channels={channels or '<unknown>'}"
        )
        try:
            if not self._use_single_listener():
                self._start_http_server()
        except Exception as exc:
            self.transport.close()
            print(f"HTTP server start failed: {exc}")
            return 3

        print(f"HTTP viewer: http://{self.args.http_host}:{self.args.http_port}/")
        print(
            f"WebSocket: ws://{self.ws_host}:{self.ws_port} "
            f"(dashboard={self.args.dashboard_channel}, log={self.args.log_channel})"
        )
        if self.monitor_writer is not None:
            print(f"Monitor export: {self.monitor_writer.path}")
        elif self.monitor_error:
            print(f"Monitor export disabled: {self.monitor_error}")

        server = await websockets.serve(
            self._ws_handler,
            self.ws_host,
            self.ws_port,
            max_size=None,
            ping_interval=20,
            ping_timeout=20,
            process_request=self._process_http_request if self._use_single_listener() else self._process_ws_request,
        )

        producer_task = asyncio.create_task(self._producer_loop())
        stats_task = asyncio.create_task(self._stats_loop())

        try:
            await self.stop_event.wait()
        finally:
            producer_task.cancel()
            stats_task.cancel()
            await asyncio.gather(producer_task, stats_task, return_exceptions=True)
            server.close()
            await server.wait_closed()
            if self.http_server is not None:
                self.http_server.shutdown()
                self.http_server.server_close()
                self.http_server = None
            if self.http_thread is not None and self.http_thread.is_alive():
                self.http_thread.join(timeout=1.0)
                self.http_thread = None
            if self.monitor_writer is not None:
                self.monitor_writer.close()
            self.transport.close()
        return 0


async def _amain() -> int:
    args = parse_args()
    bridge = RTTWebSocketBridge(args)
    return await bridge.run()


def main() -> int:
    try:
        return asyncio.run(_amain())
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
