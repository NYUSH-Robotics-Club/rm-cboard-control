#!/usr/bin/env python3
"""Connect host RTT readers through pyOCD or J-Link and own their sessions.

Normal pyOCD monitoring attaches without halting or resuming the target; failures
close the session. Reading an RTT up channel advances its target read offset.
"""
import dataclasses
import pathlib
import subprocess
import time
from typing import List, Optional


@dataclasses.dataclass
class RTTTransportConfig:
    """Probe settings; speed is in kHz and device names are backend-specific."""

    device: str = "STM32F407IG"
    serial: Optional[str] = None
    speed_khz: int = 4000
    connect_mode: str = "normal"
    backend: str = "auto"  # auto|pyocd|jlink
    iface: str = "SWD"
    elf: Optional[str] = None


class RTTTransport:
    def __init__(self, config: RTTTransportConfig) -> None:
        self.config = config
        self.backend_in_use: str = ""
        self.channel_info: str = ""
        self.error_message: Optional[str] = None

        self._session = None
        self._up_channels = None
        self._link = None

    def open(self) -> None:
        mode = self.config.backend.lower().strip()
        if mode not in ("auto", "pyocd", "jlink"):
            raise ValueError(f"Unsupported backend: {self.config.backend}")

        # On hosts that have a real J-Link attached, mixing pyOCD's J-Link
        # backend and pylink in the same process has proven fragile. Prefer a
        # direct J-Link path for auto mode when a J-Link is already visible.
        if mode == "auto" and self._has_connected_jlink():
            self._open_jlink()
            self.backend_in_use = "jlink"
            return

        if mode in ("auto", "pyocd"):
            try:
                self._open_pyocd()
                self.backend_in_use = "pyocd"
                return
            except Exception as exc:
                self.error_message = str(exc)
                # No J-Link was detected above; preserve the actionable pyOCD error.
                raise

        self._open_jlink()
        self.backend_in_use = "jlink"

    def close(self) -> None:
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

        if self._link is not None:
            try:
                self._link.rtt_stop()
            except Exception:
                pass
            try:
                self._link.close()
            except Exception:
                pass
            self._link = None

    def _has_connected_jlink(self) -> bool:
        try:
            import pylink

            return bool(pylink.JLink().connected_emulators())
        except Exception:
            return False

    def available_channels(self) -> List[str]:
        if self.backend_in_use == "pyocd" and self._up_channels is not None:
            names = []
            for idx, ch in enumerate(self._up_channels):
                name = getattr(ch, "name", None)
                names.append(f"{idx}:{name if name else '<unnamed>'}")
            return names

        if self.backend_in_use == "jlink" and self._link is not None:
            try:
                count = int(self._link.rtt_get_num_up_buffers())
            except Exception:
                count = 0
            return [f"{idx}:<unknown>" for idx in range(max(0, count))]

        return []

    def read(self, channel: int, max_bytes: int = 4096) -> bytes:
        if self.backend_in_use == "pyocd":
            if self._up_channels is None:
                return b""
            if channel < 0 or channel >= len(self._up_channels):
                return b""
            data = self._up_channels[channel].read()
            return bytes(data) if data else b""

        if self.backend_in_use == "jlink":
            if self._link is None:
                return b""
            data = self._link.rtt_read(channel, max_bytes)
            return bytes(data) if data else b""

        return b""

    def _open_pyocd(self) -> None:
        from pyocd.core.exceptions import TargetSupportError
        from pyocd.core.helpers import ConnectHelper
        from pyocd.core.target import Target
        from pyocd.debug.rtt import RTTControlBlock

        options: dict[str, object] = {
            "frequency": int(self.config.speed_khz) * 1000,
            "connect_mode": "attach" if self.config.connect_mode == "normal" else self.config.connect_mode,
            "resume_on_disconnect": False,
            "auto_unlock": False,
        }
        if self.config.connect_mode == "normal":
            # The STM32 pack's DebugCoreStart writes DHCSR outright, which can
            # resume a halted core. Use pyOCD's state-preserving implementation.
            options["pack.debug_sequences.disabled_sequences"] = ["DebugCoreStart"]

        target_override = self.config.device if self.config.device else None
        # The C-board .ioc specifies STM32F407IGH6TR. J-Link accepts the short
        # family name; select the exact H-package variant in the CMSIS pack.
        if target_override and target_override.lower() == "stm32f407ig":
            target_override = "stm32f407ighx"
        try:
            session = ConnectHelper.session_with_chosen_probe(
                unique_id=self.config.serial,
                target_override=target_override,
                options=options,
            )
        except TargetSupportError as exc:
            raise RuntimeError(
                f"pyOCD target {target_override!r} is not installed. Activate the project "
                f"environment, then run: python -m pyocd pack install {target_override}"
            ) from exc
        if session is None:
            raise RuntimeError("No ST-Link/CMSIS-DAP/J-Link probe found.")

        try:
            session.open()
            if session.target is None:
                raise RuntimeError("Failed to attach target.")

            if session.target.get_state() == Target.State.HALTED:
                if self.config.connect_mode == "normal":
                    raise RuntimeError("Target is halted; RTT monitoring did not resume it.")
                # Explicit halt/reset connection modes retain their prior restart behavior.
                session.target.resume()

            rtt_addr = self._resolve_rtt_cb_addr_from_elf()
            if rtt_addr is None:
                rtt = RTTControlBlock.from_target(session.target)
            else:
                rtt = RTTControlBlock.from_target(session.target, address=rtt_addr, size=0)
            rtt.start()
            up_channels = list(rtt.up_channels)
            if not up_channels:
                raise RuntimeError("RTT has no up channels.")
        except Exception:
            session.close()
            raise

        self._session = session
        self._up_channels = up_channels
        self.channel_info = ", ".join(self.available_channels())

    def _open_jlink(self) -> None:
        import pylink

        link = pylink.JLink()

        serial_int: Optional[int] = None
        if self.config.serial:
            serial_text = self.config.serial
            if ":" in serial_text:
                serial_text = serial_text.split(":", 1)[1]
            try:
                serial_int = int(serial_text.strip(), 10)
            except ValueError:
                serial_int = None

        # Avoid pylink's default USB0 selection path. In pylink 1.7.0 that path
        # can raise a misspelled exception name and break auto fallback after a
        # pyOCD attempt. If exactly one emulator is present, open it explicitly.
        if serial_int is None:
            emulators = link.connected_emulators()
            if len(emulators) == 1:
                serial_int = int(emulators[0].SerialNumber)
            elif len(emulators) > 1:
                raise RuntimeError(
                    "Multiple J-Link emulators found; pass --serial to select one explicitly."
                )

        if serial_int is not None:
            link.open(serial_no=serial_int)
        else:
            link.open()

        iface_text = self.config.iface.strip().lower()
        if iface_text == "jtag":
            link.set_tif(pylink.enums.JLinkInterfaces.JTAG)
        else:
            link.set_tif(pylink.enums.JLinkInterfaces.SWD)

        link.connect(self.config.device, speed=self.config.speed_khz, verbose=False)

        rtt_addr = None
        if self.config.elf:
            get_addr = getattr(link, "rtt_get_control_block_addr", None)
            if callable(get_addr):
                try:
                    rtt_addr = get_addr(self.config.elf)
                except Exception:
                    rtt_addr = None

        start_rtt = getattr(link, "rtt_start", None)
        if not callable(start_rtt):
            link.close()
            raise RuntimeError("RTT start is not supported by this J-Link API.")
        if rtt_addr:
            start_rtt(rtt_addr)
        else:
            start_rtt()

        self._link = link
        self.channel_info = ", ".join(self.available_channels())

    def _resolve_rtt_cb_addr_from_elf(self) -> Optional[int]:
        elf_path_text = self.config.elf
        if not elf_path_text:
            return None

        elf_path = pathlib.Path(elf_path_text)
        if not elf_path.is_file():
            return None

        try:
            result = subprocess.run(
                ["arm-none-eabi-nm", "-C", str(elf_path)],
                capture_output=True,
                check=False,
                text=True,
            )
        except FileNotFoundError:
            return None

        if result.returncode != 0:
            return None

        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) < 3 or fields[-1] != "_SEGGER_RTT":
                continue
            try:
                return int(fields[0], 16)
            except ValueError:
                return None

        return None


def choose_best_channel(transport: RTTTransport, candidates: List[int], score_fn) -> int:
    if not candidates:
        return 0

    scores = {idx: 0 for idx in candidates}
    for _ in range(8):
        for idx in candidates:
            data = transport.read(idx, 2048)
            if not data:
                continue
            scores[idx] += score_fn(idx, data)
        time.sleep(0.02)

    best = max(candidates, key=lambda i: scores[i])
    if scores[best] > 0:
        return best
    return candidates[0]
