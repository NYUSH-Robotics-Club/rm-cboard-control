"""Read OS USB metadata to select ST-Link serials without opening SWD.
Missing or ambiguous serials stop selection; no drivers or devices are changed.
"""
import json
import os
import platform
from pathlib import Path
import re
import subprocess

# Debug-mode IDs handled by OpenOCD ST-Link interfaces.
# Loader IDs (374d/3755) are deliberately excluded.
STLINK_PIDS = {"3748", "374b", "374e", "374f", "3752", "3753"}


def linux_serials(root=Path("/sys/bus/usb/devices")):
    """Read physical USB descriptors from sysfs; propagate permission/read failures."""
    serials = []
    for device in root.iterdir():
        if not (device / "idVendor").exists():
            continue
        if (device / "idVendor").read_text().strip().lower() != "0483":
            continue
        product = (device / "idProduct").read_text().strip().lower()
        if product in STLINK_PIDS:
            # Preserve raw control characters; sysfs adds exactly one final newline.
            serial = (device / "serial").read_bytes().decode("utf-8").removesuffix("\n")
            if product == "3748" and len(serial) == 12:
                # Match OpenOCD stlink_usb_get_alternate_serial for old V2 descriptors.
                serial = serial.encode("latin-1").hex().upper()
            serials.append(serial)
    return serials


def windows_serials(devices):
    """Use physical device instance serials; composite interface IDs are not serials."""
    serials = []
    for device in devices:
        match = re.fullmatch(r"USB\\VID_0483&PID_([0-9A-F]{4})\\(.+)", device.get("PNPDeviceID", ""), re.I)
        if match and match[1].lower() in STLINK_PIDS:
            if device.get("ConfigManagerErrorCode") != 0:
                raise ValueError("ST-Link USB device has an OS error; inspect Device Manager.")
            serials.append(match[2])
    return serials


def mac_serials(tree):
    """Extract physical ST-Link serial descriptors from system_profiler JSON."""
    serials = []
    if isinstance(tree, list):
        for item in tree:
            serials.extend(mac_serials(item))
    elif isinstance(tree, dict):
        vendor = re.match(r"0x([0-9a-f]{4})\b", str(tree.get("vendor_id", "")), re.I)
        product = re.match(r"0x([0-9a-f]{4})\b", str(tree.get("product_id", "")), re.I)
        if vendor and vendor[1].lower() == "0483" and product and product[1].lower() in STLINK_PIDS:
            serials.append(tree.get("serial_num", ""))
        for value in tree.values():
            if isinstance(value, (list, dict)):
                serials.extend(mac_serials(value))
    return serials


def probe_serials():
    """Return unique usable OS serials; duplicates/unknown strings refuse selection."""
    if platform.system() == "Linux":
        serials = linux_serials()
    elif os.name == "nt":
        serials = windows_serials(windows_devices())
    elif platform.system() == "Darwin":
        result = subprocess.run(["/usr/sbin/system_profiler", "SPUSBDataType", "-json"],
                                capture_output=True, text=True, timeout=30, check=True)
        serials = mac_serials(json.loads(result.stdout))
    else:
        raise ValueError("USB discovery is unsupported on this host.")
    if any(not re.fullmatch(r"[A-Za-z0-9]+", s) for s in serials):
        raise ValueError("ST-Link has a missing/non-alphanumeric USB serial; automatic selection is unsafe.")
    if len(serials) != len(set(serials)):
        raise ValueError("Multiple physical ST-Link devices share a serial; disconnect ambiguous probes.")
    return serials


def windows_devices():
    """Return present USB devices; a query failure is not an empty device list."""
    powershell = Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32/WindowsPowerShell/v1.0/powershell.exe"
    script = r"""$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$usbDevices = @(Get-CimInstance Win32_PnPEntity -Filter 'Present = True' |
    Where-Object { $_.PNPDeviceID -like 'USB\*' } |
    Select-Object Name, PNPDeviceID, Service, ConfigManagerErrorCode)
ConvertTo-Json -InputObject $usbDevices -Compress
"""
    result = subprocess.run([str(powershell), "-NoProfile", "-NonInteractive", "-Command", script],
                            capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=20)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "Windows USB query failed")
    devices = json.loads(result.stdout.lstrip("\ufeff"))
    if not isinstance(devices, list) or any(not isinstance(d, dict) for d in devices):
        raise ValueError("Unexpected Windows USB query format")
    return devices


def diagnose(devices):
    """Classify observed USB state, without claiming that a target is connected."""
    # IDs are from STSW-LINK009 v3; unknown future IDs remain visible in the report.
    known = re.compile(r"VID_0483&PID_(?:3748|374A|374B|374E|374F|3752|3753|3754|3757)", re.I)
    loader = re.compile(r"VID_0483&PID_(?:374D|3755)", re.I)
    probes = [d for d in devices if known.search(d.get("PNPDeviceID", ""))
              or re.search(r"ST[- ]?LINK", d.get("Name") or "", re.I)]
    if any(loader.search(d.get("PNPDeviceID", "")) for d in devices):
        return "STLINK_LOADER", "ST-Link USB loader detected. Check probe mode in ST tools; no automatic firmware update is performed."
    if probes:
        if any(d.get("ConfigManagerErrorCode") not in (0, None) for d in probes):
            return "USB_DRIVER_ERROR", "ST-Link is visible to Windows with a device error. Check Device Manager error code and the official STSW-LINK009 driver."
        return "USB_PRESENT_CLI_UNAVAILABLE", "Windows sees ST-Link. Check its physical USB serial, active USB driver and other probe sessions if OpenOCD cannot open it."
    if any(d.get("ConfigManagerErrorCode") not in (0, None) for d in devices):
        return "USB_DEVICE_ERROR", "A USB device has an error but cannot be identified as ST-Link. Check the listed error and reconnect with a known data cable."
    return "NO_STLINK_USB", "Windows has no recognized ST-Link USB device. Check the probe USB port, data cable and direct PC connection; changing the MCU/SWD target config cannot create a missing USB device."


def report():
    if os.name != "nt":
        print("[USB] Check lsusb and USB node permissions on Linux, or System Information > USB on macOS.")
        return
    try:
        devices = windows_devices()
        code, message = diagnose(devices)
        print(f"[USB:{code}] {message}")
        print("[USB present devices] " + json.dumps(devices, ensure_ascii=False, indent=2))
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        print(f"[USB:QUERY_FAILED] Unable to determine USB state: {exc}")
