"""Configure, build, inspect and explicitly flash the STM32F407 firmware.
Hardware is accessed only by flash; doctor only enumerates USB probes. Fail closed.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import struct
import subprocess
import sys
from datetime import datetime, timezone

import stlink_usb

ALIASES = {}

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / ".firmware.local.json"
ROBOTS = ("infantry_standard", "sentry_swerve")
# Compatibility floors only. Bootstrap installs latest stable releases, and each
# build records actual versions instead of requiring an old documentation pin.
MIN_VERSIONS = {"cmake": (3, 22, 0), "openocd": (0, 11, 0)}
NAMES = {"cmake": "cmake", "ninja": "ninja", "gcc": "arm-none-eabi-gcc",
         "openocd": "openocd", "just": "just", "git": "git"}


class Failure(RuntimeError):
    pass


def native_path(value):
    path = Path(value)
    for original, alias in ALIASES.items():
        if path.is_relative_to(original):
            return str(alias / path.relative_to(original))
    return str(value)


@contextlib.contextmanager
def windows_paths(paths):
    """Expose Unicode/space paths through temporary drive aliases for MinGW binutils.
    subst changes only drive mappings; it never copies/deletes the source or tools.
    """
    mapped = []
    subst = Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32/subst.exe"
    try:
        if os.name == "nt":
            roots = [ROOT] + [Path(p).parent.parent for p in paths.values()]
            for original in dict.fromkeys(roots):
                if str(original).isascii() and " " not in str(original):
                    continue
                if any(original.is_relative_to(existing) for existing in ALIASES):
                    continue
                drive = next((f"{c}:" for c in "ZYXWVUTSRQP" if not Path(f"{c}:/").exists()), None)
                if not drive:
                    raise Failure("No free drive letter for Windows tool path compatibility. Free one of P: through Z: and retry.")
                run([subst, drive, original])
                mapped.append(drive)
                ALIASES[original] = Path(drive + "/")
        yield
    finally:
        ALIASES.clear()
        for drive in reversed(mapped):
            run([subst, drive, "/d"])


def run(args, *, env=None, show=False, cwd=ROOT, log=None):
    """Run an argument array without a shell; never continue after a failure."""
    args = [native_path(a) if Path(str(a)).is_absolute() else str(a) for a in args]
    if show:
        print("[command] " + json.dumps(args, ensure_ascii=False), flush=True)
    result = subprocess.run(args, cwd=cwd, env=env, text=True, encoding="utf-8",
                            errors="replace", stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if log is not None:
        Path(log).write_text(result.stdout, encoding="utf-8")
    if show or result.returncode:
        print(result.stdout, end="", flush=True)
    if result.returncode:
        raise Failure(f"Command exited {result.returncode}: {args[0]}. Operation stopped; inspect the output above.")
    return result.stdout


def host():
    arch = os.environ.get("PROCESSOR_ARCHITEW6432", platform.machine()).lower()
    if platform.system() == "Windows" and arch in ("arm64", "aarch64"):
        raise Failure("Windows ARM64 is blocked: native host tool packages and ST-Link USB driver support are not confirmed. Use a supported native host.")
    if platform.system() not in ("Windows", "Darwin", "Linux"):
        raise Failure("Supported hosts are Windows x64, macOS and Linux x86_64/aarch64.")
    if platform.system() == "Linux" and arch not in ("x86_64", "aarch64"):
        raise Failure("Linux requires an x86_64 or aarch64 host toolchain.")
    return f"{platform.system().lower()}-{arch}"


def load_config(required=True):
    if not CONFIG.exists():
        if required:
            raise Failure("No saved configuration. Run: just configure infantry_standard (or sentry_swerve). No robot is selected implicitly.")
        return {"tools": {}}
    return json.loads(CONFIG.read_text(encoding="utf-8"))


def candidates(name):
    """Common locations supplement PATH; explicit local paths have priority."""
    exe = NAMES[name] + (".exe" if os.name == "nt" else "")
    found = shutil.which(exe)
    if found:
        yield Path(found)
    bases = []
    if os.name == "nt":
        local = Path(os.environ.get("LOCALAPPDATA", ""))
        bases += [local / "Programs/firmware-tools",
                  local / "Microsoft/WinGet/Links"]
        bases += sorted((local / "Programs/just").glob("*"))
        patterns = {
            "gcc": ["Arm GNU Toolchain arm-none-eabi/*/bin", "Arm GNU Toolchain/*/bin"],
            "cmake": ["CMake/bin"], "git": ["Git/cmd"],
            "openocd": ["OpenOCD/bin", "openocd/bin", "xpack-openocd-*/bin"],
        }
        for base in (Path(os.environ.get("ProgramFiles", "C:/Program Files")),
                     Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")), local / "Programs"):
            for pattern in patterns.get(name, []):
                bases += sorted(base.glob(pattern), reverse=True)
        if name == "ninja":
            bases += sorted((local / "Microsoft/WinGet/Packages").glob("Ninja-build.Ninja_*"))
    else:
        bases += [Path("/opt/homebrew/bin"), Path("/usr/local/bin"), Path.home() / ".local/bin",
                  Path("/Applications/CMake.app/Contents/bin")]
        bases += sorted(Path("/Applications/ArmGNUToolchain").glob("*/arm-none-eabi/bin"))
        bases += sorted((Path.home() / ".local/opt").glob("*/bin"))
        bases += sorted((Path.home() / "Library/Python").glob("*/bin"))
    for base in bases:
        if (base / exe).is_file():
            yield base / exe


def discover(cfg, name, required=True):
    explicit = cfg.get("tools", {}).get(name)
    options = [Path(explicit).expanduser()] if explicit else list(candidates(name))
    for path in options:
        if path.is_file():
            return str(path.resolve())
    if required:
        raise Failure(f"Missing {name}. Run tools/bootstrap.py or set --tool {name}=FULL_EXECUTABLE_PATH with tools/firmware.py configure.")
    return None


def toolset(cfg, openocd_required=False):
    host()
    paths, versions = {}, {}
    print(f"[host] {platform.platform()} | Python {platform.python_version()} | {sys.executable}")
    for name in NAMES:
        path = discover(cfg, name, required=name != "openocd" or openocd_required)
        if path is None:
            print("[tool] openocd: missing; build remains available")
            continue
        version = run([path, "--version"])
        match = re.search(r"\d+\.\d+(?:\.\d+)?", version)
        value = match.group() if match else "unknown"
        if name == "gcc":
            value = run([path, "-dumpfullversion"]).strip()
        minimum = MIN_VERSIONS.get(name)
        if minimum and (not match or tuple(int(v) for v in value.split(".")) < minimum):
            raise Failure(f"{name} {value} is below the compatibility minimum {minimum}. Install the latest stable release.")
        # OpenOCD distributions can carry +dev/vendor builds with the same base version.
        observed = version.splitlines()[0] if name == "openocd" else value
        paths[name], versions[name] = path, observed
        print(f"[tool] {name} {observed}: {path}")
    if run([paths["gcc"], "-dumpmachine"]).strip() != "arm-none-eabi":
        raise Failure("Compiler target must be arm-none-eabi.")
    for suffix in ("g++", "objcopy", "objdump", "readelf", "nm", "size"):
        path = Path(paths["gcc"]).with_name("arm-none-eabi-" + suffix + (".exe" if os.name == "nt" else ""))
        if not path.is_file():
            raise Failure(f"Incomplete Arm toolchain: {path} is missing.")
        paths[suffix] = str(path)
    return paths, versions


def selected(cfg, override=None):
    robot = override or cfg.get("robot")
    if robot not in ROBOTS or cfg.get("mode") not in ("Debug", "Release"):
        raise Failure("Save an explicit robot and build type with just configure ROBOT Debug (or Release).")
    return robot


def flash_settings(cfg):
    f = cfg.get("flash", {})
    if f.get("backend", "openocd") != "openocd":
        raise Failure("Only the recommended OpenOCD backend is supported. Run configure to migrate the local settings.")
    if (f.get("interface") != "SWD" or type(f.get("run_after")) is not bool
            or type(f.get("allow_single")) is not bool):
        raise Failure("Missing explicit SWD/probe/run settings. Run just configure ROBOT Debug no.")
    sn = f.get("serial")
    if sn is not None and not re.fullmatch(r"[A-Za-z0-9]+", sn):
        raise Failure("Probe serial must contain only letters and digits.")
    if not sn and not f["allow_single"]:
        raise Failure("Set --serial SERIAL or explicitly allow a sole probe with --allow-single yes.")
    return f


def environment(paths):
    env = os.environ.copy()
    bins = list(dict.fromkeys(native_path(Path(p).parent) for p in paths.values()))
    env["PATH"] = os.pathsep.join(bins + [env.get("PATH", "")])
    # Project flags come from the frozen toolchain, not another active build shell.
    for key in ("CC", "CXX", "CFLAGS", "CXXFLAGS", "ASMFLAGS", "LDFLAGS"):
        env.pop(key, None)
    return env


def build_dir(cfg, robot, paths):
    identity = hashlib.sha256((native_path(ROOT) + native_path(paths["gcc"])).encode()).hexdigest()[:8]
    # Keep Ninja/GCC scratch and object paths below legacy Windows path limits.
    platform_tag = "w64" if os.name == "nt" else host()
    target = ("inf" if robot == "infantry_standard" else "sen") + "-" + cfg["mode"].lower()
    return ROOT / "build" / (platform_tag + "-" + identity) / target


def check_cache(directory, cfg, robot, paths, cmake_version):
    cache = directory / "CMakeCache.txt"
    if not cache.exists():
        return
    data = dict(re.findall(r"^([A-Za-z0-9_.-]+):[^=\n]*=(.*)$", cache.read_text(encoding="utf-8"), re.M))
    compiler_info = directory / "CMakeFiles" / cmake_version / "CMakeCCompiler.cmake"
    if compiler_info.exists():
        match = re.search(r'set\(CMAKE_C_COMPILER "([^"]+)"\)', compiler_info.read_text(encoding="utf-8"))
        if match:
            data["CMAKE_C_COMPILER"] = match[1]
    for key, expected in (("ROBOT_TYPE", robot), ("CMAKE_BUILD_TYPE", cfg["mode"]), ("CMAKE_GENERATOR", "Ninja")):
        if data.get(key) != expected:
            raise Failure(f"Cache mismatch {key}: {data.get(key)!r} != {expected!r}. Preserve this directory and choose a fresh checkout/build location.")
    for key, expected in (("CMAKE_HOME_DIRECTORY", ROOT), ("CMAKE_C_COMPILER", paths["gcc"])):
        actual = Path(data.get(key, ""))
        if not actual.exists() or not actual.samefile(expected):
            raise Failure(f"Cache mismatch {key}. Existing build directory belongs to another source/toolchain; no files were deleted.")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_elf(elf, paths):
    """Validate linked ELF32 headers and allocated address ranges, including stack reservation."""
    data = elf.read_bytes()
    if data[:7] != b"\x7fELF\x01\x01\x01":
        raise Failure("Expected little-endian ELF32.")
    h = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
    typ, machine, _, entry, phoff, shoff, flags, _, phsize, phnum, shsize, shnum, stridx = h
    if typ != 2 or machine != 40 or flags & 0xFF000000 != 0x05000000 or not flags & 0x400:
        raise Failure("Expected executable ARM EABI5 hard-float ELF.")
    if not entry & 1 or not 0x08000000 <= (entry & ~1) < 0x08100000:
        raise Failure("Reset entry must be Thumb code in the 1 MiB flash region.")
    sections = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shsize) for i in range(shnum)]
    strings = sections[stridx]
    names = data[strings[4]:strings[4] + strings[5]]
    regions = {"FLASH": (0x08000000, 1024 * 1024), "RAM": (0x20000000, 128 * 1024), "CCMRAM": (0x10000000, 64 * 1024)}
    used = dict.fromkeys(regions, 0)
    vector = None
    for section in sections:
        n, kind, sf, address, offset, size, *_ = section
        name = names[n:].split(b"\0", 1)[0].decode()
        if name == ".isr_vector":
            vector = (address, data[offset:offset + size])
        if not sf & 2 or not size:
            continue
        region = next((r for r, (start, cap) in regions.items() if start <= address and address + size <= start + cap), None)
        if region is None:
            raise Failure(f"Allocated section {name} has unexpected range {address:#x}+{size:#x}.")
        used[region] = max(used[region], address + size - regions[region][0])
    for i in range(phnum):
        ptype, offset, va, pa, size, mem, pf, align = struct.unpack_from("<IIIIIIII", data, phoff + i * phsize)
        if ptype == 1 and size:
            if not 0x08000000 <= pa or pa + size > 0x08100000:
                raise Failure("Load segment lies outside internal flash; refuse programming.")
            used["FLASH"] = max(used["FLASH"], pa + size - 0x08000000)
    if not vector or vector[0] != 0x08000000 or len(vector[1]) < 64:
        raise Failure("Missing vector table at 0x08000000.")
    vectors = struct.unpack_from("<16I", vector[1])
    if vectors[0] != 0x20020000 or vectors[1] != entry:
        raise Failure("Initial MSP/reset vector does not match the frozen linker and entry.")
    undefined = run([paths["nm"], "-u", elf]).strip()
    if undefined:
        raise Failure("Unresolved symbols:\n" + undefined)
    attributes = run([paths["readelf"], "-h", "-A", "-l", elf])
    for required in ("v7E-M", "VFPv4-D16", "VFP registers"):
        if required not in attributes:
            raise Failure(f"Missing expected ARM attribute: {required}.")
    symbols = run([paths["nm"], "--defined-only", elf])
    syms = {m[2]: (int(m[0], 16), m[1]) for m in re.findall(r"^([0-9a-fA-F]+)\s+(\w)\s+(\S+)$", symbols, re.M)}
    for index, name in ((11, "SVC_Handler"), (14, "PendSV_Handler"), (15, "SysTick_Handler")):
        if name not in syms or syms[name][1].upper() != "T" or vectors[index] != (syms[name][0] | 1):
            raise Failure(f"Vector {name} is not bound to its strong handler.")
    rtos = {name: name in syms for name in ("RobotRtos_Start", "vTaskStartScheduler", "xPortPendSVHandler", "vPortSVCHandler", "xPortSysTickHandler")}
    print("[ELF] ARM EABI5 hard-float; Thumb reset", hex(entry), "; no unresolved symbols")
    for region, size in used.items():
        print(f"[memory] {region}: {size}/{regions[region][1]} bytes ({size / regions[region][1]:.2%})")
    print("[RTOS] linked symbols:", rtos, "(linking does not validate runtime behavior)")
    elf.with_suffix(".inspection.txt").write_text(attributes + "\n" + symbols, encoding="utf-8")
    return {"entry": hex(entry), "initial_msp": hex(vectors[0]), "memory_bytes": used, "rtos_symbols": rtos}


def build(cfg, robot, paths, versions):
    directory = build_dir(cfg, robot, paths)
    print(f"[build] {robot} {cfg['mode']} -> {directory}", flush=True)
    check_cache(directory, cfg, robot, paths, versions["cmake"])
    env = environment(paths)
    run([paths["cmake"], "-S", ROOT, "-B", directory, "-G", "Ninja",
         "-DCMAKE_TOOLCHAIN_FILE=" + Path(native_path(ROOT / "cmake/gcc-arm-none-eabi.cmake")).as_posix(),
         "-DCMAKE_MAKE_PROGRAM=" + Path(native_path(paths["ninja"])).as_posix(),
         "-DROBOT_TYPE=" + robot, "-DCMAKE_BUILD_TYPE=" + cfg["mode"]], env=env, show=True)
    run([paths["cmake"], "--build", directory, "--parallel"], env=env, show=True)
    check_cache(directory, cfg, robot, paths, versions["cmake"])
    elf, mapfile = directory / "NYUSH_Infantry.elf", directory / "NYUSH_Infantry.map"
    if not elf.is_file() or not mapfile.is_file():
        raise Failure("Successful build did not produce both ELF and map; refusing stale fallback.")
    inspection = inspect_elf(elf, paths)
    for fmt, suffix in (("binary", ".bin"), ("ihex", ".hex")):
        run([paths["objcopy"], "-O", fmt, elf, elf.with_suffix(suffix)])
    record = {"robot": robot, "build_type": cfg["mode"], "host": host(),
              "utc": datetime.now(timezone.utc).isoformat(), "tools": versions,
              "tool_sources": cfg.get("tool_sources", {}),
              "compiler_path": paths["gcc"], "toolchain_banner": run([paths["gcc"], "--version"]).splitlines()[0],
              "git_commit": run([paths["git"], "rev-parse", "HEAD"]).strip(),
              "git_dirty": bool(run([paths["git"], "status", "--porcelain"]).strip()),
              "elf": str(elf), "map": str(mapfile), "sha256": sha256(elf), **inspection}
    (directory / "firmware-manifest.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(f"[build OK] {robot}\nELF: {elf}\nSHA-256: {record['sha256']}", flush=True)
    return record


def enumerate_probes():
    # OS descriptors avoid connecting to or resetting a target just to select a probe.
    try:
        serials = stlink_usb.probe_serials()
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        stlink_usb.report()
        raise Failure(f"ST-Link USB discovery failed: {exc}") from exc
    print("[USB ST-Link serials] " + json.dumps(serials))
    if not serials:
        stlink_usb.report()
    return serials


def choose_probe(serials, settings):
    if not serials:
        raise Failure("No usable ST-Link serial enumerated. Check USB diagnostics and run just doctor after reconnecting. No target connection or write performed.")
    serial = settings.get("serial")
    if serial:
        if serial not in serials:
            raise Failure("Configured probe serial is not present; no other probe will be selected.")
        return serial
    if len(serials) != 1 or not settings["allow_single"]:
        raise Failure("Multiple probes or no single-probe permission. Save --serial SERIAL; never selecting the first probe.")
    return serials[0]


def tcl_word(value):
    """Quote one Tcl argument; shell argument arrays alone cannot prevent Tcl substitution."""
    value = str(value)
    for old, new in (("\\", "\\\\"), ('"', '\\"'), ("$", "\\$"), ("[", "\\["),
                     ("]", "\\]"), ("\n", "\\n"), ("\r", "\\r")):
        value = value.replace(old, new)
    return '"' + value + '"'


def openocd_base(paths):
    """Load the reviewed ST-Link/F407 configuration without starting a debug session."""
    return [paths.get("openocd", "openocd"),
            "-c", "noinit; gdb_port disabled; tcl_port disabled; telnet_port disabled",
            "-f", ROOT / "tools/openocd/stm32f407-stlink.cfg",
            "-f", ROOT / "tools/openocd/flash_checked.tcl"]


def check_openocd(paths):
    """Parse installed interface/target scripts, then exit without init or USB access."""
    output = run(openocd_base(paths) + ["-c", "fw_select_serial CONFIGCHECK; echo FIRMWARE_CONFIG_OK; shutdown"], show=True)
    if not re.search(r"^FIRMWARE_CONFIG_OK\s*$", output, re.M):
        raise Failure("OpenOCD configuration did not complete; check its matching scripts and ST-Link driver support.")


def programming_command(paths, serial, elf, run_after):
    """One OpenOCD session checks identity before reset and again before erasing."""
    return openocd_base(paths) + [
        "-c", "fw_select_serial " + tcl_word(serial),
        "-c", "set FW_ELF " + tcl_word(Path(native_path(elf)).as_posix()),
        "-c", "set FW_RUN_AFTER " + ("1" if run_after else "0"),
        "-c", "fw_program"]


def plan(cfg, robot, paths):
    f = flash_settings(cfg)
    elf = build_dir(cfg, robot, paths) / "NYUSH_Infantry.elf"
    serial = f.get("serial") or "<sole-connected-probe>"
    command = programming_command(paths, serial, elf, f["run_after"])
    print(json.dumps({"plan_only": True, "backend": "openocd", "robot": robot, "mode": cfg["mode"], "elf": str(elf),
        "sha256": "computed only after successful build; existing files are not authorized by this plan",
        "steps": ["check config/tools", "build and inspect ELF/cache", "read USB serial descriptors", "choose exact serial",
                  "OpenOCD init", "check device ID/flash size", "reset halt", "recheck device ID/flash size",
                  "flash write_image erase", "verify_image", "reset run" if f["run_after"] else "remain halted"],
        "write_verify": [str(a) for a in command], "log": str(elf.parent / "openocd-flash.log"),
        "before_download": "reset halt (software reset with reset_config none)",
        "after_verified": "reset run" if f["run_after"] else "remain halted; do not reset/run",
        "target": "device ID 0x413 and 1024 KiB; shared F405/407 family ID, confirm physical F407 marking",
        "hardware_commands_executed": 0}, indent=2, ensure_ascii=False))


def flash(cfg, robot, paths, versions):
    f = flash_settings(cfg)
    record = build(cfg, robot, paths, versions)
    serial = choose_probe(enumerate_probes(), f)
    elf = Path(record["elf"])
    if sha256(elf) != record["sha256"]:
        raise Failure("ELF changed after build; refusing to write.")
    logfile = elf.parent / "openocd-flash.log"
    print("[before connection] " + json.dumps({**record, "backend": "openocd", "probe": serial,
          "target_check": "OpenOCD checks device ID 0x413 and flash 1024 KiB before reset/erase",
          "before_download": "reset halt", "run_after": f["run_after"],
          "programmer_log": str(logfile)}, indent=2), flush=True)
    try:
        output = run(programming_command(paths, serial, elf, f["run_after"]), show=True, log=logfile)
    except (Failure, OSError) as exc:
        raise Failure(f"Programming command failed. Flash may be erased or partially programmed; do not run this image. "
                      f"No automatic retry or post-failure reset/run requested. Log (if created): {logfile}. {exc}") from exc
    if not all(re.search(r"^" + marker + r"\s*$", output, re.M) for marker in ("FIRMWARE_VERIFY_OK", "FIRMWARE_FLASH_OK")):
        raise Failure("OpenOCD did not positively report verified completion. Inspect openocd-flash.log; no automatic retry requested.")
    state = "MCU reset and running." if f["run_after"] else "MCU remains halted."
    print(f"[flash OK] Download and verification succeeded. {state} Robot behavior has NOT been validated.")


@contextlib.contextmanager
def operation_lock():
    """Exclude simultaneous builds/flash operations, without deleting user directories."""
    lock = ROOT / ".firmware.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        raise Failure("Another operation owns .firmware.lock. If a previous process crashed, confirm it stopped before manually removing this lock.") from None
    try:
        os.write(fd, str(os.getpid()).encode())
        os.close(fd)
        yield
    finally:
        lock.unlink()


def configure(args):
    cfg = load_config(False)
    cfg.update({"robot": args.robot, "mode": args.mode,
                "flash": {"backend": "openocd", "interface": "SWD", "serial": args.serial,
                          "allow_single": args.allow_single == "yes", "run_after": args.run_after == "yes"}})
    for item in args.tool:
        key, sep, value = item.partition("=")
        if not sep or key not in NAMES:
            raise Failure("Use --tool NAME=FULL_EXECUTABLE_PATH; names: " + ", ".join(NAMES))
        cfg.setdefault("tools", {})[key] = str(Path(value).expanduser().resolve())
    paths, versions = toolset(cfg)
    cfg["tools"] = {k: v for k, v in paths.items() if k in NAMES}
    cfg["python"] = sys.executable
    flash_settings(cfg)
    # Merge settings instead of replacing user/editor preferences. JSONC needs manual merge.
    settings_file = ROOT / ".vscode/settings.json"
    settings = json.loads(settings_file.read_text(encoding="utf-8")) if settings_file.exists() else {}
    # Python must precede /usr/bin (often contributed by Git) to preserve the venv.
    directories = list(dict.fromkeys([str(Path(sys.executable).parent)] + [str(Path(p).parent) for p in paths.values()]))
    launcher = shutil.which("py") if os.name == "nt" else None
    if launcher:
        directories.append(str(Path(launcher).parent))
    settings["firmware.justPath"] = paths["just"]
    settings["firmware.toolPath"] = os.pathsep.join(dict.fromkeys(directories))
    terminal_os = {"Windows": "windows", "Darwin": "osx", "Linux": "linux"}[platform.system()]
    key = "terminal.integrated.env." + terminal_os
    settings.setdefault(key, {})["PATH"] = os.pathsep.join(dict.fromkeys(directories)) + os.pathsep + "${env:PATH}"
    if os.name == "nt":
        settings.setdefault("terminal.integrated.defaultProfile.windows", "PowerShell")
    settings["cmake.configureOnOpen"] = False
    settings["makefile.configureOnOpen"] = False
    settings["C_Cpp.default.compilerPath"] = paths["gcc"]
    settings_file.parent.mkdir(exist_ok=True)
    settings_file.write_text(json.dumps(settings, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    CONFIG.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[configured] {args.robot} {args.mode}; SWD; run_after={cfg['flash']['run_after']}. Local paths saved, Git ignored. Open a NEW VS Code terminal.")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["--just"]:
        argv = shlex.split(argv[1])
    if argv[:1] in (["logger"], ["logger-cli"]):
        mode = argv[0]
        script = ROOT / "scripts" / "dashboard" / ("rtt_ws_bridge.py" if mode == "logger" else "rtt_dashboard.py")
        defaults = (["--dashboard-channel", "1", "--log-channel", "0", "--ws-host", "127.0.0.1", "--ws-port", "8765", "--http-host", "127.0.0.1", "--http-port", "8080"] if mode == "logger" else ["--mode", "dashboard", "--channel", "1", "--log-channel", "0", "--connect", "normal", "--poll-ms", "20", "--refresh-ms", "100"])
        return subprocess.call([sys.executable, str(script), *defaults, *argv[1:]], cwd=ROOT)
    if argv[:1] == ["monitor"]:
        import gimbal_monitor
        try:
            gimbal_monitor.main(argv[1:])
        except KeyboardInterrupt:
            print("\n监控已停止，CSV已保存。")
        except gimbal_monitor.fw.Failure as exc:
            # Direct execution loads this file as __main__; normalize exception ownership.
            raise Failure(str(exc)) from exc
        except subprocess.SubprocessError as exc:
            raise Failure(f"Monitor subprocess failed: {exc}") from exc
        return
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("help")
    sub.add_parser("monitor", help="Read yaw/pitch snapshots; use monitor --help for options")
    p = sub.add_parser("configure")
    p.add_argument("robot", choices=ROBOTS)
    p.add_argument("--mode", choices=("Debug", "Release"), default="Debug")
    p.add_argument("--run-after", choices=("yes", "no"), default="yes",
                   help="Reset and run after successful verification (default: yes); no leaves the MCU halted.")
    p.add_argument("--allow-single", choices=("yes", "no"), default="yes")
    p.add_argument("--serial")
    p.add_argument("--tool", action="append", default=[])
    sub.add_parser("doctor")
    for name in ("build", "flash", "flash-plan"):
        sub.add_parser(name).add_argument("robot", nargs="?", choices=ROBOTS)
    args = parser.parse_args(argv)
    if args.command == "help":
        parser.print_help()
        return
    if args.command == "configure":
        with operation_lock():
            configure(args)
        return
    cfg = load_config(required=args.command != "doctor")
    paths, versions = toolset(cfg, openocd_required=args.command == "flash")
    if args.command == "doctor":
        print("[doctor] BUILD ENVIRONMENT READY (hardware not required)")
        try:
            robot = selected(cfg)
            f = flash_settings(cfg)
            print(f"[config] {robot} {cfg['mode']}; SWD; run_after={f['run_after']}")
            if "openocd" not in paths:
                raise Failure("OpenOCD is missing. Install it as described in docs/quickstart.md.")
            check_openocd(paths)
            serial = choose_probe(enumerate_probes(), f)
            print(f"[doctor] OPENOCD CONFIG READY; USB probe available: {serial}. USB permissions, SWD wiring, target identity and actual programming remain unverified; no target connection made.")
        except Failure as exc:
            print(f"[doctor] FLASH NOT READY: {exc}")
        return
    robot = selected(cfg, args.robot)
    if args.command == "flash-plan":
        with operation_lock(), windows_paths(paths):
            plan(cfg, robot, paths)
    else:
        with operation_lock(), windows_paths(paths):
            (flash if args.command == "flash" else build)(cfg, robot, paths, versions)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    try:
        main()
    except (Failure, OSError, ValueError, KeyError) as exc:
        print(f"[ERROR] {exc}\nSee docs/quickstart.md for recovery.", file=sys.stderr)
        sys.exit(1)
