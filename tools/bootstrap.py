"""Install missing pinned command-line tools in a user-owned directory.
Python/Git, VS Code and ST's interactive installer are covered in quickstart.md.
"""
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import sys
import tarfile
import urllib.request
import zipfile

import firmware as fw


def fetch(url):
    request = urllib.request.Request(url, headers={"User-Agent": "firmware-bootstrap/1.0"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def github_asset(repo, tag, name):
    release = json.loads(fetch(f"https://api.github.com/repos/{repo}/releases/tags/{tag}"))
    asset = next(a for a in release["assets"] if a["name"] == name)
    digest = asset.get("digest", "") or ""
    if digest.startswith("sha256:"):
        return asset["browser_download_url"], digest[7:]
    # Older GitHub assets may predate the digest API. Require published checksum files.
    checksum = next((a for a in release["assets"] if a["name"] in ("SHA256SUMS", f"cmake-{tag.lstrip('v')}-SHA-256.txt")), None)
    if not checksum:
        raise fw.Failure(f"No official checksum for {name}; install manually after verifying its origin.")
    text = fetch(checksum["browser_download_url"]).decode()
    match = re.search(r"([a-fA-F0-9]{64})\s+\*?" + re.escape(name) + r"(?:\s|$)", text)
    if not match:
        raise fw.Failure("Checksum file has no entry for " + name)
    return asset["browser_download_url"], match[1].lower()


def sources():
    # Linux builds reuse explicitly configured tools; no Linux installer is provided.
    if platform.system() == "Linux":
        raise fw.Failure("Linux bootstrap is not implemented. Configure existing Linux ARM64 tools using docs/quickstart.md; no macOS packages will be downloaded.")
    win = os.name == "nt"
    intel = platform.machine() == "x86_64"
    arch = "x86_64" if intel or win else "aarch64"
    just_host = "x86_64-pc-windows-msvc.zip" if win else arch + "-apple-darwin.tar.gz"
    cmake_host = "windows-x86_64.zip" if win else "macos-universal.tar.gz"
    yield "just", "1.46.0", lambda: github_asset("casey/just", "1.46.0", "just-1.46.0-" + just_host)
    yield "cmake", "4.2.3", lambda: github_asset("Kitware/CMake", "v4.2.3", "cmake-4.2.3-" + cmake_host)
    yield "ninja", "1.13.1", lambda: github_asset("ninja-build/ninja", "v1.13.1", "ninja-win.zip" if win else "ninja-mac.zip")
    arm_version = "14.2.rel1" if not win and intel else "14.3.rel1"
    arm_host = "mingw-w64-x86_64" if win else "darwin-" + ("x86_64" if intel else "arm64")
    filename = f"arm-gnu-toolchain-{arm_version}-{arm_host}-arm-none-eabi." + ("zip" if win else "tar.xz")
    url = f"https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/{arm_version}/{filename}"

    def arm_source():
        checksum = fetch(url + ".sha256asc").decode()
        match = re.search(r"\b([a-fA-F0-9]{64})\b", checksum)
        if not match:
            raise fw.Failure("Arm official SHA256 unavailable; no unchecked installation performed.")
        return url, match[1].lower()
    yield "gcc", "14.2.1" if arm_version == "14.2.rel1" else "14.3.1", arm_source


def version(path, name):
    output = fw.run([path, "-dumpfullversion" if name == "gcc" else "--version"])
    match = re.search(r"\d+\.\d+\.\d+", output)
    return match[0] if match else None


def main():
    fw.host()
    if sys.version_info < (3, 12):
        raise fw.Failure("Install Python 3.13.7 first; bootstrap requires Python 3.12+ for safe tar extraction.")
    cfg = fw.load_config(False)
    base = (Path(os.environ["LOCALAPPDATA"]) / "Programs/FirmwareTools") if os.name == "nt" else Path.home() / ".local/opt/rm-firmware"
    base.mkdir(parents=True, exist_ok=True)
    for name, pin, source in sources():
        explicit = cfg.get("tools", {}).get(name)
        exe = fw.NAMES[name] + (".exe" if os.name == "nt" else "")
        folder = base / (name + "-" + pin)
        options = ([Path(explicit)] if explicit else []) + list(fw.candidates(name)) + list(folder.rglob(exe))
        usable = None
        for option in options:
            try:
                if version(option, name) == pin:
                    usable = option
                    break
            except (OSError, fw.Failure):
                pass
        if usable:
            print(f"[reuse] {name} {pin}: {usable}")
        else:
            if folder.exists():
                raise fw.Failure(f"Incomplete/unrecognized installation at {folder}. Inspect it and move it aside manually before retrying.")
            url, digest = source()
            archive = base / url.rsplit("/", 1)[1]
            if not archive.exists() or fw.sha256(archive) != digest:
                print(f"[download] {url}", flush=True)
                archive.write_bytes(fetch(url))
            if fw.sha256(archive) != digest:
                raise fw.Failure(f"SHA256 mismatch for {archive}; not extracted.")
            print(f"[verified SHA256] {digest}", flush=True)
            folder.mkdir()
            if archive.suffix == ".zip":
                with zipfile.ZipFile(archive) as z:
                    for member in z.infolist():
                        if not (folder / member.filename).resolve().is_relative_to(folder.resolve()):
                            raise fw.Failure("Unsafe archive member.")
                    z.extractall(folder)
                if os.name != "nt":
                    for p in folder.rglob("*"):
                        if p.is_file() and p.name in ("just", "ninja"):
                            p.chmod(0o755)
            else:
                with tarfile.open(archive) as t:
                    t.extractall(folder, filter="data")
            usable = next(folder.rglob(exe), None)
            if usable is None or version(usable, name) != pin:
                raise fw.Failure(f"Installed tool failed its version check: {folder}")
        cfg.setdefault("tools", {})[name] = str(usable.resolve())
        fw.CONFIG.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    print("[ready] Tool paths saved locally. Install CubeProgrammer 2.21.0 if missing, then configure your robot with tools/firmware.py configure ROBOT.")
    print("[just directory] " + str(Path(cfg["tools"]["just"]).parent))
    print("No machine PATH was changed. Configure generates VS Code terminal/task paths; use the full just path until then.")


if __name__ == "__main__":
    try:
        with fw.operation_lock():
            main()
    except (fw.Failure, OSError, ValueError, StopIteration) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
