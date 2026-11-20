"""Resolve latest published tool releases and install verified host packages locally.
Existing versions stay installed; normal builds do not auto-upgrade or use the network.
"""
import json
import os
from pathlib import Path
import platform
import re
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


def latest_tag(repo):
    release = json.loads(fetch(f"https://api.github.com/repos/{repo}/releases/latest"))
    if release.get("draft") or release.get("prerelease"):
        raise fw.Failure("Refusing a draft/prerelease from " + repo)
    return release["tag_name"]


def latest_arm_release():
    # Arm publishes release branches, not Git tags. Ignore beta/architecture variants.
    branches = json.loads(fetch("https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/repository/branches?per_page=100"))
    releases = [b["name"].split("/", 1)[1].lower() for b in branches
                if re.fullmatch(r"releases/\d+\.\d+\.rel\d+", b["name"], re.I)]
    if not releases:
        raise fw.Failure("Arm release list is unavailable; no guessed download selected.")
    return max(releases, key=lambda v: tuple(map(int, re.findall(r"\d+", v))))


def sources():
    win = os.name == "nt"
    linux = platform.system() == "Linux"
    intel = platform.machine() == "x86_64"
    arch = "x86_64" if intel or win else "aarch64"
    just_host = "x86_64-pc-windows-msvc.zip" if win else arch + "-apple-darwin.tar.gz"
    cmake_host = "windows-x86_64.zip" if win else "macos-universal.tar.gz"
    ninja_host = "ninja-win.zip" if win else "ninja-mac.zip"
    if linux:
        # Host architecture selects executables; the firmware target stays arm-none-eabi.
        just_host = arch + "-unknown-linux-musl.tar.gz"
        cmake_host = "linux-" + arch + ".tar.gz"
        ninja_host = "ninja-linux.zip" if intel else "ninja-linux-aarch64.zip"
    for name, repo, suffix in (("just", "casey/just", just_host), ("cmake", "Kitware/CMake", cmake_host),
                               ("ninja", "ninja-build/ninja", ninja_host)):
        tag = latest_tag(repo)
        release = tag.lstrip("v")
        filename = suffix if name == "ninja" else f"{name}-{release}-{suffix}"
        yield name, release, lambda r=repo, t=tag, f=filename: github_asset(r, t, f)
    arm_version = latest_arm_release()
    arm_host = "mingw-w64-x86_64" if win else "darwin-" + ("x86_64" if intel else "arm64")
    if linux:
        arm_host = arch
    filename = f"arm-gnu-toolchain-{arm_version}-{arm_host}-arm-none-eabi." + ("zip" if win else "tar.xz")
    url = f"https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/{arm_version}/{filename}"

    def arm_source():
        checksum = fetch(url + ".sha256asc").decode()
        match = re.search(r"\b([a-fA-F0-9]{64})\b", checksum)
        if not match:
            raise fw.Failure("Arm official SHA256 unavailable; no unchecked installation performed.")
        return url, match[1].lower()
    yield "gcc", arm_version, arm_source
    # xPack supplies maintained native binaries, including Linux ARM64. Its released
    # packages may include upstream +dev changes; record the exact distribution release.
    tag = latest_tag("xpack-dev-tools/openocd-xpack")
    release = tag.lstrip("v")
    openocd_host = "win32-x64" if win else ("linux-" if linux else "darwin-") + ("x64" if intel else "arm64")
    filename = f"xpack-openocd-{release}-{openocd_host}." + ("zip" if win else "tar.gz")
    yield "openocd", release, lambda: github_asset("xpack-dev-tools/openocd-xpack", tag, filename)


def matches_release(name, release, actual):
    if actual is None:
        return False
    if name == "gcc":
        return actual.split(".")[:2] == release.split(".")[:2]
    return actual == release.split("-", 1)[0]


def version(path, name):
    output = fw.run([path, "-dumpfullversion" if name == "gcc" else "--version"])
    match = re.search(r"\d+\.\d+\.\d+", output)
    return match[0] if match else None


def main():
    fw.host()
    if sys.version_info < (3, 12):
        raise fw.Failure("Install current stable Python first; bootstrap requires Python 3.12+ for safe tar extraction.")
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
                if name == "openocd" and not option.resolve().is_relative_to(folder.resolve()):
                    continue
                if matches_release(name, pin, version(option, name)):
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
            # CMake archives also contain doc/cmake directories; only run files.
            usable = next((p for p in folder.rglob(exe) if p.is_file() and os.access(p, os.X_OK)), None)
            if usable is None or not matches_release(name, pin, version(usable, name)):
                raise fw.Failure(f"Installed tool failed its version check: {folder}")
            cfg.setdefault("tool_sources", {})[name] = {"release": pin, "url": url, "sha256": digest}
        cfg.setdefault("tools", {})[name] = str(usable.resolve())
        cfg.setdefault("tool_versions", {})[name] = version(usable, name)
        fw.CONFIG.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    print("[ready] Build tool paths saved locally. Configure your robot with tools/firmware.py configure ROBOT.")
    print("OpenOCD is the default flash tool. Run doctor to check its scripts/USB descriptors without connecting to the MCU.")
    print("[just directory] " + str(Path(cfg["tools"]["just"]).parent))
    print("No machine PATH was changed. Configure generates VS Code terminal/task paths; use the full just path until then.")


if __name__ == "__main__":
    try:
        with fw.operation_lock():
            main()
    except (fw.Failure, OSError, ValueError, StopIteration) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
