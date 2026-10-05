"""Start/stop the URSim CB3 3.15.8 Docker container (UR's official simulator) for tests/test_ur_ursim.py.

    py.exe tests/ursim.py start      # run the container, wait for boot, power on + brake release
    py.exe tests/ursim.py status
    py.exe tests/ursim.py stop       # stop (the container runs with --rm, so it is removed; the image stays)

Container: `docker run --rm -d --pull never --name ursim_mauer -e ROBOT_MODEL=UR5 -p 127.0.0.1:29999:29999
-p 127.0.0.1:30001-30004:30001-30004 universalrobots/ursim_cb3:3.15.8` (research 2026-10-05). The image must already
be on disk (3.31 GB) - this helper never pulls. Docker: `docker` on PATH, else the Docker Desktop CLI.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from mauer.ur.dashboard import Dashboard, DashboardError  # noqa: E402

IMAGE = "universalrobots/ursim_cb3:3.15.8"
NAME = "ursim_mauer"
HOST = "127.0.0.1"
PORTS = ("127.0.0.1:29999:29999", "127.0.0.1:30001-30004:30001-30004")
DOCKER_CANDIDATES = (
    r"C:\Users\samue\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe",   # py.exe (Windows)
    "/mnt/c/Users/samue/AppData/Local/Programs/DockerDesktop/resources/bin/docker.exe",  # WSL
)


def docker_exe() -> str | None:
    """Docker CLI: PATH first, then the Docker Desktop install location."""
    for c in (shutil.which("docker"), *DOCKER_CANDIDATES):
        if c and os.path.isfile(c):
            return c
    return None


def _docker(*args: str, timeout: float = 60.0) -> subprocess.CompletedProcess:
    exe = docker_exe()
    if exe is None:
        raise FileNotFoundError("docker CLI not found")
    return subprocess.run([exe, *args], capture_output=True, text=True, timeout=timeout)


def available() -> tuple[bool, str]:
    """(True, '') if the docker CLI, a running daemon and the URSim image are there, else (False, reason)."""
    if docker_exe() is None:
        return False, "docker CLI not found"
    try:
        r = _docker("version", "--format", "{{.Server.Version}}", timeout=30)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, f"docker not usable: {e}"
    if r.returncode != 0:
        return False, f"docker daemon not running: {r.stderr.strip()[:200]}"
    r = _docker("image", "inspect", "--format", "{{.Id}}", IMAGE, timeout=30)
    if r.returncode != 0:
        return False, f"image {IMAGE} not on disk (not pulled automatically)"
    return True, ""


def running() -> bool:
    r = _docker("ps", "--filter", f"name=^{NAME}$", "--format", "{{.Names}}", timeout=30)
    return r.returncode == 0 and NAME in r.stdout.split()


def start(boot_timeout_s: float = 240.0, ready: bool = True) -> bool:
    """Run the container (reuse it if it already runs), wait until the Dashboard answers, then power on and release
    the brakes (ready=True). Returns True if this call started the container."""
    started = False
    if not running():
        args = ["run", "--rm", "-d", "--pull", "never", "--name", NAME, "-e", "ROBOT_MODEL=UR5"]
        for p in PORTS:
            args += ["-p", p]
        r = _docker(*args, IMAGE, timeout=120)
        if r.returncode != 0:
            raise RuntimeError(f"docker run failed: {r.stderr.strip()[:400]}")
        started = True
        print(f"URSim container {NAME} started ({r.stdout.strip()[:12]})", flush=True)
    t0 = time.time()
    with Dashboard(HOST, wait_s=boot_timeout_s) as d:
        # the server accepts connections before the controller is up: wait for a sensible robotmode
        while True:
            try:
                mode = d.robotmode()
                if mode not in ("NO_CONTROLLER", "DISCONNECTED", "BOOTING"):
                    break
            except DashboardError:
                pass
            if time.time() - t0 > boot_timeout_s:
                raise TimeoutError(f"URSim did not boot within {boot_timeout_s:.0f} s")
            time.sleep(1.0)
        print(f"URSim up after {time.time() - t0:.0f} s: {d.polyscope_version()}, robotmode {mode}", flush=True)
        if ready:
            d.wait_ready(timeout_s=max(30.0, boot_timeout_s - (time.time() - t0)))
            print(f"URSim ready: robotmode {d.robotmode()}, safetymode {d.safetymode()}", flush=True)
    return started


def stop() -> None:
    """Stop the container (removed by --rm); the image stays on disk."""
    if running():
        r = _docker("stop", "-t", "5", NAME, timeout=60)
        print(f"URSim container {NAME} stopped" if r.returncode == 0 else f"docker stop failed: {r.stderr}",
              flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["start", "stop", "status"])
    ap.add_argument("--no-ready", action="store_true", help="do not power on / release the brakes")
    a = ap.parse_args()
    ok, why = available()
    if not ok:
        sys.exit(f"URSim not available: {why}")
    if a.cmd == "start":
        start(ready=not a.no_ready)
    elif a.cmd == "stop":
        stop()
    else:
        print(f"{NAME}: {'running' if running() else 'not running'} (image {IMAGE})", flush=True)


if __name__ == "__main__":
    main()
