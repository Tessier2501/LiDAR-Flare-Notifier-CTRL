#!/usr/bin/env python3

import argparse
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

TOOL = "app_port_selftest"
SCRIPT_DIR = Path(__file__).resolve().parent


def wsl_path(path: Path) -> str:
    resolved = path.resolve()
    if not resolved.drive:
        raise SystemExit(f"not a Windows path: {resolved}")
    return f"/mnt/{resolved.drive[0].lower()}{resolved.as_posix()[2:]}"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog=TOOL,
        description="Build the portable App layer host tests with gcc inside WSL and run them.",
    )
    parser.add_argument("--distro", default="Ubuntu", help="WSL distribution name (default: Ubuntu)")
    args = parser.parse_args(argv)

    if shutil.which("wsl") is None:
        raise SystemExit("wsl not found; this tool needs Windows with WSL installed")

    firmware = SCRIPT_DIR.parents[2] / "LiDAR-Flare-Notifier-CTRL"
    app = firmware / "App"
    sensor = app / "sensor"
    flight = app / "flight"
    build_dir = SCRIPT_DIR.parents[1] / "temp" / TOOL
    sources = [
        app / "app.c",
        flight / "flight_core.c",
        flight / "flight_math.c",
        flight / "flight_params.c",
        sensor / "ptgl_parser.c",
        sensor / "ptgl_cmd.c",
        sensor / "ptgl_sensor.c",
        app / "storage" / "param_store.c",
        SCRIPT_DIR / "fake_ports.c",
        SCRIPT_DIR / "fake_param_flash.c",
        SCRIPT_DIR / "test_param.c",
        SCRIPT_DIR / "test_sensor.c",
    ]
    includes = [app, sensor, flight]

    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        raise SystemExit("missing sources: " + ", ".join(missing))

    build_dir.mkdir(parents=True, exist_ok=True)
    include_flags = " ".join(f"-I{shlex.quote(wsl_path(path))}" for path in includes)
    quoted_sources = " ".join(shlex.quote(wsl_path(path)) for path in sources)

    status = 0
    for option in ("-O0", "-O2"):
        binary = build_dir / f"test_sensor{option[1:]}"
        command = " ".join(
            [
                "gcc -std=c11 -Wall -Wextra -Werror",
                option,
                include_flags,
                quoted_sources,
                "-o",
                shlex.quote(wsl_path(binary)),
                "&&",
                shlex.quote(wsl_path(binary)),
            ]
        )
        result = subprocess.run(
            ["wsl", "-d", args.distro, "--", "bash", "-lc", command],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        print(f"stage=gcc{option} exit={result.returncode}")
        for stream in (result.stdout, result.stderr):
            if stream:
                print(stream, end="" if stream.endswith("\n") else "\n")
        if result.returncode != 0:
            status = result.returncode

    print(f"build_dir={build_dir}")
    return status


if __name__ == "__main__":
    sys.exit(main())
