#!/usr/bin/env python3

import argparse
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

TOOL = "flight_core_selftest"
SCRIPT_DIR = Path(__file__).resolve().parent


def wsl_path(path: Path) -> str:
    resolved = path.resolve()
    if not resolved.drive:
        raise SystemExit(f"not a Windows path: {resolved}")
    return f"/mnt/{resolved.drive[0].lower()}{resolved.as_posix()[2:]}"


def generate_replay(build_dir: Path) -> Path:
    sim_dir = SCRIPT_DIR.parent / "ptgl_sim"
    sim_run = sim_dir / "run.py"
    sample = sim_dir / "samples" / "descent.json"
    replay = build_dir / "replay.bin"
    for path in (sim_run, sample):
        if not path.is_file():
            raise SystemExit(f"missing simulator path: {path}")

    result = subprocess.run(
        [
            sys.executable,
            str(sim_run),
            "--scenario",
            str(sample),
            "--out",
            str(replay),
            "--seed",
            "1",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    print(f"stage=simulate exit={result.returncode}")
    for stream in (result.stdout, result.stderr):
        if stream:
            print(stream, end="" if stream.endswith("\n") else "\n")
    if result.returncode != 0:
        raise SystemExit("simulator generation failed")
    return replay


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog=TOOL,
        description="Build and run portable flight core tests with gcc inside WSL.",
    )
    parser.add_argument("--distro", default="Ubuntu", help="WSL distribution name (default: Ubuntu)")
    args = parser.parse_args(argv)

    if shutil.which("wsl") is None:
        raise SystemExit("wsl not found; this tool needs Windows with WSL installed")

    firmware = SCRIPT_DIR.parents[2] / "LiDAR-Flare-Notifier-CTRL"
    sensor = firmware / "App" / "sensor"
    flight = firmware / "App" / "flight"
    build_dir = SCRIPT_DIR.parents[1] / "temp" / TOOL
    build_dir.mkdir(parents=True, exist_ok=True)

    replay = generate_replay(build_dir)
    sources = [
        sensor / "ptgl_parser.c",
        flight / "flight_core.c",
        flight / "flight_math.c",
        flight / "flight_params.c",
        SCRIPT_DIR / "test_flight.c",
    ]
    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        raise SystemExit("missing sources: " + ", ".join(missing))

    includes = [firmware / "App", sensor, flight]
    include_flags = " ".join(f"-I{shlex.quote(wsl_path(path))}" for path in includes)
    quoted_sources = " ".join(shlex.quote(wsl_path(path)) for path in sources)
    quoted_replay = shlex.quote(wsl_path(replay))

    status = 0
    for option in ("-O0", "-O2"):
        binary = build_dir / f"test_flight{option[1:]}"
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
                quoted_replay,
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
