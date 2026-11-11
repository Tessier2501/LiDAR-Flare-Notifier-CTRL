#!/usr/bin/env python3

import argparse
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

TOOL = "ptgl_parser_selftest"
SCRIPT_DIR = Path(__file__).resolve().parent


def wsl_path(path: Path) -> str:
    resolved = path.resolve()
    if not resolved.drive:
        raise SystemExit(f"not a Windows path: {resolved}")
    return f"/mnt/{resolved.drive[0].lower()}{resolved.as_posix()[2:]}"


def generate_replay(build_dir: Path):
    sim_dir = SCRIPT_DIR.parent / "ptgl_sim"
    sim_run = sim_dir / "run.py"
    sample = sim_dir / "samples" / "descent.json"
    for path in (sim_run, sample):
        if not path.is_file():
            raise SystemExit(f"missing simulator path: {path}")

    data_path = build_dir / "replay.bin"
    expect_path = build_dir / "replay.txt"
    result = subprocess.run(
        [
            sys.executable,
            str(sim_run),
            "--scenario",
            str(sample),
            "--out",
            str(data_path),
            "--expect",
            str(expect_path),
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
    return data_path, expect_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog=TOOL,
        description="Build the PTGL-100 parser host tests with gcc inside WSL and run them.",
    )
    parser.add_argument("--distro", default="Ubuntu", help="WSL distribution name (default: Ubuntu)")
    args = parser.parse_args(argv)

    if shutil.which("wsl") is None:
        raise SystemExit("wsl not found; this tool needs Windows with WSL installed")

    firmware = SCRIPT_DIR.parents[2] / "LiDAR-Flare-Notifier-CTRL"
    sensor = firmware / "App" / "sensor"
    build_dir = SCRIPT_DIR.parents[1] / "temp" / TOOL
    sources = [sensor / "ptgl_parser.c", sensor / "ptgl_cmd.c", SCRIPT_DIR / "test_parser.c"]

    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        raise SystemExit("missing sources: " + ", ".join(missing))

    build_dir.mkdir(parents=True, exist_ok=True)
    data_path, expect_path = generate_replay(build_dir)
    include = shlex.quote(wsl_path(sensor))
    quoted_sources = " ".join(shlex.quote(wsl_path(path)) for path in sources)
    quoted_args = " ".join(shlex.quote(wsl_path(path)) for path in (data_path, expect_path))

    status = 0
    for option in ("-O0", "-O2"):
        binary = build_dir / f"test_parser{option[1:]}"
        command = " ".join(
            [
                "gcc -std=c11 -Wall -Wextra -Werror",
                option,
                f"-I{include}",
                quoted_sources,
                "-o",
                shlex.quote(wsl_path(binary)),
                "&&",
                shlex.quote(wsl_path(binary)),
                quoted_args,
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
