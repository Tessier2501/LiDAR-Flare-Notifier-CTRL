#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
from pathlib import Path

if os.name == "nt":
    _lib_bin = os.path.join(sys.prefix, "Library", "bin")
    if os.path.isdir(_lib_bin):
        os.environ["PATH"] = _lib_bin + os.pathsep + os.environ.get("PATH", "")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _load(path):
    records = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    if records and records[0].get("schema_version") != 3:
        raise ValueError("unsupported JSONL schema; expected schema_version=3")
    return records


def _series(records, key):
    return [float(value) / 1000.0 if value is not None else math.nan for value in (record[key] for record in records)]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="ptgl_plot",
        description="Plot truth, beta-only ideal, and realistic sensor height from a ptgl_sim JSONL log.",
    )
    parser.add_argument("--jsonl", required=True, help="three-series JSONL written by ptgl_sim --jsonl")
    parser.add_argument("--out", required=True, help="output image path")
    parser.add_argument("--title", help="optional figure title")
    parser.add_argument("--dpi", type=int, default=150, help="output DPI (default: 150)")
    parser.add_argument("--last-height", type=float, help="show only data at or below this true height in metres")
    args = parser.parse_args(argv)

    jsonl_path = Path(args.jsonl)
    if not jsonl_path.is_file():
        print(f"error: missing JSONL log: {jsonl_path}", file=sys.stderr)
        return 2

    out_path = Path(args.out)
    if not out_path.parent.is_dir():
        print(f"error: output directory does not exist: {out_path.parent}", file=sys.stderr)
        return 2

    try:
        records = _load(jsonl_path)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if not records:
        print("error: JSONL log has no frames", file=sys.stderr)
        return 2
    if args.last_height is not None and args.last_height <= 0.0:
        print("error: --last-height must be positive", file=sys.stderr)
        return 2

    start = 0
    if args.last_height is not None:
        start = next(
            (index for index, record in enumerate(records) if record["h_truth_mm"] <= args.last_height * 1000.0),
            len(records),
        )
        if start == len(records):
            print("error: no records below --last-height", file=sys.stderr)
            return 2

    selected = records[start:]
    start_ms = selected[0]["t_ms"]
    if args.last_height is None:
        times = [record["t_ms"] / 1000.0 for record in selected]
        x_label = "Time t (s)"
    else:
        times = [(record["t_ms"] - start_ms) / 1000.0 for record in selected]
        x_label = f"Time from {args.last_height:g} m true height (s)"

    h_truth = _series(selected, "h_truth_mm")
    h_ideal = _series(selected, "h_ideal_mm")
    h_real = _series(selected, "h_real_mm")
    ideal_error = [
        (ideal - truth) * 1000.0 if math.isfinite(ideal) and math.isfinite(truth) else math.nan
        for ideal, truth in zip(h_ideal, h_truth)
    ]
    real_error = [
        (real - truth) * 1000.0 if math.isfinite(real) and math.isfinite(truth) else math.nan
        for real, truth in zip(h_real, h_truth)
    ]
    real_valid = sum(1 for record in selected if record["real_valid"])

    figure, (height_axes, error_axes) = plt.subplots(
        2,
        1,
        figsize=(14.0, 9.0),
        sharex=True,
        gridspec_kw={"height_ratios": (2.5, 1.0)},
    )
    height_axes.plot(times, h_truth, color="black", linewidth=1.8, label="truth height")
    height_axes.plot(times, h_ideal, color="tab:blue", linewidth=1.4, label="ideal sensor: beta calibration only")
    height_axes.plot(times, h_real, color="tab:red", linewidth=1.0, label="realistic sensor")
    height_axes.set_ylim(bottom=0.0)
    height_axes.set_ylabel("Height (m)")
    height_axes.grid(True, alpha=0.3)
    height_axes.legend()
    if args.title:
        height_axes.set_title(args.title)
    else:
        height_axes.set_title("PTGL-100 truth / ideal / realistic height")

    error_axes.axhline(0.0, color="0.5", linewidth=0.9)
    error_axes.plot(times, ideal_error, color="tab:blue", linewidth=1.3, label="ideal - truth")
    error_axes.plot(times, real_error, color="tab:red", linewidth=1.0, label="real - truth")
    error_axes.set_xlabel(x_label)
    error_axes.set_ylabel("Error (mm)")
    error_axes.grid(True, alpha=0.3)
    error_axes.legend()

    figure.text(
        0.99,
        0.01,
        f"source: {jsonl_path.name} / schema=3",
        ha="right",
        va="bottom",
        fontsize=8,
        color="0.45",
    )
    figure.tight_layout(rect=(0.0, 0.03, 1.0, 1.0))
    figure.savefig(out_path, dpi=args.dpi)
    plt.close(figure)

    print(f"points={len(selected)} start_ms={start_ms} end_ms={selected[-1]['t_ms']}")
    print(f"real_valid={real_valid} real_invalid={len(selected) - real_valid}")
    print(f"out={out_path} bytes={out_path.stat().st_size}")
    return 0


if __name__ == "__main__":
    sys.exit(main())