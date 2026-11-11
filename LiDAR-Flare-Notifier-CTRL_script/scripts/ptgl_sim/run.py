#!/usr/bin/env python3

import argparse
import hashlib
import json
import sys
from pathlib import Path

import frames
import scenario


def _as_hex(stream):
    lines = []
    for offset in range(0, len(stream), frames.FRAME_LEN):
        frame = stream[offset : offset + frames.FRAME_LEN]
        lines.append(" ".join(f"{byte:02X}" for byte in frame))
    return "\n".join(lines)


def _rounded(value, digits=3):
    return None if value is None else round(value, digits)


def _write_jsonl(path, samples):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        for sample in samples:
            truth = sample.truth
            ideal = sample.ideal
            real = sample.real
            record = {
                "schema_version": scenario.SCHEMA_VERSION,
                "t_ms": truth.t_ms,
                "h_truth_mm": _rounded(truth.height_mm),
                "h_ideal_mm": _rounded(ideal.height_mm),
                "h_real_mm": _rounded(real.height_mm),
                "d_true_mm": _rounded(truth.slant_range_mm),
                "d_meas_mm": real.range_mm,
                "beta_deg": _rounded(truth.install_angle_deg),
                "attitude_deg": _rounded(truth.attitude_deg),
                "alpha_deg": _rounded(truth.alpha_deg),
                "descent_rate_mps": _rounded(truth.descent_rate_mps, 4),
                "real_valid": real.valid,
                "frame_emitted": real.frame is not None,
                "frame_checksum_ok": real.checksum_ok,
                "sta": real.sta,
                "events": real.events,
            }
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def _expect_lines(samples):
    lines = []
    for sample in samples:
        real = sample.real
        if real.frame is None or not real.checksum_ok:
            continue
        distance = real.range_mm if real.range_mm is not None else 0
        lines.append(f"{distance} {real.sta}")
    return lines


def _write_expect(path, samples):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        for line in _expect_lines(samples):
            handle.write(line + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="ptgl_sim",
        description="Generate PTGL-100 frame streams and truth/ideal/real height series.",
    )
    parser.add_argument("--scenario", required=True, help="scenario JSON path")
    parser.add_argument("--out", required=True, help="output frame-stream path")
    parser.add_argument("--format", choices=("bin", "hex"), default="bin", help="output format (default: bin)")
    parser.add_argument("--jsonl", help="optional three-series JSONL output path")
    parser.add_argument("--expect", help="optional expected parser-accepted frame text path (distance_mm sta per line)")
    parser.add_argument("--seed", type=int, default=0, help="noise and injection seed (default: 0)")
    args = parser.parse_args(argv)

    try:
        config = scenario.load(args.scenario)
        result = scenario.generate(config, seed=args.seed)
    except (OSError, scenario.ScenarioError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    out_path = Path(args.out)
    if not out_path.parent.is_dir():
        print(f"error: output directory does not exist: {out_path.parent}", file=sys.stderr)
        return 2

    optional_paths = []
    if args.expect:
        optional_paths.append(("expected", Path(args.expect)))
    if args.jsonl:
        optional_paths.append(("JSONL", Path(args.jsonl)))
    for label, optional_path in optional_paths:
        if not optional_path.parent.is_dir():
            print(f"error: {label} directory does not exist: {optional_path.parent}", file=sys.stderr)
            return 2

    if args.format == "bin":
        out_path.write_bytes(result.stream)
        written = f"{len(result.stream)} bytes"
    else:
        out_path.write_text(_as_hex(result.stream) + "\n", encoding="utf-8", newline="\n")
        written = f"{result.stats['emitted']} lines"

    if args.expect:
        _write_expect(args.expect, result.samples)
    if args.jsonl:
        _write_jsonl(args.jsonl, result.samples)

    stats = result.stats
    digest = hashlib.sha256(result.stream).hexdigest()[:8]
    print(f"frames={stats['frames']} emitted={stats['emitted']} accepted={stats['accepted']} real_valid={stats['real_valid']}")
    print(f"duration_s={config.duration_s:.3f} fps={config.frame_rate_hz:.3f}")
    print(f"height_mm_min={stats['height_mm_min']:.3f} height_mm_max={stats['height_mm_max']:.3f}")
    print(
        f"range_mm_min={stats['range_mm_min']} range_mm_max={stats['range_mm_max']}"
        f" angle_deg_max={stats['angle_deg_max']:.3f}"
        f" attitude_deg_max={stats['attitude_deg_max']:.3f}"
        f" install_angle_deg={stats['install_angle_deg']:.3f}"
    )
    print(
        f"ideal_error_mm_max_abs={stats['ideal_error_mm_max_abs']:.3f}"
        f" real_error_mm_max_abs={stats['real_error_mm_max_abs']}"
    )
    print(
        "injected"
        f" out_of_range={stats['out_of_range']}"
        f" clamped={stats['clamped']}"
        f" fail={stats['fail']}"
        f" corrupt={stats['corrupt']}"
        f" drop={stats['drop']}"
        f" freeze={stats['freeze']}"
    )
    print(
        f"range_error noise_mm={stats['noise_mm']:.3f}"
        f" bias_mm={stats['bias_mm']:.3f}"
        f" scale_ppm={stats['scale_ppm']:.3f}"
    )
    print(f"negative_rate_frames={stats['negative_rate_frames']}")
    print(f"out={out_path} format={args.format} {written} sha256={digest}")
    if args.expect:
        print(f"expect={args.expect}")
    if args.jsonl:
        print(f"jsonl={args.jsonl}")
    return 0


if __name__ == "__main__":
    sys.exit(main())