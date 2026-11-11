import json
import math
import random
from collections import Counter
from dataclasses import dataclass, field

import frames

SCHEMA_VERSION = 3
DEFAULT_MIN_RANGE_MM = 200
DEFAULT_MAX_RANGE_MM = 100000


class ScenarioError(ValueError):
    pass


def _finite(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ScenarioError(f"{name} must be a finite number")
    return float(value)


def _points(value, name, angle=False):
    if not isinstance(value, list) or len(value) < 2:
        raise ScenarioError(f"{name} must be a list of at least two [t, value] pairs")

    points = []
    previous_t = None
    for index, item in enumerate(value):
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ScenarioError(f"{name}[{index}] must be a [t, value] pair")
        t = _finite(item[0], f"{name}[{index}].t")
        v = _finite(item[1], f"{name}[{index}].value")
        if previous_t is not None and t <= previous_t:
            raise ScenarioError(f"{name} times must be strictly increasing")
        if angle and not -90.0 < v < 90.0:
            raise ScenarioError(f"{name}[{index}].value must be within (-90, 90) degrees")
        previous_t = t
        points.append((t, v))

    return points


def _rate(value, name):
    rate = _finite(value, name)
    if not 0.0 <= rate <= 1.0:
        raise ScenarioError(f"{name} must be within [0, 1]")
    return rate


class Scenario:
    def __init__(self, data):
        if not isinstance(data, dict):
            raise ScenarioError("scenario root must be a JSON object")

        self.duration_s = _finite(data.get("duration_s"), "duration_s")
        self.frame_rate_hz = _finite(data.get("frame_rate_hz", 100.0), "frame_rate_hz")
        self.initial_height_m = _finite(data.get("initial_height_m"), "initial_height_m")
        self.initial_velocity_mps = _finite(
            data.get("initial_velocity_mps", 0.0), "initial_velocity_mps"
        )
        self.install_angle_deg = _finite(
            data.get("install_angle_deg", 10.0), "install_angle_deg"
        )

        if self.duration_s <= 0.0:
            raise ScenarioError("duration_s must be positive")
        if self.frame_rate_hz <= 0.0:
            raise ScenarioError("frame_rate_hz must be positive")
        if self.initial_height_m <= 0.0:
            raise ScenarioError("initial_height_m must be positive")
        if not 0.0 <= self.install_angle_deg < 90.0:
            raise ScenarioError("install_angle_deg must be within [0, 90)")

        motion = data.get("motion")
        if not isinstance(motion, dict):
            raise ScenarioError("motion must be an object")
        self.motion_type = motion.get("type", "velocity")
        if self.motion_type not in ("velocity", "acceleration"):
            raise ScenarioError("motion.type must be 'velocity' or 'acceleration'")
        self.motion_points = _points(motion.get("points"), "motion.points")

        attitude = data.get("attitude")
        if attitude is None:
            self.attitude_points = [(0.0, 0.0), (self.duration_s, 0.0)]
        else:
            if not isinstance(attitude, dict):
                raise ScenarioError("attitude must be an object")
            self.attitude_points = _points(
                attitude.get("points"), "attitude.points", angle=True
            )

        limits = data.get("limits", {})
        if not isinstance(limits, dict):
            raise ScenarioError("limits must be an object")
        self.min_range_mm = int(
            _finite(limits.get("min_range_mm", DEFAULT_MIN_RANGE_MM), "limits.min_range_mm")
        )
        self.max_range_mm = int(
            _finite(limits.get("max_range_mm", DEFAULT_MAX_RANGE_MM), "limits.max_range_mm")
        )
        self.out_of_range = limits.get("out_of_range", "fail")
        if self.out_of_range not in ("fail", "clamp"):
            raise ScenarioError("limits.out_of_range must be 'fail' or 'clamp'")
        if self.min_range_mm < 0 or self.min_range_mm >= self.max_range_mm:
            raise ScenarioError("limits require 0 <= min_range_mm < max_range_mm")

        range_error = data.get("range_error", {})
        if not isinstance(range_error, dict):
            raise ScenarioError("range_error must be an object")
        self.noise_mm = _finite(range_error.get("noise_mm", 0.0), "range_error.noise_mm")
        self.bias_mm = _finite(range_error.get("bias_mm", 0.0), "range_error.bias_mm")
        self.scale_ppm = _finite(range_error.get("scale_ppm", 0.0), "range_error.scale_ppm")
        if self.noise_mm < 0.0:
            raise ScenarioError("range_error.noise_mm must not be negative")
        if self.scale_ppm <= -1000000.0:
            raise ScenarioError("range_error.scale_ppm must be greater than -1000000")

        injection = data.get("fault_injection", {})
        if not isinstance(injection, dict):
            raise ScenarioError("fault_injection must be an object")
        self.fail_rate = _rate(injection.get("fail_rate", 0.0), "fault_injection.fail_rate")
        self.corrupt_rate = _rate(
            injection.get("corrupt_rate", 0.0), "fault_injection.corrupt_rate"
        )
        self.drop_rate = _rate(injection.get("drop_rate", 0.0), "fault_injection.drop_rate")
        self.freeze_frames = int(
            _finite(injection.get("freeze_frames", 0), "fault_injection.freeze_frames")
        )
        self.freeze_every_frames = int(
            _finite(injection.get("freeze_every_frames", 0), "fault_injection.freeze_every_frames")
        )
        if self.freeze_frames < 0 or self.freeze_every_frames < 0:
            raise ScenarioError("fault_injection freeze values must not be negative")


def load(path):
    with open(path, "r", encoding="utf-8") as handle:
        try:
            data = json.load(handle)
        except json.JSONDecodeError as error:
            raise ScenarioError(f"{path}: {error}") from error
    return Scenario(data)


def _pchip_slopes(xs, ys):
    count = len(xs)
    if count == 2:
        slope = (ys[1] - ys[0]) / (xs[1] - xs[0])
        return [slope, slope]

    h = [xs[i + 1] - xs[i] for i in range(count - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(count - 1)]
    m = [0.0] * count
    m[0] = d[0]
    m[-1] = d[-1]

    for i in range(1, count - 1):
        if d[i - 1] * d[i] <= 0.0:
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])

    return m


def _pchip_eval(xs, ys, ms, t):
    if t <= xs[0]:
        return ys[0]
    if t >= xs[-1]:
        return ys[-1]

    lo = 0
    hi = len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= t:
            lo = mid
        else:
            hi = mid

    h = xs[lo + 1] - xs[lo]
    s = (t - xs[lo]) / h
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return h00 * ys[lo] + h10 * h * ms[lo] + h01 * ys[lo + 1] + h11 * h * ms[lo + 1]


def _interpolator(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ms = _pchip_slopes(xs, ys)
    return lambda t: _pchip_eval(xs, ys, ms, t)


def _round_mm(value):
    return int(math.floor(value + 0.5))


def _clamp(value, lower, upper):
    return min(max(value, lower), upper)


@dataclass(slots=True)
class TruthSample:
    t_ms: int
    height_mm: float
    descent_rate_mps: float
    install_angle_deg: float
    attitude_deg: float
    alpha_deg: float
    slant_range_mm: float


@dataclass(slots=True)
class IdealSample:
    height_mm: float


@dataclass(slots=True)
class RealSample:
    valid: bool
    range_mm: int | None
    height_mm: float | None
    sta: int | None
    frame: bytes | None
    checksum_ok: bool
    events: list[str] = field(default_factory=list)


@dataclass(slots=True)
class Sample:
    truth: TruthSample
    ideal: IdealSample
    real: RealSample


@dataclass(slots=True)
class Generation:
    samples: list[Sample]
    stream: bytes
    stats: dict


def generate(scenario, seed=0):
    count = int(scenario.duration_s * scenario.frame_rate_hz + 1e-9)
    if count < 1:
        raise ScenarioError("duration_s and frame_rate_hz produce no frames")

    dt = 1.0 / scenario.frame_rate_hz
    times = [i * dt for i in range(count)]
    rng = random.Random(seed)

    motion = _interpolator(scenario.motion_points)
    attitude = _interpolator(scenario.attitude_points)

    if scenario.motion_type == "velocity":
        speeds = [motion(t) for t in times]
    else:
        accels = [motion(t) for t in times]
        speeds = [scenario.initial_velocity_mps]
        for i in range(1, count):
            speeds.append(speeds[-1] + 0.5 * (accels[i - 1] + accels[i]) * dt)

    heights = [scenario.initial_height_m]
    for i in range(1, count):
        heights.append(heights[-1] - 0.5 * (speeds[i - 1] + speeds[i]) * dt)
        if heights[-1] < 0.0:
            raise ScenarioError(
                f"height reaches zero at t={times[i]:.3f}s; shorten duration_s or lower the speed table"
            )

    attitudes = [attitude(t) for t in times]
    angles = [scenario.install_angle_deg + value for value in attitudes]
    if any(abs(angle) >= 90.0 for angle in angles):
        raise ScenarioError("absolute install_angle_deg plus attitude must stay below 90 degrees")

    cos_beta = math.cos(math.radians(scenario.install_angle_deg))
    truth_samples = []
    ideal_samples = []
    negative_rate = 0
    previous_slant = None
    height_min = math.inf
    height_max = -math.inf
    angle_max = -math.inf
    attitude_max = -math.inf

    for i in range(count):
        height_mm = heights[i] * 1000.0
        angle_deg = angles[i]
        slant_range_mm = height_mm / math.cos(math.radians(angle_deg))
        if previous_slant is not None and slant_range_mm > previous_slant:
            negative_rate += 1
        previous_slant = slant_range_mm
        height_min = min(height_min, height_mm)
        height_max = max(height_max, height_mm)
        angle_max = max(angle_max, angle_deg)
        attitude_max = max(attitude_max, attitudes[i])
        truth = TruthSample(
            t_ms=_round_mm(times[i] * 1000.0),
            height_mm=height_mm,
            descent_rate_mps=-speeds[i],
            install_angle_deg=scenario.install_angle_deg,
            attitude_deg=attitudes[i],
            alpha_deg=angle_deg,
            slant_range_mm=slant_range_mm,
        )
        truth_samples.append(truth)
        ideal_samples.append(IdealSample(height_mm=slant_range_mm * cos_beta))

    freeze_starts = set()
    if scenario.freeze_frames > 0:
        if scenario.freeze_every_frames > 0:
            freeze_starts = set(
                range(scenario.freeze_every_frames, count, scenario.freeze_every_frames)
            )
        else:
            freeze_starts = {count // 2}

    samples = []
    stream = bytearray()
    last_sensor_range = None
    freeze_until = -1
    freeze_value = None
    emitted_count = 0
    accepted_count = 0
    real_valid_count = 0
    event_counts = Counter()
    range_mm_min = math.inf
    range_mm_max = -math.inf
    ideal_error_mm_max_abs = 0.0
    real_error_mm_max_abs = None

    for i, truth in enumerate(truth_samples):
        events = []
        encoded_range = None
        sta = frames.STA_OK

        if truth.slant_range_mm < scenario.min_range_mm or truth.slant_range_mm > scenario.max_range_mm:
            events.append("out_of_range")
            if scenario.out_of_range == "fail":
                sta = 0
            else:
                encoded_range = _clamp(
                    _round_mm(truth.slant_range_mm), scenario.min_range_mm, scenario.max_range_mm
                )
                events.append("clamped")
        else:
            raw_range = truth.slant_range_mm * (1.0 + scenario.scale_ppm / 1000000.0)
            raw_range += scenario.bias_mm
            if scenario.noise_mm > 0.0:
                raw_range += rng.gauss(0.0, scenario.noise_mm)
            rounded_range = _round_mm(raw_range)
            encoded_range = _clamp(rounded_range, scenario.min_range_mm, scenario.max_range_mm)
            if encoded_range != rounded_range:
                events.append("clamped")

        if encoded_range is not None:
            last_sensor_range = encoded_range

        if i in freeze_starts and last_sensor_range is not None:
            freeze_value = last_sensor_range
            freeze_until = i + scenario.freeze_frames

        if i < freeze_until and freeze_value is not None:
            encoded_range = freeze_value
            sta = frames.STA_OK
            events.append("freeze")

        dropped = rng.random() < scenario.drop_rate
        failed = rng.random() < scenario.fail_rate
        corrupted = rng.random() < scenario.corrupt_rate
        if failed:
            events.append("fail")

        frame = None
        checksum_ok = False
        decoded_range = None
        decoded_sta = None
        real_valid = False

        if dropped:
            events.append("drop")
        else:
            if failed or encoded_range is None:
                sta = 0
                encoded_distance = 0
            else:
                sta = frames.STA_OK
                encoded_distance = encoded_range

            frame_bytes = bytearray(
                frames.build_reply(frames.FUNC_STREAM, sta, encoded_distance)
            )
            checksum_ok = True
            if corrupted:
                events.append("corrupt")
                frame_bytes[-1] ^= 0xFF
                checksum_ok = False
            frame = bytes(frame_bytes)
            decoded_sta = sta
            if checksum_ok and sta == frames.STA_OK and encoded_range is not None:
                decoded_range = encoded_range
                real_valid = True

        real_height_mm = decoded_range * cos_beta if real_valid else None
        real = RealSample(
            valid=real_valid,
            range_mm=decoded_range,
            height_mm=real_height_mm,
            sta=decoded_sta,
            frame=frame,
            checksum_ok=checksum_ok,
            events=events,
        )
        sample = Sample(truth=truth, ideal=ideal_samples[i], real=real)
        samples.append(sample)
        if frame is not None:
            stream.extend(frame)
            emitted_count += 1
            if checksum_ok:
                accepted_count += 1
        if real_valid:
            real_valid_count += 1
        event_counts.update(events)
        if decoded_range is not None:
            range_mm_min = min(range_mm_min, decoded_range)
            range_mm_max = max(range_mm_max, decoded_range)
        ideal_error = abs(sample.ideal.height_mm - truth.height_mm)
        ideal_error_mm_max_abs = max(ideal_error_mm_max_abs, ideal_error)
        if real_height_mm is not None:
            real_error = abs(real_height_mm - truth.height_mm)
            real_error_mm_max_abs = (
                real_error
                if real_error_mm_max_abs is None
                else max(real_error_mm_max_abs, real_error)
            )

    stats = {
        "frames": count,
        "emitted": emitted_count,
        "accepted": accepted_count,
        "real_valid": real_valid_count,
        "out_of_range": event_counts["out_of_range"],
        "clamped": event_counts["clamped"],
        "fail": event_counts["fail"],
        "corrupt": event_counts["corrupt"],
        "drop": event_counts["drop"],
        "freeze": event_counts["freeze"],
        "negative_rate_frames": negative_rate,
        "range_mm_min": None if range_mm_min == math.inf else range_mm_min,
        "range_mm_max": None if range_mm_max == -math.inf else range_mm_max,
        "angle_deg_max": angle_max,
        "attitude_deg_max": attitude_max,
        "install_angle_deg": scenario.install_angle_deg,
        "height_mm_min": height_min,
        "height_mm_max": height_max,
        "ideal_error_mm_max_abs": ideal_error_mm_max_abs,
        "real_error_mm_max_abs": real_error_mm_max_abs,
        "bias_mm": scenario.bias_mm,
        "scale_ppm": scenario.scale_ppm,
        "noise_mm": scenario.noise_mm,
    }

    return Generation(samples=samples, stream=bytes(stream), stats=stats)