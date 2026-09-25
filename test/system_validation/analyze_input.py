#!/usr/bin/env python3
"""Read-only scientific integrity audit for FAST-LIVO2 input rosbag2 data.

The analyzer is descriptive: it reports observed values and structural
violations.  It does not assign sensor-health PASS/FAIL thresholds.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import yaml


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def stamp_ns(msg: Any) -> int | None:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def frame_id(msg: Any) -> str:
    return str(getattr(getattr(msg, "header", None), "frame_id", ""))


def finite(values: Iterable[Any]) -> bool:
    try:
        return bool(np.all(np.isfinite(np.asarray(list(values), dtype=float))))
    except (TypeError, ValueError):
        return False


def robust_summary(values: Sequence[float]) -> dict[str, Any]:
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if not data.size:
        return {"count": 0}
    median = float(np.median(data))
    mad = float(np.median(np.abs(data - median)))
    return {
        "count": int(data.size),
        "min": float(np.min(data)),
        "q05": float(np.quantile(data, 0.05)),
        "median": median,
        "q95": float(np.quantile(data, 0.95)),
        "max": float(np.max(data)),
        "mean": float(np.mean(data)),
        "std": float(np.std(data)),
        "mad": mad,
    }


def timing_metrics(rows: Sequence[dict[str, Any]], stamp_key: str) -> dict[str, Any]:
    stamps = np.asarray([row[stamp_key] for row in rows if row.get(stamp_key) is not None], dtype=np.int64)
    if not stamps.size:
        return {"available": False, "count": 0}
    delta_s = np.diff(stamps).astype(float) / 1e9
    positive = delta_s[delta_s > 0]
    median = float(np.median(positive)) if positive.size else math.nan
    mad = float(np.median(np.abs(positive - median))) if positive.size else math.nan
    # A diagnostic candidate rule, not a physical sensor requirement.
    gap_rule_s = median + 10.0 * max(mad, np.finfo(float).eps) if positive.size else math.nan
    endpoint_duration_s = float((stamps[-1] - stamps[0]) / 1e9)
    span_s = float((np.max(stamps) - np.min(stamps)) / 1e9)
    strictly_monotonic = bool(np.all(delta_s > 0))
    if stamps.size < 2:
        rate_interpretation = "not_available_insufficient_samples"
    elif not strictly_monotonic:
        rate_interpretation = "not_available_nonmonotonic_or_duplicate_endpoint_bias"
    elif endpoint_duration_s <= 0:
        rate_interpretation = "not_available_nonpositive_endpoint_duration"
    else:
        rate_interpretation = "valid_strictly_monotonic_endpoint_rate"
    return {
        "available": True,
        "count": int(stamps.size),
        "first_ns": int(stamps[0]),
        "last_ns": int(stamps[-1]),
        "endpoint_duration_s": endpoint_duration_s,
        "span_s": span_s,
        "strictly_monotonic": strictly_monotonic,
        "duplicate_count": int(np.sum(delta_s == 0)),
        "regression_count": int(np.sum(delta_s < 0)),
        "delta_s": robust_summary(delta_s.tolist()),
        "positive_delta_s": robust_summary(positive.tolist()),
        "rate_from_endpoints_hz": (
            float((stamps.size - 1) / endpoint_duration_s)
            if strictly_monotonic and endpoint_duration_s > 0 else None
        ),
        "rate_interpretation": rate_interpretation,
        "jitter_mad_s": mad,
        "gap_candidate_rule": "positive_delta > median + 10*MAD",
        "gap_candidate_rule_s": gap_rule_s,
        "gap_candidate_count": int(np.sum(positive > gap_rule_s)) if positive.size else 0,
    }


def topic_metrics(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    bag = timing_metrics(rows, "bag_stamp_ns")
    header = timing_metrics(rows, "header_stamp_ns")
    offsets = [
        (int(row["header_stamp_ns"]) - int(row["bag_stamp_ns"])) / 1e9
        for row in rows if row.get("header_stamp_ns") is not None
    ]
    frames = sorted({str(row.get("frame_id", "")) for row in rows if row.get("frame_id", "")})
    violations = []
    if bag.get("duplicate_count", 0):
        violations.append({"kind": "duplicate_bag_timestamp", "count": bag["duplicate_count"]})
    if bag.get("regression_count", 0):
        violations.append({"kind": "nonmonotonic_bag_timestamp", "count": bag["regression_count"]})
    if header.get("duplicate_count", 0):
        violations.append({"kind": "duplicate_header_timestamp", "count": header["duplicate_count"]})
    if header.get("regression_count", 0):
        violations.append({"kind": "nonmonotonic_header_timestamp", "count": header["regression_count"]})
    if rows and len(offsets) != len(rows):
        violations.append({"kind": "missing_header_timestamp", "count": len(rows) - len(offsets)})
    return {
        "message_count": len(rows),
        "bag_timing": bag,
        "header_timing": header,
        "header_minus_bag_s": robust_summary(offsets),
        "header_coverage_fraction": float(len(offsets) / len(rows)) if rows else 0.0,
        "frames": frames,
        "structural_violations": violations,
    }


def one_to_one_nearest(
    reference_ns: Sequence[int], target_ns: Sequence[int], max_delta_ns: int | None = None
) -> list[tuple[int, int, int]]:
    """Greedy chronological one-to-one nearest pairing; delta=target-reference."""
    reference = np.asarray(reference_ns, dtype=np.int64)
    target = np.asarray(target_ns, dtype=np.int64)
    pairs: list[tuple[int, int, int]] = []
    start = 0
    for ref_index, stamp in enumerate(reference):
        if start >= target.size:
            break
        pos = int(np.searchsorted(target[start:], stamp)) + start
        choices = [idx for idx in (pos - 1, pos) if start <= idx < target.size]
        if not choices:
            continue
        selected = min(choices, key=lambda idx: (abs(int(target[idx]) - int(stamp)), idx))
        delta = int(target[selected]) - int(stamp)
        if max_delta_ns is None or abs(delta) <= max_delta_ns:
            pairs.append((ref_index, selected, delta))
            start = selected + 1
    return pairs


def synchronization_metrics(
    reference: Sequence[dict[str, Any]], target: Sequence[dict[str, Any]],
    reference_name: str, target_name: str, max_delta_s: float | None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    ref = sorted(((idx, row.get("header_stamp_ns") or row["bag_stamp_ns"])
                  for idx, row in enumerate(reference)), key=lambda item: item[1])
    tgt = sorted(((idx, row.get("header_stamp_ns") or row["bag_stamp_ns"])
                  for idx, row in enumerate(target)), key=lambda item: item[1])
    pairs = one_to_one_nearest(
        [stamp for _, stamp in ref], [stamp for _, stamp in tgt],
        None if max_delta_s is None else int(max_delta_s * 1e9),
    )
    rows = [{
        "reference_index": ref[i][0], "target_index": tgt[j][0],
        "reference_stamp_ns": int(ref[i][1]), "target_stamp_ns": int(tgt[j][1]),
        "delta_s": delta / 1e9,
    } for i, j, delta in pairs]
    delta = np.asarray([row["delta_s"] for row in rows], dtype=float)
    elapsed = np.asarray([(row["reference_stamp_ns"] - rows[0]["reference_stamp_ns"]) / 1e9
                          for row in rows], dtype=float) if rows else np.array([])
    slope = None
    intercept = None
    if delta.size >= 2 and float(np.ptp(elapsed)) > 0:
        slope, intercept = np.polyfit(elapsed, delta, 1)
    summary = {
        "reference": reference_name, "target": target_name,
        "timestamp_source": "header_when_nonzero_else_bag",
        "pairing": "greedy_chronological_nearest_one_to_one",
        "max_delta_s": max_delta_s,
        "reference_count": len(reference), "target_count": len(target),
        "matched_count": len(rows),
        "reference_coverage_fraction": len(rows) / len(reference) if reference else 0.0,
        "target_coverage_fraction": len(rows) / len(target) if target else 0.0,
        "delta_s": robust_summary(delta.tolist()),
        "absolute_delta_s": robust_summary(np.abs(delta).tolist()),
        "linear_drift_s_per_s": float(slope) if slope is not None else None,
        "linear_intercept_s": float(intercept) if intercept is not None else None,
    }
    return summary, rows


def lidar_sample(msg: Any) -> dict[str, Any]:
    points = list(getattr(msg, "points", []))
    declared = getattr(msg, "point_num", None)
    xyz_finite = 0
    offsets: list[int] = []
    for point in points:
        xyz_finite += int(finite([getattr(point, axis, math.nan) for axis in "xyz"]))
        if hasattr(point, "offset_time"):
            offsets.append(int(point.offset_time))
    offset_delta = np.diff(np.asarray(offsets, dtype=np.int64)) if offsets else np.array([])
    return {
        "point_num_declared": int(declared) if declared is not None else None,
        "points_length": len(points),
        "point_count_consistent": declared is None or int(declared) == len(points),
        "xyz_finite_count": xyz_finite,
        "xyz_nonfinite_count": len(points) - xyz_finite,
        "offset_time_available": bool(offsets),
        "offset_time_count": len(offsets),
        "offset_time_min": min(offsets) if offsets else None,
        "offset_time_max": max(offsets) if offsets else None,
        "offset_time_duplicate_count": int(np.sum(offset_delta == 0)),
        "offset_time_regression_count": int(np.sum(offset_delta < 0)),
    }


def imu_sample(msg: Any) -> dict[str, Any]:
    linear = getattr(msg, "linear_acceleration", None)
    angular = getattr(msg, "angular_velocity", None)
    orientation = getattr(msg, "orientation", None)
    acc = [getattr(linear, axis, math.nan) for axis in "xyz"]
    gyro = [getattr(angular, axis, math.nan) for axis in "xyz"]
    quat = [getattr(orientation, axis, math.nan) for axis in "xyzw"]
    return {
        "linear_acceleration_finite": finite(acc), "angular_velocity_finite": finite(gyro),
        "orientation_finite": finite(quat),
        "acc_x": float(acc[0]), "acc_y": float(acc[1]), "acc_z": float(acc[2]),
        "acc_norm": float(np.linalg.norm(acc)) if finite(acc) else None,
        "gyro_x": float(gyro[0]), "gyro_y": float(gyro[1]), "gyro_z": float(gyro[2]),
        "gyro_norm": float(np.linalg.norm(gyro)) if finite(gyro) else None,
        "orientation_norm": float(np.linalg.norm(quat)) if finite(quat) else None,
    }


def _laplacian_variance(gray: np.ndarray) -> float | None:
    if gray.ndim != 2 or min(gray.shape) < 3:
        return None
    lap = (-4.0 * gray[1:-1, 1:-1] + gray[:-2, 1:-1] + gray[2:, 1:-1]
           + gray[1:-1, :-2] + gray[1:-1, 2:])
    return float(np.var(lap))


def image_sample(msg: Any) -> dict[str, Any]:
    height, width = int(getattr(msg, "height", 0)), int(getattr(msg, "width", 0))
    encoding = str(getattr(msg, "encoding", "")).lower()
    step = int(getattr(msg, "step", 0))
    raw = bytes(getattr(msg, "data", b""))
    channels = {"mono8": 1, "8uc1": 1, "rgb8": 3, "bgr8": 3,
                "rgba8": 4, "bgra8": 4}.get(encoding)
    bytes_per_channel = 2 if encoding in {"mono16", "16uc1"} else 1
    if encoding in {"mono16", "16uc1"}:
        channels = 1
    minimum_step = width * channels * bytes_per_channel if channels else None
    minimum_length = step * height
    aligned_step = bool(step % bytes_per_channel == 0)
    result = {
        "width": width, "height": height, "encoding": encoding, "step": step,
        "data_length": len(raw), "known_raw_encoding": channels is not None,
        "minimum_step": minimum_step, "step_aligned_to_channel_bytes": aligned_step,
        "step_valid": minimum_step is None or (step >= minimum_step and aligned_step),
        "data_length_valid": len(raw) >= minimum_length,
        "brightness_mean": None, "brightness_std": None,
        "low_endpoint_fraction": None, "high_endpoint_fraction": None,
        "laplacian_variance": None,
    }
    if not channels or not result["step_valid"] or not result["data_length_valid"] or not width or not height:
        return result
    dtype = (np.dtype(">u2") if bool(getattr(msg, "is_bigendian", False))
             else np.dtype("<u2")) if bytes_per_channel == 2 else np.dtype("u1")
    pixels_per_row = step // bytes_per_channel
    array = np.frombuffer(raw[:minimum_length], dtype=dtype).reshape(height, pixels_per_row)
    image = array[:, :width * channels].reshape(height, width, channels)
    if channels == 1:
        gray = image[..., 0].astype(float)
    else:
        rgb = image[..., :3].astype(float)
        if encoding.startswith("bgr"):
            rgb = rgb[..., ::-1]
        gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    maximum = float(np.iinfo(np.uint16 if bytes_per_channel == 2 else np.uint8).max)
    result.update({
        "brightness_mean": float(np.mean(gray)), "brightness_std": float(np.std(gray)),
        "low_endpoint_fraction": float(np.mean(gray == 0)),
        "high_endpoint_fraction": float(np.mean(gray == maximum)),
        "laplacian_variance": _laplacian_variance(gray),
    })
    return result


def tf_sample(msg: Any) -> list[dict[str, Any]]:
    rows = []
    for transform in getattr(msg, "transforms", []):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = [translation.x, translation.y, translation.z,
                  rotation.x, rotation.y, rotation.z, rotation.w]
        rows.append({
            "parent_frame": str(transform.header.frame_id),
            "child_frame": str(transform.child_frame_id),
            "transform_stamp_ns": stamp_ns(transform),
            "finite": finite(values),
            "quaternion_norm": float(np.linalg.norm(values[3:])) if finite(values[3:]) else None,
            "self_edge": str(transform.header.frame_id) == str(transform.child_frame_id),
        })
    return rows


@dataclass
class TopicState:
    name: str
    type_name: str
    offered_qos_profiles: str = ""
    rows: list[dict[str, Any]] = field(default_factory=list)
    samples: list[dict[str, Any]] = field(default_factory=list)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def provenance(path: Path, hash_mode: str) -> dict[str, Any]:
    files = []
    for item in sorted(path.rglob("*")):
        if not item.is_file():
            continue
        record = {"path": str(item.relative_to(path)), "size_bytes": item.stat().st_size,
                  "mtime_ns": item.stat().st_mtime_ns, "sha256": None}
        if hash_mode == "all" or item.name == "metadata.yaml":
            record["sha256"] = sha256(item)
        files.append(record)
    return {"bag_path": str(path.resolve()), "hash_mode": hash_mode, "files": files}


def read_bag(
    path: Path, declared: dict[str, str | None], sample_every: int,
    max_samples: int,
) -> tuple[dict[str, TopicState], list[dict[str, Any]], dict[str, Any]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python rosbag2/rclpy/rosidl APIs are required") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    metadata = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    wanted = {name for name in declared.values() if name}
    required = {declared[role] for role in ("lidar", "imu", "image") if declared.get(role)}
    absent = sorted(required - set(metadata))
    if absent:
        raise RuntimeError(f"declared sensor topic(s) absent from bag: {absent}")
    # TF is audited when present, but its absence does not invalidate a sensor bag.
    wanted &= set(metadata)
    states = {name: TopicState(
        name=name, type_name=metadata[name].type,
        offered_qos_profiles=str(getattr(metadata[name], "offered_qos_profiles", "") or ""),
    ) for name in sorted(wanted)}
    message_types = {name: get_message(state.type_name) for name, state in states.items()}
    tf_rows: list[dict[str, Any]] = []
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic not in states:
            continue
        state = states[topic]
        msg = deserialize_message(data, message_types[topic])
        index = len(state.rows)
        header = stamp_ns(msg)
        state.rows.append({"index": index, "bag_stamp_ns": int(bag_stamp),
                           "header_stamp_ns": header if header and header > 0 else None,
                           "frame_id": frame_id(msg)})
        if topic in {declared.get("tf"), declared.get("tf_static")}:
            transforms = tf_sample(msg)
            for transform in transforms:
                transform["topic"] = topic
                transform["bag_stamp_ns"] = int(bag_stamp)
            tf_rows.extend(transforms)
        if index % sample_every != 0 or len(state.samples) >= max_samples:
            continue
        base = {"message_index": index, "bag_stamp_ns": int(bag_stamp),
                "header_stamp_ns": header, "frame_id": frame_id(msg)}
        if topic == declared.get("lidar"):
            base.update(lidar_sample(msg))
        elif topic == declared.get("imu"):
            base.update(imu_sample(msg))
        elif topic == declared.get("image"):
            base.update(image_sample(msg))
        state.samples.append(base)
    info = {"available_topics": {name: {"type": item.type,
            "offered_qos_profiles": str(getattr(item, "offered_qos_profiles", "") or "")}
            for name, item in sorted(metadata.items())}}
    return states, tf_rows, info


def sample_metrics(role: str, rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {"sample_count": len(rows), "observed": {}}
    numeric_keys = sorted({key for row in rows for key, value in row.items()
                           if isinstance(value, (int, float)) and not isinstance(value, bool)
                           and key not in {"message_index", "bag_stamp_ns", "header_stamp_ns"}})
    for key in numeric_keys:
        result["observed"][key] = robust_summary([row[key] for row in rows if row.get(key) is not None])
    if role == "lidar":
        result["structural_violations"] = {
            "point_count_mismatch": sum(not row.get("point_count_consistent", True) for row in rows),
            "sample_with_nonfinite_xyz": sum(row.get("xyz_nonfinite_count", 0) > 0 for row in rows),
            "sample_with_offset_regression": sum(row.get("offset_time_regression_count", 0) > 0 for row in rows),
        }
    elif role == "imu":
        result["structural_violations"] = {
            "nonfinite_linear_acceleration": sum(not row.get("linear_acceleration_finite", False) for row in rows),
            "nonfinite_angular_velocity": sum(not row.get("angular_velocity_finite", False) for row in rows),
            "nonfinite_orientation": sum(not row.get("orientation_finite", False) for row in rows),
        }
    elif role == "image":
        result["encodings"] = sorted({str(row.get("encoding", "")) for row in rows})
        result["resolutions"] = sorted({f"{row.get('width')}x{row.get('height')}" for row in rows})
        result["structural_violations"] = {
            "unknown_raw_encoding": sum(not row.get("known_raw_encoding", False) for row in rows),
            "invalid_step": sum(not row.get("step_valid", False) for row in rows),
            "short_data_buffer": sum(not row.get("data_length_valid", False) for row in rows),
        }
    return result


def tf_metrics(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    edges = sorted({(row["parent_frame"], row["child_frame"]) for row in rows})
    children: dict[str, set[str]] = {}
    for parent, child in edges:
        children.setdefault(child, set()).add(parent)
    return {
        "transform_count": len(rows),
        "unique_edges": [{"parent": parent, "child": child} for parent, child in edges],
        "frames": sorted({frame for edge in edges for frame in edge}),
        "nonfinite_transform_count": sum(not row["finite"] for row in rows),
        "self_edge_count": sum(row["self_edge"] for row in rows),
        "empty_parent_frame_count": sum(not row["parent_frame"] for row in rows),
        "empty_child_frame_count": sum(not row["child_frame"] for row in rows),
        "children_with_multiple_parents": {child: sorted(parents) for child, parents in children.items()
                                           if len(parents) > 1},
        "note": "Graph cycles and time-varying parent changes require a dedicated TF reconstruction.",
    }


def coverage_metrics(states: dict[str, TopicState], declared: dict[str, str | None]) -> dict[str, Any]:
    intervals = []
    by_role = {}
    for role in ("lidar", "imu", "image"):
        name = declared.get(role)
        if not name or name not in states or not states[name].rows:
            continue
        stamps = [row.get("header_stamp_ns") or row["bag_stamp_ns"] for row in states[name].rows]
        by_role[role] = {"first_ns": min(stamps), "last_ns": max(stamps),
                         "duration_s": (max(stamps) - min(stamps)) / 1e9}
        intervals.append((min(stamps), max(stamps)))
    overlap = max(0, min(end for _, end in intervals) - max(start for start, _ in intervals)) / 1e9 if intervals else 0.0
    union = (max(end for _, end in intervals) - min(start for start, _ in intervals)) / 1e9 if intervals else 0.0
    return {"by_role": by_role, "common_overlap_s": overlap, "union_span_s": union,
            "common_overlap_over_union": overlap / union if union > 0 else None}


def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def make_plots(output: Path, states: dict[str, TopicState], sync: dict[str, list[dict[str, Any]]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(max(1, len(states)), 1, figsize=(9, max(3, 2.6 * len(states))), squeeze=False)
    for ax, state in zip(axes[:, 0], states.values()):
        stamps = np.asarray([row["bag_stamp_ns"] for row in state.rows], dtype=np.int64)
        gaps = np.diff(stamps) / 1e9
        ax.plot(np.arange(1, len(stamps)), gaps, linewidth=.7)
        ax.set(title=f"{state.name}: bag timestamp deltas", ylabel="delta [s]")
        ax.grid(True, alpha=.3)
    axes[-1, 0].set_xlabel("message index")
    fig.tight_layout(); fig.savefig(output / "input_timing.png", dpi=160); plt.close(fig)
    if sync:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for name, rows in sync.items():
            if rows:
                t0 = rows[0]["reference_stamp_ns"]
                ax.plot([(row["reference_stamp_ns"] - t0) / 1e9 for row in rows],
                        [row["delta_s"] for row in rows], ".", markersize=2, label=name)
        ax.set(xlabel="reference elapsed time [s]", ylabel="target - reference [s]",
               title="One-to-one nearest timestamp synchronization")
        ax.grid(True, alpha=.3); ax.legend()
        fig.tight_layout(); fig.savefig(output / "input_synchronization.png", dpi=160); plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--imu-topic")
    parser.add_argument("--image-topic")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--sample-every", type=int, default=100)
    parser.add_argument("--max-content-samples", type=int, default=1000)
    parser.add_argument("--sync-max-delta-s", type=float)
    parser.add_argument("--hash-mode", choices=("metadata", "all"), default="metadata")
    args = parser.parse_args()
    if args.sample_every < 1 or args.max_content_samples < 1:
        parser.error("sampling values must be positive")
    if args.sync_max_delta_s is not None and args.sync_max_delta_s < 0:
        parser.error("--sync-max-delta-s must be non-negative")
    if not args.bag.is_dir() or not (args.bag / "metadata.yaml").is_file():
        raise RuntimeError(f"invalid rosbag2 directory: {args.bag}")
    declared = {"lidar": args.lidar_topic, "imu": args.imu_topic, "image": args.image_topic,
                "tf": args.tf_topic, "tf_static": args.tf_static_topic}
    states, tf_rows, bag_info = read_bag(args.bag, declared, args.sample_every, args.max_content_samples)
    topic_reports = {}
    for role, name in declared.items():
        if name and name in states:
            topic_reports[role] = {"name": name, "type": states[name].type_name,
                "offered_qos_profiles": states[name].offered_qos_profiles,
                **topic_metrics(states[name].rows),
                "content_sampling": sample_metrics(role, states[name].samples) if role in {"lidar", "imu", "image"} else None}
    sync_reports: dict[str, Any] = {}
    sync_rows: dict[str, list[dict[str, Any]]] = {}
    lidar = states[args.lidar_topic].rows
    for role in ("imu", "image"):
        name = declared.get(role)
        if name and name in states:
            key = f"lidar_to_{role}"
            sync_reports[key], sync_rows[key] = synchronization_metrics(
                lidar, states[name].rows, "lidar", role, args.sync_max_delta_s)
    report = {
        "schema_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
        "policy": "descriptive_measurements_and_structural_violations_only",
        "gap_candidate_policy": "median_plus_10_MAD_is_diagnostic_not_a_physical_threshold",
        "command": [sys.executable, *sys.argv],
        "environment": {"python": sys.version, "platform": platform.platform(),
                        "ros_distro": os.environ.get("ROS_DISTRO"),
                        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION")},
        "provenance": provenance(args.bag, args.hash_mode),
        "declared_topics": declared, "bag_catalog": bag_info,
        "sampling": {"every_nth_message": args.sample_every,
                     "maximum_samples_per_topic": args.max_content_samples},
        "topics": topic_reports, "coverage": coverage_metrics(states, declared),
        "synchronization": sync_reports, "tf_audit": tf_metrics(tf_rows),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    safe = json_safe(report)
    (args.output / "input_integrity.json").write_text(json.dumps(safe, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (args.output / "input_integrity.yaml").write_text(yaml.safe_dump(safe, sort_keys=False), encoding="utf-8")
    for role, name in declared.items():
        if name and name in states:
            write_csv(args.output / f"input_{role}_timing.csv", states[name].rows)
            if states[name].samples:
                write_csv(args.output / f"input_{role}_samples.csv", states[name].samples)
    write_csv(args.output / "input_tf_edges.csv", tf_rows)
    for key, rows in sync_rows.items():
        write_csv(args.output / f"input_sync_{key}.csv", rows)
    make_plots(args.output, states, sync_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
