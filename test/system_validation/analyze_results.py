#!/usr/bin/env python3
"""Offline FAST-LIVO2 output analysis with optional ground truth."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

TF_TOPIC = "/tf"
ODOM_TOPIC = "/aft_mapped_to_init"
INFO_TOPIC = "/lidar_measurement_information"
LOCAL_TOPIC = "/lidar_localizability_calibration"
METRIC_CLOUD_TOPIC = "/cloud_registered_metric"


def _stamp_ns(msg: Any, bag_stamp: int) -> int:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return int(bag_stamp)
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(bag_stamp)


def _finite(values: Iterable[float]) -> bool:
    return bool(np.all(np.isfinite(np.asarray(list(values), dtype=float))))


def json_safe(value: Any) -> Any:
    """Convert NumPy and non-finite values to strict JSON-compatible values."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def exact_header_stamp(msg: Any) -> int:
    # Scientific association never substitutes recorder time, including at zero.
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def tf_row(msg: Any) -> dict[str, Any]:
    t, q = msg.transform.translation, msg.transform.rotation
    return {
        "stamp_ns": exact_header_stamp(msg),
        "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
        "x": float(t.x), "y": float(t.y), "z": float(t.z),
        "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
    }


def pose_tf_contract(odom: list[dict], transforms: list[dict]) -> dict[str, Any]:
    edge = ("camera_init", "aft_mapped")
    frame = lambda row: (row.get("frame_id"), row.get("child_frame_id"))
    # Ignore all other TF edges; an absent/misframed FAST edge leaves odom unmatched.
    fast = [row for row in transforms if frame(row) == edge]
    stamp = lambda row: int(row.get("header_stamp_ns", row["stamp_ns"]))
    odom_counts = Counter(stamp(row) for row in odom)
    tf_counts = Counter(stamp(row) for row in fast)
    by_stamp = {stamp(row): row for row in fast}
    components = ("x", "y", "z", "qx", "qy", "qz", "qw")
    mismatches = 0
    for row in odom:
        other = by_stamp.get(stamp(row))
        if other is not None:
            a = [float(row.get(k, math.nan)) for k in components]
            b = [float(other.get(k, math.nan)) for k in components]
            if not (_finite(a) and _finite(b)) or struct.pack("!7d", *a) != struct.pack("!7d", *b):
                mismatches += 1
    failures = {
        "duplicate_odometry_stamp_count": sum(n - 1 for n in odom_counts.values()),
        "duplicate_tf_stamp_count": sum(n - 1 for n in tf_counts.values()),
        "odometry_without_tf_count": sum((odom_counts - tf_counts).values()),
        "tf_without_odometry_count": sum((tf_counts - odom_counts).values()),
        "frame_mismatch_count": sum(frame(row) != edge for row in odom),
        "pose_payload_mismatch_count": mismatches,
    }
    return {
        "available": bool(odom and fast),
        "edge": list(edge), "association": "exact_header_timestamp",
        "payload_comparison": "float64_bit_exact",
        "odometry_count": len(odom), "fast_tf_count": len(fast),
        "exact_pair_count": sum((odom_counts & tf_counts).values()),
        **failures,
        "exact_pairing_complete": bool(odom and fast and not any(failures.values())),
    }


def pose_row(msg: Any, bag_stamp: int) -> dict[str, Any]:
    pose = getattr(msg, "pose", msg)
    pose = getattr(pose, "pose", pose)
    position, orientation = pose.position, pose.orientation
    values = [
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    ]
    return {
        "stamp_ns": _stamp_ns(msg, bag_stamp),
        "header_stamp_ns": exact_header_stamp(msg),
        "x": float(values[0]), "y": float(values[1]), "z": float(values[2]),
        "qx": float(values[3]), "qy": float(values[4]),
        "qz": float(values[5]), "qw": float(values[6]),
        "finite": _finite(values),
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "child_frame_id": str(getattr(msg, "child_frame_id", "")),
    }


def cloud_row(msg: Any, bag_stamp: int) -> dict[str, Any]:
    return {
        "stamp_ns": _stamp_ns(msg, bag_stamp),
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "width": int(getattr(msg, "width", 0)),
        "height": int(getattr(msg, "height", 0)),
        "point_step": int(getattr(msg, "point_step", 0)),
        "row_step": int(getattr(msg, "row_step", 0)),
        "data_size_bytes": len(getattr(msg, "data", b"")),
        "is_dense": bool(getattr(msg, "is_dense", False)),
    }


def info_row(msg: Any, bag_stamp: int, tolerance_factor: float = 1e-9) -> dict[str, Any]:
    raw = np.asarray(getattr(msg, "measurement_information", []), dtype=float)
    valid_shape = raw.size == 36
    finite = bool(valid_shape and _finite(raw))
    matrix = raw.reshape(6, 6) if finite else np.empty((0, 0))
    matrix_scale = float(np.linalg.norm(matrix, ord=2)) if finite else math.nan
    applied_tolerance = (
        float(abs(tolerance_factor) * max(1.0, matrix_scale)) if finite else math.nan
    )
    symmetry_error = float(np.max(np.abs(matrix - matrix.T))) if finite else math.nan
    symmetric = bool(finite and symmetry_error <= applied_tolerance)
    # eigvalsh is only mathematically valid on a symmetric matrix. Symmetrize
    # solely for the PSD diagnostic while preserving the measured asymmetry.
    eigen = np.linalg.eigvalsh(0.5 * (matrix + matrix.T)) if finite else np.array([])
    negative = eigen[eigen < -applied_tolerance]
    positive = eigen[eigen > applied_tolerance]
    condition = float(eigen[-1] / positive[0]) if positive.size and eigen[-1] >= 0 else math.inf
    return {
        "stamp_ns": _stamp_ns(msg, bag_stamp),
        "available": bool(getattr(msg, "available", False)),
        "effective_features": int(getattr(msg, "effective_features", 0)),
        "mean_abs_residual_m": float(getattr(msg, "mean_abs_point_to_plane_residual_m", math.nan)),
        "eigen_min": float(eigen[0]) if eigen.size else math.nan,
        "eigen_max": float(eigen[-1]) if eigen.size else math.nan,
        "condition": condition,
        "matrix_shape_valid": valid_shape,
        "matrix_scale": matrix_scale,
        "tolerance_factor": float(abs(tolerance_factor)),
        "applied_tolerance": applied_tolerance,
        "symmetry_max_abs_error": symmetry_error,
        "symmetric_within_tolerance": symmetric,
        "negative_eigenvalue_count": int(negative.size),
        "psd_within_tolerance": bool(finite and symmetric and negative.size == 0),
        "finite": finite,
    }
def local_row(msg: Any, bag_stamp: int) -> dict[str, Any]:
    rotation = np.asarray(getattr(msg, "rotation_eigenvalues", []), dtype=float)
    translation = np.asarray(getattr(msg, "translation_eigenvalues", []), dtype=float)
    if rotation.size and _finite(rotation):
        rotation = np.sort(rotation)
    if translation.size and _finite(translation):
        translation = np.sort(translation)

    def cond(values: np.ndarray) -> float:
        positive = values[values > np.finfo(float).eps]
        return float(values[-1] / positive[0]) if positive.size else math.inf

    return {
        "stamp_ns": _stamp_ns(msg, bag_stamp),
        "available": bool(getattr(msg, "available", False)),
        "input_samples": int(getattr(msg, "input_samples", 0)),
        "used_samples": int(getattr(msg, "used_samples", 0)),
        "rotation_eigen_min": float(rotation[0]) if rotation.size else math.nan,
        "rotation_eigen_max": float(rotation[-1]) if rotation.size else math.nan,
        "rotation_condition": cond(rotation) if rotation.size else math.nan,
        "translation_eigen_min": float(translation[0]) if translation.size else math.nan,
        "translation_eigen_max": float(translation[-1]) if translation.size else math.nan,
        "translation_condition": cond(translation) if translation.size else math.nan,
        "finite": _finite(np.r_[rotation, translation]) if rotation.size and translation.size else False,
    }


def read_bag(
    path: Path, gt_topic: str | None = None
) -> tuple[list[dict], list[dict], list[dict], list[dict], list[dict], list[dict]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python bag APIs are required for --bag") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {ODOM_TOPIC, INFO_TOPIC, LOCAL_TOPIC, METRIC_CLOUD_TOPIC, TF_TOPIC}
    if TF_TOPIC in types and types[TF_TOPIC] != "tf2_msgs/msg/TFMessage":
        raise RuntimeError("/tf must have type tf2_msgs/msg/TFMessage")
    if gt_topic:
        wanted.add(gt_topic)
    messages = {name: get_message(types[name]) for name in wanted if name in types}
    odom: list[dict] = []
    info: list[dict] = []
    local: list[dict] = []
    metric_cloud: list[dict] = []
    gt: list[dict] = []
    transforms: list[dict] = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in messages:
            continue
        msg = deserialize_message(data, messages[topic])
        if topic == ODOM_TOPIC:
            odom.append(pose_row(msg, timestamp))
        elif topic == INFO_TOPIC:
            info.append(info_row(msg, timestamp))
        elif topic == LOCAL_TOPIC:
            local.append(local_row(msg, timestamp))
        elif topic == METRIC_CLOUD_TOPIC:
            metric_cloud.append(cloud_row(msg, timestamp))
        elif topic == TF_TOPIC:
            transforms.extend(tf_row(t) for t in msg.transforms
                              if (t.header.frame_id, t.child_frame_id) == ("camera_init", "aft_mapped"))
        elif topic == gt_topic:
            if hasattr(msg, "poses"):
                gt.extend(pose_row(pose, timestamp) for pose in msg.poses)
            else:
                gt.append(pose_row(msg, timestamp))
    return odom, info, local, metric_cloud, gt, transforms


def load_csv(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    typed: list[dict[str, Any]] = []
    for row in rows:
        item: dict[str, Any] = {}
        for key, value in row.items():
            if key in {"frame_id", "child_frame_id"}:
                item[key] = value
            elif key in {"available", "finite"}:
                item[key] = value.lower() in {"1", "true", "yes"}
            elif key in {"stamp_ns", "header_stamp_ns"} or key.endswith("_samples") or key == "effective_features":
                item[key] = int(value)
            else:
                try:
                    item[key] = float(value)
                except (TypeError, ValueError):
                    item[key] = value
        typed.append(item)
    return typed


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def series_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"count": 0, "available": False}
    stamps = np.asarray([row["stamp_ns"] for row in rows], dtype=np.int64)
    gaps = np.diff(stamps).astype(float) / 1e9
    duration = float((stamps[-1] - stamps[0]) / 1e9)
    finite_flags = [bool(row.get("finite", True)) for row in rows]
    metrics: dict[str, Any] = {
        "count": len(rows),
        "available": True,
        "first_stamp_ns": int(stamps[0]),
        "last_stamp_ns": int(stamps[-1]),
        "duration_s": duration,
        "strictly_monotonic": bool(np.all(gaps > 0)),
        "nonpositive_gap_count": int(np.sum(gaps <= 0)),
        "median_gap_s": float(np.median(gaps)) if gaps.size else None,
        "max_gap_s": float(np.max(gaps)) if gaps.size else None,
        "mean_rate_hz": float((len(rows) - 1) / duration) if duration > 0 else None,
        "finite_count": int(sum(finite_flags)),
        "finite_fraction": float(np.mean(finite_flags)),
    }
    if "available" in rows[0]:
        flags = [bool(row["available"]) for row in rows]
        metrics["tracking_available_count"] = int(sum(flags))
        metrics["tracking_available_fraction"] = float(np.mean(flags))
    return metrics


def pose_metric_cloud_contract(
    odom: list[dict[str, Any]], metric_cloud: list[dict[str, Any]]
) -> dict[str, Any]:
    odom_stamps = [int(row["stamp_ns"]) for row in odom]
    cloud_stamps = [int(row["stamp_ns"]) for row in metric_cloud]
    odom_counts, cloud_counts = Counter(odom_stamps), Counter(cloud_stamps)
    exact_pair_count = sum((odom_counts & cloud_counts).values())
    unmatched_cloud_count = sum((cloud_counts - odom_counts).values())
    odom_without_cloud_count = sum((odom_counts - cloud_counts).values())
    duplicate_cloud_stamp_count = sum(count - 1 for count in cloud_counts.values())
    cloud_gaps = np.diff(np.asarray(cloud_stamps, dtype=np.int64))
    cloud_nonpositive_gap_count = int(np.sum(cloud_gaps <= 0))
    odom_frames: dict[int, set[str]] = {}
    for row in odom:
        odom_frames.setdefault(int(row["stamp_ns"]), set()).add(
            str(row.get("frame_id", ""))
        )
    frame_mismatch_count = sum(
        1 for row in metric_cloud
        if str(row.get("frame_id", ""))
        not in odom_frames.get(int(row["stamp_ns"]), set())
    )
    empty_cloud_count = sum(
        int(row.get("width", 0)) * int(row.get("height", 0)) <= 0
        or int(row.get("data_size_bytes", 0)) <= 0
        for row in metric_cloud
    )
    return {
        "available": bool(odom and metric_cloud),
        "odometry_count": len(odom),
        "metric_cloud_count": len(metric_cloud),
        "exact_pair_count": int(exact_pair_count),
        "exact_pair_fraction_of_clouds": (
            float(exact_pair_count / len(metric_cloud)) if metric_cloud else None
        ),
        "unmatched_metric_cloud_count": int(unmatched_cloud_count),
        "odometry_without_metric_cloud_count": int(odom_without_cloud_count),
        "duplicate_metric_cloud_stamp_count": int(duplicate_cloud_stamp_count),
        "metric_cloud_nonpositive_gap_count": cloud_nonpositive_gap_count,
        "frame_mismatch_count": int(frame_mismatch_count),
        "empty_metric_cloud_count": int(empty_cloud_count),
        "exact_pairing_complete": bool(
            metric_cloud
            and unmatched_cloud_count == 0
            and duplicate_cloud_stamp_count == 0
            and cloud_nonpositive_gap_count == 0
            and frame_mismatch_count == 0
            and empty_cloud_count == 0
        ),
        "note": (
            "Metric-cloud timestamps must be a unique strictly increasing subset "
            "of odometry timestamps; extra odometry updates are allowed in LIVO."
        ),
    }


def trajectory_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    metrics = series_metrics(rows)
    if not rows:
        return metrics
    points = np.asarray([[row["x"], row["y"], row["z"]] for row in rows], dtype=float)
    quaternions = np.asarray(
        [[row["qw"], row["qx"], row["qy"], row["qz"]] for row in rows], dtype=float
    )
    norms = np.linalg.norm(quaternions, axis=1)
    finite_points = np.all(np.isfinite(points), axis=1)
    valid_q = np.all(np.isfinite(quaternions), axis=1) & (norms > np.finfo(float).eps)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    finite_steps = steps[np.isfinite(steps)]
    median_step = float(np.median(finite_steps)) if finite_steps.size else math.nan
    # Estimate nominal motion without the largest decile so a single reset cannot
    # inflate its own descriptive outlier threshold.
    ordered_steps = np.sort(finite_steps)
    nominal_count = max(1, int(math.ceil(0.9 * ordered_steps.size)) - 1)
    nominal_steps = ordered_steps[:nominal_count] if ordered_steps.size > 1 else ordered_steps
    nominal_median = float(np.median(nominal_steps)) if nominal_steps.size else math.nan
    mad_step = (
        float(np.median(np.abs(nominal_steps - nominal_median)))
        if nominal_steps.size else math.nan
    )
    jump_threshold = max(
        1.0, nominal_median + 10.0 * max(mad_step, nominal_median, np.finfo(float).eps)
    )
    sign_flips = 0
    for first, second, ok in zip(quaternions, quaternions[1:], valid_q[:-1] & valid_q[1:]):
        if ok and float(np.dot(first, second)) < 0:
            sign_flips += 1
    distance_from_start = (
        np.linalg.norm(points - points[0], axis=1)
        if finite_points[0] else np.full(len(points), np.nan)
    )
    reset_candidates = 0
    for index, step in enumerate(steps, start=1):
        prior = distance_from_start[:index]
        prior_extent = float(np.max(prior[np.isfinite(prior)])) if np.any(np.isfinite(prior)) else 0.0
        if (np.isfinite(step) and step > jump_threshold and prior_extent > 1.0
                and distance_from_start[index] < 0.1 * prior_extent):
            reset_candidates += 1
    metrics.update({
        "path_length_m": float(np.sum(finite_steps)),
        "start_end_displacement_m": (
            float(np.linalg.norm(points[-1] - points[0]))
            if finite_points[0] and finite_points[-1] else None
        ),
        "max_step_m": float(np.max(finite_steps)) if finite_steps.size else None,
        "translation_jump_candidate_count": int(np.sum(finite_steps > jump_threshold)),
        "translation_jump_candidate_threshold_m": float(jump_threshold),
        "reset_to_origin_candidate_count": int(reset_candidates),
        "position_nonfinite_count": int(np.sum(~finite_points)),
        "quaternion_invalid_count": int(np.sum(~valid_q)),
        "quaternion_norm_abs_error_max": (
            float(np.max(np.abs(norms[valid_q] - 1.0))) if np.any(valid_q) else None
        ),
        "quaternion_norm_abs_error_median": (
            float(np.median(np.abs(norms[valid_q] - 1.0))) if np.any(valid_q) else None
        ),
        "quaternion_sign_flip_count": int(sign_flips),
        "frames": sorted({str(row.get("frame_id", "")) for row in rows}),
        "child_frames": sorted({str(row.get("child_frame_id", "")) for row in rows}),
    })
    return metrics
def _quaternion(row: dict) -> np.ndarray:
    value = np.asarray([row["qw"], row["qx"], row["qy"], row["qz"]], dtype=float)
    norm = np.linalg.norm(value)
    return value / norm if norm > 0 and np.isfinite(norm) else np.full(4, np.nan)


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.asarray([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def _qinv(q: np.ndarray) -> np.ndarray:
    return q * np.asarray([1.0, -1.0, -1.0, -1.0])


def _qrotate(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.r_[0.0, vector]
    return _qmul(_qmul(q, pure), _qinv(q))[1:]


def _slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    dot = float(np.dot(first, second))
    if dot < 0:
        second, dot = -second, -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        value = first + fraction * (second - first)
        return value / np.linalg.norm(value)
    theta = math.acos(dot)
    return (
        math.sin((1.0 - fraction) * theta) * first
        + math.sin(fraction * theta) * second
    ) / math.sin(theta)


def _interpolate_pose(
    reference: list[dict], stamp_ns: int, max_delta_ns: int
) -> tuple[dict | None, int | None]:
    """Interpolate a pose at stamp_ns; reject extrapolation and distant brackets."""
    if len(reference) < 2:
        return None, None
    stamps = np.asarray([row["stamp_ns"] for row in reference], dtype=np.int64)
    upper = int(np.searchsorted(stamps, stamp_ns, side="left"))
    if upper < len(reference) and int(stamps[upper]) == stamp_ns:
        return dict(reference[upper]), 0
    if upper == 0 or upper >= len(reference):
        return None, None
    lower = upper - 1
    delta = max(stamp_ns - int(stamps[lower]), int(stamps[upper]) - stamp_ns)
    if delta > max_delta_ns:
        return None, int(delta)
    span = int(stamps[upper]) - int(stamps[lower])
    if span <= 0:
        return None, int(delta)
    fraction = (stamp_ns - int(stamps[lower])) / span
    a, b = reference[lower], reference[upper]
    qa, qb = _quaternion(a), _quaternion(b)
    if not (_finite(qa) and _finite(qb)):
        return None, int(delta)
    result = {
        "stamp_ns": int(stamp_ns),
        "x": float(a["x"] + fraction * (b["x"] - a["x"])),
        "y": float(a["y"] + fraction * (b["y"] - a["y"])),
        "z": float(a["z"] + fraction * (b["z"] - a["z"])),
    }
    q = _slerp(qa, qb, fraction)
    result.update({"qw": q[0], "qx": q[1], "qy": q[2], "qz": q[3], "finite": True})
    return result, int(delta)


def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return np.asarray([
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        ])
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        q = [((rotation[2, 1] - rotation[1, 2]) / scale), 0.25 * scale,
             (rotation[0, 1] + rotation[1, 0]) / scale,
             (rotation[0, 2] + rotation[2, 0]) / scale]
    elif index == 1:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        q = [((rotation[0, 2] - rotation[2, 0]) / scale),
             (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
             (rotation[1, 2] + rotation[2, 1]) / scale]
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        q = [((rotation[1, 0] - rotation[0, 1]) / scale),
             (rotation[0, 2] + rotation[2, 0]) / scale,
             (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale]
    value = np.asarray(q)
    return value / np.linalg.norm(value)


def _rotation_angle(q: np.ndarray) -> float:
    return 2.0 * math.acos(float(np.clip(abs(q[0]), 0.0, 1.0)))


def _select_rpe_pairs(
    poses: list[tuple[dict, dict]], delta: float, unit: str
) -> list[tuple[int, int, float]]:
    if delta <= 0:
        raise ValueError("RPE delta must be positive")
    if unit == "seconds":
        abscissa = np.asarray([item[0]["stamp_ns"] for item in poses], dtype=float) / 1e9
    elif unit == "meters":
        points = np.asarray([[p[1]["x"], p[1]["y"], p[1]["z"]] for p in poses])
        abscissa = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    else:
        raise ValueError("RPE delta unit must be 'seconds' or 'meters'")
    result: list[tuple[int, int, float]] = []
    for first in range(len(poses)):
        target = abscissa[first] + delta
        second = int(np.searchsorted(abscissa, target, side="left"))
        if second < len(poses):
            achieved_delta = float(abscissa[second] - abscissa[first])
            result.append((first, second, achieved_delta))
    return result


def _relative_pose_error(first: dict, second: dict) -> tuple[np.ndarray, np.ndarray]:
    q_first, q_second = _quaternion(first), _quaternion(second)
    q_rel = _qmul(_qinv(q_first), q_second)
    p_first = np.asarray([first["x"], first["y"], first["z"]], dtype=float)
    p_second = np.asarray([second["x"], second["y"], second["z"]], dtype=float)
    p_rel = _qrotate(_qinv(q_first), p_second - p_first)
    return p_rel, q_rel


def ground_truth_metrics(
    odom: list[dict], gt: list[dict], max_delta_s: float,
    rpe_delta: float = 1.0, rpe_delta_unit: str = "seconds",
) -> dict[str, Any]:
    odom_sorted = sorted(odom, key=lambda row: row["stamp_ns"])
    gt_sorted = sorted(gt, key=lambda row: row["stamp_ns"])
    deltas: list[int] = []
    pairs: list[tuple[dict, dict]] = []
    for estimate in odom_sorted:
        truth, delta_ns = _interpolate_pose(
            gt_sorted, int(estimate["stamp_ns"]), int(max_delta_s * 1e9)
        )
        if truth is not None:
            pairs.append((estimate, truth))
            deltas.append(int(delta_ns or 0))
    association = {
        "method": "linear_position_slerp_orientation_no_extrapolation",
        "estimate_pose_count": len(odom_sorted),
        "matched_poses": len(pairs),
        "coverage_fraction": float(len(pairs) / len(odom_sorted)) if odom_sorted else 0.0,
        "max_allowed_bracket_delta_s": float(max_delta_s),
        "max_observed_bracket_delta_s": float(max(deltas) / 1e9) if deltas else None,
        "median_observed_bracket_delta_s": (
            float(np.median(deltas) / 1e9) if deltas else None
        ),
    }
    if len(pairs) < 2:
        return {"available": False, "association": association, "matched_poses": len(pairs)}

    estimate = np.asarray([[a["x"], a["y"], a["z"]] for a, _ in pairs])
    truth = np.asarray([[b["x"], b["y"], b["z"]] for _, b in pairs])
    valid = np.all(np.isfinite(estimate), axis=1) & np.all(np.isfinite(truth), axis=1)
    valid &= np.asarray([_finite(_quaternion(a)) and _finite(_quaternion(b)) for a, b in pairs])
    pairs = [pair for pair, keep in zip(pairs, valid) if keep]
    estimate, truth = estimate[valid], truth[valid]
    if len(pairs) < 2:
        return {"available": False, "association": association, "matched_poses": len(pairs)}

    estimate_centroid, truth_centroid = estimate.mean(axis=0), truth.mean(axis=0)
    u, _, vt = np.linalg.svd((estimate - estimate_centroid).T @ (truth - truth_centroid))
    alignment_rotation = vt.T @ u.T
    if np.linalg.det(alignment_rotation) < 0:
        vt[-1] *= -1
        alignment_rotation = vt.T @ u.T
    alignment_translation = truth_centroid - alignment_rotation @ estimate_centroid
    aligned = (alignment_rotation @ estimate.T).T + alignment_translation
    translation_error = np.linalg.norm(aligned - truth, axis=1)
    q_alignment = _rotation_matrix_to_quaternion(alignment_rotation)
    orientation_error = []
    aligned_pairs: list[tuple[dict, dict]] = []
    for (estimate_row, truth_row), aligned_position in zip(pairs, aligned):
        aligned_q = _qmul(q_alignment, _quaternion(estimate_row))
        orientation_error.append(_rotation_angle(_qmul(_qinv(_quaternion(truth_row)), aligned_q)))
        aligned_row = dict(estimate_row)
        aligned_row.update({
            "x": aligned_position[0], "y": aligned_position[1], "z": aligned_position[2],
            "qw": aligned_q[0], "qx": aligned_q[1], "qy": aligned_q[2], "qz": aligned_q[3],
        })
        aligned_pairs.append((aligned_row, truth_row))

    translation_rpe, rotation_rpe, achieved_rpe_deltas = [], [], []
    for first, second, achieved_delta in _select_rpe_pairs(
        aligned_pairs, rpe_delta, rpe_delta_unit
    ):
        estimate_rel = _relative_pose_error(aligned_pairs[first][0], aligned_pairs[second][0])
        truth_rel = _relative_pose_error(aligned_pairs[first][1], aligned_pairs[second][1])
        error_q = _qmul(_qinv(truth_rel[1]), estimate_rel[1])
        error_p = _qrotate(_qinv(truth_rel[1]), estimate_rel[0] - truth_rel[0])
        translation_rpe.append(float(np.linalg.norm(error_p)))
        rotation_rpe.append(_rotation_angle(error_q))
        achieved_rpe_deltas.append(float(achieved_delta))

    return {
        "available": True,
        "matched_poses": len(pairs),
        "association": association,
        "alignment": "rigid_SE3_Horn_no_scale",
        "alignment_scale": 1.0,
        "ate_translation_rmse_m": float(np.sqrt(np.mean(translation_error ** 2))),
        "ate_translation_median_m": float(np.median(translation_error)),
        "ate_rotation_rmse_rad": float(np.sqrt(np.mean(np.square(orientation_error)))),
        "ate_rotation_median_rad": float(np.median(orientation_error)),
        "rpe_definition": {
            "method": "SE3_relative_pose_error",
            "requested_delta": float(rpe_delta),
            "delta_unit": rpe_delta_unit,
            "pair_count": len(translation_rpe),
            "achieved_delta_min": (
                float(np.min(achieved_rpe_deltas)) if achieved_rpe_deltas else None
            ),
            "achieved_delta_median": (
                float(np.median(achieved_rpe_deltas)) if achieved_rpe_deltas else None
            ),
            "achieved_delta_mean": (
                float(np.mean(achieved_rpe_deltas)) if achieved_rpe_deltas else None
            ),
            "achieved_delta_max": (
                float(np.max(achieved_rpe_deltas)) if achieved_rpe_deltas else None
            ),
        },
        "rpe_translation_rmse_m": (
            float(np.sqrt(np.mean(np.square(translation_rpe)))) if translation_rpe else None
        ),
        "rpe_rotation_rmse_rad": (
            float(np.sqrt(np.mean(np.square(rotation_rpe)))) if rotation_rpe else None
        ),
    }
def eigen_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    metrics = series_metrics(rows)
    if not rows:
        return metrics
    for key in (
        "eigen_min", "eigen_max", "condition", "matrix_scale",
        "tolerance_factor", "applied_tolerance", "symmetry_max_abs_error",
        "rotation_eigen_min", "rotation_eigen_max", "rotation_condition",
        "translation_eigen_min", "translation_eigen_max", "translation_condition",
    ):
        values = np.asarray([row[key] for row in rows if key in row], dtype=float)
        finite = values[np.isfinite(values)]
        if finite.size:
            metrics[key] = {
                "min": float(np.min(finite)),
                "median": float(np.median(finite)),
                "max": float(np.max(finite)),
                "finite_count": int(finite.size),
            }
    for key in ("matrix_shape_valid", "symmetric_within_tolerance", "psd_within_tolerance"):
        values = [bool(row[key]) for row in rows if key in row]
        if values:
            metrics[key + "_fraction"] = float(np.mean(values))
            metrics[key + "_false_count"] = int(len(values) - sum(values))
    negative_counts = [int(row["negative_eigenvalue_count"])
                       for row in rows if "negative_eigenvalue_count" in row]
    if negative_counts:
        metrics["negative_eigenvalue_sample_count"] = int(
            sum(value > 0 for value in negative_counts)
        )
        metrics["negative_eigenvalue_total"] = int(sum(negative_counts))
    return metrics


def make_plots(output: Path, odom: list[dict], info: list[dict], local: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if odom:
        xyz = np.asarray([[r["x"], r["y"], r["z"]] for r in odom])
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(xyz[:, 0], xyz[:, 1], linewidth=1)
        ax.set(xlabel="x [m]", ylabel="y [m]", title="FAST-LIVO2 trajectory (XY)")
        ax.axis("equal"); ax.grid(True, alpha=.3)
        fig.tight_layout(); fig.savefig(output / "trajectory_xy.png", dpi=160); plt.close(fig)
        stamps = np.asarray([r["stamp_ns"] for r in odom], dtype=np.int64)
        time_s = (stamps - stamps[0]) / 1e9
        gaps = np.diff(stamps) / 1e9
        displacement = np.linalg.norm(xyz - xyz[0], axis=1)
        fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=False)
        axes[0].plot(time_s[1:], gaps); axes[0].set(ylabel="gap [s]", title="Odometry timing")
        axes[1].plot(time_s, displacement); axes[1].set(xlabel="time [s]", ylabel="from start [m]")
        for ax in axes: ax.grid(True, alpha=.3)
        fig.tight_layout(); fig.savefig(output / "timing_displacement.png", dpi=160); plt.close(fig)
    def evidence_plot(
        source: list[dict], eigen_keys: tuple[str, ...], diagnostic_keys: tuple[str, ...],
        prefix: str, title: str,
    ) -> None:
        if not source:
            return
        stamps = np.asarray([r["stamp_ns"] for r in source], dtype=np.int64)
        time_s = (stamps - stamps[0]) / 1e9
        keys = [key for key in eigen_keys if any(key in row for row in source)]
        if keys:
            fig, ax = plt.subplots(figsize=(8, 4.5))
            for key in keys:
                ax.plot(time_s, [r.get(key, math.nan) for r in source],
                        label=key, linewidth=1)
            ax.set(xlabel="time [s]", ylabel="eigenvalue", title=title)
            ax.grid(True, alpha=.3); ax.legend(fontsize=7)
            fig.tight_layout(); fig.savefig(output / f"{prefix}_eigenvalues.png", dpi=160)
            plt.close(fig)
        diagnostics = [key for key in diagnostic_keys
                       if any(key in row for row in source)]
        if diagnostics:
            fig, ax = plt.subplots(figsize=(8, 4.5))
            for key in diagnostics:
                ax.plot(time_s, [r.get(key, math.nan) for r in source], label=key)
            ax.set(xlabel="time [s]", title=f"{title}: diagnostics")
            ax.grid(True, alpha=.3); ax.legend(fontsize=7)
            fig.tight_layout(); fig.savefig(output / f"{prefix}_diagnostics.png", dpi=160)
            plt.close(fig)

    evidence_plot(
        info, ("eigen_min", "eigen_max"),
        ("condition", "mean_abs_residual_m", "symmetry_max_abs_error"),
        "measurement_information", "LiDAR measurement information",
    )
    evidence_plot(
        local,
        ("rotation_eigen_min", "rotation_eigen_max",
         "translation_eigen_min", "translation_eigen_max"),
        ("rotation_condition", "translation_condition"),
        "localizability_telemetry", "Localizability calibration telemetry",
    )


def _read_run_manifests(paths: list[Path]) -> dict[str, Any]:
    if not paths:
        return {"available": False, "reason": "no_run_manifest_provided"}
    runs: list[dict[str, Any]] = []
    repeat_groups: dict[tuple[str, str], list[float]] = {}
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        resource_rows = document.get("resource_samples", [])
        samples = [row for row in resource_rows
                   if "cpu_percent" in row and "rss_bytes" in row]
        sample_errors = [row for row in resource_rows if "error" in row]
        cpu = np.asarray([row["cpu_percent"] for row in samples], dtype=float)
        rss = np.asarray([row["rss_bytes"] for row in samples], dtype=float)
        duration_match = re.search(
            r"Duration:\s*([0-9]+(?:\.[0-9]+)?)s", document.get("bag_info", "")
        )
        input_duration = float(duration_match.group(1)) if duration_match else None
        elapsed = document.get("elapsed_s")
        analysis_path = path.parent / "analysis" / "metrics.json"
        analysis_doc: dict[str, Any] = {}
        repeat_metric = None
        if analysis_path.is_file():
            analysis_doc = json.loads(analysis_path.read_text(encoding="utf-8"))
            repeat_metric = analysis_doc.get("odometry", {}).get("path_length_m")
            if isinstance(repeat_metric, (int, float)):
                key = (str(document.get("dataset_id")), str(document.get("variant")))
                repeat_groups.setdefault(key, []).append(float(repeat_metric))

        input_analysis = document.get("input_analysis") or {}
        raw_input_counts = document.get("input_counts")
        if not isinstance(raw_input_counts, dict):
            raw_input_counts = input_analysis.get("topic_counts")
        if not isinstance(raw_input_counts, dict):
            raw_input_counts = input_analysis.get("topics")
        input_counts: dict[str, int] = {}
        if isinstance(raw_input_counts, dict):
            for topic, value in raw_input_counts.items():
                if isinstance(value, dict):
                    value = value.get("count", value.get("message_count"))
                if isinstance(value, (int, float)) and value >= 0:
                    input_counts[str(topic)] = int(value)
        reported_input_duration = input_analysis.get("duration_s")
        if isinstance(reported_input_duration, (int, float)) and reported_input_duration > 0:
            input_duration = float(reported_input_duration)

        output_counts = {
            ODOM_TOPIC: analysis_doc.get("odometry", {}).get("count"),
            INFO_TOPIC: analysis_doc.get("lidar_information", {}).get("count"),
            LOCAL_TOPIC: analysis_doc.get("localizability_telemetry", {}).get("count"),
        }
        lidar_topic = (document.get("sensor_topics") or {}).get("lidar")
        lidar_input_count = input_counts.get(lidar_topic) if lidar_topic else None
        coverage_ratios: dict[str, float | None] = {}
        for output_topic, output_count in output_counts.items():
            coverage_ratios[output_topic + "_per_lidar_input"] = (
                float(output_count / lidar_input_count)
                if isinstance(output_count, (int, float))
                and isinstance(lidar_input_count, int) and lidar_input_count > 0
                else None
            )
        output_duration = analysis_doc.get("odometry", {}).get("duration_s")
        coverage = {
            "available": bool(input_counts),
            "input_counts": input_counts or None,
            "output_counts": output_counts,
            "lidar_input_topic": lidar_topic,
            "output_per_lidar_input": coverage_ratios,
            "output_odometry_duration_over_input_duration": (
                float(output_duration / input_duration)
                if isinstance(output_duration, (int, float))
                and input_duration is not None and input_duration > 0
                else None
            ),
            "note": (
                None if input_counts else
                "input counts absent; input/output ratios are unavailable"
            ),
        }
        runs.append({
            "manifest": str(path),
            "dataset_id": document.get("dataset_id"),
            "variant": document.get("variant"),
            "repetition": document.get("repetition"),
            "elapsed_s": elapsed,
            "input_duration_s": input_duration,
            "wall_elapsed_over_input_duration": (
                float(elapsed / input_duration)
                if isinstance(elapsed, (int, float)) and input_duration and input_duration > 0
                else None
            ),
            "throughput_RTF": (
                float(input_duration / elapsed)
                if (isinstance(elapsed, (int, float)) and elapsed > 0
                    and input_duration is not None and input_duration > 0)
                else None
            ),
            "throughput_RTF_definition": "input_duration_s / wall_elapsed_s",
            "cpu_percent_mean": float(np.mean(cpu)) if cpu.size else None,
            "cpu_percent_p95": float(np.percentile(cpu, 95)) if cpu.size else None,
            "rss_bytes_max": int(np.max(rss)) if rss.size else None,
            "resource_sample_count": int(len(samples)),
            "resource_sample_error_count": int(len(sample_errors)),
            "requested_sensor_topics": document.get("sensor_topics", {}),
            "recorded_topics": document.get("recorded_topics", []),
            "input_output_coverage": coverage,
            "path_length_m_for_repeatability": repeat_metric,
        })
    groups = []
    for (dataset_id, variant), values in sorted(repeat_groups.items()):
        group: dict[str, Any] = {
            "dataset_id": dataset_id, "variant": variant,
            "available": len(values) >= 2, "sample_count": len(values),
            "metric": "odometry.path_length_m",
        }
        if len(values) >= 2:
            mean = float(np.mean(values))
            deviation = float(np.std(values, ddof=1))
            group.update({
                "mean": mean, "standard_deviation": deviation,
                "coefficient_of_variation": deviation / abs(mean) if mean else None,
                "min": float(np.min(values)), "max": float(np.max(values)),
            })
        groups.append(group)
    repeatability = {
        "available": any(group["available"] for group in groups),
        "sample_count": int(sum(group["sample_count"] for group in groups)),
        "metric": "odometry.path_length_m",
        "groups": groups,
        "grouping": "dataset_id_and_variant",
    }
    return {"available": True, "runs": runs, "repeatability": repeatability}


def analyze(
    odom: list[dict], info: list[dict], local: list[dict],
    gt: list[dict] | None = None, max_gt_delta_s: float = 0.05,
    rpe_delta: float = 1.0, rpe_delta_unit: str = "seconds",
    run_manifests: list[Path] | None = None,
    metric_cloud: list[dict] | None = None,
    transforms: list[dict] | None = None,
) -> dict[str, Any]:
    gt = gt or []
    metric_cloud = metric_cloud or []
    execution = _read_run_manifests(run_manifests or [])
    execution_runs = execution.get("runs", [])
    # During the current analysis metrics.json does not exist yet.  For the
    # single-manifest case, bind the already loaded output series explicitly.
    if len(execution_runs) == 1:
        coverage = execution_runs[0].get("input_output_coverage", {})
        current_output_counts = {
            ODOM_TOPIC: len(odom), INFO_TOPIC: len(info), LOCAL_TOPIC: len(local),
            METRIC_CLOUD_TOPIC: len(metric_cloud),
        }
        coverage["output_counts"] = current_output_counts
        lidar_topic = coverage.get("lidar_input_topic")
        input_counts = coverage.get("input_counts") or {}
        lidar_count = input_counts.get(lidar_topic) if lidar_topic else None
        coverage["output_per_lidar_input"] = {
            topic + "_per_lidar_input": (
                float(count / lidar_count)
                if isinstance(lidar_count, int) and lidar_count > 0 else None
            )
            for topic, count in current_output_counts.items()
        }
        input_duration = execution_runs[0].get("input_duration_s")
        output_duration = series_metrics(odom).get("duration_s")
        coverage["output_odometry_duration_over_input_duration"] = (
            float(output_duration / input_duration)
            if isinstance(output_duration, (int, float))
            and isinstance(input_duration, (int, float)) and input_duration > 0
            else None
        )
    manifest_input_counts_available = any(
        run.get("input_output_coverage", {}).get("available", False)
        for run in execution_runs
    )
    result = {
        "status": "MEASURED_NO_GT" if not gt else "MEASURED_WITH_GT",
        "threshold_policy": "descriptive_only_no_scientific_pass_fail_thresholds",
        "odometry": trajectory_metrics(odom),
        "lidar_information": eigen_metrics(info),
        "localizability_telemetry": eigen_metrics(local),
        "pose_metric_cloud_contract": pose_metric_cloud_contract(odom, metric_cloud),
        "pose_tf_contract": pose_tf_contract(odom, transforms or []),
        "input_output_coverage": {
            "output_counts": {
                ODOM_TOPIC: len(odom), INFO_TOPIC: len(info), LOCAL_TOPIC: len(local),
                METRIC_CLOUD_TOPIC: len(metric_cloud),
            },
            "lidar_information_per_odometry": (
                float(len(info) / len(odom)) if odom else None
            ),
            "localizability_per_odometry": (
                float(len(local) / len(odom)) if odom else None
            ),
            "input_counts_available": manifest_input_counts_available,
            "note": (
                "per-manifest input/output ratios are in execution.runs"
                if manifest_input_counts_available else
                "input counts absent; no input/output ratio is inferred"
            ),
        },
        "ground_truth": ground_truth_metrics(
            odom, gt, max_gt_delta_s, rpe_delta, rpe_delta_unit
        ),
        "execution": execution,
    }
    return result
def main() -> int:
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path)
    source.add_argument("--odom-csv", type=Path)
    parser.add_argument("--info-csv", type=Path)
    parser.add_argument("--localizability-csv", type=Path)
    parser.add_argument("--gt-csv", type=Path)
    parser.add_argument("--gt-topic")
    parser.add_argument("--max-gt-delta-s", type=float, default=0.05)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--rpe-delta-unit", choices=("seconds", "meters"), default="seconds")
    parser.add_argument("--run-manifest", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.bag:
        odom, info, local, metric_cloud, gt, transforms = read_bag(args.bag, args.gt_topic)
    else:
        odom = load_csv(args.odom_csv)
        info = load_csv(args.info_csv) if args.info_csv else []
        local = load_csv(args.localizability_csv) if args.localizability_csv else []
        metric_cloud = []
        transforms = []
        gt = load_csv(args.gt_csv) if args.gt_csv else []

    if not odom:
        raise RuntimeError(f"required odometry topic/data is absent: {ODOM_TOPIC}")
    # Preserve odometry publication order so timestamp regressions remain visible.
    gt = sorted(gt, key=lambda row: row["stamp_ns"])
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output / "odometry.csv", odom)
    write_csv(args.output / "lidar_information.csv", info)
    write_csv(args.output / "localizability.csv", local)
    write_csv(args.output / "metric_cloud.csv", metric_cloud)
    write_csv(args.output / "fast_tf.csv", transforms)
    summary = json_safe(analyze(
        odom, info, local, gt, args.max_gt_delta_s,
        args.rpe_delta, args.rpe_delta_unit, args.run_manifest,
        metric_cloud=metric_cloud, transforms=transforms,
    ))
    (args.output / "metrics.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (args.output / "metrics.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8"
    )
    make_plots(args.output, odom, info, local)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
