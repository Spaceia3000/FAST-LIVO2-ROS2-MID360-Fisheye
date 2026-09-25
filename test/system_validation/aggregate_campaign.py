#!/usr/bin/env python3
"""Aggregate a FAST-LIVO2 validation campaign without inventing quality thresholds.

This module turns cell-level runner/analyzer evidence into campaign-level tables,
repeatability summaries and a deliberately limited causal verdict.  Structural
requirements are pass/fail; performance quantities without an externally justified
criterion remain descriptive.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

AXES = ("DATA_INTEGRITY", "FUNCTIONAL", "NUMERICAL", "TEMPORAL",
        "SHUTDOWN", "REALTIME", "ACCURACY")
STRUCTURAL_AXES = ("DATA_INTEGRITY", "FUNCTIONAL", "NUMERICAL", "TEMPORAL", "SHUTDOWN")
STATUS_ORDER = {
    "DATA_INVALID": 80, "INVALID_SHUTDOWN": 70, "FAIL": 60,
    "POLICY_CONFIGURATION_INVALID": 75,
    "NOT_ESTABLISHED": 30, "MEASURED": 20, "DESCRIPTIVE": 10,
    "N/A": 5, "PASS": 0,
}
DEFAULT_POLICY = Path(__file__).with_name("decision_policy.yaml")


def load_document(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(encoding="utf-8") as stream:
        if path.suffix.lower() in {".yaml", ".yml"}:
            value = yaml.safe_load(stream)
        else:
            value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"document must be an object: {path}")
    return value


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def nested(document: dict[str, Any], dotted: str, default: Any = None) -> Any:
    value: Any = document
    for part in dotted.split("."):
        if isinstance(value, dict) and part in value:
            value = value[part]
        elif isinstance(value, list) and part.isdigit() and int(part) < len(value):
            value = value[int(part)]
        else:
            return default
    return value


def result(status: str, reasons: Iterable[str], evidence: dict[str, Any] | None = None) -> dict[str, Any]:
    return {"status": status, "reasons": list(reasons), "evidence": evidence or {}}


def worst_status(values: Iterable[str]) -> str:
    values = list(values)
    return max(values, key=lambda item: STATUS_ORDER.get(item, 50)) if values else "N/A"


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def resource_statistics(samples: list[dict[str, Any]]) -> dict[str, Any]:
    fields = ("cpu_percent", "rss_bytes")
    output: dict[str, Any] = {"sample_count": len(samples)}
    for field in fields:
        values = np.asarray([row[field] for row in samples if _finite_number(row.get(field))], dtype=float)
        if not values.size:
            output[field] = {"count": 0}
            continue
        mean = float(np.mean(values)); std = float(np.std(values, ddof=1)) if values.size > 1 else 0.0
        output[field] = {
            "count": int(values.size), "mean": mean, "standard_deviation": std,
            "coefficient_of_variation": std / abs(mean) if mean else None,
            "min": float(np.min(values)), "max": float(np.max(values)),
        }
    return output


def input_violations(input_doc: dict[str, Any]) -> list[str]:
    violations: list[str] = []
    invalid_timing_kinds = {
        "duplicate_bag_timestamp", "nonmonotonic_bag_timestamp",
        "duplicate_header_timestamp", "nonmonotonic_header_timestamp",
    }
    for role, topic in (input_doc.get("topics") or {}).items():
        for item in topic.get("structural_violations") or []:
            if item.get("kind") in invalid_timing_kinds and int(item.get("count", 0) or 0) > 0:
                violations.append(f"{role}:{item.get('kind')}={item.get('count')}")
        sampled = topic.get("content_sampling") or {}
        for kind, count in (sampled.get("structural_violations") or {}).items():
            if int(count or 0) > 0:
                violations.append(f"{role}:{kind}={count}")
    tf = input_doc.get("tf_audit") or {}
    if int(tf.get("nonfinite_transform_count", 0) or 0) > 0:
        violations.append(f"tf:nonfinite_transform_count={tf['nonfinite_transform_count']}")
    return violations


def lidar_period(input_doc: dict[str, Any]) -> float | None:
    lidar = (input_doc.get("topics") or {}).get("lidar") or {}
    for clock in ("header_timing", "bag_timing"):
        value = nested(lidar, f"{clock}.positive_delta_s.median")
        if _finite_number(value) and value > 0:
            return float(value)
    return None


def required_coverage_roles(variant: str) -> tuple[str, ...]:
    return {
        "LO": ("lidar",),
        "LIO": ("lidar", "imu"),
        "LIVO": ("lidar", "imu", "image"),
    }.get(str(variant).upper(), ("lidar",))


def effective_input_interval(
    input_doc: dict[str, Any], variant: str
) -> tuple[int | None, int | None, tuple[str, ...], list[str]]:
    roles = required_coverage_roles(variant)
    by_role = nested(input_doc, "coverage.by_role", {}) or {}
    intervals: list[tuple[int, int]] = []
    missing: list[str] = []
    for role in roles:
        row = by_role.get(role) or {}
        first, last = row.get("first_ns"), row.get("last_ns")
        if not (_finite_number(first) and _finite_number(last)):
            missing.append(role)
            continue
        intervals.append((int(first), int(last)))
    if missing or len(intervals) != len(roles):
        return None, None, roles, missing
    first = max(value[0] for value in intervals)
    last = min(value[1] for value in intervals)
    if last < first:
        return first, last, roles, ["no_common_overlap"]
    return first, last, roles, []


def classify_cell(cell: dict[str, Any], manifest: dict[str, Any] | None,
                  metrics: dict[str, Any] | None, input_doc: dict[str, Any] | None,
                  policy: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Classify one scheduled cell. Missing evidence fails only the relevant axis."""
    runner_status = str(cell.get("status", "INVALID"))
    m = manifest or {}; x = metrics or {}; inp = input_doc or {}
    violations = input_violations(inp) if input_doc else ["input_integrity.json absent"]
    data_axis = result("DATA_INVALID", violations) if violations else result("PASS", ["no structural input violation observed"])

    functional_failures: list[str] = []
    if not manifest:
        functional_failures.append("run_manifest.json absent")
    if runner_status not in {"VALID", "INVALID_SHUTDOWN"}:
        functional_failures.append(f"validation_summary status={runner_status}")
    if m and m.get("schema_version") != policy["schema_version"]:
        functional_failures.append("run manifest schema version mismatch")
    if m and not m.get("playback_completed", False):
        functional_failures.append("playback did not complete")
    parameter_check = m.get("effective_parameter_verification") if m else None
    if not parameter_check:
        functional_failures.append("effective parameter verification absent")
    elif parameter_check.get("status") != "VALID":
        functional_failures.append("effective parameter verification is not VALID")
    if input_doc and input_doc.get("schema_version") != policy["schema_version"]:
        functional_failures.append("input-integrity schema version mismatch")
    if m and m.get("postflight_error"):
        functional_failures.append(f"postflight: {m['postflight_error']}")
    if m and not m.get("output_bag_info"):
        functional_failures.append("output bag postflight evidence absent")
    analysis_record = m.get("analysis") if m else None
    if not isinstance(analysis_record, dict):
        functional_failures.append("analysis process evidence absent")
    elif analysis_record.get("returncode") != 0:
        functional_failures.append(
            f"analysis process returncode={analysis_record.get('returncode')}")
    odom = x.get("odometry") or {}
    if not metrics or int(odom.get("count", 0) or 0) <= 0:
        functional_failures.append("required odometry output absent")
    functional = result("FAIL", functional_failures) if functional_failures else result("PASS", ["process, parameters, schema, postflight and odometry present"])

    numerical_failures: list[str] = []
    numerical_policy_failures: list[str] = []
    if odom and float(odom.get("finite_fraction", 0.0) or 0.0) != 1.0:
        numerical_failures.append("odometry finite_fraction != 1")
    if odom and "quaternion_invalid_count" not in odom:
        numerical_failures.append("odometry quaternion validity evidence absent")
    elif odom and int(odom.get("quaternion_invalid_count", 0) or 0) != 0:
        numerical_failures.append("invalid odometry quaternion observed")
    info = x.get("lidar_information") or {}
    if int(info.get("count", 0) or 0) <= 0:
        numerical_failures.append("required LiDAR information absent")
    else:
        if float(info.get("finite_fraction", 0.0) or 0.0) != 1.0:
            numerical_failures.append("LiDAR information finite_fraction != 1")
        required_matrix_fields = ("matrix_shape_valid_false_count",
                                  "symmetric_within_tolerance_false_count",
                                  "psd_within_tolerance_false_count")
        missing_matrix_fields = [name for name in required_matrix_fields if name not in info]
        if missing_matrix_fields:
            numerical_failures.append("LiDAR matrix validation evidence absent: " + ",".join(missing_matrix_fields))
        if int(info.get("matrix_shape_valid_false_count", 0) or 0):
            numerical_failures.append("LiDAR information matrix shape invalid")
        if int(info.get("symmetric_within_tolerance_false_count", 0) or 0):
            numerical_failures.append("LiDAR information matrix not symmetric within tolerance")
        if int(info.get("psd_within_tolerance_false_count", 0) or 0):
            numerical_failures.append("LiDAR information matrix not PSD within tolerance")
        tolerance_summary = info.get("tolerance_factor")
        expected_factor = float(
            policy["numerical"]["lidar_information_tolerance_relative_factor"])
        if not isinstance(tolerance_summary, dict):
            numerical_policy_failures.append(
                "LiDAR tolerance_factor evidence absent")
        else:
            observed_min = tolerance_summary.get("min")
            observed_max = tolerance_summary.get("max")
            if not (_finite_number(observed_min) and _finite_number(observed_max)):
                numerical_policy_failures.append(
                    "LiDAR tolerance_factor min/max are absent or non-finite")
            elif (float(observed_min) != expected_factor
                  or float(observed_max) != expected_factor):
                numerical_policy_failures.append(
                    "LiDAR tolerance_factor min/max do not match decision policy")
    numerical_evidence = {
        "policy_tolerance_relative_factor": policy["numerical"]["lidar_information_tolerance_relative_factor"],
        "observed_tolerance_factor": info.get("tolerance_factor"),
    }
    if numerical_policy_failures:
        numerical = result("POLICY_CONFIGURATION_INVALID",
                           numerical_policy_failures + numerical_failures,
                           numerical_evidence)
    elif numerical_failures:
        numerical = result("FAIL", numerical_failures, numerical_evidence)
    else:
        numerical = result("PASS", ["finite outputs, valid quaternion, valid LiDAR matrices and matching tolerance factor"], numerical_evidence)

    temporal_failures: list[str] = []
    if odom and not bool(odom.get("strictly_monotonic", False)):
        temporal_failures.append("odometry timestamps are not strictly monotonic")
    tail_evidence: dict[str, Any] = {}
    period = lidar_period(inp) if input_doc else None
    variant = str(cell.get("variant", "LO")).upper()
    input_first, input_last, required_roles, missing_roles = (
        effective_input_interval(inp, variant) if inp
        else (None, None, required_coverage_roles(variant), ["input_integrity_absent"])
    )
    output_first = odom.get("first_stamp_ns")
    output_last = odom.get("last_stamp_ns")
    if missing_roles:
        temporal_failures.append(
            "required sensor coverage unavailable for "
            + variant + ": " + ",".join(missing_roles))
        tail_evidence = {
            "available": False,
            "status": "FAIL",
            "variant": variant,
            "required_roles": list(required_roles),
            "missing_roles": missing_roles,
            "policy": "tail coverage is evaluated on the common interval of sensors required by the mode",
        }
    elif all(_finite_number(v) for v in (period, input_first, input_last, output_first, output_last)):
        tolerance = max(3.0 * float(period), float(policy["temporal"]["tail_minimum_tolerance_s"]))
        start_delta = (float(output_first) - float(input_first)) / 1e9
        input_duration = (float(input_last) - float(input_first)) / 1e9
        output_duration = (float(output_last) - float(output_first)) / 1e9
        clocks_comparable = (
            float(input_first) - tolerance * 1e9
            <= float(output_first)
            <= float(input_last) + tolerance * 1e9
        )
        deficit = (float(input_last) - float(output_last)) / 1e9 if clocks_comparable else None
        tail_evidence = {
            "available": clocks_comparable,
            "status": ("FAIL" if clocks_comparable and deficit is not None and deficit > tolerance
                       else "PASS" if clocks_comparable else "N/A"),
            "variant": variant,
            "required_roles": list(required_roles),
            "input_first_ns": int(input_first),
            "input_last_ns": int(input_last),
            "lidar_last_ns": nested(inp, "coverage.by_role.lidar.last_ns"),
            "output_last_ns": int(output_last),
            "output_minus_input_start_s": start_delta,
            "input_duration_s": input_duration,
            "output_duration_s": output_duration,
            "deficit_s": deficit,
            "median_lidar_period_s": period,
            "tolerance_s": tolerance,
            "policy": (
                "max(3*median_lidar_period, tail_minimum_tolerance_s) "
                "over the common interval of sensors required by the mode"
            ),
            "note": None if clocks_comparable else "N/A: input/output stamp domains are not comparable",
        }
        if clocks_comparable and deficit is not None and deficit > tolerance:
            temporal_failures.append("output tail coverage deficit exceeds data-driven tolerance")
    else:
        tail_evidence = {
            "available": False,
            "status": "N/A",
            "variant": variant,
            "required_roles": list(required_roles),
            "reason": "endpoint stamps or median LiDAR period unavailable",
        }
    temporal = result("FAIL", temporal_failures, {"tail_coverage": tail_evidence}) if temporal_failures else result("PASS", ["strict timestamps; tail coverage passes or is unavailable"], {"tail_coverage": tail_evidence})

    shutdown_status = str(m.get("cell_status", runner_status))
    termination_events = m.get("termination_events") or []
    escalated = sorted({str(event.get("process")) for event in termination_events
                        if event.get("process") in {"node", "record"}
                        and event.get("escalated")})
    shutdown_evidence = {"exit_codes": m.get("exit_codes"),
                         "termination_events": termination_events,
                         "escalated_processes": escalated}
    shutdown_reasons: list[str] = []
    if escalated:
        shutdown_reasons.append("shutdown escalation required for: " + ", ".join(escalated))
    event_reasons = [f"{event.get('process')}:{event.get('reason')}"
                     for event in termination_events if event.get("reason")]
    if event_reasons:
        shutdown_reasons.append("termination events: " + ", ".join(event_reasons))
    if shutdown_status == "INVALID_SHUTDOWN" or cell.get("status") == "INVALID_SHUTDOWN" or escalated:
        shutdown_reasons.insert(0, "unexpected or escalated node/recorder shutdown")
        shutdown = result("INVALID_SHUTDOWN", shutdown_reasons, shutdown_evidence)
    elif not manifest:
        shutdown = result("FAIL", ["shutdown evidence absent"])
    elif m.get("crash_observed"):
        shutdown = result("FAIL", ["crash observed outside classified shutdown", *shutdown_reasons], shutdown_evidence)
    else:
        shutdown = result("PASS", ["no unexpected shutdown observed", *shutdown_reasons], shutdown_evidence)

    realtime = result("DESCRIPTIVE", ["no predeclared hardware-normalized realtime acceptance criterion"], {
        "elapsed_s": m.get("elapsed_s"), "resources": resource_statistics(m.get("resource_samples") or []),
        "throughput_RTF": nested(x, "execution.runs.0.throughput_RTF"),
    })
    gt = x.get("ground_truth") or {}
    accuracy = result("NOT_ESTABLISHED", ["ground truth unavailable; no accuracy claim permitted"]) if not gt.get("available") else result("MEASURED", ["ground-truth metrics measured; no universal acceptance threshold imposed"], gt)
    return {"DATA_INTEGRITY": data_axis, "FUNCTIONAL": functional,
            "NUMERICAL": numerical, "TEMPORAL": temporal, "SHUTDOWN": shutdown,
            "REALTIME": realtime, "ACCURACY": accuracy}


def stats(values: list[float], seed: int, bootstrap_samples: int,
          exploratory_n: int) -> dict[str, Any]:
    data = np.asarray(values, dtype=float); n = int(data.size)
    if not n:
        return {"sample_count": 0, "available": False}
    mean = float(np.mean(data)); std = float(np.std(data, ddof=1)) if n > 1 else 0.0
    out: dict[str, Any] = {"sample_count": n, "available": n >= 2,
        "mean": mean, "standard_deviation": std,
        "coefficient_of_variation": std / abs(mean) if mean else None,
        "min": float(np.min(data)), "max": float(np.max(data))}
    if n >= exploratory_n:
        rng = np.random.default_rng(seed)
        means = np.mean(rng.choice(data, size=(bootstrap_samples, n), replace=True), axis=1)
        out["bootstrap_mean_ci95"] = {"low": float(np.quantile(means, .025)),
            "high": float(np.quantile(means, .975)), "samples": bootstrap_samples,
            "label": "exploratory" if n == exploratory_n else "bootstrap",
            "note": "uncertainty summary, not an acceptance criterion"}
    else:
        out["bootstrap_mean_ci95"] = None
        out["ci_reason"] = f"n<{exploratory_n}"
    return out


def repeatability(cells: list[dict[str, Any]], policy: dict[str, Any]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for cell in cells:
        if cell.get("metrics"):
            grouped[(str(cell["dataset"]), str(cell["variant"]))].append(cell)
    fields = {
        "path_length_m": "odometry.path_length_m", "duration_s": "odometry.duration_s",
        "odometry_count": "odometry.count",
        "endpoint_proxy_m": "odometry.start_end_displacement_m",
        "elapsed_s": "manifest.elapsed_s",
        "cpu_mean_percent": "resources.cpu_percent.mean",
        "rss_max_bytes": "resources.rss_bytes.max",
    }
    groups: list[dict[str, Any]] = []
    for index, ((dataset, variant), group) in enumerate(sorted(grouped.items())):
        row: dict[str, Any] = {"dataset": dataset, "variant": variant,
                              "run_count": len(group), "statistics": {}}
        for name, dotted in fields.items():
            values: list[float] = []
            for item in group:
                source = {"manifest": item.get("manifest") or {}, "resources": item.get("resources") or {},
                          **(item.get("metrics") or {})}
                value = nested(source, dotted)
                if _finite_number(value): values.append(float(value))
            row["statistics"][name] = stats(values, policy["repeatability"]["bootstrap_seed"] + index,
                policy["repeatability"]["bootstrap_samples"], policy["repeatability"]["exploratory_ci_min_n"])
        groups.append(row)
    return groups


def global_verdict(cells: list[dict[str, Any]]) -> dict[str, Any]:
    if not cells:
        return {"status": "NOT_EXECUTED", "claim_scope": "no campaign cells found"}
    axis = {name: worst_status(cell["axes"][name]["status"] for cell in cells) for name in AXES}
    if axis["DATA_INTEGRITY"] == "DATA_INVALID":
        status = "DATA_INVALID"
        cause = "At least one input violates structural integrity; algorithm conclusions are blocked for that cell."
    elif axis["NUMERICAL"] == "POLICY_CONFIGURATION_INVALID":
        status = "POLICY_CONFIGURATION_INVALID"
        cause = "The observed numerical tolerance factor is absent or inconsistent with the pre-specified policy."
    elif axis["SHUTDOWN"] == "INVALID_SHUTDOWN":
        status = "INVALID_SHUTDOWN"
        cause = "At least one completed playback has an unexpected shutdown; lifecycle robustness is not closed."
    elif any(axis[name] == "FAIL" for name in STRUCTURAL_AXES):
        status = "NOT_VALIDATED"
        cause = "At least one pre-specified structural requirement failed."
    elif axis["ACCURACY"] == "NOT_ESTABLISHED":
        status = "FUNCTIONALLY_VALID_ACCURACY_NOT_ESTABLISHED"
        cause = "Structural campaign gates passed, but no ground truth supports an accuracy claim."
    else:
        status = "STRUCTURALLY_VALID_ACCURACY_MEASURED"
        cause = "Structural gates passed and ground-truth metrics were measured without imposing a universal threshold."
    return {"status": status, "axis_status": axis, "causal_summary": cause,
            "claim_scope": "limited to listed datasets, variants, repetitions, software commit, parameters and machine",
            "no_universal_claim": True}


def discover_cell(summary_path: Path, cell: dict[str, Any], input_cache: dict[str, dict[str, Any] | None]) -> dict[str, Any]:
    run_dir = Path(cell["run_directory"]) if cell.get("run_directory") else None
    if run_dir and not run_dir.is_absolute(): run_dir = (summary_path.parent / run_dir).resolve()
    manifest_path = run_dir / "run_manifest.json" if run_dir else None
    metrics_path = run_dir / "analysis" / "metrics.json" if run_dir else None
    dataset = str(cell.get("dataset")); input_path = summary_path.parent / "_input_analysis" / dataset / "input_integrity.json"
    if dataset not in input_cache:
        input_cache[dataset] = load_document(input_path) if input_path.is_file() else None
    manifest = load_document(manifest_path) if manifest_path and manifest_path.is_file() else None
    metrics = load_document(metrics_path) if metrics_path and metrics_path.is_file() else None
    resources = resource_statistics((manifest or {}).get("resource_samples") or [])
    return {**cell, "run_directory": str(run_dir) if run_dir else None,
            "manifest_path": str(manifest_path) if manifest_path else None,
            "metrics_path": str(metrics_path) if metrics_path else None,
            "input_integrity_path": str(input_path), "manifest": manifest,
            "metrics": metrics, "input_integrity": input_cache[dataset], "resources": resources}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row}) if rows else ["no_rows"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(rows)


def make_plot(path: Path, cells: list[dict[str, Any]], repetitions: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    counts = Counter(item["axes"][axis]["status"] for item in cells for axis in AXES)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    labels = sorted(counts); axes[0].bar(labels, [counts[k] for k in labels], color="#4472C4")
    axes[0].set(title="Campaign axis outcomes", ylabel="cell-axis count"); axes[0].tick_params(axis="x", rotation=30)
    for group in repetitions:
        metric = group["statistics"]["path_length_m"]
        if metric.get("sample_count", 0):
            axes[1].errorbar(f"{group['dataset']}\n{group['variant']}", metric["mean"],
                             yerr=metric["standard_deviation"], fmt="o", capsize=4)
    axes[1].set(title="Path length repeatability (mean +/- sample SD)", ylabel="path length [m]")
    axes[1].tick_params(axis="x", rotation=45); axes[1].grid(True, alpha=.3)
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


def aggregate(summary_path: Path, policy_path: Path, output: Path) -> dict[str, Any]:
    summary = load_document(summary_path); policy = load_document(policy_path)
    if policy.get("policy_schema_version") != 1: raise ValueError("unsupported policy schema")
    input_cache: dict[str, dict[str, Any] | None] = {}
    cells: list[dict[str, Any]] = []
    for raw in summary.get("cells") or []:
        cell = discover_cell(summary_path, raw, input_cache)
        cell["axes"] = classify_cell(cell, cell["manifest"], cell["metrics"], cell["input_integrity"], policy)
        cells.append(cell)
    reps = repeatability(cells, policy)
    campaign = {"schema_version": 1, "policy": policy, "validation_summary": str(summary_path.resolve()),
                "cell_count": len(cells), "cells": cells, "repeatability": reps,
                "verdict": global_verdict(cells),
                "descriptive_only": ["gaps", "synchronization", "residual", "condition_number", "resource_usage", "throughput_RTF"]}
    safe = json_safe(campaign); output.mkdir(parents=True, exist_ok=True)
    (output / "campaign_summary.json").write_text(json.dumps(safe, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "campaign_summary.yaml").write_text(yaml.safe_dump(safe, sort_keys=False), encoding="utf-8")
    flat = [{"dataset": c.get("dataset"), "variant": c.get("variant"), "repetition": c.get("repetition"),
             "runner_status": c.get("status"), **{axis: c["axes"][axis]["status"] for axis in AXES}} for c in cells]
    write_csv(output / "campaign_cells.csv", flat)
    axis_rows = [{"dataset": c.get("dataset"), "variant": c.get("variant"), "repetition": c.get("repetition"),
                  "axis": axis, "status": c["axes"][axis]["status"],
                  "reasons": " | ".join(c["axes"][axis]["reasons"])} for c in cells for axis in AXES]
    write_csv(output / "campaign_axes.csv", axis_rows)
    rep_rows = []
    for group in reps:
        for metric, value in group["statistics"].items():
            rep_rows.append({"dataset": group["dataset"], "variant": group["variant"], "metric": metric,
                             **{k: v for k, v in value.items() if not isinstance(v, dict)},
                             "bootstrap_ci95": json.dumps(value.get("bootstrap_mean_ci95"))})
    write_csv(output / "campaign_repeatability.csv", rep_rows)
    make_plot(output / "campaign_overview.png", cells, reps)
    return safe


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-summary", type=Path, required=True)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); aggregate(args.validation_summary, args.policy, args.output); return 0


if __name__ == "__main__":
    raise SystemExit(main())
