import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import aggregate_campaign as campaign


POLICY = yaml.safe_load((ROOT / "decision_policy.yaml").read_text())


def valid_input():
    return {"schema_version": 1, "topics": {"lidar": {
        "structural_violations": [],
        "content_sampling": {"structural_violations": {}},
        "header_timing": {"positive_delta_s": {"median": 0.1}},
    }}, "tf_audit": {"nonfinite_transform_count": 0},
        "coverage": {"by_role": {"lidar": {"first_ns": 0,
            "last_ns": 10_000_000_000, "duration_s": 10.0}}}}


def valid_manifest():
    return {"schema_version": 1, "cell_status": "VALID", "playback_completed": True,
            "effective_parameter_verification": {"status": "VALID"}, "postflight_error": None,
            "output_bag_info": "Files: output.db3", "analysis": {"returncode": 0},
            "crash_observed": False, "elapsed_s": 12.0, "exit_codes": {"node": -2},
            "resource_samples": [{"cpu_percent": 20.0, "rss_bytes": 100.0},
                                 {"cpu_percent": 30.0, "rss_bytes": 200.0}]}


def valid_metrics(gt=False):
    return {"odometry": {"count": 100, "finite_fraction": 1.0,
            "quaternion_invalid_count": 0, "strictly_monotonic": True,
            "duration_s": 9.8, "first_stamp_ns": 0, "last_stamp_ns": 9_800_000_000,
            "path_length_m": 20.0,
            "start_end_displacement_m": 2.0},
        "lidar_information": {"count": 100, "finite_fraction": 1.0,
            "matrix_shape_valid_false_count": 0,
            "symmetric_within_tolerance_false_count": 0,
            "psd_within_tolerance_false_count": 0,
            "tolerance_factor": {"min": 1.0e-9, "median": 1.0e-9,
                                 "max": 1.0e-9, "finite_count": 100}},
        "ground_truth": {"available": gt, "ate_translation_rmse_m": 0.2 if gt else None}}


def classify(inp=None, manifest=None, metrics=None, status="VALID"):
    return campaign.classify_cell({"status": status}, manifest or valid_manifest(),
        metrics or valid_metrics(), inp or valid_input(), POLICY)


def test_valid_no_gt_is_structural_pass_but_accuracy_not_established():
    axes = classify()
    assert all(axes[name]["status"] == "PASS" for name in campaign.STRUCTURAL_AXES)
    assert axes["REALTIME"]["status"] == "DESCRIPTIVE"
    assert axes["ACCURACY"]["status"] == "NOT_ESTABLISHED"


def test_input_duplicate_is_data_invalid():
    inp = valid_input()
    inp["topics"]["lidar"]["structural_violations"] = [
        {"kind": "duplicate_header_timestamp", "count": 2}]
    assert classify(inp=inp)["DATA_INTEGRITY"]["status"] == "DATA_INVALID"


def test_nonfinite_quaternion_and_non_psd_fail_numerical():
    metrics = valid_metrics(); metrics["odometry"]["quaternion_invalid_count"] = 1
    metrics["lidar_information"]["psd_within_tolerance_false_count"] = 1
    axes = classify(metrics=metrics)
    assert axes["NUMERICAL"]["status"] == "FAIL"
    assert len(axes["NUMERICAL"]["reasons"]) == 2


def test_tolerance_factor_mismatch_is_policy_configuration_invalid():
    metrics = valid_metrics()
    metrics["lidar_information"]["tolerance_factor"]["max"] = 2.0e-9
    axes = classify(metrics=metrics)
    assert axes["NUMERICAL"]["status"] == "POLICY_CONFIGURATION_INVALID"
    assert "do not match decision policy" in axes["NUMERICAL"]["reasons"][0]


def test_functional_requires_postflight_and_successful_analysis_process():
    manifest = valid_manifest()
    manifest["output_bag_info"] = None
    manifest["analysis"] = {"returncode": 2}
    axes = classify(manifest=manifest)
    assert axes["FUNCTIONAL"]["status"] == "FAIL"
    assert "output bag postflight evidence absent" in axes["FUNCTIONAL"]["reasons"]
    assert "analysis process returncode=2" in axes["FUNCTIONAL"]["reasons"]


def test_shutdown_is_separate_status():
    manifest = valid_manifest(); manifest["cell_status"] = "INVALID_SHUTDOWN"
    manifest["termination_events"] = [{"process": "node", "escalated": True,
        "reason": "stopped_after_sigterm", "signals": [{"signal": "SIGINT"},
        {"signal": "SIGTERM"}], "final_code": -15}]
    axes = classify(manifest=manifest, status="INVALID_SHUTDOWN")
    assert axes["SHUTDOWN"]["status"] == "INVALID_SHUTDOWN"
    assert axes["SHUTDOWN"]["evidence"]["escalated_processes"] == ["node"]
    assert "shutdown escalation required for: node" in axes["SHUTDOWN"]["reasons"]
    assert axes["NUMERICAL"]["status"] == "PASS"


def test_tail_coverage_uses_data_driven_tolerance():
    metrics = valid_metrics(); metrics["odometry"]["duration_s"] = 8.9
    metrics["odometry"]["last_stamp_ns"] = 8_900_000_000
    axes = classify(metrics=metrics)
    tail = axes["TEMPORAL"]["evidence"]["tail_coverage"]
    assert tail["tolerance_s"] == pytest.approx(1.0)
    assert tail["status"] == "FAIL"
    assert axes["TEMPORAL"]["status"] == "FAIL"


def test_tail_coverage_is_na_when_stamp_domains_are_not_comparable():
    metrics = valid_metrics()
    metrics["odometry"]["first_stamp_ns"] = 1_000_000_000_000
    metrics["odometry"]["last_stamp_ns"] = 1_009_800_000_000
    axes = classify(metrics=metrics)
    tail = axes["TEMPORAL"]["evidence"]["tail_coverage"]
    assert tail["status"] == "N/A"
    assert axes["TEMPORAL"]["status"] == "PASS"


def test_repeatability_n3_has_exploratory_bootstrap_ci():
    cells = []
    for repetition, length in enumerate((10.0, 10.5, 11.0), 1):
        metrics = valid_metrics(); metrics["odometry"]["path_length_m"] = length
        manifest = valid_manifest()
        cells.append({"dataset": "D", "variant": "LIO", "repetition": repetition,
                      "metrics": metrics, "manifest": manifest,
                      "resources": campaign.resource_statistics(manifest["resource_samples"])})
    group = campaign.repeatability(cells, POLICY)[0]
    value = group["statistics"]["path_length_m"]
    assert value["sample_count"] == 3
    assert value["bootstrap_mean_ci95"]["label"] == "exploratory"


def test_end_to_end_writes_machine_readable_tables_and_plot(tmp_path):
    run = tmp_path / "D" / "LIO" / "run_1"; (run / "analysis").mkdir(parents=True)
    (run / "run_manifest.json").write_text(json.dumps(valid_manifest()))
    (run / "analysis" / "metrics.json").write_text(json.dumps(valid_metrics()))
    inp = tmp_path / "_input_analysis" / "D"; inp.mkdir(parents=True)
    (inp / "input_integrity.json").write_text(json.dumps(valid_input()))
    summary = tmp_path / "validation_summary.json"
    summary.write_text(json.dumps({"cells": [{"dataset": "D", "variant": "LIO",
        "repetition": 1, "status": "VALID", "run_directory": str(run)}]}))
    policy = tmp_path / "policy.yaml"; policy.write_text(yaml.safe_dump(POLICY))
    output = tmp_path / "campaign"
    result = campaign.aggregate(summary, policy, output)
    assert result["verdict"]["status"] == "FUNCTIONALLY_VALID_ACCURACY_NOT_ESTABLISHED"
    for name in ("campaign_summary.json", "campaign_summary.yaml", "campaign_cells.csv",
                 "campaign_axes.csv", "campaign_repeatability.csv", "campaign_overview.png"):
        assert (output / name).is_file()
