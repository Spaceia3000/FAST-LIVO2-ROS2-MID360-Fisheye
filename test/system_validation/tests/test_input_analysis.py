from pathlib import Path
import csv
import math
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_input import (image_sample, one_to_one_nearest, robust_summary,
                           synchronization_metrics, timing_metrics, topic_metrics)


def _rows(stamps):
    return [{"bag_stamp_ns": stamp, "header_stamp_ns": stamp, "frame_id": "sensor"}
            for stamp in stamps]


def test_input_timing_exposes_duplicates_regressions_and_robust_jitter():
    rows = _rows([0, 100_000_000, 100_000_000, 90_000_000, 200_000_000])
    result = timing_metrics(rows, "bag_stamp_ns")
    assert result["duplicate_count"] == 1
    assert result["regression_count"] == 1
    assert result["strictly_monotonic"] is False
    assert result["jitter_mad_s"] is not None


def test_input_regression_separates_endpoint_duration_span_and_rate_bias():
    rows = _rows([100_000_000, 300_000_000, 50_000_000])
    result = timing_metrics(rows, "bag_stamp_ns")
    assert math.isclose(result["endpoint_duration_s"], -0.05)
    assert math.isclose(result["span_s"], 0.25)
    assert result["rate_from_endpoints_hz"] is None
    assert result["rate_interpretation"] == "not_available_nonmonotonic_or_duplicate_endpoint_bias"


def test_input_topic_metrics_reports_missing_header_and_frame():
    rows = [{"bag_stamp_ns": 10, "header_stamp_ns": None, "frame_id": "lidar"},
            {"bag_stamp_ns": 20, "header_stamp_ns": 18, "frame_id": "lidar"}]
    result = topic_metrics(rows)
    assert result["frames"] == ["lidar"]
    assert result["header_coverage_fraction"] == 0.5
    assert {item["kind"] for item in result["structural_violations"]} == {"missing_header_timestamp"}


def test_input_one_to_one_pairing_never_reuses_target():
    pairs = one_to_one_nearest([0, 10, 20], [1, 19])
    assert pairs == [(0, 0, 1), (1, 1, 9)]
    assert len({target for _, target, _ in pairs}) == len(pairs)


def test_input_sync_reports_offset_and_drift_without_pass_fail():
    reference = _rows([0, 1_000_000_000, 2_000_000_000])
    target = _rows([10_000_000, 1_020_000_000, 2_030_000_000])
    result, pairs = synchronization_metrics(reference, target, "lidar", "imu", None)
    assert result["matched_count"] == 3
    assert math.isclose(result["linear_drift_s_per_s"], 0.01)
    assert len(pairs) == 3


def test_input_raw_image_structural_and_photometric_metrics():
    data = bytes([0, 10, 20, 30, 40, 255, 60, 70, 80])
    msg = SimpleNamespace(height=3, width=3, encoding="mono8", step=3, data=data)
    result = image_sample(msg)
    assert result["step_valid"] and result["data_length_valid"]
    assert result["brightness_mean"] is not None
    assert result["laplacian_variance"] is not None


def test_input_fixture_has_expected_descriptive_summary():
    fixture = Path(__file__).parent / "fixtures" / "input_timing.csv"
    with fixture.open(newline="", encoding="utf-8") as stream:
        values = [float(row["delta_s"]) for row in csv.DictReader(stream)]
    summary = robust_summary(values)
    assert summary["count"] == 5
    assert np.isclose(summary["median"], 0.1)
