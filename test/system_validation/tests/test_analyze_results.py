import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import analyze_results as analysis


FIXTURES = Path(__file__).parent / "fixtures"


def test_trajectory_metrics_closed_path():
    rows = analysis.load_csv(FIXTURES / "odom.csv")
    metrics = analysis.trajectory_metrics(rows)
    assert metrics["count"] == 4
    assert metrics["duration_s"] == pytest.approx(3.0)
    assert metrics["path_length_m"] == pytest.approx(2 + math.sqrt(2))
    assert metrics["start_end_displacement_m"] == pytest.approx(0)
    assert metrics["strictly_monotonic"] is True
    assert metrics["finite_fraction"] == 1.0


def test_ground_truth_sorted_and_aligned():
    odom = analysis.load_csv(FIXTURES / "odom.csv")
    gt = analysis.load_csv(FIXTURES / "gt.csv")
    metrics = analysis.ground_truth_metrics(odom, gt, 0.01)
    assert metrics["available"] is True
    assert metrics["matched_poses"] == 4
    assert metrics["ate_translation_rmse_m"] == pytest.approx(0, abs=1e-12)
    assert metrics["rpe_translation_rmse_m"] == pytest.approx(0, abs=1e-12)
    assert metrics["rpe_rotation_rmse_rad"] == pytest.approx(0, abs=1e-12)


def test_no_gt_never_claims_ate_or_pass():
    rows = analysis.load_csv(FIXTURES / "odom.csv")
    result = analysis.analyze(rows, [], [])
    assert result["status"] == "MEASURED_NO_GT"
    assert result["ground_truth"]["available"] is False
    assert "PASS" not in json.dumps(result)


def test_nonfinite_json_sanitization():
    value = {"nan": math.nan, "infinity": math.inf, "nested": [1, -math.inf]}
    safe = analysis.json_safe(value)
    assert safe == {"nan": None, "infinity": None, "nested": [1, None]}
    json.dumps(safe, allow_nan=False)


def test_non_monotonic_timestamps_are_reported():
    rows = analysis.load_csv(FIXTURES / "odom.csv")
    rows[2]["stamp_ns"] = rows[1]["stamp_ns"]
    metrics = analysis.series_metrics(rows)
    assert metrics["strictly_monotonic"] is False
    assert metrics["nonpositive_gap_count"] == 1


def test_pose_metric_cloud_contract_accepts_exact_cloud_subset():
    odom = [
        {"stamp_ns": stamp, "frame_id": "camera_init"}
        for stamp in (100, 150, 200, 250, 300)
    ]
    clouds = [
        {"stamp_ns": stamp, "frame_id": "camera_init", "width": 10,
         "height": 1, "data_size_bytes": 160}
        for stamp in (100, 200, 300)
    ]
    metrics = analysis.pose_metric_cloud_contract(odom, clouds)
    assert metrics["exact_pairing_complete"] is True
    assert metrics["exact_pair_count"] == 3
    assert metrics["odometry_without_metric_cloud_count"] == 2


def test_pose_metric_cloud_contract_rejects_unmatched_duplicate_and_empty():
    odom = [{"stamp_ns": 100, "frame_id": "camera_init"}]
    clouds = [
        {"stamp_ns": 200, "frame_id": "camera_init", "width": 0,
         "height": 1, "data_size_bytes": 0},
        {"stamp_ns": 200, "frame_id": "wrong", "width": 1,
         "height": 1, "data_size_bytes": 16},
    ]
    metrics = analysis.pose_metric_cloud_contract(odom, clouds)
    assert metrics["exact_pairing_complete"] is False
    assert metrics["unmatched_metric_cloud_count"] == 2
    assert metrics["duplicate_metric_cloud_stamp_count"] == 1
    assert metrics["empty_metric_cloud_count"] == 1


def _pose(stamp_ns, x, yaw=0.0):
    return {
        "stamp_ns": stamp_ns, "x": float(x), "y": 0.0, "z": 0.0,
        "qw": math.cos(yaw / 2), "qx": 0.0, "qy": 0.0,
        "qz": math.sin(yaw / 2), "finite": True,
    }


def test_gt_interpolation_reports_coverage_and_bracket_delta():
    gt = [_pose(0, 0), _pose(2_000_000_000, 2)]
    odom = [_pose(1_000_000_000, 1)]
    metrics = analysis.ground_truth_metrics(odom, gt, 1.1)
    assert metrics["available"] is False  # one match is insufficient for ATE/RPE
    assert metrics["association"]["matched_poses"] == 1
    assert metrics["association"]["coverage_fraction"] == pytest.approx(1.0)
    assert metrics["association"]["max_observed_bracket_delta_s"] == pytest.approx(1.0)


def test_se3_alignment_does_not_estimate_scale():
    gt = [_pose(i * 1_000_000_000, i * 2.0) for i in range(4)]
    odom = [_pose(i * 1_000_000_000, i) for i in range(4)]
    metrics = analysis.ground_truth_metrics(odom, gt, 0.01)
    assert metrics["alignment"] == "rigid_SE3_Horn_no_scale"
    assert metrics["alignment_scale"] == 1.0
    assert metrics["ate_translation_rmse_m"] > 0.0


def test_rpe_is_configurable_and_reports_achieved_delta_distribution():
    stamps = [0, 1_100_000_000, 2_400_000_000, 3_800_000_000]
    gt = [_pose(stamp, i, 0.0) for i, stamp in enumerate(stamps)]
    odom = [_pose(stamp, i, i * 0.1) for i, stamp in enumerate(stamps)]
    metrics = analysis.ground_truth_metrics(
        odom, gt, 0.01, rpe_delta=2.0, rpe_delta_unit="seconds"
    )
    definition = metrics["rpe_definition"]
    assert definition["pair_count"] == 2
    assert definition["requested_delta"] == 2.0
    assert definition["delta_unit"] == "seconds"
    assert definition["achieved_delta_min"] == pytest.approx(2.4)
    assert definition["achieved_delta_median"] == pytest.approx(2.55)
    assert definition["achieved_delta_mean"] == pytest.approx(2.55)
    assert definition["achieved_delta_max"] == pytest.approx(2.7)
    assert metrics["rpe_rotation_rmse_rad"] > 0.0


def test_rpe_achieved_delta_uses_ground_truth_arc_length_for_meters():
    positions = [0.0, 0.8, 2.3, 4.0]
    pairs = [(_pose(i, x), _pose(i, x)) for i, x in enumerate(positions)]
    selected = analysis._select_rpe_pairs(pairs, 2.0, "meters")
    assert [item[2] for item in selected] == pytest.approx([2.3, 3.2])


class _InformationMessage:
    available = True
    effective_features = 20
    mean_abs_point_to_plane_residual_m = 0.01

    def __init__(self, matrix):
        self.measurement_information = list(matrix.reshape(-1))


def test_information_matrix_reports_asymmetry_and_negative_eigenvalues():
    matrix = np.eye(6)
    matrix[0, 1] = 0.1
    matrix[2, 2] = -1.0
    row = analysis.info_row(_InformationMessage(matrix), 1)
    assert row["matrix_scale"] == pytest.approx(np.linalg.norm(matrix, ord=2))
    assert row["tolerance_factor"] == pytest.approx(1e-9)
    assert row["applied_tolerance"] == pytest.approx(
        1e-9 * max(1.0, row["matrix_scale"])
    )
    assert row["symmetric_within_tolerance"] is False
    assert row["psd_within_tolerance"] is False
    assert row["negative_eigenvalue_count"] == 1
    metrics = analysis.eigen_metrics([row])
    assert metrics["negative_eigenvalue_sample_count"] == 1
    assert metrics["symmetric_within_tolerance_false_count"] == 1
    assert metrics["psd_within_tolerance_false_count"] == 1
    assert metrics["matrix_scale"]["median"] == pytest.approx(row["matrix_scale"])


def test_information_matrix_tolerance_scales_with_spectral_norm():
    matrix = np.eye(6) * 1e12
    matrix[0, 1] = 500.0
    row = analysis.info_row(_InformationMessage(matrix), 1)
    assert row["matrix_scale"] == pytest.approx(1e12, rel=1e-8)
    assert row["applied_tolerance"] == pytest.approx(1e3, rel=1e-8)
    assert row["symmetry_max_abs_error"] == pytest.approx(500.0)
    assert row["symmetric_within_tolerance"] is True
    assert row["negative_eigenvalue_count"] == 0
    assert row["psd_within_tolerance"] is True


def test_trajectory_reports_invalid_quaternion_and_jump_candidate():
    rows = [_pose(0, 0), _pose(1, 0.1), _pose(2, 100)]
    rows[1]["qw"] = 0.0
    metrics = analysis.trajectory_metrics(rows)
    assert metrics["quaternion_invalid_count"] == 1
    assert metrics["translation_jump_candidate_count"] == 1


def test_run_manifest_resources_and_repeatability(tmp_path):
    manifests = []
    for repetition, length in ((1, 10.0), (2, 11.0)):
        root = tmp_path / f"run_{repetition}"
        (root / "analysis").mkdir(parents=True)
        manifest = root / "run_manifest.json"
        manifest.write_text(json.dumps({
            "dataset_id": "D", "variant": "LIO", "repetition": repetition,
            "elapsed_s": 20.0, "bag_info": "Duration: 99.0s",
            "sensor_topics": {"lidar": "/livox/lidar"},
            "input_counts": {"/livox/lidar": 100},
            "input_analysis": {"duration_s": 10.0},
            "resource_samples": [
                {"cpu_percent": 25.0, "rss_bytes": 1000},
                {"error": "process disappeared during shutdown"},
                {"cpu_percent": 50.0, "rss_bytes": 2000},
            ],
        }))
        (root / "analysis" / "metrics.json").write_text(json.dumps({
            "odometry": {"path_length_m": length, "count": 90, "duration_s": 9.0},
            "lidar_information": {"count": 90},
            "localizability_telemetry": {"count": 0},
        }))
        manifests.append(manifest)
    metrics = analysis._read_run_manifests(manifests)
    assert metrics["runs"][0]["wall_elapsed_over_input_duration"] == pytest.approx(2.0)
    assert metrics["runs"][0]["throughput_RTF"] == pytest.approx(0.5)
    assert metrics["runs"][0]["throughput_RTF_definition"] == (
        "input_duration_s / wall_elapsed_s"
    )
    assert metrics["runs"][0]["cpu_percent_mean"] == pytest.approx(37.5)
    assert metrics["runs"][0]["cpu_percent_p95"] == pytest.approx(48.75)
    assert metrics["runs"][0]["rss_bytes_max"] == 2000
    assert metrics["runs"][0]["resource_sample_count"] == 2
    assert metrics["runs"][0]["resource_sample_error_count"] == 1
    coverage = metrics["runs"][0]["input_output_coverage"]
    assert coverage["available"] is True
    assert coverage["input_counts"]["/livox/lidar"] == 100
    assert coverage["output_per_lidar_input"][
        analysis.ODOM_TOPIC + "_per_lidar_input"
    ] == pytest.approx(0.9)
    assert coverage["output_odometry_duration_over_input_duration"] == pytest.approx(0.9)
    assert metrics["repeatability"]["available"] is True
    assert metrics["repeatability"]["sample_count"] == 2


def test_missing_input_counts_remains_unavailable(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    manifest = root / "run_manifest.json"
    manifest.write_text(json.dumps({
        "dataset_id": "D", "variant": "LO", "elapsed_s": 1.0,
        "bag_info": "Duration: 1.0s", "resource_samples": [],
    }))
    metrics = analysis._read_run_manifests([manifest])
    coverage = metrics["runs"][0]["input_output_coverage"]
    assert coverage["available"] is False
    assert coverage["input_counts"] is None
    assert all(value is None for value in coverage["output_per_lidar_input"].values())


def test_current_analysis_binds_outputs_before_metrics_json_exists(tmp_path):
    manifest = tmp_path / "run_manifest.json"
    manifest.write_text(json.dumps({
        "dataset_id": "D", "variant": "LO", "elapsed_s": 2.0,
        "sensor_topics": {"lidar": "/livox/lidar"},
        "input_counts": {"/livox/lidar": 4},
        "input_analysis": {"duration_s": 3.0},
        "resource_samples": [],
    }))
    odom = [_pose(i * 1_000_000_000, i) for i in range(4)]
    metrics = analysis.analyze(odom, [], [], run_manifests=[manifest])
    coverage = metrics["execution"]["runs"][0]["input_output_coverage"]
    assert coverage["output_counts"][analysis.ODOM_TOPIC] == 4
    assert coverage["output_per_lidar_input"][
        analysis.ODOM_TOPIC + "_per_lidar_input"
    ] == pytest.approx(1.0)
    assert coverage["output_odometry_duration_over_input_duration"] == pytest.approx(1.0)



def fast_pose(stamp=100):
    return {**_pose(stamp, 1.23, 0.713), "frame_id": "camera_init", "child_frame_id": "aft_mapped"}


def test_exact_tf_contract_pass_and_ignores_other_edges():
    odom = [fast_pose(100), fast_pose(200)]
    transforms = [dict(row) for row in odom] + [{"frame_id": "map", "child_frame_id": "odom"}]
    result = analysis.pose_tf_contract(odom, transforms)
    assert result["exact_pairing_complete"] is True
    assert result["exact_pair_count"] == 2
    assert analysis.analyze(odom, [], [], transforms=transforms)["pose_tf_contract"] == result


@pytest.mark.parametrize("case", ["missing", "duplicate", "stamp", "extra", "odom_duplicate",
                                  "parent", "child", "odom_frame", "position", "quaternion", "signed_zero"])
def test_exact_tf_contract_rejects_defects(case):
    odom = [fast_pose()]
    transforms = [fast_pose()]
    if case == "missing": transforms = []
    elif case == "duplicate": transforms.append(fast_pose())
    elif case == "extra": transforms.append(fast_pose(101))
    elif case == "odom_duplicate": odom.append(fast_pose())
    elif case == "stamp": transforms[0]["stamp_ns"] += 1
    elif case == "parent": transforms[0]["frame_id"] = "map"
    elif case == "child": transforms[0]["child_frame_id"] = "base_link"
    elif case == "odom_frame": odom[0]["frame_id"] = "map"
    elif case == "position": transforms[0]["x"] = math.nextafter(odom[0]["x"], math.inf)
    elif case == "quaternion": transforms[0]["qw"] = math.nextafter(odom[0]["qw"], math.inf)
    elif case == "signed_zero": transforms[0]["y"] = -0.0
    assert analysis.pose_tf_contract(odom, transforms)["exact_pairing_complete"] is False


def test_exact_tf_uses_zero_header_stamp_never_bag_time():
    from types import SimpleNamespace as NS
    header = NS(stamp=NS(sec=0, nanosec=0), frame_id="camera_init")
    position = NS(x=1.0, y=0.0, z=-0.0)
    rotation = NS(x=0.0, y=0.0, z=0.0, w=1.0)
    msg = NS(header=header, child_frame_id="aft_mapped", pose=NS(pose=NS(position=position, orientation=rotation)))
    tf = NS(header=header, child_frame_id="aft_mapped", transform=NS(translation=position, rotation=rotation))
    odom = analysis.pose_row(msg, 999)
    assert odom["header_stamp_ns"] == 0
    assert analysis.pose_tf_contract([odom], [analysis.tf_row(tf)])["exact_pairing_complete"]


def test_bag_reader_deserializes_tfmessage_and_retains_only_fast_edge(monkeypatch, tmp_path):
    from types import SimpleNamespace as NS
    header = NS(stamp=NS(sec=12, nanosec=345), frame_id="camera_init")
    position = NS(x=1.0, y=2.0, z=3.0)
    rotation = NS(x=0.0, y=0.0, z=0.0, w=1.0)
    fast = NS(header=header, child_frame_id="aft_mapped", transform=NS(translation=position, rotation=rotation))
    unrelated = NS(header=NS(frame_id="map"), child_frame_id="odom")
    odom = NS(header=header, child_frame_id="aft_mapped", pose=NS(pose=NS(position=position, orientation=rotation)))
    types = {analysis.ODOM_TOPIC: "nav_msgs/msg/Odometry", "/tf": "tf2_msgs/msg/TFMessage"}
    pending = [("/tf", NS(transforms=[unrelated, fast]), 999), (analysis.ODOM_TOPIC, odom, 888)]
    class Reader:
        def open(self, *args): pass
        def get_all_topics_and_types(self): return [NS(name=k, type=v) for k, v in types.items()]
        def has_next(self): return bool(pending)
        def read_next(self): return pending.pop(0)
    decoded = []
    def deserialize(data, message_type):
        decoded.append(message_type)
        return data
    monkeypatch.setitem(sys.modules, "rosbag2_py", NS(SequentialReader=Reader, StorageOptions=lambda **kw: kw, ConverterOptions=lambda *a: a))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", NS(deserialize_message=deserialize))
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py.utilities", NS(get_message=lambda value: value))
    poses, _, _, _, _, transforms = analysis.read_bag(tmp_path)
    assert decoded == ["tf2_msgs/msg/TFMessage", "nav_msgs/msg/Odometry"]
    assert len(transforms) == 1
    assert analysis.pose_tf_contract(poses, transforms)["exact_pairing_complete"]
