"""Offline contract tests for run_validation.py. No ROS process or bag is executed."""
import hashlib
import importlib.util
import json
import signal
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("run_validation", ROOT / "run_validation.py")
runner = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(runner)


def load_schema():
    return json.loads((ROOT / "dataset_manifest.schema.json").read_text(encoding="utf-8"))


def minimal_dataset(**overrides):
    dataset = {
        "id": "sample",
        "enabled": True,
        "purpose": "validation",
        "data_role": "benchmark",
        "data_root": "official",
        "bag_relative_path": "sample",
        "sensor_topics": {
            "lidar": "/livox/lidar",
            "imu": "/livox/imu",
            "image": "/left_camera/image",
        },
        "launch": {
            "launch_file": "mapping_aviz.launch.py",
            "base_params": "config/avia.yaml",
            "camera_params": "config/camera_pinhole.yaml",
        },
        "variants": ["LO", "LIO", "LIVO"],
        "repetitions": 1,
    }
    dataset.update(overrides)
    return dataset


def document(dataset):
    return {
        "schema_version": 1,
        "schedule_seed": 20260917,
        "data_roots": {"official": "/data/official", "ugv": "/data/ugv"},
        "datasets": [dataset],
    }


def schema_errors(value):
    return list(Draft202012Validator(load_schema()).iter_errors(value))


def test_example_manifest_satisfies_draft_2020_12_schema(monkeypatch):
    monkeypatch.setenv("FAST_LIVO2_OFFICIAL_DATA_ROOT", "/data/official")
    monkeypatch.setenv("FAST_LIVO2_UGV_DATA_ROOT", "/data/ugv")
    value = yaml.safe_load((ROOT / "datasets.example.yaml").read_text(encoding="utf-8"))
    assert not schema_errors(value)


@pytest.mark.parametrize("path", [
    "calibration/run_01",
    "lidar_camera_calib/run_01",
    "sensor-extrinsic/run_01",
    "camera_intrinsic/run_01",
])
def test_schema_rejects_calibration_labelled_bag_paths(path):
    assert schema_errors(document(minimal_dataset(bag_relative_path=path)))


def test_schema_rejects_non_validation_purpose():
    assert schema_errors(document(minimal_dataset(purpose="calibration")))


def test_schema_accepts_metric_cloud_recording_contract():
    value = document(minimal_dataset(record={"metric_cloud": True}))
    assert not schema_errors(value)


def test_distinct_roots_are_enforced_at_runtime(monkeypatch, tmp_path):
    monkeypatch.setenv("OFFICIAL_ROOT", str(tmp_path))
    monkeypatch.setenv("UGV_ROOT", str(tmp_path))
    value = document(minimal_dataset())
    value["data_roots"] = {"official": "${OFFICIAL_ROOT}", "ugv": "${UGV_ROOT}"}
    with pytest.raises(RuntimeError, match="must be distinct"):
        runner.resolved_roots(value)


def test_bag_cannot_escape_declared_root(tmp_path):
    roots = {"official": tmp_path / "official", "ugv": tmp_path / "ugv"}
    dataset = minimal_dataset(bag_relative_path="../ugv/sequence")
    with pytest.raises(RuntimeError, match="escapes declared root"):
        runner.resolve_bag(dataset, roots)


@pytest.mark.parametrize("input_parts,output_parts", [
    (("data",), ("data", "results")),
    (("data", "bag"), ("data",)),
    (("same",), ("same",)),
])
def test_input_output_overlap_is_rejected(tmp_path, input_parts, output_parts):
    input_path = tmp_path.joinpath(*input_parts)
    output_path = tmp_path.joinpath(*output_parts)
    with pytest.raises(RuntimeError, match="overlap forbidden"):
        runner.guard_no_overlap(input_path, output_path)


def test_three_repetition_schedule_is_latin_square():
    dataset = minimal_dataset(repetitions=3)
    assert runner.schedule_for(dataset, 17) == [
        {"repetition": 1, "variant": "LO"},
        {"repetition": 1, "variant": "LIO"},
        {"repetition": 1, "variant": "LIVO"},
        {"repetition": 2, "variant": "LIO"},
        {"repetition": 2, "variant": "LIVO"},
        {"repetition": 2, "variant": "LO"},
        {"repetition": 3, "variant": "LIVO"},
        {"repetition": 3, "variant": "LO"},
        {"repetition": 3, "variant": "LIO"},
    ]


def test_single_repetition_schedule_is_seeded_and_reproducible():
    dataset = minimal_dataset(repetitions=1)
    first = runner.schedule_for(dataset, 20260917)
    assert first == runner.schedule_for(dataset, 20260917)
    assert sorted(item["variant"] for item in first) == ["LIO", "LIVO", "LO"]


def test_mode_commands_use_typed_overrides(monkeypatch, tmp_path):
    base = tmp_path / "base.yaml"
    camera = tmp_path / "camera.yaml"
    base.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    camera.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    dataset = minimal_dataset(
        launch={"launch_file": "mapping_aviz.launch.py",
                "base_params": str(base), "camera_params": str(camera)})
    command, _ = runner.node_command(dataset, "LO")
    joined = " ".join(command)
    assert "common.img_en:=0" in joined
    assert "common.lidar_en:=1" in joined
    assert "imu.imu_en:=false" in joined
    assert "localizability.calibration.enabled:=false" in joined


def test_metric_cloud_capture_uses_s2_exact_stamp_topic(monkeypatch, tmp_path):
    base = tmp_path / "base.yaml"
    camera = tmp_path / "camera.yaml"
    base.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    camera.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    dataset = minimal_dataset(
        record={"metric_cloud": True, "cloud_registered": False},
        launch={"launch_file": "mapping_aviz.launch.py",
                "base_params": str(base), "camera_params": str(camera)},
    )
    commands, _, topics = runner.commands_for(
        dataset, "LIO", tmp_path / "input", tmp_path / "output")
    assert runner.METRIC_CLOUD_TOPIC in topics
    assert runner.LEGACY_CLOUD_TOPIC not in topics
    assert runner.METRIC_CLOUD_TOPIC in commands["record"]
    joined = " ".join(commands["node"])
    assert "publish.metric_cloud_en:=true" in joined
    assert "publish.metric_cloud_topic:=/cloud_registered_metric" in joined


def test_playback_qos_override_is_explicit_and_resolved(tmp_path):
    base = tmp_path / "base.yaml"
    qos = tmp_path / "qos.yaml"
    base.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    qos.write_text("/livox/lidar: {reliability: reliable}", encoding="utf-8")
    dataset = minimal_dataset(
        playback_qos_overrides=str(qos),
        launch={"launch_file": "mapping_aviz.launch.py",
                "base_params": str(base), "camera_params": None},
    )
    commands, _, _ = runner.commands_for(
        dataset, "LO", tmp_path / "input", tmp_path / "output")
    assert commands["play"][-2:] == [
        "--qos-profile-overrides-path", str(qos.resolve())
    ]


def test_legacy_cloud_capture_remains_supported(monkeypatch, tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text("/**: {ros__parameters: {}}", encoding="utf-8")
    dataset = minimal_dataset(
        record={"cloud_registered": True},
        launch={"launch_file": "mapping_aviz.launch.py",
                "base_params": str(base), "camera_params": None},
    )
    commands, _, topics = runner.commands_for(
        dataset, "LO", tmp_path / "input", tmp_path / "output")
    assert runner.LEGACY_CLOUD_TOPIC in topics
    assert runner.METRIC_CLOUD_TOPIC not in topics
    assert runner.LEGACY_CLOUD_TOPIC in commands["record"]


def effective_dump(dataset, variant, **overrides):
    values = {
        **runner.MODES[variant],
        "use_sim_time": True,
        "common.lid_topic": dataset["sensor_topics"]["lidar"],
        "common.imu_topic": dataset["sensor_topics"]["imu"],
        "common.img_topic": dataset["sensor_topics"]["image"],
        "publish.lidar_information_en": True,
        "publish.dense_map_en": False,
        "publish.metric_cloud_en": bool(
            dataset.get("record", {}).get("metric_cloud", False)
        ),
        "publish.metric_cloud_topic": runner.METRIC_CLOUD_TOPIC,
        "pcd_save.pcd_save_en": False,
        "localizability.calibration.enabled": False,
    }
    values.update(overrides)
    return yaml.safe_dump({runner.NODE_NAME: {"ros__parameters": values}})


def test_effective_parameter_verification_accepts_complete_contract():
    dataset = minimal_dataset()
    result = runner.verify_effective_parameters(
        effective_dump(dataset, "LIVO"), dataset, "LIVO")
    assert result["status"] == "VALID"
    assert not result["mismatches"]


def test_effective_parameter_verification_rejects_mismatch():
    dataset = minimal_dataset()
    dump = effective_dump(dataset, "LIVO", **{"publish.dense_map_en": True})
    with pytest.raises(RuntimeError, match="effective parameter verification failed"):
        runner.verify_effective_parameters(dump, dataset, "LIVO")


def test_recorded_metric_cloud_contract_is_fail_closed(tmp_path):
    metrics = tmp_path / "metrics.json"
    metrics.write_text(json.dumps({"pose_metric_cloud_contract": {
        "exact_pairing_complete": False, "unmatched_metric_cloud_count": 1,
    }}), encoding="utf-8")
    dataset = minimal_dataset(record={"metric_cloud": True})
    with pytest.raises(RuntimeError, match="exact-pairing contract failed"):
        runner.verify_recorded_contract(metrics, dataset)


def test_recorded_metric_cloud_contract_accepts_complete_pairing(tmp_path):
    metrics = tmp_path / "metrics.json"
    expected = {"exact_pairing_complete": True, "exact_pair_count": 3}
    metrics.write_text(json.dumps({"pose_metric_cloud_contract": expected}),
                       encoding="utf-8")
    dataset = minimal_dataset(record={"metric_cloud": True})
    assert runner.verify_recorded_contract(metrics, dataset) == expected


def test_baseline_topics_exclude_disabled_localizability_calibration():
    assert "/lidar_localizability_calibration" not in runner.BASE_TOPICS
    assert runner.ODOM_TOPIC in runner.BASE_TOPICS
    assert runner.INFO_TOPIC in runner.BASE_TOPICS


def test_input_analysis_command_declares_all_sensor_topics(tmp_path):
    dataset = minimal_dataset()
    command = runner.input_analysis_command(dataset, tmp_path / "bag", tmp_path / "out")
    joined = " ".join(command)
    assert "--lidar-topic /livox/lidar" in joined
    assert "--imu-topic /livox/imu" in joined
    assert "--image-topic /left_camera/image" in joined


def test_vendored_ugv_profiles_match_recorded_hashes():
    provenance_path = ROOT / "profiles" / "ugv" / "provenance.yaml"
    provenance = yaml.safe_load(provenance_path.read_text(encoding="utf-8"))
    for record in provenance["files"].values():
        path = provenance_path.parent / record["file"]
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]


def _temporary_vendored_profiles(tmp_path):
    files = {}
    records = {}
    for variant in runner.MODES:
        path = tmp_path / f"{variant.lower()}.yaml"
        path.write_text(f"variant: {variant}\\n", encoding="utf-8")
        files[variant] = path
        records[variant] = {
            "file": path.name,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    provenance = tmp_path / "provenance.yaml"
    provenance.write_text(yaml.safe_dump({"schema_version": 1, "files": records}),
                          encoding="utf-8")
    dataset = minimal_dataset(launch={
        "launch_file": "mapping_aviz.launch.py",
        "variant_params": {name: str(path) for name, path in files.items()},
        "camera_params": None,
        "profile_provenance_file": str(provenance),
    })
    return dataset, files


def test_runtime_vendored_profile_verification_checks_all_recorded_hashes(tmp_path):
    dataset, files = _temporary_vendored_profiles(tmp_path)
    result = runner.verify_vendored_profile_hashes(dataset, "LO", [files["LO"]])
    assert result["status"] == "VALID"
    assert result["selected_variant"] == "LO"
    assert len(result["verified_files"]) == len(runner.MODES)


def test_runtime_vendored_profile_verification_fails_on_nonselected_tamper(tmp_path):
    dataset, files = _temporary_vendored_profiles(tmp_path)
    files["LIO"].write_text("tampered: true\\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="SHA-256 mismatch for LIO"):
        runner.verify_vendored_profile_hashes(dataset, "LO", [files["LO"]])


def test_validation_cell_error_preserves_invalid_shutdown_status(tmp_path):
    error = runner.ValidationCellError(
        tmp_path / "run", "INVALID_SHUTDOWN", "shutdown", "segmentation fault")
    assert error.status == "INVALID_SHUTDOWN"
    assert error.failure_stage == "shutdown"
    assert error.run_directory == tmp_path / "run"


class FakePsutilError(Exception):
    pass


class FakeResourceProcess:
    def __init__(self, pid, rss, cpu=0.0, children=None):
        self.pid = pid
        self.rss = rss
        self.cpu = cpu
        self.cpu_calls = 0
        self.alive = True
        self._children = children or (lambda: [])

    def cpu_percent(self, interval):
        if not self.alive:
            raise FakePsutilError(f"process PID not found (pid={self.pid})")
        self.cpu_calls += 1
        return 0.0 if self.cpu_calls == 1 else self.cpu

    def memory_info(self):
        if not self.alive:
            raise FakePsutilError(f"process PID not found (pid={self.pid})")
        return SimpleNamespace(rss=self.rss)

    def children(self, recursive):
        if not self.alive:
            raise FakePsutilError(f"process PID not found (pid={self.pid})")
        assert recursive is True
        return self._children()


class FixedSampleEvent:
    def __init__(self, sample_count, before_sample=None):
        self.remaining = sample_count
        self.before_sample = before_sample

    def wait(self, timeout):
        assert timeout == 1
        if self.remaining == 0:
            return True
        self.remaining -= 1
        if self.before_sample is not None:
            self.before_sample()
        return False


def test_resource_monitor_reuses_child_process_and_aggregates_tree(monkeypatch):
    children = []

    def new_child():
        process = FakeResourceProcess(pid=200, rss=900, cpu=101.0)
        children.append(process)
        return [process]

    root = FakeResourceProcess(pid=100, rss=100, children=new_child)
    fake_psutil = SimpleNamespace(
        Process=lambda pid: root,
        Error=FakePsutilError,
    )
    monkeypatch.setitem(sys.modules, "psutil", fake_psutil)
    rows = []

    runner.monitor_resources(100, FixedSampleEvent(2), rows)

    assert len(rows) == 2
    assert [row["cpu_percent"] for row in rows] == [101.0, 101.0]
    assert [row["rss_bytes"] for row in rows] == [1000, 1000]
    assert len(children) == 3
    assert children[0].cpu_calls == 3
    assert all(process.cpu_calls == 0 for process in children[1:])


def test_resource_monitor_ignores_normal_process_disappearance(monkeypatch):
    child = FakeResourceProcess(pid=200, rss=900, cpu=101.0)
    root = FakeResourceProcess(pid=100, rss=100, children=lambda: [child])
    fake_psutil = SimpleNamespace(
        Process=lambda pid: root,
        Error=FakePsutilError,
    )
    monkeypatch.setitem(sys.modules, "psutil", fake_psutil)

    def stop_processes():
        root.alive = False
        child.alive = False

    rows = []
    runner.monitor_resources(
        100, FixedSampleEvent(1, before_sample=stop_processes), rows)

    assert rows == []


class FakeProcess:
    def __init__(self, outcomes, final_code):
        self.pid = 4242
        self.returncode = None
        self._outcomes = list(outcomes)
        self._final_code = final_code

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        outcome = self._outcomes.pop(0) if self._outcomes else "done"
        if outcome == "timeout":
            raise subprocess.TimeoutExpired(["fake"], timeout)
        self.returncode = self._final_code
        return self.returncode


def test_stop_process_records_sigint_to_sigterm_escalation(monkeypatch):
    sent = []
    monkeypatch.setattr(runner.os, "killpg", lambda pid, sig: sent.append((pid, sig)))
    process = FakeProcess(["timeout", "done"], -signal.SIGTERM)
    events = []
    code = runner.stop_process(
        "node", process, events, timeout=15, terminate_timeout=5)
    assert code == -signal.SIGTERM
    assert [item[1] for item in sent] == [signal.SIGINT, signal.SIGTERM]
    event = events[0]
    assert event["process"] == "node"
    assert event["pid"] == 4242
    assert event["timeouts"] == [{"after_signal": "SIGINT", "timeout_s": 15}]
    assert event["escalated"] is True
    assert event["reason"] == "stopped_after_sigterm"
    assert event["final_code"] == -signal.SIGTERM
    assert [item["signal"] for item in event["signals"]] == ["SIGINT", "SIGTERM"]
    assert all(item["delivered"] for item in event["signals"])


def test_stop_process_records_sigkill_and_shutdown_classification(monkeypatch):
    sent = []
    monkeypatch.setattr(runner.os, "killpg", lambda pid, sig: sent.append(sig))
    process = FakeProcess(["timeout", "timeout", "done"], -signal.SIGKILL)
    events = []
    runner.stop_process("record", process, events, timeout=2, terminate_timeout=1)
    assert sent == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert events[0]["reason"] == "stopped_after_sigkill"
    assert events[0]["timeouts"] == [
        {"after_signal": "SIGINT", "timeout_s": 2},
        {"after_signal": "SIGTERM", "timeout_s": 1},
    ]
    assert runner.escalated_shutdown_processes(events) == ["record"]


def test_stop_process_records_not_started():
    events = []
    assert runner.stop_process("play", None, events) is None
    assert events[0]["reason"] == "not_started"
    assert events[0]["final_code"] is None


def test_repetition_filter_selects_only_cbd03_lo_repetition_one():
    dataset = minimal_dataset(id="CBD_Building_03", repetitions=3)
    schedule = runner.schedule_for(dataset, 20260917, ["LO"])
    assert runner.filter_schedule_repetitions(schedule, [1]) == [
        {"repetition": 1, "variant": "LO"},
    ]


def test_repetition_filter_preserves_prespecified_schedule_order():
    dataset = minimal_dataset(id="CBD_Building_03", repetitions=3)
    schedule = runner.schedule_for(dataset, 20260917)
    filtered = runner.filter_schedule_repetitions(schedule, [3, 1])
    assert filtered == [cell for cell in schedule if cell["repetition"] in {1, 3}]
    assert [cell["repetition"] for cell in filtered] == [1, 1, 1, 3, 3, 3]


@pytest.mark.parametrize("invalid", [0, -1])
def test_repetition_filter_rejects_nonpositive_values(invalid):
    with pytest.raises(RuntimeError, match="must be positive integers"):
        runner.filter_schedule_repetitions(
            [{"repetition": 1, "variant": "LO"}], [invalid])


def test_repetition_filter_rejects_selection_without_match():
    with pytest.raises(RuntimeError, match="produced no schedule cells"):
        runner.filter_schedule_repetitions(
            [{"repetition": 1, "variant": "LO"}], [2])
