#!/usr/bin/env python3
"""Fail-closed, reproducible FAST-LIVO2 validation runner for local rosbag2 data."""
from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import fcntl
import hashlib
import importlib.metadata as importlib_metadata
import json
import os
import platform
import random
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

import yaml
from jsonschema import Draft202012Validator

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
NODE_NAME = "/laserMapping"
ODOM_TOPIC = "/aft_mapped_to_init"
MODES = {
    "LO": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": False},
    "LIO": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": True},
    "LIVO": {"common.img_en": 1, "common.lidar_en": 1, "imu.imu_en": True},
}
INFO_TOPIC = "/lidar_measurement_information"
LEGACY_CLOUD_TOPIC = "/cloud_registered"
METRIC_CLOUD_TOPIC = "/cloud_registered_metric"
BASE_TOPICS = [ODOM_TOPIC, INFO_TOPIC, "/parameter_events", "/rosout"]
CALIBRATION_TOKEN = re.compile(
    r"(^|[/_.-])(calib(?:ration)?|calibracion|intrinsic|extrinsic)(?=$|[/_.-])",
    re.IGNORECASE,
)
STOP_REQUESTED = threading.Event()
ACTIVE_PROCESSES: list[subprocess.Popen] = []


class ValidationCellError(RuntimeError):
    def __init__(self, run_directory: Path, status: str,
                 failure_stage: str | None, message: str):
        super().__init__(message)
        self.run_directory = run_directory
        self.status = status
        self.failure_stage = failure_stage


def run(cmd: list[str], **kwargs: Any) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, text=True, check=False, **kwargs)


def command_capture(cmd: list[str], cwd: Path | None = None) -> dict[str, Any]:
    result = run(cmd, cwd=cwd, capture_output=True)
    return {"argv": cmd, "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}


def path_for(value: str, base: Path = REPO) -> Path:
    expanded = os.path.expandvars(os.path.expanduser(value))
    if "$" in expanded:
        raise RuntimeError(f"unresolved environment variable: {value}")
    path = Path(expanded)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def tree_hashes(path: Path) -> dict[str, str]:
    return {str(item.relative_to(path)): sha(item) for item in sorted(path.rglob("*"))
            if item.is_file()}


def tracked_source_hashes() -> dict[str, str]:
    result = run(["git", "ls-files", "-co", "--exclude-standard"], cwd=REPO,
                 capture_output=True)
    if result.returncode:
        raise RuntimeError("git ls-files failed: " + result.stderr)
    hashes = {}
    source_suffixes = {
        ".py", ".cpp", ".cc", ".c", ".h", ".hpp", ".yaml", ".json",
        ".md", ".txt", ".msg", ".cmake", ".xml",
    }
    for name in result.stdout.splitlines():
        candidate = REPO / name
        if (candidate.is_file()
                and (candidate.suffix.lower() in source_suffixes
                     or candidate.name in {"CMakeLists.txt", "package.xml"})):
            hashes[name] = sha(candidate)
    return hashes


def is_within(child: Path, parent: Path) -> bool:
    try:
        child.relative_to(parent)
        return True
    except ValueError:
        return False


def reject_calibration(value: str, label: str) -> None:
    if CALIBRATION_TOKEN.search(value.replace("\\", "/")):
        raise RuntimeError(f"{label} is calibration-labelled and forbidden: {value}")


def resolve_bag(dataset: dict[str, Any], roots: dict[str, Path]) -> Path:
    root_name = dataset["data_root"]
    if root_name not in roots:
        raise RuntimeError(f"{dataset['id']}: unknown data_root {root_name!r}")
    relative = Path(dataset["bag_relative_path"])
    if relative.is_absolute():
        raise RuntimeError(f"{dataset['id']}: bag_relative_path must be relative")
    reject_calibration(str(relative), f"{dataset['id']} bag path")
    bag = (roots[root_name] / relative).resolve()
    if not is_within(bag, roots[root_name]):
        raise RuntimeError(f"{dataset['id']}: bag escapes declared root")
    return bag


def guard_no_overlap(input_bag: Path, output_root: Path) -> None:
    source, target = input_bag.resolve(), output_root.resolve()
    if source == target or is_within(source, target) or is_within(target, source):
        raise RuntimeError(f"input/output overlap forbidden: input={source} output={target}")


def preflight(path: Path) -> str:
    if not path.is_dir() or not (path / "metadata.yaml").is_file():
        raise RuntimeError(f"invalid bag directory: {path}")
    result = run(["ros2", "bag", "info", str(path)], capture_output=True)
    if result.returncode or not result.stdout.strip():
        raise RuntimeError("read-only ros2 bag info rejected bag; no repair attempted:\n"
                           + result.stdout + result.stderr)
    return result.stdout


def config(value: str | None) -> Path | None:
    if value is None:
        return None
    path = path_for(value)
    if not path.is_file():
        raise RuntimeError(f"missing configuration: {path}")
    return path


def scalar(value: Any) -> str:
    return ("true" if value else "false") if isinstance(value, bool) else str(value)


def parameter_value(parameters: dict[str, Any], dotted_name: str) -> Any:
    if dotted_name in parameters:
        return parameters[dotted_name]
    value: Any = parameters
    for part in dotted_name.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(dotted_name)
        value = value[part]
    return value


def verify_effective_parameters(dump_text: str, dataset: dict[str, Any],
                                variant: str) -> dict[str, Any]:
    document = yaml.safe_load(dump_text)
    node_document = document.get(NODE_NAME) or document.get(NODE_NAME.lstrip("/"))
    if not isinstance(node_document, dict):
        raise RuntimeError(f"parameter dump missing node {NODE_NAME}")
    parameters = node_document.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise RuntimeError("parameter dump missing ros__parameters")
    topics = dataset["sensor_topics"]
    record = dataset.get("record", {})
    expected = {
        **MODES[variant],
        "use_sim_time": True,
        "common.lid_topic": topics["lidar"],
        "common.imu_topic": topics.get("imu", "/livox/imu"),
        "common.img_topic": topics.get("image", "/left_camera/image"),
        "publish.lidar_information_en": True,
        "publish.dense_map_en": False,
        "publish.metric_cloud_en": bool(record.get("metric_cloud", False)),
        "publish.metric_cloud_topic": METRIC_CLOUD_TOPIC,
        "pcd_save.pcd_save_en": False,
        "localizability.calibration.enabled": False,
    }
    observed: dict[str, Any] = {}
    mismatches = []
    for name, wanted in expected.items():
        try:
            actual = parameter_value(parameters, name)
        except KeyError:
            mismatches.append({"parameter": name, "expected": wanted,
                               "observed": None, "reason": "missing"})
            continue
        observed[name] = actual
        if type(actual) is not type(wanted) or actual != wanted:
            mismatches.append({"parameter": name, "expected": wanted,
                               "observed": actual, "reason": "type_or_value"})
    result = {"status": "VALID" if not mismatches else "INVALID",
              "expected": expected, "observed": observed,
              "mismatches": mismatches}
    if mismatches:
        raise RuntimeError(
            "effective parameter verification failed: "
            + json.dumps(mismatches, sort_keys=True))
    return result


def verify_vendored_profile_hashes(
    dataset: dict[str, Any], variant: str, configs: list[Path]
) -> dict[str, Any]:
    """Fail closed when a declared vendored-profile hash no longer matches."""
    provenance_value = dataset["launch"].get("profile_provenance_file")
    if not provenance_value:
        return {"status": "NOT_APPLICABLE", "reason": "no provenance file declared"}
    provenance_path = config(provenance_value)
    document = yaml.safe_load(provenance_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise RuntimeError("profile provenance schema_version must be 1")
    records = document.get("files")
    if not isinstance(records, dict) or variant not in records:
        raise RuntimeError(f"profile provenance has no record for variant {variant}")
    verified = []
    resolved_by_variant: dict[str, Path] = {}
    for name, record in sorted(records.items()):
        if not isinstance(record, dict):
            raise RuntimeError(f"profile provenance record {name} is not an object")
        filename, expected = record.get("file"), record.get("sha256")
        if not isinstance(filename, str) or not filename:
            raise RuntimeError(f"profile provenance record {name} has no file")
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise RuntimeError(f"profile provenance record {name} has invalid sha256")
        profile_path = (provenance_path.parent / filename).resolve()
        if not is_within(profile_path, provenance_path.parent.resolve()):
            raise RuntimeError(f"profile provenance record {name} escapes its directory")
        if not profile_path.is_file():
            raise RuntimeError(f"vendored profile is absent: {profile_path}")
        observed = sha(profile_path)
        if observed != expected:
            raise RuntimeError(
                f"vendored profile SHA-256 mismatch for {name}: "
                f"expected={expected} observed={observed}")
        resolved_by_variant[str(name)] = profile_path
        verified.append({"variant": str(name), "path": str(profile_path),
                         "expected_sha256": expected, "observed_sha256": observed})
    if not configs or configs[0].resolve() != resolved_by_variant[variant]:
        raise RuntimeError(
            f"selected {variant} profile does not match provenance record: "
            f"selected={configs[0] if configs else None} "
            f"recorded={resolved_by_variant[variant]}")
    return {"status": "VALID", "provenance_path": str(provenance_path),
            "selected_variant": variant, "verified_files": verified}


def variant_configs(dataset: dict[str, Any], variant: str) -> list[Path]:
    launch = dataset["launch"]
    base_value = launch.get("variant_params", {}).get(variant, launch.get("base_params"))
    if not base_value:
        raise RuntimeError(f"{dataset['id']} {variant}: no parameter profile")
    return [item for item in
            (config(base_value), config(launch.get("camera_params"))) if item]


def node_command(dataset: dict[str, Any], variant: str) -> tuple[list[str], list[Path]]:
    configs = variant_configs(dataset, variant)
    cmd = ["ros2", "run", "fast_livo", "fastlivo_mapping", "--ros-args"]
    for item in configs:
        cmd += ["--params-file", str(item)]
    topics = dataset["sensor_topics"]
    record = dataset.get("record", {})
    params = dict(MODES[variant])
    params.update({
        "use_sim_time": True,
        "common.lid_topic": topics["lidar"],
        "common.imu_topic": topics.get("imu", "/livox/imu"),
        "common.img_topic": topics.get("image", "/left_camera/image"),
        "publish.lidar_information_en": True,
        "publish.dense_map_en": False,
        "publish.metric_cloud_en": bool(record.get("metric_cloud", False)),
        "publish.metric_cloud_topic": METRIC_CLOUD_TOPIC,
        "pcd_save.pcd_save_en": False,
        "localizability.calibration.enabled": False,
    })
    for name, value in params.items():
        cmd += ["-p", f"{name}:={scalar(value)}"]
    return cmd, configs


def commands_for(dataset: dict[str, Any], variant: str, bag: Path,
                 output_bag: Path) -> tuple[dict[str, list[str]], list[Path], list[str]]:
    node_cmd, configs = node_command(dataset, variant)
    topics = list(BASE_TOPICS)
    ground_truth = dataset.get("ground_truth")
    if ground_truth:
        topics.append(ground_truth["topic"])
    record = dataset.get("record", {})
    if record.get("cloud_registered", False):
        topics.append(LEGACY_CLOUD_TOPIC)
    if record.get("metric_cloud", False):
        topics.append(METRIC_CLOUD_TOPIC)
    play = ["ros2", "bag", "play", str(bag), "--clock",
            "--disable-keyboard-controls", "--rate",
            str(dataset.get("playback_rate", 1.0))]
    qos_value = dataset.get("playback_qos_overrides")
    if qos_value:
        qos_path = path_for(qos_value)
        if not qos_path.is_file():
            raise RuntimeError(f"playback QoS override is not a file: {qos_path}")
        play += ["--qos-profile-overrides-path", str(qos_path)]
    commands = {
        "record": ["ros2", "bag", "record", "-o", str(output_bag), *topics],
        "node": node_cmd,
        "play": play,
    }
    return commands, configs, topics


def analysis_command(dataset: dict[str, Any], output_bag: Path, root: Path,
                     manifest_path: Path) -> list[str]:
    command = [
        sys.executable, str(HERE / "analyze_results.py"),
        "--bag", str(output_bag),
        "--output", str(root / "analysis"),
        "--run-manifest", str(manifest_path),
    ]
    gt = dataset.get("ground_truth")
    if gt:
        command += ["--gt-topic", gt["topic"], "--max-gt-delta-s",
                    str(gt.get("max_time_delta_s", 0.05))]
    return command


def verify_recorded_contract(metrics_path: Path, dataset: dict[str, Any]) -> dict[str, Any]:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    contract = metrics.get("pose_metric_cloud_contract", {})
    if dataset.get("record", {}).get("metric_cloud", False):
        if not contract.get("exact_pairing_complete", False):
            raise RuntimeError(
                "metric cloud/odometry exact-pairing contract failed: "
                + json.dumps(contract, sort_keys=True)
            )
    return contract


def input_analysis_command(dataset: dict[str, Any], bag: Path,
                           output: Path) -> list[str]:
    topics = dataset["sensor_topics"]
    command = [
        sys.executable, str(HERE / "analyze_input.py"),
        "--bag", str(bag),
        "--output", str(output),
        "--lidar-topic", topics["lidar"],
        "--hash-mode", "metadata",
    ]
    if topics.get("imu"):
        command += ["--imu-topic", topics["imu"]]
    if topics.get("image"):
        command += ["--image-topic", topics["image"]]
    return command


def analyze_input_once(dataset: dict[str, Any], bag: Path,
                       output_root: Path) -> tuple[dict[str, Any], dict[str, int],
                                                   dict[str, Any]]:
    output = output_root / "_input_analysis" / dataset["id"]
    command = input_analysis_command(dataset, bag, output)
    result = run(command, capture_output=True)
    output.mkdir(parents=True, exist_ok=True)
    (output / "analyze_input.log").write_text(
        result.stdout + result.stderr, encoding="utf-8")
    report_path = output / "input_integrity.json"
    if result.returncode or not report_path.is_file():
        raise RuntimeError(
            f"{dataset['id']}: input analysis failed with code {result.returncode}")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    counts: dict[str, int] = {}
    timing: dict[str, Any] = {}
    for role, topic_report in report.get("topics", {}).items():
        name = topic_report.get("name")
        if not name:
            continue
        counts[str(name)] = int(topic_report.get("message_count", 0))
        timing[str(name)] = {
            "role": role,
            "bag_timing": topic_report.get("bag_timing"),
            "header_timing": topic_report.get("header_timing"),
            "header_coverage_fraction": topic_report.get(
                "header_coverage_fraction"),
        }
    required = [dataset["sensor_topics"]["lidar"]]
    required += [dataset["sensor_topics"][name] for name in ("imu", "image")
                 if dataset["sensor_topics"].get(name)]
    missing = [name for name in required if counts.get(name, 0) <= 0]
    if missing:
        raise RuntimeError(
            f"{dataset['id']}: input analysis has zero/missing required topics {missing}")
    evidence = {
        "status": "COMPLETED",
        "command": command,
        "returncode": result.returncode,
        "output_directory": str(output),
        "report_path": str(report_path),
        "report_sha256": sha(report_path),
    }
    return evidence, counts, timing


def schedule_for(dataset: dict[str, Any], seed: int,
                 selected_variants: list[str] | None = None) -> list[dict[str, Any]]:
    variants = [item for item in dataset["variants"]
                if selected_variants is None or item in selected_variants]
    repetitions = int(dataset["repetitions"])
    cells = []
    if repetitions == 3 and variants == ["LO", "LIO", "LIVO"]:
        rows = [
            ["LO", "LIO", "LIVO"],
            ["LIO", "LIVO", "LO"],
            ["LIVO", "LO", "LIO"],
        ]
        for repetition, row in enumerate(rows, 1):
            cells.extend({"repetition": repetition, "variant": variant} for variant in row)
        return cells
    for repetition in range(1, repetitions + 1):
        row = list(variants)
        key = f"{seed}:{dataset['id']}:{repetition}".encode()
        derived = int.from_bytes(hashlib.sha256(key).digest()[:8], "big")
        random.Random(derived).shuffle(row)
        cells.extend({"repetition": repetition, "variant": variant} for variant in row)
    return cells


def filter_schedule_repetitions(
    cells: list[dict[str, Any]], repetitions: list[int] | None,
) -> list[dict[str, Any]]:
    if repetitions is None:
        return list(cells)
    invalid = sorted({value for value in repetitions if value <= 0})
    if invalid:
        raise RuntimeError(
            f"repetition filters must be positive integers: {invalid}")
    selected = set(repetitions)
    filtered = [cell for cell in cells if int(cell["repetition"]) in selected]
    if not filtered:
        raise RuntimeError(
            f"repetition filter {sorted(selected)} produced no schedule cells")
    return filtered


def spawn(cmd: list[str], stream: Any) -> subprocess.Popen:
    process = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    ACTIVE_PROCESSES.append(process)
    return process


def stop_process(name: str, process: subprocess.Popen | None,
                 events: list[dict[str, Any]], timeout: int = 15,
                 terminate_timeout: int = 5) -> int | None:
    event: dict[str, Any] = {
        "process": name,
        "pid": process.pid if process is not None else None,
        "signals": [],
        "timeouts": [],
        "escalated": False,
        "reason": None,
        "final_code": None,
    }
    if process is None:
        event["reason"] = "not_started"
        events.append(event)
        return None
    if process.poll() is not None:
        event["reason"] = "already_exited"
    else:
        def send(sig: signal.Signals) -> None:
            record = {
                "signal": sig.name,
                "number": int(sig),
                "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                "delivered": True,
            }
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                record["delivered"] = False
            event["signals"].append(record)

        send(signal.SIGINT)
        try:
            process.wait(timeout=timeout)
            event["reason"] = "stopped_after_sigint"
        except subprocess.TimeoutExpired:
            event["timeouts"].append(
                {"after_signal": "SIGINT", "timeout_s": timeout})
            event["escalated"] = True
            send(signal.SIGTERM)
            try:
                process.wait(timeout=terminate_timeout)
                event["reason"] = "stopped_after_sigterm"
            except subprocess.TimeoutExpired:
                event["timeouts"].append(
                    {"after_signal": "SIGTERM", "timeout_s": terminate_timeout})
                send(signal.SIGKILL)
                process.wait()
                event["reason"] = "stopped_after_sigkill"
    with contextlib.suppress(ValueError):
        ACTIVE_PROCESSES.remove(process)
    event["final_code"] = process.returncode
    events.append(event)
    return process.returncode


def escalated_shutdown_processes(events: list[dict[str, Any]]) -> list[str]:
    return sorted({
        str(event["process"]) for event in events
        if event.get("process") in {"node", "record"} and event.get("escalated")
    })


def wait_until(label: str, predicate: Callable[[], bool],
               processes: list[subprocess.Popen], timeout_s: float,
               interval_s: float = 0.25) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if STOP_REQUESTED.is_set():
            raise RuntimeError(f"interrupted while waiting for {label}")
        dead = [process.pid for process in processes if process.poll() is not None]
        if dead:
            raise RuntimeError(f"processes exited before {label}: {dead}")
        if predicate():
            return
        time.sleep(interval_s)
    raise RuntimeError(f"timeout waiting for {label} ({timeout_s:.1f}s)")


def ros_list(kind: str) -> set[str]:
    try:
        result = run(["ros2", kind, "list"], capture_output=True, timeout=5)
    except subprocess.TimeoutExpired:
        return set()
    return set(result.stdout.splitlines()) if result.returncode == 0 else set()


def recorder_subscribed(topic: str) -> bool:
    try:
        result = run(["ros2", "topic", "info", topic, "--verbose"],
                     capture_output=True, timeout=5)
    except subprocess.TimeoutExpired:
        return False
    if result.returncode:
        return False
    count = re.search(r"Subscription count:\s*(\d+)", result.stdout)
    return (count is not None and int(count.group(1)) > 0
            and "rosbag2_recorder" in result.stdout)


def wait_for_playback(player: subprocess.Popen, recorder: subprocess.Popen,
                      node: subprocess.Popen) -> int:
    while player.poll() is None:
        if STOP_REQUESTED.is_set():
            raise RuntimeError("interrupted during playback")
        if recorder.poll() is not None:
            raise RuntimeError("recorder exited during playback")
        if node.poll() is not None:
            raise RuntimeError("FAST-LIVO2 exited during playback")
        time.sleep(0.25)
    return int(player.returncode)


def output_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def wait_for_quiet(path: Path, recorder: subprocess.Popen, node: subprocess.Popen,
                   quiet_s: float, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_size = output_size(path) if path.exists() else -1
    stable_since = time.monotonic()
    while time.monotonic() < deadline:
        if STOP_REQUESTED.is_set():
            raise RuntimeError("interrupted during recorder drain")
        if recorder.poll() is not None or node.poll() is not None:
            raise RuntimeError("recorder or FAST-LIVO2 exited during drain")
        current = output_size(path) if path.exists() else -1
        if current != last_size:
            last_size, stable_since = current, time.monotonic()
        elif current >= 0 and time.monotonic() - stable_since >= quiet_s:
            return
        time.sleep(0.25)
    raise RuntimeError(f"recorder drain timeout ({timeout_s:.1f}s)")


def monitor_resources(pid: int, event: threading.Event, rows: list[dict]) -> None:
    try:
        import psutil
        root = psutil.Process(pid)
        tracked = {root.pid: root}

        def discover() -> None:
            try:
                processes = [root, *root.children(recursive=True)]
            except psutil.Error:
                processes = []
            for process in processes:
                if process.pid in tracked:
                    continue
                try:
                    process.cpu_percent(None)
                    tracked[process.pid] = process
                except psutil.Error:
                    pass

        root.cpu_percent(None)
        discover()
        started = time.monotonic()
        while not event.wait(1):
            rss, cpu, live = 0, 0.0, 0
            for process_pid, process in list(tracked.items()):
                try:
                    rss += process.memory_info().rss
                    cpu += process.cpu_percent(None)
                    live += 1
                except psutil.Error:
                    tracked.pop(process_pid, None)
            if live:
                rows.append({"elapsed_s": time.monotonic() - started,
                             "cpu_percent": cpu, "rss_bytes": rss})
            discover()
    except Exception as exc:
        rows.append({"error": str(exc)})


def checksums(root: Path) -> dict[str, str]:
    return {str(path.relative_to(root)): sha(path) for path in sorted(root.rglob("*"))
            if path.is_file() and path.name != "sha256_manifest.json"}


def machine_provenance() -> dict[str, Any]:
    binaries = {name: shutil.which(name) for name in
                ("python3", "ros2", "git", "fastlivo_mapping")}
    package_versions = {}
    for distribution in ("numpy", "PyYAML", "matplotlib", "psutil", "jsonschema"):
        try:
            package_versions[distribution] = importlib_metadata.version(distribution)
        except importlib_metadata.PackageNotFoundError:
            package_versions[distribution] = "N/A"
    return {
        "platform": platform.platform(),
        "uname": list(platform.uname()),
        "cpu_count": os.cpu_count(),
        "os_release": Path("/etc/os-release").read_text(encoding="utf-8")
        if Path("/etc/os-release").exists() else None,
        "lscpu": command_capture(["lscpu"]),
        "binaries": binaries,
        "python_version": sys.version,
        "python_package_versions": package_versions,
        "package_prefix": command_capture(["ros2", "pkg", "prefix", "fast_livo"]),
        "package_executables": command_capture(
            ["ros2", "pkg", "executables", "fast_livo"]),
        "environment": {key: os.environ[key] for key in (
            "ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI") if key in os.environ},
    }


def repo_provenance() -> dict[str, Any]:
    return {
        "commit": command_capture(["git", "rev-parse", "HEAD"], REPO),
        "status": command_capture(["git", "status", "--porcelain=v1"], REPO),
        "diff": command_capture(["git", "diff", "--no-ext-diff", "--"], REPO),
        "diff_cached": command_capture(
            ["git", "diff", "--cached", "--no-ext-diff", "--"], REPO),
        "source_sha256": tracked_source_hashes(),
    }


def execute(dataset: dict[str, Any], variant: str, repetition: int,
            output_root: Path, bag: Path, bag_info: str,
            bag_hashes: dict[str, str], input_analysis: dict[str, Any],
            input_counts: dict[str, int], input_timing: dict[str, Any],
            resolved_document: dict[str, Any], schedule: list[dict[str, Any]],
            readiness_timeout_s: float, quiet_s: float,
            drain_timeout_s: float) -> Path:
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    root = output_root / dataset["id"] / variant / f"run_{repetition:02d}_{stamp}"
    root.mkdir(parents=True)
    disk_free_initial = shutil.disk_usage(output_root).free
    output_bag = root / "output_bag"
    commands, configs, topics = commands_for(dataset, variant, bag, output_bag)
    provenance_file = config(dataset["launch"].get("profile_provenance_file"))
    qos_value = dataset.get("playback_qos_overrides")
    playback_qos_file = path_for(qos_value) if qos_value else None
    logs = {name: (root / f"{name}.log").open("w", encoding="utf-8")
            for name in commands}
    procs: dict[str, subprocess.Popen | None] = {name: None for name in commands}
    samples: list[dict] = []
    event = threading.Event()
    monitor = None
    exits: dict[str, int | None] = {}
    termination_events: list[dict[str, Any]] = []
    error = None
    failure_stage = None
    current_stage = "startup"
    postflight_error = None
    postflight = None
    analysis_result = None
    recorded_contract = None
    effective_params = None
    parameter_verification = None
    profile_hash_verification = None
    started = time.monotonic()
    manifest_path = root / "run_manifest.json"
    try:
        profile_hash_verification = verify_vendored_profile_hashes(
            dataset, variant, configs)
        resolved_run = {
            "manifest": resolved_document,
            "selected_cell": {
                "dataset_id": dataset["id"],
                "variant": variant,
                "repetition": repetition,
            },
            "resolved_bag_path": str(bag),
            "resolved_config_paths": [str(item) for item in configs],
            "resolved_profile_provenance": (
                str(provenance_file) if provenance_file else None),
            "profile_hash_verification": profile_hash_verification,
            "resolved_output_root": str(output_root),
            "input_analysis": input_analysis,
            "input_counts": input_counts,
            "input_timing": input_timing,
            "commands": commands,
        }
        (root / "resolved_manifest.yaml").write_text(
            yaml.safe_dump(resolved_run, sort_keys=False), encoding="utf-8")
        procs["record"] = spawn(commands["record"], logs["record"])
        wait_until("recorder output", lambda: output_bag.exists(),
                   [procs["record"]], readiness_timeout_s)
        wait_until("rosbag2 recorder node",
                   lambda: any("rosbag2_recorder" in name for name in ros_list("node")),
                   [procs["record"]], readiness_timeout_s)
        procs["node"] = spawn(commands["node"], logs["node"])
        monitor = threading.Thread(target=monitor_resources,
                                   args=(procs["node"].pid, event, samples), daemon=True)
        monitor.start()
        wait_until("FAST-LIVO2 node", lambda: NODE_NAME in ros_list("node"),
                   [procs["record"], procs["node"]], readiness_timeout_s)
        required_outputs = {ODOM_TOPIC, INFO_TOPIC}
        if dataset.get("record", {}).get("metric_cloud", False):
            required_outputs.add(METRIC_CLOUD_TOPIC)
        wait_until("ROS graph outputs",
                   lambda: required_outputs.issubset(ros_list("topic")),
                   [procs["record"], procs["node"]], readiness_timeout_s)
        wait_until(
            "recorder subscriptions to required FAST-LIVO2 outputs",
            lambda: all(recorder_subscribed(topic) for topic in required_outputs),
            [procs["record"], procs["node"]], readiness_timeout_s)
        params = run(["ros2", "param", "dump", NODE_NAME],
                     capture_output=True, timeout=10)
        effective_params = {"returncode": params.returncode, "stdout": params.stdout,
                            "stderr": params.stderr}
        (root / "effective_parameters.yaml").write_text(params.stdout, encoding="utf-8")
        if params.returncode or not params.stdout.strip():
            raise RuntimeError("failed to capture effective FAST-LIVO2 parameters")
        parameter_verification = verify_effective_parameters(
            params.stdout, dataset, variant)
        current_stage = "playback"
        procs["play"] = spawn(commands["play"], logs["play"])
        exits["play"] = wait_for_playback(
            procs["play"], procs["record"], procs["node"])
        if exits["play"]:
            raise RuntimeError(f"ros2 bag play exited {exits['play']}")
        current_stage = "drain"
        wait_for_quiet(output_bag, procs["record"], procs["node"],
                       quiet_s, drain_timeout_s)
    except Exception as exc:
        error = str(exc)
        failure_stage = current_stage
    finally:
        player_code = stop_process("play", procs["play"], termination_events)
        exits["play"] = exits.get("play", player_code)
        exits["node"] = stop_process(
            "node", procs["node"], termination_events)
        exits["record"] = stop_process(
            "record", procs["record"], termination_events)
        event.set()
        if monitor:
            monitor.join(timeout=3)
        for stream in logs.values():
            stream.close()
    runtime_elapsed_s = time.monotonic() - started
    playback_completed = exits.get("play") == 0
    escalated_shutdown = escalated_shutdown_processes(termination_events)
    unexpected_shutdown = {
        name: code for name, code in exits.items()
        if (name in {"node", "record"}
            and code not in (0, -signal.SIGINT, -signal.SIGTERM))
    }
    if playback_completed and escalated_shutdown:
        detail = f"shutdown escalation required for {escalated_shutdown}"
        error = f"{error}; {detail}" if error else detail
        failure_stage = "shutdown"
    elif error is None and unexpected_shutdown:
        error = f"unexpected shutdown exit codes: {unexpected_shutdown}"
        failure_stage = "shutdown"
    # The analyzer consumes this provisional record to add resource/RTF metrics.
    provisional_status = (
        "INVALID_SHUTDOWN" if failure_stage == "shutdown"
        else "PENDING_POSTFLIGHT" if error is None else "INVALID"
    )
    provisional_manifest = {
        "schema_version": 1,
        "cell_status": provisional_status,
        "failure_stage": failure_stage,
        "dataset_id": dataset["id"],
        "variant": variant,
        "repetition": repetition,
        "elapsed_s": runtime_elapsed_s,
        "bag_info": bag_info,
        "input_analysis": input_analysis,
        "input_counts": input_counts,
        "input_timing": input_timing,
        "sensor_topics": dataset["sensor_topics"],
        "recorded_topics": topics,
        "resource_samples": samples,
        "commands": commands,
        "exit_codes": exits,
        "termination_events": termination_events,
        "profile_hash_verification": profile_hash_verification,
        "error": error,
    }
    manifest_path.write_text(
        json.dumps(provisional_manifest, indent=2) + "\n", encoding="utf-8")
    if playback_completed and output_bag.exists():
        try:
            postflight = preflight(output_bag)
            analysis_cmd = analysis_command(
                dataset, output_bag, root, manifest_path)
            result = run(analysis_cmd, capture_output=True)
            analysis_result = {"argv": analysis_cmd, "returncode": result.returncode,
                               "stdout": result.stdout, "stderr": result.stderr}
            (root / "analysis.log").write_text(
                result.stdout + result.stderr, encoding="utf-8")
            odom_csv = root / "analysis" / "odometry.csv"
            metrics_json = root / "analysis" / "metrics.json"
            if result.returncode or not odom_csv.is_file() or odom_csv.stat().st_size == 0:
                raise RuntimeError("analysis failed or required odometry.csv is absent/empty")
            if not metrics_json.is_file() or metrics_json.stat().st_size == 0:
                raise RuntimeError("required metrics.json is absent/empty")
            recorded_contract = verify_recorded_contract(metrics_json, dataset)
        except Exception as exc:
            postflight_error = str(exc)
            if error is None:
                error = postflight_error
                failure_stage = "postflight"
    disk_free_final = shutil.disk_usage(output_root).free
    if error is None:
        status = "VALID"
    elif failure_stage == "shutdown":
        status = "INVALID_SHUTDOWN"
    else:
        status = "INVALID"
    manifest = {
        "schema_version": 1,
        "cell_status": status,
        "dataset_id": dataset["id"],
        "variant": variant,
        "repetition": repetition,
        "started_utc": stamp,
        "elapsed_s": runtime_elapsed_s,
        "total_elapsed_s": time.monotonic() - started,
        "disk_free_bytes": {
            "initial": disk_free_initial,
            "final": disk_free_final,
            "change": disk_free_final - disk_free_initial,
        },
        "schedule_seed": resolved_document["schedule_seed"],
        "dataset_schedule": schedule,
        "resolved_dataset": dataset,
        "commands": commands,
        "exit_codes": exits,
        "termination_events": termination_events,
        "error": error,
        "failure_stage": failure_stage,
        "postflight_error": postflight_error,
        "playback_completed": playback_completed,
        "crash_observed": any(
            code not in (None, 0, -signal.SIGINT, -signal.SIGTERM)
            for code in exits.values()),
        "input_bag": str(bag),
        "bag_info": bag_info,
        "input_bag_info": bag_info,
        "input_analysis": input_analysis,
        "input_counts": input_counts,
        "input_timing": input_timing,
        "output_bag_info": postflight,
        "input_sha256": bag_hashes,
        "config_sha256": {str(item): sha(item) for item in configs},
        "profile_provenance": {
            "path": str(provenance_file) if provenance_file else None,
            "sha256": sha(provenance_file) if provenance_file else None,
        },
        "profile_hash_verification": profile_hash_verification,
        "playback_qos_provenance": {
            "path": str(playback_qos_file) if playback_qos_file else None,
            "sha256": sha(playback_qos_file) if playback_qos_file else None,
        },
        "sensor_topics": dataset["sensor_topics"],
        "recorded_topics": topics,
        "effective_parameters": effective_params,
        "effective_parameter_verification": parameter_verification,
        "analysis": analysis_result,
        "recorded_contract": recorded_contract,
        "resource_samples": samples,
        "machine": machine_provenance(),
        "repository": repo_provenance(),
        "schema_sha256": sha(HERE / "dataset_manifest.schema.json"),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (root / "sha256_manifest.json").write_text(
        json.dumps(checksums(root), indent=2) + "\n", encoding="utf-8")
    if error:
        raise ValidationCellError(root, status, failure_stage, f"{root}: {error}")
    return root


def validate_schema(document: Any) -> None:
    schema = json.loads((HERE / "dataset_manifest.schema.json").read_text(
        encoding="utf-8"))
    validator = Draft202012Validator(schema)
    errors = sorted(validator.iter_errors(document), key=lambda item: list(item.path))
    if errors:
        details = "\n".join(
            f"- {'/'.join(map(str, error.absolute_path)) or '<root>'}: {error.message}"
            for error in errors)
        raise RuntimeError("manifest JSON Schema validation failed:\n" + details)


def resolved_roots(document: dict[str, Any]) -> dict[str, Path]:
    roots = {name: path_for(value, Path.cwd())
             for name, value in document["data_roots"].items()}
    if (roots["official"] == roots["ugv"]
            or is_within(roots["official"], roots["ugv"])
            or is_within(roots["ugv"], roots["official"])):
        raise RuntimeError("official and UGV data roots must be distinct and non-overlapping")
    return roots


def validate_dataset(dataset: dict[str, Any]) -> None:
    if dataset["purpose"] != "validation":
        raise RuntimeError(f"{dataset['id']}: purpose must be validation")
    if dataset["data_role"] == "calibration":
        raise RuntimeError(f"{dataset['id']}: calibration role is forbidden")
    reject_calibration(dataset["bag_relative_path"], f"{dataset['id']} bag path")
    unknown = set(dataset["variants"]) - set(MODES)
    if unknown:
        raise RuntimeError(f"{dataset['id']}: unknown variants {sorted(unknown)}")
    for variant in dataset["variants"]:
        required = ["lidar"] + (["imu"] if variant != "LO" else [])
        required += ["image"] if variant == "LIVO" else []
        missing = [key for key in required if not dataset["sensor_topics"].get(key)]
        if missing:
            raise RuntimeError(f"{dataset['id']} {variant}: missing {missing}")
        variant_configs(dataset, variant)
    config(dataset["launch"].get("profile_provenance_file"))


@contextlib.contextmanager
def exclusive_lock(output_root: Path):
    output_root.mkdir(parents=True, exist_ok=True)
    handle = (output_root / ".fast_livo2_validation.lock").open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(f"another validation owns {handle.name}") from exc
    handle.seek(0)
    handle.truncate()
    handle.write(f"pid={os.getpid()} utc={dt.datetime.now(dt.timezone.utc).isoformat()}\n")
    handle.flush()
    try:
        yield
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def install_signal_handlers() -> None:
    def request_stop(signum: int, _frame: Any) -> None:
        STOP_REQUESTED.set()
        print(f"signal {signum} received; stopping process groups", file=sys.stderr)
        for process in list(ACTIVE_PROCESSES):
            if process.poll() is None:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGINT)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--dataset", action="append", default=[])
    parser.add_argument("--variant", choices=sorted(MODES), action="append")
    parser.add_argument("--repetition", type=int, action="append")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--readiness-timeout-s", type=float, default=30.0)
    parser.add_argument("--drain-quiet-s", type=float, default=5.0)
    parser.add_argument("--drain-timeout-s", type=float, default=120.0)
    args = parser.parse_args()
    if args.repetition and any(value <= 0 for value in args.repetition):
        parser.error("--repetition values must be positive integers")
    install_signal_handlers()
    document = yaml.safe_load(args.manifest.read_text(encoding="utf-8"))
    validate_schema(document)
    roots = resolved_roots(document)
    resolved_document = json.loads(json.dumps(document))
    resolved_document["data_roots"] = {name: str(path) for name, path in roots.items()}
    output_root = args.output_root.resolve()
    selected = set(args.dataset)
    cells: list[dict[str, Any]] = []
    schedules: dict[str, list[dict[str, Any]]] = {}
    selected_variants = args.variant
    for dataset in document["datasets"]:
        if not dataset.get("enabled", True) or selected and dataset["id"] not in selected:
            continue
        dataset_schedule = filter_schedule_repetitions(
            schedule_for(dataset, document["schedule_seed"], selected_variants),
            args.repetition,
        )
        if not dataset_schedule:
            raise RuntimeError(
                f"{dataset['id']}: variant selection produced no executable cells")
        schedules[dataset["id"]] = dataset_schedule
    if not schedules:
        raise RuntimeError("selection produced no datasets")
    if args.dry_run:
        print(json.dumps({"schedule_seed": document["schedule_seed"],
                          "schedule": schedules}, indent=2))
    with exclusive_lock(output_root):
        for dataset in document["datasets"]:
            if dataset["id"] not in schedules:
                continue
            schedule = schedules[dataset["id"]]
            dataset_stage = "dataset_preflight"
            try:
                validate_dataset(dataset)
                bag = resolve_bag(dataset, roots)
                guard_no_overlap(bag, output_root)
                bag_info = preflight(bag)
                bag_hashes = tree_hashes(bag)
                input_output = output_root / "_input_analysis" / dataset["id"]
                if args.dry_run:
                    print(json.dumps({
                        "dataset": dataset["id"],
                        "input_analysis_command": shlex.join(
                            input_analysis_command(dataset, bag, input_output)),
                    }, indent=2))
                    for cell in schedule:
                        variant = cell["variant"]
                        preview = output_root / dataset["id"] / variant / "<run>" / "output_bag"
                        commands, configs, topics = commands_for(
                            dataset, variant, bag, preview)
                        print(json.dumps({
                            "dataset": dataset["id"],
                            **cell,
                            "bag": str(bag),
                            "configs": [str(item) for item in configs],
                            "recorded_topics": topics,
                            "commands": {
                                **{name: shlex.join(cmd)
                                   for name, cmd in commands.items()},
                                "analyze": shlex.join(analysis_command(
                                    dataset, preview,
                                    preview.parent, preview.parent / "run_manifest.json")),
                            },
                        }, indent=2))
                    continue
                dataset_stage = "input_analysis"
                input_analysis, input_counts, input_timing = analyze_input_once(
                    dataset, bag, output_root)
            except Exception as exc:
                cells.append({"dataset": dataset["id"], "status": "INVALID",
                              "stage": dataset_stage, "error": str(exc)})
                print(str(exc), file=sys.stderr)
                continue
            for cell in schedule:
                variant, repetition = cell["variant"], cell["repetition"]
                try:
                    result = execute(
                        dataset, variant, repetition, output_root, bag, bag_info,
                        bag_hashes, input_analysis, input_counts, input_timing,
                        resolved_document, schedule,
                        args.readiness_timeout_s, args.drain_quiet_s,
                        args.drain_timeout_s)
                    cells.append({"dataset": dataset["id"], **cell, "status": "VALID",
                                  "run_directory": str(result)})
                    print(result)
                except ValidationCellError as exc:
                    cells.append({
                        "dataset": dataset["id"], **cell, "status": exc.status,
                        "failure_stage": exc.failure_stage,
                        "run_directory": str(exc.run_directory),
                        "error": str(exc),
                    })
                    print(str(exc), file=sys.stderr)
                except Exception as exc:
                    cells.append({"dataset": dataset["id"], **cell, "status": "INVALID",
                                  "failure_stage": "runner", "error": str(exc)})
                    print(str(exc), file=sys.stderr)
                if STOP_REQUESTED.is_set():
                    break
            if STOP_REQUESTED.is_set():
                break
        if not args.dry_run:
            overall = "VALID" if cells and all(
                item["status"] == "VALID" for item in cells) else "INVALID"
            summary = {
                "schema_version": 1,
                "overall_status": overall,
                "schedule_seed": document["schedule_seed"],
                "schedule": schedules,
                "cells": cells,
            }
            (output_root / "validation_summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            return 0 if overall == "VALID" else 1
        if cells:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
