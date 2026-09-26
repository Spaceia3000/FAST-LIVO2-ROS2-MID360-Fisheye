"""S1 launch and native publication regression; no bag playback."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]


@pytest.fixture
def launch_module(monkeypatch):
    # Narrow ROS launch doubles let contract tests run without a sourced ROS shell.
    class Action:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return context[self.name]
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", NS(
        get_package_prefix=lambda name: "/sensor", get_package_share_directory=lambda name: str(REPO)))
    monkeypatch.setitem(sys.modules, "launch", NS(LaunchDescription=lambda actions: actions))
    monkeypatch.setitem(sys.modules, "launch.actions", NS(
        DeclareLaunchArgument=lambda name, **kw: NS(name=name, **kw), LogInfo=Action, OpaqueFunction=Action))
    monkeypatch.setitem(sys.modules, "launch.substitutions", NS(LaunchConfiguration=Configuration))
    monkeypatch.setitem(sys.modules, "launch_ros.actions", NS(Node=Action))
    spec = importlib.util.spec_from_file_location("s1_launch", REPO / "launch/s1_fast_golden.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mode,img,imu", [("lo", 0, False), ("lio", 0, True), ("livo", 1, True)])
def test_modes(launch_module, mode, img, imu):
    paths, overrides, _ = launch_module.resolve_contract(REPO, mode)
    assert paths == [str(REPO / f"config/ugv_v1_{mode}.yaml")]
    assert type(overrides["common.img_en"]) is int and overrides["common.img_en"] == img
    assert overrides["common.lidar_en"] == 1
    assert overrides["imu.imu_en"] is imu
    assert overrides["common.enable_image_processing"] is False


@pytest.mark.parametrize("mode", ["LO", "", "unknown", " lio"])
def test_invalid_mode(launch_module, mode):
    with pytest.raises(RuntimeError, match="mode must"):
        launch_module.resolve_contract(REPO, mode)


def test_custom_files_order_and_authoritative_modes(launch_module, tmp_path):
    base = tmp_path / "base.yaml"
    camera = tmp_path / "camera.yaml"
    base.write_text("/**: {ros__parameters: {common: {img_en: 1, img_topic: /custom}, imu: {imu_en: true}}}")
    camera.write_text("/**: {ros__parameters: {camera: {fx: 123.4}, common: {img_en: 1}}}")
    before = [base.read_bytes(), camera.read_bytes()]
    paths, overrides, topic = launch_module.resolve_contract(REPO, "lo", str(base), str(camera))
    assert paths == [str(base), str(camera)]
    assert overrides["common.img_en"] == 0 and overrides["imu.imu_en"] is False
    assert "camera.fx" not in overrides and topic == "/custom"
    assert before == [base.read_bytes(), camera.read_bytes()]


@pytest.mark.parametrize("kwargs", [
    {"input_mode": "bad"}, {"input_mode": "compressed_direct"},
    *[{"image_topic": topic} for topic in ["/", "relative", "/bad topic", "/x/", "/x//y", "/1bad", " /x"]],
])
def test_invalid_input(launch_module, kwargs):
    with pytest.raises(RuntimeError): launch_module.resolve_contract(REPO, **kwargs)


def test_compressed_input(launch_module):
    _, overrides, topic = launch_module.resolve_contract(REPO, input_mode="compressed_direct", image_topic="/image/compressed")
    assert overrides["common.enable_image_processing"] is True
    assert overrides["common.img_topic"] == topic == "/image/compressed"


@pytest.mark.parametrize("content", ["[", "[]", "{}", "/**: {}", "/**: {ros__parameters: []}", "/wrong: {ros__parameters: {x: 1}}"])
def test_malformed_profile(launch_module, tmp_path, content):
    path = tmp_path / "bad.yaml"; path.write_text(content)
    with pytest.raises(RuntimeError): launch_module.resolve_contract(REPO, params_file=str(path))


def test_missing_profile(launch_module):
    with pytest.raises(RuntimeError): launch_module.resolve_contract(REPO, params_file="/missing/s1.yaml")


@pytest.mark.parametrize("rviz", ["true", "false"])
def test_launch_processes_and_parameter_order(launch_module, monkeypatch, rviz):
    monkeypatch.setattr(launch_module, "validate_livox_runtime", lambda: "/sensor/lib/typesupport.so")
    defaults = {a.name: a.default_value for a in launch_module.generate_launch_description() if hasattr(a, "name")}
    assert defaults["mode"] == "livo" and defaults["use_sim_time"] == defaults["use_rviz"] == "true"
    defaults["use_rviz"] = rviz
    actions = launch_module.launch_setup(defaults)
    nodes = [a for a in actions if hasattr(a, "package")]
    assert [(a.package, a.executable) for a in nodes] == [("fast_livo", "fastlivo_mapping")] + ([("rviz2", "rviz2")] if rviz == "true" else [])
    assert nodes[0].parameters[-1]["common.img_en"] == 1
    assert nodes[0].parameters[-1]["use_sim_time"] is True
    assert "profiles=" in actions[0].msg and "use_sim_time=True" in actions[0].msg


def test_livox_runtime_fail_closed(launch_module, monkeypatch, tmp_path):
    monkeypatch.setattr(launch_module, "get_package_prefix", lambda _: str(tmp_path))
    with pytest.raises(RuntimeError, match="typesupport"): launch_module.validate_livox_runtime()
    lib = tmp_path / "lib"; lib.mkdir()
    (lib / "liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so").touch()
    monkeypatch.setenv("LD_LIBRARY_PATH", "")
    with pytest.raises(RuntimeError, match="LD_LIBRARY_PATH"): launch_module.validate_livox_runtime()
    monkeypatch.setenv("LD_LIBRARY_PATH", str(lib))
    assert launch_module.validate_livox_runtime().startswith(str(lib))


def test_golden_rviz():
    config = yaml.safe_load((REPO / "rviz_cfg/fast_livo2_golden.rviz").read_text())["Visualization Manager"]
    assert config["Global Options"]["Fixed Frame"] == "camera_init"
    def flatten(displays):
        for item in displays:
            yield item
            yield from flatten(item.get("Displays", []))
    displays = list(flatten(config["Displays"]))
    for topic in ("/aft_mapped_to_init", "/path", "/cloud_registered_metric", "/rgb_img"):
        assert any(d.get("Topic", {}).get("Value") == topic and d["Enabled"] for d in displays)
    assert any(d["Class"] == "rviz_default_plugins/TF" for d in displays)
    assert any("GOLDEN" in d["Name"] and d.get("Topic", {}).get("Value") == "/cloud_registered_metric" for d in displays)
    assert all("diagnostic" in d["Name"] for d in displays if d.get("Topic", {}).get("Value") == "/cloud_registered")


def test_native_publication_float64_bit_exact(tmp_path):
    source = (REPO / "src/LIVMapper.cpp").read_text()
    def function(signature):
        start = source.index(signature)
        opening = source.index("{", start)
        depth = 1; end = opening + 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        return source[start:end]
    bodies = "\n".join(function(s) for s in (
        "template <typename T> void LIVMapper::set_posestamp",
        "void LIVMapper::publish_odometry("))
    template = (Path(__file__).parent / "fixtures/publication_probe.cpp").read_text()
    probe = tmp_path / "probe.cpp"; probe.write_text(template.replace("// PUBLICATION_FUNCTIONS", bodies))
    prefixes = [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
    prefixes += sorted(Path("/opt/ros").glob("*"))
    includes = [p / "include" / pkg for p in prefixes for pkg in ("nav_msgs", "geometry_msgs", "std_msgs", "builtin_interfaces", "rosidl_runtime_cpp", "rosidl_runtime_c", "rosidl_typesupport_interface")]
    binary = tmp_path / "probe"
    compiled = subprocess.run(["c++", "-std=c++17", *[f"-I{p}" for p in includes], str(probe), "-o", str(binary)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    subprocess.run([str(binary)], check=True)
