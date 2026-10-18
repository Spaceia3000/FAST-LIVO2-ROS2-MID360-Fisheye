"""UGV launch and native publication regression; no bag playback."""
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
    spec = importlib.util.spec_from_file_location("mapping_ugv_launch", REPO / "launch/mapping_ugv.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mode,img,imu", [("lo", 0, False), ("lio", 0, True), ("livo", 1, True)])
def test_modes(launch_module, mode, img, imu):
    paths, overrides, _ = launch_module.resolve_contract(REPO, mode)
    assert paths == [
        str(REPO / f"config/ugv_{mode}.yaml"),
        str(REPO / "config/camera_realsense.yaml"),
    ]
    profile = launch_module.load_parameters(paths[0])
    assert type(profile["common.img_en"]) is int and profile["common.img_en"] == img
    assert profile["common.lidar_en"] == 1
    assert profile["imu.imu_en"] is imu
    assert overrides == {"use_sim_time": True, "common.enable_image_processing": False}


@pytest.mark.parametrize("mode", ["LO", "", "unknown", " lio"])
def test_invalid_mode(launch_module, mode):
    with pytest.raises(RuntimeError, match="mode must"):
        launch_module.resolve_contract(REPO, mode)


def test_custom_files_order_without_implicit_overrides(launch_module, tmp_path):
    base = tmp_path / "base.yaml"
    camera = tmp_path / "camera.yaml"
    base.write_text("/**: {ros__parameters: {common: {img_en: 1, lidar_en: 1, img_topic: /custom}, imu: {imu_en: true}, runtime: {ingest_spin_budget_ms: 7, image_admission_hz: 12.5}}}")
    camera.write_text("/**: {ros__parameters: {camera: {fx: 123.4}, common: {img_en: 1}}}")
    before = [base.read_bytes(), camera.read_bytes()]
    paths, overrides, topic = launch_module.resolve_contract(REPO, "livo", str(base), str(camera))
    assert paths == [str(base), str(camera)]
    effective = {}
    for path in paths:
        effective.update(launch_module.load_parameters(path))
    assert effective["common.img_en"] == 1 and effective["imu.imu_en"] is True
    assert effective["runtime.ingest_spin_budget_ms"] == 7
    assert effective["runtime.image_admission_hz"] == 12.5
    assert not set(overrides) & {
        "common.img_en", "common.lidar_en", "imu.imu_en",
        "runtime.ingest_spin_budget_ms", "runtime.image_admission_hz",
        "vio.max_visual_points_per_voxel", "diagnostics.ingest.enabled",
    }
    assert "camera.fx" not in overrides and topic == "/custom"
    assert before == [base.read_bytes(), camera.read_bytes()]


@pytest.mark.parametrize("mode,profile", [
    ("lio", "ugv_livo.yaml"),
    ("lo", "avia.yaml"),
])
def test_mismatched_base_profile_fails_closed(launch_module, mode, profile):
    with pytest.raises(RuntimeError, match="incompatible"):
        launch_module.resolve_contract(REPO, mode, str(REPO / "config" / profile))


def test_camera_profile_cannot_override_mode(launch_module, tmp_path):
    camera = tmp_path / "camera.yaml"
    camera.write_text("/**: {ros__parameters: {common: {img_en: 1}}}")
    with pytest.raises(RuntimeError, match="common.img_en.*incompatible"):
        launch_module.resolve_contract(REPO, "lio", camera_params_file=str(camera))


@pytest.mark.parametrize("parameters,match", [
    ({"common": {"img_en": 0, "lidar_en": 1}}, "missing mode flag.*imu.imu_en"),
    ({"common": {"img_en": False, "lidar_en": 1}, "imu": {"imu_en": True}}, "common.img_en.*incompatible"),
    ({"common": {"img_en": 0, "lidar_en": True}, "imu": {"imu_en": True}}, "common.lidar_en.*incompatible"),
    ({"common": {"img_en": 0, "lidar_en": 1}, "imu": {"imu_en": 1}}, "imu.imu_en.*incompatible"),
])
def test_missing_or_typed_invalid_mode_flags(launch_module, tmp_path, parameters, match):
    profile = tmp_path / "profile.yaml"
    profile.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    with pytest.raises(RuntimeError, match=match):
        launch_module.resolve_contract(REPO, "lio", str(profile))


def test_custom_valid_profile_uses_canonical_flags_from_share(launch_module, tmp_path):
    share = tmp_path / "share"
    config = share / "config"
    config.mkdir(parents=True)
    canonical = REPO / "config/ugv_lio.yaml"
    (config / canonical.name).write_bytes(canonical.read_bytes())
    (config / "camera_realsense.yaml").write_bytes(
        (REPO / "config/camera_realsense.yaml").read_bytes()
    )
    custom = tmp_path / "custom.yaml"
    custom.write_text(
        "/**: {ros__parameters: {common: {img_en: 0, lidar_en: 1, img_topic: /custom}, "
        "imu: {imu_en: true}, camera: {fx: 123.4}}}"
    )
    paths, _, topic = launch_module.resolve_contract(share, "lio", str(custom))
    assert paths == [str(custom), str(config / "camera_realsense.yaml")]
    assert topic == "/custom"


def test_nested_duplicate_yaml_key_is_rejected(launch_module, tmp_path):
    profile = tmp_path / "duplicate.yaml"
    profile.write_text(
        "/**:\n  ros__parameters:\n    common:\n      img_en: 0\n      img_en: 1\n"
        "      lidar_en: 1\n    imu:\n      imu_en: true\n"
    )
    with pytest.raises(RuntimeError, match="Cannot load parameter profile"):
        launch_module.load_parameters(profile)


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
    with pytest.raises(RuntimeError): launch_module.resolve_contract(REPO, params_file="/missing/ugv.yaml")


@pytest.mark.parametrize("rviz", ["true", "false"])
def test_launch_processes_and_parameter_order(launch_module, monkeypatch, rviz):
    monkeypatch.setattr(launch_module, "validate_livox_runtime", lambda: "/sensor/lib/typesupport.so")
    defaults = {a.name: a.default_value for a in launch_module.generate_launch_description() if hasattr(a, "name")}
    assert defaults["mode"] == "livo" and defaults["use_sim_time"] == defaults["use_rviz"] == "true"
    assert set(defaults) == {
        "mode", "params_file", "camera_params_file", "input_mode", "image_topic",
        "use_sim_time", "use_rviz", "rviz_config",
    }
    defaults["use_rviz"] = rviz
    actions = launch_module.launch_setup(defaults)
    nodes = [a for a in actions if hasattr(a, "package")]
    assert [(a.package, a.executable) for a in nodes] == [("fast_livo", "fastlivo_mapping")] + ([("rviz2", "rviz2")] if rviz == "true" else [])
    assert nodes[0].parameters == [
        str(REPO / "config/ugv_livo.yaml"),
        str(REPO / "config/camera_realsense.yaml"),
        {"use_sim_time": True, "common.enable_image_processing": False},
    ]
    assert "profiles=" in actions[0].msg and "use_sim_time=True" in actions[0].msg


def test_livo_yaml_controls_runtime_options(launch_module):
    profile = launch_module.load_parameters(REPO / "config/ugv_livo.yaml")
    assert profile["runtime.ingest_spin_budget_ms"] == 5
    assert profile["runtime.image_admission_hz"] == 10.0
    assert profile["diagnostics.ingest.enabled"] is False
    assert profile["vio.max_visual_points_per_voxel"] == 64


def test_cpp_fallbacks_apply_when_yaml_omits_runtime_options(launch_module):
    for mode in ("lo", "lio"):
        profile = launch_module.load_parameters(REPO / f"config/ugv_{mode}.yaml")
        assert "runtime.ingest_spin_budget_ms" not in profile
        assert "runtime.image_admission_hz" not in profile
        assert "diagnostics.ingest.enabled" not in profile
        assert "vio.max_visual_points_per_voxel" not in profile
    source = (REPO / "src/LIVMapper.cpp").read_text()
    for declaration in (
        'operator()<int>("runtime.ingest_spin_budget_ms", 0)',
        'operator()<double>("runtime.image_admission_hz", 50.0)',
        'operator()<bool>("diagnostics.ingest.enabled", false)',
        'operator()<std::int64_t>("vio.max_visual_points_per_voxel", 0)',
    ):
        assert declaration in source


def test_canonical_camera_and_extrinsics_match_across_modes(launch_module):
    profiles = [
        launch_module.load_parameters(REPO / f"config/ugv_{mode}.yaml")
        for mode in ("lo", "lio", "livo")
    ]
    assert all(
        not any(key.startswith("camera.") for key in profile)
        for profile in profiles
    ), "UGV mode profiles must not duplicate camera intrinsics"
    protected = [key for key in profiles[0] if key.startswith("extrin_calib.")]
    assert protected
    assert all(
        profile[key] == profiles[0][key]
        for profile in profiles[1:]
        for key in protected
    )


def test_image_admission_policy_offline(tmp_path):
    source = tmp_path / "image_admission_probe.cpp"
    source.write_text(r'''
#include "image_admission.h"
#include <cassert>

int main()
{
  using D = ImageAdmissionDecision;
  assert(imageAdmissionDecision(100.0, -1.0, 0.1) == D::ACCEPT);
  assert(imageAdmissionDecision(100.0005, 100.0, 0.1) == D::DUPLICATE);
  assert(imageAdmissionDecision(99.0, 100.0, 0.1) == D::OUT_OF_ORDER);
  assert(imageAdmissionDecision(100.124, 100.0, 0.125) == D::TOO_SOON);
  assert(imageAdmissionDecision(100.125, 100.0, 0.125) == D::ACCEPT);
  assert(imageAdmissionDecision(0.02, 0.0, 0.02) == D::ACCEPT);
}
''')
    binary = tmp_path / "image_admission_probe"
    compiled = subprocess.run(
        ["c++", "-std=c++17", f"-I{REPO / 'include'}", str(source), "-o", str(binary)],
        capture_output=True, text=True,
    )
    assert compiled.returncode == 0, compiled.stderr
    subprocess.run([str(binary)], check=True)


def test_livox_runtime_fail_closed(launch_module, monkeypatch, tmp_path):
    monkeypatch.setattr(launch_module, "get_package_prefix", lambda _: str(tmp_path))
    with pytest.raises(RuntimeError, match="typesupport"): launch_module.validate_livox_runtime()
    lib = tmp_path / "lib"; lib.mkdir()
    (lib / "liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so").touch()
    monkeypatch.setenv("LD_LIBRARY_PATH", "")
    with pytest.raises(RuntimeError, match="LD_LIBRARY_PATH"): launch_module.validate_livox_runtime()
    monkeypatch.setenv("LD_LIBRARY_PATH", str(lib))
    assert launch_module.validate_livox_runtime().startswith(str(lib))


def test_ugv_rviz():
    config = yaml.safe_load((REPO / "rviz_cfg/mapping_ugv.rviz").read_text())["Visualization Manager"]
    assert config["Global Options"]["Fixed Frame"] == "camera_init"
    def flatten(displays):
        for item in displays:
            yield item
            yield from flatten(item.get("Displays", []))
    displays = list(flatten(config["Displays"]))
    for topic in ("/aft_mapped_to_init", "/path", "/cloud_registered_metric", "/rgb_img"):
        assert any(d.get("Topic", {}).get("Value") == topic and d["Enabled"] for d in displays)
    assert any(d["Class"] == "rviz_default_plugins/TF" for d in displays)
    assert any("UGV" in d["Name"] and d.get("Topic", {}).get("Value") == "/cloud_registered_metric" for d in displays)
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

@pytest.mark.parametrize("mode", ["lo", "lio", "livo"])
@pytest.mark.parametrize("camera_name,expected_size", [
    ("camera_realsense.yaml", (640, 480)),
    ("camera_sony_ev9520l.yaml", (1280, 720)),
])
def test_camera_selected_by_filename_and_mode_contract(launch_module, mode, camera_name, expected_size):
    paths, overrides, _ = launch_module.resolve_contract(
        REPO, mode, camera_params_file=camera_name
    )
    assert paths == [
        str(REPO / "config" / f"ugv_{mode}.yaml"),
        str(REPO / "config" / camera_name),
    ]
    base = launch_module.load_parameters(paths[0])
    cam = launch_module.load_parameters(paths[1])
    assert "camera.model" not in base
    assert (cam["camera.width"], cam["camera.height"]) == expected_size
    assert cam["camera.model"] == "Pinhole"
    for key in ("fx", "fy", "cx", "cy", "d0", "d1", "d2", "d3"):
        assert f"camera.{key}" in cam
    assert not any(k.startswith("extrin_calib.") for k in cam)
    assert "common.img_en" not in cam
    assert "imu.imu_en" not in overrides


def test_bare_mode_filename_uses_package_config(launch_module):
    paths, _, _ = launch_module.resolve_contract(REPO, "lio", params_file="ugv_lio.yaml")
    assert paths == [
        str(REPO / "config/ugv_lio.yaml"),
        str(REPO / "config/camera_realsense.yaml"),
    ]


@pytest.mark.parametrize("bad_name", ["../other.yaml", "./other.yaml", "config/other.yaml", "foo.txt"])
def test_camera_relative_path_rejected(launch_module, bad_name):
    with pytest.raises(RuntimeError, match="filename"):
        launch_module.resolve_contract(REPO, "livo", camera_params_file=bad_name)


def test_missing_named_camera_fails_closed(launch_module):
    with pytest.raises(RuntimeError, match="Cannot load parameter profile"):
        launch_module.resolve_contract(REPO, "livo", camera_params_file="camera_unknown.yaml")


def test_legacy_livo_launch_declares_both_configs():
    source = (REPO / "launch/mapping_ugv_livo.launch.py").read_text()
    assert '"ugv_livo.yaml"' in source
    assert '"camera_realsense.yaml"' in source
    assert 'LaunchConfiguration("camera_params_file")' in source

def test_camera_provenance_sha256_matches_versioned_profiles():
    import hashlib
    path = REPO / "test/system_validation/profiles/ugv/provenance.yaml"
    records = yaml.safe_load(path.read_text())["camera_files"]
    assert set(records) == {"realsense", "sony_ev9520l"}
    for record in records.values():
        file = REPO / record["file"]
        assert file.is_file()
        assert hashlib.sha256(file.read_bytes()).hexdigest() == record["sha256"]
