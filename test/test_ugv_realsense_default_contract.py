"""Reject Sony fallback for UGV; preserve explicit camera profile selection."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FILE = ROOT / "launch" / "mapping_ugv.launch.py"


def module():
    spec = importlib.util.spec_from_file_location("ugv_camera_contract", FILE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_empty_camera_parameter_selects_realsense_for_b3():
    m = module()
    profiles, overrides, topic = m.resolve_contract(
        str(ROOT), mode="ugv_realsense_livo.yaml",
        camera_params_file="", input_mode="raw", image_topic="", use_sim_time=False)
    assert profiles[0] == str(ROOT / "config" / "ugv_realsense_livo.yaml")
    assert profiles[1] == str(ROOT / "config" / "camera_realsense.yaml")
    assert "sony" not in str(profiles)
    assert topic == "/camera/camera/color/image_raw"
    assert overrides["common.enable_image_processing"] is False


def test_explicit_realsense_is_same_as_default():
    m = module()
    a = m.resolve_contract(str(ROOT), mode="ugv_realsense_livo.yaml", camera_params_file="")
    b = m.resolve_contract(str(ROOT), mode="ugv_realsense_livo.yaml", camera_params_file="camera_realsense.yaml")
    assert a == b


@pytest.mark.parametrize("mode", ["ugv_livo.yaml", "ugv_lio.yaml", "ugv_realsense_livo.yaml"])
def test_default_does_not_select_sony(mode):
    m = module()
    profiles, _, _ = m.resolve_contract(str(ROOT), mode=mode, camera_params_file="")
    assert profiles[1] == str(ROOT / "config" / "camera_realsense.yaml")


def test_explicit_camera_choice_remains_supported():
    m = module()
    paths, _, _ = m.resolve_contract(str(ROOT), mode="ugv_livo.yaml",
                                      camera_params_file="camera_sony_ev9520l.yaml")
    assert paths[1] == str(ROOT / "config" / "camera_sony_ev9520l.yaml")


def test_default_launch_argument_is_realsense():
    source = FILE.read_text()
    assert '"camera_params_file": "camera_realsense.yaml"' in source
    assert 'name = "camera_realsense.yaml"' in source
    assert 'name = "camera_sony_ev9520l.yaml"' not in source
