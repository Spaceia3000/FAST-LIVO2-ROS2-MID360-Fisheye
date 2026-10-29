"""UGV RealSense profile and no-runtime LIO contracts."""
from pathlib import Path
import copy
import hashlib
import yaml

REPO=Path(__file__).resolve().parents[1]
def params(name):
    return yaml.safe_load((REPO/"config"/name).read_text())["/**"]["ros__parameters"]

def test_candidate_changes_only_historic_b3_knobs():
    base=params("ugv_livo.yaml")
    candidate=params("ugv_realsense_livo.yaml")
    expected=copy.deepcopy(base)
    expected["preprocess"]["filter_size_surf"]=0.15
    expected["runtime"]["image_admission_hz"]=7.5
    expected["diagnostics"]["ingest"]["enabled"]=True
    assert candidate==expected
    assert base["runtime"]["image_admission_hz"]==10.0
    assert base["runtime"]["livo_camera_wait_sensor_s"]==0.30

def test_lio_stays_camera_free_without_visual_runtime_params():
    lio=params("ugv_lio.yaml")
    lo=params("ugv_lo.yaml")
    assert lio["common"]["img_en"]==0 and lio["imu"]["imu_en"] is True
    assert lo["common"]["img_en"]==0 and lo["imu"]["imu_en"] is False
    assert "runtime" not in lio and "runtime" not in lo

def test_calibration_profile_and_platform_are_unchanged():
    reference=params("ugv_livo.yaml")
    candidate=params("ugv_realsense_livo.yaml")
    for name in ["extrin_calib","time_offset","common","imu","lio"]:
        assert candidate[name]==reference[name]
    camera=params("camera_realsense.yaml")["camera"]
    assert (camera["width"],camera["height"])==(640,480)
    assert camera["model"]=="Pinhole"

def test_rviz_is_platform_specific_and_no_historical_rgb_accumulation():
    launch=(REPO/"launch/mapping_ugv.launch.py").read_text()
    assert '"rviz_profile": "light"' in launch
    assert 'select_rviz_filename(mode_stem, arg("rviz_profile"))' in launch
    assert 'rviz = Path(requested_rviz) if requested_rviz else' in launch
    for name in ["ugv_livo.rviz","ugv_lio.rviz"]:
        content=(REPO/"rviz_cfg"/name).read_text()
        assert "Name: Legacy accumulated diagnostic" in content
        assert "Decay Time: 10000" not in content

def test_b3_manifest_checksum_matches_actual_yaml():
    manifest=yaml.safe_load((REPO/"reports/experiments/ugv_realsense/b3_202610/manifest.yaml").read_text())
    for label,path in [("baseline","ugv_livo.yaml"),("candidate","ugv_realsense_livo.yaml"),("camera","camera_realsense.yaml")]:
        digest=hashlib.sha256((REPO/"config"/path).read_bytes()).hexdigest()
        assert digest==manifest[label+"_sha256"]
