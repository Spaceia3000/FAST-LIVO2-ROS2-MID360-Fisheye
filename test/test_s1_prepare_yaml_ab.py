import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "s1_prepare_yaml_ab.py"
SOURCE = REPO_ROOT / "config" / "ugv_livo.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("s1_prepare_yaml_ab", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _params(tree):
    return tree["/**"]["ros__parameters"]


def test_generates_full_profiles_manifest_and_is_idempotent(tmp_path):
    source_bytes = SOURCE.read_bytes()
    source_tree = _load(SOURCE)
    output_dir = tmp_path / "profiles"

    first = subprocess.run(
        [sys.executable, str(SCRIPT), "--output-dir", str(output_dir)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    snapshot = {path.name: path.read_bytes() for path in output_dir.iterdir()}
    second = subprocess.run(
        [sys.executable, str(SCRIPT), "--output-dir", str(output_dir)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert first.stdout == second.stdout
    assert snapshot == {path.name: path.read_bytes() for path in output_dir.iterdir()}
    assert SOURCE.read_bytes() == source_bytes

    values = {
        "B0": (0.1, 10.0),
        "B1": (0.15, 10.0),
        "B2": (0.1, 7.5),
        "B3": (0.15, 7.5),
    }
    manifest = json.loads((output_dir / "s1_yaml_ab_manifest.json").read_text())
    assert manifest["git_head"] == subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
    ).strip()
    assert manifest["source"] == {
        "filename": "config/ugv_livo.yaml",
        "sha256": hashlib.sha256(source_bytes).hexdigest(),
    }

    for profile, (filter_size, image_hz) in values.items():
        record = manifest["profiles"][profile]
        path = output_dir / record["filename"]
        generated = _load(path)
        expected = _load(SOURCE)
        params = _params(expected)
        params["preprocess"]["filter_size_surf"] = filter_size
        params["runtime"]["image_admission_hz"] = image_hz
        params["diagnostics"]["ingest"]["enabled"] = True
        assert generated == expected
        assert record["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
        assert record["params"] == {
            "diagnostics.ingest.enabled": True,
            "preprocess.filter_size_surf": filter_size,
            "runtime.image_admission_hz": image_hz,
        }
        generated_params = _params(generated)
        source_params = _params(source_tree)
        assert (
            generated_params["publish"]["dense_map_en"]
            is source_params["publish"]["dense_map_en"]
        )
        assert (
            generated_params["publish"]["metric_cloud_en"]
            is source_params["publish"]["metric_cloud_en"]
        )

        restored = path.read_bytes()
        restored = restored.replace(
            b"    diagnostics:\n      ingest:\n        enabled: true\n",
            b"    diagnostics:\n      ingest:\n        enabled: false\n",
        )
        restored = restored.replace(
            f"      filter_size_surf: {filter_size}\n".encode(),
            b"      filter_size_surf: 0.1\n",
        )
        restored = restored.replace(
            f"      image_admission_hz: {image_hz}\n".encode(),
            b"      image_admission_hz: 10.0\n",
        )
        assert restored == source_bytes


def test_refuses_different_or_extra_output_content(tmp_path):
    module = _module()
    output_dir = tmp_path / "profiles"
    module.prepare(output_dir)
    target = output_dir / "s1_b0_ugv_livo.yaml"
    target.write_text("different\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="refusing to overwrite different content"):
        module.prepare(output_dir)

    target.write_bytes(module._expected_files(SOURCE.read_bytes())[target.name])
    (output_dir / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="unexpected contents"):
        module.prepare(output_dir)


def test_rejects_duplicate_keys_and_canonical_contract_drift(tmp_path):
    module = _module()
    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text("a: 1\na: 2\n", encoding="utf-8")
    module.SOURCE_PATH = duplicate
    with pytest.raises(RuntimeError, match="duplicate YAML key"):
        module.prepare(tmp_path / "duplicate-output")

    missing = tmp_path / "missing.yaml"
    missing.write_bytes(
        SOURCE.read_bytes().replace(b"      image_admission_hz: 10.0\n", b"")
    )
    module.SOURCE_PATH = missing
    with pytest.raises(RuntimeError, match="missing canonical key"):
        module.prepare(tmp_path / "missing-output")

    drifted = tmp_path / "drifted.yaml"
    drifted.write_bytes(
        SOURCE.read_bytes().replace(
            b"filter_size_surf: 0.1",
            b"filter_size_surf: 0.2",
        )
    )
    module.SOURCE_PATH = drifted
    with pytest.raises(RuntimeError, match="unexpected value"):
        module.prepare(tmp_path / "drifted-output")
