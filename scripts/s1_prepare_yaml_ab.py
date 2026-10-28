#!/usr/bin/env python3
"""Prepare the four S1 YAML-only A/B profiles from the canonical LIVO YAML."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = REPO_ROOT / "config" / "ugv_livo.yaml"
MANIFEST_NAME = "s1_yaml_ab_manifest.json"

PROFILE_VALUES = {
    "B0": (0.1, 10.0),
    "B1": (0.15, 10.0),
    "B2": (0.1, 7.5),
    "B3": (0.15, 7.5),
}

PATHS = {
    "filter_size_surf": ("/**", "ros__parameters", "preprocess", "filter_size_surf"),
    "image_admission_hz": ("/**", "ros__parameters", "runtime", "image_admission_hz"),
    "diagnostics_ingest_enabled": (
        "/**",
        "ros__parameters",
        "diagnostics",
        "ingest",
        "enabled",
    ),
}

BASELINE = {
    PATHS["filter_size_surf"]: 0.1,
    PATHS["image_admission_hz"]: 10.0,
    PATHS["diagnostics_ingest_enabled"]: False,
}


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(
    loader: UniqueKeyLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict:
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"duplicate YAML key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_yaml(data: bytes, label: str) -> dict:
    try:
        loaded = yaml.load(data.decode("utf-8"), Loader=UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError, ValueError) as exc:
        raise RuntimeError(f"invalid {label}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise RuntimeError(f"invalid {label}: root must be a mapping")
    return loaded


def _get(tree: dict, path: tuple[str, ...], label: str) -> Any:
    value: Any = tree
    for key in path:
        if not isinstance(value, dict) or key not in value:
            raise RuntimeError(f"{label} is missing canonical key {'.'.join(path)}")
        value = value[key]
    return value


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _replace_line_once(data: bytes, old: bytes, new: bytes, label: str) -> bytes:
    if data.count(old) != 1:
        raise RuntimeError(f"canonical source has unexpected representation for {label}")
    return data.replace(old, new, 1)


def _render(source: bytes, filter_size: float, image_hz: float) -> bytes:
    rendered = _replace_line_once(
        source,
        b"      filter_size_surf: 0.1\n",
        f"      filter_size_surf: {filter_size}\n".encode(),
        "preprocess.filter_size_surf",
    )
    rendered = _replace_line_once(
        rendered,
        b"      image_admission_hz: 10.0\n",
        f"      image_admission_hz: {image_hz}\n".encode(),
        "runtime.image_admission_hz",
    )
    return _replace_line_once(
        rendered,
        b"    diagnostics:\n      ingest:\n        enabled: false\n",
        b"    diagnostics:\n      ingest:\n        enabled: true\n",
        "diagnostics.ingest.enabled",
    )


def _changed_paths(
    before: Any,
    after: Any,
    path: tuple[str, ...] = (),
) -> set[tuple[str, ...]]:
    if isinstance(before, dict) and isinstance(after, dict):
        if before.keys() != after.keys():
            return {path}
        changes: set[tuple[str, ...]] = set()
        for key in before:
            changes.update(_changed_paths(before[key], after[key], path + (str(key),)))
        return changes
    if isinstance(before, list) and isinstance(after, list):
        if len(before) != len(after):
            return {path}
        changes = set()
        for index, (left, right) in enumerate(zip(before, after)):
            changes.update(_changed_paths(left, right, path + (str(index),)))
        return changes
    return set() if before == after and type(before) is type(after) else {path}


def _validate_profile(
    source_tree: dict,
    output: bytes,
    profile: str,
    filter_size: float,
    image_hz: float,
) -> None:
    output_tree = _load_yaml(output, f"generated {profile} YAML")
    expected = copy.deepcopy(source_tree)
    expected["/**"]["ros__parameters"]["preprocess"]["filter_size_surf"] = filter_size
    expected["/**"]["ros__parameters"]["runtime"]["image_admission_hz"] = image_hz
    expected["/**"]["ros__parameters"]["diagnostics"]["ingest"]["enabled"] = True
    if output_tree != expected:
        raise RuntimeError(f"generated {profile} does not match the requested parameter values")

    allowed = set(BASELINE)
    changed = _changed_paths(source_tree, output_tree)
    if not changed.issubset(allowed):
        formatted = ", ".join(".".join(path) for path in sorted(changed - allowed))
        raise RuntimeError(f"generated {profile} changed unexpected parameters: {formatted}")


def _git_head() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    head = result.stdout.strip()
    if result.returncode or len(head) != 40:
        raise RuntimeError("cannot determine git HEAD")
    return head


def _expected_files(source: bytes) -> dict[str, bytes]:
    source_tree = _load_yaml(source, "canonical source YAML")
    for path, expected in BASELINE.items():
        actual = _get(source_tree, path, "canonical source YAML")
        if actual != expected or type(actual) is not type(expected):
            raise RuntimeError(
                f"canonical key {'.'.join(path)} has unexpected value {actual!r}; "
                f"expected {expected!r}"
            )

    outputs: dict[str, bytes] = {}
    manifest_profiles = {}
    for profile, (filter_size, image_hz) in PROFILE_VALUES.items():
        filename = f"s1_{profile.lower()}_ugv_livo.yaml"
        data = _render(source, filter_size, image_hz)
        _validate_profile(source_tree, data, profile, filter_size, image_hz)
        outputs[filename] = data
        manifest_profiles[profile] = {
            "filename": filename,
            "params": {
                "diagnostics.ingest.enabled": True,
                "preprocess.filter_size_surf": filter_size,
                "runtime.image_admission_hz": image_hz,
            },
            "sha256": _sha256(data),
        }

    manifest = {
        "git_head": _git_head(),
        "profiles": manifest_profiles,
        "source": {
            "filename": str(SOURCE_PATH.relative_to(REPO_ROOT)),
            "sha256": _sha256(source),
        },
    }
    outputs[MANIFEST_NAME] = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    return outputs


def prepare(output_dir: Path) -> list[Path]:
    source_before = SOURCE_PATH.read_bytes()
    expected = _expected_files(source_before)
    if SOURCE_PATH.read_bytes() != source_before:
        raise RuntimeError("canonical source changed while profiles were being prepared")

    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists() and not output_dir.is_dir():
        raise RuntimeError(f"output path is not a directory: {output_dir}")
    if output_dir.exists():
        existing = {path.name for path in output_dir.iterdir()}
        wanted = set(expected)
        if existing and existing != wanted:
            raise RuntimeError(f"output directory has unexpected contents: {output_dir}")
        for name, data in expected.items():
            path = output_dir / name
            if path.exists() and (not path.is_file() or path.read_bytes() != data):
                raise RuntimeError(f"refusing to overwrite different content: {path}")
    else:
        output_dir.mkdir(parents=True)

    written = []
    for name, data in expected.items():
        path = output_dir / name
        if not path.exists():
            path.write_bytes(data)
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        paths = prepare(args.output_dir)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f"error: {exc}\n")
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
