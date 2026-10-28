#!/usr/bin/env python3
"""Standalone UGV FAST modes, native image input, and optional RViz."""
import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODE_FLAGS = {
    "lo": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": False},
    "lio": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": True},
    "livo": {"common.img_en": 1, "common.lidar_en": 1, "imu.imu_en": True},
}
MODE_FLAG_KEYS = ("common.img_en", "common.lidar_en", "imu.imu_en")


class UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader which rejects duplicate keys at every mapping level."""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        mapping = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                duplicate = key in mapping
            except TypeError as error:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping", node.start_mark,
                    "found an unhashable key", key_node.start_mark,
                ) from error
            if duplicate:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping", node.start_mark,
                    f"found duplicate key {key!r}", key_node.start_mark,
                )
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    UniqueKeySafeLoader.construct_mapping,
)


def validate_livox_runtime():
    try:
        prefix = Path(get_package_prefix("livox_ros_driver2")).resolve()
    except LookupError as error:
        raise RuntimeError("Source the pinned livox_ros_driver2 sensor underlay") from error
    library_dir = (prefix / "lib").resolve()
    library = library_dir / "liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so"
    if not library.is_file():
        raise RuntimeError(f"Livox Fast RTPS typesupport is missing: {library}")
    loader_paths = {Path(p).resolve() for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p}
    if library_dir not in loader_paths:
        raise RuntimeError("Livox lib directory is absent from LD_LIBRARY_PATH; source the sensor underlay")
    return str(library)


def load_parameters(path):
    """Validate ROS parameter structure without rewriting user parameter files."""
    try:
        document = yaml.load(
            Path(path).read_text(encoding="utf-8"), Loader=UniqueKeySafeLoader,
        )
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Cannot load parameter profile: {path}") from error
    if not isinstance(document, dict) or not document:
        raise RuntimeError(f"Parameter profile must be a nonempty mapping: {path}")
    selected = {}
    for name, node in document.items():
        if not isinstance(name, str) or not isinstance(node, dict) or not isinstance(node.get("ros__parameters"), dict):
            raise RuntimeError(f"Malformed ROS parameter profile: {path}")
        if name in {"/**", "laserMapping", "/laserMapping"}:
            def flatten(values, prefix=""):
                for key, value in values.items():
                    if not isinstance(key, str):
                        raise RuntimeError(f"Invalid parameter name in {path}")
                    full = prefix + key
                    if isinstance(value, dict):
                        yield from flatten(value, full + ".")
                    else:
                        if value is None or not isinstance(value, (str, int, float, bool, list)):
                            raise RuntimeError(f"Invalid parameter value in {path}: {full}")
                        if isinstance(value, list) and value and (
                            type(value[0]) not in (str, int, float, bool)
                            or any(type(v) is not type(value[0]) for v in value)
                        ):
                            raise RuntimeError(f"Invalid parameter array in {path}: {full}")
                        yield full, value
            selected.update(flatten(node["ros__parameters"]))
    if not selected:
        raise RuntimeError(f"Profile has no parameters for laserMapping: {path}")
    return selected


def validate_mode_flags(effective, expected, mode):
    """Require effective mode flags to exactly match the canonical mode profile."""
    for key in MODE_FLAG_KEYS:
        if key not in expected or type(expected[key]) not in (bool, int):
            raise RuntimeError(f"Canonical {mode} profile has invalid mode flag: {key}")
        if key not in effective:
            raise RuntimeError(f"Effective parameters are missing mode flag for {mode}: {key}")
        if type(effective[key]) is not type(expected[key]) or effective[key] != expected[key]:
            raise RuntimeError(
                f"Effective mode flag {key}={effective[key]!r} is incompatible with {mode}; "
                f"canonical value is {expected[key]!r}"
            )


def validate_topic(topic):
    if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", topic):
        raise RuntimeError(f"image_topic must be an absolute ROS topic: {topic!r}")


def resolve_contract(share, mode="cohemo_livo.yaml", camera_params_file="",
                     input_mode="raw", image_topic="", use_sim_time=True):
    # A single profile filename selects the platform AND the LO/LIO/LIVO modality.
    # Restrict the operational mode selector to installed config files (no path traversal).
    if not isinstance(mode, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.yaml", mode):
        raise RuntimeError(f"mode must be a .yaml filename from package config/: {mode!r}")
    if input_mode not in {"raw", "compressed_direct"}:
        raise RuntimeError(f"invalid input_mode: {input_mode!r}")
    if input_mode == "compressed_direct" and not image_topic:
        raise RuntimeError("compressed_direct requires explicit image_topic")
    # Selected YAML is the only source of modality flags.
    mode_path = (Path(share) / "config" / mode).resolve()
    mode_params = load_parameters(mode_path)
    variant = None
    for candidate, flags in MODE_FLAGS.items():
        if all(type(mode_params.get(key)) is type(value) and mode_params[key] == value
               for key, value in flags.items()):
            variant = candidate
            break
    if variant is None:
        raise RuntimeError(f"Invalid LO/LIO/LIVO mode flags in {mode}")

    def camera_path(name):
        if not name:
            name = "camera_sony_ev9520l.yaml"
        path = Path(name)
        if path.is_absolute():
            return str(path.resolve())
        if path.name != name or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*\.yaml", name):
            raise RuntimeError("Camera profile must be a .yaml filename in config/ or an absolute path")
        return str((Path(share) / "config" / name).resolve())

    paths = [str(mode_path), camera_path(camera_params_file)]
    effective = dict(mode_params)
    effective.update(load_parameters(paths[1]))
    validate_mode_flags(effective, MODE_FLAGS[variant], variant)
    topic = image_topic or effective.get("common.img_topic", "/left_camera/image")
    validate_topic(topic)
    overrides = {
        "use_sim_time": use_sim_time,
        "common.enable_image_processing": input_mode == "compressed_direct",
    }
    if image_topic:
        overrides["common.img_topic"] = image_topic
    return paths, overrides, topic


def parse_bool(value):
    if value.lower() not in {"true", "false"}:
        raise RuntimeError(f"Expected true or false, got {value!r}")
    return value.lower() == "true"


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = get_package_share_directory("fast_livo")
    sim_time = parse_bool(arg("use_sim_time"))
    use_rviz = parse_bool(arg("use_rviz"))
    paths, overrides, topic = resolve_contract(
        share, arg("mode"), arg("camera_params_file"),
        arg("input_mode"), arg("image_topic"), sim_time,
    )
    livox = validate_livox_runtime()
    actions = [LogInfo(msg=(
        f"UGV FAST: profiles={paths}, mode={arg('mode')}, input_mode={arg('input_mode')}, "
        f"image_topic={topic}, use_sim_time={sim_time} (launch argument; "
        f"{'/clock' if sim_time else 'system clock'}), "
        f"livox_typesupport={livox}"
    )), Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
             output="screen", parameters=[*paths, overrides])]
    if use_rviz:
        # Choose a package-installed RViz profile by sensor modality.
        # Explicit rviz_config always wins; empty means automatic selection.
        requested_rviz = arg("rviz_config").strip()
        mode_stem = Path(paths[0]).stem
        if mode_stem.startswith("ugv_"):
            if mode_stem.endswith("_lio"):
                rviz_name = "ugv_lio.rviz"
            elif mode_stem.endswith("_livo"):
                rviz_name = "ugv_livo.rviz"
            else:
                rviz_name = "mapping_ugv.rviz"
        elif mode_stem.startswith("cohemo_"):
            if mode_stem.endswith("_lio"):
                rviz_name = "cohemo_lio.rviz"
            else:
                rviz_name = "cohemo_livo.rviz"
        else:
            rviz_name = "mapping_ugv.rviz"
        rviz = Path(requested_rviz) if requested_rviz else Path(share) / "rviz_cfg" / rviz_name
        if not rviz.is_file():
            raise RuntimeError(f"RViz configuration missing: {rviz}")
        actions.append(Node(package="rviz2", executable="rviz2", name="mapping_ugv_rviz",
                            output="screen", arguments=["-d", str(rviz)],
                            parameters=[{"use_sim_time": sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("fast_livo")
    defaults = {
        "mode": "ugv_livo.yaml", "camera_params_file": "camera_realsense.yaml",
        "input_mode": "raw", "image_topic": "", "use_sim_time": "false",
        "use_rviz": "true",
        "rviz_config": "",  # Empty: select cohemo_lio.rviz or cohemo_livo.rviz by mode.
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
