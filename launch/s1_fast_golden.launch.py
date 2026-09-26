#!/usr/bin/env python3
"""Standalone S1 FAST modes, native image input, and optional golden RViz."""
import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = {
    "lo": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": False},
    "lio": {"common.img_en": 0, "common.lidar_en": 1, "imu.imu_en": True},
    "livo": {"common.img_en": 1, "common.lidar_en": 1, "imu.imu_en": True},
}


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
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
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


def validate_topic(topic):
    if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", topic):
        raise RuntimeError(f"image_topic must be an absolute ROS topic: {topic!r}")


def resolve_contract(share, mode="livo", params_file="", camera_params_file="",
                     input_mode="raw", image_topic="", use_sim_time=True):
    if mode not in MODES:
        raise RuntimeError(f"mode must be exactly lo, lio or livo: {mode!r}")
    if input_mode not in {"raw", "compressed_direct"}:
        raise RuntimeError(f"invalid input_mode: {input_mode!r}")
    if input_mode == "compressed_direct" and not image_topic:
        raise RuntimeError("compressed_direct requires explicit image_topic")
    paths = [str(Path(params_file or Path(share) / "config" / f"ugv_v1_{mode}.yaml").resolve())]
    if camera_params_file:
        paths.append(str(Path(camera_params_file).resolve()))
    effective = {}
    for path in paths:
        effective.update(load_parameters(path))
    topic = image_topic or effective.get("common.img_topic", "/left_camera/image")
    validate_topic(topic)
    overrides = {
        "use_sim_time": use_sim_time,
        "common.enable_image_processing": input_mode == "compressed_direct",
    }
    if image_topic:
        overrides["common.img_topic"] = image_topic
    overrides.update(MODES[mode])
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
        share, arg("mode"), arg("params_file"), arg("camera_params_file"),
        arg("input_mode"), arg("image_topic"), sim_time,
    )
    livox = validate_livox_runtime()
    actions = [LogInfo(msg=(
        f"S1 FAST: profiles={paths}, mode={arg('mode')}, input_mode={arg('input_mode')}, "
        f"image_topic={topic}, use_sim_time={sim_time} (launch argument; "
        f"{'/clock' if sim_time else 'system clock'}), livox_typesupport={livox}"
    )), Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
             output="screen", parameters=[*paths, overrides])]
    if use_rviz:
        rviz = Path(arg("rviz_config"))
        if not rviz.is_file():
            raise RuntimeError(f"RViz configuration missing: {rviz}")
        actions.append(Node(package="rviz2", executable="rviz2", name="s1_fast_golden_rviz",
                            output="screen", arguments=["-d", str(rviz)],
                            parameters=[{"use_sim_time": sim_time}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("fast_livo")
    defaults = {
        "mode": "livo", "params_file": "", "camera_params_file": "",
        "input_mode": "raw", "image_topic": "", "use_sim_time": "true",
        "use_rviz": "true",
        "rviz_config": str(Path(share) / "rviz_cfg" / "fast_livo2_golden.rviz"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
