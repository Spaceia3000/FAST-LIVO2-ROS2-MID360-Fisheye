"""G1 profiler remains opt-in and is forwarded only as an override."""
from pathlib import Path

LAUNCH = (Path(__file__).resolve().parents[1] / "launch" / "mapping_ugv.launch.py").read_text()
SOURCE = (Path(__file__).resolve().parents[1] / "src" / "LIVMapper.cpp").read_text()


def function_body(signature):
    start = SOURCE.index(signature)
    opening = SOURCE.index("{", start)
    depth = 0
    for index in range(opening, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[opening + 1:index]
    raise AssertionError(f"unterminated function: {signature}")


def test_g1_performance_is_safe_opt_in():
    assert '"g1_performance": "false"' in LAUNCH
    assert 'g1_performance = parse_bool(arg("g1_performance"))' in LAUNCH
    assert 'overrides["diagnostics.performance.enabled"] = g1_performance' in LAUNCH


def test_rgb_publish_timer_wraps_real_vio_color_publish_only():
    vio = function_body("void LIVMapper::handleVIO()")
    lio = function_body("void LIVMapper::handleLIO()")
    publisher = function_body("void LIVMapper::publish_frame_world(")
    publish_call = "publish_frame_world(pubLaserCloudFullRes, vio_manager);"

    assert vio.index("rgb_start") < vio.index(publish_call) < vio.index("performance_stats[RGB_PUBLISH].add")
    assert "RGB_PUBLISH" not in lio
    assert "getInterpolatedPixel" in publisher
    assert "pcl::toROSMsg(*laserCloudWorldRGB" in publisher
    assert "pubLaserCloudFullRes->publish(laserCloudmsg);" in publisher


def test_sync_records_ready_and_waiting_calls_before_preserving_branch():
    run = function_body("void LIVMapper::run()")

    assert "const bool sync_ready = sync_packages(LidarMeasures);" in run
    record = run.index("performance_stats[sync_ready ? SYNC_TRUE : SYNC_FALSE].add")
    branch = run.index("if (!sync_ready)")
    assert record < branch
