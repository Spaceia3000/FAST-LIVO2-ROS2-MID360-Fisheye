"""ROS 2 UGV RViz visual density profiles: format, selector, and reproducibility."""
import importlib.util
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts/generate_ugv_rviz_profiles.py"
LAUNCH = ROOT / "launch/mapping_ugv.launch.py"
STEMS = ("ugv_lio", "ugv_livo", "ugv_realsense_livo")


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def named_display(document, group, name):
    displays = document["Visualization Manager"]["Displays"]
    if group is not None:
        displays = next(d for d in displays if d.get("Name") == group)["Displays"]
    found = [d for d in displays if d.get("Name") == name]
    assert len(found) == 1, (group, name)
    return found[0]


@pytest.mark.parametrize("stem", STEMS)
@pytest.mark.parametrize("profile", ("light", "dense"))
def test_rviz_structure_and_intended_viz_only_changes(stem, profile):
    document = yaml.safe_load((ROOT / "rviz_cfg" / f"{stem}_{profile}.rviz").read_text())
    is_livo = stem != "ugv_lio"
    dense = profile == "dense"
    corrected = named_display(document, "GLOBAL - Simple Loop Closure (GTSAM)",
                              "Mapa global RGB corregido (GTSAM)")
    xyzi = named_display(document, "GLOBAL - Simple Loop Closure (GTSAM)",
                         "Mapa global XYZI PGO (opcional)")
    assert corrected["Enabled"] is is_livo
    assert corrected["Topic"]["Value"] == "/pgo_rgb_map_cloud"
    assert xyzi["Enabled"] is (dense and not is_livo)
    assert xyzi["Topic"]["Value"] == "/pgo_map_cloud"
    assert document["Visualization Manager"]["Global Options"]["Fixed Frame"] == "camera_init"
    assert named_display(document, "GLOBAL - Simple Loop Closure (GTSAM)",
                         "Trayectoria GTSAM optimizada (magenta)")["Enabled"] is True
    assert named_display(document, "GLOBAL - Simple Loop Closure (GTSAM)",
                         "Cierres de lazo aceptados (rojo)")["Enabled"] is True
    if not dense:
        metric = named_display(document, "mapping", "UGV metric cloud")
        rgb = named_display(document, "mapping", "Legacy accumulated diagnostic")
        old = named_display(document, "mapping", "Legacy display diagnostic")
        assert metric["Enabled"] is True
        assert metric["Topic"]["Value"] == "/cloud_registered_metric"
        assert metric["Decay Time"] == 0
        assert metric["Size (Pixels)"] == 2
        assert old["Enabled"] is False
        assert rgb["Enabled"] is False
        assert rgb["Topic"]["Value"] == "/cloud_registered"
        assert rgb["Decay Time"] == 0
        assert named_display(document, None, "TF")["Enabled"] is False
        assert named_display(document, None, "Image")["Enabled"] is is_livo


@pytest.mark.parametrize("stem", STEMS)
def test_dense_preserves_complete_unmodified_upstream_original(stem):
    """Only extra SLC group is allowed. Every original FAST-LIVO2 setting stays."""
    import hashlib
    source = (ROOT / "scripts" / "rviz_templates" / "fast_livo2_upstream.rviz").read_text()
    assert hashlib.sha256(source.encode()).hexdigest() == (
        "7ab648e254601f057fa173fc627b385800711fbd49afadc2f56fa8c09eee2fdc"
    )
    document = (ROOT / "rviz_cfg" / f"{stem}_dense.rviz").read_text()
    original = yaml.safe_load(source)
    dense = yaml.safe_load(document)
    original_displays = original["Visualization Manager"]["Displays"]
    dense_displays = dense["Visualization Manager"]["Displays"]
    inserted = [d for d in dense_displays
                if d.get("Name") == "GLOBAL - Simple Loop Closure (GTSAM)"]
    assert len(inserted) == 1
    assert [d for d in dense_displays
            if d.get("Name") != "GLOBAL - Simple Loop Closure (GTSAM)"] == original_displays
    assert dense["Panels"] == original["Panels"]
    assert dense["Window Geometry"] == original["Window Geometry"]
    orig_manager = original["Visualization Manager"].copy()
    dense_manager = dense["Visualization Manager"].copy()
    orig_manager.pop("Displays")
    dense_manager.pop("Displays")
    assert dense_manager == orig_manager
    mapping = named_display(dense, "mapping", "surround")
    assert mapping["Enabled"] is True
    assert mapping["Decay Time"] == 10000
    assert mapping["Topic"]["Value"] == "/cloud_registered"
    current = named_display(dense, "mapping", "currPoints")
    assert current["Enabled"] is True
    assert current["Decay Time"] == 0
    assert current["Topic"]["Value"] == "/cloud_registered"
    # Beyond YAML semantic parity, verify that inserting the one SLC group is
    # the exact only textual change to the vendor's original RViz file.
    generator = load_module("ugv_rviz_generator_pinned", GENERATOR)
    group = generator.extract_loop_closure_group()
    group = generator.edit_display(
        group, "Mapa global RGB corregido (GTSAM)", Enabled=stem != "ugv_lio")
    group = generator.edit_display(
        group, "Mapa global XYZI PGO (opcional)", Enabled=stem == "ugv_lio")
    assert document.replace(group, "", 1) == source


def test_generated_profiles_match_pinned_sources_byte_for_byte():
    generator = load_module("ugv_rviz_generator", GENERATOR)
    for stem, template, livo in generator.SOURCES:
        for profile in generator.PROFILES:
            output = ROOT / "rviz_cfg" / f"{stem}_{profile}.rviz"
            assert output.read_text() == generator.generate(stem, template, livo, profile)


def test_launch_selects_exact_yaml_stem_and_rejects_unknown_profiles():
    launch = load_module("ugv_rviz_launcher", LAUNCH)
    for stem in STEMS:
        for profile in ("light", "dense"):
            filename = launch.select_rviz_filename(stem, profile)
            assert filename == f"{stem}_{profile}.rviz"
            assert (ROOT / "rviz_cfg" / filename).is_file()
    assert launch.select_rviz_filename("ugv_lo", "light") == "mapping_ugv.rviz"
    assert launch.select_rviz_filename("cohemo_lio", "light") == "cohemo_lio.rviz"
    assert launch.select_rviz_filename("cohemo_livo", "dense") == "cohemo_livo.rviz"
    for invalid in ("", "heavy", "full", "../dense", "dense.rviz"):
        with pytest.raises(RuntimeError, match="rviz_profile"):
            launch.select_rviz_filename("ugv_realsense_livo", invalid)


def test_launch_keeps_explicit_rviz_config_override_and_default_light():
    source = LAUNCH.read_text()
    assert '"rviz_profile": "light"' in source
    assert 'rviz = Path(requested_rviz) if requested_rviz else' in source
    assert '"use_rviz": "true"' in source


@pytest.mark.parametrize("stem", STEMS)
@pytest.mark.parametrize("profile", ("light", "dense"))
def test_actual_launch_setup_selects_profile_without_starting_nodes(monkeypatch, stem, profile):
    """Exercise launch_setup with stubbed process spawning (no ROS nodes started)."""
    import launch
    module = load_module("ugv_launch_contract_sut", LAUNCH)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(ROOT))
    monkeypatch.setattr(module, "validate_livox_runtime", lambda: "/dummy/livox_typesupport.so")
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = launch.LaunchContext()
    context.launch_configurations.update({
        "mode": f"{stem}.yaml",
        "camera_params_file": "camera_realsense.yaml",
        "input_mode": "raw",
        "image_topic": "",
        "use_sim_time": "false",
        "use_rviz": "true",
        "rviz_config": "",
        "rviz_profile": profile,
    })
    actions = module.launch_setup(context)
    rviz_action = actions[-1]
    assert rviz_action["package"] == "rviz2"
    assert rviz_action["arguments"] == [
        "-d", str(ROOT / "rviz_cfg" / f"{stem}_{profile}.rviz")
    ]
    # User-specified RViz path continues to override the built-in profile.
    explicit = str(ROOT / "rviz_cfg" / "ugv_livo.rviz")
    context.launch_configurations["rviz_config"] = explicit
    custom_actions = module.launch_setup(context)
    assert custom_actions[-1]["arguments"] == ["-d", explicit]
