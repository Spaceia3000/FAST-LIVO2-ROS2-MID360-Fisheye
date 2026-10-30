#!/usr/bin/env python3
"""Generate reversible UGV light/dense RViz variants from versioned templates.

Pinned templates are independent of operator-edited RViz files.
No sensor or SLAM parameters are changed. --check verifies byte-for-byte output.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SOURCES = (
    ("ugv_lio", "ugv_lio.rviz", False),
    ("ugv_livo", "ugv_livo.rviz", True),
    ("ugv_realsense_livo", "ugv_livo.rviz", True),
)
PROFILES = ("light", "dense")


def edit_display(text: str, name: str, **changes: object) -> str:
    """Patch one named display without reformatting or rewriting the RViz document."""
    lines = text.splitlines(keepends=True)
    locations = [i for i, line in enumerate(lines) if line.strip() == f"Name: {name}"]
    if len(locations) != 1:
        raise ValueError(f"RViz display {name!r} not unique: {len(locations)}")
    i = locations[0]
    indent = len(lines[i]) - len(lines[i].lstrip(" "))
    marker_indent = indent - 2
    starts = [j for j in range(i) if lines[j].startswith(" " * marker_indent + "- ")]
    if not starts:
        raise ValueError(f"Cannot find display start for {name}")
    start = starts[-1]
    end = next((j for j in range(i + 1, len(lines))
                if lines[j].strip()
                and len(lines[j]) - len(lines[j].lstrip(" ")) <= marker_indent), len(lines))
    display = "".join(lines[start:end])
    for field, value in changes.items():
        if value is None:
            continue
        if field == "Alpha":
            expression = r"(?m)^" + " " * marker_indent + r"- Alpha: .*?$"
            replacement = " " * marker_indent + f"- Alpha: {value}"
        else:
            expression = r"(?m)^" + " " * indent + re.escape(field) + r": .*?$"
            printable = str(value).lower() if isinstance(value, bool) else str(value)
            replacement = " " * indent + f"{field}: {printable}"
        if len(re.findall(expression, display)) != 1:
            raise ValueError(f"Expected one {field} in {name}")
        display = re.sub(expression, lambda _: replacement, display, count=1)
    lines[start:end] = [display]
    return "".join(lines)




def extract_loop_closure_group() -> str:
    """Retain the SLC group from the pinned UGV template as literal RViz YAML."""
    source = (REPO / "scripts" / "rviz_templates" / "ugv_livo.rviz").read_text()
    marker = "    - Class: rviz_common/Group\n      Displays:\n"
    start = source.index(marker)
    end = source.index("    - Class: rviz_common/Group\n", start + len(marker))
    group = source[start:end]
    if group.count("      Name: GLOBAL - Simple Loop Closure (GTSAM)\n") != 1:
        raise ValueError("Invalid pinned SLC group")
    return group


def add_visible_original_path_to_group(group: str) -> str:
    """Expose FAST-LIVO2 /path in cyan inside SLC while preserving upstream Path.

    Vendor RViz sets the original Path display Alpha=0 (fully transparent).
    Clone that display into the added SLC group with only Name and Alpha changed.
    """
    upstream = (REPO / "scripts" / "rviz_templates" / "fast_livo2_upstream.rviz").read_text()
    matches = list(re.finditer(
        r"(?ms)^    - Alpha: 0\n      Buffer Length: 2\n"
        r"      Class: rviz_default_plugins/Path\n.*?(?=^    - |\Z)", upstream))
    if len(matches) != 1:
        raise ValueError(f"Cannot identify unique original FAST-LIVO2 Path: {len(matches)}")
    path = matches[0].group(0)
    if path.count("      Name: Path\n") != 1 or path.count("        Value: /path\n") != 1:
        raise ValueError("Upstream /path display changed")
    path = path.replace("    - Alpha: 0\n", "    - Alpha: 1\n", 1)
    path = path.replace("      Name: Path\n", "      Name: Trayectoria FAST-LIVO2 (cian)\n", 1)
    # SLC Displays are nested one level below the original root displays.
    path = "".join("    " + line if line.strip() else line
                   for line in path.splitlines(keepends=True))
    marker = "      Enabled: true\n      Name: GLOBAL - Simple Loop Closure (GTSAM)\n"
    if group.count(marker) != 1:
        raise ValueError("Cannot place visible path inside SLC group")
    return group.replace(marker, path + marker, 1)


def generate_dense_original_with_closure(is_livo: bool) -> str:
    """Verbatim upstream FAST-LIVO2 RViz plus one SLC group, with visible cyan path."""
    source = (REPO / "scripts" / "rviz_templates" / "fast_livo2_upstream.rviz").read_text()
    marker = "    - Class: rviz_common/Group\n      Displays:\n"
    if source.count(marker) != 2:
        # The original contains exactly two groups: mapping and Odometry.
        raise ValueError("Unexpected upstream RViz structure")
    group = extract_loop_closure_group()
    group = edit_display(group, "Mapa global RGB corregido (GTSAM)", Enabled=is_livo)
    group = edit_display(group, "Mapa global XYZI PGO (opcional)", Enabled=not is_livo)
    group = add_visible_original_path_to_group(group)
    # Insert before the original mapping group, retaining EVERY original byte
    # (including 10000s surround, activated displays, colors and window settings).
    return source.replace(marker, group + marker, 1)

def generate(stem: str, template: str, livo: bool, profile: str) -> str:
    if profile == "dense":
        return generate_dense_original_with_closure(livo)
    if profile != "light":
        raise ValueError(f"Unsupported RViz profile: {profile}")
    dense = False
    doc = (REPO / "scripts" / "rviz_templates" / template).read_text()
    doc = edit_display(
        doc, "UGV metric cloud", Enabled=True,
        **{"Decay Time": 3 if dense else 0,
           "Size (Pixels)": 2,
           "Alpha": 0.28 if dense and livo else 1}
    )
    doc = edit_display(doc, "Legacy display diagnostic", Enabled=False)
    # RGB accumulation is visual, time-bounded and only enabled for LIVO dense.
    doc = edit_display(
        doc, "Legacy accumulated diagnostic", Enabled=dense and livo,
        **{"Decay Time": 3 if dense and livo else 0, "Size (Pixels)": 2}
    )
    doc = edit_display(doc, "Mapa global RGB corregido (GTSAM)", Enabled=livo,
                       **{"Size (Pixels)": 2})
    doc = edit_display(doc, "Mapa global XYZI PGO (opcional)",
                       Enabled=dense and not livo, **{"Size (Pixels)": 2})
    doc = edit_display(doc, "Image", Enabled=livo)
    doc = edit_display(doc, "TF", Enabled=False)
    return doc


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="Check output matches generated variants; do not write.")
    parser.add_argument("--refresh-dense", action="store_true",
                        help="Explicitly replace only the three tracked dense profiles.")
    args = parser.parse_args()
    if args.check and args.refresh_dense:
        parser.error("--check and --refresh-dense are exclusive")
    for stem, template, livo in SOURCES:
        for profile in PROFILES:
            if args.refresh_dense and profile != "dense":
                continue
            target = REPO / "rviz_cfg" / f"{stem}_{profile}.rviz"
            desired = generate(stem, template, livo, profile)
            if args.check:
                if not target.exists() or target.read_text() != desired:
                    raise SystemExit(f"FAIL: {target} differs from generated profile")
                print(f"PASS {target.name}")
            else:
                if target.exists() and target.read_text() != desired and not args.refresh_dense:
                    raise SystemExit(f"Refusing to overwrite edited RViz: {target}")
                if args.refresh_dense and not target.is_file():
                    raise SystemExit(f"Refusing to create missing dense target: {target}")
                target.write_text(desired)
                print(f"CREATED {target.name}")


if __name__ == "__main__":
    main()
