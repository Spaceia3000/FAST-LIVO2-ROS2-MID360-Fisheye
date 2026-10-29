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


def generate(stem: str, template: str, livo: bool, profile: str) -> str:
    dense = profile == "dense"
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
    args = parser.parse_args()
    for stem, template, livo in SOURCES:
        for profile in PROFILES:
            target = REPO / "rviz_cfg" / f"{stem}_{profile}.rviz"
            desired = generate(stem, template, livo, profile)
            if args.check:
                if not target.exists() or target.read_text() != desired:
                    raise SystemExit(f"FAIL: {target} differs from generated profile")
                print(f"PASS {target.name}")
            else:
                if target.exists() and target.read_text() != desired:
                    raise SystemExit(f"Refusing to overwrite edited RViz: {target}")
                target.write_text(desired)
                print(f"CREATED {target.name}")


if __name__ == "__main__":
    main()
