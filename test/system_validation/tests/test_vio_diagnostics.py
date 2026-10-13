import re
import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
VIO_DIAGNOSTICS_PATTERN = re.compile(
    r"^\[vio_map\] visual_voxels=(?P<visual_voxels>\d+) "
    r"visual_points=(?P<visual_points>\d+) "
    r"geometric_voxels=(?P<geometric_voxels>\d+) "
    r"retrieve_last_ms=(?P<retrieve_last_ms>\d+\.\d{3}) "
    r"candidates_direct=(?P<candidates_direct>\d+) "
    r"candidates_raycast=(?P<candidates_raycast>\d+)$"
)


def test_vio_diagnostics_exact_output_and_parsing(tmp_path):
    source = tmp_path / "vio_diagnostics_probe.cpp"
    source.write_text(r'''
#include "vio_diagnostics.h"
#include <iostream>

int main()
{
  VioDiagnosticsSnapshot snapshot;
  snapshot.visual_voxels = 6202;
  snapshot.visual_points = 611416;
  snapshot.geometric_voxels = 6201;
  snapshot.retrieve_last_ms = 37.8126;
  snapshot.candidates_direct = 123456;
  snapshot.candidates_raycast = 789;
  std::cout << formatVioDiagnostics(snapshot);
}
''')
    binary = tmp_path / "vio_diagnostics_probe"
    compiled = subprocess.run(
        ["c++", "-std=c++17", f"-I{REPO / 'include'}", str(source), "-o", str(binary)],
        capture_output=True,
        text=True,
    )
    assert compiled.returncode == 0, compiled.stderr

    output = subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout
    assert output == (
        "[vio_map] visual_voxels=6202 visual_points=611416 geometric_voxels=6201 "
        "retrieve_last_ms=37.813 candidates_direct=123456 candidates_raycast=789"
    )
    match = VIO_DIAGNOSTICS_PATTERN.fullmatch(output)
    assert match is not None
    assert match.groupdict() == {
        "visual_voxels": "6202",
        "visual_points": "611416",
        "geometric_voxels": "6201",
        "retrieve_last_ms": "37.813",
        "candidates_direct": "123456",
        "candidates_raycast": "789",
    }
