"""Static contract for per-mode measurement-history retention.

This test checks source structure rather than exercising ROS callbacks at runtime;
constructing LIVMapper and its private synchronized buffers would require a broad,
integration-level harness. It protects the bounded LO/LIO history and the distinct
LIVO LIO-to-VIO accumulation contract directly at their mutation sites.
"""

from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / "src/LIVMapper.cpp").read_text(
    encoding="utf-8"
)


def braced_block(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def test_lo_and_lio_replace_history_while_livo_keeps_its_transition_contract():
    sync = braced_block(SOURCE, "bool LIVMapper::sync_packages(")
    only_lio = sync[sync.index("case ONLY_LIO:") : sync.index("case LIVO:")]
    livo = sync[sync.index("case LIVO:") : sync.index("case ONLY_LO:")]
    only_lo = sync[sync.index("case ONLY_LO:") : sync.index("default:", sync.index("case ONLY_LO:"))]

    replace_current = "meas.measures.clear();\n    meas.measures.push_back(m);"
    assert only_lio.count(replace_current) == 1
    assert only_lo.count(replace_current) == 1

    livo_lio_update = livo[: livo.index("case LIO:")]
    livo_vio_update = livo[livo.index("case LIO:") :]
    assert "meas.measures.clear()" not in livo_lio_update
    assert "meas.measures.push_back(m)" in livo_lio_update
    assert livo_vio_update.index("meas.measures.clear()") < livo_vio_update.index(
        "meas.measures.push_back(m)"
    )
