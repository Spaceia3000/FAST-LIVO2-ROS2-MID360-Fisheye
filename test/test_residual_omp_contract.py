"""Structural contracts for deterministic lock-free residual collection."""

from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / "src" / "voxel_map.cpp").read_text()


def function_body(signature: str) -> str:
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


def test_residual_collection_uses_independent_flags_and_ordered_compaction():
    body = function_body("void VoxelMapManager::BuildResidualListOMP(")

    assert "std::vector<std::uint8_t> useful_ptpl(pv_list.size(), 0);" in body
    assert "std::vector<bool>" not in body
    assert "mutex" not in body
    assert "lock()" not in body

    parallel = body.index("#pragma omp parallel for")
    flag_write = body.index("useful_ptpl[i] = 1;")
    residual_write = body.index("all_ptpl_list[i] = single_ptpl;")
    ordered_loop = body.index("for (size_t i = 0; i < useful_ptpl.size(); i++)")
    compaction = "if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }"
    assert parallel < flag_write < ordered_loop
    assert parallel < residual_write < ordered_loop
    assert ordered_loop < body.index(compaction)
