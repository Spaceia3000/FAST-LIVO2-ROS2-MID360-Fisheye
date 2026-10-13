"""Static ownership contracts for VIO raw-pointer lifetime boundaries."""

from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
VIO_SOURCE = (REPO / "src/vio.cpp").read_text(encoding="utf-8")
FEATURE_HEADER = (REPO / "include/feature.h").read_text(encoding="utf-8")


def braced_block(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def test_patch_is_allocated_only_when_feature_is_added():
    function = braced_block(VIO_SOURCE, "void VIOManager::updateVisualMapPoints(cv::Mat img)")
    add_branch = braced_block(function, "if (add_flag)")

    before_add_branch = function[: function.index("if (add_flag)")]
    assert "new float[patch_size_total]" not in before_add_branch
    assert "getImagePatch(img, pc, patch_temp, 0)" not in before_add_branch

    allocation = add_branch.index("new float[patch_size_total]")
    extraction = add_branch.index("getImagePatch(img, pc, patch_temp, 0)")
    ownership_transfer = add_branch.index("new Feature(pt, patch_temp")
    assert allocation < extraction < ownership_transfer
    assert "delete[] patch_;" in FEATURE_HEADER


def test_disabled_normal_warp_cache_deletes_owned_entries_before_clear():
    function = braced_block(
        VIO_SOURCE,
        "void VIOManager::retrieveFromVisualSparseMap(cv::Mat img",
    )
    disabled_normal_branch = braced_block(function, "if (!normal_en)")

    deletion = disabled_normal_branch.index("delete pair.second")
    clearing = disabled_normal_branch.index("warp_map.clear()")
    assert "for (auto& pair : warp_map)" in disabled_normal_branch
    assert deletion < clearing
