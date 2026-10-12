#include "runtime_output.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <unistd.h>

TEST(RuntimeOutput, RejectsRelativeDirectory)
{
  EXPECT_THROW(fast_livo_runtime::set_output_directory("relative/run"), std::runtime_error);
}

TEST(RuntimeOutput, KeepsEveryProductBelowDesignatedDirectory)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("fast_livo_runtime_output_test_" + std::to_string(getpid()));
  fast_livo_runtime::set_output_directory(root.string());

  EXPECT_EQ(fast_livo_runtime::output_directory(), root);
  EXPECT_EQ(
    fast_livo_runtime::path("Log/pcd/cloud.pcd"),
    root / "Log/pcd/cloud.pcd");
  EXPECT_TRUE(std::filesystem::is_directory(root));
  EXPECT_THROW(fast_livo_runtime::path("/outside"), std::runtime_error);
}
