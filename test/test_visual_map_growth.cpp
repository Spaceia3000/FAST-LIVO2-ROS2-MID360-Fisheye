#include "visual_map_growth.h"

#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

TEST(VisualMapGrowth, ZeroCapDisablesLimit)
{
  EXPECT_TRUE(visual_map_growth::bucketAdmits(0U, 0));
  EXPECT_TRUE(visual_map_growth::bucketAdmits(std::numeric_limits<std::size_t>::max(), 0));
}

TEST(VisualMapGrowth, PositiveCapRejectsFullBucket)
{
  EXPECT_TRUE(visual_map_growth::bucketAdmits(63U, 64));
  EXPECT_FALSE(visual_map_growth::bucketAdmits(64U, 64));
  EXPECT_FALSE(visual_map_growth::bucketAdmits(65U, 64));
  EXPECT_TRUE(visual_map_growth::bucketAdmits(
      static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max() - 1),
      std::numeric_limits<std::int64_t>::max()));
  EXPECT_FALSE(visual_map_growth::bucketAdmits(
      static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()),
      std::numeric_limits<std::int64_t>::max()));
}

TEST(VisualMapGrowth, RejectsNegativeThreshold)
{
  EXPECT_FALSE(visual_map_growth::bucketAdmits(0U, -1));
  EXPECT_FALSE(visual_map_growth::bucketAdmits(
      std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::int64_t>::min()));
}

TEST(VisualMapGrowth, PreservesVisualMapBoundaryRounding)
{
  EXPECT_EQ(visual_map_growth::voxelCoordinate(0.0), 0);
  EXPECT_EQ(visual_map_growth::voxelCoordinate(0.499), 0);
  EXPECT_EQ(visual_map_growth::voxelCoordinate(0.5), 1);
  EXPECT_EQ(visual_map_growth::voxelCoordinate(-0.001), -1);
  EXPECT_EQ(visual_map_growth::voxelCoordinate(-0.499), -1);
  EXPECT_EQ(visual_map_growth::voxelCoordinate(-0.5), -2);
}

TEST(VisualMapGrowth, UsesSameGeometryForEveryAxis)
{
  const auto bucket = visual_map_growth::voxelBucket(-0.1, -0.5, -0.9);
  ASSERT_TRUE(bucket.has_value());
  EXPECT_EQ(*bucket, (visual_map_growth::VoxelBucket{-1, -2, -2}));

  for (const double coordinate : {-1.01, -1.0, -0.99, -0.51, -0.5, -0.49, -0.01})
  {
    float legacy_location = static_cast<float>(coordinate / 0.5);
    if (legacy_location < 0.0F) legacy_location -= 1.0F;
    ASSERT_TRUE(visual_map_growth::voxelCoordinate(coordinate).has_value());
    EXPECT_EQ(
        *visual_map_growth::voxelCoordinate(coordinate),
        static_cast<std::int64_t>(legacy_location));
  }
}

TEST(VisualMapGrowth, RejectsNonFiniteAndOverflowCoordinates)
{
  EXPECT_FALSE(visual_map_growth::voxelCoordinate(
      std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(visual_map_growth::voxelCoordinate(
      -std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(visual_map_growth::voxelCoordinate(
      std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(visual_map_growth::voxelCoordinate(
      std::numeric_limits<double>::max()).has_value());
  EXPECT_FALSE(visual_map_growth::voxelCoordinate(
      -std::numeric_limits<double>::max()).has_value());
}
