#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lidar_directional_localizability_aggregation.h"

namespace
{

constexpr double kTolerance = 1e-12;

LidarDirectionalAggregationPolicy makePolicy()
{
  LidarDirectionalAggregationPolicy policy;
  policy.minimal_contribution_threshold = 0.25;
  policy.strong_contribution_threshold = 0.70;
  return policy;
}

LidarDirectionalContributionVector makeContributions()
{
  LidarDirectionalContributionVector contributions(2);

  contributions[0].rotation <<
      0.10, 0.30, 0.90;

  contributions[0].translation <<
      0.20, 0.80, 0.50;

  contributions[1].rotation <<
      0.40, 0.70, 0.20;

  contributions[1].translation <<
      0.90, 0.10, 0.70;

  return contributions;
}

}  // namespace

TEST(
    LidarDirectionalAggregation,
    InvalidPolicyIsUnavailable)
{
  const auto result =
      aggregateLidarDirectionalLocalizability(
          makeContributions(),
          LidarDirectionalAggregationPolicy{});

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.input_contributions, 2U);
  EXPECT_EQ(result.used_contributions, 0U);
}

TEST(
    LidarDirectionalAggregation,
    FiltersAndAggregatesEachDirection)
{
  const auto result =
      aggregateLidarDirectionalLocalizability(
          makeContributions(),
          makePolicy());

  ASSERT_TRUE(result.available);
  ASSERT_EQ(result.used_contributions, 2U);

  const Eigen::Vector3d expected_rotation_combined(
      0.40, 1.00, 0.90);

  const Eigen::Vector3d expected_rotation_strong(
      0.00, 0.70, 0.90);

  const Eigen::Vector3d expected_translation_combined(
      0.90, 0.80, 1.20);

  const Eigen::Vector3d expected_translation_strong(
      0.90, 0.80, 0.70);

  EXPECT_LT(
      (
          result.rotation_combined -
          expected_rotation_combined
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result.rotation_strong -
          expected_rotation_strong
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result.translation_combined -
          expected_translation_combined
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result.translation_strong -
          expected_translation_strong
      ).norm(),
      kTolerance);

  EXPECT_EQ(result.rotation_contributing_count[0], 1U);
  EXPECT_EQ(result.rotation_contributing_count[1], 2U);
  EXPECT_EQ(result.rotation_contributing_count[2], 1U);

  EXPECT_EQ(result.rotation_strong_count[0], 0U);
  EXPECT_EQ(result.rotation_strong_count[1], 1U);
  EXPECT_EQ(result.rotation_strong_count[2], 1U);

  EXPECT_EQ(result.translation_contributing_count[0], 1U);
  EXPECT_EQ(result.translation_contributing_count[1], 1U);
  EXPECT_EQ(result.translation_contributing_count[2], 2U);

  EXPECT_EQ(result.translation_strong_count[0], 1U);
  EXPECT_EQ(result.translation_strong_count[1], 1U);
  EXPECT_EQ(result.translation_strong_count[2], 1U);
}

TEST(
    LidarDirectionalAggregation,
    ThresholdEdgesAreInclusive)
{
  LidarDirectionalContributionVector contributions(1);

  contributions[0].rotation <<
      0.25, 0.70, 0.0;

  contributions[0].translation <<
      0.25, 0.70, 0.0;

  const auto result =
      aggregateLidarDirectionalLocalizability(
          contributions,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_NEAR(
      result.rotation_combined(0),
      0.25,
      kTolerance);

  EXPECT_NEAR(
      result.rotation_strong(1),
      0.70,
      kTolerance);

  EXPECT_NEAR(
      result.translation_combined(0),
      0.25,
      kTolerance);

  EXPECT_NEAR(
      result.translation_strong(1),
      0.70,
      kTolerance);
}

TEST(
    LidarDirectionalAggregation,
    AggregationIsIndependentOfInputOrder)
{
  auto forward =
      makeContributions();

  auto reverse =
      forward;

  std::reverse(
      reverse.begin(),
      reverse.end());

  const auto result_forward =
      aggregateLidarDirectionalLocalizability(
          forward,
          makePolicy());

  const auto result_reverse =
      aggregateLidarDirectionalLocalizability(
          reverse,
          makePolicy());

  ASSERT_TRUE(result_forward.available);
  ASSERT_TRUE(result_reverse.available);

  EXPECT_LT(
      (
          result_forward.rotation_combined -
          result_reverse.rotation_combined
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result_forward.rotation_strong -
          result_reverse.rotation_strong
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result_forward.translation_combined -
          result_reverse.translation_combined
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          result_forward.translation_strong -
          result_reverse.translation_strong
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalAggregation,
    InvalidContributionsAreRejected)
{
  const double nan =
      std::numeric_limits<double>::quiet_NaN();

  auto contributions =
      makeContributions();

  LidarDirectionalContribution invalid;
  invalid.rotation <<
      nan, 0.5, 0.5;
  invalid.translation <<
      0.5, 0.5, 0.5;

  contributions.push_back(invalid);

  const auto result =
      aggregateLidarDirectionalLocalizability(
          contributions,
          makePolicy());

  ASSERT_TRUE(result.available);
  EXPECT_EQ(result.input_contributions, 3U);
  EXPECT_EQ(result.used_contributions, 2U);
}
