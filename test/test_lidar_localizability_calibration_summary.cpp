#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lidar_localizability_calibration_summary.h"

namespace
{

constexpr double kTolerance = 1e-12;

LidarLocalizabilityBasisSnapshot
makeBasis(
    const std::size_t sample_count)
{
  LidarLocalizabilityBasisSnapshot basis;

  basis.rotation_eigenvectors =
      Eigen::Matrix3d::Identity();

  basis.translation_eigenvectors =
      Eigen::Matrix3d::Identity();

  basis.input_samples =
      sample_count;

  basis.used_samples =
      sample_count;

  basis.available =
      sample_count > 0U;

  return basis;
}

LidarDirectionalLocalizabilitySnapshot
makeDirectional()
{
  LidarDirectionalLocalizabilitySnapshot directional;

  directional.input_samples = 2U;
  directional.used_samples = 2U;
  directional.available = true;

  directional.contributions.resize(2U);

  directional.contributions[0].rotation <<
      0.0, 0.25, 1.0;

  directional.contributions[0].translation <<
      0.10, 0.50, 0.90;

  directional.contributions[1].rotation <<
      0.24, 0.74, 0.76;

  directional.contributions[1].translation <<
      0.0, 1.0, 0.49;

  return directional;
}

}  // namespace

TEST(
    LidarLocalizabilityCalibrationSummary,
    ZeroBinsIsUnavailable)
{
  const auto result =
      buildLidarLocalizabilityCalibrationSummary(
          makeBasis(2U),
          makeDirectional(),
          0U);

  EXPECT_FALSE(result.available);
}

TEST(
    LidarLocalizabilityCalibrationSummary,
    BuildsThresholdIndependentHistograms)
{
  const auto result =
      buildLidarLocalizabilityCalibrationSummary(
          makeBasis(2U),
          makeDirectional(),
          4U);

  ASSERT_TRUE(result.available);

  EXPECT_EQ(result.histogram_bins, 4U);
  EXPECT_EQ(result.input_contributions, 2U);
  EXPECT_EQ(result.used_contributions, 2U);

  // Rotation axis 0:
  // 0.00 and 0.24 both belong to [0.00, 0.25).
  EXPECT_EQ(
      result.rotation[0].count[0],
      2U);

  EXPECT_NEAR(
      result.rotation[0].sum[0],
      0.24,
      kTolerance);

  // Rotation axis 1:
  // 0.25 -> bin 1
  // 0.74 -> bin 2
  EXPECT_EQ(
      result.rotation[1].count[1],
      1U);

  EXPECT_EQ(
      result.rotation[1].count[2],
      1U);

  // c = 1 belongs to the final bin.
  EXPECT_EQ(
      result.rotation[2].count[3],
      2U);

  // Translation axis 1:
  // 0.50 -> bin 2
  // 1.00 -> bin 3
  EXPECT_EQ(
      result.translation[1].count[2],
      1U);

  EXPECT_EQ(
      result.translation[1].count[3],
      1U);
}

TEST(
    LidarLocalizabilityCalibrationSummary,
    PreservesCountsAndContributionSums)
{
  const auto directional =
      makeDirectional();

  const auto result =
      buildLidarLocalizabilityCalibrationSummary(
          makeBasis(2U),
          directional,
          4U);

  ASSERT_TRUE(result.available);

  for (std::size_t axis = 0U; axis < 3U; ++axis)
  {
    std::uint64_t rotation_count = 0U;
    std::uint64_t translation_count = 0U;

    double rotation_sum = 0.0;
    double translation_sum = 0.0;

    for (std::size_t bin = 0U; bin < 4U; ++bin)
    {
      rotation_count +=
          result.rotation[axis].count[bin];

      translation_count +=
          result.translation[axis].count[bin];

      rotation_sum +=
          result.rotation[axis].sum[bin];

      translation_sum +=
          result.translation[axis].sum[bin];
    }

    double expected_rotation_sum = 0.0;
    double expected_translation_sum = 0.0;

    for (const auto &contribution :
         directional.contributions)
    {
      expected_rotation_sum +=
          contribution.rotation(
              static_cast<Eigen::Index>(axis));

      expected_translation_sum +=
          contribution.translation(
              static_cast<Eigen::Index>(axis));
    }

    EXPECT_EQ(
        rotation_count,
        directional.used_samples);

    EXPECT_EQ(
        translation_count,
        directional.used_samples);

    EXPECT_NEAR(
        rotation_sum,
        expected_rotation_sum,
        kTolerance);

    EXPECT_NEAR(
        translation_sum,
        expected_translation_sum,
        kTolerance);
  }
}

TEST(
    LidarLocalizabilityCalibrationSummary,
    RejectsInconsistentBasis)
{
  const auto result =
      buildLidarLocalizabilityCalibrationSummary(
          makeBasis(1U),
          makeDirectional(),
          4U);

  EXPECT_FALSE(result.available);
}

TEST(
    LidarLocalizabilityCalibrationSummary,
    RejectsNonFiniteContribution)
{
  auto directional =
      makeDirectional();

  directional.contributions[0].rotation(0) =
      std::numeric_limits<double>::quiet_NaN();

  const auto result =
      buildLidarLocalizabilityCalibrationSummary(
          makeBasis(2U),
          directional,
          4U);

  EXPECT_FALSE(result.available);
}

