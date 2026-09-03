#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "lidar_localizability_basis.h"

namespace
{

constexpr double kTolerance = 1e-10;

using Sample = LidarLocalizabilitySample;

std::vector<Sample> makeTunnelSamples()
{
  return {
      {
          Eigen::Vector3d(0.0, 1.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0)
      },
      {
          Eigen::Vector3d(0.0, 0.0, 1.0),
          Eigen::Vector3d(0.0, 0.0, 1.0)
      },
      {
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0)
      },
      {
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 0.0, 1.0)
      }
  };
}

}  // namespace

TEST(LidarLocalizabilityBasis, EmptyInputIsUnavailable)
{
  const std::vector<Sample> samples;

  const auto result =
      computeLidarLocalizabilityBasis(samples);

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.input_samples, 0U);
  EXPECT_EQ(result.used_samples, 0U);
}

TEST(LidarLocalizabilityBasis, InvalidSamplesAreRejected)
{
  const double nan =
      std::numeric_limits<double>::quiet_NaN();

  const std::vector<Sample> samples = {
      {
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0)
      },
      {
          Eigen::Vector3d(nan, 0.0, 0.0),
          Eigen::Vector3d(0.0, 1.0, 0.0)
      },
      {
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d::Zero()
      },
      {
          Eigen::Vector3d(1.0, 0.0, 0.0),
          Eigen::Vector3d(0.0, nan, 0.0)
      }
  };

  const auto result =
      computeLidarLocalizabilityBasis(samples);

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.input_samples, 4U);
  EXPECT_EQ(result.used_samples, 1U);
}

TEST(LidarLocalizabilityBasis, TunnelHasWeakLongitudinalTranslation)
{
  const auto result =
      computeLidarLocalizabilityBasis(
          makeTunnelSamples());

  ASSERT_TRUE(result.available);
  ASSERT_EQ(result.used_samples, 4U);

  EXPECT_NEAR(
      result.translation_eigenvalues(0),
      0.0,
      kTolerance);

  const double alignment =
      std::abs(
          result.translation_eigenvectors
              .col(0)
              .dot(Eigen::Vector3d::UnitX()));

  EXPECT_GT(
      alignment,
      1.0 - kTolerance);
}

TEST(LidarLocalizabilityBasis, SyntheticGeometryHasWeakXRotation)
{
  const auto result =
      computeLidarLocalizabilityBasis(
          makeTunnelSamples());

  ASSERT_TRUE(result.available);

  EXPECT_NEAR(
      result.rotation_eigenvalues(0),
      0.0,
      kTolerance);

  const double alignment =
      std::abs(
          result.rotation_eigenvectors
              .col(0)
              .dot(Eigen::Vector3d::UnitX()));

  EXPECT_GT(
      alignment,
      1.0 - kTolerance);
}

TEST(
    LidarLocalizabilityBasis,
    RigidRotationTransformsGramMatricesAndPreservesSpectrum)
{
  const auto samples =
      makeTunnelSamples();

  const auto original =
      computeLidarLocalizabilityBasis(samples);

  ASSERT_TRUE(original.available);

  Eigen::Matrix3d rotation;
  rotation <<
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0;

  std::vector<Sample> rotated_samples;
  rotated_samples.reserve(samples.size());

  for (const auto &sample : samples)
  {
    rotated_samples.push_back(
        {
            rotation * sample.point_lidar_m,
            rotation * sample.normal_lidar
        });
  }

  const auto rotated =
      computeLidarLocalizabilityBasis(
          rotated_samples);

  ASSERT_TRUE(rotated.available);

  const Eigen::Matrix3d expected_rotation_gram =
      rotation *
      original.rotation_gram_matrix *
      rotation.transpose();

  const Eigen::Matrix3d expected_translation_gram =
      rotation *
      original.translation_gram_matrix *
      rotation.transpose();

  EXPECT_LT(
      (
          rotated.rotation_gram_matrix -
          expected_rotation_gram
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          rotated.translation_gram_matrix -
          expected_translation_gram
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          rotated.rotation_eigenvalues -
          original.rotation_eigenvalues
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          rotated.translation_eigenvalues -
          original.translation_eigenvalues
      ).norm(),
      kTolerance);
}


TEST(
    LidarLocalizabilityBasis,
    ConvertsGlobalNormalUsingVerifiedGlobalImuAndImuLidarRotations)
{
  // R_GI: +90 deg around Z.
  Eigen::Matrix3d rotation_global_imu;
  rotation_global_imu <<
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0;

  // R_IL: +90 deg around X.
  Eigen::Matrix3d rotation_imu_lidar;
  rotation_imu_lidar <<
      1.0, 0.0,  0.0,
      0.0, 0.0, -1.0,
      0.0, 1.0,  0.0;

  const Eigen::Vector3d expected_point_lidar(
      2.0, -1.0, 0.5);

  const Eigen::Vector3d expected_normal_lidar =
      Eigen::Vector3d::UnitY();

  const Eigen::Vector3d normal_global =
      rotation_global_imu *
      rotation_imu_lidar *
      expected_normal_lidar;

  const auto sample =
      makeLidarLocalizabilitySample(
          expected_point_lidar,
          normal_global,
          rotation_global_imu,
          rotation_imu_lidar);

  EXPECT_LT(
      (
          sample.point_lidar_m -
          expected_point_lidar
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          sample.normal_lidar -
          expected_normal_lidar
      ).norm(),
      kTolerance);
}
