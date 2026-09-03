#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "lidar_directional_localizability.h"

namespace
{

constexpr double kTolerance = 1e-10;

LidarLocalizabilityBasisSnapshot
makeIdentityBasis()
{
  LidarLocalizabilityBasisSnapshot basis;

  basis.rotation_eigenvectors =
      Eigen::Matrix3d::Identity();

  basis.translation_eigenvectors =
      Eigen::Matrix3d::Identity();

  basis.input_samples = 1U;
  basis.used_samples = 1U;
  basis.available = true;

  return basis;
}

}  // namespace

TEST(
    LidarDirectionalLocalizability,
    EmptyInputIsUnavailable)
{
  const std::vector<LidarLocalizabilitySample>
      samples;

  const auto result =
      computeLidarDirectionalLocalizability(
          samples,
          makeIdentityBasis());

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.input_samples, 0U);
  EXPECT_EQ(result.used_samples, 0U);
  EXPECT_TRUE(result.contributions.empty());
}

TEST(
    LidarDirectionalLocalizability,
    UnavailableBasisRejectsInput)
{
  const std::vector<LidarLocalizabilitySample>
      samples = {
          {
              Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitY()
          }
      };

  const LidarLocalizabilityBasisSnapshot basis;

  const auto result =
      computeLidarDirectionalLocalizability(
          samples,
          basis);

  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.input_samples, 1U);
  EXPECT_EQ(result.used_samples, 0U);
}

TEST(
    LidarDirectionalLocalizability,
    ZeroTorqueRemainsFinite)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0)
  };

  LidarDirectionalContribution contribution;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          makeIdentityBasis(),
          contribution));

  EXPECT_LT(
      contribution.rotation.norm(),
      kTolerance);

  EXPECT_LT(
      (
          contribution.translation -
          Eigen::Vector3d::UnitX()
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    SubUnitTorqueIsPreserved)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d::UnitY()
  };

  LidarDirectionalContribution contribution;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          makeIdentityBasis(),
          contribution));

  const Eigen::Vector3d expected_rotation(
      0.0, 0.0, 0.5);

  EXPECT_LT(
      (
          contribution.rotation -
          expected_rotation
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    SuperUnitTorqueIsMappedToUnitSphere)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d::UnitY()
  };

  LidarDirectionalContribution contribution;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          makeIdentityBasis(),
          contribution));

  EXPECT_LT(
      (
          contribution.rotation -
          Eigen::Vector3d::UnitZ()
      ).norm(),
      kTolerance);

  EXPECT_NEAR(
      contribution.rotation.norm(),
      1.0,
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    NormalIsNormalizedForTranslationContribution)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, 4.0, 0.0)
  };

  LidarDirectionalContribution contribution;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          makeIdentityBasis(),
          contribution));

  EXPECT_LT(
      (
          contribution.translation -
          Eigen::Vector3d::UnitY()
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    ContributionsAreProjectedIntoProvidedEigenbases)
{
  auto basis =
      makeIdentityBasis();

  basis.rotation_eigenvectors.col(0) =
      Eigen::Vector3d::UnitZ();

  basis.rotation_eigenvectors.col(1) =
      Eigen::Vector3d::UnitX();

  basis.rotation_eigenvectors.col(2) =
      Eigen::Vector3d::UnitY();

  basis.translation_eigenvectors.col(0) =
      Eigen::Vector3d::UnitY();

  basis.translation_eigenvectors.col(1) =
      Eigen::Vector3d::UnitZ();

  basis.translation_eigenvectors.col(2) =
      Eigen::Vector3d::UnitX();

  const LidarLocalizabilitySample sample{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY()
  };

  LidarDirectionalContribution contribution;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          basis,
          contribution));

  EXPECT_LT(
      (
          contribution.rotation -
          Eigen::Vector3d::UnitX()
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          contribution.translation -
          Eigen::Vector3d::UnitX()
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    EigenvectorSignDoesNotChangeContribution)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d(0.4, 0.2, 0.1),
      Eigen::Vector3d(0.2, 0.7, 0.3)
  };

  auto basis =
      makeIdentityBasis();

  LidarDirectionalContribution original;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          basis,
          original));

  basis.rotation_eigenvectors.col(0) *= -1.0;
  basis.rotation_eigenvectors.col(2) *= -1.0;

  basis.translation_eigenvectors.col(1) *= -1.0;
  basis.translation_eigenvectors.col(2) *= -1.0;

  LidarDirectionalContribution sign_flipped;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          basis,
          sign_flipped));

  EXPECT_LT(
      (
          original.rotation -
          sign_flipped.rotation
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          original.translation -
          sign_flipped.translation
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    RigidFrameRotationPreservesDirectionalContribution)
{
  const LidarLocalizabilitySample sample{
      Eigen::Vector3d(0.5, 1.0, 0.2),
      Eigen::Vector3d(0.3, 0.4, 0.5)
  };

  auto basis =
      makeIdentityBasis();

  LidarDirectionalContribution original;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          sample,
          basis,
          original));

  Eigen::Matrix3d rotation;
  rotation <<
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0;

  LidarLocalizabilitySample rotated_sample{
      rotation * sample.point_lidar_m,
      rotation * sample.normal_lidar
  };

  auto rotated_basis =
      basis;

  rotated_basis.rotation_eigenvectors =
      rotation *
      basis.rotation_eigenvectors;

  rotated_basis.translation_eigenvectors =
      rotation *
      basis.translation_eigenvectors;

  LidarDirectionalContribution rotated;

  ASSERT_TRUE(
      makeLidarDirectionalContribution(
          rotated_sample,
          rotated_basis,
          rotated));

  EXPECT_LT(
      (
          original.rotation -
          rotated.rotation
      ).norm(),
      kTolerance);

  EXPECT_LT(
      (
          original.translation -
          rotated.translation
      ).norm(),
      kTolerance);
}

TEST(
    LidarDirectionalLocalizability,
    InvalidSamplesAreRejectedInBatch)
{
  const double nan =
      std::numeric_limits<double>::quiet_NaN();

  const std::vector<LidarLocalizabilitySample>
      samples = {
          {
              Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitY()
          },
          {
              Eigen::Vector3d(nan, 0.0, 0.0),
              Eigen::Vector3d::UnitY()
          },
          {
              Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::Zero()
          }
      };

  const auto result =
      computeLidarDirectionalLocalizability(
          samples,
          makeIdentityBasis());

  EXPECT_TRUE(result.available);
  EXPECT_EQ(result.input_samples, 3U);
  EXPECT_EQ(result.used_samples, 1U);
  ASSERT_EQ(result.contributions.size(), 1U);

  EXPECT_TRUE(
      result.contributions.front()
          .rotation.allFinite());

  EXPECT_TRUE(
      result.contributions.front()
          .translation.allFinite());
}
