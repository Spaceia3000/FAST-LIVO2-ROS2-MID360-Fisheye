#include <vector>

#include <gtest/gtest.h>

#include "lidar_geometry_evidence.h"

namespace
{

LidarLocalizabilityBasisSnapshot
makeIdentityBasis(
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

LidarDirectionalAggregationPolicy
makeAggregationPolicy()
{
  LidarDirectionalAggregationPolicy policy;

  policy.minimal_contribution_threshold = 0.25;
  policy.strong_contribution_threshold = 0.70;

  return policy;
}

LidarLocalizabilityClassificationPolicy
makeClassificationPolicy()
{
  LidarLocalizabilityClassificationPolicy policy;

  policy.kappa_1_full = 2.0;
  policy.kappa_2_partial = 1.5;
  policy.kappa_3_minimum = 0.5;

  return policy;
}

std::vector<LidarLocalizabilitySample>
makeSamples()
{
  return {
      {
          Eigen::Vector3d::UnitX(),
          Eigen::Vector3d::UnitY()
      },
      {
          Eigen::Vector3d::UnitX(),
          Eigen::Vector3d::UnitY()
      }
  };
}

}  // namespace

TEST(
    LidarGeometryEvidence,
    BuildsCompactEvidenceWithoutRetainingCorrespondences)
{
  const auto samples =
      makeSamples();

  const auto result =
      buildLidarGeometryEvidence(
          samples,
          makeIdentityBasis(samples.size()),
          makeAggregationPolicy(),
          makeClassificationPolicy());

  ASSERT_TRUE(result.input_contract_consistent);
  ASSERT_TRUE(result.raw_geometry_available());
  ASSERT_TRUE(result.directional_evidence_available());
  ASSERT_TRUE(result.classification_available());

  EXPECT_EQ(
      result.basis.input_samples,
      2U);

  EXPECT_EQ(
      result.basis.used_samples,
      2U);

  EXPECT_EQ(
      result.aggregation.input_contributions,
      2U);

  EXPECT_EQ(
      result.aggregation.used_contributions,
      2U);

  EXPECT_EQ(
      result.classification.rotation[2],
      LidarLocalizabilityCategory::kFull);

  EXPECT_EQ(
      result.classification.translation[1],
      LidarLocalizabilityCategory::kFull);

  EXPECT_EQ(
      result.classification.rotation[0],
      LidarLocalizabilityCategory::kNone);

  EXPECT_EQ(
      result.classification.translation[0],
      LidarLocalizabilityCategory::kNone);
}

TEST(
    LidarGeometryEvidence,
    DetectsBasisSampleCountMismatch)
{
  const auto samples =
      makeSamples();

  const auto result =
      buildLidarGeometryEvidence(
          samples,
          makeIdentityBasis(1U),
          makeAggregationPolicy(),
          makeClassificationPolicy());

  EXPECT_FALSE(result.input_contract_consistent);
  EXPECT_FALSE(result.raw_geometry_available());
  EXPECT_FALSE(result.directional_evidence_available());
  EXPECT_FALSE(result.classification_available());
}

TEST(
    LidarGeometryEvidence,
    InvalidAggregationPolicyPreservesRawBasisOnly)
{
  const auto samples =
      makeSamples();

  const LidarDirectionalAggregationPolicy
      invalid_aggregation_policy;

  const auto result =
      buildLidarGeometryEvidence(
          samples,
          makeIdentityBasis(samples.size()),
          invalid_aggregation_policy,
          makeClassificationPolicy());

  EXPECT_TRUE(result.input_contract_consistent);
  EXPECT_TRUE(result.raw_geometry_available());
  EXPECT_FALSE(result.directional_evidence_available());
  EXPECT_FALSE(result.classification_available());
}

TEST(
    LidarGeometryEvidence,
    InvalidClassificationPolicyPreservesDirectionalEvidence)
{
  const auto samples =
      makeSamples();

  const LidarLocalizabilityClassificationPolicy
      invalid_classification_policy;

  const auto result =
      buildLidarGeometryEvidence(
          samples,
          makeIdentityBasis(samples.size()),
          makeAggregationPolicy(),
          invalid_classification_policy);

  EXPECT_TRUE(result.input_contract_consistent);
  EXPECT_TRUE(result.raw_geometry_available());
  EXPECT_TRUE(result.directional_evidence_available());
  EXPECT_FALSE(result.classification_available());
}

TEST(
    LidarGeometryEvidence,
    UnavailableBasisStopsDownstreamEvidence)
{
  const auto samples =
      makeSamples();

  LidarLocalizabilityBasisSnapshot basis;

  basis.input_samples =
      samples.size();

  const auto result =
      buildLidarGeometryEvidence(
          samples,
          basis,
          makeAggregationPolicy(),
          makeClassificationPolicy());

  EXPECT_FALSE(result.raw_geometry_available());
  EXPECT_FALSE(result.input_contract_consistent);
  EXPECT_FALSE(result.directional_evidence_available());
  EXPECT_FALSE(result.classification_available());
}
