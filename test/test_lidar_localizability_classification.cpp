#include <limits>

#include <gtest/gtest.h>

#include "lidar_localizability_classification.h"

namespace
{

LidarLocalizabilityClassificationPolicy makePolicy()
{
  LidarLocalizabilityClassificationPolicy policy;

  policy.kappa_1_full = 10.0;
  policy.kappa_2_partial = 6.0;
  policy.kappa_3_minimum = 2.0;

  return policy;
}

}  // namespace

TEST(
    LidarLocalizabilityClassification,
    InvalidPolicyIsRejected)
{
  LidarLocalizabilityClassificationPolicy policy;

  EXPECT_FALSE(policy.valid());

  policy.kappa_1_full = 5.0;
  policy.kappa_2_partial = 2.0;
  policy.kappa_3_minimum = 2.0;

  EXPECT_FALSE(policy.valid());
}

TEST(
    LidarLocalizabilityClassification,
    CombinedContributionCanProduceFull)
{
  const auto result =
      classifyLidarLocalizabilityDirection(
          10.0,
          0.0,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.category,
      LidarLocalizabilityCategory::kFull);
}

TEST(
    LidarLocalizabilityClassification,
    StrongContributionCanProduceFull)
{
  const auto result =
      classifyLidarLocalizabilityDirection(
          0.0,
          6.0,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.category,
      LidarLocalizabilityCategory::kFull);
}

TEST(
    LidarLocalizabilityClassification,
    CombinedContributionCanProducePartial)
{
  const auto result =
      classifyLidarLocalizabilityDirection(
          6.0,
          0.0,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.category,
      LidarLocalizabilityCategory::kPartial);
}

TEST(
    LidarLocalizabilityClassification,
    SparseStrongContributionCanProducePartial)
{
  const auto result =
      classifyLidarLocalizabilityDirection(
          0.0,
          2.0,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.category,
      LidarLocalizabilityCategory::kPartial);
}

TEST(
    LidarLocalizabilityClassification,
    ValidInsufficientEvidenceProducesNone)
{
  const auto result =
      classifyLidarLocalizabilityDirection(
          5.99,
          1.99,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.category,
      LidarLocalizabilityCategory::kNone);
}

TEST(
    LidarLocalizabilityClassification,
    FullHasPriorityOverPartial)
{
  const auto combined_full =
      classifyLidarLocalizabilityDirection(
          10.0,
          2.0,
          makePolicy());

  ASSERT_TRUE(combined_full.available);

  EXPECT_EQ(
      combined_full.category,
      LidarLocalizabilityCategory::kFull);

  const auto strong_full =
      classifyLidarLocalizabilityDirection(
          6.0,
          6.0,
          makePolicy());

  ASSERT_TRUE(strong_full.available);

  EXPECT_EQ(
      strong_full.category,
      LidarLocalizabilityCategory::kFull);
}

TEST(
    LidarLocalizabilityClassification,
    InvalidEvidenceIsUnavailable)
{
  const double nan =
      std::numeric_limits<double>::quiet_NaN();

  const auto nan_result =
      classifyLidarLocalizabilityDirection(
          nan,
          4.0,
          makePolicy());

  EXPECT_FALSE(nan_result.available);

  const auto negative_result =
      classifyLidarLocalizabilityDirection(
          -1.0,
          4.0,
          makePolicy());

  EXPECT_FALSE(negative_result.available);
}

TEST(
    LidarLocalizabilityClassification,
    ClassifiesRotationAndTranslationIndependently)
{
  LidarDirectionalAggregationSnapshot aggregation;

  aggregation.rotation_combined <<
      10.0, 6.0, 1.0;

  aggregation.rotation_strong <<
      0.0, 0.0, 0.0;

  aggregation.translation_combined <<
      0.0, 0.0, 0.0;

  aggregation.translation_strong <<
      6.0, 2.0, 1.0;

  aggregation.available = true;

  const auto result =
      classifyLidarDirectionalLocalizability(
          aggregation,
          makePolicy());

  ASSERT_TRUE(result.available);

  EXPECT_EQ(
      result.rotation[0],
      LidarLocalizabilityCategory::kFull);

  EXPECT_EQ(
      result.rotation[1],
      LidarLocalizabilityCategory::kPartial);

  EXPECT_EQ(
      result.rotation[2],
      LidarLocalizabilityCategory::kNone);

  EXPECT_EQ(
      result.translation[0],
      LidarLocalizabilityCategory::kFull);

  EXPECT_EQ(
      result.translation[1],
      LidarLocalizabilityCategory::kPartial);

  EXPECT_EQ(
      result.translation[2],
      LidarLocalizabilityCategory::kNone);
}

TEST(
    LidarLocalizabilityClassification,
    UnavailableAggregationProducesUnavailableResult)
{
  const LidarDirectionalAggregationSnapshot aggregation;

  const auto result =
      classifyLidarDirectionalLocalizability(
          aggregation,
          makePolicy());

  EXPECT_FALSE(result.available);
}

TEST(
    LidarLocalizabilityClassification,
    InvalidAggregatedEvidenceProducesUnavailableResult)
{
  LidarDirectionalAggregationSnapshot aggregation;

  aggregation.rotation_combined <<
      -1.0, 6.0, 1.0;

  aggregation.rotation_strong <<
      0.0, 0.0, 0.0;

  aggregation.translation_combined <<
      0.0, 0.0, 0.0;

  aggregation.translation_strong <<
      6.0, 2.0, 1.0;

  aggregation.available = true;

  const auto result =
      classifyLidarDirectionalLocalizability(
          aggregation,
          makePolicy());

  EXPECT_FALSE(result.available);
}
