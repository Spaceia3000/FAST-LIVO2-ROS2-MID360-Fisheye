#include <gtest/gtest.h>

#include "lidar_geometry_evidence_runtime_policy.h"

namespace
{

void setValidThresholds(
    LidarGeometryEvidenceRuntimePolicy &policy)
{
  policy.aggregation_policy
      .minimal_contribution_threshold = 0.20;

  policy.aggregation_policy
      .strong_contribution_threshold = 0.70;

  policy.classification_policy
      .kappa_1_full = 10.0;

  policy.classification_policy
      .kappa_2_partial = 6.0;

  policy.classification_policy
      .kappa_3_minimum = 2.0;
}

}  // namespace

TEST(
    LidarGeometryEvidenceRuntimePolicy,
    DefaultPolicyIsDisabledAndUnavailable)
{
  const LidarGeometryEvidenceRuntimePolicy policy;

  EXPECT_FALSE(policy.enabled);
  EXPECT_FALSE(policy.thresholds_valid());
  EXPECT_FALSE(policy.active());
  EXPECT_FALSE(policy.provenance_complete());
}

TEST(
    LidarGeometryEvidenceRuntimePolicy,
    ValidThresholdsDoNotActivateDisabledPolicy)
{
  LidarGeometryEvidenceRuntimePolicy policy;

  setValidThresholds(policy);

  EXPECT_TRUE(policy.thresholds_valid());
  EXPECT_FALSE(policy.active());
}

TEST(
    LidarGeometryEvidenceRuntimePolicy,
    EnabledValidPolicyBecomesActive)
{
  LidarGeometryEvidenceRuntimePolicy policy;

  policy.enabled = true;

  setValidThresholds(policy);

  EXPECT_TRUE(policy.thresholds_valid());
  EXPECT_TRUE(policy.active());
}

TEST(
    LidarGeometryEvidenceRuntimePolicy,
    EnabledInvalidPolicyDoesNotBecomeActive)
{
  LidarGeometryEvidenceRuntimePolicy policy;

  policy.enabled = true;

  policy.aggregation_policy
      .minimal_contribution_threshold = 0.80;

  policy.aggregation_policy
      .strong_contribution_threshold = 0.20;

  EXPECT_FALSE(policy.thresholds_valid());
  EXPECT_FALSE(policy.active());
}

TEST(
    LidarGeometryEvidenceRuntimePolicy,
    ProvenanceIsIndependentFromMathematicalValidity)
{
  LidarGeometryEvidenceRuntimePolicy policy;

  policy.enabled = true;

  setValidThresholds(policy);

  ASSERT_TRUE(policy.active());
  EXPECT_FALSE(policy.provenance_complete());

  policy.provenance.profile_id =
      "mid360_retail_calibration";

  policy.provenance.calibration_source =
      "empirical_runtime_calibration";

  policy.provenance.calibration_version =
      "v1";

  EXPECT_TRUE(policy.provenance_complete());
  EXPECT_TRUE(policy.active());
}
