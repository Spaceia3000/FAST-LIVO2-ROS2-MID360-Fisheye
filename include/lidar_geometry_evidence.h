/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_GEOMETRY_EVIDENCE_H_
#define LIDAR_GEOMETRY_EVIDENCE_H_

#include <cstddef>
#include <vector>

#include "lidar_localizability_classification.h"

/**
 * Compact internal LiDAR geometric-evidence snapshot.
 *
 * This object deliberately excludes the FAST-LIVO2 measurement
 * information matrix H^T R^-1 H. That probabilistic measurement
 * evidence has a different tangent/frame contract and is transported
 * independently by LidarInformationSnapshot.
 *
 * No point-to-plane correspondences or per-correspondence directional
 * vectors are retained here.
 *
 * available != localizable != healthy.
 */
struct LidarGeometryEvidenceSnapshot
{
  LidarLocalizabilityBasisSnapshot basis;

  LidarDirectionalAggregationSnapshot aggregation;

  LidarLocalizabilityClassificationSnapshot classification;

  /**
   * Policies are retained as provenance: they record exactly which
   * operational thresholds generated aggregation/classification.
   */
  LidarDirectionalAggregationPolicy aggregation_policy;

  LidarLocalizabilityClassificationPolicy classification_policy;

  /**
   * True only when the supplied basis and supplied correspondence
   * batch are structurally consistent and the directional stage used
   * exactly the same valid-sample count as the basis.
   */
  /**
   * Structural consistency between the supplied correspondence batch
   * and the basis metadata.
   *
   * This verifies sample-count / valid-sample-count contracts only.
   * It is NOT a cryptographic or provenance proof that the basis was
   * produced from this exact correspondence batch.
   */
  bool input_contract_consistent{false};

  bool raw_geometry_available() const
  {
    return
        basis.available &&
        input_contract_consistent;
  }

  bool directional_evidence_available() const
  {
    return
        input_contract_consistent &&
        aggregation.available;
  }

  bool classification_available() const
  {
    return
        input_contract_consistent &&
        classification.available;
  }
};

/**
 * Assemble compact LiDAR geometric evidence from a basis that was
 * computed from the same LiDAR-frame correspondence batch.
 *
 * The function intentionally reuses the already-validated basis rather
 * than recomputing its eigendecomposition.
 *
 * The per-correspondence contribution vector is temporary and is
 * discarded after aggregation.
 */
inline LidarGeometryEvidenceSnapshot
buildLidarGeometryEvidence(
    const std::vector<LidarLocalizabilitySample> &samples,
    const LidarLocalizabilityBasisSnapshot &basis,
    const LidarDirectionalAggregationPolicy &aggregation_policy,
    const LidarLocalizabilityClassificationPolicy &classification_policy)
{
  LidarGeometryEvidenceSnapshot result;

  result.basis =
      basis;

  result.aggregation_policy =
      aggregation_policy;

  result.classification_policy =
      classification_policy;

  if (
      !basis.available ||
      basis.input_samples != samples.size() ||
      basis.used_samples == 0U ||
      basis.used_samples > basis.input_samples)
  {
    return result;
  }

  const auto directional =
      computeLidarDirectionalLocalizability(
          samples,
          basis);

  if (
      !directional.available ||
      directional.input_samples !=
          basis.input_samples ||
      directional.used_samples !=
          basis.used_samples)
  {
    return result;
  }

  result.input_contract_consistent = true;

  result.aggregation =
      aggregateLidarDirectionalLocalizability(
          directional.contributions,
          aggregation_policy);

  if (!result.aggregation.available)
  {
    return result;
  }

  if (
      result.aggregation.input_contributions !=
          directional.used_samples ||
      result.aggregation.used_contributions !=
          directional.used_samples)
  {
    result.input_contract_consistent = false;
    result.aggregation =
        LidarDirectionalAggregationSnapshot{};
    return result;
  }

  result.classification =
      classifyLidarDirectionalLocalizability(
          result.aggregation,
          classification_policy);

  return result;
}

#endif  // LIDAR_GEOMETRY_EVIDENCE_H_
