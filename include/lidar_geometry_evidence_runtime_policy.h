#ifndef LIDAR_GEOMETRY_EVIDENCE_RUNTIME_POLICY_H_
#define LIDAR_GEOMETRY_EVIDENCE_RUNTIME_POLICY_H_

#include <string>

#include "lidar_geometry_evidence.h"

/**
 * Provenance attached to one operational LiDAR localizability policy.
 *
 * These strings are metadata only. They do not participate in the
 * localizability mathematics.
 *
 * Sensor identity is deliberately NOT duplicated here. FAST-LIVO2
 * already owns that contract through preprocess.lidar_type.
 */
struct LidarGeometryEvidencePolicyProvenance
{
  std::string profile_id;
  std::string calibration_source;
  std::string calibration_version;

  bool complete() const
  {
    return
        !profile_id.empty() &&
        !calibration_source.empty() &&
        !calibration_version.empty();
  }
};

/**
 * Runtime configuration for the compact LiDAR geometry evidence.
 *
 * Default state is deliberately disabled and numerically unavailable.
 * No X-ICP / Perfectly Constrained experimental thresholds are used as
 * implicit defaults.
 *
 * enabled:
 *   Operator intent to use aggregated/classified localizability.
 *
 * thresholds_valid():
 *   Mathematical validity of the numerical policy.
 *
 * active():
 *   Both operator intent and mathematical validity.
 *
 * Provenance completeness is intentionally separate from mathematical
 * validity: missing metadata must not silently change the mathematics.
 */
struct LidarGeometryEvidenceRuntimePolicy
{
  bool enabled{false};

  LidarDirectionalAggregationPolicy
      aggregation_policy;

  LidarLocalizabilityClassificationPolicy
      classification_policy;

  LidarGeometryEvidencePolicyProvenance
      provenance;

  bool thresholds_valid() const
  {
    return
        aggregation_policy.valid() &&
        classification_policy.valid();
  }

  bool active() const
  {
    return
        enabled &&
        thresholds_valid();
  }

  bool provenance_complete() const
  {
    return provenance.complete();
  }
};

#endif  // LIDAR_GEOMETRY_EVIDENCE_RUNTIME_POLICY_H_
