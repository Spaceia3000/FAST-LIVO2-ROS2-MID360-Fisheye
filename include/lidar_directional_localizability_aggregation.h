#ifndef LIDAR_DIRECTIONAL_LOCALIZABILITY_AGGREGATION_H_
#define LIDAR_DIRECTIONAL_LOCALIZABILITY_AGGREGATION_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "lidar_directional_localizability.h"

/**
 * Filtering policy for directional localizability contributions.
 *
 * These are operational policy parameters, not mathematical invariants.
 * They intentionally have no valid default values.
 *
 * Valid contract:
 *
 *   0 <= minimal <= strong <= 1
 *
 * The values operate directly on the absolute directional contribution
 * values produced by GF-M1D-C-C.
 */
struct LidarDirectionalAggregationPolicy
{
  double minimal_contribution_threshold =
      std::numeric_limits<double>::quiet_NaN();

  double strong_contribution_threshold =
      std::numeric_limits<double>::quiet_NaN();

  bool valid() const
  {
    return
        std::isfinite(minimal_contribution_threshold) &&
        std::isfinite(strong_contribution_threshold) &&
        minimal_contribution_threshold >= 0.0 &&
        strong_contribution_threshold >=
            minimal_contribution_threshold &&
        strong_contribution_threshold <= 1.0;
  }
};

/**
 * Aggregated directional evidence.
 *
 * For each rotational and translational eigenvector direction j:
 *
 *   L_c(j) = sum contributions >= minimal threshold
 *   L_s(j) = sum contributions >= strong threshold
 *
 * Counts are retained independently from the sums because the same
 * accumulated magnitude may arise from many weak correspondences or
 * from a smaller number of strongly aligned correspondences.
 *
 * This structure contains evidence only.
 *
 * available != localizable != healthy.
 */
struct LidarDirectionalAggregationSnapshot
{
  Eigen::Vector3d rotation_combined =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d rotation_strong =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d translation_combined =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d translation_strong =
      Eigen::Vector3d::Zero();

  std::array<std::size_t, 3>
      rotation_contributing_count{{0U, 0U, 0U}};

  std::array<std::size_t, 3>
      rotation_strong_count{{0U, 0U, 0U}};

  std::array<std::size_t, 3>
      translation_contributing_count{{0U, 0U, 0U}};

  std::array<std::size_t, 3>
      translation_strong_count{{0U, 0U, 0U}};

  std::size_t input_contributions{0U};
  std::size_t used_contributions{0U};

  bool available{false};
};

inline bool
isValidDirectionalContribution(
    const LidarDirectionalContribution &contribution)
{
  if (
      !contribution.rotation.allFinite() ||
      !contribution.translation.allFinite())
  {
    return false;
  }

  for (Eigen::Index axis = 0; axis < 3; ++axis)
  {
    if (
        contribution.rotation(axis) < 0.0 ||
        contribution.translation(axis) < 0.0)
    {
      return false;
    }
  }

  return true;
}

/**
 * Aggregate all valid directional contributions.
 *
 * Deliberately NO early-stop is used here.
 *
 * X-ICP defines L_c and L_s as sums over the available information
 * pairs. Early termination used by downstream optimized classifiers
 * would truncate these evidence values and make them dependent on
 * correspondence ordering.
 */
inline LidarDirectionalAggregationSnapshot
aggregateLidarDirectionalLocalizability(
    const LidarDirectionalContributionVector &contributions,
    const LidarDirectionalAggregationPolicy &policy)
{
  LidarDirectionalAggregationSnapshot result;

  result.input_contributions =
      contributions.size();

  if (!policy.valid())
  {
    return result;
  }

  for (const auto &contribution : contributions)
  {
    if (!isValidDirectionalContribution(contribution))
    {
      continue;
    }

    ++result.used_contributions;

    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      const double rotation_value =
          contribution.rotation(axis);

      const double translation_value =
          contribution.translation(axis);

      if (
          rotation_value >=
          policy.minimal_contribution_threshold)
      {
        result.rotation_combined(axis) +=
            rotation_value;

        ++result.rotation_contributing_count[
            static_cast<std::size_t>(axis)];
      }

      if (
          rotation_value >=
          policy.strong_contribution_threshold)
      {
        result.rotation_strong(axis) +=
            rotation_value;

        ++result.rotation_strong_count[
            static_cast<std::size_t>(axis)];
      }

      if (
          translation_value >=
          policy.minimal_contribution_threshold)
      {
        result.translation_combined(axis) +=
            translation_value;

        ++result.translation_contributing_count[
            static_cast<std::size_t>(axis)];
      }

      if (
          translation_value >=
          policy.strong_contribution_threshold)
      {
        result.translation_strong(axis) +=
            translation_value;

        ++result.translation_strong_count[
            static_cast<std::size_t>(axis)];
      }
    }
  }

  result.available =
      result.used_contributions > 0U &&
      result.rotation_combined.allFinite() &&
      result.rotation_strong.allFinite() &&
      result.translation_combined.allFinite() &&
      result.translation_strong.allFinite();

  return result;
}

#endif  // LIDAR_DIRECTIONAL_LOCALIZABILITY_AGGREGATION_H_
