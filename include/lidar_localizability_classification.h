#ifndef LIDAR_LOCALIZABILITY_CLASSIFICATION_H_
#define LIDAR_LOCALIZABILITY_CLASSIFICATION_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "lidar_directional_localizability_aggregation.h"

/**
 * X-ICP-style ternary localizability categories.
 *
 * FULL:
 *   Sufficient geometric information.
 *
 * PARTIAL:
 *   Useful but insufficient geometric information remains.
 *
 * NONE:
 *   Valid evidence was processed, but geometric information
 *   is insufficient.
 *
 * NONE must not be used as a substitute for invalid or
 * unavailable evidence.
 *
 * These categories do not directly represent frontend health,
 * estimator health, failure state, or an ESIKF mitigation action.
 */
enum class LidarLocalizabilityCategory
{
  kNone = 0,
  kPartial = 1,
  kFull = 2
};

/**
 * X-ICP classification thresholds.
 *
 * Mathematical ordering:
 *
 *   kappa_1 >= kappa_2 > kappa_3 >= 0
 *
 * No valid numerical defaults are intentionally provided.
 * Their values must not be copied blindly from another ICP,
 * LiDAR, correspondence model, or optimization architecture.
 */
struct LidarLocalizabilityClassificationPolicy
{
  double kappa_1_full =
      std::numeric_limits<double>::quiet_NaN();

  double kappa_2_partial =
      std::numeric_limits<double>::quiet_NaN();

  double kappa_3_minimum =
      std::numeric_limits<double>::quiet_NaN();

  bool valid() const
  {
    return
        std::isfinite(kappa_1_full) &&
        std::isfinite(kappa_2_partial) &&
        std::isfinite(kappa_3_minimum) &&
        kappa_3_minimum >= 0.0 &&
        kappa_2_partial > kappa_3_minimum &&
        kappa_1_full >= kappa_2_partial;
  }
};

/**
 * Result for one principal direction.
 *
 * available == false:
 *   No valid classification was produced.
 *
 * available == true:
 *   category is a meaningful geometric classification.
 *
 * Therefore:
 *
 *   unavailable != NONE
 */
struct LidarLocalizabilityDirectionClassification
{
  LidarLocalizabilityCategory category =
      LidarLocalizabilityCategory::kNone;

  bool available{false};
};

/**
 * Classification for the three rotational and three translational
 * principal directions.
 *
 * Index j corresponds to eigenvector column j of the geometric basis.
 *
 * available != localizable != healthy.
 */
struct LidarLocalizabilityClassificationSnapshot
{
  std::array<LidarLocalizabilityCategory, 3> rotation{{
      LidarLocalizabilityCategory::kNone,
      LidarLocalizabilityCategory::kNone,
      LidarLocalizabilityCategory::kNone}};

  std::array<LidarLocalizabilityCategory, 3> translation{{
      LidarLocalizabilityCategory::kNone,
      LidarLocalizabilityCategory::kNone,
      LidarLocalizabilityCategory::kNone}};

  bool available{false};
};

/**
 * X-ICP ternary decision tree for one principal direction:
 *
 *   FULL
 *     if L_c >= kappa_1 OR L_s >= kappa_2
 *
 *   PARTIAL
 *     if not FULL and
 *        (L_c >= kappa_2 OR L_s >= kappa_3)
 *
 *   NONE
 *     otherwise.
 *
 * Invalid input is represented by available=false rather than
 * by the valid geometric category NONE.
 */
inline LidarLocalizabilityDirectionClassification
classifyLidarLocalizabilityDirection(
    const double combined_contribution,
    const double strong_contribution,
    const LidarLocalizabilityClassificationPolicy &policy)
{
  LidarLocalizabilityDirectionClassification result;

  if (
      !policy.valid() ||
      !std::isfinite(combined_contribution) ||
      !std::isfinite(strong_contribution) ||
      combined_contribution < 0.0 ||
      strong_contribution < 0.0)
  {
    return result;
  }

  if (
      combined_contribution >= policy.kappa_1_full ||
      strong_contribution >= policy.kappa_2_partial)
  {
    result.category =
        LidarLocalizabilityCategory::kFull;

    result.available = true;

    return result;
  }

  if (
      combined_contribution >= policy.kappa_2_partial ||
      strong_contribution >= policy.kappa_3_minimum)
  {
    result.category =
        LidarLocalizabilityCategory::kPartial;

    result.available = true;

    return result;
  }

  result.category =
      LidarLocalizabilityCategory::kNone;

  result.available = true;

  return result;
}

/**
 * Classify all six principal directions from the aggregated
 * GF-M1D-C-D evidence.
 *
 * Any invalid directional value invalidates this classification
 * snapshot rather than silently converting it to NONE.
 *
 * No solver action, remapping, covariance modification,
 * constraint, health decision, or keyframe policy is applied here.
 */
inline LidarLocalizabilityClassificationSnapshot
classifyLidarDirectionalLocalizability(
    const LidarDirectionalAggregationSnapshot &aggregation,
    const LidarLocalizabilityClassificationPolicy &policy)
{
  LidarLocalizabilityClassificationSnapshot result;

  if (
      !aggregation.available ||
      !policy.valid() ||
      !aggregation.rotation_combined.allFinite() ||
      !aggregation.rotation_strong.allFinite() ||
      !aggregation.translation_combined.allFinite() ||
      !aggregation.translation_strong.allFinite())
  {
    return result;
  }

  for (Eigen::Index axis = 0; axis < 3; ++axis)
  {
    const std::size_t index =
        static_cast<std::size_t>(axis);

    const auto rotation_result =
        classifyLidarLocalizabilityDirection(
            aggregation.rotation_combined(axis),
            aggregation.rotation_strong(axis),
            policy);

    const auto translation_result =
        classifyLidarLocalizabilityDirection(
            aggregation.translation_combined(axis),
            aggregation.translation_strong(axis),
            policy);

    if (
        !rotation_result.available ||
        !translation_result.available)
    {
      return LidarLocalizabilityClassificationSnapshot{};
    }

    result.rotation[index] =
        rotation_result.category;

    result.translation[index] =
        translation_result.category;
  }

  result.available = true;

  return result;
}

#endif  // LIDAR_LOCALIZABILITY_CLASSIFICATION_H_
