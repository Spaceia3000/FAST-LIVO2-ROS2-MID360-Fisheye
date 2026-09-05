#ifndef LIDAR_LOCALIZABILITY_CALIBRATION_SUMMARY_H_
#define LIDAR_LOCALIZABILITY_CALIBRATION_SUMMARY_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "lidar_directional_localizability.h"

/**
 * Runtime policy for threshold-independent calibration telemetry.
 *
 * This policy controls only calibration-data collection.
 * It does NOT activate localizability classification, health decisions,
 * degeneracy mitigation, or any ESIKF modification.
 *
 * histogram_bins is a telemetry-resolution parameter, not a
 * localizability threshold.
 */
struct LidarLocalizabilityCalibrationPolicy
{
  bool enabled{false};

  int histogram_bins{0};

  bool configuration_valid() const
  {
    return histogram_bins > 0;
  }

  bool active() const
  {
    return
        enabled &&
        configuration_valid();
  }
};

struct LidarContributionHistogram
{
  std::vector<std::uint64_t> count;
  std::vector<double> sum;
};

struct LidarLocalizabilityCalibrationSummary
{
  LidarLocalizabilityBasisSnapshot basis;

  std::array<LidarContributionHistogram, 3>
      rotation;

  std::array<LidarContributionHistogram, 3>
      translation;

  std::size_t histogram_bins{0U};

  std::size_t input_contributions{0U};
  std::size_t used_contributions{0U};

  bool available{false};
};

inline bool
normalizeLidarContribution(
    const double value,
    double &normalized)
{
  constexpr double kNumericalTolerance =
      64.0 * std::numeric_limits<double>::epsilon();

  if (
      !std::isfinite(value) ||
      value < -kNumericalTolerance ||
      value > 1.0 + kNumericalTolerance)
  {
    return false;
  }

  normalized =
      std::clamp(
          value,
          0.0,
          1.0);

  return true;
}

inline std::size_t
lidarContributionHistogramBin(
    const double value,
    const std::size_t bins)
{
  if (value >= 1.0)
  {
    return bins - 1U;
  }

  const auto index =
      static_cast<std::size_t>(
          std::floor(
              value *
              static_cast<double>(bins)));

  return std::min(
      index,
      bins - 1U);
}

inline LidarLocalizabilityCalibrationSummary
buildLidarLocalizabilityCalibrationSummary(
    const LidarLocalizabilityBasisSnapshot &basis,
    const LidarDirectionalLocalizabilitySnapshot &directional,
    const std::size_t histogram_bins)
{
  LidarLocalizabilityCalibrationSummary result;

  result.basis =
      basis;

  result.histogram_bins =
      histogram_bins;

  result.input_contributions =
      directional.input_samples;

  if (
      histogram_bins == 0U ||
      !basis.available ||
      !directional.available ||
      basis.input_samples != directional.input_samples ||
      basis.used_samples != directional.used_samples ||
      directional.contributions.size() !=
          directional.used_samples ||
      directional.used_samples == 0U)
  {
    return result;
  }

  for (std::size_t axis = 0U; axis < 3U; ++axis)
  {
    result.rotation[axis].count.assign(
        histogram_bins,
        0U);

    result.rotation[axis].sum.assign(
        histogram_bins,
        0.0);

    result.translation[axis].count.assign(
        histogram_bins,
        0U);

    result.translation[axis].sum.assign(
        histogram_bins,
        0.0);
  }

  for (const auto &contribution : directional.contributions)
  {
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
      double rotation_value = 0.0;
      double translation_value = 0.0;

      if (
          !normalizeLidarContribution(
              contribution.rotation(
                  static_cast<Eigen::Index>(axis)),
              rotation_value) ||
          !normalizeLidarContribution(
              contribution.translation(
                  static_cast<Eigen::Index>(axis)),
              translation_value))
      {
        return LidarLocalizabilityCalibrationSummary{};
      }

      const std::size_t rotation_bin =
          lidarContributionHistogramBin(
              rotation_value,
              histogram_bins);

      const std::size_t translation_bin =
          lidarContributionHistogramBin(
              translation_value,
              histogram_bins);

      ++result.rotation[axis].count[
          rotation_bin];

      result.rotation[axis].sum[
          rotation_bin] +=
          rotation_value;

      ++result.translation[axis].count[
          translation_bin];

      result.translation[axis].sum[
          translation_bin] +=
          translation_value;
    }

    ++result.used_contributions;
  }

  result.available =
      result.used_contributions ==
      directional.used_samples;

  return result;
}

#endif  // LIDAR_LOCALIZABILITY_CALIBRATION_SUMMARY_H_
