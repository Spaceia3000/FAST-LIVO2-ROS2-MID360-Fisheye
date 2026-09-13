#ifndef LIDAR_LOCALIZABILITY_CALIBRATION_TRANSPORT_H_
#define LIDAR_LOCALIZABILITY_CALIBRATION_TRANSPORT_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

#include "fast_livo/msg/lidar_localizability_calibration.hpp"
#include "lidar_directional_localizability.h"
#include "lidar_localizability_calibration_summary.h"

namespace lidar_localizability_calibration_transport
{

inline std::array<double, 9>
serializeMatrixRowMajor(const Eigen::Matrix3d &matrix)
{
  std::array<double, 9> serialized{};

  for (Eigen::Index row = 0; row < 3; ++row)
  {
    for (Eigen::Index column = 0; column < 3; ++column)
    {
      serialized[static_cast<std::size_t>(3 * row + column)] =
          matrix(row, column);
    }
  }

  return serialized;
}

inline fast_livo::msg::LidarLocalizabilityCalibration
serializeLidarLocalizabilityCalibration(
    const LidarLocalizabilityCalibrationSummary &summary)
{
  fast_livo::msg::LidarLocalizabilityCalibration message;

  message.available = summary.available;
  message.input_samples =
      static_cast<std::uint64_t>(summary.basis.input_samples);
  message.used_samples =
      static_cast<std::uint64_t>(summary.basis.used_samples);
  message.histogram_bins =
      static_cast<std::uint32_t>(summary.histogram_bins);
  message.rotation_contribution_normalization_radius_m =
      kLidarMomentNormalizationRadiusM;

  message.rotation_gram_matrix =
      serializeMatrixRowMajor(summary.basis.rotation_gram_matrix);
  message.translation_gram_matrix =
      serializeMatrixRowMajor(summary.basis.translation_gram_matrix);

  for (Eigen::Index index = 0; index < 3; ++index)
  {
    const auto serialized_index =
        static_cast<std::size_t>(index);

    message.rotation_eigenvalues[serialized_index] =
        summary.basis.rotation_eigenvalues(index);
    message.translation_eigenvalues[serialized_index] =
        summary.basis.translation_eigenvalues(index);
  }

  message.rotation_eigenvectors =
      serializeMatrixRowMajor(summary.basis.rotation_eigenvectors);
  message.translation_eigenvectors =
      serializeMatrixRowMajor(summary.basis.translation_eigenvectors);

  for (std::size_t direction = 0; direction < 3U; ++direction)
  {
    message.rotation_histogram_count.insert(
        message.rotation_histogram_count.end(),
        summary.rotation[direction].count.begin(),
        summary.rotation[direction].count.end());
    message.rotation_histogram_sum.insert(
        message.rotation_histogram_sum.end(),
        summary.rotation[direction].sum.begin(),
        summary.rotation[direction].sum.end());
    message.translation_histogram_count.insert(
        message.translation_histogram_count.end(),
        summary.translation[direction].count.begin(),
        summary.translation[direction].count.end());
    message.translation_histogram_sum.insert(
        message.translation_histogram_sum.end(),
        summary.translation[direction].sum.begin(),
        summary.translation[direction].sum.end());
  }

  return message;
}

}  // namespace lidar_localizability_calibration_transport

#endif  // LIDAR_LOCALIZABILITY_CALIBRATION_TRANSPORT_H_
