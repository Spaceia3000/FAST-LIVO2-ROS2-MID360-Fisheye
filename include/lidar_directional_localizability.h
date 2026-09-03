/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DIRECTIONAL_LOCALIZABILITY_H_
#define LIDAR_DIRECTIONAL_LOCALIZABILITY_H_

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "lidar_localizability_basis.h"

/**
 * X-ICP-style unit-moment normalization radius.
 *
 * FAST-LIVO2 localizability points are expressed in metres and normals
 * are normalized before the cross product. Therefore the moment
 *
 *   tau = p_L x n_L
 *
 * has units of metres.
 *
 * This 1 m value is NOT a localizability/degeneracy threshold.
 * It only defines the unit-ball moment normalization used to prevent
 * distant correspondences from obtaining arbitrarily large rotational
 * contribution solely because of their range.
 */
inline constexpr double kLidarMomentNormalizationRadiusM = 1.0;

/**
 * Raw per-correspondence directional contribution.
 *
 * rotation:
 *
 *   tau_i = p_i x n_i
 *
 *   f_r,i = tau_i                       if ||tau_i|| < 1 m
 *           tau_i / ||tau_i||           otherwise
 *
 *   c_r,i = | V_r^T f_r,i |
 *
 * translation:
 *
 *   f_t,i = n_i
 *
 *   c_t,i = | V_t^T f_t,i |
 *
 * V_r and V_t contain the rotational and translational eigenvectors
 * as columns and must belong to the same LiDAR-frame geometric basis
 * used to construct the samples.
 *
 * These values are raw geometric evidence. They do NOT classify a
 * direction as localizable, partially localizable, non-localizable,
 * degenerate, or healthy.
 */
struct LidarDirectionalContribution
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d rotation =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d translation =
      Eigen::Vector3d::Zero();
};

using LidarDirectionalContributionVector =
    std::vector<
        LidarDirectionalContribution,
        Eigen::aligned_allocator<
            LidarDirectionalContribution>>;

/**
 * Batch output for raw directional localizability evidence.
 *
 * available means only that at least one valid correspondence produced
 * finite directional evidence from an available geometric basis.
 *
 * available != localizable != healthy.
 */
struct LidarDirectionalLocalizabilitySnapshot
{
  LidarDirectionalContributionVector
      contributions;

  std::size_t input_samples{0U};
  std::size_t used_samples{0U};

  bool available{false};
};

/**
 * Compute raw directional contribution for one LiDAR-frame
 * point-to-plane correspondence.
 *
 * Returns false when the sample or basis cannot produce valid finite
 * directional evidence.
 */
inline bool
makeLidarDirectionalContribution(
    const LidarLocalizabilitySample &sample,
    const LidarLocalizabilityBasisSnapshot &basis,
    LidarDirectionalContribution &result)
{
  result = LidarDirectionalContribution{};

  if (
      !basis.available ||
      !basis.rotation_eigenvectors.allFinite() ||
      !basis.translation_eigenvectors.allFinite() ||
      !sample.point_lidar_m.allFinite() ||
      !sample.normal_lidar.allFinite())
  {
    return false;
  }

  const double normal_norm =
      sample.normal_lidar.norm();

  if (
      !std::isfinite(normal_norm) ||
      !(normal_norm > 0.0))
  {
    return false;
  }

  const Eigen::Vector3d normal =
      sample.normal_lidar / normal_norm;

  const Eigen::Vector3d torque =
      sample.point_lidar_m.cross(normal);

  if (!torque.allFinite())
  {
    return false;
  }

  const double torque_norm =
      torque.norm();

  if (!std::isfinite(torque_norm))
  {
    return false;
  }

  Eigen::Vector3d rotation_alignment =
      torque;

  if (
      torque_norm >=
      kLidarMomentNormalizationRadiusM)
  {
    rotation_alignment /=
        torque_norm;
  }

  result.rotation =
      (
          basis.rotation_eigenvectors.transpose() *
          rotation_alignment
      ).cwiseAbs();

  result.translation =
      (
          basis.translation_eigenvectors.transpose() *
          normal
      ).cwiseAbs();

  return
      result.rotation.allFinite() &&
      result.translation.allFinite();
}

/**
 * Compute raw directional contribution for a batch of LiDAR-frame
 * correspondences using an already-computed geometric basis.
 *
 * No thresholds, categories, health decisions, filtering policy,
 * optimization constraints, or ESIKF modifications are applied here.
 */
inline LidarDirectionalLocalizabilitySnapshot
computeLidarDirectionalLocalizability(
    const std::vector<LidarLocalizabilitySample> &samples,
    const LidarLocalizabilityBasisSnapshot &basis)
{
  LidarDirectionalLocalizabilitySnapshot result;

  result.input_samples =
      samples.size();

  if (
      !basis.available ||
      !basis.rotation_eigenvectors.allFinite() ||
      !basis.translation_eigenvectors.allFinite())
  {
    return result;
  }

  result.contributions.reserve(
      samples.size());

  for (const auto &sample : samples)
  {
    LidarDirectionalContribution contribution;

    if (
        !makeLidarDirectionalContribution(
            sample,
            basis,
            contribution))
    {
      continue;
    }

    result.contributions.push_back(
        contribution);

    ++result.used_samples;
  }

  result.available =
      result.used_samples > 0U &&
      result.contributions.size() ==
          result.used_samples;

  return result;
}

#endif  // LIDAR_DIRECTIONAL_LOCALIZABILITY_H_
