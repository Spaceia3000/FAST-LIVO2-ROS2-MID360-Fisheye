/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_LOCALIZABILITY_BASIS_H_
#define LIDAR_LOCALIZABILITY_BASIS_H_

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

/**
 * One geometric point-to-plane correspondence expressed entirely
 * in the LiDAR frame.
 */
struct LidarLocalizabilitySample
{
  Eigen::Vector3d point_lidar_m =
      Eigen::Vector3d::Zero();

  Eigen::Vector3d normal_lidar =
      Eigen::Vector3d::Zero();
};

/**
 * Raw geometric localizability basis.
 *
 * This structure contains geometric Gram matrices and their
 * eigendecompositions. It does NOT classify a direction as
 * localizable, partially localizable, degenerate, or healthy.
 *
 * Rotation:
 *
 *   tau_i = p_i x n_i
 *   A_rr  = sum_i tau_i tau_i^T
 *
 * Translation:
 *
 *   A_tt  = sum_i n_i n_i^T
 *
 * With points expressed in metres and unit normals:
 *
 *   A_rr and its eigenvalues have units of m^2.
 *   A_tt and its eigenvalues are dimensionless.
 *
 * Rotational and translational eigenvalues must therefore not be
 * compared directly as if they had the same physical scale.
 *
 * All points and normals must be expressed in the LiDAR frame.
 *
 * Eigenvalues are stored in Eigen's SelfAdjointEigenSolver order:
 *
 *   lambda_0 <= lambda_1 <= lambda_2
 *
 * Eigenvector column j corresponds to eigenvalue j.
 * Eigenvector signs are arbitrary.
 */
struct LidarLocalizabilityBasisSnapshot
{
  Eigen::Matrix3d rotation_gram_matrix =
      Eigen::Matrix3d::Zero();

  Eigen::Matrix3d translation_gram_matrix =
      Eigen::Matrix3d::Zero();

  Eigen::Vector3d rotation_eigenvalues =
      Eigen::Vector3d::Zero();

  Eigen::Matrix3d rotation_eigenvectors =
      Eigen::Matrix3d::Identity();

  Eigen::Vector3d translation_eigenvalues =
      Eigen::Vector3d::Zero();

  Eigen::Matrix3d translation_eigenvectors =
      Eigen::Matrix3d::Identity();

  std::size_t input_samples{0U};
  std::size_t used_samples{0U};

  bool available{false};
};

inline LidarLocalizabilityBasisSnapshot
computeLidarLocalizabilityBasis(
    const std::vector<LidarLocalizabilitySample> &samples)
{
  LidarLocalizabilityBasisSnapshot result;

  result.input_samples = samples.size();

  for (const auto &sample : samples)
  {
    if (
        !sample.point_lidar_m.allFinite() ||
        !sample.normal_lidar.allFinite())
    {
      continue;
    }

    const double normal_norm =
        sample.normal_lidar.norm();

    if (
        !std::isfinite(normal_norm) ||
        !(normal_norm > 0.0))
    {
      continue;
    }

    const Eigen::Vector3d normal =
        sample.normal_lidar / normal_norm;

    const Eigen::Vector3d torque =
        sample.point_lidar_m.cross(normal);

    if (!torque.allFinite())
    {
      continue;
    }

    result.rotation_gram_matrix.noalias() +=
        torque * torque.transpose();

    result.translation_gram_matrix.noalias() +=
        normal * normal.transpose();

    ++result.used_samples;
  }

  if (result.used_samples == 0U)
  {
    return result;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
      rotation_solver(result.rotation_gram_matrix);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
      translation_solver(result.translation_gram_matrix);

  if (
      rotation_solver.info() != Eigen::Success ||
      translation_solver.info() != Eigen::Success)
  {
    return result;
  }

  result.rotation_eigenvalues =
      rotation_solver.eigenvalues();

  result.rotation_eigenvectors =
      rotation_solver.eigenvectors();

  result.translation_eigenvalues =
      translation_solver.eigenvalues();

  result.translation_eigenvectors =
      translation_solver.eigenvectors();

  result.available =
      result.rotation_gram_matrix.allFinite() &&
      result.translation_gram_matrix.allFinite() &&
      result.rotation_eigenvalues.allFinite() &&
      result.rotation_eigenvectors.allFinite() &&
      result.translation_eigenvalues.allFinite() &&
      result.translation_eigenvectors.allFinite();

  return result;
}

#endif  // LIDAR_LOCALIZABILITY_BASIS_H_
