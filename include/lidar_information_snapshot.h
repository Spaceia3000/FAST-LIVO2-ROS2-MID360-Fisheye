/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_INFORMATION_SNAPSHOT_H_
#define LIDAR_INFORMATION_SNAPSHOT_H_

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Dense>

/**
 * Final LiDAR measurement-information snapshot produced by FAST-LIVO2.
 *
 * This is the measurement contribution
 *
 *   Lambda_L = H^T R^-1 H
 *
 * from the linearization used by the final accepted LiDAR ESIKF update.
 *
 * It is NOT:
 *   - posterior state information,
 *   - posterior state covariance,
 *   - a pose-graph constraint information matrix.
 *
 * Native FAST-LIVO2 tangent ordering:
 *
 *   [dtheta_Ix, dtheta_Iy, dtheta_Iz,
 *    dp_Gx,     dp_Gy,     dp_Gz]
 *
 * where rotation is updated by the right perturbation
 *
 *   R_GI <- R_GI * Exp(dtheta_I)
 *
 * while position is additive in the global frame G.
 */
struct LidarInformationSnapshot
{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  Matrix6d measurement_information =
      Matrix6d::Zero();

  std::size_t effective_features{0U};

  double mean_abs_point_to_plane_residual_m{0.0};

  std::int32_t final_iteration_index{-1};

  bool available{false};

  void reset()
  {
    measurement_information.setZero();
    effective_features = 0U;
    mean_abs_point_to_plane_residual_m = 0.0;
    final_iteration_index = -1;
    available = false;
  }

  void capture(
      const Matrix6d &information,
      const std::size_t feature_count,
      const double mean_abs_residual_m,
      const std::int32_t iteration)
  {
    measurement_information = information;
    effective_features = feature_count;
    mean_abs_point_to_plane_residual_m =
        mean_abs_residual_m;
    final_iteration_index = iteration;

    available =
        effective_features > 0U &&
        measurement_information.allFinite() &&
        std::isfinite(
            mean_abs_point_to_plane_residual_m) &&
        mean_abs_point_to_plane_residual_m >= 0.0 &&
        final_iteration_index >= 0;
  }
};

#endif  // LIDAR_INFORMATION_SNAPSHOT_H_
