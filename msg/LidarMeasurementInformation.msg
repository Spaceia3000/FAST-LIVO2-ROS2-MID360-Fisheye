# Native FAST-LIVO2 LiDAR measurement-information snapshot.
#
# This message transports the LiDAR measurement contribution:
#
#   Lambda_L = H^T R^-1 H
#
# from the linearization used by the final accepted LiDAR ESIKF update.
#
# It is NOT:
#   - posterior state covariance,
#   - posterior state information,
#   - a pose-graph constraint information matrix.
#
# Native tangent ordering:
#
#   [dtheta_x, dtheta_y, dtheta_z,
#    dp_x,     dp_y,     dp_z]
#
# dtheta is the FAST-LIVO2 right rotational perturbation in the
# body-state frame identified by body_frame_id.
#
# dp is additive in the local odometry frame identified by
# header.frame_id.
#
# measurement_information is serialized ROW-MAJOR in this tangent basis.
#
# Rotational and translational coordinates have different physical
# scaling. No normalization is applied by this transport contract.

std_msgs/Header header

# Body/state frame associated with the right rotational perturbation.
string body_frame_id

# True means that a structurally usable snapshot was produced.
# It does NOT imply that the LiDAR geometry is non-degenerate.
bool available

# Number of effective point-to-plane constraints in the final
# accepted LiDAR linearization.
uint64 effective_features

# Mean absolute point-to-plane residual for that same linearization [m].
float64 mean_abs_point_to_plane_residual_m

# Zero-based ESIKF iteration index that produced this snapshot.
int32 final_iteration_index

# Row-major 6x6 native LiDAR measurement-information matrix.
float64[36] measurement_information
