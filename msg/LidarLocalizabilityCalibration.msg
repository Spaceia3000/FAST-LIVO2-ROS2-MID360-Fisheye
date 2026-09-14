# Threshold-independent LiDAR localizability calibration telemetry only.
# This message carries no FULL/PARTIAL/NONE, health, threshold, degeneracy-
# threshold, or classification semantics.

# The runtime producer owns header assignment. stamp identifies the producing
# LIO update; frame_id is the actual LiDAR frame in which these quantities were
# calculated and must not be hardcoded to a world/odom/camera_init frame.
std_msgs/Header header

# Structural snapshot availability only. This does not mean localizable,
# healthy, or classified.
bool available

# input_samples: samples presented to basis construction.
# used_samples: input samples with finite point/normal data and a finite,
# nonzero normal that contributed after normal normalization.
uint64 input_samples
uint64 used_samples

# B, the number of bins per direction. Each histogram array uses direction-
# major layout: index = direction * B + bin, for three directions.
uint32 histogram_bins

# Applies only when constructing rotational directional contributions c_r.
# The current validated value is supplied by the producer constant; it does
# not scale A_rr and is not a degeneracy or classification threshold.
float64 rotation_contribution_normalization_radius_m

# A_rr = sum tau tau^T, where tau = p_L x n_L. Units: m^2. A_rr uses no
# 1 m normalization. A_tt = sum n_L n_L^T and is dimensionless after normal
# normalization. Neither matrix is H^T R^-1 H; never fuse or sum either with
# ESIKF measurement information.
# All 3x3 matrices are serialized ROW-MAJOR: linear index is 3*r + c and stores M(r,c).
float64[9] rotation_gram_matrix
float64[9] translation_gram_matrix

# Eigenvalues are ascending. After reconstructing each matrix, eigenvector
# column j corresponds to eigenvalue j. Eigenvector signs are raw/arbitrary.
float64[3] rotation_eigenvalues
float64[9] rotation_eigenvectors
float64[3] translation_eigenvalues
float64[9] translation_eigenvectors

# V1 histogram count/sum fields are unbounded sequences. When available, every
# array has exactly 3*B elements in the direction-major layout defined above.
uint64[] rotation_histogram_count
float64[] rotation_histogram_sum
uint64[] translation_histogram_count
float64[] translation_histogram_sum
