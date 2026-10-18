#ifndef VISUAL_MAP_GROWTH_H_
#define VISUAL_MAP_GROWTH_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace visual_map_growth
{

constexpr double kVoxelSize = 0.5;

using VoxelBucket = std::array<std::int64_t, 3>;

inline std::optional<std::int64_t> voxelCoordinate(double coordinate)
{
  float location = static_cast<float>(coordinate / kVoxelSize);
  if (!std::isfinite(location)) return std::nullopt;
  if (location < 0.0F) location -= 1.0F;

  constexpr long double kInt64Lower =
      static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr long double kInt64UpperExclusive =
      -static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  const long double checked_location = static_cast<long double>(location);
  if (checked_location < kInt64Lower || checked_location >= kInt64UpperExclusive)
  {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(location);
}

inline std::optional<VoxelBucket> voxelBucket(double x, double y, double z)
{
  const auto voxel_x = voxelCoordinate(x);
  const auto voxel_y = voxelCoordinate(y);
  const auto voxel_z = voxelCoordinate(z);
  if (!voxel_x || !voxel_y || !voxel_z) return std::nullopt;
  return VoxelBucket{*voxel_x, *voxel_y, *voxel_z};
}

inline bool bucketAdmits(std::size_t current_size, std::int64_t max_points_per_voxel)
{
  if (max_points_per_voxel < 0) return false;
  return max_points_per_voxel == 0 ||
         current_size < static_cast<std::uint64_t>(max_points_per_voxel);
}

}  // namespace visual_map_growth

#endif  // VISUAL_MAP_GROWTH_H_
