#ifndef VIO_DIAGNOSTICS_H_
#define VIO_DIAGNOSTICS_H_

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

struct VioDiagnosticsSnapshot
{
  std::size_t visual_voxels = 0;
  std::size_t visual_points = 0;
  std::int64_t configured_cap = 0;
  std::size_t max_voxel_occupancy = 0;
  std::uint64_t rejected_by_cap_total = 0;
  std::size_t geometric_voxels = 0;
  double retrieve_last_ms = 0.0;
  std::uint64_t candidates_direct = 0;
  std::uint64_t candidates_raycast = 0;
};

inline std::string formatVioDiagnostics(const VioDiagnosticsSnapshot &snapshot)
{
  std::ostringstream output;
  output << "[vio_map]"
         << " visual_voxels=" << snapshot.visual_voxels
         << " visual_points=" << snapshot.visual_points
         << " configured_cap=" << snapshot.configured_cap
         << " max_voxel_occupancy=" << snapshot.max_voxel_occupancy
         << " rejected_by_cap_total=" << snapshot.rejected_by_cap_total
         << " geometric_voxels=" << snapshot.geometric_voxels
         << " retrieve_last_ms=" << std::fixed << std::setprecision(3)
         << snapshot.retrieve_last_ms
         << " candidates_direct=" << snapshot.candidates_direct
         << " candidates_raycast=" << snapshot.candidates_raycast;
  return output.str();
}

#endif // VIO_DIAGNOSTICS_H_
