/* Sensor-time rules for the opt-in LIVO camera-gap fallback. */
#ifndef LIVO_CAMERA_FALLBACK_H
#define LIVO_CAMERA_FALLBACK_H

#include <cmath>

inline bool livoCameraFallbackDue(
    const double wait_sensor_s,
    const double last_cut_time,
    const double newest_lidar_time)
{
  return std::isfinite(wait_sensor_s) && wait_sensor_s > 0.0 &&
      std::isfinite(last_cut_time) && std::isfinite(newest_lidar_time) &&
      newest_lidar_time >= last_cut_time &&
      newest_lidar_time - last_cut_time >= wait_sensor_s;
}

inline bool livoImageIsStale(
    const double capture_time,
    const double last_cut_time,
    const double epsilon_s = 1e-5)
{
  return !std::isfinite(capture_time) ||
      capture_time < last_cut_time + epsilon_s;
}

inline bool livoPointBelongsToCut(
    const double point_time,
    const double cut_time,
    const bool include_cut_endpoint)
{
  return include_cut_endpoint ? point_time <= cut_time : point_time < cut_time;
}

#endif  // LIVO_CAMERA_FALLBACK_H
