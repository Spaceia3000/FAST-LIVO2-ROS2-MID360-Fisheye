/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_FRAME_PROVENANCE_H
#define LIDAR_FRAME_PROVENANCE_H

#include <cstddef>
#include <deque>
#include <string>

enum class LidarFrameProvenanceStatus
{
  NO_CONTRIBUTION = 0,
  VALID = 1,
  EMPTY_FRAME_ID = 2,
  CONFLICTING_FRAME_IDS = 3
};

/** Frame ownership for the points in one logical LiDAR partition. */
class LidarFrameProvenance
{
public:
  void reset()
  {
    status_ = LidarFrameProvenanceStatus::NO_CONTRIBUTION;
    candidate_frame_id_.clear();
  }

  void addContribution(
      const std::string &frame_id,
      const std::size_t point_count)
  {
    if (point_count == 0U)
    {
      return;
    }

    if (status_ ==
        LidarFrameProvenanceStatus::NO_CONTRIBUTION)
    {
      status_ = frame_id.empty()
          ? LidarFrameProvenanceStatus::EMPTY_FRAME_ID
          : LidarFrameProvenanceStatus::VALID;
    }
    else if (
        status_ !=
            LidarFrameProvenanceStatus::CONFLICTING_FRAME_IDS &&
        frame_id.empty())
    {
      status_ =
          LidarFrameProvenanceStatus::EMPTY_FRAME_ID;
    }

    if (frame_id.empty())
    {
      return;
    }

    if (candidate_frame_id_.empty())
    {
      candidate_frame_id_ = frame_id;
      return;
    }

    if (candidate_frame_id_ != frame_id)
    {
      status_ =
          LidarFrameProvenanceStatus::CONFLICTING_FRAME_IDS;
    }
  }

  bool valid() const
  {
    return status_ == LidarFrameProvenanceStatus::VALID;
  }

  LidarFrameProvenanceStatus status() const
  {
    return status_;
  }

  std::string frameId() const
  {
    return valid() ? candidate_frame_id_ : std::string();
  }

private:
  LidarFrameProvenanceStatus status_ =
      LidarFrameProvenanceStatus::NO_CONTRIBUTION;
  std::string candidate_frame_id_;
};

inline void addLidarFrameContributions(
    const std::string &frame_id,
    const std::size_t current_point_count,
    const std::size_t next_point_count,
    LidarFrameProvenance &current,
    LidarFrameProvenance &next)
{
  current.addContribution(
      frame_id,
      current_point_count);
  next.addContribution(
      frame_id,
      next_point_count);
}

inline void advanceLidarFramePartitions(
    LidarFrameProvenance &current,
    LidarFrameProvenance &next)
{
  current = next;
  next.reset();
}

/** Clear the three parallel fields owned by each accepted LiDAR scan. */
template <typename CloudType>
void clearLidarInputBuffers(
    std::deque<CloudType> &cloud_buffer,
    std::deque<double> &time_buffer,
    std::deque<std::string> &frame_buffer)
{
  cloud_buffer.clear();
  time_buffer.clear();
  frame_buffer.clear();
}

#endif  // LIDAR_FRAME_PROVENANCE_H
