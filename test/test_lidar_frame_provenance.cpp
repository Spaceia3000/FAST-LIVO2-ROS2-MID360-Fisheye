#include <deque>
#include <string>

#include <gtest/gtest.h>

#include "lidar_frame_provenance.h"

TEST(
    LidarFrameProvenance,
    SingleScanPreservesExactFrameId)
{
  LidarFrameProvenance provenance;

  provenance.addContribution("mid360/laser", 17U);

  EXPECT_TRUE(provenance.valid());
  EXPECT_EQ(provenance.frameId(), "mid360/laser");
}

TEST(
    LidarFrameProvenance,
    SameFrameMergeRemainsValid)
{
  LidarFrameProvenance provenance;

  provenance.addContribution("lidar", 3U);
  provenance.addContribution("lidar", 5U);

  EXPECT_TRUE(provenance.valid());
  EXPECT_EQ(
      provenance.status(),
      LidarFrameProvenanceStatus::VALID);
  EXPECT_EQ(provenance.frameId(), "lidar");
}

TEST(
    LidarFrameProvenance,
    ConflictingFramesBecomeInvalid)
{
  LidarFrameProvenance provenance;

  provenance.addContribution("lidar_a", 3U);
  provenance.addContribution("lidar_b", 5U);

  EXPECT_FALSE(provenance.valid());
  EXPECT_EQ(
      provenance.status(),
      LidarFrameProvenanceStatus::CONFLICTING_FRAME_IDS);
  EXPECT_TRUE(provenance.frameId().empty());
}

TEST(
    LidarFrameProvenance,
    EmptyFrameBecomesInvalid)
{
  LidarFrameProvenance provenance;

  provenance.addContribution("", 3U);

  EXPECT_FALSE(provenance.valid());
  EXPECT_EQ(
      provenance.status(),
      LidarFrameProvenanceStatus::EMPTY_FRAME_ID);
  EXPECT_TRUE(provenance.frameId().empty());
}

TEST(
    LidarFrameProvenance,
    CurrentAndNextSplitPartitionsRemainIndependent)
{
  LidarFrameProvenance current;
  LidarFrameProvenance next;

  addLidarFrameContributions(
      "lidar_a",
      2U,
      0U,
      current,
      next);
  addLidarFrameContributions(
      "lidar_b",
      0U,
      4U,
      current,
      next);

  EXPECT_TRUE(current.valid());
  EXPECT_EQ(current.frameId(), "lidar_a");
  EXPECT_TRUE(next.valid());
  EXPECT_EQ(next.frameId(), "lidar_b");

  advanceLidarFramePartitions(current, next);

  EXPECT_TRUE(current.valid());
  EXPECT_EQ(current.frameId(), "lidar_b");
  EXPECT_EQ(
      next.status(),
      LidarFrameProvenanceStatus::NO_CONTRIBUTION);
}

TEST(
    LidarFrameProvenance,
    FrameContributesOnlyWhenPointsContribute)
{
  LidarFrameProvenance current;
  LidarFrameProvenance next;

  addLidarFrameContributions(
      "ignored_frame",
      0U,
      0U,
      current,
      next);

  EXPECT_EQ(
      current.status(),
      LidarFrameProvenanceStatus::NO_CONTRIBUTION);
  EXPECT_EQ(
      next.status(),
      LidarFrameProvenanceStatus::NO_CONTRIBUTION);

  addLidarFrameContributions(
      "contributing_frame",
      0U,
      1U,
      current,
      next);

  EXPECT_EQ(
      current.status(),
      LidarFrameProvenanceStatus::NO_CONTRIBUTION);
  EXPECT_TRUE(next.valid());
  EXPECT_EQ(next.frameId(), "contributing_frame");
}

TEST(
    LidarInputBuffers,
    RollbackClearPreservesAlignment)
{
  std::deque<int> clouds{1, 2};
  std::deque<double> times{1.0, 2.0};
  std::deque<std::string> frames{"lidar_a", "lidar_b"};

  clearLidarInputBuffers(clouds, times, frames);

  EXPECT_TRUE(clouds.empty());
  EXPECT_TRUE(times.empty());
  EXPECT_TRUE(frames.empty());
}
