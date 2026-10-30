#include "fixed_interval_stats.h"
#include <gtest/gtest.h>

TEST(FixedIntervalStats, ReportsNearestRankPercentilesAndResets)
{
  FixedIntervalStats<8> stats;
  for (double value = 1.0; value <= 5.0; ++value) stats.add(value);
  const auto summary = stats.summary();
  EXPECT_EQ(summary.sample_count, 5U);
  EXPECT_EQ(summary.recent_sample_count, 5U);
  EXPECT_DOUBLE_EQ(summary.p50_recent, 3.0);
  EXPECT_DOUBLE_EQ(summary.p95_recent, 5.0);
  EXPECT_DOUBLE_EQ(summary.max_all, 5.0);
  stats.reset();
  EXPECT_EQ(stats.summary().sample_count, 0U);
}

TEST(FixedIntervalStats, LabelsBoundedWindowAndRetainsEarlyMaximum)
{
  FixedIntervalStats<4> stats;
  stats.add(1000.0);
  for (double value = 1.0; value <= 10.0; ++value) stats.add(value);
  const auto summary = stats.summary();
  EXPECT_EQ(summary.sample_count, 11U);
  EXPECT_EQ(summary.recent_sample_count, 4U);
  EXPECT_DOUBLE_EQ(summary.p50_recent, 8.0);
  EXPECT_DOUBLE_EQ(summary.p95_recent, 10.0);
  EXPECT_DOUBLE_EQ(summary.max_all, 1000.0);
}
