#include "livo_camera_fallback.h"

#include <gtest/gtest.h>
#include <limits>

TEST(LivoCameraFallback, DefaultZeroPreservesBlockingMode)
{
  EXPECT_FALSE(livoCameraFallbackDue(0.0, 10.0, 100.0));
}

TEST(LivoCameraFallback, UsesSensorTimeThresholdInclusively)
{
  EXPECT_FALSE(livoCameraFallbackDue(0.30, 10.0, 10.299999));
  EXPECT_TRUE(livoCameraFallbackDue(0.30, 10.0, 10.30));
  EXPECT_FALSE(livoCameraFallbackDue(0.30, 10.0, 9.0));
  EXPECT_FALSE(livoCameraFallbackDue(
      0.30, 10.0, std::numeric_limits<double>::quiet_NaN()));
}

TEST(LivoCameraFallback, RejectsStaleImageAndAcceptsNextValidOne)
{
  EXPECT_TRUE(livoImageIsStale(20.0, 20.0));
  EXPECT_TRUE(livoImageIsStale(20.000009, 20.0));
  EXPECT_FALSE(livoImageIsStale(20.000011, 20.0));
}

TEST(LivoCameraFallback, FullScanCutOwnsEndpointWithoutDuplication)
{
  EXPECT_TRUE(livoPointBelongsToCut(30.1, 30.1, true));
  EXPECT_FALSE(livoPointBelongsToCut(30.1, 30.1, false));
  EXPECT_FALSE(livoPointBelongsToCut(30.100001, 30.1, true));
}
