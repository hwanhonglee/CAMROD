#include <cmath>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

#include "camrod_localization/gnss_heading_fallback.hpp"

namespace
{

constexpr int64_t kSecondNs = 1000000000LL;
constexpr double kMaxEkfYawCovariance = 1.0;

double radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

camrod_localization::GnssHeadingFallback configuredFallback()
{
  camrod_localization::GnssHeadingFallback fallback;
  fallback.configure(3.0, 5.0, 0.2);
  return fallback;
}

// HH_260807 - An unanchored EKF yaw may have an arbitrary startup map offset.
TEST(GnssHeadingFallback, RejectsFallbackBeforeFirstValidGnssAnchor)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, radians(20.0), 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(0, std::nullopt);

  EXPECT_FALSE(selected.usable());
  EXPECT_FALSE(selected.gnssYawUsable());
}

TEST(GnssHeadingFallback, PropagatesAnchoredHeadingWithTimeAlignedEkfDelta)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, radians(20.0), 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(0, radians(80.0)));
  fallback.recordEkfYaw(kSecondNs, radians(35.0), 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(kSecondNs, std::nullopt);

  ASSERT_TRUE(selected.usable());
  EXPECT_EQ(selected.source, camrod_localization::LeverArmHeadingSource::kEkfDelta);
  EXPECT_NEAR(selected.yaw, radians(95.0), 1.0e-12);
  EXPECT_FALSE(selected.gnssYawUsable());
}

TEST(GnssHeadingFallback, FreshGnssAlwaysTakesPrecedence)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, radians(20.0), 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(0, radians(80.0)));
  fallback.recordEkfYaw(kSecondNs, radians(35.0), 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(kSecondNs, radians(-40.0));

  ASSERT_TRUE(selected.usable());
  EXPECT_EQ(selected.source, camrod_localization::LeverArmHeadingSource::kFreshGnss);
  EXPECT_NEAR(selected.yaw, radians(-40.0), 1.0e-12);
  EXPECT_TRUE(selected.gnssYawUsable());
}

TEST(GnssHeadingFallback, ExpiresThreeSecondsAfterValidAnchor)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, 0.0, 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(0, 0.0));
  fallback.recordEkfYaw(3100000000LL, 0.1, 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(3100000000LL, std::nullopt);

  EXPECT_FALSE(selected.usable());
  EXPECT_FALSE(selected.gnssYawUsable());
}

TEST(GnssHeadingFallback, RejectsWhenFixHasNoTimeAlignedEkfSample)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, 0.0, 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(0, 0.0));
  fallback.recordEkfYaw(500000000LL, 0.1, 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(kSecondNs, std::nullopt);

  EXPECT_FALSE(selected.usable());
}

TEST(GnssHeadingFallback, NormalizesEkfDeltaAcrossPiWraparound)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(0, radians(179.0), 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(0, radians(170.0)));
  fallback.recordEkfYaw(kSecondNs, radians(-179.0), 0.1, kMaxEkfYawCovariance);

  const auto selected = fallback.select(kSecondNs, std::nullopt);

  ASSERT_TRUE(selected.usable());
  EXPECT_NEAR(selected.yaw, radians(172.0), 1.0e-12);
}

TEST(GnssHeadingFallback, InvalidEkfCovarianceCannotCreateOrExtendFallback)
{
  auto fallback = configuredFallback();
  fallback.recordEkfYaw(
    0, 0.0, std::numeric_limits<double>::quiet_NaN(), kMaxEkfYawCovariance);
  fallback.recordEkfYaw(100000000LL, 0.0, 0.0, kMaxEkfYawCovariance);
  fallback.recordEkfYaw(200000000LL, 0.0, 2.0, kMaxEkfYawCovariance);
  EXPECT_FALSE(fallback.anchorWithValidGnss(100000000LL, 0.0));

  fallback.recordEkfYaw(300000000LL, 0.0, 0.1, kMaxEkfYawCovariance);
  ASSERT_TRUE(fallback.anchorWithValidGnss(300000000LL, 0.0));
  fallback.recordEkfYaw(1300000000LL, 0.2, 2.0, kMaxEkfYawCovariance);

  const auto selected = fallback.select(1300000000LL, std::nullopt);
  EXPECT_FALSE(selected.usable());
  EXPECT_FALSE(selected.gnssYawUsable());
}

}  // namespace
