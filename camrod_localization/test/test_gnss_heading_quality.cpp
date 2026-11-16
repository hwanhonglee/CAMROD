#include <gtest/gtest.h>

#include "camrod_localization/gnss_heading_quality.hpp"

using camrod_localization::GnssHeadingQuality;

namespace
{
constexpr int64_t kSec = 1000000000;
constexpr uint32_t kFixed = GnssHeadingQuality::kCarrSolnFixed | GnssHeadingQuality::kRelPosValid |
  GnssHeadingQuality::kRelPosHeadingValid;
constexpr uint32_t kFloat = 8 | GnssHeadingQuality::kRelPosValid | GnssHeadingQuality::kRelPosHeadingValid;

GnssHeadingQuality configured()
{
  GnssHeadingQuality q;
  q.configure(1.0, 1.0, 0.905, 0.05);
  return q;
}
}  // namespace

TEST(GnssHeadingQuality, NoSampleIsNotTrusted)
{
  const auto q = configured();
  EXPECT_FALSE(q.trusted(10 * kSec));
}

TEST(GnssHeadingQuality, FloatWithValidFlagsIsNotTrusted)
{
  auto q = configured();
  for (int i = 0; i < 30; ++i) {
    q.update(i * kSec / 10, kFloat, 0.905);
  }
  EXPECT_FALSE(q.trusted(3 * kSec));
}

TEST(GnssHeadingQuality, FixedIsTrustedOnlyAfterSettle)
{
  auto q = configured();
  q.update(0, kFixed, 0.905);
  EXPECT_FALSE(q.trusted(0));
  q.update(kSec / 2, kFixed, 0.905);
  EXPECT_FALSE(q.trusted(kSec / 2));
  q.update(kSec, kFixed, 0.905);
  EXPECT_TRUE(q.trusted(kSec));
}

TEST(GnssHeadingQuality, LosingFixRestartsSettle)
{
  auto q = configured();
  q.update(0, kFixed, 0.905);
  q.update(2 * kSec, kFixed, 0.905);
  EXPECT_TRUE(q.trusted(2 * kSec));
  q.update(2 * kSec + kSec / 10, kFloat, 0.905);
  EXPECT_FALSE(q.trusted(2 * kSec + kSec / 10));
  q.update(3 * kSec, kFixed, 0.905);
  EXPECT_FALSE(q.trusted(3 * kSec + kSec / 2));
  q.update(4 * kSec, kFixed, 0.905);
  EXPECT_TRUE(q.trusted(4 * kSec));
}

TEST(GnssHeadingQuality, MissingFlagsOrWrongBaselineAreNotFixed)
{
  const auto q = configured();
  EXPECT_FALSE(q.epochFixed(kFixed & ~GnssHeadingQuality::kRelPosValid, 0.905));
  EXPECT_FALSE(q.epochFixed(kFixed & ~GnssHeadingQuality::kRelPosHeadingValid, 0.905));
  EXPECT_FALSE(q.epochFixed(kFixed, 0.80));
  EXPECT_TRUE(q.epochFixed(kFixed, 0.93));
}

TEST(GnssHeadingQuality, StaleEpochIsNotTrusted)
{
  auto q = configured();
  q.update(0, kFixed, 0.905);
  q.update(2 * kSec, kFixed, 0.905);
  EXPECT_TRUE(q.trusted(2 * kSec));
  EXPECT_FALSE(q.trusted(3 * kSec + kSec / 10));
}

TEST(GnssHeadingQuality, BaselineCheckDisabledWithZero)
{
  GnssHeadingQuality q;
  q.configure(0.0, 1.0, 0.0, 0.05);
  EXPECT_TRUE(q.epochFixed(kFixed, 3.0));
  q.update(0, kFixed, 3.0);
  EXPECT_TRUE(q.trusted(0));
}
