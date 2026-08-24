// HH_260818 - Keep unknown/malformed detections out of the safety obstacle
// cloud without coupling the policy test to camera, PCL, or TensorRT runtime.

#include <limits>
#include <set>
#include <string>

#include "camrod_perception/classified_detection.hpp"
#include "gtest/gtest.h"

namespace camrod_perception
{

TEST(ClassifiedDetection, RejectsUnknownAndMalformedLabels)
{
  const std::set<std::string> unknown{"", "?", "unknown"};
  EXPECT_FALSE(IsClassifiedDetection("", unknown));
  EXPECT_FALSE(IsClassifiedDetection(" ? ", unknown));
  EXPECT_FALSE(IsClassifiedDetection("UNKNOWN", unknown));
}

TEST(ClassifiedDetection, AcceptsResolvedSemanticClass)
{
  const std::set<std::string> unknown{"", "?", "unknown"};
  EXPECT_EQ(ResolveClassLabel("person", "0"), "person");
  EXPECT_EQ(ResolveClassLabel("", "tent"), "tent");
  EXPECT_TRUE(IsClassifiedDetection(" Person ", unknown));
  EXPECT_TRUE(IsClassifiedDetection("80", unknown));
}

TEST(ClassifiedDetection, RejectsMalformedBoundingBoxes)
{
  EXPECT_TRUE(HasValidDetectionBox(960.0, 540.0, 120.0, 80.0));
  EXPECT_FALSE(HasValidDetectionBox(960.0, 540.0, 0.0, 80.0));
  EXPECT_FALSE(HasValidDetectionBox(960.0, 540.0, -1.0, 80.0));
  EXPECT_FALSE(
    HasValidDetectionBox(
      std::numeric_limits<double>::quiet_NaN(), 540.0, 120.0, 80.0));
  EXPECT_FALSE(
    HasValidDetectionBox(
      960.0, 540.0, std::numeric_limits<double>::infinity(), 80.0));
}

}  // namespace camrod_perception
