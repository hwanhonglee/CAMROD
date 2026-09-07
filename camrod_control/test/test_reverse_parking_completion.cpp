#include <limits>

#include "camrod_control/reverse_parking_completion.hpp"
#include "gtest/gtest.h"

namespace camrod_control {

TEST(ReverseParkingCompletion, TravelLimitDoesNotMeanStationArrival) {
  const auto result = checkReverseParkingGoal(1.5, 0.0, 3.75, 0.0, 0.25);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.reached);
  EXPECT_DOUBLE_EQ(result.xy_error_m, 2.25);
}

TEST(ReverseParkingCompletion, ExactAndInsideRadiusAreValidGoals) {
  EXPECT_TRUE(checkReverseParkingGoal(1.0, 0.0, 1.0, 0.0, 0.25).reached);
  EXPECT_TRUE(checkReverseParkingGoal(0.75, 0.0, 1.0, 0.0, 0.25).reached);
  EXPECT_TRUE(checkReverseParkingGoal(0.85, 0.10, 1.0, 0.0, 0.25).reached);
  EXPECT_FALSE(checkReverseParkingGoal(0.749, 0.0, 1.0, 0.0, 0.25).reached);
}

TEST(ReverseParkingCompletion, AxisOnlyAndOvershootCannotPassTheGoalRadius) {
  EXPECT_FALSE(checkReverseParkingGoal(0.8, 0.4, 1.0, 0.0, 0.25).reached);
  EXPECT_FALSE(checkReverseParkingGoal(0.8, 0.2, 1.0, 0.0, 0.25).reached);
  EXPECT_FALSE(checkReverseParkingGoal(1.35, 0.0, 1.0, 0.0, 0.25).reached);
}

TEST(ReverseParkingCompletion, InvalidCoordinatesAndToleranceFailClosed) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  for (const double invalid : {nan, inf, -inf}) {
    EXPECT_FALSE(checkReverseParkingGoal(invalid, 0, 1, 0, .25).valid);
    EXPECT_FALSE(checkReverseParkingGoal(0, invalid, 1, 0, .25).valid);
    EXPECT_FALSE(checkReverseParkingGoal(0, 0, invalid, 0, .25).valid);
    EXPECT_FALSE(checkReverseParkingGoal(0, 0, 1, invalid, .25).valid);
    EXPECT_FALSE(checkReverseParkingGoal(0, 0, 1, 0, invalid).valid);
  }
  EXPECT_FALSE(checkReverseParkingGoal(0, 0, 0, 0, 0).valid);
  EXPECT_FALSE(checkReverseParkingGoal(0, 0, 0, 0, -1).valid);
}

}  // namespace camrod_control
