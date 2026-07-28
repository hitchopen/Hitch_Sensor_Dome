#include "gicp_localization/navsat_fix_gate.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace {

sensor_msgs::msg::NavSatFix validFix() {
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 10;
  fix.latitude = 37.8715;
  fix.longitude = -122.273;
  fix.altitude = 50.0;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 0.001;
  fix.position_covariance[4] = 0.001;
  fix.position_covariance[8] = 0.0025;
  return fix;
}

}  // namespace

TEST(NavSatFixGate, AcceptsFreshRtkClassFix) {
  const auto result = gicp_localization::validateNavSatFix(
      validFix(), 10.1, true, 0.10, 0.5, 0.05);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.max_position_stddev_m, 0.05, 1.0e-12);
}

TEST(NavSatFixGate, RejectsUnknownZeroAndNanCovariance) {
  auto fix = validFix();
  fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  EXPECT_FALSE(gicp_localization::validateNavSatFix(
      fix, 10.1, true, 0.10, 0.5, 0.05).accepted);

  fix = validFix();
  fix.position_covariance[4] = 0.0;
  EXPECT_FALSE(gicp_localization::validateNavSatFix(
      fix, 10.1, true, 0.10, 0.5, 0.05).accepted);

  fix = validFix();
  fix.position_covariance[8] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gicp_localization::validateNavSatFix(
      fix, 10.1, true, 0.10, 0.5, 0.05).accepted);
}

TEST(NavSatFixGate, RejectsStaleAndFutureFixes) {
  EXPECT_EQ(
      gicp_localization::validateNavSatFix(
          validFix(), 11.0, true, 0.10, 0.5, 0.05).failure,
      gicp_localization::FixGateFailure::STALE);
  EXPECT_EQ(
      gicp_localization::validateNavSatFix(
          validFix(), 9.0, true, 0.10, 0.5, 0.05).failure,
      gicp_localization::FixGateFailure::FUTURE);
}

TEST(NavSatFixGate, RejectsStatusOutsideRep145) {
  for (const int status : {-2, 3, 4, 10, 127}) {
    auto fix = validFix();
    fix.status.status = static_cast<int8_t>(status);
    const auto result = gicp_localization::validateNavSatFix(
        fix, 10.1, false, 0.10, 0.5, 0.05);
    EXPECT_FALSE(result.accepted) << "status=" << status;
    EXPECT_EQ(result.failure, gicp_localization::FixGateFailure::STATUS)
        << "status=" << status;
  }
}

TEST(NavSatFixGate, InvalidThresholdsFailClosed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto& thresholds :
       std::vector<std::array<double, 3>>{
           {0.0, 0.5, 0.05},
           {nan, 0.5, 0.05},
           {0.10, 0.0, 0.05},
           {0.10, nan, 0.05},
           {0.10, 0.5, -0.01},
           {0.10, 0.5, nan}}) {
    const auto result = gicp_localization::validateNavSatFix(
        validFix(), 10.1, true,
        thresholds[0], thresholds[1], thresholds[2]);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.failure, gicp_localization::FixGateFailure::INVALID);
  }
}
