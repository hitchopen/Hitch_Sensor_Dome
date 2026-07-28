#include <limits>

#include <gtest/gtest.h>

#include "adapter/adapter_utils.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"

namespace {

fusion_engine_msgs::msg::Pose fixedPose(double xy_variance = 1.0e-4,
                                        double z_variance = 1.0e-3)
{
  fusion_engine_msgs::msg::Pose pose;
  pose.solution_type = 4;
  pose.position_covariance[0] = xy_variance;
  pose.position_covariance[4] = xy_variance;
  pose.position_covariance[8] = z_variance;
  return pose;
}

TEST(AdapterUtilsRtkGate, AcceptsFixedWithPopulatedCovariance)
{
  EXPECT_TRUE(adapter::posePassesRtkGate(fixedPose(), 1.0e-3, 5.0e-3));
}

TEST(AdapterUtilsRtkGate, RejectsFloatSolution)
{
  auto pose = fixedPose();
  pose.solution_type = 5;
  EXPECT_FALSE(adapter::posePassesRtkGate(pose, 1.0e-3, 5.0e-3));
}

TEST(AdapterUtilsRtkGate, RejectsZeroCovariance)
{
  EXPECT_FALSE(adapter::posePassesRtkGate(fixedPose(0.0), 1.0e-3, 5.0e-3));
}

TEST(AdapterUtilsRtkGate, RejectsNonFiniteCovariance)
{
  EXPECT_FALSE(adapter::posePassesRtkGate(
      fixedPose(std::numeric_limits<double>::quiet_NaN()),
      1.0e-3, 5.0e-3));
}

TEST(AdapterUtilsRtkGate, RejectsCovarianceAboveLimit)
{
  EXPECT_FALSE(adapter::posePassesRtkGate(fixedPose(2.0e-3), 1.0e-3, 5.0e-3));
}

TEST(AdapterUtilsEnuOrigin, RequiresExactlyThreeNumericFields)
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  EXPECT_TRUE(adapter::parseLocalEnuOrigin(
      "37.8715,-122.273,52.125", latitude, longitude, altitude));
  EXPECT_DOUBLE_EQ(latitude, 37.8715);
  EXPECT_DOUBLE_EQ(longitude, -122.273);
  EXPECT_DOUBLE_EQ(altitude, 52.125);

  EXPECT_FALSE(adapter::parseLocalEnuOrigin(
      "37.8715,-122.273,52.125 trailing",
      latitude, longitude, altitude));
  EXPECT_FALSE(adapter::parseLocalEnuOrigin(
      "37.8715,-122.273", latitude, longitude, altitude));
}

}  // namespace
