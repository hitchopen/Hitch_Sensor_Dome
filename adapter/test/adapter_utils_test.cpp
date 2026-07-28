#include <filesystem>
#include <fstream>
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

TEST(AdapterUtilsEnuOrigin, ParsesLastThreeColumnsFromTtlFile)
{
  const auto path =
      std::filesystem::temp_directory_path() /
      "hitch_adapter_origin_test.ttl";
  {
    std::ofstream output(path);
    output << "1700000000.0,37.8715,-122.273,52.125\n";
  }

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  EXPECT_TRUE(adapter::parseLocalEnuOriginTtl(
      path.string(), latitude, longitude, altitude));
  EXPECT_DOUBLE_EQ(latitude, 37.8715);
  EXPECT_DOUBLE_EQ(longitude, -122.273);
  EXPECT_DOUBLE_EQ(altitude, 52.125);
  std::filesystem::remove(path);
}

TEST(P1ClockMapper, RejectsZeroAndNonFiniteTimestamps)
{
  adapter::P1ClockMapper mapper(60.0);
  EXPECT_FALSE(mapper.addPosePair(1000.0, 0.0));
  EXPECT_FALSE(mapper.addPosePair(
      1000.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(mapper.ready());
}

TEST(P1ClockMapper, RejectsForwardGlitchBeforeItPoisonsMinimumLag)
{
  adapter::P1ClockMapper mapper(60.0);
  ASSERT_TRUE(mapper.addPosePair(1000.0, 100.0));
  EXPECT_DOUBLE_EQ(mapper.toRos(100.0), 1000.0);

  // A sub-5-second P1 jump is below the node's epoch threshold, but its
  // arrival-minus-P1 lag is impossible in the established clock domain.
  EXPECT_FALSE(mapper.addPosePair(1000.1, 104.0));
  EXPECT_TRUE(mapper.addPosePair(1000.2, 100.2));
  EXPECT_NEAR(mapper.toRos(100.2), 1000.2, 1.0e-9);
}

TEST(P1ClockMapper, AllowsReceiveBacklogToConvergeItsLagFloor)
{
  adapter::P1ClockMapper mapper(60.0);
  ASSERT_TRUE(mapper.addPosePair(1002.0, 100.0));
  EXPECT_DOUBLE_EQ(mapper.toRos(100.0), 1002.0);

  // A draining queue can advance P1 while ROS arrival barely advances. It
  // lowers lag gradually and must remain eligible for the minimum envelope.
  ASSERT_TRUE(mapper.addPosePair(1002.0, 100.2));
  EXPECT_NEAR(mapper.driftMs(), 0.0, 1.0e-9);
}

TEST(P1ClockMapper, BacklogDoesNotReanchorOrRewindSlewClock)
{
  adapter::P1ClockMapper mapper(60.0);
  ASSERT_TRUE(mapper.addPosePair(1000.0, 100.0));
  EXPECT_DOUBLE_EQ(mapper.toRos(100.0), 1000.0);

  ASSERT_TRUE(mapper.addPosePair(1061.1, 161.0));
  const double mapped_new = mapper.toRos(161.0);
  EXPECT_NEAR(mapped_new, 1061.0305, 1.0e-9);

  const double mapped_backlog = mapper.toRos(100.0);
  EXPECT_NEAR(mapped_backlog, 1000.0305, 1.0e-9);
  EXPECT_NEAR(mapper.toRos(162.0), 1062.031, 1.0e-9);
}

TEST(P1ClockMapper, ExplicitResetIsObservableAndAllowsNewEpoch)
{
  adapter::P1ClockMapper mapper(60.0);
  ASSERT_TRUE(mapper.addPosePair(1000.0, 100.0));
  mapper.reset();
  EXPECT_EQ(mapper.resetCount(), 1U);
  EXPECT_FALSE(mapper.ready());
  EXPECT_TRUE(mapper.addPosePair(2000.0, 10.0));
  EXPECT_DOUBLE_EQ(mapper.toRos(10.0), 2000.0);
}

}  // namespace
