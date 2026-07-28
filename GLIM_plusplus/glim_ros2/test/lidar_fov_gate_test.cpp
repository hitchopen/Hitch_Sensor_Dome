#include <glim_ros/lidar_fov_gate.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace {

sensor_msgs::msg::PointCloud2 makeElevationCloud(
    double minimum_elevation_deg,
    double maximum_elevation_deg,
    size_t point_count,
    bool add_high_outlier = false) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(point_count);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);

  for (const auto& field : std::vector<std::pair<const char*, uint32_t>>{
           {"x", 0}, {"y", 4}, {"z", 8}}) {
    sensor_msgs::msg::PointField point_field;
    point_field.name = field.first;
    point_field.offset = field.second;
    point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_field.count = 1;
    cloud.fields.push_back(point_field);
  }

  constexpr double kDegreesToRadians =
      3.141592653589793238462643383279502884 / 180.0;
  for (size_t i = 0; i < point_count; ++i) {
    double elevation_deg =
        point_count > 1
            ? minimum_elevation_deg +
                  (maximum_elevation_deg - minimum_elevation_deg) *
                      static_cast<double>(i) /
                      static_cast<double>(point_count - 1)
            : minimum_elevation_deg;
    if (add_high_outlier && i == point_count - 1) {
      elevation_deg = 60.0;
    }

    const double elevation = elevation_deg * kDegreesToRadians;
    const float x = static_cast<float>(10.0 * std::cos(elevation));
    const float y = 0.0F;
    const float z = static_cast<float>(10.0 * std::sin(elevation));
    uint8_t* point = cloud.data.data() + i * cloud.point_step;
    std::memcpy(point, &x, sizeof(x));
    std::memcpy(point + 4, &y, sizeof(y));
    std::memcpy(point + 8, &z, sizeof(z));
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeInterleavedChannelCloud(size_t point_count) {
  auto cloud = makeElevationCloud(-15.0, 15.0, point_count);
  constexpr double kDegreesToRadians =
      3.141592653589793238462643383279502884 / 180.0;
  constexpr double kElevationsDeg[] = {-15.0, -9.0, -3.0, 3.0, 9.0, 15.0};
  for (size_t i = 0; i < point_count; ++i) {
    const double elevation =
        kElevationsDeg[i % 6] * kDegreesToRadians;
    const float x = static_cast<float>(10.0 * std::cos(elevation));
    const float z = static_cast<float>(10.0 * std::sin(elevation));
    uint8_t* point = cloud.data.data() + i * cloud.point_step;
    std::memcpy(point, &x, sizeof(x));
    std::memcpy(point + 8, &z, sizeof(z));
  }
  return cloud;
}

TEST(LidarFovGate, AcceptsNominalRobinWCoverage) {
  const auto cloud = makeElevationCloud(-15.0, 15.0, 1001);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_NEAR(measurement.span_deg, 29.7, 0.1);
}

TEST(LidarFovGate, RejectsNarrowCoverage) {
  const auto cloud = makeElevationCloud(-10.0, 10.0, 1001);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_LT(measurement.span_deg, 21.0);
}

TEST(LidarFovGate, IsNotFooledByOneHighAngleOutlier) {
  const auto cloud = makeElevationCloud(-10.0, 10.0, 1001, true);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_LT(measurement.span_deg, 21.0);
}

TEST(LidarFovGate, RejectsTooFewValidReturns) {
  const auto cloud = makeElevationCloud(-15.0, 15.0, 99);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_FALSE(measurement.valid);
  EXPECT_STREQ(measurement.reason, "too few finite nonzero xyz returns");
}

TEST(LidarFovGate, RejectsMalformedCoordinateSchema) {
  auto cloud = makeElevationCloud(-15.0, 15.0, 1001);
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT64;
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_FALSE(measurement.valid);
  EXPECT_STREQ(
      measurement.reason,
      "z field is not FLOAT32/count=1 inside point_step");
}

TEST(LidarFovGate, BoundedSamplingDoesNotAliasInterleavedChannels) {
  // 120000 / 20000 = 6. A fixed stride of six would inspect only one channel
  // and falsely reject this otherwise healthy cloud.
  const auto cloud = makeInterleavedChannelCloud(120000);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_EQ(measurement.sampled_points, 20000U);
  EXPECT_GT(measurement.span_deg, 29.0);
}

}  // namespace
