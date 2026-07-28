#include <glim_ros/lidar_fov_gate.hpp>

#include <algorithm>
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
    size_t high_outlier_count = 0) {
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
  const size_t base_point_count =
      point_count > high_outlier_count
          ? point_count - high_outlier_count
          : 0;
  for (size_t i = 0; i < point_count; ++i) {
    double elevation_deg = 75.0;
    if (i < base_point_count) {
      elevation_deg =
        base_point_count > 1
            ? minimum_elevation_deg +
                  (maximum_elevation_deg - minimum_elevation_deg) *
                      static_cast<double>(i) /
                      static_cast<double>(base_point_count - 1)
            : minimum_elevation_deg;
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

void convertToBigEndian(sensor_msgs::msg::PointCloud2& cloud) {
  if (!glim_ros::hostIsBigEndian()) {
    for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
      uint8_t* point = cloud.data.data() + i * cloud.point_step;
      for (const size_t offset : {size_t{0}, size_t{4}, size_t{8}}) {
        std::reverse(point + offset, point + offset + sizeof(float));
      }
    }
  }
  cloud.is_bigendian = true;
}

// Builds a far-field band plus `near_count` spurious returns placed INSIDE the
// near-field floor and spread across steep elevations. Spreading them is the
// point: a single disconnected cluster is already caught by the occupancy
// check, whereas a spread fills every intervening bin.
sensor_msgs::msg::PointCloud2 makeNearFieldContaminatedCloud(
    double minimum_elevation_deg,
    double maximum_elevation_deg,
    size_t point_count,
    size_t near_count,
    double near_range_m) {
  auto cloud = makeElevationCloud(
      minimum_elevation_deg, maximum_elevation_deg, point_count);
  constexpr double kDegreesToRadians =
      3.141592653589793238462643383279502884 / 180.0;
  for (size_t i = 0; i < near_count && i < point_count; ++i) {
    const double elevation_deg =
        10.0 + 65.0 * static_cast<double>(i) /
                   static_cast<double>(near_count > 1 ? near_count - 1 : 1);
    const double elevation = elevation_deg * kDegreesToRadians;
    const float x = static_cast<float>(near_range_m * std::cos(elevation));
    const float z = static_cast<float>(near_range_m * std::sin(elevation));
    uint8_t* point =
        cloud.data.data() + (point_count - 1 - i) * cloud.point_step;
    std::memcpy(point, &x, sizeof(x));
    std::memcpy(point + 8, &z, sizeof(z));
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeInterleavedChannelCloud(size_t point_count) {
  auto cloud = makeElevationCloud(-15.0, 15.0, point_count);
  constexpr double kDegreesToRadians =
      3.141592653589793238462643383279502884 / 180.0;
  constexpr double kElevationsDeg[] = {
      -15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
      1.0, 3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0};
  for (size_t i = 0; i < point_count; ++i) {
    const double elevation =
        kElevationsDeg[i % 16] * kDegreesToRadians;
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
  EXPECT_TRUE(measurement.occupancy_complete);
}

TEST(LidarFovGate, AcceptsContinuousNearNominalCoverage) {
  const auto cloud = makeElevationCloud(-14.75, 14.75, 1001);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_GT(measurement.span_deg, 29.0);
  EXPECT_TRUE(measurement.occupancy_complete);
}

TEST(LidarFovGate, RejectsNarrowCoverage) {
  const auto cloud = makeElevationCloud(-10.0, 10.0, 1001);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_LT(measurement.span_deg, 21.0);
}

TEST(LidarFovGate, IsNotFooledByOneHighAngleOutlier) {
  const auto cloud = makeElevationCloud(-10.0, 10.0, 1001, 1);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_LT(measurement.span_deg, 21.0);
}

TEST(LidarFovGate, IsNotFooledByDisconnectedHighAngleCluster) {
  for (const size_t outlier_count :
       {size_t{6}, size_t{25}, size_t{49}, size_t{51},
        size_t{100}, size_t{500}, size_t{950}}) {
    const auto cloud =
        makeElevationCloud(-10.0, 10.0, 1001, outlier_count);
    glim_ros::VerticalFovMeasurement measurement;
    EXPECT_FALSE(glim_ros::verticalFovAccepted(
        cloud, 27.0, 100, &measurement));
    EXPECT_FALSE(measurement.occupancy_complete)
        << "outlier_count=" << outlier_count;
    EXPECT_LT(measurement.occupied_bins, measurement.occupancy_bins)
        << "outlier_count=" << outlier_count;
  }
}

TEST(LidarFovGate, RejectsInvalidMinimumPointCount) {
  const auto cloud = makeElevationCloud(-15.0, 15.0, 1001);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 0, &measurement));
  EXPECT_FALSE(measurement.valid);
  EXPECT_STREQ(
      measurement.reason, "minimum_valid_points is outside [1, 10000]");
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
  // 120000 / 20000 = 6. A fixed stride of six would inspect only half of the
  // 16 elevation channels and leave occupancy gaps.
  const auto cloud = makeInterleavedChannelCloud(120000);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
  EXPECT_EQ(measurement.sampled_points, 20000U);
  EXPECT_GT(measurement.span_deg, 29.0);
}

TEST(LidarFovGate, AcceptsBigEndianAndOrganizedTightClouds) {
  auto cloud = makeElevationCloud(-15.0, 15.0, 1001);
  cloud.width = 91;
  cloud.height = 11;
  cloud.row_step = cloud.width * cloud.point_step;
  convertToBigEndian(cloud);

  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(
      cloud, 27.0, 100, &measurement));
}


TEST(LidarFovGate, NearFieldScatterCannotManufactureElevationSpan) {
  // 20 deg of real coverage at 10 m, contaminated by returns 0.2 m from the
  // sensor spread from +10 to +75 deg. Those points are geometrically incapable
  // of carrying a meaningful elevation, and if they are allowed to contribute
  // one they both inflate the span AND fill the occupancy bins, defeating both
  // halves of the gate at once.
  for (const size_t near_count : {size_t{40}, size_t{200}, size_t{600}}) {
    const auto cloud = makeNearFieldContaminatedCloud(
        -10.0, 10.0, 4000, near_count, 0.2);
    glim_ros::VerticalFovMeasurement measurement;
    EXPECT_FALSE(glim_ros::verticalFovAccepted(
        cloud, 27.0, 100, &measurement))
        << "near_count=" << near_count;
    EXPECT_EQ(measurement.near_field_points, near_count)
        << "near_count=" << near_count;
    EXPECT_LT(measurement.span_deg, 21.0) << "near_count=" << near_count;
  }
}

TEST(LidarFovGate, FarFieldReturnsAreUnaffectedByTheRangeFloor) {
  // Everything at or beyond the floor still counts, so a healthy sensor is
  // untouched by this filter.
  const auto cloud = makeElevationCloud(-15.0, 15.0, 4000);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_TRUE(glim_ros::verticalFovAccepted(cloud, 27.0, 100, &measurement));
  EXPECT_EQ(measurement.near_field_points, 0U);
  EXPECT_EQ(measurement.valid_points, measurement.sampled_points);
}

TEST(LidarFovGate, AllNearFieldCloudIsRejectedNotAccepted) {
  // A cloud made entirely of near-field scatter has no usable elevation axis.
  const auto cloud = makeNearFieldContaminatedCloud(
      -10.0, 10.0, 1000, 1000, 0.2);
  glim_ros::VerticalFovMeasurement measurement;
  EXPECT_FALSE(glim_ros::verticalFovAccepted(cloud, 27.0, 100, &measurement));
  EXPECT_EQ(measurement.valid_points, 0U);
  EXPECT_EQ(measurement.near_field_points, 1000U);
}

}  // namespace
