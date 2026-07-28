#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace glim_ros {

constexpr double kNominalRobinWVerticalFovDeg = 30.0;
constexpr double kDefaultMinimumVerticalFovDeg = 27.0;
constexpr size_t kDefaultMinimumFovPoints = 100;
constexpr size_t kMaximumFovSamplePoints = 20000;

struct VerticalFovMeasurement {
  bool valid = false;
  double lower_deg = std::numeric_limits<double>::quiet_NaN();
  double upper_deg = std::numeric_limits<double>::quiet_NaN();
  double span_deg = std::numeric_limits<double>::quiet_NaN();
  size_t total_points = 0;
  size_t sampled_points = 0;
  size_t valid_points = 0;
  const char* reason = "unmeasured";
};

struct LidarQualityConfig {
  double minimum_vertical_fov_deg = kDefaultMinimumVerticalFovDeg;
  size_t minimum_valid_points = kDefaultMinimumFovPoints;
  bool primary_validated = false;
  uint64_t primary_reject_count = 0;
};

inline bool hostIsBigEndian() {
  const uint16_t value = 0x0102;
  return *reinterpret_cast<const uint8_t*>(&value) == 0x01;
}

inline float readCloudFloat32(const uint8_t* data, bool data_is_bigendian) {
  uint8_t bytes[sizeof(float)];
  if (data_is_bigendian == hostIsBigEndian()) {
    std::memcpy(bytes, data, sizeof(bytes));
  } else {
    std::reverse_copy(data, data + sizeof(bytes), bytes);
  }

  float value = 0.0F;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

inline size_t deterministicBucketOffset(size_t sample_index, size_t width) {
  uint64_t value =
      static_cast<uint64_t>(sample_index) + 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  value ^= value >> 31U;
  return static_cast<size_t>(value % static_cast<uint64_t>(width));
}

// Measures the robust elevation coverage of a raw sensor cloud. Trimming the
// outer 0.5 percent at each end prevents isolated outliers from making a
// collapsed vertical field of view look healthy.
inline VerticalFovMeasurement measureVerticalFov(
    const sensor_msgs::msg::PointCloud2& cloud,
    size_t minimum_valid_points = kDefaultMinimumFovPoints) {
  VerticalFovMeasurement result;
  result.reason = "invalid PointCloud2 dimensions or layout";

  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0 ||
      cloud.row_step == 0 ||
      static_cast<size_t>(cloud.width) >
          std::numeric_limits<size_t>::max() /
              static_cast<size_t>(cloud.height)) {
    return result;
  }

  result.total_points =
      static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  const size_t minimum_row_step =
      static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.point_step);
  if (static_cast<size_t>(cloud.row_step) < minimum_row_step ||
      static_cast<size_t>(cloud.height) >
          std::numeric_limits<size_t>::max() /
              static_cast<size_t>(cloud.row_step) ||
      cloud.data.size() <
          static_cast<size_t>(cloud.height) *
              static_cast<size_t>(cloud.row_step)) {
    return result;
  }

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  for (const auto& field : cloud.fields) {
    const bool valid_coordinate =
        field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1 &&
        static_cast<size_t>(field.offset) + sizeof(float) <= cloud.point_step;
    if (field.name == "x") {
      if (!valid_coordinate) {
        result.reason = "x field is not FLOAT32/count=1 inside point_step";
        return result;
      }
      x_offset = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      if (!valid_coordinate) {
        result.reason = "y field is not FLOAT32/count=1 inside point_step";
        return result;
      }
      y_offset = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      if (!valid_coordinate) {
        result.reason = "z field is not FLOAT32/count=1 inside point_step";
        return result;
      }
      z_offset = static_cast<int>(field.offset);
    }
  }
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    result.reason = "missing x/y/z FLOAT32 fields";
    return result;
  }

  const size_t sample_count =
      std::min(result.total_points, kMaximumFovSamplePoints);
  const size_t bucket_base = result.total_points / sample_count;
  const size_t wider_bucket_count = result.total_points % sample_count;
  std::vector<double> elevations_deg;
  elevations_deg.reserve(sample_count);
  constexpr double kRadiansToDegrees =
      180.0 / 3.141592653589793238462643383279502884;

  for (size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    const size_t bucket_begin =
        sample_index * bucket_base +
        std::min(sample_index, wider_bucket_count);
    const size_t bucket_width =
        bucket_base + (sample_index < wider_bucket_count ? 1 : 0);
    const size_t point_index =
        bucket_begin +
        deterministicBucketOffset(sample_index, bucket_width);
    const size_t row = point_index / static_cast<size_t>(cloud.width);
    const size_t column = point_index % static_cast<size_t>(cloud.width);
    const uint8_t* point =
        cloud.data.data() + row * static_cast<size_t>(cloud.row_step) +
        column * static_cast<size_t>(cloud.point_step);
    ++result.sampled_points;

    const float x = readCloudFloat32(
        point + x_offset, cloud.is_bigendian);
    const float y = readCloudFloat32(
        point + y_offset, cloud.is_bigendian);
    const float z = readCloudFloat32(
        point + z_offset, cloud.is_bigendian);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const double horizontal_range =
        std::hypot(static_cast<double>(x), static_cast<double>(y));
    const double range =
        std::hypot(horizontal_range, static_cast<double>(z));
    if (!std::isfinite(range) || range < 1.0e-4) {
      continue;
    }
    elevations_deg.push_back(
        std::atan2(static_cast<double>(z), horizontal_range) *
        kRadiansToDegrees);
  }

  result.valid_points = elevations_deg.size();
  if (result.valid_points < minimum_valid_points) {
    result.reason = "too few finite nonzero xyz returns";
    return result;
  }

  size_t trim_count = std::max<size_t>(1, result.valid_points / 200);
  if (2 * trim_count >= result.valid_points) {
    trim_count = 0;
  }
  const size_t lower_index = trim_count;
  const size_t upper_index = result.valid_points - 1 - trim_count;

  auto lower = elevations_deg.begin() + static_cast<std::ptrdiff_t>(lower_index);
  std::nth_element(elevations_deg.begin(), lower, elevations_deg.end());
  result.lower_deg = *lower;

  auto upper = elevations_deg.begin() + static_cast<std::ptrdiff_t>(upper_index);
  std::nth_element(elevations_deg.begin(), upper, elevations_deg.end());
  result.upper_deg = *upper;
  result.span_deg = result.upper_deg - result.lower_deg;
  result.valid = std::isfinite(result.span_deg) && result.span_deg >= 0.0;
  result.reason = result.valid ? "ok" : "non-finite elevation span";
  return result;
}

inline bool verticalFovAccepted(
    const sensor_msgs::msg::PointCloud2& cloud,
    double minimum_vertical_fov_deg,
    size_t minimum_valid_points,
    VerticalFovMeasurement* measurement = nullptr) {
  VerticalFovMeasurement local =
      measureVerticalFov(cloud, minimum_valid_points);
  const bool accepted =
      local.valid && local.span_deg >= minimum_vertical_fov_deg;
  if (local.valid && !accepted) {
    local.reason = "robust vertical FOV is below the configured minimum";
  }
  if (measurement) {
    *measurement = local;
  }
  return accepted;
}

}  // namespace glim_ros
