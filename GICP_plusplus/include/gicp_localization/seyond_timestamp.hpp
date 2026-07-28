#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace gicp_localization {

struct SeyondPointTimeRange {
  bool valid = false;
  double min_s = 0.0;
  double max_s = 0.0;
  size_t count = 0;
  size_t zero_timestamp_count = 0;
};

// Robin W is rated 10-20 FPS. This default is the SLOWEST supported rate, so a
// deployment that never configures it can only ever be too PERMISSIVE, never
// falsely reject a valid frame. Pass the deployment's real frame period to
// seyondCloudTimeContractValid() to get a gate that actually bites: at 20 FPS
// the default admits a 99 ms span, i.e. two fused frames, as if it were one.
inline constexpr double kSeyondFrameDurationSecondsDefault = 0.100;
inline constexpr double kSeyondPointTimeQuantumSeconds = 10.0e-6;
inline constexpr double kSeyondFrameTimeToleranceSeconds =
    kSeyondPointTimeQuantumSeconds;

inline SeyondPointTimeRange decodeSeyondPointTimeRange(
    const sensor_msgs::msg::PointCloud2& msg) {
  SeyondPointTimeRange range;
  int time_off = -1;
  for (const auto& field : msg.fields) {
    if (field.name == "timestamp" &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT64 &&
        field.count == 1) {
      time_off = static_cast<int>(field.offset);
      break;
    }
  }

  if (msg.is_bigendian || msg.width == 0 || msg.height == 0 ||
      msg.point_step == 0 || time_off < 0 ||
      static_cast<size_t>(time_off) + sizeof(double) > msg.point_step ||
      static_cast<size_t>(msg.width) >
          std::numeric_limits<size_t>::max() / msg.point_step) {
    return range;
  }

  const size_t expected_row_step =
      static_cast<size_t>(msg.width) * msg.point_step;
  if (msg.row_step != expected_row_step ||
      static_cast<size_t>(msg.height) >
          std::numeric_limits<size_t>::max() / expected_row_step) {
    return range;
  }

  const size_t expected_data_size =
      expected_row_step * static_cast<size_t>(msg.height);
  if (msg.data.size() != expected_data_size) {
    return range;
  }

  const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
  constexpr double kMinEpochSeconds = 1.0e6;
  constexpr double kMaxEpochSeconds = 3.0e9;
  range.min_s = std::numeric_limits<double>::infinity();
  range.max_s = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < num_points; ++i) {
    double point_s = 0.0;
    std::memcpy(
        &point_s,
        msg.data.data() + i * static_cast<size_t>(msg.point_step) + time_off,
        sizeof(double));
    // The pinned official driver publishes only parsed, in-range points and
    // hydrates every timestamp as packet_start + ts_10us. Absolute zero is
    // therefore not a no-return marker in the Robin W ROS contract. Preserve
    // its count for a useful rejection log, but never let it into deskew.
    if (point_s == 0.0) {
      ++range.zero_timestamp_count;
      continue;
    }
    // A NON-zero value outside the epoch band is a different failure: it means
    // the field is not absolute Unix seconds at all. That IS fatal, because
    // every downstream consumer would misread the whole cloud.
    if (!std::isfinite(point_s) || point_s < kMinEpochSeconds ||
        point_s > kMaxEpochSeconds) {
      return SeyondPointTimeRange{};
    }
    range.min_s = std::min(range.min_s, point_s);
    range.max_s = std::max(range.max_s, point_s);
    ++range.count;
  }
  if (range.count == 0) {
    // Every point had an invalid zero timestamp: there is no time axis.
    // as invalid but preserve the count so the caller can say WHY.
    range.min_s = 0.0;
    range.max_s = 0.0;
    range.valid = false;
    return range;
  }
  range.valid = true;
  return range;
}

// `frame_duration_s` is the deployment's configured frame period. It must
// match the rate the sensor is actually running at: a value larger than the
// true period makes every bound proportionally loose.
inline bool seyondCloudTimeContractValid(
    const sensor_msgs::msg::PointCloud2& msg,
    double frame_duration_s = kSeyondFrameDurationSecondsDefault,
    SeyondPointTimeRange* decoded_range = nullptr) {
  if (decoded_range) *decoded_range = SeyondPointTimeRange{};
  // Fail closed on a nonsense frame period rather than deriving nonsense bounds.
  if (!std::isfinite(frame_duration_s) || frame_duration_s <= 0.0) return false;

  const SeyondPointTimeRange range = decodeSeyondPointTimeRange(msg);
  if (decoded_range) *decoded_range = range;
  if (!range.valid || range.zero_timestamp_count != 0) return false;

  const double header_s =
      static_cast<double>(msg.header.stamp.sec) +
      static_cast<double>(msg.header.stamp.nanosec) * 1.0e-9;
  if (!std::isfinite(header_s)) return false;

  const double span_s = range.max_s - range.min_s;
  return range.min_s >= header_s - kSeyondFrameTimeToleranceSeconds &&
         range.max_s <=
             header_s + frame_duration_s + kSeyondFrameTimeToleranceSeconds &&
         span_s <= frame_duration_s + kSeyondFrameTimeToleranceSeconds;
}

inline double pointTimeEndpointDelta(
    const SeyondPointTimeRange& lhs,
    const SeyondPointTimeRange& rhs) {
  if (!lhs.valid || !rhs.valid) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(
      std::abs(lhs.min_s - rhs.min_s),
      std::abs(lhs.max_s - rhs.max_s));
}

}  // namespace gicp_localization
