#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace gicp_localization {

enum class FixGateFailure {
  NONE,
  NO_FIX,
  STATUS,
  COVARIANCE,
  STALE,
  FUTURE,
  INVALID,
};

struct FixGateVerdict {
  bool accepted = false;
  FixGateFailure failure = FixGateFailure::INVALID;
  double age_s = std::numeric_limits<double>::quiet_NaN();
  double max_position_stddev_m =
      std::numeric_limits<double>::quiet_NaN();
  std::string reason;
};

inline FixGateVerdict validateNavSatFix(
    const sensor_msgs::msg::NavSatFix& fix,
    double consumer_stamp_s,
    bool require_rtk_class,
    double max_position_stddev_m,
    double max_fix_age_s,
    double future_fix_tolerance_s) {
  FixGateVerdict result;

  if (!std::isfinite(max_position_stddev_m) ||
      max_position_stddev_m <= 0.0 ||
      !std::isfinite(max_fix_age_s) || max_fix_age_s <= 0.0 ||
      !std::isfinite(future_fix_tolerance_s) ||
      future_fix_tolerance_s < 0.0) {
    result.reason = "invalid NavSatFix gate thresholds";
    return result;
  }

  if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      !std::isfinite(fix.altitude) || fix.latitude < -90.0 ||
      fix.latitude > 90.0 || fix.longitude < -180.0 ||
      fix.longitude > 180.0 || !std::isfinite(consumer_stamp_s)) {
    result.reason = "invalid position or consumer timestamp";
    return result;
  }

  if (fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    result.failure = FixGateFailure::NO_FIX;
    result.reason = "status=NO_FIX";
    return result;
  }
  if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      fix.status.status > sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    result.failure = FixGateFailure::STATUS;
    result.reason = "status value is outside REP-145";
    return result;
  }
  if (require_rtk_class &&
      fix.status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    result.failure = FixGateFailure::STATUS;
    result.reason = "status is not GBAS/RTK-class";
    return result;
  }

  if (fix.position_covariance_type ==
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    result.failure = FixGateFailure::COVARIANCE;
    result.reason = "position covariance type is UNKNOWN";
    return result;
  }

  double max_variance = 0.0;
  for (const size_t index : {size_t{0}, size_t{4}, size_t{8}}) {
    const double variance = fix.position_covariance[index];
    if (!std::isfinite(variance) || variance <= 0.0) {
      result.failure = FixGateFailure::COVARIANCE;
      result.reason = "position covariance diagonal is non-finite or non-positive";
      return result;
    }
    max_variance = std::max(max_variance, variance);
  }
  result.max_position_stddev_m = std::sqrt(max_variance);
  if (result.max_position_stddev_m > max_position_stddev_m) {
    result.failure = FixGateFailure::COVARIANCE;
    result.reason = "position covariance exceeds limit";
    return result;
  }

  const double fix_stamp_s =
      static_cast<double>(fix.header.stamp.sec) +
      static_cast<double>(fix.header.stamp.nanosec) * 1.0e-9;
  if (!std::isfinite(fix_stamp_s)) {
    result.reason = "fix timestamp is non-finite";
    return result;
  }
  result.age_s = consumer_stamp_s - fix_stamp_s;
  if (result.age_s < -future_fix_tolerance_s) {
    result.failure = FixGateFailure::FUTURE;
    result.reason = "fix timestamp is in the future";
    return result;
  }
  if (result.age_s > max_fix_age_s) {
    result.failure = FixGateFailure::STALE;
    result.reason = "fix timestamp is stale";
    return result;
  }

  result.accepted = true;
  result.failure = FixGateFailure::NONE;
  result.reason = "accepted";
  return result;
}

}  // namespace gicp_localization
