// =====================================================================
// nav_sat_gated_odom — Hitch Sensor Dome
//
// Republishes an INS Odometry topic ONLY when the most recent NavSatFix
// reports RTK-class status with valid, cm-grade covariance. This is a legacy
// compatibility bridge: REP-145 status cannot prove fixed vs float, so the
// adapter's solution-type-gated fixed-only odometry is preferred.
//
// The Atlas Duo INS publishes /odom continuously regardless of RTK
// quality, so any downstream consumer that needs "trustworthy" GT pose
// needs an explicit gate. gicp_localization rechecks finite positive
// covariance, but this compatibility node remains only an RTK-class
// discriminator because NavSatFix does not carry the native solution type.
//
// Defaults mirror GLIM_plusplus's RTK-gated factor bridge so the two
// pipelines reject and accept on the same criteria.
//
// Topic remaps:
//   odom  (in)  — nav_msgs/Odometry; launch default /gps_p1/filtered_odom
//   fix   (in)  — sensor_msgs/NavSatFix; launch default /gps_p1/fix
//   odom_rtk_only (out) — nav_msgs/Odometry; remapped to gt_odom_topic
//
// Parameters:
//   require_rtk_fixed   (bool, default true)   require STATUS_GBAS_FIX
//   max_position_stddev (double, default 0.10) max σ on position covariance, m
//   max_fix_age_s       (double, default 0.5)  treat NavSatFix older than this as stale
//   future_fix_tolerance_s (double, default 0.05) allowed timestamp lead
//   report_interval_s   (double, default 10.0) per-counter log cadence; <=0 disables
// =====================================================================

#include "gicp_localization/navsat_fix_gate.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class NavSatGatedOdom : public rclcpp::Node
{
public:
  NavSatGatedOdom()
  : rclcpp::Node("nav_sat_gated_odom")
  {
    this->declare_parameter<bool>("require_rtk_fixed", true);
    this->declare_parameter<double>("max_position_stddev", 0.10);
    this->declare_parameter<double>("max_fix_age_s", 0.5);
    this->declare_parameter<double>("future_fix_tolerance_s", 0.05);
    this->declare_parameter<double>("report_interval_s", 10.0);

    this->get_parameter("require_rtk_fixed", require_rtk_fixed_);
    this->get_parameter("max_position_stddev", max_position_stddev_);
    this->get_parameter("max_fix_age_s", max_fix_age_s_);
    this->get_parameter("future_fix_tolerance_s", future_fix_tolerance_s_);
    double report_interval_s = 10.0;
    this->get_parameter("report_interval_s", report_interval_s);
    if (!std::isfinite(max_position_stddev_) ||
        max_position_stddev_ <= 0.0 ||
        !std::isfinite(max_fix_age_s_) || max_fix_age_s_ <= 0.0 ||
        !std::isfinite(future_fix_tolerance_s_) ||
        future_fix_tolerance_s_ < 0.0 ||
        !std::isfinite(report_interval_s)) {
      throw std::invalid_argument(
        "RTK gate limits must be finite; sigma/fix age must be positive and "
        "future tolerance must be non-negative");
    }

    rclcpp::QoS qos(10);
    qos.best_effort();
    qos.durability_volatile();

    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", qos,
      std::bind(&NavSatGatedOdom::fix_cb, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos,
      std::bind(&NavSatGatedOdom::odom_cb, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_rtk_only", 10);

    RCLCPP_INFO(this->get_logger(),
                "nav_sat_gated_odom up — require_rtk_fixed=%s, "
                "max_position_stddev=%.3f m, max_fix_age=%.2f s, "
                "future_fix_tolerance=%.3f s",
                require_rtk_fixed_ ? "true" : "false",
                max_position_stddev_, max_fix_age_s_,
                future_fix_tolerance_s_);

    if (report_interval_s > 0.0) {
      const auto period = std::chrono::milliseconds(
        static_cast<int>(report_interval_s * 1000));
      report_timer_ = this->create_wall_timer(
        period, std::bind(&NavSatGatedOdom::report_tick, this));
    }
  }

private:
  void fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_fix_ = msg;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Gate 1 — must have a NavSatFix to consult.
    if (!last_fix_) {
      ++rejected_no_fix_;
      return;
    }

    const double odom_t = rclcpp::Time(msg->header.stamp).seconds();
    const auto verdict = gicp_localization::validateNavSatFix(
      *last_fix_, odom_t, require_rtk_fixed_, max_position_stddev_,
      max_fix_age_s_, future_fix_tolerance_s_);
    if (!verdict.accepted) {
      last_reject_reason_ = verdict.reason;
      switch (verdict.failure) {
        case gicp_localization::FixGateFailure::NO_FIX:
          ++rejected_no_fix_;
          break;
        case gicp_localization::FixGateFailure::STATUS:
          ++rejected_status_;
          break;
        case gicp_localization::FixGateFailure::COVARIANCE:
          ++rejected_cov_;
          break;
        case gicp_localization::FixGateFailure::STALE:
          ++rejected_stale_;
          break;
        case gicp_localization::FixGateFailure::FUTURE:
          ++rejected_future_;
          break;
        default:
          ++rejected_invalid_;
          break;
      }
      return;
    }

    // All gates passed — forward the message unchanged.
    odom_pub_->publish(*msg);
    ++published_;
  }

  void report_tick()
  {
    const int p = published_.exchange(0);
    const int r_nofix = rejected_no_fix_.exchange(0);
    const int r_stale = rejected_stale_.exchange(0);
    const int r_stat  = rejected_status_.exchange(0);
    const int r_cov   = rejected_cov_.exchange(0);
    const int r_future = rejected_future_.exchange(0);
    const int r_invalid = rejected_invalid_.exchange(0);
    const int total_r =
      r_nofix + r_stale + r_stat + r_cov + r_future + r_invalid;

    if (p == 0 && total_r == 0) return;  // quiet network — say nothing

    RCLCPP_INFO(
      this->get_logger(),
      "interval: published=%d  rejected=%d "
      "(no_fix=%d stale=%d future=%d status=%d cov=%d invalid=%d) "
      "last_reject=\"%s\"",
      p, total_r, r_nofix, r_stale, r_future, r_stat, r_cov, r_invalid,
      last_reject_reason_.c_str());
  }

  // Parameters
  bool require_rtk_fixed_{true};
  double max_position_stddev_{0.10};
  double max_fix_age_s_{0.5};
  double future_fix_tolerance_s_{0.05};

  // Live state
  sensor_msgs::msg::NavSatFix::SharedPtr last_fix_;
  std::string last_reject_reason_ = "(none)";

  // Counters (cleared each report_tick)
  std::atomic<int> published_{0};
  std::atomic<int> rejected_no_fix_{0};
  std::atomic<int> rejected_stale_{0};
  std::atomic<int> rejected_status_{0};
  std::atomic<int> rejected_cov_{0};
  std::atomic<int> rejected_future_{0};
  std::atomic<int> rejected_invalid_{0};

  // ROS plumbing
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatGatedOdom>());
  rclcpp::shutdown();
  return 0;
}
