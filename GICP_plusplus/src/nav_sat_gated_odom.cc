// =====================================================================
// nav_sat_gated_odom — Hitch Sensor Dome
//
// Republishes an INS Odometry topic ONLY when the most recent NavSatFix
// reports RTK-fixed status with cm-grade covariance. Silently drops
// messages during RTK-float / no-fix / tunnel periods.
//
// The Atlas Duo INS publishes /odom continuously regardless of RTK
// quality, so any downstream consumer that needs "trustworthy" GT pose
// needs an explicit gate. The gicp_localization node consumes
// gt_odom on the assumption that arrival ⇒ RTK-fixed; this node makes
// that assumption true on the Hitch dome.
//
// Defaults mirror GLIM_plusplus's RTK-gated factor bridge so the two
// pipelines reject and accept on the same criteria.
//
// Topic remaps:
//   odom  (in)  — nav_msgs/Odometry, default subscribes to /odom
//   fix   (in)  — sensor_msgs/NavSatFix, default subscribes to /gps/fix
//   odom_rtk_only (out) — nav_msgs/Odometry, defaults to /odom_rtk_only
//
// Parameters:
//   require_rtk_fixed   (bool, default true)   require status >= STATUS_GBAS_FIX
//   max_position_stddev (double, default 0.10) max σ on position covariance, m
//   max_fix_age_s       (double, default 0.5)  treat NavSatFix older than this as stale
//   report_interval_s   (double, default 10.0) per-counter log cadence; <=0 disables
// =====================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace {

// Position-σ from the upper-left 3×3 of a 9-element NavSatFix
// position_covariance. Returns the square root of the average of the
// diagonal — a single scalar that's directly comparable to the
// max_position_stddev threshold.
double position_stddev(const std::array<double, 9>& cov)
{
  const double s2 = (cov[0] + cov[4] + cov[8]) / 3.0;
  return s2 > 0.0 ? std::sqrt(s2) : 0.0;
}

const char* fix_status_name(int s)
{
  switch (s) {
    case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:   return "NO_FIX";
    case sensor_msgs::msg::NavSatStatus::STATUS_FIX:      return "FIX (single-point)";
    case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX: return "SBAS_FIX";
    case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX: return "GBAS_FIX (RTK-class)";
    default:                                              return "unknown";
  }
}

}  // namespace

class NavSatGatedOdom : public rclcpp::Node
{
public:
  NavSatGatedOdom()
  : rclcpp::Node("nav_sat_gated_odom")
  {
    this->declare_parameter<bool>("require_rtk_fixed", true);
    this->declare_parameter<double>("max_position_stddev", 0.10);
    this->declare_parameter<double>("max_fix_age_s", 0.5);
    this->declare_parameter<double>("report_interval_s", 10.0);

    this->get_parameter("require_rtk_fixed", require_rtk_fixed_);
    this->get_parameter("max_position_stddev", max_position_stddev_);
    this->get_parameter("max_fix_age_s", max_fix_age_s_);
    double report_interval_s = 10.0;
    this->get_parameter("report_interval_s", report_interval_s);

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
                "max_position_stddev=%.3f m, max_fix_age=%.2f s",
                require_rtk_fixed_ ? "true" : "false",
                max_position_stddev_, max_fix_age_s_);

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

    // Gate 2 — fix freshness. A stale fix means the quality signal we
    // have is no longer trustworthy; treat as unfit.
    const double odom_t = rclcpp::Time(msg->header.stamp).seconds();
    const double fix_t  = rclcpp::Time(last_fix_->header.stamp).seconds();
    if (max_fix_age_s_ > 0.0 && std::isfinite(odom_t) && std::isfinite(fix_t)
        && (odom_t - fix_t) > max_fix_age_s_) {
      ++rejected_stale_;
      return;
    }

    // Gate 3 — RTK-class status (GBAS_FIX or better).
    const int status = last_fix_->status.status;
    if (require_rtk_fixed_ &&
        status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      last_reject_reason_ = std::string("status=") + fix_status_name(status);
      ++rejected_status_;
      return;
    }

    // Gate 4 — position covariance. RTK-fixed reliably produces σ < 0.05 m;
    // a wide σ alongside a STATUS_GBAS_FIX usually means the receiver
    // dropped lock mid-publish.
    const double pos_sigma = position_stddev(last_fix_->position_covariance);
    if (max_position_stddev_ > 0.0 && pos_sigma > max_position_stddev_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "pos σ=%.3f m > %.3f m",
                    pos_sigma, max_position_stddev_);
      last_reject_reason_ = buf;
      ++rejected_cov_;
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
    const int total_r = r_nofix + r_stale + r_stat + r_cov;

    if (p == 0 && total_r == 0) return;  // quiet network — say nothing

    RCLCPP_INFO(
      this->get_logger(),
      "interval: published=%d  rejected=%d "
      "(no_fix=%d stale=%d status=%d cov=%d)  last_reject=\"%s\"",
      p, total_r, r_nofix, r_stale, r_stat, r_cov,
      last_reject_reason_.c_str());
  }

  // Parameters
  bool require_rtk_fixed_{true};
  double max_position_stddev_{0.10};
  double max_fix_age_s_{0.5};

  // Live state
  sensor_msgs::msg::NavSatFix::SharedPtr last_fix_;
  std::string last_reject_reason_ = "(none)";

  // Counters (cleared each report_tick)
  std::atomic<int> published_{0};
  std::atomic<int> rejected_no_fix_{0};
  std::atomic<int> rejected_stale_{0};
  std::atomic<int> rejected_status_{0};
  std::atomic<int> rejected_cov_{0};

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
