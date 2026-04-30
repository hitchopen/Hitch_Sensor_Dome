#pragma once

#include <any>
#include <atomic>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Hitch Sensor Dome fork: external INS pose subscription with RTK-fix
  // gating. A pose is forwarded to set_init_state() only when:
  //   1. A NavSatFix on ins_fix_topic confirms RTK-class status
  //      (status.status >= GBAS_FIX) — unless ins_require_rtk_fixed is
  //      relaxed.
  //   2. position_covariance diagonal stddev <= ins_max_position_stddev
  //      (default 10 cm, which only RTK-fixed solutions reliably hit).
  //   3. The most recent N PoseStamped/Odometry messages are mutually
  //      consistent (translation drift < ins_max_pose_jitter, orientation
  //      |q1·q2| > 0.999), guarding against IMU-only dead-reckoning.
  // If the gate doesn't pass within ins_init_timeout_s, a bold RED
  // warning is printed periodically so the operator can decide whether
  // to wait, relax thresholds, or abort.
  void ins_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void ins_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void ins_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void ins_init_timeout_tick();

  // Hitch Sensor Dome fork — RTK-gated GNSS factor bridge.
  //
  // After init, every incoming /pose (or /odom) is also evaluated against
  // the most-recent /gps/fix; if the fix shows RTK-class status with
  // covariance below gnss_factor_max_position_stddev, the pose is
  // republished as a geometry_msgs/PoseWithCovarianceStamped on
  // gnss_factor_topic (default /gnss/pose_rtk_only). The
  // libgnss_global.so extension module subscribes to that topic and adds
  // soft prior factors to the global graph — exclusively from RTK-fixed
  // periods. During RTK-float / no-fix periods, no factor is published
  // and the optimizer relies on its own LiDAR cost.
  void try_publish_gnss_factor(
    const Eigen::Isometry3d& T_world_imu,
    const std::array<double, 36>& pose_cov,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool dump_on_unload;

  std::string intensity_field, ring_field;
  bool flip_points_y;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;

  // Hitch Sensor Dome fork: subscriptions for the external INS prior.
  // ins_pose_sub takes geometry_msgs/PoseStamped; ins_odom_sub takes
  // nav_msgs/Odometry (which carries linear velocity in twist.linear).
  // ins_fix_sub takes sensor_msgs/NavSatFix and is consulted as the
  // gating signal for RTK-fixed status. The first pose that passes the
  // gate drives set_init_state(); all three subscriptions are then
  // released and ins_init_applied flips to true.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ins_pose_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ins_fix_sub;
  rclcpp::TimerBase::SharedPtr ins_init_timeout_timer;
  std::atomic_bool ins_init_applied{false};

  // Gate state — accessed from callbacks (single executor thread; no
  // explicit mutex needed if rclcpp uses MultiThreadedExecutor it would,
  // but the default SingleThreadedExecutor serializes callbacks).
  sensor_msgs::msg::NavSatFix::SharedPtr last_fix_;
  std::deque<std::pair<Eigen::Isometry3d, Eigen::Vector3d>> pose_window_;
  rclcpp::Time ins_wait_started_;
  rclcpp::Time ins_last_warn_;
  std::string ins_last_reject_reason_;

  // Init gate thresholds (loaded from ROS params in the constructor).
  bool ins_require_rtk_fixed_ = true;
  double ins_max_position_stddev_ = 0.10;        // metres
  int ins_min_pose_window_samples_ = 10;
  double ins_max_pose_jitter_trans_ = 0.05;      // metres
  double ins_min_quat_dot_ = 0.999;              // |q1·q2|
  double ins_init_timeout_s_ = 60.0;

  // GNSS factor bridge — separate gate parameters so the per-factor
  // policy can be tuned independently of the one-shot init policy.
  // Defaults match the init gate (i.e., RTK-fixed only).
  bool gnss_factor_require_rtk_fixed_ = true;
  double gnss_factor_max_position_stddev_ = 0.10;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_pub_;
  std::atomic<int> gnss_factors_published_{0};
  std::atomic<int> gnss_factors_rejected_{0};
  rclcpp::TimerBase::SharedPtr gnss_factor_log_timer_;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
