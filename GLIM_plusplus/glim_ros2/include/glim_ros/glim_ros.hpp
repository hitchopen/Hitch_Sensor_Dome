#pragma once

#include <any>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

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

#include <glim_ros/lidar_concat.hpp>

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
  // False when any quality/safety extension (for example the GNSS anchor
  // divergence gate) has rejected the run.
  bool ok() const;
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
  // the most-recent NavSatFix on ins_fix_topic; if it shows RTK-class status with
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
  // `epoch_anchor_count` (-1 = single sensor / unused) is the primary scan's
  // point count in a concatenated multi-LiDAR cloud; forwarded to
  // extract_raw_points() so the epoch-axis rebase anchors on the primary scan's
  // earliest time rather than the global merged minimum. See points_callback_live().
  // `ingested` (optional): set true only when the cloud passed extraction and
  // TimeKeeper validation and was inserted into odometry estimation; false when
  // the frame was skipped. Offline readers use it for primary accounting.
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, int epoch_anchor_count = -1, bool* ingested = nullptr);

  // Live subscription entry point for the primary LiDAR. Online mapping with
  // concatenation is rejected until this path has a future-sweep release queue;
  // single-LiDAR online mapping forwards directly to points_callback().
  void points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Buffers an auxiliary LiDAR cloud for later time-matched merging.
  void aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

  // True only when the live subscription-based mapping passway is enabled
  // (glim_ros/enable_online_mapping). Default false: GLIM maps offline only.
  bool online_mapping_enabled() const { return online_mapping_enabled_; }

  // Topics for the INS init gate / GNSS factor bridge, parsed from
  // config_ros.json in the constructor (both modes). Exposed so the offline
  // drivers (glim_rosbag / glim_pcap_rosbag) can pull the same topics from
  // the bag and feed ins_*_callback() directly — live subscriptions to them
  // only exist when online mapping is enabled.
  const std::string& ins_pose_topic() const { return ins_pose_topic_; }
  const std::string& ins_odom_topic() const { return ins_odom_topic_; }
  const std::string& ins_fix_topic() const { return ins_fix_topic_; }
  const std::string& gnss_factor_topic() const { return gnss_factor_topic_; }

  // --- Operational state -------------------------------------------------
  // SLAM STARTS ONLY AT RTK-FIXED: the map origin must be globally
  // referenced and the INS attitude validated before any factor is built,
  // so initialization waits for a fresh, validated RTK-fixed solution.
  //
  // AFTER that, LiDAR-IMU SLAM is the primary estimator and runs
  // continuously; GNSS is an optional global constraint that comes and
  // goes with RTK availability. These report what the finished map
  // actually got, so an operator (or CI) accepts a weakly-constrained map
  // deliberately rather than by accident.
  bool slam_initialized() const { return ins_init_applied.load(); }
  // Number of GNSS position factors handed to the global graph.
  int gnss_factors_published() const { return gnss_factors_published_.load(); }
  // End-of-run map state:
  //   "uninitialized"    — RTK-fixed never validated; NO map was built.
  //   "rtk_origin_only"  — origin anchored at RTK-fixed, but RTK never
  //                        returned afterwards, so the trajectory past t0
  //                        is constrained by LiDAR-IMU alone.
  //   "rtk_anchored"     — origin anchored AND ongoing GNSS constraints.
  const char* map_anchor_state() const {
    if (!ins_init_applied.load()) return "uninitialized";
    return gnss_factors_published_.load() > 0 ? "rtk_anchored" : "rtk_origin_only";
  }
  // Operator policy: fail the run when the map came out local_only.
  bool require_rtk_anchor() const { return require_rtk_anchor_; }
  // One-line end-of-run verdict; also logged by save().
  void log_map_status() const;

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
  // Fixed input-vector calibration. Both acceleration and gyro are rotated
  // into the IMU frame used by T_lidar_imu before entering any estimator.
  Eigen::Quaterniond imu_input_rotation = Eigen::Quaterniond::Identity();
  bool dump_on_unload;

  std::string intensity_field, ring_field;
  std::string expected_time_field;
  int expected_time_datatype = 0;
  bool expected_time_is_absolute = false;
  bool float64_time_is_epoch_ns = false;  // [P2 FIX 2026-07-15] Luminar FLOAT64-epoch-ns opt-in
  bool flip_points_y;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  bool online_mapping_enabled_ = false;
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
  // Topic names (config_ros.json, "glim_ros" section) — parsed in both
  // modes; see the public accessors above.
  std::string ins_pose_topic_;
  std::string ins_odom_topic_;
  std::string ins_fix_topic_;
  std::string gnss_factor_topic_;
  rclcpp::TimerBase::SharedPtr ins_init_timeout_timer;
  std::atomic_bool ins_init_applied{false};

  // Gate state — accessed from callbacks (single executor thread; no
  // explicit mutex needed if rclcpp uses MultiThreadedExecutor it would,
  // but the default SingleThreadedExecutor serializes callbacks).
  sensor_msgs::msg::NavSatFix::SharedPtr last_fix_;
  // Timestamped INS solution window backing the stability gate.
  // The stamp is REQUIRED: without it the gate can neither enforce temporal
  // contiguity (a fix that flickers once every N seconds would otherwise
  // accumulate a "stable" window out of unrelated moments) nor form the
  // constant-velocity prediction that lets it accept a MOVING start.
  // See ins_window_push_and_check().
  struct InsSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp = 0.0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  };
  std::deque<InsSample, Eigen::aligned_allocator<InsSample>> pose_window_;
  rclcpp::Time ins_wait_started_;
  rclcpp::Time ins_last_warn_;
  std::string ins_last_reject_reason_;
  // The last SUBSTANTIVE rejection (residual / contiguity / monotonicity),
  // kept separately: after such a rejection the window restarts, so the
  // transient "accumulating" message would otherwise overwrite the real
  // cause before any diagnostic gets to print it.
  std::string ins_last_hard_reject_;
  uint64_t ins_hard_reject_count_ = 0;

  // Push one INS solution into the stability window and report whether the
  // window now demonstrates a settled, self-consistent P1 solution.
  //
  // The Atlas Duo is a TIGHTLY-COUPLED INS: attitude/velocity estimation and
  // its own calibration are P1's responsibility, not GLIM's. So this gate
  // tests that P1's output is SMOOTH AND CONTIGUOUS — it deliberately does
  // NOT test that the vehicle is stationary, which would forbid the
  // moving-start case this fork exists to support.
  //
  // On success, *v_est (when non-null) receives a world-frame velocity
  // estimated from the window by finite difference, for callers whose
  // message type carries no velocity (PoseStamped). Passing a real velocity
  // matters: the optimizer's V(0) prior is only σ = 1 m/s, so seeding zero
  // on a rolling start would be a many-sigma error on the very interval that
  // anchors the global graph.
  bool ins_window_push_and_check(double stamp,
                                 const Eigen::Isometry3d& T,
                                 Eigen::Vector3d* v_est);

  // Init gate thresholds (loaded from ROS params in the constructor).
  bool ins_require_rtk_fixed_ = true;
  double ins_max_position_stddev_ = 0.10;        // metres
  int ins_min_pose_window_samples_ = 10;
  // Max residual (m) of an INS position against a constant-velocity
  // prediction from the two preceding samples — a SMOOTHNESS bound, not a
  // displacement bound. Under constant acceleration a over sample spacing
  // dt the residual is a·dt², so 0.05 m at 10 Hz tolerates ≈5 m/s² (0.5 g)
  // of genuine vehicle dynamics before the solution is called unsettled.
  // Raise it if mapping runs begin under harder acceleration.
  double ins_max_pose_jitter_trans_ = 0.05;      // metres (residual)
  // Same idea for orientation: |q_pred · q| against a constant-angular-rate
  // prediction. 0.999 ≈ 2.5° of residual, i.e. ≈250°/s² of angular
  // acceleration at 10 Hz — a glitch bound, not a turn-rate bound.
  double ins_min_quat_dot_ = 0.999;              // |q_pred·q|
  // Max gap (s) between consecutive accepted INS samples before the window
  // is discarded as non-contiguous. Samples enter the window only after the
  // RTK gates pass, so without this bound a flickering fix could assemble a
  // "stable" window from moments minutes apart.
  double ins_max_pose_gap_s_ = 0.5;              // seconds
  double ins_init_timeout_s_ = 60.0;
  // Max age (s) of the NavSatFix backing a gating decision, measured
  // between MESSAGE header stamps (never wall time — offline replay runs
  // at arbitrary speed). A small negative allowance absorbs benign
  // publisher-side stamp ordering.
  double fix_max_age_s_ = 0.5;
  double fix_future_tolerance_s_ = 0.05;
  // Operator policy: exit non-zero unless the finished map is
  // "rtk_anchored". Default false — a map with an RTK-anchored origin but
  // no ongoing GNSS constraints is a legitimate outcome (RTK never came
  // back), and only the operator knows whether it is acceptable.
  bool require_rtk_anchor_ = false;
  // Last bridge refusal reason, so a persistent cause is logged once
  // rather than per message.
  std::string gnss_factor_last_reject_;

  // GNSS factor bridge — separate gate parameters so the per-factor
  // policy can be tuned independently of the one-shot init policy.
  // Defaults match the init gate (i.e., RTK-fixed only).
  bool gnss_factor_require_rtk_fixed_ = true;
  double gnss_factor_max_position_stddev_ = 0.10;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_pub_;
  std::atomic<int> gnss_factors_published_{0};
  std::atomic<int> gnss_factors_rejected_{0};
  rclcpp::TimerBase::SharedPtr gnss_factor_log_timer_;

  // Dual-antenna RTK heading (auto-detected from sensor_dome_tf.yaml at
  // launch time, passed in as ROS parameters). When enabled, the
  // wrapper:
  //   1. tightens the init gate thresholds — orientation locks faster
  //      and more precisely with dual-antenna heading;
  //   2. populates the orientation covariance on every factor-bridge
  //      message with a tight yaw σ derived from baseline length, so
  //      any downstream heading-aware factor module can use the
  //      RTK-derived heading directly to correct IMU yaw drift.
  bool dual_antenna_enabled_ = false;
  double dual_antenna_baseline_m_ = 0.0;
  double dual_antenna_heading_sigma_rad_ = 0.0;

  // Runtime yaw σ sanity check — see try_publish_gnss_factor in
  // glim_ros.cpp for the rationale. We compare the Atlas-reported yaw
  // covariance against dual_antenna_heading_sigma_rad_ and warn once
  // per session if the operator believes we're dual-antenna but the
  // Atlas firmware appears not to have dual-antenna heading active.
  // Defaults are the threshold multiplier (5×), the number of samples
  // required before we evaluate (20), and the violation fraction
  // required (≥75 % of samples must exceed the threshold).
  double yaw_sigma_warn_threshold_mult_ = 5.0;
  int yaw_sigma_check_window_samples_ = 20;
  double yaw_sigma_violation_fraction_ = 0.75;
  std::atomic<int> yaw_sigma_samples_{0};
  std::atomic<int> yaw_sigma_violations_{0};
  std::atomic_bool yaw_sigma_warned_{false};

  // Multi-LiDAR concatenation config and reserved live-path state. Online
  // mapping with concat is rejected until a future-sweep release queue exists.
  glim_ros::AuxConcatConfig aux_concat;
  std::mutex aux_buffers_mutex;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> aux_points_subs;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
