#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <functional>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>
#include <glim_ros/urdf_transforms.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(get_ringbuffer_sink());
  spdlog::set_default_logger(logger);

  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_log.log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    print_system_info(logger);
  }

  dump_on_unload = false;
  this->declare_parameter<bool>("dump_on_unload", false);
  this->get_parameter<bool>("dump_on_unload", dump_on_unload);

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  keep_raw_points = config_ros.param<bool>("glim_ros", "keep_raw_points", false);
  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  points_time_offset = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 0.0);

  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = config_sensors.param<std::string>("sensors", "intensity_field", "intensity");
  ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");
  flip_points_y = config_sensors.param<bool>("sensors", "flip_points_y", false);

  // Override T_lidar_imu from URDF if configured
  const std::string urdf_path = config_sensors.param<std::string>("sensors", "urdf_path", "");
  const std::string urdf_lidar_frame = config_sensors.param<std::string>("sensors", "urdf_lidar_frame", "");
  const std::string urdf_imu_frame = config_sensors.param<std::string>("sensors", "urdf_imu_frame", "");
  if (!urdf_path.empty() && !urdf_lidar_frame.empty() && !urdf_imu_frame.empty()) {
    try {
      auto urdf_transforms = glim::parse_urdf_transforms(urdf_path);
      Eigen::Isometry3d T_lidar_imu = glim::compute_transform(urdf_transforms, urdf_lidar_frame, urdf_imu_frame);
      std::stringstream ss;
      ss << T_lidar_imu.matrix();
      logger->info("URDF override T_lidar_imu ({} -> {}):\n{}", urdf_lidar_frame, urdf_imu_frame, ss.str());

      // Write override into config_sensors.json so all modules pick it up
      const std::string config_sensors_path = glim::GlobalConfig::get_config_path("config_sensors");
      config_sensors.override_param<Eigen::Isometry3d>("sensors", "T_lidar_imu", T_lidar_imu);
      config_sensors.save(config_sensors_path);
    } catch (const std::exception& e) {
      logger->error("Failed to compute T_lidar_imu from URDF: {}", e.what());
    }
  }

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
  spdlog::info("load {}", odometry_estimation_so_name);

  std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
  if (!odom) {
    spdlog::critical("failed to load odometry estimation module");
    abort();
  }
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));

  // Sub mapping
  if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
    const std::string sub_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
    if (!sub_mapping_so_name.empty()) {
      spdlog::info("load {}", sub_mapping_so_name);
      auto sub = SubMappingBase::load_module(sub_mapping_so_name);
      if (sub) {
        sub_mapping.reset(new AsyncSubMapping(sub));
      }
    }
  }

  // Global mapping
  if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
    const std::string global_mapping_so_name =
      glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
    if (!global_mapping_so_name.empty()) {
      spdlog::info("load {}", global_mapping_so_name);
      auto global = GlobalMappingBase::load_module(global_mapping_so_name);
      if (global) {
        global_mapping.reset(new AsyncGlobalMapping(global));
      }
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError& e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load_module(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  // ROS-related
  using std::placeholders::_1;
  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1));

  qos = get_qos_settings(config_ros, "glim_ros", "points_qos");
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, qos, std::bind(&GlimROS::points_callback, this, _1));
#ifdef BUILD_WITH_CV_BRIDGE
  qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
  image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

  // Hitch Sensor Dome fork: external INS init with RTK-fixed gating.
  //
  // GLIM's initial state is anchored to the Atlas Duo's INS pose, but
  // accepting that pose requires confirming the INS itself is reliable.
  // We subscribe to:
  //
  //   - ins_pose_topic (default /pose) — geometry_msgs/PoseStamped from
  //     fusion_engine_driver. Carries orientation + position; no velocity.
  //   - ins_odom_topic (default empty) — nav_msgs/Odometry, if a node
  //     republishes the Atlas Duo's full 6-DOF state.
  //   - ins_fix_topic  (default /gps/fix) — sensor_msgs/NavSatFix, the
  //     gate signal: status.status, position_covariance.
  //
  // Gate logic lives in ins_pose_callback / ins_odom_callback above. A
  // periodic timer (ins_init_timeout_tick) prints a bold RED warning
  // every 10 s while the gate is still blocking, naming the most-recent
  // rejection reason so the operator can act.
  const std::string ins_pose_topic =
    config_ros.param<std::string>("glim_ros", "ins_pose_topic", "/pose");
  const std::string ins_odom_topic =
    config_ros.param<std::string>("glim_ros", "ins_odom_topic", "");
  const std::string ins_fix_topic =
    config_ros.param<std::string>("glim_ros", "ins_fix_topic",  "/gps/fix");

  // Gate thresholds — overridable from config_ros.json.
  ins_require_rtk_fixed_       = config_ros.param<bool>(  "glim_ros", "ins_require_rtk_fixed",       true);
  ins_max_position_stddev_     = config_ros.param<double>("glim_ros", "ins_max_position_stddev",     0.10);
  ins_min_pose_window_samples_ = config_ros.param<int>(   "glim_ros", "ins_min_pose_window_samples", 10);
  ins_max_pose_jitter_trans_   = config_ros.param<double>("glim_ros", "ins_max_pose_jitter_trans",   0.05);
  ins_min_quat_dot_            = config_ros.param<double>("glim_ros", "ins_min_quat_dot",            0.999);
  ins_init_timeout_s_          = config_ros.param<double>("glim_ros", "ins_init_timeout_s",          60.0);

  rclcpp::QoS ins_qos(20);
  ins_qos.reliable();

  if (!ins_fix_topic.empty()) {
    ins_fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      ins_fix_topic, ins_qos,
      std::bind(&GlimROS::ins_fix_callback, this, _1));
    spdlog::info("Hitch fork: subscribed to NavSatFix topic '{}' (gate signal)", ins_fix_topic);
  } else {
    spdlog::warn("Hitch fork: ins_fix_topic is empty — RTK gate cannot run; INS pose will be rejected.");
  }
  if (!ins_pose_topic.empty()) {
    ins_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ins_pose_topic, ins_qos,
      std::bind(&GlimROS::ins_pose_callback, this, _1));
    spdlog::info("Hitch fork: subscribed to INS PoseStamped topic '{}'", ins_pose_topic);
  }
  if (!ins_odom_topic.empty()) {
    ins_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      ins_odom_topic, ins_qos,
      std::bind(&GlimROS::ins_odom_callback, this, _1));
    spdlog::info("Hitch fork: subscribed to INS Odometry topic '{}'", ins_odom_topic);
  }

  spdlog::info(
    "Hitch fork: RTK gate — require_rtk_fixed={}, max_pos_stddev={:.2f} m, "
    "window={} samples, max_jitter={:.3f} m, min_quat_dot={:.4f}, timeout={:.0f} s",
    ins_require_rtk_fixed_, ins_max_position_stddev_,
    ins_min_pose_window_samples_, ins_max_pose_jitter_trans_,
    ins_min_quat_dot_, ins_init_timeout_s_);

  ins_wait_started_ = this->now();
  ins_last_warn_    = ins_wait_started_;
  ins_init_timeout_timer = this->create_wall_timer(
    std::chrono::seconds(2),
    [this]() { ins_init_timeout_tick(); });

  // ---- GNSS factor bridge (post-init, RTK-gated republisher) ----
  // Reads the same NavSatFix subscription set up above (kept alive
  // post-init). Each accepted /pose / /odom is republished on
  // gnss_factor_topic as PoseWithCovarianceStamped, gated by RTK-class
  // status + covariance. libgnss_global.so subscribes to that topic
  // (configured in glim_ext/config/config_gnss_global.json) and turns
  // each message into a soft prior factor in the global graph.
  const std::string gnss_factor_topic =
    config_ros.param<std::string>("glim_ros", "gnss_factor_topic", "/gnss/pose_rtk_only");
  gnss_factor_require_rtk_fixed_ =
    config_ros.param<bool>(  "glim_ros", "gnss_factor_require_rtk_fixed",   true);
  gnss_factor_max_position_stddev_ =
    config_ros.param<double>("glim_ros", "gnss_factor_max_position_stddev", 0.10);

  if (!gnss_factor_topic.empty()) {
    gnss_pose_pub_ = this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(gnss_factor_topic, 50);
    spdlog::info(
      "Hitch fork: GNSS factor bridge — publishing on '{}' "
      "(require_rtk_fixed={}, max_pos_stddev={:.2f} m)",
      gnss_factor_topic, gnss_factor_require_rtk_fixed_,
      gnss_factor_max_position_stddev_);

    // Periodic status log — every 10 s reports accepted / rejected counts
    // so the operator can see whether the bridge is doing useful work
    // mid-session (e.g., expected to drop to zero in tunnels).
    gnss_factor_log_timer_ = this->create_wall_timer(
      std::chrono::seconds(10), [this]() {
        const int p = gnss_factors_published_.load();
        const int r = gnss_factors_rejected_.load();
        if (p == 0 && r == 0 && !ins_init_applied.load()) return;
        spdlog::info(
          "Hitch fork: GNSS factor bridge — {} published, {} rejected since start",
          p, r);
      });
  } else {
    spdlog::info("Hitch fork: gnss_factor_topic empty — bridge disabled");
  }

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");
  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/dump";
    wait(true);
    save(dump_path);
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (!GlobalConfig::instance()->has_param("meta", "imu_frame_id")) {
    spdlog::debug("auto-detecting IMU frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "imu_frame_id", msg->header.frame_id);
  }

  if (std::abs(acc_scale) < 1e-6) {
    const double norm = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z).norm();
    if (norm > 7.0 && norm < 12.0) {
      acc_scale = 1.0;
      spdlog::debug("assuming [m/s^2] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else if (norm > 0.8 && norm < 1.2) {
      acc_scale = 9.80665;
      spdlog::debug("assuming [g] for acceleration unit (acc_scale={}, norm={})", acc_scale, norm);
    } else {
      acc_scale = 1.0;
      spdlog::warn("unexpected acceleration norm {}. assuming [m/s^2] for acceleration unit (acc_scale={})", norm, acc_scale);
    }
  }

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if (!time_keeper->validate_imu_stamp(imu_stamp)) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

// =============================================================================
// Hitch Sensor Dome fork: RTK-fixed gating for INS-driven init
// =============================================================================
// A pose is forwarded to OdometryEstimationIMU::set_init_state() only when
// ALL the following are true:
//
//   (1) FIX QUALITY — the most recent NavSatFix on ins_fix_topic shows
//       status.status >= STATUS_GBAS_FIX (i.e., RTK-class). This is
//       relaxable via the ins_require_rtk_fixed parameter.
//
//   (2) COVARIANCE  — position_covariance diagonal stddev is below
//       ins_max_position_stddev (default 0.10 m). RTK-fixed solutions
//       reliably hit < 0.05 m; RTK-float typically sits at 0.1–0.5 m;
//       single-point GNSS sits at several metres.
//
//   (3) STABILITY   — the last ins_min_pose_window_samples PoseStamped /
//       Odometry messages are mutually consistent: pairwise translation
//       drift below ins_max_pose_jitter_trans, pairwise quaternion
//       |q1·q2| above ins_min_quat_dot. This rejects IMU-only
//       dead-reckoning that hasn't aligned to GNSS yet.
//
// Until the gate passes, ins_init_timeout_tick() prints a bold RED
// warning every 10 s explaining the most-recent rejection reason and
// what to do about it.
// =============================================================================

namespace {

constexpr const char* RED   = "\033[1;31m";
constexpr const char* CYAN  = "\033[1;36m";
constexpr const char* RESET = "\033[0m";

const char* fix_status_name(int s) {
  switch (s) {
    case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:   return "NO_FIX";
    case sensor_msgs::msg::NavSatStatus::STATUS_FIX:      return "FIX (single-point)";
    case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX: return "SBAS_FIX";
    case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX: return "GBAS_FIX (RTK-class)";
    default: return "UNKNOWN";
  }
}

double cov_diag_stddev(const std::array<double, 9>& C) {
  // Worst of the three position diagonals' stddev.
  double sx = std::sqrt(std::max(0.0, C[0]));
  double sy = std::sqrt(std::max(0.0, C[4]));
  double sz = std::sqrt(std::max(0.0, C[8]));
  return std::max({sx, sy, sz});
}

}  // anonymous namespace

void GlimROS::ins_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  // Track the most recent fix for the pose/odom callbacks to consult.
  // No gating decision happens here — the pose callbacks own that.
  last_fix_ = msg;
}

void GlimROS::ins_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Post-init: route the message to the RTK-gated factor bridge. The
  // bridge applies its own gate against last_fix_ and silently drops
  // poses outside RTK-fixed; gnss_global only sees factors from RTK
  // periods, suspending automatically through tunnels / urban canyons.
  if (ins_init_applied.load()) {
    Eigen::Quaterniond q(msg->pose.orientation.w,
                         msg->pose.orientation.x,
                         msg->pose.orientation.y,
                         msg->pose.orientation.z);
    if (std::abs(q.norm() - 1.0) > 1e-2) return;
    q.normalize();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    // PoseStamped has no covariance — let the bridge synthesize from
    // the NavSatFix covariance (gnss_global ignores covariance anyway,
    // but we propagate it for downstream tools / debugging).
    std::array<double, 36> cov{};
    try_publish_gnss_factor(T, cov, msg->header.stamp, msg->header.frame_id);
    return;
  }

  // ---- (1)+(2) Fix quality + covariance gate ----
  if (!last_fix_) {
    ins_last_reject_reason_ = "no NavSatFix received yet on ins_fix_topic";
    return;
  }
  const int fix_status = last_fix_->status.status;
  if (ins_require_rtk_fixed_ &&
      fix_status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    ins_last_reject_reason_ = std::string("fix status is '") + fix_status_name(fix_status) +
                              "' (need GBAS_FIX = RTK-class)";
    return;
  }
  const double pos_stddev = cov_diag_stddev(last_fix_->position_covariance);
  if (pos_stddev > ins_max_position_stddev_) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "position covariance stddev=%.3f m > %.3f m (RTK not yet fixed)",
                  pos_stddev, ins_max_position_stddev_);
    ins_last_reject_reason_ = buf;
    return;
  }

  // ---- Convert message ----
  Eigen::Quaterniond q(msg->pose.orientation.w,
                       msg->pose.orientation.x,
                       msg->pose.orientation.y,
                       msg->pose.orientation.z);
  if (std::abs(q.norm() - 1.0) > 1e-2) {
    ins_last_reject_reason_ = "non-unit quaternion (INS not calibrated)";
    return;
  }
  q.normalize();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  // ---- (3) Stability gate ----
  // PoseStamped has no velocity; assume zero (the optimizer refines it).
  pose_window_.emplace_back(T, Eigen::Vector3d::Zero());
  while (static_cast<int>(pose_window_.size()) > ins_min_pose_window_samples_) {
    pose_window_.pop_front();
  }
  if (static_cast<int>(pose_window_.size()) < ins_min_pose_window_samples_) {
    ins_last_reject_reason_ = "accumulating pose window";
    return;
  }
  // Pairwise consistency.
  for (size_t i = 1; i < pose_window_.size(); ++i) {
    const auto& a = pose_window_[i - 1].first;
    const auto& b = pose_window_[i    ].first;
    if ((a.translation() - b.translation()).norm() > ins_max_pose_jitter_trans_) {
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "pose translation jitter %.3f m > %.3f m (INS still settling)",
                    (a.translation() - b.translation()).norm(),
                    ins_max_pose_jitter_trans_);
      ins_last_reject_reason_ = buf;
      pose_window_.clear();  // start over once consistency breaks
      return;
    }
    Eigen::Quaterniond qa(a.linear()), qb(b.linear());
    if (std::abs(qa.dot(qb)) < ins_min_quat_dot_) {
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "pose orientation jitter |q1.q2|=%.4f < %.4f (INS still settling)",
                    std::abs(qa.dot(qb)), ins_min_quat_dot_);
      ins_last_reject_reason_ = buf;
      pose_window_.clear();
      return;
    }
  }

  // ---- All gates passed → apply ----
  if (odometry_estimation) {
    odometry_estimation->set_init_state(T, Eigen::Vector3d::Zero());
    ins_init_applied.store(true);
    spdlog::info(
      "{}Hitch fork: INS init pose ACCEPTED — fix={}, pos σ={:.3f} m, "
      "translation=[{:.3f}, {:.3f}, {:.3f}], qw={:.4f}{}",
      CYAN, fix_status_name(fix_status), pos_stddev,
      T.translation().x(), T.translation().y(), T.translation().z(),
      q.w(), RESET);
    // Note: subscriptions are KEPT ALIVE post-init so the factor bridge
    // can republish each subsequent /pose as PoseWithCovarianceStamped on
    // /gnss/pose_rtk_only (gated against the latest /gps/fix). The init
    // pose itself is not also republished as a factor — set_init_state
    // already pinned that information into the optimizer.
    if (ins_init_timeout_timer) ins_init_timeout_timer->cancel();
  }
}

void GlimROS::ins_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Mirror of ins_pose_callback, but also consumes linear velocity.
  // Post-init: route to the RTK-gated factor bridge (Odometry already
  // carries pose covariance; we propagate it to the published
  // PoseWithCovarianceStamped).
  if (ins_init_applied.load()) {
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    if (std::abs(q.norm() - 1.0) > 1e-2) return;
    q.normalize();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.toRotationMatrix();
    T.translation() << msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z;
    std::array<double, 36> cov{};
    for (int i = 0; i < 36; ++i) cov[i] = msg->pose.covariance[i];
    try_publish_gnss_factor(T, cov, msg->header.stamp, msg->header.frame_id);
    return;
  }

  // (1)+(2) Fix gate.
  if (!last_fix_) {
    ins_last_reject_reason_ = "no NavSatFix received yet on ins_fix_topic";
    return;
  }
  const int fix_status = last_fix_->status.status;
  if (ins_require_rtk_fixed_ &&
      fix_status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    ins_last_reject_reason_ = std::string("fix status is '") + fix_status_name(fix_status) +
                              "' (need GBAS_FIX = RTK-class)";
    return;
  }
  const double pos_stddev = cov_diag_stddev(last_fix_->position_covariance);
  if (pos_stddev > ins_max_position_stddev_) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "position covariance stddev=%.3f m > %.3f m (RTK not yet fixed)",
                  pos_stddev, ins_max_position_stddev_);
    ins_last_reject_reason_ = buf;
    return;
  }

  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                       msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z);
  if (std::abs(q.norm() - 1.0) > 1e-2) {
    ins_last_reject_reason_ = "non-unit quaternion (INS not calibrated)";
    return;
  }
  q.normalize();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;
  Eigen::Vector3d v(msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z);

  // (3) Stability gate (translation + orientation; velocity is informational).
  pose_window_.emplace_back(T, v);
  while (static_cast<int>(pose_window_.size()) > ins_min_pose_window_samples_) {
    pose_window_.pop_front();
  }
  if (static_cast<int>(pose_window_.size()) < ins_min_pose_window_samples_) {
    ins_last_reject_reason_ = "accumulating pose window";
    return;
  }
  for (size_t i = 1; i < pose_window_.size(); ++i) {
    const auto& a = pose_window_[i - 1].first;
    const auto& b = pose_window_[i    ].first;
    if ((a.translation() - b.translation()).norm() > ins_max_pose_jitter_trans_) {
      ins_last_reject_reason_ = "pose translation jitter (INS still settling)";
      pose_window_.clear();
      return;
    }
    Eigen::Quaterniond qa(a.linear()), qb(b.linear());
    if (std::abs(qa.dot(qb)) < ins_min_quat_dot_) {
      ins_last_reject_reason_ = "pose orientation jitter (INS still settling)";
      pose_window_.clear();
      return;
    }
  }

  if (odometry_estimation) {
    odometry_estimation->set_init_state(T, v);
    ins_init_applied.store(true);
    spdlog::info(
      "{}Hitch fork: INS init pose+velocity ACCEPTED — fix={}, pos σ={:.3f} m, "
      "translation=[{:.3f}, {:.3f}, {:.3f}], v=[{:.3f}, {:.3f}, {:.3f}]{}",
      CYAN, fix_status_name(fix_status), pos_stddev,
      T.translation().x(), T.translation().y(), T.translation().z(),
      v.x(), v.y(), v.z(), RESET);
    // Subscriptions stay alive post-init for the GNSS factor bridge.
    if (ins_init_timeout_timer) ins_init_timeout_timer->cancel();
  }
}

void GlimROS::try_publish_gnss_factor(
  const Eigen::Isometry3d& T,
  const std::array<double, 36>& pose_cov,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& frame_id) {
  // Hitch Sensor Dome fork — RTK-gated bridge for libgnss_global.so.
  //
  // Drops the pose if any of:
  //   - publisher not initialized (gnss factor bridge disabled)
  //   - last_fix_ is null (no NavSatFix received yet)
  //   - last_fix_ is below RTK-fixed status when require_rtk_fixed is on
  //   - position covariance stddev is above threshold
  //
  // Suspends silently during RTK-float / no-fix / tunnel periods; the
  // gnss_global module sees a quiet topic and emits no factors. When
  // RTK locks again, factors resume on the next /pose. The optimizer's
  // LiDAR cost carries the trajectory through the gap.
  if (!gnss_pose_pub_) return;
  if (!last_fix_) {
    gnss_factors_rejected_.fetch_add(1);
    return;
  }
  if (gnss_factor_require_rtk_fixed_ &&
      last_fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    gnss_factors_rejected_.fetch_add(1);
    return;
  }
  const double pos_stddev = cov_diag_stddev(last_fix_->position_covariance);
  if (pos_stddev > gnss_factor_max_position_stddev_) {
    gnss_factors_rejected_.fetch_add(1);
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;
  out.pose.pose.position.x = T.translation().x();
  out.pose.pose.position.y = T.translation().y();
  out.pose.pose.position.z = T.translation().z();
  Eigen::Quaterniond q(T.linear());
  out.pose.pose.orientation.x = q.x();
  out.pose.pose.orientation.y = q.y();
  out.pose.pose.orientation.z = q.z();
  out.pose.pose.orientation.w = q.w();

  // Prefer the upstream pose_cov (when Odometry was the source); fall
  // back to NavSatFix's position_covariance in the upper-left 3×3 block
  // when the upstream had no covariance (PoseStamped). Either way the
  // covariance is mostly informational — gnss_global ignores it (per
  // its own header comment), but downstream tools / Foxglove / RViz2
  // will read it for display.
  bool any_cov = false;
  for (double v : pose_cov) if (v != 0.0) { any_cov = true; break; }
  if (any_cov) {
    for (int i = 0; i < 36; ++i) out.pose.covariance[i] = pose_cov[i];
  } else {
    out.pose.covariance.fill(0.0);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        out.pose.covariance[i * 6 + j] = last_fix_->position_covariance[i * 3 + j];
  }

  gnss_pose_pub_->publish(out);
  gnss_factors_published_.fetch_add(1);
}

void GlimROS::ins_init_timeout_tick() {
  if (ins_init_applied.load()) {
    if (ins_init_timeout_timer) ins_init_timeout_timer->cancel();
    return;
  }
  const auto now = this->now();
  const double elapsed = (now - ins_wait_started_).seconds();

  // Throttle the bold RED warning to every 10 s.
  if ((now - ins_last_warn_).seconds() < 10.0) return;
  ins_last_warn_ = now;

  spdlog::warn("");
  spdlog::warn("{}{}{}", RED,
               "============================================================", RESET);
  spdlog::warn("{}  ⚠  GLIM is waiting for a reliable INS pose. ⚠  {}", RED, RESET);
  spdlog::warn("{}{}{}", RED,
               "============================================================", RESET);
  spdlog::warn("  Elapsed:  {:.1f} s  (timeout {:.1f} s)", elapsed, ins_init_timeout_s_);
  if (last_fix_) {
    const double s = cov_diag_stddev(last_fix_->position_covariance);
    spdlog::warn("  Last NavSatFix: status={}, pos σ={:.3f} m, "
                 "lat={:.6f}, lon={:.6f}, alt={:.2f}",
                 fix_status_name(last_fix_->status.status), s,
                 last_fix_->latitude, last_fix_->longitude, last_fix_->altitude);
  } else {
    spdlog::warn("  Last NavSatFix: NONE — is ins_fix_topic publishing?");
  }
  if (!ins_last_reject_reason_.empty()) {
    spdlog::warn("  Last reject reason: {}", ins_last_reject_reason_);
  }
  spdlog::warn("");
  spdlog::warn("  Why GLIM is blocked:");
  spdlog::warn("    The Hitch fork uses the Atlas Duo INS pose as the");
  spdlog::warn("    ground-truth orientation for SLAM initialization. To");
  spdlog::warn("    avoid anchoring the entire map to a bad pose, the");
  spdlog::warn("    wrapper enforces RTK-fixed status and covariance gating.");
  spdlog::warn("");
  spdlog::warn("  What you can do:");
  spdlog::warn("    1. Wait — RTK convergence typically takes 30–120 s outdoors.");
  spdlog::warn("    2. Check sky visibility — RTK needs unobstructed L1+L5.");
  spdlog::warn("    3. Verify NTRIP corrections are flowing (check Atlas web UI).");
  spdlog::warn("    4. Relax the gate (degraded init) by re-launching with:");
  spdlog::warn("         ins_require_rtk_fixed:=false");
  spdlog::warn("         ins_max_position_stddev:=0.5    # 0.5 m for SBAS-class");
  spdlog::warn("       Note: relaxing INVALIDATES the fix to the moving-start");
  spdlog::warn("       pathology; the map will not be reliably gravity-aligned.");

  if (elapsed > ins_init_timeout_s_) {
    spdlog::warn("");
    spdlog::warn("{}  TIMEOUT exceeded. Aborting glim_rosnode is now safe.{}", RED, RESET);
    spdlog::warn("{}  (No automatic abort — GLIM will continue to wait if you{}", RED, RESET);
    spdlog::warn("{}  prefer to leave it running until the fix locks.){}", RED, RESET);
  }
  spdlog::warn("");
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (!GlobalConfig::instance()->has_param("meta", "image_frame")) {
    spdlog::debug("auto-detecting image frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "image_frame", msg->header.frame_id);
  }

  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}
#endif

size_t GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (!GlobalConfig::instance()->has_param("meta", "lidar_frame_id")) {
    spdlog::debug("auto-detecting LiDAR frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "lidar_frame_id", msg->header.frame_id);
  }

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  if (flip_points_y) {
    for (auto& p : raw_points->points) {
      p.y() = -p.y();
    }
  }

  raw_points->stamp += points_time_offset;
  if (!time_keeper->process(raw_points)) {
    spdlog::warn("skip an invalid point cloud (stamp={})", raw_points->stamp);
    return 0;
  }
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);

  const size_t workload = odometry_estimation->workload();
  spdlog::debug("workload={}", workload);

  return workload;
}

bool GlimROS::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void GlimROS::timer_callback() {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
    }
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  if (sub_mapping) {
    for (const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimROS::save(const std::string& path) {
  if (global_mapping) global_mapping->save(path);
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);
