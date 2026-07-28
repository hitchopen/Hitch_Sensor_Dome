#include <stdexcept>
#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <cmath>
#include <limits>   // [P2 FIX 2026-07-27] quiet_NaN() in the deprecated-key conversion
#include <cstdio>
#include <algorithm>
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
#include <glim/util/urdf_transforms.hpp>

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
  expected_time_field =
    config_sensors.param<std::string>("sensors", "expected_time_field", "");
  expected_time_datatype =
    config_sensors.param<int>("sensors", "expected_time_datatype", 0);
  expected_time_is_absolute =
    config_sensors.param<bool>("sensors", "expected_time_is_absolute", false);
  reject_zero_point_times =
    config_sensors.param<bool>("sensors", "reject_zero_point_times", false);
  imu_input_rotation = config_sensors.param<Eigen::Quaterniond>("sensors", "imu_input_rotation", Eigen::Quaterniond::Identity());
  if (!imu_input_rotation.coeffs().allFinite() || imu_input_rotation.norm() < 1e-9) {
    throw std::invalid_argument("sensors.imu_input_rotation must be a finite, non-zero quaternion [x,y,z,w]");
  }
  imu_input_rotation.normalize();
  if (std::abs(imu_input_rotation.w() - 1.0) > 1e-12 || imu_input_rotation.vec().norm() > 1e-12) {
    logger->info(
      "IMU input calibration enabled: q_input_to_calibrated=[{:.9f}, {:.9f}, {:.9f}, {:.9f}]",
      imu_input_rotation.x(),
      imu_input_rotation.y(),
      imu_input_rotation.z(),
      imu_input_rotation.w());
  }
  // Explicit compatibility opt-in for a raw uint64 epoch-nanosecond carrier
  // mislabeled as FLOAT64. Robin W publishes numeric IEEE-754 epoch seconds,
  // so the Hitch profile keeps this false.
  float64_time_is_epoch_ns = config_sensors.param<bool>("sensors", "float64_time_is_epoch_ns", false);
  flip_points_y = config_sensors.param<bool>("sensors", "flip_points_y", false);

  // Multi-LiDAR concatenation is implemented by the future-aware offline
  // readers. The live callback has no equivalent release queue, so reject
  // online+concat immediately after both configuration flags are known.
  aux_concat = glim_ros::load_aux_sensors_from_config(config_sensors);

  this->online_mapping_enabled_ =
    config_ros.param<bool>("glim_ros", "enable_online_mapping", false);
  if (!glim_ros::online_concat_configuration_supported(
        this->online_mapping_enabled_, aux_concat.enabled)) {
    spdlog::critical(
      "glim_ros: online mapping (enable_online_mapping=true) combined with lidar_concat is "
      "unsupported — the live path has no future-sweep release and would drop the late aux "
      "sweep. Build the concatenated map offline (glim_rosbag / glim_pcap_rosbag). Refusing to start.");
    throw std::runtime_error(
      "glim_ros: online mapping + lidar_concat is unsupported (no future-sweep release)");
  }

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
      // [P3 FIX 2026-07-10] The override only reaches the other modules VIA
      // DISK (each constructs its own Config from this path). Config::save
      // does not check the stream: on a read-only install prefix the write
      // silently fails, the INFO above still claims the override, and the
      // estimator runs with the stale checked-in extrinsic. Verify the
      // round-trip and fail LOUDLY — the extrinsic is safety-relevant.
      {
        glim::Config verify(config_sensors_path);
        const auto readback = verify.param<Eigen::Isometry3d>("sensors", "T_lidar_imu");
        if (!readback || !readback->isApprox(T_lidar_imu, 1e-9)) {
          logger->critical(
            "URDF T_lidar_imu override did NOT persist to {} (read-only install prefix?) — "
            "modules would silently use the stale checked-in extrinsic; aborting",
            config_sensors_path);
          throw std::runtime_error("config_sensors.json override write failed");
        }
      }
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
  // [P3 FIX 2026-07-14] Hand the operator-configured points_time_offset to the
  // TimeKeeper so it survives the absolute-time stamp overwrite. Previously the
  // offset was added to raw_points->stamp before process(), but the
  // absolute-time branch of replace_points_stamp overwrites the stamp with the
  // raw min point time and silently discarded it for absolute-time clouds.
  time_keeper->set_point_time_offset(points_time_offset);
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

  // Online (live subscription) mapping passway. GLIM builds maps OFFLINE only
  // (glim_rosbag / glim_pcap_rosbag feed the callbacks directly and drive
  // timer_callback() manually), so by default we create NO live subscriptions
  // and NO wall timer. The opt-in legacy live path remains single-LiDAR only.
  // Hitch Sensor Dome fork: external INS init with RTK-fixed gating.
  //
  // GLIM's initial state is anchored to the Atlas Duo's INS pose, but
  // accepting that pose requires confirming the INS itself is reliable.
  // Inputs:
  //
  //   - ins_pose_topic (production default empty) — compatibility
  //     geometry_msgs/PoseStamped source. It has no solution_type and must
  //     not be used for Fixed-only production mapping.
  //   - ins_odom_topic (production default adapter Fixed-only topic) —
  //     nav_msgs/Odometry carrying the Atlas Duo's full 6-DOF state.
  //   - ins_fix_topic  (default /gps_p1/fix) — the adapter's synchronized
  //     sensor_msgs/NavSatFix gate signal: status and position covariance.
  //
  // Gate logic lives in ins_pose_callback / ins_odom_callback. These
  // parameters are parsed in BOTH modes: the online path subscribes live
  // below, while the offline drivers (glim_rosbag / glim_pcap_rosbag) read
  // the same topics from the bag and call ins_*_callback() directly, so
  // the INS init gate and the RTK-gated GNSS factor bridge behave
  // identically in replay. (B4 fix: this block used to be online-only,
  // which silently disabled INS init and starved gnss_global of factors in
  // the offline-only workflow.)
  ins_pose_topic_ = config_ros.param<std::string>("glim_ros", "ins_pose_topic", "");
  ins_odom_topic_ = config_ros.param<std::string>(
    "glim_ros", "ins_odom_topic", "/gps_p1/filtered_odom_rtk_fixed");
  ins_fix_topic_  =
    config_ros.param<std::string>("glim_ros", "ins_fix_topic", "/gps_p1/fix");

  // Gate thresholds — overridable from config_ros.json.
  ins_require_rtk_fixed_       = config_ros.param<bool>(  "glim_ros", "ins_require_rtk_fixed",       true);
  ins_max_position_stddev_     = config_ros.param<double>("glim_ros", "ins_max_position_stddev",     0.10);
  ins_min_pose_window_samples_ = config_ros.param<int>(   "glim_ros", "ins_min_pose_window_samples", 10);
  ins_max_pose_jitter_trans_   = config_ros.param<double>("glim_ros", "ins_max_pose_jitter_trans",   0.05);
  // [P2 FIX 2026-07-27] Attitude bound is configured in DEGREES. The legacy
  // `ins_min_quat_dot` key is honoured for one release. PRESENCE, not value,
  // selects the source (same discipline as lidar_concat's sweep threshold):
  // reading with an out-of-band default would let a deliberately invalid entry
  // masquerade as an absent one and skip validation.
  {
    const bool has_deg = config_ros.has_param("glim_ros", "ins_max_attitude_residual_deg");
    const bool has_dot = config_ros.has_param("glim_ros", "ins_min_quat_dot");
    if (has_deg) {
      ins_max_attitude_residual_deg_ =
        config_ros.param<double>("glim_ros", "ins_max_attitude_residual_deg", 2.5);
      if (has_dot) {
        spdlog::warn(
          "glim_ros: both 'ins_max_attitude_residual_deg' and the deprecated "
          "'ins_min_quat_dot' are present — using the degree form ({:.3f}°) and "
          "IGNORING the dot form. Please delete 'ins_min_quat_dot'.",
          ins_max_attitude_residual_deg_);
      }
    } else if (has_dot) {
      const double dot = config_ros.param<double>("glim_ros", "ins_min_quat_dot", 0.999);
      // Only a dot in [0, 1] maps to a real angle; anything else falls through
      // to the range check below as an out-of-range degree value rather than
      // producing NaN from std::acos.
      ins_max_attitude_residual_deg_ =
        (std::isfinite(dot) && dot >= 0.0 && dot <= 1.0)
          ? 2.0 * std::acos(dot) * 180.0 / M_PI
          : std::numeric_limits<double>::quiet_NaN();
      spdlog::warn(
        "glim_ros: 'ins_min_quat_dot' is deprecated — it is a quaternion dot "
        "product, and because |q_a·q_b| = cos(θ/2) its documented angle was "
        "half the residual actually admitted. Converted {:.6f} -> "
        "ins_max_attitude_residual_deg = {:.3f}°. NOTE: the old default 0.999 "
        "was annotated \"2.5°\" but admits 5.125°; set the degree key "
        "explicitly to get the value you intend.",
        dot, ins_max_attitude_residual_deg_);
    } else {
      ins_max_attitude_residual_deg_ = 2.5;
    }
  }
  ins_max_pose_gap_s_          = config_ros.param<double>("glim_ros", "ins_max_pose_gap_s",          0.5);
  fix_max_age_s_               = config_ros.param<double>("glim_ros", "fix_max_age_s",               0.5);
  fix_future_tolerance_s_      = config_ros.param<double>("glim_ros", "fix_future_tolerance_s",      0.05);
  require_rtk_anchor_          = config_ros.param<bool>(  "glim_ros", "require_rtk_anchor",          false);
  if (!std::isfinite(ins_max_position_stddev_) || ins_max_position_stddev_ <= 0.0 ||
      ins_min_pose_window_samples_ < 3 ||
      !std::isfinite(ins_max_pose_jitter_trans_) || ins_max_pose_jitter_trans_ <= 0.0 ||
      !std::isfinite(ins_max_attitude_residual_deg_) ||
      ins_max_attitude_residual_deg_ <= 0.0 || ins_max_attitude_residual_deg_ >= 180.0 ||
      !std::isfinite(ins_max_pose_gap_s_) || ins_max_pose_gap_s_ <= 0.0) {
    throw std::runtime_error(
      "glim_ros: invalid INS gate thresholds (stddev/jitter/gap must be finite and > 0, "
      "window >= 3, ins_max_attitude_residual_deg in (0, 180))");
  }
  if (!std::isfinite(fix_max_age_s_) || fix_max_age_s_ <= 0.0 ||
      !std::isfinite(fix_future_tolerance_s_) || fix_future_tolerance_s_ < 0.0) {
    throw std::runtime_error(
      "glim_ros: fix_max_age_s must be finite and > 0, fix_future_tolerance_s finite and >= 0");
  }
  ins_init_timeout_s_          = config_ros.param<double>("glim_ros", "ins_init_timeout_s",          60.0);
  if (!std::isfinite(ins_init_timeout_s_) || ins_init_timeout_s_ <= 0.0) {
    throw std::runtime_error("glim_ros: ins_init_timeout_s must be finite and > 0");
  }

  // Dual-antenna RTK heading mode. When enabled we tighten the init gate
  // (orientation locks faster and more precisely with dual-antenna
  // heading). The session-long factor bridge stamps the corresponding
  // orientation covariance on every published message — see
  // try_publish_gnss_factor below. NOTE: read from config_ros.json
  // ("glim_ros" section), the single source of truth in both modes.
  dual_antenna_enabled_       = config_ros.param<bool>(  "glim_ros", "dual_antenna_enabled",            false);
  dual_antenna_baseline_m_    = config_ros.param<double>("glim_ros", "dual_antenna_baseline_m",         0.0);
  dual_antenna_heading_sigma_rad_ =
                                 config_ros.param<double>("glim_ros", "dual_antenna_heading_sigma_rad", 0.0);
  if (dual_antenna_enabled_) {
    // Tighter stability + shorter timeout — RTK-fixed dual-antenna
    // heading converges within a few /pose samples, so we don't need
    // the conservative single-antenna defaults.
    // 0.8° vs the 2.5° single-antenna bound. This is the TRUE residual angle;
    // the predecessor wrote 0.9999 here and called it 0.8°, which actually
    // admitted 1.621°.
    ins_max_attitude_residual_deg_ = std::min(ins_max_attitude_residual_deg_, 0.8);
    ins_min_pose_window_samples_ = std::min(ins_min_pose_window_samples_, 5);
    ins_init_timeout_s_ = std::min(ins_init_timeout_s_, 30.0);
    spdlog::info(
      "Hitch fork: DUAL-antenna mode — baseline={:.3f} m, expected "
      "heading σ={:.3f} rad ({:.2f}°). Init gates auto-tightened: "
      "max_attitude_residual={:.2f}°, window={} samples, timeout={:.0f} s.",
      dual_antenna_baseline_m_, dual_antenna_heading_sigma_rad_,
      dual_antenna_heading_sigma_rad_ * 180.0 / 3.14159265358979,
      ins_max_attitude_residual_deg_, ins_min_pose_window_samples_, ins_init_timeout_s_);
  } else {
    spdlog::info(
      "Hitch fork: SINGLE-antenna mode — heading derived from IMU "
      "(drift-prone). Set the secondary antenna translation in "
      "config/sensor_dome_tf.yaml to enable dual-antenna heading.");
  }

  // Derive the comparison constant ONCE, after every adjustment to the degree
  // bound (including the dual-antenna tightening above). |q_a·q_b| = cos(θ/2),
  // so a residual angle θ maps to the dot-product floor cos(θ/2). The hot path
  // then compares dot products directly — no trig per sample.
  ins_min_quat_dot_ = std::cos(0.5 * ins_max_attitude_residual_deg_ * M_PI / 180.0);

  spdlog::info(
    "Hitch fork: RTK gate — require_rtk_fixed={}, max_pos_stddev={:.2f} m, "
    "window={} samples, max_jitter={:.3f} m, max_attitude_residual={:.2f}° "
    "(|q_pred·q| >= {:.6f}), timeout={:.0f} s",
    ins_require_rtk_fixed_, ins_max_position_stddev_,
    ins_min_pose_window_samples_, ins_max_pose_jitter_trans_,
    ins_max_attitude_residual_deg_, ins_min_quat_dot_, ins_init_timeout_s_);

  // ---- GNSS factor bridge (post-init, RTK-gated republisher) ----
  // Each accepted /pose / /odom is evaluated against the most recent
  // NavSatFix and republished on gnss_factor_topic as
  // PoseWithCovarianceStamped. libgnss_global.so consumes that topic
  // (configured in glim_ext/config/config_gnss_global.json) and turns
  // each message into a soft prior factor in the global graph. The
  // publisher exists in both modes; in offline replay
  // try_publish_gnss_factor additionally hands accepted samples straight
  // to the extension subscription (no live subscribers exist offline).
  gnss_factor_topic_ =
    config_ros.param<std::string>("glim_ros", "gnss_factor_topic", "/gnss/pose_rtk_only");
  gnss_factor_require_rtk_fixed_ =
    config_ros.param<bool>(  "glim_ros", "gnss_factor_require_rtk_fixed",   true);
  gnss_factor_max_position_stddev_ =
    config_ros.param<double>("glim_ros", "gnss_factor_max_position_stddev", 0.10);
  if (!std::isfinite(gnss_factor_max_position_stddev_) ||
      gnss_factor_max_position_stddev_ <= 0.0) {
    throw std::runtime_error(
      "glim_ros: gnss_factor_max_position_stddev must be finite and > 0");
  }

  if (!gnss_factor_topic_.empty()) {
    gnss_pose_pub_ = this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(gnss_factor_topic_, 50);
    spdlog::info(
      "Hitch fork: GNSS factor bridge — publishing on '{}' "
      "(require_rtk_fixed={}, max_pos_stddev={:.2f} m)",
      gnss_factor_topic_, gnss_factor_require_rtk_fixed_,
      gnss_factor_max_position_stddev_);

    // Periodic status log — every 10 s reports accepted / rejected counts
    // so the operator can see whether the bridge is doing useful work
    // mid-session (e.g., expected to drop to zero in tunnels). Serviced by
    // spin_some() in the offline drivers, by the executor when live.
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

  // [B1 FIX 2026-07-27] The INS init-gate watchdog is created in BOTH modes.
  // It used to live inside the online-mapping block below — but this platform
  // maps OFFLINE ONLY (enable_online_mapping=false, and online + lidar_concat
  // is hard-refused), so the entire operator-facing diagnostic for a blocked
  // init was dead code in glim_rosbag: a bag whose RTK never locks produced
  // zero submaps at full replay speed with no warning at all, and
  // ins_init_timeout_s was a dead knob. The offline readers call
  // rclcpp::spin_some(glim) per bag message, so a wall timer is serviced
  // there exactly as it is under a live executor.
  ins_wait_started_ = this->now();
  ins_last_warn_    = ins_wait_started_;
  ins_init_timeout_timer = this->create_wall_timer(
    std::chrono::seconds(2),
    [this]() { ins_init_timeout_tick(); });

  if (this->online_mapping_enabled_) {
    const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
    const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
    const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

    // Subscribers
    rclcpp::SensorDataQoS default_imu_qos;
    default_imu_qos.get_rmw_qos_profile().depth = 1000;
    auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1));

    qos = get_qos_settings(config_ros, "glim_ros", "points_qos");
    // Route the primary cloud through points_callback_live() so buffered aux
    // clouds are merged in before odometry sees the scan (parity with offline).
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, qos, std::bind(&GlimROS::points_callback_live, this, _1));

#ifdef BUILD_WITH_CV_BRIDGE
    qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
    image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

    // Hitch Sensor Dome fork: live subscriptions for the INS init gate +
    // GNSS factor bridge (parameters parsed above, in both modes). A
    // periodic timer (ins_init_timeout_tick) prints a bold RED warning
    // every 10 s while the gate is still blocking, naming the most-recent
    // rejection reason so the operator can act.
    rclcpp::QoS ins_qos(20);
    ins_qos.reliable();

    if (!ins_fix_topic_.empty()) {
      ins_fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        ins_fix_topic_, ins_qos,
        std::bind(&GlimROS::ins_fix_callback, this, _1));
      spdlog::info("Hitch fork: subscribed to NavSatFix topic '{}' (gate signal)", ins_fix_topic_);
    } else {
      spdlog::warn("Hitch fork: ins_fix_topic is empty — RTK gate cannot run; INS pose will be rejected.");
    }
    if (!ins_pose_topic_.empty()) {
      ins_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ins_pose_topic_, ins_qos,
        std::bind(&GlimROS::ins_pose_callback, this, _1));
      spdlog::info("Hitch fork: subscribed to INS PoseStamped topic '{}'", ins_pose_topic_);
    }
    if (!ins_odom_topic_.empty()) {
      ins_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        ins_odom_topic_, ins_qos,
        std::bind(&GlimROS::ins_odom_callback, this, _1));
      spdlog::info("Hitch fork: subscribed to INS Odometry topic '{}'", ins_odom_topic_);
    }

    for (const auto& sub : this->extension_subscriptions()) {
      spdlog::debug("subscribe to {}", sub->topic);
      sub->create_subscriber(*this);
    }

    // Start timer
    timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });
    spdlog::warn("ONLINE GLIM mapping ENABLED (glim_ros/enable_online_mapping=true) -- live subscriptions created");
  } else {
    spdlog::info(
      "online GLIM mapping DISABLED -- no live subscriptions or wall timer created. "
      "Build maps offline with glim_rosbag / glim_pcap_rosbag.");
  }

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
  const Eigen::Vector3d linear_acc = imu_input_rotation * (acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
  const Eigen::Vector3d angular_vel = imu_input_rotation * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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
//       residual below ins_max_attitude_residual_deg. This rejects IMU-only
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
  // DISPLAY ONLY. Returns +inf for any covariance that is not a usable
  // measurement, so it can never read as "tighter than" a real one.
  // Gating decisions must go through validate_fix() below, which reports
  // WHY a fix was refused.
  //
  // [B3 FIX 2026-07-27] This used to be sqrt(max(0.0, C[i])), which mapped
  // every "no information" encoding — an all-zero COVARIANCE_TYPE_UNKNOWN
  // block, and NaN (std::max(0.0, NaN) returns 0.0) — to sigma = 0.000 m,
  // i.e. the TIGHTEST POSSIBLE fix. The gate therefore failed OPEN on
  // exactly the cold-start / invalid-solution cases it existed to catch.
  double worst = 0.0;
  for (const int i : {0, 4, 8}) {
    const double v = C[i];
    if (!std::isfinite(v) || v <= 0.0) return std::numeric_limits<double>::infinity();
    worst = std::max(worst, std::sqrt(v));
  }
  return worst;
}

// Outcome of validating a NavSatFix as a gate signal. Fails CLOSED: every
// path that is not a positively-verified good fix returns ok=false with a
// reason string, so a rejection is always explicable in the log.
struct FixVerdict {
  bool ok = false;
  double pos_stddev = std::numeric_limits<double>::infinity();
  double age_s = std::numeric_limits<double>::quiet_NaN();
  std::string reason;
};

FixVerdict validate_fix(
  const sensor_msgs::msg::NavSatFix* fix,
  double consumer_stamp_s,      // header stamp of the pose being gated
  double max_age_s,
  double future_tolerance_s,
  double max_stddev,
  bool require_rtk_class,
  const std::array<double, 36>* fallback_pose_cov = nullptr) {
  FixVerdict v;
  if (!fix) {
    if (require_rtk_class) {
      v.reason = "no NavSatFix received yet";
      return v;
    }
    if (!fallback_pose_cov) {
      v.reason =
        "no NavSatFix received and this pose source carries no covariance";
      return v;
    }

    // Explicit degraded mode: an Odometry source may bootstrap without a
    // NavSatFix, but only from real, bounded position covariance. This makes
    // ins_require_rtk_fixed=false operational without turning missing quality
    // metadata into a perfect fix. PoseStamped still fails closed because it
    // has no covariance to prove position quality.
    double worst = 0.0;
    for (const int i : {0, 7, 14}) {
      const double c = (*fallback_pose_cov)[i];
      if (!std::isfinite(c) || c <= 0.0) {
        v.reason =
          "no NavSatFix and odometry position covariance is invalid";
        return v;
      }
      worst = std::max(worst, std::sqrt(c));
    }
    v.pos_stddev = worst;
    if (v.pos_stddev > max_stddev) {
      char buf[160];
      std::snprintf(
        buf, sizeof(buf),
        "no NavSatFix and odometry position sigma %.3f m > %.3f m",
        v.pos_stddev, max_stddev);
      v.reason = buf;
      return v;
    }
    v.ok = true;
    v.reason = "ok (degraded odometry-covariance fallback)";
    return v;
  }

  // (a) Solution status.
  if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    v.reason = "NavSatFix reports NO_FIX";
    return v;
  }
  if (require_rtk_class &&
      fix->status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    v.reason = std::string("fix status ") + fix_status_name(fix->status.status) +
               " is below RTK class";
    return v;
  }

  // (b) Covariance must be a real measurement. UNKNOWN, all-zero,
  // negative and non-finite are all refusals — never "perfect".
  if (fix->position_covariance_type ==
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    v.reason = "position_covariance_type = UNKNOWN (no covariance reported)";
    return v;
  }
  for (const int i : {0, 4, 8}) {
    const double c = fix->position_covariance[i];
    if (!std::isfinite(c)) {
      v.reason = "non-finite position covariance (invalid INS solution)";
      return v;
    }
    if (c <= 0.0) {
      v.reason = "non-positive position covariance (unpopulated / sentinel)";
      return v;
    }
  }
  v.pos_stddev = cov_diag_stddev(fix->position_covariance);

  // (c) The position itself must be a real WGS84 coordinate. FusionEngine
  // emits NaN lat/lon for SolutionType::Invalid.
  if (!std::isfinite(fix->latitude) || !std::isfinite(fix->longitude) ||
      !std::isfinite(fix->altitude) ||
      std::abs(fix->latitude) > 90.0 || std::abs(fix->longitude) > 180.0) {
    v.reason = "invalid latitude/longitude/altitude";
    return v;
  }

  // (d) Freshness, by MESSAGE time. Wall time is meaningless during
  // offline replay, and a stale fix must never keep authorizing factors:
  // one RTK sample would otherwise license the whole rest of a session.
  const double fix_stamp_s = to_sec(fix->header.stamp);
  if (!std::isfinite(fix_stamp_s) || !std::isfinite(consumer_stamp_s)) {
    v.reason = "non-finite message timestamp";
    return v;
  }
  v.age_s = consumer_stamp_s - fix_stamp_s;
  if (v.age_s > max_age_s) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "fix is stale (%.3f s > %.3f s)", v.age_s, max_age_s);
    v.reason = buf;
    return v;
  }
  if (v.age_s < -future_tolerance_s) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "fix is %.3f s in the future (tolerance %.3f s)",
                  -v.age_s, future_tolerance_s);
    v.reason = buf;
    return v;
  }

  // (e) Reported precision.
  if (v.pos_stddev > max_stddev) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "position sigma %.3f m > %.3f m",
                  v.pos_stddev, max_stddev);
    v.reason = buf;
    return v;
  }

  v.ok = true;
  v.reason = "ok";
  return v;
}

}  // anonymous namespace

void GlimROS::ins_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  // Track the most recent fix for the pose/odom callbacks to consult.
  // No gating decision happens here — the pose callbacks own that.
  last_fix_ = msg;
}

bool GlimROS::ins_window_push_and_check(
  double stamp,
  const Eigen::Isometry3d& T,
  Eigen::Vector3d* v_est) {
  // ---------------------------------------------------------------------
  // Hitch Sensor Dome fork — INS stability gate, MOVING-START CAPABLE.
  //
  // The Atlas Duo is a TIGHTLY-COUPLED INS: its own calibration, attitude
  // and velocity estimation are P1's responsibility. GLIM's job at the
  // boundary is therefore to confirm that P1's solution has SETTLED and is
  // SELF-CONSISTENT — not to re-derive vehicle state, and emphatically not
  // to require the vehicle to be parked.
  //
  // [FIX 2026-07-27] The previous gate compared RAW inter-sample
  // displacement against ins_max_pose_jitter_trans (0.05 m). At the Atlas's
  // 10 Hz /pose rate that is a hard 0.5 m/s (1.8 km/h) ceiling: above it
  // every consecutive pair failed, the window was cleared on the first
  // pair, and initialization could NEVER complete. That silently forbade
  // exactly the moving-start case this fork was built for (mid-session
  // restarts, trimmed bags, race-track replays) — see
  // docs/moving_start_initialization.md.
  //
  // The test is now a SMOOTHNESS residual against a constant-velocity /
  // constant-angular-rate prediction from the two preceding samples. It is
  // ~0 for any steady motion at any speed, small under moderate
  // acceleration, and large only for a genuine INS jump — which is the
  // thing worth rejecting. A stationary start still passes trivially
  // (residual ≈ 0), so this is a strict superset of the old behavior.
  // ---------------------------------------------------------------------
  if (!std::isfinite(stamp) || !T.matrix().allFinite()) {
    ins_last_reject_reason_ = "non-finite INS stamp or pose";
    pose_window_.clear();
    return false;
  }

  // Temporal contiguity. Samples reach this window only AFTER the RTK
  // status/covariance gates pass, so without this bound a fix that flickers
  // into RTK once every N seconds could assemble a "stable" window out of
  // moments minutes apart — each pair individually consistent because the
  // vehicle happened to be parked, while nothing about the run was settled.
  if (!pose_window_.empty()) {
    const double gap = stamp - pose_window_.back().stamp;
    if (gap <= 0.0) {
      // Non-monotonic (bag loop, duplicate publish). Restart from here.
      ins_last_reject_reason_ = "non-monotonic INS stamp; window restarted";
      pose_window_.clear();
    } else if (gap > ins_max_pose_gap_s_) {
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "INS gap %.3f s > %.3f s (non-contiguous); window restarted",
                    gap, ins_max_pose_gap_s_);
      ins_last_reject_reason_ = buf;
      ins_last_hard_reject_ = buf;
      ++ins_hard_reject_count_;
      pose_window_.clear();
    }
  }

  InsSample sample;
  sample.stamp = stamp;
  sample.T = T;
  pose_window_.push_back(sample);

  // A constant-velocity prediction needs three samples, so the window is at
  // least 3 deep regardless of how low the operator sets the parameter.
  const size_t required =
    static_cast<size_t>(std::max(3, ins_min_pose_window_samples_));
  while (pose_window_.size() > required) {
    pose_window_.pop_front();
  }
  if (pose_window_.size() < required) {
    char buf[288];
    if (ins_hard_reject_count_ > 0) {
      std::snprintf(buf, sizeof(buf),
                    "accumulating INS window (%zu/%zu) after %llu rejection(s); last: %s",
                    pose_window_.size(), required,
                    static_cast<unsigned long long>(ins_hard_reject_count_),
                    ins_last_hard_reject_.c_str());
    } else {
      std::snprintf(buf, sizeof(buf), "accumulating INS window (%zu/%zu)",
                    pose_window_.size(), required);
    }
    ins_last_reject_reason_ = buf;
    return false;
  }

  for (size_t i = 2; i < pose_window_.size(); ++i) {
    const auto& s0 = pose_window_[i - 2];
    const auto& s1 = pose_window_[i - 1];
    const auto& s2 = pose_window_[i];
    const double dt_prev = s1.stamp - s0.stamp;
    const double dt      = s2.stamp - s1.stamp;
    if (dt_prev <= 0.0 || dt <= 0.0) {
      ins_last_reject_reason_ = "non-monotonic INS stamps within window";
      ins_last_hard_reject_ = ins_last_reject_reason_;
      ++ins_hard_reject_count_;
      pose_window_.clear();
      return false;
    }

    // Translation: residual against a constant-velocity extrapolation.
    // Under constant acceleration a this residual is a·dt², so the 0.05 m
    // default tolerates ≈5 m/s² at 10 Hz — vehicle dynamics pass, a
    // metre-scale INS snap does not.
    const Eigen::Vector3d v_prev =
      (s1.T.translation() - s0.T.translation()) / dt_prev;
    const Eigen::Vector3d p_pred = s1.T.translation() + v_prev * dt;
    const double resid = (s2.T.translation() - p_pred).norm();
    if (resid > ins_max_pose_jitter_trans_) {
      char buf[192];
      std::snprintf(buf, sizeof(buf),
                    "INS position residual %.3f m > %.3f m vs constant-velocity "
                    "prediction (|v|=%.2f m/s) — solution not settled",
                    resid, ins_max_pose_jitter_trans_, v_prev.norm());
      ins_last_reject_reason_ = buf;
      ins_last_hard_reject_ = buf;
      ++ins_hard_reject_count_;
      pose_window_.clear();
      return false;
    }

    // Orientation: residual against a constant-angular-rate extrapolation.
    // Re-applying the previous relative rotation, time-scaled to this
    // interval, keeps a steady turn at ~0 residual while still catching an
    // attitude snap.
    const Eigen::Quaterniond q0(s0.T.linear());
    const Eigen::Quaterniond q1(s1.T.linear());
    const Eigen::Quaterniond q2(s2.T.linear());
    const Eigen::Quaterniond dq = (q0.conjugate() * q1).normalized();
    const Eigen::Quaterniond dq_scaled =
      Eigen::Quaterniond::Identity().slerp(dt / dt_prev, dq);
    const Eigen::Quaterniond q_pred = (q1 * dq_scaled).normalized();
    const double qdot = std::abs(q_pred.dot(q2));
    if (qdot < ins_min_quat_dot_) {
      char buf[192];
      std::snprintf(buf, sizeof(buf),
                    "INS attitude residual %.3f deg > %.3f deg vs "
                    "constant-rate prediction — solution not settled",
                    2.0 * std::acos(std::min(1.0, qdot)) * 180.0 / M_PI,
                    ins_max_attitude_residual_deg_);
      ins_last_reject_reason_ = buf;
      ins_last_hard_reject_ = buf;
      ++ins_hard_reject_count_;
      pose_window_.clear();
      return false;
    }
  }

  // World-frame velocity at the newest sample, for callers whose message
  // type carries none. Backward difference over the last interval; its
  // O(a·dt/2) lag is negligible against the optimizer's σ = 1 m/s V(0)
  // prior, whereas seeding zero on a rolling start is not.
  if (v_est) {
    const auto& sN = pose_window_.back();
    const auto& sP = pose_window_[pose_window_.size() - 2];
    const double dt = sN.stamp - sP.stamp;
    *v_est = (dt > 0.0)
               ? Eigen::Vector3d((sN.T.translation() - sP.T.translation()) / dt)
               : Eigen::Vector3d::Zero();
    if (!v_est->allFinite()) {
      *v_est = Eigen::Vector3d::Zero();
    }
  }
  return true;
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

  // ---- (1)+(2) Fix quality gate — SLAM STARTS ONLY AT RTK-FIXED ----
  // [B3 FIX 2026-07-27] Fails CLOSED and expires by MESSAGE time. The old
  // gate compared only status + sqrt(max(0,cov)), so an UNKNOWN/NaN
  // covariance read as sigma=0.000 m (the tightest possible fix), and a
  // single fix authorized initialization forever because the fix stamp was
  // never read. Both are now refusals with an explicit reason.
  const FixVerdict fv = validate_fix(
    last_fix_.get(), to_sec(msg->header.stamp), fix_max_age_s_,
    fix_future_tolerance_s_, ins_max_position_stddev_, ins_require_rtk_fixed_);
  if (!fv.ok) {
    ins_last_reject_reason_ = fv.reason;
    return;
  }
  const int fix_status = last_fix_->status.status;
  const double pos_stddev = fv.pos_stddev;

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
  // PoseStamped carries no velocity, so the window estimates it. A moving
  // start MUST seed a real velocity — see ins_window_push_and_check().
  Eigen::Vector3d v_est = Eigen::Vector3d::Zero();
  if (!ins_window_push_and_check(to_sec(msg->header.stamp), T, &v_est)) {
    return;
  }

  // ---- All gates passed → apply ----
  if (odometry_estimation) {
    odometry_estimation->set_init_state(T, v_est);
    ins_init_applied.store(true);
    spdlog::info(
      "{}Hitch fork: INS init pose ACCEPTED — fix={}, pos σ={:.3f} m, "
      "translation=[{:.3f}, {:.3f}, {:.3f}], qw={:.4f}, "
      "v_est=[{:.3f}, {:.3f}, {:.3f}] |v|={:.2f} m/s (from /pose finite difference){}",
      CYAN, fix_status_name(fix_status), pos_stddev,
      T.translation().x(), T.translation().y(), T.translation().z(),
      q.w(), v_est.x(), v_est.y(), v_est.z(), v_est.norm(), RESET);
    // Note: subscriptions are KEPT ALIVE post-init so the factor bridge
    // can republish each subsequent /pose as PoseWithCovarianceStamped on
    // /gnss/pose_rtk_only (gated against the latest ins_fix_topic sample). The init
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
  // SLAM STARTS ONLY AT RTK-FIXED — same fail-closed, message-time-expiring
  // validator as the pose path. See validate_fix().
  std::array<double, 36> source_cov{};
  for (int i = 0; i < 36; ++i) source_cov[i] = msg->pose.covariance[i];
  const FixVerdict fv = validate_fix(
    last_fix_.get(), to_sec(msg->header.stamp), fix_max_age_s_,
    fix_future_tolerance_s_, ins_max_position_stddev_, ins_require_rtk_fixed_,
    &source_cov);
  if (!fv.ok) {
    ins_last_reject_reason_ = fv.reason;
    return;
  }
  const std::string fix_quality =
    last_fix_ ? fix_status_name(last_fix_->status.status)
              : "ODOMETRY_COVARIANCE_ONLY";
  const double pos_stddev = fv.pos_stddev;

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
  // [P1 FIX 2026-07-27] FRAME CONVERSION — twist is a BODY-frame velocity.
  //
  // nav_msgs/Odometry defines twist in `child_frame_id`, and the adapter sets
  // child_frame_id = body_frame_id (adapter_node.cpp:478) while filling
  // twist.linear from FusionEngine `velflu` — Point One's forward-left-up
  // PLATFORM BODY velocity (adapter_node.cpp:549). The adapter is therefore
  // correct per the ROS contract, and GICP++ reads the same field as
  // `v_lin_body` (localization.cc:4170), so the conversion belongs HERE.
  //
  // set_init_state() expects v_world_imu ("IMU velocity in the world frame",
  // odometry_estimation_base.hpp:63) and it seeds the V(0) prior. Passing the
  // body vector unrotated aligned the initial velocity with the wrong world
  // axis: at 90 deg yaw a 20 m/s forward motion was seeded as 20 m/s along
  // world +X instead of +Y — a ~28 m/s vector error against a sigma = 1 m/s
  // prior, on the very interval that anchors the global graph.
  //
  // No axis permutation is needed, only the rotation: velflu is
  // forward-left-up and the dome body frame (imu_link) is REP-103
  // X-fwd/Y-left/Z-up, so the axes already coincide.
  const Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                               msg->twist.twist.linear.y,
                               msg->twist.twist.linear.z);
  const Eigen::Vector3d v = T.linear() * v_body;

  // (3) Stability gate. Odometry already carries P1's own velocity, which is
  // authoritative (tightly-coupled INS), so no estimate is requested here.
  if (!ins_window_push_and_check(to_sec(msg->header.stamp), T, nullptr)) {
    return;
  }

  if (odometry_estimation) {
    odometry_estimation->set_init_state(T, v);
    ins_init_applied.store(true);
    spdlog::info(
      "{}Hitch fork: INS init pose+velocity ACCEPTED — fix={}, pos σ={:.3f} m, "
      "translation=[{:.3f}, {:.3f}, {:.3f}], v_world=[{:.3f}, {:.3f}, {:.3f}] "
      "|v|={:.2f} m/s (rotated from body-frame twist){}",
      CYAN, fix_quality, pos_stddev,
      T.translation().x(), T.translation().y(), T.translation().z(),
      v.x(), v.y(), v.z(), v.norm(), RESET);
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
  //   - last_fix_ is null while RTK-class gating is required, or the degraded
  //     Odometry source has no usable position covariance
  //   - last_fix_ is below RTK-fixed status when require_rtk_fixed is on
  //   - position covariance stddev is above threshold
  //
  // Suspends silently during RTK-float / no-fix / tunnel periods; the
  // gnss_global module sees a quiet topic and emits no factors. When
  // RTK locks again, factors resume on the next /pose. The optimizer's
  // LiDAR cost carries the trajectory through the gap.
  if (!gnss_pose_pub_) return;
  // [B3 FIX 2026-07-27] Fail CLOSED. Every refusal path is explicit and the
  // fix is expired by MESSAGE time, so a single RTK sample can no longer
  // license GNSS factors for the rest of a session while the receiver is
  // dead-reckoning. When this refuses, NO factor is emitted and LiDAR-IMU
  // SLAM simply continues unconstrained — that is the designed
  // RTK-float / no-fix / stale-fix operating state, not an error.
  //
  // NOTE this NavSatFix-driven path is the HEURISTIC compatibility bridge:
  // REP-145 STATUS_GBAS_FIX cannot distinguish RTK-FLOAT from RTK-FIXED,
  // and float ambiguities carry a consistent decimetre-scale BIAS that no
  // robust loss or reported covariance protects against. Production
  // mapping should instead point gnss_global straight at the adapter's
  // explicit solution_type == kRtkFixed stream — see config_gnss_global.json.
  const FixVerdict fv = validate_fix(
    last_fix_.get(), to_sec(stamp), fix_max_age_s_, fix_future_tolerance_s_,
    gnss_factor_max_position_stddev_, gnss_factor_require_rtk_fixed_,
    &pose_cov);
  if (!fv.ok) {
    gnss_factors_rejected_.fetch_add(1);
    if (gnss_factor_last_reject_ != fv.reason) {
      gnss_factor_last_reject_ = fv.reason;
      spdlog::debug("Hitch fork: GNSS factor suppressed — {} (LiDAR-IMU SLAM continues)",
                    fv.reason);
    }
    return;
  }
  const double pos_stddev = fv.pos_stddev;

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
  // when the upstream had no covariance (PoseStamped).
  //
  // Position covariance: gnss_global ignores it (per its own header
  // comment), but downstream tools / Foxglove / RViz2 will read it for
  // display, and a future heading-aware ext module would.
  //
  // Orientation covariance: ALWAYS populated when dual_antenna_enabled.
  // The bottom-right 3×3 of the 6×6 covariance matrix is rpy. We give:
  //   - roll  : 1.0 rad²  (pose-stamped has no info; loose)
  //   - pitch : 1.0 rad²  (same; pitch comes from IMU not GNSS)
  //   - yaw   : (heading_sigma_rad)²   (TIGHT — RTK-derived heading)
  // This data path is what an upcoming heading-constraint ext module
  // would consume to inject yaw factors into the global graph and
  // counteract IMU yaw drift over the session. Even without that
  // module, the covariance is correct in the published message and
  // visible to anything that reads PoseWithCovarianceStamped.
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

  // Hitch Sensor Dome fork — runtime Atlas-yaw-σ sanity check.
  //
  // When the operator believes we're in dual-antenna mode (TF YAML +
  // config_gnss_global.json both say so), we expect the Atlas Duo's
  // own reported yaw σ — pose_cov[35], from the Odometry covariance —
  // to be tight: within yaw_sigma_warn_threshold_mult_ (default 5×)
  // of the dual_antenna_heading_sigma_rad_ we computed from the
  // baseline length. If the Atlas firmware was NOT configured for
  // dual-antenna heading (or the secondary antenna isn't tracking
  // satellites), it falls back to gyro-integrated yaw and the
  // reported σ is several × wider.
  //
  // This is the only check that can catch a misconfigured Atlas
  // firmware — the launch-time check only verifies our own config
  // files. Messages with zero yaw covariance (PoseStamped-only path,
  // since geometry_msgs/PoseStamped has no covariance field) carry no
  // information and are skipped; the check is then effectively
  // disabled until ins_odom_topic is wired up.
  //
  // Sampling: the first yaw_sigma_check_window_samples_ valid samples
  // are tallied; if ≥ yaw_sigma_violation_fraction_ of them exceeded
  // the threshold, a bold-yellow one-shot warning fires. A passing
  // window emits a single info-level confirmation. Either outcome
  // latches via yaw_sigma_warned_, so the check costs O(1) for the
  // remainder of the session.
  if (dual_antenna_enabled_ &&
      dual_antenna_heading_sigma_rad_ > 0.0 &&
      pose_cov[35] > 1e-9 &&
      !yaw_sigma_warned_.load()) {
    const double atlas_yaw_sigma = std::sqrt(pose_cov[35]);
    const double threshold =
      yaw_sigma_warn_threshold_mult_ * dual_antenna_heading_sigma_rad_;
    yaw_sigma_samples_.fetch_add(1);
    if (atlas_yaw_sigma > threshold) {
      yaw_sigma_violations_.fetch_add(1);
    }
    const int n = yaw_sigma_samples_.load();
    if (n >= yaw_sigma_check_window_samples_) {
      const int v = yaw_sigma_violations_.load();
      const double frac =
        static_cast<double>(v) / static_cast<double>(n);
      const std::string YELLOW = "\033[1;33m";
      const std::string RESET  = "\033[0m";
      if (frac >= yaw_sigma_violation_fraction_) {
        bool expected_false = false;
        if (yaw_sigma_warned_.compare_exchange_strong(expected_false, true)) {
          spdlog::warn("");
          spdlog::warn("{}Hitch fork: Atlas yaw σ SANITY CHECK FAILED{}",
                       YELLOW, RESET);
          spdlog::warn(
            "{}  - Operator config: dual-antenna mode (expected yaw σ ≤ "
            "{:.3f} rad ≈ {:.2f}°){}",
            YELLOW,
            dual_antenna_heading_sigma_rad_,
            dual_antenna_heading_sigma_rad_ * 180.0 / M_PI,
            RESET);
          spdlog::warn(
            "{}  - Atlas reported:  yaw σ ≈ {:.3f} rad ≈ {:.2f}° in "
            "{}/{} samples (threshold {:.3f} rad){}",
            YELLOW,
            atlas_yaw_sigma, atlas_yaw_sigma * 180.0 / M_PI,
            v, n, threshold,
            RESET);
          spdlog::warn(
            "{}  - Likely cause: the Atlas Duo firmware is NOT in "
            "dual-antenna heading mode.{}",
            YELLOW, RESET);
          spdlog::warn(
            "{}  - Verify in the Atlas web UI: gnss_lever_arm_secondary "
            "populated; secondary antenna tracking; dual-antenna heading "
            "enabled.{}",
            YELLOW, RESET);
          spdlog::warn(
            "{}  - Until then the rotation prior factor will fire using "
            "the Atlas's gyro-integrated yaw, which drifts and degrades "
            "map quality.{}",
            YELLOW, RESET);
          spdlog::warn(
            "{}  - Mitigation: relaunch with enable_orientation_prior: "
            "false in config_gnss_global.json until the Atlas firmware "
            "is producing real dual-antenna heading.{}",
            YELLOW, RESET);
          spdlog::warn("");
        }
      } else {
        // Healthy state — emit a single info-level confirmation so
        // the operator can see the check ran and passed.
        bool expected_false = false;
        if (yaw_sigma_warned_.compare_exchange_strong(expected_false, true)) {
          spdlog::info(
            "Hitch fork: Atlas yaw σ sanity check PASSED — measured σ ≈ "
            "{:.3f} rad ({:.2f}°) over {}/{} samples within threshold "
            "{:.3f} rad. Dual-antenna heading is live.",
            atlas_yaw_sigma, atlas_yaw_sigma * 180.0 / M_PI,
            n - v, n, threshold);
        }
      }
    }
  }

  if (dual_antenna_enabled_ && dual_antenna_heading_sigma_rad_ > 0.0) {
    // Indices in a 6×6 row-major covariance: [3,3]=roll, [4,4]=pitch,
    // [5,5]=yaw — i.e., 3*6+3=21, 4*6+4=28, 5*6+5=35.
    //
    // P5 gate fix: do NOT clobber a source-reported orientation
    // covariance. When the Odometry source populates rpy covariance
    // (pose_cov[35] > 0), keep it — gnss_global's per-sample yaw-quality
    // gate (orientation_prior_max_yaw_sigma_deg) tests exactly that
    // value, and unconditionally overwriting it with the constant
    // baseline-derived σ² made the gate unable to ever fire (a degraded
    // heading sample looked permanently healthy). Only stamp the
    // baseline-derived constant when the source carried no orientation
    // covariance at all (the PoseStamped path — geometry_msgs/PoseStamped
    // has no covariance field).
    const bool source_has_yaw_cov = pose_cov[35] > 1e-12;
    if (!source_has_yaw_cov) {
      out.pose.covariance[21] = 1.0;   // roll  σ² ≈ 1 rad² (loose)
      out.pose.covariance[28] = 1.0;   // pitch σ² ≈ 1 rad² (loose)
      out.pose.covariance[35] =
        dual_antenna_heading_sigma_rad_ * dual_antenna_heading_sigma_rad_;
    }
  }

  gnss_pose_pub_->publish(out);

  // Offline mapping (glim_rosbag / glim_pcap_rosbag): extension modules
  // never get live subscribers (create_subscriber() only runs in the
  // online path), so hand the accepted sample straight to any extension
  // subscription listening on gnss_factor_topic — this is how
  // libgnss_global.so receives its factors in replay. Online, the
  // publish above reaches the module's live subscriber; inserting here
  // as well would double-count.
  if (!online_mapping_enabled_) {
    static const std::string kBridgeMsgType = "geometry_msgs/msg/PoseWithCovarianceStamped";
    rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serializer;
    rclcpp::SerializedMessage serialized;
    bool serialized_done = false;
    for (const auto& sub : extension_subscriptions()) {
      if (sub->topic != gnss_factor_topic_) continue;
      if (!sub->msg_type.empty() && sub->msg_type != kBridgeMsgType) {
        spdlog::warn(
          "GNSS factor bridge: extension subscription on '{}' expects '{}' but the bridge "
          "publishes '{}' — sample not delivered (check gnss_msg_type in config_gnss_global.json)",
          sub->topic, sub->msg_type, kBridgeMsgType);
        continue;
      }
      if (!serialized_done) {
        serializer.serialize_message(&out, &serialized);
        serialized_done = true;
      }
      sub->insert_message_instance(serialized, kBridgeMsgType);
    }
  }
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
  spdlog::warn("  Elapsed:  {:.1f} s wall (timeout {:.1f} s){}", elapsed, ins_init_timeout_s_,
               online_mapping_enabled_ ? "" : " — offline replay: wall time, not bag time");
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
  spdlog::warn("    4. Relax the gate (degraded init) by editing the");
  spdlog::warn("       \"glim_ros\" section of config_ros.json:");
  spdlog::warn("         \"ins_require_rtk_fixed\": false");
  spdlog::warn("         \"ins_max_position_stddev\": 0.5   // 0.5 m for SBAS-class");
  spdlog::warn("       These are read from config_ros.json, NOT from launch");
  spdlog::warn("       arguments — a launch-time override has no effect.");
  spdlog::warn("       Note: relaxing INVALIDATES the fix to the moving-start");
  spdlog::warn("       pathology; the map will not be reliably gravity-aligned.");
  if (!online_mapping_enabled_) {
    spdlog::warn("    5. A moving start is supported and is NOT the problem:");
    spdlog::warn("       the stability gate tests solution smoothness, not");
    spdlog::warn("       stationarity. Check the reject reason above.");
  }

  if (elapsed > ins_init_timeout_s_) {
    spdlog::warn("");
    if (online_mapping_enabled_) {
      spdlog::warn("{}  TIMEOUT exceeded. Aborting glim_rosnode is now safe.{}", RED, RESET);
      spdlog::warn("{}  (No automatic abort — GLIM will continue to wait if you{}", RED, RESET);
      spdlog::warn("{}  prefer to leave it running until the fix locks.){}", RED, RESET);
    } else {
      spdlog::warn("{}  TIMEOUT exceeded, and this is an OFFLINE replay: the{}", RED, RESET);
      spdlog::warn("{}  gate will not open later unless the bag itself contains{}", RED, RESET);
      spdlog::warn("{}  a passing fix. If it does not, this run will produce an{}", RED, RESET);
      spdlog::warn("{}  EMPTY map (num_submaps: 0 in <dump>/graph.txt) and still{}", RED, RESET);
      spdlog::warn("{}  exit 0. Stop it, fix the input or the gate, and re-run.{}", RED, RESET);
    }
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

  cv_bridge::CvImagePtr cv_image;
  try {
    cv_image = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const std::exception& e) {
    // malformed frame (e.g. truncated capture assembly) -- skip, don't abort.
    // (Port of airacingtech glim_ros2@8b454f8.)
    spdlog::warn("dropping malformed image ({}x{}, {} bytes): {}", msg->width, msg->height, msg->data.size(), e.what());
    return;
  }

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

void GlimROS::aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index) {
  if (aux_index >= aux_concat.aux_sensors.size()) {
    return;
  }
  std::lock_guard<std::mutex> lock(aux_buffers_mutex);
  auto& aux = aux_concat.aux_sensors[aux_index];
  glim_ros::VerticalFovMeasurement fov;
  if (!glim_ros::verticalFovAccepted(
        *msg, aux_concat.lidar_quality.minimum_vertical_fov_deg,
        aux_concat.lidar_quality.minimum_valid_points, &fov)) {
    ++aux.vertical_fov_reject_count;
    if (aux.vertical_fov_reject_count <= 10 ||
        aux.vertical_fov_reject_count % 100 == 0) {
      spdlog::error(
        "lidar_quality: rejecting live aux cloud on {} before buffering: "
        "robust vertical FOV {:.2f} deg (elevation {:.2f}..{:.2f} deg), "
        "required >= {:.2f} deg; valid sampled returns={}/{}. Reason: {}.",
        aux.topic, fov.span_deg, fov.lower_deg, fov.upper_deg,
        aux_concat.lidar_quality.minimum_vertical_fov_deg, fov.valid_points,
        fov.sampled_points, fov.reason);
    }
    return;
  }
  if (!aux.vertical_fov_validated) {
    aux.vertical_fov_validated = true;
    spdlog::info(
      "lidar_quality: live aux vertical-FOV gate passed on {}: robust span "
      "{:.2f} deg (elevation {:.2f}..{:.2f} deg, {} valid sampled returns)",
      aux.topic, fov.span_deg, fov.lower_deg, fov.upper_deg,
      fov.valid_points);
  }

  aux.buffer.push_back(glim_ros::buffer_aux_cloud(msg, aux_concat.float64_time_is_epoch_ns));
  while (aux.buffer.size() > aux.buffer_size) {
    aux.buffer.pop_front();
  }
}

void GlimROS::points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (aux_concat.enabled) {
    spdlog::critical(
      "points_callback_live reached with lidar_concat enabled, but the live path "
      "has no future-sweep release queue; refusing a past-only merge");
    throw std::logic_error(
      "points_callback_live cannot concatenate LiDAR sweeps without a future-sweep release queue");
  }
  points_callback(msg);
}

size_t GlimROS::points_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  int epoch_anchor_count,
  bool* ingested,
  bool raw_sensor_fov_validated) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (ingested) {
    *ingested = false;
  }

  // Concatenated offline clouds have already had every raw sensor checked in
  // merge_clouds(). All other callers must be checked here before extraction.
  if (!raw_sensor_fov_validated) {
    glim_ros::VerticalFovMeasurement fov;
    if (!glim_ros::verticalFovAccepted(
          *msg, aux_concat.lidar_quality.minimum_vertical_fov_deg,
          aux_concat.lidar_quality.minimum_valid_points, &fov)) {
      ++aux_concat.lidar_quality.primary_reject_count;
      if (aux_concat.lidar_quality.primary_reject_count <= 10 ||
          aux_concat.lidar_quality.primary_reject_count % 100 == 0) {
        spdlog::error(
          "lidar_quality: rejecting primary cloud before GLIM: robust "
          "vertical FOV {:.2f} deg (elevation {:.2f}..{:.2f} deg), required "
          ">= {:.2f} deg; valid sampled returns={}/{}. Reason: {}. "
          "A narrowed vertical FOV does not provide enough vertical structure "
          "for stable SLAM.",
          fov.span_deg, fov.lower_deg, fov.upper_deg,
          aux_concat.lidar_quality.minimum_vertical_fov_deg, fov.valid_points,
          fov.sampled_points, fov.reason);
      }
      return 0;
    }
    if (!aux_concat.lidar_quality.primary_validated) {
      aux_concat.lidar_quality.primary_validated = true;
      spdlog::info(
        "lidar_quality: primary vertical-FOV gate passed: robust span {:.2f} "
        "deg (elevation {:.2f}..{:.2f} deg, {} valid sampled returns)",
        fov.span_deg, fov.lower_deg, fov.upper_deg, fov.valid_points);
    }
  }

  if (!GlobalConfig::instance()->has_param("meta", "lidar_frame_id")) {
    spdlog::debug("auto-detecting LiDAR frame ID: {}", msg->header.frame_id);
    GlobalConfig::instance()->override_param<std::string>("meta", "lidar_frame_id", msg->header.frame_id);
  }

  if (!expected_time_field.empty()) {
    const auto field = std::find_if(
      msg->fields.begin(), msg->fields.end(),
      [this](const auto& candidate) {
        return candidate.name == expected_time_field;
      });
    if (field == msg->fields.end() ||
        static_cast<int>(field->datatype) != expected_time_datatype ||
        field->count != 1) {
      spdlog::error(
        "rejecting point cloud: expected timestamp field '{}/datatype={}/count=1'. "
        "For Robin W, install the pinned driver with "
        "PTP_sync/4_setup_lidar_ptp.sh (timestamp/FLOAT64=8).",
        expected_time_field, expected_time_datatype);
      return 0;
    }
  }
  if (expected_time_is_absolute) {
    const auto range =
      glim_ros::decode_point_time_range(*msg, float64_time_is_epoch_ns);
    const double header_s = glim_ros::stamp_to_sec(msg->header.stamp);
    const double first_point_s =
      range.valid ? static_cast<double>(range.min_ns) * 1.0e-9 : 0.0;
    if (!range.valid ||
        (reject_zero_point_times && range.zero_count != 0) ||
        !std::isfinite(header_s) ||
        std::abs(first_point_s - header_s) > 1.0) {
      spdlog::error(
        "rejecting point cloud: '{}' must contain finite absolute epoch times "
        "within 1 s of header.stamp (header={:.9f}, first_point={:.9f}, "
        "zero_timestamps={}). "
        "For Robin W, expect numeric FLOAT64 Unix seconds reconstructed by the "
        "pinned Seyond ROS 2 driver.",
        expected_time_field, header_s, first_point_s, range.zero_count);
      return 0;
    }
  }

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field, epoch_anchor_count, float64_time_is_epoch_ns);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  if (flip_points_y) {
    for (auto& p : raw_points->points) {
      p.y() = -p.y();
    }
  }

  // [P3 FIX 2026-07-14] points_time_offset is now applied inside TimeKeeper
  // (see the constructor), AFTER any absolute-time stamp overwrite, so it is no
  // longer silently discarded for absolute-time clouds.
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
  if (ingested) {
    *ingested = true;
  }

  // Throttle offline bag playback on the SLOWEST stage, not just odometry.
  // glim_rosbag uses this return value to pace playback; reporting only the
  // odometry workload lets a fast front-end drain its queue while the bag keeps
  // flooding sub/global mapping, whose input queues (frames/submaps WITH points)
  // then grow unbounded and OOM. Take the max across all stages so playback
  // waits for the slowest. (Port of airacingtech glim_ros2@8b454f8.)
  size_t workload = odometry_estimation->workload();
  const size_t sub_wl = sub_mapping ? static_cast<size_t>(sub_mapping->workload()) : 0;
  const size_t global_wl = global_mapping ? static_cast<size_t>(global_mapping->workload()) : 0;
  if (sub_wl > workload) workload = sub_wl;
  if (global_wl > workload) workload = global_wl;
  spdlog::debug("workload={} (odom={} sub={} global={})", workload, odometry_estimation->workload(), sub_wl, global_wl);

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

bool GlimROS::ok() const {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      return false;
    }
  }
  return true;
}

void GlimROS::timer_callback() {
  if (!ok()) {
    rclcpp::shutdown();
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

uint64_t GlimROS::gnss_factors_delivered() const {
  // Sum over modules that report confirmed delivery. A module returning
  // nullopt does not contribute global constraints; if NO module reports,
  // the total is 0 and the map cannot be called anchored — which is the
  // point: a missing or failed GNSS extension must not pass as anchored.
  uint64_t total = 0;
  for (const auto& module : extension_modules) {
    if (!module) continue;
    const auto delivered = module->delivered_global_constraints();
    if (delivered) {
      total += *delivered;
    }
  }
  return total;
}

const char* GlimROS::map_anchor_state() const {
  if (!ins_init_applied.load()) return "uninitialized";
  return gnss_factors_delivered() > 0 ? "rtk_anchored" : "rtk_origin_only";
}

void GlimROS::log_map_status() const {
  // End-of-run verdict. LiDAR-IMU SLAM is the primary estimator; GNSS is an
  // optional global constraint, so "no GNSS factors" is a REPORTED STATE,
  // not automatically a failure. Only require_rtk_anchor makes it one.
  const char* state = map_anchor_state();
  const uint64_t delivered = gnss_factors_delivered();
  const int published = gnss_factors_published_.load();
  const int rejected = gnss_factors_rejected_.load();
  spdlog::info(
    "Hitch fork: map status: state={} slam_initialized={} "
    "gnss_factors_delivered={} gnss_bridge_published={} gnss_suppressed={} "
    "require_rtk_anchor={}",
    state, slam_initialized() ? "true" : "false", delivered, published, rejected,
    require_rtk_anchor_ ? "true" : "false");
  if (published > 0 && delivered == 0) {
    spdlog::error(
      "{}  The RTK bridge published {} GNSS pose(s) but NO extension confirmed a single "
      "factor into the graph — is libgnss_global.so loaded, and does its gnss_topic "
      "match gnss_factor_topic? The map is NOT globally anchored.{}",
      RED, published, RESET);
  }

  if (!slam_initialized()) {
    spdlog::error(
      "{}  MAP IS EMPTY: SLAM never initialized — no validated RTK-fixed INS "
      "solution was seen, so no frame was ever ingested.{}", RED, RESET);
    return;
  }
  if (delivered == 0) {
    spdlog::warn(
      "  Map origin is RTK-anchored, but RTK never returned afterwards: the "
      "trajectory past initialization is constrained by LiDAR-IMU only. This "
      "is a legitimate outcome{}",
      require_rtk_anchor_ ? " — but require_rtk_anchor=true, so this run FAILS."
                          : " (set require_rtk_anchor=true to reject it).");
  }
}

void GlimROS::save(const std::string& path) {
  if (global_mapping) {
    // TODO(follow-up refactor): replace this needs_wait() quiescence-inference
    // flush with an explicit ExtensionModule::flush_at_end_of_sequence() hook
    // (a join()-equivalent for extensions, mirroring the core async stages).
    // An EOS signal lets the GNSS backend drain synchronously to completion and
    // resolves the "waiting for more GNSS" vs "permanently un-bracketable"
    // ambiguity directly -- removing pending_associable_ and the timeout below.
    // Ideally upstreamed to koide3 (its extensions share this latent save-race).
    //
    // Flush extension backends (e.g. gnss_global) that produce factors on their
    // own threads and deliver them only via on_smoother_update(). Wait until no
    // extension reports pending work, so their FINAL position/heading factors
    // are queued before we serialize. global_mapping->save() then runs a final
    // optimize() -- which fires on_smoother_update() and injects those queued
    // factors into the graph -- so they actually reach graph.bin / trajectories.
    // Bounded so a perpetually-busy extension can't hang the save.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (needs_wait() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (needs_wait()) {
      spdlog::warn("save(): extension still reports pending work after flush timeout; some final factors may be missing");
    }

    global_mapping->save(path);
  }
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
  // After the extensions have written their own summaries, so the map
  // verdict is the last thing in the log.
  log_map_status();
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);
