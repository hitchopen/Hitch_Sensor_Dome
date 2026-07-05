#include <deque>
#include <cmath>
#include <atomic>
#include <thread>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>  // Hitch Sensor Dome fork: SLERP for orientation interpolation.

#define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/urdf_transforms.hpp>  // parse_urdf_transforms / compute_transform (IMU->GNSS lever arm)

#ifdef GLIM_ROS2
#include <glim/util/extension_module_ros2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS2;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseWithCovarianceStampedConstPtr = geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr;
using Odometry = nav_msgs::msg::Odometry;
using OdometryConstPtr = nav_msgs::msg::Odometry::ConstSharedPtr;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}
#else
#include <glim/util/extension_module_ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS;
#endif

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PoseRotationPrior.h>  // Hitch Sensor Dome fork: optional yaw prior factor.
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/util/config_ext.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

// Hitch Sensor Dome fork — GNSS sample carrying both position and an
// optional orientation. Upstream uses Eigen::Vector4d (stamp, x, y, z)
// for the same purpose; we extend it so a dual-antenna RTK heading can
// flow all the way to a PoseRotationPrior factor on each submap.
//
// has_orientation is set by push_gnss_data() only when the incoming
// quaternion passes the norm sanity check. Bracketing samples without
// orientation cause the SLERP step to skip — a submap touched by a
// half-valid pair is treated as position-only.
struct GNSSData {
  double stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  bool has_orientation = false;
  // P5#1: reported yaw variance (rad^2) from pose.covariance[35].
  // < 0 means "not populated by the publisher" (passes the yaw-quality gate
  // for backwards compatibility with covariance-less GNSS sources).
  double yaw_var = -1.0;
};

/**
 * @brief Naive implementation of GNSS constraints for the global optimization.
 * @note  This implementation was originally translation-only and ignored the IMU-GNSS
 *        transformation and observation covariance. The Hitch Sensor Dome fork adds an
 *        optional rotation prior (enable_orientation_prior) so dual-antenna RTK heading
 *        can constrain yaw alongside position. Lever-arm compensation is still expected
 *        to happen upstream of the topic (the Atlas Duo emits /pose at the IMU origin).
 */
class GNSSGlobal : public ExtensionModuleBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSGlobal() : logger(create_module_logger("gnss_global")) {
    logger->info("initializing GNSS global constraints");
    const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_global");
    logger->info("gnss_global_config_path={}", config_path);

    glim::Config config(config_path);
    gnss_topic = config.param<std::string>("gnss", "gnss_topic", "/pose_with_cov");
    gnss_msg_type = config.param<std::string>("gnss", "gnss_msg_type", "geometry_msgs/msg/PoseWithCovarianceStamped");
    prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "prior_inf_scale", Eigen::Vector3d(1e3, 1e3, 0.0));
    // Hitch Sensor Dome fork note — DUAL-ANTENNA IS THE DEFAULT. The dome
    // ships as a dual-antenna RTK platform, so enable_orientation_prior
    // defaults TRUE here (upstream art-jazzy default is false). Flip to
    // false only on single-antenna installs: there the published heading is
    // gyro-integrated and the factor would tie the optimizer to a heading
    // that itself drifts. Recommended weighting keeps all information on
    // yaw ([1e-6, 1e-6, 1e2]): roll/pitch are IMU+gravity-observed, yaw is
    // the drift-prone axis a dual-antenna baseline constrains directly.
    enable_orientation_prior = config.param<bool>("gnss", "enable_orientation_prior", true);
    orientation_prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "orientation_prior_inf_scale", Eigen::Vector3d(1e-6, 1e-6, 1e2));
    // P5#1: yaw-quality gate for the orientation prior. The RTK gate upstream
    // (adapter/rtk_fixed filter) qualifies POSITION quality only; a
    // position-FIXED sample can still carry a degraded/unsolved dual-antenna
    // heading (secondary-antenna outage, short baseline multipath). Feeding
    // that heading as a stiff PoseRotationPrior would twist the map exactly
    // where we are trying to pin it. Skip the orientation prior when the
    // publisher's reported yaw sigma (sqrt of pose.covariance[35]) exceeds
    // this threshold. <= 0 disables the gate; samples WITHOUT a populated
    // yaw covariance pass (backwards compatible with covariance-less sources).
    orientation_prior_max_yaw_sigma_deg = config.param<double>("gnss", "orientation_prior_max_yaw_sigma_deg", 3.0);
    min_baseline = config.param<double>("gnss", "min_baseline", 5.0);

    if (enable_orientation_prior && orientation_prior_inf_scale.minCoeff() < 0.0) {
      logger->warn("orientation prior enabled but orientation_prior_inf_scale has negative values; disabling orientation prior");
      enable_orientation_prior = false;
    }

    // Resolve IMU -> GNSS antenna lever arm from URDF (if configured).
    // urdf_path / urdf_imu_frame come from config_sensors.json (shared with the lidar/IMU calibration).
    // urdf_gnss_frame is gnss_global-specific (e.g., "gps_antenna_top").
    t_imu_gnss.setZero();
    warned_missing_orientation_for_lever_arm = false;

    // Explicit master switch. Tightly-coupled INS receivers (Novatel SPAN,
    // Septentrio AsteRx-i, Atlas Duo, ...) compensate the antenna->IMU lever
    // arm in firmware via LEVERARMCONFIG; doing it here as well would
    // double-compensate. Default true to preserve upstream raw-GNSS behavior,
    // but the shipped config sets it false for this vehicle.
    const bool enable_lever_arm = config.param<bool>("gnss", "enable_lever_arm", true);
    if (!enable_lever_arm) {
      logger->info("lever-arm compensation explicitly disabled via gnss.enable_lever_arm=false; t_imu_gnss=0");
    }

    try {
      glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
      const std::string urdf_path = config_sensors.param<std::string>("sensors", "urdf_path", "");
      const std::string urdf_imu_frame = config_sensors.param<std::string>("sensors", "urdf_imu_frame", "");
      const std::string urdf_gnss_frame = config.param<std::string>("gnss", "urdf_gnss_frame", "");

      if (enable_lever_arm && !urdf_path.empty() && !urdf_imu_frame.empty() && !urdf_gnss_frame.empty()) {
        const auto urdf_transforms = glim::parse_urdf_transforms(urdf_path);
        const Eigen::Isometry3d T_imu_gnss = glim::compute_transform(urdf_transforms, urdf_imu_frame, urdf_gnss_frame);
        t_imu_gnss = T_imu_gnss.translation();
        logger->info("URDF lever arm t_imu_gnss ({} -> {}): [{:.4f}, {:.4f}, {:.4f}]", urdf_imu_frame, urdf_gnss_frame, t_imu_gnss.x(), t_imu_gnss.y(), t_imu_gnss.z());

        // The lever-arm correction below assumes the GNSS message's orientation
        // is the IMU body's rotation in world. That's true when the antenna
        // frame is axis-aligned with the IMU frame (URDF rotation = identity)
        // OR when the publisher fuses GNSS+IMU and reports IMU-body orientation
        // directly (typical INS like Novatel SPAN, Septentrio AsteRx-i). When
        // the URDF rotation is non-identity AND the publisher reports
        // antenna-frame orientation, the lever arm is applied in the wrong
        // frame. Warn so a future URDF tweak doesn't silently produce a bias.
        const Eigen::Matrix3d R_imu_gnss = T_imu_gnss.linear();
        if (!R_imu_gnss.isApprox(Eigen::Matrix3d::Identity(), 1e-3)) {
          const double off_deg = Eigen::AngleAxisd(R_imu_gnss).angle() * 180.0 / M_PI;
          logger->warn(
            "URDF rotation between {} and {} is {:.2f} deg off identity; lever-arm "
            "correction is correct only if the GNSS publisher reports IMU-body "
            "orientation in world (e.g., an INS). Antenna-frame orientation will "
            "be biased.",
            urdf_imu_frame, urdf_gnss_frame, off_deg);
        }
      } else if (enable_lever_arm) {
        logger->info("URDF lever arm not configured (urdf_path/urdf_imu_frame/urdf_gnss_frame); GNSS positions used as-is");
      }
    } catch (const std::exception& e) {
      logger->error("failed to compute t_imu_gnss from URDF: {}; lever arm compensation disabled", e.what());
      t_imu_gnss.setZero();
    }
    logger->info("enable_orientation_prior={} max_yaw_sigma_deg={}",
                 enable_orientation_prior, orientation_prior_max_yaw_sigma_deg);

    transformation_initialized = false;
    T_world_utm.setIdentity();

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
  }
  ~GNSSGlobal() {
    kill_switch = true;
    thread.join();
  }

  virtual void at_exit(const std::string& dump_path) override {
    if (transformation_initialized) {
      save_transformation_to_file(dump_path);
    }
  }

  // Report pending work so GlimROS::save() drains us before the final global
  // optimize. The backend produces position/heading factors on its own thread
  // and delivers them only through on_smoother_update(); if save() ran while we
  // still had undelivered factors they would never reach the serialized graph.
  // We are NOT done while: a batch is mid-process (processing_); submaps or GNSS
  // are still queued for us (input_*_queue -- the latter closes the bag-EOF race
  // where the GNSS that brackets the last submap hasn't been drained yet); or a
  // submap in our local queue is still bracketable by available GNSS
  // (pending_associable_). Un-bracketable trailing submaps are excluded so we
  // don't block save() on factors that can never be produced.
  virtual bool needs_wait() const override {
    return processing_ || !input_submap_queue.empty() || !input_gnss_queue.empty() || pending_associable_;
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    if (gnss_msg_type == "nav_msgs/msg/Odometry") {
      const auto sub = std::make_shared<TopicSubscription<Odometry>>(gnss_topic, gnss_msg_type, [this](const OdometryConstPtr msg) { gnss_callback(msg); });
      return {sub};
    }

    const auto sub = std::make_shared<TopicSubscription<PoseWithCovarianceStamped>>(
      gnss_topic,
      gnss_msg_type,
      [this](const PoseWithCovarianceStampedConstPtr msg) { gnss_callback(msg); });
    return {sub};
  }

  void gnss_callback(const PoseWithCovarianceStampedConstPtr& gnss_msg) {
    const auto& pos = gnss_msg->pose.pose.position;
    const auto& ori = gnss_msg->pose.pose.orientation;
    const double yaw_var = gnss_msg->pose.covariance[35] > 0.0 ? gnss_msg->pose.covariance[35] : -1.0;
    push_gnss_data(to_sec(gnss_msg->header.stamp), pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, yaw_var);
  }

  void gnss_callback(const OdometryConstPtr& gnss_msg) {
    const auto& pos = gnss_msg->pose.pose.position;
    const auto& ori = gnss_msg->pose.pose.orientation;
    const double yaw_var = gnss_msg->pose.covariance[35] > 0.0 ? gnss_msg->pose.covariance[35] : -1.0;
    push_gnss_data(to_sec(gnss_msg->header.stamp), pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, yaw_var);
  }

  void on_insert_submap(const SubMap::ConstPtr& submap) { input_submap_queue.push_back(submap); }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      logger->debug("insert {} GNSS prior factors", factors.size());
      new_factors.add(factors);
    }
  }

  void backend_task() {
    logger->info("starting GNSS global thread");
    std::deque<GNSSData> utm_queue;
    std::deque<SubMap::ConstPtr> submap_queue;

    while (!kill_switch) {
      // Bound the loop rate so re-attempting association on every GNSS arrival
      // (below) cannot busy-spin while a submap waits to be bracketed.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Mark busy BEFORE draining the input queues: needs_wait() must never
      // observe processing_ == false while a drained-but-unfactored batch is
      // in flight, or save() can slip through the guard mid-association.
      // Cleared below once this cycle either idles out or finishes factoring.
      processing_ = true;

      // Convert GeoPoint(lat/lon) to UTM
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      utm_queue.insert(utm_queue.end(), gnss_data.begin(), gnss_data.end());

      // Add new submaps to the local queue.
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

      // Attempt association whenever there is a pending submap AND something new
      // arrived this cycle: new submaps to place, OR new GNSS that may have just
      // bracketed a submap already waiting in submap_queue. The old code skipped
      // the pass whenever new_submaps was empty, which stranded the last submap
      // at bag EOF -- its bracketing GNSS arrives with no accompanying submap.
      if (submap_queue.empty() || (gnss_data.empty() && new_submaps.empty())) {
        pending_associable_ = !submap_queue.empty() && !utm_queue.empty() &&
                              submap_queue.front()->frames.back()->stamp < utm_queue.back().stamp;
        processing_ = false;
        continue;
      }
      // (processing_ already true — set before the queue drain above; it
      // stays true until this batch is associated + factored.)

      // Remove submaps that are created earlier than the oldest GNSS data
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp < utm_queue.front().stamp) {
        submap_queue.pop_front();
      }

      // Interpolate UTM coords and associate with submaps
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp > utm_queue.front().stamp &&
             submap_queue.front()->frames.back()->stamp < utm_queue.back().stamp) {
        const auto& submap = submap_queue.front();
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto right = std::lower_bound(utm_queue.begin(), utm_queue.end(), stamp, [](const GNSSData& utm, const double t) { return utm.stamp < t; });
        if (right == utm_queue.end() || (right + 1) == utm_queue.end()) {
          logger->warn("invalid condition in GNSS global module!!");
          break;
        }
        const auto left = right - 1;
        logger->debug("submap={:.6f} utm_left={:.6f} utm_right={:.6f}", stamp, left->stamp, right->stamp);

        const double tl = left->stamp;
        const double tr = right->stamp;
        const double p = (stamp - tl) / (tr - tl);

        GNSSData interpolated;
        interpolated.stamp = stamp;
        interpolated.position = (1.0 - p) * left->position + p * right->position;
        // SLERP orientation only when BOTH bracketing samples carry it.
        // Mixing a valid quaternion with the identity would silently
        // bias the interpolated heading, so we drop orientation for
        // half-valid pairs instead.
        if (left->has_orientation && right->has_orientation) {
          interpolated.orientation = left->orientation.slerp(p, right->orientation);
          interpolated.has_orientation = true;
        }
        // P5#1 (ported): carry yaw variance through interpolation — conservative
        // max of the bracketing samples; unknown (<0) if either is unpopulated.
        interpolated.yaw_var = (left->yaw_var > 0.0 && right->yaw_var > 0.0)
            ? std::max(left->yaw_var, right->yaw_var) : -1.0;

        submaps.push_back(submap);
        submap_coords.push_back(interpolated);

        submap_queue.pop_front();
        utm_queue.erase(utm_queue.begin(), left);
      }

      // Initialize T_world_utm using position-only alignment. Orientation
      // can't help us solve for the unknown rotation between two frames
      // here — we have no orientation in the world frame yet to align
      // against. Once T_world_utm is locked, the GNSS quaternion is
      // transformed via T_world_utm.linear() into the world frame.
      if (!transformation_initialized && !submaps.empty() && (submaps.front()->T_world_origin.inverse() * submaps.back()->T_world_origin).translation().norm() > min_baseline) {
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          mean_est += submaps[i]->T_world_origin.translation();
          mean_gnss += submap_coords[i].position;
        }
        mean_est /= submaps.size();
        mean_gnss /= submaps.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d centered_est = submaps[i]->T_world_origin.translation() - mean_est;
          const Eigen::Vector3d centered_gnss = submap_coords[i].position - mean_gnss;
          cov += centered_gnss * centered_est.transpose();
        }
        cov /= submaps.size();

        const Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix2d U = svd.matrixU();
        const Eigen::Matrix2d V = svd.matrixV();
        const Eigen::Matrix2d D = svd.singularValues().asDiagonal();
        Eigen::Matrix2d S = Eigen::Matrix2d::Identity();

        const double det = U.determinant() * V.determinant();
        if (det < 0.0) {
          S(1, 1) = -1;
        }

        Eigen::Isometry3d T_utm_world = Eigen::Isometry3d::Identity();
        T_utm_world.linear().block<2, 2>(0, 0) = U * S * V.transpose();
        T_utm_world.translation() = mean_gnss - T_utm_world.linear() * mean_est;

        T_world_utm = T_utm_world.inverse();

        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d gnss = T_world_utm * submap_coords[i].position;
          logger->debug("submap={} gnss={}", convert_to_string(submaps[i]->T_world_origin.translation().eval()), convert_to_string(gnss));
        }

        logger->info("T_world_utm={}", convert_to_string(T_world_utm));
        transformation_initialized = true;
      }

      // Add GNSS prior factors for EVERY associated submap that doesn't have
      // them yet, not just submaps.back(). The association loop above banks all
      // eligible submaps (offline replay associates many per cycle), and the
      // whole backlog accumulated before T_world_utm initialized is still
      // unfactored at the moment it does. Emitting only for .back() silently
      // skips position + yaw priors for all but the newest submap in a batch
      // (including most pre-baseline submaps). Backfill from the cursor instead.
      if (transformation_initialized) {
        for (size_t i = factored_submap_count; i < submaps.size(); i++) {
          const GNSSData& gnss = submap_coords[i];
          const auto& submap = submaps[i];
          const Eigen::Vector3d xyz = T_world_utm * gnss.position;
          logger->debug("submap={} gnss={}", convert_to_string(submap->T_world_origin.translation().eval()), convert_to_string(xyz));

          // note: should use a more accurate information matrix
          const auto model = gtsam::noiseModel::Diagonal::Precisions(prior_inf_scale);
          output_factors.push_back(
            gtsam::NonlinearFactor::shared_ptr(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model)));

          // P5#1 yaw-quality gate: skip the heading prior when the publisher
          // reports a degraded yaw solution (dual-antenna heading can be bad
          // while position is RTK-FIXED — the upstream RTK filter qualifies
          // position only). Unpopulated covariance (yaw_var < 0) passes.
          const double max_yaw_sigma_rad = orientation_prior_max_yaw_sigma_deg * M_PI / 180.0;
          const bool yaw_quality_ok =
            orientation_prior_max_yaw_sigma_deg <= 0.0 || gnss.yaw_var < 0.0 ||
            gnss.yaw_var <= max_yaw_sigma_rad * max_yaw_sigma_rad;

          if (enable_orientation_prior && gnss.has_orientation && yaw_quality_ok) {
            const Eigen::Matrix3d R_world_gnss = T_world_utm.linear() * gnss.orientation.toRotationMatrix();
            const auto rotation_model = gtsam::noiseModel::Diagonal::Precisions(orientation_prior_inf_scale);
            output_factors.push_back(
              gtsam::NonlinearFactor::shared_ptr(new gtsam::PoseRotationPrior<gtsam::Pose3>(X(submap->id), gtsam::Rot3(R_world_gnss), rotation_model)));
          } else if (enable_orientation_prior && gnss.has_orientation && !yaw_quality_ok) {
            ++yaw_gate_skip_count;
            if (yaw_gate_skip_count == 1 || yaw_gate_skip_count % 50 == 0) {
              logger->warn(
                "orientation prior skipped for submap {}: reported yaw sigma {:.2f} deg > max {:.2f} deg "
                "({} skipped so far) — position prior still applied",
                submap->id, std::sqrt(gnss.yaw_var) * 180.0 / M_PI,
                orientation_prior_max_yaw_sigma_deg, yaw_gate_skip_count);
            }
          } else if (enable_orientation_prior && !warned_missing_orientation) {
            logger->warn("orientation prior enabled but GNSS messages contain invalid quaternions; skipping orientation priors");
            warned_missing_orientation = true;
          }
        }
        factored_submap_count = submaps.size();
      }

      // Pending state for needs_wait(): an associable submap still remains while
      // the front of submap_queue has a GNSS sample after it (so it can be
      // bracketed/interpolated). Trailing submaps with NO GNSS after them are
      // genuinely un-factorable, so they are NOT counted -- save() must not block
      // waiting on them.
      pending_associable_ = !submap_queue.empty() && !utm_queue.empty() &&
                            submap_queue.front()->frames.back()->stamp < utm_queue.back().stamp;
      processing_ = false;
    }
  }

private:
  void save_transformation_to_file(const std::string& dump_path) {
    const std::string filename = dump_path + "/T_world_utm.txt";
    std::ofstream ofs(filename);

    if (!ofs.is_open()) {
      logger->error("failed to open file for writing: {}", filename);
      return;
    }

    ofs << "# SE(3) Transformation from GNSS/UTM to Odom (World) Frame\n";
    ofs << "# This transformation aligns GNSS coordinates with GLIM's world frame\n";
    ofs << "# Format: 4x4 homogeneous transformation matrix\n";
    ofs << "T_world_utm:\n";

    const Eigen::Matrix4d mat = T_world_utm.matrix();
    ofs << std::fixed << std::setprecision(10);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        ofs << std::setw(15) << mat(i, j);
        if (j < 3) {
          ofs << " ";
        }
      }
      ofs << "\n";
    }

    logger->info("saved T_world_utm (4x4 SE(3)) to: {}", filename);
  }

  void push_gnss_data(double stamp, double x, double y, double z, double qx, double qy, double qz, double qw, double yaw_var = -1.0) {
    GNSSData gnss_data;
    gnss_data.stamp = stamp;
    gnss_data.position << x, y, z;
    gnss_data.yaw_var = yaw_var;

    Eigen::Quaterniond orientation(qw, qx, qy, qz);
    if (orientation.coeffs().allFinite() && orientation.norm() > 1e-6) {
      orientation.normalize();
      gnss_data.orientation = orientation;
      gnss_data.has_orientation = true;
    } else {
      gnss_data.has_orientation = false;
    }

    // IMU -> GNSS antenna lever-arm application (art-jazzy parity — this
    // was computed in the constructor but never applied in the original
    // port). Re-points the antenna-located position onto the IMU/body
    // point: p_imu = p_antenna - R_world_imu * t_imu_gnss, where the
    // message orientation is assumed to be the IMU body's rotation in
    // world (see the constructor's non-identity-rotation warning). The
    // shipped default keeps t_imu_gnss = 0 (gnss.enable_lever_arm=false —
    // Atlas outputs its pose at the antenna and the graph body frame
    // matches), making this a no-op. A sample without a valid orientation
    // cannot be corrected; warn once and leave it antenna-located.
    if (t_imu_gnss.squaredNorm() > 1e-12) {
      if (gnss_data.has_orientation) {
        gnss_data.position -= gnss_data.orientation * t_imu_gnss;
      } else if (!warned_missing_orientation_for_lever_arm) {
        logger->warn(
          "lever arm configured (|t_imu_gnss|={:.3f} m) but GNSS sample carries no valid "
          "orientation; position left at the antenna (warning once)",
          t_imu_gnss.norm());
        warned_missing_orientation_for_lever_arm = true;
      }
    }

    input_gnss_queue.push_back(gnss_data);
  }

  // (interpolate_gnss_data() was removed as dead code — the association loop
  // in backend_task() interpolates inline with identical semantics.)

  std::atomic_bool kill_switch;
  // True while the backend is actively associating/factoring a batch of submaps.
  std::atomic_bool processing_{false};
  // True while a submap waiting in the backend's local submap_queue can still be
  // bracketed by available GNSS (i.e. its prior factors are not yet produced).
  // Lets needs_wait() block save() until that submap is factored, without
  // blocking on un-bracketable trailing submaps.
  std::atomic_bool pending_associable_{false};
  std::thread thread;

  ConcurrentVector<GNSSData> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<GNSSData, Eigen::aligned_allocator<GNSSData>> submap_coords;
  // Number of associated submaps that have already had GNSS prior factors
  // emitted. Everything in [factored_submap_count, submaps.size()) still needs
  // factors -- this backfills the pre-T_world_utm backlog and every submap in a
  // multi-submap offline batch, not just submaps.back().
  size_t factored_submap_count = 0;

  std::string gnss_topic;
  std::string gnss_msg_type;
  Eigen::Vector3d prior_inf_scale;
  // Optional yaw-prior knobs (see the constructor comments for semantics and
  // the dual-antenna gating; defaults differ from upstream art-jazzy).
  bool enable_orientation_prior;
  Eigen::Vector3d orientation_prior_inf_scale;
  double orientation_prior_max_yaw_sigma_deg;  // P5#1 yaw-quality gate (<=0 disables)
  size_t yaw_gate_skip_count = 0;              // heading priors skipped by the gate
  double min_baseline;

  // IMU -> GNSS antenna lever arm resolved from URDF (zero when
  // gnss.enable_lever_arm=false or the URDF frames are not configured).
  Eigen::Vector3d t_imu_gnss = Eigen::Vector3d::Zero();
  bool warned_missing_orientation = false;
  bool warned_missing_orientation_for_lever_arm = false;

  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}
