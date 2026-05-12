#include <deque>
#include <atomic>
#include <thread>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>  // Hitch Sensor Dome fork: SLERP for orientation interpolation.

#define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>

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
    min_baseline = config.param<double>("gnss", "min_baseline", 5.0);

    // Hitch Sensor Dome fork — optional GNSS yaw prior.
    //
    // When enable_orientation_prior is true AND the incoming GNSS pose
    // carries a meaningful quaternion (validated in push_gnss_data),
    // each accepted submap gets a PoseRotationPrior factor with
    // information matrix diag = orientation_prior_inf_scale (roll, pitch, yaw).
    //
    // The recommended weighting puts almost all the information on yaw,
    // e.g. [1e-6, 1e-6, 1e2]. The reasons:
    //   - Roll and pitch are well-observed from IMU+gravity; GNSS adds
    //     no real information to those axes.
    //   - Yaw is the drift-prone axis and is the one a dual-antenna RTK
    //     receiver can constrain directly via baseline measurement.
    //
    // DUAL-ANTENNA IS THE DEFAULT. The Hitch Sensor Dome ships as a
    // dual-antenna RTK platform, so this flag defaults to TRUE. The
    // operator is expected to flip it to FALSE only when running on a
    // single-antenna installation. In a single-antenna setup the
    // orientation in the published topic is gyro-integrated (or copied
    // from the INS heading estimate, which is also gyro-integrated),
    // and enabling the factor would tie the optimizer to a heading
    // that is itself drifting — worse than leaving yaw under LiDAR +
    // IMU control alone.
    enable_orientation_prior = config.param<bool>("gnss", "enable_orientation_prior", true);
    orientation_prior_inf_scale = config.param<Eigen::Vector3d>(
      "gnss", "orientation_prior_inf_scale", Eigen::Vector3d(1e-6, 1e-6, 1e2));
    logger->info(
      "enable_orientation_prior={} orientation_prior_inf_scale=({}, {}, {})",
      enable_orientation_prior,
      orientation_prior_inf_scale[0],
      orientation_prior_inf_scale[1],
      orientation_prior_inf_scale[2]);

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
    push_gnss_data(to_sec(gnss_msg->header.stamp),
                   pos.x, pos.y, pos.z,
                   ori.x, ori.y, ori.z, ori.w);
  }

  void gnss_callback(const OdometryConstPtr& gnss_msg) {
    const auto& pos = gnss_msg->pose.pose.position;
    const auto& ori = gnss_msg->pose.pose.orientation;
    push_gnss_data(to_sec(gnss_msg->header.stamp),
                   pos.x, pos.y, pos.z,
                   ori.x, ori.y, ori.z, ori.w);
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
      // Drain new GNSS samples into the local queue.
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      utm_queue.insert(utm_queue.end(), gnss_data.begin(), gnss_data.end());

      // Add new submaps
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      if (new_submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

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

      // Add prior factors for the latest associated submap.
      if (transformation_initialized) {
        const auto& latest = submap_coords.back();
        const Eigen::Vector3d xyz = T_world_utm * latest.position;
        logger->debug("submap={} gnss={}", convert_to_string(submaps.back()->T_world_origin.translation().eval()), convert_to_string(xyz));

        const auto& submap = submaps.back();
        // Translation prior — unchanged from upstream.
        // note: should use a more accurate information matrix
        const auto trans_model = gtsam::noiseModel::Isotropic::Information(prior_inf_scale.asDiagonal());
        gtsam::NonlinearFactor::shared_ptr trans_factor(
          new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, trans_model));
        output_factors.push_back(trans_factor);

        // Hitch Sensor Dome fork — optional rotation prior.
        //
        // Insert a PoseRotationPrior factor on this submap when
        //   (a) enable_orientation_prior is true (operator-controlled
        //       config flag — set false by default for single-antenna
        //       safety; see header at the top of this file), AND
        //   (b) the interpolated GNSS sample carries a valid
        //       quaternion (both bracketing samples had has_orientation;
        //       upstream bridge populates this only when the source
        //       publishes a meaningful heading, which on the Hitch dome
        //       means dual_antenna_enabled is true).
        //
        // The factor pulls submap orientation toward the GNSS heading
        // composed through T_world_utm into the world frame:
        //     R_world_imu = R_world_utm · R_utm_imu
        // Information matrix is diagonal in (roll, pitch, yaw); for the
        // recommended weighting only yaw is meaningfully constrained.
        if (enable_orientation_prior && latest.has_orientation) {
          const gtsam::Rot3 R_world_imu(
            T_world_utm.linear() * latest.orientation.toRotationMatrix());
          const auto rot_model = gtsam::noiseModel::Diagonal::Information(
            orientation_prior_inf_scale.asDiagonal());
          gtsam::NonlinearFactor::shared_ptr rot_factor(
            new gtsam::PoseRotationPrior<gtsam::Pose3>(X(submap->id), R_world_imu, rot_model));
          output_factors.push_back(rot_factor);
        }
      }
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

  // Hitch Sensor Dome fork — push_gnss_data accepts an optional
  // quaternion. The four quaternion args default to zero so the
  // upstream-style call site (position only) still compiles and
  // behaves identically.
  //
  // Quaternion validation: we accept the orientation only when the
  // quaternion norm is in [~1e-3, ~1.5]. Values outside this window
  // are treated as "missing" (has_orientation=false). This catches:
  //   - all-zero quaternions (the geometry_msgs default when a
  //     publisher forgot to set orientation),
  //   - NaNs (norm becomes NaN, fails both bounds),
  //   - severely denormalized data.
  // We normalize before storing so downstream SLERP / Rot3 sees a
  // unit quaternion.
  void push_gnss_data(double stamp, double x, double y, double z,
                      double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 0.0) {
    GNSSData gnss_data;
    gnss_data.stamp = stamp;
    gnss_data.position = Eigen::Vector3d(x, y, z);
    const double qnorm2 = qx * qx + qy * qy + qz * qz + qw * qw;
    if (qnorm2 > 1e-6 && qnorm2 < 2.25 /* (1.5)^2 */) {
      gnss_data.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
      gnss_data.has_orientation = true;
    } else {
      gnss_data.has_orientation = false;
    }
    input_gnss_queue.push_back(gnss_data);
  }

  std::atomic_bool kill_switch;
  std::thread thread;

  ConcurrentVector<GNSSData> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<GNSSData> submap_coords;

  std::string gnss_topic;
  std::string gnss_msg_type;
  Eigen::Vector3d prior_inf_scale;
  double min_baseline;

  // Hitch Sensor Dome fork — optional yaw-prior knobs (see comments
  // in the constructor for semantics and the dual-antenna gating).
  bool enable_orientation_prior;
  Eigen::Vector3d orientation_prior_inf_scale;

  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}
