#include <glim/odometry/initial_state_estimation.hpp>

#include <iostream>
#include <spdlog/spdlog.h>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

namespace glim {

InitialStateEstimation::InitialStateEstimation() : logger(create_module_logger("odom")) {}

NaiveInitialStateEstimation::NaiveInitialStateEstimation(const Eigen::Isometry3d& T_lidar_imu, const Eigen::Matrix<double, 6, 1>& imu_bias)
: ready(false),
  init_stamp(0.0),
  stamp(0.0),
  sum_acc(Eigen::Vector3d::Zero()),
  imu_bias(imu_bias),
  T_lidar_imu(T_lidar_imu),
  force_init(false),
  init_T_world_imu(Eigen::Isometry3d::Identity()),
  init_v_world_imu(Eigen::Vector3d::Zero()) {
  glim::Config config(glim::GlobalConfig::get_config_path("config_odometry"));
  window_size = config.param("odometry_estimation", "initialization_window_size", 1.0);
}

NaiveInitialStateEstimation::~NaiveInitialStateEstimation() {}

void NaiveInitialStateEstimation ::set_init_state(const Eigen::Isometry3d& init_T_world_imu, const Eigen::Vector3d& init_v_world_imu) {
  force_init = true;
  this->init_T_world_imu = init_T_world_imu;
  this->init_v_world_imu = init_v_world_imu;

  this->init_T_world_imu.linear() = Eigen::Quaterniond(this->init_T_world_imu.linear()).normalized().toRotationMatrix();
}

void NaiveInitialStateEstimation::insert_imu(double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  // Hitch Sensor Dome fork: gravity is no longer derived from the
  // accelerometer mean. The vehicle may be moving when the bag starts
  // (race-track replays, mid-session restarts), and integrating
  // linear_acc into a "gravity estimate" produces a tilted world frame
  // that breaks downstream loop closure (see GLIM/docs/
  // moving_start_initialization.md). Instead, the ROS wrapper
  // subscribes to the Atlas Duo INS /pose topic and calls
  // set_init_state() with the gravity-resolved orientation before the
  // first frame is processed.
  //
  // We still track the most recent IMU stamp so the EstimationFrame
  // emitted by initial_pose() carries a sensible timestamp. The
  // sensor-range warning is preserved as a sanity check on bad input.
  if (linear_acc.norm() < 5.0 || linear_acc.norm() > 15.0) {
    logger->warn("too large or small acc found ({}[m/s^2])", linear_acc.norm());
  }
  this->stamp = stamp;
  if (init_stamp < 1e-6) {
    init_stamp = stamp;
  }
  // sum_acc / window_size / "ready" are intentionally not advanced.
  // They remain as members for ABI continuity with the upstream header
  // but no code path consumes them in this fork.
  (void)angular_vel;
}

EstimationFrame::ConstPtr NaiveInitialStateEstimation::initial_pose() {
  // External init is mandatory in this fork. Until set_init_state() has
  // been called by the ROS wrapper (with the Atlas Duo INS pose), block
  // the optimizer by returning nullptr — the caller treats nullptr as
  // "not ready yet" and re-tries on the next frame.
  if (!force_init) {
    return nullptr;
  }

  EstimationFrame::Ptr estimated(new EstimationFrame);
  estimated->id = -1;
  estimated->stamp = stamp;
  estimated->T_lidar_imu = T_lidar_imu;
  estimated->imu_bias = imu_bias;
  estimated->v_world_imu = init_v_world_imu;
  estimated->T_world_imu = init_T_world_imu;
  estimated->T_world_lidar = estimated->T_world_imu * T_lidar_imu.inverse();

  logger->info(
    "initial IMU state pinned from external INS source: "
    "T_world_imu translation=[{:.3f}, {:.3f}, {:.3f}], "
    "v_world_imu=[{:.3f}, {:.3f}, {:.3f}]",
    estimated->T_world_imu.translation().x(),
    estimated->T_world_imu.translation().y(),
    estimated->T_world_imu.translation().z(),
    estimated->v_world_imu.x(),
    estimated->v_world_imu.y(),
    estimated->v_world_imu.z());

  return estimated;
}

}  // namespace glim