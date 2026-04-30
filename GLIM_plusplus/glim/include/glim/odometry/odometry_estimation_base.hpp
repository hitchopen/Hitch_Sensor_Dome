#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef GLIM_USE_OPENCV
#include <opencv2/core.hpp>
#endif
#include <glim/odometry/estimation_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Odometry estimation base class
 *
 */
class OdometryEstimationBase {
public:
  OdometryEstimationBase();
  virtual ~OdometryEstimationBase() {}

  /**
   * @brief Returns true if the odometry estimation module requires IMU data
   */
  virtual bool requires_imu() const { return true; }

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU data
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Inject an externally-supplied initial state (Hitch Sensor Dome fork).
   *
   * GLIM normally derives the world-frame orientation from the
   * accelerometer mean over the first few seconds of IMU data, which
   * fails when the bag starts with the vehicle in motion. In this fork
   * the ROS wrapper subscribes to an INS pose topic (Atlas Duo /pose by
   * default) and calls this method once per session, before the first
   * frame is processed. Subclasses that support runtime initialization
   * (e.g. OdometryEstimationIMU via NaiveInitialStateEstimation) override
   * this to forward the values; the default no-op is safe for subclasses
   * that don't need it (e.g. CT-ICP odometry without IMU).
   *
   * @param T_world_imu  IMU pose in the world frame (gravity-aligned).
   * @param v_world_imu  IMU velocity in the world frame (m/s).
   */
  virtual void set_init_state(
    const Eigen::Isometry3d& T_world_imu,
    const Eigen::Vector3d& v_world_imu) {
    (void)T_world_imu;
    (void)v_world_imu;
  }

  /**
   * @brief Insert a point cloud
   * @param frame                       Preprocessed point cloud
   * @param marginalized_states         [out] Marginalized estimation frames
   * @return EstimationFrame::ConstPtr  Estimation result for the latest frame
   */
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states);

  /**
   * @brief Pop out the remaining non-marginalized frames (called at the end of the sequence)
   * @return std::vector<EstimationFrame::ConstPtr>
   */
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() { return std::vector<EstimationFrame::ConstPtr>(); }

  /**
   * @brief Load an odometry estimation module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded odometry estimation module
   */
  static std::shared_ptr<OdometryEstimationBase> load_module(const std::string& so_name);

protected:
  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim