/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_localization/localization.h"
#include "gicp_localization/lidar_fov_gate.hpp"
#include "gicp_localization/map_manifest.hpp"
#include "gicp_localization/seyond_timestamp.hpp"
#include "gicp_localization/urdf_transforms.hpp"
#include "dlio/utils.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/random_sample.h>   // Hitch Sensor Dome: GICP warm-start
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace {

constexpr double kRadToDeg = 57.29577951308232;
using gicp_localization::SeyondPointTimeRange;
using gicp_localization::decodeSeyondPointTimeRange;
using gicp_localization::kSeyondFrameDurationSecondsDefault;
using gicp_localization::pointTimeEndpointDelta;
using gicp_localization::seyondCloudTimeContractValid;

bool matrixFinite(const Eigen::Matrix4f& pose) {
  return pose.array().isFinite().all();
}

geometry_msgs::msg::Pose poseMsgFromMatrix(const Eigen::Matrix4f& pose) {
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();

  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

geometry_msgs::msg::PoseStamped poseStampedFromMatrix(const Eigen::Matrix4f& pose,
                                                      const rclcpp::Time& stamp,
                                                      const std::string& frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose = poseMsgFromMatrix(pose);
  return msg;
}

double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  Eigen::Quaternionf qa(a.block<3, 3>(0, 0));
  Eigen::Quaternionf qb(b.block<3, 3>(0, 0));
  qa.normalize();
  qb.normalize();

  Eigen::Quaternionf dq = qa.conjugate() * qb;
  dq.normalize();
  const double w = std::clamp(std::abs(static_cast<double>(dq.w())), 0.0, 1.0);
  return 2.0 * std::acos(w) * kRadToDeg;
}

double deltaTranslationNorm(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to) {
  return (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).norm();
}

std::string poseSummary(const Eigen::Matrix4f& pose) {
  if (!matrixFinite(pose)) {
    return "invalid";
  }

  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  const Eigen::Vector3f rpy_deg = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * static_cast<float>(kRadToDeg);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "xyz=[" << t.x() << "," << t.y() << "," << t.z() << "]"
      << " rpy_deg=[" << rpy_deg.x() << "," << rpy_deg.y() << "," << rpy_deg.z() << "]";
  return oss.str();
}

std::string scalarSummary(double value, int precision = 3) {
  if (!std::isfinite(value)) {
    return "nan";
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

double hessianConditionProxy(const Eigen::Matrix<double, 6, 6>& hessian) {
  if (!hessian.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Matrix<double, 6, 6> sym_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(sym_hessian);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  const auto abs_eigenvalues = solver.eigenvalues().cwiseAbs();
  const double max_eigenvalue = abs_eigenvalues.maxCoeff();
  const double min_eigenvalue = abs_eigenvalues.minCoeff();

  // A structural zero is the strongest possible degeneracy signal. Excluding
  // zero eigenvalues from the denominator would turn a rank-five Hessian into
  // an apparently well-conditioned one and disable the partial-update guard.
  if (!std::isfinite(max_eigenvalue) || max_eigenvalue <= 0.0 ||
      min_eigenvalue <= 1e-12) {
    return std::numeric_limits<double>::infinity();
  }

  return max_eigenvalue / min_eigenvalue;
}

// ---------------------------------------------------------------------------
// P1 gating rework: degeneracy-aware partial update ("solution remapping",
// Zhang & Singh ICRA'16). Instead of binary-rejecting a scan whose hessian is
// ill-conditioned, project the GICP correction onto the well-constrained
// eigen-subspace and keep the IMU prior along the degenerate directions.
//
// Frame handling: small_gicp's final hessian is parameterized as [omega; t]
// with the rotation taken about the WORLD ORIGIN (jacobian block is
// skew(transformed_point)). With the vehicle ~hundreds of metres from the map
// origin, a yaw about the origin is numerically indistinguishable from a
// translation, so the raw rotation block mostly measures lever-arm effects.
// We therefore re-center the hessian about the vehicle position c first:
// with new variables [omega; t_hat], t_hat = t + omega x c, the substitution
// x = A x_hat, A = [[I,0],[skew(c),I]] gives H_c = A^T H A whose rotation
// block measures rotations ABOUT THE VEHICLE. Conveniently t_hat is, to first
// order, exactly candidate_p - prior_p, so the projected translation applies
// directly to the pose difference.
// ---------------------------------------------------------------------------
struct DegeneracyProjection {
  bool valid{false};          // analysis ran (hessian finite, eigensolver ok)
  bool modified{false};       // projected pose differs from raw candidate
  bool fully_degenerate{false};  // all 6 axes degenerate -> caller should reject
  bool yaw_vetoed{false};     // yaw-consistency veto zeroed the yaw correction
  int degen_rot_axes{0};
  int degen_trans_axes{0};
  Eigen::Matrix4f projected_pose = Eigen::Matrix4f::Identity();
};

DegeneracyProjection projectDegenerateDelta(const Eigen::Matrix<double, 6, 6>& hessian,
                                            const Eigen::Matrix4f& T_prior,
                                            const Eigen::Matrix4f& candidate,
                                            bool apply_eigen_projection,
                                            bool full6d,
                                            double coupling_length_m,
                                            double rel_floor_6d,
                                            double rel_floor_rot,
                                            double rel_floor_trans,
                                            double veto_yaw_above_deg /* <=0 disables */) {
  DegeneracyProjection out;
  out.projected_pose = candidate;
  // A non-finite hessian only invalidates the EIGEN projection — the yaw veto
  // works on the pose delta alone and must not be silently disabled by it
  // (review fix). Callers treat valid=false as "reject when eigen projection
  // was required", which is still the right contract below.
  if (apply_eigen_projection && !hessian.allFinite()) return out;

  const Eigen::Matrix4d prior = T_prior.cast<double>();
  const Eigen::Matrix4d cand = candidate.cast<double>();
  const Eigen::Matrix3d R_prior = prior.block<3, 3>(0, 0);
  const Eigen::Matrix3d R_cand = cand.block<3, 3>(0, 0);
  const Eigen::Vector3d p_prior = prior.block<3, 1>(0, 3);
  const Eigen::Vector3d p_cand = cand.block<3, 1>(0, 3);

  // World-frame delta: candidate = T_delta * prior.
  const Eigen::Matrix3d R_delta = R_cand * R_prior.transpose();
  Eigen::AngleAxisd aa(R_delta);
  Eigen::Vector3d omega = aa.angle() * aa.axis();   // rotation correction (world axes, about vehicle)
  Eigen::Vector3d t_hat = p_cand - p_prior;         // translation correction of the vehicle

  Eigen::Vector3d omega_p = omega;
  Eigen::Vector3d t_hat_p = t_hat;

  if (apply_eigen_projection) {
    // Re-center the hessian about the vehicle position.
    const Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (hessian + hessian.transpose());
    Eigen::Matrix3d skew_c;
    skew_c << 0.0, -p_prior.z(), p_prior.y(),
              p_prior.z(), 0.0, -p_prior.x(),
             -p_prior.y(), p_prior.x(), 0.0;
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A.block<3, 3>(3, 0) = skew_c;
    const Eigen::Matrix<double, 6, 6> H_c = A.transpose() * H_sym * A;

    if (full6d) {
      // Full 6D solution remapping (Zhang & Singh). Rotation (rad) and
      // translation (m) are incommensurable, so scale rotation coordinates by
      // a characteristic coupling length L first: x_s = [L*omega; t_hat],
      // x = D x_s with D = diag(I/L, I), H_s = D H_c D. L should be the
      // typical constraint lever arm (~point-cloud radius after the 80 m
      // crop); with it, one unit of any scaled coordinate moves constraint
      // points by comparable metres, making the joint spectrum meaningful and
      // COUPLED rot/trans null directions (e.g. slide-along-a-wall = yaw +
      // lateral mix) visible — a blockwise analysis structurally cannot see
      // those.
      const double L = std::max(coupling_length_m, 1e-3);
      Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
      D.block<3, 3>(0, 0) /= L;
      const Eigen::Matrix<double, 6, 6> H_s = D * H_c * D;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H_s);
      if (es.info() != Eigen::Success) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      const Eigen::Matrix<double, 6, 1> evals = es.eigenvalues().cwiseAbs();
      const double lambda_max = evals.maxCoeff();
      Eigen::Matrix<double, 6, 6> P6 = Eigen::Matrix<double, 6, 6>::Zero();
      int n_degen = 0;
      for (int i = 0; i < 6; ++i) {
        const Eigen::Matrix<double, 6, 1> v = es.eigenvectors().col(i);
        if (lambda_max <= 0.0 || evals[i] < rel_floor_6d * lambda_max) {
          ++n_degen;
          // Report which sub-block the degenerate direction mostly lives in
          // (diagnostic only; the projector itself is fully coupled).
          if (v.head<3>().norm() >= v.tail<3>().norm()) {
            ++out.degen_rot_axes;
          } else {
            ++out.degen_trans_axes;
          }
        } else {
          P6 += v * v.transpose();
        }
      }
      if (n_degen == 6) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      Eigen::Matrix<double, 6, 1> dx_s;
      dx_s.head<3>() = L * omega;
      dx_s.tail<3>() = t_hat;
      const Eigen::Matrix<double, 6, 1> dx_s_p = P6 * dx_s;
      omega_p = dx_s_p.head<3>() / L;
      t_hat_p = dx_s_p.tail<3>();
    } else {
      // Blockwise fallback (degeneracy/full6d: false): independent 3x3
      // eigen-analyses of the rot/trans blocks. Unit-mixing-free but blind to
      // coupled rot/trans degeneracy; kept for A/B comparison.
      Eigen::Matrix3d P_rot = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d P_trans = Eigen::Matrix3d::Identity();
      auto blockProjector = [](const Eigen::Matrix3d& block, double rel_floor,
                               Eigen::Matrix3d& projector, int& n_degen) -> bool {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(block);
        if (es.info() != Eigen::Success) return false;
        const Eigen::Vector3d evals = es.eigenvalues().cwiseAbs();
        const double lambda_max = evals.maxCoeff();
        projector.setZero();
        n_degen = 0;
        for (int i = 0; i < 3; ++i) {
          if (lambda_max <= 0.0 || evals[i] < rel_floor * lambda_max) {
            ++n_degen;
          } else {
            const Eigen::Vector3d v = es.eigenvectors().col(i);
            projector += v * v.transpose();
          }
        }
        return true;
      };

      if (!blockProjector(H_c.block<3, 3>(0, 0), rel_floor_rot, P_rot, out.degen_rot_axes) ||
          !blockProjector(H_c.block<3, 3>(3, 3), rel_floor_trans, P_trans, out.degen_trans_axes)) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      if (out.degen_rot_axes == 3 && out.degen_trans_axes == 3) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      omega_p = P_rot * omega;
      t_hat_p = P_trans * t_hat;
    }
  }

  // Turn-aware yaw-consistency veto: T_prior already contains the
  // IMU-integrated yaw across the scan gap, so omega.z() IS the GICP-vs-IMU
  // yaw disagreement. A large disagreement on a low-confidence match is the
  // wrong-basin entry signature; keep the IMU yaw instead.
  constexpr double kRad2Deg = 180.0 / M_PI;
  if (veto_yaw_above_deg > 0.0 && std::abs(omega_p.z()) * kRad2Deg > veto_yaw_above_deg) {
    omega_p.z() = 0.0;
    out.yaw_vetoed = true;
  }

  const double kEps = 1e-12;
  const bool changed = ((omega_p - omega).norm() > kEps) || ((t_hat_p - t_hat).norm() > kEps);
  out.valid = true;
  if (changed) {
    Eigen::Matrix3d R_delta_p = Eigen::Matrix3d::Identity();
    const double angle = omega_p.norm();
    if (angle > kEps) {
      R_delta_p = Eigen::AngleAxisd(angle, omega_p / angle).toRotationMatrix();
    }
    Eigen::Matrix4d projected = Eigen::Matrix4d::Identity();
    projected.block<3, 3>(0, 0) = R_delta_p * R_prior;
    projected.block<3, 1>(0, 3) = p_prior + t_hat_p;
    out.projected_pose = projected.cast<float>();
    out.modified = true;
  }
  return out;
}

size_t pointFieldElementSize(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool pointFieldFitsStep(
    const sensor_msgs::msg::PointField& field, uint32_t point_step) {
  const size_t element_size = pointFieldElementSize(field.datatype);
  if (element_size == 0 || field.count == 0 ||
      static_cast<size_t>(field.count) >
          std::numeric_limits<size_t>::max() / element_size) {
    return false;
  }
  const size_t field_size = element_size * static_cast<size_t>(field.count);
  return static_cast<size_t>(field.offset) <= point_step &&
         field_size <=
             static_cast<size_t>(point_step) -
                 static_cast<size_t>(field.offset);
}

// Find float32 x/y/z field offsets in a PointCloud2 message. Returns false if
// any field is missing, malformed, or extends beyond point_step.
bool findXYZOffsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    const bool valid_coordinate =
        f.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        f.count == 1 && pointFieldFitsStep(f, msg.point_step);
    if (f.name == "x" && valid_coordinate) {
      x_off = static_cast<int>(f.offset);
    } else if (f.name == "y" && valid_coordinate) {
      y_off = static_cast<int>(f.offset);
    } else if (f.name == "z" && valid_coordinate) {
      z_off = static_cast<int>(f.offset);
    }
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Full point-field schema comparison, used to decide whether an auxiliary cloud
// can be byte-appended onto the primary cloud. mergeAuxClouds() transforms aux
// points using the AUX cloud's own field offsets, but the MERGED cloud keeps the
// PRIMARY's `fields`, so every downstream reader interprets the appended aux
// bytes with the primary layout. Appending is therefore only safe when the aux
// layout is byte-identical to the primary: same point_step, same endianness, and
// the same ordered set of field {name, offset, datatype, count}. A same-point_step
// cloud with different offsets/datatypes would otherwise be silently misread.
// This is O(#fields) — negligible next to GICP / voxel
// filtering / deskew, which run in ms. Returns true on match; on mismatch returns
// false and sets `reason` to a short human-readable description for logging.
bool auxSchemaMatchesPrimary(const sensor_msgs::msg::PointCloud2& aux,
                             const sensor_msgs::msg::PointCloud2& primary,
                             std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " +
             std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " +
             std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); ++i) {
    const auto& a = aux.fields[i];
    const auto& p = primary.fields[i];
    if (a.name != p.name || a.offset != p.offset || a.datatype != p.datatype || a.count != p.count) {
      reason = "field[" + std::to_string(i) + "] '" + a.name + "' (offset=" + std::to_string(a.offset) +
               ",datatype=" + std::to_string(a.datatype) + ",count=" + std::to_string(a.count) +
               ") differs from primary '" + p.name + "' (offset=" + std::to_string(p.offset) +
               ",datatype=" + std::to_string(p.datatype) + ",count=" + std::to_string(p.count) + ")";
      return false;
    }
  }
  return true;
}

// Apply rigid transform to xyz of `num_points` points starting at `data` (in place).
// Pointer-based variant — operates on a span within a larger buffer so callers
// can write aux scans directly into the merged cloud's data without an
// intermediate copy.
void transformCloudData(uint8_t* data, size_t num_points, uint32_t point_step,
                        int x_off, int y_off, int z_off,
                        const Eigen::Matrix4f& T) {
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t = T.block<3, 1>(0, 3);
  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* base = data + i * point_step;
    float x, y, z;
    std::memcpy(&x, base + x_off, sizeof(float));
    std::memcpy(&y, base + y_off, sizeof(float));
    std::memcpy(&z, base + z_off, sizeof(float));
    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(base + x_off, &p.x(), sizeof(float));
    std::memcpy(base + y_off, &p.y(), sizeof(float));
    std::memcpy(base + z_off, &p.z(), sizeof(float));
  }
}

bool findTimeField(const sensor_msgs::msg::PointCloud2& msg, int& time_off,
                   uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = static_cast<int>(f.offset);
      time_datatype = f.datatype;
      time_count = static_cast<int>(f.count);
      return true;
    }
  }
  return false;
}

inline void clearPointTimeUnion(PointType& pt) {
  const uint64_t zero = 0;
  std::memcpy(&pt.timestamp, &zero, sizeof(uint64_t));
}

// Copy per-point time from PointCloud2 into the dlio::Point union for the configured sensor.
// `point_step` bounds the field read so a malformed/short time field cannot read past the point.
void copyPointTimeFromCloud(const uint8_t* src, int time_off, uint8_t time_datatype, int time_count,
                           uint32_t point_step, dlio::SensorType sensor, PointType& dst) {
  if (time_off < 0 || static_cast<uint32_t>(time_off) >= point_step) {
    return;
  }
  const uint8_t* tp = src + time_off;
  const size_t bytes_avail = point_step - static_cast<uint32_t>(time_off);

  switch (sensor) {
    case dlio::SensorType::OUSTER: {
      uint32_t t_ns = 0;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::UINT32:
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          break;
        case sensor_msgs::msg::PointField::FLOAT32: {
          float t_s = 0.f;
          std::memcpy(&t_s, tp, sizeof(float));
          t_ns = static_cast<uint32_t>(t_s * 1e9f);
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
          double t_s = 0.;
          std::memcpy(&t_s, tp, sizeof(double));
          t_ns = static_cast<uint32_t>(t_s * 1e9);
          break;
        }
        default:
          break;
      }
      dst.t = t_ns;
      return;
    }
    case dlio::SensorType::VELODYNE: {
      float t_s = 0.f;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::FLOAT32:
          std::memcpy(&t_s, tp, sizeof(float));
          break;
        case sensor_msgs::msg::PointField::UINT32: {
          uint32_t t_ns = 0;
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          t_s = static_cast<float>(t_ns * 1e-9);
          break;
        }
        default:
          break;
      }
      dst.time = t_s;
      return;
    }
    case dlio::SensorType::SEYOND:
    case dlio::SensorType::HESAI: {
      if (time_datatype == sensor_msgs::msg::PointField::FLOAT64 &&
          time_count == 1 && bytes_avail >= sizeof(double)) {
        std::memcpy(&dst.timestamp, tp, sizeof(double));
      }
      return;
    }
    case dlio::SensorType::LIVOX: {
      if (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8) {
        uint64_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint64_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::UINT32) {
        uint32_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint32_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
        std::memcpy(&dst.timestamp, tp, sizeof(double));
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT32) {
        float t_f = 0.f;
        std::memcpy(&t_f, tp, sizeof(float));
        dst.timestamp = static_cast<double>(t_f);
      }
      return;
    }
    default:
      return;
  }
}

// Shift per-point timestamps by `dt` seconds to rebase an aux scan's per-point
// times from its own header.stamp onto the merged cloud's primary header.stamp.
//
// Whether to actually shift depends on the underlying encoding:
//   * SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start,
//     UINT32 nanoseconds-since-scan-start) -> ADD dt so the value reads as
//     "seconds since primary scan start".
//   * ABSOLUTE-EPOCH encodings (including Robin W FLOAT64 Unix seconds) ->
//     DO NOT shift. Each value already identifies the point's capture time on
//     the shared PTP axis; adding the header delta would double-count it.
void shiftCloudTimestamps(uint8_t* data, size_t num_points, uint32_t point_step,
                          int time_off, uint8_t time_datatype, int time_count,
                          double dt, bool absolute_time) {
  if (time_off < 0) return;
  (void)time_count;

  if (absolute_time) {
    (void)dt;
    return;
  }

  // FLOAT64 magnitude gate (parity with GLIM's shift_cloud_timestamps): a
  // FLOAT64 time field can carry scan-relative seconds (shiftable), Hesai
  // absolute epoch seconds, or Livox numeric epoch nanoseconds (both
  // unshifted). Scan all finite, non-zero points so a leading zero sentinel
  // cannot misclassify an absolute cloud as relative.
  if (time_datatype == sensor_msgs::msg::PointField::FLOAT64 && num_points > 0 &&
      static_cast<uint32_t>(time_off) + sizeof(double) <= point_step) {
    constexpr double kMaxRelativeSeconds = 1e6;  // ~11.6 days; real sweeps are < 1 s
    double absolute_sample = 0.0;
    bool looks_absolute = false;
    for (size_t i = 0; i < num_points; ++i) {
      double value = 0.0;
      std::memcpy(&value, data + i * point_step + time_off, sizeof(double));
      if (!std::isfinite(value) || value == 0.0) continue;
      if (std::abs(value) > kMaxRelativeSeconds) {
        absolute_sample = value;
        looks_absolute = true;
        break;
      }
    }
    if (looks_absolute) {
      static bool warned_absolute_f64 = false;
      if (!warned_absolute_f64) {
        RCLCPP_WARN(rclcpp::get_logger("gicp_localization"),
                    "shiftCloudTimestamps: FLOAT64 time field looks ABSOLUTE (sample %.3f); "
                    "leaving unshifted — absolute per-point times need no rebase onto the primary clock",
                    absolute_sample);
        warned_absolute_f64 = true;
      }
      return;
    }
  }

  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* time_ptr = data + i * point_step + time_off;
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        const int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
        val = static_cast<uint32_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint32_t));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float val;
        std::memcpy(&val, time_ptr, sizeof(float));
        val += static_cast<float>(dt);
        std::memcpy(time_ptr, &val, sizeof(float));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double val;
        std::memcpy(&val, time_ptr, sizeof(double));
        val += dt;
        std::memcpy(time_ptr, &val, sizeof(double));
        break;
      }
      case sensor_msgs::msg::PointField::UINT8: {
        // Byte-array timestamp carriers are unsupported in this repository.
        break;
      }
      default:
        break;
    }
  }
}

visualization_msgs::msg::Marker makeArrowMarker(const Eigen::Matrix4f& pose,
                                                const std::string& frame_id,
                                                const rclcpp::Time& stamp,
                                                int id,
                                                const std::string& ns,
                                                float r,
                                                float g,
                                                float b) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseMsgFromMatrix(pose);
  marker.scale.x = 2.0;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0f;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  return marker;
}

}  // namespace

gicp_localization::LocalizationNode::LocalizationNode() : Node("gicp_localization_node") {

  this->getParams();

  // Initialize flags
  this->initialized = false;
  this->first_imu_received = false;

  // Initialize pose
  this->current_pose = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->observer_prior_pose_ = Eigen::Matrix4f::Identity();
  this->last_gicp_pose_ = Eigen::Matrix4f::Identity();
  this->last_gicp_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->last_gicp_valid_ = false;

  // Initialize IMU buffer
  this->imu_buffer.set_capacity(this->imu_buffer_size_);

  // Initialize IMU calibration state
  this->imu_calibrated_ = false;
  this->imu_calib_start_stamp_ = -1.0;
  this->imu_calib_count_ = 0;
  this->imu_calib_gyro_sum_ = Eigen::Vector3f::Zero();
  this->imu_calib_accel_sum_ = Eigen::Vector3f::Zero();
  this->imu_calib_acc_norm_sum_ = 0.0;
  this->imu_calib_acc_norm_sumsq_ = 0.0;
  this->imu_calib_motion_warned_ = false;
  this->imu_calib_attempt_ = 0;

  // Initialize RTK-driven calibration state
  this->init_phase_ = InitPhase::WAITING;
  this->first_imu_stamp_ = -1.0;
  this->rtk_calib_start_stamp_ = -1.0;
  this->rtk_calib_count_ = 0;
  this->rtk_gyro_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_gyro_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->has_prev_gt_for_accel_ = false;
  this->prev_gt_stamp_ = 0.0;
  this->prev_v_world_ = Eigen::Vector3f::Zero();
  this->has_latest_rtk_seed_ = false;

  // Initialize previous scan stamp
  this->prev_scan_stamp = 0.0;
  this->t_prior_stamp_ = 0.0;
  this->observer_dt_ = 0.0;
  this->last_scan_input_frame_.clear();
  this->last_raw_point_count_ = 0;
  this->last_preprocessed_point_count_ = 0;

  // Initialize lidar pose
  this->basePose.p = Eigen::Vector3f::Zero();
  this->basePose.q = Eigen::Quaternionf::Identity();

  // Initialize previous velocity
  this->prev_vel = Eigen::Vector3f::Zero();

  // Initialize geometric observer state
  this->state.p = Eigen::Vector3f::Zero();
  this->state.q = Eigen::Quaternionf::Identity();
  this->state.v.lin.b = Eigen::Vector3f::Zero();
  this->state.v.lin.w = Eigen::Vector3f::Zero();
  this->state.v.ang.b = Eigen::Vector3f::Zero();
  this->state.v.ang.w = Eigen::Vector3f::Zero();
  this->state.b.gyro = Eigen::Vector3f::Zero();
  this->state.b.accel = Eigen::Vector3f::Zero();

  this->geo.first_opt_done = false;
  this->geo.update_seq = 0;
  this->consecutive_failures_ = 0;
  this->gt_extrinsics_cached_ = false;
  this->T_base_gtbody_.setIdentity();
  this->geo.dp = 0.0;
  this->geo.dq_deg = 0.0;
  this->geo.prev_p = Eigen::Vector3f::Zero();
  this->geo.prev_q = Eigen::Quaternionf::Identity();
  this->geo.prev_vel = Eigen::Vector3f::Zero();

  // Initialize sensor type (default to OUSTER, can be configured)
  this->sensor = dlio::SensorType::OUSTER;

  // Initialize extrinsics to identity (should be configured from parameters)
  this->extrinsics.baselink2imu.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2imu.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2lidar.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2lidar.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics_cached_ = false;
  this->imu_extrinsics_cached_ = false;
  this->pending_initial_pose_ = false;

  // Initialize point clouds
  this->map_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  this->map_cloud_ds = std::make_shared<pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  this->original_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Load map
  if (!this->loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Exiting...");
    throw std::runtime_error("Failed to load map");
  }

  // Setup GICP
  this->gicp.setNumThreads(omp_get_max_threads());
  this->gicp.setCorrespondenceRandomness(this->gicp_corr_randomness_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_epsilon_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_epsilon_);
  this->gicp.setDebugPrint(this->debug_lm_print_);

  // Prepare either the legacy full-map target or the bounded local-map index.
  // In both modes target covariances are computed once at startup.
  if (!this->prepareMapTarget()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to prepare the GICP map target");
    throw std::runtime_error("Failed to prepare GICP map target");
  }

  // ---- Hitch Sensor Dome — GICP warm-start ----
  // Even with target side prebuilt, the FIRST real align call still
  // pays first-touch overhead: OpenMP thread-pool spin-up, Eigen
  // kernel JIT warm-up, source-side kd-tree allocation, page-faults
  // through map memory loaded from disk. Run one dummy align here to
  // burn those costs at startup instead of on the first real scan.
  // The dummy source is ~200 points sampled randomly from the map;
  // enough to exercise the parallel-for and NN paths without taking
  // measurable time. Log the wall time so operators can see the
  // warm-up landed cleanly before scans start arriving.
  {
    auto dummy_src = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::RandomSample<PointType> rs;
    rs.setInputCloud(this->map_cloud);
    rs.setSample(std::min<unsigned int>(200, this->map_cloud->size()));
    rs.filter(*dummy_src);

    if (!dummy_src->empty()) {
      this->gicp.setInputSource(dummy_src);
      pcl::PointCloud<PointType> aligned_scratch;
      const auto t0 = std::chrono::steady_clock::now();
      this->gicp.align(aligned_scratch, Eigen::Matrix4f::Identity());
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      RCLCPP_INFO(this->get_logger(),
                  "GICP warm-start: dummy align done in %.1f ms over %zu pts "
                  "— first real scan will run at steady-state speed.",
                  ms, dummy_src->size());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "GICP warm-start: map is empty; skipping warm-up align.");
    }
  }

  // Setup subscribers
  auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  origin_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  origin_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  this->enu_origin_sub = this->create_subscription<std_msgs::msg::String>(
      "enu_origin", origin_qos,
      std::bind(&gicp_localization::LocalizationNode::callbackEnuOrigin,
                this, std::placeholders::_1));

  this->pointcloud_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pointcloud_sub_opt = rclcpp::SubscriptionOptions();
  pointcloud_sub_opt.callback_group = this->pointcloud_cb_group;
  // Use sensor-data QoS so rosbag/sensor publishers with BEST_EFFORT are compatible.
  this->pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&gicp_localization::LocalizationNode::callbackPointCloud, this, std::placeholders::_1),
      pointcloud_sub_opt);

  this->initial_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&gicp_localization::LocalizationNode::callbackInitialPose, this, std::placeholders::_1),
      initial_pose_sub_opt);

  // Aux LiDAR subscribers (multi-LiDAR concatenation). Use a Reentrant group so
  // aux scans can land in parallel with the primary callback and with each
  // other; each aux only writes to its own buffer (mutex-protected), so no
  // shared mutable state is touched here.
  if (this->concat_enabled_ && !this->aux_lidars_.empty()) {
    this->aux_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto aux_sub_opt = rclcpp::SubscriptionOptions();
    aux_sub_opt.callback_group = this->aux_cb_group_;
    std::vector<std::string> resolved_aux_topics;
    resolved_aux_topics.reserve(this->aux_lidars_.size());
    for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
      const std::string topic = this->aux_lidars_[i]->topic;
      const int idx = static_cast<int>(i);
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::SensorDataQoS(),
          [this, idx](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->callbackAuxPointCloud(idx, std::move(msg));
          },
          aux_sub_opt);
      resolved_aux_topics.emplace_back(sub->get_topic_name());
      this->aux_subs_.push_back(sub);
    }
    try {
      validateResolvedLidarTopics(
          this->pointcloud_sub->get_topic_name(), resolved_aux_topics);
    } catch (const std::invalid_argument& e) {
      RCLCPP_FATAL(
          this->get_logger(), "Invalid resolved LiDAR topics: %s", e.what());
      throw;
    }
    for (const auto& topic : resolved_aux_topics) {
      RCLCPP_INFO(
          this->get_logger(), "Subscribed to aux LiDAR topic: %s",
          topic.c_str());
    }
  }

  // MutuallyExclusive so IMU callbacks are serialized among THEMSELVES while
  // still running in parallel with the pointcloud group (parallelism across
  // groups is what the MultiThreadedExecutor provides; a Reentrant group here
  // additionally allowed IMU-vs-IMU concurrency, which raced the stationary/
  // RTK calibration accumulators and the init_phase_ window transitions —
  // calibration is inherently sequential).
  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;

  // Set QoS for IMU subscriber (BEST_EFFORT to match sensor publishers)
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&gicp_localization::LocalizationNode::callbackImu, this, std::placeholders::_1),
      imu_sub_opt);
  const std::string resolved_imu_topic = this->imu_sub->get_topic_name();
  if (this->imu_require_topic_allowlist_) {
    const bool allowed = std::find(this->imu_topic_allowlist_.begin(),
                                   this->imu_topic_allowlist_.end(),
                                   resolved_imu_topic) != this->imu_topic_allowlist_.end();
    if (!allowed) {
      std::ostringstream oss;
      for (size_t i = 0; i < this->imu_topic_allowlist_.size(); ++i) {
        if (i) oss << ", ";
        oss << this->imu_topic_allowlist_[i];
      }
      RCLCPP_FATAL(this->get_logger(),
                   "IMU topic hard guard triggered: resolved topic '%s' is not in allowlist [%s]. "
                   "Expected the fused Point One (Atlas) INS IMU path.",
                   resolved_imu_topic.c_str(), oss.str().c_str());
      throw std::runtime_error("IMU topic hard guard mismatch");
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "IMU input topic: %s (expect Point One (Atlas) INS IMU, frame='%s', strict_frame_match=%s)",
              resolved_imu_topic.c_str(),
              this->imu_frame.c_str(),
              this->imu_require_frame_match_ ? "true" : "false");

  // IMU input health check. The most common silent failure is launching with an
  // `imu_topic:=` arg that doesn't match any publisher — the subscription is
  // created but callbacks never fire and there's nothing in the log to tell
  // the user why. Fire a periodic timer that warns when no IMU has been
  // received AND no publisher exists on the resolved topic name. The timer
  // cancels itself once the first IMU arrives.
  this->input_health_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() {
        if (this->first_imu_received.load()) {
          this->input_health_timer_->cancel();
          return;
        }
        const std::string imu_topic = this->imu_sub->get_topic_name();
        const size_t pub_count = this->count_publishers(imu_topic);
        if (pub_count == 0) {
          RCLCPP_WARN(this->get_logger(),
                      "No IMU received on '%s' (0 publishers). Check the "
                      "imu_topic launch arg. The Hitch Sensor Dome default is "
                      "'/imu/data' (low-latency Atlas driver). Run "
                      "`ros2 topic list | grep -i imu` "
                      "to see available IMU topics.",
                      imu_topic.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "No IMU received on '%s' yet, but %zu publisher(s) exist. "
                      "QoS mismatch or sim-time/clock issue is possible.",
                      imu_topic.c_str(), pub_count);
        }
      });

  // Optional ground-truth odom subscriber for divergence cross-check.
  // Topic is remappable as "gt_odom"; the dome launch defaults it to
  // /gps_p1/filtered_odom_rtk_fixed (the adapter's fixed-only output). The GT
  // body frame is taken from msg->child_frame_id and
  // composed into base_frame via the cached TF (see composeGtPoseInBase).
  if (this->gt_odom_enabled_) {
    // MutuallyExclusive: serializes GT callbacks against each other so the
    // first-message extrinsic cache and the one-shot odom-init block
    // (use_odom_init_applied_) cannot run twice on two concurrent messages;
    // still parallel with the pointcloud/IMU groups.
    this->gt_odom_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto gt_sub_opt = rclcpp::SubscriptionOptions();
    gt_sub_opt.callback_group = this->gt_odom_cb_group;
    auto gt_qos = rclcpp::QoS(rclcpp::KeepLast(this->gt_odom_buffer_size_));
    gt_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    gt_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    this->gt_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "gt_odom", gt_qos,
        std::bind(&gicp_localization::LocalizationNode::callbackGtOdom, this, std::placeholders::_1),
        gt_sub_opt);
    RCLCPP_INFO(this->get_logger(),
                "Ground-truth odom cross-check ENABLED (topic remap 'gt_odom', buffer=%zu, max_dt=%.3fs)",
                this->gt_odom_buffer_size_, this->gt_odom_max_dt_);

    // Hitch Sensor Dome — one-shot warning when gt_odom never arrives.
    // The localizer's RTK-driven init and GT-snap recovery paths both
    // require messages on this topic. On the Hitch dome, the message
    // source is the adapter fixed-only stream, which stops when the solution is
    // float or invalid. After 10 s with zero arrivals, emit a
    // bold-yellow one-shot warning to surface a likely misconfiguration.
    this->gt_odom_health_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        [this]() {
          // Self-cancel on first invocation regardless of outcome —
          // this is a one-shot.
          if (this->gt_odom_health_timer_) {
            this->gt_odom_health_timer_->cancel();
          }
          if (this->gt_odom_received_.load()) {
            // Healthy state — confirm in a single INFO line.
            RCLCPP_INFO(this->get_logger(),
                        "gt_odom health check PASSED — messages arriving on "
                        "the remap 'gt_odom'.");
            return;
          }
          const std::string topic = this->gt_odom_sub ?
              this->gt_odom_sub->get_topic_name() : std::string("(unknown)");
          const size_t pub_count = this->count_publishers(topic);
          const char* YELLOW = "\033[1;33m";
          const char* RESET  = "\033[0m";
          RCLCPP_WARN(this->get_logger(),
            "%sgt_odom health check FAILED — no messages received on "
            "'%s' after 10 s.%s",
            YELLOW, topic.c_str(), RESET);
          RCLCPP_WARN(this->get_logger(),
            "%s  publishers on that topic: %zu%s", YELLOW, pub_count, RESET);
          if (pub_count == 0) {
            RCLCPP_WARN(this->get_logger(),
              "%s  Likely cause: the adapter is not running, OR the "
              "gt_odom_topic launch arg points at a topic no node publishes. "
              "Verify `/gps_p1/filtered_odom_rtk_fixed` and the adapter.%s",
              YELLOW, RESET);
          } else {
            RCLCPP_WARN(this->get_logger(),
              "%s  Publishers exist but no messages have arrived. The adapter "
              "has not observed a genuine RTK_FIXED solution (convergence, "
              "corrections, or sky view). Inspect the adapter summary before "
              "relying on rtk_init or gt_recovery.%s",
              YELLOW, RESET);
          }
          RCLCPP_WARN(this->get_logger(),
            "%s  The localizer is running but the rtk_init / gt_recovery "
            "paths are inert until messages start arriving.%s",
            YELLOW, RESET);
        });
  }

  // RTK quality gate is now self-contained in callbackGtOdom: it inspects
  // msg->pose.covariance on each gt_odom sample (no separate subscription).
  // Log the current configuration so the operator can see what's active.
  if (this->gt_odom_enabled_) {
    if (this->rtk_gate_enabled_) {
      RCLCPP_INFO(this->get_logger(),
                  "RTK quality gate ENABLED (P1-native): drop gt_odom when "
                  "pose covariance exceeds max_pose_var_xy=%.3f m^2 or "
                  "max_pose_var_z=%.3f m^2.",
                  this->rtk_gate_max_pose_var_xy_,
                  this->rtk_gate_max_pose_var_z_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "RTK quality gate DISABLED. gt_odom samples will be "
                  "accepted regardless of Atlas-reported pose covariance "
                  "-- snap and init may seed state from degraded GNSS. "
                  "Enable localization/rtk_gate/enable to reject samples "
                  "with poor covariance.");
    }
  }

  // Setup publishers
  this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("localized_pose", 10);

  // Note: Pose is published at IMU rate from propagateState() for perfect time synchronization
  RCLCPP_INFO(this->get_logger(), "Pose will be published at IMU rate (~100 Hz) from propagateState()");

  // High-frequency odometry publisher (100Hz from IMU propagation)
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1000));  // Large queue for 100Hz
  odom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  this->localized_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("localized_odom", odom_qos);

  if (this->utm_enabled_) {
    this->utm_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/pose_utm", 10);
    this->utm_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("gicp/localization/odom_utm", odom_qos);
    this->utm_path_pub = this->create_publisher<nav_msgs::msg::Path>("gicp/localization/path_utm", 10);
  }

  if (this->debug_pub_enabled_) {
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("localized_path", 10);
    this->gt_snap_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/gt_snap", 10);
    this->dbg_initial_guess_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/debug/initial_guess_pose", 10);
    this->dbg_final_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("gicp/localization/debug/final_pose", 10);
    this->dbg_pose_markers_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("gicp/localization/debug/pose_markers", 10);
    this->dbg_fitness_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/fitness", 10);
    this->dbg_gicp_elapsed_ms_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gicp_elapsed_ms", 10);
    this->dbg_corr_norm_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/corr_norm", 10);
    this->dbg_scan_dt_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_dt", 10);
    this->dbg_imu_age_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/imu_age", 10);
    this->dbg_num_correspondences_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/num_correspondences", 10);
    this->dbg_correspondence_ratio_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/correspondence_ratio", 10);
    this->dbg_final_error_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/final_error", 10);
    this->dbg_guess_to_solution_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_to_solution_trans_m", 10);
    this->dbg_guess_to_solution_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_to_solution_rot_deg", 10);
    this->dbg_guess_from_last_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_from_last_m", 10);
    this->dbg_guess_from_last_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/guess_from_last_deg", 10);
    this->dbg_raw_points_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/raw_points", 10);
    this->dbg_preprocessed_points_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/preprocessed_points", 10);
    this->dbg_imu_buffer_span_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/imu_buffer_span_s", 10);
    this->dbg_scan_to_latest_imu_lag_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_to_latest_imu_lag_s", 10);
    this->dbg_hessian_condition_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/hessian_condition_proxy", 10);
    this->dbg_jump_trans_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_trans", 10);
    this->dbg_jump_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_rot_deg", 10);
    this->dbg_converged_pub = this->create_publisher<std_msgs::msg::Bool>("gicp/localization/debug/converged", 10);
    this->dbg_gt_pos_err_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gt_pos_err_m", 10);
    this->dbg_gt_rot_deg_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/gt_rot_err_deg", 10);
    // P1 gating rework diagnostics
    this->dbg_fitness_ratio_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/fitness_ratio", 10);
    this->dbg_degen_rot_axes_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/degen_rot_axes", 10);
    this->dbg_degen_trans_axes_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/degen_trans_axes", 10);
    this->dbg_yaw_veto_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/yaw_veto", 10);
    // P4#3: per-frame lidar_concat diagnostics (one sample per processed frame).
    this->dbg_merged_aux_count_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/merged_aux_count", 10);
    this->dbg_scan_time_span_pub =
        this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_time_span_s", 10);
    for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
      this->dbg_aux_dt_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(
          "gicp/localization/debug/aux" + std::to_string(i) + "_merge_dt_s", 10));
      this->dbg_aux_points_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(
          "gicp/localization/debug/aux" + std::to_string(i) + "_points", 10));
    }
  }

  if (this->visualize_map_) {
    this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  }

  // TF broadcaster
  if (this->publish_tf_) {
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // TF buffer and listener for transforming incoming point clouds
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

  RCLCPP_INFO(this->get_logger(), "DLIO Localization Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Map loaded with %lu points", this->map_cloud->points.size());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->applyInitialPoseFromParams();
}

gicp_localization::LocalizationNode::~LocalizationNode() {
  ++this->local_map_generation_;
  if (this->local_map_rebuild_thread_.joinable()) {
    this->local_map_rebuild_thread_.join();
  }
}

bool gicp_localization::LocalizationNode::loadUTMTransform(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open UTM transform file: %s", path.c_str());
    return false;
  }
  Eigen::Matrix4f T_world_utm = Eigen::Matrix4f::Identity();
  std::string line;
  int row = 0;
  while (std::getline(f, line) && row < 4) {
    if (line.empty() || line[0] == '#' || line.find("T_world_utm") != std::string::npos) continue;
    std::istringstream ss(line);
    for (int col = 0; col < 4; ++col) ss >> T_world_utm(row, col);
    ++row;
  }
  if (row < 4) {
    RCLCPP_ERROR(this->get_logger(), "UTM transform file malformed (only %d rows parsed): %s", row, path.c_str());
    return false;
  }
  this->T_utm_map_ = T_world_utm.inverse();
  RCLCPP_INFO(this->get_logger(), "Loaded UTM transform from %s (T_utm_map origin: [%.2f, %.2f, %.2f])",
    path.c_str(),
    this->T_utm_map_(0, 3), this->T_utm_map_(1, 3), this->T_utm_map_(2, 3));
  return true;
}

void gicp_localization::LocalizationNode::getParams() {

  // Frame IDs
  this->declare_parameter<std::string>("localization/map_frame", "map");
  this->declare_parameter<std::string>("localization/base_frame", "base_link");
  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<std::string>("localization/imu_frame", "imu");
  this->declare_parameter<std::string>("localization/lidar_frame", "lidar");
  // Static base_frame<-lidar_frame lever arm (row-major 4x4) used to resolve the
  // extrinsic WITHOUT live TF in offline replay. Empty = rely on URDF
  // (lidar_concat/urdf_path) then live TF. See resolveBaseLidarExtrinsicOffline().
  this->declare_parameter<std::vector<double>>("localization/base_lidar_transform", std::vector<double>{});

  this->get_parameter("localization/map_frame", this->map_frame);
  this->get_parameter("localization/base_frame", this->base_frame);
  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("localization/imu_frame", this->imu_frame);
  this->get_parameter("localization/lidar_frame", this->lidar_frame);
  this->get_parameter("localization/base_lidar_transform", this->base_lidar_static_);

  // Map parameters
  this->declare_parameter<std::string>("localization/map_path", "");
  this->declare_parameter<std::string>("localization/map_manifest_path", "");
  this->declare_parameter<std::string>("localization/expected_enu_origin", "");
  this->declare_parameter<bool>("localization/require_map_manifest", true);
  this->declare_parameter<bool>("localization/require_live_enu_origin", true);
  this->declare_parameter<double>("localization/enu_origin_tolerance_m", 0.25);
  this->declare_parameter<std::string>("localization/utm_transform_path", "");
  this->declare_parameter<std::string>("localization/utm_frame", "utm");
  this->declare_parameter<bool>("localization/visualize_map", true);
  this->declare_parameter<double>("localization/map_voxel_size_vis", 0.5);
  // Voxel leaf size (m) for the GICP TARGET map / kd-tree. A dense map (e.g. a
  // 49M-point GLIM export) builds a huge kd-tree -> >10 GiB RSS and swap thrash
  // that stalls registration. Downsampling the target to ~0.3 m cuts memory and
  // per-scan search cost with negligible accuracy loss at 0.5 m scan voxels.
  // 0.0 disables (use the full-resolution map).
  this->declare_parameter<double>("localization/map_voxel_size", 0.3);
  this->declare_parameter<bool>("localization/local_map/enabled", true);
  this->declare_parameter<double>("localization/local_map/radius", 150.0);
  this->declare_parameter<int>("localization/local_map/min_points", 1000);
  this->declare_parameter<int>("localization/local_map/build_threads", 1);
  this->declare_parameter<double>("localization/map_rotation/roll_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/pitch_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/yaw_deg", 0.0);

  this->get_parameter("localization/map_path", this->map_path_);
  this->get_parameter("localization/map_manifest_path", this->map_manifest_path_);
  this->get_parameter("localization/expected_enu_origin", this->expected_enu_origin_);
  this->get_parameter("localization/require_map_manifest", this->require_map_manifest_);
  this->get_parameter("localization/require_live_enu_origin",
                      this->require_live_enu_origin_);
  this->get_parameter("localization/enu_origin_tolerance_m",
                      this->enu_origin_tolerance_m_);
  if (!std::isfinite(this->enu_origin_tolerance_m_) ||
      this->enu_origin_tolerance_m_ <= 0.0) {
    throw std::invalid_argument(
        "localization/enu_origin_tolerance_m must be finite and positive");
  }

  std::string utm_transform_path;
  this->get_parameter("localization/utm_transform_path", utm_transform_path);
  this->get_parameter("localization/utm_frame", this->utm_frame);
  this->utm_enabled_ = false;
  this->T_utm_map_ = Eigen::Matrix4f::Identity();
  if (!utm_transform_path.empty()) {
    throw std::invalid_argument(
        "localization/utm_transform_path is incompatible with the canonical "
        "ENU map export; leave it empty because inverse(T_world_utm) was "
        "already applied by export_glim_dump_to_pcd.py");
  }
  this->get_parameter("localization/visualize_map", this->visualize_map_);
  this->get_parameter("localization/map_voxel_size_vis", this->map_voxel_size_vis_);
  this->get_parameter("localization/map_voxel_size", this->map_voxel_size_);
  this->get_parameter("localization/local_map/enabled", this->local_map_enable_);
  this->get_parameter("localization/local_map/radius", this->local_map_radius_);
  int local_map_min_points = 1000;
  this->get_parameter("localization/local_map/min_points", local_map_min_points);
  this->get_parameter(
      "localization/local_map/build_threads", this->local_map_build_threads_);
  this->local_map_min_points_ =
      static_cast<size_t>(std::max(1, local_map_min_points));
  this->local_map_build_threads_ = std::max(1, this->local_map_build_threads_);
  this->get_parameter("localization/map_rotation/roll_deg", this->map_roll_deg_);
  this->get_parameter("localization/map_rotation/pitch_deg", this->map_pitch_deg_);
  this->get_parameter("localization/map_rotation/yaw_deg", this->map_yaw_deg_);

  // Localization parameters
  this->declare_parameter<bool>("localization/publish_tf", true);
  this->declare_parameter<bool>("localization/imu_only", false);
  this->declare_parameter<bool>("localization/use_odom_init", true);
  this->declare_parameter<bool>("localization/initial_pose/use", false);
  this->declare_parameter<std::string>("localization/initial_pose/frame", "lidar");
  this->declare_parameter<double>("localization/initial_pose/x", 0.0);
  this->declare_parameter<double>("localization/initial_pose/y", 0.0);
  this->declare_parameter<double>("localization/initial_pose/z", 0.0);
  this->declare_parameter<double>("localization/initial_pose/roll", 0.0);
  this->declare_parameter<double>("localization/initial_pose/pitch", 0.0);
  this->declare_parameter<double>("localization/initial_pose/yaw", 0.0);

  // Ground-truth odom cross-check (optional). Uses topic remap "gt_odom".
  this->declare_parameter<bool>("localization/gt_odom/enable", false);
  this->declare_parameter<int>("localization/gt_odom/buffer_size", 200);
  this->declare_parameter<double>("localization/gt_odom/max_dt", 0.1);
  bool gt_enable = false; int gt_buf = 200; double gt_max_dt = 0.1;
  this->get_parameter("localization/gt_odom/enable", gt_enable);
  this->get_parameter("localization/gt_odom/buffer_size", gt_buf);
  this->get_parameter("localization/gt_odom/max_dt", gt_max_dt);
  this->gt_odom_enabled_ = gt_enable;
  this->gt_odom_buffer_size_ = static_cast<size_t>(std::max(gt_buf, 1));
  this->gt_odom_max_dt_ = gt_max_dt;

  // Defense-in-depth covariance gate for the adapter's fixed-only odometry.
  // These defaults match adapter/config/adapter.yaml.
  this->declare_parameter<bool>("localization/rtk_gate/enable", true);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_xy", 0.001);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_z", 0.005);
  this->get_parameter("localization/rtk_gate/enable",
                      this->rtk_gate_enabled_);
  this->get_parameter("localization/rtk_gate/max_pose_var_xy",
                      this->rtk_gate_max_pose_var_xy_);
  this->get_parameter("localization/rtk_gate/max_pose_var_z",
                      this->rtk_gate_max_pose_var_z_);
  if (!std::isfinite(this->rtk_gate_max_pose_var_xy_) ||
      !std::isfinite(this->rtk_gate_max_pose_var_z_) ||
      this->rtk_gate_max_pose_var_xy_ <= 0.0 ||
      this->rtk_gate_max_pose_var_z_ <= 0.0) {
    throw std::invalid_argument(
        "localization/rtk_gate covariance limits must be finite and positive");
  }

  // GT-driven pose recovery (optional). Independent of gt_odom/enable; recovery
  // requires the same subscriber to be active, so it implies gt_odom/enable.
  this->declare_parameter<bool>("localization/gt_recovery/enable", false);
  // Default matches cfg/localization.yaml (P2#3: raised from 1; per-frame
  // snapping masked dead-reckoning quality). Keep the two in sync.
  this->declare_parameter<int>("localization/gt_recovery/min_consecutive_failures", 5);
  this->declare_parameter<double>(
      "localization/gt_recovery/sanity_radius", 10.0);
  this->get_parameter("localization/gt_recovery/enable", this->gt_recovery_enabled_);
  this->get_parameter("localization/gt_recovery/min_consecutive_failures",
                      this->gt_recovery_min_consecutive_failures_);
  this->get_parameter(
      "localization/gt_recovery/sanity_radius",
      this->gt_recovery_sanity_radius_);
  if (this->gt_recovery_enabled_ && !this->gt_odom_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_recovery/enable=true but gt_odom/enable=false — forcing gt_odom on so the buffer fills.");
    this->gt_odom_enabled_ = true;
  }
  if (this->gt_recovery_min_consecutive_failures_ < 1) {
    this->gt_recovery_min_consecutive_failures_ = 1;
  }
  if (!std::isfinite(this->gt_recovery_sanity_radius_) ||
      this->gt_recovery_sanity_radius_ < 0.0) {
    throw std::invalid_argument(
        "localization/gt_recovery/sanity_radius must be finite and "
        "non-negative");
  }

  this->get_parameter("localization/publish_tf", this->publish_tf_);
  this->get_parameter("localization/imu_only", this->imu_only_mode_);
  this->get_parameter("localization/use_odom_init", this->use_odom_init_);
  this->get_parameter("localization/initial_pose/use", this->use_param_initial_pose_);
  this->get_parameter("localization/initial_pose/frame", this->initial_pose_frame_);
  this->get_parameter("localization/initial_pose/x", this->initial_pose_x_);
  this->get_parameter("localization/initial_pose/y", this->initial_pose_y_);
  this->get_parameter("localization/initial_pose/z", this->initial_pose_z_);
  this->get_parameter("localization/initial_pose/roll", this->initial_pose_roll_);
  this->get_parameter("localization/initial_pose/pitch", this->initial_pose_pitch_);
  this->get_parameter("localization/initial_pose/yaw", this->initial_pose_yaw_);

  // GICP parameters
  this->declare_parameter<int>("gicp/maxIterations", 32);
  this->declare_parameter<int>("gicp/correspondenceRandomness", 20);
  this->declare_parameter<double>("gicp/maxCorrespondenceDistance", 1.0);
  this->declare_parameter<double>("gicp/transformationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/rotationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/fitnessRejectThreshold", 1.0);
  this->declare_parameter<double>("gicp/minCorrespondenceRatio", 0.50);
  this->declare_parameter<bool>("gicp/rejectLargeJumps", true);
  // Reject scans whose Hessian condition number proxy exceeds this threshold.
  // Straights typically run ~1e4; feature-poor corners spike to 1e8-1e9 and the
  // optimizer slides along the unconstrained axis. Set <=0 to disable the gate.
  this->declare_parameter<double>("gicp/hessianCondMax", 1.0e6);
  // Hessian rejection fires when condition number is high AND any of:
  //   - fitness exceeds the warn floor below
  //   - GICP applied a translation correction larger than transWarn
  //   - GICP applied a rotation correction larger than rotWarn
  // The latter two catch the optimizer sliding along an unconstrained axis: in
  // degenerate geometry IMU already provides a good prior, so a healthy GICP
  // correction is small. A large correction in degenerate geometry is the
  // slide signature even when fitness looks fine. Set any threshold <= 0 to
  // disable that specific OR branch.
  this->declare_parameter<double>("gicp/hessianFitnessWarnThreshold", 0.15);
  this->declare_parameter<double>("gicp/hessianTransWarnM", 1.0);
  this->declare_parameter<double>("gicp/hessianRotWarnDeg", 1.5);

  // P1 gating rework (docs/action_plan_turn_error_20260704.md).
  // Rolling-median fitness baseline: absolute fitness thresholds calibrated on
  // a same-run map (floor 0.03-0.06) are meaningless on cross-run maps (floor
  // ~0.27), so gates operate on fitness / rolling-median instead.
  this->declare_parameter<bool>("gicp/fitnessBaseline/enable", true);
  this->declare_parameter<int>("gicp/fitnessBaseline/window", 201);
  this->declare_parameter<int>("gicp/fitnessBaseline/minSamples", 50);
  // Warm-up seed: expected per-map fitness floor used as the baseline until
  // minSamples accepted frames exist, so ratio gates / yaw veto are live from
  // frame 1 (0 = off; gates absolute-only during warm-up, pre-review behavior).
  this->declare_parameter<double>("gicp/fitnessBaseline/seedBaseline", 0.0);
  // Wrong-basin (bad-accept) gate: run-12 data shows good accepts at ratio
  // median 1.00 / p99 1.92, bad accepts (gt_err>20m) at median 1.34 / p90 2.55.
  this->declare_parameter<double>("gicp/fitnessRatioRejectThreshold", 2.0);
  // Degeneracy partial update: when the condition proxy crosses hessianCondMax,
  // project the correction instead of rejecting the whole scan (the old binary
  // reject produced 253-frame dead-reckoning streaks on run 12).
  this->declare_parameter<bool>("gicp/degeneracy/partialUpdate", true);
  // Full 6D coupled remapping (default) vs independent 3x3 rot/trans blocks.
  // full6d sees COUPLED rot/trans null directions (slide-along-a-wall = yaw +
  // lateral mix) that a blockwise analysis structurally cannot; rotation
  // coordinates are made commensurable with translation via couplingLengthM
  // (the typical constraint lever arm, ~point-cloud radius after cropping).
  this->declare_parameter<bool>("gicp/degeneracy/full6d", true);
  this->declare_parameter<double>("gicp/degeneracy/couplingLengthM", 20.0);
  this->declare_parameter<double>("gicp/degeneracy/relFloor6d", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorRot", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorTrans", 0.02);
  // Turn-aware yaw-consistency veto (Codex finding 1).
  this->declare_parameter<bool>("gicp/yawGate/enable", true);
  this->declare_parameter<double>("gicp/yawGate/maxCorrDeg", 1.5);
  this->declare_parameter<double>("gicp/yawGate/fitnessRatio", 1.2);
  // small_gicp can constrain attitude increments inside LM. Keep 6-DoF as the
  // migration default so changing the backend does not also change vehicle
  // dynamics without a Hitch replay. 4-DoF fixes roll/pitch; 3-DoF also fixes
  // yaw. Soft prior information is disabled at zero.
  this->declare_parameter<std::string>("gicp/dof/mode", "6dof");
  this->declare_parameter<int>("gicp/dof/full6dofEveryN", 0);
  this->declare_parameter<double>("gicp/prior/yawInfo", 0.0);
  this->declare_parameter<double>("gicp/prior/rollPitchInfo", 0.0);

  this->get_parameter("gicp/maxIterations", this->gicp_max_iter_);
  this->get_parameter("gicp/correspondenceRandomness", this->gicp_corr_randomness_);
  this->get_parameter("gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_);
  this->get_parameter("gicp/transformationEpsilon", this->gicp_transformation_epsilon_);
  this->get_parameter("gicp/rotationEpsilon", this->gicp_rotation_epsilon_);
  this->get_parameter("gicp/fitnessRejectThreshold", this->gicp_fitness_reject_threshold_);
  this->get_parameter(
      "gicp/minCorrespondenceRatio",
      this->gicp_min_correspondence_ratio_);
  this->get_parameter("gicp/rejectLargeJumps", this->gicp_reject_large_jumps_);
  this->get_parameter("gicp/hessianCondMax", this->gicp_hessian_cond_max_);
  this->get_parameter("gicp/hessianFitnessWarnThreshold", this->gicp_hessian_fitness_warn_);
  this->get_parameter("gicp/hessianTransWarnM", this->gicp_hessian_trans_warn_m_);
  this->get_parameter("gicp/hessianRotWarnDeg", this->gicp_hessian_rot_warn_deg_);
  this->get_parameter("gicp/fitnessBaseline/enable", this->fitness_baseline_enable_);
  this->get_parameter("gicp/fitnessBaseline/window", this->fitness_baseline_window_);
  this->get_parameter("gicp/fitnessBaseline/minSamples", this->fitness_baseline_min_samples_);
  this->get_parameter("gicp/fitnessBaseline/seedBaseline", this->fitness_baseline_seed_);
  this->get_parameter("gicp/fitnessRatioRejectThreshold", this->fitness_ratio_reject_);
  this->get_parameter("gicp/degeneracy/partialUpdate", this->degen_partial_update_enable_);
  this->get_parameter("gicp/degeneracy/full6d", this->degen_full6d_);
  this->get_parameter("gicp/degeneracy/couplingLengthM", this->degen_coupling_length_m_);
  this->get_parameter("gicp/degeneracy/relFloor6d", this->degen_rel_floor_6d_);
  this->get_parameter("gicp/degeneracy/relFloorRot", this->degen_rel_floor_rot_);
  this->get_parameter("gicp/degeneracy/relFloorTrans", this->degen_rel_floor_trans_);
  this->get_parameter("gicp/yawGate/enable", this->yaw_gate_enable_);
  this->get_parameter("gicp/yawGate/maxCorrDeg", this->yaw_gate_max_corr_deg_);
  this->get_parameter("gicp/yawGate/fitnessRatio", this->yaw_gate_fitness_ratio_);
  this->get_parameter("gicp/dof/mode", this->gicp_dof_mode_);
  this->get_parameter("gicp/dof/full6dofEveryN", this->gicp_full6dof_every_n_);
  this->get_parameter("gicp/prior/yawInfo", this->gicp_prior_yaw_info_);
  this->get_parameter("gicp/prior/rollPitchInfo", this->gicp_prior_rollpitch_info_);
  if (this->gicp_dof_mode_ != "6dof" &&
      this->gicp_dof_mode_ != "4dof" &&
      this->gicp_dof_mode_ != "3dof") {
    RCLCPP_WARN(this->get_logger(),
                "Unknown gicp/dof/mode '%s'; using 6dof",
                this->gicp_dof_mode_.c_str());
    this->gicp_dof_mode_ = "6dof";
  }
  if (this->gicp_full6dof_every_n_ < 0) {
    RCLCPP_WARN(this->get_logger(),
                "gicp/dof/full6dofEveryN cannot be negative; disabling it");
    this->gicp_full6dof_every_n_ = 0;
  }
  if (!std::isfinite(this->gicp_min_correspondence_ratio_) ||
      this->gicp_min_correspondence_ratio_ <= 0.0 ||
      this->gicp_min_correspondence_ratio_ > 1.0) {
    throw std::invalid_argument(
        "gicp/minCorrespondenceRatio must be finite and within (0, 1]");
  }
  if (!std::isfinite(this->gicp_prior_yaw_info_) ||
      this->gicp_prior_yaw_info_ < 0.0) {
    this->gicp_prior_yaw_info_ = 0.0;
  }
  if (!std::isfinite(this->gicp_prior_rollpitch_info_) ||
      this->gicp_prior_rollpitch_info_ < 0.0) {
    this->gicp_prior_rollpitch_info_ = 0.0;
  }
  if (this->fitness_baseline_window_ < 3) this->fitness_baseline_window_ = 3;
  if (this->fitness_baseline_min_samples_ < 3) this->fitness_baseline_min_samples_ = 3;
  RCLCPP_INFO(this->get_logger(),
              "P1 gating: fitness baseline %s (window=%d, min=%d), ratio_reject=%.2f, "
              "partial_update=%s (%s, L=%.1fm, floor6d=%.3f, block floors rot=%.3f trans=%.3f), "
              "yaw_gate=%s (max=%.2fdeg, ratio>%.2f)",
              this->fitness_baseline_enable_ ? "ON" : "OFF",
              this->fitness_baseline_window_, this->fitness_baseline_min_samples_,
              this->fitness_ratio_reject_,
              this->degen_partial_update_enable_ ? "ON" : "OFF",
              this->degen_full6d_ ? "full6d" : "blockwise",
              this->degen_coupling_length_m_, this->degen_rel_floor_6d_,
              this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
              this->yaw_gate_enable_ ? "ON" : "OFF",
              this->yaw_gate_max_corr_deg_, this->yaw_gate_fitness_ratio_);
  RCLCPP_INFO(
      this->get_logger(),
      "small_gicp constraints: mode=%s, full_6dof_every_n=%d, "
      "prior_info=[roll/pitch=%.1f,yaw=%.1f] rad^-2",
      this->gicp_dof_mode_.c_str(), this->gicp_full6dof_every_n_,
      this->gicp_prior_rollpitch_info_, this->gicp_prior_yaw_info_);

  // Preprocessing parameters
  this->declare_parameter<double>("dlio/preprocessing/cropBoxFilter/size", 80.0);
  this->declare_parameter<bool>("dlio/preprocessing/voxelFilter/use", true);
  this->declare_parameter<double>("dlio/preprocessing/voxelFilter/res", 0.3);

  this->get_parameter("dlio/preprocessing/cropBoxFilter/size", this->crop_size_);
  this->get_parameter("dlio/preprocessing/voxelFilter/use", this->vf_use_);
  this->get_parameter("dlio/preprocessing/voxelFilter/res", this->vf_res_);

  if (!std::isfinite(this->map_voxel_size_) || this->map_voxel_size_ < 0.0) {
    throw std::invalid_argument(
        "localization/map_voxel_size must be finite and non-negative");
  }
  if (this->local_map_enable_) {
    if (!std::isfinite(this->local_map_radius_) ||
        this->local_map_radius_ <= 0.0) {
      throw std::invalid_argument(
          "localization/local_map/radius must be finite and positive");
    }
    if (!std::isfinite(this->crop_size_) || this->crop_size_ <= 0.0 ||
        this->crop_size_ >= 1000.0) {
      throw std::invalid_argument(
          "bounded local-map registration requires a finite, enabled "
          "dlio/preprocessing/cropBoxFilter/size");
    }
    if (!std::isfinite(this->gicp_max_corr_dist_) ||
        this->gicp_max_corr_dist_ < 0.0) {
      throw std::invalid_argument(
          "gicp/maxCorrespondenceDistance must be finite and non-negative");
    }
    const double scan_xy_extent = std::sqrt(2.0) * this->crop_size_;
    this->local_map_valid_center_offset_ =
        this->local_map_radius_ - scan_xy_extent - this->gicp_max_corr_dist_;
    if (this->local_map_valid_center_offset_ <= 0.0) {
      throw std::invalid_argument(
          "localization/local_map/radius must exceed sqrt(2) * "
          "cropBoxFilter/size + gicp/maxCorrespondenceDistance");
    }
    this->local_map_grid_size_ = this->local_map_radius_;
    this->local_map_rebuild_distance_ = std::min(
        0.2 * this->local_map_radius_,
        0.5 * this->local_map_valid_center_offset_);
  }

  // IMU and deskewing parameters
  this->declare_parameter<bool>("dlio/deskew", true);
  this->declare_parameter<double>("dlio/gravity", 9.81);
  this->declare_parameter<int>("dlio/imu/bufferSize", 2000);

  this->get_parameter("dlio/deskew", this->deskew_);
  this->get_parameter("dlio/gravity", this->gravity_);
  this->get_parameter("dlio/imu/bufferSize", this->imu_buffer_size_);

  this->declare_parameter<bool>("localization/flip_y", false);
  this->get_parameter("localization/flip_y", this->flip_y_);

  // Startup raw-cloud quality gate. Robin W's nominal vertical FOV is 30 degrees; a
  // substantially narrower point distribution removes the vertical structure
  // that stabilizes scan registration. The gate runs before crop, deskew, or
  // multi-LiDAR transforms and rejects each sensor stream independently.
  this->declare_parameter<double>(
      "localization/lidar_quality/min_vertical_fov_deg",
      kDefaultMinimumVerticalFovDeg);
  this->declare_parameter<int>(
      "localization/lidar_quality/min_valid_points",
      static_cast<int>(kDefaultMinimumFovPoints));
  int lidar_fov_min_valid_points =
      static_cast<int>(kDefaultMinimumFovPoints);
  this->get_parameter(
      "localization/lidar_quality/min_vertical_fov_deg",
      this->lidar_min_vertical_fov_deg_);
  this->get_parameter(
      "localization/lidar_quality/min_valid_points",
      lidar_fov_min_valid_points);
  if (!std::isfinite(this->lidar_min_vertical_fov_deg_) ||
      this->lidar_min_vertical_fov_deg_ < 25.0 ||
      this->lidar_min_vertical_fov_deg_ >
          kNominalRobinWVerticalFovDeg) {
    throw std::invalid_argument(
        "localization/lidar_quality/min_vertical_fov_deg must be finite "
        "and within [25, 30] degrees for the Robin W profile");
  }
  if (lidar_fov_min_valid_points <
          static_cast<int>(kDefaultMinimumFovPoints) ||
      lidar_fov_min_valid_points >
          static_cast<int>(kMaximumMinimumFovPoints)) {
    throw std::invalid_argument(
        "localization/lidar_quality/min_valid_points must be within "
        "[100, 10000]");
  }
  this->lidar_fov_min_valid_points_ =
      static_cast<size_t>(lidar_fov_min_valid_points);
  RCLCPP_INFO(
      this->get_logger(),
      "LiDAR startup FOV gate active: nominal Robin W %.1f deg, reject "
      "occupied spans below %.1f deg or clouds with fewer than %zu valid "
      "sampled returns",
      kNominalRobinWVerticalFovDeg, this->lidar_min_vertical_fov_deg_,
      this->lidar_fov_min_valid_points_);

  // Multi-LiDAR concatenation: match auxiliary sweeps by absolute point-time
  // endpoints, then transform their XYZ into the primary sensor frame. Robin W
  // timestamps remain unchanged on the shared PTP axis. lidar_mode is the
  // authoritative sensor-set selector. The old enabled boolean remains only
  // as a compatibility fallback when lidar_mode is empty.
  this->declare_parameter<std::string>(
      "localization/lidar_mode", kDefaultLidarModeParameter);
  this->declare_parameter<bool>("localization/lidar_concat/enabled", false);
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("localization/lidar_concat/aux_frames", std::vector<std::string>{});
  this->declare_parameter<double>("localization/lidar_concat/sweep_time_threshold", 0.01);
  this->declare_parameter<double>(
      "localization/lidar_concat/future_sweep_wait_timeout", 0.15);
  // Robin W frame period in seconds. 10 FPS (0.100) is the slowest supported
  // rate and therefore the safe default: too LARGE only loosens the wire
  // contract, it never falsely rejects. Deployments running 20 FPS -- which is
  // what recording/sensor_config.yaml specifies for this component -- must set
  // 0.05, otherwise the contract gate accepts a 99 ms span (two fused frames)
  // and the sweep_time_threshold ceiling below permits a full frame period.
  this->declare_parameter<double>("localization/seyond_frame_duration_s",
                                  kSeyondFrameDurationSecondsDefault);
  // P4#3: 200 for parity with GLIM's lidar_concat. At 10 Hz a depth of 20 is
  // only 2 s of aux history — a brief aux-stream stall or replay burst drops
  // the matching scan and the frame silently degrades to fewer LiDARs (run-12
  // throttled logs: only ~35% of sampled scans merged 2/2). 200 = ~20 s.
  this->declare_parameter<int>("localization/lidar_concat/buffer_size", 200);
  // Offline aux-extrinsic resolution (mirrors GLIM; no live TF needed).
  this->declare_parameter<std::string>("localization/lidar_concat/primary_frame", "lidar_front_link");
  this->declare_parameter<std::string>("localization/lidar_concat/urdf_path", "");
  this->declare_parameter<std::vector<double>>("localization/lidar_concat/aux_static_transforms", std::vector<double>{});
  // Strict merge guard. When require_all_aux=true, a scan that fails to merge every
  // configured aux is NOT localized on fewer LiDARs -- it is skipped. Brief transient
  // misses (buffers warming up, a dropped aux frame) are tolerated up to
  // max_consecutive_aux_merge_failures; past that, the node aborts if
  // abort_on_merge_failure=true, otherwise it keeps skipping non-fatally (louder
  // warning). require_all_aux=false localizes on whatever aux merged (no skip/abort).
  this->declare_parameter<bool>("localization/lidar_concat/require_all_aux", false);
  this->declare_parameter<bool>("localization/lidar_concat/abort_on_merge_failure", true);
  this->declare_parameter<int>("localization/lidar_concat/max_consecutive_aux_merge_failures", 10);

  std::string lidar_mode_param;
  bool legacy_concat_enabled = false;
  this->get_parameter("localization/lidar_mode", lidar_mode_param);
  this->get_parameter(
      "localization/lidar_concat/enabled", legacy_concat_enabled);
  try {
    const auto resolution =
        resolveLidarModeParameter(lidar_mode_param, legacy_concat_enabled);
    this->lidar_mode_ = resolution.mode;
    if (resolution.used_legacy_fallback) {
      RCLCPP_WARN(
          this->get_logger(),
          "localization/lidar_mode is unset; selected '%s' from deprecated "
          "localization/lidar_concat/enabled. Set lidar_mode explicitly.",
          lidarModeName(this->lidar_mode_));
    }
  } catch (const std::invalid_argument& e) {
    RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    throw;
  }
  this->concat_enabled_ = lidarModeUsesAuxiliaries(this->lidar_mode_);

  std::vector<std::string> aux_topics_param, aux_frames_param;
  this->get_parameter("localization/lidar_concat/aux_topics", aux_topics_param);
  this->get_parameter("localization/lidar_concat/aux_frames", aux_frames_param);
  this->get_parameter("localization/seyond_frame_duration_s",
                      this->seyond_frame_duration_s_);
  if (!std::isfinite(this->seyond_frame_duration_s_) ||
      this->seyond_frame_duration_s_ < 0.010 ||
      this->seyond_frame_duration_s_ > 0.500) {
    throw std::invalid_argument(
        "localization/seyond_frame_duration_s must be finite and within "
        "[0.010, 0.500] s (Robin W is rated 10-20 FPS => 0.100 / 0.050)");
  }
  this->get_parameter(
      "localization/lidar_concat/sweep_time_threshold",
      this->concat_sweep_time_threshold_);
  this->get_parameter(
      "localization/lidar_concat/future_sweep_wait_timeout",
      this->concat_future_sweep_wait_timeout_);
  // Half the frame period is the ambiguity bound: at a larger gate two
  // different sweeps can both satisfy it. Derived from the CONFIGURED period,
  // not a 10 Hz constant, so a 20 FPS deployment gets a 0.025 s ceiling.
  if (this->concat_enabled_ &&
      (!std::isfinite(this->concat_sweep_time_threshold_) ||
       this->concat_sweep_time_threshold_ < 0.0 ||
       this->concat_sweep_time_threshold_ >=
           0.5 * this->seyond_frame_duration_s_)) {
    throw std::invalid_argument(
        "localization/lidar_concat/sweep_time_threshold must be finite, "
        "non-negative, and below half of localization/seyond_frame_duration_s");
  }
  if (this->concat_enabled_ &&
      (!std::isfinite(this->concat_future_sweep_wait_timeout_) ||
       this->concat_future_sweep_wait_timeout_ < 0.0 ||
       this->concat_future_sweep_wait_timeout_ > 1.0)) {
    throw std::invalid_argument(
        "localization/lidar_concat/future_sweep_wait_timeout must be finite "
        "and within [0, 1] seconds");
  }
  int concat_buffer_size_int = 20;
  this->get_parameter("localization/lidar_concat/buffer_size", concat_buffer_size_int);
  this->concat_buffer_size_ = static_cast<size_t>(std::max(1, concat_buffer_size_int));
  this->get_parameter("localization/lidar_concat/primary_frame", this->concat_primary_frame_);
  this->get_parameter("localization/lidar_concat/urdf_path", this->concat_urdf_path_);
  this->get_parameter("localization/lidar_concat/require_all_aux", this->concat_require_all_aux_);
  this->get_parameter("localization/lidar_concat/abort_on_merge_failure", this->concat_abort_on_merge_failure_);
  this->get_parameter("localization/lidar_concat/max_consecutive_aux_merge_failures", this->concat_max_consec_fail_);
  std::vector<double> aux_static_flat;
  this->get_parameter("localization/lidar_concat/aux_static_transforms", aux_static_flat);

  try {
    validateLidarModeConfiguration(
        this->lidar_mode_, aux_topics_param, aux_frames_param,
        this->concat_primary_frame_);
  } catch (const std::invalid_argument& e) {
    RCLCPP_FATAL(
        this->get_logger(), "Invalid LiDAR mode configuration: %s", e.what());
    throw;
  }

  RCLCPP_INFO(
      this->get_logger(),
      "LiDAR mode: %s (primary front Robin W%s)",
      lidarModeName(this->lidar_mode_),
      this->concat_enabled_ ? " + two rear Robin W auxiliaries" : " only");

  if (this->concat_enabled_) {
    for (size_t i = 0; i < aux_topics_param.size(); ++i) {
      auto aux = std::make_unique<AuxLidar>();
      aux->topic = aux_topics_param[i];
      aux->frame = aux_frames_param[i];
      aux->T_primary_aux = Eigen::Matrix4f::Identity();
      aux->extrinsic_cached = false;
      this->aux_lidars_.push_back(std::move(aux));
    }
    RCLCPP_INFO(this->get_logger(),
                "lidar_concat enabled: %zu aux lidars, sweep_time_threshold=%.3fs "
                "(ceiling %.3fs = half the %.0f ms frame), buffer_size=%zu, "
                "future_wait=%.3fs",
                this->aux_lidars_.size(), this->concat_sweep_time_threshold_,
                0.5 * this->seyond_frame_duration_s_,
                this->seyond_frame_duration_s_ * 1.0e3,
                this->concat_buffer_size_,
                this->concat_future_sweep_wait_timeout_);
    for (const auto& a : this->aux_lidars_) {
      RCLCPP_INFO(this->get_logger(), "  aux lidar: topic='%s' frame='%s'",
                  a->topic.c_str(), a->frame.c_str());
    }

    // Split the flat static-transform array (16 row-major doubles per aux, in
    // aux order) into per-aux 4x4 matrices for the offline resolver.
    std::vector<std::vector<double>> aux_static_transforms;
    if (!aux_static_flat.empty()) {
      if (aux_static_flat.size() == 16 * this->aux_lidars_.size()) {
        aux_static_transforms.resize(this->aux_lidars_.size());
        for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
          aux_static_transforms[i].assign(aux_static_flat.begin() + 16 * i, aux_static_flat.begin() + 16 * (i + 1));
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "lidar_concat: aux_static_transforms has %zu values, expected %zu (16 x %zu aux); ignoring",
                    aux_static_flat.size(), 16 * this->aux_lidars_.size(), this->aux_lidars_.size());
      }
    }

    // Resolve aux extrinsics now, without live TF (URDF > static > TF-at-runtime).
    this->resolveAuxExtrinsicsOffline(aux_static_transforms);
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Front-only mode: rear LiDAR subscriptions, buffers, startup FOV "
        "waits, transforms, and merge work are disabled");
  }

  // IMU calibration time (seconds of stationary data to average for bias/gravity)
  this->declare_parameter<double>("dlio/imu/calibTime", 3.0);
  this->get_parameter("dlio/imu/calibTime", this->imu_calib_time_);

  // Hitch Sensor Dome — motion-variance gate for the stationary
  // calibration path. The stock DLIO calibration assumes the vehicle
  // is still during calibTime; on a Hitch dome bag started in motion
  // (race-track replay, mid-session restart) with no RTK to drive the
  // rtk_init path, that assumption produces a tilted gravity vector
  // and wrong gyro/accel bias. We compute σ_||a|| (linear-acceleration
  // norm) over the calibration window and refuse to declare the
  // calibration complete if it exceeds this threshold — instead we
  // reset the window and re-accumulate.
  //
  // 0.10 m/s² corresponds to ~10 mg jitter on a still vehicle (a
  // generous floor for the Atlas Duo's onboard IMU). Crank up if you
  // need looser triggering on vibration-heavy installations; set
  // ≤ 0 to disable the gate entirely.
  this->declare_parameter<double>("localization/calib/motion_sigma_max", 0.10);
  this->get_parameter("localization/calib/motion_sigma_max", this->imu_calib_motion_sigma_max_);

  // RTK-driven IMU calibration. When enabled, the first message on the GT odom
  // topic triggers a calibration window in which IMU residuals are computed
  // against the GT pose/twist (no stationary assumption). Falls back to
  // stationary calibration if no GT arrives within fallback_timeout seconds.
  // Hard guard for single-IMU deployments. The raw driver is the low-latency
  // live default; the adapter's retimed stream is allowed for normalized
  // replay, where its configured lookahead is intentional.
  this->declare_parameter<bool>("localization/imu/require_topic_allowlist", true);
  this->declare_parameter<std::vector<std::string>>(
      "localization/imu/topic_allowlist",
      std::vector<std::string>{"/gps_p1/imu", "/imu/data"});
  this->get_parameter("localization/imu/require_topic_allowlist", this->imu_require_topic_allowlist_);
  this->get_parameter("localization/imu/topic_allowlist", this->imu_topic_allowlist_);
  if (this->imu_topic_allowlist_.empty()) {
    this->imu_topic_allowlist_.push_back("/imu/data");
  }
  // Safety guard: reject IMU samples whose header.frame_id does not match
  // localization/imu_frame. Keep enabled by default for P1-only operation.
  this->declare_parameter<bool>("localization/imu/require_frame_match", true);
  this->get_parameter("localization/imu/require_frame_match", this->imu_require_frame_match_);

  // RTK-driven IMU calibration. When enabled, the first accepted GT odom
  // sample triggers a calibration window in which IMU residuals are computed
  // against the GT pose/twist (no stationary assumption). With the RTK gate
  // enabled, "accepted" means the gt_odom source reported pose covariance
  // within the configured thresholds; disabling the gate for bag replay
  // removes that guarantee. Falls back to stationary calibration if no
  // accepted GT arrives within fallback_timeout.
  this->declare_parameter<bool>("localization/rtk_init/enable", true);
  this->declare_parameter<double>("localization/rtk_init/calib_window", 2.0);
  this->declare_parameter<double>("localization/rtk_init/fallback_timeout", 5.0);
  this->get_parameter("localization/rtk_init/enable", this->rtk_init_enabled_);
  this->get_parameter("localization/rtk_init/calib_window", this->rtk_calib_window_sec_);
  this->get_parameter("localization/rtk_init/fallback_timeout", this->rtk_fallback_timeout_sec_);
  RCLCPP_INFO(this->get_logger(),
              "RTK-driven IMU calibration: %s (window=%.1fs, fallback_timeout=%.1fs)",
              this->rtk_init_enabled_ ? "ENABLED" : "disabled",
              this->rtk_calib_window_sec_, this->rtk_fallback_timeout_sec_);

  // Sensor type for per-point timestamp handling during deskewing.
  // Hitch Sensor Dome default: the pinned Seyond driver publishes
  // timestamp/FLOAT64 as absolute Unix seconds.
  this->declare_parameter<std::string>("localization/sensor_type", "seyond");
  std::string sensor_type_str;
  this->get_parameter("localization/sensor_type", sensor_type_str);
  if (sensor_type_str == "velodyne") {
    this->sensor = dlio::SensorType::VELODYNE;
  } else if (sensor_type_str == "seyond") {
    this->sensor = dlio::SensorType::SEYOND;
  } else if (sensor_type_str == "hesai") {
    this->sensor = dlio::SensorType::HESAI;
  } else if (sensor_type_str == "livox") {
    this->sensor = dlio::SensorType::LIVOX;
  } else if (sensor_type_str == "ouster") {
    this->sensor = dlio::SensorType::OUSTER;
  } else {
    this->sensor = dlio::SensorType::UNKNOWN;
    RCLCPP_WARN(this->get_logger(),
                "Unknown localization/sensor_type '%s'; per-point deskew needs ouster, velodyne, "
                "seyond, hesai, or livox",
                sensor_type_str.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Sensor type: %s", sensor_type_str.c_str());

  // Geometric Observer parameters. Position/orientation gains stay active, but
  // online IMU bias adaptation defaults off for the fused Point One (Atlas) INS path; the
  // initial RTK/stationary calibration still seeds state.b once before
  // propagation.
  this->declare_parameter<double>("odom/geo/Kp", 4.5);
  this->declare_parameter<double>("odom/geo/Kv", 11.25);
  this->declare_parameter<double>("odom/geo/Kq", 4.0);
  this->declare_parameter<double>("odom/geo/Kab", 0.0);
  this->declare_parameter<double>("odom/geo/Kgb", 0.0);
  this->declare_parameter<double>("odom/geo/Kz_damping", 5.0);
  this->declare_parameter<double>("odom/geo/abias_max", 5.0);
  this->declare_parameter<double>("odom/geo/gbias_max", 0.5);

  this->get_parameter("odom/geo/Kp", this->geo_Kp_);
  this->get_parameter("odom/geo/Kv", this->geo_Kv_);
  this->get_parameter("odom/geo/Kq", this->geo_Kq_);
  this->get_parameter("odom/geo/Kab", this->geo_Kab_);
  this->get_parameter("odom/geo/Kgb", this->geo_Kgb_);
  this->get_parameter("odom/geo/Kz_damping", this->geo_Kz_damping_);
  this->get_parameter("odom/geo/abias_max", this->geo_abias_max_);
  this->get_parameter("odom/geo/gbias_max", this->geo_gbias_max_);

  // Hitch Sensor Dome — yaw-rate-adaptive observer-gain attenuation.
  // See updateState() for the math; see the YAML for tuning guidance.
  // Defaults assume a race / track vehicle: full Kp/Kq below 0.5 rad/s
  // (~29°/s), gains scaled down to 25% at 1.5 rad/s (~86°/s) and above.
  this->declare_parameter<bool>(  "odom/geo/yawrate_attenuation/enable",          true);
  this->declare_parameter<double>("odom/geo/yawrate_attenuation/threshold_rad_s", 0.5);
  this->declare_parameter<double>("odom/geo/yawrate_attenuation/saturation_rad_s", 1.5);
  this->declare_parameter<double>("odom/geo/yawrate_attenuation/min_gain_scale",  0.25);
  this->get_parameter("odom/geo/yawrate_attenuation/enable",
                      this->yawrate_attenuation_enabled_);
  this->get_parameter("odom/geo/yawrate_attenuation/threshold_rad_s",
                      this->yawrate_attenuation_threshold_);
  this->get_parameter("odom/geo/yawrate_attenuation/saturation_rad_s",
                      this->yawrate_attenuation_saturation_);
  this->get_parameter("odom/geo/yawrate_attenuation/min_gain_scale",
                      this->yawrate_attenuation_min_scale_);
  RCLCPP_INFO(this->get_logger(),
              "Yaw-rate gain attenuation: %s "
              "(threshold=%.2f rad/s, saturation=%.2f rad/s, min_scale=%.2f)",
              this->yawrate_attenuation_enabled_ ? "ENABLED" : "disabled",
              this->yawrate_attenuation_threshold_,
              this->yawrate_attenuation_saturation_,
              this->yawrate_attenuation_min_scale_);
  // P3: delta-form observer correction. The GICP measurement and its IMU
  // prior are both stamped at the scan's median point time, 0.1-0.3 s before
  // the correction is applied (half sweep + queueing + GICP solve). Legacy
  // behavior pulls the CURRENT state toward that stale absolute pose, which
  // is a systematic backward/yaw-lag drag during turns (accepted-frame gt_err
  // scaled with yaw rate on run 12). Delta form instead applies
  // T_corr = T_meas * inv(T_prior) — the time-free IMU-drift correction — to
  // the current state, so a perfect IMU/GICP agreement produces a ZERO
  // correction regardless of latency. false = legacy absolute-target observer.
  this->declare_parameter<bool>("odom/geo/delta_correction", true);
  this->get_parameter("odom/geo/delta_correction", this->geo_delta_correction_);

  // Observer-correction stability bounds (P2#1).
  this->declare_parameter<double>("odom/geo/observer_dt_max", 0.15);
  this->declare_parameter<double>("odom/geo/max_pos_correction", 0.0);
  this->declare_parameter<double>("odom/geo/max_vel_correction", 0.0);
  this->get_parameter("odom/geo/observer_dt_max", this->geo_observer_dt_max_);
  this->get_parameter("odom/geo/max_pos_correction", this->geo_max_pos_correction_);
  this->get_parameter("odom/geo/max_vel_correction", this->geo_max_vel_correction_);
  if (this->geo_observer_dt_max_ <= 0.0) {
    this->geo_observer_dt_max_ = 0.15;  // guard against a non-positive cap disabling all corrections
  }

  // Time/speed-based dead-reckoning covariance growth (P3).
  this->declare_parameter<double>("odom/geo/dr_cov_time_rate", 0.5);
  this->declare_parameter<double>("odom/geo/dr_cov_dist_frac", 0.05);
  this->get_parameter("odom/geo/dr_cov_time_rate", this->dr_cov_time_rate_);
  this->get_parameter("odom/geo/dr_cov_dist_frac", this->dr_cov_dist_frac_);

  // Debug parameters
  this->declare_parameter<bool>("localization/debug/enable_pub", true);
  this->declare_parameter<bool>("localization/debug/enable_jump_log", true);
  this->declare_parameter<bool>("localization/debug/verbose_scan_log", false);
  this->declare_parameter<bool>("localization/debug/small_gicp_lm_debug", false);
  // Migration alias for existing deployments. New configuration must use
  // small_gicp_lm_debug; declaring the old key prevents an older Humble
  // parameter file from aborting node startup during rollout.
  this->declare_parameter<bool>("localization/debug/nano_gicp_lm_debug", false);
  this->declare_parameter<double>("localization/debug/jump_trans_m", 1.0);
  this->declare_parameter<double>("localization/debug/jump_rot_deg", 10.0);
  this->declare_parameter<bool>("localization/verbose", true);

  this->get_parameter("localization/debug/enable_pub", this->debug_pub_enabled_);
  this->get_parameter("localization/debug/enable_jump_log", this->debug_jump_log_enabled_);
  this->get_parameter("localization/debug/verbose_scan_log", this->debug_verbose_scan_log_);
  this->get_parameter("localization/debug/small_gicp_lm_debug", this->debug_lm_print_);
  bool legacy_nano_lm_debug = false;
  this->get_parameter("localization/debug/nano_gicp_lm_debug", legacy_nano_lm_debug);
  if (legacy_nano_lm_debug) {
    if (!this->debug_lm_print_) {
      this->debug_lm_print_ = true;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "localization/debug/nano_gicp_lm_debug is deprecated; use "
        "localization/debug/small_gicp_lm_debug");
  }
  this->get_parameter("localization/debug/jump_trans_m", this->debug_jump_trans_m_);
  this->get_parameter("localization/debug/jump_rot_deg", this->debug_jump_rot_deg_);

  // Speed/scan_dt-aware jump-gate scaling (P2#2). 0 reproduces the fixed thresholds.
  this->declare_parameter<double>("localization/jump/trans_speed_scale", 1.5);
  this->declare_parameter<double>("localization/jump/rot_dt_scale_deg", 60.0);
  this->get_parameter("localization/jump/trans_speed_scale", this->jump_trans_speed_scale_);
  this->get_parameter("localization/jump/rot_dt_scale_deg", this->jump_rot_dt_scale_deg_);

  this->get_parameter("localization/verbose", this->verbose_);

  // Suppress INFO/DEBUG logs when verbose is off; WARN/ERROR still pass through.
  // REVIEW FIX (commit 0e4916c follow-up): the WARN clamp also silenced the
  // INFO-level "SCAN DEBUG | status=ok/ok_partial" lines even when
  // debug/verbose_scan_log=true — the replay scorecard
  // (scripts/analyze_scan_debug_log.py) would then see only rejected frames
  // and report nonsense acceptance/streak/bucket stats. Keep INFO alive when
  // the per-scan evidence log is requested; verbose_ still gates the raw
  // stderr debug prints independently.
  if (!this->verbose_ && !this->debug_verbose_scan_log_) {
    rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                     RCUTILS_LOG_SEVERITY_WARN);
  } else if (!this->verbose_) {
    RCLCPP_INFO(this->get_logger(),
                "localization/verbose=false but debug/verbose_scan_log=true: keeping INFO "
                "log level so per-scan SCAN DEBUG evidence reaches the log");
  }

  RCLCPP_INFO(this->get_logger(), "Preprocessing config: crop_size=%.2f, voxel_filter=%s, voxel_res=%.2f",
              this->crop_size_, this->vf_use_ ? "ENABLED" : "DISABLED", this->vf_res_);
  RCLCPP_INFO(this->get_logger(), "IMU config: deskew=%s, gravity=%.2f, buffer_size=%d",
              this->deskew_ ? "ENABLED" : "DISABLED", this->gravity_, this->imu_buffer_size_);
  RCLCPP_INFO(this->get_logger(), "Geometric Observer: Kp=%.2f, Kv=%.2f, Kq=%.2f, Kab=%.2f, Kgb=%.2f",
              this->geo_Kp_, this->geo_Kv_, this->geo_Kq_, this->geo_Kab_, this->geo_Kgb_);
  RCLCPP_INFO(this->get_logger(), "Localization mode: %s",
              this->imu_only_mode_ ? "IMU-only (GICP disabled)" : "GICP + IMU");
  RCLCPP_INFO(this->get_logger(),
              "GICP rejection: fitness>%.3f m^2, correspondence_ratio<%.2f, "
              "large_jump=%s, hessian_cond>%.2e AND "
              "(fitness>%.3f OR trans>%.2fm OR rot>%.2fdeg) (%s)",
              this->gicp_fitness_reject_threshold_,
              this->gicp_min_correspondence_ratio_,
              this->gicp_reject_large_jumps_ ? "on" : "off",
              this->gicp_hessian_cond_max_,
              this->gicp_hessian_fitness_warn_,
              this->gicp_hessian_trans_warn_m_,
              this->gicp_hessian_rot_warn_deg_,
              this->gicp_hessian_cond_max_ > 0.0 ? "on" : "disabled");
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: %s (min consecutive failures=%d, "
              "RTK-fixed sanity radius=%.1fm)",
              this->gt_recovery_enabled_ ? "ENABLED" : "DISABLED",
              this->gt_recovery_min_consecutive_failures_,
              this->gt_recovery_sanity_radius_);
  RCLCPP_INFO(this->get_logger(), "Debug: publish=%s jump_log=%s thresholds=[%.2fm, %.1fdeg]",
              this->debug_pub_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_log_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_trans_m_, this->debug_jump_rot_deg_);
  RCLCPP_INFO(this->get_logger(), "Debug detail: verbose_scan_log=%s small_gicp_lm_debug=%s",
              this->debug_verbose_scan_log_ ? "ENABLED" : "DISABLED",
              this->debug_lm_print_ ? "ENABLED" : "DISABLED");
}

int64_t gicp_localization::LocalizationNode::localMapGridKey(
    int32_t ix, int32_t iy) {
  const uint64_t ux = static_cast<uint32_t>(ix);
  const uint64_t uy = static_cast<uint32_t>(iy);
  return static_cast<int64_t>((ux << 32) | uy);
}

void gicp_localization::LocalizationNode::buildLocalMapGrid() {
  this->local_map_grid_.clear();
  if (!this->map_cloud || this->map_cloud->empty()) {
    return;
  }
  if (this->map_cloud->size() >
      static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Prepared map has too many points for the local-map index");
    return;
  }

  this->local_map_grid_.reserve(std::min<size_t>(
      this->map_cloud->size() / 1000 + 16, 1U << 20));
  for (uint32_t i = 0; i < this->map_cloud->size(); ++i) {
    const auto& point = this->map_cloud->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const double grid_x =
        std::floor(static_cast<double>(point.x) / this->local_map_grid_size_);
    const double grid_y =
        std::floor(static_cast<double>(point.y) / this->local_map_grid_size_);
    if (grid_x < std::numeric_limits<int32_t>::min() ||
        grid_x > std::numeric_limits<int32_t>::max() ||
        grid_y < std::numeric_limits<int32_t>::min() ||
        grid_y > std::numeric_limits<int32_t>::max()) {
      continue;
    }
    this->local_map_grid_[localMapGridKey(
        static_cast<int32_t>(grid_x), static_cast<int32_t>(grid_y))]
        .push_back(i);
  }
}

gicp_localization::LocalizationNode::PreparedGicpTargetPtr
gicp_localization::LocalizationNode::createLocalMapTarget(
    const Eigen::Vector3f& center, double radius, int build_threads,
    size_t* candidate_count_out) const {
  if (!this->full_map_target_ || !this->map_cloud ||
      this->map_cloud->empty() || !center.allFinite() ||
      !std::isfinite(radius) || radius <= 0.0) {
    return nullptr;
  }

  const auto grid_index = [this](double coordinate, int32_t* index) {
    const double value =
        std::floor(coordinate / this->local_map_grid_size_);
    if (value < std::numeric_limits<int32_t>::min() ||
        value > std::numeric_limits<int32_t>::max()) {
      return false;
    }
    *index = static_cast<int32_t>(value);
    return true;
  };

  int32_t ix0;
  int32_t ix1;
  int32_t iy0;
  int32_t iy1;
  if (!grid_index(static_cast<double>(center.x()) - radius, &ix0) ||
      !grid_index(static_cast<double>(center.x()) + radius, &ix1) ||
      !grid_index(static_cast<double>(center.y()) - radius, &iy0) ||
      !grid_index(static_cast<double>(center.y()) + radius, &iy1)) {
    return nullptr;
  }

  size_t candidate_count = 0;
  for (int64_t ix = ix0; ix <= static_cast<int64_t>(ix1); ++ix) {
    for (int64_t iy = iy0; iy <= static_cast<int64_t>(iy1); ++iy) {
      const auto bucket = this->local_map_grid_.find(localMapGridKey(
          static_cast<int32_t>(ix), static_cast<int32_t>(iy)));
      if (bucket != this->local_map_grid_.end()) {
        candidate_count += bucket->second.size();
      }
    }
  }
  if (candidate_count_out) {
    *candidate_count_out = candidate_count;
  }

  auto local = std::make_shared<pcl::PointCloud<PointType>>();
  gicp_plusplus::SmallGicpBackend<
      PointType, PointType>::CovarianceVector local_covariances;
  local->points.reserve(candidate_count);
  local_covariances.reserve(candidate_count);

  const auto& full_covariances = *this->full_map_target_->covariances;
  if (full_covariances.size() != this->map_cloud->size()) {
    return nullptr;
  }
  const double radius_squared = radius * radius;
  for (int64_t ix = ix0; ix <= static_cast<int64_t>(ix1); ++ix) {
    for (int64_t iy = iy0; iy <= static_cast<int64_t>(iy1); ++iy) {
      const auto bucket = this->local_map_grid_.find(localMapGridKey(
          static_cast<int32_t>(ix), static_cast<int32_t>(iy)));
      if (bucket == this->local_map_grid_.end()) {
        continue;
      }
      for (const uint32_t index : bucket->second) {
        const auto& point = this->map_cloud->points[index];
        const double dx = static_cast<double>(point.x) - center.x();
        const double dy = static_cast<double>(point.y) - center.y();
        if (dx * dx + dy * dy <= radius_squared) {
          local->points.push_back(point);
          local_covariances.push_back(full_covariances[index]);
        }
      }
    }
  }

  local->width = static_cast<uint32_t>(local->size());
  local->height = 1;
  local->is_dense = true;
  if (local->size() < this->local_map_min_points_) {
    return nullptr;
  }

  return this->gicp.prepareInputTarget(
      local, std::move(local_covariances), this->gicp_corr_randomness_,
      std::max(1, build_threads));
}

bool gicp_localization::LocalizationNode::rebuildLocalMapTargetSync(
    const Eigen::Vector3f& center) {
  const uint64_t generation = ++this->local_map_generation_;
  {
    std::lock_guard<std::mutex> lock(this->local_map_pending_mtx_);
    this->pending_local_map_target_.reset();
  }

  const auto start = std::chrono::steady_clock::now();
  size_t candidate_count = 0;
  PreparedGicpTargetPtr prepared;
  try {
    prepared = this->createLocalMapTarget(
        center, this->local_map_radius_, omp_get_max_threads(),
        &candidate_count);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(
        this->get_logger(), "Synchronous local-map build failed: %s",
        error.what());
    return false;
  }
  if (!prepared || generation != this->local_map_generation_.load()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Local-map crop unavailable at [%.2f, %.2f] within %.1fm "
        "(candidates=%zu, minimum=%zu)",
        center.x(), center.y(), this->local_map_radius_, candidate_count,
        this->local_map_min_points_);
    return false;
  }

  this->active_local_map_target_ = prepared;
  this->gicp.setInputTarget(prepared);
  this->local_map_center_ = center;
  this->local_map_target_ready_ = true;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  RCLCPP_INFO(
      this->get_logger(),
      "Synchronous local-map target: center=[%.2f,%.2f] "
      "candidates=%zu target=%zu radius=%.1fm time=%.1fms",
      center.x(), center.y(), candidate_count, prepared->cloud->size(),
      this->local_map_radius_, elapsed_ms);
  return true;
}

void gicp_localization::LocalizationNode::buildLocalMapTargetAsync(
    Eigen::Vector3f center, uint64_t generation) {
  const auto start = std::chrono::steady_clock::now();
  size_t candidate_count = 0;
  PreparedGicpTargetPtr prepared;
  try {
    prepared = this->createLocalMapTarget(
        center, this->local_map_radius_, this->local_map_build_threads_,
        &candidate_count);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(
        this->get_logger(), "Asynchronous local-map build failed: %s",
        error.what());
  }

  if (prepared && generation == this->local_map_generation_.load()) {
    std::lock_guard<std::mutex> lock(this->local_map_pending_mtx_);
    if (generation == this->local_map_generation_.load()) {
      this->pending_local_map_target_ = prepared;
      this->pending_local_map_center_ = center;
    }
  } else if (!prepared) {
    RCLCPP_WARN(
        this->get_logger(),
        "Asynchronous local-map crop unavailable at [%.2f, %.2f] "
        "(candidates=%zu, minimum=%zu)",
        center.x(), center.y(), candidate_count,
        this->local_map_min_points_);
  }

  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  RCLCPP_DEBUG(
      this->get_logger(),
      "Asynchronous local-map build: center=[%.2f,%.2f] "
      "candidates=%zu target=%zu time=%.1fms",
      center.x(), center.y(), candidate_count,
      prepared ? prepared->cloud->size() : 0U, elapsed_ms);
  this->local_map_rebuild_busy_ = false;
}

void gicp_localization::LocalizationNode::launchLocalMapTargetBuild(
    const Eigen::Vector3f& center) {
  if (this->local_map_rebuild_busy_.exchange(true)) {
    return;
  }
  if (this->local_map_rebuild_thread_.joinable()) {
    this->local_map_rebuild_thread_.join();
  }

  const uint64_t generation = this->local_map_generation_.load();
  try {
    this->local_map_rebuild_thread_ = std::thread(
        &gicp_localization::LocalizationNode::buildLocalMapTargetAsync,
        this, center, generation);
  } catch (const std::exception& error) {
    this->local_map_rebuild_busy_ = false;
    RCLCPP_ERROR(
        this->get_logger(), "Failed to start local-map builder: %s",
        error.what());
  }
}

void gicp_localization::LocalizationNode::adoptPendingLocalMapTarget(
    const Eigen::Vector3f& current_center) {
  std::lock_guard<std::mutex> lock(this->local_map_pending_mtx_);
  if (!this->pending_local_map_target_) {
    return;
  }

  const double pending_distance =
      (current_center.head<2>() -
       this->pending_local_map_center_.head<2>()).norm();
  const double active_distance =
      this->local_map_target_ready_
          ? (current_center.head<2>() -
             this->local_map_center_.head<2>()).norm()
          : std::numeric_limits<double>::infinity();
  if (pending_distance >= active_distance) {
    this->pending_local_map_target_.reset();
    return;
  }

  this->active_local_map_target_ = this->pending_local_map_target_;
  this->gicp.setInputTarget(this->active_local_map_target_);
  this->local_map_center_ = this->pending_local_map_center_;
  this->pending_local_map_target_.reset();
  this->local_map_target_ready_ = true;
  RCLCPP_INFO(
      this->get_logger(),
      "Adopted local-map target: %zu points center=[%.2f,%.2f]",
      this->active_local_map_target_->cloud->size(),
      this->local_map_center_.x(), this->local_map_center_.y());
}

bool gicp_localization::LocalizationNode::ensureLocalMapTarget(
    const Eigen::Vector3f& center) {
  if (!this->local_map_enable_) {
    return true;
  }
  if (!center.allFinite()) {
    return false;
  }

  this->adoptPendingLocalMapTarget(center);
  if (!this->local_map_target_ready_) {
    return this->rebuildLocalMapTargetSync(center);
  }

  const double center_offset =
      (center.head<2>() - this->local_map_center_.head<2>()).norm();
  if (center_offset > this->local_map_valid_center_offset_) {
    // The async target did not arrive before the active crop could no longer
    // cover the bounded scan. Rebuild synchronously and continue LiDAR+IMU;
    // this path does not authorize a non-RTK pose reset.
    return this->rebuildLocalMapTargetSync(center);
  }
  if (center_offset > this->local_map_rebuild_distance_) {
    this->launchLocalMapTargetBuild(center);
  }
  return true;
}

bool gicp_localization::LocalizationNode::loadMap() {

  if (this->map_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Map path is empty! Please set localization/map_path parameter.");
    return false;
  }

  const std::string manifest_path =
      this->map_manifest_path_.empty()
          ? this->map_path_ + ".manifest.yaml"
          : this->map_manifest_path_;
  std::ifstream manifest_stream(manifest_path);
  const bool manifest_exists = manifest_stream.good();
  manifest_stream.close();

  if (!manifest_exists) {
    if (this->require_map_manifest_ || this->require_live_enu_origin_) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Required ENU map manifest is missing: %s. Export the GLIM dump with "
          "GLIM_plusplus/scripts/export_glim_dump_to_pcd.py.",
          manifest_path.c_str());
      return false;
    }
    RCLCPP_WARN(
        this->get_logger(),
        "Map manifest not found at %s; frame and datum compatibility are not "
        "being verified because localization/require_map_manifest=false.",
        manifest_path.c_str());
  } else {
    try {
      const MapManifest manifest = loadMapManifest(manifest_path);
      this->map_enu_origin_lat_ = manifest.enu_origin.latitude_deg;
      this->map_enu_origin_lon_ = manifest.enu_origin.longitude_deg;
      this->map_enu_origin_alt_ = manifest.enu_origin.altitude_m;
      this->map_enu_origin_loaded_ = true;
      if (!this->expected_enu_origin_.empty()) {
        const EnuOrigin expected = parseEnuOrigin(this->expected_enu_origin_);
        const double datum_error_m =
            enuOriginDistanceMeters(manifest.enu_origin, expected);
        if (datum_error_m > this->enu_origin_tolerance_m_) {
          RCLCPP_ERROR(
              this->get_logger(),
              "ENU datum mismatch: map manifest and expected_enu_origin differ "
              "by %.3f m (limit %.3f m). Refusing to localize.",
              datum_error_m, this->enu_origin_tolerance_m_);
          return false;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Validated ENU map manifest %s against expected_enu_origin "
            "(difference %.3f m)",
            manifest_path.c_str(), datum_error_m);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Invalid ENU map manifest %s: %s",
                   manifest_path.c_str(), e.what());
      return false;
    }
  }

  if (!this->require_live_enu_origin_) {
    if (this->expected_enu_origin_.empty()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "require_live_enu_origin=false requires expected_enu_origin as an "
          "explicit offline datum assertion");
      return false;
    }
    this->enu_origin_validated_ = true;
    RCLCPP_WARN(
        this->get_logger(),
        "Live adapter ENU-origin validation is disabled; relying on the "
        "explicit expected_enu_origin assertion");
  }

  if (this->utm_enabled_) {
    RCLCPP_ERROR(
        this->get_logger(),
        "localization/utm_transform_path cannot be used with the canonical "
        "ENU map export. The exporter has already applied inverse(T_world_utm); "
        "applying it again would corrupt the map frame.");
    return false;
  }
  if (this->map_enu_origin_loaded_ &&
      (std::abs(this->map_roll_deg_) > 1.0e-6 ||
       std::abs(this->map_pitch_deg_) > 1.0e-6 ||
       std::abs(this->map_yaw_deg_) > 1.0e-6)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Map rotation overrides are incompatible with a manifested ENU map; "
        "re-export the map in the correct frame instead");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  // Load PCD file
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->map_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", this->map_path_.c_str());
    return false;
  }

  if (this->map_cloud->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }

  // Optional static map rotation to correct coordinate-frame differences from source map files.
  if (std::abs(this->map_roll_deg_) > 1e-6 ||
      std::abs(this->map_pitch_deg_) > 1e-6 ||
      std::abs(this->map_yaw_deg_) > 1e-6) {
    constexpr float kDeg2Rad = 0.017453292519943295f;
    const float roll = static_cast<float>(this->map_roll_deg_ * kDeg2Rad);
    const float pitch = static_cast<float>(this->map_pitch_deg_ * kDeg2Rad);
    const float yaw = static_cast<float>(this->map_yaw_deg_ * kDeg2Rad);

    Eigen::Affine3f map_tf = Eigen::Affine3f::Identity();
    map_tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*this->map_cloud, *this->map_cloud, map_tf.matrix());

    RCLCPP_INFO(this->get_logger(),
                "Applied map rotation [roll=%.2f, pitch=%.2f, yaw=%.2f] deg",
                this->map_roll_deg_, this->map_pitch_deg_, this->map_yaw_deg_);
  }

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %lu points", this->map_cloud->points.size());

  return true;
}

bool gicp_localization::LocalizationNode::prepareMapTarget() {
  if (!this->map_cloud || this->map_cloud->empty()) {
    return false;
  }

  const size_t raw_count = this->map_cloud->size();
  if (this->local_map_enable_) {
    const auto start = std::chrono::steady_clock::now();
    this->full_map_target_ = this->gicp.preprocessInputTarget(
        *this->map_cloud, this->map_voxel_size_,
        this->gicp_corr_randomness_, omp_get_max_threads());
    if (!this->full_map_target_) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to downsample and estimate covariances for the full map");
      return false;
    }

    // Release the raw map. The prepared cloud, its KD-tree, and its
    // covariances remain immutable for the node lifetime; local targets copy
    // point/covariance pairs and only build a bounded KD-tree.
    this->map_cloud = this->full_map_target_->cloud;
    this->buildLocalMapGrid();
    if (this->local_map_grid_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Prepared map contains no finite points");
      return false;
    }
    this->gicp.setInputTarget(this->full_map_target_);

    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO(
        this->get_logger(),
        "Prepared bounded local-map registration: raw=%lu prepared=%lu "
        "voxel=%.3fm covariances=%lu cells=%lu radius=%.1fm "
        "rebuild=%.1fm valid_offset=%.1fm time=%.2fs",
        raw_count, this->map_cloud->size(), this->map_voxel_size_,
        this->full_map_target_->covariances->size(),
        this->local_map_grid_.size(), this->local_map_radius_,
        this->local_map_rebuild_distance_,
        this->local_map_valid_center_offset_, elapsed_s);
  } else {
    if (this->map_voxel_size_ > 0.0) {
      auto map_ds = std::make_shared<pcl::PointCloud<PointType>>();
      pcl::VoxelGrid<PointType> vg;
      vg.setLeafSize(
          static_cast<float>(this->map_voxel_size_),
          static_cast<float>(this->map_voxel_size_),
          static_cast<float>(this->map_voxel_size_));
      vg.setInputCloud(this->map_cloud);
      vg.filter(*map_ds);
      if (map_ds->empty()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "map_voxel_size=%.3f produced an empty GICP target",
            this->map_voxel_size_);
        return false;
      }
      this->map_cloud = map_ds;
      RCLCPP_INFO(
          this->get_logger(),
          "Downsampled full-map GICP target: %lu -> %lu points "
          "(voxel=%.3fm)",
          raw_count, this->map_cloud->size(), this->map_voxel_size_);
    }

    this->gicp.setInputTarget(this->map_cloud);
    if (!this->gicp.calculateTargetCovariances()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to calculate full-map target covariances");
      return false;
    }
  }

  if (this->visualize_map_) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(
        this->map_voxel_size_vis_, this->map_voxel_size_vis_,
        this->map_voxel_size_vis_);
    vg.setInputCloud(this->map_cloud);
    vg.filter(*this->map_cloud_ds);
    RCLCPP_INFO(
        this->get_logger(), "Downsampled map for visualization: %lu points",
        this->map_cloud_ds->size());
  }

  return true;
}

void gicp_localization::LocalizationNode::start() {
  // Publish map periodically
  if (this->visualize_map_) {
    auto timer_callback = [this]() {
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*this->map_cloud_ds, map_msg);
      map_msg.header.stamp = this->now();
      map_msg.header.frame_id = this->map_frame;
      this->map_pub->publish(map_msg);
    };
    this->map_pub_timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
  }

  // Republish the configured initial pose (PoseStamped only — NOT TF) until
  // GICP produces a real result, so RViz has something to show before scans
  // arrive. We deliberately do NOT publish a map->base_link TF here because
  // under use_sim_time, this->now() returns 0 until /clock is active, and
  // a TF stamped at time 0 poisons the TF buffer with OLD_DATA warnings.
  if (this->initialized && this->debug_pub_enabled_) {
    auto initial_pose_cb = [this]() {
      if (this->last_gicp_valid_) {
        this->initial_pose_pub_timer_->cancel();
        return;
      }
      const rclcpp::Time stamp = this->now();
      // Skip while sim time is still 0 (clock not yet flowing)
      if (stamp.nanoseconds() == 0) {
        return;
      }
      Eigen::Matrix4f pose;
      {
        std::lock_guard<std::mutex> lock(this->pose_mutex);
        pose = this->current_pose;
      }
      this->dbg_initial_guess_pose_pub->publish(
          poseStampedFromMatrix(pose, stamp, this->map_frame));
    };
    this->initial_pose_pub_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200), initial_pose_cb);
  }
}

void gicp_localization::LocalizationNode::applyInitialPoseFromParams() {

  if (!this->use_param_initial_pose_) {
    return;
  }
  if (!this->enu_origin_validated_.load()) {
    this->pending_initial_pose_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Deferring configured initial pose until the live adapter ENU origin "
        "matches the map manifest");
    return;
  }

  Eigen::Vector3f position(static_cast<float>(this->initial_pose_x_),
                           static_cast<float>(this->initial_pose_y_),
                           static_cast<float>(this->initial_pose_z_));

  Eigen::AngleAxisf roll_angle(static_cast<float>(this->initial_pose_roll_), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(this->initial_pose_pitch_), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(this->initial_pose_yaw_), Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf orientation = yaw_angle * pitch_angle * roll_angle;
  orientation.normalize();

  // Pose values may be supplied in the LiDAR frame (matching GLIM's
  // traj_lidar.txt), but internally this node tracks the base_link pose in
  // world. When frame=="lidar", convert T_world_lidar -> T_world_base by
  // post-multiplying inv(baselink2lidar_T). This requires the URDF TF from
  // robot_state_publisher; defer until the first pointcloud if not yet cached.
  if (this->initial_pose_frame_ == "lidar") {
    if (!this->extrinsics_cached_) {
      this->pending_initial_pose_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Initial pose in lidar frame; deferring apply until baselink->lidar TF is cached");
      return;
    }
    const Eigen::Matrix3f& R_bl = this->extrinsics.baselink2lidar.R;
    const Eigen::Vector3f& t_bl = this->extrinsics.baselink2lidar.t;
    // T_world_base.t = R_world_lidar * (-R_base_lidar^T * t_base_lidar) + t_world_lidar
    // With lidar == lidar_front_link (no rotation in URDF) this simplifies to
    // t_world_base = t_world_lidar - R_world_lidar * R_base_lidar * (R_base_lidar^T * t_base_lidar).
    // We compute the general form: T_world_base = T_world_lidar * inv(T_base_lidar).
    Eigen::Matrix3f R_lb = R_bl.transpose();
    Eigen::Vector3f t_lb = -R_lb * t_bl;
    Eigen::Matrix3f R_world_lidar = orientation.toRotationMatrix();
    Eigen::Vector3f position_base = R_world_lidar * t_lb + position;
    Eigen::Quaternionf orientation_base(R_world_lidar * R_lb);
    orientation_base.normalize();
    RCLCPP_INFO(this->get_logger(),
                "Converted initial lidar pose -> base_link: [%.3f, %.3f, %.3f] (lidar) -> [%.3f, %.3f, %.3f] (base)",
                position.x(), position.y(), position.z(),
                position_base.x(), position_base.y(), position_base.z());
    position = position_base;
    orientation = orientation_base;
  }
  this->pending_initial_pose_ = false;

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = position;
    this->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = position;
    this->state.q = orientation;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = position;
    this->geo.prev_q = orientation;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->basePose.p = position;
  this->basePose.q = orientation;

  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = this->now();

  RCLCPP_INFO(this->get_logger(),
              "Initial pose loaded from parameters at [%.2f, %.2f, %.2f] m with RPY [%.2f, %.2f, %.2f] rad",
              this->initial_pose_x_, this->initial_pose_y_, this->initial_pose_z_,
              this->initial_pose_roll_, this->initial_pose_pitch_, this->initial_pose_yaw_);
}

void gicp_localization::LocalizationNode::applyInitialPose(const Eigen::Vector3f& p,
                                                          const Eigen::Quaternionf& q_in,
                                                          const rclcpp::Time& stamp,
                                                          const std::string& source) {

  Eigen::Quaternionf q = q_in;
  if (q.squaredNorm() < 1e-10f) {
    RCLCPP_WARN(this->get_logger(), "Received near-zero quaternion in initial pose, ignoring");
    return;
  }
  q.normalize();

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p;
    this->initialized = true;
    if (stamp.nanoseconds() > 0) {
      this->scan_stamp = stamp;
    }
  }

  // Reset geometric observer state to prevent drift from previous estimates
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = p;
    this->state.q = q;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = p;
    this->geo.prev_q = q;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->basePose.p = p;
  this->basePose.q = q;

  // Clear trajectory path on reinitialization
  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : this->now();

  RCLCPP_INFO(this->get_logger(), "Received initial pose (%s) at [%.2f, %.2f, %.2f]",
              source.c_str(), p.x(), p.y(), p.z());
}

void gicp_localization::LocalizationNode::callbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose) {

  if (!this->enu_origin_validated_.load()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring initial pose until the live adapter ENU origin matches the map");
    return;
  }
  if (!pose->header.frame_id.empty() &&
      pose->header.frame_id != this->map_frame) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Ignoring initial pose in frame '%s'; expected '%s'",
        pose->header.frame_id.c_str(), this->map_frame.c_str());
    return;
  }

  Eigen::Quaternionf q(
      pose->pose.pose.orientation.w,
      pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z);

  Eigen::Vector3f p(
      pose->pose.pose.position.x,
      pose->pose.pose.position.y,
      pose->pose.pose.position.z);

  this->applyInitialPose(p, q, pose->header.stamp, "PoseWithCovarianceStamped");
}

void gicp_localization::LocalizationNode::callbackPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in) {

  if (!this->primary_lidar_fov_validated_) {
    VerticalFovMeasurement fov;
    if (!verticalFovAccepted(
            *pc_in, this->lidar_min_vertical_fov_deg_,
            this->lidar_fov_min_valid_points_, &fov)) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Rejecting primary LiDAR startup cloud before SLAM: occupied "
          "vertical FOV %.2f deg (elevation %.2f..%.2f deg), required "
          ">= %.2f deg; occupancy=%zu/%zu bins (%zu points/bin required); "
          "valid sampled returns=%zu/%zu. Reason: %s.",
          fov.span_deg, fov.lower_deg, fov.upper_deg,
          this->lidar_min_vertical_fov_deg_, fov.occupied_bins,
          fov.occupancy_bins, fov.minimum_points_per_bin, fov.valid_points,
          fov.sampled_points, fov.reason);
      return;
    }
    this->primary_lidar_fov_validated_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Primary LiDAR startup FOV gate passed: occupied span %.2f deg "
        "(elevation %.2f..%.2f deg, occupancy=%zu/%zu bins, "
        "%zu valid sampled returns)",
        fov.span_deg, fov.lower_deg, fov.upper_deg, fov.occupied_bins,
        fov.occupancy_bins, fov.valid_points);
  }

  if (!this->lidar_fov_startup_complete_) {
    bool all_aux_validated = true;
    if (this->concat_enabled_) {
      for (const auto& aux : this->aux_lidars_) {
        std::lock_guard<std::mutex> lock(aux->mtx);
        if (!aux->vertical_fov_validated) {
          all_aux_validated = false;
          break;
        }
      }
    }
    if (!all_aux_validated) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Waiting for every configured auxiliary LiDAR to pass its startup "
          "FOV gate; localization has not started");
      return;
    }
    this->lidar_fov_startup_complete_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "LiDAR startup quality validation complete for primary + %zu "
        "auxiliary stream(s); runtime FOV checks disabled",
        this->concat_enabled_ ? this->aux_lidars_.size() : 0U);
  }

  if (!this->enu_origin_validated_.load()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /gps_p1/local_enu_origin to match the map manifest; "
        "point clouds are not being localized");
    return;
  }

  if (this->imu_only_mode_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU-only mode enabled: skipping pointcloud/GICP updates.");
    return;
  }

  // Enforce the Robin W wire contract on the unmerged primary. The official
  // driver publishes timestamp/FLOAT64 absolute Unix seconds, with a 10 us
  // point-time quantum inside one frame period.
  if (this->sensor == dlio::SensorType::SEYOND) {
    SeyondPointTimeRange primary_range;
    if (!seyondCloudTimeContractValid(
            *pc_in, this->seyond_frame_duration_s_, &primary_range)) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Rejecting Robin W primary cloud: expected timestamp/FLOAT64 "
          "absolute Unix seconds within the %.0f ms frame beginning at "
          "header.stamp (%zu points timed, %zu invalid zero timestamps). Install the "
          "pinned driver with PTP_sync/4_setup_lidar_ptp.sh.",
          this->seyond_frame_duration_s_ * 1.0e3, primary_range.count,
          primary_range.zero_timestamp_count);
      return;
    }
    // Track the real frame period. A configured value larger than the truth
    // loosens every bound above, and nothing else would ever notice.
    const double stamp_s = rclcpp::Time(pc_in->header.stamp).seconds();
    if (this->seyond_last_primary_stamp_s_ > 0.0) {
      const double dt = stamp_s - this->seyond_last_primary_stamp_s_;
      if (dt > 0.0 && dt < 1.0) {
        this->seyond_observed_frame_interval_s_ =
            this->seyond_observed_frame_interval_s_ < 0.0
                ? dt
                : 0.9 * this->seyond_observed_frame_interval_s_ + 0.1 * dt;
        ++this->seyond_frame_interval_samples_;
        // Warn only once the estimate has settled, and only in the direction
        // that matters: sensor faster than configured => gate too loose.
        if (this->seyond_frame_interval_samples_ > 50 &&
            this->seyond_observed_frame_interval_s_ <
                0.75 * this->seyond_frame_duration_s_) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 10000,
              "Robin W frame period looks like %.0f ms but "
              "localization/seyond_frame_duration_s is %.0f ms. Every "
              "wire-contract bound is correspondingly loose (a fused "
              "double-frame would pass). Set it to the real period.",
              this->seyond_observed_frame_interval_s_ * 1.0e3,
              this->seyond_frame_duration_s_ * 1.0e3);
        }
      }
    }
    this->seyond_last_primary_stamp_s_ = stamp_s;
  }

  // Multi-LiDAR concatenation: merge point-time-aligned aux scans into the primary cloud
  // before any other processing. Downstream steps (TF cache, manual field
  // extraction, per-point timestamp read, Y-flip, deskew, GICP) all run on the
  // merged cloud unchanged — primary frame_id, point_step, and field layout
  // are preserved.
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc =
      this->concat_enabled_ ? this->mergeAuxClouds(pc_in) : pc_in;

  // Strict multi-LiDAR merge (require_all_aux) returns nullptr when it skips an
  // incomplete scan so the degraded cloud is never registered. Drop this scan;
  // IMU/geometric propagation continues until a complete merged scan arrives.
  if (!pc) {
    return;
  }

  // Cache base_link -> lidar extrinsic from TF once. With
  // robot_state_publisher providing the URDF TF tree, this is the true
  // lever arm from the vehicle chassis (base_link) to the LiDAR sensor.
  // Deskewing below chains this as `frames[i] * baselink2lidar_T` so the
  // incoming points stay in their native LiDAR frame until then.
  if (!this->extrinsics_cached_) {
    // Resolve base_link -> lidar WITHOUT live TF first (URDF / static), so replay
    // without /tf_static or robot_state_publisher still localizes. Only fall back
    // to a live TF lookup if neither URDF nor a static transform is configured.
    if (this->resolveBaseLidarExtrinsicOffline(pc->header.frame_id)) {
      this->extrinsics_cached_ = true;
    } else {
      try {
        auto tf_bl = this->tf_buffer->lookupTransform(
            this->base_frame, pc->header.frame_id, tf2::TimePointZero);
        Eigen::Quaternionf q_bl(
            tf_bl.transform.rotation.w, tf_bl.transform.rotation.x,
            tf_bl.transform.rotation.y, tf_bl.transform.rotation.z);
        Eigen::Vector3f t_bl(
            tf_bl.transform.translation.x, tf_bl.transform.translation.y,
            tf_bl.transform.translation.z);
        this->extrinsics.baselink2lidar.R = q_bl.toRotationMatrix();
        this->extrinsics.baselink2lidar.t = t_bl;
        this->extrinsics.baselink2lidar_T.setIdentity();
        this->extrinsics.baselink2lidar_T.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
        this->extrinsics.baselink2lidar_T.block<3, 1>(0, 3) = t_bl;
        this->extrinsics_cached_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Cached baselink->lidar extrinsic from TF '%s': t=[%.3f,%.3f,%.3f]",
                    pc->header.frame_id.c_str(), t_bl.x(), t_bl.y(), t_bl.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for baselink->lidar TF ('%s' -> '%s'): %s. "
                             "Set localization/lidar_concat/urdf_path or localization/base_lidar_transform "
                             "for offline replay without /tf_static.",
                             this->base_frame.c_str(), pc->header.frame_id.c_str(), ex.what());
        return;
      }
    }
    // Apply the configured initial pose once the lever arm is known (either path).
    if (this->extrinsics_cached_ && this->pending_initial_pose_) {
      this->applyInitialPoseFromParams();
    }
  }

  if (!this->initialized) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for initialization (odom or initial pose)...");
    return;
  }

  this->scan_stamp = pc->header.stamp;
  this->t_prior_stamp_ = this->scan_stamp.seconds();
  this->last_scan_input_frame_ = pc->header.frame_id;

  // Convert to PCL format using manual field extraction for robustness
  pcl::PointCloud<PointType>::Ptr raw_scan = std::make_shared<pcl::PointCloud<PointType>>();

  if (pc->width == 0 || pc->height == 0) {
    RCLCPP_WARN(this->get_logger(),
                "Received empty point cloud (width=%u, height=%u)",
                pc->width, pc->height);
    return;
  }
  if (pc->point_step == 0 ||
      static_cast<size_t>(pc->width) >
          std::numeric_limits<size_t>::max() /
              static_cast<size_t>(pc->height)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Rejecting malformed PointCloud2 dimensions");
    return;
  }

  // Calculate number of points and require the tight layout used by the
  // single-pass byte decoder below. Row-padded organized clouds need a
  // row-aware decoder and are intentionally rejected here.
  const size_t num_points =
      static_cast<size_t>(pc->width) * static_cast<size_t>(pc->height);
  if (num_points >
      std::numeric_limits<size_t>::max() /
          static_cast<size_t>(pc->point_step)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Rejecting PointCloud2 whose byte size overflows size_t");
    return;
  }
  const size_t expected_data_size =
      num_points * static_cast<size_t>(pc->point_step);
  const size_t expected_row_step =
      static_cast<size_t>(pc->width) *
      static_cast<size_t>(pc->point_step);
  if (static_cast<size_t>(pc->row_step) != expected_row_step ||
      pc->data.size() != expected_data_size) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting non-tight PointCloud2 layout "
        "(row_step=%u expected=%zu data=%zu expected=%zu)",
        pc->row_step, expected_row_step, pc->data.size(),
        expected_data_size);
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2: width=%d, height=%d, num_points=%lu, data_size=%lu",
               pc->width, pc->height, num_points, pc->data.size());

  // Single-pass conversion: resolve field offsets once, then walk pc->data
  // exactly once doing xyz + intensity + per-point time + flip_y in the same
  // iteration.
  int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
  uint8_t i_type = 0;
  if (!findXYZOffsets(*pc, x_off, y_off, z_off)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud requires x/y/z FLOAT32/count=1 fields inside point_step");
    return;
  }
  for (const auto& field : pc->fields) {
    if (field.name == "intensity") {
      if (field.count == 1 && pointFieldFitsStep(field, pc->point_step)) {
        i_off = static_cast<int>(field.offset);
        i_type = field.datatype;
      } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Ignoring malformed intensity field outside point_step");
      }
      break;
    }
  }

  int time_off = -1;
  uint8_t time_datatype = 0;
  int time_count = 0;
  const bool has_time_field = findTimeField(*pc, time_off, time_datatype, time_count);

  // Enforce the documented sensor contract before any union decode.
  const auto matches_time_contract =
      [&pc](const char* name, uint8_t datatype) {
        for (const auto& field : pc->fields) {
          if (field.name == name) {
            return field.datatype == datatype && field.count == 1;
          }
        }
        return false;
      };
  const char* expected_time_contract = nullptr;
  bool time_contract_ok = true;
  if (this->sensor == dlio::SensorType::OUSTER) {
    expected_time_contract = "t/UINT32/count=1 (nanoseconds from sweep start)";
    time_contract_ok = matches_time_contract(
        "t", sensor_msgs::msg::PointField::UINT32);
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    expected_time_contract = "time/FLOAT32/count=1 (seconds from sweep start)";
    time_contract_ok = matches_time_contract(
        "time", sensor_msgs::msg::PointField::FLOAT32);
  } else if (this->sensor == dlio::SensorType::SEYOND) {
    expected_time_contract =
        "timestamp/FLOAT64/count=1 (absolute Unix seconds)";
    time_contract_ok = matches_time_contract(
        "timestamp", sensor_msgs::msg::PointField::FLOAT64);
  } else if (this->sensor == dlio::SensorType::HESAI ||
             this->sensor == dlio::SensorType::LIVOX) {
    expected_time_contract =
        "timestamp/FLOAT64/count=1 (absolute seconds for Hesai; numeric "
        "epoch nanoseconds for Livox)";
    time_contract_ok = matches_time_contract(
        "timestamp", sensor_msgs::msg::PointField::FLOAT64);
  }
  if (!time_contract_ok) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting point cloud: configured sensor timestamp contract is %s. "
        "For Robin W, install the pinned driver with "
        "PTP_sync/4_setup_lidar_ptp.sh.",
        expected_time_contract);
    return;
  }

  if (this->deskew_ && !has_time_field) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "dlio/deskew enabled but point cloud has no per-point time field "
                         "(t/time/timestamp)");
  } else if (this->sensor != dlio::SensorType::UNKNOWN && !has_time_field) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "No per-point time field (t/time/timestamp) in cloud; deskew will not "
                     "work until one is present");
  }

  raw_scan->points.resize(num_points);
  raw_scan->width = pc->width;
  raw_scan->height = pc->height;
  raw_scan->is_dense = pc->is_dense;

  const bool flip_y = this->flip_y_;
  const uint32_t point_step = pc->point_step;
  const uint8_t* base = pc->data.data();

  // Per-point body templated on the intensity reader, so the dispatch happens
  // once outside the loop and the compiler can inline + auto-vectorize.
  auto run = [&](auto&& read_intensity) {
    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* src = base + i * point_step;
      auto& dst = raw_scan->points[i];

      float x, y, z;
      std::memcpy(&x, src + x_off, sizeof(float));
      std::memcpy(&y, src + y_off, sizeof(float));
      std::memcpy(&z, src + z_off, sizeof(float));
      dst.x = x;
      dst.y = flip_y ? -y : y;
      dst.z = z;
      dst.intensity = read_intensity(src);
      clearPointTimeUnion(dst);
      if (has_time_field) {
        copyPointTimeFromCloud(src, time_off, time_datatype, time_count, point_step, this->sensor, dst);
      }
    }
  };

  try {
    if (i_off < 0) {
      run([](const uint8_t*) { return 0.0f; });
    } else {
      const uint8_t i_type_local = i_type;
      const int i_off_local = i_off;
      switch (i_type_local) {
        case sensor_msgs::msg::PointField::FLOAT32:
          run([i_off_local](const uint8_t* src) {
            float v;
            std::memcpy(&v, src + i_off_local, sizeof(float));
            return v;
          });
          break;
        case sensor_msgs::msg::PointField::UINT16:
          run([i_off_local](const uint8_t* src) {
            uint16_t v;
            std::memcpy(&v, src + i_off_local, sizeof(uint16_t));
            return static_cast<float>(v);
          });
          break;
        case sensor_msgs::msg::PointField::UINT8:
          run([i_off_local](const uint8_t* src) {
            return static_cast<float>(*(src + i_off_local));
          });
          break;
        case sensor_msgs::msg::PointField::FLOAT64:
          run([i_off_local](const uint8_t* src) {
            double v;
            std::memcpy(&v, src + i_off_local, sizeof(double));
            return static_cast<float>(v);
          });
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown intensity type %d, ignoring", i_type_local);
          run([](const uint8_t*) { return 0.0f; });
          break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during point cloud conversion: %s", e.what());
    return;
  }

  this->last_raw_point_count_ = raw_scan->points.size();

  // Store as original scan for deskewing
  this->original_scan = raw_scan;

  // Save dt for geometric observer BEFORE deskew (which overwrites prev_scan_stamp)
  this->observer_dt_ = (this->prev_scan_stamp > 0.0)
                        ? this->scan_stamp.seconds() - this->prev_scan_stamp
                        : 0.0;

  // Crop box filter in SENSOR frame, BEFORE deskew. deskewPointcloud()
  // transforms points into the world frame, so cropping afterward (in
  // preprocessPointCloud) would clip a box centered on the MAP ORIGIN, deleting
  // the whole scan once the vehicle is more than crop_size_ from the origin. A
  // crop box is inherently a sensor-relative near/far-field filter, so it must
  // run here on the raw lidar-frame cloud. This also covers the deskew-fallback
  // paths (which leave the scan in sensor frame).
  this->cropBoxFilterSensorFrame(this->original_scan);

  // Deskew using IMU
  this->deskewPointcloud();

  RCLCPP_DEBUG(this->get_logger(), "After deskewing: current_scan has %lu points",
               this->current_scan->points.size());

  // Preprocess the deskewed scan
  RCLCPP_DEBUG(this->get_logger(), "Before preprocessing: current_scan has %lu points",
               this->current_scan->points.size());

  this->preprocessPointCloud(this->current_scan);

  RCLCPP_DEBUG(this->get_logger(), "After preprocessing: current_scan has %lu points",
               this->current_scan->points.size());
  this->last_preprocessed_point_count_ = this->current_scan->points.size();

  if (this->current_scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud empty after preprocessing (original had %lu points)",
                raw_scan->points.size());

    if (this->debug_verbose_scan_log_) {
      RCLCPP_WARN(this->get_logger(),
                  "SCAN DEBUG | stamp=%.3f frame=%s raw=%zu pre=0 status=empty_after_preprocess guess=%s",
                  this->scan_stamp.seconds(), this->last_scan_input_frame_.c_str(),
                  this->last_raw_point_count_, poseSummary(this->current_pose).c_str());
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "After preprocessing: %lu points", this->current_scan->points.size());

  // Perform localization
  RCLCPP_DEBUG(this->get_logger(), "Calling performLocalization()...");
  this->performLocalization();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization() completed");

  // Publish results
  RCLCPP_DEBUG(this->get_logger(), "Calling publishPose()...");
  this->publishPose();
  RCLCPP_DEBUG(this->get_logger(), "publishPose() completed");
}

void gicp_localization::LocalizationNode::callbackAuxPointCloud(
    int aux_index, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (aux_index < 0 || static_cast<size_t>(aux_index) >= this->aux_lidars_.size()) {
    return;
  }

  auto& aux = *this->aux_lidars_[aux_index];
  {
    std::lock_guard<std::mutex> lock(aux.mtx);
    if (!aux.vertical_fov_validated) {
      VerticalFovMeasurement fov;
      if (!verticalFovAccepted(
              *msg, this->lidar_min_vertical_fov_deg_,
              this->lidar_fov_min_valid_points_, &fov)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "lidar_concat: rejecting startup cloud on '%s' before buffering: "
            "occupied vertical FOV %.2f deg (elevation %.2f..%.2f deg), "
            "required >= %.2f deg; occupancy=%zu/%zu bins "
            "(%zu points/bin required); valid sampled returns=%zu/%zu. "
            "Reason: %s.",
            aux.topic.c_str(), fov.span_deg, fov.lower_deg, fov.upper_deg,
            this->lidar_min_vertical_fov_deg_, fov.occupied_bins,
            fov.occupancy_bins, fov.minimum_points_per_bin, fov.valid_points,
            fov.sampled_points, fov.reason);
        return;
      }
      aux.vertical_fov_validated = true;
      RCLCPP_INFO(
          this->get_logger(),
          "Aux LiDAR startup FOV gate passed on '%s': occupied span %.2f deg "
          "(elevation %.2f..%.2f deg, occupancy=%zu/%zu bins, "
          "%zu valid sampled returns)",
          aux.topic.c_str(), fov.span_deg, fov.lower_deg, fov.upper_deg,
          fov.occupied_bins, fov.occupancy_bins, fov.valid_points);
    }
  }

  SeyondPointTimeRange point_time_range;
  if (this->sensor == dlio::SensorType::SEYOND &&
      !seyondCloudTimeContractValid(
          *msg, this->seyond_frame_duration_s_, &point_time_range)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "lidar_concat: rejecting Robin W aux cloud on '%s': expected "
        "timestamp/FLOAT64 absolute Unix seconds within its %.0f ms frame "
        "(%zu points timed, %zu invalid zero timestamps)",
        this->aux_lidars_[aux_index]->topic.c_str(),
        this->seyond_frame_duration_s_ * 1.0e3, point_time_range.count,
        point_time_range.zero_timestamp_count);
    return;
  }

  {
    std::lock_guard<std::mutex> lk(aux.mtx);
    AuxLidar::BufferedCloud buffered;
    buffered.msg = std::move(msg);
    buffered.point_time_valid = point_time_range.valid;
    buffered.point_time_min_s = point_time_range.min_s;
    buffered.point_time_max_s = point_time_range.max_s;
    aux.buffer.push_back(std::move(buffered));
    while (aux.buffer.size() > this->concat_buffer_size_) {
      aux.buffer.pop_front();
    }
  }
  aux.cv.notify_all();
}

// Resolve every aux LiDAR's T_primary_aux at startup WITHOUT live TF, so the
// multi-LiDAR merge works in offline replay (no robot_state_publisher / no
// /tf_static). Priority per aux: (1) URDF (the same av24.urdf GLIM reads, single
// source of truth), (2) a static row-major 4x4 from yaml, (3) leave unresolved
// so mergeAuxClouds() falls back to a runtime TF lookup. Any aux left unresolved
// here still works online exactly as before.
void gicp_localization::LocalizationNode::resolveAuxExtrinsicsOffline(
    const std::vector<std::vector<double>>& static_transforms) {
  // (1) URDF: parse once, resolve primary_frame <- aux.frame for each aux.
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool urdf_ok = false;
  if (!this->concat_urdf_path_.empty() && !this->concat_primary_frame_.empty()) {
    try {
      urdf_transforms = gicp_localization::parse_urdf_transforms(this->concat_urdf_path_);
      urdf_ok = true;
      RCLCPP_INFO(this->get_logger(), "lidar_concat: loaded URDF '%s' (primary_frame='%s')",
                  this->concat_urdf_path_.c_str(), this->concat_primary_frame_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF parse failed (%s); falling back to static/TF", e.what());
    }
  }

  for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
    auto& aux = *this->aux_lidars_[i];

    if (urdf_ok) {
      try {
        const Eigen::Isometry3d T = gicp_localization::compute_transform(
            urdf_transforms, this->concat_primary_frame_, aux.frame);
        aux.T_primary_aux = T.matrix().cast<float>();
        aux.extrinsic_cached = true;
        aux.extrinsic_source = "urdf";
        const Eigen::Vector3f t = aux.T_primary_aux.block<3, 1>(0, 3);
        RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
        continue;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF has no %s <- %s chain (%s); trying static/TF",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), e.what());
      }
    }

    // (2) Static row-major 4x4 from yaml.
    if (i < static_transforms.size() && static_transforms[i].size() == 16) {
      Eigen::Matrix4f M;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          M(r, c) = static_cast<float>(static_transforms[i][r * 4 + c]);
      aux.T_primary_aux = M;
      aux.extrinsic_cached = true;
      aux.extrinsic_source = "static";
      RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(primary <- %s) from static yaml: t=[%.3f, %.3f, %.3f]",
                  aux.frame.c_str(), M(0, 3), M(1, 3), M(2, 3));
      continue;
    }

    // (3) Unresolved -> runtime TF fallback (existing behavior in mergeAuxClouds).
    aux.extrinsic_source = "tf";
    RCLCPP_WARN(this->get_logger(),
                "lidar_concat: aux '%s' has no URDF/static extrinsic; will rely on live TF '%s' <- '%s' "
                "(requires /tf_static at runtime -- set urdf_path or aux_static_transforms for offline replay)",
                aux.topic.c_str(), this->concat_primary_frame_.c_str(), aux.frame.c_str());
  }
}

// Resolve the base_frame <- lidar_frame lever arm WITHOUT live TF, so full GICP
// localization works in offline replay (no robot_state_publisher / /tf_static).
// Priority: (1) URDF (the same av24.urdf used for aux extrinsics, via
// lidar_concat/urdf_path), (2) a static row-major 4x4 from yaml. Returns false if
// neither is available, leaving the caller to fall back to a live TF lookup.
bool gicp_localization::LocalizationNode::resolveBaseLidarExtrinsicOffline(const std::string& lidar_frame) {
  auto apply = [this](const Eigen::Matrix4f& T) {
    this->extrinsics.baselink2lidar.R = T.block<3, 3>(0, 0);
    this->extrinsics.baselink2lidar.t = T.block<3, 1>(0, 3);
    this->extrinsics.baselink2lidar_T = T;
  };

  // (1) URDF.
  if (!this->concat_urdf_path_.empty()) {
    try {
      auto urdf = gicp_localization::parse_urdf_transforms(this->concat_urdf_path_);
      const Eigen::Matrix4f T = gicp_localization::compute_transform(urdf, this->base_frame, lidar_frame).matrix().cast<float>();
      apply(T);
      RCLCPP_INFO(this->get_logger(),
                  "Resolved baselink->lidar (%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                  this->base_frame.c_str(), lidar_frame.c_str(), T(0, 3), T(1, 3), T(2, 3));
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "baselink->lidar URDF resolution failed (%s <- %s): %s; trying static/TF",
                  this->base_frame.c_str(), lidar_frame.c_str(), e.what());
    }
  }

  // (2) Static row-major 4x4 from yaml.
  if (this->base_lidar_static_.size() == 16) {
    Eigen::Matrix4f T;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        T(r, c) = static_cast<float>(this->base_lidar_static_[r * 4 + c]);
    apply(T);
    RCLCPP_INFO(this->get_logger(),
                "Resolved baselink->lidar from static yaml: t=[%.3f, %.3f, %.3f]", T(0, 3), T(1, 3), T(2, 3));
    return true;
  }

  return false;
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr
gicp_localization::LocalizationNode::mergeAuxClouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary) {

  // P4#3: reset the per-frame concat diagnostics. Any early return below
  // leaves them at "nothing merged", which is exactly what happened.
  this->concat_last_merged_aux_ = 0;
  this->concat_last_aux_dt_.assign(this->aux_lidars_.size(),
                                   std::numeric_limits<double>::quiet_NaN());
  this->concat_last_aux_points_.assign(this->aux_lidars_.size(), 0);

  if (this->aux_lidars_.empty()) return primary;

  // Strict-merge failure handler. Single routing point for every "required merge
  // can't complete" path: incomplete aux merge AND primary-precondition failures
  // (missing XYZ, non-tight/padded primary).
  //   - require_all_aux=false -> returns false: degraded merging is allowed, the
  //     caller localizes on whatever LiDARs merged (front + any available aux).
  //   - require_all_aux=true  -> returns true: the caller returns nullptr so the
  //     degraded cloud is NEVER registered (the scan is skipped; IMU propagation
  //     continues). Past max_consecutive_aux_merge_failures the node either aborts
  //     (abort_on_merge_failure=true) or keeps skipping with a louder warning
  //     (abort_on_merge_failure=false). A fully merged scan resets the counter.
  auto on_required_failure = [this](size_t got, const char* reason) -> bool {
    if (!this->concat_require_all_aux_) return false;  // degraded merging allowed
    ++this->concat_consec_fail_;
    const bool over_budget = this->concat_consec_fail_ > this->concat_max_consec_fail_;
    if (over_budget && this->concat_abort_on_merge_failure_) {
      RCLCPP_FATAL(this->get_logger(),
                   "lidar_concat: multi-LiDAR merge REQUIRED but could not complete (%s) for %d consecutive "
                   "scans; abort_on_merge_failure=true -> shutting down. Set require_all_aux=false to localize "
                   "on available LiDARs, or abort_on_merge_failure=false to keep skipping non-fatally.",
                   reason, this->concat_consec_fail_);
      rclcpp::shutdown();
      throw std::runtime_error("lidar_concat: required multi-LiDAR merge failed");
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "lidar_concat: REQUIRED merge incomplete (%zu/%zu aux): %s for %d consecutive scan(s) "
                          "(budget %d)%s -- skipping scan (degraded cloud NOT registered).",
                          got, this->aux_lidars_.size(), reason, this->concat_consec_fail_,
                          this->concat_max_consec_fail_, over_budget ? ", budget exceeded (non-fatal)" : "");
    return true;  // skip this scan
  };

  const double t_primary = rclcpp::Time(primary->header.stamp).seconds();
  const SeyondPointTimeRange primary_time_range =
      this->sensor == dlio::SensorType::SEYOND
          ? decodeSeyondPointTimeRange(*primary)
          : SeyondPointTimeRange{};
  const uint32_t point_step = primary->point_step;
  const std::string& primary_frame = primary->header.frame_id;

  int x_off, y_off, z_off;
  if (!findXYZOffsets(*primary, x_off, y_off, z_off)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: cannot find xyz fields in primary cloud (frame='%s')",
                         primary_frame.c_str());
    if (on_required_failure(0, "primary cloud missing xyz fields")) return nullptr;
    return primary;
  }

  // Concatenation treats each cloud as a TIGHT array of point_step-sized points
  // (it byte-appends aux data and re-counts by point_step). A row-padded cloud
  // (row_step > width*point_step, i.e. data.size() != width*height*point_step)
  // would make the byte-count include padding. Organized-but-tight (height>1, no
  // padding) is fine to flatten; only padding is rejected. Reject the primary
  // loudly rather than silently miscounting.
  if (point_step == 0 || primary->data.size() != static_cast<size_t>(primary->width) * primary->height * point_step) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: primary cloud is organized/padded (data=%zu, width=%u, height=%u, step=%u); "
                         "skipping concat (only tight clouds can be byte-appended)",
                         primary->data.size(), primary->width, primary->height, point_step);
    if (on_required_failure(0, "primary cloud organized/padded (non-tight)")) return nullptr;
    return primary;
  }

  // Start the merged cloud as a copy of the primary; we'll append aux bytes.
  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  // Count points from the byte buffer (equals width*height for the tight cloud
  // validated above) so merged width/row_step always match the appended bytes.
  size_t total_points = primary->data.size() / point_step;
  size_t merged_aux_count = 0;

  // Reserve once for primary + all aux clouds (assuming roughly equal sizes).
  // Avoids per-aux reallocations as we grow merged->data.
  merged->data.reserve(primary->data.size() * (1 + this->aux_lidars_.size()));
  // A single deadline bounds the whole merge. Waiting independently for each
  // aux until "now + timeout" would multiply latency by the number of sensors.
  const auto future_wait_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              this->concat_future_sweep_wait_timeout_));

  for (size_t aux_i = 0; aux_i < this->aux_lidars_.size(); ++aux_i) {
    auto& aux = *this->aux_lidars_[aux_i];

    // Cache T_primary_aux from TF on first use. Skip this aux until TF is available.
    if (!aux.extrinsic_cached) {
      try {
        auto tf = this->tf_buffer->lookupTransform(
            primary_frame, aux.frame, tf2::TimePointZero);
        Eigen::Quaternionf q(
            tf.transform.rotation.w, tf.transform.rotation.x,
            tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Vector3f t(
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z);
        aux.T_primary_aux.setIdentity();
        aux.T_primary_aux.block<3, 3>(0, 0) = q.toRotationMatrix();
        aux.T_primary_aux.block<3, 1>(0, 3) = t;
        aux.extrinsic_cached = true;
        RCLCPP_INFO(this->get_logger(),
                    "lidar_concat: cached T(%s <- %s): t=[%.3f, %.3f, %.3f]",
                    primary_frame.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "lidar_concat: waiting for TF '%s' -> '%s': %s",
                             primary_frame.c_str(), aux.frame.c_str(), ex.what());
        continue;
      }
    }

    // Pick the aux sweep whose absolute point-time endpoints best agree with
    // the primary. Header time is only a tie-break for the Robin W path.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr match;
    double best_range_delta = std::numeric_limits<double>::infinity();
    double best_header_delta = std::numeric_limits<double>::infinity();
    {
      std::unique_lock<std::mutex> lk(aux.mtx);
      if (this->concat_future_sweep_wait_timeout_ > 0.0) {
        aux.cv.wait_until(lk, future_wait_deadline, [&]() {
          bool has_match = false;
          double newest_time = -std::numeric_limits<double>::infinity();
          for (const auto& buffered : aux.buffer) {
            const double header_time =
                rclcpp::Time(buffered.msg->header.stamp).seconds();
            newest_time = std::max(
                newest_time,
                primary_time_range.valid && buffered.point_time_valid
                    ? buffered.point_time_min_s
                    : header_time);
            SeyondPointTimeRange buffered_range;
            buffered_range.valid = buffered.point_time_valid;
            buffered_range.min_s = buffered.point_time_min_s;
            buffered_range.max_s = buffered.point_time_max_s;
            const double delta =
                primary_time_range.valid && buffered.point_time_valid
                    ? pointTimeEndpointDelta(
                          primary_time_range, buffered_range)
                    : std::abs(header_time - t_primary);
            has_match = has_match ||
                        delta <= this->concat_sweep_time_threshold_;
          }
          const double target_time =
              primary_time_range.valid ? primary_time_range.min_s : t_primary;
          return has_match ||
                 newest_time >
                     target_time + this->concat_sweep_time_threshold_;
        });
      }
      for (const auto& buffered : aux.buffer) {
        const double header_delta = std::abs(
            rclcpp::Time(buffered.msg->header.stamp).seconds() - t_primary);
        SeyondPointTimeRange buffered_time_range;
        buffered_time_range.valid = buffered.point_time_valid;
        buffered_time_range.min_s = buffered.point_time_min_s;
        buffered_time_range.max_s = buffered.point_time_max_s;
        const double range_delta =
            primary_time_range.valid && buffered.point_time_valid
                ? pointTimeEndpointDelta(
                      primary_time_range, buffered_time_range)
                : header_delta;
        if (range_delta < best_range_delta ||
            (range_delta == best_range_delta &&
             header_delta < best_header_delta)) {
          best_range_delta = range_delta;
          best_header_delta = header_delta;
          match = buffered.msg;
        }
      }
    }
    if (std::isfinite(best_range_delta)) {
      aux.endpoint_delta_sum += best_range_delta;
      aux.endpoint_delta_min =
          std::min(aux.endpoint_delta_min, best_range_delta);
      aux.endpoint_delta_max =
          std::max(aux.endpoint_delta_max, best_range_delta);
      ++aux.endpoint_delta_count;
      if (best_range_delta > this->concat_sweep_time_threshold_) {
        ++aux.endpoint_delta_reject_count;
      }
      if (aux.endpoint_delta_count % 512 == 0) {
        const double reject_fraction =
            static_cast<double>(aux.endpoint_delta_reject_count) /
            static_cast<double>(aux.endpoint_delta_count);
        const double mean_delta =
            aux.endpoint_delta_sum /
            static_cast<double>(aux.endpoint_delta_count);
        if (reject_fraction > 0.10) {
          RCLCPP_WARN(
              this->get_logger(),
              "lidar_concat: '%s' best sweep endpoint delta over %lu "
              "evaluations: mean=%.1fms min=%.1fms max=%.1fms gate=%.1fms "
              "rejected=%.1f%%. Tune from this evidence while staying below "
              "the %.1fms half-period ceiling.",
              aux.topic.c_str(),
              static_cast<unsigned long>(aux.endpoint_delta_count),
              mean_delta * 1.0e3, aux.endpoint_delta_min * 1.0e3,
              aux.endpoint_delta_max * 1.0e3,
              this->concat_sweep_time_threshold_ * 1.0e3,
              reject_fraction * 100.0,
              this->seyond_frame_duration_s_ * 0.5e3);
        } else {
          RCLCPP_INFO(
              this->get_logger(),
              "lidar_concat: '%s' best sweep endpoint delta over %lu "
              "evaluations: mean=%.1fms min=%.1fms max=%.1fms gate=%.1fms "
              "rejected=%.1f%%",
              aux.topic.c_str(),
              static_cast<unsigned long>(aux.endpoint_delta_count),
              mean_delta * 1.0e3, aux.endpoint_delta_min * 1.0e3,
              aux.endpoint_delta_max * 1.0e3,
              this->concat_sweep_time_threshold_ * 1.0e3,
              reject_fraction * 100.0);
        }
      }
    }
    if (!match ||
        best_range_delta > this->concat_sweep_time_threshold_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "lidar_concat: no point-time match for '%s' within %.3fs "
                   "of primary t=%.3f (best_endpoint_delta=%.3fs)",
                   aux.topic.c_str(), this->concat_sweep_time_threshold_,
                   t_primary, match ? best_range_delta : -1.0);
      continue;
    }
    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!auxSchemaMatchesPrimary(*match, *primary, schema_reason)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — PointCloud2 schema mismatch vs primary: %s. "
                           "(Merged cloud uses the primary field layout; appending mismatched aux bytes "
                           "would misread them. Normalize the aux layout upstream to enable concatenation.)",
                           aux.topic.c_str(), schema_reason.c_str());
      continue;
    }

    // Append aux bytes directly into merged->data, then transform xyz + shift
    // timestamps in place over the just-appended region. No intermediate copy.
    // If validation fails after the append, roll back the resize so a malformed
    // aux scan can't leak into the merged cloud in its own (un-transformed) frame.
    // Reject an organized/padded or otherwise non-tight aux: byte-appending it
    // (or counting by point_step) would desync points from the field layout.
    // Requires data.size() == width*height*point_step (subsumes the multiple-of-
    // point_step check). Organized-but-tight is acceptable; only padding fails.
    if ((match->data.size() % point_step) != 0 ||
        match->data.size() != static_cast<size_t>(match->width) * match->height * point_step) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — non-tight cloud (data=%zu, width=%u, height=%u, step=%u)",
                           aux.topic.c_str(), match->data.size(), match->width, match->height, point_step);
      continue;
    }
    const size_t old_size = merged->data.size();
    merged->data.insert(merged->data.end(), match->data.begin(), match->data.end());
    uint8_t* appended = merged->data.data() + old_size;
    const size_t aux_pts = match->data.size() / point_step;

    int ax, ay, az;
    if (!findXYZOffsets(*match, ax, ay, az)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — no x/y/z fields in PointCloud2",
                           aux.topic.c_str());
      merged->data.resize(old_size);
      continue;
    }
    transformCloudData(appended, aux_pts, point_step, ax, ay, az, aux.T_primary_aux);

    int time_off;
    uint8_t time_dt_type;
    int time_count;
    const bool has_time_field = findTimeField(*match, time_off, time_dt_type, time_count);
    if (has_time_field) {
      // dt = aux header - primary header. Adding dt rebases aux per-point times
      // onto the primary clock so deskewing sees one coherent sweep.
      const double dt = rclcpp::Time(match->header.stamp).seconds() - t_primary;
      const bool absolute_time =
          this->sensor == dlio::SensorType::SEYOND ||
          this->sensor == dlio::SensorType::HESAI ||
          this->sensor == dlio::SensorType::LIVOX;
      // Robin W already carries absolute capture time; relative layouts alone
      // receive the inter-header rebase.
      shiftCloudTimestamps(
          appended, aux_pts, point_step, time_off, time_dt_type, time_count,
          dt, absolute_time);
    } else if (this->deskew_) {
      // Without per-point timestamps the aux rays would deskew against the
      // primary scan's IMU integration with a stale (aux-header) reference,
      // smearing them. Drop the aux when deskew is enabled and times are absent.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — deskew enabled but no time field found",
                           aux.topic.c_str());
      merged->data.resize(old_size);
      continue;
    }

    // Accumulate the byte-derived count (matches the bytes actually appended),
    // not width*height, so merged->width/row_step stay consistent with data.
    total_points += aux_pts;
    ++merged_aux_count;

    // P4#3: per-frame + running merge-timing diagnostics. signed_dt is the
    // aux-vs-primary HEADER offset (not the |dt| used for matching): a stable
    // nonzero mean across the run is the signature of a constant per-aux
    // clock offset against the P1 timebase, which a per-aux time-offset
    // correction upstream could absorb (and which inflates deskew error at
    // high yaw rates).
    const double signed_dt = rclcpp::Time(match->header.stamp).seconds() - t_primary;
    this->concat_last_aux_dt_[aux_i] = signed_dt;
    this->concat_last_aux_points_[aux_i] = static_cast<int>(aux_pts);
    aux.dt_sum += signed_dt;
    aux.dt_min = std::min(aux.dt_min, signed_dt);
    aux.dt_max = std::max(aux.dt_max, signed_dt);
    if (++aux.dt_count % 512 == 0) {  // ~every 50 s at 10 Hz
      RCLCPP_INFO(this->get_logger(),
                  "lidar_concat: '%s' header offset vs primary over %lu merges: "
                  "mean=%+.1f ms, min=%+.1f ms, max=%+.1f ms%s",
                  aux.topic.c_str(), static_cast<unsigned long>(aux.dt_count),
                  1e3 * aux.dt_sum / static_cast<double>(aux.dt_count),
                  1e3 * aux.dt_min, 1e3 * aux.dt_max,
                  std::abs(aux.dt_sum / static_cast<double>(aux.dt_count)) > 0.02
                      ? " — mean >20 ms: likely constant clock offset, consider a per-aux time correction"
                      : "");
    }
  }

  this->concat_last_merged_aux_ = static_cast<int>(merged_aux_count);

  // The merged cloud is unorganized (height=1); width = total appended points.
  // Compute row_step in size_t so the point_step*total_points multiply cannot
  // overflow before the (message-mandated) uint32 assignment.
  merged->width = static_cast<uint32_t>(total_points);
  merged->height = 1;
  merged->is_dense = false;
  merged->row_step = static_cast<uint32_t>(static_cast<size_t>(point_step) * total_points);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "lidar_concat: merged %zu/%zu aux scans, total %zu points",
                       merged_aux_count, this->aux_lidars_.size(), total_points);

  // Strict guard: a REQUIRED multi-LiDAR merge that stays incomplete must not be
  // silently localized on fewer LiDARs (the run_5 "merged 0/2" failure mode). The
  // primary-precondition bail-outs above route through the same handler. When
  // require_all_aux is set, an incomplete merge SKIPS the scan (returns nullptr);
  // otherwise the degraded cloud is returned and localized. A fully merged scan
  // resets the budget.
  if (merged_aux_count < this->aux_lidars_.size()) {
    if (on_required_failure(merged_aux_count, "incomplete aux merge")) return nullptr;
  } else {
    this->concat_consec_fail_ = 0;
  }

  return merged;
}

void gicp_localization::LocalizationNode::deskewPointcloud() {

  // REVIEW FIX: reset the per-frame sweep-span diagnostic up front so early
  // returns (deskew off, unsupported sensor, empty IMU buffer, ...) publish
  // -1 instead of the previous frame's stale span.
  this->last_scan_time_span_s_ = -1.0;

  if (!this->deskew_) {
    this->current_scan = this->original_scan;

    // Even without per-point deskewing, integrate IMU to get a motion-predicted
    // T_prior so GICP starts from an IMU-advanced pose instead of the stale last result.
    if (this->first_imu_received && this->prev_scan_stamp > 0.0) {
      std::vector<double> single_ts = {this->scan_stamp.seconds()};
      {
        double latest_imu = 0.0;
        {
          std::lock_guard<std::mutex> lock(this->mtx_imu);
          if (!this->imu_buffer.empty()) latest_imu = this->imu_buffer.front().stamp;
        }
        if (latest_imu > 0.0 && single_ts[0] > latest_imu)
          single_ts[0] = latest_imu;
      }
      auto frames = this->integrateImu(this->prev_scan_stamp, this->basePose.q,
                                        this->basePose.p, this->prev_vel, single_ts);
      if (frames.size() == 1 && matrixFinite(frames[0])) {
        this->T_prior = frames[0];
      } else {
        this->T_prior = this->current_pose;
      }
    } else {
      this->T_prior = this->current_pose;
    }

    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  if (!this->first_imu_received) {
    // The registration path treats every deskew-enabled scan as world-frame
    // input. Preserve that invariant even before the first IMU sample: this is
    // a rigid prior transform (no per-point compensation), not raw
    // sensor-frame data passed to an identity-seeded optimizer.
    this->T_prior = this->current_pose;
    auto world_scan = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(
        *this->original_scan, *world_scan,
        this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = world_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      std::make_shared<pcl::PointCloud<PointType>>(1, this->original_scan->points.size());

  // Individual point timestamps should be relative to this time
  double sweep_ref_time = this->scan_stamp.seconds();

  // Sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<double(const PointType&)> extract_point_time_from_point;
  bool deskew_time_ready = false;

  if (this->sensor == dlio::SensorType::OUSTER) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.t < p2.t; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.t) * 1e-9;
    };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.time);
    };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::SEYOND ||
             this->sensor == dlio::SensorType::HESAI) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp; };
    deskew_time_ready = true;
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp * 1e-9; };
    deskew_time_ready = true;
  }

  if (!deskew_time_ready) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported localization/sensor_type for deskew; using scan without motion "
                         "compensation");
    // Deskew-mode fallback: performLocalization assumes a WORLD-frame cloud
    // (identity guess) whenever deskew_ is on, so a fallback must still
    // transform by T_prior * baselink2lidar_T — feeding the raw sensor-frame
    // cloud registers garbage and inflates the failure streak / snap counter.
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*this->original_scan, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }
  // Copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // Extract timestamps from points and build list of unique timestamps
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;

  double prev_timestamp = -1.0;
  for (size_t i = 0; i < deskewed_scan_->points.size(); i++) {
    double curr_timestamp = extract_point_time_from_point(deskewed_scan_->points[i]);
    if (std::abs(curr_timestamp - prev_timestamp) > 1e-9) {  // Unique timestamp
      timestamps.push_back(curr_timestamp);
      unique_time_indices.push_back(i);
      prev_timestamp = curr_timestamp;
    }
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  if (timestamps.empty()) {
    RCLCPP_WARN(this->get_logger(), "No timestamps extracted from point cloud, skipping deskewing");
    this->last_scan_time_span_s_ = -1.0;
    // Deskew-mode fallback: world-frame transform required (see the
    // !deskew_time_ready fallback above). Also update prev_scan_stamp —
    // this path previously skipped it, leaving the next IMU integration
    // window anchored at a stale scan time.
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*this->original_scan, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return;
  }

  // P4#3: per-frame sweep time span of the (merged) cloud. A healthy 3-LiDAR
  // merge spans ~1 sweep period; a much larger span means a badly-offset aux
  // got rebased far from the primary and is being deskewed across a long arc.
  this->last_scan_time_span_s_ = timestamps.back() - timestamps.front();

  // A Robin W sweep that collapses to a single unique timestamp means every point
  // shares one time, so deskew degenerates to a rigid transform (no motion
  // compensation). This is the symptom of a wrong per-point time encoding (e.g.
  // global_shutter/collapsed times) -- warn so the operator can fix the source.
  if (this->sensor == dlio::SensorType::SEYOND && this->deskew_ && timestamps.size() == 1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Robin W deskew collapsed to a single timestamp (%zu points share one time); "
                         "deskew reduced to a rigid transform. Check the per-point time encoding.",
                         deskewed_scan_->points.size());
  }

  int median_pt_index = timestamps.size() / 2;

  // Don't process scans on first iteration
  if (this->prev_scan_stamp == 0.0) {
    this->prev_scan_stamp = this->scan_stamp.seconds();
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    return;
  }

  // Check if we have sufficient IMU history
  // We need IMU data from BEFORE prev_scan_stamp to integrate
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (this->imu_buffer.empty()) {
      RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping deskewing");
      // Deskew-mode fallback: world-frame transform required (see the
      // !deskew_time_ready fallback above).
      this->T_prior = this->current_pose;
      pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
      this->current_scan = deskewed_scan_;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }

    // Check if oldest IMU is before prev_scan_stamp (need some margin)
    double oldest_imu_time = this->imu_buffer.back().stamp;
    double margin = 0.1;  // 100ms margin

    if (oldest_imu_time > this->prev_scan_stamp - margin) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for sufficient IMU history (oldest: %.3f, need: %.3f). Skipping deskewing.",
                           oldest_imu_time, this->prev_scan_stamp);
      this->T_prior = this->current_pose;
      pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
      this->current_scan = deskewed_scan_;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }
  }

  // IMU prior & deskewing
  RCLCPP_DEBUG(this->get_logger(),
               "Integrating IMU: prev_stamp=%.3f, pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]",
               this->prev_scan_stamp,
               this->basePose.p.x(), this->basePose.p.y(), this->basePose.p.z(),
               this->prev_vel.x(), this->prev_vel.y(), this->prev_vel.z());

  // Clamp timestamps to IMU buffer extent: when per-point timestamps are absent
  // or collapsed by the driver, sorted_timestamps.back() == scan_stamp, which
  // may be a few ms ahead of the latest IMU sample due to callback ordering.
  // Clamping avoids rejecting the integration.
  {
    double latest_imu = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->mtx_imu);
      if (!this->imu_buffer.empty()) latest_imu = this->imu_buffer.front().stamp;
    }
    if (latest_imu > 0.0) {
      for (auto& ts : timestamps) {
        if (ts > latest_imu) ts = latest_imu;
      }
    }
  }

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->basePose.q, this->basePose.p,
                              this->prev_vel, timestamps);

  // If there are no frames between the start and end of the sweep, use previous transform
  if (frames.size() != timestamps.size()) {
    RCLCPP_WARN(this->get_logger(),
                "IMU integration failed! Got %lu frames for %lu timestamps. "
                "Time range: [%.3f, %.3f], IMU buffer size: %lu, first IMU: %.3f",
                frames.size(), timestamps.size(),
                this->prev_scan_stamp, timestamps.back(),
                this->imu_buffer.size(),
                this->imu_buffer.empty() ? 0.0 : this->imu_buffer.back().stamp);
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Deskewing OK: %lu frames, scan time [%.3f, %.3f]",
                       frames.size(), timestamps.front(), timestamps.back());

  // Update prior to be the estimated pose at the median time of the scan
  this->T_prior = frames[median_pt_index];
  this->t_prior_stamp_ = timestamps[median_pt_index];

  // Deskew each point using its timestamp
  #pragma omp parallel for
  for (size_t i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // Transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->current_scan = deskewed_scan_;
  this->prev_scan_stamp = this->scan_stamp.seconds();
}

// Sensor-frame crop box. Applied to the raw lidar-frame cloud BEFORE deskew (see
// the call site in the scan handler). The box is axis-aligned ±crop_size_ around
// the lidar origin -- a near/far-field filter. It must NOT run after deskew,
// where points are in the world frame and the box would be centered on the map
// origin (clipping the whole scan far from origin).
void gicp_localization::LocalizationNode::cropBoxFilterSensorFrame(pcl::PointCloud<PointType>::Ptr& cloud) {
  if (!cloud || cloud->points.empty()) return;
  if (this->crop_size_ > 0.0 && this->crop_size_ < 1000.0) {  // Only apply if reasonable size
    const size_t original_size = cloud->points.size();
    pcl::CropBox<PointType> crop;
    crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));
    crop.setInputCloud(cloud);
    crop.filter(*cloud);
    RCLCPP_DEBUG(this->get_logger(), "Crop box (sensor frame): %lu -> %lu points", original_size, cloud->points.size());
  }
}

void gicp_localization::LocalizationNode::preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud) {

  size_t original_size = cloud->points.size();
  (void)original_size;

  // NOTE: the crop box is intentionally NOT applied here. After deskew the cloud
  // is in the world frame, so an origin-centered box would clip the scan far from
  // the map origin. Cropping happens in cropBoxFilterSensorFrame() before deskew.

  // Voxel filter
  if (this->vf_use_) {
    size_t before_voxel = cloud->points.size();
    pcl::PointCloud<PointType> filtered;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
    voxel.setInputCloud(cloud);
    voxel.filter(filtered);
    if (filtered.points.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Voxel filter removed ALL %lu input points (res=%.3f m) — "
                  "check point coordinates and frame; keeping unfiltered cloud for this scan",
                  before_voxel, this->vf_res_);
      // Leave cloud unchanged — no copy needed.
    } else {
      *cloud = std::move(filtered);
      const double reduction = 1.0 - static_cast<double>(cloud->points.size()) /
                                         static_cast<double>(before_voxel);
      if (reduction > 0.99) {
        RCLCPP_WARN(this->get_logger(),
                    "Voxel filter removed %.1f%% of points (%lu -> %lu, res=%.3f m) — "
                    "check point frame/coordinates",
                    reduction * 100.0, before_voxel, cloud->points.size(), this->vf_res_);
      }
      RCLCPP_DEBUG(this->get_logger(), "Voxel filter: %lu -> %lu points", before_voxel, cloud->points.size());
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "Preprocessing: %lu -> %lu points total", original_size, cloud->points.size());
}

void gicp_localization::LocalizationNode::performLocalization() {

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Acquiring mutex lock...");
  std::lock_guard<std::mutex> lock(this->pose_mutex);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Mutex acquired");

  const Eigen::Matrix4f T_base_lidar = this->extrinsics.baselink2lidar_T;
  const Eigen::Vector3f target_center =
      (this->T_prior * T_base_lidar).block<3, 1>(0, 3);
  if (!this->ensureLocalMapTarget(target_center)) {
    ++this->consecutive_failures_;
    this->maybeSnapPoseToGT("local-map target unavailable");
    RCLCPP_WARN(
        this->get_logger(),
        "Skipping GICP: no sufficiently populated local target at "
        "[%.2f, %.2f] (failure streak=%d)",
        target_center.x(), target_center.y(), this->consecutive_failures_);
    return;
  }

  // Set source cloud
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Setting input source (%lu points)...",
               this->current_scan->points.size());
  this->gicp.setInputSource(this->current_scan);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Input source set");

  // Align using IMU-based prior as initial guess (if deskewing is enabled)
  // The adapter retains a PCL-shaped output argument for migration
  // compatibility. GICP_plusplus consumes only the estimated transform.
  pcl::PointCloud<PointType> aligned_scratch;

  // When deskewing is enabled, points are already in world frame at T_prior,
  // so GICP initial guess is Identity and final pose = T_corr * T_prior.
  // When deskewing is disabled, points are still in lidar frame, so seed/solve
  // in map<-lidar, then convert the optimizer output back to map<-base.
  const Eigen::Matrix4f T_lidar_base = T_base_lidar.inverse();
  Eigen::Matrix4f initial_guess = this->deskew_
      ? Eigen::Matrix4f::Identity()
      : (this->T_prior * T_base_lidar);
  Eigen::Matrix4f guess_pose_map = this->T_prior;

  double guess_from_last_trans = 0.0;
  double guess_from_last_rot_deg = 0.0;
  if (this->last_gicp_valid_) {
    guess_from_last_trans = deltaTranslationNorm(this->last_gicp_pose_, guess_pose_map);
    guess_from_last_rot_deg = rotationDistanceDeg(this->last_gicp_pose_, guess_pose_map);
  }

  bool full_6dof_this_scan = this->gicp_dof_mode_ == "6dof";
  if (!full_6dof_this_scan && this->gicp_full6dof_every_n_ > 0 &&
      (++this->gicp_dof_scan_counter_ %
       static_cast<uint64_t>(this->gicp_full6dof_every_n_)) == 0) {
    full_6dof_this_scan = true;
  }
  const bool fix_roll_pitch =
      !full_6dof_this_scan &&
      (this->gicp_dof_mode_ == "4dof" ||
       this->gicp_dof_mode_ == "3dof");
  const bool fix_yaw =
      !full_6dof_this_scan && this->gicp_dof_mode_ == "3dof";
  this->gicp.setDoFMask(fix_roll_pitch, fix_roll_pitch, fix_yaw);

  if (this->gicp_prior_yaw_info_ > 0.0 ||
      this->gicp_prior_rollpitch_info_ > 0.0) {
    this->gicp.setRotationPrior(
        initial_guess.block<3, 3>(0, 0).cast<double>(),
        Eigen::Vector3d(
            this->gicp_prior_rollpitch_info_,
            this->gicp_prior_rollpitch_info_,
            this->gicp_prior_yaw_info_));
  } else {
    this->gicp.clearRotationPrior();
  }

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Starting GICP alignment...");
  auto start = std::chrono::high_resolution_clock::now();
  this->gicp.align(aligned_scratch, initial_guess);
  auto end = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: GICP alignment completed");

  double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  double fitness_score = this->gicp.getFitnessScore();
  this->last_fitness_score_ = fitness_score;
  double final_error = this->gicp.getFinalError();
  bool converged = this->gicp.hasConverged();
  int num_correspondences = this->gicp.num_correspondences;
  double correspondence_ratio =
      this->current_scan->points.empty()
          ? 0.0
          : static_cast<double>(num_correspondences) / static_cast<double>(this->current_scan->points.size());
  const Eigen::Matrix4f optimizer_solution = this->gicp.getFinalTransformation();
  const Eigen::Matrix<double, 6, 6>& final_hessian = this->gicp.getFinalHessian();

  // small_gicp reports H in the optimizer's right-multiplicative tangent.
  // Deskewed points already live in the map frame and the solved correction is
  // near identity, matching the world-frame convention used by the existing
  // degeneracy projector. For sensor-frame registration, convert H to the
  // left/world tangent before any condition or eigen-axis analysis:
  // delta_left = Ad_T * delta_right, H_left = Ad_T^-T H Ad_T^-1.
  Eigen::Matrix<double, 6, 6> analysis_hessian = final_hessian;
  if (!this->deskew_ && final_hessian.allFinite() &&
      matrixFinite(optimizer_solution)) {
    const Eigen::Matrix3d R =
        optimizer_solution.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d t =
        optimizer_solution.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix3d t_hat;
    t_hat << 0.0, -t.z(), t.y(),
             t.z(), 0.0, -t.x(),
             -t.y(), t.x(), 0.0;
    Eigen::Matrix<double, 6, 6> Ad_inv =
        Eigen::Matrix<double, 6, 6>::Zero();
    Ad_inv.block<3, 3>(0, 0) = R.transpose();
    Ad_inv.block<3, 3>(3, 0) = -R.transpose() * t_hat;
    Ad_inv.block<3, 3>(3, 3) = R.transpose();
    analysis_hessian = Ad_inv.transpose() * final_hessian * Ad_inv;
  }
  const double hessian_condition = hessianConditionProxy(analysis_hessian);

  const Eigen::Matrix4f candidate_pose = this->deskew_
      ? (optimizer_solution * this->T_prior)
      : (optimizer_solution * T_lidar_base);
  const bool candidate_pose_valid = matrixFinite(candidate_pose);

  // --- P1: per-map fitness baseline (rolling median of accepted-frame fitness).
  // Absolute fitness thresholds calibrated on a same-run map (floor 0.03-0.06)
  // are meaningless on a cross-run map (floor ~0.27); gates below operate on
  // fitness_ratio = fitness / baseline. ratio stays -1 (gates inert) until
  // minSamples accepted frames have been observed.
  double fitness_baseline = -1.0;
  double fitness_ratio = -1.0;
  if (this->fitness_baseline_enable_ &&
      static_cast<int>(this->fitness_history_.size()) >= this->fitness_baseline_min_samples_) {
    std::vector<double> tmp(this->fitness_history_.begin(), this->fitness_history_.end());
    const size_t mid = tmp.size() / 2;
    std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
    fitness_baseline = tmp[mid];
  } else if (this->fitness_baseline_enable_ && this->fitness_baseline_seed_ > 0.0) {
    // Warm-up seed (review fix): without it the ratio gates and yaw veto are
    // inert for the first minSamples accepted frames — a replay that starts
    // mid-turn is unprotected exactly when it is most fragile. Seed with the
    // expected per-map floor (read it off the previous run's scorecard,
    // scripts/analyze_scan_debug_log.py); the rolling median takes over once
    // warmed up.
    fitness_baseline = this->fitness_baseline_seed_;
  }
  if (fitness_baseline > 1e-9 && std::isfinite(fitness_score)) {
    fitness_ratio = fitness_score / fitness_baseline;
  }

  // --- P1: degeneracy-aware partial update + yaw-consistency veto.
  // Eigen-projection engages when the (calibrated) condition proxy crosses
  // hessianCondMax; the yaw veto engages independently on low-confidence
  // matches (fitness_ratio above yawGate/fitnessRatio). Either way the scan is
  // not binary-rejected: the correction is projected and the IMU prior kept
  // along untrusted directions.
  const bool eigen_projection_wanted =
      this->degen_partial_update_enable_ && candidate_pose_valid &&
      this->gicp_hessian_cond_max_ > 0.0 && analysis_hessian.allFinite() &&
      hessian_condition > this->gicp_hessian_cond_max_;
  // NOTE: yawGate is deliberately INDEPENDENT of degeneracy/partialUpdate —
  // disabling partial updates (legacy binary hessian gate) must not silently
  // disable the yaw-consistency veto.
  const bool yaw_veto_wanted =
      candidate_pose_valid &&
      this->yaw_gate_enable_ && fitness_ratio > 0.0 &&
      fitness_ratio > this->yaw_gate_fitness_ratio_;
  DegeneracyProjection degen;
  degen.projected_pose = candidate_pose;
  if (eigen_projection_wanted || yaw_veto_wanted) {
    degen = projectDegenerateDelta(analysis_hessian, this->T_prior, candidate_pose,
                                   eigen_projection_wanted,
                                   this->degen_full6d_, this->degen_coupling_length_m_,
                                   this->degen_rel_floor_6d_,
                                   this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
                                   yaw_veto_wanted ? this->yaw_gate_max_corr_deg_ : 0.0);
  }
  // REVIEW FIX: require the projected pose to be finite before adopting it —
  // final_candidate reaches basePose/current_pose ahead of the downstream
  // gicp_valid finiteness guard, so a (pathological) NaN from the projection
  // would otherwise poison the next scan's prior.
  const Eigen::Matrix4f final_candidate =
      (degen.valid && degen.modified && matrixFinite(degen.projected_pose))
          ? degen.projected_pose : candidate_pose;

  // The projected pose can differ from the optimizer result. Re-score the pose
  // that will actually be applied so a yaw veto or degeneracy projection
  // cannot retain a translation that was only plausible at the discarded
  // optimizer attitude. Failure to evaluate is fail-closed.
  if (candidate_pose_valid && degen.valid && degen.modified &&
      matrixFinite(final_candidate)) {
    const Eigen::Matrix4f T_eval = this->deskew_
        ? Eigen::Matrix4f(final_candidate * this->T_prior.inverse())
        : Eigen::Matrix4f(final_candidate * T_base_lidar);
    double applied_fitness = std::numeric_limits<double>::infinity();
    double applied_optimizer_error =
        std::numeric_limits<double>::infinity();
    int applied_correspondences = 0;
    if (this->gicp.evaluateFitnessAt(
            T_eval, &applied_fitness, &applied_correspondences,
            &applied_optimizer_error)) {
      fitness_score = applied_fitness;
      num_correspondences = applied_correspondences;
      correspondence_ratio = this->current_scan->points.empty()
          ? 0.0
          : static_cast<double>(applied_correspondences) /
                static_cast<double>(this->current_scan->points.size());
      final_error = applied_optimizer_error;
      fitness_ratio = fitness_baseline > 1e-9
          ? fitness_score / fitness_baseline
          : -1.0;
    } else {
      fitness_score = std::numeric_limits<double>::infinity();
      final_error = std::numeric_limits<double>::infinity();
      num_correspondences = 0;
      correspondence_ratio = 0.0;
      fitness_ratio = -1.0;
    }
    this->last_fitness_score_ = fitness_score;
  }

  double guess_to_solution_trans = -1.0;
  double guess_to_solution_rot_deg = -1.0;
  if (candidate_pose_valid) {
    guess_to_solution_trans = deltaTranslationNorm(guess_pose_map, candidate_pose);
    guess_to_solution_rot_deg = rotationDistanceDeg(guess_pose_map, candidate_pose);
  }

  double scan_dt = 0.0;
  if (this->last_gicp_valid_) {
    scan_dt = (this->scan_stamp - this->last_gicp_stamp_).seconds();
  }

  double imu_buffer_span = -1.0;
  double scan_to_latest_imu_lag = -1.0;
  {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      const double latest_imu_stamp = this->imu_buffer.front().stamp;
      const double oldest_imu_stamp = this->imu_buffer.back().stamp;
      imu_buffer_span = latest_imu_stamp - oldest_imu_stamp;
      scan_to_latest_imu_lag = this->scan_stamp.seconds() - latest_imu_stamp;
    }
  }

  // Jump is measured against the IMU-predicted prior, not the last GICP pose.
  // This asks "did GICP disagree with IMU?" (catastrophic failure indicator)
  // instead of "did the vehicle move far?" (expected at highway speed + time gaps).
  double jump_trans = -1.0;
  double jump_rot_deg = -1.0;
  if (candidate_pose_valid) {
    jump_trans = deltaTranslationNorm(this->T_prior, candidate_pose);
    jump_rot_deg = rotationDistanceDeg(this->T_prior, candidate_pose);
  }
  // Speed/scan_dt-aware jump thresholds (P2#2). The jump is GICP-vs-IMU-prior, so
  // the legitimate disagreement scales with how far the prior could have drifted:
  // ~speed*scan_dt for translation and ~scan_dt for rotation (scan_dt here is the
  // time since the last accepted pose, so it grows during a failure streak). This
  // loosens the gate after a gap / at high speed so a valid reacquisition is not
  // rejected, while keeping it tight for slow, short-gap scans. Setting the scales
  // to 0 reproduces the fixed thresholds.
  float speed_est = 0.0f;
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    speed_est = this->state.v.lin.w.norm();
  }
  const double scan_dt_clamped = std::min(std::max(scan_dt, 0.0), 1.0);
  const double eff_jump_trans_m = this->debug_jump_trans_m_ +
      this->jump_trans_speed_scale_ * static_cast<double>(speed_est) * scan_dt_clamped;
  const double eff_jump_rot_deg = this->debug_jump_rot_deg_ +
      this->jump_rot_dt_scale_deg_ * scan_dt_clamped;
  const bool large_jump = candidate_pose_valid &&
                          (jump_trans > eff_jump_trans_m || jump_rot_deg > eff_jump_rot_deg);
  // The jump GATE evaluates the pose that would actually be applied. A partial
  // update can only shrink the GICP-vs-prior delta, so this is never looser
  // than the raw-candidate check; raw jump_trans/jump_rot_deg stay in the
  // debug topics/log as the unprojected disagreement signal.
  double final_jump_trans = jump_trans;
  double final_jump_rot_deg = jump_rot_deg;
  if (candidate_pose_valid && degen.valid && degen.modified) {
    final_jump_trans = deltaTranslationNorm(this->T_prior, final_candidate);
    final_jump_rot_deg = rotationDistanceDeg(this->T_prior, final_candidate);
  }
  const bool large_jump_final = candidate_pose_valid &&
      (final_jump_trans > eff_jump_trans_m || final_jump_rot_deg > eff_jump_rot_deg);

  // RTK-fixed divergence cross-check. The errors remain diagnostics, and a
  // successfully composed, fresh fixed-only sample can additionally reject a
  // candidate outside gt_recovery/sanity_radius. Float/no-fix never enters the
  // buffer and therefore cannot influence registration.
  double gt_pos_err = -1.0;
  double gt_rot_err_deg = -1.0;
  double gt_dt = 0.0;
  bool gt_pose_composed_in_base = false;
  const double registration_stamp =
      this->t_prior_stamp_ > 0.0
          ? this->t_prior_stamp_
          : this->scan_stamp.seconds();
  if (this->gt_odom_enabled_ && this->gt_odom_received_.load() && candidate_pose_valid) {
    GtSample gt;
    // Cross-check is a CM-LEVEL DIAGNOSTIC -- only meaningful against
    // RTK-FIXED-quality Atlas samples. Recovery uses the same fixed-only gate.
    if (this->getGtPoseAt(registration_stamp, gt) &&
        this->gtSampleIsRtkFixed(gt)) {
      // Evaluate the pose that would actually be APPLIED (post degeneracy
      // projection), so run-report gt_err statistics describe the output.
      const Eigen::Vector3f cand_p = final_candidate.block<3, 1>(0, 3);
      const Eigen::Quaternionf cand_q(Eigen::Matrix3f(final_candidate.block<3, 3>(0, 0)));
      // Bring the GT sample from msg.child_frame_id (gt_body) into base_frame
      // using the same TF composition the snap helper uses. On the dome the
      // gt_odom source is the adapter's fixed-only INS odometry, whose body frame
      // differs from base_link by the static imu_link -> base_link TF, so
      // the composition is a real lever-arm correction here — not a no-op.
      // Without this composition, the cross-check carries a constant baseline
      // bias equal to the gt_body -> base_frame lever arm.
      Eigen::Vector3f gt_p_in_base;
      Eigen::Quaternionf gt_q_in_base;
      gt_pose_composed_in_base =
          this->composeGtPoseInBase(gt, gt_p_in_base, gt_q_in_base);
      if (!gt_pose_composed_in_base) {
        // Extrinsic not cached yet -- fall back to gt.p/gt.q directly.
        // This remains diagnostic-only; the RTK sanity gate below requires a
        // successfully composed base-frame pose.
        gt_p_in_base = gt.p;
        gt_q_in_base = gt.q;
      }
      gt_pos_err = (cand_p - gt_p_in_base).norm();
      Eigen::Quaternionf dq = cand_q.normalized() * gt_q_in_base.normalized().conjugate();
      const double w = std::clamp(static_cast<double>(std::abs(dq.w())), 0.0, 1.0);
      gt_rot_err_deg = 2.0 * std::acos(w) * 180.0 / M_PI;
      gt_dt = registration_stamp - gt.stamp;
    }
  }
  const bool rtk_fixed_wrong_lock =
      this->gt_recovery_enabled_ &&
      this->gt_recovery_sanity_radius_ > 0.0 &&
      gt_pose_composed_in_base &&
      gt_pos_err > this->gt_recovery_sanity_radius_;

  if (this->debug_pub_enabled_) {
    auto publish_float = [](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub, double value) {
      std_msgs::msg::Float64 msg;
      msg.data = value;
      pub->publish(msg);
    };

    publish_float(this->dbg_fitness_pub, fitness_score);
    publish_float(this->dbg_gicp_elapsed_ms_pub, elapsed_ms);
    publish_float(this->dbg_corr_norm_pub, guess_to_solution_trans);
    publish_float(this->dbg_scan_dt_pub, scan_dt);
    publish_float(this->dbg_imu_age_pub, imu_buffer_span);
    publish_float(this->dbg_num_correspondences_pub, static_cast<double>(num_correspondences));
    publish_float(this->dbg_correspondence_ratio_pub, correspondence_ratio);
    publish_float(this->dbg_final_error_pub, final_error);
    publish_float(this->dbg_guess_to_solution_trans_pub, guess_to_solution_trans);
    publish_float(this->dbg_guess_to_solution_rot_deg_pub, guess_to_solution_rot_deg);
    publish_float(this->dbg_guess_from_last_trans_pub, guess_from_last_trans);
    publish_float(this->dbg_guess_from_last_rot_deg_pub, guess_from_last_rot_deg);
    publish_float(this->dbg_raw_points_pub, static_cast<double>(this->last_raw_point_count_));
    publish_float(this->dbg_preprocessed_points_pub, static_cast<double>(this->last_preprocessed_point_count_));
    publish_float(this->dbg_imu_buffer_span_pub, imu_buffer_span);
    publish_float(this->dbg_scan_to_latest_imu_lag_pub, scan_to_latest_imu_lag);
    publish_float(this->dbg_hessian_condition_pub, hessian_condition);
    publish_float(this->dbg_jump_trans_pub, jump_trans);
    publish_float(this->dbg_jump_rot_deg_pub, jump_rot_deg);
    publish_float(this->dbg_fitness_ratio_pub, fitness_ratio);
    publish_float(this->dbg_degen_rot_axes_pub, static_cast<double>(degen.degen_rot_axes));
    publish_float(this->dbg_degen_trans_axes_pub, static_cast<double>(degen.degen_trans_axes));
    publish_float(this->dbg_yaw_veto_pub, degen.yaw_vetoed ? 1.0 : 0.0);
    // P4#3: per-frame concat/source-set record (merged_aux_count = -1 when
    // concat disabled; aux dt = NaN when that aux did not merge this frame).
    publish_float(this->dbg_merged_aux_count_pub,
                  static_cast<double>(this->concat_last_merged_aux_));
    publish_float(this->dbg_scan_time_span_pub, this->last_scan_time_span_s_);
    for (size_t i = 0; i < this->dbg_aux_dt_pubs_.size(); ++i) {
      const double dt_i = (i < this->concat_last_aux_dt_.size())
          ? this->concat_last_aux_dt_[i] : std::numeric_limits<double>::quiet_NaN();
      const double pts_i = (i < this->concat_last_aux_points_.size())
          ? static_cast<double>(this->concat_last_aux_points_[i]) : 0.0;
      publish_float(this->dbg_aux_dt_pubs_[i], dt_i);
      publish_float(this->dbg_aux_points_pubs_[i], pts_i);
    }

    std_msgs::msg::Bool converged_msg;
    converged_msg.data =
        (converged ||
         (candidate_pose_valid &&
          fitness_score <= this->gicp_fitness_reject_threshold_)) &&
        candidate_pose_valid &&
        correspondence_ratio >= this->gicp_min_correspondence_ratio_;
    this->dbg_converged_pub->publish(converged_msg);

    if (gt_pos_err >= 0.0) {
      publish_float(this->dbg_gt_pos_err_pub, gt_pos_err);
      publish_float(this->dbg_gt_rot_deg_pub, gt_rot_err_deg);
    }

    this->dbg_initial_guess_pose_pub->publish(
        poseStampedFromMatrix(guess_pose_map, this->scan_stamp, this->map_frame));
    if (candidate_pose_valid) {
      // Post-projection pose (what would be applied); raw disagreement is
      // still visible via jump_trans/jump_rot_deg topics.
      this->dbg_final_pose_pub->publish(
          poseStampedFromMatrix(final_candidate, this->scan_stamp, this->map_frame));
    }

    if (this->dbg_pose_markers_pub->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray markers;

      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear_marker);

      markers.markers.push_back(
          makeArrowMarker(guess_pose_map, this->map_frame, this->scan_stamp, 0, "gicp_debug_guess", 1.0f, 0.55f, 0.0f));

      if (candidate_pose_valid) {
        markers.markers.push_back(
            makeArrowMarker(candidate_pose, this->map_frame, this->scan_stamp, 1, "gicp_debug_solution", 0.0f, 0.9f, 0.2f));

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = this->scan_stamp;
        line_marker.header.frame_id = this->map_frame;
        line_marker.ns = "gicp_debug_delta";
        line_marker.id = 2;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.12;
        line_marker.color.a = 1.0f;
        line_marker.color.r = 1.0f;
        line_marker.color.g = 1.0f;
        line_marker.color.b = 0.0f;

        geometry_msgs::msg::Point guess_point;
        guess_point.x = guess_pose_map(0, 3);
        guess_point.y = guess_pose_map(1, 3);
        guess_point.z = guess_pose_map(2, 3);
        line_marker.points.push_back(guess_point);

        geometry_msgs::msg::Point final_point;
        final_point.x = candidate_pose(0, 3);
        final_point.y = candidate_pose(1, 3);
        final_point.z = candidate_pose(2, 3);
        line_marker.points.push_back(final_point);
        markers.markers.push_back(line_marker);
      }

      this->dbg_pose_markers_pub->publish(markers);
    }
  }

  auto build_scan_debug_log = [&](const char* status) {
    std::ostringstream oss;
    oss << std::fixed
        << "SCAN DEBUG | status=" << status
        << " stamp=" << std::setprecision(3) << this->scan_stamp.seconds()
        << " input_frame=" << this->last_scan_input_frame_
        << " raw=" << this->last_raw_point_count_
        << " pre=" << this->last_preprocessed_point_count_
        << " guess={" << poseSummary(guess_pose_map) << "}"
        << " guess_from_last=[" << scalarSummary(guess_from_last_trans) << "m,"
        << scalarSummary(guess_from_last_rot_deg) << "deg]"
        << " gicp_ms=" << scalarSummary(elapsed_ms, 2)
        << " converged=" << (converged ? "true" : "false")
        << " fitness=" << scalarSummary(fitness_score, 6)
        << " fit_ratio=" << scalarSummary(fitness_ratio, 3)
        << " degen=[r" << degen.degen_rot_axes << ",t" << degen.degen_trans_axes
        << ",yaw_veto=" << (degen.yaw_vetoed ? 1 : 0)
        << ",partial=" << ((degen.valid && degen.modified) ? 1 : 0) << "]"
        << " final_error=" << scalarSummary(final_error, 6)
        << " correspondences=" << num_correspondences << "/" << this->current_scan->points.size()
        << " ratio=" << scalarSummary(correspondence_ratio, 3)
        << " guess_to_solution=[" << scalarSummary(guess_to_solution_trans) << "m,"
        << scalarSummary(guess_to_solution_rot_deg) << "deg]"
        << " jump=[" << scalarSummary(jump_trans) << "m," << scalarSummary(jump_rot_deg) << "deg]"
        << " imu_buffer_span=" << scalarSummary(imu_buffer_span) << "s"
        << " scan_to_latest_imu_lag=" << scalarSummary(scan_to_latest_imu_lag) << "s"
        << " concat=[" << this->concat_last_merged_aux_ << "/" << this->aux_lidars_.size();
    for (size_t i = 0; i < this->concat_last_aux_dt_.size(); ++i) {
      oss << ",dt" << i << "=" << scalarSummary(this->concat_last_aux_dt_[i], 3)
          << "s,pts" << i << "=" << this->concat_last_aux_points_[i];
    }
    oss << ",span=" << scalarSummary(this->last_scan_time_span_s_, 3) << "s]"
        << " hessian_cond=" << scalarSummary(hessian_condition, 3)
        << " candidate={" << poseSummary(candidate_pose) << "}";

    if (this->last_gicp_valid_) {
      oss << " last_good={" << poseSummary(this->last_gicp_pose_) << "}";
    }
    if (this->gt_odom_enabled_) {
      if (gt_pos_err >= 0.0) {
        oss << " gt_err=[" << scalarSummary(gt_pos_err, 3) << "m,"
            << scalarSummary(gt_rot_err_deg, 2) << "deg,dt="
            << scalarSummary(gt_dt, 3) << "s]";
      } else {
        oss << " gt_err=unavailable";
      }
    }
    return oss.str();
  };

  // Accept results that have good fitness even when the optimizer didn't formally
  // converge (hit maxIterations before epsilon was met). At highway speed the
  // initial guess can be 1-3 m away, so the solver may need more steps than
  // maxIterations to satisfy the tight epsilon — but the result is still accurate.
  const bool effectively_converged = converged ||
      (candidate_pose_valid && fitness_score <= this->gicp_fitness_reject_threshold_);

  bool gicp_rejected_fitness = false;
  bool gicp_rejected_correspondence_ratio = false;
  bool gicp_rejected_fitness_ratio = false;
  bool gicp_rejected_rtk_sanity = false;
  bool gicp_rejected_jump = false;
  bool gicp_rejected_hessian = false;
  if (effectively_converged && candidate_pose_valid) {
    if (rtk_fixed_wrong_lock) {
      gicp_rejected_rtk_sanity = true;
    } else if (!analysis_hessian.allFinite()) {
      // Geometry diagnostics are part of the acceptance contract. A
      // non-finite small_gicp Hessian cannot be projected or conditioned
      // reliably even if a separate fitness calculation happened to be
      // finite.
      gicp_rejected_hessian = true;
    } else if (fitness_score > this->gicp_fitness_reject_threshold_) {
      gicp_rejected_fitness = true;
    } else if (correspondence_ratio <
               this->gicp_min_correspondence_ratio_) {
      gicp_rejected_correspondence_ratio = true;
    } else if (this->fitness_ratio_reject_ > 0.0 && fitness_ratio > 0.0 &&
               fitness_ratio > this->fitness_ratio_reject_) {
      // Wrong-basin gate (P1): fitness far above the map's own rolling-median
      // baseline is the bad-accept signature (run-12: bad accepts at gt_err>20m
      // had ratio median 1.34 / p90 2.55 while good accepts sat at 1.00). A
      // wrong-basin solution must not be partially applied either — fall back
      // to the IMU prior entirely.
      gicp_rejected_fitness_ratio = true;
    } else if (this->degen_partial_update_enable_) {
      // P1 partial-update path: degenerate geometry no longer rejects the scan
      // (the old binary gate produced 253-frame dead-reckoning streaks when the
      // absolute fitness warn threshold went stale on a cross-run map). The
      // correction has already been projected onto the well-constrained
      // eigen-subspace above; reject only when the analysis says NOTHING is
      // trustworthy (all axes degenerate / hessian not analyzable).
      if (eigen_projection_wanted && (!degen.valid || degen.fully_degenerate)) {
        gicp_rejected_hessian = true;
      } else if (this->gicp_reject_large_jumps_ && large_jump_final) {
        gicp_rejected_jump = true;
      }
    } else if (this->gicp_hessian_cond_max_ > 0.0 &&
               hessian_condition > this->gicp_hessian_cond_max_ &&
               ((this->gicp_hessian_fitness_warn_ > 0.0 &&
                 fitness_score > this->gicp_hessian_fitness_warn_) ||
                (this->gicp_hessian_trans_warn_m_ > 0.0 &&
                 guess_to_solution_trans > this->gicp_hessian_trans_warn_m_) ||
                (this->gicp_hessian_rot_warn_deg_ > 0.0 &&
                 guess_to_solution_rot_deg > this->gicp_hessian_rot_warn_deg_) ||
                (this->gicp_hessian_fitness_warn_ <= 0.0 &&
                 this->gicp_hessian_trans_warn_m_ <= 0.0 &&
                 this->gicp_hessian_rot_warn_deg_ <= 0.0))) {
      // LEGACY combined geometric degeneracy gate (degeneracy/partialUpdate:
      // false). High hessian condition alone is harmless when the IMU prior
      // was already good and GICP barely moved; reject only when degenerate
      // AND a slide signal fires. NOTE: the fitness/trans/rot warn thresholds
      // here are ABSOLUTE and calibrated per-map — on cross-run maps they go
      // stale (see docs/action_plan_turn_error_20260704.md), which is why the
      // partial-update path above is the default.
      gicp_rejected_hessian = true;
    } else if (this->gicp_reject_large_jumps_ && large_jump_final) {
      // large_jump_final == large_jump unless the (independent) yaw veto
      // modified the candidate; gate the pose that would actually be applied.
      gicp_rejected_jump = true;
    }
  }
  const bool gicp_accepted = effectively_converged && candidate_pose_valid &&
                             !gicp_rejected_fitness &&
                             !gicp_rejected_correspondence_ratio &&
                             !gicp_rejected_fitness_ratio &&
                             !gicp_rejected_rtk_sanity &&
                             !gicp_rejected_hessian && !gicp_rejected_jump;
  const bool gicp_partial = gicp_accepted && degen.valid && degen.modified;

  if (!candidate_pose_valid) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("invalid_solution").c_str());
  } else if (!effectively_converged) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("failed_to_converge").c_str());
  } else if (gicp_rejected_fitness) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness=%.4f > threshold=%.4f): %s",
                fitness_score, this->gicp_fitness_reject_threshold_,
                build_scan_debug_log("rejected_fitness").c_str());
  } else if (gicp_rejected_correspondence_ratio) {
    RCLCPP_WARN(
        this->get_logger(),
        "GICP REJECTED (correspondence_ratio=%.3f < %.3f): %s",
        correspondence_ratio, this->gicp_min_correspondence_ratio_,
        build_scan_debug_log("rejected_correspondence_ratio").c_str());
  } else if (gicp_rejected_fitness_ratio) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness_ratio=%.3f > %.3f, baseline=%.4f — wrong-basin signature): %s",
                fitness_ratio, this->fitness_ratio_reject_, fitness_baseline,
                build_scan_debug_log("rejected_fitness_ratio").c_str());
  } else if (gicp_rejected_rtk_sanity) {
    RCLCPP_WARN(
        this->get_logger(),
        "GICP REJECTED: candidate is %.2fm from the fresh RTK-fixed pose "
        "(limit %.2fm): %s",
        gt_pos_err, this->gt_recovery_sanity_radius_,
        build_scan_debug_log("rejected_rtk_fixed_sanity").c_str());
  } else if (gicp_rejected_hessian) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (hessian_cond=%.3e > %.3e AND [fitness=%.4f|trans=%.3fm|rot=%.3fdeg] crossed [%.4f|%.3fm|%.3fdeg] — degenerate slide): %s",
                hessian_condition, this->gicp_hessian_cond_max_,
                fitness_score, guess_to_solution_trans, guess_to_solution_rot_deg,
                this->gicp_hessian_fitness_warn_,
                this->gicp_hessian_trans_warn_m_,
                this->gicp_hessian_rot_warn_deg_,
                build_scan_debug_log("rejected_hessian").c_str());
  } else if (gicp_rejected_jump) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (jump dT=%.3fm dR=%.2fdeg > eff thresholds [%.2fm, %.2fdeg] @ speed=%.1fm/s scan_dt=%.3fs): %s",
                final_jump_trans, final_jump_rot_deg, eff_jump_trans_m, eff_jump_rot_deg,
                static_cast<double>(speed_est), scan_dt_clamped,
                build_scan_debug_log("rejected_jump").c_str());
  } else if (large_jump && !gicp_partial) {
    RCLCPP_WARN(this->get_logger(), "%s", build_scan_debug_log("large_jump").c_str());
  } else if (this->debug_verbose_scan_log_) {
    // "ok_partial" = accepted after degeneracy projection / yaw veto shrank
    // the correction; grep-compatible with "status=ok" prefix matching is NOT
    // preserved on purpose so audits can split the two populations.
    RCLCPP_INFO(this->get_logger(), "%s",
                build_scan_debug_log(gicp_partial ? "ok_partial" : "ok").c_str());
  }

  if (gicp_accepted) {
    // P1: apply the (possibly degeneracy-projected) candidate, and feed the
    // per-map fitness baseline from accepted frames only, so wrong-basin /
    // rejected fitness never inflates the baseline the gates divide by.
    this->current_pose = final_candidate;
    if (this->fitness_baseline_enable_ && std::isfinite(fitness_score) && fitness_score >= 0.0) {
      this->fitness_history_.push_back(fitness_score);
      while (static_cast<int>(this->fitness_history_.size()) > this->fitness_baseline_window_) {
        this->fitness_history_.pop_front();
      }
    }

    // Update lidar pose for next iteration
    Eigen::Vector3f new_p = this->current_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotSO3 = this->current_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotSO3);
    q.normalize();

    this->basePose.p = new_p;
    this->basePose.q = q;
    // P3: pair the measurement with the IMU prior it was registered against
    // (both at median scan time) so updateState can form the time-free delta.
    this->observer_prior_pose_ = this->T_prior;

    // Validate GICP result before using it
    bool gicp_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) && std::isfinite(new_p.z()) &&
                      std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());

    if (!gicp_valid) {
      RCLCPP_WARN(this->get_logger(), "GICP result contains invalid values, skipping geometric observer update");
    } else {
      // Initialize or update geometric observer
      if (!this->geo.first_opt_done) {
        // First time: initialize state to GICP result
        this->state.p = new_p;
        this->state.q = q;
        this->state.v.lin.w = Eigen::Vector3f::Zero();
        this->state.v.lin.b = Eigen::Vector3f::Zero();
        this->state.v.ang.w = Eigen::Vector3f::Zero();
        this->state.v.ang.b = Eigen::Vector3f::Zero();
        this->state.b.accel = Eigen::Vector3f::Zero();
        this->state.b.gyro = Eigen::Vector3f::Zero();

        // Initialize geo tracking
        this->geo.prev_p = this->state.p;
        this->geo.prev_q = this->state.q;
        this->geo.prev_vel = Eigen::Vector3f::Zero();

        // Mark as initialized
        this->geo.first_opt_done = true;

        RCLCPP_INFO(this->get_logger(), "Geometric observer initialized to pos=[%.2f,%.2f,%.2f]",
                    new_p.x(), new_p.y(), new_p.z());
      } else {
        // Update geometric observer with GICP measurement (skip on first scan)
        this->updateState();
      }
    }

    // Use geometric observer velocity for next IMU integration
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->prev_vel = this->geo.prev_vel;
    }

    if (this->debug_jump_log_enabled_ && gicp_valid && this->last_gicp_valid_) {
      if (large_jump) {
        const Eigen::Vector3f t_prior = this->T_prior.block<3, 1>(0, 3);
        const Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
        RCLCPP_WARN(this->get_logger(),
                    "JUMP DETECTED: dT=%.3fm dR=%.2fdeg | dt=%.3fs fitness=%.6f | prior=[%.2f,%.2f,%.2f] corr=[%.2f,%.2f,%.2f]",
                    jump_trans, jump_rot_deg, scan_dt, fitness_score,
                    t_prior.x(), t_prior.y(), t_prior.z(),
                    t_corr.x(), t_corr.y(), t_corr.z());
      }
    }

    // Update last GICP pose after computing jump metrics
    if (gicp_valid) {
      this->last_gicp_pose_ = this->current_pose;
      this->last_gicp_stamp_ = this->scan_stamp;
      this->last_gicp_valid_ = true;
    }

    // Reset consecutive-failure counter on any accepted scan so the GT-recovery
    // trigger only fires on sustained losing streaks.
    this->consecutive_failures_ = 0;
    // Mark this as the last known-good fix; the dead-reckoning covariance growth
    // (P3) measures elapsed time from here.
    this->last_accepted_scan_stamp_ = registration_stamp;

    // Log pose and correction
    Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(),
                "Localization: ✓ %s%s | fitness=%.6f | time=%.2fms | "
                "correction=[%.3f, %.3f, %.3f] | pose=[%.2f, %.2f, %.2f]",
                converged ? "CONVERGED" : "ACCEPTED(fitness-ok)",
                gicp_partial ? " [PARTIAL: degenerate axes kept on IMU prior]" : "",
                fitness_score, elapsed_ms,
                t_corr.x(), t_corr.y(), t_corr.z(),
                this->basePose.p.x(), this->basePose.p.y(), this->basePose.p.z());
  } else {
    // Any non-accepted scan (failed_to_converge, rejected_fitness, rejected_jump, invalid_solution)
    // falls back to the IMU-integrated prior. Freezing at last_gicp_pose_ causes cascade
    // divergence at feature-poor corners: each subsequent scan's guess drifts further from
    // reality, fitness gets worse, and the optimizer never recovers.
    if (gicp_rejected_rtk_sanity) {
      // A fresh, composed RTK-fixed pose is definitive evidence of a wrong
      // basin, so authorize the existing fixed-only recovery immediately.
      this->consecutive_failures_ = std::max(
          this->consecutive_failures_,
          this->gt_recovery_min_consecutive_failures_ - 1);
    }
    ++this->consecutive_failures_;
    const char* reason = !candidate_pose_valid ? "invalid solution"
                       : !effectively_converged ? "failed to converge"
                       : gicp_rejected_fitness ? "fitness rejected"
                       : gicp_rejected_fitness_ratio ? "fitness-ratio rejected (wrong basin)"
                       : gicp_rejected_rtk_sanity ? "RTK-fixed sanity rejected wrong lock"
                       : gicp_rejected_hessian ? "degenerate geometry"
                       : "jump rejected";
    if (matrixFinite(this->T_prior)) {
      this->current_pose = this->T_prior;
      const Eigen::Vector3f new_p = this->T_prior.block<3, 1>(0, 3);
      Eigen::Quaternionf q(this->T_prior.block<3, 3>(0, 0));
      q.normalize();
      this->basePose.p = new_p;
      this->basePose.q = q;
      {
        // P2#1 (stale-velocity bug): seed the next scan's IMU integration from
        // the CURRENT IMU-propagated velocity, not geo.prev_vel. geo.prev_vel
        // is only refreshed by updateState() (accepted scans) or a GT snap, so
        // during an N-frame rejection streak it stayed frozen at the last
        // accepted scan's velocity while the vehicle's velocity vector rotated
        // through the turn — every per-scan prior then extrapolated straight
        // ("corner cutting", run-12 webm). state.v.lin.w is maintained at IMU
        // rate by propagateState() and is the correct dead-reckoning velocity.
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->prev_vel = this->state.v.lin.w;
      }
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding IMU dead-reckoning pose [%.2f, %.2f, %.2f] | fitness=%.4f time=%.2fms",
                  reason, new_p.x(), new_p.y(), new_p.z(), fitness_score, elapsed_ms);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding last accepted pose (no valid T_prior) | fitness=%.4f time=%.2fms",
                  reason, fitness_score, elapsed_ms);
    }

    // GT-driven pose recovery: if enabled and the failure streak has hit the
    // configured threshold, snap state.{pose,velocity} to the time-matched GT
    // sample (transformed into base_frame). The snap overrides the dead-reckoned
    // pose and resets the counter; logs its own warn line.
    this->maybeSnapPoseToGT(reason);
  }
}

void gicp_localization::LocalizationNode::publishPose() {

  std::lock_guard<std::mutex> lock(this->pose_mutex);

  // Extract position and orientation from localized pose
  Eigen::Vector3f position = this->current_pose.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = this->current_pose.block<3, 3>(0, 0);
  Eigen::Quaternionf orientation(rotation);
  orientation.normalize();

  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->scan_stamp;
  pose_msg.header.frame_id = this->map_frame;
  pose_msg.pose.position.x = position.x();
  pose_msg.pose.position.y = position.y();
  pose_msg.pose.position.z = position.z();
  pose_msg.pose.orientation.w = orientation.w();
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();

  // Publish GICP-corrected pose
  // With unreliable IMU, we publish only GICP results instead of propagated poses
  this->pose_pub->publish(pose_msg);

  // Add to trajectory deque (O(1) pop_front when capping). Only build the Path
  // message + DDS-publish when a subscriber actually exists.
  if (this->path_buffer_.size() >= 10000) this->path_buffer_.pop_front();
  this->path_buffer_.push_back(pose_msg);
  if (this->path_pub && this->path_pub->get_subscription_count() > 0) {
    this->path_msg.header.stamp = this->scan_stamp;
    this->path_msg.header.frame_id = this->map_frame;
    this->path_msg.poses.assign(this->path_buffer_.begin(), this->path_buffer_.end());
    this->path_pub->publish(this->path_msg);
  }

  // Publish UTM-frame pose/path
  if (this->utm_enabled_) {
    Eigen::Matrix4f T_utm_base = this->T_utm_map_ * this->current_pose;
    Eigen::Vector3f utm_pos = T_utm_base.block<3, 1>(0, 3);
    Eigen::Quaternionf utm_q(T_utm_base.block<3, 3>(0, 0));
    utm_q.normalize();

    geometry_msgs::msg::PoseStamped utm_pose_msg;
    utm_pose_msg.header.stamp = this->scan_stamp;
    utm_pose_msg.header.frame_id = this->utm_frame;
    utm_pose_msg.pose.position.x = utm_pos.x();
    utm_pose_msg.pose.position.y = utm_pos.y();
    utm_pose_msg.pose.position.z = utm_pos.z();
    utm_pose_msg.pose.orientation.w = utm_q.w();
    utm_pose_msg.pose.orientation.x = utm_q.x();
    utm_pose_msg.pose.orientation.y = utm_q.y();
    utm_pose_msg.pose.orientation.z = utm_q.z();
    this->utm_pose_pub->publish(utm_pose_msg);

    if (this->utm_path_buffer_.size() >= 10000) this->utm_path_buffer_.pop_front();
    this->utm_path_buffer_.push_back(utm_pose_msg);
    if (this->utm_path_pub && this->utm_path_pub->get_subscription_count() > 0) {
      this->utm_path_msg_.header.stamp = this->scan_stamp;
      this->utm_path_msg_.header.frame_id = this->utm_frame;
      this->utm_path_msg_.poses.assign(this->utm_path_buffer_.begin(), this->utm_path_buffer_.end());
      this->utm_path_pub->publish(this->utm_path_msg_);
    }
  }

  // Publish TF
  if (this->publish_tf_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = pose_msg.header;
    transform_stamped.child_frame_id = this->base_frame;
    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();
    transform_stamped.transform.rotation = pose_msg.pose.orientation;
    this->tf_broadcaster->sendTransform(transform_stamped);
  }
}

void gicp_localization::LocalizationNode::callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!this->enu_origin_validated_.load()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring gt_odom until the live adapter ENU origin matches the map");
    return;
  }
  if (msg->header.frame_id != this->map_frame) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting gt_odom in frame '%s'; expected map frame '%s'. Check the "
        "adapter odom_frame_id and ENU datum.",
        msg->header.frame_id.c_str(), this->map_frame.c_str());
    return;
  }

  GtSample s;
  s.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  s.p = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  s.q = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  s.v_lin_body = Eigen::Vector3f(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
  s.v_ang_body = Eigen::Vector3f(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);
  if (!std::isfinite(s.stamp) || !s.p.allFinite() ||
      !s.q.coeffs().allFinite() || s.q.norm() < 1.0e-6f ||
      !s.v_lin_body.allFinite() || !s.v_ang_body.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Rejecting gt_odom with non-finite timestamp, pose, or twist");
    return;
  }
  s.q.normalize();
  // Defense in depth: the adapter topic is already solution-type-gated, but a
  // malformed or accidentally remapped stream must not enter the GT buffer.
  s.cov_pos_xx = msg->pose.covariance[0];
  s.cov_pos_yy = msg->pose.covariance[7];
  s.cov_pos_zz = msg->pose.covariance[14];
  if (!this->gtSampleIsRtkFixed(s)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting gt_odom that is not RTK-fixed quality "
        "(covariance=[%.6g, %.6g, %.6g]); continuing with LiDAR+IMU",
        s.cov_pos_xx, s.cov_pos_yy, s.cov_pos_zz);
    return;
  }

  // Cache base_frame ← gt_body_frame TF on the first message (mirrors the IMU
  // extrinsic caching pattern in callbackImu). Required before the snap helper
  // and the diagnostic cross-check can compose poses; callback keeps appending
  // samples even while TF is missing.  Runs BEFORE the odom-init block below
  // so composeGtPoseInBase has the extrinsic ready to bring the first GT
  // sample into base_frame coordinates before applyInitialPose seeds the state.
  if (!this->gt_extrinsics_cached_) {
    if (this->gt_body_frame_.empty()) {
      this->gt_body_frame_ = msg->child_frame_id;
      if (this->gt_body_frame_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT odom message has empty child_frame_id; assuming GT body == base_frame ('%s')",
                             this->base_frame.c_str());
        this->gt_body_frame_ = this->base_frame;
      }
    }
    if (this->gt_body_frame_ == this->base_frame) {
      this->T_base_gtbody_.setIdentity();
      this->gt_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "GT recovery: gt_body == base_frame ('%s'), using identity extrinsic",
                  this->base_frame.c_str());
    } else {
      try {
        auto tf_bg = this->tf_buffer->lookupTransform(
            this->base_frame, this->gt_body_frame_, tf2::TimePointZero);
        Eigen::Quaternionf q_bg(
            tf_bg.transform.rotation.w, tf_bg.transform.rotation.x,
            tf_bg.transform.rotation.y, tf_bg.transform.rotation.z);
        Eigen::Vector3f t_bg(
            tf_bg.transform.translation.x, tf_bg.transform.translation.y,
            tf_bg.transform.translation.z);
        this->T_base_gtbody_.setIdentity();
        this->T_base_gtbody_.block<3, 3>(0, 0) = q_bg.toRotationMatrix();
        this->T_base_gtbody_.block<3, 1>(0, 3) = t_bg;
        this->gt_extrinsics_cached_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "GT recovery: cached %s ← %s extrinsic: t=[%.3f,%.3f,%.3f]",
                    this->base_frame.c_str(), this->gt_body_frame_.c_str(),
                    t_bg.x(), t_bg.y(), t_bg.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT recovery: cannot cache %s ← %s TF: %s — deferring snap",
                             this->base_frame.c_str(), this->gt_body_frame_.c_str(), ex.what());
      }
    }
  }

  // Odom init: on the first GT odom message (after the TF cache above is
  // populated), seed state from GT so the node starts at the correct location
  // even when the bag begins mid-run.  Composes through T_base_gtbody_ so the
  // seeded state.p lands at base_frame, not at gt_body_frame -- otherwise an
  // off-base gt_odom source would seed the state with a constant lever-arm
  // offset (the same defect that previously biased the cross-check). On the
  // dome the gt_odom body frame (Atlas Duo INS) differs from base_link by the
  // static imu_link -> base_link TF, so this composition is a real
  // correction. If the extrinsic hasn't cached yet (TF lookup deferred), skip
  // this message and try again on the next one rather than seeding from a
  // frame we can't compose.
  // Overrides any param-based initial pose. Sets first_opt_done so odom starts
  // publishing immediately without waiting for the first accepted GICP scan.
  if (this->use_odom_init_ && !this->use_odom_init_applied_ &&
      this->gtSampleIsRtkFixed(s)) {
    Eigen::Vector3f init_p;
    Eigen::Quaternionf init_q;
    if (this->composeGtPoseInBase(s, init_p, init_q)) {
      this->use_odom_init_applied_ = true;
      const rclcpp::Time stamp_ros(msg->header.stamp.sec, msg->header.stamp.nanosec);
      this->applyInitialPose(init_p, init_q, stamp_ros, "gt_odom");
      {
        std::lock_guard<std::mutex> lock(this->geo.mtx);
        this->geo.first_opt_done = true;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Odom init: pose set from GT odom at t=%.3f gt_pos=[%.2f,%.2f,%.2f] "
                  "-> base_pos=[%.2f,%.2f,%.2f] (gt_body='%s')",
                  s.stamp, s.p.x(), s.p.y(), s.p.z(),
                  init_p.x(), init_p.y(), init_p.z(),
                  this->gt_body_frame_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Odom init: deferring -- gt_body -> base extrinsic not cached yet");
    }
  }

  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (!this->gt_odom_buffer_.empty() && s.stamp <= this->gt_odom_buffer_.back().stamp) {
    // Out-of-order or duplicate timestamp; drop to keep buffer monotone.
    return;
  }
  this->gt_odom_buffer_.push_back(s);
  while (this->gt_odom_buffer_.size() > this->gt_odom_buffer_size_) {
    this->gt_odom_buffer_.pop_front();
  }
  if (!this->gt_odom_received_.exchange(true)) {
    RCLCPP_INFO(this->get_logger(),
                "First ground-truth odom received at stamp=%.3f frame=%s child_frame=%s",
                s.stamp, msg->header.frame_id.c_str(),
                msg->child_frame_id.empty() ? "(empty)" : msg->child_frame_id.c_str());
  }
}

void gicp_localization::LocalizationNode::callbackEnuOrigin(
    const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!this->require_live_enu_origin_) {
    return;
  }
  if (!this->map_enu_origin_loaded_) {
    this->enu_origin_validated_.store(false);
    RCLCPP_FATAL(
        this->get_logger(),
        "Received live ENU origin but the map has no validated ENU manifest");
    rclcpp::shutdown();
    return;
  }

  try {
    const EnuOrigin live = parseEnuOrigin(msg->data);
    const EnuOrigin map{
        this->map_enu_origin_lat_,
        this->map_enu_origin_lon_,
        this->map_enu_origin_alt_};
    const double error_m = enuOriginDistanceMeters(map, live);
    if (error_m > this->enu_origin_tolerance_m_) {
      this->enu_origin_validated_.store(false);
      RCLCPP_FATAL(
          this->get_logger(),
          "Live adapter ENU origin differs from the map manifest by %.3f m "
          "(limit %.3f m); refusing localization",
          error_m, this->enu_origin_tolerance_m_);
      rclcpp::shutdown();
      return;
    }
    if (!this->enu_origin_validated_.exchange(true)) {
      RCLCPP_INFO(
          this->get_logger(),
          "Live adapter ENU origin validated against map manifest "
          "(difference %.3f m)",
          error_m);
    }
  } catch (const std::exception& e) {
    this->enu_origin_validated_.store(false);
    RCLCPP_FATAL(this->get_logger(), "Invalid live adapter ENU origin '%s': %s",
                 msg->data.c_str(), e.what());
    rclcpp::shutdown();
  }
}

bool gicp_localization::LocalizationNode::gtSampleIsRtkFixed(const GtSample& s) const {
  // When the gate is disabled, treat every sample as FIXED -- the operator
  // has explicitly opted into "trust whatever the upstream publishes".
  if (!this->rtk_gate_enabled_) return true;
  const auto valid_variance = [](double value, double limit) {
    return std::isfinite(value) && value > 0.0 && value <= limit;
  };
  return valid_variance(s.cov_pos_xx, this->rtk_gate_max_pose_var_xy_) &&
         valid_variance(s.cov_pos_yy, this->rtk_gate_max_pose_var_xy_) &&
         valid_variance(s.cov_pos_zz, this->rtk_gate_max_pose_var_z_);
}

bool gicp_localization::LocalizationNode::getGtPoseAt(double stamp, GtSample& out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) {
    if (this->gt_odom_buffer_.size() == 1 &&
        std::abs(this->gt_odom_buffer_.front().stamp - stamp) <= this->gt_odom_max_dt_) {
      out = this->gt_odom_buffer_.front();
      return true;
    }
    return false;
  }
  // Buffer is monotone non-decreasing. Find the first sample with stamp >= query.
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });

  if (it == this->gt_odom_buffer_.begin()) {
    if (std::abs(it->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *it; return true;
  }
  if (it == this->gt_odom_buffer_.end()) {
    auto last = std::prev(it);
    if (std::abs(last->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *last; return true;
  }
  auto a = std::prev(it);
  auto b = it;
  const double dt_total = b->stamp - a->stamp;
  if (dt_total <= 0.0 || std::min(stamp - a->stamp, b->stamp - stamp) > this->gt_odom_max_dt_) {
    return false;
  }
  const float u = static_cast<float>((stamp - a->stamp) / dt_total);
  out.stamp = stamp;
  out.p = (1.0f - u) * a->p + u * b->p;
  out.q = a->q.slerp(u, b->q).normalized();
  out.v_lin_body = (1.0f - u) * a->v_lin_body + u * b->v_lin_body;
  out.v_ang_body = (1.0f - u) * a->v_ang_body + u * b->v_ang_body;
  // Covariance is not linearly interpolated: carry the conservative (larger)
  // variance of the two bracketing samples so an interpolated pose can only
  // pass the RTK gate when BOTH neighbours were RTK-FIXED. Without this the
  // cov fields would keep their defaults and the gate could mis-classify the
  // sample. (Endpoint/single-sample branches above copy a real sample whole,
  // so their covariance is already valid.)
  out.cov_pos_xx = std::max(a->cov_pos_xx, b->cov_pos_xx);
  out.cov_pos_yy = std::max(a->cov_pos_yy, b->cov_pos_yy);
  out.cov_pos_zz = std::max(a->cov_pos_zz, b->cov_pos_zz);
  return true;
}

bool gicp_localization::LocalizationNode::getGtFiniteDiffVelWorld(
    double stamp, Eigen::Vector3f& v_world_out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) return false;
  // Buffer is monotone non-decreasing (out-of-order samples dropped at insert).
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });
  // Choose a bracketing (or nearest adjacent) pair around the query stamp.
  auto b = (it == this->gt_odom_buffer_.end()) ? std::prev(it) : it;
  auto a = (b == this->gt_odom_buffer_.begin()) ? b : std::prev(b);
  if (a == b) b = std::next(b);  // query before first sample: use first pair
  const double dt = b->stamp - a->stamp;
  if (dt <= 1e-6) return false;
  // Both endpoints must be reasonably close to the query, mirroring
  // getGtPoseAt's staleness contract.
  if (std::min(std::abs(stamp - a->stamp), std::abs(b->stamp - stamp)) >
      this->gt_odom_max_dt_) {
    return false;
  }
  v_world_out = (b->p - a->p) / static_cast<float>(dt);
  return v_world_out.allFinite();
}

bool gicp_localization::LocalizationNode::composeGtPoseInBase(
    const GtSample& gt, Eigen::Vector3f& p_out,
    Eigen::Quaternionf& q_out) const {
  if (!this->gt_extrinsics_cached_) {
    // Extrinsic not cached yet (first message hasn't fully run the cache
    // block, or TF lookup deferred).  Caller decides whether to fall back
    // to gt.p/gt.q directly or skip this cycle.
    return false;
  }
  // T_map_base = T_map_gtbody * inv(T_base_gtbody).  Decomposed:
  //   q_out = gt.q * inv(R_base_gtbody)
  //   p_out = gt.p - q_out * t_base_gtbody
  // Same math as the snap helper -- factored out so the cross-check and the
  // first-message applyInitialPose path use identical composition rather than
  // re-deriving (or skipping) the lever-arm.
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Quaternionf q_gtbody_in_base(R_base_gtbody);
  q_out = (gt.q * q_gtbody_in_base.conjugate()).normalized();
  p_out = gt.p - q_out * t_base_gtbody;
  return true;
}

bool gicp_localization::LocalizationNode::composeGtTwistInBase(
    const GtSample& gt, Eigen::Vector3f& v_lin_body_out,
    Eigen::Vector3f& v_ang_body_out) const {
  if (!this->gt_extrinsics_cached_) {
    return false;
  }
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  v_ang_body_out = R_gtbody_base * gt.v_ang_body;
  v_lin_body_out = R_gtbody_base * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
  return true;
}

// RTK-driven IMU bias calibration. Pairs each IMU sample with a time-matched GT
// pose/twist and accumulates the bias residual. Linear acceleration in world
// frame is estimated by finite-differencing v_world between successive paired
// samples. On window fill, biases are averaged, the state is seeded from the
// latest GT, imu_calibrated_ is flipped, and the function returns true.
bool gicp_localization::LocalizationNode::tryRtkCalibrationStep(
    double stamp, const Eigen::Vector3f& measured_gyro,
    const Eigen::Vector3f& measured_accel) {
  GtSample gt;
  // RTK-driven IMU bias calibration needs CM-LEVEL truth -- only consume
  // RTK-FIXED samples. If only dead-reckoned Atlas poses are available the
  // init machine will time out (rtk_init/fallback_timeout) and drop into
  // stationary calibration.
  if (!this->getGtPoseAt(stamp, gt) || !this->gtSampleIsRtkFixed(gt)) {
    // GT not yet available at this IMU stamp (e.g., IMU briefly ahead of buffer).
    // Don't error — just skip this sample.
    return false;
  }
  Eigen::Vector3f gt_p_in_base;
  Eigen::Quaternionf gt_q_in_base;
  Eigen::Vector3f gt_v_lin_base_body;
  Eigen::Vector3f gt_v_ang_base_body;
  if (!this->composeGtPoseInBase(gt, gt_p_in_base, gt_q_in_base) ||
      !this->composeGtTwistInBase(gt, gt_v_lin_base_body, gt_v_ang_base_body)) {
    // GT sample exists, but base<-gt_body TF is not cached yet.
    return false;
  }
  GtSample seed = gt;
  seed.p = gt_p_in_base;
  seed.q = gt_q_in_base;
  seed.v_lin_body = gt_v_lin_base_body;
  seed.v_ang_body = gt_v_ang_base_body;
  this->latest_rtk_seed_ = seed;
  this->has_latest_rtk_seed_ = true;

  // Body acceleration in world frame, from finite-differencing v_world across
  // consecutive paired samples. Skip the first sample (no derivative possible).
  const Eigen::Matrix3f R = seed.q.toRotationMatrix();
  const Eigen::Vector3f v_world = R * seed.v_lin_body;

  if (!this->has_prev_gt_for_accel_) {
    this->has_prev_gt_for_accel_ = true;
    this->prev_gt_stamp_ = stamp;
    this->prev_v_world_ = v_world;
    return false;
  }

  const double dt = stamp - this->prev_gt_stamp_;
  if (dt <= 1e-4) {
    // Sample too close in time — derivative would explode. Skip.
    return false;
  }
  const Eigen::Vector3f a_world = (v_world - this->prev_v_world_) / static_cast<float>(dt);
  this->prev_gt_stamp_ = stamp;
  this->prev_v_world_ = v_world;

  // Specific-force convention: at rest body-upright, the IMU reads ~(0,0,+g) in body
  // (confirmed by stationary-calibration gravity_dir output on this rig). Generalized
  // to moving: expected_accel_body = R^T * (a_world + (0,0,+g)). Sign of g is +,
  // not −, because the accelerometer measures proper acceleration (= inertial − g_world
  // = inertial + (0,0,+g) when world Z points up).
  const Eigen::Vector3f g_world(0.0f, 0.0f, +static_cast<float>(this->gravity_));
  const Eigen::Vector3f expected_accel_body = R.transpose() * (a_world + g_world);
  const Eigen::Vector3f expected_gyro_body = seed.v_ang_body;

  const Eigen::Vector3f gyro_res = measured_gyro - expected_gyro_body;
  const Eigen::Vector3f accel_res = measured_accel - expected_accel_body;

  this->rtk_gyro_bias_sum_ += gyro_res;
  this->rtk_accel_bias_sum_ += accel_res;
  this->rtk_gyro_bias_sq_sum_ += gyro_res.cwiseProduct(gyro_res);
  this->rtk_accel_bias_sq_sum_ += accel_res.cwiseProduct(accel_res);
  this->rtk_calib_count_++;

  const double elapsed = stamp - this->rtk_calib_start_stamp_;
  if (elapsed < this->rtk_calib_window_sec_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "IMU calibrating (RTK-driven)... %.1f/%.1fs (%d samples)",
                         elapsed, this->rtk_calib_window_sec_, this->rtk_calib_count_);
    return false;
  }
  if (this->rtk_calib_count_ < 2) {
    // Window expired but we got essentially no useful pairings (e.g., GT buffer
    // empty most of the window). Give up on RTK init and let the caller
    // decide — return false but signal via a warn.
    RCLCPP_WARN(this->get_logger(),
                "RTK init: window expired with only %d residual samples; cannot calibrate. "
                "Falling back to stationary path.", this->rtk_calib_count_);
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    return false;
  }

  const float n = static_cast<float>(this->rtk_calib_count_);
  const Eigen::Vector3f gyro_bias = this->rtk_gyro_bias_sum_ / n;
  const Eigen::Vector3f accel_bias = this->rtk_accel_bias_sum_ / n;

  // Sanity check on the averaged bias magnitudes. Noise variance is not a useful
  // signal here — finite-differencing GT velocity at IMU rate amplifies cm-level
  // GPS noise into ~10 m/s² of accel-residual stddev even on a stationary vehicle.
  // The mean averages that out cleanly, so we only reject if the averaged bias
  // itself is implausible. Typical biases on real IMUs: gyro <0.05 rad/s,
  // accel <0.3 m/s². Generous thresholds here so a moderately drifted IMU is
  // still accepted.
  if (gyro_bias.norm() > 1.0f || accel_bias.norm() > 5.0f) {
    RCLCPP_WARN(this->get_logger(),
                "RTK init: averaged bias magnitudes implausible (|gyro|=%.3f rad/s, "
                "|accel|=%.3f m/s^2); falling back to stationary calibration",
                gyro_bias.norm(), accel_bias.norm());
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    this->rtk_calib_count_ = 0;
    this->rtk_gyro_bias_sum_.setZero();
    this->rtk_accel_bias_sum_.setZero();
    this->rtk_gyro_bias_sq_sum_.setZero();
    this->rtk_accel_bias_sq_sum_.setZero();
    return false;
  }

  // Apply biases + seed state from the latest GT sample. The bias write is
  // inside geo.mtx: callbackImu reads state.b under the same mutex, and
  // (with the MutuallyExclusive IMU group) this is the only cross-thread
  // writer of state.b during init.
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.b.gyro = gyro_bias;
    this->state.b.accel = accel_bias;
    this->state.p = this->latest_rtk_seed_.p;
    this->state.q = this->latest_rtk_seed_.q;
    this->state.v.lin.b = this->latest_rtk_seed_.v_lin_body;
    this->state.v.lin.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_lin_body;
    this->state.v.ang.b = this->latest_rtk_seed_.v_ang_body;
    this->state.v.ang.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_ang_body;
    this->geo.prev_p = this->latest_rtk_seed_.p;
    this->geo.prev_q = this->latest_rtk_seed_.q;
    this->geo.prev_vel = this->state.v.lin.w;
  }

  this->imu_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(),
              "IMU calibrated (RTK-driven, %d samples, %.1fs): "
              "gyro_bias=[%.4f,%.4f,%.4f] accel_bias=[%.3f,%.3f,%.3f] "
              "seed_pos=[%.2f,%.2f,%.2f] seed_v=[%.2f,%.2f,%.2f]m/s",
              this->rtk_calib_count_, elapsed,
              gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
              accel_bias.x(), accel_bias.y(), accel_bias.z(),
              this->latest_rtk_seed_.p.x(), this->latest_rtk_seed_.p.y(), this->latest_rtk_seed_.p.z(),
              this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z());
  return true;
}

bool gicp_localization::LocalizationNode::maybeSnapPoseToGT(const char* reason) {
  // Entry trace — DEBUG: this fires on EVERY non-accepted scan (the streak
  // guard below returns early), so INFO here was pure log spam with
  // verbose_scan_log keeping INFO alive by default.
  RCLCPP_DEBUG(this->get_logger(),
               "GT recovery: maybeSnapPoseToGT entered (enabled=%d streak=%d/%d gt_received=%d cached=%d) reason='%s'",
               this->gt_recovery_enabled_,
               this->consecutive_failures_, this->gt_recovery_min_consecutive_failures_,
               this->gt_odom_received_.load(), this->gt_extrinsics_cached_.load(), reason);
  // Guards. Below-threshold guard is silent (frequent on every rejection until
  // streak builds up); the others log throttled info so a misconfiguration
  // doesn't silently disable recovery.
  if (!this->gt_recovery_enabled_) return false;
  if (this->consecutive_failures_ < this->gt_recovery_min_consecutive_failures_) return false;
  if (!this->gt_odom_received_.load()) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT odom received yet (streak=%d)",
                this->consecutive_failures_);
    return false;
  }
  if (!this->gt_extrinsics_cached_) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — base→%s TF not cached yet (streak=%d)",
                this->gt_body_frame_.c_str(), this->consecutive_failures_);
    return false;
  }

  const double recovery_stamp =
      this->t_prior_stamp_ > 0.0
          ? this->t_prior_stamp_
          : this->scan_stamp.seconds();
  GtSample gt;
  // Log buffer state before the lookup so we can diagnose silent failures.
  size_t buf_size = 0;
  double buf_oldest = 0.0, buf_newest = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
    buf_size = this->gt_odom_buffer_.size();
    if (buf_size > 0) {
      buf_oldest = this->gt_odom_buffer_.front().stamp;
      buf_newest = this->gt_odom_buffer_.back().stamp;
    }
  }
  bool got = this->getGtPoseAt(recovery_stamp, gt);
  RCLCPP_DEBUG(this->get_logger(),
               "GT recovery: lookup registration_stamp=%.3f got=%d "
               "buf=[size=%zu oldest=%.3f newest=%.3f] max_dt=%.3f",
               recovery_stamp, got, buf_size, buf_oldest, buf_newest,
               this->gt_odom_max_dt_);
  if (!got) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT sample within "
                "max_dt=%.3fs of registration stamp %.3f (streak=%d)",
                this->gt_odom_max_dt_, recovery_stamp,
                this->consecutive_failures_);
    return false;
  }
  if (!this->gtSampleIsRtkFixed(gt)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "GT recovery: deferring snap because the time-matched odometry is not "
        "RTK-fixed quality; continuing with LiDAR+IMU");
    return false;
  }

  // Pose composition: bring gt sample from gt_body_frame into base_frame.
  // Shared with the diagnostic cross-check via composeGtPoseInBase.
  Eigen::Vector3f p_new;
  Eigen::Quaternionf q_new;
  if (!this->composeGtPoseInBase(gt, p_new, q_new)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "GT recovery: composeGtPoseInBase returned false "
                         "(extrinsic not cached). Deferring snap.");
    return false;
  }

  // Twist composition (P2#2): resolve the linear and angular sources
  // INDEPENDENTLY. Run 12/13 Atlas odom carried a valid linear twist but ZERO
  // angular twist (the adapter didn't populate twist.angular from the gyro),
  // and the old all-or-nothing "twist_is_zero" check passed the zeros through
  // — every one of the 1,860 snaps reset the angular-rate state to zero
  // mid-turn. Sources by priority:
  //   angular: GT twist -> live IMU gyro (bias-corrected under geo.mtx below;
  //            angular rate is rigid-body-invariant and the Atlas gyro IS the
  //            measured body rate in these axes) -> zero.
  //   linear:  GT twist -> finite difference of the bracketing GT poses
  //            (handles both an unpopulated twist and a true standstill
  //            uniformly) -> keep the current state velocity (never zero a
  //            moving vehicle's velocity: with the old behavior the next
  //            IMU prior integrates from v=0 and immediately re-fails).
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  const bool gt_ang_valid = gt.v_ang_body.norm() >= 0.01f;
  const bool gt_lin_valid = gt.v_lin_body.norm() >= 0.05f;

  Eigen::Vector3f omega_base_body = Eigen::Vector3f::Zero();
  bool omega_from_imu = false;
  if (gt_ang_valid) {
    omega_base_body = R_gtbody_base * gt.v_ang_body;
  } else {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    if (this->first_imu_received &&
        std::abs(this->imu_meas.stamp - recovery_stamp) < 0.2) {
      // P3: imu_meas is bias-corrected at buffering time — use as-is.
      omega_base_body = this->imu_meas.ang_vel;
      omega_from_imu = true;
    }
  }

  Eigen::Vector3f v_base_body = Eigen::Vector3f::Zero();
  bool lin_resolved = false;
  bool lin_from_fd = false;
  if (gt_lin_valid) {
    v_base_body = R_gtbody_base * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
    lin_resolved = true;
  } else {
    Eigen::Vector3f v_fd_world;
    if (this->getGtFiniteDiffVelWorld(recovery_stamp, v_fd_world)) {
      v_base_body = q_new.conjugate() * v_fd_world;
      lin_resolved = true;
      lin_from_fd = true;
    }
  }

  static bool warned_twist_sources = false;
  if (!warned_twist_sources && (!gt_ang_valid || !gt_lin_valid)) {
    warned_twist_sources = true;
    RCLCPP_INFO(this->get_logger(),
                "GT recovery: GT twist partially unpopulated (lin=%.3f m/s%s, ang=%.3f rad/s%s); "
                "backfilling angular from %s and linear from %s",
                gt.v_lin_body.norm(), gt_lin_valid ? "" : " INVALID",
                gt.v_ang_body.norm(), gt_ang_valid ? "" : " INVALID",
                gt_ang_valid ? "GT" : (omega_from_imu ? "IMU gyro" : "zero"),
                gt_lin_valid ? "GT" : (lin_from_fd ? "GT pose finite-difference" : "current state"));
  }

  // Apply state. Caller (performLocalization) already holds pose_mutex (line 1768),
  // so we MUST NOT re-acquire it here — std::mutex is non-recursive and that would
  // deadlock the entire scan callback. geo.mtx, however, is taken in narrow scopes
  // by performLocalization, never held across this call site, so locking it here
  // is correct.
  this->current_pose.setIdentity();
  this->current_pose.block<3, 3>(0, 0) = q_new.toRotationMatrix();
  this->current_pose.block<3, 1>(0, 3) = p_new;
  Eigen::Vector3f v_base_world;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    // (P3: no bias subtraction here — imu_meas is bias-corrected at buffering;
    // omega_from_imu is noted only for the source log above.)
    (void)omega_from_imu;
    if (!lin_resolved) {
      // Last resort: preserve the IMU-propagated velocity through the snap.
      v_base_body = this->state.q.conjugate() * this->state.v.lin.w;
    }
    v_base_world = q_new * v_base_body;
    const Eigen::Vector3f omega_base_world = q_new * omega_base_body;
    this->state.p = p_new;
    this->state.q = q_new;
    this->state.v.lin.b = v_base_body;
    this->state.v.lin.w = v_base_world;
    this->state.v.ang.b = omega_base_body;
    this->state.v.ang.w = omega_base_world;
    // state.b.gyro and state.b.accel intentionally preserved.
    this->geo.prev_p = p_new;
    this->geo.prev_q = q_new;
    this->geo.prev_vel = v_base_world;
    ++this->geo.update_seq;  // discard any in-flight propagateState computations
  }
  this->basePose.p = p_new;
  this->basePose.q = q_new;
  this->prev_vel = v_base_world;

  RCLCPP_WARN(this->get_logger(),
              "Localization: ⟳ snapped pose to GT (%s after %d consecutive non-accepts) — "
              "pose=[%.2f,%.2f,%.2f] v=[%.2f,%.2f,%.2f]m/s ω=[%.2f,%.2f,%.2f]rad/s | gt_body=%s",
              reason, this->consecutive_failures_,
              p_new.x(), p_new.y(), p_new.z(),
              v_base_world.x(), v_base_world.y(), v_base_world.z(),
              omega_base_body.x(), omega_base_body.y(), omega_base_body.z(),
              this->gt_body_frame_.c_str());

  {
    geometry_msgs::msg::PoseStamped snap_msg;
    snap_msg.header.stamp = rclcpp::Time(
        static_cast<int64_t>(std::llround(recovery_stamp * 1.0e9)),
        this->get_clock()->get_clock_type());
    snap_msg.header.frame_id = this->map_frame;
    snap_msg.pose.position.x = p_new.x();
    snap_msg.pose.position.y = p_new.y();
    snap_msg.pose.position.z = p_new.z();
    snap_msg.pose.orientation.w = q_new.w();
    snap_msg.pose.orientation.x = q_new.x();
    snap_msg.pose.orientation.y = q_new.y();
    snap_msg.pose.orientation.z = q_new.z();
    if (this->gt_snap_pub) this->gt_snap_pub->publish(snap_msg);
  }

  this->consecutive_failures_ = 0;
  // A GT snap restores a known-good pose, so restart the dead-reckon clock (P3).
  this->last_accepted_scan_stamp_ = recovery_stamp;
  return true;
}

void gicp_localization::LocalizationNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu) {

  double stamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;

  Eigen::Vector3f ang_vel(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  Eigen::Vector3f lin_accel(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  // One-shot defensive check: warn if the incoming IMU header.frame_id does
  // not match the configured imu_frame. The dome's single-source design
  // assumes both are "imu_link" (Atlas Duo CoN, see localization.yaml); any
  // other combination usually indicates the imu_topic launch arg was
  // re-pointed at a different IMU without also updating
  // localization/imu_frame — the code would then silently treat the foreign
  // IMU's axes / lever-arm as if they were at imu_link. Atomic exchange
  // ensures the warning fires exactly once even with concurrent callbacks.
  // Empty frame_id is tolerated (some drivers leave it unset); only an
  // explicit mismatch trips the warn.
  const bool imu_frame_mismatch =
      !imu->header.frame_id.empty() &&
      imu->header.frame_id != this->imu_frame;
  if (!this->imu_frame_id_checked_.exchange(true)) {
    if (imu_frame_mismatch) {
      RCLCPP_WARN(this->get_logger(),
                  "IMU header.frame_id='%s' does not match configured "
                  "localization/imu_frame='%s'. Treating the IMU axes and "
                  "lever-arm as if at '%s' regardless of the message label. "
                  "If this is intentional (driver mislabels frame_id), "
                  "suppress this warning by updating localization/imu_frame "
                  "to match. If unintentional, the imu_topic remap is "
                  "probably pointing at the wrong IMU.",
                  imu->header.frame_id.c_str(),
                  this->imu_frame.c_str(),
                  this->imu_frame.c_str());
    }
  }
  if (imu_frame_mismatch && this->imu_require_frame_match_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Rejecting IMU sample: header.frame_id='%s' != localization/imu_frame='%s'",
                         imu->header.frame_id.c_str(), this->imu_frame.c_str());
    return;
  }

  // Cache IMU-to-baselink transform from TF (once)
  if (!this->imu_extrinsics_cached_) {
    try {
      auto tf_bi = this->tf_buffer->lookupTransform(
          this->base_frame, this->imu_frame, tf2::TimePointZero);
      Eigen::Quaternionf q_bi(
          tf_bi.transform.rotation.w, tf_bi.transform.rotation.x,
          tf_bi.transform.rotation.y, tf_bi.transform.rotation.z);
      Eigen::Vector3f t_bi(
          tf_bi.transform.translation.x, tf_bi.transform.translation.y,
          tf_bi.transform.translation.z);
      this->extrinsics.baselink2imu.R = q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu.t = t_bi;
      this->extrinsics.baselink2imu_T.setIdentity();
      this->extrinsics.baselink2imu_T.block<3, 3>(0, 0) = q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu_T.block<3, 1>(0, 3) = t_bi;
      this->imu_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Cached baselink->imu extrinsic: t=[%.3f,%.3f,%.3f]",
                  t_bi.x(), t_bi.y(), t_bi.z());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Cannot cache baselink->imu TF: %s", ex.what());
    }
  }

  if (!this->imu_extrinsics_cached_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Skipping IMU sample until baselink->imu TF is cached");
    return;
  }

  // Transform IMU measurements from IMU frame to baselink frame.
  const Eigen::Matrix3f& R = this->extrinsics.baselink2imu.R;
  const Eigen::Vector3f& t = this->extrinsics.baselink2imu.t;

  // Rotate angular velocity and linear acceleration to baselink frame
  Eigen::Vector3f ang_vel_bl = R * ang_vel;
  Eigen::Vector3f lin_accel_bl = R * lin_accel;

  // Lever-arm compensation: t is base->IMU, so r_IMU->base = -t.
  // a_base = a_imu - omega x (omega x t) - alpha x t; alpha is approximated
  // as zero because the angular-acceleration term is small at 100 Hz.
  lin_accel_bl -= ang_vel_bl.cross(ang_vel_bl.cross(t));

  ang_vel = ang_vel_bl;
  lin_accel = lin_accel_bl;

  ImuMeas imu_meas_temp;
  imu_meas_temp.stamp = stamp;
  // P3 (bonus): buffer BIAS-CORRECTED IMU so every consumer — propagateState,
  // integrateImu (T_prior) and the per-point deskew frames — integrates the
  // same corrected signal. Previously only propagateState subtracted state.b;
  // the prior/deskew path integrated the raw gyro/accel, so once RTK
  // calibration set a nonzero bias the two integration paths permanently
  // disagreed (worst during high-yaw-rate sweeps, where deskew rotation error
  // scales directly with the gyro bias). Mirrors upstream DLIO, which applies
  // the bias in the IMU callback before buffering.
  // NOTE: the calibration paths below intentionally keep consuming the RAW
  // ang_vel/lin_accel locals — bias estimation must see the uncorrected
  // signal. state.b is zero until calibration completes, so pre-calibration
  // buffered samples are unaffected.
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    imu_meas_temp.ang_vel = ang_vel - this->state.b.gyro;
    imu_meas_temp.lin_accel = lin_accel - this->state.b.accel;
  }

  // Calculate dt
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      imu_meas_temp.dt = stamp - this->imu_buffer.front().stamp;
    } else {
      imu_meas_temp.dt = 0.0;
    }

    this->imu_buffer.push_front(imu_meas_temp);
    this->imu_meas = imu_meas_temp;
  }

  if (!this->first_imu_received) {
    this->first_imu_received = true;
    this->first_imu_stamp_ = stamp;
    RCLCPP_INFO(this->get_logger(), "First IMU message received");

    // If RTK init is off entirely, go straight to the stationary path.
    if (!this->rtk_init_enabled_ && this->init_phase_.load() == InitPhase::WAITING) {
      this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    }
  }

  // Calibration phase machine. We may be:
  //   WAITING                 — RTK init enabled but no GT received yet
  //   RTK_CALIBRATING         — first GT arrived; accumulating IMU residuals against GT truth
  //   STATIONARY_CALIBRATING  — fallback (no GT in time, or RTK init disabled)
  //   DONE                    — biases applied; propagate normally
  // ----- Initialization / IMU-bias calibration (seed-and-go) -----
  // Localization output must never stop. We DECOUPLE output from bias
  // calibration: as soon as a usable state seed exists (initialized with a
  // pose/orientation from GT odom-init, a param initial pose, or the first
  // GICP scan), we fall through to propagateState() below and publish
  // continuously, using the most stable information available at the time.
  // Bias calibration (RTK-driven or stationary) then runs in the BACKGROUND
  // and applies its refined bias as a smooth correction when it completes --
  // no output gap. We suppress output (early return) ONLY while there is
  // genuinely nothing to propagate from yet, i.e. before any seed exists; in
  // that unseeded window the original WAITING -> RTK/STATIONARY fallback logic
  // still applies so we reach a first fix as fast as possible.
  if (!this->imu_calibrated_) {
    InitPhase phase = this->init_phase_.load();

    // True once we have something stable to propagate from. Mirrors the
    // propagateState() gate below so that "fall through" always yields output.
    const bool can_propagate_now =
        this->initialized.load() &&
        (this->geo.first_opt_done.load() || this->imu_only_mode_);

    if (phase == InitPhase::WAITING) {
      if (this->gt_odom_received_.load()) {
        // First GT sample has arrived — start RTK-driven calibration on the next IMU.
        this->init_phase_ = InitPhase::RTK_CALIBRATING;
        this->rtk_calib_start_stamp_ = stamp;
        RCLCPP_INFO(this->get_logger(),
                    "RTK init: GT odom received; starting RTK-driven IMU calibration "
                    "(window=%.1fs)", this->rtk_calib_window_sec_);
        phase = InitPhase::RTK_CALIBRATING;
      } else if (stamp - this->first_imu_stamp_ > this->rtk_fallback_timeout_sec_) {
        // No GT in time — fall back to stationary calibration.
        RCLCPP_WARN(this->get_logger(),
                    "RTK init: no GT odom within %.1fs of first IMU; falling back to "
                    "stationary IMU calibration", this->rtk_fallback_timeout_sec_);
        this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
        this->imu_calib_start_stamp_ = stamp;  // reset so the existing window starts now
        phase = InitPhase::STATIONARY_CALIBRATING;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "RTK init: waiting for first GT odom (elapsed=%.1f/%.1fs)",
                             stamp - this->first_imu_stamp_, this->rtk_fallback_timeout_sec_);
        // Seed-and-go: only block output if we have no seed yet. With a param
        // seed we dead-reckon here until GT arrives instead of going dark.
        if (!can_propagate_now) return;
      }
    }

    if (phase == InitPhase::RTK_CALIBRATING) {
      // Accumulate an RTK-driven bias estimate in the background. It only
      // completes on RTK-FIXED pairings; with degraded-only Atlas odom it
      // never finishes -- but that is now BENIGN, because once a seed exists
      // we keep propagating below rather than stalling, and a later FIXED
      // sample still completes the calibration opportunistically. We only
      // force a stationary fallback while still UNSEEDED (output dark), so a
      // usable estimate is reached promptly in that case (the P1 stall fix).
      if (!can_propagate_now) {
        const double rtk_elapsed = stamp - this->rtk_calib_start_stamp_;
        const double rtk_phase_budget =
            this->rtk_calib_window_sec_ + this->rtk_fallback_timeout_sec_;
        if (rtk_elapsed > rtk_phase_budget) {
          RCLCPP_WARN(this->get_logger(),
                      "RTK init: %.1fs in RTK_CALIBRATING with no seed and no usable "
                      "RTK-FIXED GT pairing; falling back to stationary IMU calibration",
                      rtk_elapsed);
          this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
          this->imu_calib_start_stamp_ = stamp;  // restart window in the stationary path
          return;  // still unseeded — stationary calibration begins next IMU
        }
      }
      if (this->tryRtkCalibrationStep(stamp, ang_vel, lin_accel)) {
        // tryRtkCalibrationStep applied biases + seeded state and set imu_calibrated_.
        this->init_phase_ = InitPhase::DONE;
      } else if (!this->imu_calibrated_ && !can_propagate_now) {
        return;  // no seed yet — keep output suppressed until we have one
      }
      // Seeded: fall through and propagate; RTK bias refines opportunistically.
    } else if (phase == InitPhase::STATIONARY_CALIBRATING) {
      // Stationary path: assume omega=0 and accel direction = gravity. Used
      // when no GT seed is available; it provides the initial gravity/orientation.
      if (this->imu_calib_start_stamp_ < 0.0) {
        this->imu_calib_start_stamp_ = stamp;
      }

      this->imu_calib_gyro_sum_ += ang_vel;
      this->imu_calib_accel_sum_ += lin_accel;
      // Hitch Sensor Dome — also track ||a|| sum and sum-of-squares so
      // we can compute σ_||a|| at the end of the window and refuse the
      // calibration if the vehicle was moving.
      const double a_norm = lin_accel.cast<double>().norm();
      this->imu_calib_acc_norm_sum_   += a_norm;
      this->imu_calib_acc_norm_sumsq_ += a_norm * a_norm;
      this->imu_calib_count_++;

      double elapsed = stamp - this->imu_calib_start_stamp_;
      if (elapsed >= this->imu_calib_time_ && this->imu_calib_count_ > 0) {
        // ---- Motion-variance gate ----
        const double n = static_cast<double>(this->imu_calib_count_);
        const double mean   = this->imu_calib_acc_norm_sum_   / n;
        const double mean_sq = this->imu_calib_acc_norm_sumsq_ / n;
        const double var = std::max(0.0, mean_sq - mean * mean);
        const double sigma = std::sqrt(var);

        if (this->imu_calib_motion_sigma_max_ > 0.0 &&
            sigma > this->imu_calib_motion_sigma_max_) {
          // Vehicle is moving — refuse this window, reset, and try again.
          // First strike emits a bold-yellow one-shot warning; subsequent
          // resets get a quieter INFO_THROTTLE so the log isn't flooded.
          this->imu_calib_attempt_++;
          const char* YELLOW = "\033[1;33m";
          const char* RESET  = "\033[0m";
          if (!this->imu_calib_motion_warned_) {
            RCLCPP_WARN(this->get_logger(),
              "%sStationary IMU calibration REFUSED — motion detected "
              "(σ_||a||=%.3f m/s² > %.3f m/s² over %.1fs / %d samples). "
              "The vehicle appears to be moving. The bias / gravity estimate "
              "from a moving-window stationary calibration would tilt the "
              "world frame and degrade scan matching for the first ~100 "
              "scans. Resetting the window. Bring the vehicle to rest, or "
              "enable localization/rtk_init/enable to calibrate from RTK GT "
              "while moving.%s",
              YELLOW, sigma, this->imu_calib_motion_sigma_max_, elapsed,
              this->imu_calib_count_, RESET);
            this->imu_calib_motion_warned_ = true;
          } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Stationary calibration window %d refused (σ_||a||=%.3f); "
              "waiting for vehicle to come to rest.",
              this->imu_calib_attempt_, sigma);
          }
          // Reset accumulators and start a fresh window.
          this->imu_calib_start_stamp_ = -1.0;
          this->imu_calib_count_ = 0;
          this->imu_calib_gyro_sum_  = Eigen::Vector3f::Zero();
          this->imu_calib_accel_sum_ = Eigen::Vector3f::Zero();
          this->imu_calib_acc_norm_sum_   = 0.0;
          this->imu_calib_acc_norm_sumsq_ = 0.0;
          return;  // wait for the next IMU sample
        }

        Eigen::Vector3f gyro_avg = this->imu_calib_gyro_sum_ / static_cast<float>(this->imu_calib_count_);
        Eigen::Vector3f accel_avg = this->imu_calib_accel_sum_ / static_cast<float>(this->imu_calib_count_);

        this->state.b.gyro = gyro_avg;

        Eigen::Vector3f grav_world(0.f, 0.f, -1.f);
        Eigen::Vector3f grav_body = accel_avg.normalized();
        Eigen::Quaternionf q_init = Eigen::Quaternionf::FromTwoVectors(grav_body, grav_world);

        Eigen::Vector3f expected_grav_body = q_init.conjugate()._transformVector(
            Eigen::Vector3f(0.f, 0.f, -static_cast<float>(this->gravity_)));
        this->state.b.accel = accel_avg - expected_grav_body;

        // Only adopt the accel-derived orientation when we have no better one.
        // If a pose source already seeded orientation (param or GT odom-init),
        // keep it -- overwriting it here would jerk the already-published
        // estimate, and the stationary assumption is invalid if that seed came
        // from a moving vehicle.
        if (!this->use_param_initial_pose_ && !this->use_odom_init_applied_) {
          std::lock_guard<std::mutex> lock(this->geo.mtx);
          this->state.q = q_init;
          this->geo.prev_q = q_init;
        }

        this->imu_calibrated_ = true;
        this->init_phase_ = InitPhase::DONE;
        RCLCPP_INFO(this->get_logger(),
                    "IMU calibrated (stationary, %d samples, %.1fs): gyro_bias=[%.4f,%.4f,%.4f] "
                    "accel_bias=[%.3f,%.3f,%.3f] gravity_dir=[%.3f,%.3f,%.3f]",
                    this->imu_calib_count_, elapsed,
                    gyro_avg.x(), gyro_avg.y(), gyro_avg.z(),
                    this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z(),
                    grav_body.x(), grav_body.y(), grav_body.z());
      } else if (!can_propagate_now) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "IMU calibrating (stationary)... %.1f/%.1fs (%d samples)",
                             elapsed, this->imu_calib_time_, this->imu_calib_count_);
        return;  // no seed yet — suppress output until gravity/orientation known
      }
      // Seeded: fall through and propagate while stationary stats accumulate.
    }
  }

  // Propagate state with geometric observer (only after initialization)
  // Note: counters are member-like but use thread_local to avoid data races
  // when the Reentrant callback group processes IMU concurrently.
  thread_local int propagate_calls = 0;
  thread_local int imu_total = 0;
  thread_local bool logged_first_propagate = false;
  imu_total++;

  if (this->initialized && (this->geo.first_opt_done || this->imu_only_mode_)) {
    this->propagateState();
    propagate_calls++;

    // Log first successful propagation
    if (!logged_first_propagate) {
      RCLCPP_INFO(this->get_logger(), "First IMU propagation successful! Starting high-frequency odometry.");
      logged_first_propagate = true;
    }
  }

  // Debug: Log IMU and propagation rates periodically
  thread_local int imu_count = 0;
  if (++imu_count % 100 == 0) {  // Log every 100 IMU messages (~1 second)
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    RCLCPP_INFO(this->get_logger(), "IMU rate check: %d callbacks, %d propagations, initialized=%d, geo_init=%d",
                imu_total, propagate_calls, this->initialized.load(), this->geo.first_opt_done.load());
    imu_total = 0;
    propagate_calls = 0;
  }
}

bool gicp_localization::LocalizationNode::imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {

  std::lock_guard<std::mutex> lock(this->mtx_imu);

  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Not enough IMU data yet
    return false;
  }

  auto imu_it = this->imu_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->imu_buffer.end()) {
    // not enough IMU measurements
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImu(
    double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
    Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT guard: empty=%d start=%.6f front=%.6f (start>front=%d)\n",
                   (int)sorted_timestamps.empty(), start_time,
                   sorted_timestamps.empty() ? 0.0 : sorted_timestamps.front(),
                   (int)(!sorted_timestamps.empty() && start_time > sorted_timestamps.front()));
      std::fflush(stderr);
    }
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it) == false) {
    double front_s = -1, back_s = -1;
    size_t sz = 0;
    {
      std::lock_guard<std::mutex> lk(this->mtx_imu);
      sz = this->imu_buffer.size();
      if (sz > 0) { front_s = this->imu_buffer.front().stamp; back_s = this->imu_buffer.back().stamp; }
    }
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT range: start=%.6f end=%.6f buf_sz=%zu front=%.6f back=%.6f (front<end=%d)\n",
                   start_time, sorted_timestamps.back(), sz, front_s, back_s,
                   (int)(sz > 0 && front_s < sorted_timestamps.back()));
      std::fflush(stderr);
    }
    return empty;
  }

  if ((begin_imu_it + 1) == end_imu_it) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT begin+1==end: start=%.6f end=%.6f begin.stamp=%.6f end.base.stamp=%.6f\n",
                   start_time, sorted_timestamps.back(),
                   begin_imu_it->stamp, end_imu_it.base()->stamp);
      std::fflush(stderr);
    }
    return empty;
  }

  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it+1);

  // Time between first two IMU samples
  double dt = f2.dt;

  if (dt < 1e-6) {
    if (this->verbose_) {
      std::fprintf(stderr, "[IMU_INT] REJECT dt: f1.stamp=%.6f f2.stamp=%.6f f2.dt=%.9f\n",
                   f1.stamp, f2.stamp, dt);
      std::fflush(stderr);
    }
    return empty;
  }

  // Time between first IMU sample and start_time
  double idt = start_time - f1.stamp;

  // Angular acceleration between first two IMU samples
  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  // Average angular velocity (reversed) between first IMU sample and start_time
  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);

  // Set q_init to orientation at first IMU sample
  q_init = Eigen::Quaternionf (
    q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
    q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
    q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
    q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
  );
  q_init.normalize();

  // Average angular velocity between first two IMU samples
  Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;

  // Orientation at second IMU sample
  Eigen::Quaternionf q2 (
    q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
    q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
    q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
    q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
  );
  q2.normalize();

  // Acceleration at first IMU sample
  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= this->gravity_;

  // Acceleration at second IMU sample
  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= this->gravity_;

  // Jerk between first two IMU samples
  Eigen::Vector3f j = (a2 - a1) / dt;

  // Set v_init to velocity at first IMU sample (go backwards from start_time)
  v_init -= a1*idt + 0.5*j*idt*idt;

  // Set p_init to position at first IMU sample (go backwards from start_time)
  p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    if (dt < 1e-6) {
      prev_imu_it = imu_it;
      continue;
    }

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

    // Orientation
    q = Eigen::Quaternionf (
      q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
      q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
      q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
      q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
    );
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;

      // Orientation
      Eigen::Quaternionf q_i (
        q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
        q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
        q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
        q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
      );
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // Position
    p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

    // Velocity
    v += a0*dt + 0.5*j_dt*dt;

    prev_imu_it = imu_it;

  }

  return imu_se3;

}

void gicp_localization::LocalizationNode::propagateState() {

  ImuMeas imu_local;
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    imu_local = this->imu_meas;
  }

  double dt = imu_local.dt;

  if (dt <= 0.0 || dt > 1.0) {
    static int skip_count = 0;
    if (++skip_count % 100 == 0) {
      RCLCPP_WARN(this->get_logger(), "Skipping propagation due to invalid dt: %.6f (skipped %d times)", dt, skip_count);
    }
    return;  // Skip invalid dt
  }

  // Read current state with minimal lock time
  Eigen::Vector3f current_p;
  Eigen::Quaternionf current_q;
  Eigen::Vector3f current_v_lin_w;
  Eigen::Vector3f bias_gyro;
  Eigen::Vector3f bias_accel;
  uint64_t seq_at_read;

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    current_p = this->state.p;
    current_q = this->state.q;
    current_v_lin_w = this->state.v.lin.w;
    bias_gyro = this->state.b.gyro;
    bias_accel = this->state.b.accel;
    seq_at_read = this->geo.update_seq;
  }

  // Do computation without holding lock
  Eigen::Quaternionf qhat = current_q;
  Eigen::Quaternionf omega;
  Eigen::Vector3f world_accel;

  // P3: biases are now subtracted ONCE, at buffering time in callbackImu, so
  // the buffered measurement is already corrected — do NOT subtract again here
  // (bias_gyro/bias_accel are still read above for the periodic status log).
  Eigen::Vector3f ang_vel_corrected = imu_local.ang_vel;
  Eigen::Vector3f lin_accel_corrected = imu_local.lin_accel;

  // Transform accel from body to world frame and subtract gravity
  world_accel = qhat._transformVector(lin_accel_corrected);

  // Log propagation status periodically
  static int propagate_count = 0;
  if (++propagate_count % 1000 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer: pos_z=%.3f vel_z=%.3f | accel_raw_z=%.3f bias_z=%.3f world_accel_z=%.3f | gravity=%.2f",
                current_p.z(), current_v_lin_w.z(),
                imu_local.lin_accel.z(), bias_accel.z(), world_accel.z(),
                this->gravity_);
  }

  // Position propagation (with gravity compensation)
  Eigen::Vector3f new_p = current_p;
  new_p += current_v_lin_w*dt + 0.5f*dt*dt*world_accel;
  new_p[2] -= 0.5f * dt * dt * static_cast<float>(this->gravity_);

  // Velocity propagation (with gravity compensation)
  Eigen::Vector3f new_v_lin_w = current_v_lin_w + world_accel*dt;
  new_v_lin_w[2] -= dt * static_cast<float>(this->gravity_);

  // Ground vehicle Z-velocity damping (same as in updateState)
  new_v_lin_w[2] *= (1.0f - dt * static_cast<float>(this->geo_Kz_damping_));

  // Orientation propagation
  omega.w() = 0;
  omega.vec() = ang_vel_corrected;
  Eigen::Quaternionf tmp = qhat * omega;
  Eigen::Quaternionf new_q;
  new_q.w() = qhat.w() + 0.5 * dt * tmp.w();
  new_q.vec() = qhat.vec() + 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  new_q.normalize();

  // Store angular velocity
  Eigen::Vector3f new_v_ang_b = ang_vel_corrected;
  Eigen::Vector3f new_v_ang_w = new_q.toRotationMatrix() * new_v_ang_b;

  // Validate computed state before publishing
  bool state_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) &&
                     std::isfinite(new_p.z()) && std::isfinite(new_q.w()) &&
                     std::isfinite(new_q.x()) && std::isfinite(new_q.y()) &&
                     std::isfinite(new_q.z()) &&
                     std::isfinite(new_v_lin_w.x()) && std::isfinite(new_v_lin_w.y()) &&
                     std::isfinite(new_v_lin_w.z());

  if (!state_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Skipping odometry publish - state contains invalid values (p=[%.3f,%.3f,%.3f], q=[%.3f,%.3f,%.3f,%.3f])",
                         new_p.x(), new_p.y(), new_p.z(),
                         new_q.w(), new_q.x(), new_q.y(), new_q.z());
    return;  // Skip publishing if state contains invalid values
  }

  // Use IMU timestamp (from bag file or sensor)
  rclcpp::Time current_time;
  current_time = rclcpp::Time(static_cast<int64_t>(imu_local.stamp * 1e9));

  // Log successful validation on first publish
  static bool logged_first_publish = false;
  if (!logged_first_publish) {
    RCLCPP_INFO(this->get_logger(), "First odometry publish! Using IMU timestamp: %.3f", imu_local.stamp);
    logged_first_publish = true;
  }

  // Build odometry message from computed values (not from this->state to avoid race condition)
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = this->map_frame;
  odom_msg.child_frame_id = this->base_frame;

  // Position and orientation from propagated state
  odom_msg.pose.pose.position.x = new_p.x();
  odom_msg.pose.pose.position.y = new_p.y();
  odom_msg.pose.pose.position.z = new_p.z();
  odom_msg.pose.pose.orientation.w = new_q.w();
  odom_msg.pose.pose.orientation.x = new_q.x();
  odom_msg.pose.pose.orientation.y = new_q.y();
  odom_msg.pose.pose.orientation.z = new_q.z();

  // Velocity from propagated state (in world frame)
  odom_msg.twist.twist.linear.x = new_v_lin_w.x();
  odom_msg.twist.twist.linear.y = new_v_lin_w.y();
  odom_msg.twist.twist.linear.z = new_v_lin_w.z();
  odom_msg.twist.twist.angular.x = new_v_ang_w.x();
  odom_msg.twist.twist.angular.y = new_v_ang_w.y();
  odom_msg.twist.twist.angular.z = new_v_ang_w.z();

  // Pose covariance: diagonal only.
  // When GICP is accepted use sqrt(fitness) as a positional sigma (metres).
  // When dead-reckoning (consecutive GICP failures) inflate by elapsed time and
  // distance travelled (speed*time), not by missed-scan count (P3).
  {
    const double kBaseSigmaXY  = 0.05;   // m   — floor for accepted scans
    const double kBaseSigmaZ   = 0.10;   // m   — z less constrained by LiDAR
    const double kBaseSigmaRot = 0.01;   // rad — roll/pitch/yaw floor
    const double kFitnessScale = 1.0;    // sigma_xy = max(base, scale * sqrt(fitness))

    double s_xy, s_z, s_rot;
    if (this->last_gicp_valid_ && this->last_fitness_score_ >= 0.0) {
      double f_sigma = kFitnessScale * std::sqrt(this->last_fitness_score_);
      s_xy  = std::max(kBaseSigmaXY,  f_sigma);
      s_z   = std::max(kBaseSigmaZ,   2.0 * f_sigma);
      s_rot = std::max(kBaseSigmaRot, 0.1 * f_sigma);
    } else {
      s_xy  = kBaseSigmaXY;
      s_z   = kBaseSigmaZ;
      s_rot = kBaseSigmaRot;
    }

    // Dead-reckoning inflation (P3). While GICP is failing the estimate rides on
    // IMU integration, whose error grows with DISTANCE travelled (speed*time)
    // plus a slow time term -- not with the raw number of missed scans. At racing
    // speed N missed scans cover ~4x more ground at 200 mph than at 100 mph, so a
    // count-based term under-reports uncertainty exactly when it matters. Keyed on
    // consecutive_failures_ (the real dead-reckon signal) and added on top of the
    // base/fitness sigma above. dr_cov_time_rate_ = dr_cov_dist_frac_ = 0 disables.
    if (this->consecutive_failures_ > 0) {
      const double elapsed_dr = (this->last_accepted_scan_stamp_ > 0.0)
          ? std::max(0.0, imu_local.stamp - this->last_accepted_scan_stamp_)
          : 0.0;
      const double speed = static_cast<double>(new_v_lin_w.norm());
      const double drift = this->dr_cov_time_rate_ * elapsed_dr +
                           this->dr_cov_dist_frac_ * speed * elapsed_dr;
      s_xy  += drift;
      s_z   += 2.0 * drift;
      s_rot += 0.05 * drift;
    }
    auto& c = odom_msg.pose.covariance;
    c.fill(0.0);
    c[0]  = s_xy  * s_xy;   // x
    c[7]  = s_xy  * s_xy;   // y
    c[14] = s_z   * s_z;    // z
    c[21] = s_rot * s_rot;  // roll
    c[28] = s_rot * s_rot;  // pitch
    c[35] = s_rot * s_rot;  // yaw
  }

  this->localized_odom_pub->publish(odom_msg);

  // Publish UTM-frame odometry
  if (this->utm_enabled_) {
    Eigen::Matrix4f T_map_base = Eigen::Matrix4f::Identity();
    T_map_base.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    T_map_base.block<3, 1>(0, 3) = new_p;
    Eigen::Matrix4f T_utm_base = this->T_utm_map_ * T_map_base;
    Eigen::Vector3f utm_p = T_utm_base.block<3, 1>(0, 3);
    Eigen::Quaternionf utm_q(T_utm_base.block<3, 3>(0, 0));
    utm_q.normalize();
    // Rotate velocity into UTM frame
    Eigen::Vector3f utm_v_lin = this->T_utm_map_.block<3, 3>(0, 0) * new_v_lin_w;

    nav_msgs::msg::Odometry utm_odom_msg;
    utm_odom_msg.header.stamp = current_time;
    utm_odom_msg.header.frame_id = this->utm_frame;
    utm_odom_msg.child_frame_id = this->base_frame;
    utm_odom_msg.pose.pose.position.x = utm_p.x();
    utm_odom_msg.pose.pose.position.y = utm_p.y();
    utm_odom_msg.pose.pose.position.z = utm_p.z();
    utm_odom_msg.pose.pose.orientation.w = utm_q.w();
    utm_odom_msg.pose.pose.orientation.x = utm_q.x();
    utm_odom_msg.pose.pose.orientation.y = utm_q.y();
    utm_odom_msg.pose.pose.orientation.z = utm_q.z();
    utm_odom_msg.twist.twist.linear.x = utm_v_lin.x();
    utm_odom_msg.twist.twist.linear.y = utm_v_lin.y();
    utm_odom_msg.twist.twist.linear.z = utm_v_lin.z();
    utm_odom_msg.twist.twist.angular.x = new_v_ang_w.x();
    utm_odom_msg.twist.twist.angular.y = new_v_ang_w.y();
    utm_odom_msg.twist.twist.angular.z = new_v_ang_w.z();
    this->utm_odom_pub->publish(utm_odom_msg);
  }

  if (this->imu_only_mode_) {
    // Publish pose/TF directly from propagated IMU state when GICP is disabled.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = this->map_frame;
    pose_msg.pose.position.x = new_p.x();
    pose_msg.pose.position.y = new_p.y();
    pose_msg.pose.position.z = new_p.z();
    pose_msg.pose.orientation.w = new_q.w();
    pose_msg.pose.orientation.x = new_q.x();
    pose_msg.pose.orientation.y = new_q.y();
    pose_msg.pose.orientation.z = new_q.z();
    this->pose_pub->publish(pose_msg);

    static int path_decimator = 0;
    if (++path_decimator % 10 == 0) {
      std::lock_guard<std::mutex> path_lock(this->pose_mutex);
      if (this->path_buffer_.size() >= 10000) this->path_buffer_.pop_front();
      this->path_buffer_.push_back(pose_msg);
      if (this->path_pub && this->path_pub->get_subscription_count() > 0) {
        this->path_msg.header.stamp = current_time;
        this->path_msg.header.frame_id = this->map_frame;
        this->path_msg.poses.assign(this->path_buffer_.begin(), this->path_buffer_.end());
        this->path_pub->publish(this->path_msg);
      }
    }

    if (this->publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = current_time;
      transform_stamped.header.frame_id = this->map_frame;
      transform_stamped.child_frame_id = this->base_frame;
      transform_stamped.transform.translation.x = new_p.x();
      transform_stamped.transform.translation.y = new_p.y();
      transform_stamped.transform.translation.z = new_p.z();
      transform_stamped.transform.rotation.w = new_q.w();
      transform_stamped.transform.rotation.x = new_q.x();
      transform_stamped.transform.rotation.y = new_q.y();
      transform_stamped.transform.rotation.z = new_q.z();
      this->tf_broadcaster->sendTransform(transform_stamped);
    }
  }

  // Note: Pose publishing is done from publishPose() at GICP rate only
  // With unreliable IMU data, geometric observer propagation is not accurate
  // Better to publish only GICP-corrected poses at ~15 Hz than poorly-propagated poses at 100 Hz

  // Debug: Count published messages
  static int odom_publish_count = 0;
  static auto last_report_time = std::chrono::steady_clock::now();
  odom_publish_count++;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time).count();
  if (elapsed >= 1000) {  // Report every second
    RCLCPP_INFO(this->get_logger(), "Odometry publish rate: %d Hz", odom_publish_count);
    odom_publish_count = 0;
    last_report_time = now;
  }

  // Update state AFTER publishing. If updateState() ran between our read and
  // this write (GICP corrected the state), skip the write to avoid overwriting
  // the fresh GICP correction with stale IMU-propagated values.
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    if (this->geo.update_seq == seq_at_read) {
      this->state.p = new_p;
      this->state.q = new_q;
      this->state.v.lin.w = new_v_lin_w;
      this->state.v.lin.b = new_q.toRotationMatrix().inverse() * new_v_lin_w;
      this->state.v.ang.b = new_v_ang_b;
      this->state.v.ang.w = new_v_ang_w;
    }
  }

  if (this->imu_only_mode_) {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = new_p;
  }

  // Don't publish TF from propagated state - only from GICP-corrected pose in publishPose()
  // High-frequency TF from IMU propagation drifts between GICP corrections
  // TF publishing is handled in publishPose() at GICP rate (15 Hz) with corrected pose

}

void gicp_localization::LocalizationNode::updateState() {

  // Lock thread to prevent state from being accessed by propagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->basePose.p;
  Eigen::Quaternionf qin = this->basePose.q;
  double dt = this->observer_dt_;

  // On very first update after initialization, dt might be large
  // Just skip the update but don't warn
  if (dt <= 0.0) {
    return;  // Skip invalid dt
  }

  if (dt > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Large dt in updateState: %.3f sec, skipping update", dt);
    return;  // Skip if dt is too large (probably first update or dropped scans)
  }

  // Bound the effective timestep used for the proportional corrections. The
  // observer applies dt*K, which is forward-Euler and stable only for dt*K < 2;
  // at Kv=11.25 that bound is hit at dt≈0.18 s, so a 0.3-0.5 s scan gap (dropped
  // LiDAR frames / high-speed racing) would otherwise inject an unstable
  // correction from a single GICP residual. At nominal ~10 Hz dt_eff == dt.
  const double dt_eff = std::min(dt, this->geo_observer_dt_max_);

  // Validate inputs
  bool inputs_valid = std::isfinite(pin.x()) && std::isfinite(pin.y()) && std::isfinite(pin.z()) &&
                      std::isfinite(qin.w()) && std::isfinite(qin.x()) && std::isfinite(qin.y()) && std::isfinite(qin.z()) &&
                      std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) && std::isfinite(this->state.p.z()) &&
                      std::isfinite(this->state.q.w()) && std::isfinite(this->state.q.x()) &&
                      std::isfinite(this->state.q.y()) && std::isfinite(this->state.q.z());

  if (!inputs_valid) {
    RCLCPP_WARN(this->get_logger(), "Invalid inputs in updateState - pin=[%.3f,%.3f,%.3f] state.p=[%.3f,%.3f,%.3f]",
                pin.x(), pin.y(), pin.z(), this->state.p.x(), this->state.p.y(), this->state.p.z());
    return;
  }

  // P3: delta-form correction target. basePose (pin/qin) is the GICP result
  // at the scan's MEDIAN POINT TIME; by now the state has been IMU-propagated
  // 0.1-0.3 s past it (half sweep + queueing + solve). Pulling the current
  // state toward the stale absolute pose drags it backwards along the
  // trajectory — zero-mean on straights, a systematic yaw/position lag in
  // turns. Instead, form the time-free world-frame correction
  //   T_corr = T_meas * inv(T_prior)   (both at median scan time)
  // and target T_corr (x) current_state: if IMU and GICP agree, T_corr = I and
  // the correction vanishes regardless of latency. Gains/structure unchanged.
  if (this->geo_delta_correction_ && matrixFinite(this->observer_prior_pose_)) {
    Eigen::Quaternionf q_prior(this->observer_prior_pose_.block<3, 3>(0, 0));
    q_prior.normalize();
    const Eigen::Vector3f p_prior = this->observer_prior_pose_.block<3, 1>(0, 3);
    const Eigen::Quaternionf q_corr = (qin * q_prior.conjugate()).normalized();
    const Eigen::Vector3f t_corr = pin - q_corr._transformVector(p_prior);
    // Sanity: the prior/measurement delta is bounded by the jump gate on
    // accepted scans; a huge delta here means observer_prior_pose_ is stale
    // or corrupted — fall back to the legacy absolute target for this update.
    const double corr_angle_deg =
        2.0 * std::acos(std::clamp(static_cast<double>(std::abs(q_corr.w())), 0.0, 1.0)) * 180.0 / M_PI;
    const float corr_trans =
        (q_corr._transformVector(this->state.p) + t_corr - this->state.p).norm();
    if (corr_angle_deg < 45.0 && std::isfinite(corr_trans) && corr_trans < 100.0f) {
      pin = q_corr._transformVector(this->state.p) + t_corr;
      qin = (q_corr * this->state.q).normalized();
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "updateState: delta correction implausible (rot=%.1fdeg trans=%.2fm); "
                           "using legacy absolute target this update",
                           corr_angle_deg, static_cast<double>(corr_trans));
    }
  }

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Construct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.0;
  if (qe.w() < 0) {
    sgn = -1.0;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - fabs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  // Position error
  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  // Optional online bias adaptation. Keep disabled by default for fused
  // Point One (Atlas) INS input so GICP residuals do not chase drift by rewriting the
  // trusted IMU bias estimate. Setting Kab/Kgb > 0 restores the upstream DLIO
  // adaptive observer behavior.
  if (this->geo_Kab_ > 0.0) {
    const double abias_max = this->geo_abias_max_;
    this->state.b.accel -= dt_eff * this->geo_Kab_ * err_body;
    this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);
  }

  if (this->geo_Kgb_ > 0.0) {
    const double gbias_max = this->geo_gbias_max_;
    this->state.b.gyro[0] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.x();
    this->state.b.gyro[1] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.y();
    this->state.b.gyro[2] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.z();
    this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);
  }

  // Hitch Sensor Dome — yaw-rate-adaptive gain attenuation.
  // At high yaw rate (corner entries) GICP is most likely to slide along
  // an unconstrained axis; meanwhile the IMU integration is at its most
  // informative (gyro doing real work). Scale Kp and Kq down so the
  // IMU prediction takes precedence during the transient. Kv and bias
  // gains are intentionally untouched — the bias estimator still
  // benefits from the (smaller) corrections.
  //
  // Scale shape:
  //   |ω_z| ≤ threshold       → scale = 1.0  (no attenuation)
  //   |ω_z| ≥ saturation      → scale = min_scale
  //   threshold < |ω_z| < sat → linear interpolation
  float gain_scale = 1.0f;
  if (this->yawrate_attenuation_enabled_) {
    const float yaw_rate = std::abs(this->state.v.ang.b[2]);
    const float lo = static_cast<float>(this->yawrate_attenuation_threshold_);
    const float hi = static_cast<float>(this->yawrate_attenuation_saturation_);
    const float min_s = static_cast<float>(this->yawrate_attenuation_min_scale_);
    if (yaw_rate <= lo) {
      gain_scale = 1.0f;
    } else if (yaw_rate >= hi) {
      gain_scale = min_s;
    } else {
      const float t = (yaw_rate - lo) / std::max(hi - lo, 1e-6f);
      gain_scale = 1.0f - t * (1.0f - min_s);
    }
    if (gain_scale < 0.99f) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Observer: yaw-rate attenuation active — |ω_z|=%.2f rad/s, "
        "Kp/Kq scale=%.2f",
        yaw_rate, gain_scale);
    }
  }
  const float Kp_eff = static_cast<float>(this->geo_Kp_) * gain_scale;
  const float Kq_eff = static_cast<float>(this->geo_Kq_) * gain_scale;


  // Proportional observer correction (matching upstream DLIO design), using the
  // bounded dt_eff and optional hard clamps on the per-update correction
  // magnitude so one GICP residual after a gap can't yank the state.
  // Position correction (gain-attenuated at high yaw rate — Hitch feature above)
  Eigen::Vector3f pos_corr = dt_eff * Kp_eff * err;
  if (this->geo_max_pos_correction_ > 0.0) {
    const float n = pos_corr.norm();
    if (n > this->geo_max_pos_correction_) {
      pos_corr *= static_cast<float>(this->geo_max_pos_correction_ / n);
    }
  }
  this->state.p += pos_corr;

  // Velocity correction
  Eigen::Vector3f vel_corr = dt_eff * this->geo_Kv_ * err;
  if (this->geo_max_vel_correction_ > 0.0) {
    const float n = vel_corr.norm();
    if (n > this->geo_max_vel_correction_) {
      vel_corr *= static_cast<float>(this->geo_max_vel_correction_ / n);
    }
  }
  this->state.v.lin.w += vel_corr;

  // Ground vehicle constraint: damp vertical velocity toward zero.
  // A ground vehicle's true Z-velocity is ~0; residual gravity miscompensation
  // causes vel_z to drift. Apply exponential decay each update.
  this->state.v.lin.w[2] *= (1.0f - dt_eff * this->geo_Kz_damping_);

  // Orientation correction (gain-attenuated at high yaw rate)
  this->state.q.w() += dt_eff * Kq_eff * qcorr.w();
  this->state.q.vec() += dt_eff * Kq_eff * qcorr.vec();
  this->state.q.normalize();

  // Validate updated state
  bool state_valid_after = std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) &&
                           std::isfinite(this->state.p.z()) && std::isfinite(this->state.q.w()) &&
                           std::isfinite(this->state.v.lin.w.x()) && std::isfinite(this->state.v.lin.w.y()) &&
                           std::isfinite(this->state.v.lin.w.z());

  if (!state_valid_after) {
    RCLCPP_ERROR(this->get_logger(), "State became invalid after update! Resetting to GICP measurement.");
    // Reset to valid GICP measurement
    this->state.p = pin;
    this->state.q = qin;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
  }

  // Store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;
  ++this->geo.update_seq;  // Signal propagateState to discard stale computations

  // Log update status periodically
  static int update_count = 0;
  if (++update_count % 20 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer | pos_err=[%.3f,%.3f,%.3f]m vel=[%.2f,%.2f,%.2f]m/s | bias_gyro=[%.4f,%.4f,%.4f] bias_accel=[%.3f,%.3f,%.3f]",
                err.x(), err.y(), err.z(),
                this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
                this->state.b.gyro.x(), this->state.b.gyro.y(), this->state.b.gyro.z(),
                this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Geo Observer: pos_err=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] bias_a=[%.3f,%.3f,%.3f]",
               err.x(), err.y(), err.z(),
               this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
               this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());

}
