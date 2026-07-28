#ifndef GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP
#define GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_proxy.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <small_gicp/registration/termination_criteria.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace gicp_plusplus {

inline Eigen::Vector3d so3LogVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  Eigen::Vector3d axis = aa.axis();
  if (!axis.allFinite() || std::abs(aa.angle()) < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return aa.angle() * axis;
}

struct GroundVehicleGeneralFactor {
  GroundVehicleGeneralFactor() {
    dof_mask.setOnes();
    rotation_prior_R.setIdentity();
    rotation_prior_info.setZero();
  }

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree>
  void update_linearized_system(
      const TargetPointCloud&,
      const SourcePointCloud&,
      const TargetTree&,
      const Eigen::Isometry3d& T,
      Eigen::Matrix<double, 6, 6>* H,
      Eigen::Matrix<double, 6, 1>* b,
      double* e) const {
    if (!H || !b || !e) {
      return;
    }

    // small_gicp uses right-multiplicative se(3) perturbations ordered
    // [rx, ry, rz, tx, ty, tz]. Soft rotation prior first, exact mask last
    // (so a fixed axis stays fixed even where the prior touches it).
    if ((rotation_prior_info.array() > 0.0).any()) {
      const Eigen::Vector3d r = so3LogVector(rotation_prior_R.transpose() * T.linear());
      const Eigen::Matrix3d W = rotation_prior_info.asDiagonal();
      H->template block<3, 3>(0, 0) += W;
      b->template head<3>() += W * r;
      *e += 0.5 * r.transpose() * W * r;
    }

    // [REVIEW FIX 2026-07-08 P2/P3] EXACT delta masking, not soft damping.
    // The previous dof_lambda=1e9 diagonal boost only shrank the masked
    // increments; there was no residual back to the initial guess and no
    // exact zeroing, so the "fixed" axes could still creep across LM
    // iterations — and 3dof was not a hard yaw lock even though the caller
    // documents it as one. Zeroing the masked rows/cols of H and entries of
    // b (with the diagonal pinned for conditioning) makes each LM step's
    // delta EXACTLY zero on those axes: with right-multiplicative
    // perturbations the masked DoF then hold the values of init_T for the
    // whole solve, which is precisely the documented "fixed to the IMU
    // prior" contract.
    if ((dof_mask.array() < 1.0).any()) {
      const double pin = std::max(1.0, H->diagonal().cwiseAbs().maxCoeff());
      for (int axis = 0; axis < 6; ++axis) {
        if (dof_mask(axis) >= 1.0) continue;
        H->row(axis).setZero();
        H->col(axis).setZero();
        (*H)(axis, axis) = pin;
        (*b)(axis) = 0.0;
      }
    }
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  void update_error(
      const TargetPointCloud&,
      const SourcePointCloud&,
      const Eigen::Isometry3d& T,
      double* e) const {
    if (!e || !(rotation_prior_info.array() > 0.0).any()) {
      return;
    }
    const Eigen::Vector3d r = so3LogVector(rotation_prior_R.transpose() * T.linear());
    *e += 0.5 * r.transpose() * rotation_prior_info.asDiagonal() * r;
  }

  Eigen::Array<double, 6, 1> dof_mask;
  Eigen::Matrix3d rotation_prior_R;
  Eigen::Vector3d rotation_prior_info;
};

struct PriorAwareLevenbergMarquardtOptimizer {
  PriorAwareLevenbergMarquardtOptimizer()
  : verbose(false),
    max_iterations(20),
    max_inner_iterations(10),
    init_lambda(1e-3),
    lambda_factor(10.0) {}

  template <
      typename TargetPointCloud,
      typename SourcePointCloud,
      typename TargetTree,
      typename CorrespondenceRejector,
      typename TerminationCriteria,
      typename Reduction,
      typename Factor,
      typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
      const TargetPointCloud& target,
      const SourcePointCloud& source,
      const TargetTree& target_tree,
      const CorrespondenceRejector& rejector,
      const TerminationCriteria& criteria,
      Reduction& reduction,
      const Eigen::Isometry3d& init_T,
      std::vector<Factor>& factors,
      GeneralFactor& general_factor) const {
    if (verbose) {
      std::cout << "--- small_gicp prior-aware LM optimization ---" << std::endl;
    }

    double lambda = init_lambda;
    small_gicp::RegistrationResult result(init_T);
    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      auto [H, b, e] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(
          target, source, target_tree, result.T_target_source, &H, &b, &e);

      bool success = false;
      for (int j = 0; j < max_inner_iterations; ++j) {
        const Eigen::Matrix<double, 6, 1> delta =
            (H + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);
        const Eigen::Isometry3d new_T = result.T_target_source * small_gicp::se3_exp(delta);

        double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &new_e);

        if (verbose) {
          std::cout << "iter=" << i << " inner=" << j
                    << " e=" << e << " new_e=" << new_e
                    << " lambda=" << lambda
                    << " dt=" << delta.tail<3>().norm()
                    << " dr=" << delta.head<3>().norm()
                    << std::endl;
        }

        if (new_e <= e) {
          result.converged = criteria.converged(delta);
          result.T_target_source = new_T;
          lambda /= lambda_factor;
          success = true;
          e = new_e;
          break;
        }

        lambda *= lambda_factor;
      }

      result.iterations = static_cast<size_t>(i);
      result.H = H;
      result.b = b;
      result.error = e;

      if (!success) {
        break;
      }
    }

    // [REVIEW FIX 2026-07-08 P3] Re-linearize at the FINAL pose so
    // result.H / result.b / result.error describe the accepted output, not
    // the linearization point BEFORE the last accepted step (with bounded
    // non-converged accepts, that step is exactly where it may not be tiny).
    //
    // [REVIEW FIX 2026-07-08 P1] Deliberately WITHOUT the general factor:
    // result.H must be the raw LiDAR-geometry Hessian. The ground-vehicle
    // factor pins the masked axes (exact delta masking) and the rotation
    // prior adds its information matrix — with the shipped 4dof default that
    // artificial roll/pitch stiffness would dominate lambda_max, distort the
    // relFloor* degeneracy floors, and make "well-constrained" axes reflect
    // the prior/DoF mask instead of map evidence in hessian_condition, the
    // eigen-projection, and yaw_marginal_stiffness. Likewise result.error
    // stays pure point residual, so fitness (= error / num_inliers) measures
    // map agreement, not prior disagreement. The augmented system exists only
    // inside the LM iterations above.
    {
      auto [H_final, b_final, e_final] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      result.H = H_final;
      result.b = b_final;
      result.error = e_final;
    }

    result.num_inliers = static_cast<size_t>(std::count_if(
        factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); }));
    return result;
  }

  bool verbose;
  int max_iterations;
  int max_inner_iterations;
  double init_lambda;
  double lambda_factor;
};

template <typename PointSource, typename PointTarget>
class SmallGicpBackend {
 public:
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;
  using TargetTree = small_gicp::KdTree<PointCloudTarget>;
  using CovarianceVector = std::vector<Eigen::Matrix4d>;

  struct PreparedTarget {
    PointCloudTargetPtr cloud;
    std::shared_ptr<TargetTree> tree;
    std::shared_ptr<CovarianceVector> covariances;
  };
  using PreparedTargetPtr = std::shared_ptr<PreparedTarget>;

  SmallGicpBackend()
  : num_threads_(1),
    k_correspondences_(20),
    max_corr_dist_(1.0),
    max_iterations_(20),
    transformation_epsilon_(1e-3),
    rotation_epsilon_(0.1 * 3.14159265358979323846 / 180.0),
    debug_print_(false),
    has_rotation_prior_(false),
    target_covs_(std::make_shared<CovarianceVector>()),
    converged_(false),
    final_transformation_(Eigen::Matrix4f::Identity()),
    final_fitness_(std::numeric_limits<double>::infinity()),
    final_error_(std::numeric_limits<double>::infinity()),
    num_correspondences(0) {
    dof_mask_.setOnes();
    rotation_prior_R_.setIdentity();
    rotation_prior_info_.setZero();
  }

  void setNumThreads(int n) { num_threads_ = std::max(1, n); }
  void setCorrespondenceRandomness(int k) { k_correspondences_ = std::max(5, k); }
  void setMaxCorrespondenceDistance(double corr) { max_corr_dist_ = std::max(0.0, corr); }
  void setMaximumIterations(int iter) { max_iterations_ = std::max(1, iter); }
  void setTransformationEpsilon(double eps) { transformation_epsilon_ = std::max(0.0, eps); }
  void setRotationEpsilon(double eps) { rotation_epsilon_ = std::max(0.0, eps); }
  void setDebugPrint(bool enabled) { debug_print_ = enabled; }

  void setInputTarget(const PointCloudTargetConstPtr& cloud) {
    if (!prepared_target_ && target_ == cloud && target_tree_) {
      return;
    }
    prepared_target_.reset();
    target_ = cloud;
    target_covs_ = std::make_shared<CovarianceVector>();
    if (target_ && !target_->empty()) {
      target_tree_ = std::make_shared<TargetTree>(
          target_, small_gicp::KdTreeBuilderOMP(num_threads_));
    } else {
      target_tree_.reset();
    }
  }

  PreparedTargetPtr preprocessInputTarget(
      const PointCloudTarget& cloud,
      double downsampling_resolution,
      int num_neighbors = 10,
      int num_threads = 0) const {
    const int threads = num_threads > 0 ? num_threads : num_threads_;
    PointCloudTargetPtr downsampled;
    if (downsampling_resolution > 0.0) {
      downsampled =
          small_gicp::voxelgrid_sampling_omp<PointCloudTarget, PointCloudTarget>(
              cloud, downsampling_resolution, threads);
    } else {
      downsampled = std::make_shared<PointCloudTarget>(cloud);
    }
    return prepareInputTarget(downsampled, {}, num_neighbors, threads);
  }

  PreparedTargetPtr prepareInputTarget(
      const PointCloudTargetPtr& cloud,
      CovarianceVector covariances = {},
      int num_neighbors = 10,
      int num_threads = 0) const {
    if (!cloud || cloud->empty()) {
      return nullptr;
    }
    if (!covariances.empty() && covariances.size() != cloud->size()) {
      return nullptr;
    }

    const int threads = num_threads > 0 ? num_threads : num_threads_;
    auto prepared = std::make_shared<PreparedTarget>();
    prepared->cloud = cloud;
    prepared->tree = std::make_shared<TargetTree>(
        cloud, small_gicp::KdTreeBuilderOMP(threads));
    prepared->covariances =
        std::make_shared<CovarianceVector>(std::move(covariances));
    if (prepared->covariances->empty()) {
      small_gicp::PointCloudProxy<PointTarget> proxy(
          *prepared->cloud, *prepared->covariances);
      small_gicp::estimate_covariances_omp(
          proxy, *prepared->tree, std::max(5, num_neighbors), threads);
    }
    if (prepared->covariances->size() != prepared->cloud->size()) {
      return nullptr;
    }
    return prepared;
  }

  void setInputTarget(const PreparedTargetPtr& prepared) {
    if (!prepared || !prepared->cloud || prepared->cloud->empty() ||
        !prepared->tree || !prepared->covariances ||
        prepared->covariances->size() != prepared->cloud->size()) {
      prepared_target_.reset();
      target_.reset();
      target_tree_.reset();
      target_covs_ = std::make_shared<CovarianceVector>();
      return;
    }
    prepared_target_ = prepared;
    target_ = prepared_target_->cloud;
    target_tree_ = prepared_target_->tree;
    target_covs_ = prepared_target_->covariances;
  }

  void setInputSource(const PointCloudSourceConstPtr& cloud) {
    input_ = cloud;
    source_covs_.clear();
    if (input_ && !input_->empty()) {
      source_tree_ = std::make_shared<small_gicp::KdTree<PointCloudSource>>(
          input_, small_gicp::KdTreeBuilderOMP(num_threads_));
    } else {
      source_tree_.reset();
    }
  }

  bool calculateTargetCovariances() {
    if (!target_ || target_->empty() || !target_tree_ || !target_covs_) {
      return false;
    }
    target_covs_->clear();
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, *target_covs_);
    small_gicp::estimate_covariances_omp(
        target_proxy, *target_tree_, k_correspondences_, num_threads_);
    return target_covs_->size() == target_->size();
  }

  void setDoFMask(bool fix_roll, bool fix_pitch, bool fix_yaw) {
    dof_mask_.setOnes();
    if (fix_roll) dof_mask_(0) = 0.0;
    if (fix_pitch) dof_mask_(1) = 0.0;
    if (fix_yaw) dof_mask_(2) = 0.0;
  }

  void setRotationPrior(const Eigen::Matrix3d& R_target, const Eigen::Vector3d& info) {
    has_rotation_prior_ = true;
    rotation_prior_R_ = R_target;
    rotation_prior_info_ = info.cwiseMax(Eigen::Vector3d::Zero());
  }

  void clearRotationPrior() {
    has_rotation_prior_ = false;
    rotation_prior_info_.setZero();
  }

  // Evaluate the legacy-compatible Euclidean fitness and small_gicp
  // correspondence support at an ARBITRARY pose in the same solution space
  // align() used. Needed because degeneracy projection /
  // the yaw veto can modify the applied pose AFTER the solve: gating that
  // modified pose on the raw optimizer fitness would validate a pose nobody
  // is applying. Requires a prior align() on the same source/target (reuses
  // its covariances); returns false when evaluation is impossible.
  bool evaluateFitnessAt(
      const Eigen::Matrix4f& T,
      double* fitness,
      int* inliers,
      double* optimizer_error = nullptr) {
    if (!target_ || target_->empty() || !target_tree_ || !target_covs_ ||
        !input_ || input_->empty() ||
        source_covs_.size() != input_->size() || target_covs_->size() != target_->size() ||
        !T.allFinite()) {
      return false;
    }
    small_gicp::PointCloudProxy<PointSource> source_proxy(*input_, source_covs_);
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, *target_covs_);
    std::vector<small_gicp::GICPFactor> factors(input_->size());
    small_gicp::ParallelReductionOMP reduction;
    reduction.num_threads = num_threads_;
    small_gicp::DistanceRejector rejector;
    rejector.max_dist_sq = max_corr_dist_ * max_corr_dist_;
    const auto [H, b, e] = reduction.linearize(
        target_proxy, source_proxy, *target_tree_, rejector,
        Eigen::Isometry3d(T.cast<double>()), factors);
    (void)H;
    (void)b;
    const size_t n = static_cast<size_t>(std::count_if(
        factors.begin(), factors.end(), [](const auto& f) { return f.inlier(); }));
    if (inliers) *inliers = static_cast<int>(n);
    if (optimizer_error) {
      *optimizer_error =
          std::isfinite(e) ? e : std::numeric_limits<double>::infinity();
    }
    const double f =
        legacyEuclideanFitness(Eigen::Isometry3d(T.cast<double>()));
    if (fitness) {
      *fitness = std::isfinite(f) ? f : std::numeric_limits<double>::infinity();
    }
    return n > 0 && std::isfinite(e) && std::isfinite(f);
  }

  void align(PointCloudSource& output, const Eigen::Matrix4f& guess) {
    converged_ = false;
    final_transformation_ = guess;
    final_fitness_ = std::numeric_limits<double>::infinity();
    final_error_ = std::numeric_limits<double>::infinity();
    num_correspondences = 0;
    result_ = small_gicp::RegistrationResult(Eigen::Isometry3d(guess.cast<double>()));

    if (!target_ || target_->empty() || !target_tree_ || !target_covs_ ||
        !input_ || input_->empty()) {
      output.clear();
      return;
    }

    small_gicp::PointCloudProxy<PointSource> source_proxy(*input_, source_covs_);
    small_gicp::PointCloudProxy<PointTarget> target_proxy(*target_, *target_covs_);

    if (!source_tree_) {
      source_tree_ = std::make_shared<small_gicp::KdTree<PointCloudSource>>(
          input_, small_gicp::KdTreeBuilderOMP(num_threads_));
    }
    if (source_covs_.size() != input_->size()) {
      small_gicp::estimate_covariances_omp(
          source_proxy, *source_tree_, k_correspondences_, num_threads_);
    }
    if (target_covs_->size() != target_->size()) {
      small_gicp::estimate_covariances_omp(
          target_proxy, *target_tree_, k_correspondences_, num_threads_);
    }

    GroundVehicleGeneralFactor general_factor;
    general_factor.dof_mask = dof_mask_;
    if (has_rotation_prior_) {
      general_factor.rotation_prior_R = rotation_prior_R_;
      general_factor.rotation_prior_info = rotation_prior_info_;
    }

    small_gicp::Registration<
        small_gicp::GICPFactor,
        small_gicp::ParallelReductionOMP,
        GroundVehicleGeneralFactor,
        small_gicp::DistanceRejector,
        PriorAwareLevenbergMarquardtOptimizer>
        registration;
    registration.criteria.rotation_eps = rotation_epsilon_;
    registration.criteria.translation_eps = transformation_epsilon_;
    registration.reduction.num_threads = num_threads_;
    registration.rejector.max_dist_sq = max_corr_dist_ * max_corr_dist_;
    registration.optimizer.verbose = debug_print_;
    registration.optimizer.max_iterations = max_iterations_;
    registration.general_factor = general_factor;

    result_ = registration.align(
        target_proxy, source_proxy, *target_tree_, Eigen::Isometry3d(guess.cast<double>()));

    converged_ = result_.converged;
    final_transformation_ = result_.T_target_source.matrix().cast<float>();
    final_error_ = result_.error;
    num_correspondences = static_cast<int>(result_.num_inliers);
    // Preserve nano_gicp's public fitness contract: mean squared Euclidean
    // nearest-neighbor distance over every source point, with no
    // correspondence-distance cutoff. The localization thresholds and
    // rolling baselines are calibrated in these m^2 units. small_gicp's
    // Mahalanobis inlier energy remains available through getFinalError().
    final_fitness_ = legacyEuclideanFitness(result_.T_target_source);
    if (!std::isfinite(final_error_) || !std::isfinite(final_fitness_)) {
      final_fitness_ = std::numeric_limits<double>::infinity();
    }

    // The localizer consumes only final_transformation_. Avoid materializing
    // a second transformed scan on every frame; retaining the output argument
    // keeps the old matcher-shaped call surface while making its unused state
    // explicit.
    output.clear();
  }

  double getFitnessScore(double = std::numeric_limits<double>::max()) const {
    return final_fitness_;
  }

  double getFitnessScoreAtFinal(double = std::numeric_limits<double>::max()) const {
    return final_fitness_;
  }

  double getFinalError() const { return final_error_; }
  bool hasConverged() const { return converged_; }
  const Eigen::Matrix<double, 6, 6>& getFinalHessian() const { return result_.H; }
  Eigen::Matrix4f getFinalTransformation() const { return final_transformation_; }
  const small_gicp::RegistrationResult& getRegistrationResult() const { return result_; }

  int num_correspondences;

 private:
  double legacyEuclideanFitness(const Eigen::Isometry3d& T) const {
    if (!target_tree_ || !input_ || input_->empty() ||
        !T.matrix().allFinite()) {
      return std::numeric_limits<double>::infinity();
    }

    double squared_distance_sum = 0.0;
    for (const auto& point : input_->points) {
      const Eigen::Vector4d source_point(
          static_cast<double>(point.x),
          static_cast<double>(point.y),
          static_cast<double>(point.z),
          1.0);
      if (!source_point.allFinite()) {
        return std::numeric_limits<double>::infinity();
      }

      size_t target_index = 0;
      double squared_distance = std::numeric_limits<double>::infinity();
      const size_t found = target_tree_->nearest_neighbor_search(
          T * source_point, &target_index, &squared_distance);
      if (found == 0 || !std::isfinite(squared_distance) ||
          squared_distance < 0.0) {
        return std::numeric_limits<double>::infinity();
      }
      squared_distance_sum += squared_distance;
      if (!std::isfinite(squared_distance_sum)) {
        return std::numeric_limits<double>::infinity();
      }
    }
    return squared_distance_sum / static_cast<double>(input_->size());
  }

  int num_threads_;
  int k_correspondences_;
  double max_corr_dist_;
  int max_iterations_;
  double transformation_epsilon_;
  double rotation_epsilon_;
  bool debug_print_;

  PointCloudSourceConstPtr input_;
  PointCloudTargetConstPtr target_;
  PreparedTargetPtr prepared_target_;
  std::shared_ptr<small_gicp::KdTree<PointCloudSource>> source_tree_;
  std::shared_ptr<TargetTree> target_tree_;
  std::vector<Eigen::Matrix4d> source_covs_;
  std::shared_ptr<CovarianceVector> target_covs_;

  Eigen::Array<double, 6, 1> dof_mask_;
  bool has_rotation_prior_;
  Eigen::Matrix3d rotation_prior_R_;
  Eigen::Vector3d rotation_prior_info_;

  bool converged_;
  Eigen::Matrix4f final_transformation_;
  double final_fitness_;
  double final_error_;
  small_gicp::RegistrationResult result_;
};

}  // namespace gicp_plusplus

#endif  // GICP_PLUSPLUS_SMALL_GICP_BACKEND_HPP
