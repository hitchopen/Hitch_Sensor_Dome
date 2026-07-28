#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "dlio/dlio.h"
#include "gicp_plusplus/small_gicp_backend.hpp"

namespace {

pcl::PointCloud<PointType>::Ptr makeCloud() {
  auto cloud = std::make_shared<pcl::PointCloud<PointType>>();
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 5; ++y) {
      PointType point;
      point.x = 0.4f * static_cast<float>(x);
      point.y = 0.3f * static_cast<float>(y);
      point.z = 0.1f * static_cast<float>((x * x + 3 * y) % 7);
      point.intensity = static_cast<float>(x + y);
      cloud->push_back(point);
    }
  }
  return cloud;
}

TEST(SmallGicpBackend, EmptyInputFailsClosed) {
  gicp_plusplus::SmallGicpBackend<PointType, PointType> backend;
  pcl::PointCloud<PointType> output;
  output.push_back(PointType{});

  backend.align(output, Eigen::Matrix4f::Identity());

  EXPECT_TRUE(output.empty());
  EXPECT_FALSE(backend.hasConverged());
  EXPECT_TRUE(std::isinf(backend.getFitnessScore()));
  EXPECT_EQ(backend.num_correspondences, 0);
}

TEST(SmallGicpBackend, IdenticalCloudProducesFiniteFinalPoseEvidence) {
  gicp_plusplus::SmallGicpBackend<PointType, PointType> backend;
  backend.setNumThreads(1);
  backend.setCorrespondenceRandomness(10);
  backend.setMaxCorrespondenceDistance(2.0);
  backend.setMaximumIterations(20);
  backend.setTransformationEpsilon(1e-4);
  backend.setRotationEpsilon(1e-4);

  auto target = makeCloud();
  auto source = makeCloud();
  backend.setInputTarget(target);
  ASSERT_TRUE(backend.calculateTargetCovariances());
  backend.setInputSource(source);

  pcl::PointCloud<PointType> output;
  backend.align(output, Eigen::Matrix4f::Identity());

  EXPECT_TRUE(output.empty());
  EXPECT_TRUE(backend.hasConverged());
  EXPECT_GT(backend.num_correspondences, 0);
  EXPECT_TRUE(std::isfinite(backend.getFitnessScore()));
  EXPECT_TRUE(backend.getFinalHessian().allFinite());
  EXPECT_TRUE(backend.getFinalTransformation().allFinite());
  EXPECT_LT(
      (backend.getFinalTransformation() - Eigen::Matrix4f::Identity()).norm(),
      1e-3f);

  double fitness = std::numeric_limits<double>::infinity();
  int inliers = 0;
  EXPECT_TRUE(backend.evaluateFitnessAt(
      Eigen::Matrix4f::Identity(), &fitness, &inliers));
  EXPECT_TRUE(std::isfinite(fitness));
  EXPECT_GT(inliers, 0);
}

TEST(SmallGicpBackend, PreparedCropReusesPointCovariancePairs) {
  using Backend =
      gicp_plusplus::SmallGicpBackend<PointType, PointType>;
  Backend backend;
  backend.setNumThreads(1);
  backend.setCorrespondenceRandomness(10);
  backend.setMaxCorrespondenceDistance(2.0);

  auto full = backend.preprocessInputTarget(*makeCloud(), 0.0, 10, 1);
  ASSERT_NE(full, nullptr);
  ASSERT_EQ(full->cloud->size(), full->covariances->size());

  auto crop = std::make_shared<pcl::PointCloud<PointType>>();
  Backend::CovarianceVector crop_covariances;
  for (size_t i = 0; i < 15; ++i) {
    crop->push_back(full->cloud->points[i]);
    crop_covariances.push_back(full->covariances->at(i));
  }

  auto prepared =
      backend.prepareInputTarget(crop, crop_covariances, 10, 1);
  ASSERT_NE(prepared, nullptr);
  ASSERT_EQ(prepared->cloud->size(), prepared->covariances->size());
  for (size_t i = 0; i < crop->size(); ++i) {
    EXPECT_TRUE(
        prepared->covariances->at(i).isApprox(crop_covariances[i], 0.0));
  }

  backend.setInputTarget(prepared);
  backend.setInputSource(crop);
  pcl::PointCloud<PointType> output;
  backend.align(output, Eigen::Matrix4f::Identity());

  EXPECT_TRUE(output.empty());
  EXPECT_GT(backend.num_correspondences, 0);
  EXPECT_TRUE(std::isfinite(backend.getFitnessScore()));
}

TEST(SmallGicpBackend, PreparedTargetRejectsMismatchedCovarianceCount) {
  using Backend =
      gicp_plusplus::SmallGicpBackend<PointType, PointType>;
  Backend backend;
  auto cloud = makeCloud();
  Backend::CovarianceVector covariances(
      1, Eigen::Matrix4d::Identity());

  EXPECT_EQ(
      backend.prepareInputTarget(cloud, std::move(covariances), 10, 1),
      nullptr);
}

TEST(SmallGicpBackend, FitnessIncludesPointsOutsideCorrespondenceRadius) {
  gicp_plusplus::SmallGicpBackend<PointType, PointType> backend;
  backend.setNumThreads(1);
  backend.setCorrespondenceRandomness(10);
  backend.setMaxCorrespondenceDistance(2.0);
  backend.setMaximumIterations(10);

  auto target = makeCloud();
  auto source = makeCloud();
  PointType unmatched;
  unmatched.x = 10.0F;
  unmatched.y = 0.0F;
  unmatched.z = 0.0F;
  source->push_back(unmatched);

  backend.setInputTarget(target);
  ASSERT_TRUE(backend.calculateTargetCovariances());
  backend.setInputSource(source);
  pcl::PointCloud<PointType> output;
  backend.align(output, Eigen::Matrix4f::Identity());
  EXPECT_EQ(backend.num_correspondences, static_cast<int>(target->size()));
  EXPECT_GT(backend.getFitnessScore(), 2.0);

  double fitness = std::numeric_limits<double>::infinity();
  double optimizer_error = std::numeric_limits<double>::infinity();
  int inliers = 0;
  ASSERT_TRUE(backend.evaluateFitnessAt(
      Eigen::Matrix4f::Identity(), &fitness, &inliers, &optimizer_error));
  EXPECT_EQ(inliers, static_cast<int>(target->size()));
  EXPECT_NEAR(optimizer_error, 0.0, 1.0e-9);
  EXPECT_GT(fitness, 2.0);
}

}  // namespace
