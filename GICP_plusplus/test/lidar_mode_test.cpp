#include "gicp_localization/lidar_mode.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using gicp_localization::LidarMode;

TEST(LidarMode, ParsesSupportedModes) {
  EXPECT_EQ(
      gicp_localization::parseLidarMode("front_only"),
      LidarMode::FrontOnly);
  EXPECT_EQ(
      gicp_localization::parseLidarMode("three_lidar"),
      LidarMode::ThreeLidar);
  EXPECT_FALSE(
      gicp_localization::lidarModeUsesAuxiliaries(LidarMode::FrontOnly));
  EXPECT_TRUE(
      gicp_localization::lidarModeUsesAuxiliaries(LidarMode::ThreeLidar));
}

TEST(LidarMode, RejectsUnknownMode) {
  EXPECT_THROW(
      gicp_localization::parseLidarMode("all"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::parseLidarMode(""),
      std::invalid_argument);
}

TEST(LidarMode, EmptyDefaultPreservesLegacyConcatSelection) {
  EXPECT_STREQ(gicp_localization::kDefaultLidarModeParameter, "");

  const auto disabled = gicp_localization::resolveLidarModeParameter(
      gicp_localization::kDefaultLidarModeParameter, false);
  EXPECT_EQ(disabled.mode, LidarMode::FrontOnly);
  EXPECT_TRUE(disabled.used_legacy_fallback);

  const auto enabled = gicp_localization::resolveLidarModeParameter(
      gicp_localization::kDefaultLidarModeParameter, true);
  EXPECT_EQ(enabled.mode, LidarMode::ThreeLidar);
  EXPECT_TRUE(enabled.used_legacy_fallback);
}

TEST(LidarMode, ExplicitModeOverridesLegacyConcatSelection) {
  const auto front =
      gicp_localization::resolveLidarModeParameter("front_only", true);
  EXPECT_EQ(front.mode, LidarMode::FrontOnly);
  EXPECT_FALSE(front.used_legacy_fallback);

  const auto three =
      gicp_localization::resolveLidarModeParameter("three_lidar", false);
  EXPECT_EQ(three.mode, LidarMode::ThreeLidar);
  EXPECT_FALSE(three.used_legacy_fallback);
}

TEST(LidarMode, ResolvedTopicsRejectPrimaryOrAuxiliaryAliases) {
  EXPECT_NO_THROW(gicp_localization::validateResolvedLidarTopics(
      "/robin_w_front/points",
      {"/robin_w_rear_left/points", "/robin_w_rear_right/points"}));
  EXPECT_THROW(
      gicp_localization::validateResolvedLidarTopics(
          "/robin_w_front/points",
          {"/robin_w_front/points", "/robin_w_rear_right/points"}),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateResolvedLidarTopics(
          "/robin_w_front/points",
          {"/robin_w_rear_left/points", "/robin_w_rear_left/points"}),
      std::invalid_argument);
}

TEST(LidarMode, FrontOnlyDoesNotRequireAuxiliaryConfiguration) {
  EXPECT_NO_THROW(gicp_localization::validateLidarModeConfiguration(
      LidarMode::FrontOnly, {}, {}, "lidar_front_link"));
}

TEST(LidarMode, ThreeLidarAcceptsExactlyTwoDistinctAuxiliaries) {
  EXPECT_NO_THROW(gicp_localization::validateLidarModeConfiguration(
      LidarMode::ThreeLidar,
      {"/robin_w_rear_left/points", "/robin_w_rear_right/points"},
      {"lidar_rear_left_link", "lidar_rear_right_link"},
      "lidar_front_link"));
}

TEST(LidarMode, ThreeLidarRejectsIncompleteOrAmbiguousTopology) {
  const std::vector<std::string> topics{
      "/robin_w_rear_left/points", "/robin_w_rear_right/points"};
  const std::vector<std::string> frames{
      "lidar_rear_left_link", "lidar_rear_right_link"};

  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, {topics[0]}, {frames[0]},
          "lidar_front_link"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, {topics[0], topics[0]}, frames,
          "lidar_front_link"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, topics, {frames[0], frames[0]},
          "lidar_front_link"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, {topics[0], ""}, frames,
          "lidar_front_link"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, topics,
          {"lidar_front_link", frames[1]}, "lidar_front_link"),
      std::invalid_argument);
  EXPECT_THROW(
      gicp_localization::validateLidarModeConfiguration(
          LidarMode::ThreeLidar, topics, frames, ""),
      std::invalid_argument);
}

}  // namespace
