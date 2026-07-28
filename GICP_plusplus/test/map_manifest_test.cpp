#include "gicp_localization/map_manifest.hpp"

#include <cmath>

#include <gtest/gtest.h>

TEST(MapManifest, ParsesCanonicalOrigin) {
  const auto origin =
      gicp_localization::parseEnuOrigin("37.87150000,-122.27300000,52.125");
  EXPECT_NEAR(origin.latitude_deg, 37.8715, 1.0e-12);
  EXPECT_NEAR(origin.longitude_deg, -122.273, 1.0e-12);
  EXPECT_NEAR(origin.altitude_m, 52.125, 1.0e-12);
}

TEST(MapManifest, RejectsMalformedOrOutOfRangeOrigin) {
  EXPECT_THROW(
      gicp_localization::parseEnuOrigin("37.0,-122.0"),
      std::runtime_error);
  EXPECT_THROW(
      gicp_localization::parseEnuOrigin("91.0,-122.0,10.0"),
      std::runtime_error);
  EXPECT_THROW(
      gicp_localization::parseEnuOrigin("nan,-122.0,10.0"),
      std::runtime_error);
}

TEST(MapManifest, MeasuresDatumDifferenceInMeters) {
  const auto a = gicp_localization::parseEnuOrigin("37.0,-122.0,10.0");
  const auto b = gicp_localization::parseEnuOrigin(
      "37.00000090,-122.0,10.0");
  EXPECT_NEAR(gicp_localization::enuOriginDistanceMeters(a, b), 0.10, 0.01);
}

TEST(MapManifest, RequiresCanonicalEnuFrameAndSurveyedDatum) {
  const auto manifest = gicp_localization::parseMapManifest(
      YAML::Load("frame: enu\nenu_origin: 37.0,-122.0,10.0\n"));
  EXPECT_EQ(manifest.frame, "enu");
  EXPECT_DOUBLE_EQ(manifest.enu_origin.altitude_m, 10.0);

  EXPECT_THROW(
      gicp_localization::parseMapManifest(
          YAML::Load("frame: world\nenu_origin: 37.0,-122.0,10.0\n")),
      std::runtime_error);
  EXPECT_THROW(
      gicp_localization::parseMapManifest(
          YAML::Load("frame: enu\nenu_origin: UNSPECIFIED\n")),
      std::runtime_error);
}
