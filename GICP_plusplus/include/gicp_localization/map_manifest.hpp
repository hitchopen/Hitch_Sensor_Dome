#pragma once

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace gicp_localization {

struct EnuOrigin {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

struct MapManifest {
  std::string frame;
  EnuOrigin enu_origin;
};

inline EnuOrigin parseEnuOrigin(const std::string& text) {
  std::string normalized = text;
  for (char& c : normalized) {
    if (c == ',') c = ' ';
  }

  std::istringstream stream(normalized);
  EnuOrigin origin;
  std::string trailing;
  if (!(stream >> origin.latitude_deg >> origin.longitude_deg >>
        origin.altitude_m) ||
      (stream >> trailing)) {
    throw std::runtime_error(
        "ENU origin must contain exactly 'lat_deg,lon_deg,alt_m': " + text);
  }
  if (!std::isfinite(origin.latitude_deg) ||
      !std::isfinite(origin.longitude_deg) ||
      !std::isfinite(origin.altitude_m) ||
      origin.latitude_deg < -90.0 || origin.latitude_deg > 90.0 ||
      origin.longitude_deg < -180.0 || origin.longitude_deg > 180.0) {
    throw std::runtime_error("ENU origin is non-finite or out of range: " + text);
  }
  return origin;
}

inline MapManifest parseMapManifest(const YAML::Node& root) {
  if (!root["frame"] || !root["enu_origin"]) {
    throw std::runtime_error(
        "map manifest must define both 'frame' and 'enu_origin'");
  }

  MapManifest manifest;
  manifest.frame = root["frame"].as<std::string>();
  if (manifest.frame != "enu") {
    throw std::runtime_error(
        "map manifest frame must be 'enu', got '" + manifest.frame + "'");
  }

  const std::string origin_text = root["enu_origin"].as<std::string>();
  if (origin_text == "UNSPECIFIED") {
    throw std::runtime_error(
        "map manifest ENU origin is UNSPECIFIED; re-export with a surveyed datum");
  }
  manifest.enu_origin = parseEnuOrigin(origin_text);
  return manifest;
}

inline MapManifest loadMapManifest(const std::string& path) {
  return parseMapManifest(YAML::LoadFile(path));
}

inline double enuOriginDistanceMeters(
    const EnuOrigin& lhs, const EnuOrigin& rhs) {
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr double kDegToRad = 0.017453292519943295;
  const double mean_lat =
      0.5 * (lhs.latitude_deg + rhs.latitude_deg) * kDegToRad;
  const double north =
      (lhs.latitude_deg - rhs.latitude_deg) * kDegToRad * kEarthRadiusM;
  const double east =
      (lhs.longitude_deg - rhs.longitude_deg) * kDegToRad *
      kEarthRadiusM * std::cos(mean_lat);
  const double up = lhs.altitude_m - rhs.altitude_m;
  return std::sqrt(north * north + east * east + up * up);
}

}  // namespace gicp_localization
