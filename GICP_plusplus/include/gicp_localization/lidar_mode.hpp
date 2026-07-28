#ifndef GICP_LOCALIZATION_LIDAR_MODE_HPP
#define GICP_LOCALIZATION_LIDAR_MODE_HPP

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace gicp_localization {

enum class LidarMode {
  FrontOnly,
  ThreeLidar,
};

inline constexpr char kDefaultLidarModeParameter[] = "";

struct LidarModeResolution {
  LidarMode mode;
  bool used_legacy_fallback;
};

inline const char* lidarModeName(LidarMode mode) {
  switch (mode) {
    case LidarMode::FrontOnly:
      return "front_only";
    case LidarMode::ThreeLidar:
      return "three_lidar";
  }
  return "unknown";
}

inline LidarMode parseLidarMode(const std::string& value) {
  if (value == "front_only") {
    return LidarMode::FrontOnly;
  }
  if (value == "three_lidar") {
    return LidarMode::ThreeLidar;
  }
  throw std::invalid_argument(
      "localization/lidar_mode must be 'front_only' or 'three_lidar'");
}

inline bool lidarModeUsesAuxiliaries(LidarMode mode) {
  return mode == LidarMode::ThreeLidar;
}

inline LidarModeResolution resolveLidarModeParameter(
    const std::string& configured_mode,
    bool legacy_concat_enabled) {
  if (configured_mode.empty()) {
    return {
        legacy_concat_enabled ? LidarMode::ThreeLidar : LidarMode::FrontOnly,
        true};
  }
  return {parseLidarMode(configured_mode), false};
}

inline void validateResolvedLidarTopics(
    const std::string& primary_topic,
    const std::vector<std::string>& auxiliary_topics) {
  if (primary_topic.empty() || auxiliary_topics.size() != 2 ||
      auxiliary_topics[0].empty() || auxiliary_topics[1].empty()) {
    throw std::invalid_argument(
        "three_lidar mode requires one primary and two resolved auxiliary "
        "topics");
  }
  if (auxiliary_topics[0] == auxiliary_topics[1]) {
    throw std::invalid_argument(
        "three_lidar mode resolved auxiliary topics must be distinct");
  }
  if (auxiliary_topics[0] == primary_topic ||
      auxiliary_topics[1] == primary_topic) {
    throw std::invalid_argument(
        "three_lidar mode resolved auxiliary topics must differ from the "
        "primary topic");
  }
}

inline void validateLidarModeConfiguration(
    LidarMode mode,
    const std::vector<std::string>& auxiliary_topics,
    const std::vector<std::string>& auxiliary_frames,
    const std::string& primary_frame) {
  if (mode == LidarMode::FrontOnly) {
    return;
  }

  if (primary_frame.empty()) {
    throw std::invalid_argument(
        "three_lidar mode requires a non-empty primary frame");
  }

  if (auxiliary_topics.size() != 2 || auxiliary_frames.size() != 2) {
    throw std::invalid_argument(
        "three_lidar mode requires exactly two auxiliary topics and two "
        "auxiliary frames");
  }

  for (std::size_t i = 0; i < 2; ++i) {
    if (auxiliary_topics[i].empty() || auxiliary_frames[i].empty()) {
      throw std::invalid_argument(
          "three_lidar mode does not allow empty auxiliary topics or frames");
    }
  }

  if (auxiliary_topics[0] == auxiliary_topics[1]) {
    throw std::invalid_argument(
        "three_lidar mode requires two distinct auxiliary topics");
  }
  if (auxiliary_frames[0] == auxiliary_frames[1]) {
    throw std::invalid_argument(
        "three_lidar mode requires two distinct auxiliary frames");
  }
  if (auxiliary_frames[0] == primary_frame ||
      auxiliary_frames[1] == primary_frame) {
    throw std::invalid_argument(
        "three_lidar mode auxiliary frames must differ from the primary "
        "frame");
  }
}

}  // namespace gicp_localization

#endif  // GICP_LOCALIZATION_LIDAR_MODE_HPP
