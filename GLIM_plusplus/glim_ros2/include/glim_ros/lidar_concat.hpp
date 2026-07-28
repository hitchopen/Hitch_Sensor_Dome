#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <glim/util/config.hpp>
#include <glim/util/urdf_transforms.hpp>
#include <glim_ros/lidar_fov_gate.hpp>

namespace glim_ros {

struct PointTimeRangeNs {
  bool valid = false;
  uint64_t min_ns = 0;
  uint64_t max_ns = 0;
  size_t count = 0;
  size_t zero_count = 0;
};

struct BufferedAuxCloud {
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  PointTimeRangeNs point_time_range;
};

struct AuxLidarSensor {
  std::string topic;
  Eigen::Isometry3d T_primary_sensor;
  std::deque<BufferedAuxCloud> buffer;
  size_t buffer_size;
  // Header phase is only a scheduling/matching hint. It must never be copied
  // into absolute point timestamps.
  double match_time_offset = 0.0;
  // A measured residual point-clock correction. Applied to the aux sweep's
  // ABSOLUTE point-time range for matching and to the merged point timestamps.
  // On Robin W (FLOAT64 epoch seconds) in the seconds domain; on a raw uint64
  // epoch-ns layout, in the ns domain.
  double point_time_offset = 0.0;
  // Signed raw header phase vs primary, over merged scans. This is acquisition
  // phase evidence, not by itself a PTP/point-clock measurement.
  double dt_sum = 0.0;
  double dt_min = std::numeric_limits<double>::infinity();
  double dt_max = -std::numeric_limits<double>::infinity();
  uint64_t dt_count = 0;
  // Best absolute point-time ENDPOINT DELTA vs the primary sweep, recorded on
  // EVERY candidate evaluation — accepted or rejected.
  //
  // This is the quantity `sweep_time_threshold` actually gates on, and it
  // is the only number that can tell an operator whether that threshold is
  // correctly configured for this rig. Without it a run that rejects every
  // aux is indistinguishable in the logs from a run with no aux configured:
  // the header dt in CONCAT DEBUG is populated ONLY for accepted merges and
  // measures header phase, not this. Sweep alignment on Robin W is a
  // deployment property (frame scheduling is free-running; PTP disciplines
  // clocks only), so it must be measured, not assumed.
  double range_delta_sum = 0.0;
  double range_delta_min = std::numeric_limits<double>::infinity();
  double range_delta_max = -std::numeric_limits<double>::infinity();
  uint64_t range_delta_count = 0;   // candidate evaluations with a decodable range
  uint64_t range_gate_rejects = 0;  // of those, how many exceeded the gate
  bool vertical_fov_validated = false;
  uint64_t vertical_fov_reject_count = 0;
};

inline double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

inline uint64_t first_ordinal_at_or_after(
  const std::vector<double>& bag_times_s, double delivered_bag_time_s) {
  return static_cast<uint64_t>(std::distance(
    bag_times_s.begin(),
    std::lower_bound(bag_times_s.begin(), bag_times_s.end(), delivered_bag_time_s)));
}

inline bool online_concat_configuration_supported(
  bool online_mapping_enabled, bool concat_enabled) {
  return !(online_mapping_enabled && concat_enabled);
}

inline uint64_t abs_diff_ns(uint64_t a, uint64_t b) {
  return (a >= b) ? (a - b) : (b - a);
}

inline int64_t seconds_to_nanoseconds(double seconds) {
  if (!std::isfinite(seconds)) return 0;
  const long double ns = static_cast<long double>(seconds) * 1.0e9L;
  if (ns >= static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return std::numeric_limits<int64_t>::max();
  }
  if (ns <= static_cast<long double>(std::numeric_limits<int64_t>::min())) {
    return std::numeric_limits<int64_t>::min();
  }
  return static_cast<int64_t>(std::llround(ns));
}

inline uint64_t shifted_timestamp_ns(uint64_t timestamp_ns, int64_t shift_ns) {
  if (shift_ns >= 0) {
    const uint64_t add = static_cast<uint64_t>(shift_ns);
    return timestamp_ns > std::numeric_limits<uint64_t>::max() - add
             ? std::numeric_limits<uint64_t>::max()
             : timestamp_ns + add;
  }
  const uint64_t sub = static_cast<uint64_t>(-(shift_ns + 1)) + 1ULL;
  return timestamp_ns >= sub ? timestamp_ns - sub : 0;
}

inline PointTimeRangeNs shifted_range(
  const PointTimeRangeNs& range, double clock_offset_s) {
  if (!range.valid) return range;
  const int64_t shift_ns = seconds_to_nanoseconds(clock_offset_s);
  PointTimeRangeNs shifted = range;
  shifted.min_ns = shifted_timestamp_ns(range.min_ns, shift_ns);
  shifted.max_ns = shifted_timestamp_ns(range.max_ns, shift_ns);
  return shifted;
}

inline double endpoint_delta_seconds(
  const PointTimeRangeNs& lhs,
  const PointTimeRangeNs& rhs) {
  if (!lhs.valid || !rhs.valid) return std::numeric_limits<double>::infinity();
  return static_cast<double>(std::max(
    abs_diff_ns(lhs.min_ns, rhs.min_ns), abs_diff_ns(lhs.max_ns, rhs.max_ns))) * 1.0e-9;
}

inline bool sweep_watermark_passed(
  const PointTimeRangeNs& primary,
  const PointTimeRangeNs& newest_aux,
  double threshold_s) {
  if (!primary.valid || !newest_aux.valid) return false;
  const uint64_t threshold_ns = static_cast<uint64_t>(
    std::max<int64_t>(0, seconds_to_nanoseconds(threshold_s)));
  const uint64_t deadline_ns =
    primary.min_ns > std::numeric_limits<uint64_t>::max() - threshold_ns
      ? std::numeric_limits<uint64_t>::max()
      : primary.min_ns + threshold_ns;
  return newest_aux.min_ns > deadline_ns;
}

inline bool find_xyz_offsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  // [P3 FIX 2026-07-10] Validate datatype/width, not just presence: a
  // malformed-but-tight cloud can declare x/y/z as a non-FLOAT32 type or at
  // an offset whose 4-byte read/write crosses the point boundary — the
  // transform below would then corrupt adjacent point fields (or, on the
  // last point, overrun the buffer). Reject the cloud instead.
  const auto valid = [&](const sensor_msgs::msg::PointField& f) {
    return f.datatype == sensor_msgs::msg::PointField::FLOAT32 && f.count == 1 &&
           static_cast<size_t>(f.offset) + sizeof(float) <= msg.point_step;
  };
  for (const auto& f : msg.fields) {
    if (f.name == "x") { if (!valid(f)) return false; x_off = f.offset; }
    else if (f.name == "y") { if (!valid(f)) return false; y_off = f.offset; }
    else if (f.name == "z") { if (!valid(f)) return false; z_off = f.offset; }
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Full point-field schema comparison: decides whether an auxiliary cloud can be
// byte-appended onto the primary. merge_clouds() transforms aux points using the
// AUX cloud's own field offsets, but the MERGED cloud keeps the PRIMARY's
// `fields`, so every downstream reader interprets the appended aux bytes with the
// primary layout. Appending is only safe when the aux layout is byte-identical to
// the primary: same point_step, same endianness, and the same ordered set of
// field {name, offset, datatype, count}. A same-point_step cloud with different
// offsets/datatypes would otherwise be silently misread. O(#fields) (~10-20 per
// typical scan) -- negligible next to deskew / voxelisation / registration.
// Returns true on match; on mismatch returns false and sets `reason` for logging.
inline bool schema_matches_primary(const sensor_msgs::msg::PointCloud2& aux,
                                   const sensor_msgs::msg::PointCloud2& primary,
                                   std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " + std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " + std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); i++) {
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

inline void transform_cloud_data(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int x_off,
  int y_off,
  int z_off,
  const Eigen::Isometry3d& T) {
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  const size_t num_points = data.size() / point_step;

  for (size_t i = 0; i < num_points; i++) {
    const size_t base = i * point_step;
    float x, y, z;
    std::memcpy(&x, &data[base + x_off], sizeof(float));
    std::memcpy(&y, &data[base + y_off], sizeof(float));
    std::memcpy(&z, &data[base + z_off], sizeof(float));

    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(&data[base + x_off], &p.x(), sizeof(float));
    std::memcpy(&data[base + y_off], &p.y(), sizeof(float));
    std::memcpy(&data[base + z_off], &p.z(), sizeof(float));
  }
}

inline sensor_msgs::msg::PointCloud2::SharedPtr find_nearest(
  const std::deque<BufferedAuxCloud>& buffer,
  double target_sec,
  double threshold,
  double match_time_offset = 0.0) {
  sensor_msgs::msg::PointCloud2::SharedPtr best;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto& buffered : buffer) {
    const auto& msg = buffered.msg;
    double dt = std::abs(stamp_to_sec(msg->header.stamp) + match_time_offset - target_sec);
    if (dt < best_dt) {
      best_dt = dt;
      best = msg;
    }
  }
  return (best && best_dt <= threshold) ? best : nullptr;
}

inline bool find_time_field(const sensor_msgs::msg::PointCloud2& msg, int& time_off, uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = f.offset;
      time_datatype = f.datatype;
      time_count = f.count;
      return true;
    }
  }
  return false;
}

// Minimum decoded-seconds value accepted as an ABSOLUTE epoch axis by the
// FLOAT64-seconds branch below (and by the shift gate in
// shift_cloud_timestamps). ~11.6 days: real scan-relative offsets are < 1 s,
// real epochs are > 1e9 s. NOTE this gate operates on the DECODED IEEE-754
// double (seconds), which is sound — unlike the raw-bit uint64 magnitude
// heuristic that upstream removed in the [P2 FIX 2026-07-15]
// extract_raw_points change.
constexpr double kMinAbsoluteEpochSeconds = 1e6;
// Livox PointCloud2 stores numeric nanoseconds in FLOAT64 (not uint64 bits).
// Values above this gate are therefore an absolute epoch-nanosecond axis.
constexpr double kMinAbsoluteEpochNanoseconds = 1e15;

// Decode the authoritative absolute point-time range carried by the primary
// LiDAR driver. Supported encodings:
//   * UINT8[8] — raw uint64 PTP epoch nanoseconds. NOT emitted by Robin W;
//     retained for upstream-bag compatibility.
//   * FLOAT64 (count 1), float64_time_is_epoch_ns=true — raw uint64 epoch-ns
//     bytes under the explicit float64_time_is_epoch_ns contract. NOT the
//     Robin W layout — Robin W emits IEEE-754 epoch SECONDS.
//   * FLOAT64 (count 1), float64_time_is_epoch_ns=false — either IEEE-754
//     absolute epoch seconds (Hesai) or numeric epoch nanoseconds (Livox).
//     Scan-relative FLOAT64 seconds fail closed to the header-based fallback.
// Seyond Robin W's canonical `timestamp`/FLOAT64/Unix-seconds layout uses this
// absolute range path directly.
// Other timestamp layouts deliberately return invalid so they continue
// through the generic header-based fallback.
inline PointTimeRangeNs decode_point_time_range(
  const sensor_msgs::msg::PointCloud2& msg,
  bool float64_time_is_epoch_ns = false) {
  PointTimeRangeNs range;
  // The decode below memcpy's little-endian payloads. A big-endian
  // payload would produce garbage ranges that could still fall inside the
  // matching gate by chance — reject it before matching (fail closed).
  if (msg.is_bigendian) {
    return range;
  }
  int time_off = -1;
  uint8_t datatype = 0;
  int count = 0;
  if (!find_time_field(msg, time_off, datatype, count)) {
    return range;
  }
  // FLOAT64 is a raw epoch-ns carrier only under the explicit driver contract.
  // Otherwise it is a numeric IEEE-754 value. Its decoded magnitude soundly
  // separates relative seconds, epoch seconds, and Livox epoch nanoseconds.
  const bool is_u8x8 = datatype == sensor_msgs::msg::PointField::UINT8 && count == 8;
  const bool is_f64 = datatype == sensor_msgs::msg::PointField::FLOAT64 && count == 1;
  const bool is_f64_epoch_ns = float64_time_is_epoch_ns && is_f64;
  const bool is_f64_numeric = !float64_time_is_epoch_ns && is_f64;
  if (!(is_u8x8 || is_f64_epoch_ns || is_f64_numeric) ||
      time_off < 0 || msg.point_step == 0 ||
      static_cast<size_t>(time_off) + sizeof(uint64_t) > msg.point_step) {
    return range;
  }
  const size_t point_count = static_cast<size_t>(msg.width) * msg.height;
  if (point_count == 0 ||
      point_count > std::numeric_limits<size_t>::max() / msg.point_step) {
    return range;
  }
  const size_t expected_row_step =
    static_cast<size_t>(msg.width) * static_cast<size_t>(msg.point_step);
  const size_t required_bytes = point_count * static_cast<size_t>(msg.point_step);
  if (static_cast<size_t>(msg.row_step) != expected_row_step ||
      msg.data.size() != required_bytes) {
    return range;
  }

  range.min_ns = std::numeric_limits<uint64_t>::max();
  enum class Float64Axis { UNKNOWN, EPOCH_SECONDS, EPOCH_NANOSECONDS };
  Float64Axis float64_axis = Float64Axis::UNKNOWN;
  for (size_t i = 0; i < point_count; ++i) {
    uint64_t timestamp_ns = 0;
    if (is_f64_numeric) {
      double value = 0.0;
      std::memcpy(&value, msg.data.data() + i * msg.point_step + time_off, sizeof(double));
      if (value == 0.0) {
        ++range.zero_count;
        continue;
      }
      if (!std::isfinite(value) || value < kMinAbsoluteEpochSeconds) {
        return PointTimeRangeNs{};
      }
      const Float64Axis sample_axis =
        value >= kMinAbsoluteEpochNanoseconds
          ? Float64Axis::EPOCH_NANOSECONDS
          : Float64Axis::EPOCH_SECONDS;
      if (float64_axis != Float64Axis::UNKNOWN && float64_axis != sample_axis) {
        return PointTimeRangeNs{};
      }
      float64_axis = sample_axis;
      const double ns =
        sample_axis == Float64Axis::EPOCH_NANOSECONDS ? value : value * 1e9;
      if (ns >= static_cast<double>(std::numeric_limits<uint64_t>::max())) {
        return PointTimeRangeNs{};
      }
      timestamp_ns = static_cast<uint64_t>(ns);
    } else {
      std::memcpy(&timestamp_ns,
                  msg.data.data() + i * msg.point_step + time_off,
                  sizeof(timestamp_ns));
    }
    if (timestamp_ns == 0) {
      ++range.zero_count;
      continue;
    }
    range.min_ns = std::min(range.min_ns, timestamp_ns);
    range.max_ns = std::max(range.max_ns, timestamp_ns);
    ++range.count;
  }
  range.valid = range.count > 0;
  return range;
}

inline BufferedAuxCloud buffer_aux_cloud(
  sensor_msgs::msg::PointCloud2::SharedPtr msg,
  bool float64_time_is_epoch_ns = false) {
  BufferedAuxCloud buffered;
  buffered.point_time_range = decode_point_time_range(*msg, float64_time_is_epoch_ns);
  buffered.msg = std::move(msg);
  return buffered;
}

struct SweepMatch {
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  double range_delta_s = std::numeric_limits<double>::infinity();
  double header_abs_delta_s = std::numeric_limits<double>::infinity();
};

inline std::optional<SweepMatch> find_closest_sweep(
  const std::deque<BufferedAuxCloud>& buffer,
  const PointTimeRangeNs& primary_range,
  double point_time_offset,
  double primary_header_s,
  double match_time_offset) {
  std::optional<SweepMatch> best;
  for (const auto& buffered : buffer) {
    if (!buffered.point_time_range.valid) continue;
    const auto candidate_range = shifted_range(buffered.point_time_range, point_time_offset);
    const double range_delta_s = endpoint_delta_seconds(primary_range, candidate_range);
    const double header_abs_delta_s = std::abs(
      stamp_to_sec(buffered.msg->header.stamp) + match_time_offset - primary_header_s);
    if (!best || range_delta_s < best->range_delta_s ||
        (range_delta_s == best->range_delta_s &&
         header_abs_delta_s < best->header_abs_delta_s)) {
      best = SweepMatch{buffered.msg, range_delta_s, header_abs_delta_s};
    }
  }
  return best;
}

// Offline readers call this before releasing a queued primary scan. A primary
// is ready once every aux either has a point-coherent match or has advanced
// beyond the point-time gate, proving that no future match can still arrive.
// [P3 FIX 2026-07-14] Cached-range overload. The streaming-fallback readiness
// poll runs on EVERY bag event while a primary waits (~20-40× per primary), and
// the primary's point-time range is invariant — decoding it each call walked
// ~10^5 points per poll. Callers cache the range once at enqueue and pass it
// here.
inline bool aux_buffers_ready_for_primary(
  const PointTimeRangeNs& primary_range,
  double primary_header_s,
  const std::vector<AuxLidarSensor>& aux_sensors,
  double sweep_time_threshold) {
  if (!primary_range.valid) {
    // Relative per-point time layouts (for example the Laguna decoder's
    // FLOAT64 seconds-since-sweep-start field) cannot use the absolute
    // point-range watermark below.  Do not release the primary immediately:
    // that would always select the latest *past* side sweep simply because
    // the next, closer sweep has not arrived yet.  This was the map-warping
    // difference from perception-ws, whose merge holds front scans until a
    // future side frame is available and then chooses the nearest corrected
    // header stamp.
    //
    // Bag messages are ordered, so after each auxiliary topic has advanced to
    // the primary header (including its scheduling-only match offset), the
    // first future candidate is present and no later candidate can be closer
    // than it without crossing another full scan period.  The caller's
    // future_sweep_wait_timeout remains the fail-safe for a dead/gappy topic.
    for (const auto& aux : aux_sensors) {
      double newest_header_s = -std::numeric_limits<double>::infinity();
      for (const auto& buffered : aux.buffer) {
        newest_header_s = std::max(newest_header_s, stamp_to_sec(buffered.msg->header.stamp) + aux.match_time_offset);
      }
      if (newest_header_s < primary_header_s) return false;
    }
    return true;
  }
  for (const auto& aux : aux_sensors) {
    if (aux.buffer.empty()) return false;
    const auto match = find_closest_sweep(
      aux.buffer, primary_range, aux.point_time_offset,
      primary_header_s, aux.match_time_offset);
    if (match && match->range_delta_s <= sweep_time_threshold) continue;

    PointTimeRangeNs newest;
    for (const auto& buffered : aux.buffer) {
      const auto candidate = shifted_range(buffered.point_time_range, aux.point_time_offset);
      if (candidate.valid && (!newest.valid || candidate.min_ns > newest.min_ns)) {
        newest = candidate;
      }
    }
    if (!sweep_watermark_passed(primary_range, newest, sweep_time_threshold)) {
      return false;
    }
  }
  return true;
}

// Convenience wrapper that decodes the primary range on the spot. Prefer the
// cached-range overload above on the hot readiness-poll path (P3-10).
inline bool aux_buffers_ready_for_primary(
  const sensor_msgs::msg::PointCloud2& primary,
  const std::vector<AuxLidarSensor>& aux_sensors,
  double sweep_time_threshold,
  bool float64_time_is_epoch_ns = false) {
  return aux_buffers_ready_for_primary(
    decode_point_time_range(primary, float64_time_is_epoch_ns), stamp_to_sec(primary.header.stamp),
    aux_sensors, sweep_time_threshold);
}

// Shift per-point timestamps by `dt` seconds to rebase an aux scan from its
// own header.stamp onto the merged cloud's primary header.stamp.
//
// SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start, UINT32
// nanoseconds-since-scan-start): add dt so the value reads as "offset since
// primary scan start" and deskew works.
//
// FLOAT64 may be scan-relative seconds or a numeric absolute axis. The decoded
// magnitude gate below keeps absolute Unix seconds (Seyond/Hesai) and numeric
// epoch nanoseconds (Livox) unchanged by dt. The explicit
// float64_time_is_epoch_ns driver contract instead routes raw uint64 bits
// through the absolute path; never infer that raw-bit choice from magnitude.
//
// ABSOLUTE-EPOCH encodings (raw UINT8[8] = uint64 PTP epoch ns, and the
// Robin W FLOAT64 epoch-seconds layout):
// do not get the header-relative dt shift. They are shifted only by the
// configured constant aux clock correction, if any, because TimeKeeper uses
// absolute epoch values directly for deskew.
//
// Provenance: the raw uint64 epoch-ns handling below was inherited from the
// upstream Iris platform, whose driver reconstructs an absolute epoch from
// header seconds + per-ray nanoseconds. The Hitch dome does NOT use that
// encoding — Robin W emits FLOAT64 absolute epoch SECONDS — but the branch is
// kept so upstream bags still replay and a future absolute-ns sensor needs no
// second decoder.
//
inline void shift_cloud_timestamps(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int time_off,
  uint8_t time_datatype,
  int time_count,
  double dt,
  double abs_clock_shift_s = 0.0,
  bool float64_time_is_epoch_ns = false) {
  if (time_off < 0) return;

  // Raw uint64 PTP epoch nanoseconds are ABSOLUTE: skip the header-relative dt
  // shift, but apply a configured constant clock correction so aux clouds align
  // with the primary/IMU timebase before GLIM deskew. The explicitly configured
  // opt-in FLOAT64-as-raw-bytes variant holds those same raw bytes, so IEEE-754
  // arithmetic on it would corrupt every merged auxiliary timestamp.
  const bool raw_epoch_ns =
    (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8) ||
    (float64_time_is_epoch_ns && time_datatype == sensor_msgs::msg::PointField::FLOAT64 && time_count == 1);
  if (raw_epoch_ns) {
    if (abs_clock_shift_s == 0.0) {
      (void)dt;
      return;
    }
    if (!std::isfinite(abs_clock_shift_s)) {
      static bool warned_uint8_nonfinite = false;
      if (!warned_uint8_nonfinite) {
        spdlog::warn("shift_cloud_timestamps: non-finite UINT8[8] clock correction {}; leaving unshifted",
                     abs_clock_shift_s);
        warned_uint8_nonfinite = true;
      }
      (void)dt;
      return;
    }
    if (point_step == 0 || static_cast<uint32_t>(time_off) + sizeof(uint64_t) > point_step) {
      static bool warned_uint8_bounds = false;
      if (!warned_uint8_bounds) {
        spdlog::warn("shift_cloud_timestamps: UINT8[8] time field offset={} does not fit point_step={}; leaving unshifted",
                     time_off, point_step);
        warned_uint8_bounds = true;
      }
      (void)dt;
      return;
    }
    const double offset_ns_d = abs_clock_shift_s * 1e9;
    if (offset_ns_d > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
        offset_ns_d < static_cast<double>(std::numeric_limits<int64_t>::min())) {
      static bool warned_uint8_range = false;
      if (!warned_uint8_range) {
        spdlog::warn("shift_cloud_timestamps: UINT8[8] clock correction {}s is out of int64 ns range; leaving unshifted",
                     abs_clock_shift_s);
        warned_uint8_range = true;
      }
      (void)dt;
      return;
    }
    const int64_t offset_ns = static_cast<int64_t>(offset_ns_d);
    const size_t num_points = data.size() / point_step;
    for (size_t i = 0; i < num_points; i++) {
      uint8_t* time_ptr = &data[i * point_step + time_off];
      uint64_t val;
      std::memcpy(&val, time_ptr, sizeof(uint64_t));
      if (offset_ns >= 0) {
        const uint64_t add = static_cast<uint64_t>(offset_ns);
        val = (std::numeric_limits<uint64_t>::max() - val < add) ? std::numeric_limits<uint64_t>::max() : val + add;
      } else {
        const uint64_t sub = static_cast<uint64_t>(-(offset_ns + 1)) + 1ULL;
        val = (val > sub) ? (val - sub) : 0;
      }
      std::memcpy(time_ptr, &val, sizeof(uint64_t));
    }
    (void)dt;
    return;
  }

  // [P3 FIX 2026-07-10] Per-width bounds for the generic (relative-time)
  // branches — parity with the UINT8[8] path's guard above: a declared time
  // field that does not fit its point must not be written.
  {
    size_t width = 0;
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32:
      case sensor_msgs::msg::PointField::FLOAT32: width = 4; break;
      case sensor_msgs::msg::PointField::FLOAT64: width = 8; break;
      default: return;  // unknown carrier: nothing safe to shift
    }
    if (point_step == 0 || static_cast<size_t>(time_off) + width > point_step) {
      static bool warned_generic_bounds = false;
      if (!warned_generic_bounds) {
        spdlog::warn("shift_cloud_timestamps: time field (datatype={}) offset={} does not fit "
                     "point_step={}; leaving unshifted", time_datatype, time_off, point_step);
        warned_generic_bounds = true;
      }
      return;
    }
  }

  // FLOAT64 magnitude gate: a numeric FLOAT64 can carry scan-relative seconds
  // (shiftable), absolute epoch seconds (Hesai), or numeric epoch nanoseconds
  // (Livox). Absolute axes must not receive header-relative dt because that
  // would double-apply the inter-scan offset. Zero is the "no valid time"
  // sentinel and NaN/inf
  // is garbage — neither may defeat the gate. The gate scans ALL points and
  // classifies the cloud ABSOLUTE if ANY finite non-zero value exceeds
  // kMinAbsoluteEpochSeconds: a mixed/corrupt cloud (some relative-looking,
  // some epoch-scaled values) must fail toward NOT shifting — adding dt to
  // even one absolute point is the double-apply this gate exists to prevent.
  // A cloud with NO finite non-zero sample at all (all sentinels/garbage) is
  // likewise left untouched: there is nothing meaningful to rebase and the
  // relative loop below would overwrite the 0.0 sentinels with dt.
  // NOTE: this magnitude test is on the DECODED double (seconds), not on raw
  // bits reinterpreted as uint64 — the unsound heuristic upstream removed in
  // the [P2 FIX 2026-07-15] extract_raw_points change was the
  // bit-reinterpretation one; a decoded-seconds magnitude test is sound.
  //
  // Parity with the raw epoch-ns branch above: an absolute cloud DOES receive
  // the configured residual point-clock correction in its native units.
  if (time_datatype == sensor_msgs::msg::PointField::FLOAT64 &&
      point_step > 0 && data.size() >= static_cast<size_t>(time_off) + sizeof(double)) {
    const size_t n_pts = data.size() / point_step;
    bool looks_absolute = false;
    bool looks_numeric_epoch_ns = false;
    bool any_finite_nonzero = false;
    for (size_t i = 0; i < n_pts; ++i) {
      double val = 0.0;
      std::memcpy(&val, &data[i * point_step + time_off], sizeof(double));
      if (val == 0.0 || !std::isfinite(val)) continue;  // sentinel / garbage: keep scanning
      any_finite_nonzero = true;
      if (std::abs(val) > kMinAbsoluteEpochSeconds) {
        looks_absolute = true;
        looks_numeric_epoch_ns =
          std::abs(val) >= kMinAbsoluteEpochNanoseconds;
        break;  // one absolute sample decides: never dt-shift this cloud
      }
    }
    if (!any_finite_nonzero) {
      // All sentinels/garbage: nothing to rebase; do not turn 0.0 sentinels
      // into dt in the relative loop below.
      (void)dt;
      return;
    }
    if (looks_absolute) {
      static bool warned_absolute_f64 = false;
      if (!warned_absolute_f64) {
        spdlog::warn(
          "shift_cloud_timestamps: FLOAT64 time field looks ABSOLUTE ({}); "
          "skipping the header-relative dt rebase — absolute per-point times need no "
          "rebase onto the primary clock",
          looks_numeric_epoch_ns ? "numeric epoch nanoseconds" : "epoch seconds");
        warned_absolute_f64 = true;
      }
      if (abs_clock_shift_s != 0.0 && std::isfinite(abs_clock_shift_s)) {
        const double correction =
          looks_numeric_epoch_ns ? abs_clock_shift_s * 1e9 : abs_clock_shift_s;
        for (size_t i = 0; i < n_pts; ++i) {
          uint8_t* time_ptr = &data[i * point_step + time_off];
          double val = 0.0;
          std::memcpy(&val, time_ptr, sizeof(double));
          if (val == 0.0 || !std::isfinite(val)) continue;  // keep the sentinel at 0
          val += correction;
          std::memcpy(time_ptr, &val, sizeof(double));
        }
      }
      (void)dt;
      return;
    }
  }

  const size_t num_points = data.size() / point_step;
  for (size_t i = 0; i < num_points; i++) {
    uint8_t* time_ptr = &data[i * point_step + time_off];
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
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
      default:
        break;
    }
  }
}

// `primary` is taken as a ConstSharedPtr so both the offline tools (which hold
// a mutable SharedPtr) and the live GlimROS points_callback (which receives a
// P4 review follow-up: per-point time span (seconds) of a cloud, for the
// CONCAT DEBUG evidence line (parity with GICP's scan_time_span_s debug
// topic). A healthy 3-LiDAR merge spans ~1 sweep period; a much larger span
// means a badly-offset aux was rebased far from the primary and will be
// deskewed across a long arc. Returns NaN when no usable time field exists.
// Units by encoding: UINT8[8] = uint64 epoch ns; UINT32 = ns;
// FLOAT32 = seconds. FLOAT64 = seconds, except Livox numeric epoch ns, which
// is detected by magnitude and scaled by 1e-9. One O(N) pass; only run when
// diagnostics are enabled.
inline double cloud_time_span_seconds(const sensor_msgs::msg::PointCloud2& cloud,
                                      bool float64_time_is_epoch_ns = false) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  int off = -1;
  uint8_t datatype = 0;
  int count = 0;
  if (!find_time_field(cloud, off, datatype, count)) return nan;
  const uint32_t step = cloud.point_step;
  if (step == 0 || off < 0 || static_cast<uint32_t>(off) >= step) return nan;
  const size_t n = cloud.data.size() / step;
  if (n == 0) return nan;
  const size_t avail = step - static_cast<uint32_t>(off);

  switch (datatype) {
    case sensor_msgs::msg::PointField::UINT8: {  // raw uint64 epoch ns
      if (count != 8 || avail < 8) return nan;
      uint64_t mn = std::numeric_limits<uint64_t>::max(), mx = 0;
      for (size_t i = 0; i < n; i++) {
        uint64_t v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
      return (mx >= mn) ? (mx - mn) * 1e-9 : nan;
    }
    case sensor_msgs::msg::PointField::UINT32: {  // scan-relative ns
      if (avail < 4) return nan;
      uint32_t mn = std::numeric_limits<uint32_t>::max(), mx = 0;
      for (size_t i = 0; i < n; i++) {
        uint32_t v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
      return (mx >= mn) ? (mx - mn) * 1e-9 : nan;
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      if (avail < 4) return nan;
      float mn = std::numeric_limits<float>::infinity(), mx = -mn;
      for (size_t i = 0; i < n; i++) {
        float v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        if (std::isfinite(v)) { mn = std::min(mn, v); mx = std::max(mx, v); }
      }
      return (mx >= mn) ? static_cast<double>(mx - mn) : nan;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      if (avail < 8) return nan;
      if (float64_time_is_epoch_ns) {
        uint64_t mn = std::numeric_limits<uint64_t>::max(), mx = 0;
        for (size_t i = 0; i < n; i++) {
          uint64_t v;
          std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
          if (v == 0) continue;
          mn = std::min(mn, v);
          mx = std::max(mx, v);
        }
        return (mn != std::numeric_limits<uint64_t>::max() && mx >= mn) ? (mx - mn) * 1e-9 : nan;
      }
      double mn = std::numeric_limits<double>::infinity(), mx = -mn;
      for (size_t i = 0; i < n; i++) {
        double v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        // Skip the 0.0 "no valid time" sentinel; otherwise one sentinel on an
        // absolute axis would report a span roughly equal to the whole epoch.
        if (v == 0.0) continue;
        if (std::isfinite(v)) { mn = std::min(mn, v); mx = std::max(mx, v); }
      }
      if (mx < mn) return nan;
      const double scale =
        std::max(std::abs(mn), std::abs(mx)) >= kMinAbsoluteEpochNanoseconds
          ? 1e-9
          : 1.0;
      return (mx - mn) * scale;
    }
    default:
      return nan;
  }
}

// ConstSharedPtr) can call this directly. The primary cloud is only read here;
// the merged output is a fresh copy.
// Strict merge guard (optional): when require_all_aux is set, a scan that fails to
// merge every configured aux must not be silently localized on fewer LiDARs. The
// caller passes a persistent counter (consec_fail); brief transient misses are
// tolerated up to max_consec_fail, after which merge_clouds throws std::runtime_error
// (stopping the node) rather than returning a degraded single-/partial-LiDAR cloud.
// Returns nullptr (not the primary) when require_all_aux is set and the merge is
// incomplete, so the caller SKIPS the scan rather than localizing a degraded cloud.
inline sensor_msgs::msg::PointCloud2::ConstSharedPtr merge_clouds(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary,
  std::vector<AuxLidarSensor>& aux_sensors,
  double time_threshold,
  LidarQualityConfig& lidar_quality,
  bool require_all_aux = true,
  int max_consec_fail = 0,
  int* consec_fail = nullptr,
  bool abort_on_merge_failure = true,
  bool frame_diag_log = false,
  double sweep_time_threshold = 0.010,
  bool float64_time_is_epoch_ns = false) {
  VerticalFovMeasurement primary_fov;
  if (!verticalFovAccepted(
        *primary, lidar_quality.minimum_vertical_fov_deg,
        lidar_quality.minimum_valid_points, &primary_fov)) {
    ++lidar_quality.primary_reject_count;
    if (lidar_quality.primary_reject_count <= 10 ||
        lidar_quality.primary_reject_count % 100 == 0) {
      spdlog::error(
        "lidar_quality: rejecting primary cloud before merge/SLAM: robust "
        "vertical FOV {:.2f} deg (elevation {:.2f}..{:.2f} deg), required "
        ">= {:.2f} deg; valid sampled returns={}/{}. Reason: {}. "
        "A narrowed vertical FOV does not provide enough vertical structure "
        "for stable GLIM.",
        primary_fov.span_deg, primary_fov.lower_deg, primary_fov.upper_deg,
        lidar_quality.minimum_vertical_fov_deg, primary_fov.valid_points,
        primary_fov.sampled_points, primary_fov.reason);
    }
    return nullptr;
  }
  if (!lidar_quality.primary_validated) {
    lidar_quality.primary_validated = true;
    spdlog::info(
      "lidar_quality: primary vertical-FOV gate passed: robust span {:.2f} "
      "deg (elevation {:.2f}..{:.2f} deg, {} valid sampled returns)",
      primary_fov.span_deg, primary_fov.lower_deg, primary_fov.upper_deg,
      primary_fov.valid_points);
  }

  const double t_primary = stamp_to_sec(primary->header.stamp);
  const auto primary_point_time_range = decode_point_time_range(*primary, float64_time_is_epoch_ns);
  const uint32_t point_step = primary->point_step;
  size_t merged_aux_count = 0;

  // P4#3 (GLIM parity): per-frame merge record. GLIM's offline mapping tools
  // have no ROS node to publish debug topics from, so the structured evidence
  // channel is one parseable INFO line per frame ("CONCAT DEBUG | ...") in the
  // mapping log, mirroring gicp_localization's per-frame debug topics.
  std::vector<double> diag_aux_dt(aux_sensors.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<size_t> diag_aux_pts(aux_sensors.size(), 0);
  const auto emit_frame_diag = [&](size_t total_pts, const sensor_msgs::msg::PointCloud2& cloud) {
    if (!frame_diag_log) return;
    std::ostringstream oss;
    oss << "CONCAT DEBUG | stamp=" << std::fixed << std::setprecision(3) << t_primary
        << " merged=" << merged_aux_count << "/" << aux_sensors.size();
    oss << std::setprecision(4);
    for (size_t i = 0; i < aux_sensors.size(); ++i) {
      oss << " dt" << i << "=" << diag_aux_dt[i] << "s pts" << i << "=" << diag_aux_pts[i];
    }
    // Merged-sweep per-point time span (GICP scan_time_span_s parity); NaN =
    // no usable time field. Computed only when the diag line is enabled.
    oss << " span=" << cloud_time_span_seconds(cloud, float64_time_is_epoch_ns) << "s total_pts=" << total_pts;
    // "{}" wrapper: never pass a runtime string as the fmt format string
    // (stray braces would throw fmt::format_error mid-mapping).
    spdlog::info("{}", oss.str());
  };

  // Strict-merge failure handler. Single routing point for every "required merge
  // can't complete" path (incomplete aux merge AND primary-precondition failures).
  //   - require_all_aux=false -> returns false: degraded merging allowed.
  //   - require_all_aux=true  -> returns true: the caller returns nullptr so the
  //     degraded cloud is never localized (scan skipped). Past max_consec_fail the
  //     node either aborts (abort_on_merge_failure=true) or keeps skipping with a
  //     louder warning (abort_on_merge_failure=false). Reset on a fully merged scan.
  auto on_required_failure = [&](size_t got, const char* reason) -> bool {
    if (!require_all_aux || !consec_fail) return false;  // degraded merging allowed
    ++(*consec_fail);
    const bool over_budget = *consec_fail > max_consec_fail;
    if (over_budget && abort_on_merge_failure) {
      const std::string msg =
        std::string("lidar_concat: multi-LiDAR merge REQUIRED but could not complete (") + reason + ") for " +
        std::to_string(*consec_fail) + " consecutive scans; abort_on_merge_failure=true -> stopping "
        "(set require_all_aux=false to localize on available LiDARs, or abort_on_merge_failure=false to keep skipping)";
      spdlog::critical(msg);
      throw std::runtime_error(msg);
    }
    spdlog::error(
      "lidar_concat: REQUIRED merge incomplete ({}/{} aux): {} for {} consecutive scan(s) (budget {}){} "
      "-- skipping scan (degraded cloud NOT localized).",
      got, aux_sensors.size(), reason, *consec_fail, max_consec_fail,
      over_budget ? ", budget exceeded (non-fatal)" : "");
    return true;  // skip this scan
  };

  int x_off, y_off, z_off;
  if (!find_xyz_offsets(*primary, x_off, y_off, z_off)) {
    spdlog::warn("lidar_concat: cannot find xyz fields in primary cloud");
    emit_frame_diag(primary->width * primary->height, *primary);
    if (on_required_failure(0, "primary cloud missing xyz fields")) return nullptr;
    return primary;
  }

  // Concatenation treats each cloud as a TIGHT array of point_step-sized points
  // (it byte-appends aux data and re-counts by point_step). Reject a row-padded
  // primary (data.size() != width*height*point_step) loudly rather than silently
  // counting padding as points. Organized-but-tight (height>1, no padding) is
  // fine to flatten; only padding is rejected. Robin W clouds are unorganized and
  // tight (PCAP reader emits height=1, row_step=point_step*width).
  if (point_step == 0 || primary->data.size() != static_cast<size_t>(primary->width) * primary->height * point_step) {
    spdlog::warn("lidar_concat: primary cloud is organized/padded (data={}, width={}, height={}, step={}); skipping concat",
                 primary->data.size(), primary->width, primary->height, point_step);
    emit_frame_diag(primary->width * primary->height, *primary);
    if (on_required_failure(0, "primary cloud organized/padded (non-tight)")) return nullptr;
    return primary;
  }

  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  // Count points from the byte buffer (equals width*height for the tight cloud
  // validated above) so merged width/row_step always match the appended bytes.
  size_t total_points = primary->data.size() / point_step;

  for (size_t aux_i = 0; aux_i < aux_sensors.size(); ++aux_i) {
    auto& aux = aux_sensors[aux_i];
    sensor_msgs::msg::PointCloud2::SharedPtr match;
    double point_range_delta_s = std::numeric_limits<double>::infinity();
    if (primary_point_time_range.valid) {
      // Hardening: an aux whose clouds carry NO decodable absolute point-time
      // range (unsupported layout, big-endian payload) can never be selected
      // under an absolute-time primary — and header matching is NOT a usable
      // fallback for it either, because the byte-append merge requires a
      // schema identical to the primary's (schema_matches_primary would
      // reject it downstream). Make that explicit instead of emitting the
      // generic "no point-time-aligned match" every frame.
      bool any_valid_range = false;
      for (const auto& buffered : aux.buffer) {
        if (buffered.point_time_range.valid) {
          any_valid_range = true;
          break;
        }
      }
      if (!aux.buffer.empty() && !any_valid_range) {
        static std::atomic<uint64_t> mixed_layout_warns{0};
        if (mixed_layout_warns.fetch_add(1) < 3) {
          spdlog::warn(
            "lidar_concat: aux {} carries no decodable absolute point-time "
            "range (UINT8[8] epoch-ns / FLOAT64 epoch-seconds; mixed sensor "
            "layout or big-endian payload); it cannot merge under an "
            "absolute-time primary and header matching is not a usable "
            "fallback (identical schema required) — normalize the sensor "
            "layout upstream (warning capped at 3)",
            aux.topic);
        }
        continue;
      }
      const auto sweep_match = find_closest_sweep(
        aux.buffer, primary_point_time_range, aux.point_time_offset,
        t_primary, aux.match_time_offset);
      // Record the best achievable endpoint delta BEFORE the accept test, so
      // near-misses and outright rejections are measured too (see the stats
      // block in AuxLidarSensor). This is what makes a misconfigured gate
      // diagnosable instead of silent.
      if (sweep_match && std::isfinite(sweep_match->range_delta_s)) {
        const double rd = sweep_match->range_delta_s;
        aux.range_delta_sum += rd;
        aux.range_delta_min = std::min(aux.range_delta_min, rd);
        aux.range_delta_max = std::max(aux.range_delta_max, rd);
        ++aux.range_delta_count;
        if (rd > sweep_time_threshold) ++aux.range_gate_rejects;
        point_range_delta_s = rd;  // reported in diagnostics even on reject
      }
      if (sweep_match && sweep_match->range_delta_s <= sweep_time_threshold) {
        match = sweep_match->msg;
      }
    } else {
      // [P2 FIX 2026-07-14] Primary point-time range is undecodable. Header-
      // nearest is only safe for an aux that ALSO carries no absolute point
      // time; an aux that DOES carry valid absolute ranges must NOT be header-
      // matched — that is exactly the wrong-sweep / up-to-149 ms mode GICP
      // forbids. Skip such an aux instead of appending a possibly-wrong sweep.
      // (Reachable on the glim_rosbag streaming fallback and live paths; the
      // two-pass staging path is already protected.)
      bool aux_has_abs_time = false;
      for (const auto& buffered : aux.buffer) {
        if (buffered.point_time_range.valid) { aux_has_abs_time = true; break; }
      }
      if (aux_has_abs_time) {
        static std::atomic<uint64_t> mismatch_warns{0};
        if (mismatch_warns.fetch_add(1) < 3) {
          spdlog::warn(
            "lidar_concat: primary point-time range undecodable but aux {} carries an absolute "
            "point time (UINT8[8] epoch-ns / FLOAT64 epoch-seconds); refusing header-nearest "
            "matching (would risk a wrong sweep) — dropping aux this scan (warning capped at 3)",
            aux.topic);
        }
        continue;
      }
      match = find_nearest(
        aux.buffer, t_primary, time_threshold, aux.match_time_offset);
    }
    if (!match) {
      if (primary_point_time_range.valid) {
        spdlog::warn(
          "lidar_concat: no point-time-aligned match for {} — best endpoint "
          "delta {:.4f}s exceeds sweep_time_threshold {:.3f}s; dropping aux "
          "instead of appending a non-contemporaneous sweep",
          aux.topic,
          std::isfinite(point_range_delta_s) ? point_range_delta_s
                                             : std::numeric_limits<double>::quiet_NaN(),
          sweep_time_threshold);
      } else {
        spdlog::debug("lidar_concat: no header match for {} (t={:.3f})", aux.topic, t_primary);
      }
      continue;
    }

    VerticalFovMeasurement aux_fov;
    if (!verticalFovAccepted(
          *match, lidar_quality.minimum_vertical_fov_deg,
          lidar_quality.minimum_valid_points, &aux_fov)) {
      ++aux.vertical_fov_reject_count;
      if (aux.vertical_fov_reject_count <= 10 ||
          aux.vertical_fov_reject_count % 100 == 0) {
        spdlog::error(
          "lidar_quality: rejecting aux cloud on {} before transform/merge: "
          "robust vertical FOV {:.2f} deg (elevation {:.2f}..{:.2f} deg), "
          "required >= {:.2f} deg; valid sampled returns={}/{}. Reason: {}.",
          aux.topic, aux_fov.span_deg, aux_fov.lower_deg, aux_fov.upper_deg,
          lidar_quality.minimum_vertical_fov_deg, aux_fov.valid_points,
          aux_fov.sampled_points, aux_fov.reason);
      }
      continue;
    }
    if (!aux.vertical_fov_validated) {
      aux.vertical_fov_validated = true;
      spdlog::info(
        "lidar_quality: aux vertical-FOV gate passed on {}: robust span "
        "{:.2f} deg (elevation {:.2f}..{:.2f} deg, {} valid sampled returns)",
        aux.topic, aux_fov.span_deg, aux_fov.lower_deg, aux_fov.upper_deg,
        aux_fov.valid_points);
    }

    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!schema_matches_primary(*match, *primary, schema_reason)) {
      spdlog::warn(
        "lidar_concat: skipping {} — PointCloud2 schema mismatch vs primary: {} "
        "(merged cloud uses the primary field layout; appending mismatched aux bytes "
        "would misread them — normalize the aux layout upstream to enable concatenation)",
        aux.topic, schema_reason);
      continue;
    }

    // Reject an organized/padded or otherwise non-tight aux: byte-appending it
    // (or counting by point_step) would desync points from the field layout.
    // Requires data.size() == width*height*point_step. Organized-but-tight is OK.
    if ((match->data.size() % point_step) != 0 ||
        match->data.size() != static_cast<size_t>(match->width) * match->height * point_step) {
      spdlog::warn("lidar_concat: skipping {} — non-tight cloud (data={}, width={}, height={}, step={})",
                   aux.topic, match->data.size(), match->width, match->height, point_step);
      continue;
    }

    std::vector<uint8_t> data(match->data.begin(), match->data.end());
    int ax, ay, az;
    if (find_xyz_offsets(*match, ax, ay, az)) {
      transform_cloud_data(data, point_step, ax, ay, az, aux.T_primary_sensor);
    }

    int time_off;
    uint8_t time_datatype;
    int time_count;
    const double raw_dt = stamp_to_sec(match->header.stamp) - t_primary;
    if (find_time_field(*match, time_off, time_datatype, time_count)) {
      shift_cloud_timestamps(
        data, point_step, time_off, time_datatype, time_count,
        raw_dt, aux.point_time_offset, float64_time_is_epoch_ns);
      spdlog::debug(
        "lidar_concat: timestamp handling for {} raw_header_phase={:+.6f}s "
        "point_clock_offset={:+.6f}s point_range_delta={:.6f}s",
        aux.topic, raw_dt, aux.point_time_offset, point_range_delta_s);
    }

    const size_t aux_pts = data.size() / point_step;
    merged->data.insert(merged->data.end(), data.begin(), data.end());
    // Accumulate the byte-derived count (matches the bytes actually appended),
    // not width*height, so merged->width/row_step stay consistent with data.
    total_points += aux_pts;
    ++merged_aux_count;

    spdlog::debug(
      "lidar_concat: merged {} (raw_header_phase={:+.4f}s, "
      "point_range_delta={:.4f}s, {} pts)",
      aux.topic, raw_dt, point_range_delta_s, aux_pts);

    // P4#3: per-frame + running merge-timing diagnostics (parity with
    // gicp_localization). This is the raw signed HEADER acquisition phase.
    // It is intentionally not corrected by point_time_offset and is not a
    // residual PTP estimate.
    diag_aux_dt[aux_i] = raw_dt;
    diag_aux_pts[aux_i] = aux_pts;
    aux.dt_sum += raw_dt;
    aux.dt_min = std::min(aux.dt_min, raw_dt);
    aux.dt_max = std::max(aux.dt_max, raw_dt);
    if (++aux.dt_count % 512 == 0) {  // ~every 50 s at 10 Hz / 25 s at 20 Hz
      const double mean = aux.dt_sum / static_cast<double>(aux.dt_count);
      spdlog::info(
        "lidar_concat: '{}' raw header acquisition phase vs primary over {} "
        "merges: mean={:+.1f} ms, min={:+.1f} ms, max={:+.1f} ms "
        "(not a point-clock estimate)",
        aux.topic, aux.dt_count, 1e3 * mean, 1e3 * aux.dt_min, 1e3 * aux.dt_max);
      // The gated quantity, over EVERY candidate evaluation including
      // rejections — this is the one to compare against
      // sweep_time_threshold when deciding whether the gate fits the rig.
      if (aux.range_delta_count > 0) {
        const double rd_mean =
          aux.range_delta_sum / static_cast<double>(aux.range_delta_count);
        const double reject_frac = static_cast<double>(aux.range_gate_rejects) /
                                   static_cast<double>(aux.range_delta_count);
        spdlog::info(
          "lidar_concat: '{}' sweep endpoint delta over {} evaluations: "
          "mean={:.1f} ms, min={:.1f} ms, max={:.1f} ms, gate={:.1f} ms, "
          "rejected={:.1f}% — this is the quantity sweep_time_threshold gates",
          aux.topic, aux.range_delta_count, 1e3 * rd_mean,
          1e3 * aux.range_delta_min, 1e3 * aux.range_delta_max,
          1e3 * sweep_time_threshold, 100.0 * reject_frac);
        if (reject_frac > 0.10) {
          spdlog::warn(
            "lidar_concat: '{}' is failing the endpoint gate on {:.1f}% of "
            "scans (mean delta {:.1f} ms vs gate {:.1f} ms). Sweep phase is a "
            "DEPLOYMENT property — PTP disciplines clocks, not frame "
            "scheduling, and Robin W exposes no frame-trigger contract. Either "
            "widen sweep_time_threshold (keeping it well under HALF the "
            "sweep period: 50 ms at 10 FPS, 25 ms at 20 FPS) or accept "
            "front-only coverage.",
            aux.topic, 100.0 * reject_frac, 1e3 * rd_mean,
            1e3 * sweep_time_threshold);
        }
      }
    }
  }

  emit_frame_diag(total_points, *merged);

  merged->width = total_points;
  merged->height = 1;
  merged->row_step = point_step * total_points;

  // Strict guard: a REQUIRED multi-LiDAR merge that stays incomplete must not be
  // silently localized on fewer LiDARs. The primary-precondition bail-outs above
  // route through the same handler. When require_all_aux is set, an incomplete
  // merge SKIPS the scan (returns nullptr); otherwise the degraded cloud is
  // returned and localized. A fully merged scan resets the budget.
  if (merged_aux_count < aux_sensors.size()) {
    if (on_required_failure(merged_aux_count, "incomplete aux merge")) return nullptr;
  } else if (consec_fail) {
    *consec_fail = 0;
  }

  return merged;
}

struct AuxConcatConfig {
  bool enabled = false;
  LidarQualityConfig lidar_quality;
  double time_threshold = 0.05;
  double sweep_time_threshold = 0.010;
  // Bag-time bound on how long an offline reader may hold a queued primary
  // waiting for a point-coherent aux match/watermark. Once the bag stream has
  // advanced this far past the primary's enqueue time, the primary is released
  // and merged with whichever aux aligned (possibly none). This guarantees a
  // dead or gappy aux stream can only degrade coverage — it can never park
  // primaries until EOF or starve the estimator of front sweeps. 0 restores
  // immediate release (no future-sweep waiting).
  // With two_pass_point_time_join it doubles as the safety slack past a planned
  // merge's known ready time before releasing the primary incomplete.
  double future_sweep_wait_timeout = 0.150;
  // Offline readers only: pre-index every LiDAR scan's absolute point-time
  // range in a first bag pass, plan each primary's aux selection by minimum
  // endpoint-range delta (header time only as tie-break), then merge during
  // the streaming pass exactly when the planned sweeps have arrived. Primaries
  // with no in-gate aux candidate are mapped front-only immediately with the
  // miss reason recorded — never deferred to EOF. Falls back to the streaming
  // wait automatically when the primary topic lacks absolute point times.
  bool two_pass_point_time_join = true;
  int buffer_size = 200;
  std::vector<AuxLidarSensor> aux_sensors;
  // Strict merge guard (see merge_clouds). consecutive_merge_failures is mutable
  // running state that the caller passes to merge_clouds each scan.
  bool require_all_aux = true;                // true = never silently build/localize on partial coverage
  bool abort_on_merge_failure = true;         // true = stop past budget; false = keep skipping non-fatally
  int max_consecutive_aux_merge_failures = 10;
  int consecutive_merge_failures = 0;
  // P4#3: one parseable "CONCAT DEBUG | ..." INFO line per primary scan
  // (merged count, per-aux signed dt + points, total points). Default ON for
  // the offline mapping tools — this is the map-side merge evidence the run
  // reports need; ~1 line / 100 ms costs a few MB per mapping run.
  bool frame_diag_log = true;
  // Must match glim::extract_raw_points(): an explicitly opted-in FLOAT64
  // field is raw uint64 PTP epoch nanoseconds, not IEEE-754 seconds.
  bool float64_time_is_epoch_ns = false;
};

// Resolve a (possibly relative) urdf_path CWD-independently. parse_urdf_transforms
// feeds the string straight to xmlReadFile() (CWD-relative), which silently fails
// from a different working directory. Try, in order: as-given (absolute or CWD),
// then relative to the GLIM config directory, then a walk UP from the config dir
// looking for the file by relative path or basename (sensor_dome.urdf lives in
// GLIM_plusplus/config/ in the source tree and is installed into
// share/glim/config in the colcon install tree). Returns the original string if
// nothing is found, so the caller's parse failure + startup guard still fire
// with a clear error.
inline std::string resolve_urdf_path(const std::string& urdf_path) {
  namespace fs = std::filesystem;
  if (urdf_path.empty() || fs::exists(urdf_path)) {
    return urdf_path;
  }
  const std::string config_dir = glim::GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  const std::string basename = fs::path(urdf_path).filename().string();
  fs::path d = config_dir;
  for (int i = 0; i < 10; ++i) {
    if (fs::exists(d / urdf_path)) return (d / urdf_path).string();
    if (fs::exists(d / basename)) return (d / basename).string();
    if (!d.has_parent_path() || d.parent_path() == d) break;
    d = d.parent_path();
  }
  return urdf_path;
}

inline AuxConcatConfig load_aux_sensors_from_config(const glim::Config& config_sensors) {
  AuxConcatConfig out;
  out.lidar_quality.minimum_vertical_fov_deg =
    config_sensors.param<double>(
      "lidar_quality", "min_vertical_fov_deg",
      kDefaultMinimumVerticalFovDeg);
  const int minimum_valid_points =
    config_sensors.param<int>(
      "lidar_quality", "min_valid_points",
      static_cast<int>(kDefaultMinimumFovPoints));
  if (!std::isfinite(out.lidar_quality.minimum_vertical_fov_deg) ||
      out.lidar_quality.minimum_vertical_fov_deg < 25.0 ||
      out.lidar_quality.minimum_vertical_fov_deg >
        kNominalRobinWVerticalFovDeg) {
    throw std::runtime_error(
      "lidar_quality.min_vertical_fov_deg must be finite and within "
      "[25, 30] degrees for the Robin W profile");
  }
  if (minimum_valid_points < static_cast<int>(kDefaultMinimumFovPoints) ||
      minimum_valid_points > static_cast<int>(kMaximumFovSamplePoints)) {
    throw std::runtime_error(
      "lidar_quality.min_valid_points must be within [100, 20000]");
  }
  out.lidar_quality.minimum_valid_points =
    static_cast<size_t>(minimum_valid_points);
  spdlog::info(
    "lidar_quality: vertical-FOV gate active: nominal Robin W {:.1f} deg, "
    "reject robust spans below {:.1f} deg or clouds with fewer than {} valid "
    "sampled returns",
    kNominalRobinWVerticalFovDeg,
    out.lidar_quality.minimum_vertical_fov_deg,
    out.lidar_quality.minimum_valid_points);

  out.enabled = config_sensors.param<bool>("lidar_concat", "enabled", false);
  out.time_threshold = config_sensors.param<double>("lidar_concat", "time_threshold", 0.05);
  // Absolute point-time endpoint gate. This is a required, vendor-neutral
  // configuration surface; unsupported historical aliases are intentionally
  // not accepted by the Hitch profile.
  out.sweep_time_threshold =
    config_sensors.param<double>("lidar_concat", "sweep_time_threshold", 0.010);
  out.future_sweep_wait_timeout =
    config_sensors.param<double>("lidar_concat", "future_sweep_wait_timeout", 0.150);
  out.two_pass_point_time_join =
    config_sensors.param<bool>("lidar_concat", "two_pass_point_time_join", true);
  out.buffer_size = config_sensors.param<int>("lidar_concat", "buffer_size", 200);
  // [P3 FIX 2026-07-10] Configuration validation, fail LOUD (same policy as
  // aux_time_offsets): a negative/NaN threshold silently disables every aux
  // match; a nonpositive buffer_size becomes a huge size_t downstream.
  if (!std::isfinite(out.time_threshold) || out.time_threshold < 0.0) {
    throw std::runtime_error("lidar_concat: time_threshold = " + std::to_string(out.time_threshold) +
                             " is invalid (must be finite and >= 0)");
  }
  if (!std::isfinite(out.sweep_time_threshold) ||
      out.sweep_time_threshold < 0.0) {
    throw std::runtime_error(
      "lidar_concat: sweep_time_threshold = " +
      std::to_string(out.sweep_time_threshold) +
      " is invalid (must be finite and >= 0)");
  }
  if (!std::isfinite(out.future_sweep_wait_timeout) ||
      out.future_sweep_wait_timeout < 0.0) {
    throw std::runtime_error(
      "lidar_concat: future_sweep_wait_timeout = " +
      std::to_string(out.future_sweep_wait_timeout) +
      " is invalid (must be finite and >= 0)");
  }
  if (out.buffer_size <= 0) {
    throw std::runtime_error("lidar_concat: buffer_size = " + std::to_string(out.buffer_size) +
                             " is invalid (must be > 0)");
  }
  out.require_all_aux = config_sensors.param<bool>("lidar_concat", "require_all_aux", true);
  out.abort_on_merge_failure = config_sensors.param<bool>("lidar_concat", "abort_on_merge_failure", true);
  out.max_consecutive_aux_merge_failures = config_sensors.param<int>("lidar_concat", "max_consecutive_aux_merge_failures", 10);
  out.frame_diag_log = config_sensors.param<bool>("lidar_concat", "frame_diag_log", true);
  out.float64_time_is_epoch_ns =
    config_sensors.param<bool>("sensors", "float64_time_is_epoch_ns", false);

  if (!out.enabled) {
    return out;
  }

  const auto aux_topics = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_topics", {});
  const auto legacy_time_offsets =
    config_sensors.param<std::vector<double>>("lidar_concat", "aux_time_offsets", {});
  const auto aux_match_time_offsets =
    config_sensors.param<std::vector<double>>("lidar_concat", "aux_match_time_offsets", {});
  auto aux_point_time_offsets =
    config_sensors.param<std::vector<double>>("lidar_concat", "aux_point_time_offsets", {});
  if (aux_point_time_offsets.empty() && !legacy_time_offsets.empty()) {
    aux_point_time_offsets = legacy_time_offsets;
    spdlog::warn(
      "lidar_concat: deprecated aux_time_offsets is being treated as "
      "aux_point_time_offsets; migrate the run config to separate header "
      "phase from residual point-clock correction");
  }

  // A REQUIRED merge that is enabled with no aux topics is a config error. Hard-fail
  // only when the strict path is also set to abort; otherwise it is handled at
  // runtime (the merge simply never completes and scans are skipped non-fatally).
  if (out.require_all_aux && out.abort_on_merge_failure && aux_topics.empty()) {
    const std::string msg = "lidar_concat: enabled with require_all_aux=true, abort_on_merge_failure=true and no aux_topics; refusing to start";
    spdlog::critical(msg);
    throw std::runtime_error(msg);
  }

  const std::string urdf_path = config_sensors.param<std::string>("lidar_concat", "urdf_path", "");
  const std::string primary_frame = config_sensors.param<std::string>("lidar_concat", "primary_frame", "");
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool use_urdf = !urdf_path.empty() && !primary_frame.empty();

  if (use_urdf) {
    const std::string resolved_urdf = resolve_urdf_path(urdf_path);
    try {
      urdf_transforms = glim::parse_urdf_transforms(resolved_urdf);
      spdlog::info("lidar_concat: loaded URDF from '{}' (config urdf_path='{}', primary_frame={})",
                   resolved_urdf, urdf_path, primary_frame);
    } catch (const std::exception& e) {
      spdlog::error("lidar_concat: failed to parse URDF '{}' (from urdf_path='{}'): {}", resolved_urdf, urdf_path, e.what());
      use_urdf = false;
    }
  }

  const auto aux_frames = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_frames", {});
  if (use_urdf && aux_frames.size() != aux_topics.size()) {
    spdlog::error("lidar_concat: aux_frames size ({}) must match aux_topics size ({})", aux_frames.size(), aux_topics.size());
    use_urdf = false;
  }

  for (size_t i = 0; i < aux_topics.size(); i++) {
    const auto& topic = aux_topics[i];
    AuxLidarSensor sensor;
    sensor.topic = topic;
    sensor.buffer_size = out.buffer_size;
    sensor.match_time_offset =
      (i < aux_match_time_offsets.size()) ? aux_match_time_offsets[i] : 0.0;
    sensor.point_time_offset =
      (i < aux_point_time_offsets.size()) ? aux_point_time_offsets[i] : 0.0;
    // [P3 FIX 2026-07-10] Configuration validation, fail LOUD: a NaN offset
    // made every match-window comparison false, silently disabling that aux
    // LiDAR for the whole run (the existing non-finite guard sits after a
    // successful match — unreachable for NaN). Offsets are clock corrections:
    // |off| >= 1 s is a config typo, not a measurement.
    if (!std::isfinite(sensor.match_time_offset) ||
        std::abs(sensor.match_time_offset) >= 1.0) {
      throw std::runtime_error(
        "lidar_concat: aux_match_time_offsets[" + std::to_string(i) + "] = " +
        std::to_string(sensor.match_time_offset) +
        " is invalid (must be finite, |off| < 1 s)");
    }
    if (!std::isfinite(sensor.point_time_offset) ||
        std::abs(sensor.point_time_offset) >= 1.0) {
      throw std::runtime_error(
        "lidar_concat: aux_point_time_offsets[" + std::to_string(i) + "] = " +
        std::to_string(sensor.point_time_offset) +
        " is invalid (must be finite, |off| < 1 s)");
    }

    if (use_urdf) {
      const std::string& aux_frame = aux_frames[i];
      try {
        sensor.T_primary_sensor = glim::compute_transform(urdf_transforms, primary_frame, aux_frame);
        std::stringstream ss;
        ss << sensor.T_primary_sensor.matrix();
        spdlog::info("lidar_concat: T_{}_{}:\n{}", primary_frame, aux_frame, ss.str());
      } catch (const std::exception& e) {
        spdlog::error("lidar_concat: failed to compute transform {} -> {}: {}", primary_frame, aux_frame, e.what());
        continue;
      }
    } else {
      std::string key = topic;
      for (auto& c : key) {
        if (c == '/') c = '_';
      }
      if (!key.empty() && key[0] == '_') key = key.substr(1);
      key = "T_primary_" + key;

      auto flat = config_sensors.param<std::vector<double>>("lidar_concat", key);
      if (!flat || flat->size() != 16) {
        spdlog::error("lidar_concat: missing or invalid transform '{}' for topic '{}'", key, topic);
        continue;
      }

      Eigen::Matrix4d mat;
      for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
          mat(r, c) = (*flat)[r * 4 + c];
      sensor.T_primary_sensor = Eigen::Isometry3d(mat);
    }

    spdlog::info(
      "lidar_concat: auxiliary sensor {} enabled "
      "(header_match_offset={:+.6f}s, point_clock_offset={:+.6f}s)",
      sensor.topic, sensor.match_time_offset, sensor.point_time_offset);
    out.aux_sensors.push_back(std::move(sensor));
  }
  spdlog::info(
    "lidar_concat: {} auxiliary sensors, header_threshold={:.3f}s, "
    "point_range_threshold={:.3f}s",
    out.aux_sensors.size(), out.time_threshold, out.sweep_time_threshold);

  // Startup strict guard: if a complete multi-LiDAR merge is REQUIRED and set to
  // abort, but some aux sensors could not even be set up (missing/invalid extrinsic,
  // bad URDF chain), fail loudly at load. With abort_on_merge_failure=false the merge
  // just never completes at runtime and scans are skipped non-fatally instead.
  if (out.require_all_aux && out.abort_on_merge_failure && out.aux_sensors.size() < aux_topics.size()) {
    const std::string msg =
      "lidar_concat: require_all_aux=true and abort_on_merge_failure=true but only " + std::to_string(out.aux_sensors.size()) + "/" +
      std::to_string(aux_topics.size()) + " aux sensors resolved at startup (check URDF/static extrinsics and aux_frames). Refusing to start.";
    spdlog::critical(msg);
    throw std::runtime_error(msg);
  }

  return out;
}

}  // namespace glim_ros
