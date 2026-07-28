#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <glim_ros/lidar_concat.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace {

sensor_msgs::msg::PointCloud2::SharedPtr cloud(
  uint64_t min_ns, uint64_t max_ns, double header_s) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = static_cast<int32_t>(header_s);
  msg->header.stamp.nanosec = static_cast<uint32_t>(
    (header_s - static_cast<double>(msg->header.stamp.sec)) * 1.0e9);
  sensor_msgs::msg::PointField time;
  time.name = "timestamp";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::UINT8;
  time.count = 8;
  msg->fields = {time};
  msg->height = 1;
  msg->width = 2;
  msg->point_step = 8;
  msg->row_step = 16;
  msg->data.resize(16);
  std::memcpy(msg->data.data(), &min_ns, sizeof(min_ns));
  std::memcpy(msg->data.data() + 8, &max_ns, sizeof(max_ns));
  return msg;
}

TEST(LidarConcatPointTime, DecodesAbsoluteRangeOnceWhenBuffered) {
  const auto buffered = glim_ros::buffer_aux_cloud(
    cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.0));
  ASSERT_TRUE(buffered.point_time_range.valid);
  EXPECT_EQ(buffered.point_time_range.min_ns, 1'000'000'000ULL);
  EXPECT_EQ(buffered.point_time_range.max_ns, 1'049'000'000ULL);
  EXPECT_EQ(buffered.point_time_range.count, 2U);
}

TEST(LidarConcatPointTime, SplitBagSeekAlignsFromFirstDeliveredLidarRecord) {
  std::vector<double> indexed_bag_times;
  for (int i = 0; i < 30; ++i) {
    indexed_bag_times.push_back(200.0 + i);
  }

  // An independent reader can seek past split 0, then open split 1 from its
  // beginning. Seeding from the requested time would skip the whole plan.
  EXPECT_EQ(glim_ros::first_ordinal_at_or_after(indexed_bag_times, 230.0), 30U);
  EXPECT_EQ(glim_ros::first_ordinal_at_or_after(indexed_bag_times, 200.0), 0U);

  // The same lazy lookup preserves ordinary seeks that resume within a split.
  EXPECT_EQ(glim_ros::first_ordinal_at_or_after(indexed_bag_times, 212.0), 12U);
}

TEST(LidarConcatPointTime, OnlineConcatRemainsRefusedWithoutFutureSweepRelease) {
  EXPECT_TRUE(glim_ros::online_concat_configuration_supported(false, false));
  EXPECT_TRUE(glim_ros::online_concat_configuration_supported(false, true));
  EXPECT_TRUE(glim_ros::online_concat_configuration_supported(true, false));
  EXPECT_FALSE(glim_ros::online_concat_configuration_supported(true, true));
}

TEST(LidarConcatPointTime, Float64EpochNsContractPreservesRawAuxBytes) {
  // The driver labels the field FLOAT64 but stores uint64 epoch-ns bits. Verify
  // that the explicit contract both enables range matching and prevents the
  // generic FLOAT64 header-phase arithmetic from corrupting those bytes.
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::msg::PointField time;
  time.name = "timestamp";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::FLOAT64;
  time.count = 1;
  msg->fields = {time};
  msg->height = 1;
  msg->width = 2;
  msg->point_step = 8;
  msg->row_step = 16;
  msg->data.resize(16);
  const uint64_t first = 1'700'000'000'000'000'000ULL;
  const uint64_t last = first + 50'000'000ULL;
  std::memcpy(msg->data.data(), &first, sizeof(first));
  std::memcpy(msg->data.data() + 8, &last, sizeof(last));

  EXPECT_FALSE(glim_ros::decode_point_time_range(*msg).valid);
  const auto range = glim_ros::decode_point_time_range(*msg, true);
  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.min_ns, first);
  EXPECT_EQ(range.max_ns, last);

  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, 8, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    0.080, 0.0, true);
  uint64_t shifted_first = 0, shifted_last = 0;
  std::memcpy(&shifted_first, bytes.data(), sizeof(shifted_first));
  std::memcpy(&shifted_last, bytes.data() + 8, sizeof(shifted_last));
  EXPECT_EQ(shifted_first, first);
  EXPECT_EQ(shifted_last, last);
}

sensor_msgs::msg::PointCloud2::SharedPtr float64_time_cloud(
  const std::vector<double>& times_sec, double header_s) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = static_cast<int32_t>(header_s);
  msg->header.stamp.nanosec = static_cast<uint32_t>(
    (header_s - static_cast<double>(msg->header.stamp.sec)) * 1.0e9);
  sensor_msgs::msg::PointField time;
  time.name = "timestamp";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::FLOAT64;
  time.count = 1;
  msg->fields = {time};
  msg->height = 1;
  msg->width = static_cast<uint32_t>(times_sec.size());
  msg->point_step = 8;
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step);
  for (size_t i = 0; i < times_sec.size(); ++i) {
    std::memcpy(msg->data.data() + i * 8, &times_sec[i], sizeof(double));
  }
  return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr robin_w_cloud(
  const std::vector<double>& times_sec, double header_s) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = static_cast<int32_t>(header_s);
  msg->header.stamp.nanosec = static_cast<uint32_t>(
    (header_s - static_cast<double>(msg->header.stamp.sec)) * 1.0e9);
  sensor_msgs::msg::PointField time;
  time.name = "timestamp";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::FLOAT64;
  time.count = 1;
  msg->fields = {time};
  msg->height = 1;
  msg->width = static_cast<uint32_t>(times_sec.size());
  msg->point_step = 8;
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step);
  for (size_t i = 0; i < times_sec.size(); ++i) {
    std::memcpy(msg->data.data() + i * 8, &times_sec[i], sizeof(double));
  }
  return msg;
}

template <typename TimeT>
sensor_msgs::msg::PointCloud2 xyz_time_cloud(
  const std::string& time_name,
  uint8_t time_datatype,
  const std::vector<TimeT>& times,
  double header_s) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = static_cast<int32_t>(header_s);
  msg.header.stamp.nanosec = static_cast<uint32_t>(
    (header_s - static_cast<double>(msg.header.stamp.sec)) * 1.0e9);
  msg.fields.resize(4);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1] = msg.fields[0];
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[2] = msg.fields[0];
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[3].name = time_name;
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = time_datatype;
  msg.fields[3].count = 1;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(times.size());
  msg.point_step = 16 + sizeof(TimeT);
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < times.size(); ++i) {
    const float xyz[3] = {static_cast<float>(i), 0.0f, 0.0f};
    std::memcpy(msg.data.data() + i * msg.point_step, xyz, sizeof(xyz));
    std::memcpy(
      msg.data.data() + i * msg.point_step + 16,
      &times[i], sizeof(TimeT));
  }
  return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr fov_primary_cloud(
    double half_span_deg) {
  std::vector<double> times(101);
  for (size_t i = 0; i < times.size(); ++i) {
    times[i] = 1'700'000'000.0 + static_cast<double>(i) * 1.0e-5;
  }
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(
      xyz_time_cloud<double>(
          "timestamp", sensor_msgs::msg::PointField::FLOAT64,
          times, times.front()));
  for (size_t i = 0; i < times.size(); ++i) {
    const double elevation =
        (-half_span_deg +
         2.0 * half_span_deg * static_cast<double>(i) /
             static_cast<double>(times.size() - 1)) *
        3.14159265358979323846 / 180.0;
    const float x = static_cast<float>(10.0 * std::cos(elevation));
    const float z = static_cast<float>(10.0 * std::sin(elevation));
    std::memcpy(msg->data.data() + i * msg->point_step, &x, sizeof(x));
    std::memcpy(msg->data.data() + i * msg->point_step + 8, &z, sizeof(z));
  }
  return msg;
}

TEST(LidarConcatPointTime, PrimaryFovFailureUsesStrictMergeHandler) {
  std::vector<glim_ros::AuxLidarSensor> aux_sensors;
  glim_ros::LidarQualityConfig quality;
  int consecutive_failures = 0;

  EXPECT_THROW(
      glim_ros::merge_clouds(
          fov_primary_cloud(10.0), aux_sensors, 0.05, quality,
          true, 0, &consecutive_failures, true),
      std::runtime_error);
  EXPECT_EQ(consecutive_failures, 1);
  EXPECT_EQ(quality.primary_reject_count, 1U);
}

TEST(LidarConcatPointTime, StartupFovFailureCannotBeBypassedByDegradedMode) {
  std::vector<glim_ros::AuxLidarSensor> aux_sensors;
  glim_ros::LidarQualityConfig quality;
  int consecutive_failures = 0;
  const auto primary = fov_primary_cloud(10.0);

  const auto result = glim_ros::merge_clouds(
      primary, aux_sensors, 0.05, quality,
      false, 0, &consecutive_failures, true);
  EXPECT_EQ(result, nullptr);
  EXPECT_EQ(consecutive_failures, 0);
  EXPECT_EQ(quality.primary_reject_count, 1U);
}

TEST(LidarConcatPointTime, StartupFovValidationIsOneShot) {
  std::vector<glim_ros::AuxLidarSensor> aux_sensors;
  glim_ros::LidarQualityConfig quality;
  int consecutive_failures = 0;

  EXPECT_NE(
      glim_ros::merge_clouds(
          fov_primary_cloud(15.0), aux_sensors, 0.05, quality,
          false, 0, &consecutive_failures, true),
      nullptr);
  EXPECT_TRUE(quality.primary_validated);
  EXPECT_TRUE(quality.startup_validation_complete);

  // Runtime clouds are not re-measured after startup validation.
  EXPECT_NE(
      glim_ros::merge_clouds(
          fov_primary_cloud(10.0), aux_sensors, 0.05, quality,
          false, 0, &consecutive_failures, true),
      nullptr);
  EXPECT_EQ(quality.primary_reject_count, 0U);
}

TEST(LidarConcatPointTime, RobinWContractIsFloat64AbsoluteTime) {
  const double t0 = 1'700'000'000.0;
  auto msg = robin_w_cloud({t0, t0 + 0.049}, t0);
  ASSERT_EQ(msg->fields.size(), 1U);
  EXPECT_EQ(msg->fields.front().name, "timestamp");
  EXPECT_EQ(msg->fields.front().datatype, sensor_msgs::msg::PointField::FLOAT64);
  EXPECT_EQ(msg->fields.front().count, 1U);

  const auto range = glim_ros::decode_point_time_range(*msg, false);
  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.count, 2U);
  EXPECT_EQ(range.zero_count, 0U);
  EXPECT_EQ(range.min_ns, static_cast<uint64_t>(t0 * 1e9));

  const auto with_zero = glim_ros::decode_point_time_range(
    *robin_w_cloud({0.0, t0, t0 + 0.049}, t0), false);
  ASSERT_TRUE(with_zero.valid);
  EXPECT_EQ(with_zero.count, 2U);
  EXPECT_EQ(with_zero.zero_count, 1U);

  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, msg->point_step, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    /*dt=*/0.080, /*abs_clock_shift_s=*/0.0, /*float64_time_is_epoch_ns=*/false);
  msg->data = bytes;
  double first = 0.0, last = 0.0;
  std::memcpy(&first, bytes.data(), sizeof(double));
  std::memcpy(&last, bytes.data() + 8, sizeof(double));
  EXPECT_DOUBLE_EQ(first, t0);
  EXPECT_DOUBLE_EQ(last, t0 + 0.049);
  EXPECT_NEAR(glim_ros::cloud_time_span_seconds(*msg), 0.049, 1e-6);
}

TEST(LidarConcatPointTime, RejectsPaddedAbsoluteTimeLayout) {
  const double t0 = 1'700'000'000.0;
  auto msg = robin_w_cloud({t0, t0 + 0.049}, t0);
  msg->row_step += 8;
  msg->data.resize(msg->row_step);
  EXPECT_FALSE(glim_ros::decode_point_time_range(*msg, false).valid);
}

TEST(LidarConcatPointTime, CanonicalVendorUnitsDecodeCorrectly) {
  const auto ouster = glim::extract_raw_points(
    xyz_time_cloud<uint32_t>(
      "t", sensor_msgs::msg::PointField::UINT32,
      {0U, 50'000'000U}, 100.0),
    "intensity", "ring");
  ASSERT_NE(ouster, nullptr);
  EXPECT_NEAR(ouster->times.back(), 0.050, 1e-9);

  const auto velodyne = glim::extract_raw_points(
    xyz_time_cloud<float>(
      "time", sensor_msgs::msg::PointField::FLOAT32,
      {0.0f, 0.050f}, 100.0),
    "intensity", "ring");
  ASSERT_NE(velodyne, nullptr);
  EXPECT_NEAR(velodyne->times.back(), 0.050, 1e-6);

  const auto robin_w = glim::extract_raw_points(
    xyz_time_cloud<double>(
      "timestamp", sensor_msgs::msg::PointField::FLOAT64,
      {1'700'000'000.0, 1'700'000'000.050}, 1'700'000'000.0),
    "intensity", "ring");
  ASSERT_NE(robin_w, nullptr);
  EXPECT_NEAR(
    robin_w->times.back() - robin_w->times.front(), 0.050, 1e-6);

  const double hesai_t0 = 1'700'000'000.0;
  const auto hesai = glim::extract_raw_points(
    xyz_time_cloud<double>(
      "timestamp", sensor_msgs::msg::PointField::FLOAT64,
      {hesai_t0, hesai_t0 + 0.050}, hesai_t0),
    "intensity", "ring");
  ASSERT_NE(hesai, nullptr);
  EXPECT_NEAR(hesai->times.front(), hesai_t0, 1e-6);
  EXPECT_NEAR(hesai->times.back() - hesai->times.front(), 0.050, 1e-6);
}

TEST(LidarConcatPointTime, HesaiFloat64AbsoluteSecondsDecodeToRange) {
  // Hesai FLOAT64 absolute Unix seconds need no raw-epoch-ns contract flag.
  const double t0 = 1'700'000'000.000;
  const auto msg = float64_time_cloud({0.0, t0, t0 + 0.049}, t0);
  const auto range = glim_ros::decode_point_time_range(*msg, false);
  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.count, 2U);
  EXPECT_EQ(range.min_ns, static_cast<uint64_t>(t0 * 1e9));
  EXPECT_EQ(range.max_ns, static_cast<uint64_t>((t0 + 0.049) * 1e9));
}

TEST(LidarConcatPointTime, RelativeFloat64SecondsRejectedFailClosed) {
  // Scan-RELATIVE FLOAT64 seconds must NOT be promoted to an absolute range:
  // the whole decode fails closed so the generic header fallback is used.
  const auto msg = float64_time_cloud({0.0, 0.010, 0.049}, 1.0);
  EXPECT_FALSE(glim_ros::decode_point_time_range(*msg, false).valid);
}

TEST(LidarConcatPointTime, HesaiAbsoluteSecondsNotRebasedButCorrected) {
  // The header-relative dt rebase must never touch an absolute-seconds axis
  // (double-apply), but the configured residual point-clock correction IS
  // applied, in the seconds domain — parity with the raw epoch-ns branch.
  // A leading zero sentinel must neither defeat the gate nor be shifted.
  const double t0 = 1'700'000'000.000;
  auto msg = float64_time_cloud({0.0, t0, t0 + 0.049}, t0);
  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, 8, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    /*dt=*/0.080, /*abs_clock_shift_s=*/-0.010, /*float64_time_is_epoch_ns=*/false);
  double p0 = 0.0, p1 = 0.0, p2 = 0.0;
  std::memcpy(&p0, bytes.data(), sizeof(double));
  std::memcpy(&p1, bytes.data() + 8, sizeof(double));
  std::memcpy(&p2, bytes.data() + 16, sizeof(double));
  EXPECT_DOUBLE_EQ(p0, 0.0);              // sentinel preserved
  EXPECT_DOUBLE_EQ(p1, t0 - 0.010);       // corrected, NOT dt-rebased
  EXPECT_DOUBLE_EQ(p2, t0 + 0.049 - 0.010);
}

TEST(LidarConcatPointTime, Float64AllSentinelCloudLeftUntouched) {
  // A FLOAT64 cloud whose every point is the 0.0 "no valid time" sentinel has
  // nothing to rebase: the relative-shift loop must not rewrite sentinels
  // into dt.
  auto msg = float64_time_cloud({0.0, 0.0}, 1.0);
  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, 8, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    /*dt=*/0.080, /*abs_clock_shift_s=*/0.0, /*float64_time_is_epoch_ns=*/false);
  double p0 = 1.0, p1 = 1.0;
  std::memcpy(&p0, bytes.data(), sizeof(double));
  std::memcpy(&p1, bytes.data() + 8, sizeof(double));
  EXPECT_DOUBLE_EQ(p0, 0.0);
  EXPECT_DOUBLE_EQ(p1, 0.0);
}

TEST(LidarConcatPointTime, Float64MixedMagnitudeCloudFailsTowardNoShift) {
  // A corrupt cloud mixing a relative-looking value with absolute epoch
  // seconds must classify ABSOLUTE (any epoch-scaled sample decides): adding
  // dt to even one absolute point is the double-apply the gate prevents.
  // The range decoder meanwhile fails such a cloud closed (invalid range).
  const double t0 = 1'700'000'000.0;
  auto msg = float64_time_cloud({0.010, t0}, t0);
  EXPECT_FALSE(glim_ros::decode_point_time_range(*msg, false).valid);
  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, 8, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    /*dt=*/0.080, /*abs_clock_shift_s=*/0.0, /*float64_time_is_epoch_ns=*/false);
  double p0 = 0.0, p1 = 0.0;
  std::memcpy(&p0, bytes.data(), sizeof(double));
  std::memcpy(&p1, bytes.data() + 8, sizeof(double));
  EXPECT_DOUBLE_EQ(p0, 0.010);  // untouched — no dt applied anywhere
  EXPECT_DOUBLE_EQ(p1, t0);
}

TEST(LidarConcatPointTime, HesaiPointTimesDriveSweepSelection) {
  // End-to-end: Hesai FLOAT64-seconds aux clouds get point-time-coherent
  // sweep selection even when their headers are misleading.
  const double t0 = 1'700'000'000.0;
  std::deque<glim_ros::BufferedAuxCloud> candidates;
  candidates.push_back(glim_ros::buffer_aux_cloud(
    float64_time_cloud({t0 - 0.100, t0 - 0.051}, t0 - 0.008), false));
  candidates.push_back(glim_ros::buffer_aux_cloud(
    float64_time_cloud({t0, t0 + 0.049}, t0 + 0.092), false));
  const auto primary_range = glim_ros::decode_point_time_range(
    *float64_time_cloud({t0, t0 + 0.049}, t0), false);
  ASSERT_TRUE(primary_range.valid);

  const auto match = glim_ros::find_closest_sweep(
    candidates, primary_range, 0.0, t0, 0.0);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->msg, candidates[1].msg);
  EXPECT_DOUBLE_EQ(match->range_delta_s, 0.0);
}

TEST(LidarConcatPointTime, LivoxFloat64NumericNanosecondsUseOneENineScale) {
  const double t0_ns = 1'700'000'000'000'000'000.0;
  auto msg = float64_time_cloud({t0_ns, t0_ns + 50'000'000.0}, 1'700'000'000.0);
  const auto range = glim_ros::decode_point_time_range(*msg, false);
  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.count, 2U);
  EXPECT_EQ(range.min_ns, static_cast<uint64_t>(t0_ns));
  EXPECT_EQ(range.max_ns, static_cast<uint64_t>(t0_ns + 50'000'000.0));
  EXPECT_NEAR(glim_ros::cloud_time_span_seconds(*msg), 0.050, 1e-6);

  auto bytes = msg->data;
  glim_ros::shift_cloud_timestamps(
    bytes, 8, 0, sensor_msgs::msg::PointField::FLOAT64, 1,
    /*dt=*/0.080, /*abs_clock_shift_s=*/-0.010, /*float64_time_is_epoch_ns=*/false);
  double shifted = 0.0;
  std::memcpy(&shifted, bytes.data(), sizeof(double));
  EXPECT_NEAR((shifted - t0_ns) * 1e-9, -0.010, 1e-6);
}

TEST(LidarConcatPointTime, LivoxNumericNanosecondsSurviveRosCloudConversion) {
  const double header_s = 1'700'000'000.0;
  const double t0_ns = header_s * 1e9;
  const auto msg = xyz_time_cloud<double>(
    "timestamp", sensor_msgs::msg::PointField::FLOAT64,
    {t0_ns, t0_ns + 50'000'000.0}, header_s);
  const auto points = glim::extract_raw_points(msg, "intensity", "ring");
  ASSERT_NE(points, nullptr);
  ASSERT_EQ(points->times.size(), 2U);
  EXPECT_NEAR(points->times.front() * 1e-9, header_s, 1e-6);
  EXPECT_NEAR((points->times.back() - points->times.front()) * 1e-9, 0.050, 1e-6);
}

TEST(LidarConcatPointTime, PointTimesOverrideMisleadingHeaderProximity) {
  std::deque<glim_ros::BufferedAuxCloud> candidates;
  candidates.push_back(glim_ros::buffer_aux_cloud(
    cloud(900'000'000ULL, 949'000'000ULL, 0.992)));
  candidates.push_back(glim_ros::buffer_aux_cloud(
    cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.092)));
  const glim_ros::PointTimeRangeNs primary{
    true, 1'000'000'000ULL, 1'049'000'000ULL, 2};

  const auto match = glim_ros::find_closest_sweep(
    candidates, primary, 0.0, 1.0, 0.0);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->msg, candidates[1].msg);
  EXPECT_DOUBLE_EQ(match->range_delta_s, 0.0);
  EXPECT_NEAR(match->header_abs_delta_s, 0.092, 1.0e-9);
}

TEST(LidarConcatPointTime, PointClockCorrectionIsSeparateFromHeaderPhase) {
  const glim_ros::PointTimeRangeNs uncorrected{
    true, 1'010'000'000ULL, 1'059'000'000ULL, 2};
  const auto corrected = glim_ros::shifted_range(uncorrected, -0.010);
  EXPECT_EQ(corrected.min_ns, 1'000'000'000ULL);
  EXPECT_EQ(corrected.max_ns, 1'049'000'000ULL);
}

TEST(LidarConcatPointTime, OfflineWatermarkWaitsForFutureSweep) {
  auto primary = cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.0);
  glim_ros::AuxLidarSensor aux;
  aux.buffer_size = 10;
  aux.buffer.push_back(glim_ros::buffer_aux_cloud(
    cloud(900'000'000ULL, 949'000'000ULL, 0.992)));
  std::vector<glim_ros::AuxLidarSensor> sensors{aux};

  EXPECT_FALSE(glim_ros::aux_buffers_ready_for_primary(
    *primary, sensors, 0.010));

  sensors[0].buffer.push_back(glim_ros::buffer_aux_cloud(
    cloud(1'010'000'001ULL, 1'059'000'001ULL, 1.192)));
  EXPECT_TRUE(glim_ros::aux_buffers_ready_for_primary(
    *primary, sensors, 0.010));
}

TEST(LidarConcatPointTime, RelativeTimeHeaderFallbackWaitsForFutureSweep) {
  auto primary = cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.0);
  // Make the primary's point-time range undecodable, as for an ordinary
  // FLOAT64 relative-seconds cloud when the raw-epoch opt-in is disabled.
  primary->fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
  primary->fields.front().count = 1;

  glim_ros::AuxLidarSensor aux;
  aux.buffer_size = 10;
  auto past = cloud(900'000'000ULL, 949'000'000ULL, 0.94);
  past->fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
  past->fields.front().count = 1;
  aux.buffer.push_back(glim_ros::buffer_aux_cloud(past));
  std::vector<glim_ros::AuxLidarSensor> sensors{aux};

  EXPECT_FALSE(glim_ros::aux_buffers_ready_for_primary(*primary, sensors, 0.010));

  auto future = cloud(1'010'000'001ULL, 1'059'000'001ULL, 1.02);
  future->fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
  future->fields.front().count = 1;
  sensors[0].buffer.push_back(glim_ros::buffer_aux_cloud(future));
  EXPECT_TRUE(glim_ros::aux_buffers_ready_for_primary(*primary, sensors, 0.010));
}

}  // namespace
