#include "gicp_localization/seyond_timestamp.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace {

sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<double>& timestamps,
    uint8_t datatype = sensor_msgs::msg::PointField::FLOAT64,
    const std::string& field_name = "timestamp") {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = 1770000000;
  msg.header.stamp.nanosec = 123456000;
  msg.width = static_cast<uint32_t>(timestamps.size());
  msg.height = 1;
  msg.is_bigendian = false;
  msg.is_dense = true;

  sensor_msgs::msg::PointField field;
  field.name = field_name;
  field.offset = 0;
  field.datatype = datatype;
  field.count = 1;
  msg.fields.push_back(field);

  msg.point_step =
      datatype == sensor_msgs::msg::PointField::FLOAT64 ? sizeof(double)
                                                        : sizeof(float);
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < timestamps.size(); ++i) {
    if (datatype == sensor_msgs::msg::PointField::FLOAT64) {
      std::memcpy(
          msg.data.data() + i * msg.point_step, &timestamps[i],
          sizeof(double));
    } else {
      const float value = static_cast<float>(timestamps[i]);
      std::memcpy(
          msg.data.data() + i * msg.point_step, &value, sizeof(float));
    }
  }
  return msg;
}

double headerSeconds(const sensor_msgs::msg::PointCloud2& msg) {
  return static_cast<double>(msg.header.stamp.sec) +
         static_cast<double>(msg.header.stamp.nanosec) * 1.0e-9;
}

TEST(SeyondTimestamp, AcceptsAbsoluteFloat64AndPreservesTenMicroseconds) {
  auto msg = makeCloud({});
  const double start = headerSeconds(msg);
  msg = makeCloud({start, start + 10.0e-6, start + 99.99e-3});

  gicp_localization::SeyondPointTimeRange range;
  ASSERT_TRUE(gicp_localization::seyondCloudTimeContractValid(
      msg, gicp_localization::kSeyondFrameDurationSecondsDefault, &range));
  EXPECT_TRUE(range.valid);
  EXPECT_EQ(range.count, 3U);
  EXPECT_NEAR(range.min_s, start, 1.0e-9);
  // 0.25 us, not 0.1 us: one ulp at a 2026 Unix epoch is already 2.38e-7 s,
  // so the difference of two stored doubles carries ~1.1e-7 of representation
  // error here. The tighter bound was unsatisfiable by ANY implementation.
  // Same reasoning as the sibling assertion below.
  EXPECT_NEAR(range.max_s - range.min_s, 99.99e-3, 0.25e-6);

  const auto decoded = gicp_localization::decodeSeyondPointTimeRange(
      makeCloud({start, start + 10.0e-6}));
  ASSERT_TRUE(decoded.valid);
  // At current Unix epochs a double's spacing is about 0.24 us. The 10 us
  // source quantum remains clearly distinguishable without pretending that
  // subtracting two epoch-scale doubles has nanosecond precision.
  EXPECT_NEAR(decoded.max_s - decoded.min_s, 10.0e-6, 0.25e-6);
}

TEST(SeyondTimestamp, RejectsRelativeOrFloat32Timestamp) {
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      makeCloud({0.0, 10.0e-6, 99.99e-3})));

  auto template_msg = makeCloud({});
  const double start = headerSeconds(template_msg);
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      makeCloud(
          {start, start + 10.0e-6},
          sensor_msgs::msg::PointField::FLOAT32)));
}

TEST(SeyondTimestamp, RejectsMalformedAndOutOfFrameValues) {
  auto template_msg = makeCloud({});
  const double start = headerSeconds(template_msg);

  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      makeCloud({start, std::numeric_limits<double>::quiet_NaN()})));
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      makeCloud({start, start + 101.0e-3})));
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      makeCloud({start}, sensor_msgs::msg::PointField::FLOAT64, "time")));

  auto padded = makeCloud({start, start + 10.0e-6});
  padded.row_step += sizeof(double);
  padded.data.resize(padded.row_step);
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(padded));
}

TEST(SeyondTimestamp, ComparesAbsoluteSweepEndpoints) {
  const gicp_localization::SeyondPointTimeRange primary{
      true, 1770000000.0, 1770000000.1, 2U};
  const gicp_localization::SeyondPointTimeRange aux{
      true, 1770000000.01, 1770000000.11, 2U};

  EXPECT_NEAR(
      gicp_localization::pointTimeEndpointDelta(primary, aux), 0.01, 1.0e-7);
  EXPECT_TRUE(std::isinf(gicp_localization::pointTimeEndpointDelta(
      primary, gicp_localization::SeyondPointTimeRange{})));
}

TEST(SeyondTimestamp, RejectsAbsoluteZeroSamples) {
  auto template_msg = makeCloud({});
  const double start = headerSeconds(template_msg);

  // The official driver publishes only parsed points and hydrates each point as
  // packet_start + ts_10us. Absolute zero is not part of that ROS wire contract.
  auto msg = makeCloud({start, 0.0, start + 10.0e-6, start + 50.0e-3});
  gicp_localization::SeyondPointTimeRange range;
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      msg, gicp_localization::kSeyondFrameDurationSecondsDefault, &range));
  EXPECT_EQ(range.count, 3U);
  EXPECT_EQ(range.zero_timestamp_count, 1U);
  EXPECT_NEAR(range.min_s, start, 1.0e-9);
  EXPECT_NEAR(range.max_s, start + 50.0e-3, 1.0e-6);

  // The diagnostic range remains useful even though the cloud fails closed.
  EXPECT_GT(range.min_s, 1.0e6);

  // An all-zero sweep has no time axis: invalid, but the count explains why.
  const auto empty = gicp_localization::decodeSeyondPointTimeRange(
      makeCloud({0.0, 0.0, 0.0}));
  EXPECT_FALSE(empty.valid);
  EXPECT_EQ(empty.count, 0U);
  EXPECT_EQ(empty.zero_timestamp_count, 3U);

  // A NON-zero out-of-band value is a different failure — wrong encoding, not a
  // dropped return — and stays fatal.
  const auto wrong_encoding = gicp_localization::decodeSeyondPointTimeRange(
      makeCloud({start, 12.5}));
  EXPECT_FALSE(wrong_encoding.valid);
}

TEST(SeyondTimestamp, FrameDurationBoundsFollowTheConfiguredRate) {
  auto template_msg = makeCloud({});
  const double start = headerSeconds(template_msg);

  // Two fused 20 FPS frames: a real fault, and invisible at the 10 FPS default.
  auto fused = makeCloud({start, start + 99.0e-3});
  EXPECT_TRUE(gicp_localization::seyondCloudTimeContractValid(fused, 0.100))
      << "the 10 FPS default is deliberately permissive";
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(fused, 0.050))
      << "at the real 20 FPS period the same sweep must be rejected";

  // An honest 20 FPS frame still passes at the 20 FPS period.
  auto honest = makeCloud({start, start + 49.0e-3});
  EXPECT_TRUE(gicp_localization::seyondCloudTimeContractValid(honest, 0.050));

  // A nonsense period fails closed rather than producing nonsense bounds.
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(honest, 0.0));
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(honest, -1.0));
  EXPECT_FALSE(gicp_localization::seyondCloudTimeContractValid(
      honest, std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
