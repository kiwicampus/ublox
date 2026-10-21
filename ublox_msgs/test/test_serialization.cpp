#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <ublox_msgs/serialization.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

namespace {

// Wraps a payload in a UBX frame: sync chars, class, id, length and checksum
std::vector<uint8_t> frame(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> out(payload.size() + 8);
  ublox::Writer writer(out.data(), out.size());
  EXPECT_TRUE(writer.write(payload.data(), payload.size(), class_id, message_id));
  return out;
}

}  // namespace

TEST(MgaACK, IdsMatchTheInterfaceDescription) {
  EXPECT_EQ(ublox_msgs::msg::MgaACK::CLASS_ID, 0x13);
  EXPECT_EQ(ublox_msgs::msg::MgaACK::MESSAGE_ID, 0x60);
  EXPECT_EQ(ublox_msgs::Class::MGA, 0x13);
  EXPECT_EQ(ublox_msgs::Message::MGA::ACK, 0x60);
}

TEST(MgaACK, DecodesAFramedRejection) {
  // MGA-GPS-EPH (msgId 0x00) for SV 12 rejected because the receiver has no time
  const std::vector<uint8_t> payload = {0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x0c, 0x00};
  const std::vector<uint8_t> data = frame(0x13, 0x60, payload);

  ublox::Reader reader(data.data(), data.size());
  ublox_msgs::msg::MgaACK m;
  ASSERT_TRUE(reader.read(m, true));
  EXPECT_EQ(m.type, ublox_msgs::msg::MgaACK::TYPE_NOT_USED);
  EXPECT_EQ(m.version, 0);
  EXPECT_EQ(m.info_code, ublox_msgs::msg::MgaACK::INFO_NO_TIME);
  EXPECT_EQ(m.msg_id, 0x00);
  EXPECT_EQ(m.msg_payload_start[0], 0x01);
  EXPECT_EQ(m.msg_payload_start[1], 0x00);
  EXPECT_EQ(m.msg_payload_start[2], 0x0c);
  EXPECT_EQ(m.msg_payload_start[3], 0x00);
}

TEST(MgaACK, RejectsACorruptedChecksum) {
  std::vector<uint8_t> data = frame(0x13, 0x60, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0c, 0x00});
  data.back() ^= 0xff;

  ublox::Reader reader(data.data(), data.size());
  ublox_msgs::msg::MgaACK m;
  EXPECT_FALSE(reader.read(m, true));
}

TEST(MgaACK, WriteRoundTripsTheBytes) {
  ublox_msgs::msg::MgaACK m;
  m.type = ublox_msgs::msg::MgaACK::TYPE_ACCEPTED;
  m.info_code = ublox_msgs::msg::MgaACK::INFO_ACCEPTED;
  m.msg_id = 0x40;
  m.msg_payload_start = {0x10, 0x00, 0x00, 0x80};

  std::vector<uint8_t> data(16);
  ublox::Writer writer(data.data(), data.size());
  ASSERT_TRUE(writer.write(m));
  const std::vector<uint8_t> expected = frame(0x13, 0x60, {0x01, 0x00, 0x00, 0x40, 0x10, 0x00, 0x00, 0x80});
  EXPECT_EQ(data, expected);
}

TEST(RxmSFRBX, DecodesASubframeWithItsSignalId) {
  // GPS (gnssId 0) SV 5 on L2 CL (sigId 3), distinct values so a swapped field
  // shows up. Ten 32-bit words, little endian
  std::vector<uint8_t> payload = {0x00, 0x05, 0x03, 0x00, 10, 0x03, 0x02, 0x00};
  for (uint32_t i = 0; i < 10; ++i) {
    const uint32_t word = 0x22c00000u + i;
    for (int b = 0; b < 4; ++b) {
      payload.push_back(static_cast<uint8_t>(word >> (8 * b)));
    }
  }
  const std::vector<uint8_t> data = frame(0x02, 0x13, payload);

  ublox::Reader reader(data.data(), data.size());
  ublox_msgs::msg::RxmSFRBX m;
  ASSERT_TRUE(reader.read(m, true));
  EXPECT_EQ(m.gnss_id, 0);
  EXPECT_EQ(m.sv_id, 5);
  EXPECT_EQ(m.sig_id, 3);
  EXPECT_EQ(m.num_words, 10);
  EXPECT_EQ(m.chn, 3);
  EXPECT_EQ(m.version, 2);
  ASSERT_EQ(m.dwrd.size(), 10u);
  EXPECT_EQ(m.dwrd[0], 0x22c00000u);
  EXPECT_EQ(m.dwrd[9], 0x22c00009u);
}
