#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rtcm_msgs/msg/message.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/node.hpp>
#include <ublox_msgs/ublox_msgs.hpp>

#include "fake_receiver.hpp"

using namespace std::chrono_literals;
using ublox_test::FakeReceiver;
using ublox_test::Frame;

namespace {

constexpr uint8_t kCfg = 0x06;
constexpr uint8_t kCfgMsg = 0x01;
constexpr uint8_t kCfgNavx5 = 0x23;
constexpr uint16_t kMask1AckAid = 0x0400;

uint16_t mask1(const Frame& navx5) { return navx5.payload[2] | (navx5.payload[3] << 8); }
uint8_t ackAiding(const Frame& navx5) { return navx5.payload[17]; }

bool isAckAidingNavx5(const Frame& f) {
  return f.class_id == kCfg && f.message_id == kCfgNavx5 && f.payload.size() == 40 && (mask1(f) & kMask1AckAid);
}

/// CFG-MSG frames that set the output rate of one message
std::vector<Frame> rateFrames(FakeReceiver& receiver, uint8_t class_id, uint8_t message_id) {
  std::vector<Frame> out;
  for (const Frame& f : receiver.framesOf(kCfg, kCfgMsg)) {
    if (f.payload.size() == 3 && f.payload[0] == class_id && f.payload[1] == message_id) {
      out.push_back(f);
    }
  }
  return out;
}

std::vector<Frame> sfrbxRateFrames(FakeReceiver& receiver) { return rateFrames(receiver, 0x02, 0x13); }

/// CFG-NAVX5 frames that set ackAiding
std::vector<Frame> ackAidingFrames(FakeReceiver& receiver) {
  std::vector<Frame> out;
  for (const Frame& f : receiver.framesOf(kCfg, kCfgNavx5)) {
    if (isAckAidingNavx5(f)) {
      out.push_back(f);
    }
  }
  return out;
}

/// RXM-SFRBX payload for a GPS L1C/A subframe with ten recognisable words
std::vector<uint8_t> gpsSubframe(uint8_t sv) {
  std::vector<uint8_t> payload = {0x00, sv, 0x00, 0x00, 10, 0x03, 0x02, 0x00};
  for (uint32_t i = 0; i < 10; ++i) {
    const uint32_t word = 0x22c00000u + i;
    for (int b = 0; b < 4; ++b) {
      payload.push_back(static_cast<uint8_t>(word >> (8 * b)));
    }
  }
  return payload;
}

bool contains(const std::vector<uint8_t>& haystack, const std::vector<uint8_t>& needle) {
  return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end()) != haystack.end();
}

}  // namespace

//
// Gps: the configuration and I/O the node builds on
//
class GpsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gps_ = std::make_shared<ublox_gps::Gps>(0, rclcpp::get_logger("gps_test"));
    gps_->initializeTcp("127.0.0.1", receiver_.url().substr(receiver_.url().rfind(':') + 1));
    ASSERT_TRUE(receiver_.waitForConnection());
  }

  FakeReceiver receiver_;
  std::shared_ptr<ublox_gps::Gps> gps_;
};

TEST_F(GpsTest, SetAckAidingOnlyTouchesTheAckAidingField) {
  ASSERT_TRUE(gps_->setAckAiding(true));

  const std::vector<Frame> navx5 = receiver_.framesOf(kCfg, kCfgNavx5);
  ASSERT_EQ(navx5.size(), 1u);
  ASSERT_EQ(navx5[0].payload.size(), 40u);
  EXPECT_EQ(navx5[0].payload[0] | (navx5[0].payload[1] << 8), 2);  // message version
  EXPECT_EQ(mask1(navx5[0]), kMask1AckAid);
  EXPECT_EQ(ackAiding(navx5[0]), 1);
  // mask2 is empty, so the receiver applies nothing else from this message
  EXPECT_EQ(navx5[0].payload[4] | navx5[0].payload[5] | navx5[0].payload[6] | navx5[0].payload[7], 0);
}

TEST_F(GpsTest, SetAckAidingReportsANack) {
  receiver_.nackIf(isAckAidingNavx5);
  EXPECT_FALSE(gps_->setAckAiding(true));
}

//
// UbloxNode: startup against a ZED-F9P on HPG 1.32 and the topics gnss_assist uses
//
class UbloxNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    probe_ = std::make_shared<rclcpp::Node>("gnss_assist_probe");
  }

  void TearDown() override {
    if (executor_) {
      // cancel() is lost if it lands before spin() starts, so stop with a flag
      stop_spinning_ = true;
      spin_thread_.join();
    }
    probe_.reset();
    node_.reset();  // its destructor shuts rclcpp down
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void startNode(const std::vector<rclcpp::Parameter>& extra) {
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("device", receiver_.url()),
        rclcpp::Parameter("debug", 0),
        rclcpp::Parameter("rate", 4.0),
        rclcpp::Parameter("nav_rate", 4),
        // As in rover.yaml. HpPosRecProduct, which HPG 1.32 selects, extends HpgRefProduct
        rclcpp::Parameter("tmode3", 0),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    // The constructor returns only once the receiver is configured, and a
    // configuration that keeps failing retries forever. Give up on it here so
    // the failure names the test, instead of the whole binary timing out
    auto started = std::async(std::launch::async, [options] { return std::make_shared<ublox_node::UbloxNode>(options); });
    if (started.wait_for(30s) != std::future_status::ready) {
      std::fprintf(stderr, "%s: UbloxNode did not finish starting within 30 s\n",
                   ::testing::UnitTest::GetInstance()->current_test_info()->name());
      std::_Exit(EXIT_FAILURE);
    }
    node_ = started.get();

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(probe_);
    spin_thread_ = std::thread([this] {
      while (!stop_spinning_) {
        executor_->spin_once(50ms);
      }
    });
  }

  FakeReceiver receiver_;
  rclcpp::Node::SharedPtr probe_;
  std::shared_ptr<ublox_node::UbloxNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spinning_{false};
};

TEST_F(UbloxNodeTest, EnablesAckAidingAndSfrbxAtStartup) {
  startNode({rclcpp::Parameter("ack_aiding", true), rclcpp::Parameter("publish.rxm.sfrb", true)});

  const std::vector<Frame> ack_cfg = ackAidingFrames(receiver_);
  ASSERT_EQ(ack_cfg.size(), 1u);
  EXPECT_EQ(ackAiding(ack_cfg[0]), 1);

  const std::vector<Frame> rate = sfrbxRateFrames(receiver_);
  ASSERT_EQ(rate.size(), 1u);
  EXPECT_EQ(rate[0].payload[2], 1);

  // MGA-ACK is an event output: setting a rate for it would be wrong
  EXPECT_TRUE(rateFrames(receiver_, 0x13, 0x60).empty());
}

TEST_F(UbloxNodeTest, DisabledSfrbxIsSwitchedOffOnTheReceiver) {
  startNode({rclcpp::Parameter("ack_aiding", false), rclcpp::Parameter("publish.rxm.sfrb", false),
             rclcpp::Parameter("publish.mon.ver", false)});

  // The rate may be saved in flash from an earlier run, so it must be set to 0
  const std::vector<Frame> rate = sfrbxRateFrames(receiver_);
  ASSERT_EQ(rate.size(), 1u);
  EXPECT_EQ(rate[0].payload[2], 0);

  const std::vector<Frame> ack_cfg = ackAidingFrames(receiver_);
  ASSERT_EQ(ack_cfg.size(), 1u);
  EXPECT_EQ(ackAiding(ack_cfg[0]), 0);

  EXPECT_EQ(probe_->count_publishers("/rxmsfrb"), 0u);
  EXPECT_EQ(probe_->count_publishers("/mgaack"), 0u);
  EXPECT_EQ(probe_->count_publishers("/monver"), 0u);
}

TEST_F(UbloxNodeTest, StartsEvenIfTheReceiverRejectsAckAiding) {
  receiver_.nackIf(isAckAidingNavx5);
  startNode({rclcpp::Parameter("ack_aiding", true)});

  // Sent once: a refusal doesn't send the configuration round again
  EXPECT_EQ(ackAidingFrames(receiver_).size(), 1u);
  EXPECT_EQ(probe_->count_publishers("/mgaack"), 1u);
}

TEST_F(UbloxNodeTest, PublishesMonVerLatched) {
  startNode({rclcpp::Parameter("publish.mon.ver", true)});

  // Subscribe after the node published, which only works with a latched topic
  std::atomic<bool> got{false};
  ublox_msgs::msg::MonVER mon_ver;
  auto sub = probe_->create_subscription<ublox_msgs::msg::MonVER>(
      "/monver", rclcpp::QoS(1).transient_local(), [&](const ublox_msgs::msg::MonVER& m) {
        mon_ver = m;
        got = true;
      });

  ASSERT_TRUE(FakeReceiver::waitFor([&] { return got.load(); }));
  const std::string sw(reinterpret_cast<const char*>(mon_ver.sw_version.data()));
  EXPECT_EQ(sw, "EXT CORE 1.00 (a59682)");
  bool has_fwver = false;
  for (const auto& ext : mon_ver.extension) {
    has_fwver |= std::string(reinterpret_cast<const char*>(ext.field.data())) == "FWVER=HPG 1.32";
  }
  EXPECT_TRUE(has_fwver);
}

TEST_F(UbloxNodeTest, PublishesSfrbxAndMgaAckFromTheReceiver) {
  startNode({rclcpp::Parameter("ack_aiding", true), rclcpp::Parameter("publish.rxm.sfrb", true)});

  std::atomic<bool> got_sfrbx{false};
  std::atomic<bool> got_ack{false};
  ublox_msgs::msg::RxmSFRBX sfrbx;
  ublox_msgs::msg::MgaACK ack;
  auto sfrbx_sub = probe_->create_subscription<ublox_msgs::msg::RxmSFRBX>(
      "/rxmsfrb", 100, [&](const ublox_msgs::msg::RxmSFRBX& m) {
        sfrbx = m;
        got_sfrbx = true;
      });
  auto ack_sub = probe_->create_subscription<ublox_msgs::msg::MgaACK>(
      "/mgaack", 100, [&](const ublox_msgs::msg::MgaACK& m) {
        ack = m;
        got_ack = true;
      });

  // Repeat until discovery has matched the subscriptions
  ASSERT_TRUE(FakeReceiver::waitFor([&] {
    if (!got_sfrbx) {
      receiver_.send(0x02, 0x13, gpsSubframe(5));
    }
    if (!got_ack) {
      receiver_.send(0x13, 0x60, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0x00});
    }
    std::this_thread::sleep_for(100ms);
    return got_sfrbx && got_ack;
  }));

  EXPECT_EQ(sfrbx.gnss_id, 0);
  EXPECT_EQ(sfrbx.sv_id, 5);
  ASSERT_EQ(sfrbx.dwrd.size(), 10u);
  EXPECT_EQ(sfrbx.dwrd[9], 0x22c00009u);
  EXPECT_EQ(ack.type, ublox_msgs::msg::MgaACK::TYPE_ACCEPTED);
  EXPECT_EQ(ack.msg_payload_start[2], 0x05);
}

TEST_F(UbloxNodeTest, ForwardsMgaToTheReceiver) {
  startNode({});

  std::vector<uint8_t> payload(24, 0);
  payload[0] = 0x10;
  rtcm_msgs::msg::Message msg;
  msg.message = ublox_test::encodeFrame(0x13, 0x40, payload);
  auto pub = probe_->create_publisher<rtcm_msgs::msg::Message>("/mga", 10);

  ASSERT_TRUE(FakeReceiver::waitFor([&] {
    pub->publish(msg);
    std::this_thread::sleep_for(100ms);
    return contains(receiver_.bytes(), msg.message);
  }));
}
