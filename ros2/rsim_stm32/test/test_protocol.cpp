#include "rsim_stm32/protocol.hpp"
#include <gtest/gtest.h>

using namespace rsim_stm32;
UplinkPacket_t fixture() {
  UplinkPacket_t p{};
  p.data.head = {0x40, 0x40, sizeof(PacketUplinkPayload_t)};
  p.data.payload.data.counter = 123;
  p.data.payload.data.sys_info.protocol_type = DELIVERY_ROBOT_PROTOCOL;
  p.data.payload.data.motion.robot_position[0] = 1.25;
  p.data.payload.data.motion.robot_position[2] = -.75;
  p.data.tail = {checksum(p.data.payload.raw_data, sizeof(p.data.payload)), 0x23, 0x23};
  return p;
}
TEST(Protocol, FragmentNoiseChecksumAndRecovery) {
  Decoder decoder;
  auto packet = fixture();
  const uint8_t noise[]{0, 1, 0x40};
  EXPECT_TRUE(decoder.feed(noise, sizeof(noise)).empty());
  EXPECT_TRUE(decoder.feed(packet.raw_data, 71).empty());
  auto decoded = decoder.feed(packet.raw_data + 71, sizeof(packet) - 71);
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_FLOAT_EQ(decoded[0].data.payload.data.motion.robot_position[0], 1.25);
  EXPECT_FLOAT_EQ(decoded[0].data.payload.data.motion.robot_position[2], -.75);
  packet.raw_data[95] ^= 1;
  EXPECT_TRUE(decoder.feed(packet.raw_data, sizeof(packet)).empty());
  packet = fixture();
  decoded = decoder.feed(packet.raw_data, sizeof(packet));
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_GT(decoder.invalid, 0u);
}
TEST(Protocol, PayloadCanContainFrameDelimiters) {
  auto p = fixture();
  p.data.payload.raw_data[120] = 0x40; p.data.payload.raw_data[121] = 0x40;
  p.data.payload.raw_data[124] = 0x23; p.data.payload.raw_data[125] = 0x23;
  p.data.tail.checksum = checksum(p.data.payload.raw_data, sizeof(p.data.payload));
  Decoder d;
  ASSERT_EQ(d.feed(p.raw_data, sizeof(p)).size(), 1u);
}
TEST(Protocol, CommandDoesNotResetConfigureOrEnableHardware) {
  auto p = velocity_packet(12, -.1, .2);
  EXPECT_EQ(p.data.payload.data.counter, 12);
  EXPECT_EQ(p.data.payload.data.twist_enable, 1);
  EXPECT_FLOAT_EQ(p.data.payload.data.ros_twist.linear_x, -.1F);
  EXPECT_FLOAT_EQ(p.data.payload.data.ros_twist.angular_z, .2F);
  auto &body = p.data.payload;
  for (int i = 4; i < 28; ++i) EXPECT_EQ(p.raw_data[i], i == 4 ? 12 : (i == 6 ? 1 : 0));
  EXPECT_EQ(p.data.tail.checksum, checksum(body.raw_data, sizeof(body)));
}
TEST(Guard, ExpiryInterlocksAndOwnerCannotBeBypassed) {
  Guard g;
  Command a{"first", "epoch", 1, 200000000, .1, 0.};
  EXPECT_EQ(g.accept(a, 100000000, true), "motion disabled");
  g.enabled = true;
  EXPECT_FALSE(g.accept(a, 100000000, false).empty());
  EXPECT_TRUE(g.accept(a, 100000000, true).empty());
  EXPECT_DOUBLE_EQ(g.output(150000000, true).first, .1);
  EXPECT_FALSE(g.accept(a, 150000000, true).empty());
  Command b{"second", "epoch", 1, 250000000, .1, 0.};
  EXPECT_FALSE(g.accept(b, 150000000, true).empty());
  EXPECT_FALSE(g.stop(b.id, b.epoch));
  EXPECT_DOUBLE_EQ(g.output(200000000, true).first, 0.);
  EXPECT_TRUE(g.accept(b, 210000000, true).empty());
  a.sequence = 2; a.deadline_ns = 300000000;
  EXPECT_EQ(g.accept(a, 260000000, true), "retired controller epoch");
  EXPECT_DOUBLE_EQ(g.output(220000000, false).first, 0.);
  EXPECT_DOUBLE_EQ(g.output(221000000, true).first, 0.);  // no replay on recovery
}
