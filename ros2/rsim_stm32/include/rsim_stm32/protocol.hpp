#pragma once
#include "rsim_stm32/vendor_protocol.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rsim_stm32 {
static_assert(sizeof(UplinkPacket_t) == 144);
static_assert(sizeof(DownlinkPacket_t) == 72);
static_assert(offsetof(_PacketUplinkPayload_t, motion) == 72);
static_assert(offsetof(_PacketDownlinkPayload_t, ros_twist) == 24);
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Vendor wire structs require a little-endian host"
#endif

inline uint16_t checksum(const uint8_t *data, size_t size) {
  uint16_t sum = 0;
  for (size_t i = 0; i < size; ++i) sum += data[i];
  return sum;
}

class Decoder {
 public:
  uint64_t invalid = 0, discarded = 0;
  std::vector<UplinkPacket_t> feed(const uint8_t *data, size_t size) {
    buffer_.insert(buffer_.end(), data, data + size);
    std::vector<UplinkPacket_t> result;
    const std::array<uint8_t, 2> head{0x40, 0x40};
    while (buffer_.size() >= 4) {
      auto start = std::search(buffer_.begin(), buffer_.end(), head.begin(), head.end());
      if (start == buffer_.end()) {
        size_t keep = buffer_.back() == 0x40 ? 1 : 0;
        discarded += buffer_.size() - keep;
        buffer_.erase(buffer_.begin(), buffer_.end() - keep);
        break;
      }
      discarded += static_cast<uint64_t>(start - buffer_.begin());
      buffer_.erase(buffer_.begin(), start);
      if (buffer_.size() < 4) break;
      if ((buffer_[2] | (buffer_[3] << 8)) != sizeof(PacketUplinkPayload_t)) {
        ++invalid;
        buffer_.erase(buffer_.begin());
        continue;
      }
      if (buffer_.size() < sizeof(UplinkPacket_t)) break;
      UplinkPacket_t packet{};
      std::memcpy(packet.raw_data, buffer_.data(), sizeof(packet));
      const auto &p = packet.data;
      bool valid = p.tail.tail1 == 0x23 && p.tail.tail2 == 0x23 &&
        p.tail.checksum == checksum(p.payload.raw_data, sizeof(p.payload));
      for (float v : p.payload.data.motion.robot_position) valid &= std::isfinite(v);
      for (float v : p.payload.data.motion.robot_speed) valid &= std::isfinite(v);
      if (!valid) {
        ++invalid;
        buffer_.erase(buffer_.begin());
        continue;
      }
      result.push_back(packet);
      buffer_.erase(buffer_.begin(), buffer_.begin() + sizeof(packet));
    }
    return result;
  }
 private:
  std::vector<uint8_t> buffer_;
};

inline DownlinkPacket_t velocity_packet(uint16_t counter, double linear, double angular) {
  DownlinkPacket_t packet{};
  packet.data.head = {0x40, 0x40, sizeof(PacketDownlinkPayload_t)};
  auto &p = packet.data.payload.data;
  p.counter = counter;
  p.twist_enable = true;
  // Reset, enable/disable, firmware, geometry and LED fields stay zero.
  p.ros_twist.linear_x = static_cast<float>(linear);
  p.ros_twist.angular_z = static_cast<float>(angular);
  packet.data.tail = {checksum(packet.data.payload.raw_data, sizeof(packet.data.payload)), 0x23, 0x23};
  return packet;
}

struct Command {
  std::string id, epoch;
  uint64_t sequence = 0;
  int64_t deadline_ns = 0;
  double linear = 0., angular = 0.;
};

class Guard {
 public:
  using Identity = std::pair<std::string, std::string>;
  bool enabled = false;
  double max_linear = .15, max_angular = .3;
  int64_t max_ttl_ns = 500000000;
  Command current;

  std::string accept(const Command &c, int64_t now, bool hardware_ok) {
    if (c.id.empty() || c.epoch.empty() || c.id.size() > 128 || c.epoch.size() > 128 || !c.sequence)
      return "invalid command identity";
    if (!std::isfinite(c.linear) || !std::isfinite(c.angular) ||
        std::abs(c.linear) > max_linear || std::abs(c.angular) > max_angular)
      return "velocity exceeds finite configured limits";
    if (c.deadline_ns <= now) return "expired";
    if (c.deadline_ns - now > max_ttl_ns) return "deadline exceeds provider TTL";
    bool nonzero = c.linear != 0. || c.angular != 0.;
    if (nonzero && !enabled) return "motion disabled";
    if (nonzero && !hardware_ok) return "hardware interlock or stale feedback";
    Identity identity{c.id, c.epoch}, owner{current.id, current.epoch};
    if (retired_.count(identity)) return "retired controller epoch";
    auto old = sequences_.find(identity);
    if (old != sequences_.end() && c.sequence <= old->second) return "replayed command";
    if (identity != owner && !current.id.empty() && now < current.deadline_ns)
      return "exclusive controller owner";
    if (old == sequences_.end() && sequences_.size() >= 1024) return "controller history full";
    if (identity != owner && !current.id.empty()) retired_.insert(owner);
    sequences_[identity] = c.sequence;
    current = c;
    return {};
  }
  bool stop(const std::string &id, const std::string &epoch) {
    if (id != current.id || epoch != current.epoch || id.empty()) return false;
    zero();
    return true;
  }
  void zero() { current.linear = current.angular = 0.; current.deadline_ns = 0; }
  std::pair<double, double> output(int64_t now, bool hardware_ok) {
    if (!hardware_ok || now >= current.deadline_ns) zero();
    return {current.linear, current.angular};
  }
 private:
  std::map<Identity, uint64_t> sequences_;
  std::set<Identity> retired_;
};
}  // namespace rsim_stm32
