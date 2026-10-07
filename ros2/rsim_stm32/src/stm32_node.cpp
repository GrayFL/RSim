#include "rsim_stm32/protocol.hpp"
#include "rsim_stm32/srv/set_velocity.hpp"
#include "rsim_stm32/srv/stop.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;
namespace rsim_stm32 {
int64_t monotonic_ns() {
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value.tv_sec * 1000000000LL + value.tv_nsec;
}
int64_t realtime_ns() {
  timespec value{};
  clock_gettime(CLOCK_REALTIME, &value);
  return value.tv_sec * 1000000000LL + value.tv_nsec;
}

class Serial {
 public:
  int fd = -1;
  bool exclusive = false;
  ~Serial() {
    if (fd >= 0) { if (exclusive) ioctl(fd, TIOCNXCL); ::close(fd); }
  }
  void open(const std::string &port, int baud) {
    speed_t speed;
    switch (baud) {
      case 115200: speed = B115200; break;
      case 230400: speed = B230400; break;
      case 460800: speed = B460800; break;
      case 500000: speed = B500000; break;
      case 921600: speed = B921600; break;
      default: throw std::invalid_argument("unsupported serial baud");
    }
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) throw std::runtime_error("open serial: " + std::string(strerror(errno)));
    if (flock(fd, LOCK_EX | LOCK_NB) || ioctl(fd, TIOCEXCL))
      throw std::runtime_error("serial port is already owned or cannot be made exclusive");
    exclusive = true;
    termios options{};
    if (tcgetattr(fd, &options)) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&options);
    options.c_cflag = (options.c_cflag & ~(CRTSCTS | CSTOPB | PARENB | CSIZE)) | CLOCAL | CREAD | CS8;
    options.c_cc[VMIN] = options.c_cc[VTIME] = 0;
    cfsetispeed(&options, speed); cfsetospeed(&options, speed);
    if (tcsetattr(fd, TCSANOW, &options)) throw std::runtime_error("tcsetattr failed");
  }
  void write(const DownlinkPacket_t &packet) {
    size_t offset = 0;
    int64_t deadline = monotonic_ns() + 5000000;
    while (offset < sizeof(packet)) {
      ssize_t count = ::write(fd, packet.raw_data + offset, sizeof(packet) - offset);
      if (count > 0) { offset += static_cast<size_t>(count); continue; }
      if (count < 0 && errno != EAGAIN && errno != EINTR)
        throw std::runtime_error("serial write failed: " + std::string(strerror(errno)));
      if (monotonic_ns() >= deadline) throw std::runtime_error("serial write exceeded 5 ms");
      pollfd p{fd, POLLOUT, 0}; poll(&p, 1, 1);
    }
  }
};

struct Sample { UplinkPacket_t packet; int64_t received_ns; };
class Stm32Node : public rclcpp::Node {
 public:
  Stm32Node() : Node("stm32_driver") {
    const auto port = declare_parameter<std::string>("port", "");
    if (port.empty()) throw std::invalid_argument("port must be explicitly configured");
    if (get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("physical driver requires system time");
    int baud = declare_parameter<int>("baudrate", 921600);
    guard_.enabled = declare_parameter<bool>("motion_enabled", false);
    guard_.max_linear = declare_parameter<double>("max_linear", .15);
    guard_.max_angular = declare_parameter<double>("max_angular", .3);
    double ttl = declare_parameter<double>("max_command_ttl", .5);
    double feedback_timeout = declare_parameter<double>("feedback_timeout", .2);
    double command_hz = declare_parameter<double>("command_hz", 100.);
    expected_protocol_ = declare_parameter<int>("protocol_type", 5);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    for (double value : {guard_.max_linear, guard_.max_angular, ttl, feedback_timeout, command_hz})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("limits must be positive and finite");
    if (ttl > .5 || feedback_timeout > .5 || command_hz < 20 || command_hz > 200 ||
        expected_protocol_ < 1 || expected_protocol_ > 5 || base_frame_.empty() || odom_frame_.empty())
      throw std::invalid_argument("invalid timing, protocol or frame configuration");
    guard_.max_ttl_ns = static_cast<int64_t>(ttl * 1e9);
    feedback_timeout_ns_ = static_cast<int64_t>(feedback_timeout * 1e9);
    period_ns_ = static_cast<int64_t>(1e9 / command_hz);
    serial_.open(port, baud);
    odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(20));
    battery_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", 5);
    estop_ = create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(1).transient_local());
    normal_ = create_publisher<std_msgs::msg::Bool>("is_normal", rclcpp::QoS(1).transient_local());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 5);
    command_service_ = create_service<srv::SetVelocity>("set_velocity",
      [this](const std::shared_ptr<srv::SetVelocity::Request> req,
             std::shared_ptr<srv::SetVelocity::Response> res) { command(*req, *res); });
    stop_service_ = create_service<srv::Stop>("stop",
      [this](const std::shared_ptr<srv::Stop::Request> req, std::shared_ptr<srv::Stop::Response> res) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!guard_.stop(req->controller_id, req->controller_epoch)) {
          res->reason = "stop does not own this session"; return;
        }
        uint64_t revision = ++revision_;
        bool ready = transmitted_.wait_for(lock, 60ms, [&] { return sent_revision_ >= revision || !io_error_.empty(); });
        res->stopped = ready && io_error_.empty() && sent_revision_ >= revision;
        res->reason = res->stopped ? "zero written to serial" : "zero write not confirmed";
        res->transmitted_ns = last_write_ns_;
      });
    publish_timer_ = create_wall_timer(5ms, [this] { publish_samples(); });
    status_timer_ = create_wall_timer(100ms, [this] { publish_status(); });
    worker_ = std::thread([this] { io_loop(); });
    RCLCPP_INFO(get_logger(), "Serial %s at %d; motion=%s; independent %.0f Hz serial watchdog",
      port.c_str(), baud, guard_.enabled ? "enabled" : "disabled", command_hz);
  }
  ~Stm32Node() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    // Best effort explicit zero; no reset or emergency-stop release.
    for (int i = 0; i < 3; ++i) {
      try { serial_.write(velocity_packet(counter_++, 0., 0.)); }
      catch (...) { break; }
      std::this_thread::sleep_for(2ms);
    }
  }
 private:
  bool healthy(int64_t now) const {
    if (!received_ || !io_error_.empty() || now - last_read_ns_ > feedback_timeout_ns_) return false;
    const auto &data = latest_.packet.data.payload.data;
    return data.sys_info.protocol_type == expected_protocol_ && !data.monitor.estop_status &&
      !data.monitor.soft_estop_status && !data.monitor.reserved1 &&
      !data.motor_driver.error_code[0] && !data.motor_driver.error_code[1];
  }
  void command(const srv::SetVelocity::Request &request, srv::SetVelocity::Response &response) {
    std::unique_lock<std::mutex> lock(mutex_);
    Command command{request.controller_id, request.controller_epoch, request.sequence,
      request.deadline_ns, request.linear_x, request.angular_z};
    int64_t now = monotonic_ns();
    response.reason = guard_.accept(command, now, healthy(now));
    if (!response.reason.empty()) { ++rejected_; return; }
    uint64_t revision = ++revision_;
    bool ready = transmitted_.wait_for(lock, 60ms, [&] { return sent_revision_ >= revision || !io_error_.empty(); });
    bool nonzero = command.linear != 0. || command.angular != 0.;
    response.accepted = ready && io_error_.empty() && sent_revision_ == revision &&
      (!nonzero || (last_sent_linear_ == command.linear && last_sent_angular_ == command.angular &&
                    last_write_ns_ < command.deadline_ns));
    if (!response.accepted) {
      guard_.zero(); ++revision_; ++rejected_;
      response.reason = "command not transmitted before timeout or interlock";
    } else response.reason = "written to serial; not a motor acknowledgement";
    response.transmitted_ns = last_write_ns_;
    response.transmit_sequence = writes_;
  }
  void io_loop() {
    Decoder decoder;
    int64_t next_write = monotonic_ns();
    try {
      while (running_) {
        pollfd event{serial_.fd, POLLIN, 0};
        int result = poll(&event, 1, 2);
        if (result < 0 && errno != EINTR) throw std::runtime_error("serial poll failed");
        if (event.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("serial disconnected");
        if (event.revents & POLLIN) {
          uint8_t buffer[4096];
          ssize_t size = ::read(serial_.fd, buffer, sizeof(buffer));
          if (size < 0 && errno != EAGAIN && errno != EINTR) throw std::runtime_error("serial read failed");
          if (size > 0) {
            const int64_t receipt = realtime_ns();
            auto packets = decoder.feed(buffer, static_cast<size_t>(size));
            std::lock_guard<std::mutex> lock(mutex_);
            invalid_ = decoder.invalid;
            for (const auto &packet : packets) {
              auto sequence = packet.data.payload.data.counter;
              if (received_ && sequence == latest_.packet.data.payload.data.counter) { ++duplicates_; continue; }
              if (received_) {
                uint16_t gap = sequence - latest_.packet.data.payload.data.counter;
                if (gap > 32768) { guard_.zero(); ++revision_; ++counter_resets_; }
                else missed_ += gap - 1;
              }
              latest_ = {packet, receipt}; received_ = true; last_read_ns_ = monotonic_ns(); ++frames_;
              if (samples_.size() >= 200) { samples_.pop_front(); ++queue_drops_; }
              samples_.push_back(latest_);
            }
          }
        }
        int64_t now = monotonic_ns();
        if (now >= next_write) {
          std::lock_guard<std::mutex> lock(mutex_);
          auto [linear, angular] = guard_.output(now, healthy(now));
          serial_.write(velocity_packet(counter_++, linear, angular));
          last_write_ns_ = monotonic_ns();
          last_sent_linear_ = linear; last_sent_angular_ = angular;
          sent_revision_ = revision_; ++writes_;
          transmitted_.notify_all();
          next_write = now + period_ns_;
        }
      }
    } catch (const std::exception &error) {
      std::lock_guard<std::mutex> lock(mutex_);
      io_error_ = error.what(); guard_.zero();
      try { serial_.write(velocity_packet(counter_++, 0., 0.)); } catch (...) {}
      transmitted_.notify_all();
      RCLCPP_ERROR(get_logger(), "%s", io_error_.c_str());
    }
  }
  void publish_samples() {
    std::deque<Sample> samples;
    { std::lock_guard<std::mutex> lock(mutex_); samples.swap(samples_); }
    for (const auto &sample : samples) {
      const auto &p = sample.packet.data.payload.data;
      if (p.sys_info.protocol_type != expected_protocol_) continue;
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = rclcpp::Time(sample.received_ns);
      msg.header.frame_id = odom_frame_; msg.child_frame_id = base_frame_;
      msg.pose.pose.position.x = p.motion.robot_position[0]; msg.pose.pose.position.y = p.motion.robot_position[1];
      msg.pose.pose.orientation.z = std::sin(p.motion.robot_position[2] / 2.);
      msg.pose.pose.orientation.w = std::cos(p.motion.robot_position[2] / 2.);
      msg.twist.twist.linear.x = p.motion.robot_speed[0]; msg.twist.twist.linear.y = p.motion.robot_speed[1];
      msg.twist.twist.angular.z = p.motion.robot_speed[2];
      // Preserve the old driver covariance; these are not calibrated noise estimates.
      for (size_t i = 0; i < 6; ++i) {
        double variance = i < 2 ? 1e-3 : (i == 5 ? 1e3 : 1e6);
        msg.pose.covariance[i * 6 + i] = msg.twist.covariance[i * 6 + i] = variance;
      }
      odom_->publish(msg);
    }
  }
  void publish_status() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name(); status.hardware_id = "robint_delivery_stm32";
    std::unique_lock<std::mutex> lock(mutex_);
    bool ok = healthy(monotonic_ns());
    std_msgs::msg::Bool estop_value, normal_value;
    estop_value.data = !received_ || latest_.packet.data.payload.data.monitor.estop_status;
    normal_value.data = ok;
    sensor_msgs::msg::BatteryState battery;
    bool has_battery = received_;
    status.level = ok ? status.OK : status.ERROR;
    status.message = !io_error_.empty() ? io_error_ : (ok ? "fresh serial feedback" : "interlock or stale feedback");
    auto put = [&](const std::string &key, const std::string &data) {
      diagnostic_msgs::msg::KeyValue entry; entry.key = key; entry.value = data; status.values.push_back(entry);
    };
    put("frames", std::to_string(frames_)); put("invalid_frames", std::to_string(invalid_));
    put("duplicate_frames", std::to_string(duplicates_)); put("missing_counter_frames", std::to_string(missed_));
    put("counter_resets", std::to_string(counter_resets_)); put("publish_queue_drops", std::to_string(queue_drops_));
    put("serial_writes", std::to_string(writes_)); put("rejected_commands", std::to_string(rejected_));
    put("feedback_age_s", received_ ? std::to_string((monotonic_ns() - last_read_ns_) * 1e-9) : "unknown");
    put("timestamp_source", "host serial reception; device has no acquisition timestamp");
    put("motion_enabled", guard_.enabled ? "true" : "false");
    put("sent_linear_x", std::to_string(last_sent_linear_)); put("sent_angular_z", std::to_string(last_sent_angular_));
    if (received_) {
      const auto &p = latest_.packet.data.payload.data;
      put("protocol_type", std::to_string(p.sys_info.protocol_type));
      put("software_version_raw", std::to_string(p.sys_info.software_version));
      put("hardware_version_raw", std::to_string(p.sys_info.hardware_version));
      put("estop", std::to_string(p.monitor.estop_status));
      put("soft_estop", std::to_string(p.monitor.soft_estop_status));
      put("bumper", std::to_string(p.monitor.reserved1));
      put("motor_error_left", std::to_string(p.motor_driver.error_code[0]));
      put("motor_error_right", std::to_string(p.motor_driver.error_code[1]));
      battery.header.stamp = rclcpp::Time(latest_.received_ns);
      battery.voltage = p.battery.voltage * .01F; battery.current = p.battery.current * .01F;
      battery.charge = p.battery.remain_capacity * .1F; battery.capacity = p.battery.total_capacity * .1F;
      battery.percentage = battery.capacity > 0 ? battery.charge / battery.capacity : NAN;
      battery.temperature = p.battery.temperature; battery.present = p.battery.type != 0;
    }
    array.status.push_back(status);
    lock.unlock();  // DDS backpressure must never hold the serial/watchdog mutex.
    estop_->publish(estop_value); normal_->publish(normal_value);
    if (has_battery) battery_->publish(battery);
    diagnostics_->publish(array);
  }
  Serial serial_;
  Guard guard_;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable transmitted_;
  Sample latest_{};
  std::deque<Sample> samples_;
  bool received_ = false;
  int expected_protocol_ = 5;
  uint16_t counter_ = 0;
  int64_t period_ns_ = 10000000, feedback_timeout_ns_ = 200000000, last_read_ns_ = 0, last_write_ns_ = 0;
  uint64_t frames_ = 0, invalid_ = 0, missed_ = 0, duplicates_ = 0, counter_resets_ = 0, queue_drops_ = 0;
  uint64_t revision_ = 0, sent_revision_ = 0, writes_ = 0, rejected_ = 0;
  double last_sent_linear_ = 0., last_sent_angular_ = 0.;
  std::string base_frame_, odom_frame_, io_error_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_, normal_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Service<srv::SetVelocity>::SharedPtr command_service_;
  rclcpp::Service<srv::Stop>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_, status_timer_;
};
}  // namespace rsim_stm32

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rsim_stm32::Stm32Node>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &error) {
    fprintf(stderr, "stm32_node: %s\n", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
