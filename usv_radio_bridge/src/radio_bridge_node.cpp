#include "usv_radio_bridge/radio_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "usv_radio_bridge/crc.hpp"

namespace usv_radio_bridge {

namespace {
constexpr std::chrono::milliseconds kOpenRetryDelay{500};
}  // namespace

RadioBridgeNode::RadioBridgeNode()
: rclcpp::Node("radio_bridge_node")
{
  declare_and_read_parameters();

  gamepad_button_pubs_.reserve(static_cast<std::size_t>(std::max(gamepad_button_count_, 0)));
  for (int i = 0; i < gamepad_button_count_; ++i) {
    gamepad_button_pubs_.push_back(this->create_publisher<std_msgs::msg::Bool>(
      "radio/gamepad/button_" + std::to_string(i), 10));
  }

  create_publishers();
  create_subscriptions();

  set_control_mode_client_ =
    this->create_client<usv_gnc_msgs::srv::SetControlMode>("gnc/set_control_mode");

  telemetry_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / telemetry_rate_hz_),
    [this]() { on_telemetry_timer(); });

  tx_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / tx_rate_hz_),
    [this]() { on_tx_timer(); });

  link_check_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0),
    [this]() { on_link_check_timer(); });

  running_ = true;
  reader_thread_ = std::thread(&RadioBridgeNode::reader_loop, this);

  RCLCPP_INFO(get_logger(),
    "radio_bridge_node ready: port=%s baud=%d tx_rate=%.1fHz telemetry_rate=%.1fHz "
    "radio_timeout=%.1fs",
    serial_port_.c_str(), baud_rate_, tx_rate_hz_, telemetry_rate_hz_, radio_timeout_);
  if (publish_remote_control_) {
    RCLCPP_WARN(get_logger(),
      "publish_remote_control ENABLED: sticks+buttons -> %s (surge axis %d, yaw axis %d), "
      "killswitch -> %s. The radio now enters the SAME chain as the physical gamepad: "
      "mode_mux_node arbitrates the mode and rc_teleop_node's deadman applies",
      joy_topic_.c_str(), joy_surge_axis_, joy_yaw_axis_, killswitch_topic_.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
      "publish_remote_control disabled (default) — CONTROL/KILLSWITCH frames are received "
      "but not acted on; set publish_remote_control:=true once the radio link is verified");
  }
}

RadioBridgeNode::~RadioBridgeNode()
{
  running_ = false;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  transport_.close();
}

// ---- Setup helpers ----------------------------------------------------------

void RadioBridgeNode::declare_and_read_parameters()
{
  serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);
  tx_rate_hz_ = this->declare_parameter<double>("tx_rate_hz", 50.0);
  telemetry_rate_hz_ = this->declare_parameter<double>("telemetry_rate_hz", 10.0);
  radio_timeout_ = this->declare_parameter<double>("radio_timeout", 2.0);
  // Deliberately false by default — see radio_bridge.yaml / package README
  // for why the Python reference's default of true was a bug, not a
  // feature, and is not being carried forward.
  publish_remote_control_ = this->declare_parameter<bool>("publish_remote_control", false);
  killswitch_topic_ = this->declare_parameter<std::string>("killswitch_topic", "/navi/killswitch");
  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 1.0);
  max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.0);
  gamepad_button_count_ = this->declare_parameter<int>("gamepad_button_count", 16);

  joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
  joy_axis_count_ = this->declare_parameter<int>("joy_axis_count", 8);
  joy_surge_axis_ = this->declare_parameter<int>("joy_surge_axis", 1);
  joy_yaw_axis_ = this->declare_parameter<int>("joy_yaw_axis", 3);

  this->declare_parameter<std::string>("position_topic", "/filter/positionlla");
  this->declare_parameter<std::string>("euler_topic", "/filter/euler");
  this->declare_parameter<std::string>("velocity_topic", "/filter/velocity");
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("gnss_topic", "/gnss");
  this->declare_parameter<std::string>("battery_topic", "/ap/battery");
  this->declare_parameter<std::string>("nmea_topic", "/nmea");
  this->declare_parameter<std::string>("control_mode_topic", "gnc/control_mode");

  if (tx_rate_hz_ <= 0.0) {
    throw std::invalid_argument("tx_rate_hz must be > 0");
  }
  if (telemetry_rate_hz_ <= 0.0) {
    throw std::invalid_argument("telemetry_rate_hz must be > 0");
  }
  if (gamepad_button_count_ < 0) {
    throw std::invalid_argument("gamepad_button_count must be >= 0");
  }
  // The axis indices address joy_state_.axes directly; a bad one would be an
  // out-of-range write on the reader thread. Reject at startup instead.
  if (joy_axis_count_ <= 0) {
    throw std::invalid_argument("joy_axis_count must be > 0");
  }
  if (joy_surge_axis_ < 0 || joy_surge_axis_ >= joy_axis_count_) {
    throw std::invalid_argument("joy_surge_axis out of range for joy_axis_count");
  }
  if (joy_yaw_axis_ < 0 || joy_yaw_axis_ >= joy_axis_count_) {
    throw std::invalid_argument("joy_yaw_axis out of range for joy_axis_count");
  }
}

void RadioBridgeNode::create_publishers()
{
  rtcm_pub_ = this->create_publisher<mavros_msgs::msg::RTCM>("/rtcm", 50);
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);
  killswitch_pub_ = this->create_publisher<std_msgs::msg::Bool>(killswitch_topic_, 10);

  std::lock_guard<std::mutex> lock(joy_mutex_);
  joy_state_.axes.assign(static_cast<std::size_t>(joy_axis_count_), 0.0F);
  joy_state_.buttons.assign(static_cast<std::size_t>(gamepad_button_count_), 0);
}

/// Stamp and publish joy_state_. Caller must hold joy_mutex_.
void RadioBridgeNode::publishJoyLocked()
{
  joy_state_.header.stamp = this->now();
  joy_state_.header.frame_id = "radio";
  joy_pub_->publish(joy_state_);
}

void RadioBridgeNode::create_subscriptions()
{
  const auto position_topic = this->get_parameter("position_topic").as_string();
  const auto euler_topic = this->get_parameter("euler_topic").as_string();
  const auto velocity_topic = this->get_parameter("velocity_topic").as_string();
  const auto imu_topic = this->get_parameter("imu_topic").as_string();
  const auto gnss_topic = this->get_parameter("gnss_topic").as_string();
  const auto battery_topic = this->get_parameter("battery_topic").as_string();
  const auto nmea_topic = this->get_parameter("nmea_topic").as_string();
  const auto control_mode_topic = this->get_parameter("control_mode_topic").as_string();

  // Matches this workspace's existing convention for sensor-ish subscriptions
  // (usv_localization, and the Python reference's qos_profile_sensor_data):
  // best-effort/volatile, tolerant of a publisher that doesn't offer reliable
  // delivery.
  position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    position_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_position(msg); });
  euler_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    euler_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_euler(msg); });
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    velocity_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_velocity(msg); });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });
  gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gnss_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { on_gnss(msg); });
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { on_battery(msg); });
  nmea_sub_ = this->create_subscription<nmea_msgs::msg::Sentence>(
    nmea_topic, rclcpp::SensorDataQoS(),
    [this](const nmea_msgs::msg::Sentence::SharedPtr msg) { on_nmea(msg); });

  // Must match mode_mux_node's gnc/control_mode publisher QoS exactly
  // (reliable, depth 1, transient_local) so a bridge that starts after the
  // mux still gets the latched last-published mode instead of defaulting
  // to MANUAL until the next transition.
  const auto mode_qos = rclcpp::QoS(1).reliable().transient_local();
  control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    control_mode_topic, mode_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) { on_control_mode(msg); });
}

// ---- Subscription callbacks (executor thread) --------------------------------

void RadioBridgeNode::on_position(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_latitude_ = msg->vector.x;
  cache_longitude_ = msg->vector.y;
  cache_altitude_ = static_cast<float>(msg->vector.z);
  position_received_ = true;
}

void RadioBridgeNode::on_euler(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_roll_ = static_cast<float>(msg->vector.x);
  cache_pitch_ = static_cast<float>(msg->vector.y);
  cache_heading_ = static_cast<float>(msg->vector.z);
  euler_received_ = true;
}

void RadioBridgeNode::on_velocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  const float vx = static_cast<float>(msg->vector.x);
  const float vy = static_cast<float>(msg->vector.y);
  const float vz = static_cast<float>(msg->vector.z);
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_velocity_x_ = vx;
  cache_velocity_y_ = vy;
  cache_velocity_z_ = vz;
  cache_speed_ = std::sqrt((vx * vx) + (vy * vy) + (vz * vz));
  velocity_received_ = true;
}

void RadioBridgeNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
{
  // None of Imu's fields are carried in the redesigned TELEMETRY payload
  // (see protocol.hpp's comment on TelemetryPayload) — this callback exists
  // purely to keep "IMU is alive" in the same required-sources gate the
  // Python reference used, so a dead IMU still blocks telemetry TX exactly
  // as it always has, even though its data no longer rides along.
  std::lock_guard<std::mutex> lock(data_mutex_);
  imu_received_ = true;
}

void RadioBridgeNode::on_gnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_gnss_fix_status_ = msg->status.status;
  gnss_received_ = true;
}

void RadioBridgeNode::on_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_battery_voltage_ = msg->voltage;
  // BatteryState.percentage is documented as 0..1 ("if unmeasured, NaN");
  // battery_percent on the wire is a human-facing 0-100 percentage, hence
  // the *100 here. NaN in -> NaN out if the source doesn't know either,
  // same best-effort spirit as the other new telemetry fields.
  cache_battery_percent_ = msg->percentage * 100.0F;
  battery_received_ = true;
}

void RadioBridgeNode::on_nmea(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  const std::string & raw = msg->sentence;
  const auto first = raw.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return;
  }
  const auto last = raw.find_last_not_of(" \t\r\n");
  const std::string sentence = raw.substr(first, last - first + 1);

  if (sentence.rfind("$GPGGA", 0) != 0 && sentence.rfind("$GNGGA", 0) != 0) {
    return;
  }

  const std::vector<uint8_t> payload(sentence.begin(), sentence.end());
  auto frame = encode_frame(PacketType::GGA, next_sequence(), payload);
  enqueue_bounded(gga_queue_, std::move(frame));
}

void RadioBridgeNode::on_control_mode(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cache_control_mode_ = (msg->data == "AUTO") ? 1U : 0U;
}

// ---- Timers -------------------------------------------------------------------

void RadioBridgeNode::on_telemetry_timer()
{
  TelemetryPayload payload;
  bool ready = false;
  bool have_position = false;
  bool have_euler = false;
  bool have_velocity = false;
  bool have_imu = false;
  bool have_gnss = false;
  bool have_battery = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    have_position = position_received_;
    have_euler = euler_received_;
    have_velocity = velocity_received_;
    have_imu = imu_received_;
    have_gnss = gnss_received_;
    have_battery = battery_received_;
    ready = have_position && have_euler && have_velocity && have_imu && have_gnss && have_battery;

    if (ready) {
      payload.latitude = cache_latitude_;
      payload.longitude = cache_longitude_;
      payload.altitude = cache_altitude_;
      payload.roll = cache_roll_;
      payload.pitch = cache_pitch_;
      payload.heading = cache_heading_;
      payload.speed = cache_speed_;
      payload.velocity_x = cache_velocity_x_;
      payload.velocity_y = cache_velocity_y_;
      payload.velocity_z = cache_velocity_z_;
      payload.battery_voltage = cache_battery_voltage_;
      payload.battery_percent = cache_battery_percent_;
      payload.gnss_fix_status = cache_gnss_fix_status_;
      payload.rtk_status = 0;  // no real RTK source wired up yet
      payload.control_mode = cache_control_mode_;
    }
  }

  if (!ready) {
    // Never invent zeroed/fake telemetry — this mirrors telemetry_sender.py's
    // send_telemetry() gate exactly: silently do nothing until every
    // required source has delivered at least one message.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "TELEMETRY waiting for ROS sources (position=%d euler=%d velocity=%d imu=%d gnss=%d "
      "battery=%d)",
      static_cast<int>(have_position), static_cast<int>(have_euler),
      static_cast<int>(have_velocity), static_cast<int>(have_imu), static_cast<int>(have_gnss),
      static_cast<int>(have_battery));
    return;
  }

  auto frame = encode_frame(PacketType::TELEMETRY, next_sequence(), encode_telemetry(payload));
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  telemetry_slot_ = std::move(frame);
  telemetry_slot_valid_ = true;
}

void RadioBridgeNode::on_tx_timer()
{
  std::vector<uint8_t> frame;
  bool have_frame = false;

  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (!killswitch_queue_.empty()) {
      frame = std::move(killswitch_queue_.front());
      killswitch_queue_.pop_front();
      have_frame = true;
    } else if (!autonomy_status_queue_.empty()) {
      frame = std::move(autonomy_status_queue_.front());
      autonomy_status_queue_.pop_front();
      have_frame = true;
    } else if (!control_queue_.empty()) {
      frame = std::move(control_queue_.front());
      control_queue_.pop_front();
      have_frame = true;
    } else if (!rtcm_queue_.empty()) {
      frame = std::move(rtcm_queue_.front());
      rtcm_queue_.pop_front();
      have_frame = true;
    } else if (!gga_queue_.empty()) {
      frame = std::move(gga_queue_.front());
      gga_queue_.pop_front();
      have_frame = true;
    }
  }

  if (!have_frame) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (telemetry_slot_valid_) {
      frame = std::move(telemetry_slot_);
      telemetry_slot_valid_ = false;
      have_frame = true;
    }
  }

  if (!have_frame) {
    return;  // nothing queued this tick — not an error, just idle
  }

  if (!transport_.write_frame(frame)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "radio TX failed (port not open or write error) — frame dropped");
  }
}

void RadioBridgeNode::on_link_check_timer()
{
  const int64_t last_ns = last_rx_steady_ns_.load();
  if (last_ns == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "radio link looks down: no bytes received yet since startup (radio_timeout=%.1fs)",
      radio_timeout_);
    link_was_down_ = true;
    return;
  }

  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const double elapsed_s = static_cast<double>(now_ns - last_ns) / 1e9;

  if (elapsed_s > radio_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "radio link looks down: no bytes received in %.1fs (radio_timeout=%.1fs)", elapsed_s,
      radio_timeout_);
    link_was_down_ = true;
  } else if (link_was_down_) {
    RCLCPP_INFO(get_logger(), "radio link recovered");
    link_was_down_ = false;
  }
}

// ---- Serial reader thread ------------------------------------------------------

void RadioBridgeNode::reader_loop()
{
  std::vector<uint8_t> buffer(4096);

  while (running_) {
    if (!transport_.is_open()) {
      try {
        transport_.open(serial_port_, baud_rate_);
        RCLCPP_INFO(get_logger(), "radio serial port opened: %s @ %d baud",
                    serial_port_.c_str(), baud_rate_);
        parser_.reset();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "failed to open radio serial port %s: %s (retrying)", serial_port_.c_str(), e.what());
        std::this_thread::sleep_for(kOpenRetryDelay);
        continue;
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "failed to open radio serial port %s: unknown error (retrying)",
          serial_port_.c_str());
        std::this_thread::sleep_for(kOpenRetryDelay);
        continue;
      }
    }

    try {
      const std::size_t n = transport_.read(buffer.data(), buffer.size());
      if (n == 0) {
        continue;
      }

      last_rx_steady_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count());

      std::vector<ParsedFrame> frames;
      try {
        frames = parser_.feed(buffer.data(), n);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "frame parser threw: %s (dropping this chunk)", e.what());
        continue;
      }

      for (const auto & frame : frames) {
        try {
          dispatch_frame(frame);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "handler for packet_type=%d threw: %s (frame dropped)",
                       static_cast<int>(frame.type), e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(),
            "handler for packet_type=%d threw an unknown exception (frame dropped)",
            static_cast<int>(frame.type));
        }
      }
    } catch (const std::exception & e) {
      // Only transport_.read() itself can throw past this point (parser and
      // handler exceptions are caught above and never reach here) — a real
      // I/O error, so closing and reopening is the correct response, not an
      // overreaction to a merely-malformed frame.
      RCLCPP_ERROR(get_logger(), "radio read failed, closing and retrying: %s", e.what());
      transport_.close();
      std::this_thread::sleep_for(kOpenRetryDelay);
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "radio read failed with an unknown error, closing and retrying");
      transport_.close();
      std::this_thread::sleep_for(kOpenRetryDelay);
    }
  }
}

void RadioBridgeNode::dispatch_frame(const ParsedFrame & frame)
{
  switch (frame.type) {
    case PacketType::RTCM:
      handle_rtcm(frame);
      break;
    case PacketType::CONTROL:
      handle_control(frame);
      break;
    case PacketType::KILLSWITCH:
      handle_killswitch(frame);
      break;
    case PacketType::GAMEPAD_BUTTON:
      handle_gamepad_button(frame);
      break;
    case PacketType::AUTONOMY_COMMAND:
      handle_autonomy_command(frame);
      break;
    case PacketType::WAYPOINT:
    case PacketType::OBSTACLE_MAP:
      RCLCPP_INFO(get_logger(), "packet type %d (#%u) is reserved/unimplemented — ignoring",
                  static_cast<int>(frame.type), static_cast<unsigned int>(frame.sequence));
      break;
    case PacketType::TELEMETRY:
    case PacketType::GGA:
    case PacketType::AUTONOMY_STATUS:
      RCLCPP_WARN(get_logger(),
        "packet type %d (#%u) is vehicle->ground only and was never expected on RX — ignoring",
        static_cast<int>(frame.type), static_cast<unsigned int>(frame.sequence));
      break;
    default:
      RCLCPP_WARN(get_logger(), "unknown packet type %d (#%u) — ignoring",
                  static_cast<int>(frame.type), static_cast<unsigned int>(frame.sequence));
      break;
  }
}

void RadioBridgeNode::handle_rtcm(const ParsedFrame & frame)
{
  const auto & payload = frame.payload;
  if (payload.size() < 6 || payload[0] != 0xD3) {
    RCLCPP_WARN(get_logger(), "RTCM #%u: invalid preamble/length, dropped",
                static_cast<unsigned int>(frame.sequence));
    return;
  }

  const std::size_t expected_length =
    3 + ((static_cast<std::size_t>(payload[1] & 0x03) << 8) | payload[2]) + 3;
  if (payload.size() != expected_length) {
    RCLCPP_WARN(get_logger(), "RTCM #%u: frame length mismatch, dropped",
                static_cast<unsigned int>(frame.sequence));
    return;
  }

  const uint32_t computed = crc24q(payload.data(), payload.size() - 3);
  const uint32_t received = (static_cast<uint32_t>(payload[payload.size() - 3]) << 16) |
                             (static_cast<uint32_t>(payload[payload.size() - 2]) << 8) |
                             static_cast<uint32_t>(payload[payload.size() - 1]);
  if (computed != received) {
    RCLCPP_WARN(get_logger(), "RTCM #%u: CRC24Q mismatch, dropped",
                static_cast<unsigned int>(frame.sequence));
    return;
  }

  mavros_msgs::msg::RTCM msg;
  msg.header.stamp = this->get_clock()->now();
  msg.data.assign(payload.begin(), payload.end());
  rtcm_pub_->publish(msg);
}

void RadioBridgeNode::handle_control(const ParsedFrame & frame)
{
  const auto decoded = decode_control(frame.payload);
  if (!decoded) {
    RCLCPP_WARN(get_logger(), "CONTROL #%u: bad payload size (%zu bytes), dropped",
                static_cast<unsigned int>(frame.sequence), frame.payload.size());
    return;
  }
  if (!publish_remote_control_) {
    return;
  }

  // Sticks go onto the SAME axes rc_teleop_node reads, so the radio looks
  // exactly like the physical gamepad to everything downstream. max_*_speed
  // stays a scale on the normalised stick, not a velocity: rc_teleop applies
  // its own surge/yaw scaling, and this axis is unitless.
  const float surge =
    std::clamp(decoded->linear_x, -1.0F, 1.0F) * static_cast<float>(max_linear_speed_);
  const float yaw =
    std::clamp(decoded->angular_z, -1.0F, 1.0F) * static_cast<float>(max_angular_speed_);

  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    joy_state_.axes[static_cast<std::size_t>(joy_surge_axis_)] = surge;
    joy_state_.axes[static_cast<std::size_t>(joy_yaw_axis_)] = yaw;
    publishJoyLocked();
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
    "REMOTE CONTROL RX #%u: surge(axis %d)=%.3f yaw(axis %d)=%.3f",
    static_cast<unsigned int>(frame.sequence), joy_surge_axis_, surge,
    joy_yaw_axis_, yaw);
}

void RadioBridgeNode::handle_killswitch(const ParsedFrame & frame)
{
  const auto decoded = decode_killswitch(frame.payload);
  if (!decoded) {
    RCLCPP_WARN(get_logger(), "KILLSWITCH #%u: bad payload size (%zu bytes), dropped",
                static_cast<unsigned int>(frame.sequence), frame.payload.size());
    return;
  }
  if (!publish_remote_control_) {
    return;
  }

  const bool active = decoded->active != 0;
  std_msgs::msg::Bool msg;
  msg.data = active;
  killswitch_pub_->publish(msg);
  RCLCPP_WARN(get_logger(), "KILLSWITCH RX #%u: %s", static_cast<unsigned int>(frame.sequence),
              active ? "ACTIVE" : "RELEASED");

  if (active) {
    // Immediate failsafe: centre the sticks AND clear every button, then send
    // it. Clearing the buttons is the part that matters — it releases the
    // deadman, so rc_teleop_node stops the thrusters itself rather than
    // waiting for its 0.5 s joy timeout. Zeroing the axes alone would leave
    // the deadman held and the boat merely idling.
    std::lock_guard<std::mutex> lock(joy_mutex_);
    std::fill(joy_state_.axes.begin(), joy_state_.axes.end(), 0.0F);
    std::fill(joy_state_.buttons.begin(), joy_state_.buttons.end(), 0);
    publishJoyLocked();
  }
}

void RadioBridgeNode::handle_gamepad_button(const ParsedFrame & frame)
{
  const auto decoded = decode_gamepad_button(frame.payload);
  if (!decoded) {
    RCLCPP_WARN(get_logger(), "GAMEPAD_BUTTON #%u: bad payload size (%zu bytes), dropped",
                static_cast<unsigned int>(frame.sequence), frame.payload.size());
    return;
  }
  if (decoded->button_index >= gamepad_button_pubs_.size()) {
    RCLCPP_WARN(get_logger(), "GAMEPAD_BUTTON #%u: button_index %u out of range (max %zu), dropped",
                static_cast<unsigned int>(frame.sequence),
                static_cast<unsigned int>(decoded->button_index), gamepad_button_pubs_.size());
    return;
  }

  const bool pressed = (decoded->pressed != 0);

  std_msgs::msg::Bool msg;
  msg.data = pressed;
  gamepad_button_pubs_[decoded->button_index]->publish(msg);

  // Also fold the button into the Joy state and republish. This is what makes
  // the GUI's mode buttons work: mode_mux_node edge-detects button 1 (B) for
  // MANUAL and 0 (A) for AUTO on /joy, and rc_teleop_node needs button 4 (LB)
  // held as its deadman. Gating on publish_remote_control_ deliberately: with
  // remote control off the buttons are still reported on their Bool topics for
  // diagnostics, but must not be able to change mode or arm the deadman.
  if (!publish_remote_control_) {
    return;
  }
  std::lock_guard<std::mutex> lock(joy_mutex_);
  if (decoded->button_index < joy_state_.buttons.size()) {
    joy_state_.buttons[decoded->button_index] = pressed ? 1 : 0;
    publishJoyLocked();
  }
}

void RadioBridgeNode::handle_autonomy_command(const ParsedFrame & frame)
{
  const auto decoded = decode_autonomy_command(frame.payload);
  if (!decoded) {
    RCLCPP_WARN(get_logger(), "AUTONOMY_COMMAND #%u: bad payload size (%zu bytes), dropped",
                static_cast<unsigned int>(frame.sequence), frame.payload.size());
    return;
  }

  const char * requested = (decoded->requested_mode != 0) ? "AUTO" : "MANUAL";

  if (!set_control_mode_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
      "AUTONOMY_COMMAND #%u: gnc/set_control_mode service not available, dropping request "
      "(mode=%s)",
      static_cast<unsigned int>(frame.sequence), requested);
    return;
  }

  auto request = std::make_shared<usv_gnc_msgs::srv::SetControlMode::Request>();
  request->mode = requested;

  // Fire-and-forget: this workspace's house convention (mode_mux_node.py's
  // own cancel-goal calls) is call_async without blocking the executor on
  // the returned future. The callback overload lets us still react once the
  // mux actually resolves the request, without stalling TX/RX in the
  // meantime.
  set_control_mode_client_->async_send_request(
    request,
    [this](rclcpp::Client<usv_gnc_msgs::srv::SetControlMode>::SharedFuture future) {
      on_set_control_mode_response(future);
    });
}

void RadioBridgeNode::on_set_control_mode_response(
  rclcpp::Client<usv_gnc_msgs::srv::SetControlMode>::SharedFuture future)
{
  const auto response = future.get();

  AutonomyStatusPayload status;
  status.accepted = response->success ? 1 : 0;
  status.resulting_mode = (response->mode == "AUTO") ? 1 : 0;

  auto frame =
    encode_frame(PacketType::AUTONOMY_STATUS, next_sequence(), encode_autonomy_status(status));
  enqueue_bounded(autonomy_status_queue_, std::move(frame));

  RCLCPP_INFO(get_logger(), "set_control_mode -> success=%d mode=%s (%s)",
              static_cast<int>(response->success), response->mode.c_str(),
              response->message.c_str());
}

// ---- Small shared helpers ------------------------------------------------------

uint32_t RadioBridgeNode::next_sequence()
{
  return tx_sequence_.fetch_add(1, std::memory_order_relaxed);
}

void RadioBridgeNode::enqueue_bounded(
  std::deque<std::vector<uint8_t>> & queue, std::vector<uint8_t> && frame)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  if (queue.size() >= kQueueCapacity) {
    // Drop-oldest: for every queue this node actually pushes to (GGA
    // sentences, autonomy-status confirmations) the newest entry is always
    // more useful than a stale queued one, so overflow prefers fresh data
    // over strict FIFO fairness.
    queue.pop_front();
  }
  queue.push_back(std::move(frame));
}

}  // namespace usv_radio_bridge
