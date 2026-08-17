// Vehicle-side RFD900x radio bridge to the ground-station GUI.
//
// C++ port of ARACPC/navi_vehicle/navi_gui/telemetry_sender.py, replacing
// its JSON command payloads with fixed binary structs and adding autonomy
// mode arbitration (AUTONOMY_COMMAND/AUTONOMY_STATUS) against
// usv_gnc's mode_mux_node. See protocol.hpp for the wire format and why the
// TELEMETRY payload dropped several of the old fields.
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/rtcm.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_gnc_msgs/srv/set_control_mode.hpp>

#include "usv_radio_bridge/frame_parser.hpp"
#include "usv_radio_bridge/protocol.hpp"
#include "usv_radio_bridge/serial_transport.hpp"

namespace usv_radio_bridge {

class RadioBridgeNode : public rclcpp::Node
{
public:
  RadioBridgeNode();
  ~RadioBridgeNode() override;

  RadioBridgeNode(const RadioBridgeNode &) = delete;
  RadioBridgeNode & operator=(const RadioBridgeNode &) = delete;

private:
  // ---- Parameters (see config/radio_bridge.yaml for defaults/docs) -------
  std::string serial_port_;
  int baud_rate_ = 115200;
  double tx_rate_hz_ = 50.0;
  double telemetry_rate_hz_ = 10.0;
  double radio_timeout_ = 2.0;
  bool publish_remote_control_ = false;
  std::string killswitch_topic_;
  double max_linear_speed_ = 1.0;
  double max_angular_speed_ = 1.0;
  int gamepad_button_count_ = 16;

  // ---- Joy output ---------------------------------------------------------
  // The radio publishes sensor_msgs/Joy on the SAME topic as the physical
  // gamepad, so it enters the existing chain instead of bypassing it:
  //
  //   /joy -> mode_mux_node  (A=AUTO, B=MANUAL)      -> gnc/control_mode
  //        -> rc_teleop_node (sticks + LB deadman)   -> /gnc/ap_joy_manual
  //   mode_mux picks manual|auto                     -> /ap/joy
  //
  // Writing /cmd_vel directly (what this node did before) went around
  // mode_mux entirely: the radio could drive the boat while the mux still
  // believed it was in AUTO, and the deadman did not apply. Feeding /joy keeps
  // mode_mux the sole arbiter and gets the deadman for free.
  std::string joy_topic_;
  int joy_axis_count_ = 8;
  int joy_surge_axis_ = 1;   // must match rc_teleop_node's surge_axis
  int joy_yaw_axis_ = 3;     // must match rc_teleop_node's yaw_axis

  // ---- ROS I/O -------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;

  rclcpp::Publisher<mavros_msgs::msg::RTCM>::SharedPtr rtcm_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr killswitch_pub_;

  /// Latest stick/button state, published as one Joy on every CONTROL frame
  /// and on every button change. Buttons must persist across CONTROL frames
  /// (the deadman is held down while the sticks move), so the state is kept
  /// here rather than rebuilt per frame. Guarded by joy_mutex_ because CONTROL
  /// and GAMEPAD_BUTTON are both handled on the serial reader thread but the
  /// killswitch failsafe can zero it from elsewhere.
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy joy_state_;
  void publishJoyLocked();
  // Fixed-size, pre-created at startup (see constructor) rather than created
  // on demand from the reader thread: bounds resource usage against a
  // garbled/out-of-range button_index and avoids calling create_publisher()
  // from a non-executor thread.
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> gamepad_button_pubs_;

  rclcpp::Client<usv_gnc_msgs::srv::SetControlMode>::SharedPtr set_control_mode_client_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr link_check_timer_;

  // ---- Telemetry source cache ----------------------------------------------
  // Subscription callbacks just update this cache under data_mutex_ and
  // return — no per-topic threads, no work done in the callback beyond a
  // few field assignments. telemetry_timer_ (10 Hz default) is the only
  // reader.
  std::mutex data_mutex_;
  double cache_latitude_ = 0.0;
  double cache_longitude_ = 0.0;
  float cache_altitude_ = 0.0F;
  float cache_roll_ = 0.0F;
  float cache_pitch_ = 0.0F;
  float cache_heading_ = 0.0F;
  float cache_speed_ = 0.0F;
  float cache_velocity_x_ = 0.0F;
  float cache_velocity_y_ = 0.0F;
  float cache_velocity_z_ = 0.0F;
  float cache_battery_voltage_ = 0.0F;
  float cache_battery_percent_ = 0.0F;
  int8_t cache_gnss_fix_status_ = 0;
  uint8_t cache_control_mode_ = 0;  // 0 = MANUAL, 1 = AUTO

  // Ported from telemetry_sender.py's received_sources/required_sources
  // gating: never send a telemetry frame built from fields that were never
  // actually populated. IMU stays a *gating* source even though none of its
  // fields are carried in the new, smaller TELEMETRY payload (see
  // radio_bridge_node.cpp's on_imu()) — losing a real sensor-liveness check
  // just because its data isn't on the wire anymore would be a regression.
  bool position_received_ = false;
  bool euler_received_ = false;
  bool velocity_received_ = false;
  bool imu_received_ = false;
  bool gnss_received_ = false;
  bool battery_received_ = false;

  // ---- TX side --------------------------------------------------------------
  // Bounded, drop-oldest queues, per the strict TX priority order
  // (KILLSWITCH > AUTONOMY_STATUS > CONTROL > RTCM > GGA > TELEMETRY). Only
  // AUTONOMY_STATUS and GGA are ever actually pushed to by this node's own
  // logic: KILLSWITCH/CONTROL/RTCM are ground->vehicle packet types that get
  // acted on immediately on RX (published to a ROS topic / applied locally)
  // rather than re-transmitted, so their TX queues exist to complete the
  // priority contract but stay empty in normal operation. TELEMETRY is not
  // one of these queues at all — see telemetry_slot_ below.
  static constexpr std::size_t kQueueCapacity = 16;
  std::mutex tx_mutex_;
  std::deque<std::vector<uint8_t>> killswitch_queue_;
  std::deque<std::vector<uint8_t>> autonomy_status_queue_;
  std::deque<std::vector<uint8_t>> control_queue_;
  std::deque<std::vector<uint8_t>> rtcm_queue_;
  std::deque<std::vector<uint8_t>> gga_queue_;

  // Single depth-1 "latest built frame" slot, rebuilt at telemetry_rate_hz_
  // and cleared once sent by the TX timer — never resent as if it were
  // still current once telemetry_timer_ would already have rebuilt it, and
  // never left populated with stale data if the required sources stop
  // arriving (see on_telemetry_timer()).
  std::mutex telemetry_mutex_;
  std::vector<uint8_t> telemetry_slot_;
  bool telemetry_slot_valid_ = false;

  std::atomic<uint32_t> tx_sequence_{0};

  // ---- Serial I/O -------------------------------------------------------------
  SerialTransport transport_;
  FrameParser parser_;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  // Nanoseconds since the steady_clock epoch of the last successful serial
  // read; a plain steady-clock timestamp (not an rclcpp::Time) so
  // radio-link-down detection works identically regardless of use_sim_time
  // and never risks an rclcpp::Time "different clock type" mismatch against
  // a wall-clock/ROS-time value elsewhere in the node.
  std::atomic<int64_t> last_rx_steady_ns_{0};
  bool link_was_down_ = false;

  // ---- Setup helpers (constructor only) --------------------------------------
  void declare_and_read_parameters();
  void create_subscriptions();
  void create_publishers();

  // ---- Subscription callbacks (executor thread) ------------------------------
  void on_position(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void on_euler(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void on_velocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_gnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void on_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void on_nmea(const nmea_msgs::msg::Sentence::SharedPtr msg);
  void on_control_mode(const std_msgs::msg::String::SharedPtr msg);

  // ---- Timers -----------------------------------------------------------------
  void on_telemetry_timer();
  void on_tx_timer();
  void on_link_check_timer();

  // ---- Serial reader thread ----------------------------------------------------
  void reader_loop();
  void dispatch_frame(const ParsedFrame & frame);
  void handle_rtcm(const ParsedFrame & frame);
  void handle_control(const ParsedFrame & frame);
  void handle_killswitch(const ParsedFrame & frame);
  void handle_gamepad_button(const ParsedFrame & frame);
  void handle_autonomy_command(const ParsedFrame & frame);
  void on_set_control_mode_response(
    rclcpp::Client<usv_gnc_msgs::srv::SetControlMode>::SharedFuture future);

  // ---- Small shared helpers ------------------------------------------------
  uint32_t next_sequence();
  void enqueue_bounded(std::deque<std::vector<uint8_t>> & queue, std::vector<uint8_t> && frame);
};

}  // namespace usv_radio_bridge
