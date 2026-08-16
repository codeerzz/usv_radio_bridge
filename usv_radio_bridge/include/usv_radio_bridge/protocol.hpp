// Wire protocol for the RFD900x link to the ground-station GUI.
//
// This is a from-scratch C++ port of ARACPC/navi_vehicle/navi_gui's
// radio_protocol.py, kept byte-for-byte compatible on the envelope (magic /
// version / header layout / CRC32) so the two implementations can talk to
// each other, but with several payload-level changes described inline below
// (smaller TELEMETRY, binary CONTROL/KILLSWITCH instead of JSON, and two new
// autonomy-arbitration packet types). There is no JSON anywhere in this
// node — every payload below is a fixed-size, manually byte-packed struct.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace usv_radio_bridge {

// ---------------------------------------------------------------------------
// Frame envelope — unchanged from radio_protocol.py:
//   [0xAA 0x55][version u8][packet_type u8][sequence u32 LE][payload_len u16 LE]
//   [payload...][crc32 u32 LE]
// CRC32 is IEEE 802.3 (zlib's crc32) over header+payload. Ported from
// HEADER_STRUCT = struct.Struct("<2sBBIH") and CRC_STRUCT = struct.Struct("<I").
// ---------------------------------------------------------------------------

inline constexpr uint8_t kMagic0 = 0xAA;
inline constexpr uint8_t kMagic1 = 0x55;
inline constexpr uint8_t kProtocolVersion = 2;

inline constexpr std::size_t kHeaderSize = 10;  // 2 (magic) + 1 + 1 + 4 + 2
inline constexpr std::size_t kCrcSize = 4;
inline constexpr std::size_t kMinFrameSize = kHeaderSize + kCrcSize;
// Matches radio_protocol.py's MAX_PAYLOAD; also bounds FrameParser's resync
// buffer so a corrupt length field can never justify holding gigabytes.
inline constexpr std::size_t kMaxPayload = 8192;

// Extends radio_protocol.py's PacketType: first 8 numeric values line up
// with the Python reference (which only actually names 7 — WAYPOINT through
// OBSTACLE_MAP — GAMEPAD_BUTTON is new here too), plus two new values for
// GUI-driven autonomy mode arbitration.
enum class PacketType : uint8_t {
  TELEMETRY = 1,
  RTCM = 2,
  GGA = 3,
  CONTROL = 4,
  KILLSWITCH = 5,
  WAYPOINT = 6,          // reserved, unimplemented — log-and-ignore if seen
  OBSTACLE_MAP = 7,      // reserved, unimplemented — log-and-ignore if seen
  GAMEPAD_BUTTON = 8,
  AUTONOMY_COMMAND = 9,
  AUTONOMY_STATUS = 10,
};

// Encodes one complete frame (header + payload + trailing CRC32).
std::vector<uint8_t> encode_frame(PacketType type, uint32_t sequence,
                                   const std::vector<uint8_t> &payload = {});

// ---------------------------------------------------------------------------
// Payload structs. Every encode/decode function below serialises field by
// field into a byte buffer instead of memcpy-ing a C++ struct: a packed C++
// struct's layout depends on compiler/ABI padding rules in ways Python's
// struct module ("<...") never has to worry about, and getting that silently
// wrong would desync this node from the ground-station GUI's own packer.
// Byte-level (de)serialisation costs nothing at this frame rate and removes
// the whole hazard class. All multi-byte fields are little-endian, which is
// also the native byte order on every platform this workspace targets
// (x86_64 dev machines, ARM64 Jetson), so no explicit byte-swapping logic is
// needed beyond picking the byte order when writing/reading.
// ---------------------------------------------------------------------------

// TELEMETRY — vehicle -> ground, redesigned vs. the Python reference: no raw
// angular velocity/acceleration/GNSS-covariance triples and no
// cmd_vel_active/joy_active flags (see radio_bridge_node for why), but now
// carries autonomy state (control_mode) so the GUI doesn't need a second
// channel just to show AUTO/MANUAL.
//
// Layout (little-endian, no padding): dd (lat, lon) + 10f (see field order
// below) + bBB (gnss_fix_status, rtk_status, control_mode) = 16 + 40 + 3
// = 59 bytes on the wire.
struct TelemetryPayload {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0F;
  float roll = 0.0F;
  float pitch = 0.0F;
  float heading = 0.0F;
  float speed = 0.0F;
  float velocity_x = 0.0F;
  float velocity_y = 0.0F;
  float velocity_z = 0.0F;
  float battery_voltage = 0.0F;
  float battery_percent = 0.0F;   // 0-100
  int8_t gnss_fix_status = 0;     // best-effort mirror of NavSatFix.status.status
  uint8_t rtk_status = 0;         // placeholder (0=unknown) until a real RTK source exists
  uint8_t control_mode = 0;       // 0 = MANUAL, 1 = AUTO, from gnc/control_mode
};
inline constexpr std::size_t kTelemetryPayloadSize = 59;

std::vector<uint8_t> encode_telemetry(const TelemetryPayload &telemetry);

// CONTROL — ground -> vehicle. Binary now (was JSON): two float32s.
struct ControlPayload {
  float linear_x = 0.0F;
  float angular_z = 0.0F;
};
inline constexpr std::size_t kControlPayloadSize = 8;
std::optional<ControlPayload> decode_control(const std::vector<uint8_t> &payload);

// KILLSWITCH — ground -> vehicle. Binary now (was JSON): one byte.
struct KillswitchPayload {
  uint8_t active = 0;
};
inline constexpr std::size_t kKillswitchPayloadSize = 1;
std::optional<KillswitchPayload> decode_killswitch(const std::vector<uint8_t> &payload);

// GAMEPAD_BUTTON — ground -> vehicle. No Python precedent (the reference
// read a *local* /joy topic on the vehicle instead); minimal new design per
// button index + pressed state.
struct GamepadButtonPayload {
  uint8_t button_index = 0;
  uint8_t pressed = 0;
};
inline constexpr std::size_t kGamepadButtonPayloadSize = 2;
std::optional<GamepadButtonPayload> decode_gamepad_button(const std::vector<uint8_t> &payload);

// AUTONOMY_COMMAND — ground -> vehicle. Requests a gnc/set_control_mode call.
struct AutonomyCommandPayload {
  uint8_t requested_mode = 0;  // 0 = MANUAL, 1 = AUTO
};
inline constexpr std::size_t kAutonomyCommandPayloadSize = 1;
std::optional<AutonomyCommandPayload> decode_autonomy_command(const std::vector<uint8_t> &payload);

// AUTONOMY_STATUS — vehicle -> ground only, never received. Built from the
// SetControlMode service response and sent immediately so the GUI doesn't
// have to wait for the next TELEMETRY tick to see whether a mode switch
// succeeded.
struct AutonomyStatusPayload {
  uint8_t accepted = 0;
  uint8_t resulting_mode = 0;  // 0 = MANUAL, 1 = AUTO
};
inline constexpr std::size_t kAutonomyStatusPayloadSize = 2;
std::vector<uint8_t> encode_autonomy_status(const AutonomyStatusPayload &status);

}  // namespace usv_radio_bridge
