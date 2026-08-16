#include "usv_radio_bridge/protocol.hpp"

#include <cstring>

#include "usv_radio_bridge/crc.hpp"

namespace usv_radio_bridge {

namespace {

void push_u16le(std::vector<uint8_t> &buf, uint16_t value)
{
  buf.push_back(static_cast<uint8_t>(value & 0xFFU));
  buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

void push_u32le(std::vector<uint8_t> &buf, uint32_t value)
{
  for (int i = 0; i < 4; ++i) {
    buf.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFFU));
  }
}

void push_f32le(std::vector<uint8_t> &buf, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  push_u32le(buf, bits);
}

void push_f64le(std::vector<uint8_t> &buf, double value)
{
  uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    buf.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFFU));
  }
}

}  // namespace

std::vector<uint8_t> encode_frame(PacketType type, uint32_t sequence,
                                   const std::vector<uint8_t> &payload)
{
  std::vector<uint8_t> frame;
  frame.reserve(kHeaderSize + payload.size() + kCrcSize);

  frame.push_back(kMagic0);
  frame.push_back(kMagic1);
  frame.push_back(kProtocolVersion);
  frame.push_back(static_cast<uint8_t>(type));
  push_u32le(frame, sequence);
  push_u16le(frame, static_cast<uint16_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());

  const uint32_t crc = crc32_ieee(frame.data(), frame.size());
  push_u32le(frame, crc);
  return frame;
}

std::vector<uint8_t> encode_telemetry(const TelemetryPayload &telemetry)
{
  std::vector<uint8_t> payload;
  payload.reserve(kTelemetryPayloadSize);

  push_f64le(payload, telemetry.latitude);
  push_f64le(payload, telemetry.longitude);
  push_f32le(payload, telemetry.altitude);
  push_f32le(payload, telemetry.roll);
  push_f32le(payload, telemetry.pitch);
  push_f32le(payload, telemetry.heading);
  push_f32le(payload, telemetry.speed);
  push_f32le(payload, telemetry.velocity_x);
  push_f32le(payload, telemetry.velocity_y);
  push_f32le(payload, telemetry.velocity_z);
  push_f32le(payload, telemetry.battery_voltage);
  push_f32le(payload, telemetry.battery_percent);
  payload.push_back(static_cast<uint8_t>(telemetry.gnss_fix_status));
  payload.push_back(telemetry.rtk_status);
  payload.push_back(telemetry.control_mode);

  return payload;
}

std::optional<ControlPayload> decode_control(const std::vector<uint8_t> &payload)
{
  if (payload.size() != kControlPayloadSize) {
    return std::nullopt;
  }
  ControlPayload out;
  uint32_t bits = 0;
  std::memcpy(&bits, payload.data(), sizeof(bits));
  std::memcpy(&out.linear_x, &bits, sizeof(out.linear_x));
  std::memcpy(&bits, payload.data() + 4, sizeof(bits));
  std::memcpy(&out.angular_z, &bits, sizeof(out.angular_z));
  return out;
}

std::optional<KillswitchPayload> decode_killswitch(const std::vector<uint8_t> &payload)
{
  if (payload.size() != kKillswitchPayloadSize) {
    return std::nullopt;
  }
  KillswitchPayload out;
  out.active = payload[0];
  return out;
}

std::optional<GamepadButtonPayload> decode_gamepad_button(const std::vector<uint8_t> &payload)
{
  if (payload.size() != kGamepadButtonPayloadSize) {
    return std::nullopt;
  }
  GamepadButtonPayload out;
  out.button_index = payload[0];
  out.pressed = payload[1];
  return out;
}

std::optional<AutonomyCommandPayload> decode_autonomy_command(const std::vector<uint8_t> &payload)
{
  if (payload.size() != kAutonomyCommandPayloadSize) {
    return std::nullopt;
  }
  AutonomyCommandPayload out;
  out.requested_mode = payload[0];
  return out;
}

std::vector<uint8_t> encode_autonomy_status(const AutonomyStatusPayload &status)
{
  std::vector<uint8_t> payload;
  payload.reserve(kAutonomyStatusPayloadSize);
  payload.push_back(status.accepted);
  payload.push_back(status.resulting_mode);
  return payload;
}

}  // namespace usv_radio_bridge
