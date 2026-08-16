// CRC algorithms used by the radio protocol.
#pragma once

#include <cstddef>
#include <cstdint>

namespace usv_radio_bridge {

// IEEE 802.3 CRC32 — the same polynomial/init/reflection Python's zlib.crc32
// computes. Delegates to system zlib rather than hand-rolling the table:
// this is the frame-envelope integrity check, and a subtly wrong hand-rolled
// implementation (wrong reflection, wrong final XOR) would silently desync
// from the ground-station GUI's Python zlib.crc32 output while looking
// correct in isolation.
uint32_t crc32_ieee(const uint8_t *data, std::size_t length);

// CRC24Q (used inside RTCM3 frames), poly 0x1864CFB, init 0, no reflection.
// Ported bit-for-bit from ntrip_relay.py's crc24q() — RTCM3's CRC is not the
// same algorithm as CRC32 and zlib has no built-in for it, so this one is
// hand-rolled directly from the reference's byte/bit loop.
uint32_t crc24q(const uint8_t *data, std::size_t length);

}  // namespace usv_radio_bridge
