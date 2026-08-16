#include "usv_radio_bridge/crc.hpp"

#include <zlib.h>

namespace usv_radio_bridge {

uint32_t crc32_ieee(const uint8_t *data, std::size_t length)
{
  // zlib's crc32(0, ...) is exactly what Python's zlib.crc32() computes
  // (both start from a crc of 0), so this matches the ground-station GUI's
  // implementation byte-for-byte.
  return static_cast<uint32_t>(
      ::crc32(0L, reinterpret_cast<const Bytef *>(data), static_cast<uInt>(length)));
}

uint32_t crc24q(const uint8_t *data, std::size_t length)
{
  uint32_t crc = 0;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x800000U) ? ((crc << 1) ^ 0x1864CFBU) : (crc << 1);
      crc &= 0xFFFFFFU;
    }
  }
  return crc;
}

}  // namespace usv_radio_bridge
