#include "usv_radio_bridge/frame_parser.hpp"

#include "usv_radio_bridge/crc.hpp"

namespace usv_radio_bridge {

namespace {

// Comfortably larger than the largest possible single frame (header + max
// payload + crc) so a legitimate max-size frame is never truncated by this
// safety cap — it only ever engages against a stream that never produces a
// valid MAGIC at all.
constexpr std::size_t kMaxBufferedBytes = kMaxPayload + kHeaderSize + kCrcSize + 4096;

}  // namespace

void FrameParser::reset()
{
  buffer_.clear();
  bad_frames_ = 0;
}

std::vector<ParsedFrame> FrameParser::feed(const uint8_t * data, std::size_t length)
{
  buffer_.insert(buffer_.end(), data, data + length);
  while (buffer_.size() > kMaxBufferedBytes) {
    buffer_.pop_front();
  }

  std::vector<ParsedFrame> frames;
  const std::size_t minimum = kHeaderSize + kCrcSize;

  while (buffer_.size() >= minimum) {
    bool found_magic = false;
    std::size_t start = 0;
    for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
      if (buffer_[i] == kMagic0 && buffer_[i + 1] == kMagic1) {
        start = i;
        found_magic = true;
        break;
      }
    }
    if (!found_magic) {
      // No MAGIC anywhere in the buffer. Keep only the last byte (it might
      // be the first half of a MAGIC split across two reads) and wait for
      // more data — mirrors radio_protocol.py's `del self.buffer[:-1]`.
      while (buffer_.size() > 1) {
        buffer_.pop_front();
      }
      break;
    }

    if (start > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
    }
    if (buffer_.size() < minimum) {
      break;  // wait for the rest of the header + crc to arrive
    }

    const uint8_t version = buffer_[2];
    const uint8_t packet_type_raw = buffer_[3];
    const uint32_t sequence = static_cast<uint32_t>(buffer_[4]) |
                               (static_cast<uint32_t>(buffer_[5]) << 8) |
                               (static_cast<uint32_t>(buffer_[6]) << 16) |
                               (static_cast<uint32_t>(buffer_[7]) << 24);
    const uint16_t payload_len = static_cast<uint16_t>(
        static_cast<uint16_t>(buffer_[8]) | (static_cast<uint16_t>(buffer_[9]) << 8));

    if (version != kProtocolVersion || payload_len > kMaxPayload) {
      buffer_.pop_front();
      ++bad_frames_;
      continue;
    }

    const std::size_t total = kHeaderSize + payload_len + kCrcSize;
    if (buffer_.size() < total) {
      break;  // frame not fully arrived yet
    }

    std::vector<uint8_t> raw(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    const uint32_t expected_crc = static_cast<uint32_t>(raw[total - 4]) |
                                   (static_cast<uint32_t>(raw[total - 3]) << 8) |
                                   (static_cast<uint32_t>(raw[total - 2]) << 16) |
                                   (static_cast<uint32_t>(raw[total - 1]) << 24);
    const uint32_t actual_crc = crc32_ieee(raw.data(), total - kCrcSize);

    if (actual_crc != expected_crc) {
      buffer_.pop_front();
      ++bad_frames_;
      continue;
    }

    ParsedFrame frame;
    frame.type = static_cast<PacketType>(packet_type_raw);
    frame.sequence = sequence;
    frame.payload.assign(raw.begin() + static_cast<std::ptrdiff_t>(kHeaderSize),
                          raw.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + payload_len));
    frames.push_back(std::move(frame));

    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
  }

  return frames;
}

}  // namespace usv_radio_bridge
