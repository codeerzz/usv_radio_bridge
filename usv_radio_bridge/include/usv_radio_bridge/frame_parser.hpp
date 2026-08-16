// Incremental, byte-stream-fed frame resync parser.
//
// Ported from radio_protocol.py's FrameParser.feed(): bytes arrive in
// arbitrary chunks off a serial port, so the parser has to cope with a
// frame split across two reads, with garbage bytes ahead of the first valid
// MAGIC, and with a corrupted length/CRC anywhere in the middle of an
// otherwise-plausible-looking frame — and it must never get permanently
// stuck on any of that. The recovery rule (drop exactly one byte and retry)
// is what the Python reference does; a full resync loop keeps every valid
// frame in a chunk from being lost to one bad byte before it.
#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "usv_radio_bridge/protocol.hpp"

namespace usv_radio_bridge {

struct ParsedFrame {
  PacketType type;
  uint32_t sequence;
  std::vector<uint8_t> payload;
};

class FrameParser
{
public:
  // Feeds newly-read bytes into the parser and returns every complete,
  // CRC-valid frame found. May return zero, one, or several frames per call
  // (a single serial read can contain more than one queued frame).
  std::vector<ParsedFrame> feed(const uint8_t * data, std::size_t length);

  void reset();

  uint64_t bad_frame_count() const { return bad_frames_; }

private:
  // std::deque, not an unbounded std::vector that only ever grows: bytes at
  // the front are dropped one at a time during resync, which is an O(1)
  // pop_front on a deque and an O(n) erase on a vector. A hard cap (applied
  // in feed()) additionally guarantees this never grows without bound even
  // if the line is flooded with non-protocol noise that contains no MAGIC.
  std::deque<uint8_t> buffer_;
  uint64_t bad_frames_ = 0;
};

}  // namespace usv_radio_bridge
