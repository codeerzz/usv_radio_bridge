// Minimal POSIX termios serial port wrapper.
//
// No ROS C++ serial-port package exists anywhere in this workspace, so this
// stays a small, dependency-free wrapper rather than reaching for something
// unprecedented here. It intentionally does the bare minimum: open/configure,
// blocking-with-timeout read, mutex-guarded write. All the retry/backoff and
// "keep the node alive while the radio is unplugged" policy lives in
// RadioBridgeNode, not here — this class just throws on failure and lets the
// caller decide what "unplugged" should mean for the rest of the node.
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace usv_radio_bridge {

class SerialTransport
{
public:
  SerialTransport() = default;
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  // Opens and configures the port (raw mode, 8N1, requested baud rate).
  // Throws std::runtime_error on any failure. Callers must catch this —
  // an absent/unplugged radio must never be allowed to crash the node.
  void open(const std::string & device, int baud_rate);

  // Idempotent; safe to call even if never opened / already closed.
  void close();

  bool is_open() const;

  // Blocking read with a short internal timeout (see .cpp for the exact
  // value and why it isn't the Python reference's precise 50 ms). Returns
  // the number of bytes read into `buffer` (0 on timeout — not an error).
  // Throws std::runtime_error if the underlying fd reports a real I/O error
  // (e.g. the USB-serial adapter was unplugged), so callers can tell "no
  // data yet" apart from "the device is gone".
  std::size_t read(uint8_t * buffer, std::size_t max_length);

  // Thread-safe (internally mutex-guarded) write of one complete frame, so
  // the RX reader thread and the TX timer — both of which can touch this
  // transport — never interleave partial writes on the wire. Returns false
  // (and leaves the port open) on a write error; the caller logs and moves
  // on rather than tearing the whole node down over one failed TX.
  bool write_frame(const std::vector<uint8_t> & frame);

private:
  // Atomic (not just mutex-guarded) because read() — called from the reader
  // thread — checks/uses it without taking write_mutex_, to avoid blocking
  // TX writes for the whole duration of a blocking-with-timeout read.
  std::atomic<int> fd_{-1};
  mutable std::mutex write_mutex_;
};

}  // namespace usv_radio_bridge
