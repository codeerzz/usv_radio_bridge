#include "usv_radio_bridge/serial_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace usv_radio_bridge {

namespace {

speed_t baud_to_speed(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      throw std::runtime_error(
          "unsupported baud_rate " + std::to_string(baud_rate) +
          " (add it to serial_transport.cpp's baud_to_speed() if your radio needs it)");
  }
}

}  // namespace

SerialTransport::~SerialTransport()
{
  close();
}

void SerialTransport::open(const std::string & device, int baud_rate)
{
  close();

  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error(
        "failed to open " + device + ": " + std::strerror(errno));
  }

  // Drop O_NONBLOCK now that the device is open: reads below rely on
  // VMIN/VTIME (see termios settings) for their timeout behaviour, not on
  // O_NONBLOCK's "return immediately" semantics.
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags != -1) {
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("tcgetattr(" + device + ") failed: " + err);
  }

  const speed_t speed = baud_to_speed(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // Raw mode, 8N1, no flow control — a plain binary byte pipe with nothing
  // in the kernel driver interpreting/transforming our frame bytes.
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);   // 1 stop bit
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);   // no parity
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);  // no hardware flow control

  // VMIN=0, VTIME=1: read() returns after up to one decisecond (100 ms) if
  // no bytes have arrived, or as soon as at least one byte is available.
  // termios timeouts are only ever specified in deciseconds, so this is the
  // closest achievable match to the Python reference's pyserial
  // timeout=0.05 (50 ms) — the extra ~50 ms only affects how quickly the
  // reader thread notices a shutdown request or a stale-link condition, it
  // has no effect on protocol correctness (FrameParser is fully incremental
  // regardless of how bytes are chunked across reads).
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("tcsetattr(" + device + ") failed: " + err);
  }

  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
}

void SerialTransport::close()
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  const int fd = fd_.exchange(-1);
  if (fd >= 0) {
    ::close(fd);
  }
}

bool SerialTransport::is_open() const
{
  return fd_.load() >= 0;
}

std::size_t SerialTransport::read(uint8_t * buffer, std::size_t max_length)
{
  const int fd = fd_.load();
  if (fd < 0) {
    throw std::runtime_error("read() called on a closed SerialTransport");
  }

  const ssize_t n = ::read(fd, buffer, max_length);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;  // no data within this read's VTIME window — not an error
    }
    throw std::runtime_error(std::string("serial read() failed: ") + std::strerror(errno));
  }
  return static_cast<std::size_t>(n);
}

bool SerialTransport::write_frame(const std::vector<uint8_t> & frame)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0) {
    return false;
  }

  std::size_t written = 0;
  while (written < frame.size()) {
    const ssize_t n = ::write(fd_, frame.data() + written, frame.size() - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(n);
  }

  // Mirrors the Python reference's ser.flush(): wait for the bytes to
  // actually leave the driver so a stalled radio can't silently pile up an
  // unbounded amount of unsent data in a kernel buffer.
  tcdrain(fd_);
  return true;
}

}  // namespace usv_radio_bridge
