#ifndef UBLOX_GPS_TEST_FAKE_RECEIVER_HPP
#define UBLOX_GPS_TEST_FAKE_RECEIVER_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ublox_serialization/serialization.hpp>

namespace ublox_test {

/// One UBX frame as seen on the wire, without sync chars, length or checksum
struct Frame {
  uint8_t class_id;
  uint8_t message_id;
  std::vector<uint8_t> payload;
};

inline std::vector<uint8_t> encodeFrame(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> out(payload.size() + 8);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(payload.data(), payload.size(), class_id, message_id)) {
    throw std::runtime_error("Failed to encode UBX frame");
  }
  return out;
}

/**
 * @brief A u-blox receiver behind a local TCP socket, the way the driver sees a
 * device at tcp://host:port.
 *
 * It records every UBX frame the driver writes, ACKs (or NACKs) configuration
 * messages, answers the MON-VER and CFG-GNSS polls the driver makes during
 * startup like a ZED-F9P on HPG 1.32 would, and lets a test push frames to
 * the driver.
 */
class FakeReceiver {
 public:
  FakeReceiver() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(listen_fd_, 1) != 0) {
      ::close(listen_fd_);
      throw std::runtime_error("FakeReceiver could not listen");
    }
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread(&FakeReceiver::run, this);
  }

  ~FakeReceiver() {
    stopping_ = true;
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      if (conn_fd_ >= 0) {
        ::shutdown(conn_fd_, SHUT_RDWR);
      }
    }
    thread_.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (std::thread& replier : repliers_) {
        replier.join();
      }
    }
    if (conn_fd_ >= 0) {
      ::close(conn_fd_);
    }
  }

  std::string url() const { return "tcp://127.0.0.1:" + std::to_string(port_); }

  /// Answer the configuration messages that match pred with ACK-NACK
  void nackIf(std::function<bool(const Frame&)> pred) {
    std::lock_guard<std::mutex> lock(mutex_);
    nack_if_ = std::move(pred);
  }

  /// Write a UBX frame to the driver, as if the receiver output it
  void send(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload) {
    sendRaw(encodeFrame(class_id, message_id, payload));
  }

  /// Every frame received so far
  std::vector<Frame> frames() {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_;
  }

  /// Raw bytes received so far, in order
  std::vector<uint8_t> bytes() {
    std::lock_guard<std::mutex> lock(mutex_);
    return bytes_;
  }

  /// Frames with this class and ID, in order
  std::vector<Frame> framesOf(uint8_t class_id, uint8_t message_id) {
    std::vector<Frame> out;
    for (const Frame& f : frames()) {
      if (f.class_id == class_id && f.message_id == message_id) {
        out.push_back(f);
      }
    }
    return out;
  }

  /// Wait until pred holds on the received frames, or the timeout passes
  static bool waitFor(const std::function<bool()>& pred, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!pred()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  /// Wait until the driver has connected
  bool waitForConnection(std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] { return connected_; });
  }

 private:
  void sendRaw(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (conn_fd_ >= 0) {
      ::send(conn_fd_, data.data(), data.size(), MSG_NOSIGNAL);
    }
  }

  void run() {
    int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      conn_fd_ = fd;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      connected_ = true;
    }
    cv_.notify_all();

    std::vector<uint8_t> buffer;
    uint8_t chunk[4096];
    while (!stopping_) {
      ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
      if (n <= 0) {
        break;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        bytes_.insert(bytes_.end(), chunk, chunk + n);
      }
      buffer.insert(buffer.end(), chunk, chunk + n);
      parse(buffer);
    }
  }

  void parse(std::vector<uint8_t>& buffer) {
    std::size_t i = 0;
    while (buffer.size() - i >= 8) {
      if (buffer[i] != 0xb5 || buffer[i + 1] != 0x62) {
        ++i;
        continue;
      }
      const std::size_t length = buffer[i + 4] | (buffer[i + 5] << 8);
      if (buffer.size() - i < length + 8) {
        break;
      }
      Frame f{buffer[i + 2], buffer[i + 3],
              std::vector<uint8_t>(buffer.begin() + i + 6, buffer.begin() + i + 6 + length)};
      i += length + 8;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(f);
      }
      respond(f);
    }
    buffer.erase(buffer.begin(), buffer.begin() + i);
  }

  void respond(const Frame& f) {
    const bool is_poll = f.payload.empty();
    if (f.class_id == 0x0a && f.message_id == 0x04 && is_poll) {
      replyToPoll(0x0a, 0x04, monVer());
    } else if (f.class_id == 0x06 && f.message_id == 0x3e && is_poll) {
      // CFG-GNSS with no blocks, so the driver sees nothing to change
      replyToPoll(0x06, 0x3e, {0x00, 0x20, 0x20, 0x00});
    } else if (f.class_id == 0x06) {
      bool nack;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        nack = nack_if_ && nack_if_(f);
      }
      send(0x05, nack ? 0x00 : 0x01, {f.class_id, f.message_id});
    }
  }

  void replyToPoll(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload) {
    // The driver registers its reader only after sending the poll, so answer a
    // little later, as a real receiver on a serial link would. A thread of its
    // own keeps the frames queued behind the poll flowing meanwhile
    std::lock_guard<std::mutex> lock(mutex_);
    repliers_.emplace_back([this, class_id, message_id, payload] {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      send(class_id, message_id, payload);
    });
  }

  static std::vector<uint8_t> monVer() {
    std::vector<uint8_t> payload;
    auto field = [&payload](const std::string& s, std::size_t size) {
      std::vector<uint8_t> f(size, 0);
      std::copy(s.begin(), s.end(), f.begin());
      payload.insert(payload.end(), f.begin(), f.end());
    };
    field("EXT CORE 1.00 (a59682)", 30);
    field("00190000", 10);
    for (const char* ext : {"ROM BASE 0x118B2060", "FWVER=HPG 1.32", "PROTVER=27.31", "MOD=ZED-F9P",
                            "GPS;GLO;GAL;BDS", "SBAS;QZSS"}) {
      field(ext, 30);
    }
    return payload;
  }

  int listen_fd_{-1};
  int conn_fd_{-1};
  uint16_t port_{0};
  std::atomic<bool> stopping_{false};
  bool connected_{false};
  std::thread thread_;
  std::vector<std::thread> repliers_;
  std::mutex mutex_;
  std::mutex write_mutex_;
  std::condition_variable cv_;
  std::vector<Frame> frames_;
  std::vector<uint8_t> bytes_;
  std::function<bool(const Frame&)> nack_if_;
};

}  // namespace ublox_test

#endif  // UBLOX_GPS_TEST_FAKE_RECEIVER_HPP
