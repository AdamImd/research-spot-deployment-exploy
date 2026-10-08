#pragma once
#include <stdint.h>

// Portable state machine. No firmware/host command can reset the physical latch.
class StopLatch {
 public:
  static constexpr uint32_t kLinkTimeoutMs = 250;
  static constexpr uint32_t kSafeStableMs = 500;
  bool armed = false;

  void trip() {
    armed = false;
    reset_ready_ = false;
  }

  void query(uint32_t now) {
    if (!queried_ || uint32_t(now - last_query_) >= kLinkTimeoutMs) trip();
    last_query_ = now;
    queried_ = true;
  }

  void update(bool circuit_closed, bool reset_pressed, uint32_t now) {
    if (!circuit_closed) {
      safe_seen_ = false;
      trip();
      return;
    }
    if (!safe_seen_) {
      safe_seen_ = true;
      safe_since_ = now;
    }
    if (!queried_ || uint32_t(now - last_query_) >= kLinkTimeoutMs) {
      trip();
      return;
    }
    if (!reset_pressed) reset_ready_ = true;
    if (reset_pressed && reset_ready_) {
      if (uint32_t(now - safe_since_) >= kSafeStableMs) armed = true;
      reset_ready_ = false;  // A held switch cannot retry or rearm.
    }
  }

 private:
  bool reset_ready_ = false, safe_seen_ = false, queried_ = false;
  uint32_t last_query_ = 0, safe_since_ = 0;
};

inline uint32_t wire_crc32(const char* text) {
  uint32_t crc = 0xffffffffu;
  for (; *text; ++text) {
    crc ^= static_cast<uint8_t>(*text);
    for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return crc ^ 0xffffffffu;
}
