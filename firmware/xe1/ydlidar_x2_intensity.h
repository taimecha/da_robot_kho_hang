#pragma once

#include <LDS_YDLIDAR_X2_X2L.h>
#include <math.h>

// Some X2/X2L units send intensity + distance_q2 (3 bytes per sample).
// LDS 0.6.3 assumes 2 bytes per sample, so it loses packet alignment.
// Keep its motor and callback API, but decode the measured wire format here.
class LDS_YDLIDAR_X2_Intensity : public LDS_YDLIDAR_X2_X2L {
 protected:
  result_t waitScanDot() override {
    for (;;) {
      int incoming = readSerial();
      if (incoming < 0) return ERROR_NOT_READY;
      uint8_t byte = (uint8_t)incoming;
      if (length_ == 0) {
        if (byte == 0xAA) frame_[length_++] = byte;
        continue;
      }
      if (length_ == 1) {
        if (byte == 0x55) frame_[length_++] = byte;
        else length_ = byte == 0xAA ? 1 : 0;
        continue;
      }
      frame_[length_++] = byte;
      if (length_ == 4) {
        uint8_t count = frame_[3];
        if (count == 0 || count > 40) {
          length_ = expected_ = 0;
          continue;
        }
        expected_ = 10 + 3 * count;
      }
      if (length_ == 8 && ((frame_[4] & 1) == 0 || (frame_[6] & 1) == 0)) {
        length_ = expected_ = 0;
        continue;
      }
      if (!expected_ || length_ != expected_) continue;

      length_ = expected_ = 0;
      uint16_t checksum = 0x55AA;
      checksum ^= wordAt(2);
      checksum ^= wordAt(4);
      checksum ^= wordAt(6);
      for (uint8_t i = 10; i < 10 + 3 * frame_[3]; i += 3) {
        checksum ^= frame_[i];
        checksum ^= wordAt(i + 1);
      }
      if (checksum != wordAt(8)) return ERROR_CHECKSUM;

      bool ring_start = (frame_[2] & 1) != 0;
      if (ring_start) markScanTime();
      postPacket(frame_, 10 + 3 * frame_[3], ring_start);

      const uint8_t count = frame_[3];
      const float first = (float)(wordAt(4) >> 1);
      const float last = (float)(wordAt(6) >> 1);
      float interval = 0;
      if (count > 1) {
        float span = last - first;
        if (span < 0) span += 23040.0f;
        interval = span / (count - 1);
      }
      for (uint8_t i = 0; i < count; ++i) {
        uint8_t offset = 10 + 3 * i;
        uint16_t distance_q2 = wordAt(offset + 1);
        float distance_mm = distance_q2 * 0.25f;
        // Same distance dependent angle correction as LDS_YDLIDAR_X4.
        float correction_q6 = distance_q2 == 0 ? 0.0f :
            atanf((21.8f * (155.3f - distance_mm) / 155.3f) / distance_mm)
            * 3666.93f;
        float angle_q6 = first + interval * i + correction_q6;
        while (angle_q6 < 0) angle_q6 += 23040.0f;
        while (angle_q6 >= 23040.0f) angle_q6 -= 23040.0f;
        postScanPoint(angle_q6 / 64.0f, distance_mm, frame_[offset],
                      ring_start && i == 0);
      }
      return RESULT_OK;
    }
  }

 private:
  uint16_t wordAt(uint8_t offset) const {
    return (uint16_t)frame_[offset] | ((uint16_t)frame_[offset + 1] << 8);
  }

  uint8_t frame_[130] = {};
  uint8_t length_ = 0;
  uint8_t expected_ = 0;
};
