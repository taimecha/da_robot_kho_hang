// Xe 1: so sanh byte UART thuc nhan voi packet/diem/vong quet do thu vien LDS giai ma.
// Chi doc LiDAR; khong khoi dong Wi-Fi, micro-ROS hay bo dieu khien banh xe.
// Sau khi do, nap lai firmware/xe1/xe1.ino.

#include <Arduino.h>
#include "../../esp32_agv_base/ydlidar_x2_intensity.h"

static HardwareSerial lidar_uart(2);
static LDS_YDLIDAR_X2_Intensity lidar;

static const int LIDAR_RX_PIN = 16;  // TX LiDAR -> RX ESP32
static const int LIDAR_PWM_PIN = 4;

static uint32_t raw_bytes = 0;
static uint32_t packets = 0;
static uint32_t ring_packets = 0;
static uint32_t points = 0;
static uint32_t valid_points = 0;
static uint32_t completed_scans = 0;
static uint32_t errors = 0;
static uint32_t raw_frames = 0, raw_checksum_ok = 0, raw_checksum_bad = 0;
static uint32_t raw_ring_headers = 0;
static uint8_t frame[90];
static uint8_t frame_n = 0, frame_expected = 0;
static uint8_t bad_frame[90], bad_frame_n = 0;
static uint8_t good_frame[90], good_frame_n = 0;
static uint16_t last_expected_checksum = 0, last_received_checksum = 0;
static volatile uint32_t uart_break_errors = 0, uart_buffer_errors = 0;
static volatile uint32_t uart_fifo_errors = 0, uart_frame_errors = 0;
static volatile uint32_t uart_parity_errors = 0;
static uint8_t intensity_frame[130], intensity_n = 0, intensity_expected = 0;
static uint32_t intensity_frames = 0, intensity_checksum_ok = 0;
static uint32_t intensity_checksum_bad = 0, intensity_valid_points = 0;
static uint32_t intensity_ring_frames = 0;
static uint8_t intensity_good_frame[130], intensity_good_frame_n = 0;
static uint8_t raw_sample[16], packet_sample[12];
static uint8_t raw_sample_n = 0, packet_sample_n = 0;
static float sample_angle = 0, sample_distance_mm = 0;
static int last_error_code = 0;

// Giai ma khung AA 55 doc lap voi LDS: 10 byte header + N khoang cach 16 bit.
// Checksum YDLIDAR la XOR 16 bit cua header (bo truong checksum) va N mau.
static void inspect_raw_byte(uint8_t value) {
  if (frame_n == 0) {
    if (value == 0xAA) frame[frame_n++] = value;
    return;
  }
  if (frame_n == 1) {
    if (value == 0x55) frame[frame_n++] = value;
    else frame_n = value == 0xAA ? 1 : 0;
    return;
  }
  frame[frame_n++] = value;
  if (frame_n == 4) {
    uint8_t samples = frame[3];
    if (samples == 0 || samples > 40) {
      frame_n = frame_expected = 0;
      return;
    }
    frame_expected = 10 + 2 * samples;
  }
  if (!frame_expected || frame_n < frame_expected) return;

  ++raw_frames;
  if (frame[2] & 1) ++raw_ring_headers;
  uint16_t checksum = 0x55AA;
  checksum ^= (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  checksum ^= (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  checksum ^= (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
  for (uint8_t i = 10; i < frame_expected; i += 2)
    checksum ^= (uint16_t)frame[i] | ((uint16_t)frame[i + 1] << 8);
  uint16_t received = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8);
  if (checksum == received) {
    ++raw_checksum_ok;
    if (good_frame_n == 0) {
      good_frame_n = frame_expected;
      memcpy(good_frame, frame, good_frame_n);
    }
  }
  else {
    ++raw_checksum_bad;
    last_expected_checksum = checksum;
    last_received_checksum = received;
    if (bad_frame_n == 0) {
      bad_frame_n = frame_expected;
      memcpy(bad_frame, frame, bad_frame_n);
    }
  }
  frame_n = frame_expected = 0;
}

// Giao thuc YDLIDAR khi co cuong do phan xa: moi mau gom 1 byte intensity
// va 2 byte khoang cach, theo tai lieu chinh thuc cua hang.
static void inspect_intensity_byte(uint8_t value) {
  if (intensity_n == 0) {
    if (value == 0xAA) intensity_frame[intensity_n++] = value;
    return;
  }
  if (intensity_n == 1) {
    if (value == 0x55) intensity_frame[intensity_n++] = value;
    else intensity_n = value == 0xAA ? 1 : 0;
    return;
  }
  intensity_frame[intensity_n++] = value;
  if (intensity_n == 4) {
    uint8_t samples = intensity_frame[3];
    if (samples == 0 || samples > 40) {
      intensity_n = intensity_expected = 0;
      return;
    }
    intensity_expected = 10 + 3 * samples;
  }
  if (!intensity_expected || intensity_n < intensity_expected) return;

  ++intensity_frames;
  uint16_t checksum = 0x55AA;
  checksum ^= (uint16_t)intensity_frame[2] | ((uint16_t)intensity_frame[3] << 8);
  checksum ^= (uint16_t)intensity_frame[4] | ((uint16_t)intensity_frame[5] << 8);
  checksum ^= (uint16_t)intensity_frame[6] | ((uint16_t)intensity_frame[7] << 8);
  for (uint8_t i = 10; i < intensity_expected; i += 3) {
    checksum ^= intensity_frame[i];
    checksum ^= (uint16_t)intensity_frame[i + 1] | ((uint16_t)intensity_frame[i + 2] << 8);
  }
  uint16_t received = (uint16_t)intensity_frame[8] | ((uint16_t)intensity_frame[9] << 8);
  if (checksum == received) {
    ++intensity_checksum_ok;
    if (intensity_good_frame_n == 0) {
      intensity_good_frame_n = intensity_expected;
      memcpy(intensity_good_frame, intensity_frame, intensity_good_frame_n);
    }
    if (intensity_frame[2] & 1) ++intensity_ring_frames;
    for (uint8_t i = 10; i < intensity_expected; i += 3) {
      uint16_t distance = ((uint16_t)intensity_frame[i + 1] |
                           ((uint16_t)intensity_frame[i + 2] << 8)) >> 2;
      if (distance >= 100 && distance <= 8000) ++intensity_valid_points;
    }
  } else ++intensity_checksum_bad;
  intensity_n = intensity_expected = 0;
}

static int read_lidar() {
  int value = lidar_uart.read();
  if (value >= 0) {
    ++raw_bytes;
    if (raw_sample_n < sizeof(raw_sample)) raw_sample[raw_sample_n++] = value;
    inspect_raw_byte((uint8_t)value);
    inspect_intensity_byte((uint8_t)value);
  }
  return value;
}

static size_t write_lidar(const uint8_t *data, size_t length) {
  return lidar_uart.write(data, length);
}

static void on_packet(uint8_t *data, uint16_t length, bool scan_completed) {
  ++packets;
  if (scan_completed) ++ring_packets;
  if (packet_sample_n == 0) {
    packet_sample_n = min((size_t)length, sizeof(packet_sample));
    memcpy(packet_sample, data, packet_sample_n);
  }
}

static void on_point(float angle, float distance_mm, float quality, bool scan_completed) {
  (void)quality;
  ++points;
  if (scan_completed) ++completed_scans;
  if (distance_mm >= 100.0f && distance_mm <= 8000.0f) {
    ++valid_points;
    sample_angle = angle;
    sample_distance_mm = distance_mm;
  }
}

static void on_motor(float value, LDS::lds_pin_t pin) {
  if (pin != LDS::LDS_MOTOR_PWM_PIN) return;
  // X2/X2L dung DIR_INPUT de mo motor, giong firmware chinh cua xe 1.
  if (value == (float)LDS::DIR_INPUT) pinMode(LIDAR_PWM_PIN, INPUT);
  else if (value == (float)LDS::DIR_OUTPUT_CONST) pinMode(LIDAR_PWM_PIN, OUTPUT);
  else if (value == (float)LDS::VALUE_HIGH) digitalWrite(LIDAR_PWM_PIN, HIGH);
  else if (value == (float)LDS::VALUE_LOW) digitalWrite(LIDAR_PWM_PIN, LOW);
}

static void on_info(LDS::info_t code, String message) {
  Serial.printf("LDS_INFO %s: %s\n", lidar.infoCodeToString(code).c_str(), message.c_str());
}

static void on_error(LDS::result_t code, String message) {
  (void)message;
  ++errors;
  last_error_code = (int)code;
}

static void print_hex(const char *label, const uint8_t *data, uint8_t length) {
  Serial.print(label);
  for (uint8_t i = 0; i < length; ++i) Serial.printf(" %02X", data[i]);
  Serial.println();
}

void setup() {
  // De hai driver dong co o trang thai dung, ke ca khi xe dang cap nguon pin.
  const int motor_pins[] = {27, 26, 25, 33, 32, 14};
  for (int pin : motor_pins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  Serial.begin(115200);
  delay(400);
  Serial.println("LIDAR_UART_PROBE xe 1: RX=GPIO16, LiDAR PWM=GPIO4, UART=115200");
  Serial.println("raw=byte UART doc duoc; packet/ring/point=thu vien LDS giai ma");
  Serial.println("Khong dieu khien banh xe. Nap lai firmware/xe1 sau khi do.");

  lidar_uart.setRxBufferSize(1024);
  lidar_uart.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX_PIN, -1);
  lidar_uart.onReceiveError([](hardwareSerial_error_t error) {
    if (error == UART_BREAK_ERROR) ++uart_break_errors;
    else if (error == UART_BUFFER_FULL_ERROR) ++uart_buffer_errors;
    else if (error == UART_FIFO_OVF_ERROR) ++uart_fifo_errors;
    else if (error == UART_FRAME_ERROR) ++uart_frame_errors;
    else if (error == UART_PARITY_ERROR) ++uart_parity_errors;
  });
  lidar.setSerialReadCallback(read_lidar);
  lidar.setSerialWriteCallback(write_lidar);
  lidar.setPacketCallback(on_packet);
  lidar.setScanPointCallback(on_point);
  lidar.setMotorPinCallback(on_motor);
  lidar.setInfoCallback(on_info);
  lidar.setErrorCallback(on_error);
  lidar.init();
  LDS::result_t result = lidar.start();
  Serial.printf("LDS_START %s\n", lidar.resultCodeToString(result).c_str());
}

void loop() {
  lidar.loop();
  static uint32_t last_report = 0;
  if (millis() - last_report < 2000) return;
  last_report = millis();

  static uint32_t prev_raw = 0, prev_packets = 0, prev_ring = 0;
  static uint32_t prev_points = 0, prev_valid = 0, prev_scans = 0, prev_errors = 0;
  static uint32_t prev_frames = 0, prev_checksum_ok = 0, prev_checksum_bad = 0;
  static uint32_t prev_intensity_frames = 0, prev_intensity_ok = 0;
  Serial.printf("PROBE t=%lus raw=%lu (+%lu/2s) uart_wait=%d packets=%lu (+%lu) ring=%lu (+%lu) "
                "points=%lu (+%lu) valid=%lu (+%lu) scans=%lu (+%lu) freq=%.2fHz errors=%lu (+%lu) last_error=%d\n",
                (unsigned long)(millis() / 1000), (unsigned long)raw_bytes,
                (unsigned long)(raw_bytes - prev_raw), lidar_uart.available(),
                (unsigned long)packets, (unsigned long)(packets - prev_packets),
                (unsigned long)ring_packets, (unsigned long)(ring_packets - prev_ring),
                (unsigned long)points, (unsigned long)(points - prev_points),
                (unsigned long)valid_points, (unsigned long)(valid_points - prev_valid),
                (unsigned long)completed_scans, (unsigned long)(completed_scans - prev_scans),
                lidar.getCurrentScanFreqHz(), (unsigned long)errors,
                (unsigned long)(errors - prev_errors), last_error_code);
  if (raw_sample_n) print_hex("RAW_HEX", raw_sample, raw_sample_n);
  if (packet_sample_n) print_hex("PACKET_HEX", packet_sample, packet_sample_n);
  if (good_frame_n) print_hex("GOOD_FRAME_2B", good_frame, good_frame_n);
  if (bad_frame_n) print_hex("BAD_FRAME", bad_frame, bad_frame_n);
  if (intensity_good_frame_n) print_hex("GOOD_FRAME_3B", intensity_good_frame,
                                        intensity_good_frame_n);
  Serial.printf("RAW_FRAME candidates=%lu (+%lu) checksum_ok=%lu (+%lu) "
                "checksum_bad=%lu (+%lu) ring_headers=%lu last_bad_calc=%04X last_bad_rx=%04X\n",
                (unsigned long)raw_frames, (unsigned long)(raw_frames - prev_frames),
                (unsigned long)raw_checksum_ok, (unsigned long)(raw_checksum_ok - prev_checksum_ok),
                (unsigned long)raw_checksum_bad, (unsigned long)(raw_checksum_bad - prev_checksum_bad),
                (unsigned long)raw_ring_headers, last_expected_checksum, last_received_checksum);
  Serial.printf("UART_ERRORS break=%lu buffer=%lu fifo=%lu frame=%lu parity=%lu\n",
                (unsigned long)uart_break_errors, (unsigned long)uart_buffer_errors,
                (unsigned long)uart_fifo_errors, (unsigned long)uart_frame_errors,
                (unsigned long)uart_parity_errors);
  Serial.printf("INTENSITY_3B frames=%lu (+%lu) checksum_ok=%lu (+%lu) "
                "checksum_bad=%lu ring=%lu valid_points=%lu\n",
                (unsigned long)intensity_frames,
                (unsigned long)(intensity_frames - prev_intensity_frames),
                (unsigned long)intensity_checksum_ok,
                (unsigned long)(intensity_checksum_ok - prev_intensity_ok),
                (unsigned long)intensity_checksum_bad,
                (unsigned long)intensity_ring_frames,
                (unsigned long)intensity_valid_points);
  if (valid_points) Serial.printf("POINT_SAMPLE angle=%.1f deg distance=%.0f mm\n",
                                  sample_angle, sample_distance_mm);
  raw_sample_n = packet_sample_n = 0;
  bad_frame_n = good_frame_n = intensity_good_frame_n = 0;
  prev_raw = raw_bytes; prev_packets = packets; prev_ring = ring_packets;
  prev_points = points; prev_valid = valid_points; prev_scans = completed_scans;
  prev_errors = errors;
  prev_frames = raw_frames; prev_checksum_ok = raw_checksum_ok;
  prev_checksum_bad = raw_checksum_bad;
  prev_intensity_frames = intensity_frames;
  prev_intensity_ok = intensity_checksum_ok;
}
