# Đo bộ giải mã LiDAR X2/X2L trên xe 1

Sketch này chỉ đọc LiDAR qua UART2 (RX GPIO16, PWM GPIO4). Nó không kết nối
Wi-Fi/micro-ROS và không điều khiển bánh xe. Sau khi đo, nạp lại
`firmware/xe1/xe1.ino` trước khi chạy xe bình thường.

1. Cắm USB ESP32 xe 1, xác định cổng bằng `arduino-cli board list`.
2. Biên dịch và nạp sketch (thay `/dev/ttyUSB0` bằng cổng vừa tìm):

   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32 firmware/tests/lidar_uart_probe
   arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 firmware/tests/lidar_uart_probe
   arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
   ```

3. Để LiDAR quay 10 giây, sao chép toàn bộ các dòng từ `LDS_START` đến ít nhất
   hai dòng `PROBE`. Cần đặc biệt các dòng `RAW_FRAME`, `INTENSITY_3B`,
   `UART_ERRORS`, `GOOD_FRAME_2B`, `GOOD_FRAME_3B` và `BAD_FRAME` nếu có.

`raw` đếm byte thật từ UART; `RAW_FRAME checksum_ok` đếm gói 2 byte/mẫu hợp lệ;
`INTENSITY_3B checksum_ok` đếm gói 3 byte/mẫu hợp lệ; `packets`, `points` và
`scans` là kết quả của bộ giải mã đang dùng trong firmware chính. Với LiDAR xe 1,
`INTENSITY_3B checksum_ok` phải gần bằng `packets`, và `valid_points` của hai
bộ đếm phải khớp. Có byte
nhưng không có gói hợp lệ cần kiểm tra định dạng gói, baud và lỗi UART. Có gói
hợp lệ nhưng thư viện không ra điểm mới là bằng chứng cụ thể để sửa bộ giải mã.
Có điểm quét mà ROS không nhận `/scan` thì cần kiểm tra bước xuất bản micro-ROS.

Không dùng kết quả probe này làm xác nhận định vị: cần kiểm tra tiếp `/agv1/scan`
và độ khớp bản đồ sau khi nạp lại firmware chính.
