# Khôi phục đường truyền LiDAR 320 tia trên ESP32

Firmware dùng 320 giá trị khoảng cách và publisher BEST_EFFORT. Bản quét
khoảng 1.360 byte không vừa thư viện micro_ros_arduino gốc MTU 512 byte.
Triệu chứng: topic scan tồn tại nhưng không có message; `scan_hz` vẫn có số
và bộ đếm `pub_fail` tăng gần theo tốc độ quay LiDAR.

`#define MICROROS_MTU 1472` trong sketch không thay đổi được bộ đệm bên trong
thư viện đã biên dịch. Firmware hiện đọc trực tiếp
`UXR_CONFIG_CUSTOM_TRANSPORT_MTU` từ thư viện, dừng biên dịch nếu MTU không
đủ cho số tia. PID và độ phân giải 320 tia được giữ nguyên.

## Tái tạo thư viện trên laptop Linux

Yêu cầu: Arduino CLI và ESP32 core đã cài, Python 3, Git, Internet,
micro_ros_arduino **2.0.7-iron** tại `~/Arduino/libraries/micro_ros_arduino`.

```bash
cd /home/tai/Robot_kho_clone
python3 tools/rebuild_microros_mtu.py --install
```

Script đọc commit từ `built_packages`, tải đúng nguồn XRCE Client và RMW,
biên dịch lại tất cả 71 object của hai thành phần cùng với header MTU mới.
Các thành phần ROS message và thành phần không phụ thuộc MTU được tái sử dụng.
Script kiểm tra danh sách archive, tập symbol và hằng MTU trong mã máy trước
khi thay thư viện. Bản cũ được lưu vào `~/firmware_backups/microros_<thời gian>/`.
Nguồn, object, mã máy và `rebuild_report.json` nằm trong
`~/microros_mtu1472_build/`. Không sửa riêng header mà bỏ qua việc biên dịch lại.

Sau đó biên dịch sạch và nạp xe 2 (đúng cổng USB thực tế):

```bash
arduino-cli compile --clean \
  --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  --build-path /tmp/agv2_firmware_mtu1472 firmware/xe2
arduino-cli upload --port /dev/ttyUSB0 \
  --fqbn esp32:esp32:esp32:PartitionScheme=huge_app \
  --input-dir /tmp/agv2_firmware_mtu1472 firmware/xe2
```

Hai file `firmware/xeN/wifi_config.h` là cấu hình cục bộ, không đưa lên Git.

## Xe 1 đang bảo trì

`firmware/xe1/xe1.ino` đã có cùng kiểm tra MTU, vẫn chọn `XE 1`.
Đã đối chiếu toàn bộ file với bản Git trước sửa: chỉ thay phần khai báo/kiểm
tra MTU; GPIO, thứ tự bánh, đảo chiều encoder/động cơ và PID đều giữ nguyên.
Xe 1 chỉ được biên dịch chuẩn bị; chưa nạp và chưa kiểm tra trên phần cứng.
Không dùng binary của xe 2 để nạp xe 1. Khi xe về, biên dịch/nạp từ thư mục
`firmware/xe1` bằng thư viện MTU 1472 đã khôi phục.

## Kiểm chứng

- Đã biên dịch lại thư viện thật: kiểm tra hằng 1472 trong
  `uxr_init_custom_transport`, không chỉ nhìn file header.
- Phép thử bàn: sketch `firmware/tests/microros_mtu_probe/` phát 320 khoảng
  cách giả lập lên **`/bench_mtu/scan_synthetic`** để kiểm tra truyền đủ gói.
  Đây không phải dữ liệu LiDAR thật, không dùng làm bằng chứng đo cảm biến.
  Sketch không có điều khiển động cơ. Sau phép thử phải nạp lại `firmware/xe2`.
- Kiểm tra xe thật sau khi lắp lại ESP32: `/agv2/scan` có 320 tia, tốc độ
  phù hợp `scan_hz`, số điểm hợp lệ khác 0; `pub_fail` không tăng do gửi scan.
  Đồng thời kiểm tra `imu_ok`, nhịp encoder/IMU, khoảng ngắt dữ liệu và reset.
  Qua kiểm tra gói giả lập chưa đủ để kết luận toàn bộ hệ xe đã ổn định.
