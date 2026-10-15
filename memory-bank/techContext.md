# Tech Context

## Phần cứng (mỗi xe)
| Phần | Chi tiết |
|---|---|
| MCU | ESP32 DevKit (`esp32:esp32:esp32`, core 3.3.8), Partition **Huge APP** |
| Động cơ | 2 động cơ DC có encoder qua **L298N** (tháo jumper ENA/ENB). Encoder ~2150 xung/vòng |
| Bánh | đường kính 64.5 mm; **bán kính lăn hiệu chỉnh 0.03281 m** (đo trên xe 1, chỉnh ở laptop `wheel_radius`); cách bánh 0.200 m; thân Ø181.4 mm |
| LiDAR | YDLIDAR **X2/X2L** (4 chân VCC/GND/Tx/PWM), UART2 RX = GPIO16, 115200; ~7 Hz; firmware resample 320 bin × 1.125°; cao 0.075 m |
| IMU | **BNO055** I2C SDA 21 / SCL 22, địa chỉ **0x29**, chế độ NDOF. Đứng yên 2 s đầu sau khi cấp nguồn để đo bias gyro |
| Nguồn | Li-ion **3S** (đầy 12.6 V) → L298N 12 V; mạch hạ áp 5 V → ESP32 VIN + LiDAR. Encoder & BNO055 cấp **3V3** |

### Khác nhau giữa 2 xe (chọn bằng `#define XE` trong firmware)
| | XE 1 (mạch cũ) | XE 2 (mạch in 2026-10-05) |
|---|---|---|
| Encoder L A/B, R A/B | 23/34, 4/35 (không kéo lên) | 23/18, 19/17 (kéo lên nội) |
| PWM LiDAR | GPIO 18 | GPIO 4 |
| Động cơ L (IN1/IN2/EN), R | 27/26/25, 33/32/14 | **đổi chéo**: L = 33/32/14, R = 27/26/25 (dây nối chéo trên mạch mới) |
| LiDAR bù góc | 180° | 180° (đo: chai trước mũi ở 1.1°) |
| LiDAR nhân khoảng cách | 1.0 | **1.062** (X2 của xe 2 đo thiếu ~6%; ICP với bản đồ: 1.0623 ± 0.0008) |
| micro-ROS | node `/agv1/esp32_agv_base_1`, client key 0x11111111 | `/agv2/esp32_agv_base_2`, 0x22222222 |

`INVERT_MOTOR_L = true`, `INVERT_ENC_L = true`, phía phải false (giống nhau cả 2 xe).
Firmware XE 1 biên dịch ra đúng kích thước bản cũ "1 Xe" khi chưa có namespace (đã đối chiếu).

### Firmware (`firmware/esp32_agv_base/esp32_agv_base.ino`)
- Bản sao để nạp: `D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino` + `wifi_config.h`
  cùng thư mục (`WIFI_SSID`, `WIFI_PASS`, `AGENT_IP`; **không vào git**).
- Core 1: chỉ `lidar.loop()`. Core 0: `motor_task` 50 Hz (PID + feedforward **Kf 3.0**,
  `INTEGRAL_MAX 200`), `ros_task` (executor, publish, đọc BNO055).
- Phát: `scan` (best-effort), `imu/data` (best-effort), `joint_states` (**góc bánh tích
  luỹ**, mất gói không mất quãng đường). Nhận `cmd_vel` (best-effort; nguồn reliable vẫn tới).
  `ODOM_ON_ESP32 0`: odometry tính ở laptop.
- Bảo vệ → **ESTOP chốt tới khi reset** (in Serial `!!! ESTOP`): kẹt bánh (PWM ≥ 200, |rpm| < 1
  trong 1 s) hoặc quay ngược chiều PWM (PWM ≥ 120, rpm ngược dấu > 5 trong 0.3 s).
  Mất `cmd_vel` 0.5 s → dừng động cơ.
- Sketch phụ: `firmware/bno055_test` (gửi số liệu qua **TCP 2323**, xem bằng
  `tools/bno055_wifi.py`), `firmware/encoder_test` (**còn chân cũ** 23/34/4/35).

## Phần mềm / môi trường
- Laptop Windows 11 (Insider 26300), GPU AMD RX 6700M. **WSL2 Ubuntu 24.04**, ROS 2 **Jazzy**.
- `.wslconfig`: `networkingMode=mirrored`, `hostAddressLoopback=true`. Laptop IP
  192.168.1.21 (= `AGENT_IP`). Xe 1 từng có IP 192.168.1.150.
- `~/.bashrc`: source `/opt/ros/jazzy`, `~/microros_ws/install` (micro_ros_agent tự build,
  không có gói apt), `~/robot_ws_3/install`; `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  (bắt buộc — multicast WSL hỏng); `GALLIUM_DRIVER=d3d12` (RViz/Gazebo vẽ bằng GPU);
  `fix_wsl_clock.sh` (đồng hồ WSL trôi).
- `C:\Users\LRGION\.wslgconfig`: `WESTON_RDP_COPY_WARNING_TITLE=false` (ẩn chữ COPY MODE).
- Workspace `~/robot_ws_3`, `colcon build --symlink-install` (file mới trong `scripts/` cần
  build lại để có symlink). Gói: `my_bot_bringup` (launch, config, `scripts/wheel_odometry.py`,
  `scripts/ns_relay.py`), `my_bot_description` (URDF xacro, `sim:=false` cho xe thật).
- Bản đồ thật: `src/my_bot_bringup/config/real_room_map.yaml` (0.05 m/ô, 159×165).
  Sa bàn: `config/grid_real.yaml` (tâm nút (3,3) = (−0.545, +0.035), `yaw_deg` 0,
  `rotate.coast: 0.36`, `done_deg: 0.9`, xung góc nhỏ bật); mô phỏng: `grid_sim.yaml`.
- Arduino CLI có thể dùng để biên dịch kiểm tra: `arduino-cli compile --fqbn esp32:esp32:esp32 <thư mục sketch>`
  (thư mục phải có `wifi_config.h`).

## Môi trường vận hành tự động (2026-10-10) — xem `docs/AGVCTL.md`
- `tools/agvctl`: `doctor | up [--emulate] | down | status | errors | events | logs | goal --go |
  stop | localize | serial | fw build N | fw copy N`. Log mỗi lần `up`: `~/agv_logs/run-*/`
  (`latest` → mới nhất): `status.json`, `errors.log` (WARN+ từ /rosout), `events.log`.
- `tools/fleet_monitor.py`: 1 tiến trình rclpy, `raw=True`, không nghe imu (≈15% một nhân).
- Firmware: `/agvN/diag` (String JSON 1 Hz: estop, rssi, reset, rpm, pwm, pub, heap) và
  `/agvN/estop_reset` (Empty; chỉ gỡ khi bánh đã dừng). Executor 2 handle.
- Biên dịch từ WSL: `arduino-cli.exe` của IDE tại `D:\CTU\Ardunio ide\Arduino IDE\resources\app\lib\backend\resources\`,
  `--config-file C:\Users\LRGION\.arduinoIDE\arduino-cli.yaml`, FQBN
  `esp32:esp32:esp32:PartitionScheme=huge_app`, sketch tạm ở `%TEMP%\agv_fw` (chạy với
  `cwd=/mnt/c` để tránh cảnh báo UNC). Lần đầu ~5 phút. Kích thước 1.07 MB (33%).
- Serial: PowerShell `System.IO.Ports.SerialPort` (DTR/RTS tắt → không reset), `ReadExisting`
  (firmware in dấu chấm không xuống dòng). **COM3 = thiết bị Lenovo (VID_17EF), không phải
  ESP32**; ESP32 nhận theo VID 10C4/1A86/0403/303A.
- MCP: `uv` ở `~/.local/bin`; `.mcp.json` → `uvx ros-mcp==3.1.2` → rosbridge `127.0.0.1:9090`
  (`agvctl up` tự bật nếu đã cài `ros-jazzy-rosbridge-server` — cần sudo, người dùng cài).
- Dò tiến trình agent bằng `pgrep -x micro_ros_agent` (dò theo dòng lệnh khớp nhầm shell khác).

## Bẫy đã gặp (đọc trước khi chạy lệnh)
- **Sửa file qua `\\wsl.localhost\...` làm mất bit chạy** → `chmod +x` (và
  `git update-index --chmod=+x` cho file mới). Triệu chứng: `Permission denied`, launch báo
  "executable not found".
- Từ Git Bash gọi WSL: `MSYS_NO_PATHCONV=1 wsl.exe -e bash /mnt/c/...script.sh`. `/tmp` của
  Git Bash **khác** `/tmp` của WSL. Viết script ra file rồi chạy, tránh heredoc `python -`.
- `pkill -f` khớp luôn chính lệnh đang chạy → dùng mẫu neo `^python3 ...`.
- Sửa văn bản có `C:\Users\...` bằng `sed`: `\U`, `\L` bị hiểu là đổi hoa/thường. Vá file
  bằng Python thì dùng chuỗi raw (`\n`, `\1` bị biến thành ký tự điều khiển).
- `set -u` làm hỏng `setup.bash` của ROS.
- **DDS LOCALHOST giới hạn số tiến trình dò được**: quá ~30–40 tiến trình ROS trong một domain
  thì tiến trình bật sau "vô hình". Nav2 mỗi xe phải chạy **gộp container**.
- **rclpy `MultiThreadedExecutor`** + topic 50 Hz + một nhóm callback loại trừ → luồng rảnh quay
  vòng, 77–90% CPU. Node đơn luồng dùng `SingleThreadedExecutor`.
- Container Nav2 phải được nạp **file tham số** (costmap là node con), không thì costmap chạy
  mặc định ("no map received", "out of bounds").
- Nav2 local costmap phải `always_send_full_costmap: True` (xe đứng yên thì không phát bản đầy
  đủ → nút bị chặn không bao giờ xoá).
- `ros2` CLI (`node list`, `topic info`, nhất là `--no-daemon`) dò tìm thiếu khi nhiều tiến
  trình → kiểm tra bằng script rclpy chờ vài giây.
- RViz in đỏ `GLSL link result ... indexed_8bit_image`: vô hại; sửa bằng
  `sudo tools/fix_rviz_shader.sh`.
- ESP32 core 3.x: UART2 chỉ gán TX mặc định (GPIO17) khi cả RX và TX = −1 → GPIO17 dùng cho
  encoder được. ESP32Encoder mặc định kéo xuống.
- Đọc Serial COM3 không được khi xe chạy pin (rút USB) — chẩn đoán qua topic ROS.
