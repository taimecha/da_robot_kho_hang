# robot_ws_3 — Ngữ cảnh gọn (đọc trước)

**Là gì:** AGV mini cho đồ án. Mỗi xe: ESP32 + L298N + 2 động cơ encoder + YDLIDAR X2 +
BNO055, micro-ROS qua Wi-Fi (UDP 8888) tới laptop **WSL2 Ubuntu 24.04 + ROS 2 Jazzy**.
Laptop: odometry, AMCL trên bản đồ có sẵn, Nav2 (Regulated Pure Pursuit), logic sa bàn.
Repo `~/robot_ws_3`, GitHub `NguyenHuynh-2004/Robot_kho` (đẩy bằng git Windows, chỉ khi được bảo).

**Bài toán:** sa bàn **5×5, ô 60 cm** (hàng 1–5 dưới→trên, cột 1–5 trái→phải). Nhập hàng, cột,
hướng (Đ/T/B/N); xe chỉ đi trên cạnh, xoay tại chỗ, né nút/cạnh bị chặn. **Đạt = tại đích
vị trí mục tiêu ≤ 3 cm, góc chấp nhận < 0,9° từ 2026-10-15.** Hai xe do `traffic_manager.py` điều phối (giữ chỗ 2 nút, ưu tiên xe gần đích,
gỡ kẹt).

## Chạy
| Kịch bản | Lệnh chính |
|---|---|
| 1 Vẽ bản đồ (tay cầm) | `bringup_real.launch.py xe:=N`, SLAM, joy bridge (xem docs) |
| 2 Một xe thật | `ros2 launch my_bot_bringup bringup_real.launch.py xe:=N` → `navigation.launch.py use_sim_time:=false` → `python3 tools/auto_initial_pose.py` → `python3 grid_navigator.py --real` |
| 3 Hai xe mô phỏng | `tools/fleet_sim.sh up` (domain 87) → `ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2`; RViz `tools/fleet.rviz`; Gazebo xem/đặt vật cản `tools/fleet_gazebo.py` |
| 4 Hai xe thật | xe 1 ở (3,1)→Đông, xe 2 ở (3,5)→Tây; `tools/fleet_real.sh up` → `python3 traffic_manager.py agv1 agv2`; `tools/fleet_real.sh down` |
| Đo hàng (camera) | lấy hàng Đông ô (2..5,5) → trả hàng 1; `fleet_real.sh up` (bật `esp32_cam_node`) → `python3 cargo_task.py agv1 agv2` ("1 3"); mô phỏng: `tools/sim_cargo.py` + `cargo_task.py --sim`; hiệu chuẩn `tools/cam_calib.py agvN` |
| **Tự động (trợ lý chạy)** | `tools/agvctl up` (chờ xe bật nguồn → tự bật hết) → `agvctl status/errors/events` → `agvctl goal "..." --go` → `agvctl down`. Xem `docs/AGVCTL.md` |

Tài liệu người dùng: `docs/HUONG_DAN_CHAY.md`, `docs/SA_BAN_VA_TAI_LIEU.md` (tra sự cố,
nối dây, thuật toán, quy tắc 2 xe, nạp firmware H3).

## Kiến trúc then chốt
- Firmware `firmware/esp32_agv_base/esp32_agv_base.ino`, chọn xe bằng **`#define XE 1|2`**
  (`agvctl fw build N` biên dịch từ WSL bằng arduino-cli của IDE; `/agvN/diag` JSON 1 Hz,
  `/agvN/estop_reset`).
  Node micro-ROS `/agvN/esp32_agv_base_N`; topic **`/agvN/scan`, `/agvN/imu/data`,
  `/agvN/joint_states`** (góc bánh tích luỹ), nhận **`/agvN/cmd_vel`**. ESTOP chốt tới reset
  (kẹt bánh / quay ngược chiều PWM); mất cmd 0.5 s thì dừng.
- **Xe 1 vs xe 2:** cùng mạch in mới (encoder 23/18/19/17 kéo lên, PWM LiDAR 4); động cơ xe 1
  **nối thẳng**, xe 2 **nối chéo** (L = 33/32/14, R = 27/26/25); LiDAR xe 2 nhân khoảng cách **1.062**.
- 1 xe: `ns_relay.py` (trong bringup_real) chuyển `/agvN/*` ↔ gốc, phần còn lại như cũ.
- 2 xe: mỗi xe cây TF riêng `/agvN/tf`; `fleet_real_robot.launch.py` (rsp, `wheel_odometry.py`,
  AMCL `map_topic:/map`), Nav2 **gộp một container/xe** (`tools/fleet_nav.launch.py`, nạp file
  tham số cho container), `fleet_agent.py` (`SingleThreadedExecutor`, lọc vật cản 0.25 m quanh
  xe kia, `/fleet/status` 5 Hz, `/fleet/agvN/cmd`). `/map` và `/fleet/*` dùng chung.
- `grid_navigator.py`: Dijkstra (ít cạnh → ít rẽ) → đoạn thẳng → action `follow_path`; xoay tại
  chỗ theo **gyro**, `coast 0.36` (đo xe 1), bù ≤ 5 lần. Chặn nút/cạnh tính lại mỗi local costmap
  (**`always_send_full_costmap: True`, 5 Hz**). Local costmap: ObstacleLayer,
  `observation_persistence 1.0`, `fleet_agent` xoá lớp vật cản mỗi 1 s (ô kẹt mất sau 1–2 s;
  1 xe: `grid_navigator.py --decay 1`).
- Cấu hình: `src/my_bot_bringup/config/{navigation.yaml, grid_real.yaml, real_room_map.yaml}`;
  `wheel_radius 0.03281`, cách bánh 0.200; `xy_goal_tolerance 0.02`.

## Môi trường & bẫy
- bashrc: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (bắt buộc), `GALLIUM_DRIVER=d3d12`,
  source `~/microros_ws/install` (agent tự build). WSL mirrored networking; laptop 192.168.1.21.
- `colcon build --symlink-install`; file mới trong `scripts/` cần build lại.
- **Sửa file qua `\\wsl.localhost` mất bit chạy → `chmod +x`.** Gọi WSL từ Git Bash:
  `MSYS_NO_PATHCONV=1 wsl.exe -e bash script.sh`; `/tmp` hai bên khác nhau.
- **WSL mirrored rơi gói UDP > 1472 B trên 127.0.0.1** (2026-10-14: node không thấy nhau, kể cả
  talker/listener) → `FASTRTPS_DEFAULT_PROFILES_FILE=~/robot_ws_3/tools/fastdds_localhost.xml`
  (gói ≤ 1400, unicast lo; đã thêm vào bashrc, fleet_real/fleet_sim). Nav2 lên chậm ~40 s/xe.
  Thử phát hiện node: chạy thẳng python (không `timeout ros2 run`), chờ ≥ 5 s.
- Quá ~30–40 tiến trình ROS/domain thì DDS LOCALHOST không thấy tiến trình bật sau.
- `MultiThreadedExecutor` + topic 50 Hz → CPU 80–90%.
- `ros2` CLI dò tìm thiếu khi nhiều tiến trình → kiểm tra bằng script rclpy.
- `sed` với `C:\Users` (`\U`,`\L`) và vá bằng Python không-raw (`\n`,`\1`) làm hỏng file.
- RViz lỗi đỏ `GLSL ... indexed_8bit_image` → `sudo tools/fix_rviz_shader.sh`.
- Thử 2 xe không cần phần cứng: `fleet_fake_robot.py ... --esp32` + `AGENT=0 EMULATE=1 tools/fleet_real.sh up` (domain riêng, vd 89).

## Quy tắc làm việc với người dùng
Tiếng Việt. **Người dùng tự nạp firmware** (giữ BOOT); trợ lý biên dịch kiểm tra, chép sang
`D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino`, đọc log. Làm từng giai đoạn, chờ "xong".
**Không gửi `/cmd_vel` tới xe thật khi chưa được phép.** Không commit `wifi_config.h`.

## Trạng thái (2026-10-10 tối)
**Hai xe thật chạy được** (Kịch bản 4, 6 bước đạt, không va). Firmware tách `Xe_1\Xe_1.ino`,
`Xe_2\Xe_2.ino` (`agvctl fw split`); xe 1 động cơ nối thẳng, xe 2 nối chéo; ESTOP báo ngay
qua `/agvN/diag`. URDF base_footprint ở tâm trục bánh. Định vị 2 xe: `tools/fleet_localize.py`.
Còn mở: phía tây sa bàn khác bản đồ (cột 1 lệch 4–6 cm). Chi tiết: `activeContext.md`.
