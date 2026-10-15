# System Patterns

## Phân vai
```
ESP32 (mỗi xe, namespace /agvN)            Laptop (WSL2, ROS 2 Jazzy)
  LiDAR  -> /agvN/scan          ─┐
  BNO055 -> /agvN/imu/data       ├─ micro-ROS UDP 8888 ─> micro_ros_agent (MỘT agent cho mọi xe)
  encoder-> /agvN/joint_states  ─┘
  /agvN/cmd_vel -> PID 2 bánh  <─────────────────────────  Nav2 (collision_monitor)
```
Laptop: `wheel_odometry.py` (joint_states + imu → `odom` + TF odom→base_footprint),
`robot_state_publisher` (URDF), AMCL (map→odom), Nav2 (RPP bám đoạn thẳng), logic sa bàn.

## Ba chế độ chạy
| Chế độ | Lệnh | Topic / TF |
|---|---|---|
| **1 xe thật** (Kịch bản 2) | `bringup_real.launch.py xe:=N` → `navigation.launch.py use_sim_time:=false` → `tools/auto_initial_pose.py` → `grid_navigator.py --real` | `ns_relay.py` chuyển `/agvN/{scan,imu/data,joint_states}` → gốc, `/cmd_vel` → `/agvN/cmd_vel`; mọi thứ còn lại ở gốc, `/tf` gốc |
| **2 xe mô phỏng** (Kịch bản 3) | `tools/fleet_sim.sh up` (domain 87) → `traffic_manager.py agv1 agv2` | `tools/fleet_fake_robot.py` (xe động học, TF map→odom hoàn hảo, LiDAR giả không tường, `inf_is_valid`, `coast 0`) |
| **2 xe thật** (Kịch bản 4) | `tools/fleet_real.sh up` → `traffic_manager.py agv1 agv2` | mỗi xe: `tools/fleet_real_robot.launch.py` (rsp, wheel_odometry, AMCL + lifecycle) trong `/agvN`; Nav2 `tools/fleet_nav.launch.py`; `fleet_agent.py` |

**Mỗi xe một cây TF riêng** `/agvN/tf`, `/agvN/tf_static` (hai xe đều có frame `odom`,
`base_footprint`). Chỉ `/map` (map_server) và `/fleet/*` là dùng chung. RViz một cửa sổ chỉ đọc
một cây TF → bản 2 xe dùng `tools/fleet.rviz` (bản đồ + MarkerArray `/fleet/markers` vẽ trên
khung `map`).

## Thành phần sa bàn
- `grid_board.py` — `Board`: toạ độ nút `xy(r,c)` (chỉ số từ 0), `closest`, `nodes`, `edges`,
  `bounds`, đối chiếu bản đồ (nút cách vật cản < `node_clearance` 0.25 m bị bỏ; cạnh <
  `edge_clearance` 0.20 m). `config_path(real)`.
- `grid_navigator.py` — `GridNavigator` (1 xe): Dijkstra (ít cạnh, rồi ít chỗ rẽ) → các
  **đoạn thẳng** gửi action `follow_path`; xoay tại chỗ riêng bằng gyro: dừng lệnh khi
  `err − coast·ω < 0`, giữ lệnh ≥ 0.15 s, settle, bù tối đa 12 lần trên xe thật;
  nếu còn sai số góc ≥ 0,9° thì báo lỗi thay vì báo đã tới.
  Nút/cạnh bị chặn tính lại ở **mỗi** `local_costmap/costmap` (tên tương đối). Marker
  `/grid_visualization` 5 Hz.
- `fleet_agent.py` — `FleetAgent(GridNavigator)` cho mỗi xe trong `/agvN`: nhận lệnh JSON
  `/fleet/agvN/cmd` (route_id, route, upto, heading), báo `/fleet/status` 5 Hz (x, y, yaw,
  state, blocked_nodes/edges). Lọc vật cản trong **0.25 m quanh xe kia** (vị trí từ
  `/fleet/status`). Mất lệnh 1 s → dừng. Chạy `SingleThreadedExecutor`.
- `traffic_manager.py` — điều phối: **giữ chỗ** LOOKAHEAD 2 nút; ưu tiên xe **còn ít nút hơn**
  (gần đích); xe đỗ ở đích của xe kia tự né; gỡ kẹt khi xe chặn đứng yên > 3 s (đi vòng, né
  sang ô, đổi vai, dừng); xe mất liên lạc (`STALE_S` 1 s) giữ nguyên ô. Lập đường thất bại →
  thử lại 2 s × 5 lần. Chế độ hỏi-đáp khi stdin là tty (nhập đủ cả 2 xe mới chạy), nhận thêm
  `/fleet/goal` (`"1 3 5 d"`, `"pair 1 3 5 d ; 2 3 1 t"`). Chi tiết quy tắc:
  `docs/SA_BAN_VA_TAI_LIEU.md` phụ lục F.

## Quyết định thiết kế (vì sao)
- **Odometry ở laptop** (ESP32 gửi góc bánh tích luỹ): mất gói Wi-Fi không mất quãng đường;
  đổi bán kính bánh không cần nạp chip.
- **Gyro cho xoay tại chỗ**: `/odom` angular velocity có gai (0 / 128°/s); coast 0.36 đo trên xe.
- **Namespace theo ID xe trong firmware**: hai ESP32 dùng chung một agent mà không đè topic;
  chế độ 1 xe giữ nguyên Nav2/RViz/grid_navigator nhờ `ns_relay.py` (relay raw, ~0.1 ms).
- **Nav2 gộp container / xe**: giảm ~12 tiến trình xuống 1 → không vượt giới hạn dò tìm DDS.
- **Nav2 bật tuần tự từng xe + thử lại 3 lần**: bật song song hay mất trả lời lifecycle.
- **Tham số Nav2 riêng từng xe** sinh bằng `sed` từ `navigation.yaml`: topic tuyệt đối
  `/agvN/scan`, `/agvN/odom`, footprint; costmap tĩnh `map_topic: /map`.
- **Gazebo trong mô phỏng 2 xe chỉ để xem** (`tools/fleet_gazebo.py`): vị trí xe đẩy vào bằng
  `set_pose`; vật cản thì ngược lại — kéo trụ đỏ trong Gazebo → `/sim/obstacles`.

## Ngưỡng / hằng số hay chỉnh
| Ở đâu | Giá trị |
|---|---|
| `navigation.yaml` goal checker | `xy_goal_tolerance 0.02` |
| `navigation.yaml` local costmap | 6×6 m, 0.05 m, update 5 Hz, **publish 5 Hz, always_send_full_costmap** |
| `grid_real.yaml rotate` | `done_deg 0.9`, `coast 0.36`, `pulse_max_deg 5`, `pulse_dead 0.35`, `tries 12` |
| `traffic_manager.py` | `WAIT_S 3`, `STALE_S 1`, `REACH_R 0.15`, `RETRY_S 2`, `MAX_FAIL 5` |
| `fleet_agent.py` | `STATUS_PERIOD 0.2`, `CMD_TIMEOUT 1.0`, `OTHER_ROBOT_R 0.25`, `NAV2_WAIT 5` |
