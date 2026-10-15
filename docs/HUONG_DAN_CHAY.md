# Hướng Dẫn Khởi Chạy Dự Án AGV (robot_ws_3)

Xe tự hành trên **sa bàn 5×5** (ô 60 cm). Bạn nhập hàng, cột và hướng đích; xe tự
tìm đường **chỉ đi trên các cạnh**, xoay tại chỗ ở chỗ rẽ và tự né vật cản.

Mở các terminal **lần lượt** — terminal trước khởi động xong mới mở terminal sau.

| Kịch bản | Dùng khi | Số terminal |
|---|---|---|
| 1️⃣ Vẽ và lưu bản đồ bằng tay cầm | phòng mới / đồ đạc thay đổi | 5 + 1 PowerShell |
| 2️⃣ Chạy một xe thật trên sa bàn | **dùng hằng ngày** | 4 |
| 3️⃣ Chạy hai xe (mô phỏng) | thử điều phối hai xe | 4 |
| 4️⃣ Chạy hai xe thật | **hai xe cùng chạy** trên sa bàn thật | 3 |
| 4️⃣ + đo hàng | xe lấy hàng ở cột 5, **camera đo rộng × cao**, trả về hàng 1 | 4 (+1 xem camera) |

> [!IMPORTANT]
> **Mở terminal MỚI** sau ngày 2026-10-14: `~/.bashrc` đã thêm cấu hình DDS
> (`FASTRTPS_DEFAULT_PROFILES_FILE=.../tools/fastdds_localhost.xml`). Thiếu dòng này thì các
> chương trình ROS **không thấy nhau** (WSL làm rơi gói dữ liệu lớn). Kiểm tra:
> `echo $FASTRTPS_DEFAULT_PROFILES_FILE` phải in đường dẫn tới file đó.

> [!IMPORTANT]
> **Mỗi xe một file firmware riêng**: xe 1 nạp `Xe_1\Xe_1.ino`, xe 2 nạp `Xe_2\Xe_2.ino`
> ([Nạp firmware](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật)). Mọi topic của xe
> nằm trong `/agv1/...` hoặc `/agv2/...`, nên chạy một xe phải ghi rõ **`xe:=1`** hoặc
> **`xe:=2`** ở lệnh cầu nối ESP32.

> [!IMPORTANT]
> **Mọi terminal** đều bắt đầu bằng hai dòng này (đã ghi sẵn trong từng khung lệnh
> để bạn chép cho tiện):
> ```bash
> cd ~/robot_ws_3
> source install/setup.bash
> ```

> [!NOTE]
> **Máy mới / lần đầu:** làm [Chuẩn bị máy mới](SA_BAN_VA_TAI_LIEU.md#h1-chuẩn-bị-máy-mới-chỉ-làm-một-lần-cho-mỗi-máy)
> và [Nạp firmware](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật) trước. Sa bàn,
> bảng tra sự cố, nối dây, thuật toán: xem [SA_BAN_VA_TAI_LIEU.md](SA_BAN_VA_TAI_LIEU.md).

> [!IMPORTANT]
> **Khởi động lại xe = tắt hẳn nguồn vài giây rồi bật**, để xe đứng yên 2 giây đầu
> (BNO055 đo độ lệch gyro). Chỉ bấm nút reset ESP32 có lúc làm BNO055 trả toàn số 0.

---

## 1️⃣ Kịch Bản 1: Vẽ Và Lưu Bản Đồ Bằng Tay Cầm (Cần 5 Terminals + 1 PowerShell)

Làm trước lần chạy sa bàn đầu tiên, hoặc khi đồ đạc trong phòng thay đổi nhiều.

> [!IMPORTANT]
> **Gốc bản đồ = chỗ xe đứng lúc bật SLAM.** Đặt xe ở chỗ dễ nhớ và nhớ hướng đầu xe.

### Terminal 1: Cầu Nối ESP32
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup bringup_real.launch.py xe:=2     # số xe đang dùng: 1 hoặc 2
```
Bật nguồn xe. Log agent phải hiện `session established`.

### Terminal 2: SLAM
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup slam.launch.py use_sim_time:=false
```

### PowerShell Trên Windows: Đọc Tay Cầm
Tay cầm USB không hiện trong WSL, nên đọc bên Windows rồi đẩy qua UDP. Mở
**PowerShell** (không phải WSL) và giữ cửa sổ này mở:
```
python \\wsl.localhost\Ubuntu\home\huynh\robot_ws_3\tools\windows_gamepad_bridge.py
```
Nó in `re=... tien=... LB=... RB=...` mỗi 2 giây.

### Terminal 3: Nhận Tay Cầm → `/joy`
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/joy_udp_bridge.py
```

### Terminal 4: `/joy` → Lệnh Chạy Xe
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup joy_teleop.launch.py start_joy_node:=false
```

> [!TIP]
> **Điều khiển:** **giữ LB rồi đẩy cần trái**; thả LB là xe dừng. Giữ thêm **RB** để
> chạy nhanh gấp đôi. `start_joy_node:=false` là bắt buộc trên WSL.

### Terminal 5: RViz Xem Bản Đồ
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Lái chậm đi khắp phòng, quay tại chỗ vài vòng ở các góc, tới khi tường đen trong
RViz kín hết.

### Lưu Bản Đồ (SLAM ở Terminal 2 vẫn phải đang chạy)
Mở thêm một terminal:
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_bot_bringup/config/real_room_map
colcon build --symlink-install
```
Ra hai file `real_room_map.pgm` và `real_room_map.yaml`. `colcon build` là bắt buộc
để launch thấy bản đồ mới.

> [!WARNING]
> - Đừng lưu đè `my_room_map` — đó là bản đồ của mô phỏng Gazebo.
> - Vẽ xong bản đồ mới phải **căn lại sa bàn** cho khớp vạch gạch:
>   [Căn sa bàn](SA_BAN_VA_TAI_LIEU.md#căn-sa-bàn-cho-khớp-vạch-gạch-sau-khi-vẽ-bản-đồ-mới).
> - Tắt hết các terminal của kịch bản này (nhất là **SLAM**) trước khi sang Kịch bản 2.

---

## 2️⃣ Kịch Bản 2: Chạy Một Xe Thật Trên Sa Bàn (Cần 4 Terminals)

Kịch bản dùng hằng ngày.

### Terminal 1: Cầu Nối ESP32
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup bringup_real.launch.py xe:=2     # số xe đang dùng: 1 hoặc 2
```
Bật nguồn xe, chờ log agent có `session established` và dòng
`Chuyen tiep /agv2/{scan, imu/data, joint_states} -> goc` (topic của xe được chuyển
về `/scan`, `/cmd_vel`... như cũ, các terminal sau giữ nguyên).

### Terminal 2: Nav2 (gồm sẵn bản đồ + định vị AMCL)
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup navigation.launch.py use_sim_time:=false
```
Chờ log có `Managed nodes are active`, rồi **đặt vị trí ban đầu** (xe đứng yên ở đâu
cũng được):
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/auto_initial_pose.py
```

> [!WARNING]
> **Đừng quên `use_sim_time:=false`** — thiếu nó node chờ `/clock` mãi mà không báo
> lỗi gì. SLAM của Kịch bản 1 phải đã tắt.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Fixed Frame = `map`. Thêm **Add → By topic → `/grid_visualization` → MarkerArray**
để thấy sa bàn. **Điểm quét đỏ phải trùng tường đen** trước khi cho xe chạy — lệch thì
bấm **2D Pose Estimate**, bấm vào chỗ xe thật rồi kéo theo hướng đầu xe.

### Terminal 4: Nhập Đích
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 grid_navigator.py --real
```
Dòng đầu phải in `Che do: XE THAT` và `Đã đối chiếu sa bàn với bản đồ: 24/25 điểm`.

Nhập lần lượt:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Hàng | `1`–`5` (1 = dưới cùng) | `3` |
| Cột | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông (phải) · `T` Tây (trái) · `B` Bắc (lên) · `N` Nam (xuống) · Enter = giữ hướng | `d` |

Xe tới đích rồi xoay về hướng đã chọn; chỉ báo hoàn thành khi sai số góc đo được
**< 0,9°**. Sai số vị trí được ghi trong log `fleet_agent` để kiểm tra riêng.
Gõ `q` để thoát.

> [!TIP]
> **Dòng đầu Terminal 4** luôn hiện, kể cả lúc chờ nhập: góc IMU, vị trí, hướng xe
> (≈ Đông/Tây/Nam/Bắc) và điểm lưới gần nhất. Không muốn: thêm `--no-status`.
>
> Lần đầu hãy chọn điểm gần (vd hàng 3 cột 3 → hàng 3 cột 4, 60 cm).

> [!CAUTION]
> **Dừng khẩn cấp:** Ctrl-C Terminal 2 (Nav2 ngừng phát lệnh, ESP32 tự dừng động cơ
> sau 0.5 s) hoặc tắt công tắc nguồn. Đừng bấm LB tay cầm khi Nav2 đang lái — hai
> lệnh xen nhau làm xe giật.

---

## 3️⃣ Kịch Bản 3: Chạy Hai Xe — Mô Phỏng (Cần 4 Terminals)

Thử bộ điều phối hai xe: giữ chỗ, ưu tiên **xe gần đích hơn**, tự gỡ kẹt. Chạy trên
`ROS_DOMAIN_ID=87` nên **không đụng tới xe thật**. Hai xe ảo nhưng Nav2 là Nav2 thật.

### Terminal 1: Bật Mô Phỏng (bản đồ, 2 xe ảo, 2 Nav2)
```bash
cd ~/robot_ws_3
source install/setup.bash
tools/fleet_sim.sh up
```
Chờ khoảng 40 giây tới khi in `Nav2 agv1 ... san sang` và `Nav2 agv2 ... san sang`.
Vị trí đầu mặc định: xe 1 ở (hàng 3, cột 1), xe 2 ở (hàng 3, cột 5). Muốn khác:
```bash
AGV1="3 1 0" AGV2="3 5 180" tools/fleet_sim.sh up     # hàng cột hướng_độ
```

### Terminal 2: Bộ Điều Phối
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2
```

Terminal hỏi lần lượt **xe 1 rồi xe 2**, giống Kịch bản 2:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Xe 1 - Hàng đích | `1`–`5` (1 = dưới cùng) | `3` |
| Xe 1 - Cột đích | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông · `T` Tây · `B` Bắc · `N` Nam · Enter = giữ hướng | `d` |
| Xe 2 - Hàng / Cột / Hướng | như trên | `3`, `1`, `t` |

Phải **nhập đủ cả hai xe** thì hai xe mới **cùng xuất phát**. Nhập sai (ngoài sa bàn,
ô sát vật cản, trùng đích với xe kia) thì báo lỗi và hỏi lại xe đó. Khi cả hai xe tới
nơi, terminal in `Xe 1 ĐÃ TỚI ...`, `Xe 2 ĐÃ TỚI ...` rồi hỏi lượt tiếp. Gõ `q` để thoát,
Ctrl-C để dừng khẩn.

> [!NOTE]
> Đích của xe này được là ô xe kia **đang đỗ** (vd hai xe đổi chỗ): xe đang đỗ sẽ tự
> né sang ô bên cạnh.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 rviz2 -d tools/fleet.rviz
```
Thấy bản đồ, lưới sa bàn (điểm đỏ = bị chặn), **hai xe** (xe 1 xanh dương, xe 2 cam,
mũi tên đen = hướng đầu xe), **đường còn lại** và **ô đang giữ** của từng xe. Xe mất
liên lạc hiện mờ kèm chữ `MAT LIEN LAC`.

> [!NOTE]
> Không dùng file RViz của Kịch bản 2 ở đây: mỗi xe có cây TF riêng (`/agv1/tf`,
> `/agv2/tf`). `tools/fleet_sim.sh` tự chạy `tools/fleet_viz.py` gộp hai cây (frame
> `agv1/...`, `agv2/...`) nên `tools/fleet.rviz` thấy được ảnh quét, costmap, đường đi của
> từng xe. Xe ảo không có URDF nên mục "Mô hình xe" và "Hạt AMCL" để trống — bình thường.

### Terminal 4: Gazebo (xem 3D + đặt vật cản)
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 python3 tools/fleet_gazebo.py
```
Cửa sổ Gazebo hiện **tường phòng** (dựng từ bản đồ), **vạch sa bàn** (nút đỏ), **hai xe**
(xe 1 xanh dương, xe 2 cam, mũi trắng = đầu xe) và **6 trụ đỏ** (vật cản) xếp hàng chờ
dưới sa bàn. Xe trong Gazebo chạy theo hai xe ảo của Terminal 1. Ctrl-C để tắt.

| Muốn | Làm |
|---|---|
| Dời khung nhìn | giữ chuột trái, kéo |
| Xoay khung nhìn | giữ con lăn (hoặc Shift + chuột trái), kéo |
| Phóng to / thu nhỏ | lăn con lăn |

**Đặt vật cản bằng chuột:**

1. Bấm phím **`T`** (hoặc biểu tượng mũi tên chữ thập ở góc trên bên trái) → chế độ dời vật.
2. Bấm vào **một trụ đỏ** → hiện 3 mũi tên.
3. Kéo **mũi tên đỏ** (dọc trục x, sang phải/trái) và **mũi tên xanh lá** (trục y, lên/xuống)
   để đưa trụ vào sa bàn. **Đừng kéo mũi tên xanh dương** (lên cao).
4. Thả chuột. Terminal 4 in vị trí, ví dụ
   `Vat can tren sa ban (1): (-0.55, +0.34) gan nut hang 4 cot 3 (30 cm)`, và LiDAR hai xe
   ảo thấy trụ ngay. Xe đang chạy gặp trụ thì tự đổi đường như vật cản thật.
5. Bấm **`Esc`** để thôi chế độ dời vật. **Bỏ vật cản** = kéo trụ ra khỏi sa bàn (về hàng chờ).

> [!NOTE]
> - Gazebo không tính vật lý, không lái xe: **xe** vẫn do Terminal 1 điều khiển, Gazebo
>   chỉ vẽ lại. Còn **vật cản** thì lấy từ Gazebo: trụ nào nằm trong sa bàn (tính
>   rộng thêm 25 cm ra ngoài nút biên) là vật cản, trụ ở ngoài thì không.
> - Mở Gazebo sẽ **xóa vật cản cũ** đặt bằng lệnh trước đó. Lệnh `ros2 topic pub ...
>   /sim/obstacles` (ở dưới) vẫn dùng được: trụ đỏ tự nhảy tới chỗ đã gửi.
> - Tắt Terminal 4 thì mô phỏng vẫn chạy bình thường, vật cản đang có vẫn giữ.

### Tắt Mô Phỏng
```bash
cd ~/robot_ws_3
tools/fleet_sim.sh down          # tắt cả Gazebo của Terminal 4
```

> [!TIP]
> - **Thả vật cản ảo bằng lệnh** (không cần Gazebo) ở (x, y) mét, nhiều vật cách nhau `;`:
>   `ROS_DOMAIN_ID=87 ros2 topic pub --once /sim/obstacles std_msgs/msg/String "{data: 'x,y'}"`
>   (gửi `''` để bỏ). Ví dụ giữa nút hàng 3 cột 3 và hàng 4 cột 3: `'-0.545,0.335'`.

Quy tắc điều phối và kết quả kiểm thử:
[SA_BAN_VA_TAI_LIEU.md — Phụ lục F](SA_BAN_VA_TAI_LIEU.md#f-quy-tắc-điều-phối-hai-xe).

### Thêm: Đo Hàng Bằng Camera (mô phỏng, rộng × cao)

Xe tới ô **lấy hàng ở cột 5** (hàng 2–5, quay **Đông**) → **LiDAR** khớp mặt trước hàng
(khoảng cách Z, góc lệch) → **camera nhìn thẳng** chụp, tách hàng khỏi nền, tìm 4 mép →
chiếu từng mép lên mặt phẳng mặt hàng → **rộng × cao** → bảng phân loại → xe tự chạy về
ô **trả hàng ở hàng 1** (cột theo loại). Xe không chở hàng. Cấu hình (ô lấy/trả, camera,
khoảng đặt hàng, bảng loại): `src/my_bot_bringup/config/cargo.yaml`.

Bật Terminal 1 + 2 như trên (Terminal 1 thêm `FAKE_LIDAR_NOISE=0.01 FAKE_RAYS=320` cho
LiDAR ảo nhiễu giống X2), rồi (mỗi terminal `cd ~/robot_ws_3` + `source install/setup.bash`):

```bash
# Terminal 5: hàng hoá + camera ảo — "hàng:rộng:cao:sâu[:xoay_độ[:lệch_ngang]]" (m),
# hàng đặt phía Đông ô (hàng, 5)
ROS_DOMAIN_ID=87 python3 tools/sim_cargo.py agv1 agv2 --goods "3:0.15:0.20:0.12, 4:0.10:0.12:0.10:5, 2:0.17:0.13:0.12:-6, 5:0.25:0.22:0.18"
# Terminal 6: nhiệm vụ đo hàng — "1 3" = xe 1 đo hàng ở (3,5); "2 4" = xe 2 ở (4,5)
ROS_DOMAIN_ID=87 python3 cargo_task.py agv1 agv2 --sim
```
Đổi hàng lúc đang chạy:
`ROS_DOMAIN_ID=87 ros2 topic pub --once /sim/cargo_set std_msgs/msg/String "{data: '3:0.2:0.18:0.15'}"`.
Xem ảnh camera ảo: `ROS_DOMAIN_ID=87 ros2 run rqt_image_view rqt_image_view /agv1/camera/image_raw`.
RViz (Terminal 3) thấy hộp hàng màu nâu bên phải cột 5 và chữ kết quả trên ô hàng 1.
Kết quả + ảnh có vẽ khung: `~/agv_logs/cargo/` (`results.csv`).

| Đo trong mô phỏng | Kết quả |
|---|---|
| 2026-10-12, lấy ở hàng 5: 9 lần, hàng 9–30 cm, xoay tới 21°, xe lệch 3 cm / 3° | sai **≤ 3,5 mm** |
| 2026-10-13, lấy ở cột 5: 4 hàng (10×12, 15×20, 17×13 xoay −6°, 25×22) | sai **≤ 1 mm**, về đúng (1,1)–(1,4) |
| Hai xe cùng loại hàng → cùng ô (1,1) | xe giao trước lùi sang hàng 2 nhường ô |
| Hàng rộng 40 cm (quá khung hình) / ô không có hàng | báo **không đo được**, xe không giao |

> [!WARNING]
> Giới hạn (camera ảo cao 11 cm, hàng cách tâm ô 85 cm): rộng ≤ ~80 cm, cao ≤ ~40 cm. Hàng
> thấp hơn camera thì thấy cả mặt trên → mép trên lấy theo vạch sáng mặt trên/mặt trước. Kích thước
> sát ngưỡng bảng loại (vd rộng 12,0 cm với ngưỡng 12 cm) có thể rơi sang loại bên cạnh.
> `--sim` = dùng thông số camera ảo; xe thật bỏ `--sim` (dùng `camera_agvN.yaml`).

---

## 4️⃣ Kịch Bản 4: Chạy Hai Xe Thật (Cần 3 Terminals)

Hai xe thật cùng chạy trên sa bàn theo đúng luật của mô phỏng: giữ chỗ trước, ưu tiên
**xe gần đích hơn**, xe đang đỗ ở đích của xe kia tự né, tự gỡ kẹt. Mỗi xe có định vị AMCL
và Nav2 riêng, dùng chung bản đồ và một micro-ROS agent.

> [!IMPORTANT]
> **Chuẩn bị:**
> - **Firmware:** xe 1 nạp `Xe_1\Xe_1.ino`, xe 2 nạp `Xe_2\Xe_2.ino` (đã nạp ngày
>   2026-10-10; chỉ nạp lại khi firmware thay đổi —
>   [cách nạp](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật)).
> - **Pin đầy**, công tắc nguồn động cơ bật. ESP32-CAM (nếu có) dùng **nguồn riêng** — chung
>   nguồn với ESP32 xe thì xe **tự khởi động lại liên tục** (BROWNOUT, xem
>   [Kiểm tra nguồn](#kiểm-tra-nguồn-sụt-áp)).
> - **Tắt hết** các terminal của Kịch bản 1 và 2 (cầu nối ESP32, Nav2, `grid_navigator`):
>   chạy song song thì hai bản đồ, hai agent đè nhau.
> - **Đặt xe:** xe 1 ở **(hàng 3, cột 1) mũi hướng Đông**, xe 2 ở **(hàng 3, cột 5) mũi
>   hướng Tây**. Tâm xe trên nút, mũi xe dọc vạch. Đặt chỗ khác thì khai báo `AGV1`/`AGV2`
>   (dưới) — đặt sai mà không khai báo thì xe sẽ chạy sai đường.

### Terminal 1: Bật Hệ Hai Xe
Bật nguồn **cả hai xe**, để yên 2 giây, rồi:
```bash
cd ~/robot_ws_3
source install/setup.bash
tools/fleet_real.sh up
```
Chờ tới khi in đủ (**khoảng 2 phút** — mỗi Nav2 mất 40–60 s; không lên sau 120 s thì
script tự bật lại):
```
Cho xe agv1 (/agv1/scan) co
Cho xe agv2 (/agv2/scan) co
Nav2 agv1 (lan 1)...... san sang
Nav2 agv2 (lan 1)...... san sang
Camera: log /tmp/fleet_real/camera.log
Da bat. ...
```
Báo `KHONG THAY` → xe đó chưa bật nguồn hoặc nạp nhầm file. Script cũng bật luôn
`tools/esp32_cam_node.py` (ảnh 2 camera); không có camera thì thêm `CAM=0` trước lệnh.

**Định vị chính xác** (cùng terminal, sau khi `Da bat`):
```bash
python3 tools/fleet_localize.py
```
Mỗi xe phải in `da gui cho AMCL`, độ dời thường vài cm. Xe không đứng ở chỗ đã khai báo
thì công cụ **tự dò cả sa bàn** (25 nút × 4 hướng, vài giây) và in
`xe dang o (hàng,cột) huong ...` — **kiểm tra đúng chỗ bạn đặt xe**. Báo `KHONG GUI` → xe
không đứng trên nút nào, hoặc quanh xe có nhiều đồ đạc không có trên bản đồ (xem cảnh báo
bản đồ bên dưới).

Đặt xe ở chỗ khác thì khai báo khi bật (hàng cột hướng_độ: 0 Đông, 90 Bắc, 180 Tây, -90 Nam):
```bash
AGV1="3 2 0" AGV2="2 2 0" tools/fleet_real.sh up      # vd hai xe ở (3,2) và (2,2), cùng quay Đông
```

### Kiểm Tra Nguồn (sụt áp)
Mỗi xe gửi `/agvN/diag` mỗi giây. Xem (terminal mới):
```bash
ros2 topic echo --qos-reliability best_effort /agv1/diag --field data
```
`"reset":"POWERON"` và `up_s` **tăng đều** là tốt. `"reset":"BROWNOUT"` hoặc `up_s` cứ về
số nhỏ = **xe tự khởi động lại vì sụt áp** (mất định vị, Nav2 chạy sai). Đồng hồ vạn năng
vẫn thấy 5 V vì sụt rất ngắn (lúc Wi-Fi phát). Sửa: camera nguồn riêng, tụ 1000 µF ở 5 V
gần ESP32, mạch hạ áp ≥ 2 A thay ổn áp trên L298N, pin đầy. (2026-10-14: hai xe sụt áp
mỗi 15–50 s khi ESP32-CAM chung nguồn; sửa nguồn xong chạy ổn định.)

### Terminal 2: Bộ Điều Phối
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 traffic_manager.py agv1 agv2
```
Hỏi lần lượt **xe 1 rồi xe 2**:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Xe 1 - Hàng đích | `1`–`5` (1 = dưới cùng) | `3` |
| Xe 1 - Cột đích | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông · `T` Tây · `B` Bắc · `N` Nam · Enter = giữ hướng | `t` |
| Xe 2 - Hàng / Cột / Hướng | như trên | `3`, `1`, `d` |

Phải **nhập đủ cả hai xe** thì hai xe mới **cùng xuất phát**. Muốn một xe **đứng yên**:
nhập đúng ô nó đang đứng và Enter ở hướng. Khi xong, terminal in `Xe 1 ĐÃ TỚI ...`,
`Xe 2 ĐÃ TỚI ...` rồi hỏi lượt tiếp. Gõ `q` để thoát.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d tools/fleet.rviz
```
Hiện đầy đủ như Kịch bản 2, cho **từng xe** (nhóm `Xe 1 (agv1)`, `Xe 2 (agv2)` ở cột trái —
bỏ dấu tick để ẩn cả nhóm):

| Mục | Ý nghĩa |
|---|---|
| Mô hình xe | thân, bánh, LiDAR theo URDF |
| LaserScan | điểm quét **đỏ, cỡ 5** — phải trùng tường đen |
| Hạt AMCL | đám mũi tên quanh xe (xe 1 xanh, xe 2 cam); co gọn = định vị chắc |
| Local Costmap | vùng vật cản quanh xe |
| Global Costmap | tắt sẵn (hai xe chồng lên nhau rối) — tick để xem |
| Footprint, Đoạn đang chạy, Local Plan | khung xe, đoạn thẳng đang giao cho Nav2 |

Cộng thêm bản đồ, lưới sa bàn (điểm đỏ = bị chặn), đường còn lại và ô đang giữ của từng xe.
`tools/fleet_real.sh` tự chạy `tools/fleet_viz.py` để gộp cây TF hai xe (frame `agv1/...`,
`agv2/...`) cho một cửa sổ RViz.

### Chạy Thử Từng Bước (đã chạy thật 2026-10-10)

Lần đầu trong ngày nên chạy lại vài bước này. Mỗi bước đạt mới sang bước sau.

| Bước | Xe 1 | Xe 2 | Phải thấy | Kết quả thật |
|---|---|---|---|---|
| 1. Chỉ xe 1 | `3 2 d` | `3 5` Enter | xe 1 tiến 60 cm; xe 2 đứng yên | lệch 0.5 cm, 1.7° |
| 2. Chỉ xe 2 | `3 2` Enter | `3 4 t` | xe 2 tiến 60 cm; xe 1 đứng yên | lệch 0.4 cm, 0.9° |
| 3. Song song | `5 2 b` | `5 4 b` | cùng chạy, không ai chờ ai | lệch 1.2 / 0.4 cm |
| 4. Đổi chỗ hàng 5 | `5 4 d` | `5 2 t` | một xe né sang ô bên cạnh, xe kia qua | không va, gần nhất 60 cm |
| 5. Đường cắt nhau | `3 1 d` | `3 5 t` | một xe chờ trước chỗ giao | không va |
| 6. Đổi chỗ (như mô phỏng) | `3 5 t` | `3 1 d` | một xe né lên hàng 4 rồi vòng về | không va, gần nhất 59 cm |

> [!WARNING]
> **Độ chính xác phụ thuộc bản đồ.** Ngày 2026-10-10, phía **tây** sa bàn (quanh cột 1) có
> nhiều đồ đạc mới không có trên bản đồ (cách (3,1) ~48 cm, (4,1) ~26 cm, (5,1) ~45 cm):
> xe đứng ở cột 1 lệch 4–6 cm, phía đông thì < 1 cm. Ngày 2026-10-14 phòng có thêm **xe máy**
> phía Đông: LiDAR chỉ khớp bản đồ 40–56% (trước ~70%), AMCL trôi ~5 cm / 4–7° sau 1,8 m.
> **Vẽ lại bản đồ** (Kịch bản 1) đúng như phòng lúc chạy rồi căn lại sa bàn
> (`tools/set_grid_center.py`), hoặc dời đồ ra khỏi quanh sa bàn.

### Khi Xe Báo ESTOP
ESP32 tự dừng khẩn khi **bánh kẹt / mất encoder / hết pin** hoặc **bánh quay ngược chiều**,
và báo ngay về máy tính — Terminal 2 in đỏ:
```
!!! [agv1] ESTOP tren ESP32: Banh ket / mat encoder - xe dung han, bo lo trinh; ...
```
Xe đó bỏ lộ trình, xe kia tránh nó như xe đang đỗ. Sửa nguyên nhân rồi gỡ ESTOP (terminal mới):
```bash
ros2 topic pub --once /agv1/estop_reset std_msgs/msg/Empty     # hoặc tắt/bật nguồn xe
```
rồi nhập đích mới. Terminal 2 in `[agv1] da go ESTOP`.

### Kiểm Tra Động Cơ + Encoder (sau khi nạp firmware / đổi dây)
**Kê xe hở bánh**, tắt Terminal 2, rồi:
```bash
python3 tools/wheel_test.py agv1     # hoặc agv2
```
Bốn dòng tiến / lùi / quay trái / quay phải phải đều `DUNG` cho cả hai bánh. Tiến mà cả hai
bánh `NGUOC` → dây động cơ trái/phải nối chéo so với firmware (xe 1 nối thẳng, xe 2 nối chéo
— đổi trong file gốc firmware rồi `tools/agvctl fw split`).

### Hiệu Chỉnh Riêng Từng Xe
Mặc định cả hai xe dùng số đo của **xe 1**. Đo được số riêng thì khai báo khi bật:
```bash
WHEEL_R_agv2=0.0330 COAST_agv2=0.30 tools/fleet_real.sh up
```
- `WHEEL_R_agvN`: bán kính lăn bánh (m) — xe đi **thiếu** quãng đường thì tăng, **thừa** thì giảm.
- `COAST_agvN`: hệ số xoay tại chỗ (s) — xe xoay **vượt** góc thì tăng, **thiếu** thì giảm.

Terminal 1 in lại `Xe agv2: wheel_radius ... m, coast ...` để xác nhận.

### Đo Hàng Bằng Camera (xe thật, ESP32-CAM)

**Luồng có nút xác nhận nhận hàng và chia 4 ô theo chiều cao 24–50 cm:** xem
[Chạy hai xe đến hộp và xem số đo trực tiếp](DO_HOP_TRUC_TIEP.md). Phần
`cargo_task.py` bên dưới là luồng tự động cũ, phân loại theo rộng/cao và không
đợi xác nhận; chỉ dùng một trong hai luồng cho mỗi lượt chạy.

Xe tới ô **lấy hàng phía Đông cột 5** (ô (2..5, 5), quay Đông) → LiDAR đo khoảng cách + góc
mặt hàng, **camera** tìm 4 mép → **rộng × cao** → bảng loại trong `cargo.yaml` → xe tự chạy
về ô **trả hàng ở hàng 1** (cột theo loại). Xe không chở hàng. Ô (1,5) không dùng (sát vật
cản trên bản đồ).

| Xe | Camera (ESP32-CAM, firmware CameraWebServer có sẵn — **không nạp lại**) |
|---|---|
| 1 | `http://192.168.1.201` (ảnh `/capture`, video `:81/stream`) |
| 2 | `http://192.168.1.200` |

Laptop chụp `/capture` 3 ảnh/s mỗi xe (~30 KB/s, nhẹ Wi-Fi) → `/agvN/camera/image_raw`,
320×240. Đổi IP / cách lấy ảnh: `src/my_bot_bringup/config/cargo.yaml` → `cameras:`.

> [!TIP]
> **Chạy hằng ngày** (camera 2 xe đã hiệu chuẩn xong ngày 2026-10-15 — mục A chỉ làm lại khi
> camera bị xê dịch / đổi camera). Mỗi terminal mở **mới**, bắt đầu bằng
> `cd ~/robot_ws_3 && source install/setup.bash`.
>
> 1. **Đặt hàng** (mục B): hộp phía Đông ô (3,5), (4,5)... cách tâm ô **85 cm**, cao ≥ 24 cm.
> 2. **Đặt xe**, ghi lại vị trí theo **(hàng, cột)** — hàng đếm **từ dưới lên** (hàng 1 là hàng
>    trả hàng, sát người đứng), cột đếm **từ trái sang** (cột 5 là cột lấy hàng). Hướng: Đông
>    = mũi xe chỉ về phía cột 5. Bật nguồn **2 xe + 2 camera**, để yên 2 giây.
> 3. **Terminal 1** — bật hệ, khai báo đúng chỗ đặt xe (hàng cột hướng_độ: 0 Đông, 180 Tây):
>    ```bash
>    AGV1="3 1 0" AGV2="3 5 180" tools/fleet_real.sh up     # ~2 phút, chờ "Da bat"
>    python3 tools/fleet_localize.py                          # mỗi xe phải in "da gui cho AMCL"
>    ```
>    Đọc dòng `xe dang o (hàng,cột) huong ...` (nếu có) — **phải đúng chỗ xe đứng**. Sai /
>    `KHONG GUI` → đặt lại xe đúng nút rồi chạy lại `fleet_localize.py`.
> 4. **Kiểm tra nguồn** (terminal mới, Ctrl-C để thoát): `ros2 topic echo --qos-reliability
>    best_effort /agv1/diag --field data` → `"reset":"POWERON"`, `up_s` tăng đều. Làm cho `agv2`.
> 5. **Terminal 2:** `python3 traffic_manager.py agv1 agv2` — để yên, không cần nhập.
> 6. **Terminal 3 (tùy chọn):** `rviz2 -d tools/fleet.rviz`.
> 7. **Terminal 4:** `python3 cargo_task.py agv1 agv2`, đầu ra phải có
>    `[agv1] camera: camera_agv1.yaml` (không phải `CHUA HIEU CHUAN`). Nhập `1 3` (xe 1 đo hàng
>    ở (3,5)), `2 4` (xe 2 ở (4,5)). Chờ `XONG: ... da ve o (1,k)`.
> 8. **Tắt:** Ctrl-C Terminal 2, 3, 4 → Terminal 1: `tools/fleet_real.sh down`.
>
> **Đứng xa sa bàn khi xe chạy** — người đứng gần bị coi là vật cản, xe dừng chờ.

**A. Chuẩn bị (một lần):**
1. **Camera:** nhìn **thẳng ra trước, nằm ngang**, ở đầu xe; nguồn **riêng**. Thử: mở
   `http://192.168.1.201/capture` bằng trình duyệt. (Đang lắp: cao 23,5 cm.)
2. **Đo vị trí camera** (thước): tâm ống kính cách **tâm trục bánh** x cm ra trước, y cm sang
   trái, z cm cao từ sàn → `src/my_bot_bringup/config/camera_agvN.yaml` (đã ghi: x 0,0967,
   z 0,235 cho cả 2 xe).
3. **Hiệu chuẩn** (mỗi xe một lần) — theo
   [Nav2 Camera Calibration](https://docs.nav2.org/rolling/tutorials/general_tutorials/camera_calibration/camera_calibration/),
   dùng gói `camera_calibration` của ROS. Bàn cờ calib.io **8×10 ô, ô 20 mm** → `--size 7x9
   --square 0.02` (đếm **góc trong**). In 100 % (không "Fit to page"), đo lại ô bằng thước
   (khác 20 mm thì sửa `--square`), dán phẳng lên bìa cứng.

   Cài một lần: `sudo apt install ros-jazzy-camera-calibration`

   ```bash
   # Terminal A: ảnh 2 camera lên ROS, 10 ảnh/s cho nhanh (fleet_real.sh up đang chạy thì tắt camera của nó: CAM=0)
   python3 tools/esp32_cam_node.py --rate 10
   # Terminal B: cửa sổ hiệu chuẩn cho xe 1
   ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --no-service-check \
     --ros-args -r image:=/agv1/camera/image_raw -p camera:=/agv1/camera
   ```
   Trong cửa sổ: cầm bàn cờ trước camera xe 1 (cách 30–70 cm, thấy **trọn** bàn cờ — có
   vạch màu nối các góc là đã nhận), di chuyển chậm cho 4 thanh bên phải dài ra:

   | Thanh | Làm |
   |---|---|
   | **X** | đưa bàn cờ sang trái, sang phải khung hình |
   | **Y** | đưa lên trên, xuống dưới khung hình |
   | **Size** | đưa lại gần (gần kín khung) và ra xa |
   | **Skew** | nghiêng bàn cờ sang trái/phải/lên/xuống ~30° |

   Cả 4 thanh **xanh lá** → nút **CALIBRATE** sáng → bấm, chờ ~1 phút (cửa sổ đứng là bình
   thường) → bấm **SAVE** (ghi `/tmp/calibrationdata.tar.gz`). **Không** bấm COMMIT.
   Đóng cửa sổ, nhập kết quả vào dự án:
   ```bash
   python3 tools/cam_calib.py agv1 --ost /tmp/calibrationdata.tar.gz
   ```
   → ghi `src/my_bot_bringup/config/camera_agv1.yaml` (giữ nguyên `x, y, z` của bước 2), in
   `fx`, `fy`, góc ngang (~60–70° là hợp lý). Terminal B in thêm sai số chiếu **`epi`/RMS** —
   dưới ~0,5 px là tốt. Làm lại cho **xe 2**: đổi `agv1` → `agv2` ở cả hai lệnh (lệnh SAVE
   ghi đè file `/tmp/calibrationdata.tar.gz`, nên nhập xe 1 xong rồi mới hiệu chuẩn xe 2).

   Cách khác (không cần cài gói, tự chụp 15 ảnh):
   `python3 tools/cam_calib.py agv1 --board 9x7 --square 0.020`.

4. **Đo góc xoay + góc ngóc camera** (mỗi xe một lần, sau bước 3; **bắt buộc** — camera
   lắp lệch vài độ làm mép LiDAR không khớp ảnh và chiều cao sai). Hệ đang chạy
   (`fleet_real.sh up`), xe **đứng yên** trước một hộp **cao hơn tia LiDAR** (≥ 24 cm), biết
   chiều cao thật (đo thước), đặt **thẳng, chính giữa**, cách 70–90 cm — dễ nhất: cho xe tới ô
   lấy hàng có hộp đó (`traffic_manager`: `1 3 5 d`):
   ```bash
   python3 tools/cam_extrinsic.py agv1 --height 0.320    # hộp cao 32,0 cm -> ghi yaw + pitch
   ```
   Các lần đo lệch nhau < 0,5° là tốt. Làm lại cho `agv2` (hộp của nó, `--height` theo hộp đó).
   Đã đo 2026-10-15: xe 1 yaw −2,49°, pitch −2,29°; xe 2 yaw +2,92°, pitch −1,09°.
   Có `pitch` thì chiều cao tính **từ mép trên tới sàn** (không cần mép dưới — sàn bóng
   phản chiếu hộp làm mép dưới sai).

**B. Đặt hàng:** hộp phía Đông ô (hàng 2–5, cột 5), **mặt trước cách tâm ô 85 cm** (camera
cao 23,5 cm, nhìn xuống được 19,7°: gần hơn thì không thấy chân hộp), vuông
với hướng Đông. **Cao hơn tia LiDAR** (LiDAR cao 19 cm nhưng tia chếch lên: ở 85 cm đã
khoảng 21–22 cm → hộp nên **≥ 24 cm**), tối đa 45 cm; rộng ≤ 33 cm (theo bảng loại).
**Tránh ngược sáng**: không để cửa kính / đèn sáng ngay sau hộp — kéo rèm hoặc dựng tấm bìa
trắng/xám phía sau. Hộp nên **khác màu / khác độ sáng rõ** với nền phía sau (vd carton nâu,
hộp xanh trước rèm trắng). Bộ đo tách hộp bằng **GrabCut**: lấy vị trí hộp từ LiDAR làm gợi ý,
tự học màu hộp và màu nền trên ảnh (chịu được sàn bóng phản chiếu, hộp ít màu).

**C. Chạy** — Terminal 1–3 như trên (`fleet_real.sh up` + `fleet_localize.py`, `traffic_manager.py`,
RViz), thêm:
```bash
# Terminal 4: nhiệm vụ đo hàng
cd ~/robot_ws_3
source install/setup.bash
python3 cargo_task.py agv1 agv2
```
Nhập `<xe> <hàng>`: `1 3` = xe 1 tới (3,5), đo, về hàng 1; `2 4` = xe 2 ở (4,5) (hai xe làm
song song được). Terminal 2 (traffic_manager) cứ để yên — không cần nhập gì.
Xem camera (tùy chọn, Terminal 5): `ros2 run rqt_image_view rqt_image_view /agv1/camera/image_raw`.

Terminal 4 in, ví dụ:
```
[agv1] -> (3,5) quay Đông
[agv1] sua AMCL theo ban do: lech 2.8 cm, -5.6 do (56% tia khop)
[agv1] Hang o (3,5): rong 15.1 cm, cao 20.1 cm (...) -> loai "cao" -> o (1,3)
[agv1] XONG: hang (3,5) -> da ve o (1,3)
```
- Dòng `sua AMCL theo ban do`: tới ô lấy hàng, xe đứng yên thì tự khớp LiDAR với bản đồ và
  sửa định vị (AMCL hay trôi vài cm khi phòng khác bản đồ) — chặng về hàng 1 đi đúng hơn.
- Đầu Terminal 4 in `CHUA HIEU CHUAN` cho xe chưa làm bước A3 — vẫn chạy được nhưng số đo
  sai vài mm tới vài cm.
- Hai xe cùng loại hàng → cùng ô: xe tới sau chờ, xe đã giao lùi sang hàng 2 nhường ô.
- Bị chặn tạm (người đứng gần sa bàn): bộ điều phối bỏ lộ trình sau ~10 s → Terminal 4 in
  `cho 5 s roi giao lai (lan 1/3)` và tự giao lại, tối đa 3 lần. **Đứng xa sa bàn khi xe chạy.**
- Không có hàng / hàng quá to / chạm mép ảnh → `THAT BAI`, xe đứng yên tại ô lấy hàng.
- `LiDAR chi thay N diem phia truoc - khong co hang?` mà camera vẫn thấy hộp → hộp **thấp hơn
  tia LiDAR** (tia ở ~21–22 cm tại 85 cm) → dùng hộp ≥ 24 cm.
- `bi chan sau 3 lan giao lai: ... failed 5 lan` và log `/tmp/fleet_real/agent_agvN.log` có
  `khong thay Nav2 (action follow_path)` → Nav2 của xe đó mất liên lạc: Terminal 1
  `tools/fleet_real.sh down` rồi `up` lại (xe đứng yên tại chỗ: khai báo `AGVN` theo chỗ đó)
  → `fleet_localize.py` → giao lại lệnh.
- Xe đứng im mà Terminal 4 không báo gì → xem `/agvN/diag` (mục **Kiểm Tra Nguồn**): ESTOP
  ("Banh ket / mat encoder" → kiểm tra động cơ, `tools/wheel_test.py agvN` khi kê hở bánh) hoặc
  `BROWNOUT` (sụt áp).
- Kết quả + ảnh có vẽ khung: `~/agv_logs/cargo/` (`results.csv`; ảnh gốc `_raw.png`; đo hỏng
  thì ảnh `_LOI.png`).
- Đo thử một lần không cho xe chạy (xe đã đứng ở ô lấy hàng, quay Đông):
  `python3 cargo_measure.py agv1 --save /tmp/do_thu.png`.

| Chạy thật 2026-10-14 | Kết quả |
|---|---|
| 2 camera vào ROS | 3 ảnh/s mỗi xe, 320×240 |
| `1 3`: (3,2) → (3,5) quay Đông; `1 4`: (3,5) → (4,5) | đi đúng đường; chưa đặt hàng → báo `khong co hang` đúng |
| Sửa định vị tại ô lấy hàng | AMCL lệch 2,8 cm / −5,6° → sau sửa trùng LiDAR 0,3 cm |
| 2026-10-15, đủ hiệu chuẩn (K, yaw, pitch) + tách hộp GrabCut | hộp 18,5×32,0 → **18,7×32,0**; hộp xanh 12,0×23,9 → **13,0×24,1**; cả 2 xe về đúng (1,4), (1,3) |

### Tắt
```bash
cd ~/robot_ws_3
tools/fleet_real.sh down      # gửi lệnh dừng cho cả hai xe rồi tắt hết (cả camera)
```
Ctrl-C trước ở Terminal 2 (traffic_manager), 4 (cargo_task), 5 (xem camera), RViz.

> [!CAUTION]
> **Dừng khẩn:** Ctrl-C Terminal 2 (mất lệnh điều phối quá 1 giây thì từng xe tự dừng),
> hoặc `tools/fleet_real.sh down`, hoặc tắt công tắc nguồn xe.

> [!NOTE]
> - Log từng phần ở `/tmp/fleet_real/` (`agent_agv1.log`, `nav_agv2.log`, `robot_agv1.log`...).
> - Mỗi lần `up`/`down`, script dọn bộ nhớ chung DDS bị bỏ lại (`fastdds shm clean`).
> - Thử không cần xe (ESP32 giả, LiDAR giả dò tường trên bản đồ): chạy hai
>   `tools/fleet_fake_robot.py ... --esp32` rồi `AGENT=0 EMULATE=1 tools/fleet_real.sh up`.
