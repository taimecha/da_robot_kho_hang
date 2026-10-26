# Lệnh thu số liệu NCKH cho đúng hai xe

Kết quả đã ghi ngày 26/10/2026: [xe thật](../data/real_trial_03/README.md),
[mô phỏng hai thùng theo số đo thật](../data/sim_trial_01_20261026/README.md).
Người sửa báo cáo xem [hướng dẫn số liệu mới](HUONG_DAN_CAP_NHAT_BAO_CAO_2_XE_2026-10-26.md).
Các lệnh dưới đây dùng **thư mục lượt mới**, tránh ghi đè dữ liệu đã bàn giao.

Các lệnh dưới đây chạy trong terminal Bash của laptop tại `/home/tai/Robot_kho_clone`. Mỗi nhóm "Terminal" là một cửa sổ riêng. Không chạy mô phỏng trên cùng ROS domain với xe thật. Một lượt được tính từ lúc bấm **BẮT ĐẦU** đến khi xe dừng ở ô giao. Nút **ĐÃ DỠ HÀNG (XÁC NHẬN)** chỉ ghi mốc dỡ kiện; nút **QUAY LẠI LẤY TIẾP** mới bắt đầu lượt kế. Chỉ đánh dấu giao vật lý thành công khi video cũng cho thấy kiện đã được dỡ.

Đổi `trial_01` thành `trial_02`, `trial_03`... cho mỗi lần thử; không ghi đè thư mục `rosbag` của lượt trước.
Nếu đổi vị trí đầu hoặc hộp, sửa cả dòng tương ứng trong `scenario.txt` và điền `measurements.csv` theo số đo thật trước khi chạy.

## A. Mô phỏng hai xe theo bố trí thử thật

Ví dụ này đặt xe 1 tại **(5,2)**, xe 2 tại **(2,3)**, đều quay Đông. Hai ô lấy vẫn là (3,5) và (4,5); hộp mô phỏng ở phía Đông hai ô lấy: hộp xe 1 rộng 18,5 cm, cao 32 cm; hộp xe 2 rộng 12,1 cm, cao 24 cm; chiều sâu tạm giả định 12 cm. Để so sánh với xe thật, thay kích thước đầu vào này bằng số đo thước của **chính các hộp sẽ dùng ngoài sa bàn** và dùng cùng ô xuất phát.

**Terminal 1 — xe ảo và Nav2:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
RUN="$PWD/data/sim_trial_02"
mkdir -p "$RUN"
LOGDIR="$RUN/fleet" AGV1='5 2 0' AGV2='2 3 0' FAKE_DELAY=0.05 \
  FAKE_DEAD_V=0.012 FAKE_DEAD_W=0.10 FAKE_LIDAR_NOISE=0.01 FAKE_RAYS=320 \
  FAKE_MAP="$PWD/src/my_bot_bringup/config/real_room_map.yaml" tools/fleet_sim.sh up
```

Chờ cả hai dòng `Nav2 agv1 ... san sang`, `Nav2 agv2 ... san sang`. Nếu `/fleet/status` không phát hoặc hai `fleet_agent.py` không còn chạy, xem `data/sim_trial_02/fleet/agent_agv1.log` và `data/sim_trial_02/fleet/agent_agv2.log`; không bắt đầu thử khi thiếu một xe.

**Terminal 2 — điều phối:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
python3 traffic_manager.py agv1 agv2 --no-stdin
```

**Terminal 3 — hộp và camera ảo:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
python3 tools/sim_cargo.py agv1 agv2 --goods '3:0.185:0.32:0.12,4:0.121:0.24:0.12'
```

**Terminal 4 — ghi ROS bag:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
RUN="$PWD/data/sim_trial_02"
mkdir -p "$RUN"
printf 'start_agv1=5,2,East\nstart_agv2=2,3,East\ngoods=3:0.185:0.32:0.12,4:0.121:0.24:0.12\n' > "$RUN/scenario.txt"
sha256sum tools/cargo_live_view.py tools/sim_cargo.py traffic_manager.py fleet_agent.py \
  src/my_bot_bringup/config/cargo.yaml src/my_bot_bringup/config/navigation.yaml > "$RUN/code_sha256.txt"
ros2 bag record --max-bag-size 47185920 -o "$RUN/rosbag" /fleet/status /fleet/goal /fleet/cargo_event \
  /fleet/cargo_result /agv1/scan /agv2/scan /agv1/odom /agv2/odom \
  /agv1/camera/image_raw /agv2/camera/image_raw /agv1/cmd_vel /agv2/cmd_vel
```

**Terminal 5 — giao diện lấy/giao và nhật ký thời gian:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
RUN="$PWD/data/sim_trial_02"
python3 tools/cargo_live_view.py --sim --agv1-pickup 3,5 --agv2-pickup 4,5 \
  --save-dir "$RUN/images" --event-log "$RUN/cargo_events.jsonl"
```

Khi giao diện hiện hai camera, bấm **BẮT ĐẦU** ở mỗi xe. Xe tới ô quan sát (3,4)/(4,4), chốt 5 ảnh rồi tới ô lấy (3,5)/(4,5). Bấm **ĐÃ NHẬN HÀNG** cho từng xe khi đã dừng ở ô lấy; đây là xác nhận kiện ảo đã chất. Khi xe tới ô giao, bấm **ĐÃ DỠ HÀNG (XÁC NHẬN)** để ghi mốc `t6` mà không cho xe chạy tiếp. Nếu muốn thử lượt mới, bấm **QUAY LẠI LẤY TIẾP**. Bấm `S` lưu ảnh, `Q` đóng.

**RViz tùy chọn, terminal khác:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=87
rviz2 -d tools/fleet.rviz
```

Mỗi giao điểm lưới trong RViz có nhãn `(1,1)` đến `(5,5)` theo thứ tự `(hàng,cột)`, đúng cách đánh số của lệnh điều phối. Nhãn do `traffic_manager.py` phát, nên nếu chương trình này đã chạy trước khi cập nhật mã, hãy khởi động lại nó sau khi kết thúc lượt hiện tại; không cần sửa cấu hình RViz.

Sau lượt thử: `Ctrl+C` ở terminal ghi bag và các terminal đang giữ tiến trình; tại Terminal 1 chạy `ROS_DOMAIN_ID=87 LOGDIR="$PWD/data/sim_trial_02/fleet" tools/fleet_sim.sh down`. Xuất bảng:

```bash
cd /home/tai/Robot_kho_clone
RUN="$PWD/data/sim_trial_02"
python3 tools/analyze_cargo_timing.py "$RUN/cargo_events.jsonl" \
  --environment sim --csv "$RUN/cargo_times.csv"
```

## B. Xe thật sau khi tự bật nguồn và đặt đúng vị trí

Đặt xe 1 tại **(5,2)**, xe 2 tại **(2,3)**, đều quay Đông. Quy trình đã kiểm chứng trong phiên này dùng **gợi ý ô/hướng** để khởi tạo rồi tinh chỉnh LiDAR quanh ô. Tìm toàn cục không có gợi ý từng bị từ chối vì đối xứng bản đồ. Hai ô lấy vẫn là (3,5) và (4,5). Đặt hộp phía Đông ô lấy tương ứng; đo rộng/cao và khoảng cách bằng thước, ghi vào phiếu cùng ID hộp. LiDAR mới của xe 1 cần đối chiếu góc và khoảng cách với một vật mốc đã đo khi xe đứng yên. Ghi video toàn cảnh thấy thao tác đặt kiện, xe chở kiện và dỡ kiện. Giữ đường chạy trống và có người trực tiếp trông xe.

**Terminal 1 — khởi động phần xe thật, sau khi cả hai đã bật nguồn:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
RUN="$PWD/data/real_trial_04"
mkdir -p "$RUN"
LOGDIR="$RUN/fleet" AUTO_LOCALIZE=0 AGV1='5 2 0' AGV2='2 3 0' tools/fleet_real.sh up
python3 tools/auto_initial_pose.py --ns agv1 --hint-cell 5,2 --hint-heading d
python3 tools/auto_initial_pose.py --ns agv2 --hint-cell 2,3 --hint-heading d
```

Chờ `Da bat` và cả hai lệnh định vị báo `AMCL da nhan va xac nhan vi tri`. Nhìn RViz, so với vị trí xe thực trước khi bấm bắt đầu. Nếu xe đổi ô/hướng, sửa cả biến AGV1/AGV2 và các đối số hint theo vị trí thật. Nếu khớp không đạt hoặc RViz sai vị trí, dừng ở bước này. Log khởi động nằm trong `$RUN/fleet/`.

**Terminal 2 — điều phối:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
python3 traffic_manager.py agv1 agv2 --no-stdin
```

**Terminal 3 — RViz:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
rviz2 -d tools/fleet.rviz
```

**Terminal 4 — ghi dữ liệu trước khi bấm chạy:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
RUN="$PWD/data/real_trial_04"
mkdir -p "$RUN"
printf 'start_agv1=5,2,East\nstart_agv2=2,3,East\n' > "$RUN/scenario.txt"
test -e "$RUN/measurements.csv" || printf 'car,cycle,box_id,width_true_cm,height_true_cm,front_gap_cm,delivery_row,delivery_col,video_file,notes\n' > "$RUN/measurements.csv"
sha256sum tools/cargo_live_view.py traffic_manager.py fleet_agent.py \
  src/my_bot_bringup/config/cargo.yaml src/my_bot_bringup/config/navigation.yaml \
  firmware/esp32_agv_base/esp32_agv_base.ino > "$RUN/code_sha256.txt"
cp src/my_bot_bringup/config/camera_agv1.yaml "$RUN/"
cp src/my_bot_bringup/config/camera_agv2.yaml "$RUN/"
ros2 bag record --max-bag-size 47185920 -o "$RUN/rosbag" /fleet/status /fleet/goal /fleet/cargo_event \
  /fleet/cargo_result /agv1/scan /agv2/scan /agv1/scan_filtered /agv2/scan_filtered \
  /agv1/odom /agv2/odom /agv1/amcl_pose /agv2/amcl_pose \
  /agv1/imu/data /agv2/imu/data /agv1/joint_states /agv2/joint_states \
  /agv1/camera/image_raw /agv2/camera/image_raw /agv1/cmd_vel /agv2/cmd_vel
```

**Terminal 5 — giao diện dùng camera/LiDAR thật:**

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
RUN="$PWD/data/real_trial_04"
python3 tools/cargo_live_view.py --agv1-pickup 3,5 --agv2-pickup 4,5 \
  --save-dir "$RUN/images" --event-log "$RUN/cargo_events.jsonl"
```

Bấm **BẮT ĐẦU** từng xe. Sau khi đo và xe dừng tại ô lấy, đặt **đúng kiện vừa đo** lên xe rồi bấm **ĐÃ NHẬN HÀNG**. Tại ô giao, dỡ kiện xong mới bấm **ĐÃ DỠ HÀNG (XÁC NHẬN)**; mốc này cần đối chiếu video và xe vẫn đứng yên. Chỉ bấm **QUAY LẠI LẤY TIẾP** khi muốn bắt đầu lượt mới.

**Dừng xe và xuất bảng:**

```bash
# Trong terminal ROS của xe thật: dừng lệnh điều phối trước
export ROS_DOMAIN_ID=0
ros2 topic pub --once /fleet/goal std_msgs/msg/String '{data: s}'
# Ctrl+C các terminal 2–5, sau đó tại Terminal 1:
RUN="$PWD/data/real_trial_04"
LOGDIR="$RUN/fleet" tools/fleet_real.sh down
python3 tools/analyze_cargo_timing.py "$RUN/cargo_events.jsonl" \
  --environment real --csv "$RUN/cargo_times.csv"
```

## C. Đọc kết quả

Mỗi dòng `cargo_times.csv` là một lượt của một xe: `T_pickup_s` (bắt đầu → xác nhận chất hàng), `T_delivery_s` (xác nhận chất hàng → tới ô giao), `T_total_s` (bắt đầu → tới ô giao). `T_wait_load_s` tách thời gian chờ người vận hành. `T_to_unload_confirmed_s` tính tới lúc bấm xác nhận đã dỡ; với xe thật chỉ gọi là **giao vật lý xong** khi video xác nhận. Script in thêm trung vị/khoảng và thời gian hoàn thành một đợt hai xe. Lượt thiếu mốc được giữ là `incomplete_or_invalid`, không tự tính thời gian hoàn thành. Đối chiếu mô phỏng và xe thật theo cùng hộp, vị trí đầu, ô lấy/ô giao và số lượt; không chép số mô phỏng vào cột thực nghiệm.
