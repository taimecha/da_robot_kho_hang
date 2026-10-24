# Thử chở hàng bằng riêng xe 2

Đặt AGV2 đứng yên ở vị trí bất kỳ trong vùng sa bàn đã có trên bản đồ,
không cần biết trước ô hoặc hướng. Quan sát (4,4), lấy (4,5).
Đặt mặt hộp phía Đông tâm ô (4,5) khoảng 35 cm, kiểm tra costmap
cho phép tiếp cận. Bật xe 2 và ESP32-CAM xe 2. Laptop nối SV BACH KHOA,
IPv4 tĩnh 192.168.1.21. Mỗi khối dưới đây chạy trong một terminal VS Code riêng.

## Terminal 1 — khởi động và định vị

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
ROBOTS=agv2 tools/fleet_real.sh up
python3 tools/auto_initial_pose.py --ns agv2
```

Đợi `Da bat` và `Da gui vi tri ban dau cho AMCL`. Lệnh auto_initial_pose dò
toàn bản đồ và mọi hướng, không phụ thuộc vị trí khởi tạo tạm của bringup.
`fleet_localize.py` chủ yếu khớp quanh AMCL, chỉ dò các nút lưới khi khớp kém;
vì vậy dùng auto_initial_pose cho yêu cầu đặt xe tự do cả giữa hai nút.
Nếu định vị báo `KHONG GUI`, hoặc xe trên RViz không khớp thực tế, chưa bấm
BẮT ĐẦU. Phòng đối xứng hoặc thay đổi nhiều có thể không xác định duy nhất;
lúc đó dùng 2D Pose Estimate trong RViz (chọn topic /agv2/initialpose) hoặc
đặt xe ở khu vực có đặc trưng rõ hơn và dò lại. Camera tự bật kèm bringup.
Chạy lại lệnh định vị khi bế xe sang chỗ khác; khi xe tự di chuyển, AMCL
tiếp tục cập nhật vị trí mà không cần chạy lại lệnh.

## Terminal 2 — bộ điều phối

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
python3 traffic_manager.py agv2 --no-stdin
```

## Terminal 3 — RViz

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
rviz2 -d tools/fleet.rviz
```

Bỏ chọn nhóm Xe 1 nếu cấu hình RViz vẫn hiện nhóm xe đang bảo trì.

## Terminal 4 — giao diện chở hàng

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
python3 tools/cargo_live_view.py --robots agv2 --agv2-pickup 4,5
```

Bấm BAT DAU: tới (4,4) → chốt 5 ảnh → tự tới (4,5).
Đặt đúng món vừa đo lên xe, bấm DA NHAN HANG → giao tới ô phân loại.
Sau thông báo thành công, dỡ hàng, chuẩn bị món tiếp ở khu lấy và bấm
DA DO HANG - QUAY LAI LAY TIEP. Xe quay về (4,4), đo mới và phân loại lại.
Góc chấp nhận hiện tại **<2,5°** (yêu cầu <3°), sai số vị trí **≤2 cm**.
Giao diện giữ mở. R chỉ đo lại tại ô quan sát; S lưu ảnh; Q đóng giao diện.

Ô giao theo chiều cao: 24–<30,5 cm → (1,1); 30,5–<37 → (1,2);
37–<43,5 → (1,3); 43,5–50 → (1,4). Dải này cần đối chiếu bằng thước
ở bố trí mới; các kiểm tra mép ảnh và LiDAR vẫn phải đạt trước khi tiếp cận.

## Dừng lệnh và tắt hệ thống

Terminal riêng, dừng điều hướng khi cần:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: 'stop'}"
```

Kết thúc: Q giao diện camera, Ctrl-C bộ điều phối và RViz, sau đó:

```bash
ROBOTS=agv2 tools/fleet_real.sh down
```

Giao diện gửi tên `agv2` trong lệnh, tránh phụ thuộc số thứ tự của danh sách xe.
Không chạy đồng thời `cargo_task.py` hoặc giao diện camera thứ hai.
