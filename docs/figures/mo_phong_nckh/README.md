# Dữ liệu minh chứng mô phỏng điều phối AGV

- `du_lieu_mo_phong.json`: bản tin `/fleet/status`, lệnh `/fleet/agv1/cmd`, `/fleet/agv2/cmd` và thời điểm các sự kiện trong một lần chạy của bốn tình huống.
- `so_lieu_tong_hop.json`: thời gian đến vùng đích và khoảng cách hai tâm xe nhỏ nhất, tính từ tệp trên.
- `ket_qua_bon_tinh_huong.png`, `khoang_cach_hai_xe.png`: hình vẽ trực tiếp từ dữ liệu ROS.
- `gazebo_*.png`, `rviz_*.png`: ảnh chụp màn hình Gazebo/RViz trong khi mô phỏng chạy.

Môi trường: ROS 2 Jazzy; `ROS_DOMAIN_ID=87`; bản đồ `real_room_map.yaml`; lưới 5 × 5, ô 0,60 m. Xe ảo dùng định vị lý tưởng `map=odom`, không có AMCL hay mô hình va chạm vật lý của Gazebo. Bộ ghi đánh dấu “đến vùng đích” khi vị trí trong vòng 7 cm liên tục 2 giây; đó không phải xác nhận sai số dừng cuối của Nav2.

Để chạy lại trong thư mục gốc dự án, mỗi terminal trước hết chạy `source /opt/ros/jazzy/setup.bash && source install/setup.bash`, sau đó mở ba terminal với các lệnh tương ứng:

```bash
ROS_DOMAIN_ID=87 AGV1='3 2 0' AGV2='3 4 180' tools/fleet_sim.sh up
ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2 --no-stdin
ROS_DOMAIN_ID=87 python3 tools/record_fleet_research.py
```

Sau khi thu dữ liệu, dựng lại hình bằng `python3 tools/plot_fleet_research.py`. Cần để `tools/fleet_sim.sh up` khởi động xong cả hai Nav2 trước khi chạy bộ ghi; khi hoàn tất dùng `tools/fleet_sim.sh down`. Mở Gazebo bằng `ROS_DOMAIN_ID=87 python3 tools/fleet_gazebo.py` nếu cần xem 3D, và RViz bằng `ROS_DOMAIN_ID=87 rviz2 -d tools/fleet.rviz`.
