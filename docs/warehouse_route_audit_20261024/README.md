# Kiểm tra điều phối ba xe — 24/10/2026

Chạy mới kịch bản `demo30`: ba xe, mỗi xe 10 kiện. ROS 2 Jazzy chạy ba Nav2,
`fleet_agent`, bộ điều phối, xe động học giả, camera RGB và LiDAR tổng hợp.
Chạy không mở Gazebo/GUI (`--no-gazebo --headless`); đây không phải phép đo xe
thật hoặc mô phỏng động lực học tiếp xúc.

```bash
ROS_LOG_DIR=/tmp/warehouse_ros_logs_20261024 ./tools/warehouse_sim.sh up \
  --scenario demo30 --headless --no-gazebo --autostart \
  --exit-on-complete --out /tmp/warehouse_route_audit_20261024_c
python3 tools/warehouse_report.py /tmp/warehouse_route_audit_20261024_c
python3 -m unittest discover -s tests -p 'test_*.py' -q
```

| Chỉ tiêu | Kết quả |
|---|---:|
| Giao thành công | 30/30, mỗi xe 10/10 |
| Camera phân loại đúng | 30/30 |
| Kiện rời băng tải nhận | 30/30 |
| Cấp kiện tiếp sau khi xe rời vùng lấy | 27/27 |
| Kiểm tra FIFO, quyền trạm, giao sai ô | 0 lỗi |
| Khoảng cách tâm xe nhỏ nhất ghi nhận | 0,589657 m |
| Cảnh báo khoảng cách dưới 0,26 m | 0 |
| Bắt đầu nhiệm vụ → kiện cuối | 715,751 s |
| Bắt đầu nhiệm vụ → cả ba xe đỗ xong | 736,402 s |
| Kiểm tra mã | 164/164 đạt |

Quãng đường từ odometry trong thời gian nhiệm vụ: AGV1 **45,689 m**, AGV2
**62,488 m**, AGV3 **68,541 m**. Tổng góc xoay tuyệt đối tương ứng **5483,6°**,
**5650,4°**, **5819,0°**; số này bao gồm chỉnh hướng tại đích và các chuyển
động tránh nhau. Trong 95 lộ trình được cấp, 4 lộ trình dài hơn khoảng cách
Manhattan tổng cộng 8 cạnh; có một lần xe nhường đường. Xe rời trạm sang Đông
8 lần, Tây 3 lần, Bắc 19 lần; không có đích thoát trung gian.

Bài kiểm tra trong `tests/test_traffic_turn_cost.py` duyệt 9.600 cặp vị trí,
hướng đầu và hướng đích trên lưới 5×5 trống. Mỗi đường được chọn đều ngắn nhất
theo số cạnh và có số lần đổi hướng ít nhất trong các đường ngắn đó. Khi có xe
khác, đường có thể dài hơn để tránh ô đang giữ hoặc giảm thời gian chờ; chưa có
chứng minh tối ưu thời gian toàn cục cho mọi lịch giao nhận.

Các file trong thư mục này là bản sao từ chính lượt chạy: [sự kiện](events.jsonl),
[quỹ đạo](trajectories.csv), [log điều phối](traffic.log), [kiểm tra](audit.json),
[tóm tắt](summary.json), [cấu hình](grid.yaml), [kịch bản](scenario.json) và
[hình quỹ đạo](trajectories_and_deliveries.png). `code_sha256.txt` ghi mã băm
các file điều khiển và báo cáo sau lượt chạy.
