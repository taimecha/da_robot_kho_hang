# Chuẩn bị thử xe thật trên laptop Ubuntu

Kiểm tra ngày 2026-10-15 tại `/home/tai/Robot_kho_clone`.

## Đã chuẩn bị và kiểm tra

- Ubuntu 24.04, ROS 2 Jazzy; Nav2, AMCL, SLAM Toolbox, RViz, xacro và các gói điều khiển tay cầm đã có.
- Python: OpenCV, NumPy, SciPy, PyYAML, Pillow, requests, pyserial đã có; import các module điều hướng và hàng hóa thành công.
- Đã tải nhánh `jazzy` của `micro-ROS/micro-ROS-Agent` và `micro-ROS/micro_ros_msgs` vào `~/microros_ws/src`, build thành công. Agent tải thêm Micro-XRCE-DDS-Agent v2.4.3 và spdlog 1.9.2 trong quá trình build.
- Đã build hai package của dự án bằng `colcon build --symlink-install`.
- Agent khởi động UDP thành công trên cổng kiểm tra 18889, đã dừng sau kiểm tra. Chưa xác nhận phiên kết nối ESP32 thật.
- Kiểm tra pub/sub ROS 2 nội bộ thành công trên domain 91 với cấu hình DDS của dự án.
- Đọc được launch arguments của hai launch xe thật/Nav2; xacro xe thật sinh URDF thành công.
- Bản đồ phòng, cấu hình lưới, cargo và hiệu chuẩn hai camera đã có trong repo.
- Hai URL camera từng trả HTTP 200 với content-type image/jpeg. Lần kiểm tra sau timeout; người dùng xác nhận đã tắt các xe. Chưa xác nhận luồng ảnh hoạt động ổn định.

## Mở terminal cho dự án

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
```

File này chỉ thiết lập môi trường terminal, không khởi động robot. Không sửa `.bashrc` để tránh ảnh hưởng các workspace khác.

## Cần xác nhận khi bật xe lại

1. Laptop tại thời điểm kiểm tra có IP Wi-Fi `192.168.1.56`. Cấu hình mẫu firmware ghi agent `192.168.1.21`; cần xác nhận IP thực sự đã nạp trên ESP32. Không suy ra cấu hình đang chạy từ file mẫu. Nếu khác IP laptop, chỉnh cấu hình và người dùng tự nạp lại firmware.
2. Agent sử dụng UDP 8888 trong vận hành. Kiểm tra `/agv1/scan`, `/agv2/scan`, `joint_states`, `imu/data`, `diag` trước khi chạy điều hướng.
3. Camera: agv1 `http://192.168.1.201/capture`, agv2 `http://192.168.1.200/capture`. Kiểm tra lại ảnh khi có nguồn.
4. Bản đồ và hiệu chuẩn trong repo thuộc sa bàn/cách lắp cũ; cần đối chiếu với bố trí thực tế.
5. Đặt đúng vị trí khởi tạo hoặc định vị lại trước khi giao đích.

## Firmware và công cụ Windows

`tools/agvctl` có các chức năng serial/build/copy firmware phụ thuộc PowerShell, đường dẫn Arduino IDE và ổ đĩa Windows của máy cũ. Chưa chuyển các chức năng đó sang Linux. Máy chưa có `arduino-cli` trong PATH; thư viện Arduino LDS và micro_ros_arduino chưa được chuẩn bị. Đây là phần cần bổ sung nếu phải biên dịch/nạp lại firmware, không bắt buộc để laptop giao tiếp với xe đã nạp firmware phù hợp.

Không có `wifi_config.h` trong repo (được gitignore chủ ý). Không đưa mật khẩu Wi-Fi vào tài liệu hay Git.

Chưa chạy toàn bộ Nav2 với xe thật, chưa kiểm tra chuyển động và chưa gửi lệnh vận tốc tới xe trong phiên chuẩn bị này.
