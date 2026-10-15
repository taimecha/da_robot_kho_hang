# Project Brief

## Dự án
**robot_ws_3**: hệ AGV (xe tự hành) mini cho đồ án. Mỗi xe là ESP32 + 2 động cơ có encoder
+ LiDAR YDLIDAR X2 + IMU BNO055, nói chuyện với laptop qua **micro-ROS trên Wi-Fi**. Laptop
chạy ROS 2 Jazzy trong WSL2 (Windows 11): định vị AMCL trên bản đồ có sẵn, Nav2 bám
đường, và logic sa bàn.

GitHub: `NguyenHuynh-2004/Robot_kho` (đẩy bằng git phía Windows; **chỉ đẩy khi người dùng
bảo**).

## Bài toán
- Sàn có **sa bàn 5×5** kẻ vạch, ô **60 cm** (nút đánh số hàng 1–5 từ dưới lên, cột 1–5
  từ trái sang). Người dùng nhập **hàng, cột, hướng** (Đ/T/B/N); xe chỉ được đi **trên các
  cạnh**, xoay tại chỗ ở chỗ rẽ, né nút/cạnh bị vật cản chặn.
- **Hai xe** chạy cùng lúc do một bộ điều phối (`traffic_manager.py`): giữ chỗ trước,
  ưu tiên xe **gần đích hơn**, tự gỡ kẹt (đi vòng, né sang ô bên, đổi vai, dừng).

## Tiêu chí đạt (người dùng đặt)
- **Tại đích:** mục tiêu sai số vị trí ≤ **3 cm**, hướng phải **< 0,9°** mới báo hoàn thành. (Đây là tiêu chí *tại đích*,
  không phải khi đang bám quỹ đạo.)
- Hai xe không va nhau (mô phỏng: khoảng cách tâm–tâm nhỏ nhất ≥ ~57 cm).

## Phạm vi
- Có: SLAM vẽ bản đồ bằng tay cầm, chạy 1 xe thật, chạy 2 xe (mô phỏng và thật),
  Gazebo để xem và đặt vật cản trong mô phỏng 2 xe, công cụ hiệu chỉnh / kiểm tra phần cứng.
- Không có: chạy trên phần cứng khác ngoài ESP32 DevKit + L298N; đẩy GitHub tự động.

## Người dùng & cách làm việc
- Người dùng nói **tiếng Việt**; tài liệu và log trong code bằng tiếng Việt
  (comment code không dấu).
- **Người dùng tự nạp firmware** (phải giữ nút BOOT). Trợ lý chuẩn bị file, biên dịch
  kiểm tra, chép vào thư mục Arduino, rồi đọc log.
- Làm **từng giai đoạn**; khi người dùng nói "xong" mới sang bước tiếp.
- **Không gửi `/cmd_vel` tới xe thật** nếu người dùng chưa cho phép rõ ràng.
- Không commit `wifi_config.h` / mật khẩu Wi-Fi.
