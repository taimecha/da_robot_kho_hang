# Chạy hai xe đến hộp và xem số đo trực tiếp

Hai hộp đặt phía Đông ô lấy hàng, xe 1 đến **(hàng 3, cột 5)**, xe 2 đến
**(hàng 4, cột 5)**, cùng quay **Đông**. Hàng 1 ở dưới cùng, cột 1 ở bên trái
trên RViz. Bật nguồn hai xe và hai ESP32-CAM; giữ xe đứng yên khoảng 2 giây
sau lúc bật nguồn.

Mở các terminal riêng trong VS Code. Ở terminal 1, lệnh dưới dùng vị trí cuối
đã ghi nhận: xe 1 ở (3,5), xe 2 ở (4,5), cùng quay Đông. Nếu đã bế xe sang
chỗ khác, thay `AGV1`, `AGV2` bằng **hàng cột góc hướng** thực tế trước khi chạy.
Chờ `Da bat` rồi mới chạy dòng định vị. Chỉ giao đích khi **cả hai** xe có dòng
`da gui cho AMCL`, và vị trí hai xe trên RViz khớp vị trí thực:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
AGV1="3 5 0" AGV2="4 5 0" tools/fleet_real.sh up
python3 tools/fleet_localize.py
```

Terminal 2, giữ bộ điều phối chạy. Chưa cần nhập vào các câu hỏi của nó:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
python3 traffic_manager.py agv1 agv2
```

Terminal 3, mở RViz:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
rviz2 -d tools/fleet.rviz
```

Terminal 4, mở ảnh và số đo hai camera:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
python3 tools/cargo_live_view.py
```

Sau khi vị trí trên RViz khớp thực tế, terminal 5 giao hai đích cùng lúc:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: 'pair 1 3 5 d ; 2 4 5 d'}"
```

Cửa sổ đo chỉ hiện số khi xe đã dừng tại ô đích, sai lệch hướng Đông dưới
**1°**, mặt hộp gần song song đầu xe trong **3°**, LiDAR và ảnh nhất quán,
và toàn bộ hộp nằm trong ảnh. Giao diện lấy **trung bình 5 lần đo hợp lệ** cho
từng xe rồi **chốt** số, không tính lại cho tới khi bấm **R**. `DIST` là khoảng
cách từ camera tới mặt trước hộp; `WIDTH`, `HEIGHT` là rộng, cao của mặt hộp.
Khi không đủ điều kiện, dấu `--` và lý do được hiển thị. Với vị trí hộp cách
tâm ô lấy hàng khoảng **85 cm**, khoảng chiều cao đo tin cậy chung của hai xe
là **24–50 cm**: dưới 24 cm tia LiDAR có thể đi qua phía trên hộp; trên 50 cm
xe 1 có thể cắt mất mép trên ảnh. Đây là giới hạn thực nghiệm/thận trọng từ
góc tia LiDAR và hiệu chuẩn camera hiện tại, không phải thông số tuyệt đối của
hai cảm biến. Chia đều khoảng này để chọn ô giao hàng:

| Chiều cao trung bình 5 lần | Ô giao |
|---|---|
| 24 đến dưới 30,5 cm | (1,1) |
| 30,5 đến dưới 37 cm | (1,2) |
| 37 đến dưới 43,5 cm | (1,3) |
| 43,5 đến 50 cm | (1,4) |

Sau khi **đã đặt hàng lên xe**, bấm nút **DA NHAN HANG** ngay dưới ảnh của xe
đó. Chương trình lấy chiều cao đã chốt để chọn ô và gửi một đích qua
`/fleet/goal` cho `traffic_manager`; không gửi lặp nếu bấm lần hai. Hộp ngoài
24–50 cm không được giao tự động. Nếu cả hai hộp thuộc cùng một ô, chờ xe trước
hoàn tất và rời ô đó rồi mới xác nhận xe sau. **Không chạy `cargo_task.py` song
song với giao diện này** vì đó là luồng đo/phân loại cũ và cũng gửi đích.

Bấm **R** để đo lại các xe chưa giao, **S** để lưu ảnh vào `/tmp/cargo_live`,
**Q** để đóng cửa sổ. Hai camera đang dùng:
`192.168.1.201` (xe 1) và `192.168.1.200` (xe 2).

Để dừng khẩn cả hai xe, ở terminal đã `source` môi trường chạy:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: 'stop'}"
```

Kết thúc: Ctrl-C bộ điều phối và RViz, đóng cửa sổ đo, rồi chạy
`tools/fleet_real.sh down`.
