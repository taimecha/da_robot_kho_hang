# Active Context (cập nhật 2026-10-16)

## Hiện tại
- Workspace đang chạy: `/home/tai/Robot_kho_clone`; remote đẩy khi người dùng yêu cầu là
  `https://github.com/taimecha/da_robot_kho_hang.git`.
- Hai xe đã thử trên sàn với sa bàn 5×5. `fleet_agent` kiểm sai số vị trí ≤ 2 cm và góc
  < 0,9° tại đích; `traffic_manager.py --no-stdin` nhận đích qua `/fleet/goal` và tránh
  lệnh còn chờ trong terminal.
- Đo hộp trực tiếp: `tools/cargo_live_view.py --agv1-pickup 3,5 --agv2-pickup 4,5` cho
  bố trí hiện tại (xe 1 hộp nâu, xe 2 hộp xanh). Hộp nâu đo 18,5×31,4 cm, hộp xanh
  12,3×23,9 cm (trung bình 5 ảnh); 23,9 cm được nhận vào ô (1,1) với nút xanh.
  Nút chỉ gửi đích sau khi người giám sát đã đặt hàng lên xe và bấm xác nhận.
  Giao diện tự đóng sau khi cả hai xe được xác nhận và tới đúng ô giao; chạy trực tiếp
  trong terminal, không dùng `setsid` hoặc `&`.
- LiDAR hai xe đã lọc nhiễu chấm đơn và kiểm tra lúc xe dừng/chạy; xem
  `docs/LOC_LIDAR.md` và `docs/DO_HOP_TRUC_TIEP.md`.

## Trạng thái
**Hai xe thật chạy được theo logic mô phỏng** (2026-10-10): 6 bước thử (từng xe → song song →
đổi chỗ hàng 5 → đường cắt nhau → đổi chỗ như mô phỏng) đều phối hợp đúng, không va (gần
nhất 59–60 cm). Người dùng xác nhận OK; hướng dẫn Kịch bản 4 đã viết lại để người dùng tự chạy.

Phần cứng đã chốt:
- Cả hai xe dùng **mạch in 2026-10-05** (encoder 23/18/19/17, PWM LiDAR 4).
- Dây động cơ: **xe 1 nối thẳng** (27/26/25 trái), **xe 2 nối chéo** (33/32/14 trái).
- LiDAR: xe 1 ×1.0, xe 2 ×1.062. Gyro xe 1 cùng chiều encoder.
- Firmware **tách 2 file**: `Xe_1\Xe_1.ino`, `Xe_2\Xe_2.ino` (sinh từ file gốc bằng
  `tools/agvctl fw split`). Có `/agvN/diag` + sự kiện ESTOP gửi ngay + `/agvN/estop_reset`.
- URDF: `base_footprint` tại **tâm trục bánh** (xoay tại chỗ không trôi vị trí).

## Vấn đề còn mở
- **Xe 2 BROWNOUT (~18:20 ngày 2026-10-10)**: ESP32 xe 2 tự khởi động lại vì sụt áp, sau đó
  `joint_states` (publisher reliable) gửi lỗi mãi (`pub_fail` ~50/s) → không odom/TF. Cần sạc pin,
  bật lại xe, `fleet_real.sh down/up`. Brownout lặp lại → kiểm tra pin / tụ nguồn ESP32.
- Costmap: đã đổi cấu hình + `fleet_agent` (decay 1 s) nhưng **chưa thử với cả stack mới**
  (đo trên agv1 bằng script: ô kẹt 0, vật thật không chớp).
- Phía **tây** sa bàn (cột 1) có đồ đạc mới không có trên bản đồ → xe ở cột 1 lệch 4–6 cm
  (phía đông < 1 cm). Cần dọn hoặc **vẽ lại bản đồ + căn lại sa bàn**.
- Chế độ 1 xe chưa chạy lại sau khi đổi URDF (base_footprint) — nên thử một lần.
- Xe 2 vẫn dùng `wheel_radius`/`coast` của xe 1 (chạy thật thấy ổn).

## Đo hàng bằng camera — code xe thật XONG (2026-10-13), CHỜ phần cứng
Chỉ đo **rộng × cao**; camera nhìn thẳng, **Z + mặt phẳng mặt hàng từ LiDAR**; xe KHÔNG chở hàng.
**Lấy hàng phía Đông ô (2..5, 5)** (xe quay Đông; (1,5) bị bỏ), **trả về hàng 1** cột theo loại
(`cargo.yaml place.*`, `classes`). Lệnh `cargo_task.py`: "<xe> <hàng>" (vd "1 3" → (3,5)).
- ESP32-CAM: `firmware/esp32_cam` (`#define XE`, IP tĩnh .200/.201, `/capture` :80, `/stream` :81,
  `/status`), biên dịch OK (board esp32cam); đã chép `D:\CTU\Ardunio ide\code\Cam_1`, `Cam_2`
  (+ wifi_config.h). Người dùng có sẵn `CameraWebServer`, `code_camera` (QQVGA) — không dùng.
- `tools/esp32_cam_node.py` (tham khảo esp32_cam_node.py người dùng gửi: luồng riêng giữ ảnh mới
  nhất; thêm mode capture mặc định, tự nối lại, rotate) — `fleet_real.sh up` tự bật (`CAM=0` tắt).
  Thử với ESP32-CAM giả: capture 5/s, stream 12/s, mất/nối lại OK.
- `tools/cam_calib.py agvN` → `config/camera_agvN.yaml` (đè `camera:`; x,y,z đo tay). Thử bàn cờ
  giả: fx 492,1/492,8, RMS 0,28. `cargo_task.py --sim` bỏ qua file hiệu chuẩn.
- Mô phỏng bố trí mới: 4 hàng sai ≤ 1 mm, về đúng (1,1)–(1,4).
- **2026-10-14: người dùng KHÔNG nạp lại được ESP32-CAM** — chỉ có firmware stream sẵn
  (**xe 1 = 192.168.1.201, xe 2 = .200** — chéo). `cargo.yaml cameras.agvN.url` = luồng đó; node đọc
  luồng (OpenCV/FFmpeg). Ảnh khác cỡ hiệu chuẩn (cùng 4:3) → phóng lên, cảnh báo. `firmware/esp32_cam`,
  `Cam_1/Cam_2` để dự phòng, không dùng. Camera phải nhìn thẳng ngang (code cũ của họ nhìn sàn?).
- **2026-10-14 camera ONLINE**: .200/.201 chạy CameraWebServer (320x240, framesize 6, quality 10),
  có cả /capture → dùng url .../capture 3 Hz (~30 KB/s/xe; stream 23-25 fps ~280 KB/s). Ảnh: nhìn
  thẳng ngang, không ngược, thấp (sàn gạch sáng). Vào ROS OK (3 ảnh/s/xe) với DDS xml mới.
- **Chờ người dùng:** bật nguồn 2 XE (agent không có phiên nào lúc thử) để thử DDS mới với agent
  thật; hiệu chuẩn bàn cờ; đo x,y,z camera.
- **Chạy thật 2026-10-14 tối:** sụt áp đã sửa (người dùng sửa nguồn; 2 xe ổn định 60 s+). Xe đặt
  (3,2)Đ và (2,2)Đ (KHÔNG phải vị trí mặc định) – tìm bằng ICP toàn sa bàn. Phòng có xe máy/đồ đạc
  ngoài bản đồ: ICP chỉ khớp 40–56%, AMCL trôi ~5 cm/4–7° sau 1,8 m →  tự ICP + sửa AMCL
  ở ô lấy hàng (đã chạy: sửa 2,8 cm/−5,6°). Lệnh 1 3, 1 4 chạy đúng đường; chưa có hàng → báo lỗi
  đúng. **Nên vẽ lại bản đồ** (Kịch bản 1). Chờ: đặt hàng, hiệu chuẩn camera, đo x,z camera.
- **Camera xe thật cao 23,5 cm**, cách **tâm thân xe** 82,7 mm (người dùng xác nhận 2026-10-14; tâm thân
  = LiDAR, trước trục 14 mm → x = 0,0967 từ trục); **2 xe lắp giống nhau** → `camera_agv1/2.yaml`. `place.gap`
  0,35 → 0,65 → **0,85** (sau hiệu chuẩn xe 1: nhìn xuống chỉ 19,7°; lidar.max_range 1,2).
  Hiệu chuẩn xe 1 (ROS camera_calibration, 53 ảnh): fx 302,4 fy 310,2 (pixel KHÔNG vuông,
  ép fx=fy thì RMS 0,23→0,44 px), góc ngang 55,8°. Xe 2: fx 297,6 fy 305,1, góc ngang 56,5°, RMS 0,28 px (bản sao ~/agv_logs/calib/). Hộp thấp hơn camera: `front_top_edge`
  (vạch sáng mặt trên/trước), offline 320×240 sai ≤ 3,5 mm. Bàn cờ: calib.io 8×10 ô 20 mm →
  `tools/cam_calib.py agvN --board 9x7 --square 0.020`.
- **Đo hàng thật đầu tiên 2026-10-14 tối:** hộp xanh (4,5) thật 12,0×23,9 cm → xe 1 đo **12,3×24,1**
  (+3/+2 mm), Z 75,6 cm, loại "cao" đúng. Cảnh báo "đáy hàng z=+1,6 cm" → camera có thể ngóc ~1,2°.
  **LiDAR thật quét ở ~21–24 cm** (thấy hộp 23,9 cm, KHÔNG thấy hộp nâu 21,1 cm) — URDF ghi 0,075 sai;
  hàng phải cao hơn LiDAR, hoặc làm đo chỉ bằng camera (chân hộp chạm sàn + bù góc ngóc). Chờ người
  dùng đo chiều cao LiDAR + chọn hướng. Xe 2 ESTOP "bánh kẹt / mất encoder" ngay khi chạy (đứng giữa
  (2,2)-(2,3)) → xe 1 không lập được đường về (1,3). Xe đặt (1,2)Đ, (2,2)Đ (không phải mặc định).
- **Vòng đo hàng thật trọn vẹn (xe 1) 2026-10-14 ~22h:** hộp (3,5) thật rộng 18,5 × cao 32,0 cm → đo
  19,3 × 31,2 (+8/−8 mm), loại "rộng" → tự về (1,4). 320×240 ở 75 cm: 1 px ≈ 2,5 mm → sai ±3 px mép.
  Cải thiện: camera VGA (CameraWebServer /control?var=framesize, không nạp lại) + hiệu chuẩn lại.
  `cargo_task`: bỏ hủy khi 'failed' thoáng qua, TIMEOUT 300 s. Bảng loại: cao/rộng tới 45 cm.
  **Xe 2 hỏng:** bánh không quay (ESTOP "bánh kẹt / mất encoder"), `/agv2/diag` KHÔNG về (0 Hz) nên
  laptop không biết ESTOP; người dùng báo vị trí xe 1 "(2,3)" nhưng thật là (3,2) (ghi cột trước?).
- **2026-10-15 ~14h45 — ĐO HÀNG THẬT ĐẠT, cả 2 xe trọn vòng:** hộp 18,5×32,0 → 18,7×32,0 (xe 1);
  hộp xanh 12,0×23,9 → 13,0×24,1 (xe 2); về đúng (1,4), (1,3). Đã đổi: tách hộp **GrabCut** gieo
  hạt từ LiDAR (hạt ở ½ độ cao tia), mép trên = hàng hộp chiếm < 50% bề rộng (chống vệt rò khe cửa),
  tinh mép theo kênh xám ±3 px, `camera.pitch` → cao = mép trên tới sàn. `cam_extrinsic.py --height`
  đo yaw+pitch: xe 1 −2,49/−2,29°, xe 2 +2,92/−1,09° (pitch tính từ chính 2 hộp này — cần hộp khác
  để kiểm độc lập). Xe 1 nằm ở (3,2), xe 2 ở (2,2) khi bật (người dùng hay ghi lẫn). Nav2 agv1 có lần
  mất DDS (fleet_agent "khong thay Nav2") → restart riêng Nav2 xe đó thì được. Firmware mới
  (restart khi tạo entity lỗi) đã nạp? — lần này diag cả 2 xe đủ.
- **2026-10-15 chiều — đã sửa (code, chưa thử xe thật):** (1) `camera.yaw` + `tools/cam_extrinsic.py`
  (mô phỏng: ước lượng 3,54/3,5°, −1,74/−1,7°) + kẹp mép ảnh vào cửa sổ quanh mút LiDAR (đã bù yaw: sai
  ≤ 6,6 mm; KHÔNG bù yaw thì tới 22 mm → bắt buộc đo yaw). (2) `cargo_task` giao lại khi traffic_manager
  bỏ lộ trình (WARN "dung:"), 3 lần × 5 s — mô phỏng OK. (3) Firmware: lỗi tạo entity → tự restart
  (≤ 3 lần, RTC) — biên dịch OK, đã `fw split` → **người dùng cần nạp Xe_1, Xe_2**. Ảnh gốc `_raw.png`.
- **2026-10-15 (bánh xe 2 đã sửa):** xe 2 trọn vòng (3,5)→(1,4); hộp 18,5×32,0 đo **22,5**×32,2 —
  rộng sai +4 cm: ảnh NGƯỢC SÁNG (cửa kính sau hộp, hộp nâu thành xám) + vạch LiDAR lệch ~20 px so với
  hộp trong ảnh → **camera xe 2 xoay ~3–4° so với trục xe** (xe 1 ~−1,7°) → thấy mặt bên. Cần thêm
  `yaw` camera (ngoại tham camera↔LiDAR) + phông nền. Xe 1 kẹt "vật cản trên đường" ở (2,4)→(3,4)
  (LiDAR sau đó sạch — có thể người đứng gần); traffic_manager bỏ sau 5 lần/~10 s (WARN "dung:"),
  cargo_task chỉ bắt ERROR → chờ tới 300 s. `diag` lúc về lúc không, đổi xe (lần này xe 1 mất).
- **DDS 2026-10-14:** gói UDP > 1472 B trên 127.0.0.1 bị rơi → `tools/fastdds_localhost.xml`
  (bashrc + fleet_real.sh + fleet_sim.sh; bashrc cũ: ~/.bashrc.bak_xml). Đã thử: mô phỏng 2 xe +
  đo hàng OK; fleet_real.sh với ESP32 giả OK (Nav2 ~50 s, chờ tối đa 120 s). **Chưa thử với
  micro-ROS agent thật** — lần chạy xe thật tới: kiểm tra /agvN/scan, joint_states về đủ.
- Bẫy: thử phát hiện node bằng `timeout ros2 run ...` cho kết quả sai (chập chờn) — chạy
  thẳng python, chờ ≥ 5 s.

## Gợi ý khi người dùng quay lại
- 2026-10-15: yêu cầu góc tại goal < 1°. `grid_real.yaml` đặt `done_deg: 0.9`, bật xung
  góc nhỏ và 12 lần bù; `fleet_agent` chỉ báo `done` khi đo được < 0,9° và ghi sai số
  vị trí/góc, hết lần bù thì báo lỗi. Thử thực tế ngay tại (3,5)/(4,5): agv1 0,25°,
  agv2 0,35°; sai số vị trí tương ứng 3,7/3,1 cm. Hai xe đã đứng yên ở đích.
- Chạy: `tools/fleet_real.sh up` → `python3 tools/fleet_localize.py` → `traffic_manager.py`.
- Lỗi xe không chạy: xem `/agvN/diag` (ESTOP + lý do), `tools/wheel_test.py agvN` (kê bánh).
- Không cho xe chạy khi người dùng chưa cho phép trong phiên.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h` và `nhiem_vu.txt`; chỉ đẩy GitHub khi được bảo.
