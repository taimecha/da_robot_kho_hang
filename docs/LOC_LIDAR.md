# Lọc điểm LiDAR đơn lẻ (hai xe thật)

## Trạng thái mới nhất: lọc theo không gian và thời gian

### Đo xe 2 khi đứng yên ngày 2026-10-15

Trong 144 vòng quét gốc, 25,9–26,2% điểm cách ô tường gần nhất trên bản đồ cũ
trên 25 cm; xe 1 ở vị trí hiện tại là 13,9%. 44 hướng tia xe 2 có điểm lệch bản
đồ trong ít nhất 80% vòng, chiếm khoảng 71% số điểm lệch. Nhiều dải 0,5–1,2 m
chỉ dao động vài mm qua thời gian. ICP khớp lại xe 2 chỉ đổi vị trí 0,9 cm,
không giải thích được các dải lệch vài chục cm. Những điểm này không thể coi là
tia lóe đơn lẻ để xóa bằng bộ lọc hiện tại: có thể là bề mặt thật chưa có trên
bản đồ, hoặc phản xạ lặp lại của kết cấu. Cần đối chiếu môi trường vật lý và
vẽ lại bản đồ nếu đồ đạc/phông nền đã đổi. Không tăng ngưỡng lọc để ép xóa
các dải ổn định vì điều đó cũng xóa xe/hàng/vật cản thật.

Sau khi bọc bốn trụ bằng băng đen, chạy xe 2 theo vòng
`(5,2) → (5,4) → (3,4) → (3,2) → (5,2)`; cả bốn đích đều báo tới,
không có ESTOP hay dừng vì vật cản. Đo lại 72 vòng khi đứng yên ở (5,2):
20,9% điểm gốc lệch tường bản đồ >25 cm, so với khoảng 25,9% ở lần trước.
Hướng xe sau vòng khác trước ~2° và thời điểm/môi trường khác nhau, nên không
quy toàn bộ thay đổi cho băng đen. Bộ lọc loại 42/15306 tia hợp lệ trong 70
vòng khớp timestamp; các dải ổn định vẫn còn. Nav2 có cảnh báo vòng điều khiển
thực tế ~7–10 Hz so với 20 Hz mong muốn, nhưng vẫn tới đủ bốn đích.

Bộ lọc một vòng ban đầu loại toàn bộ phản hồi dự kiến của xe còn lại: xe 1
mất 54/54 tia, xe 2 mất 40/40 tia. Đã thay bằng bộ lọc mới, so sánh tối đa 5
vòng gốc trước đó, cho phép lệch một ô góc. Điểm hẹp lặp lại được giữ.
Lúc xe chạy/quay, node chuyển nguyên scan để không mất vật cản mới trong lúc
góc nhìn thay đổi; sau khi dừng, chờ 0,6 s rồi mới lọc lại. Mặc định bật.

Kiểm tra trực tiếp node đang chạy khi cả hai xe đứng yên: xe 1 giữ 26/28 tia
ở vùng xe 2, xe 2 giữ 36/37 tia ở vùng xe 1. Trong 78/63 vòng tương ứng,
lọc 51/60 tia tổng cộng. Kiểm thử bản xem trước trước đó giữ 49/51 và 54/56
tia xe kia. Những phép đo này dùng vị trí AMCL để xác định vùng dự kiến; cần
tiếp tục quan sát thực tế trong RViz.

Luồng sau khi chạy lại `tools/fleet_real.sh up`:

`/agvN/scan` (ESP32, giữ nguyên) → `scan_filter` → `/agvN/scan_filtered`

AMCL, costmap local/global, collision monitor và hình LiDAR của fleet RViz dùng bản lọc.
Các công cụ đọc `/scan` trực tiếp như đo hàng và định vị ICP vẫn dùng dữ liệu gốc.
Chế độ bringup một xe và fleet_sim.sh chưa thay đổi.

Một tia hợp lệ được giữ nếu có ít nhất một tia sát bên trong cùng vòng,
hoặc một tia tại ô góc đó/ô liền kề trong tối đa 5 vòng trước, có khoảng cách
chênh không quá `0.15 + 0.02 * min(r1, r2)` mét. So sánh vòng qua 0/360 độ
chỉ với scan đủ vòng. Tia đơn lẻ không có hỗ trợ được thay bằng NaN; không
thay bằng infinity và không nội suy thành khoảng trống. Không đổi timestamp,
frame, góc tia hoặc các khoảng cách được giữ. Vòng đầu tiên đi thẳng để không
mất vật cản mới. Lịch sử xóa khi xe chạy/quay, scan ngắt trên 0,5 s hoặc
hình học scan thay đổi.

Tham số node: `enabled`, `max_jump` (m, mặc định 0.15), `relative_jump`
(mặc định 0.02), `history_size` (mặc định 5), `max_gap` (s, mặc định 0.5).
Không đổi tham số tránh va chạm, robot_radius, clearance hoặc range_min.

Giới hạn: vật nhỏ chỉ tạo một tia trong một vòng có thể bị loại khi xe đứng
yên. Nhiễu lặp lại hoặc tạo thành cụm nhiều tia vẫn được giữ; lọc không khôi
phục vùng khuất do trụ và không sửa trễ TF/Wi-Fi. Xe 2 đã chạy một vòng bốn
đích với bộ lọc mới; chưa thử nhiều lượt hoặc nhiều bố trí vật cản.

Kiểm thử: `python3 -m unittest discover -s tests -v`.
108 scan đã ghi từ xe 2 cho thấy bộ lọc một vòng ban đầu loại quá nhiều tia;
không dùng kết quả đó làm xác nhận cho bộ lọc hiện tại. Kiểm thử tổng hợp gồm
vật mới hai tia, vật hẹp lặp lại, điểm lóe một vòng và xe đang chuyển động.

Áp dụng cho hệ đang chạy: dừng nhiệm vụ, thoát traffic_manager/cargo_task trước;
chạy fleet_real.sh down rồi up, định vị lại, sau đó mới mở lại traffic_manager.
Không chạy hai bộ điều phối cùng lúc. Không tự giao lại đích cũ.

Hoàn tác bộ lọc: trong fleet_real.sh chuyển các đầu vào scan_filtered về scan
(costmap và remap fleet_viz); trong fleet_real_robot.launch.py đặt
amcl.scan_topic về scan, bỏ node scan_filter; build và khởi động lại hệ.
