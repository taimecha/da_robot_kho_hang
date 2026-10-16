# Lọc nhiễu LiDAR cho hai xe thật

## Bổ sung lọc chấm đơn ở xa (2026-10-16)

Mốc 35 cm chỉ dành cho quy tắc điểm **gần**; trước đây một chấm đơn trên 35 cm
vẫn có thể được giữ nếu lặp tại vị trí cũ. Theo thực tế của phòng thử nghiệm,
vật cản cần phát hiện có kích thước lớn. Bộ lọc mới chia nhóm các tia kề nhau
có khoảng cách phù hợp (chênh không quá `0.07 + 0.015 * min(r1, r2)` m):

- Từ 35 cm trở lên: tia đơn bị loại cả khi lặp lại; cụm 2 tia chỉ giữ sau khi
  thấy một cụm ít nhất 2 tia ở cùng vị trí trong lịch sử tối đa 5 vòng/0,5 s; cụm từ 3 tia
  trở lên được giữ ngay, kể cả khi mới xuất hiện. Nhóm qua ranh giới 0/360°
  được tính chung nếu scan đủ một vòng.
- Dưới 35 cm: giữ quy tắc thời gian trước đó, cụm từ 6 tia cần một lần xác
  nhận; cụm hẹp hơn cần hai lần xác nhận. Nếu xe đang chuyển động mà thiếu
  odometry đúng thời điểm quét, chuyển nguyên scan để không xóa nhầm vật cản.
  Scan đầu sau khởi động/ngắt vẫn giữ các tia gần, nhưng lọc tia đơn ở xa.

Replay 108 vòng quét thô xe 2 lưu ngày 2026-10-15 khi đứng yên: loại
**3.432/3.432 tia đơn** từ 35 cm trở lên; giữ 2.186/2.600 tia thuộc cụm 2,
và 18.418/18.418 tia thuộc cụm từ 3 trở lên. Dữ liệu này không chứng minh
mọi tia đơn là nhiễu; nó đo tác động của quy tắc với mẫu xe 2. Không có xe
hoặc hệ ROS đang chạy tại thời điểm sửa, nên chưa xác nhận trực tiếp trên hai
xe khi di chuyển. Node chung được dùng cho cả hai xe ở lần khởi động tiếp theo.

Giới hạn: một vật thật chỉ phản hồi đúng 1 tia sẽ bị ẩn; vật chỉ phản hồi 2
tia xuất hiện chậm một vòng quét. Xe còn lại thường cho nhiều tia ở khoảng
cách trong lưới, nhưng vẫn cần kiểm tra bằng RViz và chạy thử hai xe trước khi
coi đây là cấu hình đã nghiệm thu. Không nên nâng số tia tối thiểu quá cao
vì có thể làm mất phản hồi của xe kia ở khoảng cách lớn.

## Trạng thái trước lần bổ sung: lọc khi đứng yên và khi chạy (2026-10-15)

Node `scan_filter` lấy `/agvN/scan` và phát `/agvN/scan_filtered` cho AMCL,
costmap, collision monitor và hình LiDAR trong fleet RViz. Luồng thô vẫn giữ
nguyên cho các công cụ đo hàng/ICP. Cả xe 1 và xe 2 dùng cùng bộ lọc.

Nguyên nhân điểm nhiễu còn lọt: bản trước chuyển nguyên vòng quét trong lúc xe
di chuyển. Khi đứng yên, nó giữ cả cụm 2–6 tia nếu các tia nằm kề nhau, dù
cụm đó chỉ lóe đúng một vòng. Trên mẫu ghép raw/filtered 20 s khi đứng yên,
xe 1 còn 53 điểm dưới 35 cm lọt qua trong 13 vòng; xe 2 còn 6 điểm trong 2
vòng. Replay cùng dữ liệu qua bộ lọc mới: 0 điểm dưới 35 cm còn lọt ở cả hai
xe. Xe 1 giữ 28.325/28.615 tia trên 1,2 m; xe 2 giữ 22.893/23.055 tia trên
1,2 m. Đây là kiểm tra trên mẫu đã ghi, không khẳng định mọi loại phản xạ đều
đã biến mất trong mọi điều kiện.

Quy tắc mới: dưới `near_cutoff` (35 cm), cụm rộng ít nhất 6 tia cần thấy lại
trong một vòng quét thô trước đó; cụm hẹp 1–5 tia cần thấy lại trong hai vòng
khác nhau (tức ít nhất ba vòng liên tiếp nếu quét đều). Chỉ dùng lịch sử trong
`near_time` (0,38 s), tính bằng timestamp của vòng quét. Với tia đơn lẻ xa hơn, giữ nếu có tia kề cùng khoảng cách trong
vòng hiện tại hoặc có phản hồi gần vị trí đó trong lịch sử. Tọa độ các điểm
được đổi sang hệ `odom` nhờ vị trí xe và góc IMU trong `/agvN/odom`, nên phép
đối chiếu vẫn chạy khi xe tịnh tiến/quay. Chọn mẫu odometry gần **thời điểm
quét**, không dùng vị trí lúc gói LiDAR tới laptop (thực đo lệch ~60 ms).
Nếu thiếu odometry khớp trong 80 ms khi xe chạy, chuyển nguyên vòng quét để
tránh xóa nhầm vật cản. Lịch sử gồm tối đa 5 vòng thô,
xóa khi ngắt scan quá 0,5 s hoặc hình học quét đổi. Tia bị loại thành NaN;
không thay bằng infinity/khoảng trống. Vòng đầu sau khởi động/ngắt đi thẳng.

Mặc định: `max_jump=0.07` m, `relative_jump=0.015`, `near_cutoff=0.35` m,
`near_time=0.38` s, `near_cluster_rays=6`, `world_tolerance=0.06` m, `history_size=5`,
`max_gap=0.5` s. Có thể đổi các tham số ROS của node sau khi đo thêm. Bộ lọc
không sửa `robot_radius`, vùng dừng hoặc cấu hình Nav2.

Giới hạn an toàn: vật rộng mới xuất hiện dưới 35 cm có thể bị ẩn một vòng,
vật hẹp 1–5 tia có thể bị ẩn hai vòng quét trước khi được xác nhận, khoảng
0,24–0,28 s ở tần số đo hiện tại. Điểm
nhiễu phản xạ **lặp lại ổn định** ở cùng vị trí không thể phân biệt chắc chắn
với vật cản thật chỉ từ LiDAR; không được tăng ngưỡng để xóa cả vật cản thật.
Cần đối chiếu thực tế/RViz nếu vẫn thấy dải ổn định hoặc nhiễu do kết cấu xe.

Kiểm thử: `python3 -m unittest discover -s tests -q`. Có ca thử cụm 6 tia
chỉ xuất hiện một vòng khi xe chạy, vật gần thật sau khi xe quay/tịnh tiến,
vật xa hai tia mới xuất hiện, điểm đơn lẻ lặp lại và scan ngắt quá lâu.

Trước khi siết cụm hẹp, đo trực tiếp 53 s khi hai xe đứng yên: xe 1 loại
45/45 tia dưới 35 cm, xe 2 loại 9/10; tia còn lại của xe 2 xuất hiện qua hai
vòng quét liên tiếp. Sau khi siết, replay 165 vòng xe 1 và 133 vòng xe 2 từ
dữ liệu thô đã lưu: loại 93/93 và 16/16 tia dưới 35 cm. Cần tiếp tục kiểm
chứng trên xe đang chạy; phép đo này không thay thế thử nghiệm vận động.
Đo tiếp 38 s sau khi nạp bản siết cụm hẹp: xe 1 loại 22/22 tia dưới 35 cm,
xe 2 loại 7/7. Hai xe vẫn đứng yên; ca ROS tổng hợp xác nhận lọc khi có
odometry báo chuyển động, chưa thay được phép chạy xe thật qua vật cản.
Sau lần đồng bộ timestamp cuối, đo thêm 24 s: xe 1 loại 14/14 tia dưới 35 cm,
xe 2 loại 8/8; tia trên 1,2 m giữ lần lượt 33.344/33.555 và
25.489/25.640. Chưa đưa xe chạy thật để kiểm tra điều hướng/né vật cản với
bộ lọc này.

Sau khi sửa mã, các tiến trình node đang chạy cần được khởi động lại để nạp
mã mới. Chỉ khởi động lại khi xe đứng yên, xác nhận đã có `/scan_filtered` của
cả hai xe trước khi giao nhiệm vụ mới.

## Lịch sử khảo sát trước lần sửa này

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
Trước lần sửa 2026-10-15, lúc xe chạy/quay node chuyển nguyên scan và sau
khi dừng chờ 0,6 s mới lọc lại. Cấu hình này không còn áp dụng.

Kiểm tra trực tiếp node đang chạy khi cả hai xe đứng yên: xe 1 giữ 26/28 tia
ở vùng xe 2, xe 2 giữ 36/37 tia ở vùng xe 1. Trong 78/63 vòng tương ứng,
lọc 51/60 tia tổng cộng. Kiểm thử bản xem trước trước đó giữ 49/51 và 54/56
tia xe kia. Những phép đo này dùng vị trí AMCL để xác định vùng dự kiến; cần
tiếp tục quan sát thực tế trong RViz.

108 scan từng ghi từ xe 2 cho thấy bộ lọc một vòng đầu tiên loại quá nhiều tia;
không dùng kết quả đó làm xác nhận cho bản hiện tại. Một vòng chạy thử cũ của
xe 2 qua bốn đích đã thành công nhưng chưa kiểm chứng bản sửa 2026-10-15 khi
xe chạy thực tế.
