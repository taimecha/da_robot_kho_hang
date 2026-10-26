# Thử nghiệm hai AGV thật — 26/10/2026

Dữ liệu đã được đưa vào dự án để quản lý cùng mã nguồn. Bản gốc tại
`~/nckh_evidence/real_trial_03/` vẫn được giữ làm bản sao dự phòng.
Thời gian trong tài liệu tính theo Asia/Ho_Chi_Minh (UTC+7).

## Các file cần xem

| Đường dẫn | Nội dung |
| --- | --- |
| `cargo_times_latest.csv` | Kết quả riêng lượt cuối: kích thước đo được và thời gian từng chặng của hai xe |
| `cargo_times.csv` | Bốn chu kỳ của hai lượt, gồm lượt đầu có lỗi căn tâm và thời gian chờ xử lý |
| `cargo_events.jsonl` | Nhật ký gốc: bắt đầu, tới ô quan sát, đo, tới ô lấy, xác nhận nhận hàng, tới ô giao, xác nhận dỡ hàng |
| `rosbag/` | Camera, LiDAR gốc/lọc, odometry, AMCL, IMU, joint states, vận tốc và trạng thái điều phối |
| `fleet/` | Bản chụp log điều hướng, định vị, camera và cấu hình Nav2 đã sinh trong phiên chạy |
| `scenario.txt` | Vị trí xuất phát khai báo: AGV1 (5,2), AGV2 (2,3), cùng hướng Đông |
| `agv2_observation_failure.txt` | Ghi nhận lỗi căn tâm ô quan sát của AGV2 ở lượt đầu |
| `rosbag_verification.json` | Kết quả đối chiếu dữ liệu rosbag sau khi chia file với bản gốc |
| `SHA256SUMS` | Mã kiểm tra toàn bộ file dữ liệu trong thư mục này |

Không có ảnh chụp rời trong dữ liệu gốc của lượt này; ảnh camera nằm trong rosbag.
File PID dùng để điều khiển tiến trình đang chạy không được sao chép vào bộ dữ liệu.

## Phạm vi rosbag

- Ghi từ **16:08:43,217** đến **16:11:23,924**, thời lượng **160,706 giây**.
- Có **58.345 bản tin**, **20 topic**; bao phủ trọn lượt cuối.
- Rosbag được chuyển từ một file MCAP khoảng 234 MiB thành **6 file MCAP dưới 50 MiB/file** bằng `ros2 bag convert`.
- Đã kiểm tra từng topic: số bản tin, kiểu dữ liệu, thứ tự, timestamp nhận và toàn bộ payload tuần tự hóa đều giống bản gốc. Xem `rosbag_verification.json`.
- ROS đọc trực tiếp thư mục `rosbag/`; không cần ghép các file MCAP.

```bash
# Chạy từ thư mục gốc dự án
source tools/setup_real_env.bash
ros2 bag info data/real_trial_03/rosbag

cd data/real_trial_03
sha256sum -c SHA256SUMS
```

## Kết quả lượt cuối

Run ID: `20261026T160851-9909-3e8e0793`, chu kỳ 1 của mỗi xe.

| Xe | Rộng đo được (cm) | Cao đo được (cm) | Ô giao | Từ xác nhận nhận hàng đến tới ô giao (s) | Từ bắt đầu đến tới ô giao (s) |
| --- | ---: | ---: | --- | ---: | ---: |
| AGV1 | 19,646 | 31,652 | (1,2) | 40,236 | 102,209 |
| AGV2 | 12,256 | 23,899 | (1,1) | 51,533 | 113,744 |

Cả hai xe có sự kiện `delivery_arrived` và `unloaded_confirmed`.
Từ lúc xe đầu tiên bắt đầu đến khi cả hai tới ô giao là **114,604 giây**.
Thời gian tổng mỗi xe bao gồm chờ người thao tác; không coi đó là thời gian chạy thuần túy.

Lượt đầu có Run ID `20261026T154639-7060-82ae0e83`: AGV2 lỗi căn tâm, mở lại giao diện
và tiếp cận lại qua (4,3); AGV1 chờ nhận hàng trong lúc xử lý. Không gộp thời gian lượt
này vào thống kê thời gian vận hành thông thường của lượt cuối. Rosbag hiện có chỉ ghi lượt cuối;
nhật ký JSONL và log `fleet/` còn lưu diễn biến lượt trước.

Đây là kích thước do hệ thống đo, chưa phải kích thước chuẩn đo bằng thước. Chưa có dữ liệu
đối chiếu để tính sai số kích thước hoặc độ chính xác phân loại. Chiều cao AGV2 gần biên dưới
24 cm của cấu hình; cần ghi rõ cơ chế cho phép giá trị sát biên khi sử dụng kết quả trong báo cáo.

Các log `fleet/` được sao chép sau thử nghiệm; một số tiến trình ROS vẫn ghi log sau lúc
rosbag dừng. Cấu hình và mã nguồn hiện tại có thể bao gồm sửa đổi trong quá trình thử;
không suy ra rằng toàn bộ mã nguồn ở thời điểm sao chép giống chính xác thời điểm bắt đầu lượt đầu.
