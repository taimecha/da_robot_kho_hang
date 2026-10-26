# Mô phỏng hai xe, hai thùng theo số đo xe thật — 26/10/2026

Đã chạy một lượt đầy đủ cho mỗi xe trên ROS domain **87**, tự thao tác giao diện,
chụp ảnh và ghi dữ liệu. Đây là dữ liệu mô phỏng, tách riêng dữ liệu xe thật ở
[`../real_trial_03/`](../real_trial_03/README.md). Bộ dữ liệu được quản lý phiên bản cùng mã nguồn.

## Bố trí và kích thước đầu vào

Giữ cách gán thùng như lượt xe thật: AGV1 chở thùng lớn, AGV2 chở thùng nhỏ.

| Xe | Thùng | Rộng × cao theo người dùng (cm) | Xuất phát | Ô quan sát | Ô lấy | Ô giao |
| --- | --- | --- | --- | --- | --- | --- |
| AGV1 | 2 | 18,5 × 32 | (5,2), Đông | (3,4) | (3,5) | (1,2) |
| AGV2 | 1 | 12,1 × 24 | (2,3), Đông | (4,4) | (4,5) | (1,1) |

**Chiều sâu 12 cm là giả định**, vì chưa có số đo chiều sâu do người dùng cung cấp.
Mặt trước thùng đặt phía Đông, cách tâm ô lấy 35 cm. Lưới có khoảng cách 60 cm.

## Kết quả ghi được

Run ID: `20261026T162717-12874-27eed460`. Mỗi xe có đủ bảy mốc từ `started`
đến `unloaded_confirmed`. Thời gian dưới đây tính bằng timestamp monotonic trong nhật ký.

| Xe | Rộng đo được (cm) | Cao đo được (cm) | Sai số rộng (cm) | Sai số cao (cm) | Bắt đầu → tới ô giao (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| AGV1 | 18,7742 | 32,4647 | +0,2742 | +0,4647 | 78,603 |
| AGV2 | 12,3376 | 24,4088 | +0,2376 | +0,4088 | 93,460 |

Từ lúc xe đầu tiên bắt đầu đến khi cả hai tới ô giao: **93,590 giây**.
Thời gian từ xác nhận chất đến tới ô giao: AGV1 **33,575 giây**, AGV2 **41,566 giây**.
Chờ xác nhận chất hàng được mô phỏng khoảng 2 giây; dỡ hàng cũng được tự xác nhận sau khoảng 2 giây.
Thao tác tự động được ghi trong `sim_operator_clicks.jsonl`.

Khoảng cách tâm nhỏ nhất suy ra từ các mẫu trạng thái: **0,600 m**; đây là số đo mẫu
của mô hình động học, không phải kết quả thử va chạm vật lý. Sai số vị trí trong bản chụp
trạng thái cuối sau khi dừng: AGV1 **0,6419 cm**, AGV2 **0,6943 cm**.

## Ảnh và biểu đồ

- [Ảnh cả hai xe đã giao và dỡ hàng](images/stages/cycle1_both_completed.png).
- [Ảnh hai kết quả đo](images/stages/cycle1_agv2_measured.png).
- [Ảnh đang đo AGV1](images/stages/cycle1_agv1_measurement_preview.png).
- [Ảnh đang đo AGV2](images/stages/cycle1_agv2_measurement_preview.png).
- [Quỹ đạo hai xe](figures/quy_dao_2_xe.png).
- [Thời gian các chặng](figures/thoi_gian_lay_giao.png).
- [Sai số đo so với thùng ảo](figures/sai_so_do_hop_ao.png).
- [Khoảng cách hai xe](figures/khoang_cach_2_xe.png).

Có **60 ảnh PNG** trong `images/`, gồm **15 ảnh toàn cảnh theo các mốc** trong
`images/stages/` và các ảnh camera/giao diện tự lưu bằng chức năng S của giao diện.
Ảnh được chụp trực tiếp từ cửa sổ chạy mô phỏng; biểu đồ được tính từ nhật ký đã ghi.
RViz đã được kiểm tra có đủ **25 nhãn giao điểm**, gồm `(1,3)`.

## Dữ liệu để phân tích

| File/thư mục | Nội dung |
| --- | --- |
| `scenario.json` | Kích thước thật đầu vào, cách gán xe, vị trí và các giả định |
| `measurements.csv` | Kích thước đo, sai số so với đầu vào và vị trí cuối |
| `cargo_times.csv` | Thời gian quan sát, đo, lấy, chờ chất, giao và xác nhận dỡ |
| `comparison_real_sim.csv` | Số liệu mô phỏng và lượt cuối của xe thật, giữ nhãn môi trường riêng |
| `cargo_events.jsonl` | Nhật ký gốc 14 sự kiện của hai xe |
| `fleet_trace.jsonl` | Mẫu trạng thái/quỹ đạo và lệnh tuyến |
| `final_state.json` | Trạng thái cuối, vị trí thùng đã dỡ, danh sách nhãn RViz |
| `rosbag/` | 61.095 bản tin, 16 topic, nén Zstandard trong 5 file MCAP dưới 50 MiB/file |
| `rosbag_verification.json` | Đối chiếu toàn bộ payload, timestamp, thứ tự, kiểu và số bản tin trước/sau nén |
| `trial_validation.json` | Kiểm tra đủ mốc, đủ ảnh, hai thùng đã giao và nhãn lưới |
| `source/`, `config/`, `code_sha256.txt` | Bản chụp mã nguồn/cấu hình và mã kiểm tra của lượt chạy |
| `logs/`, `fleet/` | Log các tiến trình và cấu hình Nav2 đã sinh |
| `real_process_isolation_check.json` | Xác nhận các tiến trình xe thật trên domain 0 vẫn tồn tại sau khi tắt mô phỏng |

Rosbag ghi từ **16:27:17,867** đến **16:31:03,833** ngày 26/10/2026, UTC+7;
có cả thời gian chờ chuẩn bị trước/sau lượt chạy. Thời lượng file ghi **225,966 giây**
khác thời gian hoàn thành nhiệm vụ. Bản lưu chưa nén được thay bằng bản nén sau khi
kiểm tra khớp toàn bộ **61.095 bản tin**.

`logs/operator_setup.log` và `operator_clicks_setup.jsonl` ghi một lần hiệu chỉnh vị trí
nhấp nút trước khi nhiệm vụ bắt đầu: giao diện Qt có thanh công cụ nên hai lần nhấp đầu
chưa kích hoạt xe; chưa có sự kiện `started` ở bước chuẩn bị đó. Nhật ký nhiệm vụ được giữ nguyên.

```bash
# Từ thư mục gốc dự án
source tools/setup_real_env.bash
ros2 bag info data/sim_trial_01_20261026/rosbag

python3 tools/analyze_cargo_timing.py \
  data/sim_trial_01_20261026/cargo_events.jsonl \
  --environment sim --csv /tmp/sim_trial_01_times.csv

python3 tools/plot_cargo_two_car_run.py \
  data/sim_trial_01_20261026/cargo_events.jsonl \
  data/sim_trial_01_20261026/fleet_trace.jsonl /tmp/sim_trial_01_figures \
  --scenario data/sim_trial_01_20261026/scenario.json

cd data/sim_trial_01_20261026
sha256sum -c SHA256SUMS
```

## Phạm vi mô hình và cách so sánh

Hai xe dùng `traffic_manager.py`, `fleet_agent.py` và hai Nav2 riêng. Camera và LiDAR
là dữ liệu tổng hợp; mô hình chuyển động nhận vận tốc, có quán tính bậc nhất và độ trễ.
Thùng được gắn vào xe sau `loaded_confirmed`, di chuyển theo xe, rồi đặt xuống cách
ô giao 32 cm về phía Nam lưới sau `unloaded_confirmed`. Đây là quy ước hiển thị dỡ hàng.

Định vị của lượt này là **map→odom lý tưởng**, không chạy AMCL hoặc dựng bản đồ từ đầu.
Không mô phỏng đầy đủ tải trọng, trượt bánh, va chạm, cơ cấu gắp/dỡ hay ESP32 thật.
Các thông số: trễ 0,05 s; quán tính 0,12 s; vùng chết 0,012 m/s và 0,10 rad/s;
LiDAR 320 tia, nhiễu khoảng cách Gaussian σ=0,01 m; camera ảo 2 ảnh/s, seed ảnh bằng 1.
Nhiễu LiDAR dùng seed ngẫu nhiên từ hệ thống; muốn xem lại đúng dữ liệu đã chạy thì dùng rosbag.

`comparison_real_sim.csv` dùng số đo chuẩn người dùng vừa cung cấp cho hai thùng và
kích thước hệ thống đo được trong lượt xe thật cuối. Thời gian xe thật có thao tác người
vận hành; thời gian mô phỏng dùng xác nhận tự động. Không dùng chênh lệch thời gian này
để kết luận xe thật nhanh/chậm hơn hoặc mô hình đã khớp thực tế. Một lượt mỗi xe chưa đủ
để tính độ lặp lại hay tỷ lệ thành công thống kê.

Các tiến trình mô phỏng đã được tắt sau khi lưu dữ liệu.
