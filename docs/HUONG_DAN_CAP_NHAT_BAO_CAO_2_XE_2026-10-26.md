# Hướng dẫn cập nhật báo cáo NCKH theo dữ liệu mới

Cập nhật ngày **26/10/2026**. Đây là điểm bắt đầu cho người sửa báo cáo.
Báo cáo Word gốc chưa được chỉnh trực tiếp và không nằm trong bộ dữ liệu này.
Chỉ dùng kết quả **hai xe** ở các đường dẫn bên dưới cho phần đối chiếu mới.

## 1. Đọc các file này trước

1. [Dữ liệu xe thật và phạm vi phép thử](../data/real_trial_03/README.md).
2. [Dữ liệu mô phỏng, ảnh và các giả định](../data/sim_trial_01_20261026/README.md).
3. [Bảng đối chiếu xe thật/mô phỏng](../data/sim_trial_01_20261026/comparison_real_sim.csv).
4. [Danh sách các mục cần sửa trong báo cáo](KE_HOACH_SUA_BAO_CAO_NCKH_2_XE_2026-10-26.md).

Tài liệu kế hoạch còn chứa số liệu và kịch bản lịch sử. Khi có khác biệt, dùng
các bộ dữ liệu `data/real_trial_03/` và `data/sim_trial_01_20261026/` cho lượt đối chiếu mới.
Các kết quả ba xe trong `docs/warehouse_*` thuộc thí nghiệm khác.

## 2. Bố trí và quy trình đang báo cáo

| Xe | Thùng | Rộng × cao chuẩn do người dùng cung cấp | Xuất phát | Quan sát | Lấy hàng | Giao hàng |
| --- | --- | --- | --- | --- | --- | --- |
| AGV1 | 2 | 18,5 × 32 cm | (5,2), Đông | (3,4) | (3,5) | (1,2) |
| AGV2 | 1 | 12,1 × 24 cm | (2,3), Đông | (4,4) | (4,5) | (1,1) |

Ô được ghi theo **(hàng,cột)**, bắt đầu từ 1; khoảng cách giao điểm 60 cm.
Chiều sâu hộp ảo **12 cm là giả định**, chưa có số đo chiều sâu do người dùng cung cấp.
Kích thước chuẩn nêu trên được cung cấp sau buổi chạy; chưa có ảnh đo thước trong bộ dữ liệu.

Đoạn phương pháp có thể dùng:

> Sau lệnh bắt đầu, mỗi xe tới ô quan sát trước ô lấy hàng một bước và quay mặt
> về phía Đông. Hệ thống kết hợp LiDAR và ảnh camera, kiểm tra hình học và độ ổn định,
> rồi chốt trung bình năm ảnh hợp lệ. Chiều cao quyết định ô giao; chiều rộng được
> lưu làm kết quả đo và dùng trong các kiểm tra hình học. Xe tới ô lấy và chờ người
> vận hành đặt đúng kiện, xác nhận đã nhận hàng, rồi mới di chuyển tới ô giao.
> Khi xe dừng ở ô giao, hệ thống ghi mốc đến đích. Người vận hành dỡ kiện và bấm
> xác nhận dỡ; một lệnh quay lại riêng mới bắt đầu chu kỳ tiếp theo.

Các dải chiều cao chuẩn: 24–<30,5 cm → (1,1); 30,5–<37 → (1,2);
37–<43,5 → (1,3); 43,5–50 → (1,4). Giá trị đo sát biên được xử lý bằng dung sai
trong `tools/cargo_live_view.py`: hàm phân loại có giám sát cho phép lệch tối đa
0,5 cm ở hai biên ngoài, còn điều kiện màu xanh ở biên dưới dùng dung sai 0,2 cm.
Vì vậy AGV2 đo **23,899 cm** vẫn được chấp nhận vào ô (1,1); không đổi dải chuẩn thành 23,899–50 cm.

Định vị xe thật trong phiên này có **gợi ý ô và hướng đã được người vận hành xác nhận**.
Hệ khởi tạo gần vị trí khai báo rồi khớp LiDAR quanh ô bằng `--hint-cell` và `--hint-heading`.
Tìm toàn cục từng bị từ chối vì hai vị trí gần đối xứng. Không viết rằng phép thử này
đã chứng minh xe tự tìm đúng vị trí ở mọi nơi khi hoàn toàn không có gợi ý.

AGV2 ở lượt đầu từng lệch tâm 4,3 cm sau khi xoay tại ô quan sát, gồm lệch ngang 3,1 cm.
Nút thử lại được sửa để đi qua (4,3), dừng rồi tiếp cận lại (4,4) hướng Đông;
các mốc phục hồi có trong nhật ký. Giới hạn chấp nhận tại ô đo vẫn là 2 cm.

## 3. Bảng kết quả đo kích thước

Sai số có dấu = giá trị hệ thống đo − kích thước chuẩn người dùng cung cấp.
Đơn vị toàn bảng: **cm**.

| Môi trường | Xe | Rộng chuẩn | Rộng đo | Sai số rộng | Cao chuẩn | Cao đo | Sai số cao |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Xe thật | AGV1 | 18,5 | 19,646 | +1,146 | 32 | 31,652 | −0,348 |
| Xe thật | AGV2 | 12,1 | 12,256 | +0,156 | 24 | 23,899 | −0,101 |
| Mô phỏng | AGV1 | 18,5 | 18,7742 | +0,2742 | 32 | 32,4647 | +0,4647 |
| Mô phỏng | AGV2 | 12,1 | 12,3376 | +0,2376 | 24 | 24,4088 | +0,4088 |

Nguồn xe thật: [CSV lượt cuối](../data/real_trial_03/cargo_times_latest.csv) và
[nhật ký gốc](../data/real_trial_03/cargo_events.jsonl).
Nguồn mô phỏng: [bảng phép đo](../data/sim_trial_01_20261026/measurements.csv).
Không viết sai số xe thật đều dưới 1 cm: riêng sai số rộng AGV1 ở lượt này là **1,146 cm**.

## 4. Thời gian lấy và giao hàng

Đơn vị: **giây**. Bắt đầu = giao diện nhận lệnh bắt đầu; chốt nhận hàng = người vận hành
xác nhận đã chất kiện; tới ô giao = phần mềm ghi nhận xe dừng hợp lệ tại đích.

| Môi trường | Xe | Bắt đầu → chốt nhận hàng | Chờ chất hàng | Chốt nhận → tới ô giao | Bắt đầu → tới ô giao | Bắt đầu → xác nhận dỡ |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Xe thật | AGV1 | 61,972 | 13,164 | 40,236 | 102,209 | 114,691 |
| Xe thật | AGV2 | 62,211 | 2,267 | 51,533 | 113,744 | 115,287 |
| Mô phỏng | AGV1 | 45,028 | 2,106 | 33,575 | 78,603 | 80,669 |
| Mô phỏng | AGV2 | 51,893 | 2,033 | 41,566 | 93,460 | 95,559 |

Thời gian cả đợt tính từ mốc bắt đầu sớm nhất tới mốc tới ô giao muộn nhất:
**xe thật 114,604 s**, **mô phỏng 93,590 s**. Không cộng thời gian hai xe để gọi là thời gian cả đợt.

Lượt xe thật dùng Run ID `20261026T160851-9909-3e8e0793`; lượt mô phỏng dùng
`20261026T162717-12874-27eed460`. Mỗi môi trường có **một lượt cuối cho mỗi xe** trong bảng đối chiếu.
Hai xe thật đều có sự kiện tới ô giao và xác nhận dỡ. Chưa có video thực nghiệm trong
bộ dữ liệu này để người đọc kiểm chứng độc lập thao tác vật lý.

`data/real_trial_03/cargo_times.csv` còn chứa lượt trước có thời gian chờ xử lý lỗi rất dài.
Giữ các lần thử và lỗi đó trong hồ sơ; không lấy trung vị gộp của file này làm thời gian
đại diện cho lượt đối chiếu cuối, cũng không xóa chúng để tuyên bố chưa từng xảy ra lỗi.

## 5. Mô phỏng đã làm được và giới hạn cần ghi

Hai xe chạy cùng `traffic_manager.py`, `fleet_agent.py`, thuật toán đo hàng và hai Nav2 riêng.
Camera/LiDAR và chuyển động là mô hình tổng hợp. Thùng đi theo xe sau xác nhận chất và
được đặt xuống khu giao sau xác nhận dỡ. Có ảnh chụp trực tiếp các giai đoạn và rosbag đầy đủ.

Lượt này dùng định vị map→odom lý tưởng; không thử AMCL hay SLAM trong mô phỏng hai xe.
Mô hình có độ trễ 0,05 s, quán tính 0,12 s; LiDAR 320 tia với nhiễu Gaussian σ=0,01 m.
Không mô phỏng đầy đủ tải trọng, trượt bánh, cơ cấu gắp/dỡ hoặc động lực học va chạm.
Chi tiết và bản chụp mã/cấu hình nằm trong thư mục dữ liệu mô phỏng.

Thời gian chờ thao tác mô phỏng được đặt khoảng 2 s, còn xe thật do người thực hiện.
Chênh lệch thời gian không chứng minh thuật toán nhanh hơn trên mô phỏng hoặc đã dự báo
đúng thời gian xe thật. Khoảng cách tâm mẫu nhỏ nhất 0,600 m và sai số vị trí cuối
0,6419/0,6943 cm chỉ là số liệu của mô hình, không chuyển thành sai số thực nghiệm.

Một lượt mỗi xe chưa đủ để công bố tỷ lệ thành công, độ lặp lại, độ tin cậy hoặc độ chính xác
cho cả bốn lớp hàng. Chưa có phép đo độc lập vị trí/góc xe thật trên sàn trong bộ dữ liệu;
không dùng AMCL làm sự thật để tự chấm sai số của AMCL.

## 6. LiDAR mới xe 1 và các mục phải đồng bộ

Bộ giải mã xe 1 xử lý mẫu gồm một byte cường độ và hai byte khoảng cách, kiểm tra checksum
và ghép vòng quét thành `LaserScan` 320 ô góc. File chính:
[`ydlidar_x2_intensity.h`](../firmware/esp32_agv_base/ydlidar_x2_intensity.h).
Xe 2 vẫn dùng bộ giải mã cũ. Gói UART và bộ giải mã đọc được dữ liệu không tự chứng minh
độ chính xác khoảng cách/góc hay độ ổn định dài hạn của LiDAR.

Sửa thống nhất phần tóm tắt Việt/Anh, thông tin kết quả nghiên cứu, phương pháp đo hàng,
bảng phân loại, mô phỏng hai xe, kết quả thực nghiệm, chú thích hình và kết luận.
Bỏ hoặc ghi nguồn riêng cho số liệu cũ không cùng hộp/cấu hình/lượt chạy.

Ảnh nên dùng:

- [Hai kết quả đo mô phỏng](../data/sim_trial_01_20261026/images/stages/cycle1_agv2_measured.png).
- [Hai xe đã giao/dỡ trong mô phỏng](../data/sim_trial_01_20261026/images/stages/cycle1_both_completed.png).
- [Quỹ đạo hai xe](../data/sim_trial_01_20261026/figures/quy_dao_2_xe.png).
- [Thời gian từng chặng](../data/sim_trial_01_20261026/figures/thoi_gian_lay_giao.png).
- [Sai số đo hộp ảo](../data/sim_trial_01_20261026/figures/sai_so_do_hop_ao.png).

Chú thích ảnh phải ghi rõ **mô phỏng**. Xe thật có ảnh camera liên tục trong rosbag;
bộ dữ liệu hiện không có ảnh chụp rời hay video ngoài hiện trường.

## 7. Đọc và kiểm tra dữ liệu

Người sửa báo cáo có thể mở Markdown, CSV và PNG trực tiếp trên GitHub, không cần chạy ROS.
Hai rosbag đã chia thành các file MCAP nhỏ; ROS đọc trực tiếp thư mục `rosbag/`, không ghép file.
`SHA256SUMS` và `rosbag_verification.json` trong từng bộ dữ liệu dùng kiểm tra tính toàn vẹn.
Muốn sửa file trên GitHub cần quyền cộng tác vào repository hoặc làm qua fork/pull request.
