# Kết quả đo đáp ứng điều khiển tốc độ bánh — AGV1, 18/10/2026

## Hình và dữ liệu gốc

- [Hình sau tinh chỉnh PNG](figures/pid_real_agv1/agv1_pid_20261018_195417.png) và [PDF vector](figures/pid_real_agv1/agv1_pid_20261018_195417.pdf) để chèn vào báo cáo; dữ liệu [encoder](figures/pid_real_agv1/agv1_pid_20261018_195417_joint.csv), [PWM](figures/pid_real_agv1/agv1_pid_20261018_195417_diag.csv), [tổng hợp](figures/pid_real_agv1/agv1_pid_20261018_195417_summary.json).
- [Hình PNG độ phân giải cao](figures/pid_real_agv1/agv1_pid_20261018_194245.png) để chèn vào báo cáo.
- [Hình PDF vector](figures/pid_real_agv1/agv1_pid_20261018_194245.pdf).
- [430 mẫu tốc độ/góc bánh từ encoder](figures/pid_real_agv1/agv1_pid_20261018_194245_joint.csv), [9 mẫu PWM/chẩn đoán](figures/pid_real_agv1/agv1_pid_20261018_194245_diag.csv), [bảng tổng hợp JSON](figures/pid_real_agv1/agv1_pid_20261018_194245_summary.json).
- Công cụ có thể chạy lại: [tools/pid_field_test.py](../tools/pid_field_test.py).

## Điều kiện thử

Xe 1 chạy thẳng trên sàn, bắt đầu từ trạng thái đứng yên. Chương trình gửi trực tiếp vận tốc tuyến tính 0,12 m/s, vận tốc góc 0 rad/s trong 5,0 s rồi gửi lệnh dừng. Không dùng Nav2 trong lần thử. Với bán kính bánh **0,03225 m của firmware**, mức đặt quy đổi cho mỗi bánh là **35,53 vòng/phút**. Đây là đáp ứng của **bộ điều khiển tốc độ gồm PID, feedforward và giới hạn tăng tốc**, không phải đáp ứng của riêng ba hệ số PID. Tốc độ thực đo được tính từ encoder, đã qua lọc thấp trong firmware và phát khoảng 50 Hz trên /agv1/joint_states. PWM lấy từ /agv1/diag khoảng 1 Hz.

Giữa hai lần thử, hệ số AGV1 được đổi từ **Kp = 0,30; Ki = 0,04** thành **Kp = 1,20; Ki = 0,08**; Kf = 3,00 và Kd = 0,01 giữ nguyên. Firmware đã nạp và kiểm tra lại trên xe 1. Xe 2 không đổi hệ số điều khiển. Trước khi thử trên sàn, xe 1 được thử trên giá kê với hai bánh hở sàn; dữ liệu đó không dùng để kết luận hiệu năng khi chạy trên sàn.

| Chỉ tiêu | Trước chỉnh | Sau chỉnh |
|---|---:|---:|
| Thời gian RPM trung bình hai bánh đạt 90% mức đặt | 2,90 s | 1,74 s |
| RPM trung bình hai bánh trong 2 s cuối | ~33,9 | 33,95 |
| Sai lệch trung bình trong 2 s cuối | ~−4,5% | −4,46% |
| RPM trung bình hai bánh lớn nhất khi còn lệnh chạy | 35,35 | 35,46 |
| Thời gian từ lệnh dừng đến dưới 10% mức đặt | 0,46 s | 0,44 s |
| Quãng đường suy từ encoder | 0,494 m | 0,551 m |

Mức cải thiện chính là **thời gian tăng tốc ngắn hơn khoảng 1,16 s (40%)**, không phải độ chính xác tĩnh. Quãng đường hai lần là số tính từ encoder, không phải đo bằng thước trên sàn. Đây là một lần chạy cho mỗi cấu hình; chưa đánh giá độ lặp lại, trượt bánh, thay đổi điện áp pin hoặc quãng đường thực.

| Chỉ tiêu | Bánh trái | Bánh phải |
|---|---:|---:|
| RPM đặt từ lệnh vận tốc | 35,53 | 35,53 |
| Trung bình RPM trong 2 s cuối khi chạy | 33,97 | 33,88 |
| Sai lệch trung bình so với mức đặt | −4,39% | −4,65% |
| Thời gian từ cấp lệnh tới 90% mức đặt | 2,90 s | 2,90 s |
| Thời gian từ lệnh dừng tới dưới 10% mức đặt | 0,45 s | 0,46 s |
| PWM ở mẫu cuối trước lệnh dừng | 220/255 | 219/255 |

Quãng đường tính từ **góc encoder và bán kính lăn 0,03281 m của odometry** là **0,494 m** kể từ trước khi chạy đến khi dừng hẳn. Người vận hành chưa đo lại bằng thước, nên không gọi đây là quãng đường thực đo độc lập. Khoảng cách này thấp hơn tích 0,12 m/s × 5 s = 0,60 m vì bánh mất gần 3 s mới đạt 90% tốc độ đặt; không diễn giải chênh lệch 0,106 m là sai số định vị.

## Chú thích hình và đoạn văn có thể đưa vào báo cáo

**Hình 3.8.** Đáp ứng tốc độ hai bánh của AGV1 khi chạy thẳng trên sàn. Xe nhận lệnh vận tốc 0,12 m/s trong 5,0 s; đường đứt biểu diễn tốc độ bánh quy đổi từ lệnh, các đường xanh dương và cam biểu diễn tốc độ từng bánh suy ra từ encoder (~50 Hz), còn đường xanh lá là trung bình số học RPM của hai bánh. Đại lượng trung bình này biểu diễn tốc độ tịnh tiến của xe quy đổi về RPM bánh khi chạy thẳng, không phải một cảm biến RPM riêng trên thân xe. Các điểm ở đồ thị dưới là PWM lấy từ gói chẩn đoán (~1 Hz). Hai vạch dọc đánh dấu lúc bắt đầu và ngừng lệnh chạy.

**Hình 3.9.** Đáp ứng tốc độ AGV1 sau khi tăng Kp và Ki; cách hiển thị và điều kiện lệnh giống Hình 3.8. RPM trung bình hai bánh đạt 90% mức đặt sau 1,74 s, giảm từ 2,90 s trong lần thử trước. Đường RPM trung bình là giá trị tính từ hai encoder, không phải phép đo tốc độ độc lập của thân xe.

*Đoạn trình bày kết quả sau tinh chỉnh:* “Với cùng lệnh chạy thẳng 0,12 m/s trong 5 s, sau khi tăng Kp từ 0,30 lên 1,20 và Ki từ 0,04 lên 0,08 trên AGV1, thời gian RPM trung bình hai bánh đạt 90% mức đặt giảm từ 2,90 s xuống 1,74 s. Trong 2 s cuối, RPM trung bình hai bánh là 33,95 vòng/phút so với mức đặt 35,53 vòng/phút, sai lệch −4,46%. Không quan sát thấy tốc độ trung bình vượt mức đặt trong hai lần đo. Kết quả minh họa cải thiện đáp ứng quá độ trên một lần thử; cần lặp lại nhiều lần và đo quãng đường độc lập để đánh giá độ tin cậy và sai số di chuyển.”

*Đoạn trình bày:* “Để đánh giá bộ điều khiển tốc độ trên xe thật, nhóm cho AGV1 chạy thẳng với vận tốc đặt 0,12 m/s trong 5 s, tương đương 35,53 vòng/phút cho mỗi bánh. Tốc độ bánh từ encoder đạt 90% mức đặt sau khoảng 2,90 s. Trong 2 s cuối, tốc độ trung bình bánh trái và phải lần lượt là 33,97 và 33,88 vòng/phút, thấp hơn mức đặt 4,39% và 4,65%. Sau lệnh dừng, hai bánh giảm xuống dưới 10% mức đặt trong khoảng 0,45–0,46 s. Đáp ứng tăng tốc còn chậm; kết quả này là một lần thử trên AGV1, chưa chứng minh độ lặp lại hay sai số quãng đường ngoài sàn.”

Để dùng kết quả này như đánh giá thực nghiệm đầy đủ, nên lặp cùng điều kiện nhiều lần và đo thêm quãng đường thực bằng thước hoặc mốc trên sàn.
