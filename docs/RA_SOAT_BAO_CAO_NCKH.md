# Rà soát bản chỉnh_sửa.docx — 18/10/2026

Nguồn: /home/tai/Downloads/chỉnh_sửa.docx, mã và cấu hình trong Robot_kho_clone, các hình nhúng trong DOCX, các tệp minh chứng tìm thấy trên máy. Thiếu dữ liệu gốc không đồng nghĩa với bịa đặt. Các kết luận dưới đây phân biệt lỗi đối chiếu được và số chưa thể kiểm chứng độc lập.

## Mâu thuẫn và lỗi cần sửa

1. **Phân loại hàng không khớp chương trình đang chạy.** §2.7.4 và Bảng 2.9 chia loại theo cả rộng và cao. Theo bảng đó, hộp nâu 17,9 × 31,1 cm và hộp xanh 12,4 × 24,3 cm của Bảng 3.7 đều thuộc loại “cao”, cùng về (1,3). Bảng 3.7 lại ghi về (1,2) và (1,1). Kết quả mới đúng với nhánh chia **chiều cao 24–50 cm thành bốn khoảng đều** trong src/my_bot_bringup/config/cargo.yaml và tools/cargo_live_view.py. Cần viết lại mô tả phương pháp và Bảng 2.9.

2. **Hiệu chuẩn camera trong báo cáo đã cũ.** Bảng 2.8 ghi cả hai xe x=96,7 mm; yaw/pitch lần lượt −2,49°/−2,29° và +2,92°/−1,09°. Cấu hình hiện tại ghi xe 1 x=96,7 mm, yaw −2,14°, pitch +1,97°; xe 2 x=139 mm, yaw +2,71°, pitch +0,74° (camera_agv1.yaml, camera_agv2.yaml). §2.2.1 và công thức §2.7.3 cũng áp x=96,7 mm cho cả hai. Cần ghi rõ ngày/lần hiệu chuẩn và tham số riêng từng xe.

3. **Vận tốc tính sai và trần phần mềm từng vượt khả năng động cơ.** Chủ dự án xác nhận động cơ tối đa **60 vòng/phút**. Với bán kính lăn 32,81 mm của Bảng 2.2, trần tốc độ tịnh tiến lý thuyết là 0,206 m/s (theo đường kính hình học 64,5 mm: 0,203 m/s), trước khi tính sụt áp và tải. Vì thế câu “~0,40 m/s (động cơ 60 vòng/phút)” ở Bảng 2.2 là sai; 0,40 m/s đòi gần 120 vòng/phút. §2.3.2 mô tả cắt lệnh ở 60 vòng/phút; mã firmware đã được chỉnh lại thành MAX_WHEEL_RPM=60 theo yêu cầu, nhưng **Bảng 2.5 vẫn ghi 120** và phải sửa trước khi nộp. Tốc độ Nav2 danh định 0,15 m/s được giữ nguyên để bảo toàn hành vi chạy thường ngày; velocity smoother vẫn có trần lệnh 0,30 m/s, còn trần mỗi bánh trong firmware sẽ chặn/scaling lệnh vượt 60 RPM. Cần đo tốc độ thực trên sàn sau khi nạp firmware; không lấy tốc độ lý thuyết ~0,20 m/s làm số liệu đo thực nghiệm.

4. **Mô tả nguồn điện mâu thuẫn.** Bảng 2.3–2.4 và §2.2.3 ghi ESP32/BNO055/encoder lấy 5 V từ L298N. docs/SA_BAN_VA_TAI_LIEU.md hướng dẫn ESP32 nhận 5 V từ mạch hạ áp, BNO055 và encoder nhận 3V3, và không nuôi ESP32 từ chân +5 V của L298N. Phải đối chiếu dây thực trên cả hai xe trước khi sửa sơ đồ; mã không xác nhận được cách nối thực.

5. **Video 1 có hai bộ tọa độ khác nhau.** Bảng 3.7 ghi xe 1 (4,4)→(5,1), xe 2 (2,4)→(1,1), mỗi xe bốn cạnh. Đoạn sau Hình 3.7 lại ghi xe 1 (2,2)→(2,4), xe 2 (4,4)→(4,2), mỗi xe hai cạnh. Chỉ giữ bộ tọa độ của video/log thực.

6. **Chi phí rẽ và tiêu chí góc chưa cập nhật.** §2.6.1 ghi rẽ cộng 0,3; traffic_manager.py cộng 0,3 chi phí bắt đầu và 0,3 cho rẽ 90°, tổng 0,6. Báo cáo dùng ngưỡng hoàn tất góc 3° và kết quả ≤2,8°; grid_real.yaml hiện dùng rotate.done_deg=0.9 (<1°). Kết quả cũ không chứng minh được mục tiêu mới; ghi phiên bản cấu hình/ngày của phép thử.

## Số liệu chưa đủ bằng chứng kiểm lại

1. **Sai số định vị và dừng đích.** Bảng 3.2 nêu AMCL trung vị 3,6 cm, p90 6,1 cm; Bảng 3.3–3.5 nêu đích 0,4–1,6 cm, 9/9 lần xoay ≤2,8°, hai xe cách nhau ít nhất 59–60 cm. Chưa tìm thấy rosbag/CSV/log gốc tương ứng. eval_bag_localization.py gọi “tư thế thật” là tư thế làm chính ảnh LiDAR khớp cùng bản đồ quanh AMCL; đó không phải ground truth đo độc lập. Nên gọi là chênh lệch AMCL–scan matching. Để khẳng định sai số ngoài sàn ≤2 cm, cần phép đo tâm xe so với mốc lưới bằng thước hoặc hệ đo độc lập.

2. **Sai số đo hộp.** Câu “sai lệch 0–10 mm” chỉ đúng cho hai dòng sau hiệu chuẩn của Bảng 3.6, không đại diện cho toàn bộ năm lần đo. Chính báo cáo nói pitch được ước lượng từ hai hộp thử, nên đo lại chính hai hộp đó chưa phải kiểm định độc lập. Kích thước tham chiếu hộp xanh cũng không thống nhất: §2.7.4 ghi 12,1 × 24,0 cm; Bảng 3.6 ghi 12,0 × 24,0 cm; ghi chú cấu hình hiện tại là 12,0 × 23,9 cm. Cần ảnh thước, kết quả 5 ảnh mỗi lần và thử thêm hộp chưa dùng để hiệu chuẩn.

3. **Khoảng cách ở Hình 3.6.** Giao diện hiển thị DIST 102,3 cm và 98,2 cm, trong khi bố trí mô tả mặt hộp cách tâm ô lấy hàng 85 cm; DIST lại là từ mặt kính camera tới hộp. Nếu xe dừng đúng tâm ô và camera nằm trước tâm xe, DIST dự kiến nhỏ hơn 85 cm. Có thể đây là một bố trí khác, nhưng phải ghi và kiểm bằng thước của đúng lần chụp, nhất là xe 2 từng đo thực từ kính tới hộp khoảng 78–79 cm.

4. **Sáu kịch bản/video.** DOCX có ảnh ghép xe thật và RViz, chứng minh có các thời điểm chạy thử. Ảnh tĩnh không chứng minh toàn tuyến, không va chạm, số lần thử, thời lượng video hay mốc phản ứng 8,3/10,5/11,0/13,0/15,0 s và 2,5 s. Trên máy có các screencast thô ngày 16/10, nhưng chưa tìm thấy đủ sáu video hoàn chỉnh hoặc bag/log gắn với Bảng 3.7. docs/KICH_BAN_VIDEO_NCKH.md là **kịch bản quay**, không phải số liệu kết quả. Cần sáu clip nguyên gốc với timestamp và nhật ký cả những lần thử thất bại. Nếu chưa có, chuyển Bảng 3.7 thành “kịch bản dự kiến” và bỏ khẳng định “đều đạt”.

5. **Giao hàng.** Hình 3.6 chỉ hiển thị “ĐÃ NHẬN HÀNG”, không hiển thị “ĐÃ GIAO HÀNG”. Hình 3.7 cho thấy xe ở ô đích trên RViz, nhưng không thấy hộp trên xe; hai hộp còn xuất hiện trên sàn trong ảnh. Hai hình này chưa chứng minh hàng được chở và giao vật lý. Nếu chỉ thử logic điều phối, nên viết “sau xác nhận, xe đến ô phân loại”; để ghi “giao hàng thành công”, cần video hàng được đặt lên xe, mang theo, và thông báo/sự kiện delivered khi tới ô giao.

6. **Các con số khác cần phụ lục gốc.** Ví dụ dao động góc LiDAR 13,8°→3,2°, lỗi điểm xa 25,9→9,8 mm, bỏ 4/5 gói vẫn tính 2,000 m, chín phép đo mô phỏng sai số ≤3,5 mm. Chưa thấy bảng dữ liệu đầu vào và cách tính trong tài liệu được cung cấp. Không đủ căn cứ gọi là bịa; cũng chưa đủ để người phản biện tái tính.

## Phần mô phỏng

Nhận xét của người hướng dẫn ở §2.8.1 là hợp lý: bản gốc trộn mô phỏng một xe trong phòng 10 × 10 m/ô 2 m, hai xe động học trên sa bàn thật và đo hàng, nhưng thiếu kết quả cho từng tình huống. Phần bổ sung docs/PHAN_MO_PHONG_NCKH.md ghi bốn lần chạy hai xe trên lưới 0,60 m, có vết dữ liệu trong docs/figures/mo_phong_nckh/. Mỗi ca chỉ một lần, vật cản “xuất hiện rồi biến mất” chứ chưa mô phỏng người đi ngang. Không dùng bốn ca này để chứng minh tám ca mô phỏng hoặc sáu video xe thật của Chương 3. Đã sửa đánh số hình bổ sung thành 2.21–2.24 để không đè Hình 2.20 của báo cáo.

**Kết luận:** Chưa thể kết luận tác giả bịa số liệu. Có lỗi xác định và nhiều kết quả thiếu dữ liệu gốc. Không nên công bố sáu ca “đều đạt”, “giao hàng thành công” hoặc sai số vật lý cấp centimet như kết quả đã được chứng minh cho đến khi đối chiếu bản ghi và phép đo độc lập.
