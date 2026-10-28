# Đề xuất kiểm chứng mục tiêu 95% và phản hồi 2 giây

Đây là phương án đo để thống nhất trước khi thử, không phải kết quả đã đạt.

## 1. ArUco là gì?

ArUco là mã vuông có viền đen và các ô đen trắng bên trong. Camera nhận ra mã để đọc ID; khi có kích thước mã và hiệu chuẩn camera, có thể ước lượng vị trí tương đối. Xem [tài liệu OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

Luồng lấy–giao hiện tại đo thùng bằng LiDAR–camera và phân ô giao theo chiều cao, không dùng giải mã ArUco. Vì vậy không gọi thời gian đo thùng là thời gian giải mã ArUco.

Thuyết minh đính kèm, mục 12, nêu độ chính xác 95% và phản hồi không quá 2 giây nhưng không định nghĩa rõ mẫu số của độ chính xác hoặc hai mốc đo thời gian. Cần thống nhất cách đánh giá với người hướng dẫn trước khi dùng làm tiêu chí nghiệm thu.

## 2. Kiểm chứng mức 95%

**Trước hết chọn đúng đối tượng đánh giá:**

| Chỉ tiêu | Một lượt được tính đúng khi |
|---|---|
| Phân loại | Dải chiều cao và ô giao do hệ thống chọn khớp với chiều cao đo bằng thước và bảng phân loại đã thống nhất. |
| Hoàn thành nhiệm vụ | Phân loại đúng, tới đúng ô giao, tư thế cuối nằm trong ngưỡng vị trí/hướng đã thống nhất; không va chạm hoặc cần can thiệp sửa lỗi. Thao tác đặt/dỡ hàng theo quy trình được phép. |

Nếu mục tiêu 95% áp dụng cho toàn nhiệm vụ, phải dùng hàng thứ hai; tỷ lệ phân loại đúng riêng không thay thế tỷ lệ hoàn thành nhiệm vụ.

**Tỷ lệ đúng = số lượt đúng / tổng số lượt thử × 100%.**

Ví dụ phương án: 120 lượt, gồm 2 xe × 4 dải chiều cao × 15 lượt. Có 114 lượt đúng thì tỷ lệ quan sát là 95%. Thử các kiện ở nhiều độ cao trong mỗi dải, thêm các trường hợp gần ranh giới; dùng quy tắc ranh giới và dung sai cố định trước khi thử. Không chỉ lặp lại đúng hai kiện hiện có.

Ghi mọi lượt sai, mất kết quả và quá hạn; không bỏ lượt lỗi rồi chỉ đếm lượt thử lại thành công. Năm ảnh dùng để lấy trung bình trong một lượt vẫn chỉ là **một phép đo**, không phải năm lượt độc lập. Nếu thử lại sau lỗi, ghi liên kết với lượt lỗi và giữ cả hai trong dữ liệu.

Tỷ lệ 95% trong mẫu không đồng nghĩa với bảo đảm tỷ lệ thật của hệ thống ≥ 95%, cũng không phải “độ tin cậy 95%”. Nếu cần kết luận thống kê đó, phải thiết kế cỡ mẫu và khoảng tin cậy riêng. Dữ liệu hai xe hiện có chỉ mô tả hai lượt lấy–giao đã ghi.

## 3. Phản hồi cái gì, đo từ đâu đến đâu?

Với chức năng đo–phân loại hiện tại, đề xuất hiểu phản hồi là **hệ thống trả ra kết quả kích thước, lớp hàng và ô giao** sau khi nhận dữ liệu đầu vào để đo.

**Mốc bắt đầu:** ảnh đầu vào hợp lệ đầu tiên của lượt đo, sau khi xe ở tư thế quan sát đạt điều kiện.

**Mốc kết thúc:** hệ thống chốt kết quả trung bình đủ năm ảnh và xác định ô giao (`measured`).

**Độ trễ = thời điểm kết thúc − thời điểm bắt đầu.** Phải tính cả thời gian thu đủ năm ảnh, đồng bộ dữ liệu LiDAR cần cho phép đo và xử lý. Điều kiện ảnh hợp lệ phải được quy định trước; ghi riêng lượt không thu được ảnh hợp lệ hoặc không ra kết quả để tránh loại mất lỗi khỏi đánh giá.

Thời gian từ lúc tới ô quan sát đến lúc có kết quả bao gồm thêm thời gian chờ điều kiện đo. Thời gian từ lúc phát lệnh đến lúc xe chạy, hoặc từ khi thấy vật cản đến khi xe giảm tốc, là các chỉ tiêu phản hồi khác. Chỉ tiêu nào dùng để đối chiếu mục tiêu 2 giây phải được thống nhất trước.

**Hai giây không phải thời gian xe vận chuyển xong kiện hàng.** Không dùng tổng thời gian tới ô giao 102 s/114 s để đánh giá độ trễ xử lý ảnh.

## 4. Cách ghi để kiểm tra được

- Mỗi lượt lưu: mã lượt, xe, kích thước chuẩn, lớp/ô giao chuẩn, kết quả đo, lớp/ô giao chọn, kết quả đúng/sai, mốc bắt đầu, mốc kết thúc, độ trễ và lý do lỗi.
- Đo trên cùng đồng hồ đơn điệu của máy xử lý cho khoảng nhận ảnh → chốt kết quả. Muốn đo từ lúc camera thực sự chụp, phải bổ sung dấu thời gian chụp hoặc phép đo đối chiếu có đồng bộ. Dấu thời gian ROS hiện được gắn lúc phát ảnh, không chứng minh được thời điểm phơi sáng của ESP32.
- Lấy lại thời điểm ngay sau khi tính xong kết quả để làm mốc kết thúc. Sự kiện `measured` hiện nhận biến `now` lấy trước khi xử lý lượt ảnh, nên mốc này cần được chỉnh khi bổ sung công cụ đo độ trễ; không dùng nguyên nhật ký đó để xác nhận giới hạn 2 s.
- Giữ độ trễ gốc ở độ chính xác mili giây để xét ngưỡng. Báo cáo có thể trình bày số giây làm tròn, nhưng 2,4 s vẫn vượt 2 s dù hiển thị làm tròn thành 2 s.
- Báo cáo ngắn: số lượt có kết quả trong 2 s / tổng lượt, số lượt lỗi và độ trễ lớn nhất. Nếu yêu cầu “không quá 2 giây” cho mọi lượt thì độ trễ lớn nhất của các lượt phải ≤ 2 s, đồng thời nêu rõ các lượt không ra kết quả.

## 5. Dữ liệu đang có nói được gì?

Nhật ký đang có khoảng 3 s từ sự kiện tới ô quan sát (`observation_arrived`) đến chốt phép đo (`measured`). Khoảng này khác với ảnh đầu vào → kết quả, nên không đủ kết luận đạt hay không đạt chỉ tiêu đề xuất trên. Mã nguồn dùng năm ảnh hợp lệ để lấy trung bình; cần bổ sung mốc ảnh bắt đầu gắn với đúng lượt đo trước khi kiểm chứng.

Đối chiếu mã nguồn: [luồng đo hàng](../tools/cargo_live_view.py), [nguồn ảnh ESP32](../tools/esp32_cam_node.py). Không thay đổi mã điều khiển hoặc chạy xe khi soạn hướng dẫn này.
