# Chạy hai xe đến hộp và xem số đo trực tiếp

Hai hộp đặt phía Đông ô lấy hàng theo bố trí quay hiện tại: **hộp nâu** ở
(hàng 3, cột 5) dành cho **xe 1**; **hộp xanh** ở (hàng 4, cột 5) dành cho
**xe 2**. Cả hai xe quay **Đông**.
Hàng 1 ở dưới cùng, cột 1 ở bên trái
trên RViz. Bật nguồn hai xe và hai ESP32-CAM; giữ xe đứng yên khoảng 2 giây
sau lúc bật nguồn.

Mở các terminal riêng trong VS Code. Ở terminal 1, ví dụ dưới dùng xe 1 ở
(3,5), xe 2 ở (4,5), cùng quay Đông. Nếu đã bế xe sang
chỗ khác, thay `AGV1`, `AGV2` bằng **hàng cột góc hướng** thực tế trước khi chạy.
Chờ `Da bat` rồi mới chạy dòng định vị. Chỉ giao đích khi **cả hai** xe có dòng
`da gui cho AMCL`, và vị trí hai xe trên RViz khớp vị trí thực:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
AGV1="3 5 0" AGV2="4 5 0" tools/fleet_real.sh up
python3 tools/fleet_localize.py
```

Terminal 2, giữ bộ điều phối chạy; chỉ nhận đích qua `/fleet/goal`:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
python3 traffic_manager.py agv1 agv2 --no-stdin
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
python3 tools/cargo_live_view.py --agv1-pickup 3,5 --agv2-pickup 4,5
```

Chạy giao diện trực tiếp trong terminal, không thêm `setsid` hoặc `&`.
Giao diện không tự gửi đích lúc mở. Bấm **BAT DAU** dưới ảnh từng xe:

1. Xe 1 tới **(3,4) Đông**, xe 2 tới **(4,4) Đông** — ô quan sát trước ô lấy một bước.
2. Khi dừng và đúng hướng, camera/LiDAR chốt trung bình **5 ảnh hợp lệ**.
   Nếu phân loại được, xe tự tới ô lấy tương ứng **(3,5)** hoặc **(4,5)**.
   Nếu chưa đo được hoặc chiều cao ngoài dải, xe giữ nguyên ở ô quan sát; bấm **R** để đo lại.
3. Chờ xe dừng đúng ô lấy, đặt đúng món đã đo lên xe, rồi bấm **DA NHAN HANG**.
   Xe chỉ đi giao sau xác nhận này; không thay món khác giữa lúc đo và xác nhận.
4. Tới ô giao, giao diện hiện **DA GIAO HANG THANH CONG** và phát sự kiện
   `delivered` trên `/fleet/cargo_result`. Giao diện **giữ mở**.
5. Dỡ hàng, đặt món tiếp theo tại khu lấy, rồi bấm **DA DO HANG - QUAY LAI LAY TIEP**.
   Xe quay về ô quan sát; số đo và loại cũ được xóa để đo lại món mới.

Ảnh camera cập nhật trực tiếp ở mọi bước, kể cả khi xe đang chạy, chờ lấy
hàng và đã giao xong. Chỉ các số đo được chốt sau 5 ảnh; khung viền nhận diện
không được giữ đè lên ảnh mới. Nếu mất ảnh quá 2 giây, giao diện hiện CAMERA STALE.

Mỗi xe chở **một món mỗi chuyến**, có thể chở mọi loại ở các chuyến khác nhau.
Ô giao giữ cố định một loại theo bảng chiều cao bên dưới, không phụ thuộc xe nào giao.
Hai xe cùng loại phải chờ ô giao trống: xe trước dỡ hàng và rời ô rồi xe sau mới xác nhận giao.
Nút quay lại đồng thời là xác nhận người giám sát **đã dỡ món trước**.

Nếu đổi lại chỗ hai hộp và xe, đặt đúng ô lấy hàng cho từng xe:

```bash
python3 tools/cargo_live_view.py --agv1-pickup 4,5 --agv2-pickup 3,5
```

Không gửi lệnh `/fleet/goal` riêng để tới ô lấy khi đang dùng giao diện này;
nút BAT DAU và máy trạng thái của giao diện quản lý các chặng.
Không chạy `cargo_task.py` cùng lúc: đó là luồng cũ tự đo ở ô lấy và tự giao.

**Đặt hàng gần lưới:** gợi ý ban đầu là mặt hộp cách tâm ô lấy **35 cm** về phía Đông.
Lưới có bước 60 cm nên hộp cách tâm ô quan sát khoảng **95 cm**.
`DIST` vẫn là khoảng cách từ camera tới mặt hộp, không phải khoảng cách từ ô lấy.
Giữ toàn bộ hộp ngoài footprint và vùng costmap chặn ô/cạnh tiếp cận; không giảm
clearance để ép xe tiến vào. Nếu costmap chặn, dịch hộp ra xa hơn và thử lại.
Giá trị `place.gap: 0.35` dùng để đặt hộp mô phỏng; ở xe thật chương trình đo bằng
LiDAR, không giả định hộp đã được đặt đúng số này. Phải kiểm tra lại kích thước
bằng thước ở bố trí mới; dải chiều cao dưới đây được giữ từ bố trí cũ, chưa phải
số liệu xác minh tại khoảng đặt mới.

Cửa sổ đo chỉ chốt số khi xe đã dừng tại ô quan sát, sai lệch hướng Đông dưới
**2,5°**, mặt hộp lệch không quá **15°** (có bù bằng mặt phẳng LiDAR), LiDAR và ảnh nhất quán,
và toàn bộ hộp nằm trong ảnh. Giao diện lấy **trung bình 5 lần đo hợp lệ** cho
từng xe, chỉ chốt khi 5 số đo kích thước nhất quán trong 12 giây; ảnh nào
bị cắt mép hộp so với LiDAR sẽ bị bỏ qua. Sau khi chốt, không tính lại cho tới
khi bấm **R** tại ô quan sát hoặc bắt đầu chuyến mới. `DIST` là khoảng
cách từ camera tới mặt trước hộp; `WIDTH`, `HEIGHT` là rộng, cao của mặt hộp.
Camera xe 2 được hiệu chuẩn khoảng cách riêng trong `camera_agv2.yaml` theo
số đo bằng thước từ mặt kính camera tới mặt trước hộp xanh 78–79 cm.
Khi không đủ điều kiện, dấu `--` và lý do được hiển thị. Với vị trí hộp cách
tâm ô lấy hàng khoảng **35 cm**, khoảng chiều cao đo tin cậy chung của hai xe
là **24–50 cm**: dưới 24 cm tia LiDAR có thể đi qua phía trên hộp; trên 50 cm
xe 1 có thể cắt mất mép trên ảnh. Đây là giới hạn thực nghiệm/thận trọng từ
góc tia LiDAR và hiệu chuẩn camera hiện tại, không phải thông số tuyệt đối của
hai cảm biến. Chia đều khoảng này để chọn ô giao hàng:

Sau khi tháo/lắp hoặc va chạm camera, cần đo lại góc `yaw` và `pitch` bằng hộp
có kích thước thật đã biết. Ngày 2026-10-16, góc camera đã thay đổi so với
hiệu chuẩn cũ: chỉ lấy trung bình 5 ảnh vẫn cho chiều cao sai có hệ thống.
Hộp xanh cao thật 23,9 cm đã được kiểm tra bằng thước. Số đo từ **23,8 cm**
đến dưới 24 cm được xem là hợp lệ cho ô (1,1) và nút xác nhận hiện **màu xanh**.
Giao diện vẫn chờ người giám sát đặt hàng lên xe rồi bấm nút; chốt kết quả đo
không đồng nghĩa đã nhận hàng. Trong phạm vi ±0,5 cm quanh các ranh giới
**giữa hai ô phân loại**, nút chuyển màu cam, ghi **SAT NGUONG** để người
giám sát kiểm tra ô. Riêng số đo từ 23,5 đến dưới 23,8 cm hoặc trên
50–50,5 cm cũng hiện nút cam để xác nhận bằng thước và chọn ô đầu/cuối.
Ngoài khoảng 23,5–50,5 cm thì nút bị khóa.

| Chiều cao trung bình 5 lần | Ô giao |
|---|---|
| 24 đến dưới 30,5 cm | (1,1) |
| 30,5 đến dưới 37 cm | (1,2) |
| 37 đến dưới 43,5 cm | (1,3) |
| 43,5 đến 50 cm | (1,4) |

Sau khi **xe đã tới ô lấy và đã đặt hàng lên xe**, bấm nút **DA NHAN HANG** ngay dưới ảnh của xe
đó. Chương trình lấy chiều cao đã chốt để chọn ô và gửi một đích qua
`/fleet/goal` cho `traffic_manager`; không gửi lặp nếu bấm lần hai. Hộp ngoài
24–50 cm không được giao tự động; chỉ vùng sát ngưỡng nêu trên mới cho phép
xác nhận thủ công. Nếu cả hai hộp thuộc cùng một ô, chờ xe trước
hoàn tất và rời ô đó rồi mới xác nhận xe sau. **Không chạy `cargo_task.py` song
song với giao diện này** vì đó là luồng đo/phân loại cũ và cũng gửi đích.

Bấm **R** để đo lại các xe đang ở pha đo tại ô quan sát, **S** để lưu ảnh vào `/tmp/cargo_live`,
**Q** để đóng cửa sổ sớm. Hai camera đang dùng:
`192.168.1.201` (xe 1) và `192.168.1.200` (xe 2).

Để dừng khẩn cả hai xe, ở terminal đã `source` môi trường chạy:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: 'stop'}"
```

Kết thúc: bấm **Q** để đóng giao diện, Ctrl-C bộ điều phối và RViz. Sau đó chạy
`tools/fleet_real.sh down`.

## Cập nhật điều khiển xoay 2026-10-23

Yêu cầu góc hiện tại là nhỏ hơn 3°; chương trình chọn ngưỡng **<2,5°**
cho điều hướng và camera. Sai số vị trí cuối vẫn **≤2 cm**. Đây là tham số
điều khiển mới, chưa phải số liệu đo độ chính xác thực tế.

Mỗi chỗ đổi hướng chỉ có một lượt xoay liên tục, cùng một chiều. Tốc độ
xoay giảm theo góc còn lại và gyro, tối đa 0,65 rad/s; tốc độ chạy thẳng
không đổi. Bộ điều khiển ra lệnh dừng một lần, đợi tốc độ quay dưới
0,04 rad/s liên tục 0,35 s rồi kiểm tra góc <2,5°. Không phát xung bù,
không đổi chiều. Nếu chưa đạt góc, mất IMU, quá 15 s hoặc không tiến triển
trong 2 s thì dừng báo lỗi; bộ điều phối không tự giao lại cùng đích.
Các ngưỡng này là cấu hình, chưa phải kết quả thử trên xe thật.

Căn vị trí trước khi xoay sang hướng cuối. Đoạn cuối dùng bộ điều khiển
`PrecisionFollowPath` giảm tốc trong 20 cm cuối, tốc độ tiếp cận tối thiểu
0,025 m/s, bộ kiểm tra đích 1,2 cm (có biên so với yêu cầu cuối ≤2 cm).
Vi chỉnh vài cm dùng tiến/lùi thẳng theo hướng thân xe, tối đa 0,025 m/s,
không xoay đầu theo hướng đoạn nối tới tâm ô. Sai lệch ban đầu phải ≤8 cm,
độ lệch ngang ≤1,8 cm (chừa biên so với yêu cầu bán kính 2 cm). Mỗi lượt
vi chỉnh có giới hạn 6 s và 10 cm theo odometry. Xe kiểm tra vùng quét của
thân xe trên costmap cả phía trước lẫn phía sau, bao gồm ô chưa biết và
xe khác, đồng thời giữ chuỗi velocity_smoother → collision_monitor.

Sau khi dừng ổn định 0,4 s, phải kiểm tra lại **khoảng cách Euclid ≤2 cm**
và hướng <2,5° mới báo tới đích. Giữ nguyên cờ đã xoay hướng cuối, không
tạo lượt xoay thứ hai sau vi chỉnh. Nếu lệch ngang quá lớn, mất dữ liệu,
không tiến triển hoặc có vật cản, xe dừng báo lỗi. Thay đổi nằm ở laptop,
không cần nạp lại ESP32.

Khởi động lại fleet_real, tự định vị lại và mở lại bộ điều phối/giao diện
để nạp mã mới. Giữ micro-ROS riêng đang chạy. Cần quay và đo lại góc thực
tế khi thử trên sàn; kết quả cũ trong báo cáo không tự chuyển sang chế độ mới.

### Lần thử xe 2 ngày 2026-10-23, 17:52–17:53

Xe chạy từ gần (1,2), hướng Đông, tới (4,4), hướng Đông. Hai lượt xoay
đều liên tục cùng chiều, không xoay bù: lượt sang Bắc mất 7,1 s, sai số
0,61°; lượt sang Đông mất 6,7 s, sai số 0,02°. Đây là góc từ định vị ROS,
chưa đối chiếu bằng thước góc bên ngoài.

Trước lượt xoay cuối xe nằm trong ngưỡng vị trí; sau xoay AMCL báo lệch
xấp xỉ 3,0 cm. Hệ thống dừng ở `precision_failed`, không thông báo tới
đích thành công và không tự chạy lại. Lần thử này xác nhận hành vi xoay
một lượt, **chưa đạt yêu cầu vị trí ≤2 cm sau xoay**. Chưa kết luận nguyên
nhân trôi vị trí là chuyển động thân xe hay cập nhật định vị.

Dữ liệu gốc trên laptop: `/home/tai/agv_logs/agv2_single_turn_20261023_175141/`
(`trace.jsonl`, `result.json`, `after/agent_agv2.log`).

### Kiểm tra bước căn tâm không xoay, 2026-10-23 lúc 18:04

Thử lại đúng trường hợp lỗi khi mở giao diện: xe 2 gần ô (4,4), hướng
Đông, sai số vị trí 3,95 cm; thành phần cần lùi 3,6 cm và lệch ngang
1,7 cm. Xe lùi 2,8 cm theo odometry, không xoay thêm. Sau khi dừng ổn
định, chương trình xác nhận sai số vị trí 1,80 cm, góc 0,48°, trạng thái
`done`. Thời gian từ giao đích tới nhận `done` là 3,59 s.

Đây là kết quả từ định vị ROS và odometry, chưa phải đo độc lập trên sàn.
Một lượt thử không bảo đảm mọi vị trí/tải trọng đều đạt. Đặc biệt, tiến/lùi
thẳng không thể sửa sai lệch ngang lớn hơn bán kính cho phép.

Dữ liệu gốc: `/home/tai/agv_logs/agv2_straight_alignment_20261023_180335/`
(`result.json`, `trace.jsonl`, `after/agent_agv2.log`).

### Sửa hướng tại ô giao và khôi phục chuyến đang chở, 2026-10-23 lúc 18:14

Log chuyến sau cho thấy xe đã quan sát và tới ô lấy (4,5) với sai số
1,1 cm, 1,04°. Sau khi người giám sát xác nhận hàng, xe đi tới (1,3).
Giao diện cũ gửi thêm hướng Đông cho cả ô giao. Xe đến theo hướng Nam
rồi xoay sang Đông, sau đó AMCL báo lệch ngang 3,2 cm và dừng đúng theo
ngưỡng 2 cm. Bộ căn tiến/lùi không thể sửa một sai lệch ngang như vậy.

Giao diện hiện chỉ yêu cầu hướng camera ở ô quan sát/lấy; ô giao giữ
hướng của đoạn cuối, giống logic giao hàng của `cargo_task.py`. Bộ điều
phối tính cả chi phí xoay sang hướng đích nếu người dùng có chỉ định,
và ưu tiên hướng tiếp cận phù hợp khi tổng chi phí bằng nhau. Không thay
hướng của một đích được người dùng chỉ định rõ.

Nếu đích không yêu cầu hướng và xe đang lệch ngang trong phạm vi 8 cm,
cho phép một lượt xoay sang trục tiến/lùi phù hợp, giữ trục đó làm hướng
cuối, rồi căn vị trí. Không xoay ngược lại Đông và không lặp xoay bù.

Thử phục hồi chính ô giao (1,3): sai số trước 3,23 cm, xe xoay một lượt
về khoảng -85°, căn tiến nhẹ, xác nhận tới đích với sai số 1,74 cm,
sai số góc 0,62° so với hướng đã chọn. Cả lượt mất 7,38 s. Đây là số đo
ROS/odometry; chưa đo độc lập trên sàn.
Dữ liệu: `/home/tai/agv_logs/agv2_delivery_heading_20261023_181418/`.

Nút **THU LAI CHANG DANG LOI** gửi lại đúng chặng quan sát/lấy/giao,
giữ hàng và loại hàng đã chốt. Không tự thử lại liên tục.
Nếu phải khởi động lại giao diện giữa một chuyến đã xác nhận lấy hàng,
có thể dùng `--resume-delivery agv2,3` để phục hồi chuyến giao đã biết tới
(1,3); bấm nút tiếp tục để gửi. Chỉ dùng khi biết đúng xe, món hàng và ô
đã xác nhận; không dùng thay bước phân loại. Tuỳ chọn `--dispatch-resumed`
cho phép gửi chính chuyến phục hồi này sau khi nhận vị trí mới; mặc định
không gửi lúc mở cửa sổ. Số đo rộng/cao cũ không khôi phục được sẽ ghi
`null`, không tạo số liệu thay thế. Sau giao thành công, nút quay lại
vẫn yêu cầu người giám sát đã dỡ hàng trước khi bắt đầu chuyến tiếp.

### Giúp đo hộp dễ hơn, 2026-10-23

Tại ô quan sát (4,4), hộp nâu nằm trọn ảnh nhưng giao diện cũ vẫn chặn vì
LiDAR ước lượng mặt hộp lệch -3,9° so với giới hạn 3°. Ba trong năm ảnh
lưu khác bị mặt nạ GrabCut gộp khung cửa phía sau vào hộp, chạm cạnh trên
ảnh. Đây là lỗi tách nền, không phải hộp thực sự vượt khung hình.

Bản hiện tại dùng hình chữ nhật có cạnh ảnh khép kín, phù hợp với hai mép
LiDAR và chân hộp, để giới hạn vùng GrabCut. Giữ khởi tạo GrabCut ổn định
để cùng một ảnh không lúc được/lúc không do bước khởi tạo ngẫu nhiên.
Cho phép mặt hộp lệch tối đa 15° vì công thức chiếu tia lên mặt phẳng LiDAR
đã bù độ nghiêng; vẫn kiểm tra số điểm, độ khớp mặt, mép ảnh và sự nhất quán
camera/LiDAR. Giới hạn hướng điều hướng <2,5° và bán kính đích ≤2 cm không đổi.

Khi thấy rõ cả mép trên và mép dưới, chiều cao được tính từ hiệu tọa độ
hai mép trên mặt phẳng hộp. Mép dưới phải có gradient đủ mạnh, khớp giữa
ít nhất 9/15 cột ảnh, sai khác hàng cạnh dưới không quá 3 pixel và gần sàn.
Nếu mép dưới không đáng tin mới dùng mô hình sàn cũ. Không thay kích thước
đo được bằng kích thước người dùng cung cấp và không thay file hiệu chuẩn
camera để ép kết quả trùng hộp mẫu.

Vẫn chốt trung bình 5 ảnh mới, kiểm tra độ ổn định. Xử lý tối đa một ảnh
mỗi 0,35 s theo luồng camera 3 Hz; số đo tạm màu cam chưa phải số đo cuối
và không được gửi đích lấy hàng trước khi chốt đủ 5 ảnh. Hộp cắt mép hoặc
mất khoảng cách LiDAR vẫn bị từ chối; chỉ nhìn thấy hộp trong camera không
đủ suy ra kích thước thực nếu thiếu thước đo khoảng cách.

Người dùng đo hộp nâu bằng thước: rộng 18,5 cm, cao 32,0 cm.
Kiểm tra 5 ảnh mới sau sửa, không phát lệnh chạy xe:

| Đại lượng | Trước sửa (ảnh lưu) | Sau sửa (5 ảnh mới) | Thước |
|---|---:|---:|---:|
| Rộng | khoảng 18,7 cm | 18,68 cm | 18,5 cm |
| Cao | khoảng 33,8 cm | 32,56 cm | 32,0 cm |

Sai lệch sau sửa ở bố trí này: rộng +0,18 cm, cao +0,56 cm. Thời gian
chương trình kiểm tra mới nhận đủ 5 ảnh hợp lệ là 5,27 s, gồm thời gian
kết nối/nhận dữ liệu ban đầu. Đây là kiểm tra trên một hộp, một bố trí,
không đại diện cho mọi loại hàng hay mọi khoảng cách. Dữ liệu gốc và ảnh:
`/home/tai/agv_logs/cargo_measure_review_20261023_183235/`
(`fresh_validation.json`, `fresh_final_measurement.png`, `live_samples.npz`).
