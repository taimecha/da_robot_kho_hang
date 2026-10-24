# Mô phỏng 3 xe, 3 hàng đợi hàng hóa riêng

Mục đích: kiểm tra thuật toán điều phối, lấy hàng FIFO, tách kiện đầu hàng
khỏi kiện lớn phía sau, phân loại theo chiều cao và vận chuyển lặp lại.

[Bản hiện tại: nối đuôi trên cột giao, đường về linh hoạt và kiểm chứng 30 kiện](warehouse_column_validation/README.md).

[Hình, log và kết quả kiểm chứng ngày 24/10/2026](warehouse_sim_validation/README.md).

[Cơ chế hàng chờ gần ô giao và phép so sánh thời gian](delivery_queue_validation/README.md).

[Sửa vòng chờ ba xe và lượt chạy liên tục đủ 30 kiện](warehouse_continuous30_validation/README.md).

[RViz ba xe, cảnh nhà xưởng và sửa cơ cấu cấp hàng](warehouse_factory_validation/README.md).

[Sửa đường vòng, đổi đích giữa cạnh ở lượt đầu và kiểm chứng lại 30 kiện](warehouse_waiting_validation/README.md).

## Cách chạy

### Kịch bản hiện tại: 30 kiện, ba xe hoạt động độc lập

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh up --scenario demo30
```

Đợi `Ready`, chọn cửa sổ ba camera và bấm **Space**. Một lệnh mở Gazebo,
Nav2, RViz và giao diện camera; không cần xe thật hoặc micro-ROS. Chương trình
không tự bắt đầu khi chưa bấm Space, trừ khi thêm `--autostart`.

RViz dùng bố cục của xe thật, mở rộng thành ba nhóm xe: mô hình, LaserScan,
Footprint, đường đang chạy, Local Plan và các lớp costmap có thể bật riêng.
Mỗi xe giữ một màu. TF và dữ liệu điều hướng được chuyển sang tên khung riêng
cho hiển thị, tránh trùng `base_footprint` giữa ba xe. Mô phỏng này dùng odometry
lý tưởng, không chạy AMCL nên không hiển thị hạt AMCL giả. Thêm `--no-rviz` nếu
chỉ muốn Gazebo và camera; `--headless` không mở cả ba cửa sổ.

Mỗi xe có **10 kiện**, tổng cộng **30 lần giao**. Thứ tự 5 kiện đầu của mỗi
dãy giữ nguyên bảng 15 kiện bên dưới. Thứ tự 5 kiện sau là:

| Kiện thứ trong mỗi dãy | AGV1 | AGV2 | AGV3 | Loại hàng theo lịch đầu vào |
|---|---|---|---|---|
| 6 | T2 → (1,2) | T3 → (1,3) | T4 → (1,4) | Ba loại khác nhau |
| 7 | T4 → (1,4) | T1 → (1,1) | T2 → (1,2) | Ba loại khác nhau |
| 8 | T3 → (1,3) | T1 → (1,1) | T3 → (1,3) | Cặp xe 1–3 cùng loại |
| 9 | T4 → (1,4) | T2 → (1,2) | T1 → (1,1) | Ba loại khác nhau |
| 10 | T1 → (1,1) | T3 → (1,3) | T3 → (1,3) | Cặp xe 2–3 cùng loại |

**Mặc định không có rào đồng bộ:** xe nào nạp xong thì đi giao, giao xong thì
quay thẳng về ô quan sát của dãy mình để đo và lấy kiện kế tiếp. Không chờ hai
xe còn lại, không về điểm xuất phát giữa các lượt. Chỉ khi dãy riêng hết 10
kiện, xe mới đi đỗ ở hàng 5, cột 1–3 tương ứng.

Chỉ số trong bảng là thứ tự kiện của từng dãy, không phải đợt xuất phát chung.
Loại hàng trùng ở cùng một chỉ số không bảo đảm ba xe cùng mang loại đó tại
cùng thời điểm: thời điểm thực tế phụ thuộc đường đi và hàng chờ. Nếu cần tái
hiện các nhóm đồng thời cho một phép kiểm thử riêng, thêm `--sync-rounds`;
chế độ này có thời gian chờ chủ động và không dùng để đánh giá năng suất tự do.
Con lăn, khung băng tải, sàn và bản đồ phòng được mở rộng theo chiều dài dãy
10 kiện. Lưới vận hành vẫn **5 × 5**. Đây là lịch đầu vào, không phải bằng
chứng đã chạy hoàn thành 30 kiện. Kết luận hoàn thành phải đối chiếu log của
chính lượt chạy; các hình và kết quả cũ không được gán cho lịch 30 kiện.

### Xử lý dữ liệu trễ trong mô phỏng

- Odometry trễ hơn 1 giây chỉ ghi cảnh báo, không hủy ngay cả nhiệm vụ như trước.
- Odometry trễ hơn 2 giây hoặc trạng thái xe trễ hơn 3 giây: gửi lệnh dừng cả
  đội, giữ đích đang thực hiện, thứ tự hàng chờ và quyền ô giao; tạm ngừng
  cơ cấu nạp/đẩy hàng và đồng hồ đếm thời gian nhiệm vụ từng bước.
- Khi odometry của cả ba xe mới dưới 0,3 giây và trạng thái mới dưới 0,8 giây
  liên tục trong 1 giây, giao lại các đích chưa hoàn thành từ vị trí hiện tại.
  Không tự xác nhận lấy/giao hàng trong khoảng dữ liệu trễ.
- Dữ liệu mất từ 10 giây, hoặc đã tạm dừng 15 giây mà chưa hồi phục ổn định:
  dừng hẳn và báo lỗi; cần kiểm tra và chạy lại.

Giao diện hiển thị `TAM DUNG - CHO DU LIEU ON DINH` trong lúc phục hồi.
Log có `telemetry_warning`, `telemetry_paused`, `telemetry_recovering`,
`telemetry_resumed`, `navigation_resumed` và `telemetry_failed`.
Tuổi odometry tính theo dấu thời gian nguồn, để gói cũ tới muộn không bị coi
là dữ liệu mới. Yêu cầu cập nhật hình 3D chạy ở luồng riêng, tối đa một yêu cầu
đang xử lý; không tích hàng đợi yêu cầu Gazebo. Chỉ phát ảnh RGB qua ROS nếu
có người đăng ký nhận, còn giao diện camera tại máy vẫn cập nhật liên tục.
Các thay đổi giám sát này nằm trong bộ mô phỏng, không sửa firmware xe thật.

Dừng bằng **Ctrl+C** ở terminal chạy, hoặc:

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh down
```

### Lịch đầu vào 15 kiện (giữ để so sánh)

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh up --scenario demo15
```

Lệnh này tự mở Gazebo 3D, ba Nav2, bộ điều phối, RViz và cửa sổ ba camera.
Đợi terminal báo `Ready`, chọn cửa sổ camera rồi bấm **Space** để bắt đầu quay.
Nếu muốn chạy ngay khi khởi động xong, thêm `--autostart`.
Không cần mở riêng RViz hoặc micro-ROS cho kịch bản này.

| Kiện thứ trong mỗi dãy | AGV1 | AGV2 | AGV3 | Loại hàng theo lịch đầu vào |
|---|---|---|---|---|
| 1 | T1 → (1,1) | T1 → (1,1) | T1 → (1,1) | Cả ba cùng loại, lần 1 |
| 2 | T4 → (1,4) | T4 → (1,4) | T4 → (1,4) | Cả ba cùng loại, lần 2 |
| 3 | T2 → (1,2) | T2 → (1,2) | T3 → (1,3) | Cặp xe 1–2 cùng loại |
| 4 | T3 → (1,3) | T2 → (1,2) | T3 → (1,3) | Cặp xe 1–3 cùng loại |
| 5 | T1 → (1,1) | T2 → (1,2) | T2 → (1,2) | Cặp xe 2–3 cùng loại |

Tổng cộng **15 lần giao kiện hàng**, mỗi xe 5 kiện; không phải 15 đợt × 3 xe.
“Cùng loại” là cùng ô giao theo chiều cao; mỗi ô vẫn chỉ phục vụ một xe tại
một thời điểm. Ba xe có thể cùng di chuyển tới vùng giao, rồi xếp hàng vào ô.

Chỉ khi thêm `--sync-rounds`, kịch bản `demo15` mới có rào đồng bộ phục vụ kiểm thử: đợi cả ba xe nạp kiện của
đợt đó rồi mới cho xuất phát giao; chỉ bắt đầu đợt tiếp theo khi cả ba giao xong
và về vùng đợi hàng 5, cột 1–3 (tương ứng xe 1–3). Vùng đợi này tránh chiếm
đường vào các ô giao. Sau đó xe trở lại ô quan sát cho đợt mới.
Trong chế độ đồng bộ đó, từng tình huống trong bảng được tái hiện,
không phụ thuộc xe nào đi nhanh hơn. Đây là thời gian chờ chủ động của kịch bản,
không dùng để kết luận năng suất tối đa. Chế độ `mixed` mặc định bên dưới vẫn
cho các dãy chạy liên tục độc lập.
Sau đợt cuối, ba xe cũng về hàng 5, cột 1–3 rồi báo hoàn thành.

Các sự kiện `round_started`, `round_ready`, `round_completed` lưu từng đợt;
`audit.json` có `synchronized_round_coverage`, tính từ loại camera đã chốt và
đối chiếu kiện giao thành công. Bảng trên là đầu vào thiết kế, chỉ gọi là kết
quả hoàn thành khi log thực chạy xác nhận đủ 15 kiện và không có lỗi bất biến.

Tắt toàn bộ tiến trình của mô phỏng bằng **Ctrl+C** ở terminal chạy, hoặc mở
terminal khác:

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh down
```

### Kịch bản các dãy chạy liên tục

Không cần bật xe thật hoặc micro-ROS. Mô phỏng dùng riêng ROS domain **88**,
Gazebo partition `warehouse_queue_sim`. Các cấu hình sinh ra nằm trong thư mục
log, không sửa bản đồ, thông số điều hướng hay firmware của xe thật.

Terminal 1:

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh up
```

Đợi ba dòng kích hoạt Nav2 hoàn tất. Gazebo hiển thị mô hình 3D; RViz hiển thị
bản đồ, lưới và dữ liệu của ba xe; cửa sổ
`Warehouse - 3 AGV` hiển thị ba camera, kích thước đã đo, loại hàng, hàng đợi
và trạng thái giao hàng. Bấm **Space trong cửa sổ camera** để bắt đầu.
Mỗi xe tự chạy hết 4 kiện của dãy mình, mỗi lần chở một kiện.

Có thể chạy tự động ngay sau khi mở:

```bash
./tools/warehouse_sim.sh up --autostart
```

Terminal 2, nếu muốn bắt đầu bằng lệnh:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ros2 topic pub --once /warehouse/control std_msgs/msg/String "{data: start}"
```

Dừng toàn bộ mô phỏng bằng **Ctrl+C tại terminal 1**, hoặc:

```bash
cd /home/tai/Robot_kho_clone
./tools/warehouse_sim.sh down
```

Chỉ các nhóm tiến trình do lần mô phỏng này tạo ra bị đóng; không dùng `pkill`
theo tên chung. `Q` trong giao diện dừng nhiệm vụ để xem lỗi/kết quả; dùng
Ctrl+C hoặc `down` để đóng cả Gazebo và camera. Chạy lại `up` để có lượt mới.

Xem thư mục log và trạng thái tiến trình:

```bash
./tools/warehouse_sim.sh status
```

## Bố trí

Lưới 5 × 5, bước 0,60 m. Hàng tăng theo trục y, cột tăng theo trục x;
ô (3,3) là gốc tọa độ. Đây là bản đồ phòng mô phỏng riêng, không phải bản đồ
SLAM phòng thật.

| Xe | Màu | Ban đầu, hướng Đông | Quan sát | Lấy hàng, hướng Đông | Thứ tự FIFO |
|---|---|---|---|---|---|
| AGV1 | Xanh dương | (2,1) | (2,4) | (2,5) | T1 → T4 → T2 → T3 |
| AGV2 | Cam | (3,1) | (3,4) | (3,5) | T2 → T4 → T1 → T3 |
| AGV3 | Xanh lá | (4,1) | (4,4) | (4,5) | T1 → T3 → T4 → T2 |

Mỗi dãy chứa đủ bốn loại. Xe có thể chở nhiều loại qua nhiều lượt. Một loại
chỉ giao vào một ô cố định, dùng chung cho cả ba xe. Bộ quản lý nhiệm vụ giữ
ô giao đang phục vụ; xe khác cùng loại phải đợi. Điều phối đường đi và giữ ô
trên lưới vẫn do `traffic_manager.py` thực hiện.

Mặc định hiện dùng chế độ **`staged` với hàng nối đuôi trên cùng cột giao**.
Ví dụ ba xe cùng giao T1 vào (1,1): xe 1 đi ngang tới (2,1), xe 2 tới (3,1),
xe 3 tới (4,1), sau đó tiến lần lượt trên cột 1. Khi xe đầu đã tiến vào trạm,
hai xe sau có thể tiến tới (2,1), (3,1) theo khoảng trống thực tế. Không chia
xe sang các ô chờ ở cột khác. Loại T2, T3, T4 nối đuôi tương tự ở cột 2, 3, 4.

Các ý định lấy hàng đã được camera phân loại được đăng ký trước khi cấp
quyền giao; trong cùng chu kỳ đăng ký, xe ở phía trước được xếp trước để
tránh xe phía sau phải vượt qua xe phía trước. Sau đó giữ thứ tự hàng đợi.
Xe nạp xong có thể đi ngay tới cột giao, không cần chờ cả đội nạp xong.
Chỉ xe đầu hàng đã nạp đủ kiện và được cấp quyền trạm mới vào ô giao.

Mỗi xe nhận một lộ trình duy nhất tới (1,cột giao). Bộ điều phối giới hạn
những ô được đi, rồi mở dần giới hạn theo vị trí xe trước và quyền phục vụ
trạm. Việc mở giới hạn dùng cùng mã lộ trình, không hủy FollowPath hoặc
buộc quay về ô trước để đổi đích. Xe mới nhập hàng ở sau đuôi đang có; nếu
chưa đủ chỗ thì chờ trước khi nhập cột. Vẫn giữ kiểm tra vật cản và giữ ô.

Xe được giao **một đích duy nhất là ô giao**, hướng Nam. Bộ tìm đường buộc
cạnh cuối đi từ (2,cột giao) vào (1,cột giao), nhưng (2,cột giao) chỉ là điểm
đi qua, không còn là đích phải dừng chốt sai số rồi nhận một lệnh mới. Những
chỗ đổi hướng vẫn phải giảm tốc/xoay phù hợp với xe hai bánh vi sai; nơi bị
xe khác giữ ô vẫn phải chờ.

Sau khi dỡ hàng, xe nhận thẳng đích ô quan sát của dãy mình (hoặc ô đỗ khi đã
hết hàng), không có đích rời trạm bắt buộc. Mặc định `delivery_flow=flexible`:
có thể rời trạm sang Tây, Đông hoặc quay về cùng cột nếu không có xe khác
đang đi vào cột đó. Không bắt buộc tới (2,cột giao+1), không có đường một
chiều cố định trên hàng 1. Cột đang có xe khác đi vào được bảo vệ để tránh
quay về đối đầu với hàng xe nối đuôi. Khi xe cách tâm trạm hơn 0,45 m theo
hướng thực tế đã chọn, trả quyền để xe sau tiến vào dưới kiểm tra giữ ô.

Bộ tìm đường xét trạng thái (ô, hướng), dùng chi phí thời gian ước lượng:
quãng đường/vận tốc + thời gian khởi động xoay và góc quay + chi phí chờ
trên các ô/cạnh tranh chấp. Tính cả lần xoay cuối tại ô quan sát. Các trọng
số riêng của mô phỏng nằm ở `route_time_costs` trong `grid.yaml` sinh ra;
đây là tham số đánh giá phương án, không phải kết quả đo hoặc bảo đảm tối
ưu toàn cục của cả đội xe. Cơ chế giữ ô và kiểm tra vật cản vẫn quyết định
khi nào phương án được phép thực hiện.

Xe đang đo, nạp hoặc dỡ hàng được đánh dấu đang phục vụ, có thời hạn và cập
nhật liên tục. Bộ điều phối không coi việc dừng hợp lệ đó là kẹt đường để
đẩy xe đi hoặc hủy lộ trình của xe đang chờ. Trạng thái xác nhận lộ trình có
kèm đúng ô đích; lộ trình nhường đường không được nhầm thành đã nhận đích giao.

Khi gỡ kẹt ba xe, bộ điều phối xét cả vị trí xe thứ ba và chỉ thực hiện một
phương án phục hồi tại một thời điểm. Phương án được giữ trong lúc xe xác
nhận lệnh, xoay và rời ô đang chắn (tối đa 12 giây trước khi đánh giá lại),
tránh hai phép xử lý từng cặp liên tiếp ghi đè lộ trình khiến xe xoay qua lại.
Các ô đã cấp theo lệnh cũ chỉ được giải phóng sau khi xe xác nhận lệnh thay
thế. Đích nghiệp vụ mới và lệnh thử lại không hủy bước nhường đang thực hiện.
Xe đã nhường khôi phục đích cũ dưới cơ chế cấp quyền từng ô; nếu xe đỗ vẫn
chắn lối, bộ điều phối tiếp tục yêu cầu nhường thay vì chờ vô hạn ngoài lộ trình.

Mỗi trạm có băng tải nhận hàng đưa kiện về thùng thu có mái che. Kiện nằm
cùng một mặt đỡ, không cộng chiều cao lên kiện trước. Khi đã đi hết băng tải
vào thùng thu, mô hình kiện được đưa ra khỏi cảnh; ID và bản ghi giao vẫn giữ.
Đây là cơ cấu động học minh họa, không mô phỏng va chạm hoặc sức chứa thùng thật.
Log `outbound_cleared` ghi kiện đã rời băng tải nhận.

Sau kiện cuối, xe mới đi đỗ hàng 5 theo cột 1–3. Có thể thêm
`--delivery-flow east` để so sánh với luồng cũ buộc rời hàng giao ở cột 5.
Đây là cải tiến điều phối nhằm giảm chờ và đường vòng; chưa chứng minh tối ưu
thời gian toàn cục cho mọi cách bố trí, mật độ xe và trình tự hàng hóa.

| Loại | Rộng × cao × sâu thiết kế (cm) | Khoảng chiều cao phân loại | Ô giao |
|---|---|---|---|
| T1 | 12 × 27 × 14 | 22 ≤ H < 29 cm | (1,1) |
| T2 | 18,5 × 34 × 18 | 29 ≤ H < 36 cm | (1,2) |
| T3 | 24 × 40 × 22 | 36 ≤ H < 43 cm | (1,3) |
| T4 | 30 × 47 × 26 | 43 ≤ H ≤ 50 cm | (1,4) |

Dải phân loại hiện tại là **22–50 cm**, chia bốn khoảng bằng nhau, mỗi khoảng
7 cm. Ngoài dải này chương trình từ chối phân loại. Các kết quả mô phỏng đã
lưu được giữ nguyên theo cấu hình tại thời điểm chạy; các phép kiểm chứng mới
được ghi vào thư mục log riêng.

Kích thước và thứ tự thử nằm trong `tools/warehouse_queue.py` (`SIZES`,
`ORDERS`). Các con số này là **đầu vào thiết kế**, không phải số đo thực nghiệm.

## Chu trình và cơ cấu 3D

1. Xe tới ô quan sát, đứng yên hướng Đông.
2. Camera và LiDAR đo kiện đầu dãy. Chốt trung bình 5 ảnh hợp lệ, có độ dao
   động rộng/cao không quá 12 mm trong năm mẫu; đây là ngưỡng mô phỏng.
3. Xe tới ô lấy hàng. Chốt chặn hạ, bàn nối mở, bàn đẩy chuyển kiện đầu lên
   khay xe trong 2 giây. Các kiện còn lại đứng yên.
4. Xe mang kiện rời điểm lấy. Chỉ khi tâm xe cách điểm lấy hơn 0,42 m,
   con lăn mới quay và đưa hàng còn lại lên trong 2,5 giây; chốt giữ kiện kế
   tiếp ở vị trí đầu dãy. Mỗi dãy hoạt động độc lập.
5. Xe cùng loại nhập cùng cột giao và tiến theo quyền ô; xe đầu hàng được
   cấp quyền trạm mới đi vào ô giao.
   Kiện chuyển lên băng tải nhận, giao diện báo `DA GIAO THANH CONG`. Xe lập
   đường về ô quan sát để lấy kiện tiếp theo ngay; chỉ đi đỗ khi hết dãy.
   Xe tiếp theo được vào sau khi xe trước rời an toàn, không chờ xe trước
   trở lại ô quan sát. Các xe khác tiếp tục công việc độc lập.

Trong mô phỏng, xác nhận chất/dỡ hàng được tự động hóa bằng trạng thái cơ cấu;
xe thật vẫn cần cảm biến/cơ cấu và xác nhận phù hợp. Các thời gian và khoảng
dịch chuyển trên là tham số thiết kế mô phỏng, chưa đo trên cơ cấu thật.

## Cách ngăn đo gộp kiện trước và sau

- LiDAR trả giao điểm gần nhất của từng tia với các hộp chữ nhật. Các tia ở
  hai bên vẫn có thể thấy kiện lớn phía sau; không xóa kiện sau khỏi cảnh.
- `fit_face()` tách cụm tia và khớp mặt phẳng trước; các đầu mút được chiếu
  lên ảnh đã khử méo theo nội/ngoại tham số camera.
- Chỉ chấp nhận tứ giác kín phù hợp với hai đầu mút LiDAR và mặt băng tải
  cao 6 cm. Tứ giác này giới hạn vùng GrabCut, tránh học màu của kiện sau
  thành phần kéo dài của kiện trước. Mép ảnh đo được được chiếu lên đúng
  mặt phẳng kiện trước để suy ra rộng/cao.
- Không có tứ giác tin cậy, bị cắt mép ảnh, mất dữ liệu hoặc năm lần đo chưa
  ổn định: không phân loại và không phát lệnh lấy hàng.
- `measure()` chỉ nhận ảnh RGB, các vòng quét và hiệu chuẩn. ID, kích thước
  thiết kế và loại thật không được truyền cho bộ đo. Chúng được dùng sau
  khi chốt để kiểm tra kết quả; nếu sai, phép chạy bị đánh dấu thất bại.

Camera ảo có phép chiếu phối cảnh, méo xuyên tâm, nhiễu điểm ảnh, làm mờ và
nén JPEG. Các kiện cùng màu carton, có đường ghép/mép tối giống nhau; màu
không mã hóa loại hàng. Đây là ảnh tổng hợp có đường biên quan sát được.
Không thể bảo đảm tách đúng bằng RGB đơn mắt khi hai mặt đồng màu hoàn toàn,
không có biên phân biệt, hoặc kiện trước bị che khuất. Khi đó cần đổi góc
nhìn/ánh sáng, làm cơ cấu tách kiện, hoặc bổ sung cảm biến chiều sâu; không
được suy diễn rằng mô phỏng này đã giải quyết mọi điều kiện thực tế.

## Kiểm tra và tài nguyên đầu ra

```bash
# 16 cặp loại trước–sau × 3 độ lệch ngang = 48 ca, kèm ca phải từ chối đo.
./tools/warehouse_sim.sh vision-check --out /tmp/warehouse_vision

# Chạy ngắn mỗi xe một kiện; không mở cửa sổ, vẫn chạy Gazebo server và Nav2.
./tools/warehouse_sim.sh up --headless --autostart --items 1

# Ba xe cùng giao T2: xem xe xếp hàng gần ô (1,2), lần lượt vào giao.
./tools/warehouse_sim.sh up --autostart \
  --orders 'agv1:2;agv2:2;agv3:2'

# Chế độ cũ để so sánh: chờ ở ô lấy, chỉ trả quyền ô giao khi về ô quan sát.
./tools/warehouse_sim.sh up --headless --no-gazebo --autostart \
  --orders 'agv1:2;agv2:2;agv3:2' --delivery-mode serial --exit-on-complete

# Sau một lượt chạy, tạo đồ thị và kiểm tra FIFO/chốt liên động/ô giao.
# Thay DUONG_DAN_LOG bằng out được in ra ở terminal hoặc lệnh status.
source tools/setup_real_env.bash
python3 tools/warehouse_report.py DUONG_DAN_LOG
```

Mỗi lần `up` mặc định tạo `/home/tai/agv_logs/warehouse_3agv_NGAY_GIO/`:

- `events.jsonl`: đo/chốt, lấy hàng, xe rời đi, băng tải chạy, tới đích, giao xong.
- `trajectories.csv`: vị trí và giai đoạn của từng xe theo thời gian.
- `*_measured.png`, `dashboard_latest.jpg`, `dashboard_final.png`: bằng chứng ảnh.
- `summary.json`, `audit.json`: trạng thái hoàn thành, khoảng cách gần nhất giữa
  tâm xe, sai số so với cảnh tổng hợp, kiểm tra đúng thứ tự và đúng ô giao.
- `scenario.json`: chế độ điều phối, thứ tự loại hàng và ngưỡng phân loại của lượt chạy.
- Trong chế độ `staged`, `delivery_queued`, `column_join`, `column_permission`, `delivery_admitted`,
  `bay_released` ghi lại thời điểm xếp hàng và bàn giao ô. `audit.json` kiểm tra
  không vượt lượt/không có hai chủ ô đồng thời và ghi thời gian chuyển lượt.
- `trajectories_and_deliveries.png`, `synthetic_measurement_errors.png`: đồ thị
  dựng từ log bằng công cụ báo cáo.
- `bay_turnaround.png`: khoảng từ khi xe trước giao xong đến khi xe sau tới ô giao,
  gồm thời gian xe trước đi ra và xe sau đi vào; không coi khoảng này là toàn bộ
  thời gian trạm đứng yên không làm việc.
- `nav_fleet.log`, `agent_agv*.log`, `traffic.log`, `scene.log`, `gazebo.log`:
  log để kiểm tra lỗi, không xóa khi đóng giao diện.

## Phạm vi có thể viết trong báo cáo NCKH

“Hệ thống được kiểm tra bằng mô phỏng ba robot trên lưới dùng chung, mỗi robot
phục vụ một hàng đợi FIFO gồm bốn cỡ kiện xen kẽ. Thuật toán sử dụng mặt phẳng
gần nhất đo bằng LiDAR để giới hạn vùng ảnh của kiện đầu hàng, tránh gộp đường
bao của kiện lớn phía sau. Sau khi robot nhận kiện và rời vùng lấy hàng, cơ cấu
con lăn–chốt chặn–bàn đẩy đưa kiện tiếp theo tới vị trí nhận. Bộ điều phối quản
lý đường đi, quyền chiếm ô và truy cập các ô giao theo loại; chu trình lấy–giao
được lặp lại cho đến hết hàng.”

Ghi rõ: robot dùng mô hình **động học**, vị trí odometry lý tưởng; Nav2,
`fleet_agent.py` và `traffic_manager.py` chạy thật. Gazebo hiển thị cơ cấu và
vị trí được cập nhật theo động học; ảnh/LiDAR do chương trình cảm biến tổng
hợp sinh ra, **không phải camera/depth sensor của Gazebo**. Chưa mô phỏng tiếp
xúc, ma sát, trượt bánh, lực đẩy hay độ bền cơ cấu. Sai số ảnh tổng hợp không
được dùng thay cho độ chính xác đo hàng hoặc định vị của xe thật. Chỉ bổ sung
số liệu hoàn thành sau khi đối chiếu `summary.json` và `audit.json` của lượt chạy.
