# Sáu video NCKH liên tiếp — hai xe thật, định vị một lần

Mục tiêu của mỗi video là nhìn thấy đồng thời **hành vi ngoài sàn** và **lệnh,
đường đi, trạng thái trên RViz/terminal**. Quay góc rộng bằng điện thoại để thấy
hai xe và các ô lưới; ghi màn hình laptop riêng để đọc được tọa độ, hướng và log.
Giữ đoạn từ lúc giao lệnh đến lúc xe dừng trong bản gốc, ghi số lần thử và các
lần thất bại để số liệu báo cáo có thể đối chiếu. Sáu clip chạy **theo thứ tự**
dưới đây. Chỉ đặt xe bằng tay khi chưa cấp điện; sau khi định vị lần đầu,
mọi bước chuẩn bị vị trí đều dùng lệnh điều hướng. Không tắt nguồn, ROS hay
Nav2 giữa các video.

## Chuẩn bị chung

Hàng 1 ở dưới cùng RViz, cột 1 bên trái; mỗi ô cách nhau 60 cm. Ô **(1,5)**
bị bỏ trên bản đồ hiện tại. `d/t/b/n` = Đông/Tây/Bắc/Nam. Khi hai xe còn **tắt
nguồn**, đặt AGV1 **(2,2) hướng Đông**, AGV2 **(4,4) hướng Tây**, bánh chạm
sàn; dọn lưới và chưa đặt hộp/vật cản. Kiểm tra kỹ **cột 1**, nhất là
(3,1) và hai ô giao (1,1)/(1,2): trước đây phía tây sa bàn từng có đồ ngoài
bản đồ làm xe lệch 4–6 cm. Bật nguồn hai xe, đợi LiDAR quay và
Wi-Fi kết nối. **Terminal A — hệ xe**, chạy một lần và đợi `Da bat`:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
AGV1="2 2 0" AGV2="4 4 180" tools/fleet_real.sh up
python3 tools/fleet_localize.py
```

Góc trên dòng `AGV1`/`AGV2` tính bằng **độ**: 0 là Đông, 180 là Tây. Lệnh
định vị chạy **một lần**, phải in `da gui cho AMCL` ở **cả hai xe** và RViz
phải khớp vị trí/hướng thật. Nếu `KHONG GUI`, sửa nguyên nhân rồi mới thử
định vị lại; lần đầu chưa thành công. Nếu Nav2 báo `KHONG LEN`, xem
`/tmp/fleet_real/nav_agv1.log` hoặc `nav_agv2.log` trước khi giao lệnh.

**Terminal B — RViz**, giữ mở đến hết video 6:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
rviz2 -d tools/fleet.rviz
```

**Terminal C — điều phối:** video 1 dùng chế độ nhập tọa độ; sau video 1
gõ `q` tại prompt để thoát, rồi ngay trong terminal C chạy
`python3 traffic_manager.py agv1 agv2 --no-stdin` và giữ đến hết video 6.
Chỉ có **một** `traffic_manager.py` tại một thời điểm. Chế độ `--no-stdin`
nhận các lệnh `/fleet/goal` và không đọc ký tự còn chờ trong terminal.

Mỗi video mở **terminal D mới** để gửi lệnh. Ở đầu terminal D chạy:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
```

Chờ xe **đến nơi và đứng hẳn**, xác nhận ô/hướng trên RViz và ngoài sàn rồi
mới giao lệnh tiếp theo. Nếu phải dừng một lần thử, gửi
`ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: 'stop'}"`;
ghi là dừng/chưa đạt và đưa xe về ô xuất phát bằng lệnh điều hướng. Nếu bế
xe, khởi động lại hoặc mất định vị, bắt buộc định vị lại; đó là ngoại lệ,
không thể duy trì điều kiện “một lần” sau khi trạng thái xe đã thay đổi.

Lưu dữ liệu mỗi lần quay vào thư mục **ngoài Git**. Mỗi video mở **terminal E
mới** để ghi ROS bag; đổi `VIDEO=01` thành `02`...`06` đúng clip. Không dùng
lại thư mục đã có bag (lần thử sau: `video02_lan02`):

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
VIDEO=01
mkdir -p "$HOME/nckh_evidence/video${VIDEO}"
ros2 bag record -o "$HOME/nckh_evidence/video${VIDEO}/rosbag" \
  /fleet/goal /fleet/status /fleet/markers /fleet/cargo_result \
  /agv1/amcl_pose /agv2/amcl_pose \
  /agv1/scan_filtered /agv2/scan_filtered /rosout
```

Bấm Ctrl-C ở terminal bag sau khi xe dừng để file ghi được đóng đúng cách.
Ghi màn hình bằng công cụ sẵn có hoặc `ffmpeg` trong **terminal F** mới cho
từng video (bấm `q` để kết thúc MP4):

```bash
VIDEO=01
ffmpeg -f x11grab -video_size 1920x1080 -framerate 15 -i "$DISPLAY" \
  -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
  "$HOME/nckh_evidence/video${VIDEO}/screen.mp4"
```

Nếu đổi độ phân giải màn hình, sửa `-video_size` cho đúng. Video điện thoại là
bằng chứng chuyển động thật, bag là dữ liệu gốc để tính thời gian, sai số và
sự kiện lập lại đường.
Các lệnh **chuẩn bị** giữa clip có thể ghi riêng hoặc bỏ khỏi clip minh
chứng, nhưng phải hoàn tất trước clip tiếp theo.

## Video 1 — Nhập đích tự do và hướng

**Đầu clip:** AGV1 (2,2) Đông, AGV2 (4,4) Tây, lưới trống. **Terminal C**:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
python3 traffic_manager.py agv1 agv2
```

Mở E/F với `VIDEO=01`, bắt đầu quay. Nhập **từng dòng riêng**, không dán
nhiều giá trị vào một prompt:

| Prompt | Gõ rồi Enter |
| --- | --- |
| Xe 1 - Hàng đích | `2` |
| Xe 1 - Cột đích | `4` |
| Xe 1 - Hướng tại đích | `d` |
| Xe 2 - Hàng đích | `4` |
| Xe 2 - Cột đích | `2` |
| Xe 2 - Hướng tại đích | `t` |

Sau dòng thứ sáu hai xe mới nhận lệnh. Quay rõ lệnh nhập, đường trên RViz,
chuyển động và dòng `ĐÃ TỚI`. **Cuối clip:** AGV1 (2,4) Đông, AGV2 (4,2)
Tây. Kiểm tra sai số tâm ô và góc tại đích từ `/fleet/status` hoặc log
`fleet_agent`, đối chiếu bằng đo trên sàn.

**Chuẩn bị video 2:** Khi terminal C hỏi lượt mới, gõ `q` rồi Enter để
thoát chương trình nhập tay. Trong **chính terminal C**, chạy:

```bash
python3 traffic_manager.py agv1 agv2 --no-stdin
```

Giữ terminal C chạy đến hết video 6. Trong **terminal D** (sau ba dòng
`cd/source/export` ở phần chuẩn bị) gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String \
  "{data: 'pair 1 3 2 d ; 2 3 4 t'}"
```

Chờ AGV1 (3,2) Đông và AGV2 (3,4) Tây rồi mới quay video 2.

## Video 2 — Hai xe đối đầu và đổi chỗ

**Đầu clip:** AGV1 (3,2) Đông, AGV2 (3,4) Tây. Mở E/F với `VIDEO=02`.
Trong **terminal D mới** (sau `cd/source/export`) gửi **một lệnh cặp**:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String \
  "{data: 'pair 1 3 4 d ; 2 3 2 t'}"
```

Quay đường dự định trên cùng hàng 3, lúc một xe nhường/đi vòng, rồi cả hai
đến ô ban đầu của xe kia. Không đặt thêm vật cản ở ô né. **Cuối clip:**
AGV1 (3,4) Đông, AGV2 (3,2) Tây. Chỉ tính đạt khi hai xe không chạm nhau
và kết thúc đúng ô/hướng.

**Chuẩn bị video 3:** Giữ AGV1 ở (3,4), đưa AGV2 sang (2,3) Bắc bằng lệnh
sau trong terminal D (hoặc terminal D mới đã source):

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: '2 2 3 b'}"
```

Chờ AGV2 đứng hẳn trước khi quay clip tiếp theo.

## Video 3 — Hai tuyến giao nhau tại (3,3)

**Đầu clip:** AGV1 (3,4) Đông, AGV2 (2,3) Bắc. Mở E/F với `VIDEO=03`.
Trong **terminal D mới** (sau `cd/source/export`) gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String \
  "{data: 'pair 1 3 2 t ; 2 4 3 b'}"
```

Hai đường ngắn nhất cùng đi qua (3,3). Quay rõ một xe chờ/đi vòng theo điều
phối, không cùng chiếm giao điểm; cuối clip AGV1 (3,2) Tây, AGV2 (4,3)
Bắc. Nếu bộ điều phối chọn đường vòng ngay từ đầu, ghi lại đúng diễn biến đó; không
đổi tên thành “chờ tại giao điểm”.

**Chuẩn bị video 4:** Khi lưới còn trống, trong terminal D gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String \
  "{data: 'pair 1 3 1 d ; 2 5 5 d'}"
```

Chờ AGV1 (3,1) Đông và AGV2 (5,5) Đông, rồi mới đặt vật cản.

## Video 4 — Né vật cản tĩnh

**Đầu clip:** AGV1 (3,1) Đông; AGV2 đỗ ở (5,5). Đặt một tấm bìa mờ,
đứng vững, rộng khoảng 40 cm và cao hơn tia LiDAR tại **tâm ô (3,3)** trước
khi giao lệnh. Xác nhận tấm bìa hiện trên LaserScan/costmap. Mở E/F với
`VIDEO=04`. Trong **terminal D mới** (sau `cd/source/export`) gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: '1 3 5 d'}"
```

Quay thấy đường thẳng bị chặn, đường mới tránh qua hàng 2 hoặc 4 và AGV1
đến **(3,5) Đông**. AGV2 vẫn đứng yên. Đặt tấm bìa sao cho không đổ vào xe.

**Chuẩn bị video 5:** Nhấc **vật cản** khỏi lưới (không nhấc xe), chờ scan/
costmap không còn coi (3,3) là bị chắn. Trong terminal D gửi AGV1 về điểm
đầu video 5:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: '1 3 1 d'}"
```

Chờ AGV1 đứng hẳn ở (3,1) Đông, AGV2 vẫn ở (5,5).

## Video 5 — Vật cản động băng ngang rồi biến mất

**Đầu clip:** AGV1 (3,1) Đông; AGV2 (5,5). Ban đầu hàng 3 **trống**. Mở
E/F với `VIDEO=05`. Trong **terminal D mới** (sau `cd/source/export`) gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String "{data: '1 3 5 d'}"
```

Khi AGV1 còn cách khu vực (3,3) ít nhất một ô, cho tấm bìa mờ lớn đi
**ngang qua đường trước mũi xe**, giữ đủ lâu để LiDAR/costmap phát hiện,
rồi kéo hẳn ra ngoài lưới. Người điều khiển tấm bìa đứng ngoài đường xe;
không đi cắt sát mũi xe. Quay cả ba pha: đường thông → xe dừng hoặc lập
đường vòng khi bị chắn → vật cản biến mất và xe đi tiếp tới **(3,5) Đông**.
Ghi thời điểm vật cản vào/ra để đối chiếu `blocked_nodes`, `blocked_edges`
trong `/fleet/status` và log điều phối. Nếu không có pha phát hiện, ghi lần
thử chưa đạt, không trình bày như né thành công.

**Chuẩn bị video 6:** Bỏ vật cản khỏi lưới, chờ costmap sạch. Trong terminal
D gửi:

```bash
ros2 topic pub --once /fleet/goal std_msgs/msg/String \
  "{data: 'pair 1 3 2 d ; 2 4 2 d'}"
```

Chờ AGV1 (3,2) Đông và AGV2 (4,2) Đông.

## Video 6 — Đo, phân loại, giao hàng và báo thành công

**Đầu clip:** AGV1 (3,2) Đông; AGV2 (4,2) Đông. Đặt hộp trước ô lấy
(3,5) và (4,5), mặt hộp cách tâm ô lấy khoảng **35 cm** về phía Đông,
kiểm tra costmap cho phép xe dừng tại ô lấy. Mở E/F với `VIDEO=06`.
**Terminal G — camera**, chạy trực tiếp:

```bash
cd /home/tai/Robot_kho_clone
source tools/setup_real_env.bash
export ROS_DOMAIN_ID=0
python3 tools/cargo_live_view.py --agv1-pickup 3,5 --agv2-pickup 4,5
```

Bấm **BAT DAU** dưới ảnh mỗi xe. Không gửi thêm đích riêng ở terminal D.
Quay đủ chuỗi: xe 1 tới ô quan sát (3,4), xe 2 tới (4,4), cùng hướng Đông;
chốt 5 ảnh và phân loại; tự tiến tới (3,5)/(4,5); chờ người đặt hàng lên xe
và bấm **DA NHAN HANG**; đi giao; báo **DA GIAO HANG THANH CONG**.
Sau khi dỡ hàng, bấm **DA DO HANG - QUAY LAI LAY TIEP**. Đặt món khác loại
ở lượt tiếp và quay cảnh cùng một xe giao tới ô tương ứng loại mới.
Mỗi xe chỉ chở một món một lúc; mỗi ô giao vẫn dành riêng một loại.
Giao diện giữ mở cho tới khi bấm Q. Chi tiết điều kiện đo và bố trí mới:
[DO_HOP_TRUC_TIEP.md](DO_HOP_TRUC_TIEP.md).

## Sau khi quay xong

Bấm Ctrl-C để đóng bag, `q` để đóng ffmpeg, `q` trong RViz. Bấm Ctrl-C ở
terminal C để đóng bộ điều phối. Nếu không quay tiếp nữa, trong terminal A:

```bash
tools/fleet_real.sh down
```

## Bảng số liệu đi kèm sáu video

Với mỗi lần lặp, ghi: vị trí/hướng xuất phát thực tế, đích nhập, thời gian từ
lệnh đến `done`, trạng thái cuối, sai số vị trí so với tâm ô (cm), sai số góc
so với hướng yêu cầu (độ), số lần dừng/lập đường lại, khoảng cách gần nhất
giữa hai xe (video 2–3). Video 4–5 thêm thời điểm vật cản xuất hiện/biến mất
và thời gian phản ứng. Video 6 thêm kích thước hộp đo bằng thước, trung bình
5 ảnh, sai số từng kích thước, ô phân loại, thời điểm bấm xác nhận, thời điểm
đến ô giao. Chỉ ghi **thành công** khi log và video đều cho thấy đủ điều kiện;
nếu thiếu dữ liệu thì đánh dấu “chưa xác minh”.
