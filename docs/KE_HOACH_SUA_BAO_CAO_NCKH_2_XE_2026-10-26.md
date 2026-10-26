# Kế hoạch cập nhật báo cáo NCKH cho 2 xe — bản để đọc trước

Ngày lập: 26/10/2026. Nguồn đối chiếu chính: `/home/tai/Downloads/Báo cáo NCKH-2026.docx`, mã và cấu hình trong workspace này. **Đã có dữ liệu xe thật và mô phỏng mới; chưa sửa DOCX.** Xem [hướng dẫn cập nhật số liệu mới](HUONG_DAN_CAP_NHAT_BAO_CAO_2_XE_2026-10-26.md) trước khi dùng tài liệu này. Các mục kế hoạch và bảng mô phỏng cũ bên dưới là tài liệu lịch sử, không thay thế kết quả đối chiếu mới trong `data/`.

## 1. Cách làm và phạm vi

1. Giữ bố cục chính của báo cáo, lập bản thảo **một file văn bản/Markdown riêng** gồm đoạn thay thế, bảng số liệu, chú thích hình và nguồn của từng con số. Đồng bộ cả phần tóm tắt tiếng Việt, tiếng Anh, Chương 2, Chương 3 và kết luận.
2. Hệ thống được báo cáo là **hai AGV thật**. Mô phỏng để đưa vào báo cáo cũng phải có đúng `agv1` và `agv2`; kết quả ba xe/ba băng tải trong `docs/warehouse_*` không được đổi tên thành kết quả hai xe.
3. Tách ba mức chứng cứ: **(A) cấu hình/mã đang chạy**, **(B) kết quả đã có log/ảnh của một lượt**, **(C) kết quả thực nghiệm mới cần đo**. Một lượt quan sát không trở thành tỉ lệ thành công hoặc sai số thống kê.
4. Đã thực nghiệm hai xe và lưu dữ liệu, gồm cả lần thử lỗi và lượt đối chiếu cuối. Tiếp tục đo bổ sung theo các mục còn thiếu; không sửa trực tiếp DOCX gốc trong bước bàn giao này.

## 2. Các mục trong DOCX phải sửa

| Vị trí | Vấn đề đang có | Cách viết lại |
|---|---|---|
| Thông tin kết quả nghiên cứu, Abstract, §3.1, Kết luận | Nêu sai số 0,4–1,6 cm, ≤2,8°, sáu video đều đạt và đo hàng sai lệch 0–10 mm như kết luận chung | Chỉ giữ khi có dữ liệu gốc tương ứng phiên bản mới; phân biệt sai số AMCL với sai số tâm xe đo bằng thước trên sàn. Nếu chưa đo, ghi rõ chưa đánh giá. |
| §2.3.3, §3.1.1, phần LiDAR | Mô tả X2 và tần số quét cũ, chưa có nguyên nhân mất dữ liệu xe 1 | Bổ sung dạng gói thật **3 byte/mẫu** (1 byte cường độ + 2 byte khoảng cách), checksum 16 bit, góc, bộ giải mã mới chỉ cho xe 1. Xe 2 giữ bộ giải mã cũ cho đến khi đo UART riêng. |
| §2.7.1–2.7.4, Hình 2.19–2.21, Bảng 2.9 | Luồng `cargo_task.py` tự đo tại ô lấy, một ảnh, tự giao; phân loại theo rộng và cao; mặt hộp cách ô lấy 85 cm | Viết theo `cargo_live_view.py`: quan sát trước ô lấy một bước; chốt **5 ảnh hợp lệ**; tới ô lấy; người giám sát đặt đúng kiện và xác nhận; tới ô giao; người giám sát dỡ kiện và bấm xác nhận trước khi bắt đầu lượt mới. Mốc xác nhận dỡ hàng đã được thêm vào nhật ký sự kiện; giao vật lý vẫn cần video kiểm chứng. Mặt hộp gợi ý cách tâm ô lấy **35 cm**, nên cách tâm ô quan sát khoảng **95 cm**. `DIST` tính từ camera, không phải từ tâm ô. |
| Bảng 2.8 và công thức §2.7.3 | Cùng một vị trí camera x=96,7 mm và góc lắp cũ cho hai xe | Dùng cấu hình riêng: xe 1 x=96,7 mm, yaw −2,14°, pitch +1,97°; xe 2 x=139 mm, yaw +2,71°, pitch +0,74°. Các giá trị này là **cấu hình hiệu chuẩn**, cần kiểm tra lại nếu camera đã bị dịch chuyển. |
| Bảng 2.9, §3.1.5, Bảng 3.6–3.7 | Hộp 31–32 cm khi thì về (1,2), khi thì (1,3)/(1,4); bảng chia loại rộng×cao không khớp code | Nói rõ **chiều cao quyết định ô giao**, bề rộng là kết quả đo/kiểm tra hình học. Các dải: 24–<30,5 cm → (1,1); 30,5–<37 → (1,2); 37–<43,5 → (1,3); 43,5–50 → (1,4). Gần ngưỡng ±0,5 cm cần người giám sát xem lại; ca 23,9 cm từng được chấp nhận nhờ dung sai 2 mm ở ngưỡng dưới, không đổi định nghĩa dải chuẩn. |
| §2.8.1, Bảng 2.10, §3.1.4 | Bốn ca mô phỏng hai xe đã cũ; phần khác của dự án hiện chạy mô phỏng ba xe/ba hàng đợi | Chạy lại các ca **hai xe** bằng mã hiện tại, đồng thời thử luồng lấy–giao phù hợp sa bàn thật. Ghi rõ xe ảo động học, LiDAR/camera tổng hợp, điều khiển Nav2 thật; Gazebo là hiển thị nếu không mô phỏng va chạm/động lực học đầy đủ. |
| Bảng 3.2–3.7, §3.2, Hạn chế | Trộn số thử cũ, ảnh tĩnh, dự kiến video và kết quả đã đo | Thay bằng bảng truy vết từng lượt: thời gian, xe, firmware/cấu hình, vị trí đầu/đích, số đo, kết quả, đường dẫn bag/log/video. Ảnh giao diện `delivered` chỉ xác nhận trạng thái phần mềm; giao vật lý cần video hoặc quan sát kiện được đặt lên và dỡ khỏi xe. |

Những đoạn phụ thuộc các số cũ (tốc độ, nguồn cấp, sai số bản đồ, mốc video) sẽ được rà tiếp ở toàn văn, không chỉ các mục trên. `docs/RA_SOAT_BAO_CAO_NCKH.md` là danh sách lỗi trước đó để đối chiếu, không tự động coi mọi kết luận trong đó là số liệu mới.

## 3. Câu chữ phương pháp có thể viết trước khi đo thêm

**LiDAR xe 1.** “LiDAR của xe 1 truyền gói UART 115200 bit/s, mỗi mẫu gồm một byte cường độ và hai byte khoảng cách. Bộ giải mã kiểm tra đồng bộ, số mẫu, góc đầu/cuối và checksum trước khi nội suy góc từng điểm, hiệu chỉnh góc phụ thuộc khoảng cách và ghép vòng quét. Firmware đưa điểm hợp lệ về `LaserScan` gồm 320 ô góc, mỗi ô 1,125°. Trường cường độ hiện chưa được xuất lên ROS. Bộ giải mã này chỉ được áp dụng cho xe 1; xe 2 vẫn dùng bộ giải mã đã có vì chưa đo lại định dạng gói của xe 2.” Nguồn: `firmware/esp32_agv_base/ydlidar_x2_intensity.h`, `esp32_agv_base.ino`.

**Lấy và giao hàng.** “Sau khi người vận hành bấm bắt đầu, xe đi tới ô quan sát ngay trước ô lấy hàng, quay về phía Đông và dừng. Giao diện kết hợp mặt trước hộp từ LiDAR với mép hộp trên ảnh camera, chốt trung bình năm phép đo hợp lệ, kiểm tra độ ổn định và xác định ô giao theo chiều cao. Xe đi tới ô lấy nhưng chỉ xuất phát giao sau khi người vận hành đặt đúng kiện lên xe và bấm xác nhận. Khi xe dừng đúng ô giao, giao diện phát sự kiện đến đích; người vận hành dỡ kiện rồi bấm xác nhận dỡ. Xe chỉ bắt đầu lượt mới sau một lần bấm quay lại riêng.” Nguồn: `tools/cargo_live_view.py`, `src/my_bot_bringup/config/cargo.yaml`.

**Giới hạn của mô phỏng.** “Hai xe ảo nhận lệnh từ các thành phần điều hướng/điều phối của dự án; vị trí và chuyển động của chúng do mô hình động học tính. Camera và LiDAR ảo sinh từ cảnh tổng hợp. Kết quả này kiểm tra phần mềm với giả thiết đã công bố, không chứng minh sai số định vị, lực va chạm hoặc tỉ lệ giao hàng của xe thật.”

**Thuật toán điều khiển dùng chung đến đâu?** “Trong kịch bản mô phỏng hai xe bằng `tools/fleet_sim.sh` và kịch bản xe thật bằng `tools/fleet_real.sh`, cả hai đều chạy cùng mã `traffic_manager.py` để điều phối, `fleet_agent.py` để bám tuyến/vi chỉnh và Nav2 với cấu hình gốc `navigation.yaml` (bộ bám đường Regulated Pure Pursuit). Nếu chạy mô phỏng đo hàng qua `tools/sim_cargo.py` cùng `tools/cargo_live_view.py`, logic đo, phân loại và gửi ô giao cũng là cùng mã. Khác biệt nằm ở đầu vào và tầng chấp hành: xe ảo nhận `/cmd_vel` rồi tính chuyển động bằng mô hình động học, phát LiDAR/camera tổng hợp và mặc định dùng biến đổi `map→odom` lý tưởng; xe thật nhận `/cmd_vel` tại ESP32, điều khiển tốc độ bánh bằng PID, lấy odometry/IMU/LiDAR/camera thật và AMCL để định vị. Lượt mô phỏng mới dùng độ trễ 0,05 s và quán tính 0,12 s; cấu hình xoay được sinh từ `grid_real.yaml`. Dữ liệu scan và cách xử lý tia vô cực khác xe thật. Vì vậy có thể nói **dùng chung thuật toán điều phối và điều hướng cấp cao**, không nói toàn bộ vòng điều khiển và điều kiện cảm biến là giống hệt. Thời gian và sai số giữa hai môi trường phải đo riêng.” Nguồn: `tools/fleet_sim.sh`, `tools/fleet_real.sh`, `tools/fleet_fake_robot.py`, `tools/fleet_real_robot.launch.py`, `src/my_bot_bringup/config/navigation.yaml`, firmware ESP32.

**Định vị ban đầu của xe thật.** “Trong phiên mới, người vận hành xác nhận AGV1 tại (5,2), AGV2 tại (2,3), cùng hướng Đông. Hệ được khởi tạo theo ô khai báo rồi dùng LiDAR khớp bản đồ trong vùng quanh ô bằng `--hint-cell` và `--hint-heading`. Tìm toàn cục từng bị từ chối do các vị trí gần đối xứng; kết quả hiện tại là định vị có gợi ý, không phải tự định vị toàn cục không có thông tin ban đầu.” Nguồn: `tools/auto_initial_pose.py`, `tools/fleet_real.sh` và nhật ký dữ liệu xe thật.

## 4. Chứng cứ đã có và điều không được suy ra

| Chứng cứ hiện có | Có thể ghi | Chưa thể kết luận |
|---|---|---|
| Probe LiDAR xe 1 ngày 25/10: khoảng 9,8 kB/s UART; gói 3 byte có checksum hợp lệ liên tục; parser mới tạo khoảng 6 vòng/s, ROS nhận `LaserScan` 320 ô góc và nhiều khoảng cách hợp lệ | Bộ giải mã đọc được dữ liệu thật của xe 1 và xuất được `/agv1/scan` trong lượt kiểm tra đó | Độ chính xác đo khoảng cách/góc, độ ổn định dài hạn, chất lượng AMCL khi hai xe chạy |
| Giao diện xe 1 ngày 25/10: báo rộng 14,9 cm, cao 31,7 cm; chọn ô (1,2) và hiện “ĐÃ GIAO HÀNG THÀNH CÔNG” | Một lượt phần mềm đã đo, phân loại và báo tới đích; 31,7 cm thuộc dải (1,2) | Sai số so với thước vì chưa có số tham chiếu cùng kiện; chưa tự coi đây là bằng chứng kiện đã được giao vật lý; ảnh chụp `/tmp` không phải hồ sơ lưu trữ lâu dài |
| `docs/PHAN_MO_PHONG_NCKH.md` và hình cũ | Bốn lượt mô phỏng hai xe trước đây có mô tả và số liệu riêng, mỗi ca một lần | Không đại diện cho mã/cấu hình hiện tại; không gộp với ba xe hay với kết quả xe thật |
| `docs/nckh_2xe_sim_20261026/final/` | Một phiên mô phỏng hiện tại có 2 xe, mỗi xe hoàn thành 2 chu trình lấy–giao theo log sự kiện. Xe 1: 66,773 s và 94,757 s; xe 2: 69,165 s và 97,144 s từ lệnh bắt đầu đến lúc phần mềm báo tới ô giao. Cả đợt hai xe: 69,301 s và 97,285 s. | Đây là 4 chu trình trong **một phiên** với cảm biến và xe ảo, thao tác bấm giả lập; lượt 2 bao gồm chặng quay về ô quan sát từ ô giao lượt 1 nên khác điều kiện xuất phát lượt 1. Chưa phải tỉ lệ thành công qua nhiều phiên, thời gian xe thật hay chứng cứ dỡ hàng vật lý. Các ca giao cắt/vật cản vẫn cần log riêng. |
| `data/real_trial_03/` | Lượt đối chiếu cuối có hai xe tới ô giao và xác nhận dỡ; tổng tới ô giao AGV1 102,209 s, AGV2 113,744 s; cả đợt 114,604 s | Chưa có video ngoài hiện trường hoặc phép đo độc lập vị trí/góc trên sàn; một lượt mỗi xe chưa đủ đánh giá thống kê |
| `data/sim_trial_01_20261026/` | Hai thùng 18,5×32 cm và 12,1×24 cm; mỗi xe hoàn thành một lượt; tổng tới ô giao AGV1 78,603 s, AGV2 93,460 s; cả đợt 93,590 s; có 60 ảnh PNG và rosbag | Định vị lý tưởng, chiều sâu giả định 12 cm; mô phỏng động học, không phải thử nghiệm xe thật |
| `docs/warehouse_*` gần đây | Minh chứng phần mềm mô phỏng **ba xe** hoạt động trong điều kiện ghi tại từng thư mục | Không được chép số 30/30 kiện, khoảng cách tối thiểu hay thời gian ba xe vào báo cáo **hai xe** |

## 5. Kế hoạch mô phỏng ban đầu và các ca cần bổ sung

1. Dùng miền ROS riêng, không nối micro-ROS xe thật. Bổ sung các tình huống đổi chỗ, giao cắt, vật cản tĩnh, vật cản xuất hiện rồi biến mất với đúng **hai xe** và bản đồ/lưới 5×5. Ghi `rosbag`, `/fleet/status`, lệnh giao đích, đường đi, thời gian, khoảng cách tâm ước lượng và cả lần thất bại. Lặp **ít nhất 3 lần/tình huống** với vị trí và điều kiện được ghi rõ.
2. Đã có một phiên kiểm tra chức năng hàng bằng `tools/sim_cargo.py` và **cùng `cargo_live_view.py --sim`** đang dùng cho xe thật; tiếp tục lặp lại để đánh giá độ ổn định. Cờ `--sim` chọn hiệu chuẩn camera ảo và vị trí LiDAR ảo (x=0, z=7,5 cm) đúng với bộ dựng cảm biến: ô quan sát → 5 ảnh → ô lấy → xác nhận chất hàng → ô giao → ghi nhận dỡ hàng. Bố trí đúng hai xe bắt đầu tại **(2,3)** và **(3,3)**, cùng quay Đông; hai ô lấy là (3,5) và (4,5); dùng cùng ngưỡng chiều cao **24–50 cm**. Nếu dùng `tools/warehouse_sim.sh`, trước tiên phải bỏ mọi cấu phần `agv3` và băng tải/cấp kiện tự động không có trên xe thật, hoặc mô tả riêng đó là thí nghiệm khác; script hiện còn mặc định **ba xe** và mốc chiều cao **22/29/36/43/50 cm**, nên chưa thể lấy kết quả làm mô phỏng trực tiếp của quy trình đang báo cáo.
3. Mỗi kết quả mô phỏng có `scenario.json`, cấu hình, mã phiên bản/hash, log sự kiện, bảng quỹ đạo, ảnh hoặc video RViz/Gazebo và phép tính lại các chỉ tiêu. Bảng báo cáo ghi số lần hoàn thành/tổng số, thời gian theo từng lượt, quãng đường, thời gian chờ/đổi đường, khoảng cách tâm nhỏ nhất; số đo hộp ảo so với kích thước đầu vào chỉ được gọi là **sai số mô phỏng**.

## 6. Kế hoạch đo bổ sung sau lượt thực nghiệm đã ghi

**Trước chuyển động.** Ghi ngày, phiên bản firmware và mã, pin/nguồn, sơ đồ ô/hướng khởi đầu, vị trí hộp, số đo thước và cấu hình camera/LiDAR. Đứng yên kiểm tra `/agv1/scan`, `/agv2/scan`, timestamp, tần số, số tia hợp lệ, `/scan_filtered`, odometry, IMU, camera và TF. Đặt vật ở hướng/khoảng cách đã đo để kiểm tra **góc lắp và thang khoảng cách của LiDAR xe 1 sau khi đổi parser**; so ảnh quét với bản đồ trước khi tin AMCL. Xe 2 kiểm tra riêng, không suy từ xe 1.

**Chạy đơn lẻ.** Mỗi xe chạy các tuyến ngắn đã chọn trên sa bàn, tối thiểu 3 lượt/tuyến. Tại từng đích ghi vị trí và góc **đo độc lập trên sàn** so với tâm/vạch ô, cùng giá trị AMCL, thời gian, số lần dừng/ESTOP và video. Không dùng chính AMCL làm “sự thật” để chấm sai số AMCL.

**Chạy đồng thời.** Sau khi từng xe đạt kiểm tra đơn lẻ, thử đổi chỗ, giao cắt, vật cản tĩnh và vật cản xuất hiện/rời đi theo thứ tự; tối thiểu 3 lượt/ca nếu điều kiện sa bàn cho phép. Ghi toàn bộ lượt, kể cả thử lỗi. Khoảng cách nhỏ nhất từ `/fleet/status` chỉ là **ước lượng theo định vị**; muốn khẳng định khoảng cách vật lý phải đo từ video nhìn từ trên có thước chuẩn hoặc phép đo ngoài xe khác.

**Lấy và giao hàng.** Dùng hộp có kích thước rộng/cao được đo và chụp bằng thước; thử trên **cả hai xe**, ghi từng kết quả 5 ảnh, tia LiDAR dùng được, khoảng cách camera–mặt hộp, ô phân loại, thao tác đặt/dỡ hàng, thời gian và sự kiện `/fleet/cargo_result`. Ưu tiên có ít nhất một hộp ở giữa **mỗi** dải 24–30,5; 30,5–37; 37–43,5; 43,5–50 cm, mỗi hộp lặp ít nhất 3 lần/xe nếu có đủ hộp. Hộp 23,9 cm nằm ngoài dải chuẩn, chỉ ghi riêng là thử dung sai ngưỡng dưới. Nếu chỉ có hai hộp, giới hạn kết luận vào hai hộp ấy; không công bố độ chính xác cho cả bốn loại. Chỉ ghi “giao vật lý thành công” khi video cho thấy kiện được đặt lên xe, xe mang tới đúng ô và được dỡ ra.

**Dữ liệu gốc.** Mỗi lượt có ID, ngày/giờ, cấu hình, `rosbag` tối thiểu gồm `/agvN/scan`, `/agvN/scan_filtered`, `/agvN/odom`, `/agvN/imu/data`, `/agvN/joint_states`, `/agvN/cmd_vel`, `/fleet/status`, `/fleet/goal`, `/fleet/cargo_result`, cùng log console, ảnh/video và bảng thước. Trước buổi chạy ghi cách định nghĩa mốc bắt đầu/kết thúc. Các bảng tính xuất CSV/JSON từ dữ liệu gốc và lưu công thức/đoạn mã tính.

## 7. Thời gian lấy hàng và giao hàng để so sánh xe thật với mô phỏng

Nhật ký `/fleet/cargo_event` và file JSONL hiện đã ghi các mốc theo **cùng một định nghĩa** cho `agv1` và `agv2`, cả xe thật lẫn xe ảo. Mỗi sự kiện có `run_id`, xe, `cycle`, tên mốc, thời gian đồng hồ thực và thời gian đơn điệu. `tools/analyze_cargo_timing.py` xuất CSV theo công thức dưới đây. Đã có lượt xe thật cuối tại `data/real_trial_03/` và lượt mô phỏng với cùng kích thước hộp tại `data/sim_trial_01_20261026/`, mỗi lượt một chu kỳ cho mỗi xe. Bốn chu trình mô phỏng ở `docs/nckh_2xe_sim_20261026/final/` là kịch bản cũ, có kích thước hộp và vị trí đầu khác; không gộp vào đối chiếu mới.

| Mốc | Định nghĩa khi đo |
|---|---|
| `t0` | Giao diện chấp nhận lệnh **BẮT ĐẦU** và gửi đích ô quan sát. |
| `t1` | Xe dừng hợp lệ ở ô quan sát. |
| `t2` | Chốt trung bình **5 ảnh hợp lệ** và xác định ô giao. |
| `t3` | Xe dừng hợp lệ ở ô lấy. |
| `t4` | Kiện đã được đặt lên xe và người vận hành bấm xác nhận; lệnh tới ô giao được gửi thành công. |
| `t5` | Xe dừng hợp lệ ở đúng ô giao; phát sự kiện `delivered`. Đây là **đến đích của phần mềm**, chưa chứng minh đã dỡ kiện. |
| `t6` | Kiện được dỡ khỏi xe và người vận hành ghi/xác nhận mốc dỡ kiện; kiểm tra bằng video. |

Các chỉ tiêu cho **từng kiện, từng xe** (giây):

- **Lấy hàng đến khi sẵn sàng giao:** `T_lay = t4 − t0`. Tách thêm `T_di_den_o_lay = t3 − t0` và **thời gian chất hàng/chờ người vận hành** `T_cho_chat = t4 − t3`.
- **Giao hàng bằng xe:** `T_giao_xe = t5 − t4`. Đây là thời gian từ khi xác nhận chất xong tới khi xe dừng ở ô giao, gồm cả chờ điều phối và tránh xe kia.
- **Tổng chu trình tới ô giao:** `T_tong_xe = t5 − t0 = T_lay + T_giao_xe`. Đây là con số chính để so sánh hoạt động của xe và phần mềm.
- **Tổng tới lúc giao vật lý xong:** `T_tong_vat_ly = t6 − t0`; phần dỡ hàng `T_do_hang = t6 − t5`. Với mô phỏng, chỉ ghi `t6` nếu có thao tác xác nhận dỡ tương ứng; không gọi xe ảo là đã giao vật lý.
- Ghi thêm thời gian **đi đến ô quan sát** `t1−t0`, **đo hàng** `t2−t1`, **đi tới ô lấy** `t3−t2`; nhờ vậy phân tích được pha nào làm chậm toàn chu trình. Thời gian chờ do người vận hành được tách riêng, không lấy thời gian bấm nhanh trong mô phỏng làm lợi thế của thuật toán.

Để so sánh công bằng, hai môi trường dùng cùng điểm xuất phát, ô lấy/ô giao, loại hộp, số lượt và quy tắc bấm xác nhận. Bảng chính báo **`T_lay`, `T_giao_xe`, `T_tong_xe`** cho từng lượt của mỗi xe và trung vị/khoảng của các lượt; bảng phụ ghi thời gian chờ chất/dỡ, thất bại và lần thử lại. Chỉ tính tỉ số `T_that/T_mo_phong` khi cùng một kịch bản, nêu rõ mô phỏng có thao tác chất hàng bằng tay hay giả lập thời gian thao tác cố định. Với **hai xe chạy đồng thời**, báo thêm thời gian hoàn thành cả đợt `max(t5_agv1, t5_agv2) − min(t0_agv1, t0_agv2)` và số kiện giao thành công trên phút; không cộng hai thời gian xe để gọi là thời gian của cả đợt. Lượt không đến đích được ghi thất bại hoặc quá thời gian, không loại khỏi mẫu.

**Đoạn dự thảo cho phần kết quả sau khi có log:** “Ở mỗi môi trường, nghiên cứu thử đồng thời hai xe với cùng vị trí xuất phát, ô lấy, kích thước hộp và ô giao. Thời gian lấy hàng được tính từ khi nhận lệnh bắt đầu đến khi kiện đã được chất và xác nhận; thời gian giao bằng xe tính từ xác nhận chất đến khi xe dừng đúng ô giao. Tổng thời gian là hiệu giữa mốc bắt đầu và mốc đến ô giao. Thời gian chờ chất/dỡ hàng được trình bày riêng; thời điểm giao vật lý trên xe thật được kiểm tra bằng video. Bảng [điền số bảng] trình bày từng lượt, trung vị và khoảng dao động của mô phỏng và thực tế. Các lượt thất bại được ghi riêng, không loại khỏi mẫu.” Chỉ thay vị trí `[điền số bảng]` và thêm nhận xét định lượng sau khi có CSV/video thật.

**Bảng lịch sử trước lượt đối chiếu mới — dùng bảng mới trong hướng dẫn cập nhật, không dùng dòng “chờ đo” dưới đây làm tình trạng hiện tại:**

| Môi trường | Xe | Lượt | Hộp/ô giao | `T_lay` (s) | `T_cho_chat` (s) | `T_giao_xe` (s) | `T_tong_xe` (s) | `T_tong_vat_ly` (s) | Kết quả/nguồn |
|---|---|---:|---|---:|---:|---:|---:|---:|---|
| Mô phỏng | agv1 | 1 | 15×32 cm → (1,2) | 33,356 | 2,183 | 33,417 | 66,773 | không áp dụng | `final/cargo_events.jsonl` |
| Mô phỏng | agv2 | 1 | 17×40 cm → (1,3) | 34,567 | 2,202 | 34,598 | 69,165 | không áp dụng | `final/cargo_events.jsonl` |
| Mô phỏng | agv1 | 2 | 15×32 cm → (1,2) | 61,380 | 11,423 | 33,377 | 94,757 | không áp dụng | `final/cargo_events.jsonl` |
| Mô phỏng | agv2 | 2 | 17×40 cm → (1,3) | 60,911 | 17,194 | 36,233 | 97,144 | không áp dụng | `final/cargo_events.jsonl` |
| Xe thật | agv1/agv2 | … | … | chờ đo | chờ đo | chờ đo | chờ đo | chờ đo | bag/video/thước |

## 8. Chỉ tiêu và cách trình bày kết quả

- **LiDAR:** số gói có checksum đúng/tổng gói, vòng quét mỗi giây (trung vị và khoảng), tia hợp lệ/vòng, sai khác góc và khoảng cách so với vật đặt mốc, tỉ lệ scan nhận ở ROS. Không so trực tiếp cường độ nếu `PUBLISH_INTENSITIES=0`.
- **Định vị/điều hướng:** sai số tâm đích và góc đích so với mốc đo trên sàn; AMCL–mốc sàn là chỉ tiêu riêng. Thời gian tính từ lệnh được nhận tới lúc xe dừng ổn định; nêu số mẫu, trung vị/khoảng và cả ca thất bại.
- **Điều phối:** hoàn thành/tổng lượt, va chạm hoặc xâm nhập vùng an toàn quan sát được, thời gian chờ, số lần đổi đường/gỡ kẹt, khoảng cách tâm nhỏ nhất theo nguồn đo đã ghi. Không dùng “không va chạm” để suy ra xác suất an toàn chung.
- **Đo/giao hàng:** sai số tuyệt đối rộng/cao so với thước, kết quả phân loại từng kiện, số lượt chốt đủ 5 ảnh/tổng lượt, đúng ô đến và đúng kiện được dỡ. Thời gian `T_lay`, `T_giao_xe`, `T_tong_xe` và thời gian chờ thao tác trình bày theo mục 7. Phân biệt `delivered` của phần mềm với quan sát giao hàng vật lý.

**Sản phẩm bàn giao sau khi có dữ liệu:** một file Markdown hoặc TXT dễ đọc, theo thứ tự mục của DOCX, chứa đoạn văn thay thế và bảng số liệu thật kèm đường dẫn nguồn; một bảng “giữ/sửa/bỏ/chờ đo” cho các số cũ. Không chèn trực tiếp vào Word trong bước này.

Lệnh chạy và đường dẫn file dữ liệu được ghi tại `docs/LENH_THU_NGHIEM_NCKH_2_XE.md`.
