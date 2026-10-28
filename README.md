# Robot_kho

**Dành cho người sửa báo cáo:** [hướng dẫn cập nhật theo số liệu xe thật và mô phỏng mới](docs/HUONG_DAN_CAP_NHAT_BAO_CAO_2_XE_2026-10-26.md).

## Báo cáo và video

- [Bản báo cáo Word hiện tại](docs/Bao_cao_NCKH.docx).
- [Bộ sáu video demo tốc độ 2×](docs/video_demo_2x/README.md), gồm Video 6 ghép cảnh thực tế, RViz và hai camera.
- [Video mô phỏng ba xe trong kho, tốc độ 3×](docs/video_mo_phong_3xe_3x.mp4).
- [Ảnh minh chứng Video 6 và thông tin nguồn](docs/minh_chung_video_6/).
- [Đề xuất kiểm chứng mục tiêu 95% và phản hồi 2 giây](docs/QUY_UOC_KIEM_CHUNG_95_PHAN_TRAM_VA_2_GIAY.md).
- [Chỉ mục các lượt mô phỏng bổ sung](data/README_cac_luot_mo_phong_bo_sung.md).

Các video được lưu trực tiếp trong kho mã; trên GitHub có thể cần tải tệp MP4 về để xem.

## Dữ liệu thử nghiệm

Dữ liệu xe thật: [thử nghiệm hai xe ngày 26/10/2026](data/real_trial_03/README.md),
gồm rosbag, nhật ký lấy/giao hàng, bảng CSV và log điều hướng.
Mô phỏng hai xe theo cùng kích thước thùng: [kết quả, ảnh chụp và dữ liệu](data/sim_trial_01_20261026/README.md).

Mô phỏng 3 xe với 3 hàng đợi FIFO riêng, camera–LiDAR phân loại kiện đầu dãy
và cơ cấu cấp hàng 3D: [hướng dẫn chạy và phạm vi kiểm chứng](docs/MO_PHONG_3_XE_HANG_DOI.md).

```bash
./tools/warehouse_sim.sh up
```

Kịch bản hiện tại **30 kiện**, ba xe chạy độc lập:
`./tools/warehouse_sim.sh up --scenario demo30`.
Mỗi xe có một dãy FIFO gồm 10 kiện; lấy xong thì giao, giao xong quay về lấy
kiện tiếp theo, không đợi đủ đội và không về điểm xuất phát giữa các lượt.
Xe đi thẳng tới ô giao trong một lộ trình, quay về theo đường ngắn phù hợp
với góc quay và giữ chỗ. Băng tải nhận đưa hàng ra khỏi trạm, không xếp tháp.

Lịch **15 kiện** cũ: `./tools/warehouse_sim.sh up --scenario demo15`.
Chỉ thêm `--sync-rounds` khi cần cố ý đồng bộ các nhóm cho một phép thử riêng.

Bấm Space trong giao diện camera để bắt đầu. Dừng bằng Ctrl+C hoặc
`./tools/warehouse_sim.sh down`. Mô phỏng dùng ROS domain 88, không cần xe thật.
Lệnh tự mở thêm RViz ba xe và cảnh nhà xưởng trong Gazebo. Cơ cấu cấp kiện
nâng bàn, mở cầu nối, đẩy rồi hạ thanh đẩy trước khi đưa kiện sau lên.
Thêm `--no-rviz` để không mở RViz.

Các xe cùng loại hàng cùng đi tới **cột của ô giao**, nối đuôi trên một đường
thẳng rồi lần lượt tiến vào trạm. Ví dụ T1 giao (1,1): nhập cột 1 tại các hàng
2, 3, 4; xe đầu giao xong chọn đường rời trạm phù hợp để xe sau tiến lên. Mỗi xe
giữ nguyên một đích ô giao; bộ điều phối mở dần quyền đi tiếp trên cùng lộ
trình, không giao các đích chờ rời rạc ở cột khác. Hàng đã nạp mới được chở
đi; không có rào đợi cả ba xe lấy xong.

Đường về lấy hàng được tìm thẳng tới ô quan sát, không bắt buộc rẽ Đông hay
qua một ô trung gian. Chi phí mô phỏng tính thời gian đi, xoay và chờ ước
lượng; xét cả hướng cuối để camera quan sát. Cột đang có xe khác đi vào được
bảo vệ khỏi xe quay về ngược chiều. Các trọng số là tham số lập kế hoạch,
không phải số liệu thực nghiệm hoặc chứng minh tối ưu toàn cục.

[Hình và log kiểm chứng nối đuôi, đường về linh hoạt: 30/30 kiện](docs/warehouse_column_validation/README.md).

## Quét bản đồ và tự định vị trong mô phỏng

```bash
./tools/warehouse_sim.sh up --scenario demo30 --lidar-map
```

Chế độ `--lidar-map` cho một xe ảo quét phòng bằng LiDAR, dùng `slam_toolbox`
tạo `map.pgm`/`map.yaml`, đặt lưới 5×5 theo khoảng cách đã đo từ tường Tây và
Nam, rồi chạy ba AMCL trên bản đồ vừa lưu. Sau khi hiện `Ready`, bấm Space để
bắt đầu. Quy tắc hàng chờ và giữ chỗ vẫn áp dụng. Các file
`mapping.json`, `grid_alignment.json`, `localization.csv` và báo cáo nằm trong
thư mục log mà lệnh in ra.

Bộ điều phối dùng chung cho xe thật và mô phỏng kiểm tra lại đường vòng khi xe
dừng ở một ô. Nếu xe khác đã đi qua và tuyến mới thông, xe nhận tuyến ngắn hơn
khi thời gian đi và xoay ước tính giảm ít nhất 1 giây; quyền giữ ô cũ chỉ được
nhả sau khi xe xác nhận tuyến mới.

LiDAR không nhìn thấy vạch sơn dưới sàn, nên phải biết khoảng cách từ tường
đến hàng/cột đầu tiên để đặt lưới. Chế độ này vẫn dùng xe động học và odometry
ảo gần lý tưởng; kết quả AMCL không thay thế phép thử định vị trên xe thật.
Riêng mô phỏng AMCL nhận điểm dừng trong phạm vi 3 cm trên bản đồ và kiểm tra
riêng sai số vị trí vật lý khi lấy/giao hàng; cấu hình xe thật vẫn giữ 2 cm.
