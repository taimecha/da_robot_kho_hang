# Robot_kho

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
