# Bộ video demo hai AGV — tốc độ 2×

Các tệp MP4 được xuất ở 1920 × 1080, 30 hình/giây, H.264 để mở trên trình phát thông dụng.

| Video | Nội dung | Độ dài bản xuất |
|---|---|---:|
| [Video 1](video_1_nhap_dich_2x.mp4) | Nhập đích cho hai xe | 18 s |
| [Video 2](video_2_doi_dau_doi_cho_2x.mp4) | Hai xe đối đầu, đổi chỗ | 40 s |
| [Video 3](video_3_giao_nhau_3_3_2x.mp4) | Hai tuyến giao nhau tại ô (3,3) | 17 s |
| [Video 4](video_4_ne_vat_can_tinh_2x.mp4) | Né vật cản tĩnh | 25 s |
| [Video 5](video_5_ne_vat_can_dong_2x.mp4) | Né vật cản động | 28 s |
| [Video 6](video_6_thuc_te_rviz_camera_2x.mp4) | Thực tế + RViz + camera của hai xe | 60 s |

## Bố trí Video 6

- Bên trái: toàn cảnh quay thực tế, không che hai xe bằng lớp giao diện.
- Phía trên bên phải: sa bàn RViz có nhãn các giao điểm, vị trí và đường đi của hai xe.
- Phía dưới bên phải: cả hai khung camera, số đo và trạng thái gốc của giao diện, hiển thị xuyên suốt.
- Số đo lớn bên trái chỉ xuất hiện sau khi giao diện nguồn đã chốt phép đo. Các giá trị giữ đúng cách hiển thị trong video nguồn: AGV1 19,6 × 31,7 cm; AGV2 12,3 × 23,9 cm.
- Nội dung chuyển động chạy 2×. Khung cuối giữ thêm 2 s để đọc kết quả và quan sát hai xe đã tới ô giao.
- Âm thanh thực tế được tăng tốc bằng bộ lọc giữ cao độ. Video 1–5 nguồn không có âm thanh.

[Ảnh xem trước lúc đo hàng](xem_truoc_video_6.jpg) · [Ảnh xem trước khi tới ô giao](xem_truoc_cuoi_video_6.jpg)

## Nguồn và cách căn hai bản quay

Video 6 dùng cảnh thực tế từ `7244486068532648805.mp4` và màn hình Ubuntu từ `Screencast from 2026-10-26 16-09-05.webm` do người dùng cung cấp. Căn theo quan hệ:

```text
thời điểm trên bản quay màn hình ≈ thời điểm trên cảnh thực tế + 9,6 s
```

Độ lệch được đối chiếu bằng chuyển động đầu lượt, lúc tới ô quan sát, lượt quay về và vị trí cuối. Đây là cách căn khi dựng video, không phải phép đồng bộ phần cứng. Giữ trọn nội dung chuyển động của cảnh thực tế, cắt phần màn hình tương ứng.

Video 1–5 được tăng tốc 2× so với đúng các tệp đã gửi. Mọi phần chèn sẵn, kể cả phần RViz ở cuối Video 5, cũng được tăng tốc cùng tệp.

## Dựng lại

Từ thư mục gốc dự án, chạy công cụ [build_demo_videos_2x.py](../../tools/build_demo_videos_2x.py):

```bash
python3 tools/build_demo_videos_2x.py
```

Có thể chỉ dựng một số video, ví dụ:

```bash
python3 tools/build_demo_videos_2x.py --only 6 --jobs 1
```

Cần FFmpeg, Python 3 và font DejaVu Sans. Chương trình có các tùy chọn `--source-dir`, `--real6`, `--screen6`, `--screen-offset` và `--output` để đổi đường dẫn nguồn hoặc nơi xuất. Lệnh dựng sẽ ghi lại bản xuất cùng tên.

Thư mục `_nguon/` lưu lệnh FFmpeg, vùng cắt, tốc độ, độ dài và SHA-256 của nguồn/bản xuất để đối chiếu. `kiem_tra.json` lưu kết quả kiểm tra giải mã và thông số bản xuất.
