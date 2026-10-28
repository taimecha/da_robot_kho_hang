# Các lượt mô phỏng bổ sung

| Thư mục | Nội dung ghi nhận | Trạng thái |
|---|---|---|
| [sim_2xe_demo](sim_2xe_demo/) | Hai xe lấy–giao, 14 sự kiện trong nhật ký | Mỗi xe có sự kiện tới ô giao và xác nhận dỡ hàng |
| [warehouse_demo_20261027_104104](warehouse_demo_20261027_104104/) | Ba xe trong kho, 5 kiện đã giao | Lượt dừng trước khi hoàn tất; `complete=false`, không dùng làm bằng chứng hoàn tất 30 kiện |
| [warehouse_demo_20261027_105246](warehouse_demo_20261027_105246/) | Ba xe trong kho, mỗi xe giao 10 kiện | Hoàn tất 30 kiện theo `summary.json`; `complete=true`, `failed=false` |

## Đọc dữ liệu

- `events.jsonl`: sự kiện nhiệm vụ của mô phỏng kho.
- `cargo_events.jsonl`: sự kiện lấy–giao của mô phỏng hai xe.
- `summary.json`, `audit.json`, `trajectories.csv`: tổng hợp, kiểm tra và quỹ đạo nếu lượt đó có xuất các tệp này.
- Các ảnh camera, bản đồ, cấu hình và log trong từng thư mục thuộc chính lượt đó.

Đây là dữ liệu mô phỏng; giữ riêng với dữ liệu xe thật trong [real_trial_03](real_trial_03/README.md). Trạng thái hoàn tất được đối chiếu từ nhật ký, không suy ra từ tốc độ phát video. Phép mô phỏng ba xe dùng mô hình động học, Nav2 và cảm biến tổng hợp, không phải mô phỏng đầy đủ động lực học tiếp xúc.

[Video quay màn hình kho ở tốc độ 3×](../docs/video_mo_phong_3xe_3x.mp4) là bản tăng tốc của tệp quay màn hình người dùng cung cấp. Các bản ghi sự kiện và quỹ đạo vẫn giữ nguyên thời gian gốc.

Thư mục `rviz_resources/` là bản sao thư viện hệ thống được công cụ mô phỏng tự tạo để hiển thị RViz, được bỏ qua khi commit. Chạy lại mô phỏng sẽ tạo lại thư mục này.
