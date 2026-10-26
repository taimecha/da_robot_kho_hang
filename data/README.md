# Dữ liệu thử nghiệm

- [Hai xe thật ngày 26/10/2026 — real_trial_03](real_trial_03/README.md): rosbag, nhật ký lấy/giao hàng, kết quả CSV và log điều hướng.
- [Mô phỏng hai xe với thùng 12,1×24 và 18,5×32 cm](sim_trial_01_20261026/README.md): ảnh tự chụp, rosbag, CSV và biểu đồ của lượt hoàn thành.

Lưu các lượt xe thật mới trong `data/real_trial_04/`, `data/real_trial_05/`, v.v.
Mỗi lượt dùng một thư mục mới để giữ nguyên dữ liệu đã thu. Từ thư mục gốc dự án:

```bash
RUN="$PWD/data/real_trial_04"
mkdir -p "$RUN"
```

Khi khởi động xe, đặt `LOGDIR="$RUN/fleet"` cùng lệnh `tools/fleet_real.sh up`.
Khi mở giao diện hàng, đặt `--save-dir "$RUN/images" --event-log "$RUN/cargo_events.jsonl"`.
Khi ghi rosbag, dùng `--max-bag-size 47185920 -o "$RUN/rosbag"` cùng danh sách topic cần ghi.
ROS sẽ tự chia dữ liệu thành nhiều file MCAP, vẫn đọc cả lượt bằng thư mục `rosbag/`.

GitHub chặn file Git thường lớn hơn 100 MiB; dữ liệu ở đây dùng các file MCAP dưới 50 MiB.
Tham khảo [giới hạn file của GitHub](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).
