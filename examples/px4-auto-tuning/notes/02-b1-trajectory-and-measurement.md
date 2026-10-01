# 02 — B1: bay quỹ đạo và phép đo

## Quỹ đạo

| Loại | Công thức (NED, tương đối điểm cất cánh) | Mặc định | Đỉnh vận tốc / gia tốc |
|---|---|---|---|
| `figure8` | x = A·sin(ωs), y = B·sin(2ωs) | A=8 m, B=4 m, chu kỳ 20 s | ≈3.6 m/s, ≈1.8 m/s² |
| `sine` | x = A·(1−cos ωs), y = B·sin(kπx/A) — đi rồi về trên cùng đường | A=8 m, B=1.5 m, k=2, chu kỳ 30 s | ≈2.6 m/s, ≈2.5 m/s² |

Tốc độ được ramp mượt (smoothstep) từ hover lên tốc độ đầy và ngược lại, để không
có bước nhảy vận tốc ở đầu/cuối. Node gửi position + velocity + acceleration
(feedforward); `--no-ff` chỉ gửi position.

## Ba loại sai số

| Tên | Định nghĩa | Ý nghĩa |
|---|---|---|
| tracking | setpoint − estimate | cái controller thấy và sửa — gain tác động vào đây |
| estimation | estimate − ground truth | đóng góp của GPS/EKF |
| true | setpoint − ground truth | drone thật sự lệch bao nhiêu so với kế hoạch |

Ground truth của Gazebo ở hệ world, estimate ở hệ local của EKF; hai gốc lệch nhau
một hằng số. Hằng số này lấy từ đoạn hover trước khi bay quỹ đạo, nên "estimation"
đo mức estimate trôi so với thật *trong lúc bay*.

Thêm: `true_crosstrack` (khoảng cách tới điểm gần nhất trên đường tham chiếu, tức
sai số thật đã bỏ thành phần trễ theo thời gian), `*_lag_s` (độ trễ thời gian khớp
nhất), `rate_hf_rms` (rung: angular rate bỏ trung bình trượt 0.5 s),
`motor_saturation`, `sim_max_gap_ms`.

## Những điều phát hiện khi dựng phép đo

1. **Timestamp qua uXRCE-DDS bị đổi sang đồng hồ của agent** (Unix time), còn ulog
   dùng thời gian từ lúc boot. Node đọc `/fmu/out/timesync_status.estimated_offset`
   để ghi mốc sự kiện theo boot time, khớp với ulog.
2. **Mỗi lần arm tạo một file ulog riêng**; `trajectory_setpoint` chỉ được log ở
   5 Hz, `vehicle_local_position` 125 Hz, ground truth 50 Hz. Sai số được tính trên
   mốc thời gian của setpoint (5 Hz).
3. **Gazebo không chờ PX4.** `gz_bridge` lấy thời gian từ topic clock của Gazebo
   nhưng Gazebo vẫn chạy tiếp khi PX4 hoặc máy bị khựng. Khi đó PX4 báo
   `Accel #0 fail: TIMEOUT`, mất mẫu IMU, và drone có thể rơi — hoàn toàn không
   liên quan tới gain. Trên máy này (RAM gần đầy, đang dùng swap) hiện tượng xảy ra
   vài lần mỗi chục chuyến bay.
   → Mọi runner đều kiểm tra `sim_max_gap_ms` và dòng `ERROR` trong log PX4; chuyến
   nào dính thì đánh dấu **invalid**, restart SITL và bay lại. Nếu không có bước
   này, một lần máy khựng sẽ bị hiểu nhầm thành "gain này fail" — đúng kiểu
   "33 chạy, 34 fail, 35 chạy".
4. **Phần lớn sai số là trễ thời gian, không phải lệch đường.** Ở gain mặc định:
   true RMSE ngang ≈0.5 m nhưng cross-track chỉ ≈0.15 m; estimate trễ so với
   setpoint ≈90 ms. Số liệu trễ giữa estimate và ground truth (≈110 ms, estimate
   "đi trước") còn nghi ngờ: có thể do timestamp của ground truth trong GZBridge
   được gán lúc callback chạy chứ không phải lúc Gazebo lấy mẫu. **Chưa kiểm chứng.**
5. **Noise GPS mô phỏng rất nhỏ.** Mô hình Gauss-Markov trong GZBridge với "biên độ
   0.8 m" cho độ lệch chuẩn vị trí chỉ ≈2.2 cm (0.01·0.8/√(1−0.93²)) và vận tốc
   ≈1.9 cm/s, trong khi `eph` báo cho EKF là 0.9 m. Tức là SITL mặc định gần với
   RTK hơn là GPS thường.
