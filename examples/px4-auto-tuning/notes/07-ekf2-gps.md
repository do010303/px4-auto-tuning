# 07 — EKF2: một mẫu GPS trở thành ước lượng vị trí thế nào

Đường dẫn tính từ `src/modules/ekf2`, base commit `583e1cdb47`. Ghi chú này giải thích ba điều đã thấy
trong thí nghiệm: vì sao báo `eph` lớn hơn lại có ích khi GPS nhiễu, vì sao profile `urban` không cất cánh
được, và vì sao tần số GPS ảnh hưởng ít.

## Đường đi của một mẫu

1. `GZBridge` publish `sensor_gps`: vị trí, vận tốc, `eph`, `epv`, `s_variance_m_s`, số vệ tinh.
2. `sensors/vehicle_gps_position` gắn mốc thời gian. Nếu driver không đặt `timestamp_sample`, nó trừ đi
   `SENS_GPS0_DELAY` (`VehicleGPSPosition.cpp:166-169`). Xem `04-gps-delay-mismatch.md`.
3. EKF2 chạy trên một mốc thời gian trễ (`EKF2_DELAY_MAX`, 200 ms). Mẫu nằm trong buffer tới khi mốc trễ
   chạm `timestamp_sample` của nó, nên cảm biến chậm vẫn được fuse đúng thời điểm nó đo, không phải thời
   điểm nó tới. Sau đó output predictor dùng IMU đưa trạng thái về "hiện tại".
4. Các phép kiểm tra chất lượng quyết định có dùng GPS hay không (`EKF/aid_sources/gnss/gnss_checks.cpp`).
5. Vận tốc và vị trí được fuse riêng, mỗi cái có measurement noise và innovation gate riêng
   (`EKF/aid_sources/gnss/gps_control.cpp`).

## Measurement noise: độ chính xác GPS tự báo được dùng, param chỉ là mức sàn

```
vel_var   = max(sacc, EKF2_GPS_V_NOISE, 0.01)^2      gps_control.cpp:316
pos_noise = max(hacc, EKF2_GPS_P_NOISE)              gps_control.cpp:354
```

`hacc` chính là `eph` do bộ thu báo. Hai param (`EKF2_GPS_P_NOISE` 0.5 m, `EKF2_GPS_V_NOISE` 0.3 m/s) là
**mức sàn**, không phải giá trị được dùng: bộ thu báo độ chính xác kém hơn thì tự động bị tin ít hơn.

Hệ quả, đo ở `sweep_SIM_GZ_GPS_EPH_noisy` (noise vị trí SD ≈ 0.5 m): báo `eph` 0.3 m cho true RMSE 0.56 m,
báo 3 m cho 0.41 m. Bộ thu báo sai số thấp hơn thực tế gây hại nhiều hơn bộ thu nhiễu mà báo đúng.

## Innovation gate

`EKF2_GPS_P_GATE` và `EKF2_GPS_V_GATE` (đều 5 SD) loại mẫu có innovation lớn hơn 5 độ lệch chuẩn của
innovation variance. `eph` lớn hơn cũng làm gate rộng ra. Nếu không mẫu nào qua trong `reset_timeout_max`
(7 s, `EKF/common.h:513`), EKF reset vị trí hoặc vận tốc về giá trị GPS (`gps_control.cpp:142-150`,
`200-208`); trong log đó là một bước nhảy của estimate.

## Kiểm tra: chặt khi còn trên mặt đất, lỏng khi đã bay

`GnssChecks::run` (`gnss_checks.cpp:43-84`):

| | Trên mặt đất | Đang bay |
|---|---|---|
| Hàm | `runInitialFixChecks` | `runSimplifiedChecks` |
| Sai số ngang | `eph` ≤ `EKF2_REQ_EPH` (3 m) | `eph` ≤ 50 m |
| Sai số dọc | `epv` ≤ `EKF2_REQ_EPV` (5 m) | `epv` ≤ 50 m |
| Sai số vận tốc | ≤ `EKF2_REQ_SACC` (0.5 m/s) | ≤ 10 m/s |
| Số vệ tinh, PDOP, độ trôi | có kiểm (bitmask `EKF2_GPS_CHECK`) | không kiểm |
| Phải đạt liên tục trong | `EKF2_REQ_GPS_H` (10 s) | một phần mười mức đó, tối thiểu 1 s |

Khi drone chưa bay, bộ kiểm tra ban đầu được kích hoạt lại ở mỗi lần gọi (`_initial_checks_passed = false`),
nên GPS kém trên mặt đất luôn chặn việc fuse.

Hệ quả đã thấy trong thí nghiệm:

- **Profile `urban` (`eph` 6 m) đặt khi còn trên mặt đất: không cất cánh được.** PX4 in
  `Preflight: GPS Horizontal Pos Error too high`; ước lượng vị trí mất hiệu lực, drone rớt khỏi offboard
  khoảng 1 s sau khi arm và blind land. 5/5 lần.
- **Cùng `eph` 6 m nhưng đặt khi đang bay thì được chấp nhận** (`sweep_SIM_GZ_GPS_EPH_noisy` bay không
  fail lần nào), vì lúc đó chỉ còn ngưỡng 50 m.
- Drone cất cánh ngoài trời trống rồi bay vào dưới tán cây vẫn tiếp tục dùng một GPS mà nó sẽ từ chối
  nếu đang ở mặt đất. Ngược lại, drone khởi động dưới tán cây sẽ không arm được ở mode vị trí cho tới
  khi nới `EKF2_REQ_EPH`.

## Vì sao tần số GPS ảnh hưởng ít

Giữa hai mẫu GPS, trạng thái được truyền bằng IMU, mà IMU trong mô phỏng gần như hoàn hảo. Ở 1 Hz, EKF
dead-reckon trong 1 giây và sai số vị trí tăng chậm trong khoảng đó. Đã đo: 30 Hz → 0.256 m, 1 Hz →
0.266 m true RMSE với GPS sạch; 0.422 → 0.476 m với GPS nhiễu. Với IMU thật (bias, rung) cái giá của GPS
chậm dự kiến lớn hơn; chưa kiểm ở đây.

## Chưa đọc

GPS yaw, bộ ước lượng yaw EKF-GSF, chọn nguồn độ cao, và phát hiện lỗi GNSS (cờ spoofing, jamming).
