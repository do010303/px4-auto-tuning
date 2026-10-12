# 04 — Estimate đi trước drone 125 ms: `SENS_GPS0_DELAY`

## Triệu chứng

- Estimation error ngang ~0.30 m, **không đổi** dù quét gain nào, và là sàn của true error.
- `estimation_lag_s ≈ −0.125`: estimate khớp ground truth nhất khi dịch nó *muộn* đi 125 ms,
  tức estimate đang chạy trước vị trí thật.
- GPS càng thưa thì sai số càng nhỏ (30 Hz → 1 Hz: true 0.572 → 0.389 m) — ngược trực giác.

## Kiểm tra phép đo trước

Hai nguồn ground truth độc lập trong GZBridge:

| Nguồn | Topic uORB | Lấy từ |
|---|---|---|
| Pose | `vehicle_local_position_groundtruth` | `/world/<world>/pose/info` |
| NavSat | `vehicle_global_position_groundtruth` | cảm biến navsat, trước khi cộng noise |

So trễ bằng cách dịch thời gian tới khi khớp nhất, trên một ulog bay hình số 8:

| Cặp | Độ lệch thời gian | Sai số còn lại |
|---|---|---|
| Pose so với NavSat | 4 ms | 0.037 m |
| Estimate EKF so với NavSat | −124 ms | 0.059 m |

Hai ground truth khớp nhau, vậy độ lệch nằm ở estimate chứ không phải ở phép đo.

## Nguyên nhân

`SENS_GPS0_DELAY` mặc định 110 ms: PX4 coi mỗi mẫu GPS là đã cũ 110 ms
(`src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp:109`) và EKF bù lại bằng cách
ghép nó với trạng thái trong quá khứ rồi dự đoán tới hiện tại. GPS của Gazebo thì không có trễ.
Kết quả: estimate bị đẩy về phía trước một quãng ≈ vận tốc × ~0.12 s (≈ 0.3 m ở 2.5 m/s).

Các airframe SIH (`10040_sihsim_quadx`…) đã đặt `SENS_GPS0_DELAY 0`; airframe Gazebo
(`4001_gz_x500`) thì không.

Điều này cũng giải thích nghịch lý tần số: GPS càng thưa, EKF càng dựa vào IMU và càng ít bị
kéo về phía sai.

## Kiểm chứng

Figure8, gain mặc định, 4×, 6 chuyến mỗi cấu hình:

| | `SENS_GPS0_DELAY` = 110 (mặc định) | = 0 |
|---|---|---|
| Estimation RMSE ngang | 0.296 ± 0.009 m | 0.048 ± 0.003 m |
| Độ lệch thời gian estimate | −125 ms | 0 ms |
| True RMSE ngang | 0.536 ± 0.010 m | 0.257 ± 0.004 m |
| Tracking RMSE ngang | 0.253 ± 0.004 m | 0.222 ± 0.003 m |
| Trễ tracking | 90 ms | 53 ms |
| True cross-track RMSE | 0.166 ± 0.005 m | 0.225 ± 0.011 m |

Cross-track tăng lên là điều chưa giải thích: có thể trước đây estimate chạy trước đã vô tình bù
một phần độ trễ của controller theo phương ngang quỹ đạo.

## Xử lý

`run_sitl.sh` đặt `PX4_PARAM_SENS_GPS0_DELAY=0` lúc boot (ghi đè được bằng biến môi trường).
Mọi kết quả đo trước thay đổi này nằm trong `results/gps_delay_110ms/`: số tracking vẫn có nghĩa,
số estimation và true thì chứa sai lệch 125 ms.

## Ý nghĩa cho drone thật

Trên phần cứng thật GPS có trễ thật (thường 100–200 ms tuỳ module) và tham số này phải khớp với
nó. Thí nghiệm trên cho thấy cái giá của việc đặt sai: lệch 110 ms làm true error tăng gấp đôi ở
2.5 m/s, lớn hơn bất kỳ cải thiện nào đạt được nhờ chỉnh gain trong các lượt quét. Tune gain trên
một estimator cấu hình sai là tune nhầm chỗ.
