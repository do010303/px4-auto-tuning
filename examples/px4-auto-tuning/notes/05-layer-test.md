# 05 — Mỗi tầng controller của PX4 làm gì: bài test bỏ dần từng tầng

Công cụ: `px4_auto_tuning/layer_test.py`. Kết quả: `results/layer_test.csv`, `results/layer_test_windy.csv`.

## Cách test

1. Drone cất cánh và hover ở 10 m bằng position control bình thường (offboard, mức `position`).
2. Chuyển cờ `OffboardControlMode` sang mức cần test. ROS gửi setpoint **tối giản nhất** mà mức đó
   nhận, không có vòng phản hồi nào phía ROS:

   | Mức | ROS gửi | Tầng PX4 bị bỏ qua |
   |---|---|---|
   | `position` | điểm hover | không |
   | `velocity` | vận tốc 0 | P vị trí |
   | `acceleration` | gia tốc 0 | P vị trí, PID vận tốc |
   | `attitude` | quaternion nằm ngang + lực đẩy hover | toàn bộ position control |
   | `body_rate` | tốc độ góc 0 + lực đẩy hover | position + attitude control |
   | `thrust_and_torque` | lực đẩy hover, moment 0 | position + attitude + rate control |
   | `direct_actuator` | 4 motor cùng một giá trị | mọi controller và control allocator |

3. Giữ 4 giây, đo trôi ngang, lệch độ cao, góc nghiêng, tốc độ.
4. Trả lại mức `position` trong 10 giây: drone có về được điểm hover không.
5. (tuỳ chọn `--silence`) ngừng gửi hoàn toàn để xem failsafe.

Lực đẩy hover đặt cứng 0.73 (trung bình lệnh motor của x500 khi hover, đo từ ulog).
Bài này là **test để hiểu**, không dựng lại flight controller trong ROS.

## Kết quả

| Mức | Lặng gió: trôi / nghiêng / cứu được | Gió 5.4 m/s: trôi / nghiêng / cứu được |
|---|---|---|
| position | 0.01 m / 0.3° / có | 0.01 m / 4.2° / có |
| velocity | 0.04 m / 0.2° / có | 0.15 m / 3.9° / có |
| acceleration | 0.06 m / 0.2° / có | 3.88 m / 3.7° / có |
| attitude | 0.07 m / 0.1° / có | 3.38 m / 3.5° / có |
| body_rate | 0.24 m / 0.2° / có | 0.25 m / 3.8° / có |
| thrust_and_torque | 0.48 m / 9.7° / có | cắt sớm / 71.7° / không |
| direct_actuator | 0.12 m / 5.2° / có | cắt sớm / 72.3° / không |

## Đọc kết quả

- **Tầng velocity là tầng chống nhiễu ngoài.** Trong gió, còn nó thì drone đứng yên (nghiêng ~4° để
  chống gió); bỏ nó thì drone bị thổi đi gần 4 m sau 4 giây dù attitude vẫn được giữ.
- **Tầng rate là tầng giữ thăng bằng.** Bỏ nó thì không còn gì chống lại moment nhiễu: lặng gió nghiêng
  ~10° sau 4 giây, có gió thì lật quá 70° và không cứu được.
- **Tầng position chỉ để không trôi dần.** Bỏ nó (mức `velocity`) gần như không thấy khác biệt trong 4 giây.
- Từ `acceleration` trở xuống không còn ai giữ độ cao: drone trôi dọc theo sai số của lực đẩy hover đặt cứng.
- Mức `body_rate` trong gió trôi ít bất ngờ (0.25 m). Giả thuyết: "tốc độ góc 0" giữ nguyên góc nghiêng
  chống gió đã có sẵn lúc chuyển mức. Chưa kiểm chứng.
- Lặng gió: `direct_actuator` trôi ít hơn `thrust_and_torque`; chưa giải thích được, mỗi mức mới chạy 1 lần.

## Khi ROS im lặng

Sau mỗi mức, ngừng gửi 8 giây: PX4 rời offboard sau ~1.1 giây và chuyển sang Return
(`nav_state` 14 → 5), giống nhau ở cả 7 mức. Drone giữ vị trí ngang và hạ độ cao có kiểm soát.
Failsafe mất tín hiệu offboard nằm ở commander, độc lập với việc tầng nào đang bị bỏ qua.

## Ý nghĩa cho "ROS điều khiển thay PX4"

| Muốn thay tầng | Yêu cầu với phía ROS |
|---|---|
| position | dễ: sai lệch tích luỹ chậm, tần số thấp là đủ |
| velocity | phải tự chống gió và nhiễu ngoài; sai một chút là trôi mét/giây |
| attitude | phải tự giữ độ cao và hướng lực đẩy |
| rate | phải đóng vòng nhanh và liên tục: mất vài giây là lật, không cứu được |

## Đường truyền ROS ↔ PX4 và tần số gửi

Đo ngày 2026-10-07. Công cụ: `dds_latency.py`, `layer_test.py --rate / --closed-loop`.

**Độ trễ** (SITL tốc độ thật, agent và PX4 cùng máy; `results/dds_latency.json`):

| Chiều | Kết quả |
|---|---|
| PX4 → ROS (`vehicle_attitude`) | tới đủ 50 Hz; trễ so với message nhanh nhất: trung vị 0.16 ms, p99 0.34 ms, max 0.74 ms |
| ROS → PX4 → ROS (lệnh disarm trên mặt đất → ack) | 100/100 có trả lời; trung vị 12.0 ms, p99 12.3 ms, max 12.4 ms |

Chiều PX4 → ROS chỉ đo được độ dao động, không đo được trễ tuyệt đối (hai đồng hồ lệch nhau một khoảng không
biết). Khứ hồi 12 ms gồm cả vòng lặp của commander xử lý lệnh, nên là cận trên của đường truyền. Trên máy bay
thật, đường này đi qua UART hoặc Ethernet và sẽ khác.

**Tần số gửi tối thiểu để PX4 còn nhận** (setpoint cố định, tốc độ thật, giữ 6 s; `results/layer_test_rate_1x.csv`):

| Tần số gửi | position | body_rate |
|---|---|---|
| 2 Hz | giữ offboard | giữ offboard |
| 1 Hz | giữ offboard | giữ offboard |
| 0.8 Hz | rời offboard sau 1.12 s → Return | rời sau 1.08 s → Return |
| 0.5 Hz | rời sau 1.12 s → Return | rời sau 1.06 s → Return |

Ngưỡng là `COM_OF_LOSS_T` = 1 s: khoảng cách giữa hai message `offboard_control_mode` vượt 1 s thì commander
coi là mất tín hiệu. Ngưỡng này không phụ thuộc mức offboard (ở 50–1 Hz cả position, attitude, body_rate đều
được nhận; `results/layer_test_rate.csv`).

**ROS tự đóng vòng vị trí** (mức `velocity`, ROS gửi vận tốc = 0.95 × sai số vị trí, gió 5.4 m/s, giữ 20–30 s;
`results/layer_test_velocity_loop.csv`):

| Tần số vòng ROS | Trôi lớn nhất | Nghiêng lớn nhất |
|---|---|---|
| 50 / 20 / 10 / 5 / 2 / 1 Hz | 0.05–0.07 m | 4.0–4.2° |
| 0.5 Hz (chỉ chạy được ở mô phỏng 4×) | 0.09 m | 4.4° |
| 0.33 Hz (chỉ chạy được ở mô phỏng 4×) | 0.50 m | 7.1° |

- Thay tầng P vị trí bằng một vòng trong ROS thì **1 Hz là đủ** với gain này: vòng ngoài chậm (băng thông cỡ
  1 rad/s) nên không cần gửi nhanh. Vòng chỉ bắt đầu kém đi quanh 0.3–0.5 Hz, tức là giới hạn thực tế là
  ngưỡng mất tín hiệu 1 Hz của PX4 chứ không phải độ ổn định của vòng điều khiển.
- Điều này chỉ đúng cho tầng ngoài cùng. Các tầng sâu hơn (attitude, rate) nhanh hơn nhiều bậc; chưa đo vì phải
  viết lại controller trong ROS, ngoài phạm vi bài test.
- **Cạm bẫy khi tua nhanh**: ngưỡng mất tín hiệu offboard chạy theo đồng hồ thật, không theo thời gian mô
  phỏng. Ở 4×, PX4 chịu được khoảng trống 4 giây mô phỏng (đo được: ngừng gửi thì 4.1 s sau mới Return, so với
  1.1 s ở tốc độ thật). Mọi phép đo liên quan tới failsafe phải chạy ở 1×.

