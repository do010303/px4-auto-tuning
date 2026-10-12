# 06 — Tune tự động: cái PX4 có sẵn và bộ tune vòng vị trí khi đang bay

## Autotune có sẵn của PX4 (`mc_autotune_attitude_control`)

Chỉ tune vòng **rate và attitude**. Chạy bằng `px4_auto_tuning/px4_autotune.py`: hover offboard ở 10 m,
gửi `VEHICLE_CMD_DO_AUTOTUNE_ENABLE`, `MC_AT_APPLY=0` (chỉ tính, không áp).

Cách nó làm (`src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.cpp`):

1. Lần lượt roll → pitch → yaw. Mỗi trục: cộng vào rate setpoint một chuỗi bước vuông có độ rộng
   thay đổi, biên độ `MC_AT_SYSID_AMP` (`getIdentificationSignal()`).
2. Nhận dạng mô hình ARX bậc 2 từ torque setpoint → angular rate bằng RLS (`src/lib/system_identification`).
3. Trục xong khi phương sai mọi hệ số < 50 **và** đã chạy > 5 s; quá 20 s thì fail.
4. Tính PID bằng `pid_design::computePidGmvc()` theo rise time `MC_AT_RISE_TIME` (yaw: 0.2 s).
5. Attitude P theo quy tắc "sai 60° cho đầu ra tối đa": `1 / (rad(60°)·K)`, kẹp 2–6.5.
6. Huỷ ngay nếu đổi flight mode, phi công động cần, hoặc timeout.
7. `MC_AT_APPLY=2`: áp trên không, theo dõi 4 s; nếu công suất điều khiển tăng bất thường thì trả gain cũ.

Kết quả trên x500 SITL (`results/px4_autotune.json`):

| Trục | Thời gian | K | I = K·ki | D = K·kd | Attitude P | Đang dùng (P / I / D / att P) |
|---|---|---|---|---|---|---|
| roll | 5.0 s | 0.147 | 0.196 | 0.0028 | 6.5 | 0.15 / 0.2 / 0.003 / 4.0 |
| pitch | 5.0 s | 0.111 | 0.147 | 0.0022 | 6.5 | 0.15 / 0.2 / 0.003 / 4.0 |
| yaw | 20 s → **fail** (timeout) | 0.180 | 0.176 | 0.00004 | 5.3 | 0.2 / 0.1 / 0 / 2.8 |

- Roll ra gần đúng gain mặc định: gain rate mặc định đã khớp model mô phỏng.
- Yaw không hội tụ trong 20 s nên cả quy trình kết thúc ở trạng thái fail; chưa tìm hiểu vì sao.
- Mới chạy 1 lần.

### Vì sao yaw không hội tụ (đọc từ ulog của lần chạy đó)

Điều kiện hội tụ (`mc_autotune_attitude_control.cpp:280,367`): cả 5 phương sai hệ số của mô hình ARX phải
dưới 50, sau ít nhất 5 s; quá 20 s thì fail.

| Trục | Phương sai 2 hệ số mẫu số (cuối) | Hệ số mẫu số | Lệnh kích thích (RMS, đã lọc) | Tốc độ góc đáp ứng (RMS) | Motor min–max |
|---|---|---|---|---|---|
| roll | 17 / 18 | −1.627 / 0.626 | 0.08 | 0.34 rad/s | 0.68–0.77 |
| pitch | 8 / 8 | −1.630 / 0.628 | 0.05 | 0.33 rad/s | 0.66–0.80 |
| yaw | **1586 / 1579** | −1.023 / 0.025 | 0.37 | 0.26 rad/s | 0.53–0.91 |

- Ba hệ số tử số của yaw hội tụ (phương sai 10–38); chỉ hai hệ số mẫu số là không.
- Yaw cần lệnh lớn gấp 5–7 lần roll/pitch mà đáp ứng còn nhỏ hơn: quyền điều khiển yaw của x500 trong
  Gazebo thấp (moment yaw chỉ đến từ phản moment cánh quạt, `CA_ROTOR*_KM` = 0.05).
- Mô hình nhận dạng được gần như một khâu tích phân thuần (cực ≈ 1.0, hệ số thứ hai ≈ 0). **Giả thuyết**:
  với động học chậm như vậy, y(k−1) và y(k−2) gần trùng nhau nên hai hệ số mẫu số không tách được, phương
  sai của chúng không giảm dù kích thích đủ lớn. Chưa kiểm chứng.
- Chưa thử: tăng `CA_ROTOR*_KM` của model.

### Cập nhật 2026-10-09: yaw hội tụ khi tăng biên độ

Chạy lại theo đúng trình tự của QGroundControl (`qgc_autotune.py`, Hold 10 m, `MC_AT_APPLY=1`), mỗi mức một lần:

| `MC_AT_SYSID_AMP` | Kết quả |
|---|---|
| 0.7 (mặc định) | fail ở yaw sau 39 s |
| 1.7 | fail ở yaw sau 39 s |
| 2.7 | thành công sau 27 s, ghi gain khi disarm |

- Kích thích lớn hơn đủ để yaw hội tụ. Giả thuyết "hai hệ số mẫu số không tách được" ở trên vì vậy không
  giải thích hết: nếu đúng hoàn toàn thì tăng biên độ không giúp được.
- Trong firmware của repo, autotune chỉ chạy được **một lần mỗi lần khởi động PX4**: sau lượt đầu (xong hay
  fail) `stopAutotune()` gỡ callback đánh thức module; lần bấm thứ hai đứng ở 0% (3/3 lần).
- Thao tác từng bước, bảng gain và cách xử lý khi fail: `09-qgc-autotune.md`.

## Bộ tune vòng vị trí khi đang bay (`online_tuner.py`)

PX4 không có autotune cho vòng vị trí/vận tốc. Bộ này nằm ngoài firmware: bay quỹ đạo liên tục,
chấm điểm từng vòng từ dữ liệu ROS, đổi gain qua MAVLink.

- **Điểm một vòng** = tracking RMSE + 0.5 × rung. Rung lấy từ tốc độ đổi góc nghiêng (topic
  `vehicle_attitude`) sau khi bỏ thành phần chậm.
- **Tìm kiếm theo từng toạ độ**: mỗi lần tăng một gain 30%, nghỉ nửa vòng, đo một vòng. Giữ nếu điểm
  tốt hơn ≥ 3% và rung dưới ngưỡng; ngược lại trả gain cũ và đánh dấu gain đó hết đường.
- **Watchdog**: trong lúc thử, nếu rung 1 giây gần nhất hoặc sai số tức thời vượt ngưỡng cứng thì trả
  gain tốt gần nhất ngay, không đợi hết vòng.
- Khi mọi gain đều bị từ chối: giảm nửa bước, thử lại một lượt, rồi dừng.
- **Lùi lại**: gain cuối = 75% quãng đường từ gain ban đầu tới điểm tốt nhất, vì các lượt quét cho
  thấy điểm tốt nhất nằm sát vùng mất ổn định.
- Ghi param chạy ở luồng riêng. Bản đầu ghi ngay trong vòng điều khiển: lệnh ghi chờ xác nhận, luồng
  setpoint bị nghẽn và PX4 rơi khỏi offboard.

Kết quả (4×, hình số 8; RMSE là **tracking** do tuner tự đo, `results/online_tuner_*.json`):

| Điều kiện | Gain cuối (XY_P / VEL_P) | Tracking RMSE trước → sau | Rung trước → sau | Số lần thử | Thời gian bay |
|---|---|---|---|---|---|
| x500 | 1.80 / 3.42 | 0.256 → 0.117 | 0.025 → 0.049 | 10 | 408 s |
| x500 (lặp lại) | 1.80 / 3.42 | 0.276 → 0.114 | 0.025 → 0.046 | 10 | 408 s |
| x500_heavy | 2.04 / 2.73 | 0.247 → 0.112 | 0.025 → 0.041 | 10 | 415 s |
| x500_long | 2.04 / 2.73 | 0.252 → 0.114 | 0.024 → 0.038 | 10 | 408 s |
| x500_weak | 1.44 / 2.73 | 0.285 → 0.157 | 0.026 → 0.036 | 8 | 340 s |
| GPS noise vị trí (SD ≈ 0.5 m) | 0.95 / 2.24 | 0.307 → 0.287 | 0.040 → 0.054 | 6 | 280 s |
| GPS noise vận tốc (SD ≈ 19 cm/s) | 1.16 / 2.47 | 0.306 → 0.239 | 0.050 → 0.075 | 7 | 309 s |
| x500, thêm `VEL_I_ACC` | 1.44 / 2.73 / I 0.49 | 0.265 → 0.143 | 0.026 → 0.034 | 11 | 469 s |

- Cả 8 lần đều kết thúc bình thường, không lần nào mất ổn định, watchdog chưa phải can thiệp lần nào.
- Trên 4 model với GPS sạch: tracking error giảm 45–55% sau khoảng 6–7 phút bay.
- Gain cuối rơi vào đúng vùng mà lượt quét 2 chiều chỉ ra là tốt và an toàn.
- GPS nhiễu: tuner tự dừng sớm ở gain thấp hơn, vì gain cao khuếch đại nhiễu thành rung.
- Thêm gain I làm kết quả kém hơn chỉ dùng hai gain P: I được thử xen kẽ, chiếm lượt và bước của P.

## Tối ưu Bayes khi đang bay (`bayes_tuner.py`)

Cùng chuyến bay, cùng điểm số và watchdog như `online_tuner.py`, nhưng gain thử tiếp theo do một mô hình
Gaussian process của điểm số đề xuất (expected improvement), thay cho tìm theo từng toạ độ. Mỗi lần thử là
một vòng. Vùng tìm giới hạn trong vùng ổn định đã lập bản đồ bằng sweep: `MPC_XY_P` 0.6–3.5,
`MPC_XY_VEL_P_ACC` 1.2–9, `MPC_XY_VEL_I_ACC` 0.2–3 (thang log). Lần thử bị watchdog chặn hoặc rung quá
ngưỡng vẫn được đưa vào mô hình với điểm phạt. GP tự viết bằng numpy (không cần optuna).

Kết quả (x500, GPS sạch, 4×, `results/bayes_tuner_*.csv`):

| Lượt | Số lần thử | Gain cuối (XY_P / VEL_P / VEL_I) | Tracking RMSE trước → sau | Rung sau | Watchdog | Thời gian bay |
|---|---|---|---|---|---|---|
| 2 gain, seed 0 | 25 | 1.55 / 5.00 / – | 0.259 → 0.110 | 0.055 | 1 lần | 786 s |
| 2 gain, seed 1 | 25 | 2.80 / 2.60 / – | 0.262 → 0.102 | 0.059 | 1 lần | 787 s |
| 3 gain, seed 0 | 35 | 0.98 / 4.48 / 2.71 | 0.262 → 0.093 | 0.043 | 2 lần | 1098 s |
| Tìm theo toạ độ (so sánh) | 10 | 1.80 / 3.42 / – | 0.256 → 0.117 | 0.049 | 0 | 408 s |

- Với 2 gain, Bayes không hơn rõ tìm theo toạ độ (0.10–0.11 so với 0.117) mà tốn gấp đôi thời gian bay.
  Hai seed dừng ở hai điểm khác hẳn nhau với điểm số gần bằng nhau: đáy của hàm mục tiêu là một
  **thung lũng rộng và phẳng** chạy chéo (XY_P cao + VEL_P thấp, hoặc ngược lại), không phải một điểm.
  Chênh lệch giữa các điểm trong thung lũng (0.13–0.14) cùng cỡ với nhiễu của phép đo một vòng.
- Với 3 gain, Bayes tìm ra vùng mà tìm theo toạ độ bỏ lỡ: `VEL_I` cao (≈ 2.7) với `XY_P` gần mặc định cho
  sai số thấp nhất *và* rung thấp hơn. Khớp với lượt quét 1 chiều (`VEL_I` tốt nhất quanh 2).
- Watchdog phải can thiệp ở các điểm góc (cả hai gain cùng cao), và đã đưa drone về gain tốt mà không rơi.
- Mỗi lần thử chỉ đo một vòng nên điểm "tốt nhất" có phần may rủi; gain cuối được đo lại 2 vòng để xác nhận.
- Lỗi đã sửa khi làm bài này: sau khi watchdog chặn một ứng viên, rung chưa kịp tắt thì ứng viên kế tiếp đã
  bị đổ lỗi (24/25 lần thử bị loại oan). Giờ tuner chờ rung lắng xuống trên gain tốt rồi mới thử tiếp.
- Mới chạy 2 seed cho 2 gain và 1 seed cho 3 gain; chưa chạy trên model khác hay GPS nhiễu.

## Giới hạn của bộ tune này

- Tối ưu **tracking** (so với estimate), không phải sai số thật; estimator sai thì nó không biết.
- Cần một quỹ đạo lặp lại để so sánh vòng với vòng; chưa dùng được trong lúc bay nhiệm vụ bất kỳ.
- Chỉ tăng gain, không giảm: xuất phát từ gain quá cao thì nó không cứu được.
- Ngưỡng rung và trọng số đặt bằng tay theo số liệu SITL của x500.
- Mới chạy ở tốc độ 4× trong mô phỏng; chưa chạy ở tốc độ thật, chưa chạy trong gió.
- Tìm theo từng toạ độ với bước 30% là thô; điểm dừng phụ thuộc thứ tự gain.
