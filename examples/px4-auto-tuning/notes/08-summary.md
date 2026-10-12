# 08 — Tổng kết: trả lời các câu hỏi ban đầu

Tất cả số liệu đo trong PX4 SITL + Gazebo Harmonic, model x500 và ba biến thể, quỹ đạo hình số 8
(16 × 8 m, 20 s/vòng) trừ khi ghi khác. Kết luận là về **phương pháp và xu hướng**; con số gain không
chuyển thẳng sang drone thật. Số liệu chi tiết: `README.local.md` (local) và `results/*.csv`.

## 1. Parameter hoạt động thế nào, production làm ra sao

- Ba tầng: firmware defaults → runtime defaults (airframe `param set-default`) → user config (`param set`,
  tự lưu). Production ship gain trong file airframe; người dùng ghi đè ở tầng user config. → `00`
- `param set` publish `parameter_update`; controller đọc lại gain ở chu kỳ kế tiếp. **Đổi gain khi đang
  bay không cần restart gì.** Đã dùng cho mọi lượt quét: một chuyến bay liên tục, đổi gain ở ranh giới vòng.
- Firmware **không kẹp** giá trị theo `min/max` trong YAML (đã bay `MPC_XY_P` = 8 với max 2). YAML chỉ
  ràng buộc phía GCS.
- uXRCE-DDS không có dịch vụ param: từ ROS phải đi qua MAVLink `PARAM_SET`.

## 2. Các tầng của flight controller, bỏ đi thì sao

Cascade: P vị trí → PID vận tốc → P attitude → PID rate → control allocator. → `01`, `05`

| Bỏ tầng (ROS gửi lệnh tối giản thay) | Lặng gió, 4 s | Gió 5.4 m/s, 4 s |
|---|---|---|
| P vị trí | gần như không khác | trôi 0.15 m |
| + PID vận tốc | trôi 0.06 m | **trôi 3.9 m** |
| + attitude | trôi 0.24 m | trôi 0.25 m |
| + rate | nghiêng 10° | **lật > 70°, không cứu được** |

- Tầng vận tốc là tầng chống nhiễu ngoài; tầng rate là tầng giữ thăng bằng; tầng vị trí chỉ chống trôi dần.
- ROS ngừng gửi: PX4 rời offboard sau ~1.1 s và Return, giống nhau ở cả 7 mức. Failsafe nằm ở commander,
  không phụ thuộc tầng nào đang bị bỏ.
- Đường ROS ↔ PX4 (uXRCE-DDS, cùng máy): message tới ROS đều 50 Hz, dao động dưới 1 ms; khứ hồi lệnh → ack 12 ms.
- Tần số gửi: PX4 nhận setpoint xuống tới 1 Hz ở mọi mức đã thử; dưới 1 Hz thì rời offboard sau ~1.1 s
  (`COM_OF_LOSS_T` = 1 s). ROS tự đóng vòng vị trí bằng P (gain 0.95) trong gió vẫn giữ được drone ở 1 Hz.
- Mỗi mức mới chạy 1 lần.

## 3. B1 — Bay quỹ đạo và sai số

- Ba sai số luôn đo tách: tracking (setpoint − estimate), estimation (estimate − thật), true (setpoint − thật).
- Baseline gain mặc định: true RMSE ngang 0.257 m, độ lệch giữa các chuyến ~0.01–0.02 m. Khác biệt dưới
  ~0.03 m không đủ để nói hai gain khác nhau.
- Feedforward vận tốc/gia tốc là thiết yếu: bỏ đi thì 0.257 → 2.5 m.
- Hai lỗi đo lường đã phải sửa trước khi số liệu có nghĩa:
  - Mô phỏng giật do Gazebo transport đi qua card Wi-Fi → điểm "fail" giả. Sửa: `GZ_IP=127.0.0.1`. → `03`
  - `SENS_GPS0_DELAY` mặc định 110 ms trong khi GPS mô phỏng không trễ → estimate đi trước drone 125 ms.
    Đặt 0 thì true RMSE 0.536 → 0.257 m. → `04`
- Tua nhanh 4× cho kết quả giống 1×; 8× không nhanh hơn và kém tin cậy.

## 4. B2 — GPS và môi trường

- Mặc định: GPS 30 Hz, noise vị trí SD ~2 cm (Markov), `eph` 0.9 m. Đã param hoá trong GZBridge
  (`SIM_GZ_GPS_PNOIS/VNOIS/EPH/EPV/DIV`).
- **Gazebo không có plugin sai số GPS theo môi trường.** Bay ở `forest`, `lawn`, `baylands` cho cùng sai số
  GPS (đã đo). Muốn khác phải tự đặt noise (profile) hoặc viết plugin (chưa làm).
- Cái gì hại, xếp theo mức độ:
  1. **Noise vận tốc GPS**: SD 38 cm/s → true 0.64 m; 76 cm/s → 1.2 m.
  2. **Noise vị trí có tương quan**: SD 0.5 m → 0.42 m; 1.1 m → 0.69 m. Dưới 14 cm không ảnh hưởng.
  3. **Tần số**: 30 → 1 Hz gần như không đổi (0.256 → 0.266 m).
- Noise trắng SD 0.5 m (noise gốc của Gazebo) gần như vô hại (0.277 m); noise Markov cùng biên độ hại gấp
  ~3 lần ở estimation error. **Dạng noise quan trọng hơn biên độ.**
- Báo `eph` đúng có ích: GPS nhiễu mà báo `eph` 0.3 → 0.56 m; báo 3 → 0.41 m. → `07`
- PX4 từ chối GPS có `eph` > 3 m khi còn trên mặt đất, nhưng chấp nhận tới 50 m khi đang bay. → `07`
- Khi GPS kém, sai số thật ≈ estimation error. Gain không sửa được phần này (mục 5.2).

## 5. B3 — Vùng gain

| Gain | Mặc định | Vùng tốt | Bất định | Hỏng |
|---|---|---|---|---|
| `MPC_XY_P` | 0.95 | 2–3 | 4.5 | ≥ 4.75 (rơi ở 8) |
| `MPC_XY_VEL_P_ACC` | 1.8 | 3.5–7 | – | rung mạnh từ 14 |
| `MPC_XY_VEL_I_ACC` | 0.4 | ~2 | 4 | 8 |
| `MPC_XY_VEL_D_ACC` | 0.2 | 0 | – | càng tăng càng tệ |
| `MPC_Z_P` | 1.0 | ≥ 1 | – | không thấy tới 6 |
| `MPC_Z_VEL_P_ACC` | 4.0 | ≥ 4 | – | không thấy tới 40 |

- **Biên ổn định trong mặt phẳng (`XY_P`, `VEL_P`) là đường chéo**: tích hai gain mới là cái quyết định.
  Quét từng gain một sẽ cho ngưỡng sai khi gain kia đổi.
- **"33 chạy, 34 fail, 35 chạy" có thật, nhưng chỉ ở sát biên** (`MPC_XY_P` ≈ 4.5): cùng một gain có lần
  0.10 m, có lần 1.3 m. Ở xa biên, sai số thay đổi trơn và đơn điệu. Xem mục 5.1.
- Vì vậy không chọn điểm tối thiểu; chọn giữa vùng ổn định. Đề xuất **(`XY_P` 2, `VEL_P` 5)**: true RMSE
  0.257 → ~0.09–0.10 m trên cả 4 model (gốc, nặng, tay dài, motor yếu), đổi lại rung tăng 0.003 → 0.01–0.05 rad/s.
- Biến thể model (nặng hơn, tay dài hơn, motor yếu hơn) **không dịch biên** của `MPC_XY_P`: cả ba tốt tới 4, hỏng ở 5,
  giống x500 gốc (lưới thô 0.95 / 2 / 3 / 4 / 5). Bộ tune online thì dừng ở gain thấp hơn trên model motor yếu.
- Khi GPS nhiễu, gain cao được lợi ít dần rồi về 0, trong khi rung tăng 5–8 lần (mục 5.2).

### 5.1 Lặp 20 lần ở vùng bất định

`sweep_MPC_XY_P_edge20` (1 vòng ổn định + 2 vòng đo, 20 lần/điểm) và `sweep_MPC_XY_P_edge_settle3`
(3 vòng ổn định, 8 lần/điểm). Chỉ ba giá trị 4.0 / 4.25 / 4.5 xen kẽ ngẫu nhiên, không có giá trị hỏng hẳn chen vào.

| `MPC_XY_P` | 20 lần: trung vị / max / số lần > 0.3 m | 3 vòng ổn định, 8 lần: trung vị / max / số lần > 0.3 m |
|---|---|---|
| 4.0 | 0.109 / 0.162 / 0 | 0.108 / 0.220 / 0 |
| 4.25 | 0.119 / 0.199 / 0 | 0.125 / 0.163 / 0 |
| 4.5 | 0.169 / 0.442 / **6** | 0.322 / 1.303 / **5** |

- **4.0 và 4.25 ổn định**: 0/28 lần xấu. Các lần xấu ở hai giá trị này trong lượt quét trước (0.20–0.58 m)
  đều đứng ngay sau một giá trị hỏng hẳn (4.75 hoặc 5.0): đó là **dao động thừa hưởng từ gain trước**,
  một vòng ổn định không đủ để nó tắt. Thứ tự thử có ảnh hưởng tới kết quả.
- **4.5 bất định thật**: cùng một gain cho từ 0.10 tới 0.44 m (20 lần), và tới 1.3 m khi ở lại lâu hơn.
  Cho thêm vòng ổn định không giúp mà còn tệ hơn (5/8 so với 6/20) → không phải quá độ chưa tắt, mà là
  dao động lớn dần ở sát biên ổn định; kết quả tuỳ thời điểm đo.
- Vậy biên nằm giữa 4.25 và 4.5, hẹp hơn ước lượng ban đầu (4.0–4.5).
- Bài học cho phương pháp: (1) mỗi điểm cần nhiều lần lặp và báo số lần xấu, không báo pass/fail một lần;
  (2) sau một giá trị xấu phải cho drone lắng trên gain tốt rồi mới đo điểm kế; (3) số vòng ổn định là
  một tham số của phép đo, phải ghi kèm kết quả.
- Cả hai bảng đều là một chuyến bay liên tục mỗi bảng.

### 5.2 Ma trận model × mức GPS × quỹ đạo

4 model × 4 mức GPS × 2 quỹ đạo = 32 chuyến bay; mỗi chuyến đo 4 tổ hợp gain xen kẽ ngẫu nhiên, 2 lần lặp
(256 lượt đo, 0 fail, 0 loại). So sánh gain nằm trong cùng một chuyến nên cùng chịu một thực tế noise.
Bảng: true RMSE ngang (m), khoảng giá trị qua 4 model. `results/matrix_*.csv`.

| Mức GPS | Quỹ đạo | Mặc định (0.95, 1.8) | Đề xuất (2, 5) | Rung: mặc định → đề xuất | Đề xuất tốt hơn |
|---|---|---|---|---|---|
| Sạch | số 8 | 0.24–0.28 | 0.08–0.09 | 0.006 → 0.04–0.06 | 4/4 model |
| Sạch | sin | 0.23–0.24 | 0.07–0.08 | 0.007 → 0.03–0.04 | 4/4 |
| Đồng trống | số 8 | 0.38–0.45 | 0.24–0.28 | 0.006 → 0.03–0.05 | 4/4 |
| Đồng trống | sin | 0.32–0.35 | 0.20–0.24 | 0.007 → 0.03–0.05 | 4/4 |
| Rừng | số 8 | 0.73–1.01 | 0.66–0.90 | 0.016 → 0.07–0.12 | 2/4 |
| Rừng | sin | 0.70–0.89 | 0.60–0.79 | 0.014 → 0.08–0.12 | 4/4 |
| Đô thị | số 8 | 1.11–1.74 | 1.26–1.73 | 0.026 → 0.18–0.22 | 3/4 |
| Đô thị | sin | 1.31–1.59 | 1.17–1.45 | 0.027 → 0.15–0.20 | 3/4 |

Tỉ lệ sai số đề xuất / mặc định, trung bình 8 ô mỗi mức: sạch 0.33 · đồng trống 0.64 · rừng 0.93 · đô thị 0.98.

- **Lợi ích của gain cao giảm dần theo chất lượng GPS**: giảm 67% khi GPS sạch, 36% ở đồng trống, và gần
  bằng 0 ở rừng và đô thị, nơi kết quả tốt hay xấu hơn tuỳ model.
- **Cái giá thì tăng theo chiều ngược lại**: rung ở đô thị với gain đề xuất là 0.15–0.22 rad/s, gấp khoảng
  7 lần mặc định.
- Kết luận về gain đúng trên cả 4 model và cả 2 quỹ đạo; biến thể model không làm đổi bức tranh.
- **Quy tắc rút ra**: GPS tốt thì sai số nằm ở controller, tăng gain có tác dụng; GPS kém thì sai số nằm ở ước
  lượng vị trí (sai số thật ≈ estimation error ở mọi tổ hợp gain), tăng gain vô ích và chỉ thêm rung.
- Giới hạn: mức noise của ba profile là giả định; mỗi ô là 2 lượt đo trong một chuyến bay.

## 6. Tune tự động khi đang bay

- Autotune có sẵn của PX4 chỉ tune attitude/rate (nhận dạng ARX-RLS rồi suy gain). Trong SITL: roll, pitch
  hội tụ sau 5 s mỗi trục, ra gain gần mặc định; yaw không hội tụ trong 20 s. Mới chạy 1 lần. → `06`
- **Không có autotune cho vòng vị trí** → viết hai bộ tune ngoài firmware, đổi gain qua MAVLink trong lúc bay:

| Bộ tune | Tracking RMSE trước → sau | Thời gian bay | Ghi chú |
|---|---|---|---|
| Tìm theo toạ độ (`online_tuner.py`) | 0.256 → 0.117 m | ~7 phút | 8/8 lượt ổn, 4 model + GPS nhiễu |
| Bayes 2 gain (`bayes_tuner.py`) | 0.26 → 0.10–0.11 m | ~13 phút | không hơn rõ, tốn gấp đôi |
| Bayes 3 gain | 0.262 → 0.093 m | ~18 phút | tìm ra `VEL_I` cao, ít rung hơn |

- Watchdog (đo rung trong 1 s gần nhất, vượt ngưỡng thì trả gain tốt ngay) đã can thiệp thật ở bộ Bayes và
  không lần nào rơi.
- Giới hạn: tối ưu tracking chứ không phải sai số thật; cần quỹ đạo lặp lại; ngưỡng đặt tay theo x500 SITL;
  chỉ chạy ở 4× trong mô phỏng.

## 7. Chưa làm hoặc chưa chắc

- Plugin che khuất bầu trời theo vị trí (mục mở rộng 3d).
- Ba profile môi trường dùng số liệu giả định; sai số GPS cố định cả chuyến, không đổi theo vị trí.
- Không làm (đã cân nhắc, ít giá trị cho câu hỏi chính): plugin che khuất bầu trời, lặp lại bài test các tầng,
  thí nghiệm sửa yaw autotune, chạy thử Gazebo có giao diện.
- `baylands` cho sai số cao hơn ~15%, chưa rõ vì sao.
- Yaw autotune không hội tụ: đã có chẩn đoán từ log (xem `06`), chưa thử cách khắc phục.
- Chưa thử có gió giật cho `VEL_D`; chưa thử rate loop (`MC_*RATE_*`) bằng sweep.
- Chưa có số liệu trên phần cứng thật.
