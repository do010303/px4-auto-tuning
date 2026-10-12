# 09 — Autotune PX4 bằng QGroundControl: thao tác từng bước

Autotune có sẵn của PX4 (`mc_autotune_attitude_control`) chỉ tune vòng **rate và attitude** (roll, pitch, yaw).
Nó không đụng tới gain vị trí/vận tốc (`MPC_*`). Cách nó nhận dạng và tính gain: xem `06-autotune.md`.

Nguồn: trang "Auto-tuning" của tài liệu PX4 v1.13, mã trong `src/modules/mc_autotune_attitude_control/` và
`src/modules/mavlink/mavlink_receiver.cpp` (base commit `583e1cdb47`), và các lượt đo trên x500 SITL ngày
2026-10-09.

**Lưu ý**: tên menu và nút của QGC dưới đây lấy từ tài liệu v1.13. Tôi chưa mở QGC trên máy này để đối
chiếu; bản QGC mới có thể đặt nhãn khác. Phần đo dùng `qgc_autotune.py`, gửi đúng các lệnh MAVLink mà QGC
gửi, không qua giao diện QGC.

## Tóm tắt

- Trên x500 SITL, biên độ mặc định (`MC_AT_SYSID_AMP` = 0.7) **fail ở yaw**. Đặt 2.7 thì xong trong 27 s.
- Gain chỉ được ghi **sau khi hạ cánh và disarm** (`MC_AT_APPLY` = 1, mặc định). Fail thì không ghi gì.
- Trong firmware của repo, autotune chỉ chạy được **một lần mỗi lần khởi động PX4**. Muốn thử lại phải
  khởi động lại PX4.
- Mỗi mức biên độ mới đo một lần.
- Gain autotune ra **không làm bay hình số 8 tốt hơn** gain mặc định (bảng ở cuối mục kết quả).

## 1. Chuẩn bị

1. Mở mô phỏng ở tốc độ thật (1×), có biên độ đã đặt sẵn:

   ```bash
   PX4_PARAM_MC_AT_SYSID_AMP=2.7 ./run_sitl.sh
   ```

   Trên drone thật: đặt `MC_AT_SYSID_AMP` trong QGC (Vehicle Setup > Parameters) trước khi bay.
2. Mở QGroundControl. Nó tự kết nối với SITL qua UDP 14550.
3. Kiểm tra param (Vehicle Setup > Parameters):

   | Param | Giá trị | Ý nghĩa |
   |---|---|---|
   | `MC_AT_EN` | 1 | Bật module. SITL đã đặt sẵn 1; trên board thật mặc định là 0, đổi xong phải khởi động lại. |
   | `MC_AT_APPLY` | 1 | 0: chỉ tính, không ghi. 1: ghi sau khi disarm. 2: ghi ngay trên không (có rủi ro). |
   | `MC_AT_SYSID_AMP` | 0.7 → 2.7 | Biên độ tín hiệu kích thích. Khoảng cho phép 0.1–6. |
   | `MC_AT_RISE_TIME` | 0.14 | Rise time mong muốn của vòng rate (s). Lớn hơn thì gain mềm hơn. |

4. Chờ QGC báo sẵn sàng cất cánh (EKF2 đã có vị trí).

## 2. Thử trước khi tune

Autotune giả định drone đã bay ổn với gain hiện tại. Tài liệu yêu cầu kiểm tra điều đó trước:

1. Cất cánh, giữ hover.
2. Gạt cần roll nghiêng vài độ: trái → phải → về giữa. Lặp lại với biên độ lớn dần.
3. Làm tương tự với pitch.
4. Nếu drone dao động thì **không** chạy autotune. Hạ cánh và sửa bằng tay trước:
   - dao động chậm: giảm `MC_ROLL_P`, `MC_PITCH_P` mỗi lần 1.0;
   - dao động nhanh: giảm `MC_ROLLRATE_K`, `MC_PITCHRATE_K`, `MC_YAWRATE_K` mỗi lần 0.02.

Trên x500 SITL với gain mặc định, bước này không cần: drone hover ổn.

## 3. Chạy autotune

1. Cất cánh và hover ở độ cao an toàn. Các lượt đo ở đây hover 10 m trong chế độ Hold.
2. Trong QGC mở **Vehicle Setup > PID Tuning**.
3. Chọn tab **Rate Controller** hoặc **Attitude Controller**.
4. Bật **Autotune enabled**. Nút **Autotune** hiện ra, thay cho các ô chỉnh tay.
5. Thả cần điều khiển, bấm **Autotune**, đọc cảnh báo rồi bấm OK.
6. Drone lắc nhanh theo roll, rồi pitch, rồi yaw. QGC hiện thanh tiến độ:

   | Tiến độ | Trạng thái | Thời điểm đo được (biên độ 2.7) |
   |---|---|---|
   | 0% | khởi tạo | 0 s |
   | 20% | roll | 5 s |
   | 40% | pitch | 12 s |
   | 60% | yaw | 19 s |
   | 95% | đã có gain, chờ disarm | 27 s |
   | 100% | đã ghi gain | sau khi disarm |

7. Trong lúc tune **không** đổi flight mode và **không** động cần roll/pitch (lệch quá 5% là huỷ). Mỗi trục
   có tối đa 20 s; quá thì fail.

## 4. Hạ cánh để ghi gain

1. Khi tiến độ tới 95%, hạ cánh và để drone disarm. PX4 ghi gain mới đúng lúc disarm.
2. Nếu tắt nguồn hoặc khởi động lại PX4 trước khi disarm thì mất kết quả.

## 5. Kiểm tra

1. Mở Vehicle Setup > Parameters, xem các gain `MC_*RATE_*` và `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` đã đổi.
2. Cất cánh thấp, cẩn thận, lặp lại bài thử ở mục 2. Nếu dao động: xử lý như mục 2, hoặc tăng
   `MC_AT_RISE_TIME` rồi tune lại.

## Kết quả đo trên x500 SITL

1×, hover 10 m, `MC_AT_APPLY` = 1, mỗi mức một lần (`results/raw/qgc_autotune_amp*.json`).

| `MC_AT_SYSID_AMP` | Kết quả | Ghi chú |
|---|---|---|
| 0.7 (mặc định) | **fail** ở yaw sau 39 s | roll, pitch xong; yaw hết 20 s; không ghi gain nào |
| 1.7 | **fail** ở yaw sau 39 s | như trên |
| 2.7 | **thành công**, 27 s | ghi 12 gain khi disarm (bảng dưới) |
| 3.7 | chưa có số | lượt chạy bị ngắt giữa chừng |

Gain trước và sau ở mức 2.7:

| Trục | Rate P | Rate I | Rate D | Attitude P |
|---|---|---|---|---|
| roll | 0.15 → 0.146 | 0.2 → 0.194 | 0.003 → 0.0028 | 4.0 → 6.5 |
| pitch | 0.15 → 0.110 | 0.2 → 0.146 | 0.003 → 0.0021 | 4.0 → 6.5 |
| yaw | 0.2 → 0.216 | 0.1 → 0.209 | 0 → 0.012 | 2.8 → 4.41 |

- Roll ra gần đúng gain mặc định: gain rate mặc định đã khớp model mô phỏng.
- Pitch P thấp hơn mặc định 27%. Attitude P của roll và pitch chạm trần 6.5 của thuật toán.
- Yaw đổi nhiều nhất (I gấp đôi, có thêm D).

### Gain autotune có làm bay tốt hơn không

Hình số 8, 2 vòng, 4×, 3 chuyến hợp lệ / 3 mỗi ô, trung bình ± độ lệch chuẩn (`results/autotune_check_*.csv`,
đo 2026-10-12). "Autotune" là 12 gain của bảng trên, đặt từ lúc khởi động.

| Gain vòng trong | Gain vòng ngoài (`MPC_XY_P` / `MPC_XY_VEL_P_ACC`) | True RMSE XY (m) | Tracking RMSE XY (m) | Rung rate (`rate_hf_rms`) |
|---|---|---|---|---|
| mặc định | mặc định (0.95 / 1.8) | 0.256 ± 0.016 | 0.224 ± 0.009 | 0.0041 |
| autotune | mặc định (0.95 / 1.8) | 0.249 ± 0.011 | 0.217 ± 0.010 | 0.0042 |
| mặc định | 2 / 5 | 0.085 ± 0.008 | 0.050 ± 0.003 | 0.0129 |
| autotune | 2 / 5 | 0.096 ± 0.005 | 0.060 ± 0.003 | 0.0139 |

- Với gain vòng ngoài mặc định, gain autotune **không đổi** sai số: chênh 0.007 m, nhỏ hơn độ lệch giữa các chuyến.
- Với gain vòng ngoài 2 / 5, gain autotune **kém hơn một chút**: tracking 0.050 → 0.060 m (chênh gấp ba độ lệch),
  true 0.085 → 0.096 m (chênh cỡ 1.5 lần độ lệch). Mới 3 chuyến mỗi ô nên đây là dấu hiệu, chưa phải kết luận.
- Đổi gain vòng ngoài làm sai số giảm ba lần; đổi gain vòng trong bằng autotune thay đổi dưới 0.011 m. Trên
  x500 SITL, vòng ngoài mới là chỗ có lợi.
- Không chuyến nào bão hoà motor hay có lỗi PX4. Đã kiểm tra trong ulog rằng mỗi ô bay đúng bộ gain.

## Khi fail

| Hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| Dừng ở 60% khoảng 20 s rồi báo fail | Yaw không hội tụ: quyền điều khiển yaw của x500 trong Gazebo thấp, kích thích mặc định quá nhỏ (`06-autotune.md`) | Tăng `MC_AT_SYSID_AMP` mỗi lần 1 (tài liệu khuyên vậy). Trên x500 SITL: 2.7. |
| Bấm Autotune lần hai, đứng mãi ở 0% | Sau lượt đầu (xong hay fail) module tự gỡ callback đánh thức nó (`stopAutotune()`), không đăng ký lại. Thấy 3/3 lần; đổi param không gỡ được. | Khởi động lại PX4 (tắt và mở lại `run_sitl.sh`), rồi thử lại. |
| Fail ngay khi đang lắc | Đổi flight mode, hoặc cần roll/pitch lệch quá 5% | Giữ nguyên mode, thả cần. Khởi động lại PX4 rồi thử lại. |
| Tới 95% nhưng gain không đổi | Chưa disarm, hoặc `MC_AT_APPLY` = 0 | Hạ cánh, chờ disarm. Kiểm tra `MC_AT_APPLY`. |
| Không có phản ứng gì | `MC_AT_EN` = 0, module không chạy | Đặt 1, khởi động lại. |
| Dao động sau khi tune | Gain quá cứng cho khung | Xử lý như mục 2, hoặc tăng `MC_AT_RISE_TIME` rồi tune lại. |

Vì fail không ghi gain nào, drone vẫn giữ gain cũ; thử lại không mất gì ngoài một lần khởi động lại.

Giới hạn "một lần mỗi lần khởi động" mới thấy trên firmware của repo này; chưa đối chiếu với bản PX4 khác.

## Khác biệt giữa tài liệu v1.13 và firmware này

- Tài liệu "PID Tuning (Advanced)" nhắc tới nhiều giá trị của `MPC_POS_MODE`. Firmware này chỉ còn hai:
  0 (Direct velocity) và 4 (Acceleration based). Muốn tune vòng vận tốc bằng cần thì dùng 0.
- Tài liệu không nói autotune chỉ chạy được một lần mỗi lần khởi động.
- Tài liệu không nói yaw có thể fail riêng trong khi roll và pitch đã xong; khi đó cả ba trục đều không được ghi.

## Không cần QGC

`qgc_autotune.py` làm đúng trình tự trên: cất cánh → Hold → gửi `MAV_CMD_DO_AUTOTUNE_ENABLE`, gửi lại mỗi
giây để đọc tiến độ → hạ cánh → đọc lại gain sau khi disarm.

```bash
HEADLESS=1 PX4_DAEMON=1 PX4_PARAM_MC_AT_SYSID_AMP=2.7 ./run_sitl.sh
python3 -m px4_auto_tuning.qgc_autotune --out results/raw/qgc_autotune_amp2.7_b.json
```

Mỗi lần chạy cần một phiên `run_sitl.sh` mới. File kết quả có gain trước, gain sau và dòng thời gian của
tiến độ.
