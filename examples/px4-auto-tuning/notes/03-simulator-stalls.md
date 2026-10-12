# 03 — Vì sao mô phỏng "giật" và drone rơi không do gain

## Triệu chứng

- PX4 in `ERROR [sensors] Accel #0 fail: TIMEOUT!`, đôi khi `Attitude failure (roll)`.
- Drone đang bám quỹ đạo với sai số ~0.2 m thì lộn nhào.
- Xảy ra ngẫu nhiên: 8/18 chuyến baseline, 21 lượt đo trong hai lượt quét gain đầu tiên.
  Nếu không lọc, nó hiện ra đúng dạng "gain 1.2 chạy, 1.4 fail, 1.6 chạy".

## Những giả thuyết đã loại

| Giả thuyết | Kết quả kiểm tra |
|---|---|
| Thiếu RAM / swap | Sai. Sau khi máy trống 9 GB, không dùng swap, vẫn giật (3/3 chuyến hỏng). |
| CPU quá tải | Sai. 16 nhân, load ~2–4. |
| Gazebo không kịp bước mô phỏng | Sai. Thời gian thực do chính Gazebo đóng dấu trên message clock cách nhau tối đa 10 ms. |
| Gazebo làm mất message | Sai. Một subscriber độc lập nhận đủ mọi mẫu IMU, cách đều 4 ms theo thời gian mô phỏng. |

## Nguyên nhân

Gazebo transport mặc định gắn vào **địa chỉ của card mạng chính**, trên máy này là Wi-Fi
(`wlp5s0`). Trên đường đó message tới nơi theo từng đợt, có lúc khựng tới ~330 ms rồi dồn
về một lúc.

Đo bằng một subscriber Python trên topic `/world/default/clock`, 60 s, chỉ chạy Gazebo:

| Cấu hình | Số lần khựng > 20 ms | Lâu nhất |
|---|---|---|
| Mặc định (qua địa chỉ Wi-Fi) | 12 | 335 ms |
| `GZ_IP=127.0.0.1` | 0 | 8 ms |

Chuỗi hậu quả phía PX4:

1. `gz_bridge` đặt đồng hồ PX4 theo message clock và đóng dấu mẫu IMU bằng thời điểm
   callback chạy (`GZBridge.cpp`, `clockCallback` / `imuCallback`).
2. Sau một lần khựng, hàng chục message clock + IMU tới cùng lúc. Đồng hồ PX4 nhảy vọt,
   các mẫu IMU dồn cục và một phần bị bỏ qua.
3. Trong ulog thấy `sensor_combined` mất ~13% mẫu ngay cả khi drone đứng yên, khoảng trống
   tới 64 ms (bình thường 4 ms).
4. Ước lượng attitude sai → điều khiển sai → rơi.

## Cách sửa

`run_sitl.sh` đặt `GZ_IP=127.0.0.1` (ghi đè được bằng biến môi trường nếu cần chạy Gazebo
qua mạng). Không sửa firmware.

Vẫn giữ kiểm tra `sim_max_gap_ms` và dòng `ERROR` trong các runner như lưới an toàn.

## Bài học cho phương pháp

- Trước khi kết luận "gain X fail", phải loại trừ lỗi của chính dụng cụ đo.
- Kết quả không đơn điệu (chạy – fail – chạy) trong mô phỏng nên bị nghi là nhiễu của hệ
  mô phỏng trước, và chỉ tin khi lặp lại được nhiều lần.
