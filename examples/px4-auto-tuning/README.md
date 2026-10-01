# px4-auto-tuning

Nghiên cứu ảnh hưởng của gain (Kp/Ki/Kd) lên sai số bay của PX4 và cách tìm/chỉnh
gain ngay khi đang bay, trên Gazebo SITL với x500.

Thư mục này đặt trong một bản PX4 tại `examples/px4-auto-tuning/` nhưng là repo git riêng.

## Yêu cầu

- PX4 đã build SITL: `make px4_sitl` (base commit đang dùng: `583e1cdb47`)
- Gazebo Harmonic, ROS 2 Humble, workspace có `px4_msgs` (mặc định `~/px4_ws`)
- `micro-xrce-dds-agent`, `pip install pyulog pymavlink`

## Chạy thủ công (có GUI, tự giám sát)

Mỗi bước một terminal, đứng ở `examples/px4-auto-tuning/`.

**Terminal 1 — mô phỏng.** Mở Gazebo có GUI, PX4 chạy foreground với console `pxh>`,
kèm DDS agent. Rootfs riêng trong `./run` (param bắt đầu từ mặc định).

```bash
./run_sitl.sh                      # PX4_GZ_WORLD=forest ./run_sitl.sh để đổi world
```

**Terminal 2 — (tuỳ chọn) QGroundControl** để xem bản đồ, trạng thái, sửa param bằng tay:

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

**Terminal 3 — bay quỹ đạo.**

```bash
source env.sh
python3 -m px4_auto_tuning.trajectory_node --kind figure8 --laps 5 --events events.csv
#   --kind sine | figure8      --period 20   --size-x 8 --size-y 4   --altitude 5
#   --no-ff    chỉ gửi position (không feedforward)
#   --no-land  bay xong giữ hover trong offboard
```

**Terminal 4 — đổi gain khi drone đang bay** (không restart gì, có hiệu lực ngay):

```bash
python3 -m px4_auto_tuning.param_client MPC_XY_P            # đọc
python3 -m px4_auto_tuning.param_client MPC_XY_P 1.4        # ghi
```

Hoặc gõ thẳng trong console `pxh>` ở Terminal 1: `param set MPC_XY_P 1.4`.

**Sau khi hạ cánh — tính sai số.** Mỗi lần arm sinh một file ulog trong `run/rootfs/log/<ngày>/`:

```bash
python3 -m px4_auto_tuning.metrics "$(ls -t run/rootfs/log/*/*.ulg | head -1)" --events events.csv
```

### Giám sát khi đang chạy

| Muốn xem | Lệnh |
|---|---|
| Trạng thái, cảnh báo, lỗi của PX4 | console `pxh>` ở Terminal 1 (`commander status`, `ekf2 status`, `listener sensor_gps`) |
| Một topic uORB từ terminal khác | `../../build/px4_sitl_default/bin/px4-listener vehicle_local_position` |
| Giá trị param đang dùng / đã đổi | `../../build/px4_sitl_default/bin/px4-param show MPC_XY*` , `px4-param show -c` |
| Vị trí, setpoint qua ROS 2 | `source env.sh; ros2 topic echo /fmu/out/vehicle_local_position_v1` |
| Vẽ đồ thị trực tiếp | QGroundControl → Analyze Tools → MAVLink Inspector |
| Tốc độ mô phỏng (phải ≈1.0) | `gz topic -e -t /stats -n 1 \| grep real_time_factor` |
| Sim có bị giật không | console in `Accel #0 fail: TIMEOUT` ⇒ chuyến bay đó không dùng được |
| Xem lại sau khi bay | kéo file `.ulg` vào https://review.px4.io hoặc `ulog_info`, `ulog2csv` |

## Chạy tự động (lặp nhiều lần)

Hai runner dưới đây tự bật SITL ở chế độ headless, tự bỏ chuyến bay bị sim giật và
tự restart khi drone mất kiểm soát. Không cần Terminal 1.

```bash
source env.sh

# Lặp N lần cùng một chuyến bay, gom số liệu vào results/<name>.csv
python3 -m px4_auto_tuning.run_trials --name baseline_figure8 -n 10 -- --kind figure8 --laps 2

# Quét một gain trong một chuyến bay liên tục
python3 -m px4_auto_tuning.sweep_runner --param MPC_XY_P --values 0.2:2.0:0.2 --repeats 3
```

- `--gui`: vẫn tự bật SITL nhưng mở cửa sổ Gazebo để nhìn.
- `--attach`: dùng SITL bạn đã tự bật ở Terminal 1 (có console, có GUI). Runner không
  đụng tới mô phỏng; nếu drone rơi thì dừng lại và báo bạn restart bằng tay.
- Tiến độ in ra màn hình; log PX4 của chế độ tự động ở `run/px4.log` (`tail -f run/px4.log`).

## Cấu trúc

| Đường dẫn | Nội dung |
|---|---|
| `px4_auto_tuning/trajectories.py` | Quỹ đạo sin / số 8, có ramp tốc độ |
| `px4_auto_tuning/trajectory_node.py` | Node ROS 2 bay offboard, ghi mốc thời gian từng pha |
| `px4_auto_tuning/param_client.py` | Đọc/ghi param qua MAVLink |
| `px4_auto_tuning/metrics.py` | Sai số tracking / estimation / true từ ulog |
| `px4_auto_tuning/run_trials.py` | Chạy lặp, xuất bảng kết quả, bỏ chuyến bay bị sim giật |
| `px4_auto_tuning/sweep_runner.py` | Quét gain khi đang bay, chỉ restart khi drone mất kiểm soát |
| `px4_auto_tuning/sim.py` | Bật/tắt/restart SITL |
| `notes/` | Ghi chú kỹ thuật từng giai đoạn |
| `results/` | Bảng kết quả (csv) |
| `firmware_patches/` | Patch cho cây PX4 |

## Patch firmware

Các thay đổi trong cây PX4 nằm ở `firmware_patches/`, áp lên base commit `583e1cdb47`:

```bash
cd <PX4> && git apply examples/px4-auto-tuning/firmware_patches/*.patch && make px4_sitl
```

| Patch | Nội dung |
|---|---|
| `0001-gz-bridge-gps-params` | GPS mô phỏng chỉnh được khi đang chạy: `SIM_GZ_GPS_PNOIS`, `SIM_GZ_GPS_VNOIS` (biên độ noise), `SIM_GZ_GPS_EPH`, `SIM_GZ_GPS_EPV` (độ chính xác báo cho EKF), `SIM_GZ_GPS_DIV` (chia tần số 30 Hz). Mặc định giữ nguyên hành vi cũ. |

## Lưu ý môi trường

- `env.sh` đặt `ROS_LOCALHOST_ONLY=0`: agent bản snap quảng bá địa chỉ không phải
  loopback, ROS ở chế độ localhost-only sẽ không thấy topic `/fmu/*`.
- `sitl_setup.params` tắt yêu cầu GCS/RC (`NAV_DLL_ACT=0`, `COM_RCL_EXCEPT=4`) để
  arm được khi không có QGroundControl. Gain và estimator giữ mặc định.
