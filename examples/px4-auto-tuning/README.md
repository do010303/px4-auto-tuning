# px4-auto-tuning

Nghiên cứu ảnh hưởng của gain (Kp/Ki/Kd) lên sai số bay của PX4 và cách tìm/chỉnh
gain ngay khi đang bay, trên Gazebo SITL với x500.

## Cài đặt

Repo này là một bản PX4-Autopilot (base `583e1cdb47`) kèm thay đổi firmware và thư mục
nghiên cứu `examples/px4-auto-tuning/`.

```bash
git clone --recursive git@github.com:do010303/px4-auto-tuning.git PX4
cd PX4 && make px4_sitl
pip install --user pyulog pymavlink
```

Cần thêm: Gazebo Harmonic, ROS 2 Humble, một workspace đã build `px4_msgs`
(mặc định `~/px4_ws`, đổi bằng biến `PX4_ROS_WS`), và `micro-xrce-dds-agent`.

## Chạy thủ công (có GUI, tự giám sát)

Mỗi bước một terminal, đứng ở `examples/px4-auto-tuning/`.

**Terminal 1 — mô phỏng.** Mở Gazebo có GUI, PX4 chạy foreground với console `pxh>`,
kèm DDS agent. Rootfs riêng trong `./run`; mỗi lần chạy param bắt đầu lại từ mặc định
(bộ param lần trước giữ ở `run/rootfs/parameters.bson.prev`; `KEEP_PARAMS=1 ./run_sitl.sh` để dùng tiếp).

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

### Lấy số đo của ba bài bằng tay

Cùng code với lúc chạy tự động; khác ở chỗ bạn tự bật/tắt mô phỏng. Mỗi lượt đo mở lại `./run_sitl.sh`
để param về mặc định. Kết quả ghi đè file cùng tên trong `results/`, nên đặt `--name` / `--out` mới
nếu muốn giữ số cũ.

**1. Ma trận model × mức GPS × quỹ đạo** (một ô = một lần chạy; được tua nhanh):

```bash
# Terminal 1: model chọn ở đây; tốc độ ở đây và ở --speed bên dưới phải bằng nhau
HEADLESS=1 PX4_SIM_SPEED_FACTOR=4 PX4_SIM_MODEL=gz_x500_heavy ./run_sitl.sh
# Terminal 2
source env.sh
python3 -m px4_auto_tuning.sweep_runner --attach --speed 4 --profile forest \
    --param MPC_XY_P,MPC_XY_VEL_P_ACC --values "0.95,2;1.8,5" --repeats 2 --settle-laps 1 --measure-laps 2 \
    --name matrix_x500_heavy_forest_sine_tay -- --kind sine
#   GPS sạch: bỏ --profile · đô thị: thêm --set EKF2_REQ_EPH=10 --set EKF2_REQ_EPV=15
python3 results/raw/matrix/summary.py
```

Chỉ chạy Terminal 2 khi Terminal 1 đã in `Ready for takeoff` (hoặc `PX4 SITL ready`) và vẫn đang chạy; nếu không sẽ gặp
`TimeoutError: no MAVLink heartbeat on udpin:0.0.0.0:14540`. Profile GPS được đặt khi drone đã bay vòng đầu, không đặt
trên mặt đất (GPS nhiễu trên mặt đất làm EKF2 từ chối GPS và drone rơi về chế độ giữ độ cao).

Có GUI thì nên để tốc độ 1 hoặc 2: vẽ hình tốn CPU, ở 4× dễ giật và lượt đo bị loại. Nếu drone rơi hoặc
sim giật giữa chừng, runner dừng (nó không tự khởi động lại được SITL bạn mở): tắt, mở lại Terminal 1,
chạy lại lệnh kèm `--resume`.

**2. Trễ đường ROS ↔ PX4** (bắt buộc tốc độ thật, drone phải đang nằm đất vì công cụ gửi lệnh disarm):

```bash
./run_sitl.sh                                   # Terminal 1, tốc độ 1
python3 -m px4_auto_tuning.dds_latency --out results/dds_latency_tay.json   # Terminal 2
```

**3. Tần số gửi tối thiểu** (bắt buộc tốc độ thật: ngưỡng mất offboard chạy theo đồng hồ thật). Mỗi lệnh
là một lần cất cánh – thử – hạ cánh; chạy liền nhau được trên cùng một SITL, kết quả in ra dòng `RESULT`:

```bash
for r in 2 1 0.8 0.5; do python3 -m px4_auto_tuning.layer_test --level position --rate $r --hold 6; done
python3 -m px4_auto_tuning.layer_test --level body_rate --rate 0.8 --hold 6
# ROS tự đóng vòng vị trí, trong gió: Terminal 1 mở bằng PX4_GZ_WORLD=windy ./run_sitl.sh
python3 -m px4_auto_tuning.layer_test --level velocity --closed-loop --rate 1 --hold 20
```

Số tham chiếu (chạy tự động, 07-10): ma trận `results/matrix_*.csv`; trễ khứ hồi 12.0 ms, PX4 → ROS dao động
p99 0.34 ms; giữ offboard ở 1 Hz, rời sau ~1.1 s ở 0.8 Hz. Ô có GPS sạch lặp lại trong khoảng ±0.02 m; ô có
GPS nhiễu dao động nhiều hơn vì mỗi lần bay gặp noise khác.

### Một phiên Gazebo cho nhiều kịch bản

Không cần tắt mô phỏng giữa các kịch bản. Chỉ chạy một `run_sitl.sh` trên máy tại một thời điểm; lần mở thứ hai
và các lệnh tự động thiếu `--attach` sẽ từ chối thay vì tắt phiên đang chạy.

```bash
./reset_params.sh          # gain, GPS mô phỏng và thiết lập GPS của EKF về mặc định (giữa hai kịch bản)
./restart_agent.sh         # ROS không còn thấy /fmu (thường sau khi Wi-Fi đổi mạng): chỉ khởi động lại DDS agent
tail run/px4_console.log   # mọi thứ PX4 in ra ở Terminal 1 và mã thoát của nó
```

Khi có giao diện Gazebo, `px4_auto_tuning/gz_viz.py` tự chạy và vẽ vào cảnh 3D: đường setpoint (xanh dương) và đường
bay thật tô theo mức rung (xanh lá → vàng → đỏ ở 0.3 rad/s). `NO_VIZ=1 ./run_sitl.sh` để tắt;
`python3 -m px4_auto_tuning.gz_viz --color error` tô theo khoảng cách tới setpoint.

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

# Vẽ kết quả quét (results/sweep_<param>.png)
python3 -m px4_auto_tuning.plots results/sweep_MPC_XY_P.csv --baseline results/baseline_figure8.csv
```

- Nhiều tham số thành lưới: `--param MPC_XY_P,MPC_XY_VEL_P_ACC --values "1,2,3;1.8,3.5,5"`.
- `--set NAME=VALUE`: giữ cố định một tham số khác trong cả lượt quét (ví dụ mức noise GPS).
- `--model x500_heavy`: dùng biến thể model (xem bên dưới). `--world windy`: đổi world.
- `--resume`: chạy tiếp một lượt quét bị ngắt, giữ các điểm đã đo.
- `--speed 4`: tua nhanh mô phỏng 4 lần. Đã kiểm chứng tới 4× kết quả không đổi; 8× thì máy không theo kịp.
- `--gui`: vẫn tự bật SITL nhưng mở cửa sổ Gazebo để nhìn.
- `--attach`: dùng SITL bạn đã tự bật ở Terminal 1 (có console, có GUI). Runner không
  đụng tới mô phỏng; nếu drone rơi thì dừng lại và báo bạn restart bằng tay.
- Tiến độ in ra màn hình; log PX4 của chế độ tự động ở `run/px4.log` (`tail -f run/px4.log`).

## Các bài khác

```bash
source env.sh

# Biến thể x500 (nặng hơn, tay dài hơn, motor yếu): sinh vào ./models
python3 make_models.py
PX4_SIM_MODEL=gz_x500_heavy ./run_sitl.sh        # hoặc --model x500_heavy trên các runner

# Bỏ dần từng tầng controller của PX4 và xem drone trôi thế nào (7 mức offboard)
python3 -m px4_auto_tuning.layer_test --level all --silence 8
python3 -m px4_auto_tuning.layer_test --level all --world windy --out results/layer_test_windy.csv
python3 -m px4_auto_tuning.layer_test --level attitude           # một mức, trên SITL đang chạy

# Gửi chậm dần để tìm tần số tối thiểu PX4 còn nhận (chạy ở tốc độ thật)
python3 -m px4_auto_tuning.layer_test --level all --levels position,body_rate --rate 2,1,0.8,0.5 --hold 6 --out results/layer_test_rate_1x.csv
# ROS tự đóng vòng vị trí bằng P ở mức velocity, trong gió
python3 -m px4_auto_tuning.layer_test --level all --levels velocity --closed-loop --rate 50,10,2,1 --hold 20 --world windy --out results/layer_test_velocity_loop.csv
# Độ trễ và dao động đường ROS <-> PX4, trên SITL đang chạy ở tốc độ thật
python3 -m px4_auto_tuning.dds_latency

# Tune gain vị trí/vận tốc ngay khi đang bay, có watchdog và rollback
python3 -m px4_auto_tuning.online_tuner --sitl --speed 4
python3 -m px4_auto_tuning.online_tuner                          # trên SITL đang chạy, tốc độ thật

# Như trên nhưng chọn gain thử tiếp theo bằng tối ưu Bayes, trong vùng --bounds
python3 -m px4_auto_tuning.bayes_tuner --sitl --speed 4 --trials 25
python3 -m px4_auto_tuning.bayes_tuner --sitl --speed 4 --trials 35 --gains MPC_XY_P,MPC_XY_VEL_P_ACC,MPC_XY_VEL_I_ACC

# Chạy autotune attitude/rate có sẵn của PX4 trong lúc hover và đọc kết quả
python3 -m px4_auto_tuning.px4_autotune --sitl

# Autotune theo đúng trình tự của QGroundControl (cất cánh, Hold, tune, hạ cánh để ghi gain).
# Cần một phiên SITL mới cho mỗi lần chạy; xem notes/09-qgc-autotune.md
HEADLESS=1 PX4_DAEMON=1 PX4_PARAM_MC_AT_SYSID_AMP=2.7 ./run_sitl.sh    # terminal 1
python3 -m px4_auto_tuning.qgc_autotune --out results/raw/qgc_autotune.json   # terminal 2
```

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
| `px4_auto_tuning/plots.py` | Vẽ kết quả quét gain |
| `px4_auto_tuning/tuner_plots.py` | Biểu đồ của các bộ tune khi đang bay: hội tụ, bản đồ tìm kiếm, trước / sau |
| `px4_auto_tuning/layer_test.py` | Bài test các tầng controller qua các mức offboard |
| `px4_auto_tuning/gz_viz.py` | Vẽ setpoint và đường bay thật (tô theo rung / sai số) vào Gazebo |
| `reset_params.sh`, `restart_agent.sh` | Giữ một phiên mô phỏng dùng lâu: đưa param về mặc định, khởi động lại DDS agent |
| `px4_auto_tuning/dds_latency.py` | Độ trễ và dao động đường uXRCE-DDS |
| `px4_auto_tuning/online_tuner.py` | Tune gain vị trí/vận tốc khi đang bay |
| `px4_auto_tuning/bayes_tuner.py` | Như `online_tuner.py` nhưng chọn gain bằng tối ưu Bayes |
| `px4_auto_tuning/px4_autotune.py` | Chạy và đọc kết quả autotune attitude có sẵn của PX4 |
| `px4_auto_tuning/qgc_autotune.py` | Autotune của PX4 theo trình tự của QGroundControl, ghi gain khi disarm |
| `make_models.py`, `models/` | Biến thể vật lý của x500 và param PX4 đi kèm |
| `profiles/` | Profile GPS theo môi trường (số liệu giả định), dùng với `sweep_runner --profile` |
| `notes/` | Ghi chú kỹ thuật từng giai đoạn |
| `results/` | Bảng kết quả (csv) |

## Thay đổi firmware so với PX4 gốc

| Vị trí | Nội dung |
|---|---|
| `src/modules/simulation/gz_bridge/` | GPS mô phỏng chỉnh được khi đang chạy: `SIM_GZ_GPS_PNOIS`, `SIM_GZ_GPS_VNOIS` (biên độ noise), `SIM_GZ_GPS_EPH`, `SIM_GZ_GPS_EPV` (độ chính xác báo cho EKF), `SIM_GZ_GPS_DIV` (chia tần số 30 Hz). Mặc định giữ nguyên hành vi cũ. |

## Lưu ý môi trường

- `run_sitl.sh` đặt `GZ_IP=127.0.0.1`: mặc định Gazebo gửi dữ liệu qua card mạng chính; qua Wi-Fi
  dữ liệu cảm biến bị khựng tới 0.3 s, PX4 mất mẫu IMU và drone có thể rơi (xem `notes/03-simulator-stalls.md`).
- `run_sitl.sh` đặt `SENS_GPS0_DELAY=0`: GPS mô phỏng không có trễ nhưng PX4 mặc định bù 110 ms, làm
  estimate chạy trước drone ~125 ms (xem `notes/04-gps-delay-mismatch.md`).
- `run_sitl.sh` đặt `UXRCE_DDS_SYNCT=0`: timestamp trên ROS là thời gian boot của PX4, khớp với ulog.
- `env.sh` đặt `ROS_LOCALHOST_ONLY=0`: agent bản snap quảng bá địa chỉ không phải
  loopback, ROS ở chế độ localhost-only sẽ không thấy topic `/fmu/*`.
- `sitl_setup.params` tắt yêu cầu GCS/RC (`NAV_DLL_ACT=0`, `COM_RCL_EXCEPT=4`) để
  arm được khi không có QGroundControl. Gain và estimator giữ mặc định.
