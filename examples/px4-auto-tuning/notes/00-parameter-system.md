# 00 — Hệ thống parameter của PX4

Đường dẫn tính từ gốc repo PX4, base commit `583e1cdb47`.

## Ba tầng lưu trữ

`src/lib/parameters/parameters.cpp:103-105`:

```cpp
static ConstLayer firmware_defaults;
static DynamicSparseLayer runtime_defaults{&firmware_defaults};
DynamicSparseLayer user_config{&runtime_defaults};
```

| Tầng | Nguồn giá trị | Ai ghi |
|---|---|---|
| `firmware_defaults` | `default:` trong các file `*params.yaml` của module, biên dịch thẳng vào firmware | build |
| `runtime_defaults` | `param set-default` trong airframe / rc script lúc boot | script khởi động |
| `user_config` | `param set`, MAVLink `PARAM_SET`, QGC, autotune | người dùng / runtime |

`param_get()` hỏi `user_config`; tầng nào không có giá trị thì hỏi tầng cha
(`ParamLayer::_parent`). Chỉ `user_config` được ghi xuống file.

Hệ quả cho production: gain của một dòng máy nằm trong **airframe file** dưới dạng
`param set-default` (ví dụ `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`).
Giá trị đó là default của máy, không tính là "người dùng đã đổi", nên firmware mới
có default mới sẽ tự áp dụng. Chỉ những gì tune riêng cho từng chiếc mới nằm ở `user_config`.

## Từ YAML đến code

Mỗi module khai báo param trong YAML (ví dụ
`src/modules/mc_pos_control/multicopter_position_control_gain_params.yaml`) với
`default / min / max / decimal / unit`. Lúc build, `src/lib/parameters/px_process_params.py`
và `px_generate_params.py` sinh ra bảng param + metadata. `min/max` là metadata cho
GCS và tài liệu (cần kiểm chứng bằng thí nghiệm: set giá trị ngoài khoảng xem firmware có kẹp không).

Module C++ dùng `DEFINE_PARAMETERS(...)` và kế thừa `ModuleParams`; `updateParams()`
(`platforms/common/include/px4_platform_common/module_params.h:84`) đọc lại toàn bộ
param của module và các lớp con.

## Chuyện gì xảy ra khi `param set` lúc đang bay

`param_set_internal()` (`parameters.cpp:404`):

1. `user_config.store(param, value)`
2. `param_autosave()` — hẹn ghi file sau ≥300 ms, giới hạn 2 s/lần
   (`autosave.cpp:56-70`). Trên board lưu param vào flash nội (không có file param),
   việc ghi bị **hoãn tới khi disarm** vì ghi flash có thể treo CPU ~300 ms
   (`autosave.cpp:88-96`): gain đổi trên không vẫn có hiệu lực ngay nhưng chưa được lưu
3. `param_notify_changes()` (`parameters.cpp:156`) publish uORB `parameter_update`

Mỗi controller tự kiểm tra topic đó ở đầu vòng lặp:

- `MulticopterPositionControl::parameters_update()` (`MulticopterPositionControl.cpp:75`)
  → `updateParams()` → `setPositionGains()` / `setVelocityGains()` (dòng 198-202)
- `MulticopterRateControl` (`MulticopterRateControl.cpp:116`) → `setPidGains()` (dòng 85)

Không module nào restart, không state nào bị reset (kể cả integrator). Gain mới có
hiệu lực ở chu kỳ điều khiển kế tiếp.

Ngoại lệ: param có `reboot_required: true` (ví dụ `SENS_EN_GPSSIM`) chỉ được đọc lúc
module khởi động.

## Kênh đổi param từ bên ngoài

| Kênh | Dùng được? |
|---|---|
| MAVLink `PARAM_SET` | Có — `px4_auto_tuning/param_client.py` |
| Shell `px4-param set NAME VALUE` (SITL) | Có |
| ROS 2 / uXRCE-DDS | **Không** — `src/modules/uxrce_dds_client/dds_topics.yaml` không có topic/service param |

Lưu ý khi dùng MAVLink: PX4 gửi param kiểu INT32 bằng cách chép nguyên byte vào
trường float, phải `struct.pack/unpack` chứ không ép kiểu số.

## Gain liên quan

| Vòng | Param | Mặc định | min–max (YAML) |
|---|---|---|---|
| Vị trí XY (P) | `MPC_XY_P` | 0.95 | 0–2 |
| Vị trí Z (P) | `MPC_Z_P` | 1.0 | 0.1–1.5 |
| Vận tốc XY | `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 1.8 / 0.4 / 0.2 | 1.2–5 / 0–60 / 0.1–2 |
| Vận tốc Z | `MPC_Z_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 4.0 / 2.0 / 0.0 | 2–15 / 0.2–3 / 0–2 |
| Attitude (P) | `MC_ROLL_P` / `MC_PITCH_P` / `MC_YAW_P` | 4.0 / 4.0 / 2.8 | |
| Rate roll/pitch | `MC_ROLLRATE_P` / `_I` / `_D` (+ `_K`, `_FF`) | 0.15 / 0.2 / 0.003 | |
| Rate yaw | `MC_YAWRATE_P` / `_I` / `_D` | 0.2 / 0.1 / 0.0 | |
