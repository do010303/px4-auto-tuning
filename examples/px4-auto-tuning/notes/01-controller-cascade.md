# 01 — Các tầng điều khiển multicopter

Đường dẫn tính từ gốc repo PX4, base commit `583e1cdb47`.

```
trajectory_setpoint (pos, vel, acc)
      │
      ▼  mc_pos_control — chạy mỗi khi có vehicle_local_position mới
 P vị trí      vel_sp = MPC_*_P · (pos_sp − pos) + vel_ff
 PID vận tốc   acc_sp = P·(vel_sp − vel) + ∫I − D·vel_dot + acc_ff
 acc → thrust vector + attitude setpoint
      │
      ▼  mc_att_control — chạy mỗi khi có vehicle_attitude mới
 P quaternion  rate_sp = MC_*_P · eq   (ưu tiên roll/pitch hơn yaw)
      │
      ▼  mc_rate_control — chạy mỗi khi có vehicle_angular_velocity mới
 PID + FF      torque = P·e + ∫I − D·ang_accel + FF·rate_sp
      │
      ▼  control_allocator → lệnh motor → GZBridge → Gazebo
```

## Position / velocity (`src/modules/mc_pos_control/PositionControl/PositionControl.cpp`)

- `_positionControl()` (dòng 128): chỉ có **P**. Vận tốc feedforward từ setpoint được
  cộng vào; khi vượt `MPC_XY_VEL_MAX` thì phần do sai số vị trí được ưu tiên hơn feedforward.
- `_velocityControl()` (dòng 144): PID, trong đó **D tác động lên gia tốc đo được**
  (`_vel_dot`), không phải đạo hàm sai số — setpoint nhảy không gây D-kick.
- Anti-windup: trục Z ngừng tích phân khi thrust chạm giới hạn; trục XY dùng
  tracking anti-windup với hệ số `2 / MPC_XY_VEL_P_ACC` (dòng 189-195). Nghĩa là
  **đổi `VEL_P` cũng đổi hành vi anti-windup**, hai gain này không độc lập khi bão hoà.
- Thrust ngang ↔ gia tốc đổi qua `_hover_thrust` (`MPC_THR_HOVER` hoặc bộ ước lượng
  hover thrust). Hover thrust sai làm gain vận tốc hiệu dụng sai theo cùng tỉ lệ.
- Đơn vị gain vận tốc là gia tốc (`*_ACC`): m/s² trên mỗi m/s sai số.

Tần số đo được trong ulog SITL: `vehicle_local_position` 125 Hz → position loop 125 Hz.

## Attitude (`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`)

Chỉ có P trên sai số quaternion (dòng 95), `MC_YAW_WEIGHT` giảm ưu tiên yaw, đầu ra
kẹp theo giới hạn rate.

## Rate (`src/lib/rate_control/rate_control.cpp:70-86`)

`torque = P·e + I + FF·rate_sp − D·angular_accel`. Gain đi qua dạng "K":
`MulticopterRateControl.cpp:80-88` nhân `MC_*RATE_K` vào cả P, I, D — K đổi độ lớn
tổng, P/I/D giữ hình dạng. Integrator dừng theo cờ bão hoà từ control allocator và bị
giảm khi sai số rate lớn (dòng 91-107).

## Autotune có sẵn (`src/modules/mc_autotune_attitude_control/`)

- Chỉ cho **rate + attitude**, lần lượt roll → pitch → yaw.
- Bơm tín hiệu kích thích vào rate setpoint, nhận dạng mô hình ARX bậc 2
  torque → angular rate bằng RLS (`src/lib/system_identification/`).
- Tính PID bằng `pid_design::computePidGmvc()` với rise time mong muốn
  `MC_AT_RISE_TIME` (yaw cố định 0.2 s) (`mc_autotune_attitude_control.cpp:194-196`).
- Attitude P suy theo quy tắc kinh nghiệm "sai 60° cho đầu ra tối đa":
  `K_att = 1 / (rad(60°)·K_rate)`, kẹp trong 2–6.5 (dòng 204).
- `MC_AT_APPLY`: 0 = chỉ log, 1 = áp sau khi disarm (mặc định), 2 = áp trên không.
- **Không có gì tương tự cho position/velocity loop.**

## Offboard: PX4 bỏ tầng nào theo cờ nào

`src/modules/commander/ModeUtil/control_mode.cpp:123-164` — cờ đầu tiên bật trong
`OffboardControlMode` quyết định các tầng còn chạy:

| Cờ | Tầng PX4 còn chạy |
|---|---|
| `position` | position, velocity, acceleration, attitude, rate, allocation |
| `velocity` | velocity, acceleration, attitude, rate, allocation |
| `acceleration` | acceleration, attitude, rate, allocation |
| `attitude` | attitude, rate, allocation |
| `body_rate` | rate, allocation |
| `thrust_and_torque` | allocation |
| `direct_actuator` | không tầng nào |
