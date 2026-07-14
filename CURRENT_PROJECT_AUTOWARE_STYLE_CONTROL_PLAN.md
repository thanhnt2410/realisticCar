# Kế hoạch chuyển project hiện tại sang cấu trúc longitudinal control kiểu Autoware

## 1. Mục tiêu

Chuyển project `realisticCar` từ kiến trúc điều khiển trực tiếp vận tốc:

```text
cmd_vel = target_velocity + velocity_correction
```

sang kiến trúc longitudinal control gần với Autoware:

```text
Planning
  -> reference velocity + reference acceleration
  -> PID hoặc Fuzzy PID
  -> acceleration correction
  -> final target acceleration
  -> longitudinal vehicle model/interface
  -> vehicle velocity
  -> cmd_vel chỉ dùng để hiển thị Prius trong Gazebo
```

Mục tiêu thứ hai là tạo phép thử có ý nghĩa. PID và Fuzzy PID phải chịu cùng actuator dynamics, độ dốc, tải và chuỗi nhiễu có seed, thay vì điều khiển một DiffDrive gần như bám vận tốc lý tưởng.

## 2. Quyết định kiến trúc

### 2.1. Đổi cả PID và Fuzzy PID

Không chỉ đổi Fuzzy PID. PID thường cũng phải dùng cùng input/output mới để phép so sánh công bằng.

Hai controller chỉ được khác nhau ở luật feedback:

```text
PID:       e -> fixed Kp/Ki/Kd -> acceleration correction
Fuzzy PID: e -> adaptive Kp/Ki/Kd -> acceleration correction
```

Mọi phần còn lại phải giống nhau:

- Reference acceleration.
- Acceleration limit.
- Jerk limit.
- Anti-windup.
- Điều kiện reset/khóa tích phân.
- Input timestamps và control period.
- Vehicle model và disturbance.

### 2.2. Không thêm dependency Autoware vào workspace hiện tại

Tạo package message cục bộ `car_control_msgs` có cấu trúc tương tự Autoware. Cách này giúp project build độc lập trước khi tích hợp Autoware thật.

Message cục bộ chỉ **shape-compatible về ý nghĩa**, không wire-compatible với namespace/type của Autoware. Khi tích hợp Autoware thật sẽ dùng adapter hoặc thay bằng message chính thức.

### 2.3. Gazebo Prius chỉ làm visualization trong giai đoạn này

Plugin `DiffDrive` hiện nhận vận tốc trực tiếp và gần như là inner velocity servo lý tưởng. Vì vậy nó che mất phần lớn khác biệt giữa PID và Fuzzy PID.

Thêm một longitudinal vehicle simulator 1D làm plant có thẩm quyền. Plant tính vận tốc thật, còn Gazebo nhận vận tốc đó qua `/cmd_vel` để xe Prius di chuyển theo kết quả plant.

Giữ hai chế độ launch:

```text
vehicle_model:=longitudinal_sim   # mặc định cho đánh giá controller
vehicle_model:=ideal_diff_drive   # chế độ cũ để smoke test/GUI
```

## 3. Contract topic và message mới

### 3.1. Planning reference

Thêm `car_control_msgs/msg/LongitudinalReference.msg`:

```text
builtin_interfaces/Time stamp
float32 velocity
float32 acceleration
float32 jerk
bool is_defined_acceleration
bool is_defined_jerk
```

Topic:

```text
/planning/longitudinal_reference
```

### 3.2. Controller output

Thêm `car_control_msgs/msg/Longitudinal.msg`, bám theo cấu trúc `autoware_control_msgs/msg/Longitudinal`:

```text
builtin_interfaces/Time stamp
builtin_interfaces/Time control_time
float32 velocity
float32 acceleration
float32 jerk
bool is_defined_acceleration
bool is_defined_jerk
```

Topic:

```text
/control/trajectory_follower/longitudinal_cmd
```

Ý nghĩa:

- `velocity`: vận tốc mục tiêu từ planning.
- `acceleration`: gia tốc cuối cùng sau feed-forward, feedback và limits.
- `jerk`: tốc độ thay đổi gia tốc sau jerk limiting.

Acceleration correction không phải lệnh cuối cùng. Publish riêng để debug:

```text
/control/trajectory_follower/acceleration_correction
```

### 3.3. Vehicle state

```text
/simulation/ground_truth/odometry   # vận tốc thật để tính metrics
/localization/kinematic_state       # vận tốc đo/noisy cho controller
/localization/acceleration          # gia tốc đo/applied
/simulation/gazebo_odometry         # chỉ chẩn đoán Gazebo visualization
/simulation/gazebo_cmd_vel          # đầu ra plant đưa vào DiffDrive
```

Noise chỉ được thêm vào feedback đo hoặc process disturbance. Không thêm noise vào target velocity của planning.

## 4. Công thức điều khiển mới

Với cả PID và Fuzzy PID:

```text
e_v = v_ref - v_measured

a_correction =
    Kp * e_v
    + Ki * integral(e_v)
    + Kd * derivative(e_v)

a_unlimited = a_ref + a_correction
a_target = acceleration_and_jerk_limiter(a_unlimited)
```

Fuzzy PID thay `Kp`, `Ki`, `Kd` bằng gain thích nghi nhưng vẫn trả acceleration correction `[m/s²]`.

Đơn vị gain mới:

| Gain | Đơn vị |
|---|---|
| `Kp` | `1/s` |
| `Ki` | `1/s²` |
| `Kd` | không thứ nguyên |

Không tái sử dụng trực tiếp gain hiện tại vì gain cũ tạo velocity correction `[m/s]`.

## 5. Longitudinal vehicle simulator

Thêm node/package `longitudinal_vehicle_simulator`.

### Input

```text
/control/trajectory_follower/longitudinal_cmd
```

### Mô hình tối thiểu

Lệnh gia tốc đi qua:

1. Command delay.
2. Acceleration clamp.
3. First-order actuator lag.
4. Jerk/rate limit.
5. Grade, rolling resistance và aerodynamic drag.
6. Process disturbance có seed.
7. Tích phân fixed-step để tạo vận tốc thật.

Phương trình tổng quát:

```text
a_total =
    a_applied
    - g * sin(grade)
    - rolling_resistance_acceleration
    - aerodynamic_drag_acceleration
    + process_disturbance_acceleration

v_true[k+1] = max(0, v_true[k] + a_total * dt)
```

Grade phần trăm được đổi thành góc:

```text
grade_angle = atan(grade_percent / 100)
```

Ví dụ `+5%` tạo disturbance xấp xỉ `-0.49 m/s²`.

### Output

```text
/simulation/ground_truth/odometry
/localization/kinematic_state
/localization/acceleration
/simulation/gazebo_cmd_vel
/simulation/vehicle_dynamics/debug
```

Sensor noise/latency chỉ thay đổi `/localization/kinematic_state`, không được thay đổi trực tiếp ground truth.

RNG phải persistent và dựa trên `random_seed` + simulation step; không seed bằng system clock.

## 6. Thay đổi planning

`trapezoid_velocity_profile` sẽ publish đồng thời `v_ref`, `a_ref` và `jerk_ref`.

Trong ramp tuyến tính:

```text
v_ref = v0 + alpha * (v1 - v0)
a_ref = (v1 - v0) / ramp_duration
```

Trong hold:

```text
a_ref = 0
```

Không tính `a_ref` bằng sai phân các sample `v_ref`, vì timer jitter sẽ đưa nhiễu không mong muốn vào feed-forward.

Profile tuyến tính có bước nhảy gia tốc tại biên ramp/hold, tương đương jerk lý tưởng vô hạn. Giai đoạn đầu để controller jerk limiter xử lý; giai đoạn sau có thể đổi planning sang S-curve.

Profile hiện tại với ramp `3 s` đòi hỏi tới `+3.33 m/s²`. Nếu muốn giới hạn gần xe thật `+2/-3 m/s²`, đổi `ramp_duration` tối thiểu thành `5 s`. Runner phải tự tính lại tổng thời lượng profile.

## 7. Các phase triển khai

### Phase 0 - Đóng băng baseline

- Build project hiện tại.
- Lưu config và một cặp log PID/Fuzzy hiện tại làm legacy baseline.
- Ghi rõ legacy controller tạo velocity correction.
- Không dùng lại log legacy trong thống kê của kiến trúc mới.

Tiêu chí hoàn thành:

- Có manifest ghi commit/config/profile của baseline.
- Có thể phân biệt rõ CSV schema cũ và mới.

### Phase 1 - Thêm message contract

- Tạo package `car_control_msgs`.
- Thêm `LongitudinalReference.msg` và `Longitudinal.msg`.
- Thêm rosidl/CMake/package dependencies.
- Viết smoke test kiểm tra message build và publish/subscribe.

Tiêu chí hoàn thành:

- Message có đúng trường và đơn vị.
- Workspace build thành công nhưng đường điều khiển cũ chưa bị xóa.

### Phase 2 - Sửa planning

- Đổi planning từ `std_msgs/Float64` sang `LongitudinalReference`.
- Tính `v_ref/a_ref` theo công thức giải tích.
- Publish timestamp theo simulation clock.
- Cập nhật config, launch và test boundary của profile.

Tiêu chí hoàn thành:

- Ramp có gia tốc hằng đúng giá trị.
- Hold có gia tốc bằng 0.
- Profile bắt đầu đúng khi `/clock` hợp lệ.

### Phase 3 - Refactor PID/Fuzzy core

- Tách PID và Fuzzy PID core khỏi ROS node để unit test trực tiếp.
- Đổi output thành acceleration correction.
- Thêm conditional anti-windup cho cả hai.
- Thêm reset khi dừng/mất target.
- Thêm acceleration và jerk limiter dùng chung.
- Đổi tên parameter để thể hiện đúng đơn vị.

Parameter mới dự kiến:

```text
max_acceleration_correction
min_target_acceleration
max_target_acceleration
min_jerk
max_jerk
max_integral_error
derivative_filter_alpha
control_period_ms
```

Tiêu chí hoàn thành:

- Cả hai controller nhận cùng reference/state.
- Cả hai publish cùng `Longitudinal` type.
- Controller không publish `/cmd_vel` trực tiếp.
- Debug P/I/D của cả hai có đơn vị `[m/s²]`.

### Phase 4 - Thêm longitudinal vehicle simulator

- Thêm plant fixed-step theo simulation time.
- Thêm delay, lag, accel/jerk limits, grade, drag và rolling resistance.
- Thêm process/sensor noise có seed.
- Tách true odometry và measured odometry.
- Plant là node duy nhất publish vận tốc cho Gazebo visualization.

Tiêu chí hoàn thành:

- `Kp=Ki=Kd=0` không còn đồng nghĩa bám vận tốc lý tưởng.
- Cùng seed cho cùng trace nhiễu.
- Khác seed cho trace đo khác nhau.
- Noise sensor không làm thay đổi true state.

### Phase 5 - Sửa launch và bridge

- Bridge Gazebo odom về `/simulation/gazebo_odometry` để chẩn đoán.
- Bridge `/simulation/gazebo_cmd_vel` sang Gazebo `/cmd_vel`.
- Start plant trước planning.
- Thêm launch arguments:

```text
vehicle_model
scenario_file
random_seed
headless
log_file_path
```

- Giữ `ideal_diff_drive` làm smoke mode; không dùng mode này để đánh giá PID/Fuzzy.

### Phase 6 - Sửa logger, plot và metrics

Logger mới ghi sample theo plant/debug timestamp thay vì ghép các last-value bằng wall timer.

CSV schema tối thiểu:

```text
time_sec,sim_step,scenario,seed,controller,
target_velocity_mps,reference_acceleration_mps2,
true_velocity_mps,measured_velocity_mps,
true_velocity_error_mps,controller_velocity_error_mps,
acceleration_correction_mps2,
unlimited_target_acceleration_mps2,target_acceleration_mps2,
applied_acceleration_mps2,jerk_mps3,cmd_vel_mps,
grade_percent,grade_acceleration_mps2,
drag_acceleration_mps2,rolling_acceleration_mps2,
process_disturbance_acceleration_mps2,
adaptive_kp,adaptive_ki,adaptive_kd,
p_term_mps2,i_term_mps2,d_term_mps2,
correction_saturated
```

Tracking metrics phải dùng `true_velocity`. Measured velocity chỉ dùng để đánh giá feedback/noise.

Plot đề xuất:

1. `v_ref`, `v_true`, `v_measured`.
2. `a_ref`, `a_correction`, `a_target`, `a_applied`.
3. True error và controller error.
4. Grade/load/noise contributions.
5. Adaptive gains và P/I/D effort.

### Phase 7 - Sửa simulation runner

CLI dự kiến:

```text
--scenario baseline|grade_step|sensor_noise|combined|all
--seeds 1001:1010
--sim-duration auto
--wall-timeout <seconds>
--headless
--output-dir <path>
```

Runner phải:

- Dừng theo simulation time trong log, không sleep một khoảng wall-time cố định.
- Tính thời lượng từ profile cộng settling margin.
- Reset plant/Gazebo hoàn toàn giữa PID và Fuzzy.
- Chạy PID/Fuzzy theo cặp với cùng scenario và seed.
- Lưu parameter/config snapshot và `manifest.json`.
- Không trộn log schema cũ với log mới.

Cấu trúc output:

```text
logs/experiments/<experiment_id>/<scenario>/<seed>/
|-- fuzzy_pid.csv
|-- pid.csv
`-- manifest.json
```

## 8. Scenario kiểm thử

### baseline

- Actuator lag, acceleration/jerk limits.
- Không grade, không noise.
- Deterministic; chạy một lượt chức năng, có thể lặp thêm một lần để kiểm reproducibility.

### grade_step

```text
0% -> +5% -> 0% -> -5% -> 0%
```

- Không noise.
- Đánh giá maximum deviation và recovery time.
- Đây là road-grade disturbance tổng hợp; mặt đường Gazebo phẳng sẽ không nghiêng về mặt hình ảnh.

### sensor_noise

- Velocity noise sigma dự kiến `0.05 m/s`.
- Process noise tùy chọn `0.05 m/s²`.
- Chạy tối thiểu 10 paired seeds, ví dụ `1001..1010`.

### combined

- Grade step.
- Sensor/process noise.
- Actuator delay khoảng `0.2 s`.
- Load disturbance pulse, ví dụ `-0.8 m/s²`.
- Chạy tối thiểu 10 paired seeds.

Giá trị trên là mặc định phục vụ test, chưa phải tham số đã nhận dạng từ Prius thật.

## 9. Bộ test cần thêm

Hiện tại CMake mới chỉ chạy lint; chưa có unit/integration test thực sự.

### C++ unit test

`test_fuzzy_pid_core.cpp`:

- Zero error.
- Đúng dấu correction.
- Correction bound.
- Anti-windup.
- Reset.
- Invalid `dt` không tạo NaN.
- Adaptive gain bounds.
- Output có đơn vị acceleration.

`test_pid_core.cpp`:

- Các test tương ứng với PID thường.
- Kiểm tra cùng acceleration/jerk limits với Fuzzy.

`test_longitudinal_profile.cpp`:

- `v_ref/a_ref` tại đầu, giữa và cuối ramp.
- Hold acceleration bằng 0.
- Boundary giữa các segment.

`test_longitudinal_vehicle_model.cpp`:

- Zero command.
- Acceleration/jerk limit.
- Actuator lag và delay.
- Dấu disturbance uphill/downhill.
- Drag/rolling resistance.
- Invalid `dt` và finite output.

`test_seeded_noise.cpp`:

- Sigma 0 luôn bằng 0.
- Cùng seed/step cho cùng output.
- Khác seed cho output khác.
- Sensor noise không đổi true state.

### ROS integration test

`test_longitudinal_pipeline.launch.py`:

- Planning -> controller -> plant có đủ topic.
- Command timestamp hợp lệ.
- Acceleration và jerk bounded.
- Không có NaN/Inf.
- Grade tạo open-loop error và closed loop có khả năng reject disturbance.

### Python metrics test

- MAE/RMSE/IAE trên CSV giả lập.
- Phát hiện thiếu cột/schema sai.
- Pair đúng PID/Fuzzy theo scenario và seed.
- Same seed reproducible, different seed khác measured trace.

## 10. Metrics so sánh

Mỗi run:

- True-velocity MAE, RMSE, maximum error và IAE.
- Steady-state MAE.
- Overshoot và settling time.
- Stop residual/creeping.
- Grade maximum deviation và recovery time.
- Correction RMS/max và saturation ratio.
- Applied acceleration/jerk RMS và peak.
- CPU/callback latency nếu cần chạy trên mini PC.

Với stochastic scenario:

- Mean, standard deviation, median và p95.
- Paired delta `Fuzzy - PID` theo cùng seed.
- Chỉ kết luận thống kê từ các paired stochastic runs.

## 11. File dự kiến thay đổi

### Thêm mới

```text
src/car_control_msgs/
src/car_controller/include/.../pid_core.hpp
src/car_controller/include/.../fuzzy_pid_core.hpp
src/car_controller/include/.../longitudinal_vehicle_model.hpp
src/car_controller/src/longitudinal_vehicle_simulator.cpp
src/car_controller/config/vehicle_simulator.yaml
src/car_controller/config/simulation_scenarios/*.yaml
src/car_controller/test/*
```

### Sửa

```text
src/car_planning/src/trapezoid_velocity_profile.cpp
src/car_planning/config/trapezoid_velocity_profile.yaml
src/car_controller/src/fuzzy_pid_controller.cpp
src/car_controller/src/pid_velocity_controller.cpp
src/car_controller/src/velocity_logger.cpp
src/car_controller/config/fuzzy_pid_controller.yaml
src/car_controller/launch/*.launch.py
src/car_controller/worlds/prius_empty.sdf
src/car_controller/CMakeLists.txt
src/car_controller/package.xml
src/car_planning/CMakeLists.txt
src/car_planning/package.xml
run_controller_simulations.py
logs/plot_velocity_log.py
```

`noisy_controller.cpp` sẽ không còn nằm trong đường test chính vì hiện seed theo system clock và không phù hợp pipeline Prius mới. Có thể giữ lại làm legacy hoặc loại khỏi build sau khi pipeline mới ổn định.

## 12. Điều kiện nghiệm thu

Thay đổi được xem là hoàn thành khi:

- Planning xuất vận tốc và gia tốc tham chiếu có timestamp.
- PID và Fuzzy PID nhận cùng input và trả cùng `Longitudinal` output.
- Feedback output của controller là acceleration correction `[m/s²]`.
- Final acceleration bằng reference acceleration cộng correction, sau limits.
- Controller không publish `/cmd_vel` trực tiếp.
- Plant là nơi duy nhất chuyển acceleration command thành vận tốc.
- Ground truth và measured velocity được tách riêng.
- Baseline deterministic tái lập được.
- Cùng stochastic seed cho cùng disturbance trace.
- PID/Fuzzy dùng cùng danh sách seed.
- Grade/noise/load tạo sai số có thể quan sát và controller phải phản ứng.
- Acceleration/jerk không vượt giới hạn.
- Unit, integration và metrics tests đều đạt.
- Runner không phụ thuộc real-time factor để quyết định thời gian kết thúc.

## 13. Thứ tự thực hiện đề xuất

```text
[0] Lưu legacy baseline
        |
[1] Thêm message contract
        |
[2] Planning xuất v_ref + a_ref
        |
[3] Đổi PID/Fuzzy sang acceleration correction
        |
[4] Thêm longitudinal vehicle simulator
        |
[5] Sửa launch/bridge
        |
[6] Sửa logger/plot/metrics
        |
[7] Thêm scenarios và runner có seed
        |
[8] Unit + integration tests
        |
[9] Chạy A/B và tuning gain mới
```

