# Plan: Backend `gazebo_effort` cho realisticCar

## 1. Mục tiêu

Thêm một backend mô phỏng dọc dùng torque bánh xe và Gazebo physics làm plant thực sự, trong khi giữ nguyên hợp đồng điều khiển hiện tại:

```text
car_planning
  -> LongitudinalReference {velocity, acceleration, jerk}

PID / Fuzzy PID
  -> Longitudinal {velocity, acceleration, jerk}

vehicle backend
  -> /localization/kinematic_state
```

Backend mới phải đáp ứng:

- PID và Fuzzy PID tiếp tục xuất target acceleration; không đổi core controller.
- `gazebo_effort` nhận `Longitudinal`, chuyển acceleration thành wheel torque và để Gazebo tích phân trạng thái.
- Controller lấy feedback từ Gazebo qua `/localization/kinematic_state`.
- Giữ nguyên `longitudinal_sim` làm baseline deterministic để test và A/B comparison.
- Giai đoạn đầu chỉ điều khiển dọc, hai bánh sau, không làm Ackermann/path following.

## 2. Hiện trạng cần ghi nhớ

- `longitudinal_sim` hiện là plant chính. Nó tính `v_true`, thêm noise measurement và gửi velocity sang Gazebo chỉ để hiển thị.
- Prius hiện được include từ Gazebo Fuel và được gắn `ignition::gazebo::systems::DiffDrive`, nhận `/cmd_vel` và command hai rear wheel joints.
- Model Prius Fuel không có sẵn hybrid engine, drivetrain hoặc `ros2_control` plugin.
- `ideal_diff_drive` chỉ là launch scaffold lỗi thời: controller hiện không publish `/cmd_vel`, mode này cũng không cung cấp `/localization/kinematic_state`.
- Package differential-drive `car_description` cũ không thuộc pipeline Prius và đã được loại khỏi workspace.

## 3. Quyết định kiến trúc

1. Không xóa hoặc thay đổi hành vi của `longitudinal_sim` trong công việc này.
2. Thêm `vehicle_model=gazebo_effort`; không dùng `ideal_diff_drive` làm baseline.
3. Controller giữ nguyên hai topic chuẩn:

   ```text
   input:  /localization/kinematic_state
   output: /control/trajectory_follower/longitudinal_cmd
   ```

4. Mọi backend tự chịu trách nhiệm thực hiện hợp đồng topic trên.
5. `gazebo_vehicle_interface` là nơi hiểu `car_msgs/Longitudinal`; tầng Gazebo/effort controller phải generic và không phụ thuộc message riêng của dự án.
6. Backend chính ưu tiên `gz_ros2_control` với effort interface, nhưng chỉ chốt sau feasibility spike riêng.
7. Không sửa trực tiếp Fuel cache. Model custom phải nằm trong workspace và được quản lý phiên bản nếu license cho phép.
8. Không dùng `JointController` force mode cho phép đo physics thô; dùng `ApplyJointForce` để command torque trực tiếp.
9. Không nhét steering vào `Longitudinal.msg`. Lateral control sẽ là interface riêng trong tương lai.

## 4. Kiến trúc đích

```text
/planning/longitudinal_reference
            |
            v
PID hoặc Fuzzy PID
            |
            | Longitudinal {velocity, acceleration, jerk}
            v
gazebo_vehicle_interface
  - validate command
  - safety watchdog
  - acceleration safety clamp
  - acceleration -> drive force -> rear wheel torque
  - torque clamp / optional actuator model
            |
            | generic effort command
            v
JointGroupEffortController + gz_ros2_control
            |
            v
Prius rear wheel joints -> Gazebo contact physics
            |
            v
OdometryPublisher -> ros_ign_bridge -> odometry_adapter
            |                           |
            |                           +-> /localization/kinematic_state
            +-> /simulation/ground_truth/odometry
```

## 5. Công thức phiên bản đầu

Với hai bánh sau chủ động và command trực tiếp tại wheel joints:

```text
F_drive = m_eff * a_target
tau_each = F_drive * r_wheel / 2
```

Không chia gear ratio vì model chưa có motor shaft hoặc transmission ảo.

Effective mass lý thuyết trong điều kiện bánh lăn không trượt:

```text
m_eff = m_translational_total + sum(I_spin_i / r_i^2)
```

Trong đó:

- `m_translational_total` là tổng mass của tất cả moving links.
- `I_spin_i` phải lấy quanh đúng trục quay của từng bánh.
- Tất cả bánh quay theo xe đều đóng góp rotational inertia, không chỉ bánh chủ động.
- Công thức chỉ là baseline; friction, slip, suspension và solver có thể làm acceleration đo được lệch khỏi dự đoán.

Để ước lượng thực nghiệm ít nhạy với lực cản, ưu tiên:

```text
m_eff_measured ~= delta(F_drive) / delta(a)
```

với hai mức torque gần nhau tại cùng vùng vận tốc.

## 6. Phase 0 — Audit môi trường và license

### Công việc

- Xác định chính xác ROS distro, Gazebo major version đang được launch và phiên bản `gz_ros2_control` tương thích.
- Xác định tên thư viện runtime của:
  - `ApplyJointForce`;
  - `OdometryPublisher`;
  - `gz_ros2_control` system plugin.
- Kiểm tra license và điều khoản phân phối lại của Prius Fuel model version 2.
- Ghi lại URI, version, license và checksum/model source trong tài liệu model.

### Kết quả mong đợi

- Có compatibility matrix ngắn, không dựa trên suy đoán từ tên package.
- Có quyết định rõ ràng:
  - vendor toàn bộ SDF/mesh; hoặc
  - vendor SDF custom nhưng giữ asset URI bên ngoài, kèm ghi chú phụ thuộc mạng.

### Tiêu chí hoàn thành

- Không sửa Fuel cache.
- Có bằng chứng license cho chiến lược vendor được chọn.
- Các plugin cần dùng tồn tại đúng với Gazebo binary thực tế.

## 7. Phase 1 — Prius custom và physics spike

### 7.1 Tạo model custom

- Tạo model trong package phù hợp, ví dụ:

  ```text
  car_controller/models/prius_hybrid_custom/
  ```

- Copy hoặc tham chiếu asset theo quyết định license ở Phase 0.
- Xóa `ignition-gazebo-diff-drive-system` khỏi biến thể effort để tránh hai hệ thống cùng command rear wheel joints.
- Giữ tên joint cần thiết:
  - `rear_left_wheel_joint`;
  - `rear_right_wheel_joint`.
- Khóa front steering axes ở góc 0 cho spike longitudinal. Không chỉ để chúng ở trạng thái không command.
- Tạo world riêng cho spike; không thay world baseline ngay lập tức.

### 7.2 Thêm nguồn odometry độc lập

Thêm `ignition::gazebo::systems::OdometryPublisher` vì odometry cũ thuộc DiffDrive plugin và sẽ mất khi gỡ plugin đó.

Yêu cầu:

- `gaussian_noise = 0` trong spike.
- Publish frequency cố định, ví dụ 50 Hz.
- Xác định `odom_frame`, `robot_base_frame`, dimensions và topic thực tế.
- Bridge odometry sang ROS để ghi log hoặc dùng Gazebo Transport logger trong spike.

### 7.3 Thêm torque command thô

- Gắn một `ApplyJointForce` system cho mỗi rear wheel joint.
- Xác nhận topic `cmd_force` và message type bằng `ign topic -l/-i`.
- Không giả định torque hai bánh cùng dấu sẽ tạo chuyển động thẳng; xác nhận dấu từ joint state và chuyển động chassis.
- Thêm `JointStatePublisher` nếu cần để quan sát angular position/velocity.

### 7.4 Dữ liệu phải ghi

Mỗi run ghi tối thiểu:

- simulation time;
- commanded torque trái/phải;
- wheel angular velocity trái/phải;
- chassis longitudinal velocity;
- longitudinal acceleration;
- yaw/yaw rate để phát hiện drift;
- physics timestep;
- real-time factor.

### 7.5 Test matrix

| Case | Điều kiện | Mục đích |
|---|---|---|
| `T, m` | vượt breakaway, chưa slip | Baseline vùng tuyến tính |
| `2T, m` | vẫn chưa slip | Kiểm tra torque scaling |
| `T, m_modified` | định nghĩa rõ links/inertias được scale | Kiểm tra mass sensitivity bằng `m_eff` mới, không mặc định chính xác 0.5 nếu chỉ đổi chassis mass |
| `0, m` sau tăng tốc | `v > 0` | Đo coasting loss |
| `-T, m` sau tăng tốc | đo khi `v > 0`, dừng trước khi qua 0 | Braking |
| Torque sweep gần 0 | tăng dần | Tìm breakaway threshold |
| Torque sweep lớn | tăng đến phi tuyến | Tìm traction saturation/slip |

### 7.6 Trình tự debug nếu quan hệ không hợp lý

1. Plugin và topic có command đúng joint không.
2. Joint axis và dấu torque.
3. Front steering có thật sự bị giữ ở 0 không.
4. Joint friction.
5. Contact friction và collision geometry.
6. Wheel slip.
7. Rear axle/suspension.
8. Physics timestep và solver stability.

### Gate A — Physics feasibility

PASS khi:

- Torque đúng dấu tạo chuyển động đúng chiều.
- Xe gần như không yaw trong test thẳng.
- Có vùng torque mà `delta(F)/delta(a)` tương đối ổn định.
- Mass/inertia thay đổi gây tác động đúng xu hướng.
- Coasting, braking, breakaway và saturation được đo, không chỉ quan sát GUI.

Nếu Gate A fail, sửa model/physics spike. Kết quả Gate A không tự quyết định việc dùng hay bỏ `gz_ros2_control`.

## 8. Phase 2 — `gz_ros2_control` feasibility spike

Chỉ bắt đầu sau Gate A.

### Công việc

- Thêm `<ros2_control>` block vào Prius custom với effort command interface cho hai rear wheel joints.
- Khai báo state interfaces cần thiết: position, velocity và effort nếu backend hỗ trợ.
- Thêm `gz_ros2_control::GazeboSimROS2ControlPlugin` đúng phiên bản.
- Tạo YAML riêng cho Prius, không tái sử dụng `car_controllers.yaml` của differential-drive robot.
- Cấu hình:
  - `joint_state_broadcaster`;
  - `effort_controllers/JointGroupEffortController` hoặc controller tương đương được xác nhận có sẵn.
- Spawn/activate controller và xác nhận claimed interfaces bằng `ros2 control list_hardware_interfaces` và `ros2 control list_controllers`.
- Command torque bằng CLI, chưa viết `gazebo_vehicle_interface`.
- Chạy lại baseline torque và coasting test từ Phase 1.

### Gate B — ros2_control feasibility

PASS khi:

- Controller manager khởi động ổn định.
- Hai effort interfaces được export và claimed.
- Effort command đến đúng joints.
- Kết quả chassis/wheel không lệch bất thường so với `ApplyJointForce` baseline.
- Controller deactivate đưa effort về trạng thái an toàn đã xác minh.

Nếu Gate B fail:

- Phân biệt lỗi cấu hình với incompatibility thật của phiên bản/plugin.
- Chỉ chọn custom generic Gazebo effort plugin khi incompatibility đã được chứng minh.
- Custom plugin vẫn không được phụ thuộc `car_msgs`.

## 9. Phase 3 — `gazebo_vehicle_interface`

### API

```text
Subscribe:
  /control/trajectory_follower/longitudinal_cmd
  car_msgs/msg/Longitudinal

Publish:
  effort controller command topic
  generic controller message type
```

### Xử lý command

- Trước lệnh hợp lệ đầu tiên: zero effort.
- Reject NaN/Inf ở velocity, acceleration, jerk và timestamp khi cần.
- Kiểm tra command freshness.
- Clamp acceleration theo safety range của vehicle interface.
- Chuyển `a_target` thành torque.
- Clamp torque theo traction/actuator safety range đã xác định từ spike.
- Không gọi lại stateful `LongitudinalLimits`: controller đã acceleration/jerk-limit; gọi lần hai sẽ làm thay đổi command ngoài ý muốn.
- Nếu cần actuator lag, torque slew-rate hoặc delay, triển khai thành actuator model có parameter riêng ở phase sau.

### Safe-stop

Safe-stop phải có hai tầng:

1. `gazebo_vehicle_interface` watchdog publish zero khi command stale.
2. Tầng controller/hardware/Gazebo generic phải zero hoặc deactivate khi interface chết và không còn publish.

Không coi destructor publish-zero là bảo đảm an toàn; đó chỉ là best effort.

### Unit tests

- Non-finite command bị từ chối.
- Acceleration và torque clamp đúng.
- Torque conversion đúng dấu và đúng phân bố.
- Watchdog về zero khi command stale.
- Invalid command không làm mất safe state.

## 10. Phase 4 — `odometry_adapter`

### API

```text
Subscribe:
  Gazebo odometry bridged sang ROS

Publish:
  /simulation/ground_truth/odometry
  /localization/kinematic_state
```

### Xử lý

- Xác định frame semantics bằng metadata và test thực tế; không suy đoán từ tên topic.
- Nếu twist ở world frame:

  ```text
  v_longitudinal = vx*cos(yaw) + vy*sin(yaw)
  ```

- Nếu twist ở body frame, dùng `linear.x`.
- Ground truth không có measurement noise.
- Kinematic state nhận Gaussian noise có seeded RNG khi bật.
- Integration baseline chạy với `velocity_noise_sigma = 0`; chỉ bật noise sau khi pipeline cơ bản pass.
- Timestamp và frame IDs phải nhất quán.

### Unit tests

- Chiếu world-frame velocity sang body longitudinal đúng.
- Body-frame passthrough đúng.
- Sigma 0 cho output bằng ground truth.
- Cùng seed tạo cùng noise sequence.
- Khác seed tạo sequence khác.

## 11. Phase 5 — Config, launch và backend contract

### Launch

- Thêm `gazebo_effort` vào `vehicle_model` choices.
- Với `longitudinal_sim`:
  - giữ hành vi hiện tại;
  - chạy `longitudinal_vehicle_simulator`.
- Với `gazebo_effort`:
  - dùng custom Prius world/model;
  - không chạy plant 1D;
  - chạy `gz_ros2_control`, effort controller, `gazebo_vehicle_interface`, odometry bridge và `odometry_adapter`;
  - chỉ start planner/controller sau khi odometry và effort controller thật sự ready/active.
- Xóa `ideal_diff_drive` khỏi choices hoặc đánh dấu rõ unsupported; không để nó trông như backend hoạt động.

### Readiness

Không chỉ chờ một Gazebo topic. Backend được xem là ready khi:

- model đã spawn;
- OdometryPublisher đã publish;
- controller manager tồn tại;
- effort controller active;
- vehicle interface và adapter đang chạy.

### Config

Tạo config riêng cho:

- Prius `ros2_control` controllers;
- `gazebo_vehicle_interface`;
- `odometry_adapter`;
- backend-specific safety/physics parameters.

Scenario YAML phải route parameter đến đúng node backend. Không giả định YAML của `longitudinal_vehicle_simulator_node` tự áp dụng cho Gazebo backend.

Ban đầu chỉ hỗ trợ rõ ràng `baseline`. Các scenario sau chỉ bật khi implementation tương ứng tồn tại:

- sensor noise -> odometry adapter;
- process disturbance -> external force/torque model;
- grade -> inclined world hoặc grade-force model;
- combined -> tổng hợp các lớp đã xác minh.

## 12. Phase 6 — Runner và logger

### Runner

Mở rộng `run_controller_simulations.py`:

```text
--vehicle-model longitudinal_sim|gazebo_effort
```

- Truyền backend vào launch.
- Không chạy scenario chưa được backend hỗ trợ.
- Startup readiness và timeout phải phù hợp backend.
- Tên archive chứa controller, backend, scenario và seed.

Ví dụ:

```text
fuzzy_pid_gazebo_effort_baseline_1001.csv
pid_longitudinal_sim_sensor_noise_1001.csv
```

### Logger

- Giữ schema chung khi các đại lượng có cùng ý nghĩa.
- Ghi thêm `vehicle_backend`.
- Không gọi Gazebo velocity command là `cmd_vel` trong effort mode.
- Phân biệt:
  - target acceleration;
  - commanded wheel torque;
  - measured/applied acceleration;
  - ground-truth velocity;
  - measured velocity.
- Không hard-code saturation threshold; lấy từ config/backend debug.

## 13. Phase 7 — Integration và failure tests

Viết test trước khi thêm drag/rolling/delay/noise nâng cao.

### Happy-path contract test

1. Planner publish reference.
2. Controller nhận `/localization/kinematic_state`.
3. Controller publish `Longitudinal`.
4. Vehicle interface nhận command.
5. Effort controller active và nhận torque.
6. Wheel velocity và chassis velocity tăng.
7. Odometry quay về adapter.
8. Adapter publish ground truth và kinematic state.
9. Feedback quay lại controller.

### Failure tests

- Dừng planner/controller command publisher -> torque về zero sau timeout.
- Kill `gazebo_vehicle_interface` -> tầng safety thấp hơn đưa torque về zero/deactivate.
- Gửi NaN/Inf -> không truyền torque không hợp lệ.
- Effort controller không active -> launch báo lỗi/không start planner.
- Odometry mất -> controller không tiếp tục dùng dữ liệu stale mà không có diagnostic.

### Acceptance baseline

- Xe tăng tốc từ đứng yên với target dương.
- Xe giảm tốc với target acceleration âm khi đang chạy tiến.
- Không có yaw/drift đáng kể trong bài test thẳng.
- Không có publisher cạnh tranh trên rear wheel effort interfaces.
- `longitudinal_sim` tests cũ vẫn pass.

## 14. Phase 8 — Realism layers, chỉ sau integration baseline

Thêm từng lớp độc lập, mỗi lớp có parameter và test riêng:

1. Actuator delay/dead time.
2. First-order actuator lag.
3. Torque slew-rate/saturation.
4. Rolling-resistance model hoặc calibration.
5. Aerodynamic drag.
6. Process disturbance.
7. Measurement noise.
8. Grade model/terrain.

Không bật đồng thời nhiều lớp trước khi từng lớp được kiểm chứng riêng.

## 15. Deliverables dự kiến

- Tài liệu license/model provenance.
- Prius custom SDF/world có version control.
- Physics spike scripts và raw/summary results.
- `gz_ros2_control` effort configuration.
- `gazebo_vehicle_interface` cùng unit tests.
- `odometry_adapter` cùng unit tests.
- Launch backend `gazebo_effort`.
- Runner/logger hỗ trợ backend.
- Integration/failure tests.
- Báo cáo so sánh baseline giữa `longitudinal_sim` và `gazebo_effort`.

## 16. Checklist thực thi

### Phase 0

- [x] Xác nhận Gazebo binary/version thực tế.
- [x] Xác nhận compatibility với `gz_ros2_control`.
- [x] Kiểm tra metadata/license Prius Fuel model (upstream không cung cấp license rõ ràng).
- [x] Chốt vendor SDF custom, giữ asset URI ngoài.

### Phase 1 / Gate A

- [x] Prius custom local được tạo, không có DiffDrive.
- [x] Front steering được giữ tại 0.
- [x] OdometryPublisher hoạt động độc lập.
- [x] Effort command hoạt động trên hai rear joints.
- [x] Joint states và dấu torque được xác nhận.
- [ ] Test matrix hoàn thành và có log.
- [ ] `m_eff` lý thuyết và thực nghiệm được so sánh.
- [ ] Gate A PASS.

### Phase 2 / Gate B

- [x] Runtime được chuẩn hóa trên Fortress 6 và plugin link `libignition-gazebo6.so.6`.
- [ ] Effort interfaces export/claim đúng.
- [ ] Effort controller active.
- [ ] CLI effort test khớp hợp lý với Phase 1.
- [ ] Deactivation/safe state được xác minh.
- [x] Generic Fortress 6 plugin có watchdog được build và smoke test.

### Phase 3–5

- [x] Vehicle interface có validation, clamp và watchdog.
- [x] Lower-layer timeout xử lý node death.
- [x] Odometry adapter xử lý frame/trục dọc Prius đúng.
- [x] Noise sigma 0 baseline pass.
- [x] `gazebo_effort` chờ odometry trước khi start pipeline.
- [x] `ideal_diff_drive` được bỏ khỏi choices.

### Phase 6–8

- [x] Runner và logger phân biệt backend.
- [ ] Happy-path integration test pass.
- [ ] Failure/safe-stop tests pass.
- [x] Regression unit tests của `longitudinal_sim` pass.
- [ ] Realism layers được thêm và test lần lượt.

## 17. Definition of Done

Backend `gazebo_effort` chỉ được coi là hoàn thành khi:

1. Cùng PID/Fuzzy PID binary có thể chạy với `longitudinal_sim` hoặc `gazebo_effort` chỉ bằng launch/config.
2. Trong `gazebo_effort`, Gazebo odometry thật sự nằm trong feedback loop.
3. Target acceleration được chuyển thành generic wheel torque command; không còn `/cmd_vel` trong effort path.
4. Rear wheels chỉ có một nguồn effort command.
5. Timeout, invalid command và interface death đều dẫn đến trạng thái effort an toàn.
6. Ground truth và measured state được tách rõ.
7. Physics spike, integration test và regression test đều pass.
8. Model provenance/license và các giới hạn mô phỏng được tài liệu hóa.
