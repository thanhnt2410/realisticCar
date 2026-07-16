# Gazebo effort backend compatibility audit

Audit date: 2026-07-15.

| Component | Installed version / runtime | Decision |
|---|---|---|
| ROS 2 | Humble | Supported workspace baseline |
| `gz sim` | Sim 8.14.0 | Runtime used by existing launch |
| `ign gazebo` | Fortress 6.18.0 | Installed, but not used by existing launch |
| `gz_ros2_control` | ROS package 0.7.20 | Its system library links to `libignition-gazebo6.so.6`; ABI-incompatible with Sim 8 |
| ApplyJointForce | `gz-sim-apply-joint-force-system` | Loaded successfully in Sim 8 |
| OdometryPublisher | `gz-sim-odometry-publisher-system` | Loaded successfully in Sim 8 at 50 Hz |

Gate A smoke result: equal `+250 Nm` commands moved the unrotated upstream model along
its longitudinal `-Y` axis at about `3.43 m/s`, with yaw magnitude below `1e-6 rad`.
The effort world rotates the model by +90 degrees so forward motion follows ROS +X.

Gate B cannot use the installed `gz_ros2_control` library in the Sim 8 process. This is
an ABI mismatch rather than a controller YAML issue. The Sim 8 backend therefore uses
the generic Gazebo `ApplyJointForce` systems and ROS-Gazebo scalar bridges. This decision
must be revisited if the runtime is standardized on Fortress or a Sim 8-compatible
`gz_ros2_control` package is installed.
