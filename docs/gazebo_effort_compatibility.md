# Gazebo effort backend compatibility audit

Audit date: 2026-07-21.

The `gz6_PSO` branch targets Gazebo Fortress / Gazebo Sim 6 exclusively.

| Component | Required version / runtime | Decision |
|---|---|---|
| ROS 2 | Humble | Supported workspace baseline |
| Simulator | `ign gazebo --force-version 6` (Fortress 6.18.0 tested) | Runtime used by launch files |
| Gazebo C++ API | `ignition-gazebo6` | Custom effort plugin build dependency |
| Gazebo messages / transport | `ignition-msgs8` / `ignition-transport11` | Fortress ABI used by the plugin and bridge |
| ROS bridge | `ros_ign_bridge` compatibility package | Selects the Humble bridge built for Fortress |
| Built-in systems | `ignition-gazebo-*-system` | Physics, user commands, scene, joint state and odometry plugins |

The rear-wheel effort plugin has an independent command watchdog and writes
`JointForceCmd` components directly. It is linked to `libignition-gazebo6.so.6`; no
Gazebo Sim 8 library is loaded into the Fortress process.

## Verification

The clean build and headless smoke test verified:

- Fortress 6 loaded `prius_effort_spike.sdf`;
- the create service spawned `prius_gz.urdf` successfully;
- the custom effort plugin accepted `ignition.msgs.Double` commands and moved the car;
- `OdometryPublisher` published `/model/car/odometry`;
- the Fortress ROS bridge created clock, odometry and both effort bridges;
- `odometry_adapter` and `gazebo_vehicle_interface` started after odometry became ready.

## Host package note

The Fortress and Harmonic variants of the Humble bridge conflict at the Debian package
level. A host that currently has `ros-humble-ros-gzharmonic-bridge` must replace that
variant with `ros-humble-ros-ign-bridge` before running this branch normally. Do not
load a Harmonic bridge (`gz-msgs10` / `gz-transport13`) into this Fortress stack.
