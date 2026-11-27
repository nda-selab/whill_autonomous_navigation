# WHILL Autonomous Navigation

WHILLをROS 2 Humbleで制御し，Ouster LiDAR，FAST-LIO2，Nav2を組み合わせて自律走行を行うためのパッケージである．

## Environment

| Item | Details |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |

## System Overview

### Offline Map Preparation

```text
Prior 3D Map
 └── merged_map.pcd
        ↓
PCD to OccupancyGrid Conversion
 ├── map.pgm
 └── map.yaml
```

### Map-based Localization

```text
Prior 3D Map（PCD / PLY）
        ↓
lidar_localization_ros2
 ├── Input: /cloud_registered
 ├── Input: /initialpose
 ├── Input: odom -> base_link (TF)
 ├── Output: /alignment_status
 └── TF: map -> odom
```

### Real-time Localization

```text
Ouster LiDAR
 ├── /ouster/points
 └── /ouster/imu
        ↓
FAST-LIO2
 ├── /cloud_registered
 └── /Odometry
         ↓
odom_to_base_link
 ├── /odom_nav
 └── odom -> base_link
```

### Obstacle Detection

```text
Ouster LiDAR
 └── /ouster/points
        ↓
Local Costmap
```

```text
Hokuyo URG
        ↓
urg_node2
 ├── /urg_front/scan
 ├── /urg_left/scan
 └── /urg_right/scan
        ↓
Local Costmap
```

### Static TF

```text
base_link
 ├── os_sensor
 │     ├── os_lidar
 │     └── os_imu
 └── laser
```

### Navigation and Control

```text
Nav2
 ├── Controller Server (Normal Navigation)
 └── Behavior Server (Recovery)
        ↓
/cmd_vel_nav
        ↓
velocity_smoother
        ↓
/cmd_vel
        ↓
WHILL CmdVel Watchdog
        ↓
/whill/controller/cmd_vel
        ↓
WHILL Driver
```

### Velocity Command Safety

通常走行（Controller Server）と復帰動作（Behavior Server）の速度指令は、いずれも`/cmd_vel_nav`を経由してVelocity Smootherに入力する。

WHILL CmdVel Watchdogは、平滑化後の`/cmd_vel`を監視し、以下の制限を適用する。

| Parameter | Value |
|---|---|
| Maximum forward velocity | 0.25 m/s |
| Maximum reverse velocity | 0.15 m/s |
| Maximum angular velocity | 0.5 rad/s |
| Command timeout | 0.4 s |
| Output frequency | 20 Hz |

指令受信がタイムアウトした場合、Watchdogはゼロ速度指令を出力する。

ただし、Watchdog自身の停止、ROS 2通信断、WHILLドライバの異常時には停止を保証できない。WHILL本体またはドライバ側の独立した停止機構が必要である。

### Nav2 Frame Parameters

Nav2の詳細な設定パラメータはconfig/nav2_params.yamlを参照する．

## Quick Start

[WHILL Nav2 Bringup](whill_nav2_bringup)を参照

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  whill_nav2_bringup \
  cmd_vel_watchdog \
  lidar_obstacle_filter \
  nav2_waypoint_tools \
  robot_system_monitor
```

## Documentation

- [Installation](doc/installation.md)
- [TF Configuration](doc/tf.md)
- [Troubleshooting](doc/troubleshooting.md)
