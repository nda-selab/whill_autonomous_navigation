# WHILL Nav2 Bringup

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
Local Costmap, Global Costmap
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

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom_nav

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      rolling_window: false
      plugins:
        - static_layer
        - obstacle_layer
        - inflation_layer

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

behavior_server:
  ros__parameters:
    global_frame: odom
    robot_base_frame: base_link
```

## Quik Start

Terminal 1: WHILL Nav2 Bringup
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=< 2D Map YAML File > \
  prior_3d_map:=< 3D Map PCD or PLY File >
```

起動後，RVizの「2D Pose Estimate」からロボットの初期位置・姿勢を設定する．
`map → odom`のTFが配信され，Nav2の自己位置が正常に取得できることを確認してから自律走行を開始する．

Terminal 2: Waypoint Route Runner
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route < Waypoints YAML File > \
  --map-yaml < 2D Map YAML File > \
  --nav2-params '/home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml'
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信

## Documentation

- [Installation](doc/installation.md)
- [TF Configuration](doc/tf.md)
- [Troubleshooting](doc/troubleshooting.md)
