# Waypoint設定手順

## Selab

Waypoint editor起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:='/home/selab/data/selab/nav2_map_csf/map.yaml' \
  waypoints_file:='/home/selab/data/selab/nav2_map_csf/waypoints.yaml'
```

RVizおよびMap server起動
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=/home/selab/data/selab/nav2_map_csf/map.yaml \
  prior_3d_map:=/home/selab/data/selab/pointcloud_2016-02-12-01-30-04_replay_loop_closed.ply
```

Waypoints保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

経路事前検証
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route '/home/selab/data/selab/nav2_map_csf/waypoints.yaml' \
  --map-yaml '/home/selab/data/selab/nav2_map_csf/map.yaml' \
  --nav2-params '/home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml'
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信

---

## NDA outdoor

Waypoint editor起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:=/home/selab/data/nda_outdoor/nav2_map_csf/map.yaml \
  waypoints_file:=/home/selab/data/nda_outdoor/nav2_map_csf/waypoints.yaml
```

RVizおよびMap server起動
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=/home/selab/data/nda_outdoor/nav2_map_csf/map.yaml \
  prior_3d_map:=/home/selab/data/nda_outdoor/pointcloud_ds_2016-02-12-01-29-20_replay_loop_closed.ply
```

Waypoints保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

経路事前検証
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route /home/selab/data/nda_outdoor/nav2_map_csf/waypoints.yaml \
  --map-yaml /home/selab/data/nda_outdoor/nav2_map_csf/map.yaml \
  --nav2-params '/home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml'
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信

---

## Bikan indoor

Waypoint editor起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:=/home/selab/data/bikan_indoor/maps/map.yaml \
  waypoints_file:=/home/selab/data/bikan_indoor/maps/waypoints.yaml
```

RVizおよびMap server起動
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=/home/selab/data/bikan_indoor/maps/map.yaml \
  prior_3d_map:=/home/selab/data/bikan_indoor/PCD/merged_map.pcd
```

Waypoints保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```

経路事前検証
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route /home/selab/data/bikan_indoor/maps/waypoints.yaml \
  --map-yaml /home/selab/data/bikan_indoor/maps/map.yaml \
  --nav2-params '/home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml'
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信

---

## Trajectory to Waypoints

```bash
ros2 run whill_nav2_bringup trajectory_to_waypoints.py \
  --trajectory /home/selab/data/tsukuba/trajectory_2016-02-12-01-54-42_replay_loop_closed.ply \
  --map-yaml /home/selab/data/tsukuba/map/map.yaml \
  --output /home/selab/data/tsukuba/map/waypoints.yaml \
  --waypoint-spacing 5.0 \
  --corner-angle-deg 20.0
```

## ウェイポイントの編集

Map Serverの起動
```bash
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:='/home/selab/data/tsukuba/map/map.yaml' 

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

Waypoint_editorの起動
```bash
ros2 launch whill_nav2_bringup waypoint_editor.launch.py \
  map_yaml:='/home/selab/data/tsukuba/map/map.yaml' \
  waypoints_file:='/home/selab/data/tsukuba/map/waypoints.yaml' 
```

RViz
```bash
rviz2
```

保存
```bash
ros2 service call /waypoint_editor/save std_srvs/srv/Trigger '{}'
```