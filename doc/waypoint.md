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
  prior_2d_map:=/home/selab/data/selab/nav2_map_csf/map.yaml
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
  prior_2d_map:=/home/selab/data/nda_outdoor/nav2_map_csf/map.yaml
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
  prior_2d_map:=/home/selab/data/bikan_indoor/maps/map.yaml
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