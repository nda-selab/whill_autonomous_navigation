# Autonomous driving

## Selab

Terminal 1: WHILL Nav2 Bringup
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=/home/selab/data/selab/nav2_map_csf/map.yaml \
  prior_3d_map:=/home/selab/data/selab/pointcloud_2016-02-12-01-30-04_replay_loop_closed.pcd
```

Terminal 2: Waypoint Route Runner
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route /home/selab/data/selab/nav2_map_csf/waypoints.yaml \
  --map-yaml /home/selab/data/selab/nav2_map_csf/map.yaml \
  --nav2-params /home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信

## NDA outdoor

Terminal 1: WHILL Nav2 Bringup
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_2d_map:=/home/selab/data/nda_outdoor/nav2_map_csf/map.yaml \
  prior_3d_map:=/home/selab/data/nda_outdoor/pointcloud_ds_2016-02-12-01-29-20_replay_loop_closed.pcd
```

Terminal 2: Waypoint Route Runner
```bash
ros2 run whill_nav2_bringup waypoint_route_runner.py \
  --route /home/selab/data/nda_outdoor/nav2_map_csf/waypoints.yaml \
  --map-yaml /home/selab/data/nda_outdoor/nav2_map_csf/map.yaml \
  --nav2-params /home/selab/ros2_ws/src/whill_nav2_bringup/config/nav2_params.yaml
```

- `--plan-preview` Plannerによる経路確認の実行（Nav2起動時に使用）
- `--execute`　走行指令の送信