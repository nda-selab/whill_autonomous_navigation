# Lattice

例：10cm用Latticeを生成
```json
{
  "motion_model": "diff",
  "turning_radius": 0.5,
  "grid_resolution": 0.10,
  "stopping_threshold": 5,
  "num_of_headings": 16
}
```

```bash
python3 generate_motion_primitives.py
python3 generate_motion_primitives.py \
  --config ~/ros2_ws/src/whill_nav2_bringup/config/lattice/diff_10cm_config.json \
  --output ~/ros2_ws/src/whill_nav2_bringup/config/lattice/diff_10cm.json  \
  --visualizations ./visualizations_10cm
```