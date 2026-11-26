# Web Monitor

Tailscale
```bash
ros2 launch whill_web_monitor web_monitor.launch.py \
  map_yaml:=/home/selab/data/selab/nav2_map_csf/map.yaml \
  route_yaml:=/home/selab/data/selab/nav2_map_csf/waypoints.yaml \
  bind_address:=127.0.0.1
```

Tailscale Statusを確認
```bash
tailscale status
```

Tailscale Serveの有効化（初回のみ）
```bash
tailscale serve --bg 8765
```
- `serve` ローカルWebサービスをTelnet内に公開
- `--bg` バックグラウンドで継続実行
- `8765` 転送先のWeb Monitorのポート番号

アクセスするURLを確認
```bash
tailscale serve status
```

---

In same network
```bash
ros2 launch whill_web_monitor web_monitor.launch.py \
  map_yaml:='/home/selab/data/bikan_indoor/maps/map.yaml' \
  route_yaml:='/home/selab/data/bikan_indoor/maps/waypoints.yaml' \
  bind_address:=192.168.40.95
```