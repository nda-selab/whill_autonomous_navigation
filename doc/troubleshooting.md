# Trouble Shooting

`--nav2-params`には，
Nav2起動時と同じ設定YAMLを指定する．省略時は本リポジトリの
`config/nav2_params.yaml`を使用する．事前チェックではlocal_costmapの
`footprint`と`footprint_padding`（省略時はNav2 Humbleと同じ0.01 m）を読み込み，
各目標のyawで回転させた車体の内側と外周を地図上で確認する．

`--clearance`は**車体の外周から追加で確保する距離**である．従来の
「waypoint中心からの自由半径」とは意味が異なり，`--clearance 0.45`では
車体外周からさらに0.45 mの余裕を要求する．追加余裕を付けずNav2の
footprintとpaddingだけで確認する場合は`--clearance 0.0`を指定する．
障害物セルは対角線の半分の半径を持つ円で覆って判定するため，セルの端や
角の接触も検出する一方，地図解像度に応じて判定が少し保守的になる．
このチェックは各目標姿勢と軌跡開始姿勢に対するものであり，区間全体の
走行可否はPlannerによる経路検査と走行中のNav2の衝突判定で確認する．

## Initial Setup

### Check user groups

WHILLをUSBシリアル経由で制御する場合，
ユーザが`dialout`グループに所属している必要がある．

```bash
groups
```

`dialout`が含まれていない場合は以下を実行，
ログアウト・再ログインする．

```bash
sudo usermod -aG dialout $USER
```

## Manual Driving Test

- Terminal 1: WHILL

```bash
ros2 launch whill_bringup whill_launch.py
```

- Terminal 2: Joystick

```bash
ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0 \
  -r /joy:=/whill/controller/joy
```

ジョイスティックでWHILLを操作できることを確認する．

## Sensor and FAST-LIO2 Test

### Ouster Connection

```bash
ping -c 3 169.254.185.67
ip addr
```

- Terminal 1: Ouster

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=original \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```

- Terminal 2: FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py
```

## Topic Check

```bash
ros2 topic list | grep ouster

ros2 topic info /cloud_registered -v
ros2 topic info /scan -v
ros2 topic info /map -v

ros2 topic echo /Odometry --once
ros2 topic echo /amcl_pose --once

ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /whill/controller/cmd_vel
```

## TF Check

```bash
ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

## Costmap Check

```bash
ros2 topic echo /global_costmap/costmap --once --field header
ros2 topic echo /local_costmap/costmap --once --field header
```

## 地図更新時のTF遅延と経路走行の停止理由

`lidar_localization_ros2` のローカル地図切り出し・NDTターゲット再構築・
fitness計算は、姿勢配信タイマーと共有する状態ロックを解放して実行する。
更新中に初期位置を再設定した場合は、その更新結果を採用せず次のスキャンで再構築する。
`Local map target update: ... s, ... points` のログで地図更新の所要時間を確認できる。
この修正はロックによるTF配信待ちを解消するもので、地図更新自体の計算量は減らさない。

経路走行のヘルスチェック失敗時には、コンソール、`*.results.yaml` の
`results` 内の `id: mission` の `reason`、`/navigation/waypoint_event` の
`reason` に具体的な原因が入る。

- `localization`: `/alignment_status` の未受信・古さ、収束状態、地図・初期位置の受信状態。
- `URG left/right`: センサーごとの診断欠落、レベル、メッセージ。
- `odometry`: `/odom_nav` の受信遅延、元タイムスタンプの古さ・逆行、不正なフレームや速度。
- `watchdog`: 非アクティブ状態、watchdogが報告した異常。

監視タイムアウトと停止条件は従来どおり。複数の異常がある場合は併記する。

## MPPIの負荷比較

`whill_nav2_bringup/config/nav2_params_mppi_light.yaml` は比較用の完全な設定ファイル。
通常設定の `time_steps: 160` を `60` に変更し、`model_dt: 0.05`、
`batch_size: 500`、制御周期20 Hzは維持する。予測時間は8秒から3秒、
1反復のサンプル×ステップ数は80,000から30,000になる。
実際の処理時間が同じ割合で減るとは限らず、短い予測時間が経路追従に与える影響は実機で確認する。
既定設定は変更していない。

通常の起動コマンドに、次の引数を追加して比較用設定を選ぶ。

```bash
nav2_params_file:=/home/selab/ros2_ws/src/whill_autonomous_navigation/whill_nav2_bringup/config/nav2_params_mppi_light.yaml
```

`waypoint_route_runner.py` の `--nav2-params` にも同じファイルを指定する。
比較用ファイルは通常設定のコピーなので、他の設定を調整した際は両方に反映する。
同じ経路で、制御周期超過、MPPI失敗、TF遅延、結果ファイルの停止理由を比較する。
