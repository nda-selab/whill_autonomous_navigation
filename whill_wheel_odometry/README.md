# whill_wheel_odometry

WHILL Model CR2 の実測モータ速度から差動二輪モデルにより車体速度と平面オドメトリを推定する ROS 2 Humble 向けパッケージ。`robot_localization` の EKF への入力を想定し、TF は配信しない。

## 入出力

| 種別 | トピック | 型 |
|---|---|---|
| 入力 | `/whill/states/model_cr2` | `whill_msgs/msg/ModelCr2State` |
| 出力 | `/wheel_odom` | `nav_msgs/msg/Odometry` |
| 出力 | `/wheel_odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

`/wheel_odom` は正常な入力サンプルが連続して得られた場合のみ配信する。車両停止時の実測ゼロ速度も正常な観測として配信する。入力が停止した場合、過去の速度で補間・予測配信しない。メッセージに `header` がないため、コールバック時の ROS 時刻を使用する。診断の受信鮮度は monotonic clock で監視する。シリアル通信やDDSの遅延は別途計測する必要がある。

## モデル

公式ドライバの `whill.cpp` ではモータ速度を km/h、モータ角度を rad に変換して配信する。本パッケージでは速度のみを使用し、左右それぞれに符号とスケールを適用する。

- `v_l = left_sign * left_scale * left_motor_speed / 3.6`
- `v_r = right_sign * right_scale * right_motor_speed / 3.6`
- `v = (v_l + v_r) / 2`
- `w = (v_r - v_l) / wheel_tread`
- 駆動軸中心から base_link へのオフセット `(a_x, a_y)` がある場合、`vx = v - w*a_y`, `vy = w*a_x`, `wz = w`。base_link の軸は駆動軸中心の車体座標軸と平行と仮定する。

姿勢は前後サンプルの平均速度を区間内一定とした円弧積分で更新する。初期 base_link の平面姿勢を原点とし、駆動輪速度の独立ノイズと横滑りの仮定に基づき共分散を計算する。これは校正済みの確率モデルではなく、ノイズ値は初期値である。傾斜面・スリップ等の系統誤差は完全には反映しない。観測しない z/roll/pitch は大きな共分散を設定する。入力途絶後は欠落区間の移動を積分しない。積分済み pose は以後、再起動まで大きな共分散と `pose_valid=false` を付ける。EKFでは twist のみ融合すること。

## ビルド

既存ワークスペースの `src/whill_autonomous_navigation` の下に本ディレクトリを配置する（同名パッケージの重複に注意）。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to whill_wheel_odometry
source install/setup.bash
```

`--symlink-install` は開発時にソースの変更を反映しやすくする。`--packages-up-to` は指定パッケージとワークスペース内の依存パッケージを対象とする。既存ワークスペースに `whill_msgs` が必要。

## 実機校正と起動

**初期値のまま自律走行を実施しないこと。** `calibration_confirmed: false` の初期設定ではオドメトリを配信しない。診断は配信する。安全な環境で手動低速走行し、前進・後退・左右回旋の実測値を確認する。

```bash
ros2 topic echo /whill/states/model_cr2
ros2 topic hz /whill/states/model_cr2
ros2 param get /whill publish_interval_ms
```

1. 直進前進時に両輪の変換後速度が正、後退時に負になるよう `left_sign`, `right_sign` を設定する。
2. 実測直進距離・時間に基づき左右スケールを校正する。速度が実際の車輪周速度に対応するか確認する。
3. 実測旋回角と左右の速度差から実効トレッド幅を調整する。公式ドライバの速度指令変換では 0.496 m、一方 URDF の後輪中心間距離は 0.520 m であり、実機の有効値は未確定。
4. 車両の駆動輪軸中心と既存 `base_link` の位置関係を測り、オフセットを設定する。ゼロの場合のみ `vy=0` となる（または純直進時）。
5. 速度・ジオメトリの検証後に `calibration_confirmed: true` として再起動し、手動走行による出力を確認する。ノイズ標準偏差と入力タイムアウトも実測から調整する。

設定ファイルを指定して起動：

```bash
ros2 launch whill_wheel_odometry wheel_odometry.launch.py \
  params_file:=$HOME/ros2_ws/src/whill_autonomous_navigation/whill_wheel_odometry/config/wheel_odometry.yaml
```

`params_file` は外部設定ファイルのパス。省略時はインストールされた同梱 YAML を使用する。パラメータ変更はノード再起動時に反映する。

```bash
ros2 topic echo /wheel_odom --once
ros2 topic hz /wheel_odom
ros2 topic echo /wheel_odometry/diagnostics
```

`--once` は一件だけ表示する。状態取得周期が 400 ms（2.5 Hz）の場合、出力も約 2.5 Hz となる。20 Hz への増速はドライバ・本体・シリアル通信の対応範囲を安全に検証してから行う。50 Hz EKF の予測出力は 50 Hz の車輪実測値を意味しない。

## EKFとの接続

`whill_nav2_bringup/launch/components/localization/ekf.launch.py` で本パッケージの launch を include し、`odom_to_base_link.py` は起動しない。`ekf_params_file` の既定値を設定し、既存 EKF の出力リマップ `/odometry/filtered` → `/odom_nav` を維持する。

```python
wheel_odometry = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('whill_wheel_odometry'), 'launch', 'wheel_odometry.launch.py',
    ])),
    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
)
```

必要な import は `IncludeLaunchDescription`, `PythonLaunchDescriptionSource`, `PathJoinSubstitution`, `LaunchConfiguration`, `FindPackageShare`。既存の `LaunchDescription` に `wheel_odometry` を追加する。bringup の `package.xml` に `robot_localization` と `whill_wheel_odometry` の実行時依存を追加する。

```yaml
# ekf_params.yaml (既存ファイルの一部)
odom0: /wheel_odom
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,
               false, false, true,
               false, false, false]
imu0: /ouster/imu
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
```

横方向速度の融合は平面差動二輪モデルの拘束または base_link のオフセットに伴う速度として扱う。坂道等の三次元運動は別途検証する。LiDAR Localization の EKF モードの入力は `/cloud_registered` ではなくセンサ基準の点群に変更する。

## 異常監視

診断名は `whill_wheel_odometry/input`。入力途絶、エラー状態、電源OFF、速度範囲外、時刻異常、未校正を検出する。`twist_valid`, `pose_valid`, `input_age_sec`, `valid_age_sec` などを提供する。途絶後に速度情報が復帰しても pose の不確実性は残る。再開は上位側で判断する。

**本パッケージは停止指令やWatchdog制御を実行しない。** EKF は入力が失われても予測を配信する場合があるため、既存の `route_health_monitor.py` と `cmd_vel_watchdog` に車輪とIMUの入力鮮度、診断異常の監視を追加し、走行許可と連動させること。初期の入力タイムアウト 0.75 s は 400 ms 周期での試験用であり、安全な停止期限を保証しない。実運用時は通信周期と停止距離に基づき厳格化する。

## テスト

```bash
cd ~/ros2_ws
colcon test --packages-select whill_wheel_odometry
colcon test-result --verbose
```

`--packages-select` はテスト対象を指定し、`--verbose` は結果の詳細を表示する。ROSなしで数式部分を試験する場合：

```bash
cd ~/ros2_ws/src/whill_autonomous_navigation/whill_wheel_odometry
python3 -m pytest -q tests
```

`-q` は簡潔なテスト出力。作成時は7件の数式単体テストに合格。ROS 2 Humble が存在しない作成環境のため、colcon ビルド・ROSノード通信・実機動作は未試験。

## 参考資料

- [WHILL driver: モータ値の単位と変換](https://github.com/whill-labs/ros2_whill/blob/humble/whill_driver/src/model_cr2/whill.cpp)
- [WHILL driver: cmd_vel変換・トレッド幅](https://github.com/whill-labs/ros2_whill/blob/humble/whill_driver/src/whill_node.cpp)
- [WHILL Model CR2 URDF](https://github.com/whill-labs/ros2_whill/blob/humble/whill_description/urdf/whill_model_cr2.urdf)
- [ModelCr2State.msg](https://github.com/whill-labs/ros2_whill_interfaces/blob/humble/whill_msgs/msg/ModelCr2State.msg)
- [REP-105](https://www.ros.org/reps/rep-0105.html)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)

`package.xml` の maintainer は仮の値であり、リポジトリ登録前に実際の管理者情報に変更すること。
