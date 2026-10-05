# 最終課題のシミュレーション環境

既存の TurtleBot3 Burger の走行・LiDAR を使います。最終課題の起動時だけ、
魚眼カメラを通常のカメラ（640×480、15 fps、水平画角80度）に変更します。
既存の `turtlebot3_world.launch.py` などの起動方法には影響しません。

## 最初の1回：ビルド

WSLのUbuntuで、Apptainerに入ります。

```bash
cd ~/ros2_lecture_ws
bash 0_shell.sh
```

Apptainer内で実行します。

```bash
colcon build --symlink-install --packages-select turtlebot3_gazebo
source install/setup.bash
```

以下の起動コマンドは、すべてApptainer内で実行してください。
別ターミナルでも同様にApptainerに入り、`source install/setup.bash` を実行します。
複数の課題や既存のGazeboを同時に起動しないでください。

## 課題①：自律探索・案内ロボット

ターミナル1：Gazeboを起動します。

```bash
ros2 launch turtlebot3_gazebo final_tasks.launch.py task:=exploration
```

SLAM・Navigationは、演習用パッケージ `lecture04_py` の手順で起動します。
最終課題向けの補足は、同パッケージの `README_FINAL_TASKS.md` を参照してください。

- 約8 m四方の区画に、赤・緑・青のブロックを1個ずつ配置しています。
- ブロックは幅・奥行き40 cm、高さ60 cmです。ArUcoはありません。
- 探索では事前地図を読み込まず、演習のSLAMを使ってLiDARで地図を作ります。
- この起動ファイルはGazebo・ロボット・通信のみを起動します。探索アルゴリズムは含みません。
- 自律探索、色検出、ブロック位置の推定・記録、探索完了の判定、指定色への案内を実装してください。
- 画像中の位置だけでは地図座標は求められません。CameraInfo、LiDAR、TFを使います。
- 移動目標はブロックの内部ではなく、手前の通行可能な場所に設定してください。
- 評価時にはブロック位置を変更し、ワールドに書かれた座標を直接使わずに発見できるか確認します。

## 課題②：画像認識によるライントレース

```bash
ros2 launch turtlebot3_gazebo final_tasks.launch.py task:=line
```

- 青い線（幅10 cm）に沿って進み、最後の赤い横線で停止します。
- 直線と半径1 mの左右カーブがあります。初期位置は青い線の上で、前向きです。
- この課題だけカメラを約37度下に向け、床のラインを見えるようにします。
- 色抽出、ずれの算出、旋回・速度調整、見失ったときの処理、ゴール判定を実装してください。
- Nav2は起動せず、`/cmd_vel` に速度指令を送ります。

## 課題③：自然言語指示による自律移動

ターミナル1：Gazeboを起動します。

```bash
ros2 launch turtlebot3_gazebo final_tasks.launch.py task:=places
```

SLAM・Navigation用の設定はGazebo側には置きません。
課題③の地図と場所名の一覧は、演習用パッケージの
`lecture04_py/config/final_tasks/` にあります。
Navigationの起動と初期位置の設定は `lecture04_py/README_FINAL_TASKS.md` を参照してください。

例：「研究室に行って、その後会議室に移動して」「停止して」「右に回転して」

形態素解析器の選定・導入、場所名と動詞の抽出、行動の順序付け、Nav2への指示、
停止・回転を受講者側で実装します。解析器や解答プログラムはこの環境には追加していません。
停止時にはNav2の実行中ゴールもキャンセルし、複数のノードが速度指令を競合して出さないようにします。

## 共通の入出力

| 用途 | ROSインターフェース |
|---|---|
| カメラ画像 | `/camera/image_raw` : `sensor_msgs/msg/Image` |
| カメラ内部パラメータ | `/camera/camera_info` : `sensor_msgs/msg/CameraInfo` |
| LiDAR | `/scan` : `sensor_msgs/msg/LaserScan` |
| 走行指令 | `/cmd_vel` : `geometry_msgs/msg/TwistStamped` |
| 地図（課題①・③） | `/map` : `nav_msgs/msg/OccupancyGrid` |
| 地点への移動（課題①・③） | `/navigate_to_pose` : `nav2_msgs/action/NavigateToPose` |

画像とLiDARの購読は `qos_profile_sensor_data` を使います。
画像の座標系は `camera_rgb_optical_frame` です。地図と結び付けるときはTFを使ってください。
Gazeboは、演習と同じ `TwistStamped` の速度指令を受け取ります。
送信するメッセージの `header.stamp` に現在のROS時刻を設定し、
`twist.linear.x` と `twist.angular.z` に速度を指定してください。
送信ノードも `use_sim_time:=true` にしてGazeboの時刻を使います。

RVizで画像を表示するには、`Add` → `Image` を追加し、Topicを `/camera/image_raw`、
Reliabilityを `Best Effort` に設定します。ライントレースだけを確認する場合は、
別ターミナルで `rviz2 --ros-args -p use_sim_time:=true` を起動できます。

終了は各起動ターミナルで `Ctrl + C` を押します。
Gazeboを画面なしで確認したい場合は `gui:=false` を追加できます。

Gazebo通信は最終課題専用の区画（`GZ_PARTITION=turtlebot3_final_課題名`）を使います。
ROSのトピック名や使い方は変わりません。外部から `gz` コマンドを使う場合だけ、
同じ `GZ_PARTITION` を設定してください。起動前に値を指定した場合はその値を使います。

このPCのWSLでの試験では、Gazeboの終了時に `Segmentation fault` が出るケースがありました。
走行中の試験は成功しており、終了後にサーバーが残っていないことも確認しています。
配布前には対象PCでも起動・終了を確認してください。

## 配置を変更する場合

- ブロック位置：`worlds/final_exploration.world` の `include/pose`
- ブロック形状・色：`models/final_block_red`、`final_block_green`、`final_block_blue`
- ライン：`worlds/final_line.world`
- 場所指定用ワールド：`worlds/final_places.world`
- カメラ：`launch/final_tasks.launch.py`

課題③の壁を変更した場合は、地図も作り直してください。場所を変更した場合は、
床の表示と演習用パッケージの `places_names.yaml` の座標を合わせてください。

## ワールドの見た目・配置

- 3課題ともGazeboのマス目（グリッド）を非表示にしています。
- 課題①は仕切り壁で視線を遮っています。初期位置から色を見つけるだけでなく、
  通路を移動して壁の裏側を探索してください。
- 課題②はコース全体を外周の壁で囲っています。線の上や曲がり角には壁を置いていません。
- 課題③は「受付・研究室・会議室・充電場所」を日本語で床に表示し、色付きの枠で囲っています。
  床の表示に衝突判定はなく、地図・目的地座標は変更していません。
