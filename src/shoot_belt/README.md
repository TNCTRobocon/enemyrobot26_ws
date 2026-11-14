# shoot_belt

ベル直射出制御用のROS 2パッケージです。CAN送受信とシーケンスを3ノードに分離し、ジョイスティック入力用のテストAction Clientを別ノードにしています。STM32側ファームウェア、hon26_can、ros2_socketcanのソースコードは変更していません。

## ノード構成

```text
Action / stop / emergency-stop
            |
            v
+----------------------------+
| shoot_belt_sequence        |
| Action、射出状態遷移、停止 |
+----------------------------+
        | command topic
        v
+----------------------------+        +---------------------------+
| shoot_belt_can_tx          |------->| /can_tx                   |
| command -> CAN frame       |        +---------------------------+
+----------------------------+                    |
                                                   v
                                              ros2_socketcan
                                                   |
                                                  CAN
                                                   |
+----------------------------+                    |
| shoot_belt_can_rx          |<-------| /can_rx                   |
| CAN frame -> feedback      |        +---------------------------+
+----------------------------+
        |
        v
typed feedback topics
        |
        +----> shoot_belt_sequence
```

### ノードと入出力

| ノード | 入力 | 出力 | 責務 |
|---|---|---|---|
| `shoot_belt_sequence` | Action、停止要求、フィードバック | 内部コマンド、受付状態 | 射出シーケンス、モード確認、位置復帰、異常処理 |
| `shoot_belt_joy_test` | `/joy`、`/shoot_belt/operation_state` | `/kari` Action | ボタン入力の立ち上がり検出とテストAction送信 |
| `shoot_belt_can_rx` | `/can_rx` | 位置、モード、フィードバック異常 | CAN受信フレームの検証とデコード |
| `shoot_belt_can_tx` | 内部コマンド | `/can_tx` | コマンドをCANフレームへ変換して送信トピックへ出力 |

## トピック

| トピック | 型 | 用途 |
|---|---|---|
| `/can_tx` | `can_msgs/msg/Frame` | 送信するCANフレーム |
| `/can_rx` | `can_msgs/msg/Frame` | 受信したCANフレーム |
| `/shoot_belt/can_tx_command` | `shoot_belt/msg/ShootBeltCommand` | シーケンスからCAN TXノードへの順序付きコマンド |
| `/shoot_belt/feedback/position` | `std_msgs/msg/Float32` | 現在位置 (符号付き mm) |
| `/shoot_belt/feedback/control_mode` | `std_msgs/msg/UInt8` | 現在の制御モード |
| `/shoot_belt/feedback/fault` | `std_msgs/msg/String` | CAN RX側で発見した不正フィードバック |
| `/shoot_belt/operation_state` | `std_msgs/msg/UInt8` | 射出状態。`0=ERROR`、`1=RUNNING`、`2=READY`、`3=CANCELLED`、`4=RESETTING`、`5=RESET_FAILED`。100 ms周期、transient-local |
| `/joy` | `sensor_msgs/msg/Joy` | `joy_node` から受信するジョイスティック入力 |
| `/shoot_belt/stop` | `std_msgs/msg/Empty` | 専用停止要求。ブレーキモード3と0 Aを指令し、射出中ならシーケンスを中断 |
| `/shoot_belt/emergency_stop` | `std_msgs/msg/Bool` | 非常停止状態入力。trueで停止、falseで入力解除 |
| `/shoot_belt/execute` | `sequence_interfaces/action/Sequence` | 射出シーケンス |

内部コマンドは単一トピックで送るため、モード・位置・電流の指令を別々のトピックへ分ける方式より、指令順序を維持しやすい設計です。

## サービス

| サービス | 型 | 用途 |
|---|---|---|
| `/shoot_belt/manual_current` | `shoot_belt/srv/ManualCurrent` | 次回の射出に使用する電流設定値の変更（モーター指令は送らない） |
| `/shoot_belt/reset_origin` | `std_srvs/srv/Trigger` | ブレーキ・停止確認後に原点を再設定し、位置制御で0 mm保持する。初期化中は射出Actionを受け付けない |

## 射出電流設定 GUI

`target_gui` の `/shoot_belt/manual_current` サービスは、射出に使用する設定値を更新するだけです。「高」「中」「低」ボタンは、それぞれ `manual_bucket_1_current_a`、`manual_bucket_2_current_a`、`manual_bucket_3_current_a`、「移動」ボタンは `manual_move_current_a`（`shoot_belt/config/shoot_belt.yaml`）の値を設定し、UP/DOWNは設定値を増減します。このサービスは制御モードを切り替えず、電流指令やCANフレームを送信しません。

現在の射出電流設定値は `/shoot_belt/manual_current_state` (`std_msgs/msg/Float32`) で通知します。射出Actionを受理した時点の設定値をその射出中は固定し、射出中にGUIで変更した値は次回以降の射出に使用します。

`/shoot_belt/operation_state` (`std_msgs/msg/UInt8`) はshoot_beltが一元管理し、100 ms周期で通知します。状態コードは `0=ERROR`、`1=RUNNING`、`2=READY`、`3=CANCELLED`、`4=RESETTING`、`5=RESET_FAILED` です。`READY` 以外ではAction Goalを受け付けません。Action正常終了時だけ `READY` に戻り、CANCEL/停止では `CANCELLED`、その他のAction失敗では `ERROR` になります。`CANCELLED`、`ERROR`、`RESET_FAILED` からの復帰はRESET要求だけが可能で、RESET処理中は `RESETTING` を維持し、初期化成功時だけ `READY`、失敗時は `RESET_FAILED` になります。

## ジョイスティックによるテストAction送信

`shoot_belt_joy_test` は `joy_node` が配信する `/joy` (`sensor_msgs/msg/Joy`) を購読します。`buttons[0]` の押下立ち上がり（0→1）を1回検出し、`/kari` (`sequence_interfaces/action/Sequence`) に `operation=1` (`OPERATION_FULL_CYCLE`) のGoalを送信します。送信先はテスト用の `/kari` に固定しており、実機用の `/shoot_belt/execute` には接続しません。

送信前に `/shoot_belt/operation_state` が `2=READY` であることを確認します。`ERROR`、`RUNNING`、`CANCELLED`、`RESETTING`、`RESET_FAILED`、状態未受信の場合は送信しません。初回Joyメッセージはボタン状態の初期化に使用するため、ボタンを押したまま起動した場合は、一度離してから押してください。Actionの応答待ち・実行中に追加のGoalは送りません。Action Server `/kari` が起動していない場合も送信しません。

`shoot_belt.launch.py` でテストクライアントも起動します。ジョイスティックドライバーは別ターミナルから起動してください。

```bash
ros2 run joy joy_node
```

## CANプロトコル

| ID | 方向 | 内容 | DLC / encoding |
|---|---|---|---|
| 0x100 | 送信 | 目標電流 A | 4 bytes, float32 little-endian |
| 0x120 | 送信 | 目標位置 mm (符号付き) | 4 bytes, float32 little-endian |
| 0x130 | 送信 | 制御モード (0=current, 1=velocity, 2=position, 3=brake) | 1 byte |
| 0x140 | 送信 | 起動時の原点リセット指令 (true = 現在位置を0 mmにリセット) | 1 byte, 0x01 |
| 0x210 | 受信 | モーター速度 [rad/s] | 4 bytes, float32 little-endian (要・基板仕様照合) |
| 0x220 | 受信 | 現在位置 mm (符号付き、直近の原点リセット基準) | 4 bytes, float32 little-endian |
| 0x230 | 受信 | 現在の制御モード | 1 byte |

## 原点初期化と通常シーケンス

ノード起動時は自動で原点初期化します。また、GUIの「リセット」ボタンから `/shoot_belt/reset_origin` (`std_srvs/srv/Trigger`) を呼び出して手動でも再初期化できます。どちらも位置・モードのフィードバックを確認し、ブレーキモードへの切り替えと速度停止を確認してから 0x140 に 0x01 を送信します。リセット後の位置フィードバックが0 mm付近で安定し、位置制御モードで0 mmを保持できるまで、射出Actionは受け付けません。初期化中は新しい射出Actionを拒否し、停止要求・非常停止・フィードバック異常時は処理を中断してブレーキ指令を送ります。原点はその時点の物理的位置に設定され、機構を物理的に移動させる原点探索は行いません。

通常の射出シーケンス:

1. 現在位置・制御モードのフィードバックが新鮮か確認し、加速開始位置を保存。
2. 電流指令を0 Aにして電流制御モード0へ切り替え、モードフィードバックを確認。
3. YAML指定の一定電流で加速し、開始位置からの移動距離が max_acceleration_distance_mm に達するまで加速。
4. 電流指令を0 Aにしてブレーキモード3へ切り替え、速度ID 0x210がしきい値以下で安定するまで待つ。速度が更新されない場合は異常、最大待機時間を超えた場合は異常終了。
5. ブレーキ完了後の最新位置フィードバックを取得し、現在位置以上となる最小の origin_return_interval_mm の整数倍を目標位置として計算する。目標位置はそのAction中は固定する。
6. 計算した目標位置を送信し、位置制御モード2に切り替える。
7. 計算した目標位置への到達と位置の安定を確認し、その位置を位置制御モード2で保持してActionを正常終了する。シーケンス正常完了時には原点リセットを行わず、位置座標を維持する。

目標位置は ceil(現在位置 / origin_return_interval_mm) * origin_return_interval_mm で計算します。例えば間隔が1125 mmの場合、現在位置1125 mmなら1125 mm、1126 mmなら2250 mmを目標にします。CAN ID 0x140による原点リセットは、ノード起動時または `/shoot_belt/reset_origin` サービスによる初期化時に実行し、通常シーケンス完了時には送信しません。原点リセットは、リセット要求より後に受信した位置フィードバックが0 mm付近で安定したことを確認します。origin_reset_timeout_sec 内に確認できなければ初期化を失敗とし、Actionを受け付けません。

ブレーキ終了判定には、速度しきい値 brake_velocity_threshold_rad_s、安定時間 brake_velocity_stable_time_sec、最大待機時間 brake_timeout_sec、速度フィードバックの更新監視 velocity_feedback_timeout_sec を使います。brake_wait_sec は停止要求・異常時にブレーキ指令を繰り返す有限時間として残し、通常シーケンスの停止判定には使用しません。

専用停止トピック、非常停止入力、Action cancel、フィードバック異常、タイムアウトは通常の原点復帰を中止し、原点リセット指令を新たに送信せず、ブレーキモードと0 A指令でActionを失敗またはキャンセル終了します。起動時初期化に失敗した場合もActionは受け付けず、問題を解消してノードを再起動する必要があります。

## ビルド

```bash
cd /home/miquiteru/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to shoot_belt --symlink-install
source install/setup.bash
```

## 起動

まず `ros2_socketcan` を別ターミナルで起動します。

送信側:

```bash
ros2 launch ros2_socketcan socket_can_sender.launch.py interface:=can0 to_can_bus_topic:=/can_tx
```

受信側:

```bash
ros2 launch ros2_socketcan socket_can_receiver.launch.py interface:=can0 from_can_bus_topic:=/can_rx
```

3ノードを起動:

```bash
ros2 launch shoot_belt shoot_belt.launch.py
```

Actionを実行:

```bash
ros2 action send_goal /shoot_belt/execute sequence_interfaces/action/Sequence "{operation: 1}"
```

停止要求（GUIの「ブレーキ」ボタンも同じトピックを使用）:

```bash
ros2 topic pub --once /shoot_belt/stop std_msgs/msg/Empty "{}"
```

手動原点初期化:

```bash
ros2 service call /shoot_belt/reset_origin std_srvs/srv/Trigger "{}"
```

非常停止状態入力 (trueでブレーキ指令):

```bash
ros2 topic pub --once /shoot_belt/emergency_stop std_msgs/msg/Bool "{data: true}"
```
