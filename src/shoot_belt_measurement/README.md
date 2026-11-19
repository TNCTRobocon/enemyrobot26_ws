# shoot_belt_measurement

shoot_belt の Action 開始、目標電流 CAN フレームのSocketCAN受信、位置フィードバック変化のタイミングを計測し、位置・速度の時系列を CSV に保存する ROS 2 パッケージです。can_socketcan_common::CanReceiver を使用します。

## 計測内容

- /shoot_belt/execute に operation=1 の Action Goal を1回送信する。
- Action Client API の Goal 送信直前を計測時刻0とする。
- can_interface（既定値 can0）を直接受信し、標準CAN ID 0x100、DLC=4のfloat32 little-endianフレームのうち、絶対値が current_zero_epsilon_a を超える最初の電流指令を検出する。初期化時の0 Aフレームは除外する。
- 最初の非ゼロ電流CANフレーム受信時点の最新位置を基準にし、位置変化しきい値を連続サンプルで超えた場合に位置変化を確定する。
- しきい値を最初に超えた時刻と、連続サンプル数を満たして確定した時刻の両方を保存する。
- Action開始からAction終了まで、位置または速度の更新ごとに最新値と各サンプルの時刻・経過時間・サンプル年齢を保存する。

CANフレームは CanReceiver::receive() がSocketCANソケットから返した直後に検出します。Action送信時刻、電流CAN受信時刻、位置・速度フレーム受信時刻はすべて std::chrono::steady_clock で記録します。送信側ソケットからのローカルループバック受信を利用するため、SocketCANのループバックが有効である必要があります。ハードウェアタイムスタンプではないため、CSVの時刻は物理バス上の厳密な送信時刻ではなく、Linux SocketCAN受信経路でユーザー空間に届いた時刻です。

位置変化の基準位置は、同じ受信スレッドで電流CANフレームより前に処理した最新の位置フィードバックです。ROSトピックのコールバック順序やROS時刻との変換には依存しません。

## ビルド

```bash
source /opt/ros/jazzy/setup.bash
cd ~/enemyrobot26_ws
colcon build --packages-up-to shoot_belt_measurement --symlink-install
source install/setup.bash
```

## 起動・計測

従来どおり shoot_belt と ros2_socketcan の送受信ノードを起動します。shoot_belt_measurement はそれらのROSトピックを購読せず、同じ can0 を独立したSocketCANソケットで受信します。

    source /opt/ros/jazzy/setup.bash
    cd ~/enemyrobot26_ws
    source install/setup.bash
    ros2 launch shoot_belt_measurement measurement.launch.py

別ターミナルから計測開始を明示的に要求します。

    source /opt/ros/jazzy/setup.bash
    cd ~/enemyrobot26_ws
    source install/setup.bash
    ros2 service call /shoot_belt_measurement/start std_srvs/srv/Trigger "{}"

サービス要求を受け付けると、Action Server と新しい位置CANフィードバックが利用可能になった時点で operation=1 の Goal を1回送信します。ノード起動だけではモーター動作を開始しません。同時に複数の計測を開始することはできません。

## CSV出力

デフォルトの保存先は ~/.ros/shoot_belt_measurement/<日時>/ です。

- summary.csv: Action→SocketCANでの電流CAN受信、電流CAN→位置変化、位置変化の確定時間、最大速度など。action_to_current_can_socketcan_receive_ms はAction送信直前からSocketCAN受信までの時間です。
- timeseries.csv: Action開始からの位置・速度のサンプル時刻と値。各行の updated_signal がその行を追加した信号（position または velocity）を示す。もう一方の値は最新サンプルを保持し、*_age_ms で古さを確認できる。

## 主なパラメータ

config/measurement.yaml を参照してください。can_interface は受信するLinux CANインターフェース、can_error_log_path と can_error_log_interval_sec は共通ライブラリの受信エラーログ設定です。特に position_change_threshold_mm と position_change_confirmation_samples は位置フィードバックのノイズに合わせて調整します。

current_can_to_position_change_ms は最初のしきい値交差までの時間です。current_can_to_position_confirmation_ms は連続サンプル条件を満たすまでの時間です。
