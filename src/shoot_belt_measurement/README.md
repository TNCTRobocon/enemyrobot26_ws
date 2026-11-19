# shoot_belt_measurement

shoot_belt の Action 開始、目標電流 CAN 送信、位置フィードバック変化のタイミングを計測し、位置・速度の時系列を CSV に保存する ROS 2 パッケージです。

## 計測内容

- /shoot_belt/execute に operation=1 の Action Goal を1回送信する。
- Action Client API の Goal 送信直前を計測時刻0とする。
- /can_tx の標準CAN ID 0x100を監視し、絶対値が current_zero_epsilon_a を超える最初の電流指令を検出する。初期化時の0 Aフレームは除外する。
- 最初の非ゼロ電流CANフレーム受信時点の最新位置を基準にし、位置変化しきい値を連続サンプルで超えた場合に位置変化を確定する。
- しきい値を最初に超えた時刻と、連続サンプル数を満たして確定した時刻の両方を保存する。
- Action開始からAction終了まで、位置または速度の更新ごとに最新値と各サンプルの時刻・経過時間・サンプル年齢を保存する。

CANフレームは /can_tx の購読コールバックで検出します。summary.csv には CAN TX ノードが設定する header.stamp を利用した publish 時刻基準の遅延と、計測ノードでの受信時刻基準の遅延を別々に保存します。publish 時刻は同一 ROS clock を共有する場合に利用できる値で、物理的にCANバスへ送信された時刻そのものではありません。

位置変化の基準位置は、CANフレームの header.stamp 以前に受信した位置フィードバックのうち最新のサンプルです。これにより、ROSコールバックの処理順が入れ替わった場合でも、CAN送信後の位置を基準値に取り込むことを避けます。

## 起動・計測

shoot_belt と CAN送受信ノードを通常どおり起動した後、別ターミナルで実行します。

    source /opt/ros/jazzy/setup.bash
    cd ~/enemyrobot26_ws
    source install/setup.bash
    ros2 launch shoot_belt_measurement measurement.launch.py

別ターミナルから計測開始を明示的に要求します。

    source /opt/ros/jazzy/setup.bash
    cd ~/enemyrobot26_ws
    source install/setup.bash
    ros2 service call /shoot_belt_measurement/start std_srvs/srv/Trigger "{}"

サービス要求を受け付けると、Action Server と新しい位置フィードバックが利用可能になった時点で operation=1 の Goal を1回送信します。ノード起動だけではモーター動作を開始しません。同時に複数の計測を開始することはできません。

## CSV出力

デフォルトの保存先は ~/.ros/shoot_belt_measurement/<日時>/ です。

- summary.csv: Action→電流CAN、電流CAN→位置変化、位置変化の確定時間、最大速度など。
- timeseries.csv: Action開始からの位置・速度のサンプル時刻と値。各行の updated_signal がその行を追加したトピックを示す。もう一方の値は最新サンプルを保持し、*_age_ms で古さを確認できる。

## 主なパラメータ

config/measurement.yaml を参照してください。特に position_change_threshold_mm と position_change_confirmation_samples は位置フィードバックのノイズに合わせて調整します。

current_can_to_position_change_ms は最初のしきい値交差までの時間です。current_can_to_position_confirmation_ms は連続サンプル条件を満たすまでの時間です。
