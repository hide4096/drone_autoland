# ロボット設計製作論4 
移動ロボットに設置した上向きカメラによるドローンの自動着陸の実装と評価

## About

Telloドローンを地上のカメラで誘導して自動着陸させるプログラムです

## Requirement
- ROS2 Humble
- [tello_ros](https://github.com/clydemcqueen/tello_ros)
- [ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco)
- [ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)

次の環境で動作を確認しています
- Ubuntu 22.04
- ROS2 Humble

## Installation

- Requirementにあるパッケージをインストールします
  - [tello_ros](https://github.com/clydemcqueen/tello_ros)
  - [ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco)
  - [ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)

- このリポジトリをクローンして、ビルドします
    ```
    cd ~/ros2ws/src
    https://github.com/hide4096/drone_autoland
    cd ~/ros2_ws && catkin_make
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

  - 準備
    - Telloを地面に置いて電源を入れます
    - コンピューターをTelloが発しているアクセスポイントに接続します
  - 実行
    次のコマンドを入力すると動作を開始します
    ```
    ros2 launch drone_autoland guide_drone
    ```

## License

- Telloの操作は[tello_ros](https://github.com/clydemcqueen/tello_ros)を使用しています。
- Arucoマーカーの検知、生成は[ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco)を使用しています。
- ARマーカーの検出は[ar_track_alvar](https://github.com/ros-perception/ar_track_alvar)を使用しています。
- Webカメラのドライバは[ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)を使用しています。
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます．

© 2022 Aso Hidetoshi
