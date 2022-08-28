# consai_msg_publisher

[![industrial_ci](https://github.com/SSL-Roots/consai_msg_publisher/actions/workflows/industrial_ci.yaml/badge.svg?branch=main)](https://github.com/SSL-Roots/consai_msg_publisher/actions/workflows/industrial_ci.yaml)


このパッケージは、consai_ros2やconsai_frootspi_msgsのメッセージをpublishするノードで構成されています。

## Installation

```sh
$ cd ~/ros2_ws/src
$ git clone https://github.com/SSL-Roots/consai_msg_publisher.git
$ git clone https://github.com/SSL-Roots/consai_frootspi_msgs.git
$ rosdep install -r -y --from-paths . --ignore-src

$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## Usage

### robot_command_publisher

キーボード操作で
consai_frootspi_msgsのRobotCommandメッセージをパブリッシュします

トピック名は`/robot*/command`です

```sh
$ ros2 run consai_msg_publisher robot_command_publisher 

Esc: トピックを初期化
p: トピックPublishのON/OFF (現在:1
q/z: ロボットIDの増減 (現在:0
1: team_is_yellowの切り替え (現在:0
i/k: velocity_x 前後速度m/sの増減（現在:0
j/l: velocity_y 左右速度m/sの増減（現在:0
a/d: velocity_theta 回転速度rad/sの増減（現在:0
s: 前後左右回転速度を0にする
f/v: kick_power m/sを増減する（現在:0
g/b: dribble_powerを増減する（現在:0

...
# 終了時はCtrl+Cを押す
```

## LICENSE

Apache-2.0
