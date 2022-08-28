// Copyright 2022 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "consai_frootspi_msgs/msg/robot_command.hpp"
#include "consai_msg_publisher/key_input.hpp"

using namespace std::chrono_literals;
using namespace consai_frootspi_msgs::msg;


class MinimalPublisher : public rclcpp::Node
{

constexpr static int MAX_ROBOT_ID = 15;
constexpr static double MAX_VELOCITY_XY = 2.0;
constexpr static double ADD_VELOCITY_XY = 0.1;
constexpr static double MAX_VELOCITY_THETA = 2.0 * M_PI;
constexpr static double ADD_VELOCITY_THETA = 0.1 * M_PI;
constexpr static double MAX_KICK_POWER = 8.0;  // m/s
constexpr static double MAX_DRIBBLE_POWER = 1.0;  // 0.0 ~ 1.0

// Ref: https://www.k-cube.co.jp/wakaba/server/ascii_code.html
constexpr static int ESC_ASCII_VALUE = 0x1b;

public:
  MinimalPublisher()
  : Node("robot_command_publisher")
  {
    for (auto i = 0; i <= MAX_ROBOT_ID; i++) {
      std::string topic_name = "robot" + std::to_string(i) + "/command";
      auto publisher = this->create_publisher<RobotCommand>(topic_name, 10);
      publishers_.push_back(publisher);
    }

    timer_ = this->create_wall_timer(
      50ms, std::bind(&MinimalPublisher::timer_callback, this));

    robot_id_ = 0;
    team_is_yellow_ = false;
    publish_has_enabled_ = true;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_theta_ = 0.0;

    print_instructions();
  }

private:
  void timer_callback()
  {
    if (key_input::kbhit()) {
      auto key_value = key_input::getch();
      if (key_value == ESC_ASCII_VALUE) {
        RCLCPP_INFO(this->get_logger(), "トピックを初期化します");
        reset_velocity();
        reset_kick_dribble_power();

      } else if (key_value == 'p' || key_value == 'P') {
        RCLCPP_INFO(this->get_logger(), "PublishをOn/Offします");
        publish_has_enabled_ = !publish_has_enabled_;

      } else if (key_value == 'q' || key_value == 'Q') {
        RCLCPP_INFO(this->get_logger(), "ロボットIDを増やします");
        // 現在のトピックを初期化してパブリッシュ
        reset_velocity();
        reset_kick_dribble_power();
        publish_topic();
        if (robot_id_ < MAX_ROBOT_ID) {
          robot_id_++;
        }

      } else if (key_value == 'z' || key_value == 'Z') {
        RCLCPP_INFO(this->get_logger(), "ロボットIDを減らします");
        // 現在のトピックを初期化してパブリッシュ
        reset_velocity();
        reset_kick_dribble_power();
        publish_topic();
        if (robot_id_ > 0) {
          robot_id_--;
        }

      } else if (key_value == '1') {
        RCLCPP_INFO(this->get_logger(), "team_is_yellowを切り替えます");
        // 現在のトピックを初期化してパブリッシュ
        reset_velocity();
        reset_kick_dribble_power();
        publish_topic();
        team_is_yellow_ = !team_is_yellow_;

      } else if (key_value == 'i' || key_value == 'I') {
        RCLCPP_INFO(this->get_logger(), "前速度の増加: velocity_x > 0");
        if (velocity_x_ < MAX_VELOCITY_XY) {
          velocity_x_ += ADD_VELOCITY_XY;
        }

      } else if (key_value == 'k' || key_value == 'K') {
        RCLCPP_INFO(this->get_logger(), "後速度の増加: velocity_x < 0");
        if (velocity_x_ > -MAX_VELOCITY_XY) {
          velocity_x_ -= ADD_VELOCITY_XY;
        }

      } else if (key_value == 'j' || key_value == 'J') {
        RCLCPP_INFO(this->get_logger(), "左速度の増加: velocity_y > 0");
        if (velocity_y_ < MAX_VELOCITY_XY) {
          velocity_y_ += ADD_VELOCITY_XY;
        }

      } else if (key_value == 'l' || key_value == 'L') {
        RCLCPP_INFO(this->get_logger(), "右速度の増加: velocity_y < 0");
        if (velocity_y_ > -MAX_VELOCITY_XY) {
          velocity_y_ -= ADD_VELOCITY_XY;
        }

      } else if (key_value == 'a' || key_value == 'A') {
        RCLCPP_INFO(this->get_logger(), "回転速度の増加: velocity_theta > 0");
        if (velocity_theta_ < MAX_VELOCITY_THETA) {
          velocity_theta_ += ADD_VELOCITY_THETA;
        }

      } else if (key_value == 'd' || key_value == 'D') {
        RCLCPP_INFO(this->get_logger(), "回転速度の増加: velocity_theta < 0");
        if (velocity_theta_ > -MAX_VELOCITY_THETA) {
          velocity_theta_ -= ADD_VELOCITY_THETA;
        }

      } else if (key_value == 's' || key_value == 'S') {
        RCLCPP_INFO(this->get_logger(), "ストップ");
        reset_velocity();

      } else if (key_value == 'f' || key_value == 'F') {
        RCLCPP_INFO(this->get_logger(), "キックパワーの増加: kick_power > 0");
        if (kick_power_ < MAX_KICK_POWER) {
          kick_power_ += 1.0;
        }
        if (kick_power_ > MAX_KICK_POWER) {
          kick_power_ = MAX_KICK_POWER;
        }

      } else if (key_value == 'v' || key_value == 'V') {
        RCLCPP_INFO(this->get_logger(), "キックパワーの減少: kick_power < 0");
        if (kick_power_ > 0.0) {
          kick_power_ -= 1.0;
        }
        if (kick_power_ < 0.0) {
          kick_power_ = 0.0;
        }

      } else if (key_value == 'g' || key_value == 'G') {
        RCLCPP_INFO(this->get_logger(), "ドリブルパワーの増加: dribble_power > 0");
        if (dribble_power_ < MAX_DRIBBLE_POWER) {
          dribble_power_ += 0.1;
        }
        if (dribble_power_ > MAX_DRIBBLE_POWER) {
          dribble_power_ = MAX_DRIBBLE_POWER;
        }

      } else if (key_value == 'b' || key_value == 'B') {
        RCLCPP_INFO(this->get_logger(), "ドリブルパワーの減少: dribble_power < 0");
        if (dribble_power_ > 0.0) {
          dribble_power_ -= 0.1;
        }
        if (dribble_power_ < 0.0) {
          dribble_power_ = 0.0;
        }
      }
      print_instructions();
    }

    round_velocities();
    publish_topic();
  }

  void publish_topic()
  {
    // RobotCommandをパブリッシュする
    if (publish_has_enabled_) {
      auto msg = RobotCommand();
      msg.robot_id = robot_id_;
      msg.team_is_yellow = team_is_yellow_;
      msg.velocity_x = velocity_x_;
      msg.velocity_y = velocity_y_;
      msg.velocity_theta = velocity_theta_;
      msg.kick_power = kick_power_;
      msg.dribble_power = dribble_power_;

      publishers_[robot_id_]->publish(msg);
    }
  }

  void print_instructions()
  {
    // 操作方法を表示する
    std::cout << std::endl;
    std::cout << "Esc: トピックを初期化" << std::endl;
    std::cout << "p: トピックPublishのON/OFF (現在:" << publish_has_enabled_ << std::endl;
    std::cout << "q/z: ロボットIDの増減 (現在:" << robot_id_ << std::endl;
    std::cout << "1: team_is_yellowの切り替え (現在:" << team_is_yellow_ << std::endl;
    std::cout << "i/k: velocity_x 前後速度m/sの増減（現在:" << velocity_x_ << std::endl;
    std::cout << "j/l: velocity_y 左右速度m/sの増減（現在:" << velocity_y_ << std::endl;
    std::cout << "a/d: velocity_theta 回転速度rad/sの増減（現在:" << velocity_theta_ << std::endl;
    std::cout << "s: 前後左右回転速度を0にする" << std::endl;
    std::cout << "f/v: kick_power m/sを増減する（現在:" << kick_power_ << std::endl;
    std::cout << "g/b: dribble_powerを増減する（現在:" << dribble_power_ << std::endl;
  }

  void round_velocities()
  {
    // 走行速度を丸める
    if (std::fabs(velocity_x_) < ADD_VELOCITY_XY) {
      velocity_x_ = 0.0;
    }
    if (std::fabs(velocity_y_) < ADD_VELOCITY_XY) {
      velocity_y_ = 0.0;
    }
    if (std::fabs(velocity_theta_) < ADD_VELOCITY_THETA) {
      velocity_theta_ = 0.0;
    }
    velocity_x_ = std::clamp(velocity_x_, -MAX_VELOCITY_XY, MAX_VELOCITY_XY);
    velocity_y_ = std::clamp(velocity_y_, -MAX_VELOCITY_XY, MAX_VELOCITY_XY);
    velocity_theta_ = std::clamp(velocity_theta_, -MAX_VELOCITY_THETA, MAX_VELOCITY_THETA);
  }

  void reset_velocity()
  {
    // 走行速度を0にする
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_theta_ = 0.0;
  }

  void reset_kick_dribble_power()
  {
    // キックパワーとドリブルパワーを0にする
    kick_power_ = 0.0;
    dribble_power_ = 0.0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<RobotCommand>::SharedPtr> publishers_;

  int robot_id_;
  bool team_is_yellow_;
  bool publish_has_enabled_;
  double velocity_x_;
  double velocity_y_;
  double velocity_theta_;
  double kick_power_;
  double dribble_power_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
