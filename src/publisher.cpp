// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher() : Node("minimal_publisher"), tf_broadcaster_(*this), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Alternate between sending data and transform
    if (count_ % 2 == 1) {
      geometry_msgs::msg::PointStamped message;
      message.header.stamp.sec = count_;
      publisher_->publish(message);
    } else {
      geometry_msgs::msg::TransformStamped message;
      message.header.stamp.sec = count_;
      message.header.frame_id = "a";
      message.child_frame_id = "b";
      tf_broadcaster_.sendTransform(message);
    }
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
