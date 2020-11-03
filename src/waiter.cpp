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

#include <chrono>
#include <functional>
#include <memory>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Waiter : public rclcpp::Node
{
public:
  Waiter() : Node("minimal_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "point", 10, std::bind(&Waiter::topic_callback, this, _1));
    tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(cti);
    timeout_ms_ = this->declare_parameter("timeout_ms").get<int>();
    wait_inside_ = this->declare_parameter("wait_inside").get<bool>();
    wait_outside_ = this->declare_parameter("wait_outside").get<bool>();
    most_recent_ = this->declare_parameter("most_recent").get<bool>();
    // Actually there is another dimension, whether or not to pass 0 to the timeout
    RCLCPP_INFO(get_logger(), "timeout is %dms", timeout_ms_);
  }

private:
  void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    tf2::TimePoint msg_time = most_recent_ ? tf2::TimePointZero : tf2_ros::fromMsg(msg->header.stamp);
    int t = msg->header.stamp.sec;

    RCLCPP_INFO(get_logger(), "[outside %d] Calling waitForTransform()", t);
    auto future = tf_buffer_.waitForTransform(
      "a", "b", msg_time, std::chrono::milliseconds(timeout_ms_),
      [this, msg_time, t](const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) {
        if (this->wait_inside_) {
          try {
            RCLCPP_INFO(get_logger(), "[inside %d] Calling get() ...", t);
            tf2::TimePoint lu_time = tf2_ros::fromMsg(tf.get().header.stamp);
            if (msg_time != lu_time) {
              int t_wrong = tf.get().header.stamp.sec;
              RCLCPP_WARN(get_logger(),
                "[inside " + std::to_string(t) + "] got wrong timestamp " +
                std::to_string(t_wrong));
            }
            RCLCPP_INFO(get_logger(), "[inside %d] Returned from get() with transform", t);
          } catch (tf2::TimeoutException & e) {
            RCLCPP_INFO(get_logger(), "[inside %d] Transform timed out", t);
          }
        }
      });
    RCLCPP_INFO(get_logger(), "[outside %d] Returned from waitForTransform()", t);
    if (this->wait_outside_) {
      RCLCPP_INFO(get_logger(), "[outside %d] Calling wait_for()", t);
      auto status = future.wait_for(std::chrono::milliseconds(this->timeout_ms_));
      if (status == std::future_status::deferred) {
        RCLCPP_INFO(get_logger(), "[outside %d] Status is deferred", t);
      } else if (status == std::future_status::timeout) {
        RCLCPP_INFO(get_logger(), "[outside %d] Status is timeout", t);
      } else {
        RCLCPP_INFO(get_logger(), "[outside %d] Status is ready, calling get()", t);
        tf2::TimePoint lu_time = tf2_ros::fromMsg(future.get().header.stamp);
        if (msg_time != lu_time) {
          int t_wrong = future.get().header.stamp.sec;
          RCLCPP_WARN(get_logger(),
            "[outside " + std::to_string(t) + "] got wrong timestamp " +
            std::to_string(t_wrong));
        }
      }
    }
  }


  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  int timeout_ms_;
  bool wait_inside_;
  bool wait_outside_;
  bool most_recent_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Waiter>());
  rclcpp::shutdown();
  return 0;
}
