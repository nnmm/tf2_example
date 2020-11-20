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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <chrono>
#include <memory>
#include <unordered_set>

using namespace std::chrono_literals;

class Sender : public rclcpp::Node
{
public:
  Sender() : Node("sender"), tf_broadcaster_(*this)
  {
    send_first_transform_ = this->declare_parameter("send_first_tf").get<std::string>();
    send_second_transform_ = this->declare_parameter("send_second_tf").get<std::string>();
    const std::unordered_set<std::string> allowed = {"before", "during", "after"};
    if (!allowed.count(send_first_transform_)) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid value for parameter 'send_first_tf'. Allowed values are 'before', 'during' "
        "or 'after' the waiting period.");
    }
    if (!allowed.count(send_second_transform_)) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid value for parameter 'send_second_tf'. Allowed values are 'before', "
        "'during' or 'after' the waiting period.");
    }

    // Only start sending after the receiver is up, which it signals with a "ready" message
    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    auto callback = [this](const std_msgs::msg::Empty::ConstSharedPtr) { this->run(); };
    ready_sub_ = this->create_subscription<std_msgs::msg::Empty>("ready", durable_qos, callback);

    trigger_pub_ = this->create_publisher<std_msgs::msg::Empty>("trigger", 1);

    // For less code duplication
    tf_1_.header.frame_id = "a";
    tf_1_.child_frame_id = "b";
    tf_1_.header.stamp.sec = 100;
    tf_2_ = tf_1_;
    tf_2_.header.stamp.sec = 104;

    // I assume it takes some time for transforms to propagate to the other
    // node's buffer, and the actual time that a timeout is active can not be
    // predicted exactly either because of thread startup time etc. So after
    // sending things to the other node, or as a buffer zone for timer-related
    // things, wait for a certain amount of time.
    const int send_buffer_time_ms = this->declare_parameter("send_buffer_time_ms", 5);
    send_buffer_time_ = std::chrono::milliseconds(send_buffer_time_ms);

    const int timeout_ms = this->declare_parameter("timeout_ms").get<int>();
    timeout_ = std::chrono::milliseconds(timeout_ms);
  }

private:
  void send_first_transform()
  {
    RCLCPP_INFO(get_logger(), "Sending first transform");
    tf_broadcaster_.sendTransform(tf_1_);
    std::this_thread::sleep_for(send_buffer_time_);
  }

  void send_second_transform()
  {
    RCLCPP_INFO(get_logger(), "Sending second transform");
    tf_broadcaster_.sendTransform(tf_2_);
    std::this_thread::sleep_for(send_buffer_time_);
  }

  void send_trigger()
  {
    RCLCPP_INFO(get_logger(), "Sending trigger");
    trigger_pub_->publish(std_msgs::msg::Empty());
    std::this_thread::sleep_for(send_buffer_time_);
  }

  void run()
  {
    const auto half_timeout = std::chrono::milliseconds(timeout_.count() / 2);
    if (send_first_transform_ == "before") {
      send_first_transform();
    }
    if (send_second_transform_ == "before") {
      send_second_transform();
    }

    send_trigger();
    std::this_thread::sleep_for(half_timeout);

    if (send_first_transform_ == "during") {
      send_first_transform();
    }
    if (send_second_transform_ == "during") {
      send_second_transform();
    }

    // As a buffer for the timer in the receiver, wait an extra half timeout
    std::this_thread::sleep_for(timeout_);
    if (send_first_transform_ == "after") {
      send_first_transform();
    }
    if (send_second_transform_ == "after") {
      send_second_transform();
    }
    rclcpp::shutdown(); // Just for testing
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ready_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr trigger_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_1_;
  geometry_msgs::msg::TransformStamped tf_2_;
  std::chrono::milliseconds send_buffer_time_;
  std::chrono::milliseconds timeout_;
  std::string send_first_transform_;
  std::string send_second_transform_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
