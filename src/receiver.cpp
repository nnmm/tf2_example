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

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Receiver : public rclcpp::Node
{
public:
  Receiver() : Node("receiver"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    typedef void (Receiver::*MemberFnPtr)(void);
    const std::string method = this->declare_parameter("method").get<std::string>();
    MemberFnPtr trigger_cb = nullptr;
    if (method == "wft_callback") {
      trigger_cb = &Receiver::wft_callback;
    } else if (method == "wft_synchronous") {
      trigger_cb = &Receiver::wft_synchronous;
    } else if (method == "lt") {
      trigger_cb = &Receiver::lt;
    } else {
      RCLCPP_FATAL(
        get_logger(), "Invalid value for parameter 'method'. Allowed values are 'wft_callback', 'wft_synchronous', 'lt'.");
    }

    auto callback = [this, trigger_cb](const std_msgs::msg::Empty::ConstSharedPtr) {
      (this->*trigger_cb)();
    };
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>("trigger", 1, callback);
    tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(cti);

    const int timeout_ms = this->declare_parameter("timeout_ms").get<int>();
    timeout_ = std::chrono::milliseconds(timeout_ms);

    request_time_.sec = 20;
    request_time_.nanosec = 0;

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready", durable_qos);
    ready_pub_->publish(std_msgs::msg::Empty{});
  }

private:
  void wft_callback()
  {
    const rclcpp::Logger logger = this->get_logger().get_child("callback");
    auto callback = [this, logger](const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) {
      try {
        RCLCPP_INFO(logger, "Calling get() ...");
        const auto lu_time = rclcpp::Time(tf.get().header.stamp);
        if (request_time_ != lu_time) {
          int t_wrong = tf.get().header.stamp.sec;
          RCLCPP_WARN(logger, "got wrong timestamp " + std::to_string(t_wrong));
        }
        RCLCPP_INFO(logger, "Returned from get() with transform");
      } catch (tf2::TimeoutException & e) {
        RCLCPP_INFO(logger, "Transform timed out");
      }
    };

    RCLCPP_INFO(get_logger(), "Calling waitForTransform()");
    tf_buffer_.waitForTransform(
      "a", "b", tf2_ros::fromMsg(request_time_), timeout_, callback);
    RCLCPP_INFO(get_logger(), "Returned from waitForTransform()");
  }

  void wft_synchronous()
  {
    RCLCPP_INFO(get_logger(), "Calling waitForTransform()");
    auto future = tf_buffer_.waitForTransform(
      "a", "b", tf2_ros::fromMsg(request_time_), timeout_, [](auto){});
    RCLCPP_INFO(get_logger(), "Returned from waitForTransform()");
    RCLCPP_INFO(get_logger(), "Calling wait_for()");
    auto status = future.wait_for(timeout_);
    if (status == std::future_status::deferred) {
      RCLCPP_INFO(get_logger(), "Status is deferred");
    } else if (status == std::future_status::timeout) {
      RCLCPP_INFO(get_logger(), "Status is timeout");
    } else {
      RCLCPP_INFO(get_logger(), "Status is ready, calling get()");
      const auto lu_time = rclcpp::Time(future.get().header.stamp);
      if (request_time_ != lu_time) {
        int t_wrong = future.get().header.stamp.sec;
        RCLCPP_WARN(get_logger(),
          "got wrong timestamp " + std::to_string(t_wrong));
      }
    }
  }

  void lt()
  {
    RCLCPP_INFO(get_logger(), "Calling lookupTransform()");
    tf_buffer_.lookupTransform(
      "a", "b", tf2_ros::fromMsg(request_time_));
    RCLCPP_INFO(get_logger(), "Returned from lookupTransform()");
  }

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::chrono::milliseconds timeout_;
  builtin_interfaces::msg::Time request_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Receiver>());
  rclcpp::shutdown();
  return 0;
}
