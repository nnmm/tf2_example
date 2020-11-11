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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
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
    } else if (method == "wft_wait_for") {
      trigger_cb = &Receiver::wft_wait_for;
    } else if (method == "wft_get") {
      trigger_cb = &Receiver::wft_get;
    } else if (method == "lt") {
      trigger_cb = &Receiver::lt;
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid value for parameter 'method'. Allowed values are 'wft_callback', "
        "'wft_wait_for', 'wft_get', 'lt'.");
    }

    auto trigger_callback = [this, trigger_cb](const std_msgs::msg::Empty::ConstSharedPtr) {
      (this->*trigger_cb)();
    };
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>("trigger", 1, trigger_callback);
    tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(cti);

    const int timeout_ms = this->declare_parameter("timeout_ms").get<int>();
    timeout_ = std::chrono::milliseconds(timeout_ms);

    const std::string executor = this->declare_parameter("executor").get<std::string>();
    if (executor == "single") {
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    } else if (executor == "multi") {
      executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid value for parameter 'executor'. Allowed values are 'single', 'multi'.");
    }

    const std::string request_time_str = this->declare_parameter("request_time").get<std::string>();
    if (request_time_str == "newest") {
      newest_ = true;
      request_time_ = tf2::TimePointZero;
    } else if (request_time_str == "current") {
      newest_ = false;
      builtin_interfaces::msg::Time msg_time;
      msg_time.sec = 102;
      msg_time.nanosec = 0;
      request_time_ = tf2_ros::fromMsg(msg_time);
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid value for parameter 'request_time'. Allowed values are 'current', 'newest'.");
    }

    callback_ = [this, logger = get_logger().get_child("callback")](
                  const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) {
      try {
        RCLCPP_INFO(logger, "Calling get() ...");
        const auto lu_time = tf2_ros::fromMsg(tf.get().header.stamp);
        if (request_time_ != lu_time && !newest_) {
          int t_wrong = tf.get().header.stamp.sec;
          RCLCPP_WARN(logger, "got wrong timestamp " + std::to_string(t_wrong));
        }
        RCLCPP_INFO(logger, "Returned from get() with transform");
      } catch (tf2::TimeoutException & e) {
        RCLCPP_INFO(logger, "Transform timed out");
      }
    };

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    ready_pub_ = this->create_publisher<std_msgs::msg::Empty>("ready", durable_qos);
    // This is racy, this node is not spinning yet, but good enough here
    ready_pub_->publish(std_msgs::msg::Empty{});
  }

private:
  void wft_callback()
  {
    RCLCPP_INFO(get_logger(), "Calling waitForTransform()");
    tf_buffer_.waitForTransform("a", "b", request_time_, timeout_, callback_);
    RCLCPP_INFO(get_logger(), "Returned from waitForTransform()");
  }

  void wft_wait_for()
  {
    RCLCPP_INFO(get_logger(), "Calling waitForTransform()");
    auto future = tf_buffer_.waitForTransform("a", "b", request_time_, timeout_, callback_);
    RCLCPP_INFO(get_logger(), "Calling future.wait_for()");
    auto status = future.wait_for(timeout_);
    if (status == std::future_status::deferred) {
      RCLCPP_INFO(get_logger(), "Status is deferred");
    } else if (status == std::future_status::timeout) {
      RCLCPP_INFO(get_logger(), "Status is timeout");
    } else {
      RCLCPP_INFO(get_logger(), "Status is ready, calling future.get()");
      const auto lu_time = tf2_ros::fromMsg(future.get().header.stamp);
      if (request_time_ != lu_time && !newest_) {
        int t_wrong = future.get().header.stamp.sec;
        RCLCPP_WARN(get_logger(), "got wrong timestamp " + std::to_string(t_wrong));
      }
    }
    RCLCPP_INFO(get_logger(), "Returned from future.get()");
  }

  void wft_get()
  {
    RCLCPP_INFO(get_logger(), "Calling waitForTransform()");
    auto future = tf_buffer_.waitForTransform("a", "b", request_time_, timeout_, callback_);
    RCLCPP_INFO(get_logger(), "Returned from waitForTransform(), calling get()");
    RCLCPP_INFO(get_logger(), "Calling future.get()");
    const auto lu_time = tf2_ros::fromMsg(future.get().header.stamp);
    if (request_time_ != lu_time && !newest_) {
      int t_wrong = future.get().header.stamp.sec;
      RCLCPP_WARN(get_logger(), "got wrong timestamp " + std::to_string(t_wrong));
    }
    RCLCPP_INFO(get_logger(), "Returned from future.get()");
  }

  void lt()
  {
    RCLCPP_INFO(get_logger(), "Calling lookupTransform()");
    tf_buffer_.lookupTransform("a", "b", request_time_, timeout_);
    RCLCPP_INFO(get_logger(), "Returned from lookupTransform()");
  }

public:
  std::unique_ptr<rclcpp::Executor> executor_;

private:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::function<void(std::shared_future<geometry_msgs::msg::TransformStamped>)> callback_;

  std::chrono::milliseconds timeout_;
  tf2::TimePoint request_time_;
  bool newest_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Receiver>();
  auto & exec = node->executor_;
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  rclcpp::shutdown();
  return 0;
}
