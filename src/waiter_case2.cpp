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

class Waiter : public rclcpp::Node
{
public:
  Waiter() : Node("minimal_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(cti);
    timeout_ms_ = this->declare_parameter("timeout_ms").get<int>();
    RCLCPP_INFO(get_logger(), "timeout is %dms", timeout_ms_);
    
    waitUntilTFAvailable();
  }

private:
  void waitUntilTFAvailable()
  {
    using namespace std::chrono_literals;
    double timeout_sec = timeout_ms_ * 0.001;
    while (rclcpp::ok()) {
      static constexpr auto from = "a", to = "b";
      RCLCPP_INFO(get_logger(), "waitForTransform");
      auto tf_future = tf_buffer_.waitForTransform(
        from, to, tf2::TimePointZero, tf2::durationFromSec(0.0), [](auto &) {});
      const auto status = tf_future.wait_for(tf2::durationFromSec(timeout_sec));
      if (status == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "tf gets ready. try tf_future.get()...");
        try{
          auto transform = tf_future.get();
          RCLCPP_INFO(get_logger(), "Successed to get transform. End processing.");
          break;
        } catch (const tf2::LookupException & ex)
        {
          RCLCPP_WARN(get_logger(), "catch LookupException. what:[%s]. try again.", ex.what());
        }
      } else {
        RCLCPP_INFO(
          get_logger(), "tf is not ready. waiting another %f seconds for %s->%s transform",
          timeout_sec, from, to);
        std::this_thread::sleep_for(10000ms);
      }
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  int timeout_ms_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Waiter>());
  rclcpp::shutdown();
  return 0;
}
