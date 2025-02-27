/*
Author: David Akre
Date: 2/26/24
Description: Minimal publisher which reads flows from shared memory and
publishes them
*/

#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "optical_flow_msgs/msg/flows.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace bip = boost::interprocess;

namespace optical_flow {
class OpticalFlowPublisher : public rclcpp::Node {
 public:
  OpticalFlowPublisher() : Node("optical_flow_publisher") {
    const std::string pub_name = "optical_flow_vectors_";
    for (int i = 0; i < 4; ++i) {
      const auto shm_name = pub_name + std::to_string(i);
      flow_pubs_.emplace_back(
          this->create_publisher<optical_flow_msgs::msg::Flows>(shm_name, 10));

      shm_.emplace_back(std::make_unique<bip::shared_memory_object>(
          bip::open_only, shm_name.c_str(), bip::read_only));
    }
    timer_ = this->create_wall_timer(
        10ms, std::bind(&OpticalFlowPublisher::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    optical_flow_msgs::msg::Flows msg;

    for (auto& pub : flow_pubs_) {
      pub->publish(msg);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<optical_flow_msgs::msg::Flows>::SharedPtr>
      flow_pubs_;
  std::vector<std::unique_ptr<bip::shared_memory_object>> shm_;
  std::vector<std::unique_ptr<bip::mapped_region>> region_;
};
}  // namespace optical_flow

int main(int argc, char** argv) {
  (void)argc, (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::OpticalFlowPublisher>());
  rclcpp::shutdown();
  return 0;
}
