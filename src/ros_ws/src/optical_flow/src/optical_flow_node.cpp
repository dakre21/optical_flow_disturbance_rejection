/*
Author: David Akre
Date: 2/26/24
Description: Minimal publisher which reads flows from shared memory and
publishes them
*/

#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "optical_flow/optical_flow_constants.h"
#include "optical_flow_msgs/msg/flows.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace bip = boost::interprocess;

namespace optical_flow {
class OpticalFlowPublisher : public rclcpp::Node {
 public:
  OpticalFlowPublisher()
      : Node("optical_flow_publisher"), type_(ProjectionScheme::CONCENTRIC) {
    const std::string pub_name = "optical_flow_vectors_";
    for (int i = 0; i < NUM_CAMERAS_PER_PI; ++i) {
      const auto shm_name = pub_name + std::to_string(i);
      flow_pubs_.emplace_back(
          this->create_publisher<optical_flow_msgs::msg::Flows>(shm_name, 10));

      shms_.emplace_back(std::make_unique<bip::shared_memory_object>(
          bip::open_only, shm_name.c_str(), bip::read_only));

      regions_.emplace_back(
          std::make_unique<bip::mapped_region>(*shms_[i], bip::read_only));

      const auto sem_name_reader =
          std::string("ReaderSemaphore") + std::to_string(i);
      const auto sem_name_writer =
          std::string("WriterSemaphore") + std::to_string(i);

      reader_sems_.emplace_back(std::make_unique<bip::named_semaphore>(
          bip::open_only, sem_name_reader.c_str()));
      writer_sems_.emplace_back(std::make_unique<bip::named_semaphore>(
          bip::open_only, sem_name_writer.c_str()));
    }

    num_points_ = ProjectionScheme::CONCENTRIC == type_ ? NUM_POINTS / NUM_RINGS
                                                        : NUM_POINTS;

    timer_ = this->create_wall_timer(
        5ms, std::bind(&OpticalFlowPublisher::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    assert(flow_pubs_.size() == NUM_CAMERAS_PER_PI);
    assert(regions_.size() == NUM_CAMERAS_PER_PI);

    for (size_t i = 0; i < flow_pubs_.size(); ++i) {
      const auto& pub = flow_pubs_[i];
      auto& reader_sem = reader_sems_[i];
      auto& writer_sem = writer_sems_[i];

      optical_flow_msgs::msg::Flows msg;
      msg.stamp = this->get_clock()->now();
      msg.frame_id = std::string("camera_") + std::to_string(i);
      msg.u.resize(num_points_, 0.0f);
      msg.v.resize(num_points_, 0.0f);

      if (reader_sem->try_wait()) {
        const size_t num_bytes = num_points_ * sizeof(float);
        std::memcpy(msg.u.data(), regions_[i]->get_address(), num_bytes);
        std::memcpy(
            msg.v.data(),
            static_cast<std::byte*>(regions_[i]->get_address()) + num_bytes,
            num_bytes);

        pub->publish(msg);

        writer_sem->post();
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Writer %d is busy skipping frame",
                     (int)i);
      }
    }
  }
  size_t num_points_;
  int type_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<optical_flow_msgs::msg::Flows>::SharedPtr>
      flow_pubs_;
  std::vector<std::unique_ptr<bip::shared_memory_object>> shms_;
  std::vector<std::unique_ptr<bip::mapped_region>> regions_;
  std::vector<std::unique_ptr<bip::named_semaphore>> reader_sems_;
  std::vector<std::unique_ptr<bip::named_semaphore>> writer_sems_;
};
}  // namespace optical_flow

int main(int argc, char** argv) {
  (void)argc, (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::OpticalFlowPublisher>());
  rclcpp::shutdown();
  return 0;
}
