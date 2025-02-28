/*
Author: David Akre
Date: 2/28/24
Description: Aggregator ros2 node that collects all optical flows asynchronously
and transforms them to robot states for the pixhawk
*/

#include <libserial/SerialStream.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "optical_flow_msgs/msg/flows.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define NUM_CAMERAS 8
#define TOPIC_NS "rpi"
#define TOPIC "optical_flow_vectors_"
#define SERIAL_PORT "/dev/ttyAMA0"

namespace optical_flow {

class FlowAggregatorSubscriber : public rclcpp::Node {
 public:
  FlowAggregatorSubscriber() : Node("flow_aggregator_node") {
    serial_port_ = std::make_unique<LibSerial::SerialStream>();

    serial_port_->Open(SERIAL_PORT);
    assert(serial_port_->IsOpen());

    serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
      auto topic = TOPIC_NS + std::string("1/") + TOPIC + std::to_string(i);
      if (i >= 4) {
        topic = TOPIC_NS + std::string("2/") + TOPIC + std::to_string(i - 4);
      }

      // (dakre) TODO few ways to go about this, but simple straight forward way
      // is to simply transform the flows from each camera to robot states and
      // send that to the pixhawk. So that will be the approach right now.
      flow_subs_.emplace_back(
          this->create_subscription<optical_flow_msgs::msg::Flows>(
              topic, 10, [this](optical_flow_msgs::msg::Flows::SharedPtr msg) {
                assert(msg != nullptr);
                assert(msg->u.size() == msg->v.size());

                // RCLCPP_DEBUG(this->get_logger(), "Message recv from %s",
                //              msg->frame_id.c_str());

                Eigen::MatrixXd flow_mat(2, msg->u.size());
                for (size_t i = 0; i < msg->u.size(); ++i) {
                  flow_mat(0, i) = msg->u[i];
                  flow_mat(1, i) = msg->v[i];
                }

                // (dakre) TODO fill this out when we have it: y = Cx -> \hat{x}
                // = C^{-1} y
                Eigen::MatrixXd x_hat(6, 1);
                x_hat << 0, 0, 0, 0, 0, 0;

                std::stringstream ss;
                ss << "[";
                for (auto i = 0l; i < x_hat.rows(); ++i) {
                  ss << x_hat(i, 0);
                  if (i <= x_hat.rows() - 1) {
                    ss << ",";
                  }
                }
                ss << "]";

                *serial_port_ << ss.str();
                serial_port_->flush();
              }));
    }
  }

 private:
  std::vector<rclcpp::Subscription<optical_flow_msgs::msg::Flows>::SharedPtr>
      flow_subs_;
  std::unique_ptr<LibSerial::SerialStream> serial_port_;
};
}  // namespace optical_flow

int main(int argc, char* argv[]) {
  (void)argc, (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::FlowAggregatorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
