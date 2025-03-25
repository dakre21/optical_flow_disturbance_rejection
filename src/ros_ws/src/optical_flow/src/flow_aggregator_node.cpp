/*
Author: David Akre
Date: 2/28/24
Description: Aggregator ros2 node that collects all optical flows asynchronously
and transforms them to robot states for the pixhawk
*/

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "optical_flow_msgs/msg/flows.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define NUM_CAMERAS 8
#define TOPIC_NS "rpi"
#define TOPIC "optical_flow_vectors_"

namespace optical_flow {

class FlowAggregatorSubscriber : public rclcpp::Node {
 public:
  FlowAggregatorSubscriber() : Node("flow_aggregator_node") {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/velocity_observer/velocity", 10);

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
      auto topic = TOPIC_NS + std::string("1/") + TOPIC + std::to_string(i);
      if (i >= 4) {
        topic = TOPIC_NS + std::string("2/") + TOPIC + std::to_string(i - 4);
      }

      // (dakre) few ways to go about this, but simple straight forward way
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
                // \hat{x} = C^{-1} y
                Eigen::MatrixXd x_hat(6, 1);
                x_hat << 0, 0, 0, 0, 0, 0;

                geometry_msgs::msg::TwistStamped twist;
                twist.header.stamp = this->get_clock()->now();
                twist.header.frame_id = "base_link";
                twist.twist.linear.x = x_hat(3);
                twist.twist.linear.y = x_hat(4);
                twist.twist.linear.z = x_hat(5);
                twist.twist.angular.x = x_hat(0);
                twist.twist.angular.y = x_hat(1);
                twist.twist.angular.z = x_hat(2);

                twist_pub_->publish(twist);
              }));
    }
  }

 private:
  std::vector<rclcpp::Subscription<optical_flow_msgs::msg::Flows>::SharedPtr>
      flow_subs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};
}  // namespace optical_flow

int main(int argc, char* argv[]) {
  (void)argc, (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::FlowAggregatorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
