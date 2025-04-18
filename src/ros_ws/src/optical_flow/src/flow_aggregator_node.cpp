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
#include "optical_flow/optical_flow_constants.h"
#include "optical_flow_msgs/msg/flows.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace optical_flow {

using namespace std::chrono_literals;

class FlowAggregator : public rclcpp::Node {
 public:
  FlowAggregator() : Node("flow_aggregator_node") {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/velocity_observer/velocity", 10);

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
      auto topic = TOPIC_NS + std::string("1/") + TOPIC + std::to_string(i);
      if (i >= 4) {
        topic = TOPIC_NS + std::string("2/") + TOPIC + std::to_string(i - 4);
      }

      flow_subs_.emplace_back(
          this->create_subscription<optical_flow_msgs::msg::Flows>(
              topic, 10, [this](optical_flow_msgs::msg::Flows::SharedPtr msg) {
                assert(msg != nullptr);
                assert(msg->u.size() == msg->v.size());
                assert(msg->u.size() == NUM_POINTS);

                for (const auto& v : msg->v) v_.emplace_back(v);
                for (const auto& u : msg->u) u_.emplace_back(u);
              }));
    }

    timer_ = this->create_wall_timer(
        10ms, std::bind(&FlowAggregator::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    assert(u_.size() == v_.size());

    Eigen::MatrixXd flow_mat(2, u_.size());
    for (size_t i = 0; i < u_.size(); ++i) {
      flow_mat(0, i) = u_[i];
      flow_mat(1, i) = v_[i];
    }

    // (dakre) TODO perform projection when we have it: y = Cx -> \hat{x}
    // \hat{x} = C^{-1} y... furthermore, may need to truncate u and v to match
    // expected size of the C linear mapping
    Eigen::MatrixXd x_hat(6, 1);
    x_hat << 0, 0, 0, 0, 0, 0;

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = x_hat(0);
    twist.twist.linear.y = x_hat(1);
    twist.twist.linear.z = x_hat(2);

    // (dakre) bastardizing angular vel message with num cam inputs
    twist.twist.angular.z = u_.size() / NUM_POINTS;

    twist_pub_->publish(twist);

    u_.clear();
    v_.clear();
  }

  std::vector<float> u_, v_;

  std::vector<rclcpp::Subscription<optical_flow_msgs::msg::Flows>::SharedPtr>
      flow_subs_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace optical_flow

int main(int argc, char* argv[]) {
  (void)argc, (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::FlowAggregator>());
  rclcpp::shutdown();
  return 0;
}
