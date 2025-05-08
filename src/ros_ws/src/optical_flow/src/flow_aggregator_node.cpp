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
  FlowAggregator()
      : Node("flow_aggregator_node"), type_(ProjectionScheme::CONCENTRIC) {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/velocity_observer/velocity", 10);

    num_points_ = ProjectionScheme::CONCENTRIC == type_ ? NUM_POINTS / NUM_RINGS
                                                      : NUM_POINTS;

    int pi = 1, ncam = 0;
    for (size_t i = 0; i < NUM_CAMERAS_PER_PI * NUM_PIS; ++i) {
      if (i != 0 && i % NUM_PIS == 0) {
        ncam = 0, pi++;
      }
      auto topic =
          TOPIC_NS + std::to_string(pi) + "/" + TOPIC + std::to_string(ncam++);

      flow_subs_.emplace_back(
          this->create_subscription<optical_flow_msgs::msg::Flows>(
              topic, 10, [this](optical_flow_msgs::msg::Flows::SharedPtr msg) {
                assert(msg != nullptr);
                assert(msg->u.size() == msg->v.size());
                assert(msg->u.size() == num_points_);

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

    // (dakre) TODO perform projection on flows for v
    double v = 0.0;

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.y = v;

    twist_pub_->publish(twist);

    u_.clear();
    v_.clear();
  }

  size_t num_points_;
  int type_;

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
