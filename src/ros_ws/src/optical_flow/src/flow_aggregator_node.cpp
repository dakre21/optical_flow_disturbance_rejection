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

// (dakre) TODO make these configurable
#define NUM_CAMERAS 8
#define FEATURE_POINTS 100
#define TOPIC_NS "rpi"
#define TOPIC "optical_flow_vectors_"

namespace optical_flow {

using namespace std::chrono_literals;

class FlowAggregator : public rclcpp::Node {
 public:
  FlowAggregator() : Node("flow_aggregator_node") {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/velocity_observer/velocity", 10);

    flow_mat_.resize(2, FEATURE_POINTS * NUM_CAMERAS);

    for (size_t i = 0; i < NUM_CAMERAS; ++i) {
      auto topic = TOPIC_NS + std::string("1/") + TOPIC + std::to_string(i);
      if (i >= 4) {
        topic = TOPIC_NS + std::string("2/") + TOPIC + std::to_string(i - 4);
      }

      flow_subs_.emplace_back(
          this->create_subscription<optical_flow_msgs::msg::Flows>(
              topic, 10, [&, this](optical_flow_msgs::msg::Flows::SharedPtr msg) {
                assert(msg != nullptr);
                assert(msg->u.size() == msg->v.size() &&
                       msg->v.size() == FEATURE_POINTS);

                const int offset = i * FEATURE_POINTS;
                for (int j = 0; j < FEATURE_POINTS; ++j) {
                  flow_mat_(0, j + offset) = msg->u[j];
                  flow_mat_(1, j + offset) = msg->v[j];
                }
              }));
    }

    timer_ = this->create_wall_timer(
        10ms, std::bind(&FlowAggregator::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    // (dakre) TODO perform projection when we have it: y = Cx -> \hat{x}
    // \hat{x} = C^{-1} y... furthermore, may need to truncate u and v to
    // match expected size of the C linear mapping
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

    flow_mat_.setZero();
  }

  Eigen::MatrixXd flow_mat_;

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
