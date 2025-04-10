#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ros2_hw_pi_hat/quaro_interface.hpp"

QuaroInterface::QuaroInterface() : Node("quaro_interface") {
  joint_state_pub =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

QuaroInterface::~QuaroInterface() {}

int main(int argc, char *argv[]) { rclcpp::init(argc, argv); }
