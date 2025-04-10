#ifndef __QUARO_INTERFACE_HPP__
#define __QUARO_INTERFACE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cstdint>
#include <string>
#include <vector>

class QuaroInterface : public rclcpp::Node {
public:
  QuaroInterface();
  ~QuaroInterface();

private:
  std::vector<uint8_t> joint_id;
  std::vector<uint16_t> joint_values;
  std::vector<std::string> joint_names;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
};


#endif
