#include "aw_whill_interface/aw_whill_interface_node.hpp"

namespace sample_module // need to change
{

AwWhillInterface::AwWhillInterface(const rclcpp::NodeOptions & node_options)
: Node("aw_whill_interface", node_options)
{
  RCLCPP_INFO(this->get_logger(), "AW WHILL interface launched.");

  sub_autoware_cmd_ = this->create_subscription<AwCmdType>(
    "~/input/autoware_command",
    rclcpp::QoS(1),
    std::bind(&AwWhillInterface::callbackAwCmd, this, std::placeholders::_1)
  );

  pub_whill_cmd_ = this->create_publisher<Twist>("~/output/whill_command", rclcpp::QoS(1));

}

void AwWhillInterface::callbackAwCmd(const AwCmdType::ConstSharedPtr & msg)
{
  Twist output_cmd;
  // output_cmd.axes.push_back(msg->lateral.steering_tire_angle);
  // output_cmd.axes.push_back(msg->longitudinal.speed);
  output_cmd.linear.x = msg->longitudinal.speed;
  output_cmd.angular.z = msg->lateral.steering_tire_angle;
  pub_whill_cmd_->publish(output_cmd);

  return;
}

} // namespace sample_module

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sample_module::AwWhillInterface)