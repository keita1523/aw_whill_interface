#include "aw_whill_interface/aw_whill_interface_node.hpp"

namespace sample_module // need to change
{

AwWhillInterface::AwWhillInterface(const rclcpp::NodeOptions & node_options)
: Node("aw_whill_interface", node_options)
{
  RCLCPP_INFO(this->get_logger(), "AW WHILL interface launched.");
}


} // namespace sample_module

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sample_module::AwWhillInterface)