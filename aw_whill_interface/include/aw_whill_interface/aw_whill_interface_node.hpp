#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace sample_module
{
  using AwCmdType = autoware_auto_control_msgs::msg::AckermannControlCommand;
  // using WhillCmdType = sensor_msgs::msg::Joy;
  using Twist = geometry_msgs::msg::Twist;

class AwWhillInterface : public rclcpp::Node
{
public:
  explicit AwWhillInterface(const rclcpp::NodeOptions & node_options);

private:
  void callbackAwCmd(const AwCmdType::ConstSharedPtr & msg);


  rclcpp::Subscription<AwCmdType>::SharedPtr sub_autoware_cmd_;


  rclcpp::Publisher<Twist>::SharedPtr pub_whill_cmd_;

};

} // namespace sample_module