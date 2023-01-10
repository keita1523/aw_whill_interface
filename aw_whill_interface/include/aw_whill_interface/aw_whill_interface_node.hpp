# include <rclcpp/rclcpp.hpp>

namespace sample_module
{

class AwWhillInterface : public rclcpp::Node
{
public:
  explicit AwWhillInterface(const rclcpp::NodeOptions & node_options);

};

} // namespace sample_module