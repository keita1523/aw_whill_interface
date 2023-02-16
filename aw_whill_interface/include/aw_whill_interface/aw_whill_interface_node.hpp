#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>    // abs() for float, and fabs()
#include <unistd.h>

namespace sample_module
{
  using AwCmdType = autoware_auto_control_msgs::msg::AckermannControlCommand;
  // using WhillCmdType = sensor_msgs::msg::Joy;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using AwSteering = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using AwVelocity = autoware_auto_vehicle_msgs::msg::VelocityReport;

class AwWhillInterface : public rclcpp::Node
{
public:
  explicit AwWhillInterface(const rclcpp::NodeOptions & node_options);

private:
  void callbackAwCmd(const AwCmdType::ConstSharedPtr & msg);
  void callbackWhillOdom(const Odometry::ConstSharedPtr & msg);
  bool limitAcceleration(
    double prev_velocity,
    double current_velocity,
    double time,
    double min_acc,
    double max_acc
  );
  struct Param {
    double max_velocity_;

    double min_acc_;
    double max_acc_;
    double min_jerk_;
    double max_jerk_;
  };

  Param param_;
  // double prev_velocity;


  rclcpp::Subscription<AwCmdType>::SharedPtr sub_autoware_cmd_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_whill_odom_;


  rclcpp::Publisher<Twist>::SharedPtr pub_whill_cmd_;
  rclcpp::Publisher<AwVelocity>::SharedPtr pub_velocity_status_;
  rclcpp::Publisher<AwSteering>::SharedPtr pub_steering_status_;

};

} // namespace sample_module