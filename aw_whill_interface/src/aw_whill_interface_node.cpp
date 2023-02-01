#include "aw_whill_interface/aw_whill_interface_node.hpp"

namespace sample_module // need to change
{

AwWhillInterface::AwWhillInterface(const rclcpp::NodeOptions & node_options)
: Node("aw_whill_interface", node_options)
{
  RCLCPP_INFO(this->get_logger(), "AW WHILL interface launched.");

  // set Subscriber
  sub_autoware_cmd_ = this->create_subscription<AwCmdType>(
    "~/input/autoware_command",
    rclcpp::QoS(1),
    std::bind(&AwWhillInterface::callbackAwCmd, this, std::placeholders::_1)
  );

  sub_whill_odom_ = this->create_subscription<Odometry>(
    "~/input/whill_odom",
    rclcpp::QoS(1),
    std::bind(&AwWhillInterface::callbackWhillOdom, this, std::placeholders::_1)
  );

  // set Publisher
  pub_whill_cmd_ = this->create_publisher<Twist>("~/output/whill_command", rclcpp::QoS(1));

  pub_velocity_status_ = this->create_publisher<AwVelocity>("~/output/aw_velocity", 1);
  AwVelocity set_vel;
  set_vel.longitudinal_velocity = 0.0;
  set_vel.lateral_velocity = 0.0;
  set_vel.heading_rate = 0.0;
  set_vel.header.frame_id = "base_link";
  set_vel.header.stamp = Node::now();
  pub_velocity_status_->publish(set_vel);


  pub_steering_status_ = this->create_publisher<AwSteering>("~/output/aw_steering", 1);
  AwSteering set_steer;
  set_steer.steering_tire_angle = 0.0;
  set_steer.stamp = Node::now();
  pub_steering_status_->publish(set_steer);
  // usleep(100 * 1000);
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

void AwWhillInterface::callbackWhillOdom(const Odometry::ConstSharedPtr& msg)
{
  AwSteering steer_angle;
  AwVelocity velocity;
  double mps_twist = msg->twist.twist.linear.x / 3.6;
  // double rps_twist = msg->twist.twist.angular.z / 3.6;
  RCLCPP_INFO(this->get_logger(),"pose  x:%f, y:%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  RCLCPP_INFO(this->get_logger(),"twist x:%f, z:%f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  if(mps_twist == 0)
  {
    velocity.longitudinal_velocity = 0;
    steer_angle.steering_tire_angle = 0;
  }else{
    // velocity.longitudinal_velocity = mps_twist;
    // // 速度・角速度から4輪のときの速度・ステア角を求める
    // steer_angle.steering_tire_angle = std::atan(rps_twist * 0.6 / mps_twist);
    velocity.longitudinal_velocity = 0;
    steer_angle.steering_tire_angle = 0;
  }
  steer_angle.stamp = Node::now();
  velocity.header.stamp = Node::now();
  velocity.header.frame_id = "base_link";
  // steer_angle.stamp = msg->header.stamp;
  // velocity.header = msg->header;
  RCLCPP_INFO(this->get_logger(),"velocity:%f, angle:%f", velocity.longitudinal_velocity, steer_angle.steering_tire_angle);
  pub_steering_status_->publish(steer_angle);
  pub_velocity_status_->publish(velocity);
  return;
}

} // namespace sample_module

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sample_module::AwWhillInterface)