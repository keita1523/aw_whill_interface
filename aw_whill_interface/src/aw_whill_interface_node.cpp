#include "aw_whill_interface/aw_whill_interface_node.hpp"

namespace sample_module // need to change
{

AwWhillInterface::AwWhillInterface(const rclcpp::NodeOptions & node_options)
: Node("aw_whill_interface", node_options)
{
  RCLCPP_INFO(this->get_logger(), "AW WHILL interface launched.");

  // get parameter
  auto & p = param_;
  std::string ns = "normal.";
  p.min_acc_ = declare_parameter(ns + "min_acc", -0.5);
  p.max_acc_ = declare_parameter(ns + "max_acc", 0.5);
  p.min_jerk_ = declare_parameter(ns + "min_jerk", -0.5);
  p.max_jerk_ = declare_parameter(ns + "max_jerk", 0.5);
  p.max_velocity_ = declare_parameter("max_velocity", 1.6667);

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
}

// void limitAwCmd(float velocity, float angle)
// {
//   return;
// }

double AwWhillInterface::limitAcceleration(
  double prev_velocity,
  double current_velocity,
  double time,
  double min_acc,
  double max_acc
)
{
  if (time == 0){ return 0.0; }
  double current_acc = (current_velocity - prev_velocity) / time;
  RCLCPP_INFO(this->get_logger(),"accel : %f", current_acc);
  if (current_acc < min_acc) {
    return prev_velocity * min_acc * time;
  } else if (current_acc > max_acc) {
    return prev_velocity * max_acc * time;
  } else {
    return current_velocity;
  }
}

void AwWhillInterface::callbackAwCmd(const AwCmdType::ConstSharedPtr & msg)
{
  Twist output_cmd;
  // output_cmd.linear.x = msg->longitudinal.speed;
  // output_cmd.angular.z = msg->lateral.steering_tire_angle;
  // static prev_velocity = velocity;
  // isInAccLimit(prev_velocity, velocity, param_.acc_min_, param_.acc_max_);
  double velocity = msg->longitudinal.speed;
  rclcpp::Time time(msg->stamp);
  double current_time = time.seconds() + (time.nanoseconds() / 1e9);
  double angle = msg->lateral.steering_tire_angle;
  static double prev_velocity = velocity;
  static double prev_time = current_time;


  RCLCPP_INFO(this->get_logger(),"AwCmd     velocity:%f, angle:%f", velocity, angle);
  auto safe_velocity = limitAcceleration(prev_velocity, velocity, current_time - prev_time, param_.min_acc_, param_.max_acc_);
  output_cmd.linear.x = safe_velocity;
  output_cmd.angular.z = angle;

  pub_whill_cmd_->publish(output_cmd);

  return;
}

void AwWhillInterface::callbackWhillOdom(const Odometry::ConstSharedPtr& msg)
{
  AwSteering steer_angle;
  AwVelocity velocity;
  // double mps_twist = msg->twist.twist.linear.x / 3.6;
  // double rps_twist = msg->twist.twist.angular.z / 3.6;
  double linear = msg->twist.twist.linear.x;
  double angular = msg->twist.twist.angular.z;
  // RCLCPP_INFO(this->get_logger(),"pose  x:%f, y:%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  RCLCPP_INFO(this->get_logger(),"WhillOdom velocity:%f, angle:%f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  if(linear <= 0.001)
  {
    velocity.longitudinal_velocity = 0;
    steer_angle.steering_tire_angle = 0;
  }else{
    // velocity.longitudinal_velocity = mps_twist;
    // // 速度・角速度から4輪のときの速度・ステア角を求める
    // steer_angle.steering_tire_angle = std::atan(rps_twist * 0.6 / mps_twist);
    velocity.longitudinal_velocity = linear;
    steer_angle.steering_tire_angle = angular;
  }
  steer_angle.stamp = Node::now();
  velocity.header.stamp = Node::now();
  velocity.header.frame_id = "base_link";
  // steer_angle.stamp = msg->header.stamp;
  // velocity.header = msg->header;
  // RCLCPP_INFO(this->get_logger(),"velocity:%f, angle:%f", velocity.longitudinal_velocity, steer_angle.steering_tire_angle);
  pub_steering_status_->publish(steer_angle);
  pub_velocity_status_->publish(velocity);
  return;
}

} // namespace sample_module

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sample_module::AwWhillInterface)