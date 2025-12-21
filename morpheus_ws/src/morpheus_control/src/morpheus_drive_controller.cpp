#include "morpheus_control/morpheus_drive_controller.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace morpheus_control
{

controller_interface::CallbackReturn MorpheusDriveController::on_init()
{
  auto_declare<double>("wheel_base", 1.072);
  auto_declare<double>("wheel_radius", 0.125);
  auto_declare<double>("wheel_separation", 0.615);
  auto_declare<double>("wheel_steering_y_offset", 0.0);
  auto_declare<double>("drive_gain", 10.0);
  auto_declare<double>("deadzone", 0.05);

  auto_declare<std::vector<std::string>>("steering_joints",
    {"steering_fl_joint", "steering_fr_joint", "steering_rl_joint", "steering_rr_joint"});
  auto_declare<std::vector<std::string>>("wheel_joints",
    {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"});

  auto_declare<bool>("publish_odom", true);
  auto_declare<bool>("publish_odom_tf", true);
  auto_declare<std::string>("odom_frame", "odom");
  auto_declare<std::string>("base_frame", "chassis_link");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MorpheusDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  params_.wheel_base = get_node()->get_parameter("wheel_base").as_double();
  params_.wheel_radius = get_node()->get_parameter("wheel_radius").as_double();
  params_.wheel_separation = get_node()->get_parameter("wheel_separation").as_double();
  params_.wheel_steering_y_offset = get_node()->get_parameter("wheel_steering_y_offset").as_double();
  params_.drive_gain = get_node()->get_parameter("drive_gain").as_double();
  params_.deadzone = get_node()->get_parameter("deadzone").as_double();

  steering_joint_names_ = get_node()->get_parameter("steering_joints").as_string_array();
  wheel_joint_names_ = get_node()->get_parameter("wheel_joints").as_string_array();

  if (steering_joint_names_.size() != 4 || wheel_joint_names_.size() != 4) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exactly 4 steering and 4 wheel joints required");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_buffer_.writeFromNonRT(*msg);
    });

  auto odom_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "/wheel_odom", rclcpp::SystemDefaultsQoS());
  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    odom_publisher);

  auto tf_publisher = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::SystemDefaultsQoS());
  tf_pub_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
    tf_publisher);

  // Dynamic reconfigure
  param_callback_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        if (p.get_name() == "wheel_base") { params_.wheel_base = p.as_double(); }
        else if (p.get_name() == "wheel_radius") { params_.wheel_radius = p.as_double(); }
        else if (p.get_name() == "wheel_separation") { params_.wheel_separation = p.as_double(); }
        else if (p.get_name() == "wheel_steering_y_offset") { params_.wheel_steering_y_offset = p.as_double(); }
        else if (p.get_name() == "drive_gain") { params_.drive_gain = p.as_double(); }
        else if (p.get_name() == "deadzone") { params_.deadzone = p.as_double(); }
      }
      RCLCPP_INFO(get_node()->get_logger(), "Parameters updated");
      return result;
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MorpheusDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : steering_joint_names_) {
    conf.names.push_back(name + "/position");
  }
  for (const auto & name : wheel_joint_names_) {
    conf.names.push_back(name + "/velocity");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MorpheusDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : steering_joint_names_) {
    conf.names.push_back(name + "/position");
  }
  for (const auto & name : wheel_joint_names_) {
    conf.names.push_back(name + "/position");
    conf.names.push_back(name + "/velocity");
  }
  return conf;
}

controller_interface::CallbackReturn MorpheusDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  odom_x_ = odom_y_ = odom_yaw_ = 0.0;
  odom_initialized_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MorpheusDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Zero all commands
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(0.0);      // steering
    command_interfaces_[i + 4].set_value(0.0);  // wheel vel
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MorpheusDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto cmd = *cmd_vel_buffer_.readFromRT();
  double vx = cmd.linear.x;
  double vy = cmd.linear.y;
  double wz = cmd.angular.z;

  if (std::abs(vx) < params_.deadzone) { vx = 0.0; }
  if (std::abs(vy) < params_.deadzone) { vy = 0.0; }
  if (std::abs(wz) < params_.deadzone) { wz = 0.0; }

  double pos[4] = {0.0, 0.0, 0.0, 0.0};
  double vel[4] = {0.0, 0.0, 0.0, 0.0};

  if (wz != 0.0) {
    if (vx != 0.0) {
      compute_ackermann(vx, wz, pos, vel);
    } else {
      compute_pivot(wz, pos, vel);
    }
  } else {
    compute_crab(vx, vy, pos, vel);
  }

  // Write to hardware interfaces
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(pos[i]);
    command_interfaces_[i + 4].set_value(vel[i]);
  }

  // Update and publish odometry
  update_odometry(period);

  if (odom_pub_->trylock()) {
    auto & odom_msg = odom_pub_->msg_;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = get_node()->get_parameter("odom_frame").as_string();
    odom_msg.child_frame_id = get_node()->get_parameter("base_frame").as_string();
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = wz;
    odom_pub_->unlockAndPublish();
  }

  if (get_node()->get_parameter("publish_odom_tf").as_bool() && tf_pub_->trylock()) {
    auto & tf_msg = tf_pub_->msg_;
    tf_msg.transforms.resize(1);
    auto & t = tf_msg.transforms[0];
    t.header.stamp = time;
    t.header.frame_id = get_node()->get_parameter("odom_frame").as_string();
    t.child_frame_id = get_node()->get_parameter("base_frame").as_string();
    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_yaw_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

void MorpheusDriveController::compute_ackermann(
  double vx, double wz, double pos[4], double vel[4]) const
{
  const double g = params_.drive_gain;
  const double st = params_.steering_track();
  const double wb = params_.wheel_base;

  if (std::abs(wz) >= 1e-5) {
    double r = std::abs(vx) / wz * 2.0 * M_PI;
    double r_bl = r + st / 2.0;
    double r_br = r - st / 2.0;

    double a_fl = std::atan(wb / r_bl);
    double a_fr = std::atan(wb / r_br);

    if (r_bl > 0.0 && r < 0.0) { a_fl -= M_PI; }
    if (r_br < 0.0 && r > 0.0) { a_fr += M_PI; }

    pos[0] = a_fl * 1.57;
    pos[1] = a_fr * 1.57;
  }

  double vel_offset = wz * params_.wheel_steering_y_offset;
  double sign = (vx != 0.0) ? ((vx > 0.0) ? 1.0 : -1.0) : 1.0;

  vel[0] = sign * std::hypot(vx - wz * st / 2.0, wz * wb / 2.0) - vel_offset;
  vel[1] = sign * std::hypot(vx + wz * st / 2.0, wz * wb / 2.0) + vel_offset;
  vel[2] = vel[0];
  vel[3] = vel[1];
  for (int i = 0; i < 4; ++i) { vel[i] *= g; }
}

void MorpheusDriveController::compute_pivot(
  double wz, double pos[4], double vel[4]) const
{
  const double g = params_.drive_gain;
  double ang = std::atan(params_.wheel_base / params_.steering_track());
  pos[0] = -ang;
  pos[1] = ang;
  pos[2] = ang;
  pos[3] = -ang;
  vel[0] = -wz * g;
  vel[1] = wz * g;
  vel[2] = -wz * g;
  vel[3] = wz * g;
}

void MorpheusDriveController::compute_crab(
  double vx, double vy, double pos[4], double vel[4]) const
{
  const double g = params_.drive_gain;

  if (vx > 0.0 || vy != 0.0) {
    double angle = (vx != 0.0 || vy != 0.0) ? std::atan2(vy, vx) : 0.0;
    if (std::abs(angle) >= M_PI / 2.0) {
      angle = -((angle > 0.0) ? 1.0 : -1.0) * (M_PI - std::abs(angle));
    }
    for (int i = 0; i < 4; ++i) { pos[i] = angle; }
  }

  double mag = std::hypot(vx, vy);
  double sign = (vx > 0.0) ? 1.0 : ((vx < 0.0) ? -1.0 : 1.0);
  for (int i = 0; i < 4; ++i) { vel[i] = mag * sign * g; }
}

void MorpheusDriveController::update_odometry(const rclcpp::Duration & /*period*/)
{
  // State interfaces layout: 4 steering pos, then 4*(pos+vel)
  // steering: state_interfaces_[0..3]
  // wheel pos: state_interfaces_[4], [6], [8], [10]
  // wheel vel: state_interfaces_[5], [7], [9], [11]
  double wheel_pos[4], steer_pos[4];
  for (size_t i = 0; i < 4; ++i) {
    steer_pos[i] = state_interfaces_[i].get_value();
    wheel_pos[i] = state_interfaces_[4 + i * 2].get_value();
  }

  if (!odom_initialized_) {
    for (size_t i = 0; i < 4; ++i) { prev_wheel_pos_[i] = wheel_pos[i]; }
    odom_initialized_ = true;
    return;
  }

  // Average wheel displacement
  double d_left = 0.0, d_right = 0.0;
  for (size_t i : {0u, 2u}) {
    d_left += (wheel_pos[i] - prev_wheel_pos_[i]) * params_.wheel_radius;
  }
  for (size_t i : {1u, 3u}) {
    d_right += (wheel_pos[i] - prev_wheel_pos_[i]) * params_.wheel_radius;
  }
  d_left /= 2.0;
  d_right /= 2.0;

  double d_center = (d_left + d_right) / 2.0;
  double d_theta = (d_right - d_left) / params_.steering_track();

  // Average steering angle for crab motion
  double avg_steer = 0.0;
  for (int i = 0; i < 4; ++i) { avg_steer += steer_pos[i]; }
  avg_steer /= 4.0;

  odom_x_ += d_center * std::cos(odom_yaw_ + avg_steer);
  odom_y_ += d_center * std::sin(odom_yaw_ + avg_steer);
  odom_yaw_ += d_theta;

  for (size_t i = 0; i < 4; ++i) { prev_wheel_pos_[i] = wheel_pos[i]; }
}

}  // namespace morpheus_control

PLUGINLIB_EXPORT_CLASS(
  morpheus_control::MorpheusDriveController,
  controller_interface::ControllerInterface)
