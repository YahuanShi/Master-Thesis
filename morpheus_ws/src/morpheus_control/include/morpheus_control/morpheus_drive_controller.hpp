#ifndef MORPHEUS_CONTROL__MORPHEUS_DRIVE_CONTROLLER_HPP_
#define MORPHEUS_CONTROL__MORPHEUS_DRIVE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace morpheus_control
{

class MorpheusDriveController : public controller_interface::ControllerInterface
{
public:
  MorpheusDriveController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct ChassisParams
  {
    double wheel_base{1.072};
    double wheel_radius{0.125};
    double wheel_separation{0.615};
    double wheel_steering_y_offset{0.0};
    double drive_gain{10.0};
    double deadzone{0.05};
    double steering_track() const {return wheel_separation - 2.0 * wheel_steering_y_offset;}
  };

  void compute_ackermann(double vx, double wz, double pos[4], double vel[4]) const;
  void compute_pivot(double wz, double pos[4], double vel[4]) const;
  void compute_crab(double vx, double vy, double pos[4], double vel[4]) const;
  void update_odometry(const rclcpp::Duration & period);

  ChassisParams params_;

  // Joint names: FL, FR, RL, RR
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_pub_;

  // Odometry state
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  // Previous wheel positions for odometry integration
  double prev_wheel_pos_[4]{0.0, 0.0, 0.0, 0.0};
  bool odom_initialized_{false};

  // Dynamic reconfigure callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace morpheus_control

#endif  // MORPHEUS_CONTROL__MORPHEUS_DRIVE_CONTROLLER_HPP_
