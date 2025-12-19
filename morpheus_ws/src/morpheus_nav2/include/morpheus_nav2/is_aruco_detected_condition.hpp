#ifndef MORPHEUS_NAV2__IS_ARUCO_DETECTED_CONDITION_HPP_
#define MORPHEUS_NAV2__IS_ARUCO_DETECTED_CONDITION_HPP_

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace morpheus_nav2
{

class IsArucoDetectedCondition : public BT::ConditionNode
{
public:
  IsArucoDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsArucoDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/aruco_markers", "ArUco markers topic"),
      BT::InputPort<double>(
        "timeout", 1.0, "Max age (seconds) of last detection to count as 'detected'"),
    };
  }

private:
  void markersCallback(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr sub_;
  std::mutex mutex_;
  rclcpp::Time last_detection_time_;
  bool detected_;
  double timeout_;
  std::vector<int64_t> last_marker_ids_;
};

}  // namespace morpheus_nav2

#endif  // MORPHEUS_NAV2__IS_ARUCO_DETECTED_CONDITION_HPP_
