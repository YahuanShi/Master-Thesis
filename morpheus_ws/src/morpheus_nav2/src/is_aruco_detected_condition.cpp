#include "morpheus_nav2/is_aruco_detected_condition.hpp"

namespace morpheus_nav2
{

IsArucoDetectedCondition::IsArucoDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  detected_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic;
  getInput("topic", topic);
  getInput("timeout", timeout_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&IsArucoDetectedCondition::markersCallback, this, std::placeholders::_1),
    sub_option);

  last_detection_time_ = node_->now();
}

BT::NodeStatus IsArucoDetectedCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  if (detected_) {
    auto age = (node_->now() - last_detection_time_).seconds();
    if (age <= timeout_) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

void IsArucoDetectedCondition::markersCallback(
  ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg->marker_ids.empty()) {
    detected_ = true;
    last_detection_time_ = node_->now();
    last_marker_ids_ = msg->marker_ids;
  }
}

}  // namespace morpheus_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<morpheus_nav2::IsArucoDetectedCondition>("IsArucoDetected");
}
