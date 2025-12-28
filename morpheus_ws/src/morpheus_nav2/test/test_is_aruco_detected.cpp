#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "morpheus_nav2/is_aruco_detected_condition.hpp"

class IsArucoDetectedTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(IsArucoDetectedTest, RegistersInFactory)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<morpheus_nav2::IsArucoDetectedCondition>("IsArucoDetected");

  auto registered = factory.manifests();
  ASSERT_TRUE(registered.count("IsArucoDetected") > 0);
}

TEST_F(IsArucoDetectedTest, HasExpectedPorts)
{
  auto ports = morpheus_nav2::IsArucoDetectedCondition::providedPorts();
  bool has_topic = false;
  bool has_timeout = false;
  for (auto & [name, info] : ports) {
    if (name == "topic") has_topic = true;
    if (name == "timeout") has_timeout = true;
  }
  EXPECT_TRUE(has_topic);
  EXPECT_TRUE(has_timeout);
}

TEST_F(IsArucoDetectedTest, TickReturnsFailureWithoutMarkers)
{
  auto node = std::make_shared<rclcpp::Node>("bt_test_node");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<morpheus_nav2::IsArucoDetectedCondition>("IsArucoDetected");

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <IsArucoDetected topic="/aruco_markers" timeout="1.0"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml, blackboard);
  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}
