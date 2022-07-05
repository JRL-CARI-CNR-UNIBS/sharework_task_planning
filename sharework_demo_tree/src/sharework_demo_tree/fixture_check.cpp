
#include "sharework_demo_tree/fixture_check.h"


FixtureCheck::FixtureCheck(const std::string &name) : BT::SyncActionNode(name, {})
{
  if(!FixtureCheck::getInput<std::string>("fixture",fixture_name_))
  {
    ROS_ERROR("Missing required input [fixture]");
    throw BT::RuntimeError("Missing required input [fixture]");
  }
  //   if(!FixtureCheck::getInput<std::string>("fixture_base_topic_name_",fixture_base_topic_name_))
  //   {
  //     ROS_ERROR("Missing required input [fixture_base_topic_name_]");
  //     throw BT::RuntimeError("Missing required input [fixture_base_topic_name_]");
  //   }

  ROS_INFO_STREAM("Fixture topic name: " << "fixture_base_topic_name_"+fixture_name_);
  fixture_state_sub_.reset(new ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture> (nh_, "fixture_base_topic_name_"+fixture_name_,10));

}

BT::NodeStatus FixtureCheck::tick()
{
  return BT::NodeStatus::SUCCESS;
}

FixtureCheck::~FixtureCheck()
{
}
