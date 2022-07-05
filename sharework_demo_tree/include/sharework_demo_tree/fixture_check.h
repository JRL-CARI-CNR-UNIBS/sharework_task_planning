#ifndef FIXTURE_CHECK_H
#define FIXTURE_CHECK_H

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <mqtt_scene_integration/Fixture.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

class FixtureCheck : public BT::SyncActionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string fixture_name_;
  std::shared_ptr<ros_helper::SubscriptionNotifier <mqtt_scene_integration::Fixture>> fixture_state_sub_;

public:
  FixtureCheck(const std::string &name);
  virtual ~FixtureCheck() override;

  virtual BT::NodeStatus tick() override;

//  static BT::PortsList providedPorts()
//  {
//    return{BT::InputPort<std::string>("fixture")};
//  }


};
//class FixtureCheck : public BT::SyncActionNode
//{
//public:
//    FixtureCheck(const std::string& name);

//    BT::NodeStatus tick() override;

//private:
//    ros::NodeHandle n_;
//    ros::ServiceClient skill_exec_clnt_;
//};

#endif // FIXTURE_CHECK_H
