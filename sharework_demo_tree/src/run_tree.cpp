#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <sharework_demo_tree/fixture_check.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_tree_node");

    ros::NodeHandle nh("tree_test_server");


    std::string tree_path;
    if (not nh.getParam("tree_path",tree_path))
    {
      ROS_ERROR("tree_path not defined");
      return 0;
    }

    ROS_INFO("Start running tree");

    BT::BehaviorTreeFactory factory;

    FixtureCheck("fixture");

//    factory.registerNodeType<BT::SyncActionNode>("fixture");

//    ROS_INFO("FixtureCheck registered");


//    BT::Tree tree = factory.createTreeFromFile(tree_path);

//    ROS_INFO("Tree created");

//    tree.tickRoot();

//    ROS_INFO("Tree finish");
}
