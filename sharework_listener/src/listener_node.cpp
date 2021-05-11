#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
//#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <manipulation_msgs/PickObjectsActionResult.h>
#include <manipulation_msgs/PlaceObjectsActionResult.h>
#include <manipulation_msgs/GoToActionResult.h>

void cycleTimeCallback(const std_msgs::Float64ConstPtr& msg){}

void scalingCallback(const std_msgs::Float64ConstPtr& msg){}

void pickOutcomeCallback(const manipulation_msgs::PickObjectsActionResultConstPtr& msg){}
void placeOutcomeCallback(const manipulation_msgs::PlaceObjectsActionResultConstPtr& msg){}
void gotoOutcomeCallback(const manipulation_msgs::GoToActionResultConstPtr& msg){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sharework_listener_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string human_cycle_topic = "/human_task_interface/cycle_time";
  ros_helper::SubscriptionNotifier<std_msgs::Float64> human_cycle_notif(nh,human_cycle_topic,1);
  human_cycle_notif.setAdvancedCallback(&cycleTimeCallback);

  std::string robot_cycle_topic = "/robot_task_interface/cycle_time";
  ros_helper::SubscriptionNotifier<std_msgs::Float64> robot_cycle_notif(nh,robot_cycle_topic,1);
  robot_cycle_notif.setAdvancedCallback(&cycleTimeCallback);

  std::string scaling_topic = "/planner_hw/ur5_task_joint_ssm/scaling";
  ros_helper::SubscriptionNotifier<std_msgs::Float64> scaling_notif(nh,scaling_topic,1);
  scaling_notif.setAdvancedCallback(&scalingCallback);

  std::string pick_result_topic = "/ur5_on_guide/pick/result";
  ros_helper::SubscriptionNotifier<manipulation_msgs::PickObjectsActionResult> pick_notif(nh,pick_result_topic,1);
  pick_notif.setAdvancedCallback(&pickOutcomeCallback);

  std::string place_result_topic = "/ur5_on_guide/place/result";
  ros_helper::SubscriptionNotifier<manipulation_msgs::PlaceObjectsActionResult> place_notif(nh,place_result_topic,1);
  place_notif.setAdvancedCallback(&placeOutcomeCallback);

  std::string goto_result_topic = "/ur5_on_guide/goto/result";
  ros_helper::SubscriptionNotifier<manipulation_msgs::GoToActionResult> goto_notif(nh,goto_result_topic,1);
  goto_notif.setAdvancedCallback(&gotoOutcomeCallback);

  ros::Rate loop_rate(100);

  int failure_counter=0;
  int stop_counter=0;
  double stop_threshold=0.1;
  bool stop_flag=false;
  ros::Time stop_begin = ros::Time::now();
  ros::Duration stopping_time(0.0);
  while (ros::ok())
  {
    ros::spinOnce();

    /* Counting failures */
    if (pick_notif.isANewDataAvailable()){
      if (pick_notif.getData().result.result<0){
        failure_counter++;
        ROS_ERROR("new failure. Counter = %d", failure_counter);
      }
    }
    if (place_notif.isANewDataAvailable()){
      if (place_notif.getData().result.result<0){
        failure_counter++;
        ROS_ERROR("new failure. Counter = %d", failure_counter);
      }
    }
    if (goto_notif.isANewDataAvailable()){
      if (goto_notif.getData().result.result<0){
        failure_counter++;
        ROS_ERROR("new failure. Counter = %d", failure_counter);
      }
    }

    /* Counting robot stops */
    if (scaling_notif.isANewDataAvailable())
    {
      if (scaling_notif.getData().data > stop_threshold)
      {
        if (stop_flag==true)
        {
          stopping_time += ros::Time::now() - stop_begin;
          ROS_INFO("stopping time total = %f", stopping_time.toSec());
        }
        stop_flag=false;
      }
      else
      {
        if (stop_flag==false){
          stop_counter++;
          ROS_WARN("new stop. Counter = %d", stop_counter);
          stop_begin = ros::Time::now();
        }
        stop_flag=true;
      }
    }
    /* Cycle time */
    if (human_cycle_notif.isANewDataAvailable())
    {
      ROS_FATAL("Human cycle time = %f", human_cycle_notif.getData().data);
    }
    if (robot_cycle_notif.isANewDataAvailable())
    {
      ROS_FATAL("Robot cycle time = %f", robot_cycle_notif.getData().data);
      failure_counter=0;
      stop_counter=0;
      stopping_time=ros::Duration(0.0);
    }

    loop_rate.sleep();
  }

  ROS_INFO("Listener node: exit normally.");
  return 0;
}


