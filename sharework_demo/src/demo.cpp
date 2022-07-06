#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <mqtt_scene_integration/Fixture.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>

#include <queue>

#define FIXTURES_NUMBER 4
#define STATION_P0 0
#define STATION_P1 1
#define STATION_P2 2
#define STATION_P3 3
#define HUMAN "human"
#define ROBOT "robot"

enum fixture_state {Empty, ToLoad, ToUnload, ToWork};
enum agent_status {Busy=1,Free=0};
struct fixture_request{
  mqtt_scene_integration::Fixture fixture_msg;
  int n_station;
};


void publishTask(const ros::Publisher pub,
                 const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
//    task_to_publish.cmd_id = 1;
    task_to_publish.tasks={task};
    pub.publish(task_to_publish);
}
fixture_state getFixtureState(const std::string &fixture_state)
{
  if(fixture_state.compare("Empty")==0)
    return Empty;
  else if(fixture_state.compare("ToLoad")==0)
    return ToLoad;
  else if(fixture_state.compare("ToUnload")==0)
    return ToUnload;
  else if(fixture_state.compare("ToWork")==0)
    return ToWork;
  else
    throw std::invalid_argument( "Fixture state not known" );
}
bool manageStationP0(const std::vector<fixture_state> &fixture_vec_state, std::vector<std::string> &task_to_pub)
{
  if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) == Empty)
  {
    task_to_pub = {"move_PO_tO_P2"};
    return true;
  }
  else if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) != Empty)
  {
    task_to_pub = {"move_P0_to_P1"};
    return true;
  }
  else if(fixture_vec_state.at(STATION_P1) != Empty && fixture_vec_state.at(STATION_P2) == Empty) /*Da aggiungere una condizione P1 to_work o to_load*/
  {
    task_to_pub = {"move_P1_to_P2","move_P0_to_P1"};
    return true;
  }
  else
  {
    return false;
  }
}



void addTaskToRightAgentQueue(std::queue<std::string> &human_task_queue, std::queue<std::string> &robot_task_queue, const std::string task_name)
{
  if(human_task_queue.size()>robot_task_queue.size())
  {
    robot_task_queue.push(task_name);
    ROS_INFO_STREAM("Task "<<task_name <<"added in robot task queue");
  }
  else
  {
    human_task_queue.push(task_name);
    ROS_INFO_STREAM("Task "<<task_name <<"added in human task queue");
  }
}

void resetAgentsStatus(std::map<std::string,agent_status> &agents_status)
{
  for (auto it = agents_status.begin(); it != agents_status.end(); it++)
  {
      it->second = Free;
  }
}
bool isAnAgentFree(std::map<std::string,agent_status> &agents_status)
{
  bool an_agent_free = true;
  for (auto it = agents_status.begin(); it != agents_status.end(); it++)
  {
    if(it->second == Free)
      an_agent_free = true;
  }
  return an_agent_free;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  /*Fixture subscribers and state*/
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>>> fixture_vec_sub;
  std::vector<fixture_state> fixture_vec_state;

  /*Agents task feedback and status*/
  std::map<std::string,std::shared_ptr<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>>> task_feedback;
  std::map<std::string,agent_status> agents_status;

  mqtt_scene_integration::Fixture received_msg;
  std::map<int, mqtt_scene_integration::Fixture> received_msgs;

  /* Load params */
  std::string fixture_base_topic_name;//"/JFMX/L1/sharework/station/p"
  if (!pnh.getParam("fixture_base_topic_name",fixture_base_topic_name))
  {
    ROS_ERROR("fixture_base_topic_name not defined");
    return 0;
  }

  std::string human_task_feedback_topic;
  if (!pnh.getParam("human_feedback_to_planner",human_task_feedback_topic))
  {
    ROS_ERROR("human_feedback_to_planner not defined");
    return 0;
  }
  std::string robot_task_feedback_topic;
  if (!pnh.getParam("robot_feedback_to_planner",robot_task_feedback_topic))
  {
    ROS_ERROR("robot_feedback_to_planner not defined");
    return 0;
  }
  std::string human_task_request_topic;
  if (!pnh.getParam("human_request_from_planner",human_task_request_topic))
  {
    ROS_ERROR("human_request_from_planner not defined");
    return 0;
  }
  std::string robot_task_request_topic;
  if (!pnh.getParam("robot_request_from_planner",robot_task_request_topic))
  {
    ROS_ERROR("robot_request_from_planner not defined.");
  }

  ros::Publisher human_task_pub = nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequest>(human_task_request_topic, 10);
  ros::Publisher robot_task_pub = nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequest>(robot_task_request_topic, 10);

  /* Initialize fixture station as empty*/
  for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
  {
    fixture_vec_sub.push_back(std::make_shared<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>> (nh, fixture_base_topic_name+std::to_string(k_fixture),10));
    fixture_vec_state.push_back(Empty);
  }

  /* Initialize agent task feedback and status*/
  task_feedback[HUMAN] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, human_task_feedback_topic,10);
  task_feedback[ROBOT] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, robot_task_feedback_topic,10);
  agents_status[HUMAN] = Free;
  agents_status[ROBOT] = Free;
  resetAgentsStatus(agents_status);

  std::vector<std::string> task_to_pub;
  std::string agent_executor;

  std::queue<fixture_request> fixture_queue;
  std::queue<std::shared_ptr<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>>> task_queue;

  std::vector<bool> station_flag_changed ={false,false,false,false};

  std::queue<std::string> human_task_queue;
  std::queue<std::string> robot_task_queue;

  /* Main cycle */
  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Robot status: "<<agents_status[ROBOT]);
    ROS_INFO_STREAM("Human status: "<<agents_status[HUMAN]);

    /* Check task Feedback and Reset agent status */
    for (auto  it = task_feedback.begin(); it != task_feedback.end(); it++)
    {
      if(it->second->isANewDataAvailable())
      {
        it->second->getData();
        agents_status[it->first] = Free;
        ROS_INFO_STREAM("Arrivato feedback: "<<it->first);
      }
    }

    if(agents_status[HUMAN] == Free)
    {
      if(human_task_queue.size()>0)
      {
        publishTask(human_task_pub,human_task_queue.front());
        agents_status[HUMAN] = Busy;
        ROS_INFO_STREAM("Task "<<human_task_queue.front() <<"request to human");
        human_task_queue.pop();

      }
    }
    if(agents_status[ROBOT] == Free)
    {
      if(robot_task_queue.size()>0)
      {
        publishTask(robot_task_pub,robot_task_queue.front());
        agents_status[ROBOT]=Busy;
        ROS_INFO_STREAM("Task "<<human_task_queue.front() <<"request to robot");
        robot_task_queue.pop();
      }
    }

    /* Check if event occur iterating stations */
    bool event = false;
    received_msgs.clear();
    for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
    {
      if(fixture_vec_sub.at(k_fixture)->isANewDataAvailable())
      {
        ROS_INFO_STREAM("ARRIVATO");
        received_msgs[k_fixture] = fixture_vec_sub.at(k_fixture)->getData();  // Deve essere vet
        fixture_vec_state.at(k_fixture) = getFixtureState(received_msgs[k_fixture].state);
        station_flag_changed.at(k_fixture)=true;
        event = true;
      }
    }
    if(not event)
    {
      continue;
    }

    std::vector<std::string> task_to_pub;
    if(station_flag_changed.at(STATION_P0))
    {
      if(manageStationP0(fixture_vec_state,task_to_pub))
      {
        for(std::string task_name : task_to_pub)
        {
          human_task_queue.push(task_name);
          ROS_INFO_STREAM("Task "<<task_name <<"added in human task queue");
        }
        station_flag_changed.at(STATION_P0)=false;
      }
    }
    for(int k_fixture=1; k_fixture<=2;k_fixture++)
    {
      switch(fixture_vec_state.at(k_fixture))
      {
        case ToUnload:
          addTaskToRightAgentQueue(human_task_queue, robot_task_queue, "unmount_P"+std::to_string(k_fixture)+"_"+received_msgs[k_fixture].content);
          station_flag_changed.at(k_fixture) = false;
          break;
        case ToLoad:
          addTaskToRightAgentQueue(human_task_queue, robot_task_queue, "mount_P"+std::to_string(k_fixture)+"_"+received_msgs[k_fixture].content);
          station_flag_changed.at(k_fixture) = false;
          break;
        case ToWork:
          if(fixture_vec_state.at(STATION_P3) == Empty)
          {
            station_flag_changed.at(k_fixture) = false;
            human_task_queue.push("move_P"+std::to_string(k_fixture)+"_to_P3");
          }
          break;
      }
    }





  }

}
