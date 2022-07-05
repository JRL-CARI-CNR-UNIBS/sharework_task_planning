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
enum agent_status {Busy,Free};

void publishTask(const ros::Publisher pub,
                 const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
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
std::vector<std::string> manageStationP0(const std::vector<fixture_state> &fixture_vec_state)
{
  if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) == Empty)
    return {"move_PO_tO_P2"};
  else if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) != Empty)
    return {"move_P0_to_P1"};
  else if(fixture_vec_state.at(STATION_P1) != Empty && fixture_vec_state.at(STATION_P2) == Empty) /*Da aggiungere una condizione P1 to_work o to_load*/
    return {"move_P1_to_P2","move_P0_to_P1"};
  else
    return {"loop"};
//    throw std::invalid_argument("All station empty, put this task in queue");
   /*magari mettere return loop*/
}
std::string manageStationP1(const mqtt_scene_integration::Fixture & fixture_msg,
                            std::vector<fixture_state> &fixture_vec_state)
{
  fixture_vec_state.at(1)=getFixtureState(fixture_msg.state);
  switch(fixture_vec_state.at(1))
  {
    case ToUnload:
      return "unmount_P1_"+fixture_msg.content;
      break;
    case ToLoad:
      return "mount_P1_"+fixture_msg.content;
      break;
    case ToWork:
      return "move_P1_to_P3";
      break;
    default:
      throw std::invalid_argument("No task with that state");
      break;
  }
}
std::string manageStationP2(const mqtt_scene_integration::Fixture & fixture_msg,
                            std::vector<fixture_state> &fixture_vec_state)
{
  fixture_vec_state.at(2)=getFixtureState(fixture_msg.state);
  switch (fixture_vec_state.at(2))
  {
    case ToUnload:
      return "unmount_P2_"+fixture_msg.content;
      break;
    case ToLoad:
      return "mount_P2_"+fixture_msg.content;
      break;
    case ToWork:
      return "move_P2_to_P3";
      break;
    default:
      throw std::invalid_argument("No task with that state");
      break;
  }
}
void selectAgent()
{
  /*-   -*/
}
std::string manageStation(const int & n_station,
                          const mqtt_scene_integration::Fixture & fixture_msg,
                          std::vector<fixture_state> &fixture_vec_state,
                          std::string &agent_executor)
{
  /* Update the state */
  fixture_vec_state.at(n_station) = getFixtureState(fixture_msg.state);
  switch (getFixtureState(fixture_msg.state))
  {
    case ToUnload:
      return "unmount_"+std::to_string(n_station)+"_"+fixture_msg.content;
      break;
    case ToLoad:
      return "mount_"+std::to_string(n_station)+"_"+fixture_msg.content;
      break;
    case ToWork:
      /*agent_executor=HUMAN_AGENT*/;
      if(fixture_vec_state.at(STATION_P3)==Empty)
        return "move_"+std::to_string(n_station)+"_"+"_to_P3";
      else
        return "loop";
      break;
    default:
      throw std::invalid_argument("No task with that state");
      break;
  }
}

bool set_agent_status(std::map<std::string,agent_status> &agents_status, const std::string &agent, const agent_status &status)
{
  try
  {
    agents_status[agent] = status;
    return true;
  }
  catch(const std::out_of_range & err)
  {
    return false;
  }
}
bool reset_agent_status(std::map<std::string,agent_status> &agents_status, const std::string agent)
{
  try
  {
    agents_status.at(agent) = Free;
    return true;
  }
  catch(const std::out_of_range & err)
  {
      return false;
  }
}
void reset_agents_status(std::map<std::string,agent_status> &agents_status)
{
  for (auto it = agents_status.begin(); it != agents_status.end(); it++)
  {
      it->second = Free;
  }
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

  /* Load params */
  std::string fixture_base_topic_name;
  if (!pnh.getParam("fixture_base_topic_name",fixture_base_topic_name))
  {
    ROS_ERROR("fixture_base_topic_name not defined");
    return 0;
  }
  std::string human_task_topic_name;
  if (!pnh.getParam("human_task_topic_name",human_task_topic_name))
  {
    ROS_ERROR("human_task_topic_name not defined");
    return 0;
  }
  std::string human_task_feedback_topic;
  if (!pnh.getParam("human_task_feedback_topic",human_task_feedback_topic))
  {
    ROS_ERROR("human_task_feedback_topic not defined");
    return 0;
  }
  std::string robot_task_feedback_topic;
  if (!pnh.getParam("robot_task_feedback_topic",robot_task_feedback_topic))
  {
    ROS_ERROR("robot_task_feedback_topic not defined");
    return 0;
  }
  std::string robot_task_topic_name = "/JFMX/L1/sharework/station/p";
  if (!pnh.getParam("robot_task_topic_name",robot_task_topic_name))
  {
    ROS_ERROR("robot_task_topic_name not defined set to /JFMX/L1/sharework/station/p");
  }

  ros::Publisher human_task_pub = nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequest>(human_task_topic_name, 10);
  ros::Publisher robot_task_pub = nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequest>(robot_task_topic_name, 10);

  /* Initialize fixture station as empty*/
  for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
    fixture_vec_sub.push_back(std::make_shared<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>> (nh, fixture_base_topic_name+"/p"+std::to_string(k_fixture),10));
    fixture_vec_state.push_back(Empty);

  /* Initialize agent task feedback and status*/
  task_feedback[HUMAN] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, human_task_feedback_topic,10);
  task_feedback[ROBOT] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, robot_task_feedback_topic,10);
  agents_status[HUMAN] = Free;
  agents_status[ROBOT] = Free;
  reset_agents_status(agents_status);

  std::vector<std::string> task_to_pub;
  std::string agent_executor;
  std::queue<std::shared_ptr<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>>> task_queue;

  /* Main cycle */
  while(ros::ok())
  {
    /* Check task Feedback*/

    /*Verifica la coda ...*/

    /* Iterate stations */

    for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
    {
      if(fixture_vec_sub.at(k_fixture)->isANewDataAvailable())
      {
        received_msg = fixture_vec_sub.at(k_fixture)->getData();

        /* Reset station state when a fixture left */
        if(getFixtureState(received_msg.state)==Empty)
        {
          fixture_vec_state.at(k_fixture) = Empty;
          continue;
        }

//        fixtureManager(fixture_vec_sub.at(k_fixture)->getData(),fixture_vec_state);
        switch(k_fixture)
        {

          case STATION_P0:
//              try
            task_to_pub = manageStationP0(fixture_vec_state);
            /*publishTask*/(robot_task_pub,task_to_pub);
//              catch
//              {
//                /* put received_msg and k_fixture in queue */
//              }
            break;
          case STATION_P1:
//            task_to_pub = manageStation(k_fixture,received_msg,fixture_vec_state,agent_executor);
//            publishTask(robot_task_pub,task_to_pub);
            break;
          case STATION_P2:
//            task_to_pub = manageStation(k_fixture,received_msg,fixture_vec_state,agent_executor);
//            publishTask(robot_task_pub,task_to_pub);
            break;
          case STATION_P3:
            /* Update station P3 state*/
            fixture_vec_state.at(STATION_P3) = getFixtureState(received_msg.state);
            ROS_INFO_STREAM("Fixture left");
            break;

        }
      }
    }
  }

}
