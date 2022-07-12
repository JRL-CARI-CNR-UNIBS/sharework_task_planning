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
//#define HUMAN "human"
//#define ROBOT "robot"

enum fixture_state {Empty, ToLoad, ToUnload, ToWork};
enum agent_status {Busy=1,Free=0};
struct fixture_request{
  mqtt_scene_integration::Fixture fixture_msg;
  int n_station;
};
enum agent{Human,Robot};


/**
 * Publish a task
 *
 * @param pub A ros publisher
 * @param yask_name the name of the task to publish
 */
void publishTask(const ros::Publisher &pub,
                 const std::string &task_name)
{
    task_planner_interface_msgs::MotionTaskExecutionRequestArray task_to_publish;
    task_planner_interface_msgs::MotionTaskExecutionRequest task;
    task.task_id = task_name;
//    task_to_publish.cmd_id = 1;
    task_to_publish.tasks={task};
    ROS_INFO_STREAM("Task published: "<< task_to_publish);
    pub.publish(task_to_publish);
    ros::spinOnce();

}
/**
  Get Fixture state from string
 * @brief Get Fixture state from string
 * @param fixture_state in string format
 * @return a fixture state (from enum)
 */
fixture_state getFixtureState(const std::string &fixture_state)
{
  /**/
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
/**
  Manager of station P0: define task to pub.
 * @brief manageStationP0 Manager of station P0: define task to pub.
 * @param fixture_vec_state state of all fixture
 * @param task_to_pub a vector in which will be put the task to publish
 * @return false if all fixture are busy true otherwise
 */
bool manageStationP0(const std::vector<fixture_state> &fixture_vec_state,std::map<agent,std::queue<std::string>> &agents_task_queue)
{
  if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) == Empty)
  {
    agents_task_queue[Human].push("move_PO_tO_P2");
    ROS_INFO_STREAM("Task "<<"move_PO_tO_P2" <<" added in human task queue");
//    task_to_pub = {"move_PO_tO_P2"};
    return true;
  }
  else if(fixture_vec_state.at(STATION_P1) == Empty && fixture_vec_state.at(STATION_P2) != Empty)
  {
    agents_task_queue[Human].push("move_P0_to_P1");
    ROS_INFO_STREAM("Task "<< "move_P0_to_P1" <<" added in human task queue");
//    task_to_pub = {"move_P0_to_P1"};
    return true;
  }
  else if(fixture_vec_state.at(STATION_P1) != Empty && fixture_vec_state.at(STATION_P2) == Empty) /*Da aggiungere una condizione P1 to_work o to_load*/
  {
    int agent_queue_size;
    for (auto  it = agents_task_queue.begin(); it != agents_task_queue.end(); it++)
    {
      if(it->first == Human)
      {
        it->second.push("move_P1_to_P2");
        it->second.push("move_P0_to_P1");
      }

      agent_queue_size = it->second.size();
      for(int k=0;k<agent_queue_size;k++)
      {
        if(it->second.front().find("unmount_P1") == std::string::npos && it->second.front().find("mount_P1") == std::string::npos && it->second.front().find("move_P1") == std::string::npos)
        {
          it->second.push(it->second.front());
        }
        it->second.pop();
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}
/**
  Add one task to the shortes given queues
 * @brief addTaskToRightAgentQueue Add one task to the shortes given queues
 * @param human_task_queue  human task queue
 * @param robot_task_queue  robot task queue
 * @param task_name   task to add to one queue
 */
void addTaskToRightAgentQueue(std::queue<std::string> &human_task_queue, std::queue<std::string> &robot_task_queue, const std::string task_name)
{
  if(human_task_queue.size()>robot_task_queue.size())
  {
    robot_task_queue.push(task_name);
    ROS_INFO_STREAM("Task "<<task_name <<" added in robot task queue");
  }
  else
  {
    human_task_queue.push(task_name);
    ROS_INFO_STREAM("Task "<<task_name <<" added in human task queue");
  }
}
/**
 * @brief resetAgentsStatus Reset all agent status (Free)
 * @param agents_status A Map <agent name - status>
 */
void resetAgentsStatus(std::map<agent,agent_status> &agents_status)
{
  for (auto it = agents_status.begin(); it != agents_status.end(); it++)
  {
      it->second = Free;
  }
}
/**
 * @brief isAnAgentFree Check if there is at least one free agent
 * @param agents_status agent status map<agent name - status>
 * @return true if there is at least one free agent, false otherwise
 */
bool isAnAgentFree(std::map<agent,agent_status> &agents_status)
{
  bool an_agent_free = true;
  for (auto it = agents_status.begin(); it != agents_status.end(); it++)
  {
    if(it->second == Free)
      an_agent_free = true;
  }
  return an_agent_free;
}
void printQueue(std::map<agent, std::queue<std::string>> agents_task_queue)
{
  int agent_queue_size;
  for(auto it = agents_task_queue.begin(); it != agents_task_queue.end(); it++)
  {
    ROS_INFO_STREAM("Queue of agent"<<it->first);
    agent_queue_size = it->second.size();
    for(int k=0;k<agent_queue_size;k++)
    {
      ROS_INFO_STREAM(it->second.front());
      it->second.pop();
    }
  }

}

std::string getAgentStatus(const agent_status &status)
{
  if(status == Free)
  {
    return "Free";
  }
  else
  {
    return "Busy";
  }
}
std::string getAgentName(const agent &agent_name)
{
  if(agent_name == Human)
  {
    return "Human";
  }
  else
  {
    return "Robot";
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
  std::map<agent,std::shared_ptr<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>>> task_feedback;
  std::map<agent,agent_status> agents_status;

  std::map<int, mqtt_scene_integration::Fixture> received_msgs;

  /* Load params */
  std::string fixture_base_topic_name;
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
  double dt;
  if (!pnh.getParam("sampling_time",dt))
  {
    ROS_ERROR("dt not defined.");
    if(dt<=0)
      ROS_ERROR("dt must be greater than 0.");
  }


  /* Initialize fixture station as empty*/
  for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
  {
    fixture_vec_sub.push_back(std::make_shared<ros_helper::SubscriptionNotifier<mqtt_scene_integration::Fixture>> (nh, fixture_base_topic_name+std::to_string(k_fixture),10));
    fixture_vec_state.push_back(Empty);
  }

  /* Initialize agent task feedback and status*/
  task_feedback[Human] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, human_task_feedback_topic,10);
  task_feedback[Robot] = std::make_shared<ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionFeedback>> (nh, robot_task_feedback_topic,10);
  agents_status[Human] = Free;
  agents_status[Robot] = Free;


  std::vector<std::string> task_to_pub;

  std::vector<bool> station_flag_changed ={false,false,false,false};

  std::map<agent, std::queue<std::string>> agents_task_queue;
  std::map<agent, ros::Publisher> agents_task_request_pub;
  agents_task_request_pub[Human] =  nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(human_task_request_topic, 10);
  agents_task_request_pub[Robot] =  nh.advertise<task_planner_interface_msgs::MotionTaskExecutionRequestArray>(robot_task_request_topic, 10);



  /* Main cycle */
  while(ros::ok())
  {
    ros::spinOnce();
    ros::Duration(dt).sleep();
    ROS_INFO_STREAM("Robot status: "<<getAgentStatus(agents_status[Robot]));
    ROS_INFO_STREAM("Human status: "<<getAgentStatus(agents_status[Human]));

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



    /* Check if agent are free and send task request*/
    for (auto  it = agents_task_queue.begin(); it != agents_task_queue.end(); it++)
    {
      if(agents_status[it->first] == Free)
      {
        if(it->second.size()>0)
        {
          publishTask(agents_task_request_pub[it->first],it->second.front());
          agents_status[it->first] = Busy;
          ROS_INFO_STREAM("Task "<<it->second.front() <<" request to agent: "<< getAgentName(it->first));
          it->second.pop();
        }
      }
    }

    /* Check if event occur iterating stations */
    bool event = false;
    received_msgs.clear();
    for(unsigned int k_fixture=0;k_fixture<FIXTURES_NUMBER; k_fixture++)
    {
      if(fixture_vec_sub.at(k_fixture)->isANewDataAvailable())
      {
        printQueue(agents_task_queue);
        ROS_INFO_STREAM("New fixture trigger arrived at fixture :" << k_fixture);
        received_msgs[k_fixture] = fixture_vec_sub.at(k_fixture)->getData();  // Deve essere vet
        fixture_vec_state.at(k_fixture) = getFixtureState(received_msgs[k_fixture].state);
        if(fixture_vec_state.at(k_fixture)!=Empty)
        {
          station_flag_changed.at(k_fixture)=true;
          event = true;
        }
      }
    }
    if(not event)
    {
      continue;
    }

    task_to_pub.clear();
    if(station_flag_changed.at(STATION_P0))
    {
      if(manageStationP0(fixture_vec_state,agents_task_queue))
      {
        station_flag_changed.at(STATION_P0)=false;
      }
    }
    for(int k_fixture=1; k_fixture<=2;k_fixture++)
    {
      if(not station_flag_changed.at(k_fixture))
        continue;

      switch(fixture_vec_state.at(k_fixture))
      {
        case ToUnload:
          addTaskToRightAgentQueue(agents_task_queue[Human], agents_task_queue[Robot], "unmount_P"+std::to_string(k_fixture)+"_"+received_msgs[k_fixture].content);
          station_flag_changed.at(k_fixture) = false;
          break;
        case ToLoad:
          addTaskToRightAgentQueue(agents_task_queue[Robot], agents_task_queue[Human], "mount_P"+std::to_string(k_fixture)+"_"+received_msgs[k_fixture].content);
          station_flag_changed.at(k_fixture) = false;
          break;
        case ToWork:
          if(fixture_vec_state.at(STATION_P3) == Empty)
          {
            station_flag_changed.at(k_fixture) = false;
            agents_task_queue[Human].push("move_P"+std::to_string(k_fixture)+"_to_P3");
            ROS_INFO_STREAM("Task "<<"move_P"+std::to_string(k_fixture)+"_to_P3" <<" added in human task queue");
          }
          break;
      }
    }

  }

}
