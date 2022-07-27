#! /usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, MotionTaskExecutionRequestArray, MotionTaskExecutionFeedback

GREEN = '\033[92m'
RED = '\033[91m'
END = '\033[0m'


PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
SERVICE_NAME = "check_agent_state_"
REQUEST_TOPIC_NAME ="task_request_{}"
FEEDBACK_TOPIC_NAME = "task_feedback_{}"

class Agent:
    """docstring for ClassName."""
    def __init__(self,agent_name):
        self.name_ = agent_name
        self.state_ = None
    def updateAgentState(self,task_request):
        self.state_ = True
    def resetAgentState(self,task_response):
        self.state_ = False
    def checkAgentState(self,req):
        return TriggerResponse(self.state_,"")
        
        
    
def main():
    rospy.init_node('check_agent_state', anonymous=True)
    rospy.loginfo(GREEN + "Check agent state service on" + END)
    
    # #Required ROS parameters (agents)
    try:
        agents=rospy.get_param("~agents")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("agents") + END)
        return 0
    
    agents={"robot":1,"human":3}

    agents_name = agents.keys()
    agent_state = dict()
    for agent in agents_name:
        try:
            task_feedback_topic_name=rospy.get_param(FEEDBACK_TOPIC_NAME.format(agent))
        except KeyError:
            rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format(FEEDBACK_TOPIC_NAME.format(agent)) + END)
            return 0
        try:
            task_request_topic_name=rospy.get_param(REQUEST_TOPIC_NAME.format(agent))
        except KeyError:
            rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format(REQUEST_TOPIC_NAME.format(agent)) + END)
            return 0
        
        agent_state[agent] = []
        agent_state[agent].append(Agent(agent))
        agent_state[agent].append(rospy.Service(SERVICE_NAME+agent, Trigger, agent_state[agent][0].checkAgentState))
        agent_state[agent].append(rospy.Subscriber(task_request_topic_name.format(agent),MotionTaskExecutionRequestArray, agent_state[agent][0].updateAgentState))
        agent_state[agent].append(rospy.Subscriber(task_feedback_topic_name.format(agent),MotionTaskExecutionFeedback, agent_state[agent][0].resetAgentState))
    
    rospy.spin()

    

if __name__ == "__main__":
    main()