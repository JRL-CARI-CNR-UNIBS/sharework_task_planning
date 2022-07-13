#! /usr/bin/env python3

import rospy

from task_planner_interface_msgs.msg import MotionTaskExecutionRequest, MotionTaskExecutionRequestArray, MotionTaskExecutionFeedback
from mqtt_scene_integration.msg import Fixture
from functools import partial

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'


PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
TO_UNLOAD = "ToUnload"
TO_WORK = "ToWork"
EMPTY = "Empty"
TO_LOAD = "ToLoad"
PEZZO = "A10"
HUMAN = "human"
ROBOT = "robot"

def sendAgentFeedback(agent_name):
    pub = rospy.Publisher("Non mi ricordo", MotionTaskExecutionFeedback, queue_size=10)
    pub.publish(MotionTaskExecutionFeedback(1,1))
        
def moveP0ToP2():
    print("Fai un pò di cose")
def newPallet():
    print("New pallet")
    
def my_quit_fn():
   raise SystemExit

def invalid():
   print("INVALID CHOICE!")

class MenuOptionManager:
    
    def __init__(self,human_feedback_topic_name,robot_feedback_topic_name,fixture_base_topic_name):
        self.human_feedback_pub_ = rospy.Publisher(human_feedback_topic_name, MotionTaskExecutionFeedback, queue_size=10)
        self.robot_feedback_pub_ = rospy.Publisher(robot_feedback_topic_name, MotionTaskExecutionFeedback, queue_size=10)
        
        self.n_fixtures = 4
        
        self.fixtures_pub_=dict()
        for fixture in range(0,self.n_fixtures):
            self.fixtures_pub_[fixture] = rospy.Publisher(fixture_base_topic_name + str(fixture), Fixture, queue_size=10)               

    def newPallet(self):
        self.fixtures_pub_[0].publish(Fixture(TO_UNLOAD, PEZZO))
    
    def movedP0ToP2(self):
        self.fixtures_pub_[0].publish(Fixture(EMPTY, PEZZO))
        self.fixtures_pub_[2].publish(Fixture(TO_UNLOAD, PEZZO))
        self.sendAgentFeedback(HUMAN)
        
    def movedP0ToP1(self):
        self.fixtures_pub_[0].publish(Fixture(EMPTY, PEZZO))
        self.fixtures_pub_[1].publish(Fixture(TO_UNLOAD, PEZZO))
        self.sendAgentFeedback(HUMAN)
            
    def movedToP3(self,starting_station):
        self.fixtures_pub_[starting_station].publish(Fixture(EMPTY, PEZZO))
        self.fixtures_pub_[3].publish(Fixture(TO_WORK, PEZZO))
        self.sendAgentFeedback(HUMAN)
    
    def unloaded(self,agent,station):
        self.sendAgentFeedback(agent)
        self.fixtures_pub_[station].publish(Fixture(TO_LOAD, PEZZO)) 
    def loaded(self,agent,station):
        self.sendAgentFeedback(agent)
        self.fixtures_pub_[station].publish(Fixture(TO_WORK, PEZZO))
         
    def freeP3(self):
        self.fixtures_pub_[3].publish(Fixture(EMPTY, PEZZO))
        
    def sendAgentFeedback(self,agent):
        if agent == HUMAN:
            self.human_feedback_pub_.publish(MotionTaskExecutionFeedback(0,0))
        else:
            self.robot_feedback_pub_.publish(MotionTaskExecutionFeedback(0,0))
def printMenu(menu):
    for key in menu.keys():
        print(key+":" + menu[key][0])    
def main():
    rospy.init_node('debug', anonymous=True)
    rospy.loginfo(GREEN + "Debugging node started" + END)

    try:
        human_feedback_to_planner=rospy.get_param("~human_feedback_to_planner")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("human_feedback_to_planner") + END)
        return 0
    try:
        robot_feedback_to_planner=rospy.get_param("~robot_feedback_to_planner")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("robot_feedback_to_planner") + END)
        return 0
    try:
        fixture_base_topic_name=rospy.get_param("~fixture_base_topic_name")
    except KeyError:
        rospy.logerr(RED + PARAM_NOT_DEFINED_ERROR.format("fixture_base_topic_name") + END)
        return 0
    
    menu_manager = MenuOptionManager(human_feedback_to_planner,robot_feedback_to_planner,fixture_base_topic_name)    
    rospy.sleep(1)
    
    menu = {"0":("Exit",my_quit_fn),
            "1":("New Pallet",menu_manager.newPallet),
            "2":("Moved P0 to P1",menu_manager.movedP0ToP1),
            "3":("Moved P0 to P2",menu_manager.movedP0ToP2),
            "4":("Moved P1 to P3",partial(menu_manager.movedToP3,1)),
            "5":("Moved P2 to P3",partial(menu_manager.movedToP3,2)),
            "6":("Human Unmounted P1",partial(menu_manager.unloaded,HUMAN,1)),
            "7":("Robot Unmounted P1",partial(menu_manager.unloaded,ROBOT,1)),
            "8":("Human Unmounted P2",partial(menu_manager.unloaded,HUMAN,2)),
            "9":("Robot Unmounted P2",partial(menu_manager.unloaded,ROBOT,2)),
            "10":("Human Mounted P1",partial(menu_manager.loaded,HUMAN,1)),
            "11":("Robot Mounted P1",partial(menu_manager.loaded,ROBOT,1)),
            "12":("Human Mounted P2",partial(menu_manager.loaded,HUMAN,2)),
            "13":("Robot Mounted P2",partial(menu_manager.loaded,ROBOT,2)),
            "14":("Free P3",menu_manager.freeP3)                       
        }
#    menu_manager.newPallet() 
    another_command = True

    for key in menu.keys():
        print(key+":" + menu[key][0])
    
    while True and not rospy.is_shutdown():                                 # until user want another position
        
        choise = input("Command: ")
        menu.get(choise,[None,invalid])[1]()

    # rospy.loginfo(position_list)


if __name__ == "__main__":
    main()