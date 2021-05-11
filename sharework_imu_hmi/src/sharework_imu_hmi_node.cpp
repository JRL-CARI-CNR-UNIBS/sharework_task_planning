/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Marco Faroni marco.faroni@stiima.cnr.it
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <task_planner_interface_msgs/MotionTaskExecutionFeedback.h>
#include <task_planner_interface_msgs/MotionTaskExecutionRequestArray.h>

#define INIT 0
#define WAIT_FOR_REQUEST 1
#define WAIT_FOR_GESTURE 2

void newTaskRequestCallback(const task_planner_interface_msgs::MotionTaskExecutionRequestArrayConstPtr& msg){}
void newGestureCallback(const std_msgs::StringConstPtr& msg){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sharework_imu_hmi_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic_task_request;
  if (!pnh.getParam("topic_task_request",topic_task_request))
  {
    ROS_ERROR("topic_task_request not defined");
    return false;
  }
  std::string topic_task_feedback;
  if (!pnh.getParam("topic_task_feedback",topic_task_feedback))
  {
    ROS_ERROR("topic_task_feedback not defined");
    return false;
  }

  std::string gesture_command="FIST";
  if (!pnh.getParam("gesture_command",gesture_command))
    ROS_ERROR_STREAM("gesture_command not defined. Default:" << gesture_command);

  ros::Publisher feedback_pub=nh.advertise<task_planner_interface_msgs::MotionTaskExecutionFeedback>(topic_task_feedback,1);
  ros::Publisher vibration_pub=nh.advertise<std_msgs::UInt8>("/myo_raw/vibrate",1);

  ros_helper::SubscriptionNotifier<task_planner_interface_msgs::MotionTaskExecutionRequestArray> task_request_notif(nh,topic_task_request,1);
  task_request_notif.setAdvancedCallback(&newTaskRequestCallback);
  task_request_notif.waitForANewData(ros::Duration(1));

  ros_helper::SubscriptionNotifier<std_msgs::String> gesture_notif(nh,"/myo_raw/myo_gest_str",1);
  gesture_notif.setAdvancedCallback(&newGestureCallback);

  int state=INIT;
  while (ros::ok())
  {
    if (state==INIT)
    {
      if (gesture_notif.waitForANewData(ros::Duration(60)))
      {
        if (!gesture_command.compare(gesture_notif.getData().data))
        {
          state=WAIT_FOR_REQUEST;
          ROS_INFO("First gesture recevied. Waiting for task request...");
        }
      }
      else
      {
        ROS_ERROR("No initial gesture from wristband. Killing node...");
        return 0;
      }
    }
    else if (state==WAIT_FOR_REQUEST)
    {
      if (task_request_notif.waitForANewData(ros::Duration(30)))
      {
        ROS_INFO("Task request received. Waiting for gesture feedback...");
        std_msgs::UInt8Ptr vibration_msg(new std_msgs::UInt8());
        ROS_INFO("qui");
        vibration_msg->data=3;
        vibration_pub.publish(vibration_msg);
        state=WAIT_FOR_GESTURE;
        ros::Duration(2.0).sleep();
      }
    }
    else if (state==WAIT_FOR_GESTURE)
    {
      if (gesture_notif.waitForANewData(ros::Duration(90)))
      {
        if (!gesture_command.compare(gesture_notif.getData().data))
        {
          std_msgs::UInt8Ptr vibration_msg(new std_msgs::UInt8());;
          vibration_msg->data=1;
          vibration_pub.publish(vibration_msg);
          task_planner_interface_msgs::MotionTaskExecutionFeedbackPtr feedback_msg(new task_planner_interface_msgs::MotionTaskExecutionFeedback());;
          feedback_msg->cmd_id = task_request_notif.getData().cmd_id;
          feedback_msg->result=1;
          feedback_pub.publish(feedback_msg);
          state=WAIT_FOR_REQUEST;
          ROS_INFO("Gesture feedback sent. Waiting for new task request...");
        }
      }
    }
  }
  return 0;
}


