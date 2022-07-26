#! /usr/bin/env python3

import rospy

from std_srvs.srv import SetBool,SetBoolResponse

from pymongo import MongoClient
import pymongo.errors

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


import numpy as np
import pprint
import os
import copy

import pandas as pd
import seaborn as sns

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'


SERVICE_CALLBACK = GREEN + "Service call {} received" + END
READY = GREEN + "Ready to manage db" + END
PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
CONNECTION_FAILED = "Connection to db failed: Request Timeout"
CONNECTION_OK = GREEN + "Connection to db executed" + END
SUCCESSFUL = "Successfully executed"
NOT_SUCCESSFUL = "Not Successfully executed"
CONNECTION_LOST = RED + "Connection to Database lost" + END
DURATION_OK = "Duration computed correctly"


def main():

    rospy.init_node("mongo_statistics")
    
    db_name = "sharework_demo"
    coll_properties_name = "hrc_task_properties"



    client = MongoClient(serverSelectionTimeoutMS=5000)     # 5 seconds of maximum connection wait
    try:
        client.server_info()
        rospy.loginfo(GREEN + CONNECTION_OK + END)
    except pymongo.errors.ServerSelectionTimeoutError:
        rospy.logerr(RED + CONNECTION_FAILED + END)
        raise
        
    db = client[db_name]
    task_properties = db[coll_properties_name]


    try:
        results = task_properties.find({})
        print(results)
    except pymongo.errors.AutoReconnect:
        rospy.logerr(CONNECTION_LOST)
        return 0

    for task in results:
        task_name = task["name"]
        id_task = task["_id"]
        if(not "pickplace" in task_name):
            goal = task_name
            goal = goal.replace("-robot","")
            goal = goal.replace("-human","")
            goal = goal.replace("-","_")
            goal = goal.replace("p","P")
            goal = goal.replace("a","A")
            print(goal)
            try:
                task_properties.update_one({"_id":id_task},{ "$set": { "goal": [goal] } })

            except pymongo.errors.AutoReconnect:
                rospy.logerr(CONNECTION_LOST)
            
            
            


    
    
    rospy.loginfo(READY)
    rospy.spin()

if __name__ == "__main__":
    main()