#!/usr/bin/env python
import os
import rospy
import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import numpy as np
import cv2 

class Frontier:
    def __init__(self):
        # get the raw image from camera
        rospy.Subscriber("/map", OccupancyGrid, self.get_map)
        # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base action available")


    def get_map(self,map_msg):
        
        res=(map_msg.info.resolution)
        height=map_msg.info.height
        width=map_msg.info.width
        map_ex=np.reshape(np.array(map_msg.data),(width,height))
        map_front=np.zeros((width,height))
        for i in range(width):
            for j in range(height):
                if map_ex[i,j]==0:
                    if map_ex[i+1,j]==-1 :
                        map_front[i,j]=255
                        map_front[i+1,j]=255
                    if map_ex[i-1,j]==-1:
                        map_front[i,j]=255
                        map_front[i-1,j]=255
                    if map_ex[i,j+1]==-1 :
                        map_front[i,j]=255
                        map_front[i,j+1]=255
                    if map_ex[i,j-1]==-1: 
                        map_front[i,j]=255
                        map_front[i,j-1]=255 
        image_front=cv2.cvtColor(map_front.astype('uint8'), cv2.COLOR_GRAY2BGR)
        
        elements = cv2.findContours(map_front.astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[-2]
        

        size_max=0
        final_ele=None
        for e in elements:
            x,y,w,h = cv2.boundingRect(e)
            if w>12 or h>12 and size_max<w*h:
                size_max=w*h
                final_ele=e
        if final_ele is not None:
            x,y,w,h = cv2.boundingRect(final_ele)
            x=x+w/2
            y=y+h/2
            center = (int(x),int(y))
            radius = 2
            cv2.circle(image_front,center,radius,(0,255,0),2)
            pose = Pose()
            
            pose.position.x = (x-width/2)*res -0.5
            pose.position.y = (y-height/2)*res -0.5

            pose.orientation.w = 1
            pose.orientation.z = 0
     
            action_goal = MoveBaseActionGoal()
            action_goal.goal.target_pose.header.frame_id = "map"
            action_goal.goal.target_pose.pose = pose
            self.move_base_action_client_.send_goal(action_goal.goal)

        cv2.imshow("frontiers",image_front)
        cv2.waitKey(3)
                       

        
if __name__ =="__main__":
    rospy.init_node('get_frontier', anonymous=True)
    front=Frontier()

    front.rate=rospy.Rate(20)
    rospy.spin()