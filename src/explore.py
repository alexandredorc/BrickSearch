#!/usr/bin/env python
import os
import rospy
import math
from nav_msgs.msg import OccupancyGrid
import numpy as np

class Frontier:
    def __init__(self):
        # get the raw image from camera
        rospy.Subscriber("/map", OccupancyGrid, self.get_map)

    def get_map(self,data):
        rospy.logfatal("voila")
        map_fr=np.array(data.data)
        rospy.logdebug(np.shape(map_fr))

        
if __name__ =="__main__":
    rospy.init_node('get_frontier', anonymous=True)
    front=Frontier()

    front.rate=rospy.Rate(20)
    rospy.spin()