#!/usr/bin/env python

import rospy
import roslib
import math
import cv2 as cv # OpenCV2
from cv_bridge import CvBridge
import numpy as np
from nav_msgs.srv import GetMap
import tf
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import ros_numpy
from marker_brick import *



def wrap_angle(angle):
    # Function to wrap an angle between 0 and 2*Pi
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta/2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose

class BrickSearch:
    def __init__(self):

        # Variables/Flags
        self.localised_ = False
        self.brick_found_ = False
        self.image_msg_count_ = 0

        # Get the map via a ROS service call
        rospy.loginfo("Waiting for static_map service...")
        rospy.wait_for_service('static_map')
        get_map_service = rospy.ServiceProxy('static_map', GetMap)
        try:
            resp = get_map_service()
            self.map_ = resp.map
        except rospy.ServiceException as exc:
            rospy.logerror('Service did not process request: ' + str(exc))
            rospy.signal_shutdown('Service did not process request')
        rospy.loginfo("Map received")

        # Convert map into a CV image
        self.cv_bridge_ = CvBridge()
        self.map_image_ = np.reshape(self.map_.data, (self.map_.info.height, self.map_.info.width)).astype(np.int32)

        # Wait for the transform to become available
        rospy.loginfo("Waiting for transform from map to base_link")
        self.tf_listener_ = tf.TransformListener()

        while not rospy.is_shutdown() and not self.tf_listener_.canTransform("map", "base_link", rospy.Time(0.)):
            rospy.sleep(0.1)

        # Subscribe to the camera to get image
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)
        # Subscribe to the camera to get the depth data into pointcloud
        self.depth_sub_ = rospy.Subscriber("/camera/depth/points", PointCloud2, self.depth_callback, queue_size=1)


        # Advertise "cmd_vel" publisher to control TurtleBot manually
        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base action available")

        # Reinitialise AMCL
        global_localization_service_client = rospy.ServiceProxy('global_localization', Empty)
        empty = global_localization_service_client()
        rospy.sleep(0.5)

        # Subscribe to "amcl_pose" to get pose covariance
        self.amcl_pose_sub_ = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback, queue_size=1)

        self.bricks=[]

    def get_pose_2d(self):

        # Lookup the latest transform
        (trans,rot) = self.tf_listener_.lookupTransform('map', 'base_link', rospy.Time(0))

        print(trans)
        print(rot)

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = trans[0]
        pose.y = trans[1]

        qw = rot[3]
        qz = rot[2]

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        return pose

    def amcl_pose_callback(self, pose_msg):
        
        pose=pose_msg.pose.pose
        self.robot_pose_=[pose.position.x,pose.position.y,pose.orientation.z]
        # Check the covariance
        frobenius_norm = 0.0

        for e in pose_msg.pose.covariance:
            frobenius_norm += e**2

        if frobenius_norm < 0.05:
            self.localised_ = True

            # Unsubscribe from "amcl_pose" because we should only need to localise once at start up
            self.amcl_pose_sub_.unregister()

   

    def depth_callback(self, depth_msg):
        self.depth_data_=ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)

    def image_callback(self, image_msg):
        # Use this method to identify when the brick is visible

        
        # The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
        if self.image_msg_count_ < 1:
            self.image_msg_count_ += 1
            return
        else:
            self.image_msg_count_ = 0

        timeStamp=image_msg.header.stamp
        
        # Copy the image message to a cv_bridge image
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, "bgr8")
        HSV_frame = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        low_bound=np.array([0,240,100])
        high_bound=np.array([10,255,255])
        mask=cv.inRange(HSV_frame, low_bound, high_bound)

        elements=cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        size=0
        final_ele=None
        for e in elements:
            rec=cv.boundingRect(e)#find the rectangle that frames the element
            temp=rec[2]*rec[3] #calculate the area of the frame rectangle
            if size<temp:
                size=temp
                final_ele=e
        if final_ele is not None and self.localised_:
            self.brick_found_=True
            rec=cv.boundingRect(final_ele)
            # get center coordinate of the element in the image
            x=int(rec[0]+(rec[2])/2)
            y=int(rec[1]+(rec[3])/2)
            depth_brick=self.depth_data_[y,x]
            brick_coord=[depth_brick[2]+0.05,-depth_brick[0]]

            cv.rectangle(image, (int(rec[0]), int(rec[1])), (int(rec[0])+int(rec[2]), int(rec[3])+int(rec[1])), np.array([0,0,255]), 2)
            cv.circle(image, (int(x), int(y)), 5, np.array([0,0,255]), 10)
            cv.line(image, (int(x), int(y)), (int(x)+150, int(y)), np.array([0,0,255]), 2)
            cv.putText(image, "Brick-", (int(x)+10, int(y) -10), cv.FONT_HERSHEY_DUPLEX, 1, np.array([0,0,255]), 1, cv.LINE_AA)

            self.manage_brick(brick_coord,timeStamp)
        cv.imshow('Mask', cv.resize(mask,(960,540)))
        cv.imshow('image',cv.resize(image,(960,540)))
        cv.waitKey(2)

        rospy.loginfo('image_callback')
        rospy.loginfo('brick_found_: ' + str(self.brick_found_))

    def manage_brick(self,coord,time):
        createPose = init_PoseStamped(coord,time)
        transfPose = self.tf_listener_.transformPose("map", createPose)

        x=transfPose.pose.position.x
        y=transfPose.pose.position.y

        if all((math.sqrt((x-aBrick[0])**2 + (y-aBrick[1])**2))>0.3 for aBrick in self.bricks):
            self.bricks.append([x,y,1])
            
        else:
            for id,aBrick in enumerate(self.bricks):
                if math.sqrt((x-aBrick[0])**2 + (y-aBrick[1])**2) < 0.70 :
                    x=(x+aBrick[0]*9)/10
                    y=(y+aBrick[1]*9)/10
                    self.bricks[id][0]=x
                    self.bricks[id][1]=y
                    self.bricks[id][2]+=1

                    if self.bricks[id][2]>=2:
                        marker(x,y,0,id+1,time)

                    for id2 in range(0,id,1):
                        dist= math.sqrt((self.bricks[id2][0]-aBrick[0])**2 + (self.bricks[id2][1]-aBrick[1])**2)
                    
                        if dist < 0.25 :
                            marker_delete(aBrick,id,time)
                            self.bricks.pop(id) 

    def main_loop(self):

        # Wait for the TurtleBot to localise
        rospy.loginfo('Localising...')
        while not rospy.is_shutdown():

            # Turn slowly
            twist = Twist()
            twist.angular.z = 1.
            self.cmd_vel_pub_.publish(twist)

            if self.localised_:
                rospy.loginfo('Localised')
                break

            rospy.sleep(0.1)

        # Stop turning
        twist = Twist()
        twist.angular.z = 0.
        self.cmd_vel_pub_.publish(twist)


        # The map is stored in "map_"
        # You will probably need the data stored in "map_.info"
        # You can also access the map data as an OpenCV image with "map_image_"
        print(np.shape(self.map_image_))

        rospy.loginfo('Search the Brick...')
        while not rospy.is_shutdown():

            # Turn slowly
            twist = Twist()
            twist.angular.z = 0.8
            self.cmd_vel_pub_.publish(twist)

            if self.brick_found_:
                rospy.loginfo('Brick Found')
                pose_2d = self.get_pose_2d()
                
                rospy.loginfo('Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

                # Move toward the brick 

                rospy.sleep(0.1)

                slope = ( self.bricks[-1][1]-pose_2d.y )  / ( self.bricks[-1][0]-pose_2d.x )
                add=0
                if (self.bricks[-1][0]-pose_2d.x)<0:
                    add=math.pi
                pose_2d.theta = math.atan(slope) +add

                pose_2d.x = self.bricks[-1][0] -0.4*math.cos(pose_2d.theta)
                pose_2d.y = self.bricks[-1][1] -0.4*math.sin(pose_2d.theta) 
                rospy.loginfo(str(self.bricks[-1]))
                rospy.logwarn(slope)
                rospy.logwarn(pose_2d.theta)

                rospy.loginfo('Target pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

                # Send a goal to "move_base" with "self.move_base_action_client_"
                action_goal = MoveBaseActionGoal()
                action_goal.goal.target_pose.header.frame_id = "map"
                action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

                rospy.loginfo('Sending goal...')
                self.move_base_action_client_.send_goal(action_goal.goal)
                
                break

            rospy.sleep(0.1)


        # Stop turning
        twist = Twist()
        twist.angular.z = 0.
        self.cmd_vel_pub_.publish(twist)


        # This loop repeats until ROS is shutdown
        while not rospy.is_shutdown():

            rospy.loginfo('main_loop')

            # Get the state of the goal
            state = self.move_base_action_client_.get_state()

            rospy.loginfo('action state: ' + self.move_base_action_client_.get_goal_status_text())

            if state == actionlib.GoalStatus.SUCCEEDED:
  
                rospy.loginfo('Action succeeded!')

                # Shutdown when done
                #rospy.signal_shutdown('Action succeeded!')

            # Delay so the loop doesn't run too fast
            rospy.sleep(0.2)



if __name__ == '__main__':

    # Create the ROS node
    rospy.init_node('brick_search')

    # Create the brick search
    brick_search = BrickSearch()

    # Loop forever while processing callbacks
    brick_search.main_loop()




