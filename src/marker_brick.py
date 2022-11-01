import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import  PoseStamped

commandPublisher = rospy.Publisher(
    '/brick',
    Marker, queue_size=10
)

# Initialize the marker message
def init_markers(x,y,z,id,time):
    marker = Marker()
    marker.header.frame_id = 'map' #self.global_frame
    marker.header.stamp=time
    marker.ns= "marker"
    marker.id= id
    marker.type = 1
    marker.action = 0
    marker.pose.position.x= x
    marker.pose.position.y= y
    marker.pose.position.z= z
    marker.lifetime= rospy.Duration.from_sec(0)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.2
    return marker

# Set the brick pose in camera_link frame
def init_PoseStamped(coor,time):
    Pose = PoseStamped()
    Pose.header.frame_id = 'camera_link'
    Pose.header.stamp= time
    Pose.pose.position.x= coor[0]
    Pose.pose.position.y= coor[1]
    Pose.pose.position.z= 0.0
    Pose.pose.orientation.x = 0.0
    Pose.pose.orientation.y = 0.0
    Pose.pose.orientation.z = 0.0
    Pose.pose.orientation.w = 1.0
    return Pose

# publish marker to Rviz
def marker(x,y,z,id,time):
    brick= init_markers(x,y,z,id,time)
    commandPublisher.publish(brick)

# Modify a marker in Rviz
def marker_modify(x,y,z,id,time):
    brick= init_markers(x,y,z,id,time)
    brick.action = Marker.MODIFY
    commandPublisher.publish(brick)

# Delete a marker in Rviz
def marker_delete(coor,id,time):
    delete_brick=init_markers(coor[0],coor[1],0,id,time)
    delete_brick.action=Marker.DELETE
    commandPublisher.publish(delete_brick)
