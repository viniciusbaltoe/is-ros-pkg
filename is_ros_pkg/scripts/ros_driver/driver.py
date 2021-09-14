# Python
import time
import json

# IS
from is_wire.core import Logger, Status ,StatusCode, logger
from is_msgs.robot_pb2 import BasicMoveTask, RobotTaskReply 
from is_msgs.common_pb2 import Position

# ROS
import rospy
from nav_msgs.msg import Odometry

class ROS_Robot(object):
    logger = Logger("ROS_Robot")

# -=-=-=-=-=-=-=-=-=-=-= INITIALIZATION FUNCTIONS =-=-=-=-=-=-=-=-=-=-=-
    def __init__(self, config):
        self.robot_id = config['robot_id']
        self.position = Position()

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-= FUNCTIONS =-=-=-=-=-=-=-=-=-=-=-=-=-=-      
    def get_position(self):
        rospy.init_node('get_position', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.callback)
        while not rospy.is_shutdown():
            time.sleep(1)
        return self.position
    def callback(self, odom):
        self.position.x = odom.pose.pose.position.x
        self.position.y = odom.pose.pose.position.y
        self.position.z = odom.pose.pose.position.z
        rospy.signal_shutdown('Get Position Finished.')

    #def cell_position(position):
