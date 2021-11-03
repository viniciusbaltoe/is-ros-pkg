# Python
import time
import json
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# IS
from is_wire.core import Logger, Status, StatusCode, logger
from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply 
from is_msgs.common_pb2 import Position

# ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, BoolParameter

class ROS_Robot(object):
    logger = Logger("ROS_Robot")

# -=-=-=-=-=-=-=-=-=-=-= INITIALIZATION FUNCTIONS =-=-=-=-=-=-=-=-=-=-=-
    def __init__(self, config):
        self.robot_id = config['robot_id']
        self.position = Position()
        self.robot_orientation_q = Quaternion()
        self.mbag_status = GoalStatusArray()
        self.goal_id = 0

# -=-=-=-=-=-=-=-=-=-=-=-=-=-=-= FUNCTIONS =-=-=-=-=-=-=-=-=-=-=-=-=-=-      
    def get_position(self):
        self.function_status = False
        rospy.Subscriber("odom", Odometry, self.get_position_callback)
        while self.function_status is not True:
            time.sleep(1)
        (roll, pitch, yaw) = euler_from_quaternion([self.robot_orientation_q.x, self.robot_orientation_q.y, self.robot_orientation_q.z, self.robot_orientation_q.w])
        self.logger.info("Getting position.")
        self.logger.info("Robot orientation (roll, pitch, yaw):  \n({}, {}, {})".format(roll, pitch, yaw))
        return self.position
    
    def get_position_callback(self, odom):
        self.position.x = odom.pose.pose.position.x
        self.position.y = odom.pose.pose.position.y
        self.position.z = odom.pose.pose.position.z
        self.robot_orientation_q = odom.pose.pose.orientation
        self.function_status = True

    def goal_position(self, position):
        goal_publisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
        mbag = MoveBaseActionGoal()

        self.goal_id = self.goal_id +1
        mbag.goal_id.id = str(self.goal_id)

        mbag.goal.target_pose.header.frame_id = "map"

        mbag.goal.target_pose.pose.position.x = position.x
        mbag.goal.target_pose.pose.position.y = position.y
        mbag.goal.target_pose.pose.position.z = position.z
        mbag.goal.target_pose.pose.orientation.w = 1.0

        rospy.sleep(1) # This delay is necessary.
        goal_publisher.publish(mbag)
        #rospy.sleep(1)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.status_callback)
        while len(self.mbag_status.status_list) < 1:
            time.sleep(1)
        while self.mbag_status.status_list[0].goal_id.id != str(self.goal_id):
            time.sleep(1) 
        while self.mbag_status.status_list[0].status == 1:
            time.sleep(1)
        
        return Status(StatusCode.OK)

    def status_callback(self, status):
        self.mbag_status = status        

    def robot_task_request(self, task_request):
        self.set_allowed_error(task_request.basic_move_task.allowed_error)
        goal_publisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
        mbag = MoveBaseActionGoal()

        self.goal_id = self.goal_id +1
        mbag.goal_id.id = str(self.goal_id)

        mbag.goal.target_pose.header.frame_id = "map"

        self.logger.info("New goal (x, y, z):  ({}, {}, {})".format(task_request.basic_move_task.positions[0].x, task_request.basic_move_task.positions[0].y, task_request.basic_move_task.positions[0].z))
        mbag.goal.target_pose.pose.position.x = task_request.basic_move_task.positions[0].x
        mbag.goal.target_pose.pose.position.y = task_request.basic_move_task.positions[0].y
        mbag.goal.target_pose.pose.position.z = task_request.basic_move_task.positions[0].z

        yaw = task_request.basic_move_task.final_orientation.yaw
        pitch = task_request.basic_move_task.final_orientation.pitch        
        roll = task_request.basic_move_task.final_orientation.roll
        robot_orientation_q = quaternion_from_euler(roll, pitch, yaw, 'sxyz')

        mbag.goal.target_pose.pose.orientation.x = robot_orientation_q[0]
        mbag.goal.target_pose.pose.orientation.y = robot_orientation_q[1]
        mbag.goal.target_pose.pose.orientation.z = robot_orientation_q[2]
        mbag.goal.target_pose.pose.orientation.w = robot_orientation_q[3]

        rospy.sleep(1)
        goal_publisher.publish(mbag)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.status_callback)
        while len(self.mbag_status.status_list) < 1:
            time.sleep(1)
        while self.mbag_status.status_list[0].goal_id.id != str(self.goal_id):
            time.sleep(1) 
        while self.mbag_status.status_list[0].status == 1:
            time.sleep(1)
        return Status(StatusCode.OK)

    def set_allowed_error(self, allowed_error):
        rospy.wait_for_service('move_base/DWAPlannerROS/set_parameters')
        try:
            self.logger.info("Changing goal position tolerance to {}".format(allowed_error))
            xy_goal_tolerance = rospy.ServiceProxy('move_base/DWAPlannerROS/set_parameters', Reconfigure)
            changes = Config()
            bool_p = BoolParameter()
            bool_p.name = 'xy_goal_tolerance'
            bool_p.value = allowed_error
            changes.doubles = [bool_p]
            service_call = xy_goal_tolerance(changes)
        except rospy.ServiceException as e:
            logger.error("Service call failed: {} \nThe change on /move_base topic to allow the allowed_error select failed.", e)