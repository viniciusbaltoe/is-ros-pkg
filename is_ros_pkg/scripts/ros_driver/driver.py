# Python
import time
import json
import math

# IS
from is_wire.core import Logger, Status ,StatusCode, logger
from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply 
from is_msgs.common_pb2 import Position

# ROS
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray


def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

def euler_to_quaternion(yaw, pitch, roll): # Verificar para ver se está correto.
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

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
        (roll, pitch, yaw) = euler_from_quaternion(self.robot_orientation_q.x, self.robot_orientation_q.y, self.robot_orientation_q.z, self.robot_orientation_q.w)
        
        print((roll, pitch, yaw))
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

        #mbag.goal.target_pose.header.seq = 1
        #mbag.goal.target_pose.header.stamp = rospy.Time.now()
        mbag.goal.target_pose.header.frame_id = "map"

        mbag.goal.target_pose.pose.position.x = position.x
        mbag.goal.target_pose.pose.position.y = position.y
        mbag.goal.target_pose.pose.position.z = position.z

        #mbag.goal.target_pose.pose.orientation.x = 0.0
        #mbag.goal.target_pose.pose.orientation.y = 0.0
        #mbag.goal.target_pose.pose.orientation.z = 0.0
        mbag.goal.target_pose.pose.orientation.w = -1.0

        rospy.sleep(1) # This delay is necessary.
        goal_publisher.publish(mbag)
        #rospy.sleep(1)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.status_callback)
        while len(self.mbag_status.status_list) < 1:
            time.sleep(1)
        while self.mbag_status.status_list[0].goal_id.id != str(self.goal_id):
            time.sleep(1) 
        while self.mbag_status.status_list[0].status is 1:
            time.sleep(1)
        
        return Status(StatusCode.OK)

    def status_callback(self, status):
        self.mbag_status = status        

    def robot_task_request(self, task_request):
        goal_publisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
        mbag = MoveBaseActionGoal()

        
        self.goal_id = self.goal_id +1
        mbag.goal_id.id = str(self.goal_id)

        #mbag.goal.target_pose.header.seq = 1
        #mbag.goal.target_pose.header.stamp = rospy.Time.now()
        mbag.goal.target_pose.header.frame_id = "map"

        mbag.goal.target_pose.pose.position.x = task_request.basic_move_task.positions[0].x
        mbag.goal.target_pose.pose.position.y = task_request.basic_move_task.positions[0].y
        mbag.goal.target_pose.pose.position.z = task_request.basic_move_task.positions[0].z

        #mbag.goal.target_pose.pose.orientation.x = 0.0
        #mbag.goal.target_pose.pose.orientation.y = 0.0
        #mbag.goal.target_pose.pose.orientation.z = 0.0
        mbag.goal.target_pose.pose.orientation.w = 1.0

        rospy.sleep(1) # This delay is necessary.
        goal_publisher.publish(mbag)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.status_callback)
        while len(self.mbag_status.status_list) < 1:
            time.sleep(1)
        while self.mbag_status.status_list[0].goal_id.id != str(self.goal_id):
            time.sleep(1) 
        while self.mbag_status.status_list[0].status is 1:
            time.sleep(1)
        
        return Status(StatusCode.OK)