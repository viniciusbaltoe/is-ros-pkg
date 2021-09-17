#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    goal = PoseStamped()

    #goal.header.seq = 1
    #goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = 1.5
    goal.pose.position.y = -0.5
    goal.pose.position.z = 0.0

    #goal.pose.orientation.x = 0.0
    #goal.pose.orientation.y = 0.0
    #goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.sleep(0.1)
    goal_publisher.publish(goal)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass