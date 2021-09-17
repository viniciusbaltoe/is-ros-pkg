#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal

def talker():
    goal_publisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    mbag = MoveBaseActionGoal()

    mbag.goal_id.id = "1"

    #mbag.goal.target_pose.header.seq = 1
    #mbag.goal.target_pose.header.stamp = rospy.Time.now()
    mbag.goal.target_pose.header.frame_id = "map"

    mbag.goal.target_pose.pose.position.x = 1.5
    mbag.goal.target_pose.pose.position.y = -1.5
    mbag.goal.target_pose.pose.position.z = 0.0

    #mbag.goal.target_pose.pose.orientation.x = 0.0
    #mbag.goal.target_pose.pose.orientation.y = 0.0
    #mbag.goal.target_pose.pose.orientation.z = 0.0
    mbag.goal.target_pose.pose.orientation.w = 1.0

    rospy.sleep(0.1)
    goal_publisher.publish(mbag)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass