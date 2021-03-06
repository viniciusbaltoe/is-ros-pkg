#!/usr/bin/env python3
from ros_driver.driver import ROS_Robot
from gateway.gateway import RobotGateway
from is_msgs.common_pb2 import Position
import os.path
import json
import rospy

def main():
    rospy.init_node('is_ros_gt_node')

    homedir = os.path.expanduser("~")
    config_file = '{}/catkin_ws/src/is-ros-pkg/is_ros_pkg/etc/conf/config.json'.format(homedir)
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    robot_config = config['robot']

    driver = ROS_Robot(robot_config)

    service = RobotGateway(driver=driver)
    service.run(broker_uri=broker_uri)

if __name__ == "__main__":
    main()