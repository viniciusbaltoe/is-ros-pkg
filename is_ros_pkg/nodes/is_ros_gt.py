#!/usr/bin/env python3
from ros_driver.driver import ROS_Robot
from gateway.gateway import RobotGateway
from is_msgs.common_pb2 import Position
import json
import sys

def main():
    config_file = sys.argv[1] if len(sys.argv) > 1 else '../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    robot_config = config['robot']

    driver = ROS_Robot(robot_config)

    service = RobotGateway(driver=driver)
    service.run(broker_uri=broker_uri)

if __name__ == "__main__":
    main()