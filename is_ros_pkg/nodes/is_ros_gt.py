import json
import sys
from is_ros_pkg.src.robot_driver.ros_driver import ROS_Robot
from is_ros_pkg.src.robot_gateway.gateway import RobotGateway
from is_msgs.robot_pb2 import RobotControllerProgress


def main():
    config_file = sys.argv[1] if len(sys.argv) > 1 else '../../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    #robot_config = config['robot']

    driver = ROS_Robot(robot_config)

    service = RobotGateway(driver=driver)
    service.run(broker_uri=broker_uri)

if __name__ == "__main__":
    main()