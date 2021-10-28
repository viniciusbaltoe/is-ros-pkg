from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply
from is_wire.core import Channel, Subscription, Message
from google.protobuf.empty_pb2 import Empty
from is_msgs.common_pb2 import Position
import json
import socket
import time

if __name__ == "__main__":
# -------------------------- Options ------------------------- 
    cont = True
    config_file = '../etc/conf/config.json'
    config = json.load(open(config_file, 'r'))    
    channel = Channel(config["broker_uri"])
    robot_config = config["robot"]
    subscription = Subscription(channel)

# ---------------------- Get Position ------------------------
    topic = "ROSRobot.{}.GetPosition".format(robot_config["robot_id"])
    print("Publishing to topic: {}".format(topic))
    request = Empty()
    channel.publish(
        Message(content=request, reply_to=subscription),
        topic=topic)
    try:
        reply = channel.consume(timeout=3.0)
        unpacked_msg = reply.unpack(Position)
        print('Position:\n', unpacked_msg)
    except socket.timeout:
        print('No reply to Get 1 :(')

# ---------------------- Set Goal Position -------------------
    topic = "ROSRobot.{}.RobotTask".format(robot_config["robot_id"])
    print("Publishing to topic: {}".format(topic))

    request = RobotTaskRequest()
    request.id = 10
    request.basic_move_task.positions.extend([Position(x=1.5, y=1.5, z=0)])
    request.basic_move_task.final_orientation.yaw = 1.0
    request.basic_move_task.final_orientation.pitch = 0.0
    request.basic_move_task.final_orientation.roll = 0.0
    request.basic_move_task.allowed_error = 0.15
    request.basic_move_task.rate = 1 # It doesn't matter here.

    channel.publish(
        Message(content=request, reply_to=subscription),
        topic=topic)
    try:
        reply = channel.consume()
        unpacked_msg = reply.unpack(RobotTaskReply)
        print('Task ID: {} \nRPC Status: {}'.format(unpacked_msg.id, reply.status))
    except socket.timeout:
        print('No reply to RobotTaskRequest :(')

# ---------------------- Get Position ------------------------
    topic = "ROSRobot.{}.GetPosition".format(robot_config["robot_id"])
    print("Publishing to topic: {}".format(topic))
    request = Empty()
    channel.publish(
        Message(content=request, reply_to=subscription),
        topic=topic)
    try:
        reply = channel.consume(timeout=3.0)
        unpacked_msg = reply.unpack(Position)
        print('Position:\n', unpacked_msg)
    except socket.timeout:
        print('No reply :(')