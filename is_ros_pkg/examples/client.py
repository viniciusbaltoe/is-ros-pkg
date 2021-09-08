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
        print('RPC Status:', reply.status, '\nReply:', unpacked_msg)
    except socket.timeout:
        print('No reply :(')