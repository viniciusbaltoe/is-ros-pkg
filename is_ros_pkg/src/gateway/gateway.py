from is_wire.core import Channel, Message, Logger, Status, StatusCode
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.robot_pb2 import RobotTaskRequest
from google.protobuf.empty_pb2 import Empty
#from is_msgs.common_pb2 import FieldSelector
import socket

def get_obj(callable, obj):
    value = callable()
    if value is not None:
        obj.CopyFrom(value)

def get_val(callable, obj, attr):
    value = callable()
    if value is not None:
        setattr(obj, attr, value)

class RobotGateway(object):
    def __init__(self, driver):
        self.driver = driver
        self.logger = Logger("RobotGateway")

    def task_request(self, task, ctx):
        return self.driver.new_task(task)
    
    def get_position(self, ctx):
        position = Position()
        get_obj(self.driver.get_position, position)
        return position

    def call_position(self, position, ctx):
        maybe_ok = self.driver.call_position(position)
        if maybe_ok.code != StatusCode.OK:
            return maybe_ok
        return Status(StatusCode.OK)

    def run(self,broker_uri):
        service_name = "ROSRobot.{}".format(self.driver.robot_id)

        publish_channel = Channel(broker_uri)
        rpc_channel = Channel(broker_uri)
        server = ServiceProvider(rpc_channel)
        logging = LogInterceptor()
        server.add_interceptor(logging)

        server.delegate(
            topic=service_name + ".GetPosition",
            request_type=Empty,
            reply_type=Position,
            function=self.task_request)

        server.delegate(
            topic=service_name + ".CallPosition",
            request_type=Position,
            reply_type=Empty,
            function=self.task_request)
        
        self.logger.info("RPC listening for requests")
        while True:
            try:
                message = rpc_channel.consume(timeout=0)
                if server.should_serve(message):
                    server.serve(message)
            except socket.timeout:
                pass