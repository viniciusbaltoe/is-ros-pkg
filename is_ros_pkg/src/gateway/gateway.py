from is_wire.core import Channel, Message, Logger, Status, StatusCode
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.robot_pb2 import RobotControllerProgress
from is_msgs.common_pb2 import FieldSelector
from google.protobuf.empty_pb2 import Empty
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
    
    def get_param(self, field_selector, ctx):
        robot_param = RobotControllerProgress()
        self.logger.info("Gateway: Requesting parameters to robot driver.")
        get_obj(self.driver.get_position, robot_param.current_pose)
        return robot_param

    '''
    def set_config(self, robot_config, ctx):
        if robot_config.HasField("speed"):
            self.driver.set_speed(robot_config.speed)
        return Empty()
    ''' 

    def run(self,broker_uri):
        service_name = "RobotGazebo.{}".format(self.driver.robot_id)

        publish_channel = Channel(broker_uri)
        
        rpc_channel = Channel(broker_uri)
        server = ServiceProvider(rpc_channel)

        logging = LogInterceptor()
        server.add_interceptor(logging)

        server.delegate(
            topic=service_name + ".GetParam",
            request_type=Empty,
            reply_type=RobotControllerProgress,
            function=self.get_param)
        '''
        server.delegate(
            topic=service_name + ".SetConfig",
            request_type=RobotConfig,
            reply_type=Empty,
            function=self.set_config)
        '''
        self.logger.info("RPC listening for requests")

        while True:
            try:
                message = rpc_channel.consume(timeout=0)
                if server.should_serve(message):
                    server.serve(message)
            except socket.timeout:
                pass