# is-ros-pkg
ROS package that contains Codes for integration between Intelligent Spaces (from [UFES](https://github.com/labviros) or IFES Guarapari) with the Robot Operating System (ROS), so that the resources of both platforms can be used together.

## Status

The codes made so far only serve to read the real position of a robot and send it a list of the different positions or destinations to go.

The simple application elaborated serves as an example for discussing the subject in project meetings.

## How it works

The ROS package contains the [is-wire-py](https://github.com/labviros/is-wire-py) and [is-msgs](https://github.com/labviros/is-msgs) libraries, that communicate the ROS internal environment with the IS from the IS message exchange structure itself. In this way, the robot remains independent of the IS, as it can continue operating in its absence, and once connected to the IS, the robot can receive stimulus from outside of it, working similarly as a service (with only one container) of the Intelligent Space.

### Schematic

![schematic](./is_ros_pkg/etc/schematic.png)

## Usage

Clone this repository to your catkin workspace and make it.

### Prepare environment

Install the requirements:

```shell
pip3 install -r requirements.txt
```

This package uses other packages for its functioning, consider installing them to avoid problems.

```shell
sudo apt-get install -y ros-noetic-map-server ros-noetic-amcl ros-noetic-move-base ros-noetic-dwa-local-planner
```

In order to send/receive messages an amqp broker is necessary, to create one simply run:

```shell
docker run -d --rm -p 5672:5672 -p 15672:15672 rabbitmq:3.7.6-management
```

### Start the gateway

```shell
roslaunch is-ros-pkg gateway.launch
```

### Examples
For gateway testing, three examples provided in the repository can be used.

1. [Client](./is_ros_pkg/examples/client.py)
2. [Interactive Client](./is_ros_pkg/examples/interactive_client.py)
3. [New task](./is_ros_pkg/examples/new_task.py) - It uses the standard Message from is-msgs.

To do so, from /examples simply run:

```shell
python3 client.py
```
