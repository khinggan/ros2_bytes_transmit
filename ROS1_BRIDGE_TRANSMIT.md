# How to bytes transmit using ros1_bridge package? (ROS1 <-> ROS2)

## Introduction
This is a tutorial for how to transmit bytes data between ROS1 and ROS2 node using client server mode. `ros1_bridge` package is used to transmit data between ROS1 and ROS2.

This tutorial refers from [custom_msg tutorial](https://www.youtube.com/watch?v=vBlUFIOHEIo) and [ros1_bridge foxy branch official tutorial](https://github.com/ros2/ros1_bridge/tree/foxy).

### File Structure
Three workspace is need: ros1_ws, ros2_ws and bridge_ws.
```
# ROS1 workspace
ros1_ws/src/custom_srv_ros1/
├── CMakeLists.txt
├── include
│   └── custom_srv_ros1
├── msg
├── package.xml
├── scripts
│   ├── ros1_client.py
│   └── ros1_server.py
├── src
└── srv
    └── Ros2BytesTransmit.srv

# ROS2 workspace
ros2_ws/src/custom_srv_ros2/
├── CMakeLists.txt
├── custom_srv_ros2
│   └── __init__.py
├── include
│   └── custom_srv_ros2
├── my_bridge_mapping2.yaml
├── package.xml
├── scripts
│   ├── ros2_client.py
│   └── ros2_server.py
├── src
└── srv
    └── Ros2BytesTransmit.srv

# bridge workspace
bridge_ws/src/ros1_bridge/
...
```
### Codes
Ros2BytesTransmit.srv
```
uint8[] req
---
bool resp
```
Custom interface must be same in ROS1 and ROS2 packages.

Noticed that the data type is `uint8`, it is because ROS1 don't have `byte` built-in type.
Refer to [this](http://wiki.ros.org/msg) and [this](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#message-description-specification) 


ROS1 server and client code are refer from [this](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) 
ros1_server.py
```python
#!/usr/bin/env python3
from __future__ import print_function
import rospy
from custom_srv_ros1.srv import Ros2BytesTransmit, Ros2BytesTransmitRequest, Ros2BytesTransmitResponse

def bytes_transmit(request):
    rospy.loginfo("Incoming request is %s", request)
    response = Ros2BytesTransmitResponse(True if request.req else False)

    rospy.loginfo("Response request is %s", request)
    return response

def bytes_transmit_server():
    rospy.init_node('bytes_service')
    srv = rospy.Service('ros2_bytes_transmit', Ros2BytesTransmit, bytes_transmit)
    print("Ready to receive BYTES data")
    rospy.spin()

if __name__ == '__main__':
    bytes_transmit_server()

```

ros1_client.py
```python
#!/usr/bin/env python3

from __future__ import print_function

import rospy
from custom_srv_ros1.srv import Ros2BytesTransmit,Ros2BytesTransmitRequest, Ros2BytesTransmitResponse

absolute_path = "/home/khinggan/coding/colcon_ws/src/ros2_bytes_transmit/"

def send_bytes():
    rospy.wait_for_service('ros2_bytes_transmit')
    
    pickle_path = absolute_path + "example.pkl"
    with open(pickle_path, 'rb') as file:
        loaded_data = file.read()
    loaded_data = [b for b in loaded_data]  # byte []
    print(type(loaded_data[0]))
    
    req = Ros2BytesTransmitRequest()
    req.req = loaded_data
    rospy.loginfo("The request data is: %s", req)

    try:
        client = rospy.ServiceProxy('ros2_bytes_transmit', Ros2BytesTransmit)
        resp = client(req)
        print(resp)
        rospy.loginfo("The response data is: %s", resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    try:
        send_bytes()
    except rospy.ROSInterruptException:
        pass

```

Configure ROS1's the `CMakeLists.txt` and `package.xml` according to the [custom msg and srv of ROS1](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

ros2_server.py
```python
#!/usr/bin/env python3
from custom_srv_ros2.srv import Ros2BytesTransmit

import rclpy
from rclpy.node import Node


class BytesService(Node):

    def __init__(self):
        super().__init__('bytes_service')
        self.srv = self.create_service(Ros2BytesTransmit, 'ros2_bytes_transmit', self.bytes_transmit)

    
    def bytes_transmit(self, request, response):
        response.resp = True if request.req is not None else False

        self.get_logger().info("Income request is {}".format(request))

        return response

def main(args=None):
    rclpy.init(args=args)

    bytes_service = BytesService()

    rclpy.spin(bytes_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

ros2_client.py
```python
#!/usr/bin/env python3
from custom_srv_ros2.srv import Ros2BytesTransmit
import sys
import rclpy
from rclpy.node import Node
import pickle
import os

absolute_path = "/home/khinggan/coding/colcon_ws/src/ros2_bytes_transmit/"

class BytesClientAsync(Node):

    def __init__(self):
        super().__init__('bytes_client_async')
        self.cli = self.create_client(Ros2BytesTransmit, 'ros2_bytes_transmit')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Ros2BytesTransmit.Request() 
    
    def send_bytes(self):
        pickle_path = absolute_path + "example.pkl"
        with open(pickle_path, 'rb') as file:
            loaded_data = file.read()
        loaded_data = [b for b in loaded_data]   # uint8 []; if you want to use byte, b.to_bytes(4, 'big')
        print(type(loaded_data[0]))
        self.req.req = loaded_data
        self.get_logger().info("The request data is: {}".format(self.req.req))
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    bytes_client = BytesClientAsync()
    bytes_client.send_bytes()

    while rclpy.ok():
        rclpy.spin_once(bytes_client)
        if bytes_client.future.done():
            try:
                response = bytes_client.future.result()
            except Exception as e:
                bytes_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                bytes_client.get_logger().info(
                    "result is: {}".format(response.resp)
                )
            break

    bytes_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Configure ROS2's `CMakeLists.txt` and `package.xml` according to [this link](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

### Configure the bridge
In ROS2 package, new a new yaml file in root directory.
According to the [tutorial](https://www.youtube.com/watch?v=vBlUFIOHEIo), config the all files. 

## Getting Started

### Build
Build ROS1 Package
```bash
# In ros1_ws
catkin_make
```

Build ROS2 Package
```bash
# In ros2_ws
colcon_build
```

Build bridge
```bash
# In bridge_ws
colcon build --symlink-install --cmake-force-configure
```

after build bridge, check the custom service is bind or not
```bash
# In bridge_ws
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep custom
```

### Run
> same with https://github.com/ros2/ros1_bridge/tree/foxy?tab=readme-ov-file#example-3-run-the-bridge-for-addtwoints-service

```bash
# Terminal 1
source <ros-install-dir>/setup.bash
roscore

# Terminal 2
source <ros-install-dir>/setup.bash
source <ros2-install-dir>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

# Terminal 3
source <ros-install-dir>/setup.bash
export ROS_MASTER_URI=http://localhost:11311
rosrun custom_srv_ros1 ros1_server.py

# Terminal 4
source <ros2-install-dir>/setup.bash
ros2 run custom_srv_ros2 ros2_client.py

# or ros1 client, ros2 server...
```