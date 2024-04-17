#!/usr/bin/env python3
from tutorial_interfaces.srv import Ros2BytesTransmit       # CHANGE
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
        loaded_data = [b.to_bytes(4, 'big') for b in loaded_data]   # byte []
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