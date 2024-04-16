from tutorial_interfaces.srv import Ros2BytesTransmit

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