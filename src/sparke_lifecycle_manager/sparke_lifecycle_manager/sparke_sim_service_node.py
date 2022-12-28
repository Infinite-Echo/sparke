import sys
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('sparke_simulation_service_node')
        self.srv = self.create_service(Empty, 'reset_sparke_sim', self.reset_sparke_sim_callback)
        self.init_clients()
        
        self.req = Empty.Request()

    def init_clients(self):
        self.reset_cli = self.create_client(Empty, '/reset_simulation')
        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pause_cli = self.create_client(Empty, '/pause_physics')
        while not self.pause_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.unpause_cli = self.create_client(Empty, '/unpause_physics')
        while not self.unpause_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def pause_gazebo(self):
        self.get_logger().info('Pausing Gazebo...')
        self.pause_cli.call_async(self.req)

    def reset_gazebo(self):
        self.get_logger().info('Resetting Gazebo...')
        self.reset_cli.call_async(self.req)

    def unpause_gazebo(self):
        self.get_logger().info('Unpausing Gazebo...')
        self.unpause_cli.call_async(self.req)

    def reset_sparke_sim_callback(self, request, response):
        self.pause_gazebo()
        self.reset_gazebo()
        self.unpause_gazebo()
        self.get_logger().info("this worked")
        return Empty.Response()

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()