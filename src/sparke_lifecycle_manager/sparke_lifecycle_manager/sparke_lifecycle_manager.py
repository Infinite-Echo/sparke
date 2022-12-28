import sys

from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node


class SparkeSimManager(Node):

    def __init__(self):
        super().__init__('sparke_simulation_manager')
        self.cli = self.create_client(Empty, 'reset_sparke_simulation')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        self.future = self.cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Simulation has been reset!")


def main():
    rclpy.init()

    sim_manager = SparkeSimManager()
    response = sim_manager.send_request()

    sim_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()