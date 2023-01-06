import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from controller_manager_msgs.srv import SwitchController

class sparke_manager(Node):
    def __init__(self):
        super().__init__("test_manager", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        client_cb_group = MutuallyExclusiveCallbackGroup()

        #create clients
        self.switch_controller_client = self.create_client(Empty, "/reset_sparke", callback_group=client_cb_group)
        self.service_ready_check(self.switch_controller_client)
        self.empty_req = Empty.Request()

    def service_ready_check(self, client):
        while not client.service_is_ready():
            self.get_logger().info("Waiting for service...")

def main(args=None):
    rclpy.init(args=args)

    manager_node = sparke_manager()

    executor = MultiThreadedExecutor()
    executor.add_node(manager_node)

    count = 0

    while rclpy.ok():
        if count == 0:
            future = manager_node.switch_controller_client.call_async(manager_node.empty_req)
            if not future.done():
                executor.spin_once()
            count +=1
            manager_node.get_logger().info("Request completed!")
        executor.spin()

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()