import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration

class sparke_manager(Node):
    def __init__(self):
        super().__init__("sparke_reset_manager", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        client_cb_group = MutuallyExclusiveCallbackGroup()
        service_cb_group = MutuallyExclusiveCallbackGroup()

        #create service
        self.reset_sparke_service = self.create_service(Empty, "/reset_sparke", self.reset_sparke_cb, callback_group=service_cb_group)

        #create clients
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=client_cb_group)
        self.reset_gazebo_client = self.create_client(Empty, "/reset_simulation", callback_group=client_cb_group)

        self.stop_controllers_req = SwitchController.Request()
        self.start_controllers_req = SwitchController.Request()
        self.empty_req = Empty.Request()
        self.init_msgs()
        self.reset_called = False

    def reset_sparke_cb(self, req, res):
        #Stop Controllers
        self.service_ready_check(self.switch_controller_client)
        stop_future = self.switch_controller_client.call_async(self.stop_controllers_req)
        while not stop_future.done():
            self.get_logger().info("Waiting on Future 1...")
        self.future_check(stop_future)
        self.get_logger().info("Controllers Stopped Successfully.")

        #Reset Gazebo
        self.service_ready_check(self.reset_gazebo_client)
        reset_future = self.reset_gazebo_client.call_async(self.empty_req)
        while not reset_future.done():
            self.get_logger().info("Waiting on Future 2...")
        self.future_check(reset_future)
        self.get_logger().info("Gazebo Reset Successfully.")

        #Start Controllers
        self.service_ready_check(self.switch_controller_client)
        start_future = self.switch_controller_client.call_async(self.start_controllers_req)
        while not start_future.done():
            self.get_logger().info("Waiting on Future 3...")
        self.future_check(start_future)
        self.get_logger().info("Controller Started Successfully.")

        #Confirm Program Reached This Point
        self.get_logger().info("If you see this then the future result should have been printed already.")
        res = Empty.Response()
        return res

    def service_ready_check(self, client):
        while not client.service_is_ready():
            self.get_logger().info("Waiting for service...")

    def future_check(self, future_obj):
        if future_obj.done():
            self.get_logger().info(f"Future Result: {future_obj.result()}")
        else: 
            self.get_logger().info("Future never finished.")

    def init_msgs(self):
        self.stop_controllers_req.stop_controllers = ["joint_group_effort_controller"]
        self.stop_controllers_req.strictness = 1
        self.start_controllers_req.start_controllers = ["joint_group_effort_controller"]
        self.start_controllers_req.strictness = 1
        # self.start_controllers_req.timeout.nanosec = 0
        # self.start_controllers_req.start_asap = True

def main(args=None):
    rclpy.init(args=args)

    manager_node = sparke_manager()

    executor = MultiThreadedExecutor()
    executor.add_node(manager_node)

    while rclpy.ok():
        executor.spin()

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()