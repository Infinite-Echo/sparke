import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from controller_manager_msgs.srv import SwitchController

def service_ready_check(node, client, executor):
    while not client.service_is_ready():
        node.get_logger().info("Waiting for service...")
        executor.spin_once(timeout_sec=0.1)

def future_check(node, future_obj):
    if future_obj.done():
        node.get_logger().info(f"Future Result: {future_obj.result()}")
    else: 
        node.get_logger().info("Future never finished.")

def main(args=None):
    rclpy.init(args=args)
    client_node = rclpy.create_node("sparke_reset_manager", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    client_cb_group = None

    #create clients
    switch_client = client_node.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=client_cb_group)
    reset_client = client_node.create_client(Empty, "/reset_simulation", callback_group=client_cb_group)

    loop_count = 0

    #create request messages
    stop_controllers_req = SwitchController.Request()
    stop_controllers_req.stop_controllers = ["joint_states_controller", "joint_group_effort_controller"]
    stop_controllers_req.strictness = 1
    start_controllers_req = SwitchController.Request()
    start_controllers_req.start_controllers = ["joint_states_controller", "joint_group_effort_controller"]
    start_controllers_req.strictness = 1
    empty_req = Empty.Request()

    executor = MultiThreadedExecutor()
    executor.add_node(client_node)

    while rclpy.ok():
        if not loop_count >= 1:
            #Stop Controllers
            service_ready_check(client_node, switch_client, executor)
            stop_future = switch_client.call_async(stop_controllers_req)
            while not stop_future.done():
                executor.spin_once(timeout_sec=1.0)
            future_check(client_node, stop_future)
            client_node.get_logger().info("Controllers Stopped Successfully.")

            #Pause Gazebo
            service_ready_check(client_node, reset_client, executor)
            reset_future = reset_client.call_async(empty_req)
            while not reset_future.done():
                executor.spin_once(timeout_sec=1.0)
            future_check(client_node, reset_future)
            client_node.get_logger().info("Gazebo Reset Successfully.")

            #Start Controllers
            service_ready_check(client_node, switch_client, executor)
            start_future = switch_client.call_async(start_controllers_req)
            while not start_future.done():
                executor.spin_once(timeout_sec=1.0)
            future_check(client_node, start_future)
            client_node.get_logger().info("Controllers Started Successfully.")

            #Confirm Program Reached This Point
            client_node.get_logger().info("If you see this then the future result should have been printed already.")
            loop_count += 1

    rclpy.shutdown()

if __name__ == '__main__':
    main()