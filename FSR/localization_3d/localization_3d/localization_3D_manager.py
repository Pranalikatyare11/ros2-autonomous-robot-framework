#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import subprocess
import os
import signal
import time


class MappingManager(Node):

    def __init__(self):
        super().__init__('localization_3D_manager')
        self.get_logger().info("localization_3D Manager node started, waiting for service calls...")

        self.mapping_process = None
        self.emergency_active = False

        # Subscribe to emergency stop topic
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # Service to start/stop localization
        self.srv = self.create_service(
            SetBool,
            'localization_3D_control',
            self.mapping_callback
        )

    def emergency_callback(self, msg: Bool):
        self.emergency_active = msg.data

        if self.emergency_active:
            self.get_logger().warn("EMERGENCY STOP ACTIVE! Localization is frozen.")
            # DO NOT stop localization_3D here
        else:
            self.get_logger().info("Emergency stop released. Localization control allowed.")

    def mapping_callback(self, request, response):

        # User tries to START localization
        if request.data:

            if self.emergency_active:
                response.success = False
                response.message = "Cannot start localization_3D — Emergency Stop is active!"
                return response

            if self.mapping_process is not None and self.mapping_process.poll() is None:
                response.success = False
                response.message = "localization_3D already running!"
                return response

            self.get_logger().info("Starting localization_3D launch file...")

            self.mapping_process = subprocess.Popen(
                ["ros2", "launch", "localization_3d", "3D.localization.launch.py"],
                preexec_fn=os.setsid
            )

            response.success = True
            response.message = "localization_3D started."
            return response

        # User tries to STOP localization
        else:

            if self.emergency_active:
                response.success = False
                response.message = "Cannot stop localization_3D — Emergency Stop is active!"
                return response

            self.get_logger().info("Stopping localization_3D launch...")

            if self.mapping_process is not None and self.mapping_process.poll() is None:
                try:
                    os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGINT)
                    self.mapping_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().info("Force killing localization_3D launch...")
                    os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGKILL)

                self.mapping_process = None

            response.success = True
            response.message = "localization_3D stopped."
            return response


def main(args=None):
    rclpy.init(args=args)
    node = MappingManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mapping_process is not None and node.mapping_process.poll() is None:
            node.get_logger().info("Shutting down localization_3D launch on node exit...")
            try:
                os.killpg(os.getpgid(node.mapping_process.pid), signal.SIGINT)
                node.mapping_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                node.get_logger().info("Force killing localization_3D launch on node exit...")
                os.killpg(os.getpgid(node.mapping_process.pid), signal.SIGKILL)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
