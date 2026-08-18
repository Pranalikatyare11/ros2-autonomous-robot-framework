#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class RobotStateManager(Node):
    def __init__(self):
        super().__init__('robot_state_manager')

        self.mapping_active = False
        self.navigation_active = False
        self.emergency_active = False
        self.current_state = "IDLE"

        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        self.create_subscription(Bool, '/mapping_active', self.mapping_active_callback, 10)
        self.create_subscription(Bool, '/navigation_active', self.navigation_active_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.timer = self.create_timer(1.0, self.publish_state)

        self.get_logger().info("Robot State Manager started with state IDLE")

    def mapping_active_callback(self, msg: Bool):
        self.mapping_active = msg.data
        self.update_state()

    def navigation_active_callback(self, msg: Bool):
        self.navigation_active = msg.data
        self.update_state()

    def emergency_callback(self, msg: Bool):
        self.emergency_active = msg.data
        self.update_state()

    def update_state(self):
        previous_state = self.current_state

        # EMERGENCY has highest priority
        if self.emergency_active:
            self.current_state = "EMERGENCY"
        elif self.mapping_active:
            self.current_state = "MAPPING"
        elif self.navigation_active:
            self.current_state = "NAVIGATING"
        else:
            self.current_state = "IDLE"

        if previous_state != self.current_state:
            self.get_logger().info(f"Robot state changed from {previous_state} to {self.current_state}")

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
