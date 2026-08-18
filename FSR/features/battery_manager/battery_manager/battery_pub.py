#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.battery_level = 100.0
        self.drain_rate = 0.5  
        self.charge_rate = 1.0  

        self.status = 'idle'  

        self.battery_pub = self.create_publisher(Float32, '/battery_status', 10)
        self.status_sub = self.create_subscription(String, '/robot/status', self.status_callback, 10)
        self.low_battery_pub = self.create_publisher(String, '/battery/low_warning', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self._log_counter = 0
        self._low_battery_warned = False  
        self.get_logger().info("BatteryPublisher started with 100% battery")
        self.publish_battery()

    def status_callback(self, msg: String):
        self.status = msg.data
        self.get_logger().info(f"Robot status updated to: {self.status}")

    def publish_battery(self):
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

    def publish_low_battery_warning(self):
        warning_msg = String()
        warning_msg.data = f"Battery low at {self.battery_level:.1f}%!"
        self.low_battery_pub.publish(warning_msg)
        self.get_logger().warn(warning_msg.data)

    def timer_callback(self):
        if self.status in ['idle', 'navigating', 'multi_navigating', 'canceling_navigation']:
            if self.battery_level > 0:
                self.battery_level -= self.drain_rate
                self.battery_level = max(self.battery_level, 0)
        elif self.status == 'charging':
            if self.battery_level < 100:
                self.battery_level += self.charge_rate
                self.battery_level = min(self.battery_level, 100)
            else:
                self.get_logger().info(" Battery fully charged!")
                self._low_battery_warned = False  

        
        self.publish_battery()

   
        self._log_counter += 1
        if self._log_counter >= 5:
            self._log_counter = 0
            self.get_logger().info(f"Battery level: {self.battery_level:.1f}%")

            if self.battery_level <= 20.0 and self.status != 'charging' and not self._low_battery_warned:
                self.publish_low_battery_warning()
                self._low_battery_warned = True

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


