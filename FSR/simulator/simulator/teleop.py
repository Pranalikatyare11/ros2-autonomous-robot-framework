import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import termios
import tty
import select

instructions = """
---------------------------
Manual Teleoperation
---------------------------
Use the following keys to move the robot:
    w - Move forward
    s - Stop
    a - Turn left
    d - Turn right
    x - Move backward   
    Ctrl+C - Exit
---------------------------
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop_node')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.Parameter.Type.BOOL, True)
        ])

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)

        self.emergency_active = False
        self.speed = 0.4
        self.turn = 1.0
        self.settings = termios.tcgetattr(sys.stdin)
        self.last_action = None  

        print(instructions, flush=True)
        self.get_logger().info("Custom Teleop Node Started")

    def emergency_callback(self, msg):
        previously_active = self.emergency_active
        self.emergency_active = msg.data

        if self.emergency_active and not previously_active:
            self.get_logger().warn("Emergency Stop Activated! Teleop disabled.")
            self.publisher.publish(Twist())
            self.last_action = None  

        elif not self.emergency_active and previously_active:
            self.get_logger().info("Emergency Stop Released. Teleop enabled again.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                key = self.get_key()
                twist = Twist()

                if self.emergency_active:
                    continue

                action = None

                if key == 'w':
                    twist.linear.x = self.speed
                    action = "forward"
                elif key == 'x':
                    twist.linear.x = -self.speed
                    action = "backward"
                elif key == 'a':
                    twist.angular.z = self.turn
                    action = "left"
                elif key == 'd':
                    twist.angular.z = -self.turn
                    action = "right"
                elif key == 's':
                    twist = Twist()
                    action = "stop"
                elif key == '\x03':
                    print("\nExiting teleop...", flush=True)
                    break
                else:
                    continue

       
                if action != self.last_action:
                    print(f"Robot moving: {action}", flush=True)
                    self.last_action = action

                self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

        finally:
            self.publisher.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Robot stopped. Goodbye!", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
