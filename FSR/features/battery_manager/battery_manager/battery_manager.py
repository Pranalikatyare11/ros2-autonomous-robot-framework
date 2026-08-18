#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from opennav_docking_msgs.action import DockRobot, UndockRobot
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

class BatteryDockingManager(Node):
    def __init__(self):
        super().__init__('battery_docking_manager')

        # Subscribers
        self.battery_sub = self.create_subscription(Float32, '/battery_status', self.battery_callback, 10)

        # Publisher to robot status
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # Action Clients
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.dock_client = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, 'undock_robot')

        # Dock info (consider loading from parameter or YAML later)
        self.dock_pose = [-4.48, -5.84, 0.0]
        self.battery_threshold = 20.0

        # State variables
        self.low_battery_triggered = False
        self.navigating_to_dock = False
        self.charging = False

        # Track navigation activity
        self.navigate_to_pose_active = False
        self.navigate_through_poses_active = False

        # Cancellation tracking
        self.cancel_futures = []
        self.cancel_completed_count = 0

        # Latest battery level
        self.latest_battery = 100.0

        self.get_logger().info("BatteryDockingManager started")

    def battery_callback(self, msg: Float32):
        battery_level = msg.data
        self.latest_battery = battery_level
        self.get_logger().info(f"Battery: {battery_level:.1f}%")

        if battery_level <= self.battery_threshold and not self.low_battery_triggered:
            self.get_logger().warn(f"Battery low ({battery_level:.1f}%). Initiating docking procedure.")
            self.low_battery_triggered = True
            self.start_docking_procedure()

    def start_docking_procedure(self):
        self.publish_status('canceling_navigation')
        self.cancel_all_goals()

    def cancel_all_goals(self):
        self.cancel_futures.clear()
        self.cancel_completed_count = 0
        canceled_any = False

        if self.navigate_to_pose_active and self.navigate_to_pose_client.server_is_ready():
            self.get_logger().info("Canceling active navigate_to_pose goal")
            cancel_future = self.navigate_to_pose_client._client.cancel_all_goals()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.cancel_futures.append(cancel_future)
            canceled_any = True
        else:
            self.get_logger().info("No active navigate_to_pose goals or server not ready")
            self.cancel_completed_count += 1

        if self.navigate_through_poses_active and self.navigate_through_poses_client.server_is_ready():
            self.get_logger().info("Canceling active navigate_through_poses goal")
            cancel_future = self.navigate_through_poses_client._client.cancel_all_goals()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.cancel_futures.append(cancel_future)
            canceled_any = True
        else:
            self.get_logger().info("No active navigate_through_poses goals or server not ready")
            self.cancel_completed_count += 1

        if not canceled_any:
            self.get_logger().info("No active goals to cancel, proceeding to dock")
            self.navigate_to_dock()

    def cancel_done_callback(self, future):
        self.get_logger().info("Cancel request confirmed for one action server")
        self.cancel_completed_count += 1

        if self.cancel_completed_count >= 2:
            self.get_logger().info("All cancel requests confirmed, proceeding to dock")
            self.create_timer(1.0, self.navigate_to_dock)

    def navigate_to_dock(self):
        if self.navigating_to_dock:
            return  # Already navigating

        self.get_logger().info("Publishing status 'docking'")
        self.publish_status('docking')
        self.navigating_to_dock = True

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.dock_pose[0]
        goal_pose.pose.position.y = self.dock_pose[1]
        goal_pose.pose.position.z = self.dock_pose[2]
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.navigate_to_pose_client.wait_for_server()
        send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_dock_feedback)
        send_goal_future.add_done_callback(self.navigate_to_dock_response_callback)

    def navigate_to_dock_feedback(self, feedback_msg):
        pass  # Optional feedback handling

    def navigate_to_dock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Docking navigation goal rejected, retrying in 5 seconds...")
            self.navigating_to_dock = False
            self.create_timer(5.0, self.navigate_to_dock)
            return

        self.get_logger().info("Docking navigation goal accepted.")
        self.navigate_to_pose_active = True

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_navigation_result_callback)

    def dock_navigation_result_callback(self, future):
        status = future.result().status

        self.navigate_to_pose_active = False
        self.navigating_to_dock = False

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Robot arrived at dock location.")
            self.start_docking()
        else:
            self.get_logger().warn(f"Docking navigation failed with status {status}. Retrying...")
            self.create_timer(5.0, self.navigate_to_dock)

    def start_docking(self):
        self.get_logger().info("Starting docking process (robot will start charging)...")
        self.publish_status('charging')

        self.dock_client.wait_for_server()
        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = "flex_dock1"  

        send_goal_future = self.dock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.docking_result_callback)

    def docking_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Docking action rejected, retrying in 5 seconds...")
            self.create_timer(5.0, self.start_docking)
            return

        self.get_logger().info("Docking action accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.docking_complete_callback)

    def docking_complete_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info("Docking complete. Robot is charging.")
            self.charging = True
            self.monitor_charging()
        else:
            self.get_logger().warn(f"Docking failed with status {status}. Retrying docking...")
            self.create_timer(5.0, self.start_docking)

    def monitor_charging(self):
        self.get_logger().info("Monitoring battery charging...")

        def charging_check():
            if self.latest_battery >= 100.0:
                self.get_logger().info("Battery full. Starting undocking.")
                self.charging = False
                self.start_undocking()
                return False
            return True

        def timer_callback():
            if not charging_check():
                timer.cancel()

        timer = self.create_timer(1.0, timer_callback)

    def start_undocking(self):
        self.get_logger().info("Starting undocking process...")
        self.publish_status('undocking')

        self.undock_client.wait_for_server()
        goal_msg = UndockRobot.Goal()

        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undocking_result_callback)

    def undocking_result_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Undocking action rejected, retrying in 5 seconds...")
            self.create_timer(5.0, self.start_undocking)
            return

        self.get_logger().info("Undocking action accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undocking_complete_callback)

    def undocking_complete_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info("Undocking complete. Robot ready for normal operation.")
            self.publish_status('idle')
            self.low_battery_triggered = False
        else:
            self.get_logger().warn(f"Undocking failed with status {status}. Retrying undocking...")
            self.create_timer(5.0, self.start_undocking)

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"Published robot status: {status_str}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryDockingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
