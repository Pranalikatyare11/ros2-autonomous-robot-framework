#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import os
import yaml
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
import threading
import sys

class PathNavigationManager(Node):
    def __init__(self):
        super().__init__('path_navigation_manager')

        self.paths_dir = os.path.join(os.path.expanduser('~'), '/home/panu/FSR_WS/src/FSR/features/waypoint_recorder/points')

        self.load_paths_srv = self.create_service(Trigger, '/load_paths', self.load_paths_callback)
        self.stop_navigation_srv = self.create_service(Trigger, '/stop_navigation', self.stop_navigation_callback)

        self.marker_pub = self.create_publisher(MarkerArray, '/navigator/markers', 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

        self.loaded_points = {}
        self.selected_points = []
        self.selected_mode = None

        self.emergency_active = False  

        self.e_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.e_stop_callback, 10)

     
        self.nav_to_pose_goal_handle = None
        self.nav_through_poses_goal_handle = None

        self.get_logger().info("Path Navigation Manager started....")

    def e_stop_callback(self, msg: Bool):
        if msg.data != self.emergency_active:
            self.emergency_active = msg.data
            if self.emergency_active:
                self.get_logger().error("Emergency Stop ACTIVATED! Cancelled navigation and shutting down...")
                self.cancel_navigation_and_shutdown()
            else:
                self.get_logger().info("Emergency Stop RELEASED! Resuming normal operation.")

    def cancel_navigation_and_shutdown(self):
        self.get_logger().info("Cancelled active navigation goals before shutdown...")

        cancel_futures = []

        if self.nav_to_pose_goal_handle is not None:
            self.get_logger().info("Cancelling NavigateToPose goal...")
            cancel_futures.append(self.nav_to_pose_goal_handle.cancel_goal())

        if self.nav_through_poses_goal_handle is not None:
            self.get_logger().info("Cancelling NavigateThroughPoses goal...")
            cancel_futures.append(self.nav_through_poses_goal_handle.cancel_goal())

        def shutdown_procedure():
            for fut in cancel_futures:
                try:
                    fut.result()  
                except Exception as e:
                    self.get_logger().error(f"Exception while cancelling goal: {e}")

            self.get_logger().info("All goals cancelled. Shutting down node now.")
            self.destroy_node()
            rclpy.shutdown()
            import os
            os._exit(0)

        if cancel_futures:
            threading.Thread(target=shutdown_procedure, daemon=True).start()
        else:
            self.get_logger().info("No active goals to cancel. Shutting down node now.")
            threading.Thread(target=shutdown_procedure, daemon=True).start()

    # ---------- Load Paths ----------
    def load_paths_callback(self, request, response):
        if self.emergency_active:
            self.get_logger().error("Cannot load paths: Emergency Stop is ACTIVE.")
            response.success = False
            response.message = "Emergency stop active. Navigation disabled."
            return response

        try:
            if not os.path.exists(self.paths_dir):
                self.get_logger().error(f"Directory not found: {self.paths_dir}")
                response.success = False
                response.message = f"Directory not found: {self.paths_dir}"
                return response

            yaml_files = [f for f in os.listdir(self.paths_dir) if f.endswith('.yaml')]
            if not yaml_files:
                self.get_logger().warn("No YAML files found in directory.")
                response.success = False
                response.message = "No YAML files found."
                return response

            self.get_logger().info("\nAvailable YAML path files:")
            for i, f in enumerate(yaml_files, start=1):
                self.get_logger().info(f"{i}. {f}")

            while True:
                try:
                    file_number = int(input("\nEnter file number to load: "))
                    if 1 <= file_number <= len(yaml_files):
                        break
                    else:
                        self.get_logger().warn(f"Invalid number. Please enter between 1 and {len(yaml_files)}.")
                except ValueError:
                    self.get_logger().warn("Please enter a valid number.")

            yaml_path = os.path.join(self.paths_dir, yaml_files[file_number - 1])

            with open(yaml_path, 'r') as file:
                self.loaded_points = yaml.safe_load(file)

            if not self.loaded_points:
                self.get_logger().warn("No points found in YAML file.")
                response.success = False
                response.message = "No points found in file."
                return response

            self.get_logger().info(f"Loaded {len(self.loaded_points)} points from {yaml_files[file_number - 1]}:")
            for key in self.loaded_points.keys():
                self.get_logger().info(f" - {key}")

            ordered_points = sorted(self.loaded_points.keys())

            while True:
                mode_input = input("\nChoose navigation mode (1= NavigateToPose, 2= NavigateThroughPoses ): ").strip()
                if mode_input == "1":
                    selected_mode = "single"
                    break
                elif mode_input == "2":
                    selected_mode = "multi"
                    break
                else:
                    self.get_logger().warn("Invalid input. Please enter 1 or 2.")

            while True:
                target_point = input(f"\nEnter target point from available points {', '.join(ordered_points)}: ").strip()
                if target_point in ordered_points:
                    break
                else:
                    self.get_logger().warn("Invalid point name. Please choose a valid point.")

            if selected_mode == "single":
                self.selected_points = [target_point]
                self.selected_mode = "single"
                self.get_logger().info(f"Navigating directly to point: {target_point}")
                self.publish_markers()
                self._navigate_to_pose(target_point)
            else:
                idx = ordered_points.index(target_point)
                self.selected_points = ordered_points[:idx + 1]
                self.selected_mode = "multi"
                self.get_logger().info(f"Navigating through points: {self.selected_points}")
                self.publish_markers()
                self._navigate_through_poses(self.selected_points)

            response.success = True
            response.message = "Paths loaded and navigation started successfully."
        except Exception as e:
            self.get_logger().error(f"Error loading paths: {e}")
            response.success = False
            response.message = str(e)

        return response

    # ---------- Stop Navigation ----------
    def stop_navigation_callback(self, request, response):
        self.cancel_current_goal()
        response.success = True
        response.message = "Navigation stopped."
        return response

    def cancel_current_goal(self):
        if self.nav_to_pose_goal_handle is not None:
            self.get_logger().info("Cancelling NavigateToPose goal...")
            self.nav_to_pose_goal_handle.cancel_goal()

        if self.nav_through_poses_goal_handle is not None:
            self.get_logger().info("Cancelling NavigateThroughPoses goal...")
            self.nav_through_poses_goal_handle.cancel_goal()

    # ---------- Publish Markers ----------
    def publish_markers(self):
        marker_array = MarkerArray()

        if not self.selected_points:
            self.get_logger().warn("No points selected — showing all points.")
            points_to_display = self.loaded_points.items()
        else:
            points_to_display = [(name, self.loaded_points[name]) for name in self.selected_points]

        for i, (name, pose_data) in enumerate(points_to_display):
            marker = Marker()
            marker.header.frame_id = pose_data["header"]["frame_id"]
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose_data["pose"]["position"]["x"]
            marker.pose.position.y = pose_data["pose"]["position"]["y"]
            marker.pose.position.z = pose_data["pose"]["position"]["z"]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.color.r = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = pose_data["header"]["frame_id"]
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose_data["pose"]["position"]["x"] + 0.25
            text_marker.pose.position.y = pose_data["pose"]["position"]["y"]
            text_marker.pose.position.z = pose_data["pose"]["position"]["z"] + 0.1
            text_marker.scale.z = 0.25
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"[{name}]"
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        shown_names = [name for name, _ in points_to_display]
        self.get_logger().info(f"Published markers for: {shown_names}")

    # === Navigation methods ===

    def _navigate_to_pose(self, point_name):
        if point_name not in self.loaded_points:
            self.get_logger().error(f"Point '{point_name}' not found in loaded points.")
            return

        pose_data = self.loaded_points[point_name]

        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = pose_data['header']['frame_id']
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = pose_data['pose']['position']['x']
        pose_stamped.pose.position.y = pose_data['pose']['position']['y']
        pose_stamped.pose.position.z = pose_data['pose']['position']['z']
        pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
        pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
        pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
        pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']
        goal_msg.pose = pose_stamped

        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info(f"Sending NavigateToPose goal for {point_name}...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected by server.')
                return
            self.get_logger().info('Goal accepted by server.')
            self.get_logger().info('Robot is moving to the goal now...')
            self.nav_to_pose_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def _navigate_through_poses(self, point_names):
        if not self.nav_through_poses_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateThroughPoses action server not available!")
            return

        goal_msg = NavigateThroughPoses.Goal()
        poses = []

        for point_name in point_names:
            if point_name not in self.loaded_points:
                self.get_logger().error(f"Point '{point_name}' not found in loaded points.")
                return

            pose_data = self.loaded_points[point_name]
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = pose_data['header']['frame_id']
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = pose_data['pose']['position']['x']
            pose_stamped.pose.position.y = pose_data['pose']['position']['y']
            pose_stamped.pose.position.z = pose_data['pose']['position']['z']
            pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']
            poses.append(pose_stamped)

        goal_msg.poses = poses

        self.get_logger().info(f"Sending NavigateThroughPoses goal for points: {point_names}...")
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected by server.')
                return
            self.get_logger().info('Goal accepted by server.')
            self.get_logger().info('Robot is moving to the goals now...')
            self.nav_through_poses_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    # Action callbacks

    def _feedback_callback(self, feedback_msg):
  
        pass

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Navigation completed with status: {status}, result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = PathNavigationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
