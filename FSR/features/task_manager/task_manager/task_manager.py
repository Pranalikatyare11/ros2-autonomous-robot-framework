#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import yaml
import json
import time
import os
import threading
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from task_manager_server.action import RunTaskFile
import math
from geometry_msgs.msg import Quaternion


# Action types
ACTION_NAV_TO_POSE = 1
ACTION_NAV_THROUGH_POSES = 2
ACTION_WAIT = 3

# Error codes
ERROR_SUCCESS = 0
ERROR_GOAL_REJECTED = 100
ERROR_ROBOT_STUCK = 102
ERROR_GOAL_PAUSED_MANUAL = 103
ERROR_GOAL_PAUSED_EMERGENCY = 105
ERROR_NAVIGATION_STOPPED = 107


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def create_pose_stamped(x, y, yaw=0.0, z=0.0, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose


class TaskManagerNode(Node):
    MAX_RETRIES = 3

    def __init__(self):
        super().__init__('task_manager_node')

        self.callback_group = ReentrantCallbackGroup()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self._run_task_action_server = ActionServer(
            self,
            RunTaskFile,
            'run_task_file',
            execute_callback=self.execute_task_file_action,
            callback_group=self.callback_group,
        )

        # Feedback topic
        self.feedback_pub = self.create_publisher(String, 'task_feedback', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # parameter
        self.declare_parameter('wait_duration', -1)
        self.wait_duration_param = self.get_parameter('wait_duration').value
        self.wait_duration_override_next = False
        self.add_on_set_parameters_callback(self.parameter_callback)

        # runtime state
        self.is_paused = False
        self.is_navigating = False
        self.current_goal_handle = None
        self.stop_navigation_flag = False
        self.task_thread = None

        self.current_subtask_index = 0
        self.task_start_time = None

        self.paused_subtask = None

        self._pause_msg_printed = False
        self.paused_due_to_emergency = False
        self._cancelled_due_to_pause = False

        self.current_task_file_path = None
        self._active_goal_handle = None

        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.emergency_stop_active = False
        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # services
        self.pause_navigation_srv = self.create_service(
            SetBool,
            '/pause_navigation',
            self.pause_navigation_callback,
            callback_group=self.callback_group,
        )

        self.stop_navigation_srv = self.create_service(
            Trigger,
            '/stop_navigation',
            self.stop_navigation_callback,
            callback_group=self.callback_group,
        )

        self.shutdown_srv = self.create_service(
            Trigger,
            '/shutdown_node',
            self.shutdown_node_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Waiting for navigation action servers...")
        self.nav_to_pose_client.wait_for_server()
        self.nav_through_poses_client.wait_for_server()
        self.get_logger().info("TaskManagerNode initialized and ready for run_task_file action calls.")


    def calculate_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        if pose1 is None or pose2 is None:
            return float('inf')
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return (dx**2 + dy**2) ** 0.5

    def calculate_eta_to_pose(self, current_pose: PoseStamped, target_pose: PoseStamped, avg_speed=0.3) -> float:
        if current_pose is None or target_pose is None:
            return -1
        dist = self.calculate_distance(current_pose, target_pose)
        return dist / avg_speed if avg_speed > 0 else -1

    def calculate_eta_through_poses(self, current_pose: PoseStamped, poses: list, avg_speed=0.3) -> float:
        if current_pose is None or not poses:
            return -1
        total_dist = self.calculate_distance(current_pose, poses[0])
        for i in range(len(poses) - 1):
            total_dist += self.calculate_distance(poses[i], poses[i + 1])
        return total_dist / avg_speed if avg_speed > 0 else -1

    def amcl_pose_callback(self, msg):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def emergency_stop_callback(self, msg: Bool):
        if msg.data and not self.emergency_stop_active:
            self.get_logger().warn("Emergency STOP ACTIVATED!")
            feedback = String()
            feedback.data = "EMERGENCY STOP ACTIVATED! Robot halted immediately."
            self.feedback_pub.publish(feedback)

            self.emergency_stop_active = True
            self.paused_subtask = self.current_subtask_index
            self.paused_due_to_emergency = True
            self.is_paused = True
            self.cancel_current_goal()
            stop_msg = Twist()
            for _ in range(5):
                self.cmd_vel_pub.publish(stop_msg)
                time.sleep(0.02)
            self.get_logger().info("Robot stopped and task execution paused by emergency.")
            return

        if not msg.data and self.emergency_stop_active:
            self.get_logger().info("Emergency STOP released.")
            feedback = String()
            feedback.data = "Emergency stop released. Resuming task..."
            self.feedback_pub.publish(feedback)
 
            self.emergency_stop_active = False
            self.paused_due_to_emergency = False
            threading.Thread(target=self.resume_after_emergency, daemon=True).start()
            return

    def pause_navigation_callback(self, request, response):
        if self.emergency_stop_active:
            response.success = False
            response.message = "Cannot pause/resume navigation: Emergency stop active."
            self.get_logger().warn(response.message)
            return response
        self.is_paused = request.data
        response.success = True
        response.message = "Navigation paused" if self.is_paused else "Navigation resumed"
        
        feedback = String()
        if self.is_paused:
            feedback.data = "Navigation paused."
        else:
            feedback.data = "Navigation resumed. Continuing task..."
        
        self.feedback_pub.publish(feedback)
        self.get_logger().info(feedback.data)

        if self.emergency_stop_active:  
            feedback = String()
            feedback.data = "Emergency active — pause/resume ignored."
            self.feedback_pub.publish(feedback)



        return response
        
       
    

    def stop_navigation_callback(self, request, response):
        if self.emergency_stop_active:
            response.success = False
            self.get_logger().warn(response.message)
            return response
        self.get_logger().info("Stop navigation service called: stopping all navigation")
        self.stop_navigation_flag = True
        self.cancel_current_goal()
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.02)
        self.stop_navigation_flag = False
        response.success = True
        response.message = "Navigation stopped"

        feedback = String()
        feedback.data = "Navigation stopped by user. Awaiting next task."
        self.feedback_pub.publish(feedback)
        self.get_logger().info(feedback.data)

        if self.emergency_stop_active:
            feedback = String()
            # feedback.data = "Emergency active — stop command ignored."
            self.feedback_pub.publish(feedback)


        return response

    def shutdown_node_callback(self, request, response):

        if self.emergency_stop_active:
            response.success = False
            response.message = "Shutdown ignored: Emergency stop active."
            self.get_logger().warn(response.message)

            feedback = String()
            self.feedback_pub.publish(feedback)

            return response
        
        self.get_logger().info("Shutdown node service called, initiating full shutdown...")
        
        self.stop_navigation_flag = True
        self.is_paused = False
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

        stop_msg = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.05)

        feedback = String()
        feedback.data = "Task Manager shutting down now. Current task aborted."
        self.feedback_pub.publish(feedback)
        self.get_logger().info(feedback.data)


        response.success = True
        response.message = "Node shutting down now."

        threading.Thread(
            target=lambda: (time.sleep(0.1), rclpy.shutdown(), os._exit(0)),
            daemon=True
        ).start()

        return response

     

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'wait_duration':
                try:
                    value = float(param.value)
                except (ValueError, TypeError):
                    return SetParametersResult(successful=False)
                if value < -1.0:
                    return SetParametersResult(successful=False)
                self.wait_duration_param = value
                if value >= 0:
                    self.wait_duration_override_next = True
                feedback = String()
                feedback.data = f"Wait duration overridden to {value} seconds."
                self.feedback_pub.publish(feedback)
                self.get_logger().info(feedback.data)

            if self.emergency_stop_active:
                feedback = String()
                feedback.data = "Emergency active — wait override ignored."
                self.feedback_pub.publish(feedback)
            return SetParametersResult(successful=False)


        return SetParametersResult(successful=True)

    def cancel_current_goal(self):
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def interruptible_wait(self, total_duration, feedback_callback=None, eta=-1):
        elapsed = 0.0
        interval = 1.0
        while elapsed < total_duration:
            if self.stop_navigation_flag:
                break
            if self.is_paused or self.emergency_stop_active:
                time.sleep(0.2)
                continue
            time.sleep(0.1)
            elapsed += 0.1
            if feedback_callback and elapsed % interval < 0.1:
                feedback_callback(elapsed)

    def send_navigate_to_pose_goal(self, pose, eta=-1):
        if self.emergency_stop_active:
            return ERROR_GOAL_PAUSED_EMERGENCY
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_completed_event = threading.Event()
        result_code = {'code': None}
        self.current_goal_handle = None

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
            except Exception:
                result_code['code'] = ERROR_GOAL_REJECTED
                goal_completed_event.set()
                return
            if not goal_handle.accepted:
                result_code['code'] = ERROR_GOAL_REJECTED
                goal_completed_event.set()
                return
            self.current_goal_handle = goal_handle

            def result_callback(future_result):
                try:
                    result = future_result.result()
                    status = result.status
                except Exception:
                    status = None
                if status == GoalStatus.STATUS_SUCCEEDED:
                    result_code['code'] = ERROR_SUCCESS
                else:
                    result_code['code'] = ERROR_ROBOT_STUCK
                goal_completed_event.set()

            goal_handle.get_result_async().add_done_callback(result_callback)

        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(goal_response_callback)

        start_time = time.time()
        while not goal_completed_event.is_set():
            if self.is_paused or self.emergency_stop_active:
                if self.current_goal_handle:
                   self.current_goal_handle.cancel_goal_async()
                while self.is_paused or self.emergency_stop_active:
                    if self.stop_navigation_flag:
                        if self.current_goal_handle:
                            self.current_goal_handle.cancel_goal_async()
                        return ERROR_NAVIGATION_STOPPED
                    time.sleep(0.2)
                return self.send_navigate_to_pose_goal(pose, eta=eta)

        # Check stop flag even when not paused
            if self.stop_navigation_flag:
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                return ERROR_NAVIGATION_STOPPED

            time.sleep(0.1)

        self.current_goal_handle = None
        return result_code['code']


    def send_navigate_through_poses_goal(self, poses, eta=-1):
        if self.emergency_stop_active:
            return ERROR_GOAL_PAUSED_EMERGENCY
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_completed_event = threading.Event()
        result_code = {'code': None}
        self.current_goal_handle = None

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
            except Exception:
                result_code['code'] = ERROR_GOAL_REJECTED
                goal_completed_event.set()
                return
            if not goal_handle.accepted:
                result_code['code'] = ERROR_GOAL_REJECTED
                goal_completed_event.set()
                return
            self.current_goal_handle = goal_handle

            def result_callback(future_result):
                try:
                    result = future_result.result()
                    status = result.status
                except Exception:
                    status = None
                if status == GoalStatus.STATUS_SUCCEEDED:
                    result_code['code'] = ERROR_SUCCESS
                else:
                    result_code['code'] = ERROR_ROBOT_STUCK
                goal_completed_event.set()

            goal_handle.get_result_async().add_done_callback(result_callback)

        self.nav_through_poses_client.send_goal_async(goal_msg).add_done_callback(goal_response_callback)

        while not goal_completed_event.is_set():
            if self.is_paused or self.emergency_stop_active:
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                while self.is_paused or self.emergency_stop_active:
                    if self.stop_navigation_flag:
                        if self.current_goal_handle:
                            self.current_goal_handle.cancel_goal_async()
                        return ERROR_NAVIGATION_STOPPED
                    time.sleep(0.2)
                return self.send_navigate_through_poses_goal(poses, eta=eta)

       
            if self.stop_navigation_flag:
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                return ERROR_NAVIGATION_STOPPED

            time.sleep(0.1)

        self.current_goal_handle = None
        return result_code['code']

    def resume_after_emergency(self):
        self.is_paused = False
        self.paused_due_to_emergency = False
        if self.paused_subtask is not None:
            self.current_subtask_index = self.paused_subtask
            self.paused_subtask = None


    def execute_task_file_action(self, goal_handle):
        self._active_goal_handle = goal_handle
        task_file_param = goal_handle.request.filename
        task_file_path = os.path.join(get_package_share_directory('task_manager'), 'tasks', task_file_param)
        if not os.path.isfile(task_file_path):
            goal_handle.abort()
            self._active_goal_handle = None
            return RunTaskFile.Result(success=False, message=f"Task file '{task_file_param}' not found.")
    

        self.is_navigating = True
        self.stop_navigation_flag = False
        self.is_paused = False
        self.current_subtask_index = 0
        self.paused_subtask = None
        self.current_task_file_path = task_file_path

   
        start_msg = String()
        start_msg.data = f"Task file '{task_file_param}' loaded. Starting execution..."
        self.feedback_pub.publish(start_msg)
        self.get_logger().info(start_msg.data)




    
        try:
            if task_file_path.endswith('.json'):
                with open(task_file_path, 'r') as f:
                    task_data = json.load(f)
            else:
                with open(task_file_path, 'r') as f:
                    task_data = yaml.safe_load(f)
        except Exception as e:
            goal_handle.abort()
            self._active_goal_handle = None
            return RunTaskFile.Result(success=False, message=f"Failed to read task file: {e}")

        tasks = task_data.get('subtasks', [])
        total_subtasks = len(tasks)
        if not tasks:
            goal_handle.abort()
            self._active_goal_handle = None
            return RunTaskFile.Result(success=False, message="No subtasks found in task file.")
        
        task_counter = 0
        while self.current_subtask_index < total_subtasks:
            
            if self.stop_navigation_flag:
                goal_handle.abort()
                self.is_navigating = False
                self._active_goal_handle = None
                return RunTaskFile.Result(success=False, message="Task stopped by user.")

            while self.is_paused or self.emergency_stop_active:
                time.sleep(0.2)

            subtask = tasks[self.current_subtask_index]
            action = subtask.get('action')
            eta = -1
            start_time = time.time()

            pose = None
            poses = None
            duration = 0

            if action == ACTION_NAV_TO_POSE:
                goal = subtask.get('goal', {})
                pose = create_pose_stamped(goal.get('x', 0.0), goal.get('y', 0.0), goal.get('yaw', 0.0))
                eta = self.calculate_eta_to_pose(self.current_pose, pose)

            elif action == ACTION_NAV_THROUGH_POSES:
                goals = subtask.get('goals', [])
                poses = [create_pose_stamped(g.get('x', 0.0), g.get('y', 0.0), g.get('yaw', 0.0)) for g in goals]
                eta = self.calculate_eta_through_poses(self.current_pose, poses)

            elif action == ACTION_WAIT:
                duration = self.wait_duration_param if self.wait_duration_override_next else subtask.get('duration', 5)
                self.wait_duration_override_next = False
                eta = duration

            error_code = ERROR_SUCCESS
            try:
                if action == ACTION_NAV_TO_POSE:
                    error_code = self.send_navigate_to_pose_goal(pose, eta=eta)
                elif action == ACTION_NAV_THROUGH_POSES:
                    error_code = self.send_navigate_through_poses_goal(poses, eta=eta)
                elif action == ACTION_WAIT:
                    self.interruptible_wait(duration, eta=eta)
                    error_code = ERROR_SUCCESS
            except Exception:
                error_code = ERROR_ROBOT_STUCK

            elapsed_time = time.time() - start_time
            delay = max(0.0, elapsed_time - eta) if eta > 0 else 0.0

            
            msg = String()
            if action == ACTION_WAIT:
                msg.data = f"Wait of {duration:.1f}s completed"
            else:
                task_counter += 1
                msg.data = f"Task {task_counter} completed | ETA={eta:.1f}s | Delay={delay:.1f}s"
            self.feedback_pub.publish(msg)
            if error_code != ERROR_SUCCESS:
                goal_handle.abort()
                self.is_navigating = False
                self._active_goal_handle = None
                return RunTaskFile.Result(success=False, message=f"Subtask {self.current_subtask_index + 1} failed with error {error_code}")

            self.current_subtask_index += 1

        goal_handle.succeed()
        self.is_navigating = False
        self._active_goal_handle = None
        return RunTaskFile.Result(success=True, message="Task execution completed.")


def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(task_manager_node)
    try:
        executor.spin()
    except KeyboardInterrupt: 
        task_manager_node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        task_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
 
