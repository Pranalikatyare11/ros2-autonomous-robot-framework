#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import subprocess
import os
import psutil

class NavigationController(Node):
    def __init__(self):
        super().__init__('mapper_controller')

        self.current_robot_state = "IDLE"  

        self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        self.navigation_active_pub = self.create_publisher(Bool, '/mapping_active', 10)

        self.sim_processes = {
            '2d': None,
            '3d': None
        }

        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('mapper')
        self.launch_dir = os.path.join(pkg_share, 'launch')

        self.create_service(SetBool, 'control_mapper_2d', self.make_callback('2d'))
        self.create_service(SetBool, 'control_mapper_3d', self.make_callback('3d'))

        self.get_logger().info('Mapper Controller Running')

    def robot_state_callback(self, msg):
        self.current_robot_state = msg.data
        self.get_logger().debug(f"Robot state updated to: {self.current_robot_state}")

    def kill_simulation_processes(self, parent_pid):
        TARGETS = ["gzserver", "gzclient", "gazebo", "rviz2"]

        try:
            parent = psutil.Process(parent_pid)
        except psutil.NoSuchProcess:
            return

        # Kill child processes first
        for child in parent.children(recursive=True):
            try:
                if any(t in child.name().lower() for t in TARGETS):
                    child.kill()
                else:
                    child.terminate()
            except Exception as e:
                self.get_logger().warn(f"Failed to terminate child process: {e}")

        try:
            parent.terminate()
            parent.wait(timeout=5)
        except Exception as e:
            self.get_logger().warn(f"Failed to terminate parent process: {e}")

    def make_callback(self, mode):
        def callback(request, response):

            if request.data:  
                if self.current_robot_state != "IDLE":
                    response.success = False
                    response.message = f"Cannot start {mode.upper()} mapper, robot state = {self.current_robot_state}"
                    return response

            process = self.sim_processes[mode]

            if request.data:
              
                if process is None or process.poll() is not None:
                    launch_file = f"{mode.upper()}.mapper.launch.py"
                    launch_path = os.path.join(self.launch_dir, launch_file)

                    if not os.path.exists(launch_path):
                        response.success = False
                        response.message = f"Missing launch file: {launch_file}"
                        return response

                    self.sim_processes[mode] = subprocess.Popen(
                        ["ros2", "launch", "mapper", launch_file]
                    )

                    self.navigation_active_pub.publish(Bool(data=True))

                    response.success = True
                    response.message = f"{mode.upper()} mapper started"
                else:
                    response.success = True
                    response.message = f"{mode.upper()} mapper already running"
            else:
           
                if process is not None and process.poll() is None:
                    self.get_logger().info(f"Stopping {mode.upper()} mapper...")
                    self.kill_simulation_processes(process.pid)
                    self.sim_processes[mode] = None

                    self.navigation_active_pub.publish(Bool(data=False))

                response.success = True
                response.message = f"{mode.upper()} mapper stopped"

            return response

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
