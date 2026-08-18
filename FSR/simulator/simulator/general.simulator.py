import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import psutil
import time


class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        self.sim_processes = {
            '2d': None,
            '3d': None
        }

        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('simulator')
        self.launch_dir = os.path.join(pkg_share, 'launch')

        self.create_service(SetBool, 'control_simulation_2d', self.make_callback('2d'))
        self.create_service(SetBool, 'control_simulation_3d', self.make_callback('3d'))

        self.get_logger().info('Simulation Controller Running')

    def kill_simulation_processes(self, parent_pid):
        TARGETS = ["gzserver", "gzclient", "gazebo", "rviz2"]

        try:
            parent = psutil.Process(parent_pid)
        except psutil.NoSuchProcess:
            return

        # kill children
        for child in parent.children(recursive=True):
            try:
                if any(t in child.name().lower() for t in TARGETS):
                    child.kill()
                else:
                    child.terminate()
            except:
                pass

        try:
            parent.terminate()
        except:
            pass

    def make_callback(self, mode):
        def callback(request, response):
            process = self.sim_processes[mode]

            if request.data:
                # START
                if process is None or process.poll() is not None:
                    launch_file = f"{mode.upper()}.simulator.launch.py"
                    launch_path = os.path.join(self.launch_dir, launch_file)

                    if not os.path.exists(launch_path):
                        response.success = False
                        response.message = f"Missing launch file: {launch_file}"
                        return response

                    self.sim_processes[mode] = subprocess.Popen(
                        ["ros2", "launch", "simulator", launch_file]
                    )
                    response.success = True
                    response.message = f"{mode.upper()} simulator started"

                else:
                    response.success = True
                    response.message = f"{mode.upper()} simulator already running"

            else:
                # STOP
                if process is not None and process.poll() is None:
                    self.get_logger().info(f"Stopping {mode.upper()} simulator...")

                    self.kill_simulation_processes(process.pid)

                    self.sim_processes[mode] = None

                response.success = True
                response.message = f"{mode.upper()} simulator stopped"

            return response

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = SimulationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
