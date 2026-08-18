#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import os, yaml, re, time, threading, math

WAYPOINT_DIR = os.path.expanduser('~/FSR_WS/src/FSR/features/waypoint_recorder/points')
FILENAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')

FALLBACK_DIRS = [
    os.path.expanduser('~/FSR_WS/src/FSR/features/waypoint_recorder/points'),
]

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        os.makedirs(WAYPOINT_DIR, exist_ok=True)

        self.recording = False
        self.points = []
        self.marker_array = MarkerArray()
        self.marker_id = 0

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self._point_cb, qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_recorder/markers', qos)

        self.srv_start = self.create_service(Trigger, 'start_recording', self._start_cb)
        self.srv_stop = self.create_service(Trigger, 'stop_recording', self._stop_cb)

        self._list_all_yaml_files()

        self.get_logger().info(" Waypoint Recorder Node Started.")
        self.get_logger().info("\nUse /start_recording and /stop_recording services to control sessions.")
      

    def _list_all_yaml_files(self):
        found = set()
        for base in [WAYPOINT_DIR] + FALLBACK_DIRS:
            if os.path.isdir(base):
                for root, _, files in os.walk(base):
                    for f in files:
                        if f.endswith('.yaml'):
                            found.add(f)

        if found:
            print(" Existing waypoint files:")
            for f in sorted(found):
                print(f"  - {f}")
        else:
            print(" No waypoint files found yet.")

    def _start_cb(self, request, response):
        if self.recording:
            response.success = False
            response.message = "Already recording."
            return response

        self.recording = True
        self.points.clear()
        self.marker_array = MarkerArray()
        self.marker_id = 0
        self._publish_delete_all()

        response.success = True
        response.message = "Recording started."
        self.get_logger().info("\nRecording started. Click points in RViz.")
        return response

    def _stop_cb(self, request, response):
        if not self.recording:
            response.success = False
            response.message = "Not recording."
            return response

        self.recording = False
        self.get_logger().info("\n Recording stopped via service.")
        threading.Thread(target=self._save_prompt_and_exit, daemon=True).start()

        response.success = True
        response.message = "Recording stopped; saving started."
        return response

    def _point_cb(self, msg: PointStamped):
        if not self.recording:
            return

        self.points.append(msg)
        coords = (round(msg.point.x, 3), round(msg.point.y, 3), round(msg.point.z, 3))
        self.get_logger().info(f"[INFO] Clicked point: {coords}")

        #  Marker setup
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "waypoints"
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = msg.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = Duration(sec=0)

        self.marker_array.markers.append(m)
        self.marker_pub.publish(self.marker_array)

    def _save_prompt_and_exit(self):
        try:
            if not self.points:
                self.get_logger().warn("[WARN] No points recorded. Nothing to save.")
                time.sleep(0.2)
                self._final_exit()
                return

            # ask for unique filename (check in all dirs)
            while True:
                user_input = input("\nEnter filename: ").strip()
                if not user_input or not FILENAME_RE.match(user_input):
                    print(" Invalid filename format.")
                    continue

                # check across all directories
                duplicate_found = False
                for d in [WAYPOINT_DIR] + FALLBACK_DIRS:
                    if os.path.exists(os.path.join(d, user_input + ".yaml")):
                        duplicate_found = True
                        break

                if duplicate_found:
                    print(f" File '{user_input}.yaml' already exists in system. Please choose another name.")
                    continue

                filename = user_input
                break


            names = []
            for i in range(len(self.points)):
                pname = input(f"Enter name for point {i+1} (default=p{i+1}): ").strip() or f"p{i+1}"
                names.append(pname)

            positions = [{'x': round(p.point.x, 3), 'y': round(p.point.y, 3), 'z': round(p.point.z, 3)} for p in self.points]

            def compute_yaw(a, b):
                return math.atan2(b['y'] - a['y'], b['x'] - a['x'])

            def yaw_to_quat(yaw):
                return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}

            orientations = [{'x':0.0,'y':0.0,'z':0.0,'w':1.0} for _ in positions]
            for i in range(len(positions)-1):
                orientations[i] = yaw_to_quat(compute_yaw(positions[i], positions[i+1]))
            if len(positions) > 1:
                orientations[-1] = orientations[-2]

            data = {
                names[i]: {
                    'header': {'frame_id': 'map'},
                    'pose': {'position': positions[i], 'orientation': orientations[i]}
                } for i in range(len(self.points))
            }

            out_path = os.path.join(WAYPOINT_DIR, filename + ".yaml")
            with open(out_path, 'w') as f:
                yaml.dump(data, f, sort_keys=False)

            self.get_logger().info(f"[SAVE] {len(self.points)} points saved to {out_path}")
            print(f"[SAVE] {len(self.points)} points saved to {out_path}")
            self._publish_delete_all()
            time.sleep(0.1)
            self._final_exit()

        except Exception as e:
            print("Error:", e)
            self._final_exit()

    def _publish_delete_all(self):
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[del_marker]))
        self.marker_array = MarkerArray()

    def _final_exit(self):
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt -> shutting down")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
