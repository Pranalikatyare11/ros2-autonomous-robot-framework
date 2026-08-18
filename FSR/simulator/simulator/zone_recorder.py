#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import SetBool
import yaml
import os
import sys

class PolygonRecorder(Node):
    def __init__(self):
        super().__init__('polygon_recorder')

        self.points = []
        self.is_recording = False
        self.marker_id_counter = 0  

        self.marker_pub = self.create_publisher(Marker, 'allowed_zone_marker', 10)


        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

       
        self.srv = self.create_service(SetBool, 'record_polygon', self.record_polygon_callback)

        self.get_logger().info("Polygon Recorder node ready.")
        self.get_logger().info("Call /record_polygon with data=True to start recording points.")
        self.get_logger().info("Call /record_polygon with data=False to stop recording and save polygon.")

    def clicked_point_callback(self, msg: PointStamped):
        if not self.is_recording:
            return

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z 
        self.points.append((x, y, z))
        self.get_logger().info(f"Point added from clicked_point: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.publish_polygon_marker()

    def publish_polygon_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "polygon"
        marker.id = self.marker_id_counter
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        for x, y, z in self.points:  
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            marker.points.append(pt)

        self.marker_pub.publish(marker)

    def record_polygon_callback(self, request, response):
        if request.data:  
            if self.is_recording:
                response.success = False
                response.message = "Already recording."
                return response
       
            self.points = []
            self.is_recording = True
      
            self.delete_marker()
            self.marker_id_counter += 1  
            self.get_logger().info("Started polygon recording.")
            response.success = True
            response.message = "Started recording polygon points."
            return response
        else:
            if not self.is_recording:
                response.success = False
                response.message = "Not currently recording."
                return response
            self.is_recording = False
            self.get_logger().info("Stopped polygon recording.")
            filename = self.ask_filename()
            self.save_polygon(filename)
            response.success = True
            response.message = f"Stopped recording and saved polygon to {filename}"
            return response

    def ask_filename(self):
        print("Enter filename to save polygon (without extension): ", end='', flush=True)
        filename_input = sys.stdin.readline().strip()
        if not filename_input:
            filename_input = "allowed_zone"
        return filename_input

    def save_polygon(self, filename_without_ext):
        folder_path = os.path.expanduser('~/FSR_WS/src/FSR/simulator/keepout_zones')
        os.makedirs(folder_path, exist_ok=True)
        filename = os.path.join(folder_path, f"{filename_without_ext}.yaml")

      
        data = {'allowed_zone': [
            {'x': float(x), 'y': float(y), 'z_min': float(z), 'z_max': float(z) + 0.1}
            for x, y, z in self.points
        ]}

        try:
            with open(filename, 'w') as f:
                yaml.dump(data, f)
            self.get_logger().info(f"Polygon saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save polygon: {e}")

    def delete_marker(self):
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp.sec = 0
        delete_marker.header.stamp.nanosec = 0
        delete_marker.ns = "polygon"
        delete_marker.id = self.marker_id_counter 
        delete_marker.action = Marker.DELETE
        self.marker_pub.publish(delete_marker)
        self.get_logger().info("Deleted previous polygon marker from RViz.")

    def destroy_node(self):
        self.delete_marker()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PolygonRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down, deleting markers...")
        node.delete_marker()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
