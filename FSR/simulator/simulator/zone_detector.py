#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
import yaml
import os
import math


class MultiKeepoutZoneDetector(Node):
    def __init__(self):
        super().__init__('multi_keepout_zone_detector')

        self.get_logger().info(" Multi Keepout Zone Detector Started")

        
        self.polygons = self.load_all_polygons()
        if not self.polygons:
            self.get_logger().error(" No polygons found! Node cannot run.")
            return

      
        self.zone_pub = self.create_publisher(Bool, '/robot_in_keepout_zone', 10)
        self.zone_name_pub = self.create_publisher(String, '/robot_zone_name', 10)
        self.left_zone_pub = self.create_publisher(String, '/robot_left_zone', 10)
        self.warning_pub = self.create_publisher(String, '/keepout_zone_warning', 10)

        self.prev_zones = None
        self.warning_distance = 0.15

      
        self.warning_sent = {}

    
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def load_all_polygons(self):
        folder = os.path.expanduser('~/FSR_WS/src/FSR/simulator/keepout_zones')
        if not os.path.exists(folder):
            self.get_logger().error(f"Folder not found: {folder}")
            return {}

        polygons = {}
        for file in os.listdir(folder):
            if file.endswith('.yaml'):
                path = os.path.join(folder, file)
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)
                pts = data.get('allowed_zone', [])
                polygons[file] = [(p['x'], p['y']) for p in pts]
                self.get_logger().info(f"Loaded polygon: {file} with {len(pts)} points")
        return polygons

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        current_zones = set()

        for name, poly in self.polygons.items():

            if name not in self.warning_sent:
                self.warning_sent[name] = False

            if self.is_inside(poly, x, y):
                current_zones.add(name)
                self.warning_sent[name] = False 
                continue

            dist = self.distance_to_polygon(poly, x, y)

         
            if dist <= self.warning_distance and not self.warning_sent[name]:
                warn_text = f"⚠ Robot {self.warning_distance}m from entering keepout zone: {name}"
                self.warning_pub.publish(String(data=warn_text))
                self.get_logger().warn(warn_text)

                self.warning_sent[name] = True  

         
            if dist > self.warning_distance:
                self.warning_sent[name] = False

     
        if self.prev_zones is None:
            self.prev_zones = current_zones
            if current_zones:
                for zone in current_zones:
                    self.get_logger().warn(f"⚠ Robot is currently INSIDE keepout zone: {zone}")
            else:
                self.get_logger().info("ℹ Robot is currently OUTSIDE all keepout zones")

        else:
          
            left = self.prev_zones - current_zones
            for zone in left:
                msg = f"ℹ Robot LEFT keepout zone: {zone}"
                self.left_zone_pub.publish(String(data=msg))
                self.get_logger().info(msg)

         
            entered = current_zones - self.prev_zones
            for zone in entered:
                self.get_logger().warn(f"!!!! Robot ENTERED keepout zone: {zone}")

            self.prev_zones = current_zones

        # Publish status
        self.zone_pub.publish(Bool(data=bool(current_zones)))
        self.zone_name_pub.publish(String(data=", ".join(current_zones) if current_zones else ""))

    def is_inside(self, polygon, x, y):
        inside = False
        n = len(polygon)
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if min(p1y, p2y) < y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y + 1e-9) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def distance_to_polygon(self, poly, x, y):
        min_dist = float('inf')
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]

            dx = x2 - x1
            dy = y2 - y1
            if dx == 0 and dy == 0:
                dist = math.hypot(x - x1, y - y1)
            else:
                t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                dist = math.hypot(x - proj_x, y - proj_y)

            min_dist = min(min_dist, dist)
        return min_dist


def main(args=None):
    rclpy.init(args=args)
    node = MultiKeepoutZoneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
