import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        

        self.emergency_active = False

        self.publisher = self.create_publisher(Bool, '/emergency_stop', 10)
      
        self.service = self.create_service(SetBool, 'emergency_stop_toggle', self.toggle_callback)
        
        self.get_logger().info("Emergency Stop Node started")

    def toggle_callback(self, request, response):
        if self.emergency_active != request.data:
            self.emergency_active = request.data
            
            
            msg = Bool()
            msg.data = self.emergency_active
            self.publisher.publish(msg)
            
        
            self.get_logger().info(f"Emergency stop {'activated' if self.emergency_active else 'released'}")
        else:

            self.get_logger().info(f"Emergency stop already {'activated' if self.emergency_active else 'released'} (no change)")
        
        response.success = True
        response.message = f"Emergency stop {'activated' if self.emergency_active else 'released'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
