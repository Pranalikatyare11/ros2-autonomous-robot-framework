#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import ChangeState
from task_manager_server.srv import ChangePlanners 
import time

class PlannerSwitcherNode(Node):
    def __init__(self):
        super().__init__('planner_switcher')

      
        self.planner_client = self.create_client(SetParameters, '/planner_server/set_parameters')
        self.controller_client = self.create_client(SetParameters, '/controller_server/set_parameters')

       
        self.planner_lifecycle_client = self.create_client(ChangeState, '/planner_server/change_state')
        self.controller_lifecycle_client = self.create_client(ChangeState, '/controller_server/change_state')

      
        self.get_logger().info('Waiting for parameter and lifecycle services...')
        self.planner_client.wait_for_service()
        self.controller_client.wait_for_service()
        self.planner_lifecycle_client.wait_for_service()
        self.controller_lifecycle_client.wait_for_service()
        self.get_logger().info('All services available.')

        self.srv = self.create_service(ChangePlanners, 'change_planners', self.change_planners_callback)

    def change_planners_callback(self, request, response):
       
        params_to_set = []

        if request.global_planner.strip():
            self.get_logger().info(f"Changing global planner to: {request.global_planner}")
            param = Parameter(
                name='GridBased.plugin',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=request.global_planner)
            )
            params_to_set.append(('planner', param))

        if request.local_planner.strip():
            self.get_logger().info(f"Changing local planner to: {request.local_planner}")
            param = Parameter(
                name='FollowPath.plugin',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=request.local_planner)
            )
            params_to_set.append(('controller', param))

        if not params_to_set:
            response.success = True
            response.message = "No planners changed; using defaults."
            self.get_logger().info(response.message)
            return response

       
        for server, param in params_to_set:
            if server == 'planner':
                future = self.planner_client.call_async(SetParameters.Request(parameters=[param]))
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None:
                    response.success = False
                    response.message = "Failed to set global planner parameter."
                    self.get_logger().error(response.message)
                    return response
            elif server == 'controller':
                future = self.controller_client.call_async(SetParameters.Request(parameters=[param]))
                rclpy.spin_until_future_complete(self, future)
                if future.result() is None:
                    response.success = False
                    response.message = "Failed to set local planner parameter."
                    self.get_logger().error(response.message)
                    return response

      
        for client, name in [(self.planner_lifecycle_client, 'planner_server'), (self.controller_lifecycle_client, 'controller_server')]:
            self.get_logger().info(f"Reloading {name} to apply new parameters...")
 
            req = ChangeState.Request()
            req.transition.id = 3  
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None or future.result().success is False:
                response.success = False
                response.message = f"Failed to deactivate {name}."
                self.get_logger().error(response.message)
                return response

            time.sleep(1)  

           
            req = ChangeState.Request()
            req.transition.id = 1 
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None or future.result().success is False:
                response.success = False
                response.message = f"Failed to activate {name}."
                self.get_logger().error(response.message)
                return response

        response.success = True
        response.message = "Planners updated and reloaded successfully."
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PlannerSwitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
