#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action  import DriveDistance
import math

namespace = ''

class drive_node(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("node_name")  # MODIFY NAME

        '''
        Here we initiate a new action server. We include where to add the action client
        (self), the type of action (DockServo), and the action name ('dock'). 
        '''  
        print('Initiating a new action server...')
        self._action_client = ActionClient(self, DriveDistance, namespace + 'drive_distance')        

    def send_goal(self, goal_msg):

        #Get the action goal message for Undock
        #goal_msg = Undock.Goal()
        print('Goal Message: ' + str(goal_msg))

        MAX_WAIT_SECS = 60

        #Wait for server. Shutdown if no response in MAX_WAIT_SECS seconds
        print('Waiting for action server, {0} secs max...'.format(MAX_WAIT_SECS))
        if not self._action_client.wait_for_server(MAX_WAIT_SECS):
            print("Timed-out waiting for goal")
            rclpy.shutdown()
            return
 
        print('Send goal and get future')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        print('Set response callback')
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    '''Response Callback ------------------------------
    This is a "done" callback for the initial send_goal_async future registered in the send_goal function.
    In this case, response indicates whether the goal was accepted or rejected.
    '''
    def goal_response_callback(self, future):
        print('Checking if goal was accepted or rejected...')
        
        accepted = True

        result = future.result()
        accepted = result.accepted

        '''
        If the goal was accepted, create a callback for the "done" status
        This step is registering a callback (similar to that of the goal response).
        '''
        if accepted:
            self.get_logger().info('goal_response_callback: Goal ACCEPTED...')
            self._get_result_future = result.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        else:
            self.get_logger().info('goal_response_callback: Goal REJECTED...')
            print('Shutting down action client node.')
            rclpy.shutdown()            
    
    '''Result Callback ------------------------------
    The response callback detected whether the goal was accepted. This is also a "done" callback,
    but for the get_result_async() future from the send_goal_async function 1st created in 
    the send_goal() function.
    '''
    def get_result_callback(self, future):
        '''
        Here, we are logging the result sequence.
        '''
        result = future.result().result

        ''' 
        result, in this case, is type any. You have to look up the properties. In this case,
        you can run ros2 interface show irobot_create_msgs/action/Dock and you'll see the 
        result type is:  bool is_docked
        result.pose.header works
        '''
        print("get_result_callback: DriveDistance_Result = {0}".format(result.pose.pose.position.x))

        print('Shutting down action client node.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    #instance of drive_node class
    node = drive_node() 

    '''
    Sends a goal to the server.
    '''
    print('Action server available. Sending undock goal to server.')
    goal = DriveDistance.Goal()
    goal.distance = float(0.3)                   #Drive 1m
    goal.max_translation_speed = 0.15   #

    node.send_goal(goal)
    
    '''
    When an action server accepts or rejects the goal, future is completed.
    '''
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopped via keyboard interrupt")
        rclpy.shutdown()
    finally:
        print("Completed")

if __name__ == "__main__":
    main()
