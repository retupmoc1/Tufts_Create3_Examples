'''
This file was heavily influenced by Sawyer Paccione's action undock file:
https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/action_undock.py
action_undock.py
Tufts Create® 3 Educational Robot Example
This file is a simple action client file that will undock the robot if it is docked. 
'''

'''
These statements allow the Node class to be used and actions to be performed. 
'''
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

'''
This statement imports the undock action.
'''
from irobot_create_msgs.action import Undock

'''
Input your namespace here as a global variable. 
'''
namespace = ''

class UndockingActionClient(Node):
    '''
    This is an action client. Action clients send goal requests to action servers.
    We are defining a class "UndockingActionClient" which is a subclass of Node. 
    '''

    def __init__(self):
        '''
        We initialize the class by calling the Node constructor then
        naming the node 'undocking_action_client'
        '''
        super().__init__('undocking_action_client')
        
        '''
        Here we initiate a new action server. We include where to add the action client
        (self), the type of action (DockServo), and the action name ('dock'). 
        '''  
        print('Initiating a new action server...')
        self._action_client = ActionClient(self, Undock, namespace + '/undock')

    def send_goal(self):

        #Get the action goal message for Undock
        goal_msg = Undock.Goal()
        print('Goal Message: ' + str(goal_msg))

        print('Waiting for action server...')
        self._action_client.wait_for_server(5)
 
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

        if accepted:
            self.get_logger().info('goal_response_callback: Goal ACCEPTED...')
        else:
            self.get_logger().info('goal_response_callback: Goal REJECTED...')
            return

        '''
        If the goal was accepted, create a callback for the "done" status
        This step is registering a callback (similar to that of the goal response).
        '''
        self._get_result_future = result.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    
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
        '''
        print("get_result_callback: is_docked = {0}".format(result.is_docked))

        print('Shutting down action client node.')
        rclpy.shutdown()


def main(args=None):
    '''
    Initialize ROS2 communication and create an instance of 'UndockingActionClient'
    '''
    rclpy.init(args=args)
    undock_client = UndockingActionClient()
    
    '''
    Sends a goal to the server.
    '''
    print('Action server available. Sending undock goal to server.')
    undock_client.send_goal()
    
    '''
    When an action server accepts or rejects the goal, future is completed.
    '''
    rclpy.spin(undock_client)

if __name__ == '__main__':
    main()