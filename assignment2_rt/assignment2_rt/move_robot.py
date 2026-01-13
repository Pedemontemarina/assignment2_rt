import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_message.srv import Average
from custom_message.srv import StopRestart
import time


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot') # name of the node
        
        self.mode = "user_control"

        self.user_publisher_ = self.create_publisher(Twist, '/cmd_user_vel', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.client = self.create_client(Average,'get_average')

        while not self.client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('Waiting for get_average service...') 
        
        # restarting the robot
        # need to create a new server node 
        self.client_restart = self.create_client(StopRestart,'stop_restart')

        while not self.client_restart.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('Waiting for get_average service...') 
        


        

    def send_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg) #moves the robot
        self.user_publisher_.publish(msg) #topic of user commands
        self.get_logger().info(f"Publishing cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

        time.sleep(1)  # wait for 1 second
        # Stop the robot
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Publishing cmd_vel: linear_x=0.0, angular_z=0.0")

    def ask_float(self, prompt):
        while True:
            value = input(prompt)
            try:
                return float(value)
            except ValueError:
                print("invalid input, float value needed.")
    
    def ask_yes_no(self, prompt):
        while True:
            value = input(prompt).lower().strip()
            if value in ['y', 'yes']:
                return True
            elif value in ['n', 'no']:
                return False
            else:
                print("invalid input, please enter 'y' or 'n'.")
    
    def call_service(self):
        req = Average.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None: 

            print(f"\n Average of last 5 commands:") 
            print(f" Linear = {response.avg_linear:.3f}") 
            print(f" Angular = {response.avg_angular:.3f}\n") 
        else:
            print("Service call failed.")
    

    def call_service_restarting(self, stop=False, restart=False):
        req = StopRestart.Request()
        req.stop = stop
        req.restart = restart

        future = self.client_restart.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is not None:
            self.get_logger().info(f"Stop/Restart service response: {response.status}")
        else:
            self.get_logger().error("Stop/Restart service call failed")
            


    def loop(self):
        while rclpy.ok():
            if self.mode == "user_control":
                
                linear_x = self.ask_float("Linear velocity (m/s): ")
                angular_z = self.ask_float("Angular velocity (rad/s): ")

                self.send_command(linear_x, angular_z)

                if self.ask_yes_no("Do you want to get the average of last 5 commands? (y/n): "):
                    self.call_service()
                
                if self.ask_yes_no("Do you want to stop the robot? (y/n): "):
                    self.call_service_restarting(stop=True, restart=False)
                    self.mode = "stop"

            
            if self.mode== "stop":

                print("\nRobot is STOPPED.")
                if self.ask_yes_no("Do you want to restart the robot? (y/n): "):
                self.call_service_restarting(stop=False, restart=True)
                self.mode = "user_control"


                



            
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    node.loop() # start the loop to read commands and send them
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()



