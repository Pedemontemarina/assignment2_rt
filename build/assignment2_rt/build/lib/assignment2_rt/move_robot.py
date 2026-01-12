import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_message.srv import Average
import time


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot') # name of the node

        self.user_publisher_ = self.create_publisher(Twist, '/cmd_user_vel', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.client = self.create_client(Average,'get_average')

        while not self.client.wait_for_service(timeout_sec=1.0): 
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

    def loop(self):
        while rclpy.ok():
            linear_x = self.ask_float("Linear velocity (m/s): ")
            angular_z = self.ask_float("Angular velocity (rad/s): ")

            self.send_command(linear_x, angular_z)

            if self.ask_yes_no("Do you want to get the average of last 5 commands? (y/n): "):
                self.call_service()

            
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    node.loop() # start the loop to read commands and send them
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()



