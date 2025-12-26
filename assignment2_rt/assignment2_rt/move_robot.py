''' this node allows the user to move the robt using inputs
The command should be sent for 1 second, and then the robot should
stop, and the user should be able again to insert the command.

We need to understand the type of message exchanged and wich topic to use.
- Topic: /cmd_vel
- Message type: geometry_msgs/msg/Twist

- Start the class MoveRobot that inherits from Node
- Create a publisher to the topic /cmd_vel with message type Twist
- Create a loop function, read from the terminal the command and publish the corresponding Twist message
- After publishing the command, publish a zero Twist message to stop the robot after 1 second.

This robot movement controller is very simple, it only accepts linear velocity in x direction and angular velocity in z direction.

'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot') # name of the node
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)


    def send_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
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

    def loop(self):
        while rclpy.ok():
            linear_x = self.ask_float("Linear velocity (m/s): ")
            angular_z = self.ask_float("Angular velocity (rad/s): ")

            self.send_command(linear_x, angular_z)


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    node.loop() # start the loop to read commands and send them
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()



