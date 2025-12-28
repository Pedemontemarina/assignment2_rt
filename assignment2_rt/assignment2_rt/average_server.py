from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_message.srv import Average   


class AverageServer(Node):

    def __init__(self):
        super().__init__('average_server')

        self.linear_queue = deque(maxlen=5)
        self.angular_queue = deque(maxlen=5)

        # Subscriber using cmd_user_vel
        self.subscriber = self.create_subscription(Twist,'/cmd_user_vel',self.velocity_callback,10)

        self.srv = self.create_service(Average,'get_average',self.callback)

        self.get_logger().info("AverageServer ready.")


    def velocity_callback(self, msg):
        self.linear_queue.append(msg.linear.x)
        self.angular_queue.append(msg.angular.z)


    def callback(self, request, response):
      
        if len(self.linear_queue) == 0:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            return response

        response.avg_linear = sum(self.linear_queue) / len(self.linear_queue)
        response.avg_angular = sum(self.angular_queue) / len(self.angular_queue)

        self.get_logger().info( f"Average → linear velocity ={response.avg_linear:.2f}, angular velocity={response.avg_angular:.2f}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AverageServer()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

