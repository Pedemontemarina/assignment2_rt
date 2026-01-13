import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custo:mnessage.srv import StopRestart


class StopRestartServer(Node):

    def __init__(self):
        super().__init__('stop_restart_server')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.srv = self.create_service(StopRestart,'stop_restart', self.callback)
        self.get_logger().info("StopRestart server ready.")

    def callbak(self, request, response):

        if request.stop:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            response.status = "Robot stopped"

        elif request.restart:
            response.status = "Robot restarted"
            self.get_logger().info("Robot RESTARTED")

        else:
            response.status = "No action requested"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = StopRestartServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


