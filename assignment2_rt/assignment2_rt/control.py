
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_message.srv import Threshold
from custom_message.msg import Distance
import time

class Control(Node):

    def __init__(self):
        super().__init__('control') # name of the node
        
        self.min_distance = float('inf')
        self.linear_velocity = None
        self.angular_velocity = None
        self.backup_active = False
        self.user_linear_velocity = 0.0
        self.user_angular_velocity = 0.0

        # laser scanner subscriber - to read the distances from obstacles
        self.subscription = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        
        # velocity subscriber - to get the position/velocity of the robot
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.velocity_callback,10)
        
        # user velocity subscriber - to get the user commands
        self.user_subscription = self.create_subscription(Twist,'/cmd_user_vel',self.userinput_callback,10) 

        #client to set threshold service
        self.client = self.create_client(Threshold,'get_threshold')
       
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for threshold service...")

        self.threshold = self.threshold_service_call()
        self.get_logger().info(f"Threshold received: {self.threshold}")

        # pubblisher to cmd_vel, to move the robot back to safe position
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # publisher to custom message with info about obstacle avoidance
        self.publisher_info = self.create_publisher(Distance, '/obstacle_info', 10)
        
        # timer to publish obstacle info at regular intervals
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_obstacle_info)

        #timer to run control loop
        control_timer_period = 0.1  # seconds
        self.control_timer = self.create_timer(control_timer_period, self.control_loop) 
      

    def threshold_service_call(self):
        # call the service to set the threshold
        req = Threshold.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.threshold


    def laser_callback(self, msg):
        # store on wich range index the minimum distance is detected 
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        self.min_distance = min_distance
        self.min_index = min_index
    
    def velocity_callback(self, msg):
        # store the current velocity of the robot
        current_velocity = msg
        self.linear_velocity = current_velocity.linear.x
        self.angular_velocity = current_velocity.angular.z
    
    def userinput_callback(self, msg):
        # store the user commanded velocity of the robot
        user_velocity = msg
        self.user_linear_velocity = user_velocity.linear.x
        self.user_angular_velocity = user_velocity.angular.z

    
    def publish_obstacle_info(self):
        # publish info about the closest obstacle
        distance_msg = Distance()
        distance_msg.distance = self.min_distance
        distance_msg.direction = define_direction_from_index(self.min_index)
        distance_msg.threshold = self.threshold
        self.publisher_info.publish(distance_msg)
    
    def control_loop(self):

        if self.backup_active:

            if self.min_distance >= self.threshold:
                self.get_logger().info("Safe distance restored, resuming normal control")
                self.backup_active = False

                # stop the robot
                twist = Twist()
                self.publisher_.publish(twist)
                return

            twist = self.compute_backup_twist()
            self.publisher_.publish(twist)
            return

        if self.min_distance < self.threshold:
            self.get_logger().info("Obstacle too close, starting backup maneuver")

            self.backup_active = True

            twist = self.compute_backup_twist()
            self.publisher_.publish(twist)
            return


    
    def compute_backup_twist(self):
        twist = Twist()
        if self.linear_velocity is None or self.angular_velocity is None:
            return twist  # no movement if no velocity info
        
        twist.linear.x = -self.user_linear_velocity
        twist.angular.z = -self.user_angular_velocity
        return twist

        

def define_direction_from_index(index):
    # define the direction of the obstacle from the index of the laser scan
    if index >= 0 and index < 180:
        return "left"
    elif index >= 180 and index < 540:
        return "front"
    elif index >= 540 and index < 720:
        return "right"
    else:
        return "unknown"




def main(args=None):
    rclpy.init(args=args)

    node = Control()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()