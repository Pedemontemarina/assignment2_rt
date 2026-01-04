'''if the users input causes the robot to be “too close” to one of the obstacles (e.g., the minimum value 
of the laser scanner is below a certain threshold) moves the robot back to the previous position, to 
remain in a safe area.

The type of message exchanged in the /scan topic is sensor_msgs/msg/LaserScan

 1. Angoli
scansione a 360 gradi
angle_min → angolo iniziale ( -3.14 rad)

angle_max → angolo finale (es. +3.14 rad)

angle_increment → passo tra una misura e la successiva 0.5rad
il numero di misure sarà quindi:
num_readings = (angle_max - angle_min) / angle_increment = 720

2. Distanze
msg.ranges --> array di 720 float
ranges[] → array di float, uno per ogni raggio laser
Esempio:
ranges[0] = distanza a angle_min  
ranges[1] = distanza a angle_min + angle_increment  
…
range_min: 0.05 m
range_max: 10.0 m

ranges[0] → distanza a -180° (dietro il robot)
ranges[179] → distanza a -90° (sinistra del robot)
ranges[360] → distanza a 0° (davanti al robot)
ranges[540] → distanza a +90° (destra del robot)
ranges[719] → distanza a +180° (dietro il robot)

3. Intensità (opzionale)
intensities[] → qualità del segnale
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_message.srv import Threshold
from custom_message.msg import Distance
from custom_message.msg import UserCommand
import time

class Control(Node):

    def __init__(self):
        super().__init__('control') # name of the node
        
        self.min_distance = float('inf')
        self.min_index = -1
        self.user_linear_velocity = None
        self.user_angular_velocity = None
        self.safe_position_x = None
        self.safe_position_y = None
        self.current_x = None
        self.current_y = None
        self.safepositioning_active = False


        # laser scanner subscriber - to read the distances from obstacles
        self.subscription = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        
        # velocity subscriber to /cmd_user_vel - to get the input velocity of the robot
        self.subscription = self.create_subscription(Twist,'/cmd_user_vel',self.safety_callback,10) 

        # odometry subscriber to /odom - to get the current position of the robot
        self.subscription = self.create_subscription(Odometry,'/odom',self.odometry_callback,10)

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

        #client to set threshold service
        self.client = self.create_client(Threshold,'get_threshold')
       
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for threshold service...")

        self.threshold = self.threshold_service_call()
        self.get_logger().info(f"Threshold received: {self.threshold}")

      

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

    def safety_callback(self, msg: UserCommand):
        # store the input velocity of the robot
        self.user_linear_velocity = msg.cmd.linear.x
        self.user_angular_velocity = msg.cmd.angular.z
        self.safe_position_x = msg.safe_pose.x
        self.safe_position_y = msg.safe_pose.y
    
    def odometry_callback(self, msg):
        # store the current position of the robot
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    
    def publish_obstacle_info(self):
        # publish info about the closest obstacle
        distance_msg = Distance()
        distance_msg.distance = self.min_distance
        if self.min_index == -1: 
            distance_msg.direction = "unknown"
        else:
            distance_msg.direction = define_direction_from_index(self.min_index)

        distance_msg.threshold = self.threshold
        self.publisher_info.publish(distance_msg)
        # self.get_logger().info(f"Publishing obstacle info: distance={self.min_distance}, direction={distance_msg.direction}, threshold={self.threshold}")
    
    def control_loop(self):
        
        if self.min_index == -1:
            return

        if self.safepositioning_active:

            if self.min_distance >= self.threshold:
                self.get_logger().info("Safe distance restored, resuming normal control")
                self.safepositioning_active = False
                # ferma il robot
                twist = Twist()
                self.publisher_.publish(twist)
                return

            # continue moving to safe position
            twist = self.compute_backup_twist()
            self.publisher_.publish(twist)
            return

        # safepositioning not active, check distance
        if self.min_distance < self.threshold:
            self.get_logger().info("Obstacle too close, going back to safe position")

            self.safepositioning_active = True
            twist = self.compute_backup_twist()
            self.publisher_.publish(twist)
            return

    def compute_backup_twist(self):
        twist = Twist()

        if self.current_x is None or self.current_y is None:
            # stop if current position is unknown
            return twist

        dx = self.safe_position_x - self.current_x
        dy = self.safe_position_y - self.current_y
        distance = (dx**2 + dy**2)**0.5

        if distance < 0.05:
            return twist

        k = 0.5  # gain for proportional control

        angle_to_goal = math.atan2(dy, dx)

        twist.linear.x = k * distance
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