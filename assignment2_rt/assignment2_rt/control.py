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
import time

class Control(Node):

    def __init__(self):
        super().__init__('control') # name of the node
        
        self.min_distance = float('inf')
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # laser scanner subscriber - to read the distances from obstacles
        self.subscription = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        
        # velocity subscriber - to get the position/velocity of the robot
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.velocity_callback,10)

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

    
    def publish_obstacle_info(self):
        # publish info about the closest obstacle
        distance_msg = Distance()
        distance_msg.distance = self.min_distance
        distance_msg.direction = define_direction_from_index(self.min_index)
        distance_msg.threshold = self.threshold
        self.publisher_info.publish(distance_msg)
        self.get_logger().info(f"Publishing obstacle info: distance={self.min_distance}, direction={distance_msg.direction}, threshold={self.threshold}")
    
    def control_loop(self):
        # check if the minimum distance is below the threshold
        # if so, move the robot back to the last safe position
        if self.min_distance < self.threshold:
            self.get_logger().info("Too close to obstacle, moving back to safe position")
            
            # create a Twist message to move the robot back
            twist = Twist()
            twist.linear.x = -self.linear_velocity 
            twist.angular.z = -self.angular_velocity  
            self.publisher_.publish(twist)
            time.sleep(1)  # move back for 1 second
            # stop the robot
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
        

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