# assignment1_rt

This package is part of the *Research Track I* course assignment.  
It contains three ROS2 packages:

- The simulation environment (GAZEBO/RVIZ)
- Assigment2 nodes that allow to perform the requested tasks
- Custom interfaces that contains messages and services crated 

---

## 1. Package Assignment2 Overview

The package `assignment2_rt` contains:

- `node1`: **move_robot**
This node allows the user to control the robot by providing linear and angular velocity inputs.
It also calls the `average_server`, which asks the user whether they want to display the average of the last five velocity commands sent to the simulation.

- `node2`: **control**
This node manages the robot’s movement by preventing collisions with obstacles.
It uses the laser scanner data to detect nearby obstacles. If the robot gets too close, its previous position is restored.
The node also calls the `threshold_service`, which lets the user choose a suitable threshold value for obstacle avoidance.
Additionally, it publishes on the `/obstacle_info` topic the distance and orientation of the closest obstacle to the robot.
 
- `node3`: **average_server**
When called, this node provides the average of the last five velocity inputs.
It listens to the `/cmd_user_vel topic`, where the user’s velocity commands are published, and computes the average based on the most recent values.

- `node4`: **threshold_service**
This node asks the user to provide a threshold value to be used for obstacle avoidance.
When called, it returns the user‑selected value, which is then used by the `control` node.
---

## 2. Directory Structure

```bash
ASSIGNMENT2_RT
.
├── assignment2_rt
│   ├── __init__.py
│   ├── average_server.py
│   ├── control.py
│   ├── move_robot.py
│   └── threshold_service.py
├── launch
│   └── launch.py
├── package.xml
├── setup.py
└── test

```
---
## 3. Installation

Clone this repository into your ROS2 workspace:

```bash
cd ~/ws/src
git clone https://github.com/Pedemontemarina/assignment2_rt.git assignment2_rt
cd ..
colcon build 
```
Source your workspace:
```bash
source install/local_setup.bash
```
---
## 4. Usage
Run the simulation environment:
```bash 
ros2 launch bme_gazebo_sensors1 spawn_robot.launch.py
```
A ROS2 launch file was appositely created to run all the nodes:
```bash 
ros2 launch assignment2_rt launch.py
```

---
## 5. Nodes Description 

**MoveRobot Node**

The move_robot node provides an interactive interface to manually control the robot from the terminal. It asks the user for linear and angular velocities and publishes the corresponding Twist messages to /cmd_vel to move the robot, and to /cmd_user_vel to keep track of user-issued commands.
Each command is published for 1 second, after which the robot is automatically stopped by sending a zero-velocity command.
The node also validates user input to ensure only correct numerical values are accepted. Additionally, it can call the get_average service to retrieve and display the average linear and angular velocities of the last five commands sent by the user.

**Control Node**

The control node is responsible for obstacle monitoring and safety control. It subscribes to laser scan data to detect the closest obstacle and continuously compares its distance with a configurable safety threshold obtained from the get_threshold service.
User velocity commands are monitored through the /cmd_user_vel topic, while the actual robot velocity is read from /cmd_vel.
When an obstacle is detected closer than the threshold, the node activates a backup maneuver, automatically publishing inverse linear and angular velocities to move the robot away from the obstacle until a safe distance is restored.
The node also publishes periodic obstacle information (distance, direction, and threshold) on the /obstacle_info topic using a custom message, providing real-time feedback about the obstacle avoidance behavior.

**AverageServer Node**

The average_server node provides a service to compute the average of the most recent user velocity commands. It subscribes to the /cmd_user_vel topic and stores the last five linear and angular velocity commands using a fixed-size queue.
When the get_average service is called, the node returns the average linear and angular velocities computed from the stored commands. If no commands have been received yet, the service returns zero values.
This node allows other components to analyze recent user inputs without interfering with the robot’s motion.

**ThresholdServer Node**

The threshold_server node provides a service that allows the user to set the safety distance threshold used for obstacle avoidance. When it is called, the node interactively asks the user to input a new threshold value via the terminal.
The selected threshold is then returned to the requesting node and used to determine when an obstacle is considered too close. 

---
## 6. Requirements

ROS2
XLaunch (for graphical visualization)

---
## 7. Author

Pedemonte Marina

Research Track I – Assignment 2

