# assignment1_rt

This package is part of the *Research Track I* course assignment.  
It contains three ROS2 packages:

- The simulation environment (GAZEBO/RVIZ)
- Assigment2 nodes
- Custom interfaces

---

## 1. Package Assignment2 Overview

The package `assignment2_rt` contains:

- `node1`: **move_robot** 

- `node2`: **control**  
 
- `node3`: **average_server**  

- `node4`: **threshold_service**  
---

## 2. Directory Structure

```bash
ASSIGNMENT2_RT
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
cd ~/ros_ws/src
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




---
## 6. Requirements

ROS2

turtlesim package installed

---
## 7. Author

Pedemonte Marina

Research Track I – Assignment 2

