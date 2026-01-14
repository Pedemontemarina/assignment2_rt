from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'bash -c "ros2 run assignment2_rt move_robot --ros-args -r /cmd_vel:=/robot_vel"'
            ],
            output='screen'),
        
         ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'bash -c "ros2 run assignment2_rt threshold_service"'
            ],
            output='screen'),

        Node(
            package='assignment2_rt',
            executable='control',
            name='robot_controller',
            remappings=[ ('/cmd_vel', '/robot_vel')]),
        
      

        Node(
            package='assignment2_rt',
            executable='average_server',
            name='average_server')
        ])