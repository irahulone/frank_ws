from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            # Node responsible for serial communication to the roboteq drivers.
            package="locomotion_core",
            executable="cmd_roboteq",
        ),
        Node(
            # Node responsible for calculating and publishing the velocities of the rover.
            package="locomotion_core",
            executable="rover_velocity",
        ),
    ])
