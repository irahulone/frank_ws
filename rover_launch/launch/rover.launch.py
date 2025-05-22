import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    ld = LaunchDescription()
    
    # Nodes
    rover_vel = Node(
        package="locomotion_core",
        executable="rover_velocity",
    )

    cmd_roboteq = Node(
        package="locomotion_core",
        executable="cmd_roboteq",
    )

    # include another launch file
    launch_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_core'),
                '/home/pioneer-ii/frank_ws/src/teleop_core/launch/teleop_node.launch.py'))
    )

    

    ld.add_action(launch_joy)
    ld.add_action(cmd_roboteq)
    ld.add_action(rover_vel)

    return ld


