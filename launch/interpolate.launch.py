from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
import socket

def generate_launch_description():
    frame_name = "wrist_3_link"
    print("Using ",frame_name," as ee_frame_name")
    return LaunchDescription(
        [
            Node(
                package="spline_executor",
                executable="spline_executor",
            ),
            Node(
                package="interpolator",
                executable="interpolator",
            ),
            Node(
                package="interpolator",
                executable="tool0_broadcaster",
                parameters=[{"ee_frame_name" : frame_name}],

            )
        ]
    )
