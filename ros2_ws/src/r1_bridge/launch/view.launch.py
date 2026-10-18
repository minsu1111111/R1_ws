"""R1 시각화: lowstate_bridge + robot_state_publisher + RViz.

    source env/setup_sim.sh
    ros2 launch r1_bridge view.launch.py                 # 시뮬
    ros2 launch r1_bridge view.launch.py has_head:=true  # 실기 (머리 모터 29, 30)
    ros2 launch r1_bridge view.launch.py rviz:=false     # RViz 없이
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("r1_description")
    bridge_share = get_package_share_directory("r1_bridge")
    with open(os.path.join(desc_share, "urdf", "r1.urdf")) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("has_head", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="r1_bridge",
            executable="lowstate_bridge",
            parameters=[{"has_head": LaunchConfiguration("has_head")}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(bridge_share, "rviz", "r1.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
