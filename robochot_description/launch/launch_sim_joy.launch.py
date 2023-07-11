import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "robochot_description"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # include gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                )
            ]
        )
    )

    # run spawner node
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robochot_sim"],
        output="screen"
    )

    # run spawner node
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    teleop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("teleop_twist_joy"), 
                    "launch", 
                    "teleop-launch.py"
                )
            ]
        ),
        launch_arguments={"joy_config": "xbox", "joy_vel": "/diff_cont/cmd_vel_unstamped"}.items()
    )
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arg
    # )
    # launch
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            teleop_twist_joy,
        ]
    )