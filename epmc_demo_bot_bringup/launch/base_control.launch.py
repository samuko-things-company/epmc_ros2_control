import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # delare any path variable
    bringup_pkg_path = get_package_share_directory('epmc_demo_bot_bringup')
    description_pkg_path = get_package_share_directory('epmc_demo_bot_description') 
  
    # create needed nodes or launch files
    rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(description_pkg_path,'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': 'false'}.items()
    )

    robot_controllers = os.path.join(bringup_pkg_path,'config','epmc_diff_drive_controller.yaml')

    # see -> https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/bringup/launch/diffbot.launch.py
    # see -> https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/epmc_diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/epmc_diff_drive_controller/odom", "/odom"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    epmc_diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["epmc_diff_drive_controller"],
    )    

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_epmc_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[epmc_diff_drive_controller_spawner],
        )
    )


     # Create the launch description and populate
    ld = LaunchDescription()
    

    # Add the nodes to the launch description
    ld.add_action(rsp_launch)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_epmc_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner)


    return ld      # return (i.e send) the launch description for excecution