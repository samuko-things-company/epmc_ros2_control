import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # delare any path variable
    bringup_pkg_path = get_package_share_directory('epmc_demo_bot_bringup')
  
    # create needed nodes or launch files
    base_control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(bringup_pkg_path,'launch','base_control.launch.py')]
            )
    )

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(bringup_pkg_path,'launch','rviz.launch.py')]
            )
    )

     # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(base_control_launch)
    ld.add_action(rviz_launch)


    return ld      # return (i.e send) the launch description for excecution