# .bashrc has an alias "rc25" which builds launches this project
# as well as the delphi_descriptins urdf launch file
# The extra delayed urdf launch fixes the oak frame issues when nav2 is launched
#
# alias rc25=' \
#  cd ~/rc25_ws ; colcon build ; source install/setup.bash ; \
#  parallel --lb ::: \
#  "ros2 launch robocolumbus25_stuff robocolumbus25_bringup_launch.py" \
#  "sleep 5 ; ros2 launch depthai_descriptions urdf_launch.py" ; \
#  wait ; echo "RC25 terminated" \
# '

import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

#added to launch other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

 
    return launch.LaunchDescription([

  
        # Fuse wheel odom and imu data
        # Generate odom -> base_footprint TF
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_local',
            parameters=['config/efk_local_config.yaml'],
            # output topic
            remappings=[("odometry/filtered", "odometry/local")],
        ),

        # Fuse GPS wheel odom and imu data
        # Generate map -> odom TF
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_global',
            parameters=['config/efk_global_config.yaml'],
            # output topic
            remappings=[("odometry/filtered", "odometry/global")],
        ),

        # Transform GPS lat/lon to map coordinates
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=['config/navsat_transform_config.yaml'],
            remappings=[
                # input topics
                ("imu/data", "imu"),
                ("gps/fix", "gps_nav"),
                ("odometry/filtered", "odometry/global"),
                # output topics
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
            ],
        ),


    ])

