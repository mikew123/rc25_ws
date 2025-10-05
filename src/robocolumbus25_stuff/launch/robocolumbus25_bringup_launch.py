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

    # from nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    bridge_bringup_dir = get_package_share_directory('rosbridge_server')
    bridge_launch_dir = os.path.join(bridge_bringup_dir, 'launch')
 
    # Get the text of the robot description URDF - robot_stat_publisher does not open a file
    with open('urdfs/rc25.urdf','r') as infp:
        robot_desc = infp.read()

    return launch.LaunchDescription([

        # ## External launch files: NAV2, SLLIDAR, CONESLAYER

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_launch_dir,
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'params_file': 'config/rc25_params.yaml',
                "map": "maps/2000x2000_empty_map.yaml",
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '../sllidar_ros2/launch/sllidar_s3_launch.py'
            ),
            launch_arguments={
                'frame_id': 'lidar_link'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '../ros_coneslayer/launch/coneslayer_publisher.launch.py'
            ),
            launch_arguments={
                'namespace': 'coneslayer'
            }.items()
        ),


        #### MY ROBOT RC25 packages

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_speaker_node',
            name='speaker_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_controller_node',
            name='controller_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_tof_node',
            name='tof_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_nav_node',
            name='rc25_nav_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_cone_node',
            name='cone_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_imu_gps_node',
            name='imu_gps_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_wheel_controler_node',
            name='wheel_controler_node',
            namespace="",
        ),

        launch_ros.actions.Node(
            package='robocolumbus25_stuff',
            executable='robocolumbus25_teleop_node',
            name='teleop_node',
            namespace="",
        ),


        #### ROS2 packages

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':robot_desc,
                }],
            remappings=[('/robot_description', '/rc25/robot_description')],
        ),

        launch_ros.actions.Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux'
        ),
  
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='efk_odom',
            parameters=[
                'config/efk_config.yaml'
            ]
        ),

        ### This did not function well, the cmd line launch was added to .bashrc -> works
        # # Bring up rosapi and rosbridge to allow access from visual code and foxglove
        # launch_ros.actions.Node(
        #     package='rosapi',
        #     executable='rosapi_node',
        #     name='rosapi_node',
        # ),

        # # ros2 launch rosbridge_server rosbridge_websocket_launch.xml    
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource(
        #         os.path.join(
        #             bridge_launch_dir,
        #             "rosbridge_websocket_launch.xml",
        #         )
        #     )
        # ),


    ])

