from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rs_motion_capture',
            executable='calib_infrared_tracker_v2.py',
            name='calib_infrared_tracker',
            # parameters=[{
            #     'calib_mode': LaunchConfiguration('calib_mode'),
            #     'check_mode': LaunchConfiguration('check_mode'),
            #     'collector': LaunchConfiguration('collector'),
            #     'glove_type': LaunchConfiguration('glove_type'),
            #     'config': LaunchConfiguration('config'),
            # }],
            output='screen',
        )
    ])
