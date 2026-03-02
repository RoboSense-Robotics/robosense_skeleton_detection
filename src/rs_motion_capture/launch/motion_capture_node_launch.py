from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    collector_arg = DeclareLaunchArgument(
        'collector',
        default_value='zed',
        description='采集员'
    )

    calib_mode_arg = DeclareLaunchArgument(
        'calib_mode',
        default_value='false',
        description='是否进入标定模式'
    )

    check_mode_arg = DeclareLaunchArgument(
        'check_mode',
        default_value='false',
        description='是否进入标定检测模式'
    )

    glove_type_arg = DeclareLaunchArgument(
        'glove_type',
        default_value='noiton',
        description='动捕手套类型，支持noiton, manus'
    )

    config_path_arg = DeclareLaunchArgument(
        'config',
        default_value='config/config.yaml',
        description='读取config路径'
    )

    center_z_arg = DeclareLaunchArgument(
        'ws_center_z',
        default_value='1.35',
        description='工作台高度'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('rs_motion_capture'),
            'config/rviz',
            'motion_capture.rviz' 
        ]),
        description='RViz 配置文件路径'
    )

    return LaunchDescription([
        collector_arg,
        calib_mode_arg,
        check_mode_arg,
        glove_type_arg,
        config_path_arg,
        center_z_arg,
        rviz_config_arg,
        Node(
            package='rs_motion_capture',
            executable='motion_capture_node',
            name='motion_capture_node',
            parameters=[{
                'calib_mode': LaunchConfiguration('calib_mode'),
                'check_mode': LaunchConfiguration('check_mode'),
                'collector': LaunchConfiguration('collector'),
                'glove_type': LaunchConfiguration('glove_type'),
                'config': LaunchConfiguration('config'),
                'ws_center_z': LaunchConfiguration('ws_center_z')
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        )
    ])