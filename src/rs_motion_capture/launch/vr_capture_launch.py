from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(
        package='rs_hci',
        executable='robot_hci_node',
        name='robot_hci_node',
        output='screen',
    ))
    ld.add_action(DeclareLaunchArgument(
        'arm_mode',
        default_value='right',
        description='arm number'
    ))
    ld.add_action(DeclareLaunchArgument(
        'hand_type',
        default_value='rs_hand',
        description='hand type'
    ))
    ld.add_action(DeclareLaunchArgument(
        'control_mode',
        default_value='controller',
        description='control'
    ))
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'ros2',
                'launch',
                'ros_tcp_endpoint',
                'endpoint.py',
            ],
            output='screen'
        )
    )
    control_mode = LaunchConfiguration('control_mode')
    is_hand = PythonExpression(['"', control_mode, '" == "hand_tracking"'])
    # hand_retarget_node = Node(
    #     package='my_robot_pkg',
    #     executable='robot_node',
    #     # 当mode为'real_robot'时启动该节点
    #     condition=IfCondition(is_real_robot),
    #     parameters=[{
    #         'use_sim_time': use_sim_time  # 将参数值传递给节点参数
    #     }]
    # )
    # if LaunchConfiguration('control_mode') == "hand_tracking":

    # 启动 串口控手节点
    ld.add_action(Node(
        package='smart_hand_ros_cpp',
        executable='usb_control',
        name='usb_control',
        parameters=[{
            'config': '/teleop_ws/src/smart_hand_ros2_cpp/config/hand_left_dexpilot.yml',
        }],
        output='screen'
    ))
    ld.add_action(
        ExecuteProcess(
            cmd=[
                'python3', '/teleop_ws/src/smart_hand_ros2_cpp/script/dex_retarget_pure.py',
                '--config', '/teleop_ws/src/smart_hand_ros2_cpp/config/hand_left_dexpilot.yml',
                '--type', 'quest3'
            ],
            output='screen',
            condition=IfCondition(is_hand),
        )
    )

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_pub',
        arguments=[
            '0', '0', '0.0',       # x y z
            '0.5', '-0.5', '-0.5', '0.5', # yaw pitch roll
            'map', 'quest3'        # parent frame, child frame
        ]
    ))

    quest3_node = Node(
            package='rs_motion_capture',
            executable='quest3_teleop_node.py',
            name='quest3_teleop_node',
            parameters=[
                {
                    "arm_mode": LaunchConfiguration('arm_mode'),
                    "control_mode": LaunchConfiguration('control_mode'),
                    "hand_type": LaunchConfiguration('hand_type'),
                }
            ],
            output='screen',
        )
    ld.add_action(quest3_node)

    rs_teleop_launch_package = FindPackageShare('rs_motion_capture')
    default_rviz_config_path = PathJoinSubstitution([rs_teleop_launch_package, 'config', "rviz" ,'quest3.rviz'])
    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    # 添加事件处理器，当coord_calib_node退出时关闭所有节点
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=quest3_node,
            on_exit=[
                Shutdown(reason="calib_node has exited")
            ]
        )
    )
    ld.add_action(shutdown_handler)
    return ld
