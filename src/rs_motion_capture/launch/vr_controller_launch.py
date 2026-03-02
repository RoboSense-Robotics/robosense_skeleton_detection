from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_pub',
    #     arguments=[
    #         '0', '0', '0.0',       # x y z
    #         '0.5', '-0.5', '-0.5', '0.5', # yaw pitch roll
    #         'map', 'quest3'        # parent frame, child frame
    #     ]
    # ))
# ros2 launch ros_tcp_endpoint endpoint.py
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
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_pub',
        arguments=[
            '0', '0', '0.0',       # x y z
            '0.5', '-0.5', '-0.5', '-0.5', # yaw pitch roll
            'quest3', 'map'         # parent frame, child frame
        ]
    ))

    quest3_node = Node(
            package='rs_motion_capture',
            executable='quest3_teleop_node.py',
            name='quest3_teleop_node',
            parameters=[
                {"control_mode": "controller"}
            ],
            output='screen',
        )
    ld.add_action(quest3_node)

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
