from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()


    ld.add_action(Node(
            package='rs_hci',
            executable='robot_hci_node',
            name='robot_hci_node',
            output='screen',
        ))

    calib_node = Node(
            package='rs_motion_capture',
            executable='survive_cli.py',
            name='survive_cli',
            output='screen',
        )
    ld.add_action(calib_node)

    # 添加事件处理器，当coord_calib_node退出时关闭所有节点
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=calib_node,
            on_exit=[
                Shutdown(reason="calib_node has exited")
            ]
        )
    )
    ld.add_action(shutdown_handler)
    return ld
