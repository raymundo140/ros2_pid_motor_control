from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'motor_control'
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, 'config', 'controller_params.yaml')

    motor_node = Node(
        package=package_name,
        executable='dc_motor',
        name="motor_sys",
        output='screen',
        parameters=[config_file]
    )

    sp_node = Node(
        package=package_name,
        executable='set_point',
        name="sp_gen",
        output='screen',
        parameters=[config_file]
    )

    ctrl_node = Node(
        package=package_name,
        executable='controller',
        name="ctrl",
        output='screen',
        parameters=[config_file]
    )

    # Automatically launch visualization tools
    rqt_graph_node = ExecuteProcess(
        cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
        output="screen"
    )

    rqt_reconfigure_node = ExecuteProcess(
        cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
        output="screen"
    )

    plotjuggler_node = ExecuteProcess(
        cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
        output="screen"
    )

    return LaunchDescription([
        motor_node,
        sp_node,
        ctrl_node,
        rqt_graph_node,
        rqt_reconfigure_node,
        plotjuggler_node
    ])