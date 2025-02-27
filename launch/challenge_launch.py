# Import libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    groups = ["group1", "group2", "group3"] # Define 3 groups of nodes for the launch
    nodes = []
    for group in groups:
        nodes.append(Node( # Initialize dc motor nodes
            package='motor_control',
            executable='dc_motor',
            namespace=group,
            output='screen'
        ))
        nodes.append(Node( # Initialize set point nodes
            package='motor_control',
            executable='set_point',
            namespace=group,
            output='screen'
        ))
        nodes.append(Node( # Initialize controller nodes 
            package='motor_control',
            executable='controller',
            namespace=group,
            output='screen'
        ))
    rqt_graph_node = ExecuteProcess( # Initialize rqt_graph
        cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
        output="screen"
    )
    rqt_reconfigure_node = ExecuteProcess( # Initialize rqt_reconfigure
        cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
        output="screen"
    )
    plotjuggler_node = ExecuteProcess( # Initialize PlotJuggler
        cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
        output="screen"
    )
    return LaunchDescription(nodes + [
        rqt_graph_node,
        rqt_reconfigure_node,
        plotjuggler_node
    ])