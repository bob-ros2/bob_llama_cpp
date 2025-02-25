import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # use config file if provided
    launch_config_yaml = DeclareLaunchArgument('config_yaml', 
        default_value=TextSubstitution(
            text=os.path.join(
                get_package_share_directory("bob_llama_cpp"),
                "config", "llm.yaml")))

    # used namespace for the nodes
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text='/llm'))

    # launch with terminal
    launch_terminal = DeclareLaunchArgument('terminal', 
        default_value="false")

    # nodes

    llm = Node(
        package='bob_llama_cpp',
        executable='llm',
        name='llm',
        namespace=LaunchConfiguration('ns'),
        parameters=[LaunchConfiguration('config_yaml')]
    )

    terminal = Node(
        condition=IfCondition(LaunchConfiguration("terminal")),
        package='bob_llama_cpp',
        executable='terminal',
        name='terminal',
        namespace=LaunchConfiguration('ns'),
        parameters=[LaunchConfiguration('config_yaml')],
        remappings=[
            ('topic_in', 'llm_generator'),
            ('topic_out', 'llm_in')]
    )

    return LaunchDescription([
        launch_config_yaml,
        launch_ns,
        launch_terminal,
        llm,
        terminal,
        RegisterEventHandler( # Shutdown if llm ends
            OnProcessExit(
                target_action=llm,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='llm ended'))
                ]
            )
        )
    ])
