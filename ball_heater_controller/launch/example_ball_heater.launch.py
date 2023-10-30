from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    workspace = get_package_share_directory("ball_heater_controller").split("/install")[
        0
    ]
    config_file = f"{workspace}/src/ball_heater_controller/ball_heater_controller/config/example_config.yaml"

    example_ball_heater_client = Node(
        package="ball_heater_controller",
        executable="ball_heater_client",
        name="ball_heater_client",
        parameters=[config_file],
    )
    ld.add_action(example_ball_heater_client)

    ball_heater = Node(
        package="ball_heater_controller",
        executable="ball_heater_node",
        name="ball_heater_node",
        parameters=[config_file],
    )
    ld.add_action(ball_heater)

    return ld
