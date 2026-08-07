from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("project_perry", package_name="umrt-project-perry-moveit-config").to_moveit_configs()

    joy_params = os.path.join(get_package_share_directory('umrt-arm-joystick-operator'),'launch','joystick.yaml')
    servo_params = {
        "moveit_servo": ParameterBuilder("umrt-arm-joystick-operator")
        .yaml("launch/servo_params.yaml")
        .to_dict()
    }
    servo_params_gripper = {
        "moveit_servo": ParameterBuilder("umrt-arm-joystick-operator")
        .yaml("launch/servo_params_gripper.yaml")
        .to_dict()
    }

    joy_node = Node(
            package='joy',
            executable='game_controller_node',
            parameters=[joy_params],
            remappings=[("/joy", "/arm/joy"), ("/joy/set_feedback", "/arm/joy/set_feedback")]
         )
    joystick_operator_node = Node(
            package='umrt-arm-joystick-operator',
            executable='umrt-arm-joystick-operator',
            name = 'joystick_operator_node',
            parameters=[joy_params]
            )

    # babbler_node = Node(
    #     package="ros2_j1939_babbler",
    #     executable="static_bridge",
    #     parameters=[{"sensor_name": "arm/babbler"}],
    #     output="both",
    #     remappings=[("/from_can_bus", "/arm/from_can_bus"), ("/to_can_bus", "/arm/to_can_bus")]
    # )

    # ros2_socketcan_launch_path = os.path.join(
    #     get_package_share_directory('ros2_socketcan'),
    #     'launch',
    #     'socket_can_bridge.launch.xml'
    # )

    # ros2_socketcan_nodes = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(ros2_socketcan_launch_path),
    #     launch_arguments=[
    #         ("interface", "dcan0"),
    #         ("to_can_bus_topic", "arm/to_can_bus"),
    #         ("from_can_bus_topic", "arm/from_can_bus")
    #     ]
    # )

    # NOTE: Servo node is disabled by default until you run
    # ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
    )
    servo_node_gripper = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node_gripper",
        parameters=[
            servo_params_gripper,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
    )


    return LaunchDescription([
        joy_node,
        joystick_operator_node,
        servo_node,
        servo_node_gripper,
        # can_ppm_node,
        # babbler_node,
        # ros2_socketcan_nodes
    ])

