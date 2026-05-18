from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    mav_controller_dir = get_package_share_directory("mav_controller")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time")
    

    # Declare the suitable configurations for the joy node and the joy teleop node
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("mav_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("mav_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    # Include the suitable launch files for the joy node and the joy teleop node
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py"),
        launch_arguments={
            "cmd_vel_out": "mav_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(mav_controller_dir, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(mav_controller_dir, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(mav_controller_dir, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    
    twist_relay = Node(
        package="mav_controller",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time"),}]
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay
        ]
    )