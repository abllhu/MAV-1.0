import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , OpaqueFunction
from launch.substitutions import LaunchConfiguration

def noisy_controller(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error").perform(context)
    wheel_separation_error = LaunchConfiguration("wheel_separation_error").perform(context)
    wheel_radius = LaunchConfiguration("wheel_radius").perform(context)
    wheel_separation = LaunchConfiguration("wheel_separation").perform(context)

    noisy_controller = Node(
        package="mav_controller",
        executable="noisy_controller.py",
        parameters=[{
            "wheel_radius_error": wheel_radius_error + wheel_radius,
            "wheel_separation_error": wheel_separation_error + wheel_separation,
            "use_sim_time": use_sim_time
        }]
    )

    return [noisy_controller]


def generate_launch_description():

    wr_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
        description="Radius of the wheels"
    )
    ws_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
        description="Separation between the wheels"
    )

    wr_arg_error = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.003",
        description="Radius error of the wheels"
    )
    ws_arg_error = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.01",
        description="Separation error between the wheels"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_sim_time = LaunchConfiguration("use_sim_time")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )

    controller = Node(
        package="mav_controller",
        executable="controller.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "use_sim_time": use_sim_time
        }]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            use_sim_time_arg,
            wr_arg,
            ws_arg,
            wr_arg_error,
            ws_arg_error,
            controller,
            joint_state_broadcaster_spawner,
            simple_controller,
            noisy_controller_launch
        ]
    )