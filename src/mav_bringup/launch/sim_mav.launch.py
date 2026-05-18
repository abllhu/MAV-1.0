import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():


    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mav_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mav_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "wheel_radius": "0.033",
            "wheel_separation": "0.17",
            "wheel_radius_error": "0.003",
            "wheel_separation_error": "0.01"
        }.items()
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mav_controller"),
            "launch",
            "joystick.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )


    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mav_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mav_mapping"),
            "launch",
            "slam_toolbox.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        rviz, 
    ])