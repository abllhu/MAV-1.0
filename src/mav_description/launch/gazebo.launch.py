from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mav_description = get_package_share_directory("mav_description")
    ros_distro = os.environ.get("ROS_DISTRO")
    gazebo_control_humble = "true" if ros_distro == "humble" else "false"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(mav_description, "urdf", "mav.urdf.xacro"),
        description="Absolute path to mav urdf file"
    )

    gazebo_env_var = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(mav_description).parent.resolve())]
    )

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            LaunchConfiguration("model"),
            " ",
            "gazebo_control_humble:=",
            gazebo_control_humble
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    gz_spawn_mav =  Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "mav"],
        )
            
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/imu', '/imu_output'),
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_env_var,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_mav,
        gz_ros2_bridge,
    ])