from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration , PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from os import pathsep
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

    world_arg = DeclareLaunchArgument(name="world_name", 
        default_value="empty", 
        description="Gazebo world to load"
        )

    world_path = PathJoinSubstitution([
            mav_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Gazebo needs the model path to find the included meshes in the urdf file
    model_path = [str(Path(mav_description).parent.resolve())]
    model_path += [pathsep + os.path.join(get_package_share_directory("mav_description"), "models")]

    gazebo_env_var = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
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
            "use_sim_time": use_sim_time
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
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
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ('/imu', '/imu_output'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        world_arg,
        gazebo_env_var,
        gazebo,
        gz_ros2_bridge,
        robot_state_publisher_node,
        gz_spawn_mav,
    ])