import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package name
    package_name = "project1_description"
    bringup_package = 'project1_bringup'

    # Get the package directory
    pkg_share = get_package_share_directory(package_name)
    bringup_share = get_package_share_directory(bringup_package)

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, "urdf", "robot_gazebo.urdf")
    controller_config_file = os.path.join(bringup_share, 'config', 'carlikebot_controllers.yaml')

    # Set Gazebo model path to include our package
    gazebo_model_path = os.path.join(pkg_share, "..")

    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    z_pose = LaunchConfiguration("z_pose", default="0.3")

    # Read URDF file
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    # Set environment variable for Gazebo to find meshes
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[os.environ.get("GAZEBO_MODEL_PATH", ""), os.pathsep, gazebo_model_path],
    )

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
    )
    
    # Controller Manager (ros2_control) - FIXED: Removed robot_description parameter
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            controller_config_file
        ],
        output='screen'
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "my_robot",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Bicycle Steering Controller Spawner
    bicycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bicycle_steering_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # TF Relay (optional, for odometry)
    tf_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/bicycle_steering_controller/tf_odometry', '/tf'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "x_pose", default_value="0.0", description="X position of robot spawn"
            ),
            DeclareLaunchArgument(
                "y_pose", default_value="0.0", description="Y position of robot spawn"
            ),
            DeclareLaunchArgument(
                "z_pose", default_value="0.3", description="Z position of robot spawn"
            ),
            set_gazebo_model_path,
            gazebo,
            spawn_entity,
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            bicycle_controller_spawner,
            tf_relay
        ]
    )
