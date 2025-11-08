import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package names
    package_name = 'project1_description'
    bringup_package = 'project1_bringup'

    # Get package directories
    pkg_share = get_package_share_directory(package_name)
    bringup_share = get_package_share_directory(bringup_package)

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_rviz.urdf')

    # Path to RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf_config_description.rviz')

    # Path to controller config
    controller_config_file = os.path.join(bringup_share, 'config', 'carlikebot_controllers.yaml')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Controller Manager (ros2_control)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time},
            controller_config_file
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Bicycle Steering Controller
    bicycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bicycle_steering_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # TF Relay (to forward controller odometry TFs to /tf)
    tf_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/bicycle_steering_controller/tf_odometry', '/tf'],
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        bicycle_controller_spawner,
        tf_relay,
        rviz
    ])