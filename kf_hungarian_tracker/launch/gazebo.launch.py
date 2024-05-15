from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os
import datetime


def generate_launch_description():
    # Define default parameters
    robot_name = "example_robot"
    robot_description_pkg = "example_robot_description"
    controller_name = "vehicle_controller"
    mcap_directory = '/tmp/mcap/' + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    # Initial pose of the robot
    x_val = LaunchConfiguration("x_val")
    x_val_arg = DeclareLaunchArgument("x_val", default_value="0.0")

    y_val = LaunchConfiguration("y_val")
    y_val_arg = DeclareLaunchArgument("y_val", default_value="0.0")

    z_val = LaunchConfiguration("z_val")
    z_val_arg = DeclareLaunchArgument("z_val", default_value="0.0")

    roll_val = LaunchConfiguration("roll_val")
    roll_val_arg = DeclareLaunchArgument("roll_val", default_value="0.0")

    pitch_val = LaunchConfiguration("pitch_val")
    pitch_val_arg = DeclareLaunchArgument("pitch_val", default_value="0.0")

    yaw_val = LaunchConfiguration("yaw_val")
    yaw_val_arg = DeclareLaunchArgument("yaw_val", default_value="0.0")

    # Declare arguments
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument("world", default_value="farm")

    gui = LaunchConfiguration("gui")
    gui_arg = DeclareLaunchArgument(
        name="gui", default_value="False", description="Whether to start the simulator with a GUI."
    )

    # Lets gazebo know where your meshes are
    gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        PathJoinSubstitution(
            [FindPackageShare("example_robot_description"), '..'])
    )

    localization_params = LaunchConfiguration("localization_yaml")
    localization_params_arg = DeclareLaunchArgument(
        'localization_yaml',
        default_value=os.path.join(
            get_package_share_directory("example_robot_config"),
            "config",
            "localization",
            "local.yaml"
        ),
        description='Path to the localization ros params YAML file.'
    )
    odom_localizer_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odom_filter_node',
        output='screen',
        arguments=['--ros-args', '--log-level', "INFO"],
        parameters=[localization_params]
    )

    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("polymath_sim"),
                 "launch/spawn_robot.launch.py"]
            )
        ),
        launch_arguments={
            "robot_description": robot_description_pkg,
            "robot_name": robot_name,
            "controller_name": controller_name,
            "x_val": x_val,
            "y_val": y_val,
            "z_val": z_val,
            "roll_val": roll_val,
            "pitch_val": pitch_val,
            "yaw_val": yaw_val,
            "world": world,
            "gui": gui,
        }.items(),
    )

    velocity_publisher_node = Node(
        package='example_robot_sim',
        executable='velocity_publisher_node.py',
        name='velocity_publisher_node'
    )

    config = os.path.join(
        get_package_share_directory("kf_hungarian_tracker"),
        "config",
        "kf_hungarian.yaml",
    )

    kf_hungarian_node = Node(
        package="kf_hungarian_tracker",
        name="kf_hungarian_node",
        executable="kf_hungarian_node",
        parameters=[config],
    )

    # Add actions
    ld = LaunchDescription()
    ld.add_action(localization_params_arg)
    ld.add_action(odom_localizer_node)
    ld.add_action(gazebo_model_path)
    ld.add_action(x_val_arg)
    ld.add_action(y_val_arg)
    ld.add_action(z_val_arg)
    ld.add_action(roll_val_arg)
    ld.add_action(pitch_val_arg)
    ld.add_action(yaw_val_arg)
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(spawn_robot)
    ld.add_action(kf_hungarian_node)

    return ld
