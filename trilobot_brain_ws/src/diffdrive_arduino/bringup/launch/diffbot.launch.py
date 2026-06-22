from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node,LifecycleNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    package_name = 'diffdrive_arduino'

    left_image_throttle = Node(
            package="topic_tools",
            executable="throttle",
            name='left_image_throttle',
            arguments=['messages', '/left/image_rect/compressed', '10'],
            output='screen',
        )
    
    right_image_throttle = Node(
            package="topic_tools",
            executable="throttle",
            name='right_image_throttle',
            arguments=['messages', '/right/image_rect/compressed', '10'],
            output='screen',
        )
    


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare(package_name), 'config', 'nav2_params.yaml']),
            'autostart': 'true',         # Auto-activates lifecycle nodes
            'sim_time':'false',
        }.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py'])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare(package_name),'config','mapper_params_online_async.yaml'])
        }.items()
    )

    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('diffdrive_arduino'), 'launch', 'imu_filter_oakd.launch.py'])
        ]),
    )

   # Camera parameters
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('diffdrive_arduino'), 'config', 'oak_camera.yaml']),
        description='Path to OAK camera params'
    )

    oak_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', 'oakd_camera.launch.py'])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([FindPackageShare(package_name), 'config', 'oak_camera.yaml']),
            'camera_model': 'OAK-D-S2',
            'pass_tf_args_as_params': 'true',
            'imu_from_descr': 'true',
        }.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

    # Lidar launcher
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    lidar_lanch_arguments = [
            DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
            DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
            DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
            DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),
            DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),
            DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
            DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        ]

    lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode
                        }],
            output='screen'
        )

    robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("diffdrive_arduino"), "urdf", "diffbot.urdf.xacro"]
                ),
            ]
        )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("diffdrive_arduino"),
                "config",
                "diffbot_controllers.yaml",
            ]
        )

    rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("diffdrive_arduino"), "rviz", "diffbot.rviz"]
        )

    control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
        )

    robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
        )

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

    robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            )

    head_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
            )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner, gripper_spawner, head_spawner],
                )
            )

    nodes = lidar_lanch_arguments + [
            twist_mux,
            lidar_node,
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            # delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            declare_params_file,
            oak_camera_launch,
            imu_filter_launch,
            nav2_launch,
            slam_toolbox_launch,
            left_image_throttle,
            right_image_throttle,
        ]

    return LaunchDescription(nodes)