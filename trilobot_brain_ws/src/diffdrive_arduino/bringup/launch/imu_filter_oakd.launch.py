import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('diffdrive_arduino'), 'config')

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
                remappings=[('/imu/data_raw','/imu')],
            ),
             launch_ros.actions.Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(config_dir, 'imu_chassis_filter.yaml')],
                remappings=[('/imu/data_raw','/chassis_imu/raw'),
                            ('/imu/data','chassis_imu/data'),
                            ('/imu/mag','chassis_imu/mag')],
            ) ,

            launch_ros.actions.Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(config_dir, 'ekf.yaml')],
                remappings=[('/odometry/filtered', '/odometry/filtered')],
            )
        ]
    )