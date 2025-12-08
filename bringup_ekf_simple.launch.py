"""
Simple EKF bringup launch file for UGV
Uses EKF to fuse RF2O laser odometry with IMU data
Does NOT require imu_complementary_filter package
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='false',  # EKF publishes odom->base_footprint
        description='Whether base_node publishes odom TF (false when using EKF)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='bringup',
        description='Choose which rviz configuration to use'
    )

    # Include the robot state launch from the ugv_description package
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    )

    # UGV bringup node
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
    )

    # UGV driver node
    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
    )

    # Include laser lidar launch file
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ldlidar.launch.py')
        )
    )

    # Include RF2O laser odometry - publishes to /odom_rf2o
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        )
    )

    # Base node with EKF-compatible settings
    # pub_odom_tf=false because EKF will publish the odom->base_footprint transform
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_ekf',  # Use EKF-compatible base node
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # EKF node for sensor fusion
    # Fuses RF2O laser odometry with IMU data
    ekf_config = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'param',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')]  # Remap to /odom for compatibility
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        robot_state_launch,
        bringup_node,
        driver_node,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        base_node,
        ekf_node
    ])
