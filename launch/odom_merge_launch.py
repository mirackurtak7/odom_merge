import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Lidar launch directory
    rplidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')

    # EKF Configuration
    ekf_config_file = os.path.join(get_package_share_directory('odom_merge'), 'config', 'ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # RF2O Lidar Odometry Node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_lidar',
            'publish_tf': False,  # TF yayını devre dışı
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom_lidar',
            'init_pose_from_topic': '',
            'freq': 50.0
        }]
    )

    # Encoder node
    enc_node = Node(
        package='tortoisebot_firmware',
        executable='encoder.py',
        name='encoder',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyUSB1', 'baud_rate': 115200}]
    )

    # Odom Encoder node
    odom_enc_node = Node(
        package='tortoisebot_firmware',
        executable='odom_encoder.py',
        name='odom_encoder',
        output='screen'
    )

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}]
    )

    # Include Lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar_s2_launch.py')),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '1000000',
            'frame_id': 'lidar',
            'use_sim_time': 'True'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time if true'),
        rf2o_node,
        enc_node,
        odom_enc_node,
        ekf_node,
        lidar_launch
    ])

