import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_vio_ekf')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    tb3_description_share = get_package_share_directory('turtlebot3_description')

    urdf_path = os.path.join(tb3_description_share, 'urdf', 'turtlebot3_waffle.urdf')
    world_path = os.path.join(tb3_gazebo_share, 'worlds', 'turtlebot3_world.world')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    gui = LaunchConfiguration('gui', default='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gui': gui}.items()
    )

    # The apt-packaged turtlebot3_description .urdf files contain an
    # unresolved '${namespace}' placeholder on every link/frame name,
    # normally string-substituted by ROBOTIS's own bringup launch files
    # for multi-robot namespacing. Read the file ourselves and strip it
    # (single-robot sim -> empty namespace) rather than shelling out to
    # xacro/cat, which would pass the placeholder through literally and
    # break every frame name (base_link, camera_link, etc.).
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read().replace('${namespace}', '')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'turtlebot3_waffle',
                   '-x', '-2.0', '-y', '-0.5', '-z', '0.02',
                   '-timeout', '60'],
    )

    imu_relay = Node(
        package='tb3_vio_ekf',
        executable='imu_covariance_relay',
        name='imu_covariance_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # NOTE: package name for this executable varies by rtabmap_ros release:
    # newer split releases use 'rtabmap_odom', older monolithic ones use
    # 'rtabmap_ros'. Run `ros2 pkg executables | grep rgbd_odometry` to check
    # which applies to your install, and adjust `package=` below if needed.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': False,   # the EKF node owns odom->base_footprint TF, not this node
            'approx_sync': True,
            'subscribe_depth': True,
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('odom', '/rtabmap/odom'),
        ],
    )

    vo_relay = Node(
        package='tb3_vio_ekf',
        executable='vo_odom_relay',
        name='vo_odom_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        imu_relay,
        rgbd_odometry,
        vo_relay,
        ekf_node,
        rviz,
    ])
