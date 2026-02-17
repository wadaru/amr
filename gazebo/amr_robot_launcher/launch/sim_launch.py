import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    home = os.path.expanduser('~')
    
    # パスの定義
    world_path = os.path.join(home, 'git/gazebo-rcll/worlds/btr_2025.world')
    models_dir = os.path.join(home, 'git/gazebo-rcll/models')
    teleop_script = os.path.join(home, 'git/amr/gazebo/docker/scripts/teleop_keyboard_robot.sh')

    # 1. Gazebo Simの起動
    # GZ_SIM_RESOURCE_PATHを設定することで、cdしなくてもモデルを読み込めます
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        additional_env={'GZ_SIM_RESOURCE_PATH': models_dir},
        output='screen'
    )

    # 2. Rviz2の起動
    # GZ_SIM_RESOURCE_PATHを設定することで、cdしなくてもモデルを読み込めます
    rviz2 = ExecuteProcess(
        cmd=['rviz2', 'rviz2'],
        output='screen'
    )

    # 3. Teleopを別々のターミナル(terminator)で起動
    # terminator -e の形式を再現します
    def create_teleop_process(robot_name):
        return ExecuteProcess(
            cmd=['terminator', '--new-tab', '-e', f'bash -c "source {home}/.bashrc; source {home}/colcon_ws/install/setup.bash; {teleop_script} {robot_name}; exec bash"'],
            output='screen'
        )

    teleop_robot_2dw1c = create_teleop_process('robot_2dw1c')
    teleop_robot_3dw = create_teleop_process('robot_3dw')
    teleop_robot_4dw = create_teleop_process('robot_4dw')

    # 4. ROS-GZ Bridge の起動
    # ロボット名が含まれた Gazebo トピックを ROS 2 の /robot_name/odom に変換します
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Odomのブリッジ
            '/model/robot_2dw1c/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_3dw/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_4dw/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TFのブリッジ
            '/model/robot_2dw1c/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_3dw/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/robot_4dw/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # --- Lidar (LaserScan) の追加 ---
            '/model/robot_2dw1c/laser@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/robot_3dw/laser@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/robot_4dw/laser@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # --- Lidar (PointCloud2) の追加 ---
            '/model/robot_2dw1c/laser/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/robot_3dw/laser/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/robot_4dw/laser/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            # Odomのリマップ
            ('/model/robot_2dw1c/odom', '/robot_2dw1c/odom'),
            ('/model/robot_3dw/odom', '/robot_3dw/odom'),
            ('/model/robot_4dw/odom', '/robot_4dw/odom'),
            # Tfのリマップ
            ('/model/robot_2dw1c/tf', '/tf'),
            ('/model/robot_3dw/tf', '/tf'),
            ('/model/robot_4dw/tf', '/tf'),
            # Lidar (LaserScan) のリマップ
            ('/model/robot_2dw1c/laser', '/robot_2dw1c/scan'),
            ('/model/robot_3dw/laser', '/robot_3dw/scan'),
            ('/model/robot_4dw/laser', '/robot_4dw/scan'),
            # Lidar (PointCloud2) のリマップ
            ('/model/robot_2dw1c/laser/points', '/robot_2dw1c/points'),
            ('/model/robot_3dw/laser/points', '/robot_3dw/points'),
            ('/model/robot_4dw/laser/points', '/robot_4dw/points'),

        ],
        output='screen'
    )

    # world 座標から各ロボットの odom への固定位置関係を定義
    static_tf_2dw1c = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 引数: x y z yaw pitch roll parent_frame child_frame
        arguments = ['-4.5', '0.5', '0', '1.57', '0', '0', 'world', 'robot_2dw1c/odom']
    )
    static_tf_3dw = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'robot_3dw/odom']
    )
    static_tf_4dw = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'robot_4dw/odom']
    )

    # Gazeboのプロセスが開始されたらブリッジを起動する設定
    delayed_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_sim,
            on_start=[bridge_node]
        )
    )

    return LaunchDescription([
        gz_sim,
        rviz2,
        teleop_robot_2dw1c,
        teleop_robot_3dw,
        teleop_robot_4dw,
        static_tf_2dw1c,
        static_tf_3dw,
        static_tf_4dw,
        delayed_bridge
    ])
