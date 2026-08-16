import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, GroupAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import subprocess

def generate_launch_description():
    home = os.path.expanduser('~')
    
    # パスの定義
    world_path = os.path.join(home, 'git/gazebo-rcll/worlds/btr_2025_simple.world')
    models_dir = os.path.join(home, 'git/gazebo-rcll/models')
    teleop_script = os.path.join(home, 'git/amr/gazebo/docker/scripts/teleop_keyboard_robot.sh')
    rviz_config_path = os.path.join(home, 'git/amr/gazebo/amr_robot_launcher/rviz/amr.rviz')
    gz_resource_path = ':'.join([
        models_dir,
        os.path.join(models_dir, 'carologistics'),
        os.path.join(models_dir, 'bbu'),
        os.path.join(models_dir, 'pyro'),
    ])
    # nav2_bringup パッケージのパスを取得
    nav2_navigation_dir = get_package_share_directory('nav2_bringup')
    nav2_navigation_launch_path = os.path.join(nav2_navigation_dir, 'launch', 'navigation_launch.py')
    # パラメータファイルやマップファイルのパス（環境に合わせて調整してください）
    map_yaml_file = os.path.join(home, 'git/amr/gazebo/amr_robot_launcher/map/map.yaml')
    params_file_path = os.path.join(home, 'git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml')

    # 起動時に「gz」を全消去
    subprocess.run("pkill -9 -f gz", shell=True)

    # 1. Gazebo Simの起動
    # GZ_SIM_RESOURCE_PATHを設定することで、cdしなくてもモデルを読み込めます
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        # additional_env={'GZ_SIM_RESOURCE_PATH': models_dir},
        additional_env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path},
        output='screen'
    )

    # 2. Rviz2の起動
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
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
            # clock のブリッジ
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # /cmd_vel
            '/robot_2dw1c/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot_3dw/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot_4dw/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
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
            # Depth Image（gz.msgs.Image → sensor_msgs/Image）
            '/model/robot_2dw1c/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot_3dw/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot_4dw/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            # CameraInfo（gz.msgs.CameraInfo → sensor_msgs/CameraInfo）
            '/model/robot_2dw1c/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/robot_3dw/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/robot_4dw/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Depth PointCloud（gz.msgs.PointCloudPacked → sensor_msgs/PointCloud2）
            '/model/robot_2dw1c/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/robot_3dw/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/robot_4dw/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # RGB Camera
            '/model/robot_2dw1c/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot_2dw1c/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/robot_3dw/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot_3dw/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/robot_4dw/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot_4dw/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
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
            # Depth Image
            ('/model/robot_2dw1c/depth_camera', '/robot_2dw1c/depth/image'),
            ('/model/robot_3dw/depth_camera', '/robot_3dw/depth/image'),
            ('/model/robot_4dw/depth_camera', '/robot_4dw/depth/image'),
            # CameraInfo
            ('/model/robot_2dw1c/depth_camera/camera_info', '/robot_2dw1c/depth/camera_info'),
            ('/model/robot_3dw/depth_camera/camera_info', '/robot_3dw/depth/camera_info'),
            ('/model/robot_4dw/depth_camera/camera_info', '/robot_4dw/depth/camera_info'),
            # Depth PointCloud
            ('/model/robot_2dw1c/depth_camera/points', '/robot_2dw1c/depth/points'),
            ('/model/robot_3dw/depth_camera/points', '/robot_3dw/depth/points'),
            ('/model/robot_4dw/depth_camera/points', '/robot_4dw/depth/points'),
            # RGB Camera
            ('/model/robot_2dw1c/rgb_camera', '/robot_2dw1c/rgb/image'),
            ('/model/robot_2dw1c/rgb_camera/camera_info', '/robot_2dw1c/rgb/camera_info'),
            ('/model/robot_3dw/rgb_camera', '/robot_3dw/rgb/image'),
            ('/model/robot_3dw/rgb_camera/camera_info', '/robot_3dw/rgb/camera_info'),
            ('/model/robot_4dw/rgb_camera', '/robot_4dw/rgb/image'),
            ('/model/robot_4dw/rgb_camera/camera_info', '/robot_4dw/rgb/camera_info'),
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        output='screen'
    )

    # world 座標から各ロボットの odom への固定位置関係を定義
    static_tf_2dw1c = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        # 引数: x y z yaw pitch roll parent_frame child_frame
        arguments = ['-6.5', '0.5', '0', '0', '0', '0', 'world', 'robot_2dw1c/odom']
    )
    static_tf_3dw = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'robot_3dw/odom']
    )
    static_tf_4dw = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'robot_4dw/odom']
    )

    # URDFのtf用
    static_tf_2dw1c_depth_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '1.0',  '0', '0', '0', '1',  'robot_2dw1c/base_link', 'robot_2dw1c/depth_camera_frame']
    )
    static_tf_3dw_depth_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '1.0',  '0', '0', '0', '1',  'robot_3dw/body', 'robot_3dw/depth_camera_frame']
    )
    static_tf_4dw_depth_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '1.0',  '0', '0', '0', '1',  'robot_4dw/base_footprint', 'robot_4dw/depth_camera_frame']
    )

    # --- slam_toolbox のノード定義 ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='robot_4dw',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(home, 'git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_slam.yaml'),
            {
                'use_sim_time': True,
                'autostart': True,
                'odom_frame': 'robot_4dw/odom',
                'base_frame': 'robot_4dw/base_footprint',
                'map_frame': 'robot_4dw/map',
                'scan_topic': '/robot_4dw/scan',
            }
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('~/transition_event',           '/slam_toolbox/transition_event'),
            ('~/change_state',               '/slam_toolbox/change_state'),
            ('~/describe_parameters',        '/slam_toolbox/describe_parameters'),
            ('~/get_available_states',       '/slam_toolbox/get_available_states'),
            ('~/get_available_transitions',  '/slam_toolbox/get_available_transitions'),
            ('~/get_parameter_types',        '/slam_toolbox/get_parameter_types'),
            ('~/get_parameters',             '/slam_toolbox/get_parameters'),
            ('~/get_state',                  '/slam_toolbox/get_state'),
            ('~/get_transition_graph',       '/slam_toolbox/get_transition_graph'),
            ('~/get_type_description',       '/slam_toolbox/get_type_description'),
            ('~/list_parameters',            '/slam_toolbox/list_parameters'),
            ('~/save_map',                   '/slam_toolbox/save_map'),
            ('~/serialize_map',             '/slam_toolbox/serialize_map'),
            ('~/deserialize_map',           '/slam_toolbox/deserialize_map'),
            ('~/set_parameters',             '/slam_toolbox/set_parameters'),
            ('~/set_parameters_atomically',  '/slam_toolbox/set_parameters_atomically'),
        ]
    )
    map_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='map_relay',
        parameters=[{'use_sim_time': True}],
        arguments=['/map', '/robot_4dw/map'],
        output='screen'
    )

    static_tf_4dw_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.0', '0.0', '0.25',  # X, Y, Z オフセット（実際のセンサー位置に調整してください）
            '0.0', '0.0', '0.0',  # Yaw, Pitch, Roll
            'robot_4dw/base_footprint',
            'robot_4dw/laser_frame'
        ],
        parameters=[
            {'use_sim_time': True},
        ],
        # remappings=[
        #     ('/tf', '/tf'),
        #     ('/tf_static', '/tf_static'),
        #  ]
    )

    # 既存の static_tf_4dw は SLAM と競合するため、SLAM 起動時は除外するか、
    # 接続先を robot_4dw/map に繋ぎ変える必要があります。
    # ここでは、world -> robot_4dw/map を繋ぐ static_tf に書き換えます。
    static_tf_4dw_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'robot_4dw/map']
    )

    # --- 2. ノード起動完了後に configure & activate を順番に発行 ---
    configure_and_activate = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam_toolbox_node,
            on_start=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                         'sleep 3 && '
                         # 1. Configure (ID: 1) をサービス経由で直接呼ぶ
                         'ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" && '
                         'sleep 1 && '
                         # 2. Activate (ID: 3) をサービス経由で直接呼ぶ
                         'ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"'
                         ],
                    output='screen'
                )
            ]
        )
    )

    # Gazeboのプロセスが開始されたらブリッジを起動する設定
    delayed_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_sim,
            on_start=[bridge_node]
        )
    )

    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        parameters=[{'use_sim_time': True}],
        arguments=['/cmd_vel', '/robot_4dw/cmd_vel'],
        output='screen'
    )

    # ros2 run nav2_map_server map_server   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml -p yaml_filename:=/home/ubuntu/map.yaml
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot_4dw',  # -r __ns:=/robot_4dw に対応
        output='screen',
        parameters=[
            params_file_path,   # --params-file に対応
            {'yaml_filename': map_yaml_file}  # -p yaml_filename:=... に対応
        ]
    )

    # ros2 run nav2_amcl amcl   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot_4dw',
        output='screen',
        parameters=[
            params_file_path
        ]
    )

    # ros2 run nav2_planner planner_server   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='robot_4dw',
        output='screen',
        parameters=[
            params_file_path
        ]
    )

    # ros2 run nav2_bt_navigator bt_navigator   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='robot_4dw',
        output='screen',
        parameters=[
            params_file_path
        ]
    )

    # ros2 run nav2_controller controller_server   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='robot_4dw',
        output='screen',
        parameters=[
            params_file_path
        ]
    )

    # ros2 run nav2_behaviors behavior_server   --ros-args   -r __ns:=/robot_4dw   --params-file /home/ubuntu/git/amr/gazebo/amr_robot_launcher/launch/yaml/robot_4dw_nav2.yaml
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='robot_4dw',
        output='screen',
        parameters=[
            params_file_path
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='robot_4dw',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 4.0,  # 応答待ちのタイムアウト（秒）を追加
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        gz_sim,
        rviz2,
        teleop_robot_2dw1c,
        teleop_robot_3dw,
        teleop_robot_4dw,
        static_tf_2dw1c,
        static_tf_3dw,
        # static_tf_4dw,
        static_tf_2dw1c_depth_cam,
        static_tf_3dw_depth_cam,
        static_tf_4dw_depth_cam,
        static_tf_4dw_to_map,
        delayed_bridge,
        slam_toolbox_node,
        static_tf_4dw_base_to_laser,
        map_relay_node,
        # nav2_action,
        cmd_vel_relay_node,
        map_server_node,
        amcl_node,
        planner_server_node,
        bt_navigator_node,
        controller_server_node,
        behavior_server_node,
        configure_and_activate,
        lifecycle_manager_node,
    ])
