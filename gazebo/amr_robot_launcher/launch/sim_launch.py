import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

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

    # 2. Teleopを別々のターミナル(terminator)で起動
    # terminator -e の形式を再現します
    def create_teleop_process(robot_name):
        return ExecuteProcess(
            cmd=['terminator', '--new-tab', '-e', f'bash -c "source {home}/.bashrc; source {home}/colcon_ws/install/setup.bash; {teleop_script} {robot_name}; exec bash"'],
            output='screen'
        )

    teleop_robot_2dw1c = create_teleop_process('robot_2dw1c')
    teleop_robot_3dw = create_teleop_process('robot_3dw')
    teleop_robot_4dw = create_teleop_process('robot_4dw')

    return LaunchDescription([
        gz_sim,
        # Gazeboが起動してから少し遅らせてTeleopを起動したい場合は
        # RegisterEventHandlerを使いますが、今回はシンプルに並列起動します
        teleop_robot_2dw1c,
        teleop_robot_3dw,
        teleop_robot_4dw
    ])
