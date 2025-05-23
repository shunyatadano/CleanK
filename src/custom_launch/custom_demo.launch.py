import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveIt設定を取得
    moveit_config = MoveItConfigsBuilder("openarm_bimanual", package_name="openarm_bimanual_moveit_config").to_moveit_configs()
    
    # 引数宣言
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_fake_controller = LaunchConfiguration('use_fake_controller', default='true')
    
    # RViz設定
    robot_description = moveit_config.robot_description
    robot_description_semantic = moveit_config.robot_description_semantic
    
    # 必要なファイルのパス
    rviz_config_file = PathJoinSubstitution([FindPackageShare("openarm_bimanual_moveit_config"), "config", "moveit.rviz"])
    controllers_file = PathJoinSubstitution([FindPackageShare("openarm_bimanual_moveit_config"), "config", "moveit_controllers.yaml"])
    
    # 起動説明
    return LaunchDescription([
        # 引数
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_fake_controller', default_value='true', description='Use fake controller for testing'),
        
        # ロボットステートパブリッシャー
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time}
            ],
        ),
        
        # フェイクジョイントドライバ（フェイクコントローラモード用）
        Node(
            condition=IfCondition(use_fake_controller),
            package="fake_joint_driver",
            executable="fake_joint_driver_node",
            name="fake_joint_driver_node",
            parameters=[
                robot_description,
                os.path.join(
                    moveit_config.package_path, "config", "ros2_controllers.yaml"
                ),
                {"use_sim_time": use_sim_time}
            ],
        ),
        
        # MoveItのコントローラマネージャー
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                os.path.join(
                    moveit_config.package_path, "config", "ros2_controllers.yaml"
                ),
                {"use_sim_time": use_sim_time}
            ],
        ),
        
        # ジョイントステートブロードキャスタの起動
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        
        # 各コントローラのスポナー
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_gripper_controller", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
        ),
        
        # MoveGroupノード
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                moveit_config.planning_scene_monitor,
                controllers_file,
                {"use_sim_time": use_sim_time}
            ],
        ),
        
        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time}
            ],
        ),
    ])