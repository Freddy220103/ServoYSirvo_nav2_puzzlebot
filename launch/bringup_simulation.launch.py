from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Ruta del paquete
    pkg = get_package_share_directory('ServoYSirvo_nav2_puzzlebot')

    # Ruta del mundo
    world_path = os.path.join(pkg, 'worlds', 'world_prueba.world')

    # 1. Lanzar Ignition Gazebo con el mundo
    launch_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path, '-v', '4'],
        output='screen'
    )

    # 2. Lanzar robot_state_publisher desde su propio launch
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ServoYSirvo_nav2_puzzlebot'),
                'launch',
                'robot_state_publisher_launch.py'
            ])
        )
    )

    # 3. Lanzar el bridge ROS ↔ Gazebo con el nombre correcto del mundo: default
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/puzzlebot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose'
        ],
        output='screen'
    )

    # 4. Spawnear el robot con un pequeño delay para dar tiempo al mundo de inicializar
    spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'puzzlebot',
                    '-x', '0', '-y', '0', '-z', '0.1',
                    '-topic', 'robot_description'
                ],
                output='screen'
            )
        ]
    )

    # Lanzar todo junto
    return LaunchDescription([
        launch_gazebo,
        robot_state_publisher,
        bridge,
        spawn
    ])
