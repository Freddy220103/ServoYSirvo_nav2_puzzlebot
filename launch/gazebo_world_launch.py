import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('ServoYSirvo_nav2_puzzlebot')
    world_file = os.path.join(pkg_path, 'worlds', 'world_prueba.world')
    urdf_file = os.path.join(pkg_path, 'urdf', 'puzzlebot.urdf')

    # Lee el URDF como string
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Iniciar Ignition Gazebo con el mundo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # Publicar el robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Spawnear el robot desde el robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'puzzlebot',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_entity
    ])
