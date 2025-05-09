# Importaciones necesarias
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Obtener la ruta del paquete donde está el mundo y el robot
    pkg_puzzlebot = get_package_share_directory('ServoYSirvo_nav2_puzzlebot')

    # Ruta absoluta al archivo del mundo .world (Ignition Gazebo)
    world_path = os.path.join(pkg_puzzlebot, 'worlds', 'world_prueba.world')

    # Ruta al launch de robot_state_publisher que publicará el robot_description
    robot_state_pub_launch = PathJoinSubstitution([
        FindPackageShare('ServoYSirvo_nav2_puzzlebot'),
        'launch',
        'robot_state_publisher_launch.py'
    ])

    # Lanzar Ignition Gazebo con el mundo cargado
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    # Lanzar robot_state_publisher desde su propio launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_state_pub_launch)
    )

    # Comando para crear/spawnear el robot en Ignition Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',  # ros_ign en distros anteriores
        executable='create',
        arguments=[
            '-name', 'puzzlebot',
            '-x', '0', '-y', '0', '-z', '0.1',  # posición inicial
            '-topic', 'robot_description'       # viene del robot_state_publisher
        ],
        output='screen'
    )

    # Regresar los nodos como parte del LaunchDescription
    return LaunchDescription([
        gazebo,
        robot_description_launch,
        spawn_robot
    ])
