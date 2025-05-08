from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('ServoYSirvo_nav2_puzzlebot')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    world_path = os.path.join(pkg_dir, 'worlds', 'world_prueba.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path, '--gui-config', '', '-v', '4'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-name', 'puzzlebot',
                 '-file', urdf_path],
            output='screen'
        )
    ])
