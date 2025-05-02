import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('ServoYSirvo_nav2_puzzlebot')
    world_file = os.path.join(pkg_path, 'worlds', 'world_prueba.world')
    rviz_dir = os.path.join(pkg_path, 'rviz')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'puzzlebot'],
        output='screen'
    )

    rviz_mapping = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', os.path.join(rviz_dir, 'mapping.rviz')],
        output='screen'
    )

    rviz_navigation = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        arguments=['-d', os.path.join(rviz_dir, 'navigation.rviz')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        rviz_mapping,
        rviz_navigation
    ])
