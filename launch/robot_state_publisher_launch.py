from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'ServoYSirvo_nav2_puzzlebot'


    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(robot_name),
            'urdf',
            'puzzlebot.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )
    ])
