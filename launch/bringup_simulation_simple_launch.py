import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------

    world = 'mundo1.world'
    pause = 'true'
    verbosity = '4'
    use_sim_time = 'True'
    # -----------------------------------------------------------------------------
    #                          ROBOT CONFIGURATION
    # -----------------------------------------------------------------------------

    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]

    # -----------------------------------------------------------------------------
    #                         LOAD GAZEBO WORLD
    # -----------------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ServoYSirvo_nav2_puzzlebot'),
                'launch',
                'gazebo_world_launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
            'verbosity': verbosity,
            'use_sim_time': use_sim_time
        }.items()
    )

    # -----------------------------------------------------------------------------
    #                       SPAWN EACH ROBOT DYNAMICALLY
    # -----------------------------------------------------------------------------

    robot_launches = []
    for robot in robot_config_list:
        robot_name = robot['name']
        robot_type = robot['type']
        x = str(robot.get('x', 0.0))
        y = str(robot.get('y', 0.0))
        yaw = str(robot.get('yaw', 0.0))
        lidar_frame = robot.get('lidar_frame', 'laser_frame')
        camera_frame = robot.get('camera_frame', 'camera_link_optical')
        tof_frame = robot.get('tof_frame', 'tof_link')
        prefix = f'{robot_name}/' if robot_name != '' else ''

        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ServoYSirvo_nav2_puzzlebot'),
                    'launch',
                    'gazebo_puzzlebot_launch.py'
                )
            ),
            launch_arguments={
                'robot': robot_type,
                'robot_name': robot_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'prefix': prefix,
                'lidar_frame': lidar_frame,
                'camera_frame': camera_frame,
                'tof_frame': tof_frame,
                'use_sim_time': use_sim_time
            }.items()
        )

        puzzle_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ServoYSirvo_nav2_puzzlebot'),
                    'launch',
                    'puzzlebot_sim_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        )

        robot_launches.append(robot_launch)
        robot_launches.append(puzzle_launch)

    # -----------------------------------------------------------------------------
    #                          LOAD NAVIGATION CONFIGURATION
    # -----------------------------------------------------------------------------
    # Load the navigation configuration
    # Make sure to have the map file in the correct path
    # and the nav2_params.yaml file in the param folder
    # You can change the map name and path as needed

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    # Cambiar la ruta de mapa y param según sea necesario en tu sistema
    # Asegúrate de tener el archivo map.yaml en la ruta correcta y param.yaml en la carpeta param
    home = os.path.expanduser('~') #Home Linux
    map_dir = os.path.join(home,'ros2_ws/src/ServoYSirvo_nav2_puzzlebot/maps/map_actualizado.yaml')
    param_file = os.path.join(home,'ros2_ws/src/ServoYSirvo_nav2_puzzlebot/param/puzzlebot.yaml')  # asegúrate de tener ese archivo

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_file
        }.items()
    )


    # -----------------------------------------------------------------------------
    #                          BUILD FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('USE_SIM_TIME', 'True'),
        nav2_launch,
        # cometar si es en fisico
        gazebo_launch,
        *robot_launches,
    ])
