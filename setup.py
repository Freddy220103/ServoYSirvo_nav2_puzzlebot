from setuptools import setup

package_name = 'ServoYSirvo_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_launch.py']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/rviz', ['config/rviz/nav.rviz']),
        ('share/' + package_name + '/rviz', ['config/rviz/slam.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Integrante 1, Integrante 2, Integrante 3',
    maintainer_email='email1@equipo.com, email2@equipo.com, email3@equipo.com',
    description='Paquete de navegación autónoma para PuzzleBot con Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
