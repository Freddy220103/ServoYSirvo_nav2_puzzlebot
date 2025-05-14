from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'ServoYSirvo_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ]+ [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('urdf') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('meshes') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('models') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('worlds') for file in files
    ]
    +
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('plugins') for file in files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario Martinez',
    maintainer_email='mario.mtz@manchester-robotics.com',
    description='Puzzlebot Gazebo Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_stabilisation_control = ServoYSirvo_nav2_puzzlebot.point_stabilisation_control:main',
            'localisation = ServoYSirvo_nav2_puzzlebot.localisation:main',
            'joint_state_pub = ServoYSirvo_nav2_puzzlebot.joint_state_pub:main',
            'dynamic_tf = ServoYSirvo_nav2_puzzlebot.dynamic_tf:main',
            'static_tf = ServoYSirvo_nav2_puzzlebot.static_tf:main',
            'uncertainty_ellipse = ServoYSirvo_nav2_puzzlebot.uncertainty_ellipse:main',
            'puzzlebot_kinematic_model= ServoYSirvo_nav2_puzzlebot.puzzlebot_kinematic_model:main',
            
            
        ],
    },
)
