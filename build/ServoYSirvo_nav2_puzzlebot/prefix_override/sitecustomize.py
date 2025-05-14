import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victorn65/ros2_ws/src/ServoYSirvo_nav2_puzzlebot/install/ServoYSirvo_nav2_puzzlebot'
