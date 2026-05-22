import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/betheln/ros2_ws/src/Robotics/delivery_robot2/install/delivery_robot'
