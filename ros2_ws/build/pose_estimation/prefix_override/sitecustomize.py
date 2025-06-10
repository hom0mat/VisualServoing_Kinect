import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mateo/VisualServoing_Kinect/ros2_ws/install/pose_estimation'
