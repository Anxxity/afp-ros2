import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/remin/Desktop/ros2_assignment-afp/install/hello_world'
