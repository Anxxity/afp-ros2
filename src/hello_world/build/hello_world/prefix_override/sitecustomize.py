import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/remin/Desktop/exp-1-ros/src/hello_world/install/hello_world'
