import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kaveh/diffbot_ros2_workspace_humble/install/diffbot_pi'
