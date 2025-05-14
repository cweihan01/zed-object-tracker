import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/camera_workspace/src/zed-object-tracker/install/zed-object-tracker'
