import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eugene/auvc_ws/src/teamcream/install/teamcream'
