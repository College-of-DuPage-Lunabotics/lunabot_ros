# Systems running on the robot, published by launch_manager
# Names match LaunchSystem.srv: lio, mapping, nav2, hardware, record

# Every running system, whether started by launch_manager or not
string[] running

# Subset of running that was started outside launch_manager (e.g. over SSH)
string[] external

# Subset of running with at least one node that died unexpectedly; details go to /rosout
# and ~/.ros/launch_manager/<system>.log on the robot
string[] degraded
