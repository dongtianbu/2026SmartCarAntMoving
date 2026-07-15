"""CarB 上电启动入口。"""

import sys


for module_dir in ("control", "connection", "driver"):
    if module_dir not in sys.path:
        sys.path.append(module_dir)

from FollowLeaderYawColorTrace import build_config, run_follow_leader_yaw_color_trace


CONFIG = build_config()


def main():
    run_follow_leader_yaw_color_trace(CONFIG)


main()




# import sys
# 
# for module_dir in ("control", "connection", "driver"):
#     if module_dir not in sys.path:
#         sys.path.append(module_dir)
# 
# from UWBUsartLocation import build_config as build_uwb_location_config
# from UWBUsartLocation import run_uwb_location_bridge
# 
# 
# UWB_LOCATION_CONFIG = build_uwb_location_config()
# 
# 
# def main():
#     run_uwb_location_bridge(UWB_LOCATION_CONFIG)
# 
# 
# main()