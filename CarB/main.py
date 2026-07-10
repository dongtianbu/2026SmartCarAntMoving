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
