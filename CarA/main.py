"""CarA 上电启动入口。"""

from control.LeaderYawColorTrace import build_config, run_leader_yaw_color_trace


CONFIG = build_config()


def main():
    run_leader_yaw_color_trace(CONFIG)


main()
