"""CarA startup entry."""

from machine import Pin

from test.WirelessCarsHeartbeatTest import build_config, run_heartbeat_test


# CarA 是双车中的 1 号节点，因此上电后立即开始发送心跳。
CONFIG = build_config(
    self_id=1,
    peer_id=2,
    start_delay_ms=0,
    stop_switch_pin="D9",
    stop_switch_pull=Pin.PULL_UP_47K,
)


def main():
    # 启动入口尽量保持精简，实际逻辑放到可复用的测试模块里。
    run_heartbeat_test(CONFIG)


main()
