"""CarB startup entry."""

from machine import Pin

from test.WirelessCarsHeartbeatTest import build_config, run_heartbeat_test


# CarB 是双车中的 2 号节点，稍微延迟启动，避免两车完全同拍发送。
CONFIG = build_config(
    self_id=2,
    peer_id=1,
    start_delay_ms=350,
    stop_switch_pin="D9",
    stop_switch_pull=Pin.PULL_UP_47K,
)


def main():
    # 启动入口尽量保持精简，实际逻辑放到可复用的测试模块里。
    run_heartbeat_test(CONFIG)


main()
