from test.WirelessCarsHeartbeatTest import build_config, run_heartbeat_test


CONFIG = build_config(
    self_id=1,
    peer_id=2,
    start_delay_ms=0,
)


run_heartbeat_test(CONFIG)
