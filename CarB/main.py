from test.WirelessCarsHeartbeatTest import build_config, run_heartbeat_test


CONFIG = build_config(
    self_id=2,
    peer_id=1,
    start_delay_ms=350,
)


run_heartbeat_test(CONFIG)
