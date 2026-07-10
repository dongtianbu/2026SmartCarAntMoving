"""Runnable leader demo: broadcast CarA yaw for the follower."""

from machine import Pin
import gc
import time

from control.LeaderYawBroadcaster import LeaderYawBroadcaster


DEFAULT_CONFIG = {
    "self_id": 1,
    "peer_id": 2,
    "uart_id": 2,
    "baudrate": 115200,
    "send_interval_ms": 20,
    "imu_capture_div": 1,
    "imu_tick_ms": 10,
    "imu_acc_range_g": 8,
    "imu_gyro_range_dps": 2000,
    "imu_acc_alpha": 0.20,
    "imu_comp_alpha": 0.98,
    "imu_gyro_cali_n": 300,
    "status_print_every": 20,
    "loop_delay_ms": 1,
    "led_pin": "C4",
    "led_toggle_every_sends": 25,
    "stop_switch_pin": "D9",
    "stop_switch_pull": Pin.PULL_UP_47K,
}


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


def run_leader_yaw_broadcast_demo(config=None):
    if config is None:
        config = build_config()

    led = Pin(config["led_pin"], Pin.OUT, value=True)
    stop_switch = Pin(
        config["stop_switch_pin"],
        Pin.IN,
        pull=config["stop_switch_pull"],
    )
    stop_state = stop_switch.value()

    broadcaster = LeaderYawBroadcaster(config)

    print("=== CarA Leader Yaw Broadcast Demo ===")
    print("CarA will keep sending leader yaw to CarB.")
    print("Stop switch: {}".format(config["stop_switch_pin"]))
    print("Please keep the car still during IMU calibration.\n")

    broadcaster.start()
    print("IMU ready. Yaw broadcast started.\n")

    tick = 0
    send_count = 0
    try:
        while True:
            data = broadcaster.update()
            if data is not None:
                tick += 1
                if data["sent"]:
                    send_count += 1
                    if send_count % config["led_toggle_every_sends"] == 0:
                        led.toggle()
                if tick % config["status_print_every"] == 0:
                    print(
                        "yaw={:>7.2f} gyro_z={:>7.2f} sent={} send_count={}".format(
                            data["yaw"],
                            data["gyro_z"],
                            data["sent"],
                            send_count,
                        )
                    )

            if stop_switch.value() != stop_state:
                print("Stop requested.")
                break

            time.sleep_ms(config["loop_delay_ms"])
            gc.collect()
    finally:
        broadcaster.stop()
        led.value(1)
        print("Leader yaw broadcast stopped.")


if __name__ == "__main__":
    run_leader_yaw_broadcast_demo()
