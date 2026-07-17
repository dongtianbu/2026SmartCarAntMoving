"""CarB 轻量组合控制的运行入口。"""

from machine import Pin
import gc
import time

from FollowLeaderYawColorTraceLiteConfig import build_config, _yaw_mode_name
from FollowLeaderYawColorTraceLiteController import FollowLeaderYawColorTraceLite
from FollowLeaderYawColorTraceLiteSupport import SmallStatusLed, _metric_value


def run_follow_leader_yaw_color_trace(config=None):
    """启动 CarB 轻量组合控制流程。"""
    if config is None:
        config = build_config()
    led = SmallStatusLed(config["led_pin"], config["led_active_level"])
    stop_switch = None
    stop_state = None
    if config["stop_switch_enabled"]:
        stop_switch = Pin(config["stop_switch_pin"], Pin.IN, pull=config["stop_switch_pull"])
        stop_state = stop_switch.value()

    gc.collect()
    controller = FollowLeaderYawColorTraceLite(config)
    gc.collect()
    try:
        controller.start()
        if config["motor_self_test_on_start"]:
            controller.motor.rotate(config["motor_self_test_speed"])
            time.sleep_ms(config["motor_self_test_ms"])
            controller.motor.stop(0)
    except Exception:
        controller.motor.stop(0)
        while True:
            led.error()
            time.sleep_ms(config["loop_delay_ms"])

    if config["enable_serial_log"]:
        print("=== CarB lite color trace + yaw control ===")
        print("yaw mode={} ({})".format(config["yaw_control_mode"], _yaw_mode_name(config["yaw_control_mode"])))
        print("line angle control={}".format(config["line_angle_control_enabled"]))

    tick = 0
    last_gc_ms = time.ticks_ms()
    try:
        while True:
            now_ms = time.ticks_ms()
            result = controller.update()
            led.healthy()
            if result is not None:
                tick += 1
                if tick % config["status_print_every"] == 0:
                    if config["enable_serial_log"]:
                        print("yaw_mode={} leader_yaw={} follower_yaw={} yaw_error={} yaw_rotate={} line_angle={} line_err={} line_rotate={} rotate={} target_locked={} err_x={} err_y={} vx={} vy={} duty={}".format(
                            result["yaw_mode"],
                            _metric_value(result["leader_yaw"]),
                            _metric_value(result["follower_yaw"]),
                            _metric_value(result["yaw_error"]),
                            _metric_value(result["yaw_rotate_speed"]),
                            _metric_value(result["line_angle_deg"]),
                            _metric_value(result["line_angle_error"]),
                            _metric_value(result["line_angle_rotate_speed"]),
                            _metric_value(result["rotate_speed"]),
                            result["target_locked"],
                            result["err_x"],
                            result["err_y"],
                            _metric_value(result["vx"]),
                            _metric_value(result["vy"]),
                            result["motor_duty"],
                        ))
                    if config["ai_metric_enabled"] and not result["ai_control_paused"]:
                        line = "{} tick={} yaw_mode={} leader_yaw={} follower_yaw={} yaw_error={} yaw_rotate={} line_angle={} line_err={} line_rotate={} rotate={} target_locked={} in_center={} err_x={} err_y={} vx={} vy={} paused={} deviation_active={} manual_active={}".format(
                            config["ai_metric_prefix"],
                            tick,
                            result["yaw_mode"],
                            _metric_value(result["leader_yaw"]),
                            _metric_value(result["follower_yaw"]),
                            _metric_value(result["yaw_error"]),
                            _metric_value(result["yaw_rotate_speed"]),
                            _metric_value(result["line_angle_deg"]),
                            _metric_value(result["line_angle_error"]),
                            _metric_value(result["line_angle_rotate_speed"]),
                            _metric_value(result["rotate_speed"]),
                            result["target_locked"],
                            result["in_center"],
                            result["err_x"],
                            result["err_y"],
                            _metric_value(result["vx"]),
                            _metric_value(result["vy"]),
                            result["ai_control_paused"],
                            result["ai_deviation_active"],
                            result["ai_manual_active"],
                        )
                        print(line)
                        if config["ai_metric_wireless_enabled"] and controller.tuner is not None:
                            controller.tuner.reply(line, config["tuning_metric_reply_repeat_count"])
            if stop_switch is not None and stop_switch.value() != stop_state:
                break
            if time.ticks_diff(now_ms, last_gc_ms) >= config["gc_interval_ms"]:
                gc.collect()
                last_gc_ms = now_ms
            time.sleep_ms(config["loop_delay_ms"])
    except Exception:
        controller.motor.stop(0)
        while True:
            led.error()
            time.sleep_ms(config["loop_delay_ms"])
    finally:
        controller.stop()
        led.off()
