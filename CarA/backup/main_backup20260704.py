"""历史备份主程序：视觉追踪版。

这个文件不是当前默认入口，但保留了较完整的“视觉追踪主循环”示例，
适合拿来查初始化顺序和调参位置。
"""

from control import MotorControl
# import IMUVertical
# import PID
# import PIDYawUsart
from connection.MCXVisionUsart import MCXVisionUsart
from connection.WirelessUsart import WirelessUsart
from control.ColorTrace import ColorTraceController

from machine import Pin
import time
import gc

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ===== 运行开关：最适合直接改的地方 =====
ENABLE_STARTUP_PRINT = True
ENABLE_TERMINAL_DEBUG = False
ENABLE_WIRELESS_REPORT = False
ENABLE_NO_TARGET_REPORT = False
ENABLE_WIRELESS_ERROR_REPORT = True
ENABLE_WIRELESS_NO_TARGET_REPORT = False

VISION_BAUDRATE = 115200
WIRELESS_BAUDRATE = 460800

TRACK_BASE_DUTY = 6300
TRACK_MAX_DUTY = 8000
TRACK_MIN_ACTIVE_DUTY = 6300
TRACK_TARGET_HOLD_MS = 120
TRACK_KP_X = 0.2
TRACK_KD_X = 0.75
TRACK_KP_Y = 0.2
TRACK_KD_Y = 0.75
TRACK_DEAD_ZONE = 2
TRACK_CENTER_EXIT_ZONE = 4
TRACK_INPUT_FILTER_ALPHA = 0.45
TRACK_COMMAND_FILTER_ALPHA = 0.20
TRACK_NEAR_CENTER_MIN_DUTY = 2200
TRACK_NEAR_CENTER_MAX_DUTY = 5200
TRACK_SLOW_ZONE = 15
TRACK_DUTY_RAMP_STEP = 180

DEBUG_PRINT_INTERVAL_MS = 120
WIRELESS_REPORT_INTERVAL_MS = 50
WIRELESS_ERROR_REPORT_INTERVAL_MS = 50
NO_TARGET_REPORT_INTERVAL_MS = 300
MAIN_LOOP_DELAY_MS = 1
GC_COLLECT_INTERVAL_MS = 1000


def calc_box_error(box, center_x, center_y):
    """根据目标框算出相对画面中心的原始误差。"""
    box_center_x = (box[0] + box[2]) / 2.0
    box_center_y = (box[1] + box[3]) / 2.0
    return center_x - box_center_x, center_y - box_center_y


if __name__ == "__main__":
    # 这是一个完整业务主程序示例：
    # 1. 初始化视觉串口和无线串口
    # 2. 创建颜色追踪控制器
    # 3. 在主循环里不断 update
    # imu = IMUVertical.ImuSensorVertical()
    # imu.init()
    # imu.calibrate()
    # print("IMU ready.\n")

    # controller = PID.StraightLineController(
    #     imu, base_duty=BASE_DUTY,
    #     kp=PID_KP, ki=PID_KI, kd=PID_KD,
    #     dead_zone=0.3
    # )

    mcx = MCXVisionUsart(baudrate=VISION_BAUDRATE)
    wireless = WirelessUsart(baudrate=WIRELESS_BAUDRATE)
    if ENABLE_STARTUP_PRINT:
        wireless.info()

    color_trace = ColorTraceController(mcx,
                                       base_duty=TRACK_BASE_DUTY,
                                       max_tracking_duty=TRACK_MAX_DUTY,
                                       min_tracking_duty=TRACK_MIN_ACTIVE_DUTY,
                                       target_hold_ms=TRACK_TARGET_HOLD_MS,
                                       kp_x=TRACK_KP_X,
                                       kd_x=TRACK_KD_X,
                                       kp_y=TRACK_KP_Y,
                                       kd_y=TRACK_KD_Y,
                                       dead_zone=TRACK_DEAD_ZONE,
                                       center_exit_zone=TRACK_CENTER_EXIT_ZONE,
                                       input_filter_alpha=TRACK_INPUT_FILTER_ALPHA,
                                       command_filter_alpha=TRACK_COMMAND_FILTER_ALPHA,
                                       near_center_min_duty=TRACK_NEAR_CENTER_MIN_DUTY,
                                       near_center_max_duty=TRACK_NEAR_CENTER_MAX_DUTY,
                                       slow_zone=TRACK_SLOW_ZONE,
                                       duty_ramp_step=TRACK_DUTY_RAMP_STEP)

    if ENABLE_STARTUP_PRINT:
        print("=== Color Trace ===")
        print(f"Screen: {color_trace.SCREEN_W}x{color_trace.SCREEN_H}")
        print(f"Target center: ({color_trace.CENTER_X}, {color_trace.CENTER_Y})")
        print(f"Tracking duty: min={color_trace.min_tracking_duty} max={color_trace.max_tracking_duty}")
        print(f"Near-center duty: min={color_trace.near_center_min_duty} max={color_trace.near_center_max_duty}")
        print(f"Target hold: {color_trace.target_hold_ms} ms")
        print(f"Center zone: enter={color_trace.dead_zone} exit={color_trace.center_exit_zone}")
        print(f"Input filter alpha: {color_trace.input_filter_alpha}")
        print(f"Command filter alpha: {color_trace.command_filter_alpha}")
        print(f"Duty ramp step: {color_trace.duty_ramp_step}")
        print(f"PID: kp_x={TRACK_KP_X} kd_x={TRACK_KD_X} kp_y={TRACK_KP_Y} kd_y={TRACK_KD_Y}")
        if ENABLE_WIRELESS_ERROR_REPORT:
            print("Wireless TX format: VISION_ERR:+12.3,-4.5")
        print("Press switch2 (D9) to stop.\n")

    time.sleep(1)
    color_trace.start()
    if ENABLE_STARTUP_PRINT:
        print("GO! Tracking sandbag...\n")

    loop_count = 0
    last_debug_ms = time.ticks_ms()
    last_wireless_ms = last_debug_ms
    last_wireless_error_ms = last_debug_ms
    last_no_target_ms = last_debug_ms
    last_wireless_no_target_ms = last_debug_ms
    last_gc_ms = last_debug_ms

    while True:
        loop_count += 1
        result = color_trace.update(MotorControl.motor_1, MotorControl.motor_2, MotorControl.motor_3)
        now_ms = time.ticks_ms()

        if result is not None:
            led.toggle()

            if ENABLE_WIRELESS_ERROR_REPORT and result["has_target"] and time.ticks_diff(now_ms, last_wireless_error_ms) >= WIRELESS_ERROR_REPORT_INTERVAL_MS:
                raw_err_x, raw_err_y = calc_box_error(result["box"], color_trace.CENTER_X, color_trace.CENTER_Y)
                wireless.send_tracking_error(raw_err_x, raw_err_y)
                last_wireless_error_ms = now_ms
            elif ENABLE_WIRELESS_NO_TARGET_REPORT and (not result["target_locked"]) and time.ticks_diff(now_ms, last_wireless_no_target_ms) >= NO_TARGET_REPORT_INTERVAL_MS:
                wireless.send_tracking_no_target()
                last_wireless_no_target_ms = now_ms

            if ENABLE_WIRELESS_REPORT and result["has_target"] and time.ticks_diff(now_ms, last_wireless_ms) >= WIRELESS_REPORT_INTERVAL_MS:
                center_x = result["cx"]
                center_y = result["cy"]
                width = result["box"][2] - result["box"][0] + 1
                height = result["box"][3] - result["box"][1] + 1
                send_str = "COLORTRACE:{:.1f},{:.1f},{:.0f},{:.0f}\n".format(
                    center_x, center_y, width, height
                )
                wireless.send_line(send_str[:-1])
                last_wireless_ms = now_ms
            elif ENABLE_NO_TARGET_REPORT and (not result["target_locked"]) and time.ticks_diff(now_ms, last_no_target_ms) >= NO_TARGET_REPORT_INTERVAL_MS:
                wireless.send_line("COLORTRACE:NO_TARGET")
                last_no_target_ms = now_ms

            if ENABLE_TERMINAL_DEBUG and time.ticks_diff(now_ms, last_debug_ms) >= DEBUG_PRINT_INTERVAL_MS:
                target_str = "YES" if result["has_target"] else ("HOLD" if result["target_locked"] else "NO")
                if result["in_center"]:
                    action = "CENTER"
                elif abs(result["err_x"]) > abs(result["err_y"]):
                    if result["err_x"] > 0:
                        action = "LEFT"
                    else:
                        action = "RIGHT"
                else:
                    if result["err_y"] > 0:
                        action = "UP"
                    else:
                        action = "DOWN"
                msg = (f"{target_str} err=({result['err_x']:>+5.0f},{result['err_y']:>+5.0f}) " +
                       f"filt=({result['filtered_cx']:>5.1f},{result['filtered_cy']:>5.1f}) " +
                       f"corr=({result['correction_x']:>+6.0f},{result['correction_y']:>+6.0f}) " +
                       f"raw=({result['raw_duty'][0]:>+5.0f},{result['raw_duty'][1]:>+5.0f},{result['raw_duty'][2]:>+5.0f}) " +
                       f"target=({result['target_duty'][0]:>+5.0f},{result['target_duty'][1]:>+5.0f},{result['target_duty'][2]:>+5.0f}) " +
                       f"duty=({result['duty'][0]:>+5.0f},{result['duty'][1]:>+5.0f},{result['duty'][2]:>+5.0f}) [{action}]")
                print(msg)
                last_debug_ms = now_ms

        else:
            if ENABLE_TERMINAL_DEBUG and loop_count % 100 == 0:
                print("[DEBUG] update() returned None!")

        if switch2.value() != state2:
            color_trace.stop()
            MotorControl.motor_1.duty(0)
            MotorControl.motor_2.duty(0)
            MotorControl.motor_3.duty(0)
            if ENABLE_STARTUP_PRINT or ENABLE_TERMINAL_DEBUG:
                print("\nStopped.")
            break

        time.sleep_ms(MAIN_LOOP_DELAY_MS)
        if time.ticks_diff(now_ms, last_gc_ms) >= GC_COLLECT_INTERVAL_MS:
            gc.collect()
            last_gc_ms = now_ms
