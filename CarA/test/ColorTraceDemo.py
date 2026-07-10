"""颜色追踪联调演示脚本。

用途：
1. 联调视觉模块串口输出。
2. 验证 `ColorTraceController` 能否驱动三电机跟踪目标。
3. 通过无线串口把目标中心点和目标框大小发送给上位机。

使用方法：
1. 接好视觉模块，并确认视觉串口波特率为 `115200`。
2. 接好无线串口模块，并确认上位机监听波特率为 `460800`。
3. 保证底盘三电机、视觉模块和电源都已连接正常。
4. 直接运行本文件。
5. 将目标放入视觉识别范围，观察：
   - 小车是否开始追踪；
   - 串口是否输出 `COLORTRACE:...`；
   - 控制台是否打印误差、修正量和电机占空比。
6. 按下 `D9` 按键停止测试。

说明：
- 本文件来自外部平铺项目中的 `main.py` 功能，
  这里改成了 CarA 当前的分类目录结构，避免覆盖现有主入口。
"""

from machine import Pin
import gc
import time

from connection.MCXVisionUsart import MCXVisionUsart
from connection.WirelessUsart import WirelessUsart
from control import MotorControl
from control.ColorTrace import ColorTraceController


led = Pin("C4", Pin.OUT, value=True)
switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 串口参数。
VISION_BAUDRATE = 115200
WIRELESS_BAUDRATE = 460800

# 颜色追踪控制参数。
TRACK_BASE_DUTY = 6000
TRACK_MAX_DUTY = 7500
TRACK_MIN_ACTIVE_DUTY = 5700
TRACK_TARGET_HOLD_MS = 120
TRACK_KP_X = 0.2
TRACK_KD_X = 0.75
TRACK_KP_Y = 0.2
TRACK_KD_Y = 0.75
TRACK_DEAD_ZONE = 6

# 调试输出参数。
DEBUG_PRINT_EVERY = 10
NO_TARGET_REPORT_EVERY = 50
MAIN_LOOP_DELAY_MS = 1


def run():
    """运行颜色追踪联调演示。"""
    mcx = MCXVisionUsart(baudrate=VISION_BAUDRATE)
    wireless = WirelessUsart(baudrate=WIRELESS_BAUDRATE)
    wireless.info()

    color_trace = ColorTraceController(
        mcx,
        base_duty=TRACK_BASE_DUTY,
        max_tracking_duty=TRACK_MAX_DUTY,
        min_tracking_duty=TRACK_MIN_ACTIVE_DUTY,
        target_hold_ms=TRACK_TARGET_HOLD_MS,
        kp_x=TRACK_KP_X,
        kd_x=TRACK_KD_X,
        kp_y=TRACK_KP_Y,
        kd_y=TRACK_KD_Y,
        dead_zone=TRACK_DEAD_ZONE,
    )

    print("=== Color Trace Demo ===")
    print("Screen: {}x{}".format(color_trace.SCREEN_W, color_trace.SCREEN_H))
    print("Target center: ({}, {})".format(color_trace.CENTER_X, color_trace.CENTER_Y))
    print("Base duty: {}".format(color_trace.base_duty))
    print(
        "Tracking duty: min={} max={}".format(
            color_trace.min_tracking_duty,
            color_trace.max_tracking_duty,
        )
    )
    print("Target hold: {} ms".format(color_trace.target_hold_ms))
    print(
        "PID: kp_x={} kd_x={} kp_y={} kd_y={}".format(
            TRACK_KP_X,
            TRACK_KD_X,
            TRACK_KP_Y,
            TRACK_KD_Y,
        )
    )
    print("Press switch2 (D9) to stop.\n")

    time.sleep(1)
    color_trace.start()
    print("GO! Tracking sandbag...\n")

    tick = 0

    while True:
        # 把视觉检测结果交给控制器，控制器内部会决定是否驱动电机。
        result = color_trace.update(
            MotorControl.motor_1,
            MotorControl.motor_2,
            MotorControl.motor_3,
        )

        if result is not None:
            led.toggle()
            tick += 1

            if result["has_target"]:
                center_x = result["cx"]
                center_y = result["cy"]
                width = result["box"][2] - result["box"][0] + 1
                height = result["box"][3] - result["box"][1] + 1

                # 把目标中心点和目标框尺寸发给上位机，便于观察识别结果。
                wireless.send_line(
                    "COLORTRACE:{:.1f},{:.1f},{:.0f},{:.0f}".format(
                        center_x,
                        center_y,
                        width,
                        height,
                    )
                )
            elif tick % NO_TARGET_REPORT_EVERY == 0:
                # 长时间没目标时，周期性告诉上位机当前丢目标。
                wireless.send_line("COLORTRACE:NO_TARGET")

            if tick % DEBUG_PRINT_EVERY == 0:
                target_str = "YES" if result["has_target"] else ("HOLD" if result["target_locked"] else "NO")
                if result["in_center"]:
                    action = "CENTER"
                elif abs(result["err_x"]) > abs(result["err_y"]):
                    action = "LEFT" if result["err_x"] > 0 else "RIGHT"
                else:
                    action = "UP" if result["err_y"] > 0 else "DOWN"

                # 打印每次控制计算的关键量，便于调 PID 和占空比范围。
                msg = (
                    "{} err=({:>+5.0f},{:>+5.0f}) corr=({:>+6.0f},{:>+6.0f}) "
                    "raw=({:>+5.0f},{:>+5.0f},{:>+5.0f}) duty=({:>+5.0f},{:>+5.0f},{:>+5.0f}) [{}]"
                ).format(
                    target_str,
                    result["err_x"],
                    result["err_y"],
                    result["correction_x"],
                    result["correction_y"],
                    result["raw_duty"][0],
                    result["raw_duty"][1],
                    result["raw_duty"][2],
                    result["duty"][0],
                    result["duty"][1],
                    result["duty"][2],
                    action,
                )
                print(msg)

        else:
            if tick % 100 == 0:
                print("[DEBUG] update() returned None!")

        if switch2.value() != state2:
            color_trace.stop()
            MotorControl.motor_1.duty(0)
            MotorControl.motor_2.duty(0)
            MotorControl.motor_3.duty(0)
            print("\nStopped.")
            break

        time.sleep_ms(MAIN_LOOP_DELAY_MS)
        gc.collect()


if __name__ == "__main__":
    run()
