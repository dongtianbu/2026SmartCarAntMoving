import MotorControl
# import IMUVertical
# import PID
# import PIDYawUsart
from MCXVisionUsart import MCXVisionUsart
from WirelessUsart import WirelessUsart
from ColorTrace import ColorTraceController

from machine import Pin
import time
import gc

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

VISION_BAUDRATE = 115200
WIRELESS_BAUDRATE = 460800

TRACK_BASE_DUTY = 6000
TRACK_MAX_DUTY = 7500
TRACK_MIN_ACTIVE_DUTY = 5700
TRACK_TARGET_HOLD_MS = 120
TRACK_KP_X = 0.2
TRACK_KD_X = 0.75
TRACK_KP_Y = 0.2
TRACK_KD_Y = 0.75
TRACK_DEAD_ZONE = 6

DEBUG_PRINT_EVERY = 10
NO_TARGET_REPORT_EVERY = 50
MAIN_LOOP_DELAY_MS = 1


if __name__ == "__main__":
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
                                       dead_zone=TRACK_DEAD_ZONE)

    print("=== Color Trace ===")
    print(f"Screen: {color_trace.SCREEN_W}x{color_trace.SCREEN_H}")
    print(f"Target center: ({color_trace.CENTER_X}, {color_trace.CENTER_Y})")
    print(f"Base duty: {color_trace.base_duty}")
    print(f"Tracking duty: min={color_trace.min_tracking_duty} max={color_trace.max_tracking_duty}")
    print(f"Target hold: {color_trace.target_hold_ms} ms")
    print(f"PID: kp_x={TRACK_KP_X} kd_x={TRACK_KD_X} kp_y={TRACK_KP_Y} kd_y={TRACK_KD_Y}")

    print("Press switch2 (D9) to stop.\n")

    time.sleep(1)
    color_trace.start()
    print("GO! Tracking sandbag...\n")

    tick = 0

    while True:
        result = color_trace.update(MotorControl.motor_1, MotorControl.motor_2, MotorControl.motor_3)

        if result is not None:
            led.toggle()
            tick += 1

            if result["has_target"]:
                center_x = result["cx"]
                center_y = result["cy"]
                width = result["box"][2] - result["box"][0] + 1
                height = result["box"][3] - result["box"][1] + 1
                send_str = "COLORTRACE:{:.1f},{:.1f},{:.0f},{:.0f}\n".format(
                    center_x, center_y, width, height
                )
                wireless.send_line(send_str[:-1])
            elif tick % NO_TARGET_REPORT_EVERY == 0:
                wireless.send_line("COLORTRACE:NO_TARGET")

            if tick % DEBUG_PRINT_EVERY == 0:
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
                       f"corr=({result['correction_x']:>+6.0f},{result['correction_y']:>+6.0f}) " +
                       f"raw=({result['raw_duty'][0]:>+5.0f},{result['raw_duty'][1]:>+5.0f},{result['raw_duty'][2]:>+5.0f}) " +
                       f"duty=({result['duty'][0]:>+5.0f},{result['duty'][1]:>+5.0f},{result['duty'][2]:>+5.0f}) [{action}]")
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
