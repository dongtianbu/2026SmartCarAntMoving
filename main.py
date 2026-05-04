import MotorControl
import IMUVertical
import PID
import PIDYawUsart

from machine import Pin
import time
import gc

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

BASE_DUTY = 6300
PID_KP = 50
PID_KI = 3
PID_KD = 40


if __name__ == "__main__":
    imu = IMUVertical.ImuSensorVertical()
    imu.init()
    imu.calibrate()
    print("IMU ready.\n")

    controller = PID.StraightLineController(
        imu, base_duty=BASE_DUTY,
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        dead_zone=0.3
    )

    yaw_sender = PIDYawUsart.PIDYawUsart()

    print("=== PID Straight Line ===")
    print(f"Base Duty: {BASE_DUTY}")
    print(f"PID: Kp={PID_KP}  Ki={PID_KI}  Kd={PID_KD}")
    print("USART data sending enabled")
    print("Press switch2 (D9) to stop.\n")

    time.sleep(1)
    print("GO!\n")

    controller.start(yaw_target=imu.yaw)

    while True:
        result = controller.update(MotorControl.motor_1, MotorControl.motor_3)

        if result is not None:
            led.toggle()
            yaw_sender.send_yaw_data(
                result['yaw'],
                result['yaw_target'],
                result['yaw_error'],
                result['correction']
            )
            if imu._tick_count % 10 == 0:
                print(f"yaw={result['yaw']:>7.1f}°  err={result['yaw_error']:>6.1f}°  corr={result['correction']:>6.0f}  M1={result['duty_1']:>5.0f} M3={result['duty_3']:>5.0f}")

        if switch2.value() != state2:
            controller.stop()
            MotorControl.motor_1.duty(0)
            MotorControl.motor_3.duty(0)
            print("\nStopped.")
            break

        time.sleep_ms(1)
        gc.collect()
