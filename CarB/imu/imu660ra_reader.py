"""IMU660RA 原始读数测试脚本。

这个脚本直接打印陀螺仪和加速度计换算值，
适合先确认 IMU 本身和采样链路是否正常。
"""

from machine import Pin
from smartcar import ticker
from seekfree import IMU660RA
import gc
import time

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

GYRO_SENSITIVITY = 16.4
ACC_SENSITIVITY = 2048.0

imu = IMU660RA()
imu_data = imu.get()

ticker_flag = False
ticker_count = 0

def pit_handler(instance):
    """定时器回调：告诉主循环“该取一拍新数据了”。"""
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else 1

pit = ticker(1)
pit.capture_list(imu)
pit.callback(pit_handler)
pit.start(10)

print("IMU660RA Gyroscope Reader Started")
print("Gyro unit: dps (degrees per second)")
print("Acc unit: g")
print("Press switch2 (D9) to stop.\n")

while True:
    if ticker_flag and ticker_count % 20 == 0:
        led.toggle()

        gyro_x_dps = imu_data[3] / GYRO_SENSITIVITY
        gyro_y_dps = imu_data[4] / GYRO_SENSITIVITY
        gyro_z_dps = imu_data[5] / GYRO_SENSITIVITY

        acc_x_g = imu_data[0] / ACC_SENSITIVITY
        acc_y_g = imu_data[1] / ACC_SENSITIVITY
        acc_z_g = imu_data[2] / ACC_SENSITIVITY

        print("gyro = {:>8.2f}, {:>8.2f}, {:>8.2f} dps".format(gyro_x_dps, gyro_y_dps, gyro_z_dps))
        print("acc  = {:>8.3f}, {:>8.3f}, {:>8.3f} g".format(acc_x_g, acc_y_g, acc_z_g))
        print("---")
        ticker_flag = False

    if switch2.value() != state2:
        pit.stop()
        print("IMU660RA Reader stopped.")
        break

    gc.collect()
