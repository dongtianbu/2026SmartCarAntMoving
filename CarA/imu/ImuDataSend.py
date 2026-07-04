from machine import Pin
from seekfree import WIRELESS_UART
from imu.Imu import ImuSensor
import time
import gc

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

WIRELESS_UART.help()
time.sleep_ms(500)

wireless = WIRELESS_UART(460800)
wireless.info()
time.sleep_ms(500)

data_flag = wireless.data_analysis()
for i in range(8):
    wireless.get_data(i)

imu = ImuSensor()

print("IMU Data Send via Wireless USART Started")
print("TX=C6  RX=C7  RTS=C5  baudrate=460800")
print("Mode: printf (text) - open printf switch in oscilloscope!")
print("Channels: ch0=Roll  ch1=Pitch  ch2=Yaw")
print("Press switch2 (D9) to stop.\n")

imu.init()
imu.calibrate()
print("Start sending...\n")

send_count = 0

while True:
    data = imu.update()
    if data is not None:
        send_count += 1
        led.toggle()

        data_flag = wireless.data_analysis()

        if send_count % 20 == 0:
            wireless.send_str("RPY:{:.2f},{:.2f},{:.2f}\n".format(
                data["roll"], data["pitch"], data["yaw"]
            ))

    if switch2.value() != state2:
        imu.stop()
        print("IMU Data Send stopped.")
        break

    time.sleep_ms(1)
    gc.collect()
