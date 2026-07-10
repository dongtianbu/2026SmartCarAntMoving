"""无线串口基础测试脚本。

用途：
1. 确认板载无线串口能否正常初始化。
2. 确认上位机能否持续收到文本输出。
3. 确认板载 LED 和停止按键工作正常。

使用方法：
1. 保证无线串口模块已经接好。
   默认说明：`TX=C6`、`RX=C7`、`RTS=C5`
2. 直接在板子上运行本文件。
3. 打开上位机串口工具，波特率设为 `460800`。
4. 观察是否持续收到 `TEST:count=...` 文本。
5. 按下 `D9` 按键停止测试。
"""

from machine import Pin
from seekfree import WIRELESS_UART
import time


led = Pin("C4", Pin.OUT, value=True)
switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

WIRELESS_BAUDRATE = 460800
SEND_INTERVAL_MS = 100


def run():
    """运行无线串口基础测试。"""
    wireless = WIRELESS_UART(WIRELESS_BAUDRATE)
    wireless.info()
    time.sleep_ms(500)

    print("=== Wireless USART Test ===")
    print("TX=C6  RX=C7  RTS=C5")
    print("Baudrate={}".format(WIRELESS_BAUDRATE))
    print("Press switch2 (D9) to stop.\n")

    count = 0

    while True:
        count += 1

        # 周期性发送计数和时间戳，便于确认串口链路是否稳定。
        wireless.send_str("TEST:count={} time={}\n".format(count, time.ticks_ms()))
        led.toggle()

        if switch2.value() != state2:
            print("\nStopped.")
            break

        time.sleep_ms(SEND_INTERVAL_MS)

    print("Test completed.")


if __name__ == "__main__":
    run()
