from seekfree import WIRELESS_UART
from machine import Pin
import time

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

wireless = WIRELESS_UART(460800)
wireless.info()
time.sleep_ms(500)

print("=== Wireless USART Test ===")
print("TX=C6  RX=C7  RTS=C5")
print("Press switch2 (D9) to stop.\n")

count = 0

while True:
    count += 1
    
    wireless.send_str("TEST:count={} time={}\n".format(count, time.ticks_ms()))
    
    led.toggle()
    
    if switch2.value() != state2:
        print("\nStopped.")
        break
    
    time.sleep_ms(100)

print("Test completed.")
