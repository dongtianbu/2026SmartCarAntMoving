"""HardwareBlindBox 默认启动文件。

烧录到板子后，程序会先完成启动阶段需要的固定引脚配置，
然后进入三路互补 PWM 功率输出程序。
"""

from machine import Pin

from PwmPowerOutput import run_pwm_power_output


# ---------------------------------------------------------------------------
# 人工调试区
# 这里放启动阶段需要固定设置的引脚，便于现场快速修改。
# ---------------------------------------------------------------------------

D19_OUTPUT_PIN = "D19"
# 启动时需要拉高的控制引脚。

D19_OUTPUT_LEVEL = 1
# D19 输出电平。
# 1 表示上电后立即拉高。


# 启动后立刻把 D19 配成推挽输出并拉高。
d19_output = Pin(D19_OUTPUT_PIN, Pin.OUT, value=D19_OUTPUT_LEVEL)


run_pwm_power_output()
