"""把 PID 偏航信息通过无线串口发出去。

这个文件很简单，主要给调试直行 PID 用。
最常直接调用的方法是 `send_yaw_data(...)`。
"""

from seekfree import WIRELESS_UART


class PIDYawUsart:
    """PID 偏航调试发送器。"""
    def __init__(self, baudrate=460800):
        self._wuart = WIRELESS_UART(baudrate)
        self._wuart.info()

    def send_yaw_data(self, yaw, yaw_target, yaw_error, correction):
        """发送当前航向、目标航向、误差和修正量。"""
        self._wuart.send_str("PIDYAW:{:.2f},{:.2f},{:.2f},{:.0f}\n".format(
            yaw, yaw_target, yaw_error, correction
        ))
