from seekfree import WIRELESS_UART


class PIDYawUsart:
    def __init__(self, baudrate=460800):
        self._wuart = WIRELESS_UART(baudrate)
        self._wuart.info()

    def send_yaw_data(self, yaw, yaw_target, yaw_error, correction):
        self._wuart.send_str("PIDYAW:{:.2f},{:.2f},{:.2f},{:.0f}\n".format(
            yaw, yaw_target, yaw_error, correction
        ))
