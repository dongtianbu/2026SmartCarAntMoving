"""无线串口助手。

这个模块主要给上位机调试、波形观察、文字回传用。
如果业务里只想“发字符串到无线串口”，直接使用：
- `WirelessUsart.send_line(...)`
- `WirelessUsart.send_tracking_error(...)`

快速创建入口：`create_wireless_usart(...)`
"""

from seekfree import WIRELESS_UART

TRACK_ERROR_PREFIX = "VISION_ERR"


class WirelessUsart:
    """对 `seekfree.WIRELESS_UART` 的轻量封装。"""

    def __init__(self, baudrate=115200):
        self._wuart = WIRELESS_UART(baudrate)
        self._baudrate = baudrate
        self._data_wave = [0.0] * 8

    @property
    def wuart(self):
        return self._wuart

    @property
    def baudrate(self):
        return self._baudrate

    def send_str(self, data):
        """发送一段字符串。"""
        if isinstance(data, bytes):
            data = data.decode('utf-8', errors='ignore')
        return self._wuart.send_str(data)

    def send_line(self, data=""):
        """发送一行文本，自动补换行。"""
        return self.send_str(str(data) + "\n")

    def send_tracking_error(self, err_x, err_y, prefix=TRACK_ERROR_PREFIX):
        """发送视觉误差，格式适合上位机直接解析。"""
        return self.send_line("{}:{:+.1f},{:+.1f}".format(prefix, err_x, err_y))

    def send_tracking_no_target(self, prefix=TRACK_ERROR_PREFIX):
        """告诉上位机当前没有识别到目标。"""
        return self.send_line("{}:NO_TARGET".format(prefix))

    def send_oscilloscope(self, *channels):
        """把多个通道值发给逐飞配套示波器。"""
        return self._wuart.send_oscilloscope(*channels)

    def send_ccd_image(self, index):
        return self._wuart.send_ccd_image(index)

    def data_analysis(self):
        return self._wuart.data_analysis()

    def get_data(self, channel=0):
        if 0 <= channel <= 7:
            return self._wuart.get_data(channel)
        raise ValueError("channel must be in range [0, 7]")

    def get_all_data(self):
        """读取 8 个通道里目前缓存的最新值。"""
        flag = self.data_analysis()
        for i in range(8):
            if flag[i]:
                self._data_wave[i] = self.get_data(i)
        return self._data_wave

    def get_updated_data(self):
        """只返回这次新收到的通道值，避免每次都处理 8 路。"""
        flag = self.data_analysis()
        updated = {}
        for i in range(8):
            if flag[i]:
                val = self.get_data(i)
                self._data_wave[i] = val
                updated[i] = val
        return updated

    def set_channel(self, channel, value):
        """手动改本地缓存的通道值，不会主动下发到硬件。"""
        if 0 <= channel <= 7:
            self._data_wave[channel] = value
        else:
            raise ValueError("channel must be in range [0, 7]")

    def get_channel(self, channel):
        """读取本地缓存的某一路通道值。"""
        if 0 <= channel <= 7:
            return self._data_wave[channel]
        raise ValueError("channel must be in range [0, 7]")

    def info(self):
        return self._wuart.info()

    @staticmethod
    def help():
        WIRELESS_UART.help()


def create_wireless_usart(baudrate=115200):
    """快速创建无线串口对象。"""
    return WirelessUsart(baudrate)
