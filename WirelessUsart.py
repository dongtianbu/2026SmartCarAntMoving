
from seekfree import WIRELESS_UART


class WirelessUsart:

    def __init__(self, baudrate=460800):
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
        if isinstance(data, bytes):
            data = data.decode('utf-8', errors='ignore')
        return self._wuart.send_str(data)

    def send_line(self, data=""):
        return self.send_str(str(data) + "\n")

    def send_oscilloscope(self, *channels):
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
        flag = self.data_analysis()
        for i in range(8):
            if flag[i]:
                self._data_wave[i] = self.get_data(i)
        return self._data_wave

    def get_updated_data(self):
        flag = self.data_analysis()
        updated = {}
        for i in range(8):
            if flag[i]:
                val = self.get_data(i)
                self._data_wave[i] = val
                updated[i] = val
        return updated

    def set_channel(self, channel, value):
        if 0 <= channel <= 7:
            self._data_wave[channel] = value
        else:
            raise ValueError("channel must be in range [0, 7]")

    def get_channel(self, channel):
        if 0 <= channel <= 7:
            return self._data_wave[channel]
        raise ValueError("channel must be in range [0, 7]")

    def info(self):
        return self._wuart.info()

    @staticmethod
    def help():
        WIRELESS_UART.help()


def create_wireless_usart(baudrate=460800):
    return WirelessUsart(baudrate)
