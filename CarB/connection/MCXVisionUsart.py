"""MCX 视觉模块串口封装。

直接调用时常用入口：
- `recv_bytes()`：拿原始字节
- `parse_frame(raw)`：解析一帧视觉结果
- `clear_rx()`：开始识别前清空残留数据

如果你只是想拿到“目标框中心点和边界框”，这个类就是主入口。
"""

from machine import UART
import struct


FRAME_HEADER = 0xAA
FRAME_TAIL = 0xFF
FRAME_SIZE = 11
NO_TARGET_IDX = 0xFF
VIEW_WIDTH = 160
VIEW_HEIGHT = 120


class MCXVisionUsart:
    """读取并解析视觉模块输出的固定长度帧。"""
    FRAME_HEADER = FRAME_HEADER
    FRAME_TAIL = FRAME_TAIL
    FRAME_SIZE = FRAME_SIZE
    NO_TARGET_IDX = NO_TARGET_IDX
    VIEW_WIDTH = VIEW_WIDTH
    VIEW_HEIGHT = VIEW_HEIGHT

    def __init__(self, baudrate=115200, bits=8, parity=None, stop=1):
        self._uart = UART(5)
        self._uart.init(baudrate=baudrate, bits=bits, parity=parity, stop=stop)
        self._baudrate = baudrate

    @property
    def uart(self):
        return self._uart

    @property
    def baudrate(self):
        return self._baudrate

    def send(self, data):
        """发送原始数据到视觉串口，一般调试时才会用到。"""
        if isinstance(data, str):
            data = data.encode("utf-8")
        return self._uart.write(data)

    def send_line(self, data=""):
        if isinstance(data, bytes):
            data = data.decode("utf-8")
        return self._uart.write((data + "\r\n").encode("utf-8"))

    def send_hex(self, data_bytes):
        hex_str = " ".join("{:02X}".format(b) for b in data_bytes)
        return self.send(hex_str)

    def available(self):
        return self._uart.any()

    def read(self, size=None):
        if size is None:
            size = self._uart.any()
        if size > 0:
            return self._uart.read(size)
        return None

    def read_all(self):
        count = self._uart.any()
        if count > 0:
            return self._uart.read(count)
        return None

    def read_line(self):
        return self._uart.readline()

    def read_into(self, buf):
        return self._uart.readinto(buf)

    def recv_str(self, size=None):
        """读取并尽量转成字符串，调试串口时比较方便。"""
        data = self.read(size)
        if data is not None:
            try:
                return data.decode("utf-8")
            except UnicodeError:
                return data.hex()
        return None

    def recv_bytes(self, size=None):
        return self.read(size)

    @staticmethod
    def parse_frame(raw):
        """
        解析一帧视觉数据。

        这是本文件最重要的“直接调用入口”之一。
        输入完整 11 字节原始帧，输出一个易读字典：
        - `has_target`：当前是否识别到目标
        - `x1/y1/x2/y2`：目标框四个边界
        - `center_x/center_y`：目标中心
        """
        if raw is None or len(raw) < FRAME_SIZE:
            return None
        if raw[0] != FRAME_HEADER or raw[FRAME_SIZE - 1] != FRAME_TAIL:
            return None

        try:
            idx = raw[1]
            x1 = struct.unpack("<H", raw[2:4])[0]
            y1 = struct.unpack("<H", raw[4:6])[0]
            x2 = struct.unpack("<H", raw[6:8])[0]
            y2 = struct.unpack("<H", raw[8:10])[0]

            has_target = (idx != NO_TARGET_IDX)
            if has_target:
                if not (0 <= x1 <= x2 < VIEW_WIDTH and 0 <= y1 <= y2 < VIEW_HEIGHT):
                    return None
                width = (x2 - x1 + 1)
                height = (y2 - y1 + 1)
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
            else:
                x1 = 0
                y1 = 0
                x2 = 0
                y2 = 0
                width = 0
                height = 0
                center_x = VIEW_WIDTH / 2.0
                center_y = VIEW_HEIGHT / 2.0

            return {
                "idx": idx,
                "has_target": has_target,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "width": width,
                "height": height,
                "center_x": center_x,
                "center_y": center_y,
                "view_w": VIEW_WIDTH,
                "view_h": VIEW_HEIGHT,
            }
        except Exception:
            return None

    def clear_rx(self):
        """清空视觉串口接收区，避免旧数据影响新一轮识别。"""
        while self._uart.any() > 0:
            self._uart.read(self._uart.any())

    def reinit(self, baudrate=None, bits=None, parity=None, stop=None):
        """重新配置视觉串口参数。"""
        kwargs = {}
        if baudrate is not None:
            kwargs["baudrate"] = baudrate
            self._baudrate = baudrate
        if bits is not None:
            kwargs["bits"] = bits
        if parity is not None:
            kwargs["parity"] = parity
        if stop is not None:
            kwargs["stop"] = stop
        self._uart.init(**kwargs)


def create_mcx_usart(baudrate=115200):
    """快速创建视觉串口对象。"""
    return MCXVisionUsart(baudrate)


if __name__ == "__main__":
    from machine import Pin
    import time
    import gc

    led = Pin("C4", Pin.OUT, value=True)
    switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    mcx = MCXVisionUsart(baudrate=115200)
    rx_buf = bytearray()

    print("MCX Vision USART Test Started")
    print("TX=D20  RX=D21  baudrate=115200")
    print("Protocol:")
    print("  AA | idx | x1(uint16) | y1(uint16) | x2(uint16) | y2(uint16) | FF")
    print("  idx=0   : target detected")
    print("  idx=255 : no target")
    print("  coordinate view: 160x120")
    print("Press switch2 (D9) to stop.\n")

    mcx.send_line("MCX Vision USART ready!")
    total_recv = 0

    while True:
        if mcx.available() > 0:
            raw = mcx.recv_bytes()
            if raw is not None:
                total_recv += len(raw)
                rx_buf.extend(raw)

                while len(rx_buf) >= FRAME_SIZE:
                    frame = MCXVisionUsart.parse_frame(bytes(rx_buf[:FRAME_SIZE]))
                    if frame is not None:
                        if frame["has_target"]:
                            print(
                                "obj[{}] x1={:>4d} y1={:>4d} x2={:>4d} y2={:>4d} cx={:>5.1f} cy={:>5.1f}".format(
                                    frame["idx"],
                                    frame["x1"],
                                    frame["y1"],
                                    frame["x2"],
                                    frame["y2"],
                                    frame["center_x"],
                                    frame["center_y"],
                                )
                            )
                        else:
                            print("NO_TARGET")
                        rx_buf = rx_buf[FRAME_SIZE:]
                    else:
                        rx_buf = rx_buf[1:]
        else:
            time.sleep_ms(10)

        led.toggle()

        if switch2.value() != state2:
            print("\nMCX Vision USART test stopped.")
            print("Total received: {} bytes".format(total_recv))
            break

        gc.collect()
