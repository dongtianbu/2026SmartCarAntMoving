
from machine import UART
import struct


FRAME_HEADER = 0xAA
FRAME_TAIL = 0xFF


class MCXVisionUsart:

    def __init__(self, baudrate=460800, bits=8, parity=None, stop=1):
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
        if isinstance(data, str):
            data = data.encode('utf-8')
        return self._uart.write(data)

    def send_line(self, data=""):
        if isinstance(data, bytes):
            data = data.decode('utf-8')
        return self._uart.write((data + "\r\n").encode('utf-8'))

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
        data = self.read(size)
        if data is not None:
            try:
                return data.decode('utf-8')
            except UnicodeError:
                return data.hex()
        return None

    def recv_bytes(self, size=None):
        return self.read(size)

    @staticmethod
    def parse_frame(raw):
        if raw is None or len(raw) < 11:
            return None
        if raw[0] != FRAME_HEADER or raw[10] != FRAME_TAIL:
            return None
        try:
            idx = raw[1]
            x1 = struct.unpack('<H', raw[2:4])[0]
            y1 = struct.unpack('<H', raw[4:6])[0]
            x2 = struct.unpack('<H', raw[6:8])[0]
            y2 = struct.unpack('<H', raw[8:10])[0]
            return {"idx": idx, "x1": x1, "y1": y1, "x2": x2, "y2": y2}
        except Exception:
            return None

    def clear_rx(self):
        while self._uart.any() > 0:
            self._uart.read(self._uart.any())

    def reinit(self, baudrate=None, bits=None, parity=None, stop=None):
        kwargs = {}
        if baudrate is not None:
            kwargs['baudrate'] = baudrate
            self._baudrate = baudrate
        if bits is not None:
            kwargs['bits'] = bits
        if parity is not None:
            kwargs['parity'] = parity
        if stop is not None:
            kwargs['stop'] = stop
        self._uart.init(**kwargs)


def create_mcx_usart(baudrate=460800):
    return MCXVisionUsart(baudrate)


if __name__ == "__main__":
    from machine import Pin
    import time
    import gc

    led = Pin('C4', Pin.OUT, value=True)
    switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    mcx = MCXVisionUsart(baudrate=115200)

    print("MCX Vision USART Test Started")
    print("TX=D20  RX=D21  baudrate=115200")
    print("Protocol: AA | idx | x1(uint16) | y1(uint16) | x2(uint16) | y2(uint16) | FF")
    print("         frame_header | target_index | bounding_box (left,top,right,bottom) | tail")
    print("Press switch2 (D9) to stop.\n")

    mcx.send_line("MCX Vision USART ready!")
    total_recv = 0

    while True:
        if mcx.available() > 0:
            raw = mcx.recv_bytes()
            if raw is not None:
                total_recv += len(raw)
                frame = MCXVisionUsart.parse_frame(raw)
                if frame is not None:
                    print("obj[{}] x1={:>4d} y1={:>4d} x2={:>4d} y2={:>4d}".format(
                        frame["idx"], frame["x1"], frame["y1"],
                        frame["x2"], frame["y2"]))
                else:
                    hex_str = " ".join("{:02X}".format(b) for b in raw)
                    print("[RAW] {}".format(hex_str))
        else:
            time.sleep_ms(10)

        led.toggle()

        if switch2.value() != state2:
            print("\nMCX Vision USART test stopped.")
            print("Total received: {} bytes".format(total_recv))
            break

        gc.collect()
