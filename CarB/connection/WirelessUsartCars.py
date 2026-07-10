"""双车无线串口协议封装。

这个模块最适合直接给业务代码调用：
- 发送文本：`send_text(...)`
- 发送命令：`send_command(...)`
- 发送状态值：`send_state_value(...)`
- 接收本车消息：`recv_packet_for_me()`

如果只想快速创建对象，直接调文件底部的
`create_wireless_usart_cars(...)` 即可。
"""

from machine import UART
import struct


FRAME_HEAD_1 = 0xAA
FRAME_HEAD_2 = 0x55
FRAME_BROADCAST_ID = 0xFF

MSG_TYPE_TEXT = 0x01
MSG_TYPE_STATE = 0x02
MSG_TYPE_COMMAND = 0x03
MSG_TYPE_ACK = 0x7F

# 协议帧由 2 字节帧头、5 字节固定字段、变长负载和 1 字节校验组成。
_FRAME_FIXED_SIZE = 8


class WirelessUsartCars:
    """
    车和车之间的无线串口协议类。

    直接调用时最常用的就是这几个方法：
    - `send_text(text, dst_id=...)`
    - `send_command(command, dst_id=...)`
    - `send_state_value(state_id, value, dst_id=...)`
    - `recv_packet_for_me()`

    默认串口映射：
    - `UART(2)` -> `TX=C6`, `RX=C7`

    协议帧格式：
    `[HEAD1][HEAD2][msg_type][src_id][dst_id][seq][payload_len][payload...][checksum]`
    """

    FRAME_HEAD_1 = FRAME_HEAD_1
    FRAME_HEAD_2 = FRAME_HEAD_2
    FRAME_BROADCAST_ID = FRAME_BROADCAST_ID

    MSG_TYPE_TEXT = MSG_TYPE_TEXT
    MSG_TYPE_STATE = MSG_TYPE_STATE
    MSG_TYPE_COMMAND = MSG_TYPE_COMMAND
    MSG_TYPE_ACK = MSG_TYPE_ACK

    def __init__(
        self,
        uart_id=2,
        baudrate=460800,
        bits=8,
        parity=None,
        stop=1,
        self_id=1,
        rx_buf_limit=512,
    ):
        self._uart = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=bits, parity=parity, stop=stop)
        self._uart_id = uart_id
        self._baudrate = baudrate
        self._self_id = self_id & 0xFF
        self._seq = 0
        # 软件缓冲区用于处理半包、粘包和帧头错位的情况。
        self._rx_buf_limit = max(rx_buf_limit, 64)
        self._rx_buf = bytearray()

    @property
    def uart(self):
        """底层 UART 对象，只有需要做更底层操作时才直接用。"""
        return self._uart

    @property
    def uart_id(self):
        return self._uart_id

    @property
    def baudrate(self):
        return self._baudrate

    @property
    def self_id(self):
        """当前这台车的逻辑编号。"""
        return self._self_id

    def set_self_id(self, self_id):
        """运行过程中修改本车 ID。"""
        self._self_id = self_id & 0xFF

    def send(self, data):
        """发送原始字节或字符串，不带协议封装。"""
        if isinstance(data, str):
            data = data.encode("utf-8")
        return self._uart.write(data)

    def send_str(self, data):
        """发送字符串；非字符串会先转成字符串。"""
        if isinstance(data, bytes):
            return self.send(data)
        return self.send(str(data))

    def send_line(self, data=""):
        """发送一行文本，自动补 `\\r\\n`。"""
        if isinstance(data, bytes):
            data = data.decode("utf-8", errors="ignore")
        return self.send_str(str(data) + "\r\n")

    def available(self):
        """查看串口接收区里还有多少字节可读。"""
        return self._uart.any()

    def read(self, size=None):
        """读取原始字节；不做协议解析。"""
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
        """读取原始数据并尽量按 UTF-8 转成字符串。"""
        data = self.read(size)
        if data is None:
            return None
        try:
            return data.decode("utf-8")
        except UnicodeError:
            return data.hex()

    def recv_bytes(self, size=None):
        return self.read(size)

    @staticmethod
    def _checksum(data):
        return sum(data) & 0xFF

    def next_seq(self):
        """生成下一个流水号，协议内部自动使用。"""
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    @staticmethod
    def _to_payload_bytes(payload):
        if payload is None:
            return b""
        if isinstance(payload, bytes):
            return payload
        if isinstance(payload, bytearray):
            return bytes(payload)
        if isinstance(payload, str):
            return payload.encode("utf-8")
        return str(payload).encode("utf-8")

    def build_packet(self, msg_type, payload=b"", src_id=None, dst_id=FRAME_BROADCAST_ID, seq=None):
        """只构造协议包，不发送。

        适合想先看原始帧内容、或者要自行缓存/转发时使用。
        """
        payload = self._to_payload_bytes(payload)
        payload_len = len(payload)
        if payload_len > 255:
            raise ValueError("payload too large, maximum length is 255 bytes")

        if src_id is None:
            src_id = self._self_id
        if seq is None:
            seq = self.next_seq()

        # 校验和只覆盖协议体，不包含前面的两个帧头字节。
        body = bytearray(
            (
                msg_type & 0xFF,
                src_id & 0xFF,
                dst_id & 0xFF,
                seq & 0xFF,
                payload_len & 0xFF,
            )
        )
        body.extend(payload)
        checksum = self._checksum(body)

        frame = bytearray((FRAME_HEAD_1, FRAME_HEAD_2))
        frame.extend(body)
        frame.append(checksum)
        return bytes(frame)

    def send_packet(self, msg_type, payload=b"", src_id=None, dst_id=FRAME_BROADCAST_ID, seq=None):
        """发送任意类型的协议包。"""
        packet = self.build_packet(msg_type, payload, src_id=src_id, dst_id=dst_id, seq=seq)
        return self.send(packet)

    def send_text(self, text, dst_id=FRAME_BROADCAST_ID, src_id=None):
        """最常用发送接口：发一条文本消息给指定车辆。"""
        return self.send_packet(MSG_TYPE_TEXT, text, src_id=src_id, dst_id=dst_id)

    def send_command(self, command, dst_id=FRAME_BROADCAST_ID, src_id=None):
        """发送命令字符串，适合做简单指令控制。"""
        return self.send_packet(MSG_TYPE_COMMAND, command, src_id=src_id, dst_id=dst_id)

    def send_state_value(self, state_id, value, dst_id=FRAME_BROADCAST_ID, src_id=None):
        """发送“状态编号 + 浮点值”，适合传传感器/控制量。"""
        payload = struct.pack("<Bf", state_id & 0xFF, float(value))
        return self.send_packet(MSG_TYPE_STATE, payload, src_id=src_id, dst_id=dst_id)

    def send_ack(self, seq, dst_id, src_id=None, ok=True):
        """发送确认包，通常用于告诉对方“我收到你第 seq 包了”。"""
        payload = struct.pack("<BB", seq & 0xFF, 1 if ok else 0)
        return self.send_packet(MSG_TYPE_ACK, payload, src_id=src_id, dst_id=dst_id)

    @staticmethod
    def parse_packet(raw):
        """把一个完整原始帧解析成字典。"""
        if raw is None or len(raw) < _FRAME_FIXED_SIZE:
            return None
        if raw[0] != FRAME_HEAD_1 or raw[1] != FRAME_HEAD_2:
            return None

        payload_len = raw[6]
        expect_len = _FRAME_FIXED_SIZE + payload_len
        if len(raw) != expect_len:
            return None

        body = raw[2:-1]
        checksum = raw[-1]
        if WirelessUsartCars._checksum(body) != checksum:
            return None

        payload = bytes(raw[7:-1])
        packet = {
            "msg_type": raw[2],
            "src_id": raw[3],
            "dst_id": raw[4],
            "seq": raw[5],
            "payload_len": payload_len,
            "payload": payload,
            "checksum": checksum,
            "raw": bytes(raw),
        }

        if packet["msg_type"] in (MSG_TYPE_TEXT, MSG_TYPE_COMMAND):
            try:
                packet["text"] = payload.decode("utf-8")
            except UnicodeError:
                packet["text"] = None
        elif packet["msg_type"] == MSG_TYPE_STATE and payload_len == 5:
            state_id, value = struct.unpack("<Bf", payload)
            packet["state_id"] = state_id
            packet["value"] = value
        elif packet["msg_type"] == MSG_TYPE_ACK and payload_len >= 2:
            ack_seq, ok = struct.unpack("<BB", payload[:2])
            packet["ack_seq"] = ack_seq
            packet["ok"] = bool(ok)

        return packet

    def _fill_rx_buf(self):
        """内部函数：把串口收到的新字节灌进接收缓冲区。"""
        count = self._uart.any()
        if count <= 0:
            return 0

        data = self._uart.read(count)
        if not data:
            return 0

        self._rx_buf.extend(data)
        overflow = len(self._rx_buf) - self._rx_buf_limit
        if overflow > 0:
            # 优先丢掉最旧的数据，保证解析器尽量处理最新收到的内容。
            del self._rx_buf[:overflow]
        return len(data)

    def recv_packet(self):
        """从接收缓冲区里取出一帧完整协议包。"""
        self._fill_rx_buf()

        while len(self._rx_buf) >= _FRAME_FIXED_SIZE:
            if self._rx_buf[0] != FRAME_HEAD_1 or self._rx_buf[1] != FRAME_HEAD_2:
                # 每次滑动 1 字节，直到重新对齐到合法帧头。
                del self._rx_buf[0]
                continue

            payload_len = self._rx_buf[6]
            frame_len = _FRAME_FIXED_SIZE + payload_len
            if len(self._rx_buf) < frame_len:
                return None

            raw = bytes(self._rx_buf[:frame_len])
            packet = self.parse_packet(raw)
            if packet is not None:
                del self._rx_buf[:frame_len]
                return packet
            # 如果校验失败或负载异常，就继续滑动查找下一帧。
            del self._rx_buf[0]

        return None

    def recv_all_packets(self, max_packets=None):
        """一次性把当前缓冲区里能解析的包全部取出来。"""
        packets = []
        while True:
            packet = self.recv_packet()
            if packet is None:
                break
            packets.append(packet)
            if max_packets is not None and len(packets) >= max_packets:
                break
        return packets

    def recv_latest_packet(self, msg_type=None):
        """拿到当前最新的一包，可按消息类型过滤。"""
        latest = None
        for packet in self.recv_all_packets():
            if msg_type is None or packet["msg_type"] == msg_type:
                latest = packet
        return latest

    def is_for_me(self, packet, self_id=None):
        """判断一包数据是不是发给本车的。"""
        if packet is None:
            return False
        if self_id is None:
            self_id = self._self_id
        return packet["dst_id"] in (self_id & 0xFF, FRAME_BROADCAST_ID)

    def recv_packet_for_me(self):
        """最实用的接收入口：只返回发给本车或广播给所有车的包。"""
        while True:
            packet = self.recv_packet()
            if packet is None:
                return None
            if self.is_for_me(packet):
                return packet

    def clear_rx(self):
        """清空接收缓冲区和串口残留数据。"""
        self._rx_buf = bytearray()
        while self._uart.any() > 0:
            self._uart.read(self._uart.any())

    def info(self):
        print("=== Wireless USART Cars ===")
        print("UART({}) baudrate={}".format(self._uart_id, self._baudrate))
        print("Protocol: AA 55 type src dst seq len payload checksum")
        print("self_id={}".format(self._self_id))

    @staticmethod
    def help():
        print("WirelessUsartCars helper")
        print("Methods:")
        print("  send_text(text, dst_id=255)")
        print("  send_command(command, dst_id=255)")
        print("  send_state_value(state_id, value, dst_id=255)")
        print("  recv_packet() / recv_packet_for_me() / recv_all_packets() / recv_latest_packet()")
        print("Message type:")
        print("  TEXT=0x01 STATE=0x02 COMMAND=0x03 ACK=0x7F")

    def reinit(self, uart_id=None, baudrate=None, bits=None, parity=None, stop=None):
        """重新初始化串口参数，不必重新 new 一个对象。"""
        kwargs = {}
        if uart_id is not None and uart_id != self._uart_id:
            self._uart_id = uart_id
            self._uart = UART(uart_id)
        if baudrate is not None:
            kwargs["baudrate"] = baudrate
            self._baudrate = baudrate
        if bits is not None:
            kwargs["bits"] = bits
        if parity is not None:
            kwargs["parity"] = parity
        if stop is not None:
            kwargs["stop"] = stop
        if kwargs or uart_id is not None:
            self._uart.init(**kwargs)


def create_wireless_usart_cars(uart_id=2, baudrate=460800, self_id=1):
    """快速创建 `WirelessUsartCars` 对象的便捷入口。"""
    return WirelessUsartCars(uart_id=uart_id, baudrate=baudrate, self_id=self_id)
