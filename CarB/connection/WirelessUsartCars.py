from machine import UART
import struct


FRAME_HEAD_1 = 0xAA
FRAME_HEAD_2 = 0x55
FRAME_BROADCAST_ID = 0xFF

MSG_TYPE_TEXT = 0x01
MSG_TYPE_STATE = 0x02
MSG_TYPE_COMMAND = 0x03
MSG_TYPE_ACK = 0x7F

_FRAME_FIXED_SIZE = 8


class WirelessUsartCars:
    """
    Car-to-car wireless UART wrapper.

    Default hardware mapping follows the common wireless serial module wiring:
    UART(2) -> TX=C6, RX=C7

    Frame format:
    [HEAD1][HEAD2][msg_type][src_id][dst_id][seq][payload_len][payload...][checksum]

    checksum = sum(bytes from msg_type to payload end) & 0xFF
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
        self._rx_buf_limit = max(rx_buf_limit, 64)
        self._rx_buf = bytearray()

    @property
    def uart(self):
        return self._uart

    @property
    def uart_id(self):
        return self._uart_id

    @property
    def baudrate(self):
        return self._baudrate

    @property
    def self_id(self):
        return self._self_id

    def set_self_id(self, self_id):
        self._self_id = self_id & 0xFF

    def send(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        return self._uart.write(data)

    def send_str(self, data):
        if isinstance(data, bytes):
            return self.send(data)
        return self.send(str(data))

    def send_line(self, data=""):
        if isinstance(data, bytes):
            data = data.decode("utf-8", errors="ignore")
        return self.send_str(str(data) + "\r\n")

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
        payload = self._to_payload_bytes(payload)
        payload_len = len(payload)
        if payload_len > 255:
            raise ValueError("payload too large, maximum length is 255 bytes")

        if src_id is None:
            src_id = self._self_id
        if seq is None:
            seq = self.next_seq()

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
        packet = self.build_packet(msg_type, payload, src_id=src_id, dst_id=dst_id, seq=seq)
        return self.send(packet)

    def send_text(self, text, dst_id=FRAME_BROADCAST_ID, src_id=None):
        return self.send_packet(MSG_TYPE_TEXT, text, src_id=src_id, dst_id=dst_id)

    def send_command(self, command, dst_id=FRAME_BROADCAST_ID, src_id=None):
        return self.send_packet(MSG_TYPE_COMMAND, command, src_id=src_id, dst_id=dst_id)

    def send_state_value(self, state_id, value, dst_id=FRAME_BROADCAST_ID, src_id=None):
        payload = struct.pack("<Bf", state_id & 0xFF, float(value))
        return self.send_packet(MSG_TYPE_STATE, payload, src_id=src_id, dst_id=dst_id)

    def send_ack(self, seq, dst_id, src_id=None, ok=True):
        payload = struct.pack("<BB", seq & 0xFF, 1 if ok else 0)
        return self.send_packet(MSG_TYPE_ACK, payload, src_id=src_id, dst_id=dst_id)

    @staticmethod
    def parse_packet(raw):
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
        count = self._uart.any()
        if count <= 0:
            return 0

        data = self._uart.read(count)
        if not data:
            return 0

        self._rx_buf.extend(data)
        overflow = len(self._rx_buf) - self._rx_buf_limit
        if overflow > 0:
            del self._rx_buf[:overflow]
        return len(data)

    def recv_packet(self):
        self._fill_rx_buf()

        while len(self._rx_buf) >= _FRAME_FIXED_SIZE:
            if self._rx_buf[0] != FRAME_HEAD_1 or self._rx_buf[1] != FRAME_HEAD_2:
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
            del self._rx_buf[0]

        return None

    def recv_all_packets(self, max_packets=None):
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
        latest = None
        for packet in self.recv_all_packets():
            if msg_type is None or packet["msg_type"] == msg_type:
                latest = packet
        return latest

    def is_for_me(self, packet, self_id=None):
        if packet is None:
            return False
        if self_id is None:
            self_id = self._self_id
        return packet["dst_id"] in (self_id & 0xFF, FRAME_BROADCAST_ID)

    def recv_packet_for_me(self):
        while True:
            packet = self.recv_packet()
            if packet is None:
                return None
            if self.is_for_me(packet):
                return packet

    def clear_rx(self):
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
    return WirelessUsartCars(uart_id=uart_id, baudrate=baudrate, self_id=self_id)
