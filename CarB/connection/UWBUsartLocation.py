"""Receive UWB location lines from UART2, parse them, and forward the current coordinate
through the on-board wireless serial module.

Supported incoming protocols:
- Single solution: `POS:<x>,<y>\r\n`
- Dual solution:   `POS:<x1>,<y1>,<x2>,<y2>\r\n`

Forwarded protocol:
- `POS:<x>,<y>\r\n`

When a dual-solution line is received, `selected_solution_index` decides which solution is
forwarded as the current coordinate:
- 0: first solution
- 1: second solution
"""

from machine import UART
from seekfree import WIRELESS_UART
import gc
import time


DEFAULT_CONFIG = {
    "rx_uart_id": 2,
    "rx_baudrate": 115200,
    "tx_baudrate": 115200,
    "bits": 8,
    "parity": None,
    "stop": 1,
    "selected_solution_index": 0,
    "rx_text_buf_limit": 160,
    "loop_delay_ms": 5,
    "enable_print_log": True,
}


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


class UWBUsartLocation:
    def __init__(
        self,
        rx_uart_id=2,
        rx_baudrate=115200,
        tx_baudrate=115200,
        bits=8,
        parity=None,
        stop=1,
        selected_solution_index=0,
        rx_text_buf_limit=160,
    ):
        self._uart = UART(rx_uart_id)
        self._uart.init(baudrate=rx_baudrate, bits=bits, parity=parity, stop=stop)
        self._wireless = WIRELESS_UART(tx_baudrate)
        self._rx_uart_id = rx_uart_id
        self._rx_baudrate = rx_baudrate
        self._tx_baudrate = tx_baudrate
        self._selected_solution_index = 0 if selected_solution_index <= 0 else 1
        self._rx_text_buf_limit = max(rx_text_buf_limit, 64)
        self._rx_text_buf = ""
        self.latest_packet = None
        self.latest_position = None
        self.latest_raw_line = None
        self.line_count = 0
        self.parse_error_count = 0

    @property
    def uart(self):
        return self._uart

    @property
    def wireless(self):
        return self._wireless

    @property
    def selected_solution_index(self):
        return self._selected_solution_index

    def set_selected_solution_index(self, index):
        self._selected_solution_index = 0 if index <= 0 else 1

    def start(self):
        self.clear_rx()

    def clear_rx(self):
        while self._uart.any() > 0:
            self._uart.read(self._uart.any())
        self._rx_text_buf = ""

    def available(self):
        return self._uart.any()

    def read(self, size=None):
        if size is None:
            size = self._uart.any()
        if size > 0:
            return self._uart.read(size)
        return None

    def read_line(self):
        return self._uart.readline()

    def update(self):
        raw = self.read()
        if raw is not None:
            self._append_text(raw)
        return self._pop_latest_packet()

    def send_current_position(self):
        if self.latest_position is None:
            return 0
        line = self.format_position_line(
            self.latest_position["x"],
            self.latest_position["y"],
        )
        return self._wireless.send_str(line)

    @staticmethod
    def format_position_line(x, y):
        return "POS:{:.2f},{:.2f}\r\n".format(x, y)

    def _append_text(self, raw):
        try:
            text = raw.decode("utf-8")
        except Exception:
            self.parse_error_count += 1
            return

        self._rx_text_buf += text
        if len(self._rx_text_buf) > self._rx_text_buf_limit:
            self._rx_text_buf = self._rx_text_buf[-self._rx_text_buf_limit:]

    def _pop_latest_packet(self):
        latest_packet = None
        while "\n" in self._rx_text_buf:
            line, self._rx_text_buf = self._rx_text_buf.split("\n", 1)
            packet = self._parse_pos_line(line.strip())
            if packet is not None:
                latest_packet = packet
        if latest_packet is not None:
            self.latest_packet = latest_packet
            self.latest_raw_line = latest_packet["raw_line"]
            self.latest_position = {
                "x": latest_packet["active_x"],
                "y": latest_packet["active_y"],
                "solution_index": latest_packet["active_index"],
            }
        return latest_packet

    def _parse_pos_line(self, line):
        if not line.startswith("POS:"):
            return None

        payload = line[4:]
        parts = [part.strip() for part in payload.split(",") if part.strip() != ""]
        try:
            values = [float(part) for part in parts]
        except Exception:
            self.parse_error_count += 1
            return None

        if len(values) == 2:
            x, y = values
            packet = {
                "raw_line": line,
                "solution_count": 1,
                "solutions": ((x, y),),
                "active_index": 0,
                "active_x": x,
                "active_y": y,
            }
        elif len(values) == 4:
            x1, y1, x2, y2 = values
            active_index = self._selected_solution_index
            active_x, active_y = ((x1, y1), (x2, y2))[active_index]
            packet = {
                "raw_line": line,
                "solution_count": 2,
                "solutions": ((x1, y1), (x2, y2)),
                "active_index": active_index,
                "active_x": active_x,
                "active_y": active_y,
            }
        else:
            self.parse_error_count += 1
            return None

        self.line_count += 1
        return packet


def run_uwb_location_bridge(config=None):
    if config is None:
        config = build_config()

    bridge = UWBUsartLocation(
        rx_uart_id=config["rx_uart_id"],
        rx_baudrate=config["rx_baudrate"],
        tx_baudrate=config["tx_baudrate"],
        bits=config["bits"],
        parity=config["parity"],
        stop=config["stop"],
        selected_solution_index=config["selected_solution_index"],
        rx_text_buf_limit=config["rx_text_buf_limit"],
    )
    bridge.start()

    if config.get("enable_print_log", True):
        print("UWB location bridge started")
        print("RX: UART({}) {} baud".format(config["rx_uart_id"], config["rx_baudrate"]))
        print("TX: wireless {} baud".format(config["tx_baudrate"]))
        print("selected_solution_index={}".format(config["selected_solution_index"]))

    while True:
        packet = bridge.update()
        if packet is not None:
            bridge.send_current_position()
            if config.get("enable_print_log", True):
                print(
                    "UWB raw={} active=({:.2f}, {:.2f}) solution_index={} count={}".format(
                        packet["raw_line"],
                        packet["active_x"],
                        packet["active_y"],
                        packet["active_index"],
                        packet["solution_count"],
                    )
                )
        time.sleep_ms(config["loop_delay_ms"])
        gc.collect()