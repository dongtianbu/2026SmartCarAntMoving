"""从车通过无线串口接收主车 yaw 的测试程序。

第一阶段测试使用的协议：
- 主车 CarA 发送可读 ASCII 文本行："YAW:<deg>\\r\\n"。
- 本文件只解析这种文本格式，不解析旧的二进制协议帧。

C4 LED 说明：
- 当前板子的 C4 是低电平点亮，所以 Pin.value(0) 表示亮。
- 收到有效 yaw：LED 基本常亮，每 4 秒短灭一次。
- 还没有收到有效 yaw：每 4 秒一个长闪。
"""

from machine import Pin
import gc
import time

from WirelessUsartCars import WirelessUsartCars


STATE_ID_LEADER_YAW = 0x31

STATUS_NO_SIGNAL = 0
STATUS_RAW_ONLY = 1
STATUS_OTHER_PACKET = 2
STATUS_YAW = 3


DEFAULT_CONFIG = {
    "self_id": 2,
    "leader_id": 1,
    "uart_id": 2,
    "baudrate": 115200,
    "yaw_state_id": STATE_ID_LEADER_YAW,
    "led_pin": "C4",
    # 当前板子的 C4 是低电平点亮：value(0)=亮，value(1)=灭。
    "led_active_level": 0,
    "stop_switch_pin": "D9",
    "stop_switch_pull": Pin.PULL_UP_47K,
    "stop_switch_enabled": False,
    "timeout_ms": 1500,
    "loop_delay_ms": 1,
    "rx_text_buf_limit": 128,
    "led_pattern_period_ms": 4000,
    "led_blink_on_ms": 700,
    "led_blink_gap_ms": 500,
    "yaw_valid_period_ms": 4000,
    "yaw_valid_on_ms": 3600,
    "error_strobe_period_ms": 160,
    "error_strobe_on_ms": 80,
    "setup_error_period_ms": 3000,
    "setup_error_on_ms": 1500,
    "enable_serial_log": False,
    "demo_led_patterns": False,
    "demo_stage_ms": 3000,
    "demo_stage_gap_ms": 3000,
}


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


def wrap_angle_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


class LeaderYawReceiver:
    """接收并解析主车发来的最新 ASCII 文本行 "YAW:<deg>"。"""

    def __init__(self, config):
        self.config = config
        self.radio = WirelessUsartCars(
            uart_id=config["uart_id"],
            baudrate=config["baudrate"],
            self_id=config["self_id"],
        )
        self.latest_yaw = None
        self.latest_packet = None
        self.last_rx_ms = None
        self.rx_text_buf = ""
        self.raw_byte_count = 0
        self.packet_count = 0
        self.other_packet_count = 0
        self.yaw_packet_count = 0
        self.parse_error_count = 0
        self.last_raw_ms = None
        self.last_packet_ms = None
        self.last_other_packet_ms = None
        self.last_parse_error_ms = None

    def start(self):
        self.radio.clear_rx()

    def update(self):
        now_ms = time.ticks_ms()
        raw = self.radio.read()
        if raw is not None:
            self.raw_byte_count += len(raw)
            self.last_raw_ms = now_ms
            self._append_text(raw)

        latest_yaw = self._pop_latest_yaw()
        if latest_yaw is None:
            return None

        self.latest_packet = None
        self.latest_yaw = latest_yaw
        self.last_rx_ms = now_ms
        self.yaw_packet_count += 1
        return {
            "leader_yaw": self.latest_yaw,
            "src_id": self.config["leader_id"],
            "dst_id": self.config["self_id"],
            "seq": self.yaw_packet_count & 0xFF,
            "rx_ms": now_ms,
            "raw_byte_count": self.raw_byte_count,
            "packet_count": self.packet_count,
            "yaw_packet_count": self.yaw_packet_count,
            "other_packet_count": self.other_packet_count,
        }

    def _append_text(self, raw):
        try:
            text = raw.decode("utf-8")
        except Exception:
            self.parse_error_count += 1
            return

        self.rx_text_buf += text
        limit = self.config["rx_text_buf_limit"]
        if len(self.rx_text_buf) > limit:
            self.rx_text_buf = self.rx_text_buf[-limit:]

    def _pop_latest_yaw(self):
        latest_yaw = None
        while "\n" in self.rx_text_buf:
            line, self.rx_text_buf = self.rx_text_buf.split("\n", 1)
            yaw = self._parse_yaw_line(line.strip())
            if yaw is not None:
                latest_yaw = yaw
        return latest_yaw

    def _parse_yaw_line(self, line):
        # 主车发送 "YAW:<angle>" 纯文本，串口助手和从车程序看到的是同一份可读数据。
        if not line.startswith("YAW:"):
            return None
        try:
            return wrap_angle_deg(float(line[4:]))
        except Exception:
            self.parse_error_count += 1
            return None

    def is_timeout(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        if self.last_rx_ms is None:
            return True
        return time.ticks_diff(now_ms, self.last_rx_ms) > self.config["timeout_ms"]

    def get_status(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        timeout_ms = self.config["timeout_ms"]

        if self.last_rx_ms is not None and time.ticks_diff(now_ms, self.last_rx_ms) <= timeout_ms:
            return STATUS_YAW
        return STATUS_NO_SIGNAL


class C4StatusLed:
    """C4 状态灯，支持配置点亮电平。

    当前 CarB 硬件的 C4 是低电平点亮。如果收到有效 yaw 时表现为基本灭、短亮，
    就说明 led_active_level 配反了。
    """

    def __init__(self, pin, config):
        self.pin = pin
        self.config = config
        self.active_level = 1 if config["led_active_level"] else 0
        self.inactive_level = 0 if self.active_level else 1
        self.base_ms = time.ticks_ms()
        self.off()

    def on(self):
        self.pin.value(self.active_level)

    def off(self):
        self.pin.value(self.inactive_level)

    def update(self, status, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()

        if status == STATUS_YAW:
            pos = time.ticks_diff(now_ms, self.base_ms) % self.config["yaw_valid_period_ms"]
            self._set_on(pos < self.config["yaw_valid_on_ms"])
            return

        if status == STATUS_RAW_ONLY:
            blink_count = 2
        elif status == STATUS_OTHER_PACKET:
            blink_count = 3
        else:
            blink_count = 1

        pos = time.ticks_diff(now_ms, self.base_ms) % self.config["led_pattern_period_ms"]
        self._set_on(self._is_in_counted_blink(pos, blink_count))

    def update_error(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        pos = time.ticks_diff(now_ms, self.base_ms) % self.config["error_strobe_period_ms"]
        self._set_on(pos < self.config["error_strobe_on_ms"])

    def update_setup_error(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        pos = time.ticks_diff(now_ms, self.base_ms) % self.config["setup_error_period_ms"]
        self._set_on(pos < self.config["setup_error_on_ms"])

    def _is_in_counted_blink(self, pos_ms, blink_count):
        on_ms = self.config["led_blink_on_ms"]
        gap_ms = self.config["led_blink_gap_ms"]
        step_ms = on_ms + gap_ms
        for index in range(blink_count):
            start_ms = index * step_ms
            if start_ms <= pos_ms < start_ms + on_ms:
                return True
        return False

    def _set_on(self, is_on):
        if is_on:
            self.on()
        else:
            self.off()


def run_c4_led_pattern_demo(status_led, config):
    stages = (
        STATUS_NO_SIGNAL,
        STATUS_RAW_ONLY,
        STATUS_OTHER_PACKET,
        STATUS_YAW,
        "error",
    )
    stage_index = 0
    stage_start_ms = time.ticks_ms()
    status_led.base_ms = stage_start_ms

    while True:
        now_ms = time.ticks_ms()
        stage = stages[stage_index]
        if stage == "error":
            status_led.update_error(now_ms)
        else:
            status_led.update(stage, now_ms)

        if time.ticks_diff(now_ms, stage_start_ms) >= config["demo_stage_ms"]:
            stage_index = (stage_index + 1) % len(stages)
            stage_start_ms = now_ms
            status_led.base_ms = now_ms
            status_led.off()
            time.sleep_ms(config["demo_stage_gap_ms"])

        time.sleep_ms(config["loop_delay_ms"])
        gc.collect()


def run_leader_yaw_receive_test(config=None):
    if config is None:
        config = build_config()

    inactive_level = 0 if config["led_active_level"] else 1
    led_pin = Pin(config["led_pin"], Pin.OUT, value=inactive_level)
    status_led = C4StatusLed(led_pin, config)

    if config["demo_led_patterns"]:
        run_c4_led_pattern_demo(status_led, config)
        return

    stop_switch = Pin(
        config["stop_switch_pin"],
        Pin.IN,
        pull=config["stop_switch_pull"],
    )
    stop_state = stop_switch.value()
    try:
        receiver = LeaderYawReceiver(config)
    except Exception:
        while True:
            status_led.update_setup_error()
            time.sleep_ms(config["loop_delay_ms"])

    if config["enable_serial_log"]:
        print("=== CarB Leader Yaw Receive Test ===")
        print("C4: 1 long blink=no valid yaw, mostly on=valid yaw")
        print("UART({}) baudrate={}".format(config["uart_id"], config["baudrate"]))
        print("self_id={} leader_id={}".format(config["self_id"], config["leader_id"]))
        print("Stop switch: {}".format(config["stop_switch_pin"]))
        print("")

    receiver.start()

    rx_count = 0
    last_wait_print_ms = time.ticks_ms()

    try:
        while True:
            data = receiver.update()
            now_ms = time.ticks_ms()

            if data is not None:
                rx_count += 1
                if config["enable_serial_log"]:
                    print(
                        (
                            "yaw_rx={} leader_yaw={:>7.2f} seq={} src={} dst={} "
                            "raw_bytes={} packets={} other_packets={}"
                        ).format(
                            rx_count,
                            data["leader_yaw"],
                            data["seq"],
                            data["src_id"],
                            data["dst_id"],
                            data["raw_byte_count"],
                            data["packet_count"],
                            data["other_packet_count"],
                        )
                    )
            elif config["enable_serial_log"] and receiver.is_timeout(now_ms):
                if time.ticks_diff(now_ms, last_wait_print_ms) >= config["timeout_ms"]:
                    print(
                        (
                            "waiting for leader yaw... raw_bytes={} packets={} "
                            "yaw_rx={} other_packets={}"
                        ).format(
                            receiver.raw_byte_count,
                            receiver.packet_count,
                            rx_count,
                            receiver.other_packet_count,
                        )
                    )
                    last_wait_print_ms = now_ms

            status_led.update(receiver.get_status(now_ms), now_ms)

            if config["stop_switch_enabled"] and stop_switch.value() != stop_state:
                if config["enable_serial_log"]:
                    print("Stop requested.")
                break

            time.sleep_ms(config["loop_delay_ms"])
            gc.collect()
    except Exception:
        while True:
            status_led.update_error()
            time.sleep_ms(config["loop_delay_ms"])
    finally:
        status_led.off()
        if config["enable_serial_log"]:
            print("Leader yaw receive test stopped. total_rx={}".format(rx_count))


if __name__ == "__main__":
    run_leader_yaw_receive_test()
