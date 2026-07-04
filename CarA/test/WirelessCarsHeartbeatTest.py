from machine import Pin
import gc
import time

from connection.WirelessUsartCars import WirelessUsartCars


STATE_SEARCHING_PEER = "searching_peer"
STATE_COMMUNICATION_OK = "communication_ok"
STATE_PROGRAM_ERROR = "program_error"

DEFAULT_CONFIG = {
    "uart_id": 2,
    "baudrate": 460800,
    "send_interval_ms": 700,
    "start_delay_ms": 0,
    "link_hold_ms": 2000,
    "gc_interval_ms": 1000,
    "loop_delay_ms": 5,
    "led_pin": "C4",
    "startup_blink_count": 6,
    "led_search_cycle_ms": 1200,
    "led_search_on_ms": 120,
    "led_ok_on_ms": 200,
    "led_ok_inner_gap_ms": 200,
    "led_ok_group_gap_ms": 1000,
    "led_fast_blink_period_ms": 100,
}


def build_config(self_id, peer_id, **overrides):
    config = DEFAULT_CONFIG.copy()
    config["self_id"] = self_id
    config["peer_id"] = peer_id
    for key in overrides:
        config[key] = overrides[key]
    return config


class WirelessCarsHeartbeatTest:
    def __init__(self, config):
        self.config = config
        self.led = Pin(config["led_pin"], Pin.OUT, value=True)
        self.radio = None

    def _set_led(self, is_on):
        self.led.value(0 if is_on else 1)

    def _get_led_output(self, state, now_ms):
        if state == STATE_SEARCHING_PEER:
            phase = now_ms % self.config["led_search_cycle_ms"]
            return phase < self.config["led_search_on_ms"]

        if state == STATE_COMMUNICATION_OK:
            cycle_ms = (
                self.config["led_ok_on_ms"] * 2
                + self.config["led_ok_inner_gap_ms"]
                + self.config["led_ok_group_gap_ms"]
            )
            phase = now_ms % cycle_ms
            first_blink = phase < self.config["led_ok_on_ms"]
            second_blink_start = self.config["led_ok_on_ms"] + self.config["led_ok_inner_gap_ms"]
            second_blink_end = second_blink_start + self.config["led_ok_on_ms"]
            second_blink = second_blink_start <= phase < second_blink_end
            return first_blink or second_blink

        period_ms = self.config["led_fast_blink_period_ms"]
        return (now_ms % period_ms) < (period_ms // 2)

    def _apply_led_state(self, state, now_ms):
        self._set_led(self._get_led_output(state, now_ms))

    def _startup_blink(self):
        for _ in range(self.config["startup_blink_count"]):
            self._set_led(True)
            time.sleep_ms(100)
            self._set_led(False)
            time.sleep_ms(100)

    def _error_forever(self):
        while True:
            self._apply_led_state(STATE_PROGRAM_ERROR, time.ticks_ms())
            time.sleep_ms(20)

    def _create_context(self):
        now_ms = time.ticks_ms()
        return {
            "send_count": 0,
            "last_send_ms": time.ticks_add(
                now_ms,
                -self.config["send_interval_ms"] + self.config["start_delay_ms"],
            ),
            "last_peer_rx_ms": time.ticks_add(now_ms, -self.config["link_hold_ms"]),
            "last_peer_seq": None,
            "last_gc_ms": now_ms,
        }

    def _send_heartbeat_if_due(self, ctx, now_ms):
        if time.ticks_diff(now_ms, ctx["last_send_ms"]) < self.config["send_interval_ms"]:
            return

        self.radio.send_packet(
            self.radio.MSG_TYPE_TEXT,
            "HB:{}:{}".format(self.config["self_id"], ctx["send_count"]),
            dst_id=self.config["peer_id"],
        )
        ctx["send_count"] += 1
        ctx["last_send_ms"] = now_ms

    def _handle_rx_packets(self, ctx, now_ms):
        while True:
            packet = self.radio.recv_packet_for_me()
            if packet is None:
                break

            is_peer_text = (
                packet["msg_type"] == self.radio.MSG_TYPE_TEXT
                and packet["src_id"] == self.config["peer_id"]
            )
            if not is_peer_text:
                continue

            if packet["seq"] != ctx["last_peer_seq"]:
                ctx["last_peer_seq"] = packet["seq"]
            ctx["last_peer_rx_ms"] = now_ms

    def _get_comm_state(self, ctx, now_ms):
        if time.ticks_diff(now_ms, ctx["last_peer_rx_ms"]) < self.config["link_hold_ms"]:
            return STATE_COMMUNICATION_OK
        return STATE_SEARCHING_PEER

    def run_forever(self):
        self._startup_blink()

        self.radio = WirelessUsartCars(
            uart_id=self.config["uart_id"],
            baudrate=self.config["baudrate"],
            self_id=self.config["self_id"],
        )
        self.radio.clear_rx()

        ctx = self._create_context()

        while True:
            now_ms = time.ticks_ms()

            self._handle_rx_packets(ctx, now_ms)
            self._send_heartbeat_if_due(ctx, now_ms)
            self._apply_led_state(self._get_comm_state(ctx, now_ms), now_ms)

            if time.ticks_diff(now_ms, ctx["last_gc_ms"]) >= self.config["gc_interval_ms"]:
                gc.collect()
                ctx["last_gc_ms"] = now_ms

            time.sleep_ms(self.config["loop_delay_ms"])

    def run(self):
        try:
            self.run_forever()
        except Exception:
            self._error_forever()


def run_heartbeat_test(config):
    WirelessCarsHeartbeatTest(config).run()
