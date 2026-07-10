"""双车心跳通信测试。

这是 CarA/CarB 当前默认主程序真正调用的核心测试文件。

用途：
1. 验证两车之间的无线串口链路是否连通。
2. 验证双方是否能稳定互发心跳包。
3. 通过 LED 闪烁方式显示当前通信状态。

使用方法：
1. 在两台车上都部署本文件以及 `connection/WirelessUsartCars.py`。
2. 通过 `build_config(self_id=..., peer_id=...)` 分别给两台车配置不同 ID。
3. 在各自 `main.py` 中调用 `run_heartbeat_test(config)`。
4. 上电后观察：
   - 慢单闪：正在等待对方；
   - 双闪：通信正常；
   - 快闪：程序异常。
5. 如果配置了 `stop_switch_pin`，按下对应按键即可结束测试。

最常直接调用的两个入口：
- `build_config(...)`：生成配置
- `run_heartbeat_test(config)`：启动测试
"""

from machine import Pin
import gc
import time

from connection.WirelessUsartCars import WirelessUsartCars


STATE_SEARCHING_PEER = "searching_peer"
STATE_COMMUNICATION_OK = "communication_ok"
STATE_PROGRAM_ERROR = "program_error"

DEFAULT_CONFIG = {
    # UART 链路参数。
    "uart_id": 2,
    "baudrate": 460800,
    # 心跳收发时序参数。
    "send_interval_ms": 700,
    "start_delay_ms": 0,
    "link_hold_ms": 2000,
    "gc_interval_ms": 1000,
    "loop_delay_ms": 5,
    # LED 状态灯闪烁参数。
    "led_pin": "C4",
    "startup_blink_count": 6,
    "led_search_cycle_ms": 1200,
    "led_search_on_ms": 120,
    "led_ok_on_ms": 200,
    "led_ok_inner_gap_ms": 200,
    "led_ok_group_gap_ms": 1000,
    "led_fast_blink_period_ms": 100,
    # 可选的物理停止按键，便于实车调试。
    "stop_switch_pin": None,
    "stop_switch_pull": None,
}


def build_config(self_id, peer_id, **overrides):
    """生成测试配置。

    常用参数：
    - `self_id`：当前车辆 ID
    - `peer_id`：对方车辆 ID

    其他配置项可通过 `overrides` 按需覆盖。
    """
    config = DEFAULT_CONFIG.copy()
    config["self_id"] = self_id
    config["peer_id"] = peer_id
    for key in overrides:
        config[key] = overrides[key]
    return config


class WirelessCarsHeartbeatTest:
    """双车心跳测试主类。"""

    def __init__(self, config):
        self.config = config
        self.led = Pin(config["led_pin"], Pin.OUT, value=True)
        self.radio = None
        self.stop_switch = None
        self.stop_switch_state = None

    def _set_led(self, is_on):
        """统一处理 LED 亮灭。"""
        self.led.value(0 if is_on else 1)

    def _get_led_output(self, state, now_ms):
        """根据状态计算当前时刻 LED 应该亮还是灭。"""
        if state == STATE_SEARCHING_PEER:
            # 慢单闪：表示还没有等到对方心跳。
            phase = now_ms % self.config["led_search_cycle_ms"]
            return phase < self.config["led_search_on_ms"]

        if state == STATE_COMMUNICATION_OK:
            # 双闪：表示链路正常，双方都在稳定收发。
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

        # 快闪：程序出现异常时的告警模式。
        period_ms = self.config["led_fast_blink_period_ms"]
        return (now_ms % period_ms) < (period_ms // 2)

    def _apply_led_state(self, state, now_ms):
        """应用 LED 状态。"""
        self._set_led(self._get_led_output(state, now_ms))

    def _startup_blink(self):
        """上电后先闪几下灯，表示程序已经开始运行。"""
        for _ in range(self.config["startup_blink_count"]):
            self._set_led(True)
            time.sleep_ms(100)
            self._set_led(False)
            time.sleep_ms(100)

    def _setup_stop_switch(self):
        """按配置初始化停止按键。"""
        pin_name = self.config.get("stop_switch_pin")
        if not pin_name:
            return

        pull = self.config.get("stop_switch_pull")
        if pull is None:
            self.stop_switch = Pin(pin_name, Pin.IN)
        else:
            self.stop_switch = Pin(pin_name, Pin.IN, pull=pull)
        self.stop_switch_state = self.stop_switch.value()

    def _stop_requested(self):
        """检测是否请求停止测试。"""
        if self.stop_switch is None:
            return False
        return self.stop_switch.value() != self.stop_switch_state

    def _stop_gracefully(self):
        """正常结束测试时的收尾动作。"""
        print("Test program stop.")
        self._set_led(False)

    def _error_forever(self):
        """异常后进入快闪循环，便于肉眼识别故障。"""
        while True:
            self._apply_led_state(STATE_PROGRAM_ERROR, time.ticks_ms())
            time.sleep_ms(20)

    def _create_context(self):
        """创建主循环里反复更新的运行状态。"""
        now_ms = time.ticks_ms()
        return {
            # send_count 用于人读日志，seq 由底层协议自行维护。
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
        """到发送周期时发送一包文本心跳。"""
        if time.ticks_diff(now_ms, ctx["last_send_ms"]) < self.config["send_interval_ms"]:
            return

        # 负载保留可读文本格式，方便上位机和串口工具直接观察。
        self.radio.send_packet(
            self.radio.MSG_TYPE_TEXT,
            "HB:{}:{}".format(self.config["self_id"], ctx["send_count"]),
            dst_id=self.config["peer_id"],
        )
        ctx["send_count"] += 1
        ctx["last_send_ms"] = now_ms

    def _handle_rx_packets(self, ctx, now_ms):
        """处理收到的包，只关心对方发来的心跳文本。"""
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

            # 即使 seq 重复，也说明链路依然存活，只是可能收到了重复帧。
            if packet["seq"] != ctx["last_peer_seq"]:
                ctx["last_peer_seq"] = packet["seq"]
            ctx["last_peer_rx_ms"] = now_ms

    def _get_comm_state(self, ctx, now_ms):
        """根据最近一次收到对方心跳的时间判断链路状态。"""
        if time.ticks_diff(now_ms, ctx["last_peer_rx_ms"]) < self.config["link_hold_ms"]:
            return STATE_COMMUNICATION_OK
        return STATE_SEARCHING_PEER

    def run_forever(self):
        """运行主循环。

        主循环顺序：
        1. 检查是否请求停止；
        2. 处理接收；
        3. 按周期发送心跳；
        4. 刷新 LED；
        5. 周期性执行垃圾回收。
        """
        self._setup_stop_switch()
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

            if self._stop_requested():
                self._stop_gracefully()
                return

            self._handle_rx_packets(ctx, now_ms)
            self._send_heartbeat_if_due(ctx, now_ms)
            self._apply_led_state(self._get_comm_state(ctx, now_ms), now_ms)

            if time.ticks_diff(now_ms, ctx["last_gc_ms"]) >= self.config["gc_interval_ms"]:
                gc.collect()
                ctx["last_gc_ms"] = now_ms

            time.sleep_ms(self.config["loop_delay_ms"])

    def run(self):
        """带异常保护的启动入口。"""
        try:
            self.run_forever()
        except Exception:
            self._error_forever()


def run_heartbeat_test(config):
    """最简启动入口：传入配置后直接跑完整测试。"""
    WirelessCarsHeartbeatTest(config).run()
