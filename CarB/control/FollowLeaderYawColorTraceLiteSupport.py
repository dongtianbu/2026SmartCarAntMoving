"""CarB 轻量组合控制的基础支持模块。

这里存放运行期会复用的动态加载器、通用函数和轻量类，
避免把所有逻辑继续堆在单一文件里。
"""

from machine import Pin
import gc
import math
import time

from FollowLeaderYawColorTraceLiteConfig import (
    IMU_TICK_MS,
    TUNABLE_SPECS,
    _find_tunable,
    _is_legacy_yaw_follow_name,
    _parse_bool,
    _yaw_mode_uses_leader,
)


_MOTOR = None
_VISION_CLASS = None
_COLOR_CLASS = None
_IMU_CLASS = None
_RADIO_CLASS = None
_LEADER_CLASS = None
_LINE_ANGLE_HELPER_CLASS = None


def _load_motor():
    global _MOTOR
    if _MOTOR is None:
        gc.collect()
        import MotorControl as motor
        _MOTOR = motor
        gc.collect()
    return _MOTOR


def _load_vision_class():
    global _VISION_CLASS
    if _VISION_CLASS is None:
        gc.collect()
        from MCXVisionUsart import MCXVisionUsart
        _VISION_CLASS = MCXVisionUsart
        gc.collect()
    return _VISION_CLASS


def _load_color_class():
    global _COLOR_CLASS
    if _COLOR_CLASS is None:
        gc.collect()
        from ColorTrace import ColorTraceController
        _COLOR_CLASS = ColorTraceController
        gc.collect()
    return _COLOR_CLASS


def _load_imu_class():
    global _IMU_CLASS
    if _IMU_CLASS is None:
        gc.collect()
        from IMUVertical import ImuSensorVertical
        _IMU_CLASS = ImuSensorVertical
        gc.collect()
    return _IMU_CLASS


def _load_radio_class():
    global _RADIO_CLASS
    if _RADIO_CLASS is None:
        gc.collect()
        from WirelessUsartCars import WirelessUsartCars
        _RADIO_CLASS = WirelessUsartCars
        gc.collect()
    return _RADIO_CLASS


def _load_leader_class():
    global _LEADER_CLASS
    if _LEADER_CLASS is None:
        gc.collect()
        from LeaderYawReceiveTest import LeaderYawReceiver
        _LEADER_CLASS = LeaderYawReceiver
        gc.collect()
    return _LEADER_CLASS


def _load_line_angle_helper_class():
    """按需导入线角闭环辅助模块，减少默认路径下的内存占用。"""
    global _LINE_ANGLE_HELPER_CLASS
    if _LINE_ANGLE_HELPER_CLASS is None:
        gc.collect()
        from VisionLineAngleControl import VisionLineAngleController
        _LINE_ANGLE_HELPER_CLASS = VisionLineAngleController
        gc.collect()
    return _LINE_ANGLE_HELPER_CLASS


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _metric_value(value):
    if value is None:
        return "NA"
    if isinstance(value, float):
        return "{:.4f}".format(value)
    return str(value)


def _continuous_yaw_error(target_deg, current_deg):
    return target_deg - current_deg


def _apply_min_vector_speed(vx, vy, min_speed):
    mag = math.sqrt(vx * vx + vy * vy)
    if min_speed <= 0 or mag <= 1e-6 or mag >= min_speed:
        return vx, vy
    scale = min_speed / mag
    return vx * scale, vy * scale


def _map_translate_vector_for_camera(vx, vy, config):
    if config["camera_look_right_enabled"]:
        return vy, -vx
    return vx, vy


class YawPID:
    """轻量 yaw PID，避免额外导入更重的通用 PID 模块。"""

    def __init__(self, config):
        self.kp = config["pid_kp"]
        self.ki = config["pid_ki"]
        self.kd = config["pid_kd"]
        self.output_limit = abs(config["max_rotate_speed"])
        self.integral_limit = abs(config["pid_integral_limit"])
        self.integral = 0.0
        self.last_error = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None

    def compute(self, error, dt_s):
        if dt_s <= 0:
            dt_s = IMU_TICK_MS / 1000.0
        self.integral = _clamp(self.integral + error * dt_s, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.last_error is None else (error - self.last_error) / dt_s
        self.last_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return _clamp(output, -self.output_limit, self.output_limit)


class SmallStatusLed:
    """状态灯辅助类，正常时常亮，异常时快闪。"""

    def __init__(self, pin_name, active_level):
        self.active = active_level
        self.inactive = 0 if active_level else 1
        self.pin = Pin(pin_name, Pin.OUT, value=self.inactive)
        self.last_ms = time.ticks_ms()
        self.on = False

    def healthy(self):
        self.pin.value(self.active)

    def error(self):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self.last_ms) >= 160:
            self.last_ms = now_ms
            self.on = not self.on
            self.pin.value(self.active if self.on else self.inactive)

    def off(self):
        self.pin.value(self.inactive)


class WirelessTuningSession:
    """无线调参会话，负责接收上位机命令并回写运行参数。"""

    def __init__(self, config, controller):
        radio_class = _load_radio_class()
        self.config = config
        self.controller = controller
        self.radio = radio_class(uart_id=config["uart_id"], baudrate=config["baudrate"], self_id=config["self_id"])
        self.buf = ""

    def start(self):
        self.radio.clear_rx()
        self.reply("TUNE READY", self.config["tuning_banner_repeat_count"])
        self.reply("CMD READY", self.config["tuning_banner_repeat_count"])

    def wrap(self, text):
        text = str(text).strip()
        if not text.startswith(self.config["tuning_protocol_prefix"]):
            text = self.config["tuning_protocol_prefix"] + text
        return self.config["tuning_protocol_frame_head"] + text + self.config["tuning_protocol_frame_tail"]

    def reply(self, text, repeat_count=None):
        if not self.config["tuning_reply_enabled"]:
            return
        if repeat_count is None:
            repeat_count = self.config["tuning_command_reply_repeat_count"]
        frame = self.wrap(text)
        for _ in range(max(1, int(repeat_count))):
            self.radio.send_line(frame)

    def _payloads(self):
        head = self.config["tuning_protocol_frame_head"]
        tail = self.config["tuning_protocol_frame_tail"]
        prefix = self.config["tuning_protocol_prefix"]
        payloads = []
        while True:
            start = self.buf.find(head)
            if start < 0:
                if self.config["tuning_protocol_accept_legacy"] and "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    payloads.append(line.strip())
                    continue
                break
            end = self.buf.find(tail, start + len(head))
            if end < 0:
                if start > 0:
                    self.buf = self.buf[start:]
                break
            text = self.buf[start + len(head):end].strip()
            self.buf = self.buf[end + len(tail):]
            index = text.find(prefix)
            if index >= 0:
                payloads.append(text[index + len(prefix):].strip())
        limit = max(128, int(self.config["tuning_frame_buffer_limit"]))
        if len(self.buf) > limit:
            self.buf = self.buf[-limit:]
        return payloads

    def update(self):
        raw = self.radio.read()
        if raw is None:
            return
        try:
            self.buf += raw.decode("utf-8")
        except Exception:
            self.reply("ERR decode")
            return
        limit = max(int(self.config["tuning_rx_text_buf_limit"]), int(self.config["tuning_frame_buffer_limit"]))
        if len(self.buf) > limit:
            self.buf = self.buf[-limit:]
        for line in self._payloads():
            self.handle(line)

    def handle(self, line):
        line = line.strip()
        upper = line.upper()
        if not line:
            return
        if upper == "HELP":
            self.reply("OK HELP GET LIST SET AI_STOP AI_START AI_DEVIATE AI_PERTURB AI_STATUS")
            return
        if upper == "LIST":
            for name, _, _ in TUNABLE_SPECS:
                self.reply("{}={}".format(name, self.controller.read_tunable(name)))
            return
        if upper.startswith("GET "):
            name = line[4:].strip().upper()
            if _find_tunable(name) is None and not _is_legacy_yaw_follow_name(name):
                self.reply("ERR unknown {}".format(name))
            else:
                self.reply("{}={}".format(name, self.controller.read_tunable(name)))
            return
        if "=" in line:
            name, value = line.split("=", 1)
            ok, message = self.controller.apply_tunable(name.strip().upper(), value.strip())
            self.reply(message if message else ("OK" if ok else "ERR"))
            return
        handled, message = self.controller.handle_ai_command(line)
        self.reply(message if handled else "ERR unsupported")
