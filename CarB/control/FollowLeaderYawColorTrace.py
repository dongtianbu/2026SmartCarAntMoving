"""CarB 组合控制器。

职责分工：
1. 视觉追踪负责计算平移速度 vx、vy。
2. 无线串口负责提供主车 yaw，或在调参模式下接收电脑发送的参数。
3. 本车 IMU 和 yaw PID 负责计算旋转速度 omega。
4. 最终统一通过 MotorControl.drive_vector() 混合输出到底盘。
"""

from machine import Pin
import gc
import math
import time

from ColorTrace import ColorTraceController
from FollowLeaderYawPID import YawPID, continuous_yaw_error
from IMUVertical import ImuSensorVertical
from LeaderYawReceiveTest import (
    C4StatusLed,
    LeaderYawReceiver,
    STATUS_NO_SIGNAL,
    STATUS_YAW,
)
from MCXVisionUsart import MCXVisionUsart
from WirelessUsartCars import WirelessUsartCars
import MotorControl


# ---------------------------------------------------------------------------
# 人工调参区
# 所有需要现场人工调试的参数都集中放在文件顶部，注释放在代码右侧并尽量对齐。
# ---------------------------------------------------------------------------

SELF_ID                      = 2                 # 无线通信中本车的设备编号
LEADER_ID                    = 1                 # 无线通信中主车的设备编号
YAW_UART_ID                  = 2                 # yaw 跟随或无线调参共用的串口号
YAW_BAUDRATE                 = 115200            # yaw 跟随或无线调参共用的串口波特率
RX_TEXT_BUF_LIMIT            = 128               # 主车 yaw 文本接收缓冲区上限，过小可能截断数据
LEADER_TIMEOUT_MS            = 1500              # 跟随主车 yaw 时的超时阈值，单位毫秒
VISION_BAUDRATE              = 115200            # 视觉模块串口波特率，当前视觉模块固定接在 UART5

YAW_FOLLOW_LEADER_ENABLED    = False              # True 时闭环串口收到的主车 yaw，False 时固定闭环 0 度并启用电脑无线调参
YAW_FIXED_TARGET_DEG         = 0.0               # 关闭主车 yaw 跟随后，本车要闭环到的固定目标角，默认 0 度
TUNING_RX_TEXT_BUF_LIMIT     = 256               # 电脑无线调参文本缓冲区上限，过小时可能截断命令
TUNING_REPLY_ENABLED         = True              # 无线调参时是否通过同一串口回发执行结果

IMU_CAPTURE_DIV              = 1                 # IMU 采样分频系数，越大则实际处理频率越低
IMU_TICK_MS                  = 10                # IMU 更新周期，单位毫秒
IMU_ACC_RANGE_G              = 8                 # 加速度计量程，单位 g
IMU_GYRO_RANGE_DPS           = 2000              # 陀螺仪量程，单位度每秒
IMU_ACC_ALPHA                = 0.20              # 加速度一阶滤波系数，越大越灵敏，越小越平滑
IMU_COMP_ALPHA               = 0.98              # 互补滤波系数，越接近 1 越依赖陀螺仪短时响应
IMU_GYRO_CALI_N              = 300               # 陀螺仪校准采样次数，越大越稳，但启动等待越久
IMU_CALIBRATE_ON_START       = True              # 上电后是否自动做 IMU 校准
IMU_CALIBRATION_SETTLE_MS    = 500               # 开始校准前的静置等待时间，单位毫秒

PID_KP                       = 0.3               # yaw 比例系数，决定 yaw 误差对应的基础旋转力度
PID_KI                       = 0.05              # yaw 积分系数，用于消除长期小偏差
PID_KD                       = 0.1               # yaw 微分系数，用于抑制 yaw 闭环冲过头
PID_INTEGRAL_LIMIT           = 60.0              # yaw 积分限幅，防止积分累积过大后突然猛冲
YAW_DEADBAND_DEG             = 0.2               # yaw 死区，误差绝对值小于该角度时认为已经对正，单位度
MAX_ROTATE_SPEED             = 85.0              # yaw 旋转速度上限，对应底盘 speed 标度 0~100
MIN_COMMAND_SPEED            = 50.0              # yaw 非零输出时允许的最小旋转指令，过小可能转不动，过大容易抖动
ROTATE_SIGN                  = -1.0              # yaw 旋转方向修正符号，若本车转向与期望相反就改成相反数

COLOR_BASE_DUTY              = 6000              # 视觉独立控制里兼容旧状态字段使用的基础 duty 参考值
COLOR_MAX_TRACKING_DUTY      = 6500              # 视觉独立控制允许的最大电机 duty
COLOR_MIN_TRACKING_DUTY      = 5500              # 视觉独立控制下的最小非零 duty，用于帮助电机起转
COLOR_TARGET_HOLD_MS         = 100               # 丢帧后继续沿用上一帧目标的保持时间，单位毫秒
COLOR_KP_X                   = 0.015               # 横向闭环比例系数，决定 error_x 的主要修正力度
COLOR_KD_X                   = 0.25              # 横向闭环微分系数，主要用于抑制横向过冲
COLOR_X_OUTPUT_SIGN          = -1.0              # 横向输出方向修正符号，目标在右边但车往左修时改成相反数
COLOR_KP_Y                   = 0.015               # 前后闭环比例系数，决定 error_y 的主要修正力度
COLOR_KD_Y                   = 0.25              # 前后闭环微分系数，主要用于抑制前后方向过冲
COLOR_Y_OUTPUT_SIGN          = -1.0              # 前后输出方向修正符号，目标偏上/偏下时若车前后修正方向反了就改成相反数
COLOR_DEAD_ZONE              = 3                 # 最终允许的中心误差，单位像素，你当前要求尽量收敛到 3 像素内
COLOR_CENTER_EXIT_ZONE       = 5                 # 从“已居中”重新退出的阈值，单位像素，应略大于上面的死区
COLOR_MICRO_ADJUST_ZONE      = 8                 # 进入微调脉冲区的误差阈值，单位像素
COLOR_MICRO_ADJUST_ON_CYCLES = 1                 # 微调区内连续允许输出的周期数
COLOR_MICRO_ADJUST_OFF_CYCLES = 2                # 微调区内暂停输出的周期数
COLOR_INPUT_FILTER_ALPHA     = 0.25              # 视觉中心点输入滤波系数，越大反应越快，越小越平滑
COLOR_COMMAND_FILTER_ALPHA   = 0.18              # 视觉输出速度滤波系数，越大跟手，越小柔和
COLOR_MAX_TRACKING_SPEED     = 10.0              # 视觉闭环输出的平移速度上限，对应底盘 speed 标度 0~100
COLOR_COMMAND_RAMP_STEP      = 1.5               # 单次控制更新允许的最大速度变化量，用于限制突变
MIN_TRANSLATE_SPEED          = 10.0              # 平移向量的最小幅值，需要车在小误差时也能推得动时再调大

DRIVE_MAX_DUTY               = 8500              # 最终三路电机 duty 总上限
DRIVE_TRANSLATE_BIAS_DUTY    = 5200              # 平移分量的起转补偿 duty，用于越过电机平移死区
DRIVE_ROTATE_BIAS_DUTY       = 0                 # 旋转分量的起转补偿 duty，当前按要求关闭 yaw 补偿
DRIVE_MIN_DUTY_START         = 0                 # 速度向量换算 duty 时的最小非零整体缩放阈值，当前保持 0
DRIVE_ACCELERATION           = 0                 # 电机 duty 的斜坡加速度，0 表示立即到位

LOOP_DELAY_MS                = 1                 # 主循环的基础延时，单位毫秒
GC_INTERVAL_MS               = 1000              # 主循环中执行垃圾回收的时间间隔，单位毫秒
STATUS_PRINT_EVERY           = 25                # 每隔多少个循环打印一次调试状态
ENABLE_SERIAL_LOG            = True              # 是否打开串口调试打印
LED_PIN                      = "C4"              # 状态灯引脚
LED_ACTIVE_LEVEL             = 0                 # 状态灯有效电平
STOP_SWITCH_PIN              = "D9"              # 停止开关引脚
STOP_SWITCH_PULL             = Pin.PULL_UP_47K   # 停止开关上拉配置
STOP_SWITCH_ENABLED          = False             # 是否启用停止开关

START_KEY_PIN                = "C8"              # IMU 校准完成后等待启动确认的按键引脚
START_KEY_PULL               = Pin.PULL_UP_47K   # 启动确认按键上拉配置
START_KEY_ACTIVE_LEVEL       = 0                 # 启动确认按键有效电平
START_KEY_DEBOUNCE_MS        = 30                # 启动确认按键消抖时间，单位毫秒

MOTOR_SELF_TEST_ON_START     = False             # 是否在启动后执行一次电机自检
MOTOR_SELF_TEST_SPEED        = 60.0              # 电机自检使用的速度
MOTOR_SELF_TEST_MS           = 300               # 电机自检持续时间，单位毫秒


# 无线调参命令仅在 YAW_FOLLOW_LEADER_ENABLED = False 时生效
# HELP                                  # 查看无线调参帮助
# LIST                                  # 列出全部当前可调参数及当前值
# GET PID_KP                            # 查询单个参数当前值
# PID_KP=0.25                           # 设置 yaw 比例参数
# COLOR_KP_X=0.12                       # 设置横向闭环比例参数
# YAW_FIXED_TARGET_DEG=0                # 设置固定闭环 yaw 目标角
# ENABLE_SERIAL_LOG=1                   # 布尔量支持 1/0、TRUE/FALSE、ON/OFF、YES/NO
#
# 当前无线可调参数名称如下
# YAW_FIXED_TARGET_DEG
# PID_KP
# PID_KI
# PID_KD
# PID_INTEGRAL_LIMIT
# YAW_DEADBAND_DEG
# MAX_ROTATE_SPEED
# MIN_COMMAND_SPEED
# ROTATE_SIGN
# COLOR_MAX_TRACKING_DUTY
# COLOR_MIN_TRACKING_DUTY
# COLOR_TARGET_HOLD_MS
# COLOR_KP_X
# COLOR_KD_X
# COLOR_X_OUTPUT_SIGN
# COLOR_KP_Y
# COLOR_KD_Y
# COLOR_Y_OUTPUT_SIGN
# COLOR_DEAD_ZONE
# COLOR_CENTER_EXIT_ZONE
# COLOR_MICRO_ADJUST_ZONE
# COLOR_MICRO_ADJUST_ON_CYCLES
# COLOR_MICRO_ADJUST_OFF_CYCLES
# COLOR_INPUT_FILTER_ALPHA
# COLOR_COMMAND_FILTER_ALPHA
# COLOR_MAX_TRACKING_SPEED
# COLOR_COMMAND_RAMP_STEP
# MIN_TRANSLATE_SPEED
# DRIVE_MAX_DUTY
# DRIVE_TRANSLATE_BIAS_DUTY
# DRIVE_ROTATE_BIAS_DUTY
# DRIVE_MIN_DUTY_START
# DRIVE_ACCELERATION
# STATUS_PRINT_EVERY
# ENABLE_SERIAL_LOG

DEFAULT_CONFIG = {
    "self_id": SELF_ID,
    "leader_id": LEADER_ID,
    "uart_id": YAW_UART_ID,
    "baudrate": YAW_BAUDRATE,
    "rx_text_buf_limit": RX_TEXT_BUF_LIMIT,
    "timeout_ms": LEADER_TIMEOUT_MS,
    "vision_baudrate": VISION_BAUDRATE,
    "yaw_follow_leader_enabled": YAW_FOLLOW_LEADER_ENABLED,
    "yaw_fixed_target_deg": YAW_FIXED_TARGET_DEG,
    "tuning_rx_text_buf_limit": TUNING_RX_TEXT_BUF_LIMIT,
    "tuning_reply_enabled": TUNING_REPLY_ENABLED,
    "imu_capture_div": IMU_CAPTURE_DIV,
    "imu_tick_ms": IMU_TICK_MS,
    "imu_acc_range_g": IMU_ACC_RANGE_G,
    "imu_gyro_range_dps": IMU_GYRO_RANGE_DPS,
    "imu_acc_alpha": IMU_ACC_ALPHA,
    "imu_comp_alpha": IMU_COMP_ALPHA,
    "imu_gyro_cali_n": IMU_GYRO_CALI_N,
    "imu_calibrate_on_start": IMU_CALIBRATE_ON_START,
    "imu_calibration_settle_ms": IMU_CALIBRATION_SETTLE_MS,
    "pid_kp": PID_KP,
    "pid_ki": PID_KI,
    "pid_kd": PID_KD,
    "pid_integral_limit": PID_INTEGRAL_LIMIT,
    "yaw_deadband_deg": YAW_DEADBAND_DEG,
    "max_rotate_speed": MAX_ROTATE_SPEED,
    "min_command_speed": MIN_COMMAND_SPEED,
    "rotate_sign": ROTATE_SIGN,
    "color_base_duty": COLOR_BASE_DUTY,
    "color_max_tracking_duty": COLOR_MAX_TRACKING_DUTY,
    "color_min_tracking_duty": COLOR_MIN_TRACKING_DUTY,
    "color_target_hold_ms": COLOR_TARGET_HOLD_MS,
    "color_kp_x": COLOR_KP_X,
    "color_kd_x": COLOR_KD_X,
    "color_x_output_sign": COLOR_X_OUTPUT_SIGN,
    "color_kp_y": COLOR_KP_Y,
    "color_kd_y": COLOR_KD_Y,
    "color_y_output_sign": COLOR_Y_OUTPUT_SIGN,
    "color_dead_zone": COLOR_DEAD_ZONE,
    "color_center_exit_zone": COLOR_CENTER_EXIT_ZONE,
    "color_micro_adjust_zone": COLOR_MICRO_ADJUST_ZONE,
    "color_micro_adjust_on_cycles": COLOR_MICRO_ADJUST_ON_CYCLES,
    "color_micro_adjust_off_cycles": COLOR_MICRO_ADJUST_OFF_CYCLES,
    "color_input_filter_alpha": COLOR_INPUT_FILTER_ALPHA,
    "color_command_filter_alpha": COLOR_COMMAND_FILTER_ALPHA,
    "color_max_tracking_speed": COLOR_MAX_TRACKING_SPEED,
    "color_command_ramp_step": COLOR_COMMAND_RAMP_STEP,
    "min_translate_speed": MIN_TRANSLATE_SPEED,
    "drive_max_duty": DRIVE_MAX_DUTY,
    "drive_translate_bias_duty": DRIVE_TRANSLATE_BIAS_DUTY,
    "drive_rotate_bias_duty": DRIVE_ROTATE_BIAS_DUTY,
    "drive_min_duty_start": DRIVE_MIN_DUTY_START,
    "drive_acceleration": DRIVE_ACCELERATION,
    "loop_delay_ms": LOOP_DELAY_MS,
    "gc_interval_ms": GC_INTERVAL_MS,
    "status_print_every": STATUS_PRINT_EVERY,
    "enable_serial_log": ENABLE_SERIAL_LOG,
    "led_pin": LED_PIN,
    "led_active_level": LED_ACTIVE_LEVEL,
    "stop_switch_pin": STOP_SWITCH_PIN,
    "stop_switch_pull": STOP_SWITCH_PULL,
    "stop_switch_enabled": STOP_SWITCH_ENABLED,
    "start_key_pin": START_KEY_PIN,
    "start_key_pull": START_KEY_PULL,
    "start_key_active_level": START_KEY_ACTIVE_LEVEL,
    "start_key_debounce_ms": START_KEY_DEBOUNCE_MS,
    "motor_self_test_on_start": MOTOR_SELF_TEST_ON_START,
    "motor_self_test_speed": MOTOR_SELF_TEST_SPEED,
    "motor_self_test_ms": MOTOR_SELF_TEST_MS,
    "led_pattern_period_ms": 4000,
    "led_blink_on_ms": 700,
    "led_blink_gap_ms": 500,
    "yaw_valid_period_ms": 4000,
    "yaw_valid_on_ms": 3600,
    "error_strobe_period_ms": 160,
    "error_strobe_on_ms": 80,
    "setup_error_period_ms": 3000,
    "setup_error_on_ms": 1500,
}


TUNABLE_SPECS = {
    "YAW_FIXED_TARGET_DEG": ("yaw_fixed_target_deg", float),
    "PID_KP": ("pid_kp", float),
    "PID_KI": ("pid_ki", float),
    "PID_KD": ("pid_kd", float),
    "PID_INTEGRAL_LIMIT": ("pid_integral_limit", float),
    "YAW_DEADBAND_DEG": ("yaw_deadband_deg", float),
    "MAX_ROTATE_SPEED": ("max_rotate_speed", float),
    "MIN_COMMAND_SPEED": ("min_command_speed", float),
    "ROTATE_SIGN": ("rotate_sign", float),
    "COLOR_MAX_TRACKING_DUTY": ("color_max_tracking_duty", int),
    "COLOR_MIN_TRACKING_DUTY": ("color_min_tracking_duty", int),
    "COLOR_TARGET_HOLD_MS": ("color_target_hold_ms", int),
    "COLOR_KP_X": ("color_kp_x", float),
    "COLOR_KD_X": ("color_kd_x", float),
    "COLOR_X_OUTPUT_SIGN": ("color_x_output_sign", float),
    "COLOR_KP_Y": ("color_kp_y", float),
    "COLOR_KD_Y": ("color_kd_y", float),
    "COLOR_Y_OUTPUT_SIGN": ("color_y_output_sign", float),
    "COLOR_DEAD_ZONE": ("color_dead_zone", int),
    "COLOR_CENTER_EXIT_ZONE": ("color_center_exit_zone", int),
    "COLOR_MICRO_ADJUST_ZONE": ("color_micro_adjust_zone", int),
    "COLOR_MICRO_ADJUST_ON_CYCLES": ("color_micro_adjust_on_cycles", int),
    "COLOR_MICRO_ADJUST_OFF_CYCLES": ("color_micro_adjust_off_cycles", int),
    "COLOR_INPUT_FILTER_ALPHA": ("color_input_filter_alpha", float),
    "COLOR_COMMAND_FILTER_ALPHA": ("color_command_filter_alpha", float),
    "COLOR_MAX_TRACKING_SPEED": ("color_max_tracking_speed", float),
    "COLOR_COMMAND_RAMP_STEP": ("color_command_ramp_step", float),
    "MIN_TRANSLATE_SPEED": ("min_translate_speed", float),
    "DRIVE_MAX_DUTY": ("drive_max_duty", int),
    "DRIVE_TRANSLATE_BIAS_DUTY": ("drive_translate_bias_duty", int),
    "DRIVE_ROTATE_BIAS_DUTY": ("drive_rotate_bias_duty", int),
    "DRIVE_MIN_DUTY_START": ("drive_min_duty_start", int),
    "DRIVE_ACCELERATION": ("drive_acceleration", int),
    "STATUS_PRINT_EVERY": ("status_print_every", int),
    "ENABLE_SERIAL_LOG": ("enable_serial_log", bool),
}


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


def _apply_min_vector_speed(vx, vy, min_speed):
    """在不改变方向的前提下，抬高平移向量幅值。"""
    if min_speed <= 0:
        return vx, vy

    mag = math.sqrt(vx * vx + vy * vy)
    if mag <= 1e-6 or mag >= min_speed:
        return vx, vy

    scale = min_speed / mag
    return vx * scale, vy * scale


def _is_start_key_pressed(start_key, config):
    return start_key.value() == config["start_key_active_level"]


def _wait_start_key(start_key, config):
    """等待 C8 启动确认。"""
    if config["enable_serial_log"]:
        print(
            "CarB IMU calibration done, waiting for {} confirmation.".format(
                config["start_key_pin"],
            )
        )

    while True:
        MotorControl.stop(0)
        if _is_start_key_pressed(start_key, config):
            time.sleep_ms(config["start_key_debounce_ms"])
            if _is_start_key_pressed(start_key, config):
                if config["enable_serial_log"]:
                    print("{} pressed, CarB follow mode start.".format(config["start_key_pin"]))
                return

        time.sleep_ms(config["loop_delay_ms"])
        gc.collect()


def _parse_bool(text):
    text_upper = text.strip().upper()
    if text_upper in ("1", "TRUE", "ON", "YES"):
        return True
    if text_upper in ("0", "FALSE", "OFF", "NO"):
        return False
    raise ValueError("invalid bool")


class WirelessTuningSession:
    """固定 0 度模式下，通过无线串口接收电脑发来的文本调参命令。"""

    def __init__(self, config, apply_callback, read_callback):
        self.config = config
        self.apply_callback = apply_callback
        self.read_callback = read_callback
        self.radio = WirelessUsartCars(
            uart_id=config["uart_id"],
            baudrate=config["baudrate"],
            self_id=config["self_id"],
        )
        self.rx_text_buf = ""

    def start(self):
        self.radio.clear_rx()
        if self.config["tuning_reply_enabled"]:
            self.radio.send_line("TUNE READY")
            self.radio.send_line("CMD: NAME=VALUE | GET NAME | LIST | HELP")

    def update(self):
        raw = self.radio.read()
        if raw is None:
            return 0

        try:
            text = raw.decode("utf-8")
        except Exception:
            self._reply("ERR decode")
            return 0

        self.rx_text_buf += text
        limit = max(64, int(self.config["tuning_rx_text_buf_limit"]))
        if len(self.rx_text_buf) > limit:
            self.rx_text_buf = self.rx_text_buf[-limit:]

        handled = 0
        while "\n" in self.rx_text_buf:
            line, self.rx_text_buf = self.rx_text_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            handled += 1
            self._handle_line(line)
        return handled

    def _reply(self, text):
        if self.config["tuning_reply_enabled"]:
            self.radio.send_line(text)

    def _handle_line(self, line):
        line_upper = line.upper()
        if line_upper == "HELP":
            self._reply("CMD: NAME=VALUE | GET NAME | LIST")
            return

        if line_upper == "LIST":
            for name in sorted(TUNABLE_SPECS.keys()):
                self._reply("{}={}".format(name, self.read_callback(name)))
            return

        if line_upper.startswith("GET "):
            name = line[4:].strip().upper()
            if name not in TUNABLE_SPECS:
                self._reply("ERR unknown {}".format(name))
                return
            self._reply("{}={}".format(name, self.read_callback(name)))
            return

        if "=" in line:
            name, value_text = line.split("=", 1)
            ok, message = self.apply_callback(name.strip().upper(), value_text.strip())
            self._reply(message if message else ("OK" if ok else "ERR"))
            return

        self._reply("ERR unsupported")


class FollowLeaderYawColorTrace:
    """组合控制器：视觉给平移，yaw 模式按开关决定。"""

    def __init__(self, config):
        self.config = config
        self.receiver = None
        self.tuner = None
        if self.config["yaw_follow_leader_enabled"]:
            self.receiver = LeaderYawReceiver(config)
        else:
            self.tuner = WirelessTuningSession(
                config,
                self._apply_tunable_text,
                self._read_tunable_text,
            )

        self.imu = ImuSensorVertical(
            capture_div=config["imu_capture_div"],
            tick_ms=config["imu_tick_ms"],
            acc_range_g=config["imu_acc_range_g"],
            gyro_range_dps=config["imu_gyro_range_dps"],
            acc_alpha=config["imu_acc_alpha"],
            comp_alpha=config["imu_comp_alpha"],
            gyro_cali_n=config["imu_gyro_cali_n"],
        )
        self.pid = YawPID(
            config["pid_kp"],
            config["pid_ki"],
            config["pid_kd"],
            config["max_rotate_speed"],
            config["pid_integral_limit"],
        )
        self.vision = MCXVisionUsart(baudrate=config["vision_baudrate"])
        self.color = ColorTraceController(
            self.vision,
            base_duty=config["color_base_duty"],
            max_tracking_duty=config["color_max_tracking_duty"],
            min_tracking_duty=config["color_min_tracking_duty"],
            target_hold_ms=config["color_target_hold_ms"],
            kp_x=config["color_kp_x"],
            kd_x=config["color_kd_x"],
            x_output_sign=config["color_x_output_sign"],
            kp_y=config["color_kp_y"],
            kd_y=config["color_kd_y"],
            y_output_sign=config["color_y_output_sign"],
            dead_zone=config["color_dead_zone"],
            center_exit_zone=config["color_center_exit_zone"],
            input_filter_alpha=config["color_input_filter_alpha"],
            command_filter_alpha=config["color_command_filter_alpha"],
            max_tracking_speed=config["color_max_tracking_speed"],
            command_ramp_step=config["color_command_ramp_step"],
        )
        self._apply_runtime_config()
        self.leader_yaw = None
        self.last_loop_ms = time.ticks_ms()
        self.micro_adjust_cycle = 0
        self.running = False

    def _apply_runtime_config(self):
        """把 config 中可调参数重新同步到控制对象。"""
        self.pid.kp = float(self.config["pid_kp"])
        self.pid.ki = float(self.config["pid_ki"])
        self.pid.kd = float(self.config["pid_kd"])
        self.pid.output_limit = abs(float(self.config["max_rotate_speed"]))
        self.pid.integral_limit = abs(float(self.config["pid_integral_limit"]))

        self.color.base_duty = int(self.config["color_base_duty"])
        self.color.dead_zone = max(0, int(self.config["color_dead_zone"]))
        self.color.center_exit_zone = max(self.color.dead_zone, int(self.config["color_center_exit_zone"]))
        self.color.x_output_sign = -1.0 if float(self.config["color_x_output_sign"]) < 0 else 1.0
        self.color.y_output_sign = -1.0 if float(self.config["color_y_output_sign"]) < 0 else 1.0
        self.color.input_filter_alpha = min(1.0, max(0.05, float(self.config["color_input_filter_alpha"])))
        self.color.command_filter_alpha = min(1.0, max(0.05, float(self.config["color_command_filter_alpha"])))
        self.color.command_ramp_step = max(0.0, float(self.config["color_command_ramp_step"]))
        self.color.max_tracking_duty = min(MotorControl.MAX_DUTY, int(self.config["color_max_tracking_duty"]))
        self.color.min_tracking_duty = min(self.color.max_tracking_duty, max(0, int(self.config["color_min_tracking_duty"])))
        self.color.target_hold_ms = max(0, int(self.config["color_target_hold_ms"]))
        self.color.max_tracking_speed = min(
            int(self.color.max_tracking_duty * MotorControl.MAX_SPEED / MotorControl.MAX_DUTY),
            max(1.0, float(self.config["color_max_tracking_speed"])),
        )
        self.color.x_pid.kp = float(self.config["color_kp_x"])
        self.color.x_pid.kd = float(self.config["color_kd_x"])
        self.color.y_pid.kp = float(self.config["color_kp_y"])
        self.color.y_pid.kd = float(self.config["color_kd_y"])
        self.color.x_pid.output_min = -self.color.max_tracking_speed
        self.color.x_pid.output_max = self.color.max_tracking_speed
        self.color.y_pid.output_min = -self.color.max_tracking_speed
        self.color.y_pid.output_max = self.color.max_tracking_speed

    def _parse_tunable_value(self, name, value_text):
        config_key, value_type = TUNABLE_SPECS[name]
        if value_type is bool:
            return config_key, _parse_bool(value_text)
        return config_key, value_type(value_text)

    def _apply_tunable_text(self, name, value_text):
        if name not in TUNABLE_SPECS:
            return False, "ERR unknown {}".format(name)

        try:
            config_key, parsed_value = self._parse_tunable_value(name, value_text)
        except Exception:
            return False, "ERR invalid {}".format(name)

        self.config[config_key] = parsed_value
        self._apply_runtime_config()
        if name in ("PID_KP", "PID_KI", "PID_KD", "PID_INTEGRAL_LIMIT", "MAX_ROTATE_SPEED"):
            self.pid.reset()
        if name.startswith("COLOR_"):
            self.color.x_pid.reset()
            self.color.y_pid.reset()
        return True, "OK {}={}".format(name, self._read_tunable_text(name))

    def _read_tunable_text(self, name):
        config_key, _ = TUNABLE_SPECS[name]
        return self.config[config_key]

    def start(self):
        MotorControl.stop(0)
        if self.receiver is not None:
            self.receiver.start()
        if self.tuner is not None:
            self.tuner.start()
        self.vision.clear_rx()
        self.color.start()
        start_key = Pin(
            self.config["start_key_pin"],
            Pin.IN,
            pull=self.config["start_key_pull"],
        )

        print("CarB IMU initializing...")
        self.imu.init()
        if self.config["imu_calibrate_on_start"]:
            print("CarB IMU calibrating, keep vehicle still.")
            time.sleep_ms(self.config["imu_calibration_settle_ms"])
            self.imu.calibrate()
            print("CarB IMU calibration finished.")
        else:
            print("CarB IMU calibration skipped.")

        _wait_start_key(start_key, self.config)
        self.pid.reset()
        self.last_loop_ms = time.ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.pid.reset()
        self.color.stop()
        self.micro_adjust_cycle = 0
        MotorControl.stop(0)
        self.imu.stop()

    def _should_output_micro_adjust(self, color_state):
        """在目标附近采用脉冲式微调，减轻持续震荡。"""
        if color_state is None:
            self.micro_adjust_cycle = 0
            return True

        if color_state.get("in_center", False):
            self.micro_adjust_cycle = 0
            return False

        err_x = abs(color_state.get("err_x", 0))
        err_y = abs(color_state.get("err_y", 0))
        micro_adjust_zone = max(0, int(self.config["color_micro_adjust_zone"]))

        if max(err_x, err_y) > micro_adjust_zone:
            self.micro_adjust_cycle = 0
            return True

        on_cycles = max(1, int(self.config["color_micro_adjust_on_cycles"]))
        off_cycles = max(0, int(self.config["color_micro_adjust_off_cycles"]))
        period = on_cycles + off_cycles
        if period <= 1:
            return True

        allow_output = self.micro_adjust_cycle < on_cycles
        self.micro_adjust_cycle = (self.micro_adjust_cycle + 1) % period
        return allow_output

    def _update_yaw_target(self):
        """按模式更新当前 yaw 目标。"""
        if self.config["yaw_follow_leader_enabled"]:
            rx_data = self.receiver.update()
            if rx_data is not None:
                self.leader_yaw = rx_data["leader_yaw"]
            return self.leader_yaw is not None, "leader"

        if self.tuner is not None:
            self.tuner.update()
        self.leader_yaw = float(self.config["yaw_fixed_target_deg"])
        return True, "fixed"

    def _compute_yaw_rotate_speed(self, imu_data, dt_s):
        """持续闭环当前 yaw 目标。"""
        if self.leader_yaw is None:
            self.pid.reset()
            return False, None, 0.0

        yaw_error = continuous_yaw_error(self.leader_yaw, imu_data["yaw"])
        if abs(yaw_error) <= self.config["yaw_deadband_deg"]:
            self.pid.reset()
            return True, yaw_error, 0.0

        raw_speed = self.pid.compute(yaw_error, dt_s)
        rotate_speed = raw_speed * self.config["rotate_sign"]
        if 0.0 < abs(rotate_speed) < self.config["min_command_speed"]:
            rotate_speed = self.config["min_command_speed"] if rotate_speed > 0.0 else -self.config["min_command_speed"]
        return True, yaw_error, rotate_speed

    def _build_result(self, has_yaw_target, yaw_mode, imu_data, yaw_error, rotate_speed, color_state, motor_duty, vx, vy):
        if color_state is None:
            color_state = {}

        return {
            "has_leader_yaw": has_yaw_target,
            "yaw_mode": yaw_mode,
            "leader_yaw": self.leader_yaw,
            "follower_yaw": imu_data["yaw"],
            "yaw_error": yaw_error,
            "rotate_speed": rotate_speed,
            "has_target": color_state.get("has_target", False),
            "target_locked": color_state.get("target_locked", False),
            "in_center": color_state.get("in_center", False),
            "err_x": color_state.get("err_x", 0),
            "err_y": color_state.get("err_y", 0),
            "micro_adjust_active": color_state.get("micro_adjust_active", False),
            "micro_adjust_output_enabled": color_state.get("micro_adjust_output_enabled", True),
            "vx": vx,
            "vy": vy,
            "motor_duty": motor_duty,
        }

    def update(self):
        if not self.running:
            return None

        color_state = self.color.update_tracking_command()
        imu_data = self.imu.update()
        if imu_data is None:
            return None

        now_ms = time.ticks_ms()
        dt_s = time.ticks_diff(now_ms, self.last_loop_ms) / 1000.0
        self.last_loop_ms = now_ms

        has_yaw_target, yaw_mode = self._update_yaw_target()
        has_yaw_target, yaw_error, rotate_speed = self._compute_yaw_rotate_speed(
            imu_data,
            dt_s,
        )

        if not has_yaw_target:
            MotorControl.stop(0)
            return self._build_result(
                False,
                "leader",
                imu_data,
                None,
                0.0,
                color_state,
                (0, 0, 0),
                0.0,
                0.0,
            )

        vx = 0.0
        vy = 0.0
        if (
            color_state is not None
            and color_state["target_locked"]
            and color_state["moving"]
            and not color_state["in_center"]
        ):
            micro_adjust_active = max(abs(color_state["err_x"]), abs(color_state["err_y"])) <= self.config["color_micro_adjust_zone"]
            micro_adjust_output_enabled = self._should_output_micro_adjust(color_state)
            color_state["micro_adjust_active"] = micro_adjust_active
            color_state["micro_adjust_output_enabled"] = micro_adjust_output_enabled

            vx = color_state["vx"]
            vy = color_state["vy"]
            vx, vy = _apply_min_vector_speed(
                vx,
                vy,
                self.config["min_translate_speed"],
            )
            if not micro_adjust_output_enabled:
                vx = 0.0
                vy = 0.0
        elif color_state is not None:
            color_state["micro_adjust_active"] = False
            color_state["micro_adjust_output_enabled"] = True
            self.micro_adjust_cycle = 0

        if abs(vx) <= 1e-6 and abs(vy) <= 1e-6 and abs(rotate_speed) <= 1e-6:
            MotorControl.stop(0)
            motor_duty = (0, 0, 0)
        else:
            motor_duty = MotorControl.drive_vector(
                vx,
                vy,
                omega=rotate_speed,
                acceleration=self.config["drive_acceleration"],
                max_duty=self.config["drive_max_duty"],
                min_duty_start=self.config["drive_min_duty_start"],
                translate_duty_bias_start=self.config["drive_translate_bias_duty"],
                rotate_duty_bias_start=self.config["drive_rotate_bias_duty"],
            )

        return self._build_result(
            True,
            yaw_mode,
            imu_data,
            yaw_error,
            rotate_speed,
            color_state,
            motor_duty,
            vx,
            vy,
        )


def run_follow_leader_yaw_color_trace(config=None):
    if config is None:
        config = build_config()

    inactive_level = 0 if config["led_active_level"] else 1
    led_pin = Pin(config["led_pin"], Pin.OUT, value=inactive_level)
    status_led = C4StatusLed(led_pin, config)
    stop_switch = Pin(
        config["stop_switch_pin"],
        Pin.IN,
        pull=config["stop_switch_pull"],
    )
    stop_state = stop_switch.value()

    controller = FollowLeaderYawColorTrace(config)

    try:
        controller.start()
        if config["motor_self_test_on_start"]:
            MotorControl.rotate(config["motor_self_test_speed"])
            time.sleep_ms(config["motor_self_test_ms"])
            MotorControl.stop(0)
    except Exception:
        MotorControl.stop(0)
        while True:
            status_led.update_setup_error()
            time.sleep_ms(config["loop_delay_ms"])

    if config["enable_serial_log"]:
        print("=== CarB color trace + yaw control ===")
        print("yaw mode={}".format("leader" if config["yaw_follow_leader_enabled"] else "fixed"))
        print("yaw input uart: UART{} {}bps".format(config["uart_id"], config["baudrate"]))
        print(
            "vision input: UART5 {}bps, frame {}x{}".format(
                config["vision_baudrate"],
                MCXVisionUsart.VIEW_WIDTH,
                MCXVisionUsart.VIEW_HEIGHT,
            )
        )
        print(
            "yaw PID: kp={}, ki={}, kd={}, deadband=+/-{}".format(
                config["pid_kp"],
                config["pid_ki"],
                config["pid_kd"],
                config["yaw_deadband_deg"],
            )
        )
        if not config["yaw_follow_leader_enabled"]:
            print("wireless tuning: NAME=VALUE | GET NAME | LIST")

    tick = 0
    last_gc_ms = time.ticks_ms()

    try:
        while True:
            now_ms = time.ticks_ms()
            result = controller.update()

            if config["yaw_follow_leader_enabled"]:
                status = STATUS_YAW
                if controller.leader_yaw is None or controller.receiver.is_timeout(now_ms):
                    status = STATUS_NO_SIGNAL
            else:
                status = STATUS_YAW
            status_led.update(status, now_ms)

            if result is not None:
                tick += 1
                if config["enable_serial_log"] and tick % config["status_print_every"] == 0:
                    print(
                        (
                            "yaw_mode={} leader_yaw={:>7} follower_yaw={:>7.2f} yaw_error={:>7} "
                            "rotate={:>6.1f} target_locked={} in_center={} err_x={:>5} err_y={:>5} "
                            "micro_adjust_active={} micro_adjust_output_enabled={} "
                            "vx={:>6.1f} vy={:>6.1f} motor_duty={}"
                        ).format(
                            result["yaw_mode"],
                            "--" if result["leader_yaw"] is None else "{:.2f}".format(result["leader_yaw"]),
                            result["follower_yaw"],
                            "--" if result["yaw_error"] is None else "{:.2f}".format(result["yaw_error"]),
                            result["rotate_speed"],
                            result["target_locked"],
                            result["in_center"],
                            result["err_x"],
                            result["err_y"],
                            result["micro_adjust_active"],
                            result["micro_adjust_output_enabled"],
                            result["vx"],
                            result["vy"],
                            result["motor_duty"],
                        )
                    )

            if config["stop_switch_enabled"] and stop_switch.value() != stop_state:
                break

            if time.ticks_diff(now_ms, last_gc_ms) >= config["gc_interval_ms"]:
                gc.collect()
                last_gc_ms = now_ms

            time.sleep_ms(config["loop_delay_ms"])
    except Exception:
        MotorControl.stop(0)
        while True:
            status_led.update_error()
            time.sleep_ms(config["loop_delay_ms"])
    finally:
        controller.stop()
        status_led.off()


if __name__ == "__main__":
    run_follow_leader_yaw_color_trace()
