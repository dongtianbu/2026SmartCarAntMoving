"""CarB 组合控制器。

职责分工：
1. 视觉追踪负责计算平移速度 `vx`、`vy`。
2. 无线串口负责提供主车 `yaw`，或在调参模式下接收电脑下发的参数。
3. 本车 IMU 与 yaw PID 负责计算旋转速度 `omega`。
4. 最终通过 `MotorControl.drive_vector()` 将平移与旋转混合输出到底盘。
"""

from machine import Pin
import gc
import math
import time

_COLOR_TRACE_CONTROLLER_CLASS = None
_IMU_SENSOR_CLASS = None
_LEADER_YAW_SUPPORT = None
_MOTOR_CONTROL_MODULE = None
_VISION_USART_CLASS = None
_WIRELESS_USART_CLASS = None


def _load_imu_sensor_class():
    """延迟导入 IMU 驱动，降低从车冷启动时的导入内存峰值。"""
    global _IMU_SENSOR_CLASS
    if _IMU_SENSOR_CLASS is None:
        gc.collect()
        from IMUVertical import ImuSensorVertical as imu_sensor_class
        _IMU_SENSOR_CLASS = imu_sensor_class
        gc.collect()
    return _IMU_SENSOR_CLASS


def _load_color_trace_controller_class():
    """延迟导入视觉闭环控制器，降低冷启动导入峰值。"""
    global _COLOR_TRACE_CONTROLLER_CLASS
    if _COLOR_TRACE_CONTROLLER_CLASS is None:
        gc.collect()
        from ColorTrace import ColorTraceController as color_trace_controller_class
        _COLOR_TRACE_CONTROLLER_CLASS = color_trace_controller_class
        gc.collect()
    return _COLOR_TRACE_CONTROLLER_CLASS


def _load_leader_yaw_support():
    """延迟导入状态灯和主车 yaw 接收器，固定 yaw 模式下不再提前占内存。"""
    global _LEADER_YAW_SUPPORT
    if _LEADER_YAW_SUPPORT is None:
        gc.collect()
        from LeaderYawReceiveTest import (
            C4StatusLed,
            LeaderYawReceiver,
            STATUS_NO_SIGNAL,
            STATUS_YAW,
        )
        _LEADER_YAW_SUPPORT = (
            C4StatusLed,
            LeaderYawReceiver,
            STATUS_NO_SIGNAL,
            STATUS_YAW,
        )
        gc.collect()
    return _LEADER_YAW_SUPPORT


def _load_motor_control_module():
    """延迟导入底盘控制模块，避免导入阶段立刻创建电机对象。"""
    global _MOTOR_CONTROL_MODULE
    if _MOTOR_CONTROL_MODULE is None:
        gc.collect()
        import MotorControl as motor_control_module
        _MOTOR_CONTROL_MODULE = motor_control_module
        gc.collect()
    return _MOTOR_CONTROL_MODULE


def _load_vision_usart_class():
    """延迟导入视觉串口类。"""
    global _VISION_USART_CLASS
    if _VISION_USART_CLASS is None:
        gc.collect()
        from MCXVisionUsart import MCXVisionUsart as vision_usart_class
        _VISION_USART_CLASS = vision_usart_class
        gc.collect()
    return _VISION_USART_CLASS


def _load_wireless_usart_class():
    """延迟导入无线串口类。"""
    global _WIRELESS_USART_CLASS
    if _WIRELESS_USART_CLASS is None:
        gc.collect()
        from WirelessUsartCars import WirelessUsartCars as wireless_usart_class
        _WIRELESS_USART_CLASS = wireless_usart_class
        gc.collect()
    return _WIRELESS_USART_CLASS


# ---------------------------------------------------------------------------
# 人工调参
# 把现场需要频繁修改的参数集中放在文件顶部，便于统一查看和调整。
# ---------------------------------------------------------------------------

SELF_ID                      = 2                 # 无线通信中本车设备 ID
LEADER_ID                    = 1                 # 无线通信中主车设备 ID
YAW_UART_ID                  = 2                 # yaw 跟随和无线调参共用的串口号
YAW_BAUDRATE                 = 115200            # yaw 跟随和无线调参共用的串口波特率
RX_TEXT_BUF_LIMIT            = 128               # 主车 yaw 文本接收缓冲区上限
LEADER_TIMEOUT_MS            = 1500              # 主车 yaw 超时阈值，单位 ms
VISION_BAUDRATE              = 115200            # 视觉模块串口波特率，当前接在 UART5

YAW_FOLLOW_LEADER_ENABLED    = True             # True 时跟随主车 yaw，False 时改为固定目标角并开启电脑调试
YAW_FIXED_TARGET_DEG         = 0.0               # 固定 yaw 模式下的目标角度
TUNING_RX_TEXT_BUF_LIMIT     = 512               # 无线调参文本缓冲区上限
TUNING_REPLY_ENABLED         = False              # 是否回发调参执行结果
TUNING_PROTOCOL_PREFIX       = "AIPID|"         # 上位机与 CarB 调参协议固定前缀；只有带此前缀的命令和回包才视为有效协议内容
TUNING_PROTOCOL_FRAME_HEAD   = "<AIPID_BEGIN>"  # 调参协议包头；用于在杂乱串口流中定位一帧协议消息的开始
TUNING_PROTOCOL_FRAME_TAIL   = "<AIPID_END>"    # 调参协议包尾；用于在杂乱串口流中定位一帧协议消息的结束
TUNING_PROTOCOL_ACCEPT_LEGACY = False           # False 时仅接受带协议前缀的新协议；如需临时兼容旧版上位机可改为 True
TUNING_FRAME_BUFFER_LIMIT    = 512             # 从车侧协议解析缓冲区上限，防止长时间噪声输入把缓存撑满
CAMERA_LOOK_RIGHT_ENABLED    = True              # True 表示摄像头朝车右侧，需要做坐标映射
AI_TUNING_CONTROL_ENABLED    = False              # True 时允许 AI_STOP / AI_START / AI_DEVIATE 等命令
AI_CONTROL_PAUSED_ON_START   = False              # True 时上电后先暂停 AI 控制，等待 AI_START
AI_METRIC_ENABLED            = False              # True 时周期输出 AI_METRIC 数据
AI_METRIC_WIRELESS_ENABLED   = False              # True 时同时从无线串口回发 AI_METRIC 数据
AI_METRIC_PREFIX             = "AI_METRIC"       # AI 指标行前缀
AI_PERTURB_MAX_TRANSLATE_SPEED = 7.0            # AI_PERTURB 允许的最大平移速度
AI_PERTURB_MAX_ROTATE_SPEED  = 64.0              # AI_PERTURB 允许的最大旋转速度
AI_PERTURB_MAX_DURATION_MS   = 600              # AI_PERTURB 最大持续时间，单位 ms
TUNING_COMMAND_REPLY_REPEAT_COUNT = 1           # 普通命令回包重复发送次数；最简单模式下保持单次回包
TUNING_BANNER_REPEAT_COUNT   = 1                 # 启动横幅重复发送次数；最简单模式下保持单次发送
TUNING_METRIC_REPLY_REPEAT_COUNT = 1             # AI_METRIC 无线回包重复发送次数；最简单模式下保持单次发送

IMU_CAPTURE_DIV              = 1                 # IMU 采样分频
IMU_TICK_MS                  = 10                # IMU 更新周期，单位 ms
IMU_ACC_RANGE_G              = 8                 # 加速度计量程，单位 g
IMU_GYRO_RANGE_DPS           = 2000              # 陀螺仪量程，单位度每秒
IMU_ACC_ALPHA                = 0.20              # 加速度一阶滤波系数
IMU_COMP_ALPHA               = 0.98              # 互补滤波系数，越接近 1 越依赖陀螺仪
IMU_GYRO_CALI_N              = 300               # 陀螺仪校准采样次数
IMU_CALIBRATE_ON_START       = True              # 是否在启动时执行 IMU 校准
IMU_CALIBRATION_SETTLE_MS    = 500               # 校准前静置等待时间，单位 ms
IMU_YAW_SIGN                 = -1                 # 1 保持当前 yaw 方向；-1 反向，可切换顺时针旋转时角度增减关系

PID_KP                       = 0.15                # yaw 比例系数，决定误差对应的基础旋转力度
PID_KI                       = 0.07              # yaw 积分系数，用于消除稳态误差
PID_KD                       = 0.01                 # yaw 微分系数，用于抑制超调和抖动
PID_INTEGRAL_LIMIT           = 60.0                # yaw 积分限幅，防止积分累积过大
YAW_DEADBAND_DEG             = 0.5               # yaw 死区，误差绝对值小于该角度时视为对正
MAX_ROTATE_SPEED             = 64.0                # yaw 旋转速度上限，对应底盘 speed 标度 0~100
MIN_COMMAND_SPEED            = 50.0                # yaw 非零输出时的最小旋转指令，用于克服静摩擦
ROTATE_SIGN                  = 1.0              # yaw 旋转方向修正符号，方向反了就改成相反数

COLOR_BASE_DUTY              = 6000              # 视觉控制保留的基础 duty 参数
COLOR_MAX_TRACKING_DUTY      = 6500              # 视觉追踪允许的最大 duty
COLOR_MIN_TRACKING_DUTY      = 5500              # 视觉追踪允许的最小非零 duty
COLOR_TARGET_HOLD_MS         = 100               # 丢目标后继续沿用上次结果的保持时间，单位 ms
COLOR_KP_X                   = 0.033               # 横向误差 error_x 的比例系数
COLOR_KD_X                   = 0.25                # 横向误差的微分系数
COLOR_X_OUTPUT_SIGN          = 1.0              # 横向输出方向修正符号，方向反了可改为 -1.0
COLOR_KP_Y                   = 0.033               # 前后误差 error_y 的比例系数
COLOR_KD_Y                   = 0.25                # 前后误差的微分系数
COLOR_Y_OUTPUT_SIGN          = -1.0              # 前后输出方向修正符号
COLOR_DEAD_ZONE              = 3.0                 # 目标居中死区，单位像素
COLOR_CENTER_EXIT_ZONE       = 5.0                 # 从“已居中”退出的阈值，通常略大于死区
COLOR_MICRO_ADJUST_ZONE      = 8                 # 进入微调脉冲区的误差阈值
COLOR_MICRO_ADJUST_ON_CYCLES = 1                 # 微调脉冲开启的循环数
COLOR_MICRO_ADJUST_OFF_CYCLES = 2                # 微调脉冲关闭的循环数
COLOR_INPUT_FILTER_ALPHA     = 0.25              # 视觉输入滤波系数
COLOR_COMMAND_FILTER_ALPHA   = 0.4               # 速度指令滤波系数
COLOR_MAX_TRACKING_SPEED     = 5.5                 # 视觉追踪输出的最大平移速度，对应底盘 speed 标度 0~100
COLOR_COMMAND_RAMP_STEP      = 1.1               # 单次控制更新允许的最大速度变化量
MIN_TRANSLATE_SPEED          = 2.0                 # 非零平移时的最小速度幅值

DRIVE_MAX_DUTY               = 7100.0              # 单路电机 duty 绝对值上限
DRIVE_TRANSLATE_BIAS_DUTY    = 5200.0              # 平移分量的单路起转补偿 duty
DRIVE_ROTATE_BIAS_DUTY       = 5200                 # 旋转分量的单路起转补偿 duty
DRIVE_MIN_DUTY_START         = 0                 # 输出非零 duty 时的最低起步阈值，0 表示关闭
DRIVE_ACCELERATION           = 0                 # duty 斜坡步长，0 表示直接输出目标 duty

LOOP_DELAY_MS                = 1                 # 主循环基础延时，单位 ms；MicroPython 的 sleep_ms 只能接收整数
GC_INTERVAL_MS               = 1000              # 垃圾回收间隔，单位 ms
STATUS_PRINT_EVERY           = 25                # 每隔多少个循环打印一次调试状态
ENABLE_SERIAL_LOG            = True              # 是否启用串口日志
LED_PIN                      = "C4"              # 状态灯引脚
LED_ACTIVE_LEVEL             = 0                 # 状态灯有效电平
STOP_SWITCH_PIN              = "D9"              # 停止开关引脚
STOP_SWITCH_PULL             = Pin.PULL_UP_47K   # 停止开关上拉配置
STOP_SWITCH_ENABLED          = False             # 是否启用停止开关

MOTOR_SELF_TEST_ON_START     = False             # 是否在启动后执行一次电机自检
MOTOR_SELF_TEST_SPEED        = 60.0              # 电机自检速度
MOTOR_SELF_TEST_MS           = 300               # 电机自检持续时间，单位 ms


def clamp(value, lower, upper):
    """把数值限制在给定上下限之间。"""
    return max(lower, min(upper, value))


def continuous_yaw_error(target_deg, current_deg):
    """返回连续 yaw 误差，不做 180/-180 折返。"""
    return target_deg - current_deg


class YawPID:
    """轻量级 yaw PID 控制器。

    这里直接内联到 FollowLeaderYawColorTrace 中，避免为了复用两个小工具而整包导入
    FollowLeaderYawPID.py，降低主程序导入阶段的内存占用。
    """

    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.last_error = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None

    def compute(self, error, dt_s):
        if dt_s <= 0.0:
            dt_s = IMU_TICK_MS / 1000.0

        self.integral += error * dt_s
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )

        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt_s
        self.last_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return clamp(output, -self.output_limit, self.output_limit)


# 无线调参命令仅在 YAW_FOLLOW_LEADER_ENABLED = False 时生效
# AIPID|HELP                            # 查看帮助；新版上位机与手工调试都应带此前缀
# AIPID|LIST                            # 列出全部当前可调参数
# AIPID|GET PID_KP                      # 查询单个参数当前值
# AIPID|PID_KP=0.18                     # 设置 yaw 比例系数
# AIPID|COLOR_KP_X=0.033                # 设置横向比例系数
# AIPID|YAW_FIXED_TARGET_DEG=0          # 设置固定 yaw 目标
# ENABLE_SERIAL_LOG=1                   # 布尔量支持 1/0、TRUE/FALSE、ON/OFF、YES/NO
# CAMERA_LOOK_RIGHT_ENABLED=1           # 开启“摄像头朝车右侧”模式，0 或 OFF 时恢复朝前
# AIPID|AI_STOP                         # 暂停 AI 控制
# AIPID|AI_START                        # 恢复 AI 控制
# AIPID|AI_DEVIATE 12 1200              # 先偏离 12 度，持续 1200ms，再自动回到固定 yaw 目标
# AIPID|AI_PERTURB 45 20 45 1200        # 按 45 度方向平移，速度 20，同时以 45 旋转 1200ms
# AIPID|AI_STATUS                       # 查询当前 AI 控制状态
#
# 可调参数列表
# CAMERA_LOOK_RIGHT_ENABLED
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

# ---------------------------------------------------------------------------
# ai_round2_2 override block
# source:
# dist/logs/sessions/2026-07-14-11-23-44/trial_history.json
# keep all tuning variables near the file top for manual adjustment.
# ---------------------------------------------------------------------------


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
    "tuning_protocol_prefix": TUNING_PROTOCOL_PREFIX,
    "tuning_protocol_frame_head": TUNING_PROTOCOL_FRAME_HEAD,
    "tuning_protocol_frame_tail": TUNING_PROTOCOL_FRAME_TAIL,
    "tuning_protocol_accept_legacy": TUNING_PROTOCOL_ACCEPT_LEGACY,
    "tuning_frame_buffer_limit": TUNING_FRAME_BUFFER_LIMIT,
    "camera_look_right_enabled": CAMERA_LOOK_RIGHT_ENABLED,
    "ai_tuning_control_enabled": AI_TUNING_CONTROL_ENABLED,
    "ai_control_paused_on_start": AI_CONTROL_PAUSED_ON_START,
    "ai_metric_enabled": AI_METRIC_ENABLED,
    "ai_metric_wireless_enabled": AI_METRIC_WIRELESS_ENABLED,
    "ai_metric_prefix": AI_METRIC_PREFIX,
    "ai_perturb_max_translate_speed": AI_PERTURB_MAX_TRANSLATE_SPEED,
    "ai_perturb_max_rotate_speed": AI_PERTURB_MAX_ROTATE_SPEED,
    "ai_perturb_max_duration_ms": AI_PERTURB_MAX_DURATION_MS,
    "tuning_command_reply_repeat_count": TUNING_COMMAND_REPLY_REPEAT_COUNT,
    "tuning_banner_repeat_count": TUNING_BANNER_REPEAT_COUNT,
    "tuning_metric_reply_repeat_count": TUNING_METRIC_REPLY_REPEAT_COUNT,
    "imu_capture_div": IMU_CAPTURE_DIV,
    "imu_tick_ms": IMU_TICK_MS,
    "imu_acc_range_g": IMU_ACC_RANGE_G,
    "imu_gyro_range_dps": IMU_GYRO_RANGE_DPS,
    "imu_acc_alpha": IMU_ACC_ALPHA,
    "imu_comp_alpha": IMU_COMP_ALPHA,
    "imu_gyro_cali_n": IMU_GYRO_CALI_N,
    "imu_calibrate_on_start": IMU_CALIBRATE_ON_START,
    "imu_calibration_settle_ms": IMU_CALIBRATION_SETTLE_MS,
    "imu_yaw_sign": IMU_YAW_SIGN,
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
    "CAMERA_LOOK_RIGHT_ENABLED": ("camera_look_right_enabled", bool),
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
    # 这些配置最终会传给 time.sleep_ms() 或时间差比较，必须保持为非负整数。
    for key in (
        "loop_delay_ms",
        "gc_interval_ms",
        "imu_calibration_settle_ms",
        "motor_self_test_ms",
        "ai_perturb_max_duration_ms",
    ):
        config[key] = max(0, int(config[key]))
    return config


def _apply_min_vector_speed(vx, vy, min_speed):
    """在不改变方向的前提下，把平移向量幅值抬到最小速度阈值。"""
    if min_speed <= 0:
        return vx, vy

    mag = math.sqrt(vx * vx + vy * vy)
    if mag <= 1e-6 or mag >= min_speed:
        return vx, vy

    scale = min_speed / mag
    return vx * scale, vy * scale


def _map_translate_vector_for_camera(vx, vy, config):
    """根据摄像头朝向，把视觉平移指令映射到底盘坐标系。"""
    if not config["camera_look_right_enabled"]:
        return vx, vy

    return vy, -vx


def _parse_bool(text):
    text_upper = text.strip().upper()
    if text_upper in ("1", "TRUE", "ON", "YES"):
        return True
    if text_upper in ("0", "FALSE", "OFF", "NO"):
        return False
    raise ValueError("invalid bool")


def _metric_value(value):
    """把状态值转成上位机脚本更容易解析的短文本。"""
    if value is None:
        return "NA"
    if isinstance(value, float):
        return "{:.4f}".format(value)
    return str(value)

class WirelessTuningSession:
    """通过无线串口收发文本调参命令。"""

    def __init__(self, config, apply_callback, read_callback, command_callback=None):
        self.config = config
        self.apply_callback = apply_callback
        self.read_callback = read_callback
        self.command_callback = command_callback
        wireless_usart_class = _load_wireless_usart_class()
        self.radio = wireless_usart_class(
            uart_id=config["uart_id"],
            baudrate=config["baudrate"],
            self_id=config["self_id"],
        )
        self.rx_text_buf = ""

    def _normalize_text(self, text):
        """压缩协议文本中的多余空白，降低无线串口粘包时的解析难度。"""
        return " ".join(str(text).strip().split())

    def _wrap_protocol_text(self, text):
        """给发回上位机的调参文本统一包装成带包头包尾的完整协议帧。"""
        payload = self._normalize_text(text)
        if not payload:
            return ""
        prefix = str(self.config["tuning_protocol_prefix"])
        if payload.startswith(prefix):
            protocol_payload = payload
        else:
            protocol_payload = "{}{}".format(prefix, payload)
        return "{}{}{}".format(
            self.config["tuning_protocol_frame_head"],
            protocol_payload,
            self.config["tuning_protocol_frame_tail"],
        )

    def _extract_protocol_payload(self, text):
        """从接收到的串口文本里提取 AIPID 协议负载，优先识别完整包头包尾帧。"""
        raw_text = str(text)
        if not raw_text:
            return ""

        head = str(self.config["tuning_protocol_frame_head"])
        tail = str(self.config["tuning_protocol_frame_tail"])
        frame_start = raw_text.find(head)
        if frame_start >= 0:
            frame_end = raw_text.find(tail, frame_start + len(head))
            if frame_end >= 0:
                raw_text = raw_text[frame_start + len(head):frame_end]

        normalized = self._normalize_text(raw_text)
        if not normalized:
            return ""

        prefix = str(self.config["tuning_protocol_prefix"])
        index = normalized.find(prefix)
        if index >= 0:
            return self._normalize_text(normalized[index + len(prefix):])

        if bool(self.config["tuning_protocol_accept_legacy"]):
            return normalized
        return ""

    def _extract_protocol_frames(self):
        """从无线串口接收缓冲区中提取所有完整协议帧，剩余半帧继续保留等待下次拼接。"""
        payloads = []
        head = str(self.config["tuning_protocol_frame_head"])
        tail = str(self.config["tuning_protocol_frame_tail"])

        while True:
            start = self.rx_text_buf.find(head)
            if start < 0:
                break

            end = self.rx_text_buf.find(tail, start + len(head))
            if end < 0:
                if start > 0:
                    self.rx_text_buf = self.rx_text_buf[start:]
                break

            frame_text = self.rx_text_buf[start:end + len(tail)]
            payload = self._extract_protocol_payload(frame_text)
            if payload:
                payloads.append(payload)
            self.rx_text_buf = self.rx_text_buf[end + len(tail):]

        buffer_limit = max(128, int(self.config["tuning_frame_buffer_limit"]))
        if len(self.rx_text_buf) > buffer_limit:
            if head in self.rx_text_buf:
                self.rx_text_buf = self.rx_text_buf[-buffer_limit:]
            else:
                self.rx_text_buf = self.rx_text_buf[-len(head):]
        return payloads

    def start(self):
        self.radio.clear_rx()
        if self.config["tuning_reply_enabled"]:
            self._reply("TUNE READY", repeat_count=self.config["tuning_banner_repeat_count"])
            self._reply(
                "CMD: FRAME={}...{} ; SEND {}NAME=VALUE | {}GET NAME | {}LIST | {}AI_STOP | {}AI_START | {}AI_DEVIATE deg ms | {}AI_PERTURB angle speed rotate ms | {}AI_STATUS".format(
                    self.config["tuning_protocol_frame_head"],
                    self.config["tuning_protocol_frame_tail"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                )
                ,
                repeat_count=self.config["tuning_banner_repeat_count"],
            )

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
        limit = max(
            int(self.config["tuning_frame_buffer_limit"]),
            int(self.config["tuning_rx_text_buf_limit"]),
        )
        if len(self.rx_text_buf) > limit:
            self.rx_text_buf = self.rx_text_buf[-limit:]

        handled = 0
        for line in self._extract_protocol_frames():
            handled += 1
            self._handle_line(line)
        return handled

    def _reply(self, text, repeat_count=None):
        if self.config["tuning_reply_enabled"]:
            protocol_text = self._wrap_protocol_text(text)
            if protocol_text:
                if repeat_count is None:
                    repeat_count = self.config["tuning_command_reply_repeat_count"]
                for _ in range(max(1, int(repeat_count))):
                    self.radio.send_line(protocol_text)

    def _handle_line(self, line):
        line = self._extract_protocol_payload(line)
        if not line:
            return

        line_upper = line.upper()
        if line_upper == "HELP":
            self._reply(
                "CMD: FRAME={}...{} ; SEND {}NAME=VALUE | {}GET NAME | {}LIST | {}AI_STOP | {}AI_START | {}AI_DEVIATE deg ms | {}AI_PERTURB angle speed rotate ms | {}AI_STATUS".format(
                    self.config["tuning_protocol_frame_head"],
                    self.config["tuning_protocol_frame_tail"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                    self.config["tuning_protocol_prefix"],
                )
            )
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

        if self.command_callback is not None:
            handled, message = self.command_callback(line)
            if handled:
                if message:
                    self._reply(message)
                return

        self._reply("ERR unsupported")


class FollowLeaderYawColorTrace:
    """CarB 组合控制器：视觉给平移，yaw PID 给旋转。"""

    def __init__(self, config):
        self.config = config
        self.motor_control = _load_motor_control_module()
        self.receiver = None
        self.tuner = None
        if self.config["yaw_follow_leader_enabled"]:
            _, leader_yaw_receiver_class, _, _ = _load_leader_yaw_support()
            self.receiver = leader_yaw_receiver_class(config)
            gc.collect()
        else:
            self.tuner = WirelessTuningSession(
                config,
                self._apply_tunable_text,
                self._read_tunable_text,
                self._handle_ai_tuning_command,
            )
            gc.collect()

        imu_sensor_class = _load_imu_sensor_class()
        self.imu = imu_sensor_class(
            capture_div=config["imu_capture_div"],
            tick_ms=config["imu_tick_ms"],
            acc_range_g=config["imu_acc_range_g"],
            gyro_range_dps=config["imu_gyro_range_dps"],
            acc_alpha=config["imu_acc_alpha"],
            comp_alpha=config["imu_comp_alpha"],
            gyro_cali_n=config["imu_gyro_cali_n"],
            yaw_sign=config["imu_yaw_sign"],
        )
        gc.collect()
        self.pid = YawPID(
            config["pid_kp"],
            config["pid_ki"],
            config["pid_kd"],
            config["max_rotate_speed"],
            config["pid_integral_limit"],
        )
        vision_usart_class = _load_vision_usart_class()
        color_trace_controller_class = _load_color_trace_controller_class()
        self.vision = vision_usart_class(baudrate=config["vision_baudrate"])
        self.color = color_trace_controller_class(
            self.vision,
            motor_control=self.motor_control,
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
        gc.collect()
        self._apply_runtime_config()
        self.leader_yaw = None
        self.last_loop_ms = time.ticks_ms()
        self.micro_adjust_cycle = 0
        self.ai_control_paused = bool(config["ai_control_paused_on_start"])
        self.ai_deviation_offset_deg = 0.0
        self.ai_deviation_until_ms = None
        self.ai_manual_until_ms = None
        self.ai_manual_vx = 0.0
        self.ai_manual_vy = 0.0
        self.ai_manual_rotate_speed = 0.0
        self.ai_pause_after_manual = False
        self.running = False
    def _apply_runtime_config(self):
        """把 config 中的可调参数同步到运行时控制对象。"""
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
        self.color.max_tracking_duty = min(self.motor_control.MAX_DUTY, int(self.config["color_max_tracking_duty"]))
        self.color.min_tracking_duty = min(self.color.max_tracking_duty, max(0, int(self.config["color_min_tracking_duty"])))
        self.color.target_hold_ms = max(0, int(self.config["color_target_hold_ms"]))
        self.color.max_tracking_speed = min(
            int(self.color.max_tracking_duty * self.motor_control.MAX_SPEED / self.motor_control.MAX_DUTY),
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

    def _reset_control_after_ai_command(self):
        """AI 控制状态切换后，清空 PID 和微调残留状态。"""
        self.pid.reset()
        self.color.x_pid.reset()
        self.color.y_pid.reset()
        self.micro_adjust_cycle = 0

    def _handle_ai_tuning_command(self, line):
        """处理 AI_STOP、AI_START、AI_DEVIATE、AI_PERTURB 等控制命令。"""
        if not self.config["ai_tuning_control_enabled"]:
            return True, "ERR AI_CONTROL_DISABLED"

        parts = line.strip().split()
        if not parts:
            return False, ""

        command = parts[0].upper()
        if command == "AI_STOP":
            self.ai_control_paused = True
            self.ai_deviation_offset_deg = 0.0
            self.ai_deviation_until_ms = None
            self.ai_manual_until_ms = None
            self.ai_manual_vx = 0.0
            self.ai_manual_vy = 0.0
            self.ai_manual_rotate_speed = 0.0
            self.ai_pause_after_manual = False
            self._reset_control_after_ai_command()
            self.motor_control.stop(0)
            return True, "OK AI_STOP"

        if command == "AI_START":
            self.ai_control_paused = False
            self.ai_deviation_offset_deg = 0.0
            self.ai_deviation_until_ms = None
            self.ai_manual_until_ms = None
            self.ai_manual_vx = 0.0
            self.ai_manual_vy = 0.0
            self.ai_manual_rotate_speed = 0.0
            self.ai_pause_after_manual = False
            self._reset_control_after_ai_command()
            return True, "OK AI_START"

        if command == "AI_DEVIATE":
            if len(parts) < 3:
                return True, "ERR AI_DEVIATE_USAGE"
            try:
                offset_deg = float(parts[1])
                duration_ms = max(0, int(float(parts[2])))
            except Exception:
                return True, "ERR AI_DEVIATE_VALUE"

            self.ai_control_paused = False
            self.ai_deviation_offset_deg = offset_deg
            self.ai_deviation_until_ms = time.ticks_ms() + duration_ms
            self._reset_control_after_ai_command()
            return True, "OK AI_DEVIATE offset={} duration_ms={}".format(offset_deg, duration_ms)

        if command == "AI_PERTURB":
            if len(parts) < 5:
                return True, "ERR AI_PERTURB_USAGE"
            try:
                angle_deg = float(parts[1])
                translate_speed = float(parts[2])
                rotate_speed = float(parts[3])
                duration_ms = int(float(parts[4]))
            except Exception:
                return True, "ERR AI_PERTURB_VALUE"

            translate_limit = abs(float(self.config["ai_perturb_max_translate_speed"]))
            rotate_limit = abs(float(self.config["ai_perturb_max_rotate_speed"]))
            duration_limit = max(0, int(self.config["ai_perturb_max_duration_ms"]))
            translate_speed = max(-translate_limit, min(translate_limit, translate_speed))
            rotate_speed = max(-rotate_limit, min(rotate_limit, rotate_speed))
            duration_ms = max(0, min(duration_limit, duration_ms))

            angle_rad = math.radians(angle_deg)
            self.ai_manual_vx = translate_speed * math.cos(angle_rad)
            self.ai_manual_vy = translate_speed * math.sin(angle_rad)
            self.ai_manual_rotate_speed = rotate_speed
            self.ai_manual_until_ms = time.ticks_ms() + duration_ms
            self.ai_pause_after_manual = True
            self.ai_control_paused = False
            self.ai_deviation_offset_deg = 0.0
            self.ai_deviation_until_ms = None
            self._reset_control_after_ai_command()
            return True, "OK AI_PERTURB angle={} translate={} rotate={} duration_ms={}".format(
                angle_deg,
                translate_speed,
                rotate_speed,
                duration_ms,
            )

        if command == "AI_STATUS":
            return True, self._build_ai_status_reply()

        return False, ""

    def _build_ai_status_reply(self):
        """构造当前 AI 状态字符串。"""
        remaining_ms = 0
        if self.ai_deviation_until_ms is not None:
            remaining_ms = max(0, time.ticks_diff(self.ai_deviation_until_ms, time.ticks_ms()))
        manual_remaining_ms = 0
        if self.ai_manual_until_ms is not None:
            manual_remaining_ms = max(0, time.ticks_diff(self.ai_manual_until_ms, time.ticks_ms()))
        return "OK AI_STATUS paused={} deviation_offset={} remaining_ms={} manual_remaining_ms={}".format(
            self.ai_control_paused,
            self.ai_deviation_offset_deg,
            remaining_ms,
            manual_remaining_ms,
        )

    def start(self):
        self.motor_control.stop(0)
        if self.receiver is not None:
            self.receiver.start()
        if self.tuner is not None:
            self.tuner.start()
        self.vision.clear_rx()
        self.color.start()

        print("CarB IMU initializing...")
        self.imu.init()
        if self.config["imu_calibrate_on_start"]:
            print("CarB IMU calibrating, keep vehicle still.")
            time.sleep_ms(self.config["imu_calibration_settle_ms"])
            self.imu.calibrate()
            print("CarB IMU calibration finished.")
        else:
            print("CarB IMU calibration skipped.")

        if self.config["enable_serial_log"]:
            print("CarB startup ready, follow mode auto-starting.")
        self.pid.reset()
        self.last_loop_ms = time.ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.pid.reset()
        self.color.stop()
        self.micro_adjust_cycle = 0
        self.motor_control.stop(0)
        self.imu.stop()

    def _should_output_micro_adjust(self, color_state):
        """目标接近中心时按脉冲节奏输出微调，降低持续抖动。"""
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
        """按当前模式更新 yaw 目标。"""
        if self.config["yaw_follow_leader_enabled"]:
            rx_data = self.receiver.update()
            if rx_data is not None:
                self.leader_yaw = rx_data["leader_yaw"]
            return self.leader_yaw is not None, "leader"

        if self.tuner is not None:
            self.tuner.update()

        fixed_target = float(self.config["yaw_fixed_target_deg"])
        if self.ai_deviation_until_ms is not None:
            if time.ticks_diff(self.ai_deviation_until_ms, time.ticks_ms()) > 0:
                fixed_target += self.ai_deviation_offset_deg
            else:
                self.ai_deviation_offset_deg = 0.0
                self.ai_deviation_until_ms = None

        self.leader_yaw = fixed_target
        return True, "fixed"

    def _compute_yaw_rotate_speed(self, imu_data, dt_s):
        """根据当前 yaw 误差计算旋转速度。"""
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
            "ai_control_paused": self.ai_control_paused,
            "ai_deviation_active": self.ai_deviation_until_ms is not None,
            "ai_manual_active": self.ai_manual_until_ms is not None,
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

        if self.ai_manual_until_ms is not None:
            if time.ticks_diff(self.ai_manual_until_ms, now_ms) > 0:
                yaw_error = None
                if self.leader_yaw is not None:
                    yaw_error = continuous_yaw_error(self.leader_yaw, imu_data["yaw"])
                motor_duty = self.motor_control.drive_vector(
                    self.ai_manual_vx,
                    self.ai_manual_vy,
                    omega=self.ai_manual_rotate_speed,
                    acceleration=0,
                    max_duty=self.config["drive_max_duty"],
                    min_duty_start=self.config["drive_min_duty_start"],
                    translate_duty_bias_start=self.config["drive_translate_bias_duty"],
                    rotate_duty_bias_start=self.config["drive_rotate_bias_duty"],
                )
                return self._build_result(
                    has_yaw_target,
                    yaw_mode,
                    imu_data,
                    yaw_error,
                    self.ai_manual_rotate_speed,
                    color_state,
                    motor_duty,
                    self.ai_manual_vx,
                    self.ai_manual_vy,
                )

            self.ai_manual_until_ms = None
            self.ai_manual_vx = 0.0
            self.ai_manual_vy = 0.0
            self.ai_manual_rotate_speed = 0.0
            self.motor_control.stop(0)
            if self.ai_pause_after_manual:
                self.ai_control_paused = True
                self.ai_pause_after_manual = False
            self._reset_control_after_ai_command()

        if self.ai_control_paused:
            self.pid.reset()
            self.micro_adjust_cycle = 0
            self.motor_control.stop(0)
            yaw_error = None
            if self.leader_yaw is not None:
                yaw_error = continuous_yaw_error(self.leader_yaw, imu_data["yaw"])
            return self._build_result(
                has_yaw_target,
                yaw_mode,
                imu_data,
                yaw_error,
                0.0,
                color_state,
                (0, 0, 0),
                0.0,
                0.0,
            )

        has_yaw_target, yaw_error, rotate_speed = self._compute_yaw_rotate_speed(
            imu_data,
            dt_s,
        )

        if not has_yaw_target:
            self.motor_control.stop(0)
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
            vx, vy = _map_translate_vector_for_camera(vx, vy, self.config)
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
            self.motor_control.stop(0)
            motor_duty = (0, 0, 0)
        else:
            motor_duty = self.motor_control.drive_vector(
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

    c4_status_led_class, _, status_no_signal, status_yaw = _load_leader_yaw_support()
    vision_usart_class = _load_vision_usart_class()
    inactive_level = 0 if config["led_active_level"] else 1
    led_pin = Pin(config["led_pin"], Pin.OUT, value=inactive_level)
    status_led = c4_status_led_class(led_pin, config)
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
            controller.motor_control.rotate(config["motor_self_test_speed"])
            time.sleep_ms(config["motor_self_test_ms"])
            controller.motor_control.stop(0)
    except Exception:
        controller.motor_control.stop(0)
        while True:
            status_led.update_setup_error()
            time.sleep_ms(config["loop_delay_ms"])

    if config["enable_serial_log"]:
        print("=== CarB color trace + yaw control ===")
        print("yaw mode={}".format("leader" if config["yaw_follow_leader_enabled"] else "fixed"))
        print("camera mode={}".format("right" if config["camera_look_right_enabled"] else "front"))
        print("yaw input uart: UART{} {}bps".format(config["uart_id"], config["baudrate"]))
        print(
            "vision input: UART5 {}bps, frame {}x{}".format(
                config["vision_baudrate"],
                vision_usart_class.VIEW_WIDTH,
                vision_usart_class.VIEW_HEIGHT,
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
            print(
                "wireless tuning frame={}...{} prefix={} ; send {}NAME=VALUE | {}GET NAME | {}LIST | {}AI_STOP".format(
                    config["tuning_protocol_frame_head"],
                    config["tuning_protocol_frame_tail"],
                    config["tuning_protocol_prefix"],
                    config["tuning_protocol_prefix"],
                    config["tuning_protocol_prefix"],
                    config["tuning_protocol_prefix"],
                    config["tuning_protocol_prefix"],
                )
            )

    tick = 0
    last_gc_ms = time.ticks_ms()

    try:
        while True:
            now_ms = time.ticks_ms()
            result = controller.update()

            if config["yaw_follow_leader_enabled"]:
                status = status_yaw
                if controller.leader_yaw is None or controller.receiver.is_timeout(now_ms):
                    status = status_no_signal
            else:
                status = status_yaw
            status_led.update(status, now_ms)

            if result is not None:
                tick += 1
                if tick % config["status_print_every"] == 0:
                    if config["enable_serial_log"]:
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
                    if config["ai_metric_enabled"] and not result["ai_control_paused"]:
                        metric_line = (
                            "{} tick={} yaw_mode={} leader_yaw={} follower_yaw={} yaw_error={} "
                            "rotate={} target_locked={} in_center={} err_x={} err_y={} "
                            "vx={} vy={} paused={} deviation_active={} manual_active={}"
                        ).format(
                            config["ai_metric_prefix"],
                            tick,
                            result["yaw_mode"],
                            _metric_value(result["leader_yaw"]),
                            _metric_value(result["follower_yaw"]),
                            _metric_value(result["yaw_error"]),
                            _metric_value(result["rotate_speed"]),
                            result["target_locked"],
                            result["in_center"],
                            result["err_x"],
                            result["err_y"],
                            _metric_value(result["vx"]),
                            _metric_value(result["vy"]),
                            result["ai_control_paused"],
                            result["ai_deviation_active"],
                            result["ai_manual_active"],
                        )
                        print(metric_line)
                        if (
                            config["ai_metric_wireless_enabled"]
                            and getattr(controller, "tuner", None) is not None
                        ):
                            controller.tuner._reply(
                                metric_line,
                                repeat_count=config["tuning_metric_reply_repeat_count"],
                            )

            if config["stop_switch_enabled"] and stop_switch.value() != stop_state:
                break

            if time.ticks_diff(now_ms, last_gc_ms) >= config["gc_interval_ms"]:
                gc.collect()
                last_gc_ms = now_ms

            time.sleep_ms(config["loop_delay_ms"])
    except Exception:
        controller.motor_control.stop(0)
        while True:
            status_led.update_error()
            time.sleep_ms(config["loop_delay_ms"])
    finally:
        controller.stop()
        status_led.off()


if __name__ == "__main__":
    run_follow_leader_yaw_color_trace()
