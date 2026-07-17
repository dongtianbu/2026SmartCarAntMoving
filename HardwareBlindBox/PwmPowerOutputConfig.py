"""HardwareBlindBox PWM 功率输出配置。

所有需要人工调试的变量都集中在本文件开头，现场调试时优先修改这里。
"""

from machine import Pin	


# ---------------------------------------------------------------------------
# 人工调试区
# 所有需要人工调试的变量都集中在这里，修改后可直接烧录验证。
# ---------------------------------------------------------------------------

BATTERY_INPUT_VOLTAGE = 12.12
# 当前电池输入电压，单位 V。
# 用于把目标输出电压换算成 PWM 占空比。

PWM_OUTPUT_FREQUENCY_HZ = 15000
# 三路互补 PWM 的输出频率，单位 Hz。

TARGET_OUTPUT_VOLTAGE = 4.86
# 默认目标输出电压，单位 V。
# 正负号分别表示两种极性方向。

OUTPUT_ENABLED_ON_START = True
# 上电后是否立即开始输出 PWM。

OUTPUT_MODE = 0
# 输出模式：
# 0 = 固定电压输出
# 1 = 三角波扫压
# 2 = 高低电平脉冲切换

OUTPUT_POLARITY_SIGN = 1
# 极性修正：
# 1 = 保持默认方向
# -1 = 三路 PWM 整体反向

MAX_ALLOWED_OUTPUT_VOLTAGE = 11.0
# 软件允许输出的最大绝对电压，单位 V。

MIN_EFFECTIVE_DUTY = 0
# 非零输出时强制拉到的最小有效 duty，范围 0~10000。

RAMP_STEP_DUTY = 150
# duty 变化时每一步最多增加或减少的量。
# 设为 0 表示直接跳变到目标值。

RAMP_STEP_MS = 10
# 每一步 duty 变化后的等待时间，单位 ms。

STARTUP_DELAY_MS = 300
# 上电后开始输出前的等待时间，单位 ms。

SWEEP_MIN_OUTPUT_VOLTAGE = 1.0
# 扫压模式下的最小输出电压，单位 V。

SWEEP_MAX_OUTPUT_VOLTAGE = 6.0
# 扫压模式下的最大输出电压，单位 V。

SWEEP_STEP_VOLTAGE = 0.2
# 扫压模式下每一步变化的电压，单位 V。

SWEEP_STEP_INTERVAL_MS = 120
# 扫压模式下每一步停留时间，单位 ms。

PULSE_HIGH_OUTPUT_VOLTAGE = 5.0
# 脉冲模式高电平对应的目标输出电压，单位 V。

PULSE_LOW_OUTPUT_VOLTAGE = 0.0
# 脉冲模式低电平对应的目标输出电压，单位 V。

PULSE_HIGH_HOLD_MS = 300
# 脉冲模式高电平保持时间，单位 ms。

PULSE_LOW_HOLD_MS = 300
# 脉冲模式低电平保持时间，单位 ms。

ENABLE_SERIAL_LOG = True
# 是否通过串口打印调试信息。

LOG_INTERVAL_MS = 500
# 状态日志打印间隔，单位 ms。

SHOW_STARTUP_INFO = True
# 启动时是否打印一遍主要参数。

STATUS_LED_PIN = "C4"
# 状态灯引脚。

STATUS_LED_ACTIVE_LEVEL = 0
# 状态灯有效电平。

HEARTBEAT_INTERVAL_MS = 200
# 输出关闭时状态灯慢闪周期，单位 ms。

STOP_SWITCH_ENABLED = True
# 是否启用 D9 停止开关。

STOP_SWITCH_PIN = "D9"
# 停止开关引脚。

STOP_SWITCH_PULL = Pin.PULL_UP_47K
# 停止开关上下拉方式。

TOGGLE_KEY_ENABLED = True
# 是否启用 C15 按键切换输出开关。

TOGGLE_KEY_PIN = "C15"
# 切换输出开关的按键引脚。

TOGGLE_KEY_PULL = Pin.PULL_UP_47K
# 切换按键上下拉方式。

TOGGLE_KEY_DEBOUNCE_MS = 250
# 切换按键消抖时间，单位 ms。

CHANNEL_1_INVERT = False
# C28/C29 是否反相。

CHANNEL_2_INVERT = False
# D4/D5 是否反相。

CHANNEL_3_INVERT = False
# C30/C31 是否反相。

ZERO_OUTPUT_ON_EXIT = True
# 程序退出或异常时，是否自动把三路输出拉回 0。

PWM_MAX_DUTY = 10000
# seekfree.MOTOR_CONTROLLER 的 duty 满量程。


def build_config():
    """把文件顶部的调试变量整理成统一配置字典。"""
    return {
        "battery_input_voltage": BATTERY_INPUT_VOLTAGE,
        "pwm_output_frequency_hz": PWM_OUTPUT_FREQUENCY_HZ,
        "target_output_voltage": TARGET_OUTPUT_VOLTAGE,
        "output_enabled_on_start": OUTPUT_ENABLED_ON_START,
        "output_mode": OUTPUT_MODE,
        "output_polarity_sign": OUTPUT_POLARITY_SIGN,
        "max_allowed_output_voltage": MAX_ALLOWED_OUTPUT_VOLTAGE,
        "min_effective_duty": MIN_EFFECTIVE_DUTY,
        "ramp_step_duty": RAMP_STEP_DUTY,
        "ramp_step_ms": RAMP_STEP_MS,
        "startup_delay_ms": STARTUP_DELAY_MS,
        "sweep_min_output_voltage": SWEEP_MIN_OUTPUT_VOLTAGE,
        "sweep_max_output_voltage": SWEEP_MAX_OUTPUT_VOLTAGE,
        "sweep_step_voltage": SWEEP_STEP_VOLTAGE,
        "sweep_step_interval_ms": SWEEP_STEP_INTERVAL_MS,
        "pulse_high_output_voltage": PULSE_HIGH_OUTPUT_VOLTAGE,
        "pulse_low_output_voltage": PULSE_LOW_OUTPUT_VOLTAGE,
        "pulse_high_hold_ms": PULSE_HIGH_HOLD_MS,
        "pulse_low_hold_ms": PULSE_LOW_HOLD_MS,
        "enable_serial_log": ENABLE_SERIAL_LOG,
        "log_interval_ms": LOG_INTERVAL_MS,
        "show_startup_info": SHOW_STARTUP_INFO,
        "status_led_pin": STATUS_LED_PIN,
        "status_led_active_level": STATUS_LED_ACTIVE_LEVEL,
        "heartbeat_interval_ms": HEARTBEAT_INTERVAL_MS,
        "stop_switch_enabled": STOP_SWITCH_ENABLED,
        "stop_switch_pin": STOP_SWITCH_PIN,
        "stop_switch_pull": STOP_SWITCH_PULL,
        "toggle_key_enabled": TOGGLE_KEY_ENABLED,
        "toggle_key_pin": TOGGLE_KEY_PIN,
        "toggle_key_pull": TOGGLE_KEY_PULL,
        "toggle_key_debounce_ms": TOGGLE_KEY_DEBOUNCE_MS,
        "channel_1_invert": CHANNEL_1_INVERT,
        "channel_2_invert": CHANNEL_2_INVERT,
        "channel_3_invert": CHANNEL_3_INVERT,
        "zero_output_on_exit": ZERO_OUTPUT_ON_EXIT,
        "pwm_max_duty": PWM_MAX_DUTY,
    }
