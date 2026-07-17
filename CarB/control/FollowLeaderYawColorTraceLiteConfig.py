"""CarB 轻量组合控制的配置模块。

这个文件只保留人工调试参数、配置构造函数和少量配置辅助函数。
这样主程序在上电导入入口模块时，不需要同时解析整份运行控制逻辑。
"""

from machine import Pin


# ---------------------------------------------------------------------------
# 人工调试区
# 所有需要人工反复调试的参数都统一放在文件开头，便于现场快速修改。
# ---------------------------------------------------------------------------

SELF_ID = 2  # 从车自身无线 ID。
LEADER_ID = 1  # 主车无线 ID，仅在跟随主车 yaw 时使用。
YAW_UART_ID = 2  # yaw 跟随与无线调参共用的 UART 编号。
YAW_BAUDRATE = 115200  # yaw/调参串口波特率。
RX_TEXT_BUF_LIMIT = 128  # 主车 yaw 文本接收缓冲上限。
LEADER_TIMEOUT_MS = 1500  # 主车 yaw 数据超时阈值，单位 ms。
VISION_BAUDRATE = 115200  # 视觉模块串口波特率。

YAW_CONTROL_MODE = 1  # yaw 控制模式：0=不开闭环；1=固定角度闭环；2=跟随主车 yaw。
YAW_FOLLOW_LEADER_ENABLED = (YAW_CONTROL_MODE == 2)  # 兼容旧布尔变量名，建议后续统一使用 YAW_CONTROL_MODE。
YAW_FIXED_TARGET_DEG = 0.0  # 当 YAW_CONTROL_MODE=1 时使用的固定目标角度。
CAMERA_LOOK_RIGHT_ENABLED = True  # 相机朝向车体右侧时设为 True，用于平移向量映射。

TUNING_RX_TEXT_BUF_LIMIT = 512  # 无线调参串口文本缓存上限。
TUNING_REPLY_ENABLED = False  # 是否回发调参命令执行结果。
TUNING_PROTOCOL_PREFIX = "AIPID|"  # 调参协议前缀。
TUNING_PROTOCOL_FRAME_HEAD = "<AIPID_BEGIN>"  # 调参协议帧头。
TUNING_PROTOCOL_FRAME_TAIL = "<AIPID_END>"  # 调参协议帧尾。
TUNING_PROTOCOL_ACCEPT_LEGACY = False  # 是否兼容旧版无前缀命令。
TUNING_FRAME_BUFFER_LIMIT = 512  # 调参串口内部帧缓存上限。
TUNING_COMMAND_REPLY_REPEAT_COUNT = 1  # 普通命令回复重复次数。
TUNING_BANNER_REPEAT_COUNT = 1  # 上电握手提示重复次数。
TUNING_METRIC_REPLY_REPEAT_COUNT = 1  # AI_METRIC 无线回包重复次数。

AI_TUNING_CONTROL_ENABLED = False  # 是否允许 AI_STOP、AI_START、AI_PERTURB 等命令。
AI_CONTROL_PAUSED_ON_START = False  # 上电后是否先暂停 AI 闭环，等待上位机发 AI_START。
AI_METRIC_ENABLED = False  # 是否定期打印 AI_METRIC。
AI_METRIC_WIRELESS_ENABLED = False  # 是否通过无线串口回传 AI_METRIC。
AI_METRIC_PREFIX = "AI_METRIC"  # AI 指标行前缀。
AI_PERTURB_MAX_TRANSLATE_SPEED = 7.0  # AI_PERTURB 允许的最大平移速度。这里的 speed 使用 MotorControl 的 0~100 标度，约等于 duty 0~10000，所以 7.0 约等于单路 700 duty 的量级。
AI_PERTURB_MAX_ROTATE_SPEED = 64.0  # AI_PERTURB 允许的最大旋转速度。这里的 speed 使用 MotorControl 的 0~100 标度，64.0 约等于单路 6400 duty 的量级。
AI_PERTURB_MAX_DURATION_MS = 600  # AI_PERTURB 允许的最长持续时间，单位 ms。

IMU_CAPTURE_DIV = 1  # IMU 采样分频。
IMU_TICK_MS = 5  # IMU 更新周期，单位 ms。
IMU_ACC_RANGE_G = 8  # 加速度量程，单位 g。
IMU_GYRO_RANGE_DPS = 2000  # 陀螺仪量程，单位 dps。
IMU_ACC_ALPHA = 0.20  # 加速度一阶滤波系数。
IMU_COMP_ALPHA = 0.98  # 互补滤波系数，越接近 1 越依赖陀螺仪。
IMU_GYRO_CALI_N = 2000  # 陀螺仪校准采样次数。
IMU_CALIBRATE_ON_START = True  # 是否在上电启动时执行 IMU 校准。
IMU_CALIBRATION_SETTLE_MS = 1000  # 校准前静止等待时间，单位 ms。
IMU_YAW_SIGN = -1  # yaw 正负方向修正，方向反了改成 1。

PID_KP = 0.15  # yaw 比例系数。
PID_KI = 0.07  # yaw 积分系数。
PID_KD = 0.01  # yaw 微分系数。
PID_INTEGRAL_LIMIT = 60.0  # yaw 积分限幅。
YAW_DEADBAND_DEG = 0.5  # yaw 死区，单位度。
MAX_ROTATE_SPEED = 64.0  # yaw 输出旋转速度上限。这里的 speed 使用 MotorControl 的 0~100 标度，64.0 约等于单路 6400 duty 的量级。
MIN_COMMAND_SPEED = 50.0  # 非零旋转时的最小输出，避免静摩擦。这里的 speed 使用 MotorControl 的 0~100 标度，50.0 约等于单路 5000 duty 的量级。
ROTATE_SIGN = 1.0  # yaw 方向修正，方向反了改成 -1。

COLOR_BASE_DUTY = 6000  # 视觉平移控制基础 duty。
COLOR_MAX_TRACKING_DUTY = 6500  # 视觉平移最大 duty。
COLOR_MIN_TRACKING_DUTY = 5500  # 视觉平移最小非零 duty。
COLOR_TARGET_HOLD_MS = 100  # 丢目标后沿用上一帧结果的保持时间，单位 ms。
COLOR_KP_X = 0.045  # 横向误差比例系数。
COLOR_KD_X = 0.25  # 横向误差微分系数。
COLOR_X_OUTPUT_SIGN = 1.0  # 横向输出方向修正。
COLOR_KP_Y = 0.045  # 前后误差比例系数。
COLOR_KD_Y = 0.25  # 前后误差微分系数。
COLOR_Y_OUTPUT_SIGN = -1.0  # 前后输出方向修正。
COLOR_DEAD_ZONE = 1.0  # 视觉居中死区，单位像素。
COLOR_CENTER_EXIT_ZONE = 3.0  # 从居中状态退出的阈值，单位像素。
COLOR_MICRO_ADJUST_ZONE = 6  # 进入微调脉冲区的误差阈值。
COLOR_MICRO_ADJUST_ON_CYCLES = 2  # 微调脉冲开启周期数。
COLOR_MICRO_ADJUST_OFF_CYCLES = 1  # 微调脉冲关闭周期数。
COLOR_INPUT_FILTER_ALPHA = 0.25  # 视觉输入滤波系数。
COLOR_COMMAND_FILTER_ALPHA = 0.4  # 平移输出滤波系数。
COLOR_MAX_TRACKING_SPEED = 70  # 视觉跟踪最大平移速度。这里的 speed 使用 MotorControl 的 0~100 标度，5.5 约等于单路 550 duty 的量级。
COLOR_COMMAND_RAMP_STEP = 2  # 每次控制更新允许的最大速度变化。
MIN_TRANSLATE_SPEED = 50  # 非零平移时的最小速度幅值。这里的 speed 使用 MotorControl 的 0~100 标度，2.0 约等于单路 200 duty 的量级。

DRIVE_MAX_DUTY = 7100.0  # 电机输出最大 duty。
DRIVE_TRANSLATE_BIAS_DUTY = 5200.0  # 平移起转补偿 duty。
DRIVE_ROTATE_BIAS_DUTY = 5200  # 旋转起转补偿 duty。
DRIVE_MIN_DUTY_START = 0  # 最小起转 duty。
DRIVE_ACCELERATION = 0  # 电机 duty 斜坡步长，0 表示直接给目标值。

LOOP_DELAY_MS = 1  # 主循环延时，单位 ms。
GC_INTERVAL_MS = 1000  # 主循环主动 GC 间隔，单位 ms。
STATUS_PRINT_EVERY = 25  # 每多少个有效周期打印一次状态。
ENABLE_SERIAL_LOG = True  # 是否启用串口日志。
LED_PIN = "C4"  # 状态灯引脚。
LED_ACTIVE_LEVEL = 0  # 状态灯有效电平。
STOP_SWITCH_PIN = "D9"  # 停止开关引脚。
STOP_SWITCH_PULL = Pin.PULL_UP_47K  # 停止开关上拉模式。
STOP_SWITCH_ENABLED = False  # 是否启用停止开关。

MOTOR_SELF_TEST_ON_START = False  # 启动后是否执行一次电机自检。
MOTOR_SELF_TEST_SPEED = 60.0  # 电机自检旋转速度。这里的 speed 使用 MotorControl 的 0~100 标度，60.0 约等于单路 6000 duty 的量级。
MOTOR_SELF_TEST_MS = 300  # 电机自检持续时间，单位 ms。

LINE_ANGLE_CONTROL_ENABLED = False  # 是否启用“双红外点连线角度”闭环。
LINE_ANGLE_TARGET_MODE = 1  # 线角目标模式：0=使用固定的 LINE_ANGLE_TARGET_DEG；1=锁定上电后首次稳定看到双点时的线角。
LINE_ANGLE_TARGET_DEG = 0.0  # 当 LINE_ANGLE_TARGET_MODE=0 时使用的固定目标角度；0 表示尽量与屏幕水平线平行。
LINE_ANGLE_DEADBAND_DEG = 1.5  # 线角误差死区。
LINE_ANGLE_KP = 0.01  # 线角 PID 比例系数。
LINE_ANGLE_KI = 0.01  # 线角 PID 积分系数。
LINE_ANGLE_KD = 0.05  # 线角 PID 微分系数。
LINE_ANGLE_INTEGRAL_LIMIT = 10.0  # 线角 PID 积分限幅。
LINE_ANGLE_MAX_ROTATE_SPEED = 8.0  # 线角闭环允许附加的最大旋转速度。这里的 speed 使用 MotorControl 的 0~100 标度，5.0 约等于单路 500 duty 的量级。
LINE_ANGLE_MIN_COMMAND_SPEED = 5.0  # 线角闭环最小非零输出。这里的 speed 使用 MotorControl 的 0~100 标度，10.0 约等于单路 1000 duty 的量级；当线角误差一旦触发输出，小于该值会被抬到这个量级。
LINE_ANGLE_OUTPUT_SIGN = -1.0  # 线角输出方向修正。
LINE_ANGLE_FILTER_ALPHA = 0.35  # 线角测量滤波系数。
LINE_ANGLE_MIN_LENGTH_PX = 2.0  # 双点连线长度小于该值时，不参与线角闭环。
LINE_ANGLE_HOLD_MS = 120  # 丢帧后沿用上一帧线角的保持时间，单位 ms。
LINE_ANGLE_BLEND_RATIO = 1.0  # 线角旋转补偿混合比例，建议范围 0~1。


TUNABLE_SPECS = (
    ("CAMERA_LOOK_RIGHT_ENABLED", "camera_look_right_enabled", bool),
    ("YAW_CONTROL_MODE", "yaw_control_mode", int),
    ("YAW_FIXED_TARGET_DEG", "yaw_fixed_target_deg", float),
    ("PID_KP", "pid_kp", float),
    ("PID_KI", "pid_ki", float),
    ("PID_KD", "pid_kd", float),
    ("PID_INTEGRAL_LIMIT", "pid_integral_limit", float),
    ("YAW_DEADBAND_DEG", "yaw_deadband_deg", float),
    ("MAX_ROTATE_SPEED", "max_rotate_speed", float),
    ("MIN_COMMAND_SPEED", "min_command_speed", float),
    ("ROTATE_SIGN", "rotate_sign", float),
    ("LINE_ANGLE_CONTROL_ENABLED", "line_angle_control_enabled", bool),
    ("LINE_ANGLE_TARGET_MODE", "line_angle_target_mode", int),
    ("LINE_ANGLE_TARGET_DEG", "line_angle_target_deg", float),
    ("LINE_ANGLE_DEADBAND_DEG", "line_angle_deadband_deg", float),
    ("LINE_ANGLE_KP", "line_angle_kp", float),
    ("LINE_ANGLE_KI", "line_angle_ki", float),
    ("LINE_ANGLE_KD", "line_angle_kd", float),
    ("LINE_ANGLE_INTEGRAL_LIMIT", "line_angle_integral_limit", float),
    ("LINE_ANGLE_MAX_ROTATE_SPEED", "line_angle_max_rotate_speed", float),
    ("LINE_ANGLE_MIN_COMMAND_SPEED", "line_angle_min_command_speed", float),
    ("LINE_ANGLE_OUTPUT_SIGN", "line_angle_output_sign", float),
    ("LINE_ANGLE_FILTER_ALPHA", "line_angle_filter_alpha", float),
    ("LINE_ANGLE_MIN_LENGTH_PX", "line_angle_min_length_px", float),
    ("LINE_ANGLE_HOLD_MS", "line_angle_hold_ms", int),
    ("LINE_ANGLE_BLEND_RATIO", "line_angle_blend_ratio", float),
    ("COLOR_MAX_TRACKING_DUTY", "color_max_tracking_duty", int),
    ("COLOR_MIN_TRACKING_DUTY", "color_min_tracking_duty", int),
    ("COLOR_TARGET_HOLD_MS", "color_target_hold_ms", int),
    ("COLOR_KP_X", "color_kp_x", float),
    ("COLOR_KD_X", "color_kd_x", float),
    ("COLOR_X_OUTPUT_SIGN", "color_x_output_sign", float),
    ("COLOR_KP_Y", "color_kp_y", float),
    ("COLOR_KD_Y", "color_kd_y", float),
    ("COLOR_Y_OUTPUT_SIGN", "color_y_output_sign", float),
    ("COLOR_DEAD_ZONE", "color_dead_zone", int),
    ("COLOR_CENTER_EXIT_ZONE", "color_center_exit_zone", int),
    ("COLOR_MICRO_ADJUST_ZONE", "color_micro_adjust_zone", int),
    ("COLOR_MICRO_ADJUST_ON_CYCLES", "color_micro_adjust_on_cycles", int),
    ("COLOR_MICRO_ADJUST_OFF_CYCLES", "color_micro_adjust_off_cycles", int),
    ("COLOR_INPUT_FILTER_ALPHA", "color_input_filter_alpha", float),
    ("COLOR_COMMAND_FILTER_ALPHA", "color_command_filter_alpha", float),
    ("COLOR_MAX_TRACKING_SPEED", "color_max_tracking_speed", float),
    ("COLOR_COMMAND_RAMP_STEP", "color_command_ramp_step", float),
    ("MIN_TRANSLATE_SPEED", "min_translate_speed", float),
    ("DRIVE_MAX_DUTY", "drive_max_duty", int),
    ("DRIVE_TRANSLATE_BIAS_DUTY", "drive_translate_bias_duty", int),
    ("DRIVE_ROTATE_BIAS_DUTY", "drive_rotate_bias_duty", int),
    ("DRIVE_MIN_DUTY_START", "drive_min_duty_start", int),
    ("DRIVE_ACCELERATION", "drive_acceleration", int),
    ("STATUS_PRINT_EVERY", "status_print_every", int),
    ("ENABLE_SERIAL_LOG", "enable_serial_log", bool),
)


def _parse_bool(text):
    value = str(text).strip().upper()
    if value in ("1", "TRUE", "ON", "YES"):
        return True
    if value in ("0", "FALSE", "OFF", "NO"):
        return False
    raise ValueError("invalid bool")


def _normalize_yaw_control_mode(value):
    """把 yaw 模式约束到 0/1/2，避免误设后进入未知分支。"""
    try:
        mode = int(value)
    except Exception:
        mode = YAW_CONTROL_MODE
    if mode < 0:
        return 0
    if mode > 2:
        return 2
    return mode


def _yaw_mode_uses_closed_loop(mode):
    return _normalize_yaw_control_mode(mode) in (1, 2)


def _yaw_mode_uses_leader(mode):
    return _normalize_yaw_control_mode(mode) == 2


def _yaw_mode_name(mode):
    mode = _normalize_yaw_control_mode(mode)
    if mode == 0:
        return "open"
    if mode == 1:
        return "fixed"
    return "leader"


def _find_tunable(name):
    name = name.strip().upper()
    for item in TUNABLE_SPECS:
        if item[0] == name:
            return item
    return None


def _is_legacy_yaw_follow_name(name):
    name = name.strip().upper()
    return name in ("YAW_FOLLOW_LEADER_ENABLED", "FOLLOW_LEADER_YAW_ENABLED")


def build_config(**overrides):
    """构造运行配置，尽量把导入阶段常驻对象压小。"""
    config = {
        "self_id": SELF_ID,
        "leader_id": LEADER_ID,
        "uart_id": YAW_UART_ID,
        "baudrate": YAW_BAUDRATE,
        "rx_text_buf_limit": RX_TEXT_BUF_LIMIT,
        "timeout_ms": LEADER_TIMEOUT_MS,
        "vision_baudrate": VISION_BAUDRATE,
        "yaw_control_mode": YAW_CONTROL_MODE,
        "yaw_follow_leader_enabled": YAW_FOLLOW_LEADER_ENABLED,
        "yaw_fixed_target_deg": YAW_FIXED_TARGET_DEG,
        "camera_look_right_enabled": CAMERA_LOOK_RIGHT_ENABLED,
        "tuning_rx_text_buf_limit": TUNING_RX_TEXT_BUF_LIMIT,
        "tuning_reply_enabled": TUNING_REPLY_ENABLED,
        "tuning_protocol_prefix": TUNING_PROTOCOL_PREFIX,
        "tuning_protocol_frame_head": TUNING_PROTOCOL_FRAME_HEAD,
        "tuning_protocol_frame_tail": TUNING_PROTOCOL_FRAME_TAIL,
        "tuning_protocol_accept_legacy": TUNING_PROTOCOL_ACCEPT_LEGACY,
        "tuning_frame_buffer_limit": TUNING_FRAME_BUFFER_LIMIT,
        "tuning_command_reply_repeat_count": TUNING_COMMAND_REPLY_REPEAT_COUNT,
        "tuning_banner_repeat_count": TUNING_BANNER_REPEAT_COUNT,
        "tuning_metric_reply_repeat_count": TUNING_METRIC_REPLY_REPEAT_COUNT,
        "ai_tuning_control_enabled": AI_TUNING_CONTROL_ENABLED,
        "ai_control_paused_on_start": AI_CONTROL_PAUSED_ON_START,
        "ai_metric_enabled": AI_METRIC_ENABLED,
        "ai_metric_wireless_enabled": AI_METRIC_WIRELESS_ENABLED,
        "ai_metric_prefix": AI_METRIC_PREFIX,
        "ai_perturb_max_translate_speed": AI_PERTURB_MAX_TRANSLATE_SPEED,
        "ai_perturb_max_rotate_speed": AI_PERTURB_MAX_ROTATE_SPEED,
        "ai_perturb_max_duration_ms": AI_PERTURB_MAX_DURATION_MS,
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
        "line_angle_control_enabled": LINE_ANGLE_CONTROL_ENABLED,
        "line_angle_target_mode": LINE_ANGLE_TARGET_MODE,
        "line_angle_target_deg": LINE_ANGLE_TARGET_DEG,
        "line_angle_deadband_deg": LINE_ANGLE_DEADBAND_DEG,
        "line_angle_kp": LINE_ANGLE_KP,
        "line_angle_ki": LINE_ANGLE_KI,
        "line_angle_kd": LINE_ANGLE_KD,
        "line_angle_integral_limit": LINE_ANGLE_INTEGRAL_LIMIT,
        "line_angle_max_rotate_speed": LINE_ANGLE_MAX_ROTATE_SPEED,
        "line_angle_min_command_speed": LINE_ANGLE_MIN_COMMAND_SPEED,
        "line_angle_output_sign": LINE_ANGLE_OUTPUT_SIGN,
        "line_angle_filter_alpha": LINE_ANGLE_FILTER_ALPHA,
        "line_angle_min_length_px": LINE_ANGLE_MIN_LENGTH_PX,
        "line_angle_hold_ms": LINE_ANGLE_HOLD_MS,
        "line_angle_blend_ratio": LINE_ANGLE_BLEND_RATIO,
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
    }
    for key in overrides:
        config[key] = overrides[key]
    if "yaw_control_mode" not in overrides and "yaw_follow_leader_enabled" in overrides:
        config["yaw_control_mode"] = 2 if overrides["yaw_follow_leader_enabled"] else 1
    config["yaw_control_mode"] = _normalize_yaw_control_mode(config.get("yaw_control_mode", YAW_CONTROL_MODE))
    config["yaw_follow_leader_enabled"] = _yaw_mode_uses_leader(config["yaw_control_mode"])
    for key in ("loop_delay_ms", "gc_interval_ms", "imu_calibration_settle_ms", "motor_self_test_ms", "line_angle_hold_ms"):
        config[key] = max(0, int(config[key]))
    return config
