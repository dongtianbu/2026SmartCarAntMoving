"""主车 yaw 广播与颜色追踪组合控制。

主车上电后的目标流程：
1. 停止三电机，避免初始化期间误动作。
2. 初始化并校准主车 IMU，然后持续向从车广播主车 yaw。
3. 等待视觉模块完成上电稳定和摄像头标定，确认串口能收到有效视觉帧。
4. 视觉准备完成后立即启动颜色追踪；首次识别到目标物时锁定当前 yaw，追踪目标的同时闭环保持车头角度。

说明：
- 摄像头颜色阈值、曝光等真正的标定一般在视觉模块侧完成。
- 本文件中的“摄像头标定等待”指主控上电后给视觉模块留出稳定/标定时间，
  并等待它输出若干个格式正确的视觉帧；在此之前主车不会启动追踪电机。
- yaw 广播和颜色追踪在同一个主循环中运行，避免主车开始追踪后从车收不到 yaw。
- 颜色追踪只负责计算平移速度 vx/vy，yaw 闭环只负责计算旋转速度 omega，
  最后统一合成三轮占空比，避免两个控制器分别写电机互相覆盖。
"""

from machine import Pin
import gc
import time

from connection.MCXVisionUsart import MCXVisionUsart
from control import MotorControl
from control.ColorTrace import ColorTraceController
from control.LeaderYawBroadcaster import LeaderYawBroadcaster


# ---------------------------------------------------------------------------
# 人工调参区
# ---------------------------------------------------------------------------

# 主车无线 yaw 广播 ID。主车固定为 1 号，从车固定为 2 号。
SELF_ID = 1
PEER_ID = 2

# yaw 无线串口号。当前无线模块接在 UART2，必须与从车接收端一致。
YAW_UART_ID = 2

# yaw 无线串口波特率。无线串口模块最高支持 115200，不要再调高。
YAW_BAUDRATE = 115200

# yaw 发送周期，单位 ms。20ms 约等于 50Hz，足够从车 yaw 跟随使用。
YAW_SEND_INTERVAL_MS = 20

# 主车 IMU 参数。上电后会先校准陀螺仪，请保持主车静止。
IMU_CAPTURE_DIV = 1
IMU_TICK_MS = 10
IMU_ACC_RANGE_G = 8
IMU_GYRO_RANGE_DPS = 2000
IMU_ACC_ALPHA = 0.20
IMU_COMP_ALPHA = 0.98
IMU_GYRO_CALI_N = 300

# 视觉串口波特率。MCXVisionUsart 固定使用 UART5。
VISION_BAUDRATE = 115200

# 摄像头/视觉模块上电稳定等待时间，单位 ms。
# 这段时间用于等待视觉模块完成自启动、曝光稳定、颜色阈值标定等动作。
# 等待期间主车不做颜色追踪，但会继续刷新 yaw 广播。
CAMERA_CALIBRATION_WAIT_MS = 2500

# 等待视觉模块有效帧的最长时间，单位 ms。
# 有效帧指帧头、帧尾、坐标范围都能通过 MCXVisionUsart.parse_frame() 校验；
# 有目标帧和无目标帧都算有效，因为它们都说明摄像头串口已经正常输出。
VISION_READY_TIMEOUT_MS = 8000

# 启动追踪前要求连续收到的有效视觉帧数量。
# 数值越大，启动越稳但等待越久；3 帧通常足够确认串口和摄像头已正常输出。
VISION_READY_FRAME_COUNT = 3

# 颜色追踪基础占空比。当前主要用于状态输出，实际控制由颜色 PID 和占空比限幅决定。
TRACK_BASE_DUTY = 6000

# 颜色追踪最大占空比。速度到占空比的基础映射在 MotorControl 中完成：
#   duty = speed * (max_duty / 100)
# 例如 max_duty=7500 时，speed=40 的理论占空比约为 3000。
# 之后还会经过三轮混合、近中心限幅和占空比斜坡，所以最终每个轮子的 duty 会不同。
TRACK_MAX_DUTY = 7500

# 颜色追踪最小启动占空比。目标偏离中心且产生非零输出时，控制器会把占空比抬到此区间，
# 用来帮助电机克服静摩擦。该值与参考工程颜色追踪入口参数一致。
TRACK_MIN_ACTIVE_DUTY = 5700

# 目标短暂丢帧保持时间，单位 ms。过大时目标消失后仍可能继续沿旧方向运动。
TRACK_TARGET_HOLD_MS = 120

# X 方向颜色追踪比例系数。该 PID 参数与参考工程颜色追踪入口参数一致。
TRACK_KP_X = 0.2

# X 方向颜色追踪微分系数。该 PID 参数与参考工程颜色追踪入口参数一致。
TRACK_KD_X = 0.75

# Y 方向颜色追踪比例系数。该 PID 参数与参考工程颜色追踪入口参数一致。
TRACK_KP_Y = 0.2

# Y 方向颜色追踪微分系数。该 PID 参数与参考工程颜色追踪入口参数一致。
TRACK_KD_Y = 0.75

# 进入中心区阈值，单位像素。目标误差小于该值时认为已经到达屏幕中心。
TRACK_DEAD_ZONE = 6

# 离开中心区阈值，单位像素。必须大于进入阈值，用迟滞避免中心附近反复启停。
TRACK_CENTER_EXIT_ZONE = 22

# 视觉中心点滤波系数，范围 0.05~1.0。越小越稳但延迟越大，越大越跟手。
TRACK_INPUT_FILTER_ALPHA = 0.25

# 速度指令滤波系数，范围 0.05~1.0。越小电机越柔和，越大响应越快。
TRACK_COMMAND_FILTER_ALPHA = 0.18

# 颜色追踪平移速度最大值，单位是 MotorControl 的 speed 标度 0~100，不是 duty。
# speed 会先作为 vx/vy 参与三轮底盘解算，再按 max_duty/100 映射到占空比。
# 当前 max_duty=7500 时，speed 每增加 1，理论占空比约增加 75。
# 该值属于本文件保留的速度限幅参数，参考工程没有对应项。
TRACK_MAX_TRACKING_SPEED = 36.0

# 平移速度每次 update 最大变化量。它限制的是 speed 变化量，不直接限制 duty。
# 当前 max_duty=7500 时，1.5 speed 理论上约等于 112 duty 的变化；
# 实际最终 duty 还会受三轮混合和限幅影响。该值属于本文件保留的斜坡参数，参考工程没有对应项。
TRACK_COMMAND_RAMP_STEP = 1.5

# 是否启用“识别到目标物后保持当前 yaw”的闭环。调试颜色追踪方向时可先改成 False。
YAW_HOLD_ENABLE = True

# yaw 闭环比例系数。误差单位是度，输出单位是 MotorControl 的旋转 speed 标度。
# 数值越大，车头偏离锁定 yaw 时修正越强；如果追踪时左右摆头明显，优先降低它。
YAW_HOLD_KP = 0.6

# yaw 闭环积分系数。用于补偿长期偏差；初期调试建议为 0，避免积分累积导致越调越猛。
YAW_HOLD_KI = 0.0

# yaw 闭环微分系数。用于抑制 yaw 误差快速变化；过大可能让旋转输出变尖锐。
YAW_HOLD_KD = 0.05

# yaw 闭环积分限幅，单位是“度*秒”。只有 YAW_HOLD_KI 非 0 时才明显起作用。
YAW_HOLD_INTEGRAL_LIMIT = 30.0

# yaw 保持死区，单位度。误差小于该值时认为车头角度足够接近，不输出旋转修正。
YAW_HOLD_DEADBAND_DEG = 1.0

# yaw 闭环最大旋转速度，单位是 MotorControl 的 speed 标度 0~100。
# 该值越大，叠加到颜色追踪上的转向越强；首次实车建议从 8~15 之间试。
YAW_HOLD_MAX_ROTATE_SPEED = 12.0

# yaw 闭环最小旋转速度。设为 0 表示小误差时不强行抬高旋转输出，动作更柔和。
# 如果发现 yaw 明显偏了但电机完全不修正，可以逐步加到 3、5、8 试。
YAW_HOLD_MIN_ROTATE_SPEED = 0.0

# yaw 闭环独立旋转时的最小启动占空比。设为 0 表示不把纯 yaw 小输出强行放大。
# 如果目标已经居中但车头角度无法修回来，可以再逐步增大；过大容易让车在中心附近猛转。
YAW_HOLD_MIN_DUTY_START = 0

# yaw 旋转方向符号。如果识别到目标后车头朝远离锁定 yaw 的方向修正，把这里改成 1.0。
YAW_HOLD_ROTATE_SIGN = -1.0

# yaw 闭环输出斜坡，单位是每次主循环允许变化的 rotate speed。
# 数值越小，转向越柔和但响应越慢；数值越大，车头修正更快但更容易抖。
YAW_HOLD_RAMP_STEP = 0.8

# 主循环延时，单位 ms。越小控制刷新越快，但串口和垃圾回收压力更大。
MAIN_LOOP_DELAY_MS = 1

# 垃圾回收周期，单位 ms。定期回收可降低长时间运行时的内存碎片风险。
GC_INTERVAL_MS = 1000

# 状态打印间隔，单位循环次数。设大一些可以减少串口打印对控制周期的影响。
STATUS_PRINT_EVERY = 50

# 是否启用串口日志。调车时 True 方便观察，正式跑车可改 False。
ENABLE_SERIAL_LOG = True

# C4 状态灯引脚。当前逻辑只是周期翻转，表示主循环仍在运行。
LED_PIN = "C4"

# C4 翻转间隔，单位为 yaw 成功发送次数。数值越小闪烁越快。
LED_TOGGLE_EVERY_SENDS = 25

# 停止按键引脚。按下后主车会停止颜色追踪和三电机。
STOP_SWITCH_PIN = "D9"

# 停止按键上拉配置。保持与现有硬件用法一致。
STOP_SWITCH_PULL = Pin.PULL_UP_47K


DEFAULT_CONFIG = {
    "self_id": SELF_ID,
    "peer_id": PEER_ID,
    "uart_id": YAW_UART_ID,
    "baudrate": YAW_BAUDRATE,
    "send_interval_ms": YAW_SEND_INTERVAL_MS,
    "imu_capture_div": IMU_CAPTURE_DIV,
    "imu_tick_ms": IMU_TICK_MS,
    "imu_acc_range_g": IMU_ACC_RANGE_G,
    "imu_gyro_range_dps": IMU_GYRO_RANGE_DPS,
    "imu_acc_alpha": IMU_ACC_ALPHA,
    "imu_comp_alpha": IMU_COMP_ALPHA,
    "imu_gyro_cali_n": IMU_GYRO_CALI_N,
    "vision_baudrate": VISION_BAUDRATE,
    "camera_calibration_wait_ms": CAMERA_CALIBRATION_WAIT_MS,
    "vision_ready_timeout_ms": VISION_READY_TIMEOUT_MS,
    "vision_ready_frame_count": VISION_READY_FRAME_COUNT,
    "track_base_duty": TRACK_BASE_DUTY,
    "track_max_duty": TRACK_MAX_DUTY,
    "track_min_active_duty": TRACK_MIN_ACTIVE_DUTY,
    "track_target_hold_ms": TRACK_TARGET_HOLD_MS,
    "track_kp_x": TRACK_KP_X,
    "track_kd_x": TRACK_KD_X,
    "track_kp_y": TRACK_KP_Y,
    "track_kd_y": TRACK_KD_Y,
    "track_dead_zone": TRACK_DEAD_ZONE,
    "track_center_exit_zone": TRACK_CENTER_EXIT_ZONE,
    "track_input_filter_alpha": TRACK_INPUT_FILTER_ALPHA,
    "track_command_filter_alpha": TRACK_COMMAND_FILTER_ALPHA,
    "track_max_tracking_speed": TRACK_MAX_TRACKING_SPEED,
    "track_command_ramp_step": TRACK_COMMAND_RAMP_STEP,
    "yaw_hold_enable": YAW_HOLD_ENABLE,
    "yaw_hold_kp": YAW_HOLD_KP,
    "yaw_hold_ki": YAW_HOLD_KI,
    "yaw_hold_kd": YAW_HOLD_KD,
    "yaw_hold_integral_limit": YAW_HOLD_INTEGRAL_LIMIT,
    "yaw_hold_deadband_deg": YAW_HOLD_DEADBAND_DEG,
    "yaw_hold_max_rotate_speed": YAW_HOLD_MAX_ROTATE_SPEED,
    "yaw_hold_min_rotate_speed": YAW_HOLD_MIN_ROTATE_SPEED,
    "yaw_hold_min_duty_start": YAW_HOLD_MIN_DUTY_START,
    "yaw_hold_rotate_sign": YAW_HOLD_ROTATE_SIGN,
    "yaw_hold_ramp_step": YAW_HOLD_RAMP_STEP,
    "main_loop_delay_ms": MAIN_LOOP_DELAY_MS,
    "gc_interval_ms": GC_INTERVAL_MS,
    "status_print_every": STATUS_PRINT_EVERY,
    "enable_serial_log": ENABLE_SERIAL_LOG,
    "led_pin": LED_PIN,
    "led_toggle_every_sends": LED_TOGGLE_EVERY_SENDS,
    "stop_switch_pin": STOP_SWITCH_PIN,
    "stop_switch_pull": STOP_SWITCH_PULL,
}


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


def _stop_all_motors():
    MotorControl.motor_1.duty(0)
    MotorControl.motor_2.duty(0)
    MotorControl.motor_3.duty(0)


def _apply_motor_duty(duty):
    """一次性下发三轮占空比，避免颜色追踪和 yaw 闭环分别抢电机。"""
    d1, d2, d3 = duty
    MotorControl.motor_1.duty(d1)
    MotorControl.motor_2.duty(d2)
    MotorControl.motor_3.duty(d3)


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _wrap_angle_deg(angle_deg):
    """旧版角度折返工具，仅保留给需要等效朝向判断的代码使用。"""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def _continuous_yaw_error(target_deg, current_deg):
    """返回连续 yaw 误差，不做 180/-180 折返。"""
    return target_deg - current_deg


def _step_towards(current, target, step):
    """按固定步长逼近目标值，用于限制 yaw 旋转输出突变。"""
    if step <= 0:
        return target
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def _scale_drive_duty_vector(raw_duty, max_duty, min_duty_start):
    """按同一比例缩放三轮占空比，保持 vx/vy/omega 混合后的运动比例。"""
    max_abs_duty = max(abs(int(value)) for value in raw_duty)
    if max_abs_duty <= 0:
        return (0, 0, 0)

    scale = 1.0
    if 0 < max_abs_duty < min_duty_start:
        # 只让最大轮达到启动占空比，其余轮按原比例同步缩放，避免额外引入旋转。
        scale = min_duty_start / float(max_abs_duty)

    if max_abs_duty * scale > max_duty:
        scale = max_duty / float(max_abs_duty)

    return tuple(
        max(-max_duty, min(max_duty, int(round(value * scale))))
        for value in raw_duty
    )


class YawHoldPID:
    """主车颜色追踪时使用的 yaw 保持 PID。"""

    def __init__(self, kp, ki, kd, integral_limit, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.integral = 0.0
        self.last_error = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None

    def compute(self, error, dt_s):
        if dt_s <= 0.0:
            dt_s = MAIN_LOOP_DELAY_MS / 1000.0

        self.integral += error * dt_s
        self.integral = _clamp(
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
        return _clamp(output, -self.output_limit, self.output_limit)


def _create_color_trace(config):
    return ColorTraceController(
        MCXVisionUsart(baudrate=config["vision_baudrate"]),
        base_duty=config["track_base_duty"],
        max_tracking_duty=config["track_max_duty"],
        min_tracking_duty=config["track_min_active_duty"],
        target_hold_ms=config["track_target_hold_ms"],
        kp_x=config["track_kp_x"],
        kd_x=config["track_kd_x"],
        kp_y=config["track_kp_y"],
        kd_y=config["track_kd_y"],
        dead_zone=config["track_dead_zone"],
        center_exit_zone=config["track_center_exit_zone"],
        input_filter_alpha=config["track_input_filter_alpha"],
        command_filter_alpha=config["track_command_filter_alpha"],
        max_tracking_speed=config["track_max_tracking_speed"],
        command_ramp_step=config["track_command_ramp_step"],
    )


def _service_yaw_broadcast(broadcaster, led, config, counters):
    data = broadcaster.update()
    if data is None:
        return None

    counters["yaw_tick"] += 1
    if data["sent"]:
        counters["send_count"] += 1
        if counters["send_count"] % config["led_toggle_every_sends"] == 0:
            led.toggle()
    return data


def _update_yaw_hold(result, current_yaw, yaw_pid, yaw_state, config, now_ms):
    """识别到目标物后锁定当前 yaw，并计算保持该 yaw 所需的 omega。"""
    info = {
        "active": False,
        "target_yaw": yaw_state["target_yaw"],
        "error": 0.0,
        "omega": 0.0,
    }

    if (
        not config["yaw_hold_enable"]
        or result is None
        or not result["target_locked"]
        or current_yaw is None
    ):
        yaw_pid.reset()
        yaw_state["target_yaw"] = None
        yaw_state["last_ms"] = None
        yaw_state["omega"] = 0.0
        return info

    if yaw_state["target_yaw"] is None:
        # 第一次识别到目标物时，把当前车头角度作为追踪期间的 yaw 目标。
        yaw_state["target_yaw"] = current_yaw
        yaw_state["last_ms"] = now_ms
        yaw_state["omega"] = 0.0
        yaw_pid.reset()

    if yaw_state["last_ms"] is None:
        dt_s = config["main_loop_delay_ms"] / 1000.0
    else:
        dt_s = time.ticks_diff(now_ms, yaw_state["last_ms"]) / 1000.0
    yaw_state["last_ms"] = now_ms

    error = _continuous_yaw_error(yaw_state["target_yaw"], current_yaw)
    if abs(error) <= config["yaw_hold_deadband_deg"]:
        target_omega = 0.0
        yaw_pid.reset()
    else:
        target_omega = yaw_pid.compute(error, dt_s) * config["yaw_hold_rotate_sign"]
        target_omega = _clamp(
            target_omega,
            -config["yaw_hold_max_rotate_speed"],
            config["yaw_hold_max_rotate_speed"],
        )
        min_speed = config["yaw_hold_min_rotate_speed"]
        if min_speed > 0 and abs(target_omega) < min_speed:
            target_omega = min_speed if target_omega > 0 else -min_speed

    omega = _step_towards(
        yaw_state["omega"],
        target_omega,
        config["yaw_hold_ramp_step"],
    )
    yaw_state["omega"] = omega

    info["active"] = True
    info["target_yaw"] = yaw_state["target_yaw"]
    info["error"] = error
    info["omega"] = omega
    return info


def _drive_color_trace_with_yaw_hold(result, yaw_info, config):
    """把颜色追踪平移量和 yaw 闭环旋转量合成为一次电机输出。"""
    if result is None:
        _stop_all_motors()
        return

    vx = result["vx"] if result["target_locked"] else 0.0
    vy = result["vy"] if result["target_locked"] else 0.0
    omega = yaw_info["omega"] if yaw_info["active"] else 0.0

    if not result["target_locked"] or (not result["moving"] and abs(omega) <= 1e-6):
        _stop_all_motors()
        result["omega"] = 0.0
        result["yaw_hold_error"] = yaw_info["error"]
        result["yaw_hold_target"] = yaw_info["target_yaw"]
        result["raw_duty"] = (0, 0, 0)
        result["duty"] = (0, 0, 0)
        return

    raw_duty = MotorControl.vector_to_duty(
        vx,
        vy,
        omega=omega,
        max_duty=config["track_max_duty"],
        min_duty_start=0,
    )

    if result["moving"]:
        min_duty_start = config["track_min_active_duty"]
    else:
        # 目标已在中心时，如果只有 yaw 修正，使用单独的较小启动占空比参数。
        min_duty_start = config["yaw_hold_min_duty_start"]

    duty = _scale_drive_duty_vector(
        raw_duty,
        config["track_max_duty"],
        min_duty_start,
    )
    _apply_motor_duty(duty)

    result["omega"] = omega
    result["yaw_hold_error"] = yaw_info["error"]
    result["yaw_hold_target"] = yaw_info["target_yaw"]
    result["raw_duty"] = raw_duty
    result["duty"] = duty


def _wait_camera_ready(color_trace, broadcaster, led, stop_switch, stop_state, config):
    """等待视觉模块稳定输出有效帧，同时保持 yaw 广播不断。"""
    mcx = color_trace.mcx
    mcx.clear_rx()
    start_ms = time.ticks_ms()
    counters = {"yaw_tick": 0, "send_count": 0}

    if config["enable_serial_log"]:
        print("摄像头/视觉模块标定等待中，请完成视觉侧颜色标定。")

    while time.ticks_diff(time.ticks_ms(), start_ms) < config["camera_calibration_wait_ms"]:
        _service_yaw_broadcast(broadcaster, led, config, counters)
        if stop_switch.value() != stop_state:
            return False
        time.sleep_ms(config["main_loop_delay_ms"])

    mcx.clear_rx()
    rx_buf = bytearray()
    ready_frames = 0
    wait_start_ms = time.ticks_ms()

    while ready_frames < config["vision_ready_frame_count"]:
        now_ms = time.ticks_ms()
        _service_yaw_broadcast(broadcaster, led, config, counters)

        raw = mcx.recv_bytes()
        if raw is not None:
            rx_buf.extend(raw)

        while len(rx_buf) >= mcx.FRAME_SIZE:
            frame = MCXVisionUsart.parse_frame(bytes(rx_buf[:mcx.FRAME_SIZE]))
            if frame is not None:
                ready_frames += 1
                rx_buf = rx_buf[mcx.FRAME_SIZE:]
            else:
                rx_buf = rx_buf[1:]

        if ready_frames >= config["vision_ready_frame_count"]:
            break

        if time.ticks_diff(now_ms, wait_start_ms) > config["vision_ready_timeout_ms"]:
            if config["enable_serial_log"]:
                print("等待视觉有效帧超时，继续等待，不启动颜色追踪电机。")
            wait_start_ms = now_ms

        if stop_switch.value() != stop_state:
            return False

        time.sleep_ms(config["main_loop_delay_ms"])
        gc.collect()

    mcx.clear_rx()
    if config["enable_serial_log"]:
        print("视觉模块已输出有效帧，主车将直接启动颜色追踪。")
    return True



def run_leader_yaw_color_trace(config=None):
    if config is None:
        config = build_config()

    led = Pin(config["led_pin"], Pin.OUT, value=True)
    stop_switch = Pin(
        config["stop_switch_pin"],
        Pin.IN,
        pull=config["stop_switch_pull"],
    )
    stop_state = stop_switch.value()

    _stop_all_motors()
    broadcaster = LeaderYawBroadcaster(config)
    color_trace = _create_color_trace(config)

    if config["enable_serial_log"]:
        print("=== CarA 主车 yaw 广播 + 颜色追踪 ===")
        print("当前主车上电功能：先校准 IMU 并广播 yaw，视觉准备完成后自动启动颜色追踪。")
        print("颜色追踪期间：识别到目标物后锁定当前 yaw，并叠加 yaw 闭环保持车头角度。")
        print("视觉屏幕：{}x{}，目标中心：({}, {})".format(
            color_trace.SCREEN_W,
            color_trace.SCREEN_H,
            color_trace.CENTER_X,
            color_trace.CENTER_Y,
        ))
        print("速度/占空比：speed=100 对应 max_duty，当前 max_duty={}。".format(
            config["track_max_duty"],
        ))
        print("停止按键：{}".format(config["stop_switch_pin"]))
        print("请先保持主车静止，等待 IMU 校准。")

    try:
        broadcaster.start()
        if config["enable_serial_log"]:
            print("IMU 校准完成，yaw 广播已启动。")

        if not _wait_camera_ready(color_trace, broadcaster, led, stop_switch, stop_state, config):
            return

        color_trace.start()
        if config["enable_serial_log"]:
            print("视觉准备完成，颜色追踪已自动启动。")
        counters = {"yaw_tick": 0, "send_count": 0, "loop_count": 0}
        last_gc_ms = time.ticks_ms()
        latest_yaw = None
        yaw_pid = YawHoldPID(
            config["yaw_hold_kp"],
            config["yaw_hold_ki"],
            config["yaw_hold_kd"],
            config["yaw_hold_integral_limit"],
            config["yaw_hold_max_rotate_speed"],
        )
        yaw_state = {
            "target_yaw": None,
            "last_ms": None,
            "omega": 0.0,
        }

        while True:
            counters["loop_count"] += 1
            yaw_data = _service_yaw_broadcast(broadcaster, led, config, counters)
            if yaw_data is not None:
                latest_yaw = yaw_data["yaw"]

            now_ms = time.ticks_ms()
            result = color_trace.update_tracking_command()
            yaw_info = _update_yaw_hold(
                result,
                latest_yaw,
                yaw_pid,
                yaw_state,
                config,
                now_ms,
            )
            _drive_color_trace_with_yaw_hold(result, yaw_info, config)

            if (
                config["enable_serial_log"]
                and result is not None
                and counters["loop_count"] % config["status_print_every"] == 0
            ):
                yaw_text = "--" if latest_yaw is None else "{:.2f}".format(latest_yaw)
                yaw_target_text = "--" if yaw_info["target_yaw"] is None else "{:.2f}".format(yaw_info["target_yaw"])
                print(
                    (
                        "yaw={} yaw_target={} yaw_err={:>+5.2f} omega={:>+5.2f} "
                        "target={} center={} err=({:>+5.0f},{:>+5.0f}) "
                        "corr=({:>+6.1f},{:>+6.1f}) duty={}"
                    ).format(
                        yaw_text,
                        yaw_target_text,
                        yaw_info["error"],
                        yaw_info["omega"],
                        result["target_locked"],
                        result["in_center"],
                        result["err_x"],
                        result["err_y"],
                        result["correction_x"],
                        result["correction_y"],
                        result["duty"],
                    )
                )

            if stop_switch.value() != stop_state:
                if config["enable_serial_log"]:
                    print("收到停止按键，主车停车。")
                break

            if time.ticks_diff(now_ms, last_gc_ms) >= config["gc_interval_ms"]:
                gc.collect()
                last_gc_ms = now_ms

            time.sleep_ms(config["main_loop_delay_ms"])
    finally:
        color_trace.stop()
        broadcaster.stop()
        _stop_all_motors()
        led.value(1)


if __name__ == "__main__":
    run_leader_yaw_color_trace()
