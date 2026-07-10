"""颜色追踪与主车 yaw 跟随的组合控制。

目标：
- 视觉模块负责让目标物移动到屏幕中心。
- 无线串口负责接收主车 yaw。
- 从车在追踪目标物的同时，用 yaw PID 让自身 yaw 与主车 yaw 保持一致。

关键点：
- 颜色追踪只计算平移速度 vx/vy，不直接写电机。
- yaw PID 只计算旋转速度 omega，不直接写电机。
- 最后统一调用 MotorControl.drive_vector(vx, vy, omega)，避免两个控制器互相覆盖电机输出。
"""

from machine import Pin
import gc
import math
import time

from ColorTrace import ColorTraceController
from FollowLeaderYawPID import YawPID, shortest_angle_error
from IMUVertical import ImuSensorVertical
from LeaderYawReceiveTest import (
    C4StatusLed,
    LeaderYawReceiver,
    STATUS_NO_SIGNAL,
    STATUS_YAW,
)
from MCXVisionUsart import MCXVisionUsart
import MotorControl


# ---------------------------------------------------------------------------
# 人工调参区
# ---------------------------------------------------------------------------

# 无线 yaw 输入参数。主车发送 ASCII 文本行："YAW:<deg>\r\n"。
SELF_ID = 2
LEADER_ID = 1
YAW_UART_ID = 2
YAW_BAUDRATE = 115200
RX_TEXT_BUF_LIMIT = 128
LEADER_TIMEOUT_MS = 1500

# 视觉串口参数。MCXVisionUsart 固定使用 UART5。
VISION_BAUDRATE = 115200

# IMU 参数，保持与 FollowLeaderYawPID.py 当前用法一致。
IMU_CAPTURE_DIV = 1
IMU_TICK_MS = 10
IMU_ACC_RANGE_G = 8
IMU_GYRO_RANGE_DPS = 2000
IMU_ACC_ALPHA = 0.20
IMU_COMP_ALPHA = 0.98
IMU_GYRO_CALI_N = 300
IMU_CALIBRATE_ON_START = True
IMU_CALIBRATION_SETTLE_MS = 500

# yaw PID 参数。按用户当前调参值原样保留，不在组合功能里改动。
PID_KP = 0.3
PID_KI = 0.05
PID_KD = 0.1
PID_INTEGRAL_LIMIT = 60.0
YAW_DEADBAND_DEG = 0.2
MAX_ROTATE_SPEED = 85.0
MIN_COMMAND_SPEED = 52.0

# 如果从车朝远离主车 yaw 的方向转，把这里改成相反数。
ROTATE_SIGN = -1.0

# 颜色追踪基础占空比。当前主要用于状态显示和兼容旧颜色追踪逻辑。
COLOR_BASE_DUTY = 6000

# 颜色追踪最大占空比。目标偏离中心较大时，平移输出不会超过这个占空比。
# MotorControl 的基础映射是 duty = speed * (max_duty / 100)。
# 例如 COLOR_MAX_TRACKING_DUTY=6500 时，speed=36 的理论占空比约为 2340；
# 三轮混合后每个轮子的最终占空比会随 vx/vy/omega 的组合而变化。
# 目标在中心附近大幅抖动时，优先降低这个值或 COLOR_MAX_TRACKING_SPEED。
COLOR_MAX_TRACKING_DUTY = 6500

# 颜色追踪最小启动占空比。只在独立颜色追踪直接写电机时更明显；
# 当前组合控制主要使用 vx/vy 混合输出，所以这里保持为较温和的 5500。
COLOR_MIN_TRACKING_DUTY = 5500

# 视觉短暂丢帧保持时间，单位 ms。过大可能导致目标已经丢失但车仍继续移动。
COLOR_TARGET_HOLD_MS = 100

# X 方向颜色追踪比例系数。数值越大，目标左右偏离时横向修正越强。
# 之前响应过强导致中心附近大幅抖动，因此这里降低到更柔和的初始值。
COLOR_KP_X = 1.2

# X 方向颜色追踪微分系数。用于抑制误差变化，但过大时输出会变尖锐。
COLOR_KD_X = 0.08

# Y 方向颜色追踪比例系数。数值越大，目标上下偏离时前后修正越强。
COLOR_KP_Y = 1.2

# Y 方向颜色追踪微分系数。用于抑制误差变化，但过大时输出会变尖锐。
COLOR_KD_Y = 0.08

# 进入中心区阈值，单位像素。目标误差小于该值时认为已经居中并停止平移修正。
COLOR_DEAD_ZONE = 12

# 离开中心区阈值，单位像素。必须大于 COLOR_DEAD_ZONE，用迟滞避免刚进中心就反向修正。
COLOR_CENTER_EXIT_ZONE = 22

# 视觉目标中心点滤波系数，范围 0.05~1.0。越小越稳但延迟越大，越大响应越快。
COLOR_INPUT_FILTER_ALPHA = 0.25

# 平移速度指令滤波系数，范围 0.05~1.0。越小动作越柔，越大动作越跟手。
COLOR_COMMAND_FILTER_ALPHA = 0.18

# 颜色追踪平移速度最大值，单位是 MotorControl 的 speed 标度 0~100，不是 duty。
# speed 会先作为 vx/vy 参与三轮底盘解算，再按 max_duty/100 映射到占空比。
# 当前 max_duty=6500 时，speed 每增加 1，理论占空比约增加 65。
# 这是限制追踪动作强度的主要参数；若追踪太慢，可以每次增加 5 左右。
COLOR_MAX_TRACKING_SPEED = 36.0

# 平移速度每次 update 最大变化量。它限制的是 speed 变化量，不直接限制 duty。
# 当前 max_duty=6500 时，1.5 speed 理论上约等于 98 duty 的变化；
# 实际最终 duty 还会受三轮混合、yaw 旋转叠加和限幅影响。
# 数值越小起停越柔和，太小会显得拖沓。
COLOR_COMMAND_RAMP_STEP = 1.5

# 平移速度下限。设为 0 表示完全使用颜色 PID 输出；如果小误差时电机不动，
# 可以只调大这个值，不需要动 yaw PID 参数。
MIN_TRANSLATE_SPEED = 0.0

# 混合驱动参数。这里用 10000 保持原 yaw speed 到 duty 的映射：
# MAX_ROTATE_SPEED=85 时，纯旋转最大 duty 约为 8500。
# DRIVE_MIN_DUTY_START 默认 0，避免很小的混合量被整体强行放大。
DRIVE_MAX_DUTY = 10000
DRIVE_MIN_DUTY_START = 0
DRIVE_ACCELERATION = 0

# 主循环和状态灯。当前 CarB 的 C4 是低电平点亮：
# Pin.value(0) 表示亮，Pin.value(1) 表示灭。
LOOP_DELAY_MS = 1
GC_INTERVAL_MS = 1000
STATUS_PRINT_EVERY = 25
ENABLE_SERIAL_LOG = True
LED_PIN = "C4"
LED_ACTIVE_LEVEL = 0
STOP_SWITCH_PIN = "D9"
STOP_SWITCH_PULL = Pin.PULL_UP_47K
STOP_SWITCH_ENABLED = False

# 可选电机链路自检。正常运行时建议保持 False。
MOTOR_SELF_TEST_ON_START = False
MOTOR_SELF_TEST_SPEED = 60.0
MOTOR_SELF_TEST_MS = 300


DEFAULT_CONFIG = {
    "self_id": SELF_ID,
    "leader_id": LEADER_ID,
    "uart_id": YAW_UART_ID,
    "baudrate": YAW_BAUDRATE,
    "rx_text_buf_limit": RX_TEXT_BUF_LIMIT,
    "timeout_ms": LEADER_TIMEOUT_MS,
    "vision_baudrate": VISION_BAUDRATE,
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
    "color_kp_y": COLOR_KP_Y,
    "color_kd_y": COLOR_KD_Y,
    "color_dead_zone": COLOR_DEAD_ZONE,
    "color_center_exit_zone": COLOR_CENTER_EXIT_ZONE,
    "color_input_filter_alpha": COLOR_INPUT_FILTER_ALPHA,
    "color_command_filter_alpha": COLOR_COMMAND_FILTER_ALPHA,
    "color_max_tracking_speed": COLOR_MAX_TRACKING_SPEED,
    "color_command_ramp_step": COLOR_COMMAND_RAMP_STEP,
    "min_translate_speed": MIN_TRANSLATE_SPEED,
    "drive_max_duty": DRIVE_MAX_DUTY,
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
    # 复用 LeaderYawReceiveTest 中 C4StatusLed 需要的闪烁时序参数。
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


def build_config(**overrides):
    config = DEFAULT_CONFIG.copy()
    for key in overrides:
        config[key] = overrides[key]
    return config


def _apply_min_vector_speed(vx, vy, min_speed):
    """在保持方向不变的前提下，给颜色追踪平移量设置可选下限。"""
    if min_speed <= 0:
        return vx, vy

    mag = math.sqrt(vx * vx + vy * vy)
    if mag <= 1e-6 or mag >= min_speed:
        return vx, vy

    scale = min_speed / mag
    return vx * scale, vy * scale


class FollowLeaderYawColorTrace:
    """组合控制器：颜色追踪给平移，主车 yaw 跟随给旋转。"""

    def __init__(self, config):
        self.config = config
        self.receiver = LeaderYawReceiver(config)
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
            kp_y=config["color_kp_y"],
            kd_y=config["color_kd_y"],
            dead_zone=config["color_dead_zone"],
            center_exit_zone=config["color_center_exit_zone"],
            input_filter_alpha=config["color_input_filter_alpha"],
            command_filter_alpha=config["color_command_filter_alpha"],
            max_tracking_speed=config["color_max_tracking_speed"],
            command_ramp_step=config["color_command_ramp_step"],
        )
        self.leader_yaw = None
        self.last_loop_ms = time.ticks_ms()
        self.running = False

    def start(self):
        MotorControl.stop(0)
        self.receiver.start()
        self.vision.clear_rx()
        self.color.start()

        print("从车 IMU 初始化中...")
        self.imu.init()
        if self.config["imu_calibrate_on_start"]:
            print("从车陀螺仪校准中，请保持从车静止。")
            time.sleep_ms(self.config["imu_calibration_settle_ms"])
            self.imu.calibrate()
            print("从车陀螺仪校准完成，开始组合追踪。")
        else:
            print("从车跳过陀螺仪校准，直接开始组合追踪。")

        self.pid.reset()
        self.last_loop_ms = time.ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.pid.reset()
        self.color.stop()
        MotorControl.stop(0)
        self.imu.stop()

    def _compute_yaw_rotate_speed(self, imu_data, now_ms, dt_s):
        if self.leader_yaw is None or self.receiver.is_timeout(now_ms):
            self.pid.reset()
            return False, None, 0.0

        yaw_error = shortest_angle_error(self.leader_yaw, imu_data["yaw"])
        if abs(yaw_error) <= self.config["yaw_deadband_deg"]:
            self.pid.reset()
            return True, yaw_error, 0.0

        raw_speed = self.pid.compute(yaw_error, dt_s)
        rotate_speed = raw_speed * self.config["rotate_sign"]
        if 0.0 < abs(rotate_speed) < self.config["min_command_speed"]:
            if rotate_speed > 0.0:
                rotate_speed = self.config["min_command_speed"]
            else:
                rotate_speed = -self.config["min_command_speed"]
        return True, yaw_error, rotate_speed

    def update(self):
        if not self.running:
            return None

        rx_data = self.receiver.update()
        if rx_data is not None:
            self.leader_yaw = rx_data["leader_yaw"]

        color_state = self.color.update_tracking_command()

        imu_data = self.imu.update()
        if imu_data is None:
            return None

        now_ms = time.ticks_ms()
        dt_s = time.ticks_diff(now_ms, self.last_loop_ms) / 1000.0
        self.last_loop_ms = now_ms

        has_leader_yaw, yaw_error, rotate_speed = self._compute_yaw_rotate_speed(
            imu_data,
            now_ms,
            dt_s,
        )

        if not has_leader_yaw:
            MotorControl.stop(0)
            return {
                "has_leader_yaw": False,
                "leader_yaw": self.leader_yaw,
                "follower_yaw": imu_data["yaw"],
                "yaw_error": None,
                "rotate_speed": 0.0,
                "has_target": False if color_state is None else color_state["has_target"],
                "target_locked": False if color_state is None else color_state["target_locked"],
                "in_center": False if color_state is None else color_state["in_center"],
                "err_x": 0 if color_state is None else color_state["err_x"],
                "err_y": 0 if color_state is None else color_state["err_y"],
                "vx": 0.0,
                "vy": 0.0,
                "motor_duty": (0, 0, 0),
            }

        vx = 0.0
        vy = 0.0
        if (
            color_state is not None
            and color_state["target_locked"]
            and color_state["moving"]
            and not color_state["in_center"]
        ):
            vx = color_state["vx"]
            vy = color_state["vy"]
            vx, vy = _apply_min_vector_speed(
                vx,
                vy,
                self.config["min_translate_speed"],
            )

        if abs(vx) <= 1e-6 and abs(vy) <= 1e-6 and abs(rotate_speed) <= 1e-6:
            motor_duty = MotorControl.stop(0)
            motor_duty = (0, 0, 0)
        else:
            motor_duty = MotorControl.drive_vector(
                vx,
                vy,
                omega=rotate_speed,
                acceleration=self.config["drive_acceleration"],
                max_duty=self.config["drive_max_duty"],
                min_duty_start=self.config["drive_min_duty_start"],
            )

        return {
            "has_leader_yaw": True,
            "leader_yaw": self.leader_yaw,
            "follower_yaw": imu_data["yaw"],
            "yaw_error": yaw_error,
            "rotate_speed": rotate_speed,
            "has_target": False if color_state is None else color_state["has_target"],
            "target_locked": False if color_state is None else color_state["target_locked"],
            "in_center": False if color_state is None else color_state["in_center"],
            "err_x": 0 if color_state is None else color_state["err_x"],
            "err_y": 0 if color_state is None else color_state["err_y"],
            "vx": vx,
            "vy": vy,
            "motor_duty": motor_duty,
        }


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
        print("=== CarB 颜色追踪 + 主车 yaw 跟随 ===")
        print("yaw 输入：UART{} {}bps，文本 YAW:<deg>".format(config["uart_id"], config["baudrate"]))
        print("视觉输入：UART5 {}bps，目标中心为 {}x{} 屏幕中心".format(
            config["vision_baudrate"],
            MCXVisionUsart.VIEW_WIDTH,
            MCXVisionUsart.VIEW_HEIGHT,
        ))
        print("C4：一个长闪=未收到有效 yaw，基本常亮=正在组合控制")
        print("yaw PID：kp={} ki={} kd={}，死区=+/-{} 度".format(
            config["pid_kp"],
            config["pid_ki"],
            config["pid_kd"],
            config["yaw_deadband_deg"],
        ))

    tick = 0
    last_gc_ms = time.ticks_ms()

    try:
        while True:
            now_ms = time.ticks_ms()
            result = controller.update()

            status = STATUS_YAW
            if controller.leader_yaw is None or controller.receiver.is_timeout(now_ms):
                status = STATUS_NO_SIGNAL
            status_led.update(status, now_ms)

            if result is not None:
                tick += 1
                if (
                    config["enable_serial_log"]
                    and tick % config["status_print_every"] == 0
                ):
                    print(
                        (
                            "leader={:>7} follower={:>7.2f} yaw_err={:>7} "
                            "rot={:>6.1f} target={} center={} ex={:>5} ey={:>5} "
                            "vx={:>6.1f} vy={:>6.1f} duty={}"
                        ).format(
                            "--" if result["leader_yaw"] is None else "{:.2f}".format(result["leader_yaw"]),
                            result["follower_yaw"],
                            "--" if result["yaw_error"] is None else "{:.2f}".format(result["yaw_error"]),
                            result["rotate_speed"],
                            result["target_locked"],
                            result["in_center"],
                            result["err_x"],
                            result["err_y"],
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
