"""使用 PID 闭环让从车 yaw 跟随主车 yaw。

流程说明：
- 主车 CarA 发送 ASCII 文本行 `"YAW:<deg>\\r\\n"`。
- 从车 CarB 接收主车 yaw，同时读取自身 IMU yaw。
- PID 根据两车连续 yaw 误差输出原地旋转速度，直到两车朝向一致。

安全提示：
- 第一次调参建议把车轮架空。
- 如果越调越偏，修改 `ROTATE_SIGN` 的符号。
- 主车 yaw 丢失时，从车会立即停止电机。
"""

from machine import Pin
import gc
import time

import MotorControl
from IMUVertical import ImuSensorVertical
from LeaderYawReceiveTest import (
    C4StatusLed,
    LeaderYawReceiver,
    STATUS_NO_SIGNAL,
    STATUS_YAW,
)


# ---------------------------------------------------------------------------
# 人工调参
# ---------------------------------------------------------------------------

# 无线文本 yaw 输入参数，需与 CarA/connection/CarFollowProtocol.py 保持一致
SELF_ID = 2
LEADER_ID = 1
UART_ID = 2
BAUDRATE = 115200
RX_TEXT_BUF_LIMIT = 128
LEADER_TIMEOUT_MS = 1500

# IMU 参数，保持与 IMUVertical 用法一致
IMU_CAPTURE_DIV = 1
IMU_TICK_MS = 10
IMU_ACC_RANGE_G = 8
IMU_GYRO_RANGE_DPS = 2000
IMU_ACC_ALPHA = 0.20
IMU_COMP_ALPHA = 0.98
IMU_GYRO_CALI_N = 300
IMU_CALIBRATE_ON_START = True
IMU_CALIBRATION_SETTLE_MS = 500
IMU_YAW_SIGN = 1  # 1: 保持当前 yaw 方向；-1: 整体反向，可切换顺时针旋转时角度增减关系

# PID 参数决定 yaw 误差如何映射到旋转速度。
# 输出最终通过 MotorControl.rotate(speed) 驱动。
# MIN_COMMAND_SPEED 用于克服静摩擦，避免指令太小转不动。
PID_KP = 0.18
PID_KI = 0.07
PID_KD = 0.3
PID_INTEGRAL_LIMIT = 60.0
YAW_DEADBAND_DEG = 0.8
MAX_ROTATE_SPEED = 64.0
MIN_COMMAND_SPEED = 42.0

# 如果跟随方向反了，把 ROTATE_SIGN 改成相反数
ROTATE_SIGN = -1.0

# 主循环时序和状态灯配置，当前 C4 为低电平点亮
LOOP_DELAY_MS = 1
GC_INTERVAL_MS = 1000
STATUS_PRINT_EVERY = 25
ENABLE_SERIAL_LOG = True
LED_PIN = "C4"
LED_ACTIVE_LEVEL = 0
STOP_SWITCH_PIN = "D9"
STOP_SWITCH_PULL = Pin.PULL_UP_47K
STOP_SWITCH_ENABLED = False

START_KEY_PIN = "C8"
START_KEY_PULL = Pin.PULL_UP_47K
START_KEY_ACTIVE_LEVEL = 0
START_KEY_DEBOUNCE_MS = 30

# 是否启用启动前电机自检，正常运行建议保持 False
MOTOR_SELF_TEST_ON_START = False
MOTOR_SELF_TEST_SPEED = 60.0
MOTOR_SELF_TEST_MS = 300

DEFAULT_CONFIG = {
    "self_id": SELF_ID,
    "leader_id": LEADER_ID,
    "uart_id": UART_ID,
    "baudrate": BAUDRATE,
    "rx_text_buf_limit": RX_TEXT_BUF_LIMIT,
    "timeout_ms": LEADER_TIMEOUT_MS,
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
    # 复用 LeaderYawReceiveTest / C4StatusLed 所需的状态灯时序参数
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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle_deg(angle_deg):
    """把角度归一化到 [-180, 180] 区间。"""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def continuous_yaw_error(target_deg, current_deg):
    """返回连续 yaw 误差，不做 180/-180 折返。"""
    return target_deg - current_deg


def shortest_angle_error(target_deg, current_deg):
    """兼容旧接口；当前实现等价于 continuous_yaw_error。"""
    return continuous_yaw_error(target_deg, current_deg)


def _is_start_key_pressed(start_key, config):
    return start_key.value() == config["start_key_active_level"]


def _wait_start_key(start_key, config):
    if config["enable_serial_log"]:
        print("等待 IMU 初始化完成后，按下 {} 开始 yaw 跟随".format(
            config["start_key_pin"],
        ))

    while True:
        MotorControl.stop(0)
        if _is_start_key_pressed(start_key, config):
            time.sleep_ms(config["start_key_debounce_ms"])
            if _is_start_key_pressed(start_key, config):
                if config["enable_serial_log"]:
                    print("{} 已按下，开始 yaw 跟随".format(
                        config["start_key_pin"],
                    ))
                return

        time.sleep_ms(config["loop_delay_ms"])
        gc.collect()

class YawPID:
    """带 dt、积分限幅和输出限幅的简化 PID 控制器。"""

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


class FollowLeaderYawPID:
    """接收主车 yaw，并控制 CarB 原地旋转直到两车 yaw 一致。"""

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
            yaw_sign=config["imu_yaw_sign"],
        )
        self.pid = YawPID(
            config["pid_kp"],
            config["pid_ki"],
            config["pid_kd"],
            config["max_rotate_speed"],
            config["pid_integral_limit"],
        )
        self.leader_yaw = None
        self.last_loop_ms = time.ticks_ms()
        self.running = False

    def start(self):
        MotorControl.stop(0)
        self.receiver.start()
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
            print("CarB IMU calibration finished, ready for yaw follow.")
        else:
            print("CarB IMU calibration skipped, ready for yaw follow.")
        _wait_start_key(start_key, self.config)
        self.pid.reset()
        self.last_loop_ms = time.ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.pid.reset()
        MotorControl.stop(0)
        self.imu.stop()

    def update(self):
        if not self.running:
            return None

        rx_data = self.receiver.update()
        if rx_data is not None:
            self.leader_yaw = rx_data["leader_yaw"]

        imu_data = self.imu.update()
        if imu_data is None:
            return None

        now_ms = time.ticks_ms()
        dt_s = time.ticks_diff(now_ms, self.last_loop_ms) / 1000.0
        self.last_loop_ms = now_ms

        if self.leader_yaw is None:
            self.pid.reset()
            MotorControl.stop(0)
            return {
                "has_leader_yaw": False,
                "leader_yaw": self.leader_yaw,
                "follower_yaw": imu_data["yaw"],
                "yaw_error": None,
                "rotate_speed": 0.0,
                "motor_duty": (0, 0, 0),
            }

        follower_yaw = imu_data["yaw"]
        yaw_error = continuous_yaw_error(self.leader_yaw, follower_yaw)

        if abs(yaw_error) <= self.config["yaw_deadband_deg"]:
            self.pid.reset()
            rotate_speed = 0.0
            motor_duty = MotorControl.rotate(0)
        else:
            raw_speed = self.pid.compute(yaw_error, dt_s)
            rotate_speed = raw_speed * self.config["rotate_sign"]
            if 0.0 < abs(rotate_speed) < self.config["min_command_speed"]:
                if rotate_speed > 0.0:
                    rotate_speed = self.config["min_command_speed"]
                else:
                    rotate_speed = -self.config["min_command_speed"]
            motor_duty = MotorControl.rotate(rotate_speed)

        return {
            "has_leader_yaw": True,
            "leader_yaw": self.leader_yaw,
            "follower_yaw": follower_yaw,
            "yaw_error": yaw_error,
            "rotate_speed": rotate_speed,
            "motor_duty": motor_duty,
        }


def run_follow_leader_yaw_pid(config=None):
    if config is None:
        config = build_config()

    led_pin = Pin(config["led_pin"], Pin.OUT, value=1)
    status_led = C4StatusLed(led_pin, config)
    stop_switch = Pin(
        config["stop_switch_pin"],
        Pin.IN,
        pull=config["stop_switch_pull"],
    )
    stop_state = stop_switch.value()

    controller = FollowLeaderYawPID(config)

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
        print("=== CarB yaw follow PID ===")
        print("Leader car should keep sending ASCII line: YAW:<deg>")
        print("Keep the follower still before yaw follow starts; C4 status LED indicates link state.")
        print("Yaw deadband: +/-{} deg".format(config["yaw_deadband_deg"]))
        print(
            "PID kp={} ki={} kd={} max_rotate_speed={} rotate_sign={}".format(
                config["pid_kp"],
                config["pid_ki"],
                config["pid_kd"],
                config["max_rotate_speed"],
                config["rotate_sign"],
            )
        )

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
                            "leader={:>7} follower={:>7.2f} err={:>7} "
                            "rot={:>6.1f} duty={}"
                        ).format(
                            "--" if result["leader_yaw"] is None else "{:.2f}".format(result["leader_yaw"]),
                            result["follower_yaw"],
                            "--" if result["yaw_error"] is None else "{:.2f}".format(result["yaw_error"]),
                            result["rotate_speed"],
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
    run_follow_leader_yaw_pid()
