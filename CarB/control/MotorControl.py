"""三轮全向底盘运动控制。

这是 CarA 底盘运动最核心、也最适合直接调用的模块。

外部最常直接调用这些函数：
- `forward(speed)`
- `backward(speed)`
- `move_left(speed)`
- `move_right(speed)`
- `move_angle(angle, speed)`
- `rotate(speed)`
- `stop()`

如果要做更复杂控制，推荐直接调用：
- `drive_vector(vx, vy, omega=0)`
- `vector_to_duty(vx, vy, omega=0)`
"""

from machine import *
from seekfree import MOTOR_CONTROLLER
import gc
import time
import math
from math import pi


# 这些电机对象在模块加载时创建，导入 `MotorControl` 的其他模块可直接复用。
# 底层硬件对象：本文件直接持有 3 个电机实例。
# 这样其他模块导入 `MotorControl` 后，可以直接复用这些电机对象。
led = Pin("C4", Pin.OUT, value=True)
switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 15000, duty=0, invert=False)
motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_PWM_D5, 15000, duty=0, invert=False)
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_PWM_C31, 15000, duty=0, invert=False)


# 这些常量决定了“速度值”和“PWM 占空比”之间的换算关系。
MAX_DUTY = 10000
MAX_SPEED = 100
MIN_DUTY_START = 1500

current_v1 = 0
current_v2 = 0
current_v3 = 0

# 每个元组表示一个轮子在底盘坐标系下对 x/y 方向速度的贡献。
# 三个轮子的速度混合矩阵。
# 输入是底盘速度 `(vx, vy, omega)`，
# 输出是三个轮子的目标速度 `(v1, v2, v3)`。
WHEEL_MIXING = (
    (-0.5, -math.cos(pi / 6)),
    (1.0, 0.0),
    (-0.5, math.cos(pi / 6)),
)

Vx = 0
Vy = 0
omiga = 0
r = 0


def _clamp(value, lower, upper):
    """内部工具函数：把数值限制在指定范围内。"""
    return max(lower, min(upper, value))


def chassis_vector_to_wheel_speed(vx, vy, omega=0):
    """把底盘速度转换成三个轮子的目标速度。

    - `vx`：左右速度
    - `vy`：前后速度
    - `omega`：旋转速度
    """
    return tuple(
        (mix_x * vx) + (mix_y * vy) + omega
        for mix_x, mix_y in WHEEL_MIXING
    )


def V1V2V3DSpeedToDuty(v1, v2, v3, max_duty=MAX_DUTY, min_duty_start=MIN_DUTY_START):
    """把三个轮子的目标速度换算成可直接下发的 PWM 占空比。

    这里做了两件重要的事：
    1. 超过最大速度时自动整体缩放；
    2. 低速时抬到最小启动占空比，避免电机不转。
    """
    speeds = [float(v1), float(v2), float(v3)]
    max_abs_speed = max(abs(speed) for speed in speeds)
    if max_abs_speed > MAX_SPEED and max_abs_speed > 0:
        # 整体等比例缩放，保证方向不变，同时把速度拉回允许范围。
        scale = MAX_SPEED / max_abs_speed
        speeds = [speed * scale for speed in speeds]

    duties = [speed * (max_duty / MAX_SPEED) for speed in speeds]
    non_zero_duties = [abs(duty) for duty in duties if abs(duty) > 1e-6]
    if not non_zero_duties:
        return 0, 0, 0

    scale_up = 1.0
    min_non_zero = min(non_zero_duties)
    if min_non_zero < min_duty_start:
        # 小占空比需要抬高一点，帮助电机克服静摩擦顺利起转。
        scale_up = min_duty_start / min_non_zero

    max_scaled = max(abs(duty) * scale_up for duty in duties)
    scale_down = 1.0
    if max_scaled > max_duty:
        # 抬高后再做一次限幅，避免最强的那个轮子超出 PWM 上限。
        scale_down = max_duty / max_scaled

    final_scale = scale_up * scale_down
    duty_values = []
    for duty in duties:
        if abs(duty) <= 1e-6:
            duty_values.append(0)
        else:
            duty_values.append(int(round(_clamp(duty * final_scale, -max_duty, max_duty))))

    return tuple(duty_values)


def vector_to_duty(vx, vy, omega=0, max_duty=MAX_DUTY, min_duty_start=MIN_DUTY_START):
    """从底盘速度直接得到三个轮子的占空比。"""
    return V1V2V3DSpeedToDuty(
        *chassis_vector_to_wheel_speed(vx, vy, omega),
        max_duty=max_duty,
        min_duty_start=min_duty_start
    )


def _set_motors(v1, v2, v3):
    """内部函数：立刻把三个 PWM 值下发到电机。"""
    global current_v1, current_v2, current_v3
    v1 = max(-MAX_DUTY, min(MAX_DUTY, int(v1)))
    v2 = max(-MAX_DUTY, min(MAX_DUTY, int(v2)))
    v3 = max(-MAX_DUTY, min(MAX_DUTY, int(v3)))
    motor_1.duty(v1)
    motor_2.duty(v2)
    motor_3.duty(v3)
    current_v1, current_v2, current_v3 = v1, v2, v3


def _ramp_to(target_v1, target_v2, target_v3, acceleration):
    """内部函数：按步进慢慢逼近目标占空比，避免瞬间冲击。"""
    global current_v1, current_v2, current_v3
    if acceleration <= 0:
        _set_motors(target_v1, target_v2, target_v3)
        return
    step = acceleration
    while (
        abs(current_v1 - target_v1) > step
        or abs(current_v2 - target_v2) > step
        or abs(current_v3 - target_v3) > step
    ):
        if current_v1 < target_v1:
            current_v1 = min(current_v1 + step, target_v1)
        elif current_v1 > target_v1:
            current_v1 = max(current_v1 - step, target_v1)
        if current_v2 < target_v2:
            current_v2 = min(current_v2 + step, target_v2)
        elif current_v2 > target_v2:
            current_v2 = max(current_v2 - step, target_v2)
        if current_v3 < target_v3:
            current_v3 = min(current_v3 + step, target_v3)
        elif current_v3 > target_v3:
            current_v3 = max(current_v3 - step, target_v3)
        _set_motors(current_v1, current_v2, current_v3)
        time.sleep_ms(10)
    _set_motors(target_v1, target_v2, target_v3)


def ConvertVToVxVy(VSpeed, theta):
    """把“速度 + 方向角”转换成 `(vx, vy)`。"""
    global Vx, Vy
    rad = math.radians(theta)
    Vx = VSpeed * math.cos(rad)
    Vy = VSpeed * math.sin(rad)
    return Vx, Vy


def drive_vector(vx, vy, omega=0, acceleration=0, max_duty=MAX_DUTY, min_duty_start=MIN_DUTY_START):
    """底盘控制总入口之一。

    只要已经知道期望底盘速度，就优先直接调用这个函数。
    """
    # 先把底盘运动换算成轮速和 PWM，再按需要平滑地下发到电机。
    duty_v1, duty_v2, duty_v3 = vector_to_duty(
        vx,
        vy,
        omega=omega,
        max_duty=max_duty,
        min_duty_start=min_duty_start
    )
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)
    return duty_v1, duty_v2, duty_v3


def forward(speed, acceleration=0, direction=0):
    """直接前进。"""
    vx, vy = ConvertVToVxVy(speed, 90)
    return drive_vector(vx, vy, acceleration=acceleration)


def backward(speed, acceleration=0, direction=0):
    """直接后退。"""
    vx, vy = ConvertVToVxVy(speed, 270)
    return drive_vector(vx, vy, acceleration=acceleration)


def move_left(speed, acceleration=0, direction=0):
    """直接左移。"""
    vx, vy = ConvertVToVxVy(speed, 180)
    return drive_vector(vx, vy, acceleration=acceleration)


def move_right(speed, acceleration=0, direction=0):
    """直接右移。"""
    vx, vy = ConvertVToVxVy(speed, 0)
    return drive_vector(vx, vy, acceleration=acceleration)


def stop(acceleration=0):
    """停车。"""
    if acceleration <= 0:
        _set_motors(0, 0, 0)
    else:
        # 停车也走同一套渐变流程，手感会和加速保持一致。
        _ramp_to(0, 0, 0, acceleration)


def _speed_to_duty(speed):
    """内部工具函数：把单个速度值映射成占空比。"""
    return int(max(-MAX_SPEED, min(MAX_SPEED, speed)) * (MAX_DUTY / MAX_SPEED))


def rotate(speed, acceleration=0):
    """原地旋转；正值/负值代表两个相反方向。"""
    return drive_vector(0, 0, omega=_clamp(speed, -MAX_SPEED, MAX_SPEED), acceleration=acceleration)


def rotate_ccw(speed, acceleration=0):
    """逆时针旋转。"""
    rotate(speed, acceleration)


def rotate_cw(speed, acceleration=0):
    """顺时针旋转。"""
    rotate(-speed, acceleration)


def move_angle(angle, speed, acceleration=0, direction=0):
    """沿任意角度移动，同时可叠加一点旋转量。"""
    vx, vy = ConvertVToVxVy(speed, angle)
    omega = _clamp(direction, -MAX_SPEED, MAX_SPEED)
    return drive_vector(vx, vy, omega=omega, acceleration=acceleration)


def move_forward_left(speed, acceleration=0, direction=0):
    """左前方斜向移动。"""
    move_angle(135, speed, acceleration, direction)


def move_forward_right(speed, acceleration=0, direction=0):
    """右前方斜向移动。"""
    move_angle(45, speed, acceleration, direction)


def move_backward_left(speed, acceleration=0, direction=0):
    """左后方斜向移动。"""
    move_angle(225, speed, acceleration, direction)


def move_backward_right(speed, acceleration=0, direction=0):
    """右后方斜向移动。"""
    move_angle(315, speed, acceleration, direction)


def demo():
    """演示若干基础动作，主要用于单板联调。"""
    time.sleep_ms(500)
    print("Move left...")
    move_left(60, 100)
    time.sleep_ms(1000)
    print("Stop...")
    stop(100)
    time.sleep_ms(500)
    print("Move right...")
    move_right(60, 100)
    time.sleep_ms(1000)
    print("Stop...")
    stop(100)
    time.sleep_ms(500)
    print("Rotate CCW...")
    rotate_ccw(60, 100)
    time.sleep_ms(1000)
    print("Stop...")
    stop(100)
    print("Demo finished")


if __name__ == "__main__":
    # 直接运行这个文件时，会把最常用控制接口打印出来，并执行一个简单演示。
    print("Omni-wheel motor control module")
    print("Available functions:")
    print("  forward(speed, acceleration, direction)")
    print("  backward(speed, acceleration, direction)")
    print("  move_left(speed, acceleration, direction)")
    print("  move_right(speed, acceleration, direction)")
    print("  stop(acceleration)")
    print("  rotate(speed, acceleration)")
    print("  move_angle(angle, speed, acceleration, direction)")
    print("  demo()")
    time.sleep_ms(1000)
    demo()
    while True:
        led.toggle()
        if switch2.value() != state2:
            print("Test program stop.")
            stop(0)
            break
        gc.collect()
        time.sleep_ms(100)


# 经验说明：
# - `speed` 推荐先从 40~60 开始试；
# - `acceleration` 越大，变化越平滑，但响应越慢；
# - 业务代码里最常用的还是 `forward / move_left / move_angle / rotate / stop`。
