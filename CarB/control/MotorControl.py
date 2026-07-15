from machine import *
from seekfree import MOTOR_CONTROLLER
import gc
import time
import math
from math import pi
# led = Pin('C4', Pin.OUT, value=True)
# switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
# state2 = switch2.value()

# MOTOR_CONTROLLER.help()
# time.sleep_ms(500)

# # 关键：用 PWM_D4_DIR_D5
# # motor = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 100, duty=0, invert=False)

# #该代码是驱动三个电机的示例代码
# # 构造接口 用于构建一个 MOTOR_CONTROLLER 对象
# #   index   电机索引    |   必要参数 [  PWM_C30_DIR_C31, PWM_C28_DIR_C29, PWM_D4_DIR_D5, PWM_D6_DIR_D7,
# #                       |               PWM_C30_PWM_C31, PWM_C28_PWM_C29, PWM_D4_PWM_D5, PWM_D6_PWM_D7]
# #   freq    信号频率    |   必要参数 PWM 信号的频率 范围是 [1 - 100000]
# #   duty    占空比值    |   可选参数 关键字参数 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# #   invert  反向设置    |   可选参数 关键字参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
# motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 15000, duty=0, invert=False)
# motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_PWM_D5, 15000, duty=0, invert=False)
# motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_PWM_C31, 15000, duty=0, invert=False)

# time.sleep_ms(500)

# motor_1.info()
# motor_2.info()
# motor_3.info()

led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 15000, duty=0, invert=False)
motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_PWM_D5, 15000, duty=0, invert=False)
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_PWM_C31, 15000, duty=0, invert=False)

MAX_DUTY = 10000
MAX_SPEED = 100
MIN_DUTY_START = 1500
FORWARD_DIRECTION_ANGLE = 270           # 车体“前进”对应的运动角度；当前装车后前后轴反了，因此前进改为 270 度
BACKWARD_DIRECTION_ANGLE = 90           # 车体“后退”对应的运动角度；需与 FORWARD_DIRECTION_ANGLE 保持相反
LEFT_DIRECTION_ANGLE = 180              # 车体“左移”对应的运动角度；若后续发现左右也反了，只需要修改这里
RIGHT_DIRECTION_ANGLE = 0               # 车体“右移”对应的运动角度；需与 LEFT_DIRECTION_ANGLE 保持相反
FORWARD_LEFT_DIRECTION_ANGLE = 225      # 左前方向；前后轴反向后，同步调整四个斜向角，避免组合运动定义不一致
FORWARD_RIGHT_DIRECTION_ANGLE = 315     # 右前方向
BACKWARD_LEFT_DIRECTION_ANGLE = 135     # 左后方向
BACKWARD_RIGHT_DIRECTION_ANGLE = 45     # 右后方向
current_v1 = 0
current_v2 = 0
current_v3 = 0
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
    return max(lower, min(upper, value))

def _scale_duty_tuple(duty_tuple, scale):
    """按统一比例缩放三路电机占空比。"""
    return tuple(int(round(duty_value * scale)) for duty_value in duty_tuple)

def _bias_single_duty(duty_value, bias_start, max_duty=MAX_DUTY):
    """对单路非零电机占空比做起转补偿。

    该函数用于处理“电机在较小占空比下不转”的实际情况。
    当 duty 非零时，会把其绝对值从 [0, max_duty] 重新映射到
    [bias_start, max_duty]，从而保证一旦需要转动，就直接越过电机死区。
    """
    duty_int = int(round(duty_value))
    if duty_int == 0:
        return 0

    bias_start = max(0, min(int(bias_start), int(max_duty)))
    if bias_start <= 0:
        return max(-max_duty, min(max_duty, duty_int))

    duty_sign = 1 if duty_int > 0 else -1
    duty_mag = min(int(max_duty), abs(duty_int))

    if bias_start >= max_duty:
        return duty_sign * max_duty

    biased_mag = bias_start + int(
        duty_mag * (max_duty - bias_start) / max_duty
    )
    return duty_sign * min(int(max_duty), biased_mag)

def bias_duty_tuple(duty_tuple, bias_start, max_duty=MAX_DUTY):
    """对三路电机占空比统一做起转补偿。"""
    return tuple(
        _bias_single_duty(duty_value, bias_start, max_duty=max_duty)
        for duty_value in duty_tuple
    )

def chassis_vector_to_wheel_speed(vx, vy, omega=0):
    return tuple(
        (mix_x * vx) + (mix_y * vy) + omega
        for mix_x, mix_y in WHEEL_MIXING
    )

def V1V2V3DSpeedToDuty(v1, v2, v3, max_duty=MAX_DUTY, min_duty_start=MIN_DUTY_START):
    speeds = [float(v1), float(v2), float(v3)]
    max_abs_speed = max(abs(speed) for speed in speeds)
    if max_abs_speed > MAX_SPEED and max_abs_speed > 0:
        scale = MAX_SPEED / max_abs_speed
        speeds = [speed * scale for speed in speeds]

    duties = [speed * (max_duty / MAX_SPEED) for speed in speeds]
    non_zero_duties = [abs(duty) for duty in duties if abs(duty) > 1e-6]
    if not non_zero_duties:
        return 0, 0, 0

    scale_up = 1.0
    min_non_zero = min(non_zero_duties)
    if min_non_zero < min_duty_start:
        scale_up = min_duty_start / min_non_zero

    max_scaled = max(abs(duty) * scale_up for duty in duties)
    scale_down = 1.0
    if max_scaled > max_duty:
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
    return V1V2V3DSpeedToDuty(
        *chassis_vector_to_wheel_speed(vx, vy, omega),
        max_duty=max_duty,
        min_duty_start=min_duty_start
    )

def _set_motors(v1, v2, v3):
    global current_v1, current_v2, current_v3
    v1 = max(-MAX_DUTY, min(MAX_DUTY, int(v1)))
    v2 = max(-MAX_DUTY, min(MAX_DUTY, int(v2)))
    v3 = max(-MAX_DUTY, min(MAX_DUTY, int(v3)))
    motor_1.duty(v1)
    motor_2.duty(v2)
    motor_3.duty(v3)
    current_v1, current_v2, current_v3 = v1, v2, v3

def _ramp_to(target_v1, target_v2, target_v3, acceleration):
    global current_v1, current_v2, current_v3
    if acceleration <= 0:
        _set_motors(target_v1, target_v2, target_v3)
        return
    step = acceleration
    while abs(current_v1 - target_v1) > step or abs(current_v2 - target_v2) > step or abs(current_v3 - target_v3) > step:
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
    #theta only can be 0~360
    global Vx, Vy
    rad = math.radians(theta)
    Vx = VSpeed * math.cos(rad)
    Vy = VSpeed * math.sin(rad)
    return Vx, Vy

def drive_vector(
    vx,
    vy,
    omega=0,
    acceleration=0,
    max_duty=MAX_DUTY,
    min_duty_start=MIN_DUTY_START,
    duty_bias_start=0,
    translate_duty_bias_start=None,
    rotate_duty_bias_start=None,
):
    """把平移和旋转分别换算后再合成最终三路电机占空比。

    这样可以让平移分量和旋转分量使用不同的起转补偿。
    当前 CarB 跟随控制中，平移需要越过死区，旋转先不补偿。
    注意：
    - 这里的 `max_duty` 限制的是三路混合后“单个电机”的 duty 绝对值上限；
    - 不是三路 duty 相加后的总和上限；
    - 如果三路相加后某一路超过 `max_duty`，会把三路按同一比例整体缩小。
    """
    if translate_duty_bias_start is None:
        translate_duty_bias_start = duty_bias_start
    if rotate_duty_bias_start is None:
        rotate_duty_bias_start = duty_bias_start

    translate_duty = vector_to_duty(
        vx,
        vy,
        omega=0,
        max_duty=max_duty,
        min_duty_start=min_duty_start
    )
    rotate_duty = vector_to_duty(
        0,
        0,
        omega=omega,
        max_duty=max_duty,
        min_duty_start=0,
    )

    if translate_duty_bias_start > 0:
        translate_duty = bias_duty_tuple(
            translate_duty,
            translate_duty_bias_start,
            max_duty=max_duty,
        )

    if rotate_duty_bias_start > 0:
        rotate_duty = bias_duty_tuple(
            rotate_duty,
            rotate_duty_bias_start,
            max_duty=max_duty,
        )

    duty_v1 = translate_duty[0] + rotate_duty[0]
    duty_v2 = translate_duty[1] + rotate_duty[1]
    duty_v3 = translate_duty[2] + rotate_duty[2]

    max_abs_duty = max(abs(duty_v1), abs(duty_v2), abs(duty_v3))
    if max_abs_duty > max_duty and max_abs_duty > 0:
        duty_v1, duty_v2, duty_v3 = _scale_duty_tuple(
            (duty_v1, duty_v2, duty_v3),
            max_duty / max_abs_duty,
        )

    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)
    return duty_v1, duty_v2, duty_v3

def forward(speed, acceleration=0, direction=0):
    vx, vy = ConvertVToVxVy(speed, FORWARD_DIRECTION_ANGLE)
    return drive_vector(vx, vy, acceleration=acceleration)

def backward(speed, acceleration=0, direction=0):
    vx, vy = ConvertVToVxVy(speed, BACKWARD_DIRECTION_ANGLE)
    return drive_vector(vx, vy, acceleration=acceleration)

def move_left(speed, acceleration=0, direction=0):
    vx, vy = ConvertVToVxVy(speed, LEFT_DIRECTION_ANGLE)
    return drive_vector(vx, vy, acceleration=acceleration)

def move_right(speed, acceleration=0, direction=0):
    vx, vy = ConvertVToVxVy(speed, RIGHT_DIRECTION_ANGLE)
    return drive_vector(vx, vy, acceleration=acceleration)

def stop(acceleration=0):
    if acceleration <= 0:
        _set_motors(0, 0, 0)
    else:
        _ramp_to(0, 0, 0, acceleration)

def _speed_to_duty(speed):
    return int(max(-MAX_SPEED, min(MAX_SPEED, speed)) * (MAX_DUTY / MAX_SPEED))

def rotate(speed, acceleration=0):
    return drive_vector(0, 0, omega=_clamp(speed, -MAX_SPEED, MAX_SPEED), acceleration=acceleration)

def rotate_ccw(speed, acceleration=0):
    rotate(speed, acceleration)

def rotate_cw(speed, acceleration=0):
    rotate(-speed, acceleration)

def move_angle(angle, speed, acceleration=0, direction=0):
    vx, vy = ConvertVToVxVy(speed, angle)
    omega = _clamp(direction, -MAX_SPEED, MAX_SPEED)
    return drive_vector(vx, vy, omega=omega, acceleration=acceleration)

def move_forward_left(speed, acceleration=0, direction=0):
    move_angle(FORWARD_LEFT_DIRECTION_ANGLE, speed, acceleration, direction)

def move_forward_right(speed, acceleration=0, direction=0):
    move_angle(FORWARD_RIGHT_DIRECTION_ANGLE, speed, acceleration, direction)

def move_backward_left(speed, acceleration=0, direction=0):
    move_angle(BACKWARD_LEFT_DIRECTION_ANGLE, speed, acceleration, direction)

def move_backward_right(speed, acceleration=0, direction=0):
    move_angle(BACKWARD_RIGHT_DIRECTION_ANGLE, speed, acceleration, direction)

def demo():
    print("前进...")
    forward(70, 100)
    time.sleep_ms(1000)
    print("停止...")
    stop(100)
    time.sleep_ms(500)
    print("后退...")
    backward(70, 100)
    time.sleep_ms(1000)
    print("停止...")
    stop(100)
    time.sleep_ms(500)
    print("左移...")
    move_left(60, 100)
    time.sleep_ms(1000)
    print("停止...")
    stop(100)
    time.sleep_ms(500)
    print("右移...")
    move_right(60, 100)
    time.sleep_ms(1000)
    print("停止...")
    stop(100)
    time.sleep_ms(500)
    print("逆时针旋转...")
    rotate_ccw(60, 100)
    time.sleep_ms(1000)
    print("停止...")
    stop(100)
    print("演示结束")

if __name__ == "__main__":
    print("全向轮三轮小车运动控制模块")
    print("可用函数:")
    print("  forward(speed, acceleration, direction)  - 前进")
    print("  backward(speed, acceleration, direction) - 后退")
    print("  move_left(speed, acceleration, direction)  - 左移")
    print("  move_right(speed, acceleration, direction) - 右移")
    print("  stop(acceleration) - 停止(acceleration=0立即停止)")
    print("  rotate(speed, acceleration) - 旋转(正=逆时针,负=顺时针)")
    print("  move_angle(angle, speed, acceleration, direction) - 任意角度移动")
    print("  demo() - 运动演示")
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


# #方向1从电机屁股看过去是逆时针旋转
# #1电机是全向轮三轮小车顶部电机（三轮小车按正三角形底边水平摆放）
# #从1号电机逆时针数，分别是2号、3号电机
# motor_dir = 1 
# motor_duty = 0
# motor_duty_max = 5500

# motor_1.duty(motor_duty_max)
# motor_2.duty(motor_duty_max)
# motor_3.duty(motor_duty_max)
# speed:    0 ~ 100   （内部 clamp，超出无效）
#           60 = 推荐（6V电机/12V电池）
#           80 = 较快（注意发热）
# 
# acceleration: 0 ~ 10000
#               0   = 立即到达（无缓冲）
#               200 = 平滑加速（每10ms步进200 duty）
#               500 = 慢速起步
# 
# 对应实际 PWM 占空比 = speed × 100：
#   speed=60  → duty=±6000  → 电压≈7.2V
#   speed=70  → duty=±7000  → 电压≈8.4V
#   speed=100 → duty=±10000 → 电压=12V（满压，慎用）
