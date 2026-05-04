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

motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 15000, duty=0, invert=False)
motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_PWM_D5, 15000, duty=0, invert=False)
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_PWM_C31, 15000, duty=0, invert=False)

MAX_DUTY = 10000
MAX_SPEED = 100
MIN_DUTY_START = 1500
current_v1 = 0
current_v2 = 0
current_v3 = 0

Vx = 0
Vy = 0
omiga = 0
r = 0

def V1V2V3DSpeedToDuty(v1, v2, v3):
    v1 = max(-MAX_SPEED, min(MAX_SPEED, v1))
    v2 = max(-MAX_SPEED, min(MAX_SPEED, v2))
    v3 = max(-MAX_SPEED, min(MAX_SPEED, v3))

    duty1 = int(v1 * (MAX_DUTY / MAX_SPEED))
    duty2 = int(v2 * (MAX_DUTY / MAX_SPEED))
    duty3 = int(v3 * (MAX_DUTY / MAX_SPEED))

    for i in range(3):
        d = [duty1, duty2, duty3][i]
        if 0 < abs(d) < MIN_DUTY_START:
            sign = 1 if d > 0 else -1
            if i == 0:
                duty1 = sign * MIN_DUTY_START
            elif i == 1:
                duty2 = sign * MIN_DUTY_START
            else:
                duty3 = sign * MIN_DUTY_START

    return duty1, duty2, duty3

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

def forward(speed, acceleration=0, direction=0):
    global Vx, Vy, omiga, r
    ConvertVToVxVy(speed, 90)
    omiga = direction
    v1 = -Vx * math.sin(pi/6) - Vy * math.cos(pi/6) + omiga * r
    v2 = Vx + omiga * r
    v3 = -Vx * math.sin(pi/6) + Vy * math.cos(pi/6) + omiga * r
    duty_v1, duty_v2, duty_v3 = V1V2V3DSpeedToDuty(v1, v2, v3)
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)

def backward(speed, acceleration=0, direction=0):
    global Vx, Vy, omiga, r
    ConvertVToVxVy(speed, 270)
    omiga = direction
    v1 = -Vx * math.sin(pi/6) - Vy * math.cos(pi/6) + omiga * r
    v2 = Vx + omiga * r
    v3 = -Vx * math.sin(pi/6) + Vy * math.cos(pi/6) + omiga * r
    duty_v1, duty_v2, duty_v3 = V1V2V3DSpeedToDuty(v1, v2, v3)
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)

def move_left(speed, acceleration=0, direction=0):
    global Vx, Vy, omiga, r
    ConvertVToVxVy(speed, 180)
    omiga = direction
    v1 = -Vx * math.sin(pi/6) - Vy * math.cos(pi/6) + omiga * r
    v2 = Vx + omiga * r
    v3 = -Vx * math.sin(pi/6) + Vy * math.cos(pi/6) + omiga * r
    duty_v1, duty_v2, duty_v3 = V1V2V3DSpeedToDuty(v1, v2, v3)
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)

def move_right(speed, acceleration=0, direction=0):
    global Vx, Vy, omiga, r
    ConvertVToVxVy(speed, 0)
    omiga = direction
    v1 = -Vx * math.sin(pi/6) - Vy * math.cos(pi/6) + omiga * r
    v2 = Vx + omiga * r
    v3 = -Vx * math.sin(pi/6) + Vy * math.cos(pi/6) + omiga * r
    duty_v1, duty_v2, duty_v3 = V1V2V3DSpeedToDuty(v1, v2, v3)
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)

def stop(acceleration=0):
    if acceleration <= 0:
        _set_motors(0, 0, 0)
    else:
        _ramp_to(0, 0, 0, acceleration)

def _speed_to_duty(speed):
    return int(max(-MAX_SPEED, min(MAX_SPEED, speed)) * (MAX_DUTY / MAX_SPEED))

def rotate(speed, acceleration=0):
    duty = _speed_to_duty(speed)
    _ramp_to(duty, duty, duty, acceleration)

def rotate_ccw(speed, acceleration=0):
    rotate(speed, acceleration)

def rotate_cw(speed, acceleration=0):
    rotate(-speed, acceleration)

def move_angle(angle, speed, acceleration=0, direction=0):
    ConvertVToVxVy(speed, angle)
    v1 = -Vx * math.sin(pi/6) - Vy * math.cos(pi/6) + omiga * r
    v2 = Vx + omiga * r
    v3 = -Vx * math.sin(pi/6) + Vy * math.cos(pi/6) + omiga * r
    if direction != 0:
        rot = _speed_to_duty(direction * speed / 100)
        v1 += rot / (MAX_DUTY / MAX_SPEED)
        v2 += rot / (MAX_DUTY / MAX_SPEED)
        v3 += rot / (MAX_DUTY / MAX_SPEED)
    duty_v1, duty_v2, duty_v3 = V1V2V3DSpeedToDuty(v1, v2, v3)
    _ramp_to(duty_v1, duty_v2, duty_v3, acceleration)

def move_forward_left(speed, acceleration=0, direction=0):
    move_angle(135, speed, acceleration, direction)

def move_forward_right(speed, acceleration=0, direction=0):
    move_angle(45, speed, acceleration, direction)

def move_backward_left(speed, acceleration=0, direction=0):
    move_angle(225, speed, acceleration, direction)

def move_backward_right(speed, acceleration=0, direction=0):
    move_angle(315, speed, acceleration, direction)

def demo():
#     print("前进...")
#     forward(70, 100)
#     time.sleep_ms(1000)
#     print("停止...")
#     stop(100)
#     time.sleep_ms(500)
#     print("后退...")
#     backward(70, 100)
#     time.sleep_ms(1000)
#     print("停止...")
#     stop(100)
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

