"""CarA 上电启动入口。"""

from machine import Pin
import time

from control import MotorControl
from control.LeaderYawColorTrace import build_config, run_leader_yaw_color_trace


# ---------------------------------------------------------------------------
# 人工调参区
# 所有需要现场人工调试的变量都集中放在文件顶部，便于下载到车前快速确认。
# ---------------------------------------------------------------------------

MAIN_PROGRAM_MODE = "follow_trace"   # 主程序模式："square_test" 表示等待 C8 后执行方形运动测试；"follow_trace" 表示进入原主车跟随流程

D19_ENABLE_PIN = "D19"              # 主程序上电后需要立即拉到使能电平的引脚名
D19_ENABLE_LEVEL = 1                # D19 输出电平，1 表示高电平使能，0 表示低电平使能

START_KEY_PIN = "C8"                # 方形运动测试的启动按键引脚
START_KEY_PULL = Pin.PULL_UP_47K    # C8 按键上拉配置，未按下时为高电平
START_KEY_ACTIVE_LEVEL = 0          # C8 按下有效电平，当前为低电平有效
START_KEY_DEBOUNCE_MS = 30          # C8 消抖时间，单位毫秒
START_KEY_USE_EDGE_TRIGGER = True   # True 表示以上电空闲电平为基准，只要按键电平发生稳定翻转就触发，兼容不同接线方式
START_KEY_STATUS_PRINT_MS = 1000    # 等待 C8 期间的状态打印周期，单位毫秒；用于现场观察按键是否真的有电平变化

SQUARE_MOVE_SPEED = 72              # 方形运动测试的平移速度，范围建议 0~100
SQUARE_MOVE_DURATION_MS = 3000      # 每一段运动持续时间，单位毫秒；当前按需求设置为 2 秒
SQUARE_STOP_ACCELERATION = 0        # 最终停车时的减速斜坡；0 表示立即停车


def enable_d19_pin():
    """把 D19 配置为输出，并拉到使能电平。"""
    return Pin(D19_ENABLE_PIN, Pin.OUT, value=D19_ENABLE_LEVEL)


def wait_for_c8_press(start_key):
    """等待 C8 按下，加入简单消抖，避免误触发。"""
    idle_level = start_key.value()
    last_level = idle_level
    last_status_ms = time.ticks_ms()

    print("CarA 初始化完成，等待按下 {} 启动方形运动测试。".format(START_KEY_PIN))
    print(
        "{} 当前空闲电平={}，触发方式={}。".format(
            START_KEY_PIN,
            idle_level,
            "电平翻转" if START_KEY_USE_EDGE_TRIGGER else "固定有效电平 {}".format(START_KEY_ACTIVE_LEVEL),
        )
    )

    while True:
        current_level = start_key.value()
        now_ms = time.ticks_ms()

        if current_level != last_level:
            print("{} 电平变化：{} -> {}".format(START_KEY_PIN, last_level, current_level))
            last_level = current_level

        if START_KEY_USE_EDGE_TRIGGER:
            is_pressed = current_level != idle_level
        else:
            is_pressed = current_level == START_KEY_ACTIVE_LEVEL

        if is_pressed:
            time.sleep_ms(START_KEY_DEBOUNCE_MS)
            confirm_level = start_key.value()
            if START_KEY_USE_EDGE_TRIGGER:
                confirmed = confirm_level != idle_level
            else:
                confirmed = confirm_level == START_KEY_ACTIVE_LEVEL

            if confirmed:
                print("{} 已按下，开始执行方形运动测试。".format(START_KEY_PIN))
                return

        if time.ticks_diff(now_ms, last_status_ms) >= START_KEY_STATUS_PRINT_MS:
            print("等待 {} 中，当前电平={}".format(START_KEY_PIN, current_level))
            last_status_ms = now_ms

        time.sleep_ms(10)


def run_square_motion_test():
    """按前、右、后、左各运行 2 秒，最后停车。"""
    start_key = Pin(START_KEY_PIN, Pin.IN, pull=START_KEY_PULL)
    MotorControl.stop(0)
    wait_for_c8_press(start_key)

    print("第 1 段：向前运行 {} ms".format(SQUARE_MOVE_DURATION_MS))
    MotorControl.forward(SQUARE_MOVE_SPEED)
    time.sleep_ms(SQUARE_MOVE_DURATION_MS)

    print("第 2 段：向右运行 {} ms".format(SQUARE_MOVE_DURATION_MS))
    MotorControl.move_right(SQUARE_MOVE_SPEED)
    time.sleep_ms(SQUARE_MOVE_DURATION_MS)

    print("第 3 段：向后运行 {} ms".format(SQUARE_MOVE_DURATION_MS))
    MotorControl.backward(SQUARE_MOVE_SPEED)
    time.sleep_ms(SQUARE_MOVE_DURATION_MS)

    print("第 4 段：向左运行 {} ms".format(SQUARE_MOVE_DURATION_MS))
    MotorControl.move_left(SQUARE_MOVE_SPEED)
    time.sleep_ms(SQUARE_MOVE_DURATION_MS)

    MotorControl.stop(SQUARE_STOP_ACCELERATION)
    print("方形运动测试完成，主车已停车。")


CONFIG = build_config()
D19_ENABLE = enable_d19_pin()


def main():
    """主车入口：按模式选择执行方形运动测试或原有跟随流程。"""
    if MAIN_PROGRAM_MODE == "square_test":
        run_square_motion_test()
        return

    if MAIN_PROGRAM_MODE == "follow_trace":
        run_leader_yaw_color_trace(CONFIG)
        return

    raise ValueError("不支持的 MAIN_PROGRAM_MODE: {}".format(MAIN_PROGRAM_MODE))


main()
