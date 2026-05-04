
# 本示例程序演示如何使用 seekfree 库的 IMU660RA 类接口读取陀螺仪数据
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 IMU660RA 模块测试
#
# 姿态传感器引脚 (LPSPI2 / SPI id=1):
#   SCK  = C10
#   MOSI = C12
#   MISO = C13
#   CS   = C11
#
# 示例程序运行效果为每 200ms(0.2s) 通过 Type-C 的 CDC 虚拟串口输出陀螺仪数据
# 当 D9 引脚电平出现变化时退出测试程序
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 IMU660RA 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 IMU660RA
from seekfree import IMU660RA

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 D9 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 显示帮助信息
IMU660RA.help()
time.sleep_ms(500)

# 构造接口 用于构建一个 IMU660RA 对象
#   IMU660RA([capture_div])
#   capture_div 采集分频    |   非必要参数 默认为 1 也就是每次都采集 代表多少次触发进行一次采集
imu = IMU660RA()

# 其余接口：
# IMU660RA.capture()    # 执行一次 IMU 数据采集触发 达到触发数时执行采集并将数据缓存
# IMU660RA.get()        # 输出当前采集缓存的 IMU 数据
# IMU660RA.read()       # 立即进行一次 capture 并输出缓存数据
# IMU660RA.help()       # 可以直接通过类调用 也可以通过对象调用 输出模块的使用帮助信息
# IMU660RA.info()       # 通过对象调用 输出当前对象的自身信息

imu.info()
time.sleep_ms(500)

# 通过 get 接口读取数据
# 本质上是将 Python 对象与传感器数据缓冲区链接起来
# 所以只需要一次 IMU660RA.get() 后就不需要再调用这个接口
# 之后直接使用获取的列表对象即可 它的数据会随 caputer 更新
# 返回列表: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
imu_data = imu.get()

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time_instance):
    global ticker_flag
    global ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
pit1.capture_list(imu)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒 数据更新周期 = 10ms * 1 = 10ms
pit1.start(10)

while True:
    if (ticker_flag and ticker_count % 20 == 0):
        # 翻转 C4 LED 电平
        led.toggle()
        # imu_data: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        print("gyro = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))
        print("acc  = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
        ticker_flag = False

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break

    # 回收内存
    gc.collect()

# 如何换算 IMU 数据到物理数值
# 需要在模块的资料中找到对应芯片的手册
# 手册中通常会标注芯片的 Sensitivity 灵敏度
#
# 以陀螺仪为例 其手册描述可能是 LSB/dps 或者 mdps/LSB
# LSB/dps 代表多少数值变化对应每秒一度的角速度变化
# mdps/LSB 代表每 0.001 度/秒的角速度变化对应多少数值变化
# 假设手册描述的是 ±2000dps 量程下灵敏度为 16.4 LSB/dps
# 那么假设获取的陀螺仪原始值为 1640 此时对应的换算方式为
# 1640 / 16.4 = 100 dps (度/秒)
