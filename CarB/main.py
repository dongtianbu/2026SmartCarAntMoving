"""CarB 上电启动入口。"""

import gc
import sys
import time
from machine import Pin


# ---------------------------------------------------------------------------
# 人工调试区
# 所有现场可能需要人工修改的启动参数统一放在文件开头，便于脱机排障时快速调整。
# ---------------------------------------------------------------------------

MODULE_DIRS = ("control", "connection", "driver")   # CarB 运行依赖的模块目录
BOOT_ERROR_LOG_PATHS = (                            # 脱机启动失败时依次尝试写入的日志文件路径
    "/carb_boot_error.txt",
    "carb_boot_error.txt",
    "/boot_error.txt",
    "boot_error.txt",
)
BOOT_STATUS_LED_PIN = "C4"                          # 启动失败时用于提示的状态灯引脚
BOOT_STATUS_LED_ACTIVE_LEVEL = 0                    # C4 点亮电平；当前硬件为低电平点亮
BOOT_ERROR_BLINK_MS = 120                           # 启动失败后的错误快闪周期，单位 ms
BOOT_IMPORT_GC_COLLECT_TIMES = 2                    # 导入主控制模块前主动执行垃圾回收的次数
BOOT_AUTO_GC_THRESHOLD_ENABLED = True               # 是否在启动早期配置自动 GC 阈值
BOOT_AUTO_GC_THRESHOLD_DIVISOR = 4                  # 自动 GC 阈值按当前空闲内存的 1/N 计算


BOOT_STAGE = "power_on"                             # 当前启动阶段；用于脱机排障时定位卡死位置


def _set_boot_stage(stage_name):
    """记录当前启动阶段，异常时一起写入日志。"""
    global BOOT_STAGE
    BOOT_STAGE = stage_name


def _ensure_module_paths():
    """同时加入相对路径和绝对路径，兼容 Thonny 运行与脱机上电两种环境。"""
    for module_dir in MODULE_DIRS:
        for candidate in (module_dir, "/" + module_dir):
            if candidate not in sys.path:
                sys.path.append(candidate)


def _configure_gc():
    """尽早配置垃圾回收阈值，减少大模块导入时的碎片化风险。"""
    gc.collect()
    if not BOOT_AUTO_GC_THRESHOLD_ENABLED:
        return
    if not hasattr(gc, "threshold"):
        return
    if not hasattr(gc, "mem_free") or not hasattr(gc, "mem_alloc"):
        return

    divisor = max(1, int(BOOT_AUTO_GC_THRESHOLD_DIVISOR))
    threshold = gc.mem_alloc() + max(1024, gc.mem_free() // divisor)
    gc.threshold(threshold)


def _log_boot_error(exc):
    """把脱机启动异常写到板子根目录，便于断电重启后在 Thonny 中读取。"""
    log_text_lines = [
        "CarB boot failed",
        "stage={}".format(BOOT_STAGE),
        "exception={}".format(repr(exc)),
        "traceback:",
    ]

    for log_path in BOOT_ERROR_LOG_PATHS:
        try:
            with open(log_path, "w") as handle:
                for line in log_text_lines:
                    handle.write(line + "\n")
                if hasattr(sys, "print_exception"):
                    sys.print_exception(exc, handle)
                else:
                    handle.write("print_exception unavailable\n")
            return
        except Exception:
            pass

    try:
        print("CarB boot failed")
        print("stage={}".format(BOOT_STAGE))
        if hasattr(sys, "print_exception"):
            sys.print_exception(exc)
        else:
            print(repr(exc))
    except Exception:
        pass


def _blink_boot_error_forever():
    """启动失败时持续快闪 C4，让现场能直接看出主程序没有正常进入。"""
    inactive_level = 0 if BOOT_STATUS_LED_ACTIVE_LEVEL else 1
    led = Pin(BOOT_STATUS_LED_PIN, Pin.OUT, value=inactive_level)
    while True:
        led.value(BOOT_STATUS_LED_ACTIVE_LEVEL)
        time.sleep_ms(BOOT_ERROR_BLINK_MS)
        led.value(inactive_level)
        time.sleep_ms(BOOT_ERROR_BLINK_MS)


def main():
    """初始化导入环境并启动 CarB 主控制程序。"""
    _set_boot_stage("ensure_module_paths")
    _ensure_module_paths()

    _set_boot_stage("configure_gc")
    _configure_gc()

    _set_boot_stage("pre_import_gc")
    for _ in range(BOOT_IMPORT_GC_COLLECT_TIMES):
        gc.collect()

    _set_boot_stage("import_follow_leader_lite")
    # CarB 板端内存较紧，脱机启动时优先导入轻量组合控制入口。
    # 旧版 FollowLeaderYawColorTrace.py 文件较大，在 MicroPython 解析阶段容易触发 MemoryError。
    from FollowLeaderYawColorTraceLite import build_config, run_follow_leader_yaw_color_trace

    _set_boot_stage("build_config")
    config = build_config()

    _set_boot_stage("run_follow_leader")
    run_follow_leader_yaw_color_trace(config)


try:
    main()
except Exception as exc:
    _log_boot_error(exc)
    _blink_boot_error_forever()
