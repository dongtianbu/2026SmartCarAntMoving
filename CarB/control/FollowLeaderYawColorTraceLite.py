"""CarB 低内存启动入口。

这个文件故意保持很小，只做两件事：
1. 提供 build_config；
2. 延迟导入真正的运行模块。

这样主程序在 import_follow_leader_lite 阶段只需要解析这一小层包装，
尽量避免因为一次性解析大文件而触发 MemoryError。
"""

import gc


def build_config(**overrides):
    """延迟导入配置模块，减少主程序导入阶段的峰值内存。"""
    gc.collect()
    from FollowLeaderYawColorTraceLiteConfig import build_config as _build_config
    gc.collect()
    return _build_config(**overrides)


def run_follow_leader_yaw_color_trace(config=None):
    """延迟导入运行模块，只有真正启动控制时才装载主体逻辑。"""
    gc.collect()
    from FollowLeaderYawColorTraceLiteRunner import run_follow_leader_yaw_color_trace as _run_follow_leader
    gc.collect()
    return _run_follow_leader(config)


if __name__ == "__main__":
    run_follow_leader_yaw_color_trace()
