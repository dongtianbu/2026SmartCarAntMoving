# -*- coding: utf-8 -*-
"""把 best_result.json 中的最佳参数一次性下发到 CarB，并可立即启动闭环。"""

from __future__ import annotations

import random
import time

import auto_tune_carb_pid as tune


# ---------------------------------------------------------------------------
# 人工调试区
# 所有需要现场人工改动的变量集中放在文件开头，优先只改这里。
# ---------------------------------------------------------------------------

START_AFTER_APPLY = True                 # True 时下发最佳参数后立即发送 AI_START，让从车直接跑这组参数
PRINT_PARAMETER_SUMMARY = True           # True 时在电脑端打印本次下发的最佳参数，方便现场确认
STATUS_CHECK_AFTER_START = True          # True 时启动后再读一次 AI_STATUS，确认从车已退出暂停状态
POST_START_STATUS_WAIT_S = 0.3           # 启动后等待多久再查询 AI_STATUS，给从车一点状态切换时间


def main() -> int:
    if tune.RANDOM_SEED is not None:
        random.seed(tune.RANDOM_SEED)

    snapshot = tune.load_saved_best_snapshot()
    if snapshot is None:
        raise RuntimeError(
            "没有找到可用的 best_result.json，无法下发最佳参数。"
            "请先运行一次自动调参脚本，或确认 {} 存在且内容完整。".format(tune.BEST_RESULT_JSON)
        )

    best_parameters, best_result = snapshot
    serial_module, list_ports_module = tune.load_pyserial()
    bridge = tune.open_bridge(serial_module, list_ports_module)
    try:
        tune.pause_carb_before_parameter_read(bridge)
        tune.require_ai_firmware(bridge)
        tune.apply_parameters(bridge, best_parameters)

        if PRINT_PARAMETER_SUMMARY:
            print("\n=== 已下发最佳参数 ===")
            for name, value in best_parameters.items():
                print("{}={}".format(name, tune.format_value(name, value)))
            print("历史最佳损失分数：{:.4f}".format(best_result.score))

        if START_AFTER_APPLY:
            tune.require_ok(
                tune.command_with_retry(
                    bridge,
                    "AI_START",
                    "AI_START",
                    timeout_s=tune.CONTROL_COMMAND_TIMEOUT_S,
                ),
                "AI_START",
            )
            print("已发送 AI_START，从车开始按最佳参数运行。")

            if STATUS_CHECK_AFTER_START:
                time.sleep(POST_START_STATUS_WAIT_S)
                status_reply = bridge.command("AI_STATUS", timeout_s=2.0)
                print("AI_STATUS: {}".format(status_reply))
        else:
            print("已完成最佳参数下发，当前未自动启动闭环。")
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
