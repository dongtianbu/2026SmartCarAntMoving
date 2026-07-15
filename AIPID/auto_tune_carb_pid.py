# -*- coding: utf-8 -*-
"""CarB 电脑端全自动闭环调参脚本。

使用流程：
1. 先把 CarB 最新代码下载到从车。
2. 给从车上电，等待 IMU 校准结束。
3. 在电脑运行本脚本，脚本会自动找串口、发送停止/参数/主动偏离/开始指令。
4. 脚本根据 CarB 回传的 AI_METRIC 数据评分，找到更稳定的参数后写回 CarB 控制代码。
"""

from __future__ import annotations

import csv
import json
import math
import random
import re
import statistics
import sys
import time
import urllib.error
import urllib.request
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
import threading


def detect_app_dir() -> Path:
    """返回当前程序的运行目录；源码模式取脚本目录，打包模式取 exe 所在目录。"""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


def detect_project_root(app_dir: Path) -> Path:
    """尽量自动定位项目根目录，便于打包后仍能回写 CarB 源码。"""
    candidates = [
        Path.cwd(),
        app_dir,
        app_dir.parent,
        app_dir.parent.parent,
        Path(__file__).resolve().parents[1],
    ]
    checked = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in checked:
            continue
        checked.add(candidate)
        if (candidate / "CarB" / "control" / "FollowLeaderYawColorTrace.py").exists():
            return candidate
    return Path(__file__).resolve().parents[1]


APP_DIR = detect_app_dir()                 # AIPID 源码目录，或打包后 exe 所在目录
PROJECT_ROOT_DIR = detect_project_root(APP_DIR)  # 项目根目录，供自动回写 CarB 参数使用


# ---------------------------------------------------------------------------
# 人工调试区
# 所有需要人工改动的变量集中放在文件开头，现场调试时优先只改这里。
# ---------------------------------------------------------------------------

SERIAL_PORT = ""                         # 指定电脑串口号，例如 "COM5"；留空时脚本会自动探测
PORT_KEYWORD = ""                        # 自动探测串口时的设备名关键词，例如 "CH340"、"USB"；留空则不按名称过滤
BAUDRATE = 115200                        # 必须与 CarB/control/FollowLeaderYawColorTrace.py 中 YAW_BAUDRATE 保持一致
SERIAL_BYTESIZE = 8                      # 串口数据位默认值；上位机默认使用 8 位数据位
SERIAL_STOPBITS = 1.0                    # 串口停止位默认值；上位机默认使用 1 位停止位
SERIAL_PARITY = "N"                      # 串口校验位默认值；N=无校验，E=偶校验，O=奇校验，M=Mark，S=Space
SERIAL_FLOW_CONTROL = "none"             # 串口流控默认值；支持 "none"、"xonxoff"、"rtscts"、"dsrdtr"
SERIAL_TIMEOUT_S = 0.08                  # 串口单次读取超时时间，过大影响响应速度，过小可能读不到完整行
PORT_PROBE_SECONDS = 3.0                 # 每个候选串口等待 CarB 回包的最长时间
READY_WAIT_SECONDS = 60.0                # 等待 CarB 完成启动并输出 TUNE READY/AI_METRIC 的最长时间
COMMAND_DRAIN_SECONDS = 0.15             # 每次发送控制命令前清理旧回包的时间，避免串口残留数据导致命令错位
COMMAND_RETRY_ATTEMPTS = 5               # 参数或控制命令空回包时的最多重试次数，用于抵抗无线串口偶发丢包
PARAMETER_COMMAND_TIMEOUT_S = 4.0        # 单个参数读写命令等待回包的最长时间，过短容易误判为空回包
CONTROL_COMMAND_TIMEOUT_S = 3.0          # AI_STOP、AI_START、AI_PERTURB 等控制命令等待回包的最长时间
REQUIRE_AI_FIRMWARE = True               # True 时要求 CarB 支持 AI_STATUS/AI_PERTURB；检测到旧代码会立即停止并提示重新下载
FAILED_TRIAL_CONTINUE = True             # True 时某一轮参数下发/控制命令失败只给大惩罚分并继续搜索，不让脚本整体退出
INITIAL_STOP_SETTLE_SECONDS = 0.8        # 连接成功后先发送 AI_STOP 并等待这段时间，让从车停止发送 AI_METRIC 后再读取参数
INITIAL_DRAIN_SECONDS = 0.8              # 初始停车后继续清理串口残留数据的时间，避免旧 AI_METRIC 混入 GET 回包
ONLY_SEND_CHANGED_PARAMETERS = True      # True 时每轮只发送相对当前车上状态发生变化的参数，显著减少无线串口命令拥堵
PARAMETER_COMMAND_GAP_SECONDS = 0.08     # 两条参数命令之间的最小间隔，避免无线串口连续写入过快造成丢包

STOP_SETTLE_SECONDS = 0.8                # 每轮试验前发送 AI_STOP 后等待车辆完全停稳的时间
PRETRIAL_RANDOM_PERTURB_ENABLED = True   # True 时每轮试验前先做随机开环扰动；当前默认只保留平移扰动
RANDOM_SEED = None                       # 随机扰动种子；填整数可复现实验，保持 None 则每次运行随机
PERTURB_TRANSLATE_SPEED_MIN = 16.0       # 随机平移扰动最小速度，对应底盘 speed 标度 0~100
PERTURB_TRANSLATE_SPEED_MAX = 28.0       # 随机平移扰动最大速度，过大会让目标脱出视野
PERTURB_ROTATE_ENABLED = False           # False 时关闭每轮试验前的主动旋转扰动，只保留随机方向平移扰动
PERTURB_ROTATE_SPEED_MIN = 52.0          # 随机 yaw 扰动最小旋转速度，对应底盘 speed 标度 0~100，需高于电机旋转死区
PERTURB_ROTATE_SPEED_MAX = 65.0          # 随机 yaw 扰动最大旋转速度，过大会导致恢复阶段过冲
PERTURB_DURATION_MS_MIN = 900            # 随机扰动最短持续时间，单位毫秒
PERTURB_DURATION_MS_MAX = 1200           # 随机扰动最长持续时间，单位毫秒
PERTURB_SETTLE_SECONDS = 0.5             # 随机扰动结束后额外停车等待时间，让车辆机械惯性先消掉
TRIAL_SECONDS = 8.0                     # 每组参数完整采样时间，越长评分越稳但调参越慢
TRIAL_WARMUP_SECONDS = 0.8               # 评分时忽略试验初期的时间，避免刚启动瞬间影响过大
MIN_METRIC_SAMPLES = 10                  # 一轮试验至少需要的 AI_METRIC 样本数，不足则判为失败
LOCAL_COORDINATE_SEARCH_ENABLED = False  # False 时关闭本地坐标爬山搜索，只使用 AI 推荐参数并通过实车试验筛选
MAX_SEARCH_ROUNDS = 4                    # 本地坐标爬山搜索轮数；仅当 LOCAL_COORDINATE_SEARCH_ENABLED=True 时生效
STEP_SHRINK = 0.55                       # 每轮未显著改善时步长缩小比例
SCORE_IMPROVEMENT_EPS = 0.03             # 分数改善超过该阈值才接受新参数，防止噪声导致来回跳

SCORE_SETTLE_TIME_WEIGHT = 12.0          # 达到“基本纠正并进入稳定段”的时间权重，越大越偏向更快完成调整
SCORE_STEADY_YAW_WEIGHT = 2.6            # 稳定段 yaw 误差权重，越大越偏向后段转角更稳
SCORE_STEADY_COLOR_WEIGHT = 0.22         # 稳定段视觉误差权重，越大越偏向后段目标更居中
SCORE_STEADY_OSC_WEIGHT = 0.85           # 稳定段输出抖动权重，越大越压制后段震荡
SCORE_UNLOCK_WEIGHT = 12.0               # 整轮目标未锁定惩罚权重，越大越不接受频繁丢目标参数
SCORE_NOT_CENTER_WEIGHT = 10.0           # 整轮未居中惩罚权重，越大越不接受长时间调不进中心区
SCORE_NO_SETTLE_PENALTY = 25.0           # 整轮都没进入稳定段时附加惩罚，防止“始终在调”却侥幸均值不差
SCORE_TIMEOUT_PENALTY = 1000000.0        # 串口无数据或样本不足时使用的大惩罚分
MIN_LOCKED_RATIO_FOR_VALID_TRIAL = 0.60  # 目标锁定比例参考阈值；低于该值不再直接判死，而是用于额外提示当前参数容易丢目标
MIN_CENTER_RATIO_FOR_GOOD_TRIAL = 0.15   # 居中比例低于该值时额外重罚，避免“看见但始终追不上”的参数被误选
SETTLE_YAW_ERROR_DEG = 1.2               # 判定“yaw 已基本纠正”的误差阈值，单位度
SETTLE_COLOR_ERROR_PX = 6.0              # 判定“视觉已基本纠正”的综合误差阈值，单位像素
SETTLE_ROTATE_STD_LIMIT = 18.0           # 判定“开始平稳”时允许的瞬时旋转输出幅值阈值，越小越严格
SETTLE_TRANSLATE_SPEED_LIMIT = 4.0       # 判定“开始平稳”时允许的平移输出幅值阈值，越小越严格
SETTLE_HOLD_SECONDS = 0.6                # 上述稳定条件需要连续保持多久，才算真正进入稳定阶段

AI_ANALYSIS_ENABLED = True               # True 时启用外部 AI API，根据试验日志推荐额外候选参数；关闭后只使用本地坐标搜索
AI_WIRE_API = "responses"                # AI 接口协议；"responses" 对应 /v1/responses，"chat_completions" 对应 /v1/chat/completions
AI_API_URL = "https://ai.lupoapi.com/v1/responses"  # OpenAI-compatible Responses 接口地址，由 base_url=https://ai.lupoapi.com 和 wire_api=responses 组成
AI_API_KEY = "sk-b3ec22509c22050debd92051869d6025847a3d683478edcb9ffb919ebd1cad41"  # AI API Key；脚本会放入 Authorization: Bearer，请不要提交到公开仓库
AI_MODEL = "gpt-5.4"                     # AI 分析使用的模型名称，按你的 API 服务支持情况填写
AI_REASONING_EFFORT = "medium"            # Responses API 推理强度；你的服务配置为 xhigh，若接口报错可改为 high/medium/low
AI_STORE_RESPONSE = False                # False 时请求接口不要保存响应内容，对应 disable_response_storage=true
AI_STREAM_RESPONSE = False               # False 时要求 API 尽量一次性返回完整 JSON；若代理仍返回事件流，脚本会自动解析
AI_API_TIMEOUT_S = 90.0                  # AI API 单次请求超时时间，单位秒；xhigh 推理较慢，建议不要低于 60 秒
AI_TEMPERATURE = 0.2                     # AI 推荐参数时的随机性，越低越保守
AI_RESPONSES_INCLUDE_TEMPERATURE = False # Responses 模式是否发送 temperature；GPT-5/部分代理不支持该字段，默认关闭
AI_MAX_HISTORY_TRIALS = 18               # 每次发给 AI 的最近试验条数，太大容易浪费 token
AI_MAX_CANDIDATES_PER_CALL = 3           # AI 每次最多推荐并实测多少组候选参数；适度增加可减少总轮次
AI_RECOMMENDATION_ROUNDS = 12             # 关闭本地爬山后，AI 连续分析日志并推荐新参数的最大轮数
COMPARE_WITH_SAVED_BEST_SCORE = True     # True 时每次 AI 调参都直接拿 best_result.json 中的历史最佳分数做比较门槛，不再额外复测
AI_SUGGEST_AFTER_BASELINE = True         # True 时 baseline 后先让 AI 推荐候选参数，加快早期搜索
AI_SUGGEST_AFTER_EACH_ROUND = True       # True 时每个坐标搜索轮次后再让 AI 根据新日志推荐候选参数

WRITE_BACK_TO_SOURCE = True              # True 时把最佳参数写回 CarB 控制代码顶部常量区
SOURCE_PARAMETER_FILE = PROJECT_ROOT_DIR / "CarB" / "control" / "FollowLeaderYawColorTrace.py"  # 最终回写参数的代码文件
LOG_DIR = APP_DIR / "logs"  # 保存每轮试验 CSV/JSON 记录的目录；打包后会落在 exe 同级目录
SESSION_LOG_ROOT = LOG_DIR / "sessions"  # 每次上位机启动后的独立日志目录根路径；目录名使用启动时间戳
SESSION_NAME_FORMAT = "%Y-%m-%d-%H-%M-%S"  # 会话日志目录名时间格式，便于按启动时间快速查找
SAVE_BEST_RESULT_ON_EACH_UPDATE = True   # True 时 baseline 后立即保存最佳参数，之后只有实测分数更好才更新文件，防止中途故障丢参数

PARAMETER_SPACE = OrderedDict([
    # name, 当前值兜底, 最小值, 最大值, 初始搜索步长, 数据类型
    ("PID_KP", (0.3, 0.05, 1.20, 0.08, float)),
    ("PID_KI", (0.05, 0.0, 0.20, 0.02, float)),
    ("PID_KD", (0.1, 0.0, 0.60, 0.05, float)),
    ("PID_INTEGRAL_LIMIT", (60.0, 10.0, 120.0, 8.0, float)),
    ("YAW_DEADBAND_DEG", (0.2, 0.05, 1.50, 0.10, float)),
    ("MAX_ROTATE_SPEED", (85.0, 20.0, 85.0, 6.0, float)),
    ("MIN_COMMAND_SPEED", (50.0, 35.0, 70.0, 4.0, float)),
    ("COLOR_KP_X", (0.015, 0.002, 0.080, 0.006, float)),
    ("COLOR_KD_X", (0.25, 0.02, 0.80, 0.05, float)),
    ("COLOR_KP_Y", (0.015, 0.002, 0.080, 0.006, float)),
    ("COLOR_KD_Y", (0.25, 0.02, 0.80, 0.05, float)),
    ("COLOR_DEAD_ZONE", (3.0, 1.0, 12.0, 1.0, int)),
    ("COLOR_CENTER_EXIT_ZONE", (5.0, 2.0, 18.0, 1.0, int)),
    ("COLOR_INPUT_FILTER_ALPHA", (0.25, 0.05, 0.65, 0.04, float)),
    ("COLOR_COMMAND_FILTER_ALPHA", (0.18, 0.05, 0.80, 0.05, float)),
    ("COLOR_MAX_TRACKING_SPEED", (10.0, 2.0, 18.0, 1.5, float)),
    ("COLOR_COMMAND_RAMP_STEP", (1.5, 0.3, 5.0, 0.4, float)),
    ("MIN_TRANSLATE_SPEED", (10.0, 0.0, 22.0, 2.0, float)),
    ("DRIVE_MAX_DUTY", (8500.0, 4500.0, 9500.0, 400.0, int)),
    ("DRIVE_TRANSLATE_BIAS_DUTY", (5200.0, 2500.0, 6500.0, 300.0, int)),
])


AI_METRIC_PREFIX = "AI_METRIC"           # 必须与 CarB 中 AI_METRIC_PREFIX 保持一致
BEST_RESULT_JSON = LOG_DIR / "best_result.json"  # 最佳参数和评分输出文件
TRIAL_LOG_CSV = LOG_DIR / "trial_log.csv"         # 每轮参数试验汇总输出文件


@dataclass
class SerialSettings:
    """电脑端串口配置。"""

    port: str = SERIAL_PORT
    port_keyword: str = PORT_KEYWORD
    baudrate: int = BAUDRATE
    bytesize: int = SERIAL_BYTESIZE
    stopbits: float = SERIAL_STOPBITS
    parity: str = SERIAL_PARITY
    flow_control: str = SERIAL_FLOW_CONTROL
    timeout_s: float = SERIAL_TIMEOUT_S

    def normalized(self) -> "SerialSettings":
        """返回清洗后的串口配置，避免 GUI 传入大小写或空白差异。"""
        parity = str(self.parity or SERIAL_PARITY).strip().upper()
        if parity not in {"N", "E", "O", "M", "S"}:
            raise ValueError("不支持的校验位设置：{}".format(self.parity))

        flow_control = str(self.flow_control or SERIAL_FLOW_CONTROL).strip().lower()
        if flow_control not in {"none", "xonxoff", "rtscts", "dsrdtr"}:
            raise ValueError("不支持的流控设置：{}".format(self.flow_control))

        return SerialSettings(
            port=str(self.port or "").strip(),
            port_keyword=str(self.port_keyword or "").strip(),
            baudrate=int(self.baudrate),
            bytesize=int(self.bytesize),
            stopbits=float(self.stopbits),
            parity=parity,
            flow_control=flow_control,
            timeout_s=float(self.timeout_s),
        )


@dataclass
class SessionLogPaths:
    """一次完整调参会话对应的日志文件路径。"""

    session_name: str
    session_dir: Path
    trial_log_csv: Path
    trial_history_json: Path
    summary_json: Path
    serial_settings_json: Path
    console_log: Path


ACTIVE_SESSION_LOG_PATHS: SessionLogPaths | None = None
CANCEL_REQUEST_EVENT = threading.Event()  # 上位机终止按钮对应的协作取消信号；置位后各层循环会尽快收尾退出


class FirmwareMismatchError(RuntimeError):
    """串口连接到了 CarB，但从车运行的代码不支持自动调参命令。"""


class TrialSetupError(RuntimeError):
    """单轮试验准备失败，例如参数回包丢失或控制命令连续无回包。"""


class CancellationRequestedError(RuntimeError):
    """上位机请求终止自动调参时抛出的协作取消异常。"""


@dataclass
class MetricSample:
    timestamp_s: float
    yaw_error: float | None
    err_x: float
    err_y: float
    rotate: float
    vx: float
    vy: float
    target_locked: bool
    in_center: bool
    paused: bool
    deviation_active: bool
    manual_active: bool


@dataclass
class TrialResult:
    score: float
    samples: list[MetricSample]
    detail: dict[str, float]


def clear_cancel_request() -> None:
    """开始新会话前清掉旧的终止请求。"""
    CANCEL_REQUEST_EVENT.clear()


def request_cancel() -> None:
    """供 GUI 终止按钮调用，通知底层调参流程尽快退出。"""
    CANCEL_REQUEST_EVENT.set()


def is_cancel_requested() -> bool:
    """查询当前是否已收到终止请求。"""
    return CANCEL_REQUEST_EVENT.is_set()


def raise_if_cancel_requested() -> None:
    """在长循环或串口等待中轮询终止请求。"""
    if is_cancel_requested():
        raise CancellationRequestedError("用户请求终止自动调参。")


def resolve_serial_settings(serial_settings: SerialSettings | None = None) -> SerialSettings:
    """获取本次运行实际使用的串口配置。"""
    if serial_settings is None:
        return SerialSettings().normalized()
    return serial_settings.normalized()


def serial_settings_to_dict(serial_settings: SerialSettings) -> dict[str, object]:
    """把串口配置转换成适合写 JSON 的字典。"""
    settings = resolve_serial_settings(serial_settings)
    return {
        "port": settings.port,
        "port_keyword": settings.port_keyword,
        "baudrate": settings.baudrate,
        "bytesize": settings.bytesize,
        "stopbits": settings.stopbits,
        "parity": settings.parity,
        "flow_control": settings.flow_control,
        "timeout_s": settings.timeout_s,
    }


def build_serial_open_kwargs(serial_module, port: str, serial_settings: SerialSettings | None = None) -> dict[str, object]:
    """把统一串口配置转换成 pyserial 打开串口时需要的参数。"""
    settings = resolve_serial_settings(serial_settings)
    parity_map = {
        "N": serial_module.PARITY_NONE,
        "E": serial_module.PARITY_EVEN,
        "O": serial_module.PARITY_ODD,
        "M": serial_module.PARITY_MARK,
        "S": serial_module.PARITY_SPACE,
    }
    stopbits_map = {
        1.0: serial_module.STOPBITS_ONE,
        1.5: serial_module.STOPBITS_ONE_POINT_FIVE,
        2.0: serial_module.STOPBITS_TWO,
    }
    if settings.stopbits not in stopbits_map:
        raise ValueError("不支持的停止位设置：{}".format(settings.stopbits))

    kwargs = {
        "port": port,
        "baudrate": settings.baudrate,
        "bytesize": settings.bytesize,
        "parity": parity_map[settings.parity],
        "stopbits": stopbits_map[settings.stopbits],
        "timeout": settings.timeout_s,
        "write_timeout": 1.0,
        "xonxoff": settings.flow_control == "xonxoff",
        "rtscts": settings.flow_control == "rtscts",
        "dsrdtr": settings.flow_control == "dsrdtr",
    }
    return kwargs


def create_session_log_paths(session_name: str | None = None) -> SessionLogPaths:
    """按启动时间戳创建一套独立日志路径。"""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    SESSION_LOG_ROOT.mkdir(parents=True, exist_ok=True)
    if not session_name:
        session_name = time.strftime(SESSION_NAME_FORMAT)
    session_dir = SESSION_LOG_ROOT / session_name
    session_dir.mkdir(parents=True, exist_ok=True)
    return SessionLogPaths(
        session_name=session_name,
        session_dir=session_dir,
        trial_log_csv=session_dir / "trial_log.csv",
        trial_history_json=session_dir / "trial_history.json",
        summary_json=session_dir / "summary.json",
        serial_settings_json=session_dir / "serial_settings.json",
        console_log=session_dir / "console.log",
    )


def activate_session_logging(session_paths: SessionLogPaths | None, serial_settings: SerialSettings | None = None) -> None:
    """激活当前会话日志目录，并写入基础串口配置。"""
    global ACTIVE_SESSION_LOG_PATHS
    ACTIVE_SESSION_LOG_PATHS = session_paths
    if session_paths is None:
        return
    payload = {
        "session_name": session_paths.session_name,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if serial_settings is not None:
        payload["serial_settings"] = serial_settings_to_dict(serial_settings)
    write_json_atomic(session_paths.serial_settings_json, payload)


def write_json_atomic(path: Path, payload: object) -> None:
    """原子写 JSON，避免程序中途中断时写出半截文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(path.name + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_path.replace(path)


def persist_trial_history(trial_history: list[dict]) -> None:
    """把当前会话全部轮次历史写入独立 JSON，便于上位机随时查看。"""
    if ACTIVE_SESSION_LOG_PATHS is None:
        return
    payload = {
        "session_name": ACTIVE_SESSION_LOG_PATHS.session_name,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trial_count": len(trial_history),
        "trials": trial_history,
    }
    write_json_atomic(ACTIVE_SESSION_LOG_PATHS.trial_history_json, payload)


def load_persisted_trial_history() -> list[dict]:
    """从当前会话 JSON 中读回已记录的轮次历史，供异常恢复和会话收尾使用。"""
    if ACTIVE_SESSION_LOG_PATHS is None:
        return []
    if not ACTIVE_SESSION_LOG_PATHS.trial_history_json.exists():
        return []
    try:
        payload = json.loads(ACTIVE_SESSION_LOG_PATHS.trial_history_json.read_text(encoding="utf-8"))
    except Exception:
        return []
    trials = payload.get("trials", [])
    return trials if isinstance(trials, list) else []


def persist_session_summary(
    best_parameters: dict[str, float] | None,
    best_result: TrialResult | None,
    trial_history: list[dict],
    status: str,
    error_message: str = "",
) -> None:
    """把当前会话概要写入 summary.json，便于上位机结束后快速回看。"""
    if ACTIVE_SESSION_LOG_PATHS is None:
        return
    payload = {
        "session_name": ACTIVE_SESSION_LOG_PATHS.session_name,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "status": status,
        "error_message": error_message,
        "trial_count": len(trial_history),
        "best_parameters": best_parameters,
        "best_score": None if best_result is None else best_result.score,
        "best_detail": {} if best_result is None else best_result.detail,
    }
    write_json_atomic(ACTIVE_SESSION_LOG_PATHS.summary_json, payload)


class SerialBridge:
    """封装电脑串口读写，统一处理文本行、命令回包和 AI_METRIC 采样。"""

    def __init__(self, serial_module, port: str, serial_settings: SerialSettings | None = None):
        self.serial_settings = resolve_serial_settings(serial_settings)
        self.serial = serial_module.Serial(**build_serial_open_kwargs(serial_module, port, self.serial_settings))
        self.port = port

    def close(self) -> None:
        self.serial.close()

    def send_line(self, line: str) -> None:
        self.serial.write((line.strip() + "\n").encode("utf-8"))
        self.serial.flush()

    def read_line(self) -> str | None:
        raw = self.serial.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="ignore").strip()

    def drain_input(self, seconds: float = COMMAND_DRAIN_SECONDS) -> list[str]:
        """发送新命令前清掉串口里遗留的旧日志，避免把上一条回包误判成当前回包。"""
        drained = []
        end_time = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end_time:
            raise_if_cancel_requested()
            line = self.read_line()
            if line:
                drained.append(line)
        return drained

    def read_lines_for(self, seconds: float) -> list[str]:
        end_time = time.monotonic() + seconds
        lines = []
        while time.monotonic() < end_time:
            raise_if_cancel_requested()
            line = self.read_line()
            if line:
                lines.append(line)
                print(line)
        return lines

    def command(self, line: str, timeout_s: float = 1.5, drain_before: bool = True) -> str:
        if drain_before:
            self.drain_input()
        self.send_line(line)
        deadline = time.monotonic() + timeout_s
        last_line = ""
        command_upper = line.strip().upper()
        get_name = command_upper[4:].strip() if command_upper.startswith("GET ") else ""
        set_name = command_upper.split("=", 1)[0].strip() if "=" in command_upper else ""
        while time.monotonic() < deadline:
            raise_if_cancel_requested()
            reply = self.read_line()
            if not reply:
                continue
            last_line = reply
            print(reply)
            if get_name and reply.upper().startswith(get_name + "="):
                return reply
            if set_name and reply.upper().startswith("OK " + set_name + "="):
                return reply
            if set_name and reply.upper().startswith("OK "):
                # 这是其它参数的延迟回包，继续等当前参数自己的确认。
                continue
            if reply.startswith("OK") or reply.startswith("ERR"):
                return reply
        return last_line


def load_pyserial():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("缺少 pyserial：请先执行 python -m pip install pyserial")
        raise
    return serial, list_ports


def list_available_ports(list_ports_module, port_keyword: str = "") -> list[str]:
    """枚举电脑当前可见串口，供 GUI 刷新下拉框。"""
    ports = list(list_ports_module.comports())
    if port_keyword:
        keyword = port_keyword.lower().strip()
        ports = [
            port for port in ports
            if keyword in port.device.lower() or keyword in (port.description or "").lower()
        ]
    return [port.device for port in ports]


def candidate_ports(list_ports_module, serial_settings: SerialSettings | None = None) -> list[str]:
    settings = resolve_serial_settings(serial_settings)
    if settings.port:
        return [settings.port]
    return list_available_ports(list_ports_module, settings.port_keyword)


def open_bridge(serial_module, list_ports_module, serial_settings: SerialSettings | None = None) -> SerialBridge:
    settings = resolve_serial_settings(serial_settings)
    ports = candidate_ports(list_ports_module, settings)
    if not ports:
        raise RuntimeError("没有找到可用串口，请检查 USB 转串口连接，或在 SERIAL_PORT 中指定端口")

    print("候选串口：{}".format(", ".join(ports)))
    deadline = time.monotonic() + READY_WAIT_SECONDS
    last_error = None
    while time.monotonic() < deadline:
        raise_if_cancel_requested()
        for port in ports:
            raise_if_cancel_requested()
            bridge = None
            try:
                print("尝试连接 {} ...".format(port))
                bridge = SerialBridge(serial_module, port, settings)
                bridge.drain_input()
                bridge.send_line("AI_STATUS")
                lines = bridge.read_lines_for(PORT_PROBE_SECONDS)
                has_ai_status = any(
                    line.startswith("OK AI_STATUS")
                    or ("paused=" in line and "manual_remaining_ms=" in line)
                    for line in lines
                )
                has_old_tuning_ready = any("TUNE READY" in line for line in lines)
                has_unsupported = any(line.startswith("ERR unsupported") for line in lines)
                has_metric = any(line.startswith(AI_METRIC_PREFIX) for line in lines)

                if has_ai_status or has_metric:
                    print("已连接 CarB：{}".format(port))
                    return bridge

                if has_old_tuning_ready or has_unsupported:
                    if REQUIRE_AI_FIRMWARE:
                        raise FirmwareMismatchError(
                            "已连接到 CarB 串口 {}，但板子运行的是旧版程序：不支持 AI_STATUS/AI_PERTURB。"
                            "请把当前电脑上的 CarB/control/FollowLeaderYawColorTrace.py 重新下载到从车，"
                            "重新上电后再运行本脚本。".format(port)
                        )
                    print("已连接 CarB：{}，但未检测到 AI 控制命令支持。".format(port))
                    return bridge

                if len(ports) == 1:
                    print("单一串口暂未收到 AI 握手，继续等待从车启动后的回包。")
            except Exception as exc:
                if isinstance(exc, FirmwareMismatchError):
                    raise
                last_error = exc
            if bridge is not None:
                try:
                    bridge.close()
                except Exception:
                    pass
        time.sleep(0.5)

    raise RuntimeError("等待 CarB 串口握手超时，最后错误：{}".format(last_error))


def parse_bool(text: str) -> bool:
    return text.strip().lower() in ("true", "1", "yes", "on")


def parse_float(text: str) -> float | None:
    if text in ("NA", "--", ""):
        return None
    try:
        return float(text)
    except ValueError:
        match = re.match(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", text.strip())
        if match is None:
            return None
        try:
            return float(match.group(0))
        except ValueError:
            return None


def metric_float(fields: dict[str, str], name: str, default: float | None = 0.0) -> float | None:
    """从 AI_METRIC 字段里读取浮点数，遇到半行/粘包时尽量取数字前缀。"""
    value = parse_float(fields.get(name, ""))
    if value is None:
        return default
    return value


def parse_metric_line(line: str, timestamp_s: float) -> MetricSample | None:
    if not line.startswith(AI_METRIC_PREFIX):
        return None

    fields = {}
    for token in line.split()[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value

    if not fields:
        return None

    yaw_error = parse_float(fields.get("yaw_error", "NA"))
    err_x = metric_float(fields, "err_x")
    err_y = metric_float(fields, "err_y")
    rotate = metric_float(fields, "rotate")
    vx = metric_float(fields, "vx")
    vy = metric_float(fields, "vy")
    if err_x is None or err_y is None or rotate is None or vx is None or vy is None:
        return None

    return MetricSample(
        timestamp_s=timestamp_s,
        yaw_error=yaw_error,
        err_x=err_x,
        err_y=err_y,
        rotate=rotate,
        vx=vx,
        vy=vy,
        target_locked=parse_bool(fields.get("target_locked", "False")),
        in_center=parse_bool(fields.get("in_center", "False")),
        paused=parse_bool(fields.get("paused", "False")),
        deviation_active=parse_bool(fields.get("deviation_active", "False")),
        manual_active=parse_bool(fields.get("manual_active", "False")),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def format_value(name: str, value: float) -> str:
    value_type = PARAMETER_SPACE[name][4]
    if value_type is int:
        return str(int(round(value)))
    return "{:.6g}".format(float(value))


def parameter_equal(name: str, left: float, right: float) -> bool:
    """判断两个参数值在下发格式意义上是否一致。"""
    value_type = PARAMETER_SPACE[name][4]
    if value_type is int:
        return int(round(left)) == int(round(right))
    return abs(float(left) - float(right)) <= 1e-9


def read_current_parameters(bridge: SerialBridge) -> dict[str, float]:
    current = {}
    for name, spec in PARAMETER_SPACE.items():
        raise_if_cancel_requested()
        fallback = float(spec[0])
        reply = ""
        match = None
        for attempt in range(1, COMMAND_RETRY_ATTEMPTS + 1):
            raise_if_cancel_requested()
            reply = bridge.command(
                "GET {}".format(name),
                timeout_s=PARAMETER_COMMAND_TIMEOUT_S,
            )
            match = re.search(r"{}=([-+0-9.eE]+)".format(re.escape(name)), reply)
            if match:
                break
            print("GET {} 第 {} 次未读到完整回包，回包={!r}。".format(name, attempt, reply))
            time.sleep(0.15)
        current[name] = float(match.group(1)) if match else fallback
    return current


def require_ai_firmware(bridge: SerialBridge) -> None:
    """正式试验前确认从车已经运行支持 AI 自动调参命令的新代码。"""
    if not REQUIRE_AI_FIRMWARE:
        return

    reply = bridge.command("AI_STATUS", timeout_s=2.0)
    if reply.startswith("OK AI_STATUS") or ("paused=" in reply and "manual_remaining_ms=" in reply):
        return

    if reply.startswith("ERR unsupported"):
        raise FirmwareMismatchError(
            "CarB 当前程序不支持 AI_STATUS/AI_PERTURB，说明从车还在运行旧版代码。"
            "请重新下载 CarB 文件夹中的新版代码到从车，重新上电后再运行脚本。"
        )

    raise RuntimeError(
        "没有收到 CarB 的 AI_STATUS 确认，当前回包为 {!r}。"
        "请确认从车已上电、IMU 校准完成，且串口号/波特率正确。".format(reply)
    )


def pause_carb_before_parameter_read(bridge: SerialBridge) -> None:
    """连接后先让 CarB 停止闭环和 AI_METRIC 输出，再读取当前参数。"""
    raise_if_cancel_requested()
    print("发送 AI_STOP，让从车进入暂停状态并清空启动阶段的 AI_METRIC。")
    require_ok(
        command_with_retry(
            bridge,
            "AI_STOP",
            "AI_STOP",
            timeout_s=CONTROL_COMMAND_TIMEOUT_S,
        ),
        "AI_STOP",
    )
    time.sleep(INITIAL_STOP_SETTLE_SECONDS)
    raise_if_cancel_requested()
    bridge.drain_input(INITIAL_DRAIN_SECONDS)


def require_ok(reply: str, command_name: str) -> None:
    """关键控制命令必须收到 OK，否则停止自动调参，避免无效或危险试验。"""
    if reply.startswith("OK"):
        return
    if reply.startswith("ERR unsupported"):
        raise TrialSetupError(
            "{} 回包 ERR unsupported，可能是无线串口丢字导致命令不完整，本轮跳过。".format(command_name)
        )
    raise TrialSetupError("{} 执行失败，回包：{!r}".format(command_name, reply))


def command_with_retry(
    bridge: SerialBridge,
    line: str,
    command_name: str,
    timeout_s: float,
    attempts: int = COMMAND_RETRY_ATTEMPTS,
) -> str:
    """发送命令并在空回包、错位回包时重试，最终返回最后一次回包。"""
    last_reply = ""
    for attempt in range(1, max(1, attempts) + 1):
        raise_if_cancel_requested()
        reply = bridge.command(line, timeout_s=timeout_s)
        last_reply = reply
        if reply.startswith("OK"):
            return reply
        if reply.startswith("ERR unsupported"):
            return reply
        if reply.startswith("ERR"):
            return reply
        print("{} 第 {} 次未收到有效 OK，回包={!r}，准备重试。".format(command_name, attempt, reply))
        time.sleep(0.15)
    return last_reply


def apply_parameters(bridge: SerialBridge, parameters: dict[str, float]) -> None:
    applied_parameters = getattr(bridge, "applied_parameters", None)
    if applied_parameters is None:
        applied_parameters = {}
        bridge.applied_parameters = applied_parameters

    for name, value in parameters.items():
        raise_if_cancel_requested()
        if (
            ONLY_SEND_CHANGED_PARAMETERS
            and name in applied_parameters
            and parameter_equal(name, applied_parameters[name], value)
        ):
            continue

        reply = command_with_retry(
            bridge,
            "{}={}".format(name, format_value(name, value)),
            name,
            timeout_s=PARAMETER_COMMAND_TIMEOUT_S,
        )
        require_ok(reply, name)
        applied_parameters[name] = value
        time.sleep(PARAMETER_COMMAND_GAP_SECONDS)


def build_random_perturb_command() -> tuple[str, float]:
    """生成一条随机开环扰动命令，并返回扰动加停车等待所需时间。"""
    angle_deg = random.uniform(0.0, 360.0)
    translate_speed = random.uniform(PERTURB_TRANSLATE_SPEED_MIN, PERTURB_TRANSLATE_SPEED_MAX)
    rotate_speed = 0.0
    if PERTURB_ROTATE_ENABLED:
        rotate_speed = random.uniform(PERTURB_ROTATE_SPEED_MIN, PERTURB_ROTATE_SPEED_MAX)
        if random.choice((True, False)):
            rotate_speed = -rotate_speed
    duration_ms = random.randint(PERTURB_DURATION_MS_MIN, PERTURB_DURATION_MS_MAX)
    command = "AI_PERTURB {:.2f} {:.2f} {:.2f} {}".format(
        angle_deg,
        translate_speed,
        rotate_speed,
        duration_ms,
    )
    wait_seconds = duration_ms / 1000.0 + PERTURB_SETTLE_SECONDS
    return command, wait_seconds


def run_pretrial_perturb(bridge: SerialBridge) -> None:
    """每轮正式闭环采样前，主动制造随机位置偏差；需要时也可额外叠加 yaw 扰动。"""
    if not PRETRIAL_RANDOM_PERTURB_ENABLED:
        return

    raise_if_cancel_requested()
    command, wait_seconds = build_random_perturb_command()
    print("发送随机扰动：{}".format(command))
    reply = command_with_retry(
        bridge,
        command,
        "AI_PERTURB",
        timeout_s=CONTROL_COMMAND_TIMEOUT_S,
    )
    require_ok(reply, "AI_PERTURB")
    bridge.read_lines_for(wait_seconds)


def collect_trial_samples(bridge: SerialBridge, seconds: float) -> list[MetricSample]:
    start_time = time.monotonic()
    samples = []
    while time.monotonic() - start_time < seconds:
        raise_if_cancel_requested()
        line = bridge.read_line()
        if not line:
            continue
        print(line)
        sample = parse_metric_line(line, time.monotonic() - start_time)
        if sample is not None:
            samples.append(sample)
    return samples


def score_trial(samples: list[MetricSample]) -> TrialResult:
    usable = [
        sample for sample in samples
        if sample.timestamp_s >= TRIAL_WARMUP_SECONDS and not sample.paused and not sample.manual_active
    ]
    if len(usable) < MIN_METRIC_SAMPLES:
        return TrialResult(
            score=SCORE_TIMEOUT_PENALTY,
            samples=samples,
            detail={"usable_samples": float(len(usable))},
        )

    yaw_errors = [abs(sample.yaw_error) for sample in usable if sample.yaw_error is not None]
    color_errors = [math.sqrt(sample.err_x * sample.err_x + sample.err_y * sample.err_y) for sample in usable]
    rotate_values = [sample.rotate for sample in usable]
    vx_values = [sample.vx for sample in usable]
    vy_values = [sample.vy for sample in usable]

    mean_yaw = statistics.fmean(yaw_errors) if yaw_errors else 999.0
    rms_color = math.sqrt(statistics.fmean([err * err for err in color_errors]))
    rotate_std = statistics.pstdev(rotate_values) if len(rotate_values) > 1 else 0.0
    vx_std = statistics.pstdev(vx_values) if len(vx_values) > 1 else 0.0
    vy_std = statistics.pstdev(vy_values) if len(vy_values) > 1 else 0.0
    locked_ratio = statistics.fmean([1.0 if sample.target_locked else 0.0 for sample in usable])
    center_ratio = statistics.fmean([1.0 if sample.in_center else 0.0 for sample in usable])

    settle_start_time = None
    settle_time_s = usable[-1].timestamp_s + SETTLE_HOLD_SECONDS
    stable_hold_begin = None
    settled_index = None
    for index, sample in enumerate(usable):
        yaw_ok = sample.yaw_error is not None and abs(sample.yaw_error) <= SETTLE_YAW_ERROR_DEG
        color_error = math.sqrt(sample.err_x * sample.err_x + sample.err_y * sample.err_y)
        color_ok = color_error <= SETTLE_COLOR_ERROR_PX
        motion_ok = (
            abs(sample.rotate) <= SETTLE_ROTATE_STD_LIMIT
            and abs(sample.vx) <= SETTLE_TRANSLATE_SPEED_LIMIT
            and abs(sample.vy) <= SETTLE_TRANSLATE_SPEED_LIMIT
        )
        stable_now = sample.target_locked and sample.in_center and yaw_ok and color_ok and motion_ok
        if stable_now:
            if stable_hold_begin is None:
                stable_hold_begin = sample.timestamp_s
            if sample.timestamp_s - stable_hold_begin >= SETTLE_HOLD_SECONDS:
                settle_start_time = stable_hold_begin
                settle_time_s = stable_hold_begin
                settled_index = index
                break
        else:
            stable_hold_begin = None

    post_settle = usable[settled_index:] if settled_index is not None else usable
    post_yaw_errors = [abs(sample.yaw_error) for sample in post_settle if sample.yaw_error is not None]
    post_color_errors = [math.sqrt(sample.err_x * sample.err_x + sample.err_y * sample.err_y) for sample in post_settle]
    post_rotate_values = [sample.rotate for sample in post_settle]
    post_vx_values = [sample.vx for sample in post_settle]
    post_vy_values = [sample.vy for sample in post_settle]

    steady_mean_yaw = statistics.fmean(post_yaw_errors) if post_yaw_errors else 999.0
    steady_rms_color = math.sqrt(statistics.fmean([err * err for err in post_color_errors])) if post_color_errors else 999.0
    steady_rotate_std = statistics.pstdev(post_rotate_values) if len(post_rotate_values) > 1 else 0.0
    steady_vx_std = statistics.pstdev(post_vx_values) if len(post_vx_values) > 1 else 0.0
    steady_vy_std = statistics.pstdev(post_vy_values) if len(post_vy_values) > 1 else 0.0

    center_shortfall = max(0.0, MIN_CENTER_RATIO_FOR_GOOD_TRIAL - center_ratio)
    lock_shortfall = max(0.0, MIN_LOCKED_RATIO_FOR_VALID_TRIAL - locked_ratio)
    steady_oscillation = steady_rotate_std + 0.35 * (steady_vx_std + steady_vy_std)
    score = (
        SCORE_SETTLE_TIME_WEIGHT * settle_time_s
        + SCORE_STEADY_YAW_WEIGHT * steady_mean_yaw
        + SCORE_STEADY_COLOR_WEIGHT * steady_rms_color
        + SCORE_STEADY_OSC_WEIGHT * steady_oscillation
        + SCORE_UNLOCK_WEIGHT * (1.0 - locked_ratio)
        + SCORE_NOT_CENTER_WEIGHT * (1.0 - center_ratio)
        + SCORE_NOT_CENTER_WEIGHT * 2.0 * center_shortfall
        + SCORE_UNLOCK_WEIGHT * 0.6 * lock_shortfall
    )
    if settled_index is None:
        score += SCORE_NO_SETTLE_PENALTY

    detail = {
        "usable_samples": float(len(usable)),
        "mean_abs_yaw_error": mean_yaw,
        "rms_color_error": rms_color,
        "rotate_std": rotate_std,
        "vx_std": vx_std,
        "vy_std": vy_std,
        "settled": 1.0 if settled_index is not None else 0.0,
        "settle_time_s": settle_time_s,
        "steady_mean_abs_yaw_error": steady_mean_yaw,
        "steady_rms_color_error": steady_rms_color,
        "steady_rotate_std": steady_rotate_std,
        "steady_vx_std": steady_vx_std,
        "steady_vy_std": steady_vy_std,
        "locked_ratio": locked_ratio,
        "center_ratio": center_ratio,
    }
    if locked_ratio < MIN_LOCKED_RATIO_FOR_VALID_TRIAL:
        detail["failure_reason"] = "target_lost_often_but_recoverable"

    return TrialResult(
        score=score,
        samples=samples,
        detail=detail,
    )


def failed_trial_result(reason: str) -> TrialResult:
    """单轮试验失败时返回大惩罚分，并把原因写入日志明细。"""
    return TrialResult(
        score=SCORE_TIMEOUT_PENALTY,
        samples=[],
        detail={
            "usable_samples": 0.0,
            "failure_reason": reason,
        },
    )


def run_trial(bridge: SerialBridge, parameters: dict[str, float], trial_name: str) -> TrialResult:
    print("\n=== 试验 {} ===".format(trial_name))
    try:
        raise_if_cancel_requested()
        require_ok(
            command_with_retry(bridge, "AI_STOP", "AI_STOP", timeout_s=CONTROL_COMMAND_TIMEOUT_S),
            "AI_STOP",
        )
        time.sleep(STOP_SETTLE_SECONDS)
        raise_if_cancel_requested()
        apply_parameters(bridge, parameters)
        run_pretrial_perturb(bridge)
        require_ok(
            command_with_retry(bridge, "AI_START", "AI_START", timeout_s=CONTROL_COMMAND_TIMEOUT_S),
            "AI_START",
        )
        samples = collect_trial_samples(bridge, TRIAL_SECONDS)
        require_ok(
            command_with_retry(bridge, "AI_STOP", "AI_STOP", timeout_s=CONTROL_COMMAND_TIMEOUT_S),
            "AI_STOP",
        )
        result = score_trial(samples)
    except CancellationRequestedError:
        try:
            bridge.command("AI_STOP", timeout_s=CONTROL_COMMAND_TIMEOUT_S, drain_before=False)
        except Exception:
            pass
        raise
    except FirmwareMismatchError:
        raise
    except TrialSetupError as exc:
        if not FAILED_TRIAL_CONTINUE:
            raise
        print("试验 {} 准备失败，记为惩罚分并继续：{}".format(trial_name, exc))
        try:
            bridge.command("AI_STOP", timeout_s=CONTROL_COMMAND_TIMEOUT_S)
        except Exception:
            pass
        result = failed_trial_result(str(exc))
    print("试验 {} 损失分数：{:.4f}（越低越好） 细节：{}".format(trial_name, result.score, result.detail))
    return result


def append_trial_log(writer, trial_name: str, parameters: dict[str, float], result: TrialResult) -> None:
    row = {"trial": trial_name, "score": result.score}
    row.update(parameters)
    row.update(result.detail)
    writer.writerow(row)


def record_trial(
    writer,
    trial_history: list[dict],
    trial_name: str,
    parameters: dict[str, float],
    result: TrialResult,
) -> dict:
    """统一记录一轮试验的 CSV 和 JSON 历史。"""
    append_trial_log(writer, trial_name, parameters, result)
    record = build_trial_record(trial_name, parameters, result)
    trial_history.append(record)
    persist_trial_history(trial_history)
    return record


def build_trial_record(trial_name: str, parameters: dict[str, float], result: TrialResult) -> dict:
    """把一轮试验整理成适合写日志和发给 AI 的结构化记录。"""
    record = {
        "trial": trial_name,
        "score": result.score,
        "parameters": {name: float(parameters[name]) for name in PARAMETER_SPACE},
    }
    record.update(result.detail)
    return record


def load_saved_best_snapshot() -> tuple[dict[str, float], TrialResult] | None:
    """从 best_result.json 读取历史最佳参数和分数，读取失败时返回 None。"""
    if not BEST_RESULT_JSON.exists():
        return None

    try:
        payload = json.loads(BEST_RESULT_JSON.read_text(encoding="utf-8"))
    except Exception as exc:
        print("读取历史最佳参数失败，忽略本次历史基准：{}".format(exc))
        return None

    parameters = payload.get("parameters")
    if not isinstance(parameters, dict):
        print("历史最佳文件缺少 parameters 字段，忽略本次历史基准。")
        return None

    score = payload.get("score")
    try:
        score = float(score)
    except Exception:
        print("历史最佳文件缺少有效 score 字段，忽略本次历史基准。")
        return None

    detail = payload.get("detail")
    if not isinstance(detail, dict):
        detail = {}

    fallback = {name: float(spec[0]) for name, spec in PARAMETER_SPACE.items()}
    saved_parameters = clamp_candidate_parameters(parameters, fallback)
    saved_result = TrialResult(score=score, samples=[], detail=detail)
    return saved_parameters, saved_result


def clamp_candidate_parameters(candidate: dict, fallback: dict[str, float]) -> dict[str, float]:
    """把 AI 给出的候选参数限制在允许范围内，缺失项沿用当前最佳参数。"""
    clamped = fallback.copy()
    for name, spec in PARAMETER_SPACE.items():
        if name not in candidate:
            continue
        _, lower, upper, _, value_type = spec
        try:
            raw_value = value_type(candidate[name])
        except Exception:
            continue
        clamped[name] = clamp(float(raw_value), float(lower), float(upper))
    return clamped


def build_ai_analysis_prompt(
    trial_history: list[dict],
    best_parameters: dict[str, float],
    best_result: TrialResult,
    steps: dict[str, float],
) -> str:
    """生成给 AI 的调参分析提示词，要求返回严格 JSON，便于脚本自动解析。"""
    recent_trials = trial_history[-AI_MAX_HISTORY_TRIALS:]
    parameter_bounds = {
        name: {
            "min": spec[1],
            "max": spec[2],
            "current_step": steps[name],
        }
        for name, spec in PARAMETER_SPACE.items()
    }
    payload = {
        "task": "根据实车闭环调参日志，推荐更稳更快收敛的下一组候选参数。",
        "score_rule": "score 是损失分数，越低越好。当前评分优先奖励更快进入稳定纠正阶段、后段更平稳、震荡更小；1000000 只表示样本不足或命令失败。",
        "optimization_goal": [
            "优先缩短 settle_time_s，让车辆更快完成纠正并进入稳定阶段。",
            "优先保证 target_locked 和 center_ratio，不能因为 yaw 稳而丢目标。",
            "在不丢目标的前提下降低 steady_mean_abs_yaw_error、steady_rms_color_error 和 steady_* 抖动。",
            "推荐要保守，避免一次把参数推到边界。",
            "如果出现超调、响应过快、持续震荡或目标被甩出画面，优先考虑降低速度上限、最小输出、总 duty、起转补偿，并适当增加死区、滤波或减小斜率。"
        ],
        "priority_tuning_hints": {
            "yaw_overshoot_or_rotate_std_too_high": [
                "MAX_ROTATE_SPEED",
                "MIN_COMMAND_SPEED",
                "PID_KP",
                "PID_KD",
                "PID_INTEGRAL_LIMIT",
                "YAW_DEADBAND_DEG",
            ],
            "translation_overshoot_or_target_lost": [
                "COLOR_MAX_TRACKING_SPEED",
                "MIN_TRANSLATE_SPEED",
                "DRIVE_MAX_DUTY",
                "DRIVE_TRANSLATE_BIAS_DUTY",
                "COLOR_COMMAND_RAMP_STEP",
                "COLOR_COMMAND_FILTER_ALPHA",
                "COLOR_INPUT_FILTER_ALPHA",
            ],
            "near_center_but_keep_hunting": [
                "COLOR_DEAD_ZONE",
                "COLOR_CENTER_EXIT_ZONE",
                "COLOR_KP_X",
                "COLOR_KD_X",
                "COLOR_KP_Y",
                "COLOR_KD_Y",
            ],
        },
        "allowed_parameters": list(PARAMETER_SPACE.keys()),
        "parameter_bounds": parameter_bounds,
        "best_parameters": best_parameters,
        "best_score": best_result.score,
        "best_detail": best_result.detail,
        "recent_trials": recent_trials,
        "required_json_schema": {
            "candidate_parameters": [
                {
                    "PID_KP": "float，可只填需要改变的参数",
                    "PID_KI": "float",
                    "PID_KD": "float",
                    "MAX_ROTATE_SPEED": "float",
                    "COLOR_MAX_TRACKING_SPEED": "float",
                    "DRIVE_MAX_DUTY": "int",
                    "DRIVE_TRANSLATE_BIAS_DUTY": "int"
                }
            ],
            "reason": "简短中文说明",
            "confidence": "0到1之间的小数"
        },
    }
    return json.dumps(payload, ensure_ascii=False)


def extract_json_object(text: str) -> dict | None:
    """从 AI 回复中提取 JSON 对象，兼容模型偶尔包一层说明文字的情况。"""
    try:
        return json.loads(text)
    except Exception:
        pass

    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        return json.loads(text[start:end + 1])
    except Exception:
        return None


def build_ai_request_body(prompt: str) -> dict:
    """按顶部 AI_WIRE_API 配置构造请求体，便于现场在两种 OpenAI 兼容协议之间切换。"""
    system_prompt = "你是智能车闭环 PID 和视觉追踪调参助手。只返回合法 JSON，不要输出 Markdown。"
    if AI_WIRE_API == "responses":
        request_body = {
            "model": AI_MODEL,
            "input": [
                {
                    "role": "system",
                    "content": system_prompt,
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ],
            "store": AI_STORE_RESPONSE,
            "stream": AI_STREAM_RESPONSE,
            "text": {"format": {"type": "json_object"}},
        }
        if AI_RESPONSES_INCLUDE_TEMPERATURE:
            request_body["temperature"] = AI_TEMPERATURE
        if AI_REASONING_EFFORT:
            request_body["reasoning"] = {"effort": AI_REASONING_EFFORT}
        return request_body

    return {
        "model": AI_MODEL,
        "temperature": AI_TEMPERATURE,
        "messages": [
            {
                "role": "system",
                "content": system_prompt,
            },
            {
                "role": "user",
                "content": prompt,
            },
        ],
        "response_format": {"type": "json_object"},
    }


def extract_text_from_sse_response(raw_response_text: str) -> str:
    """从代理返回的 text/event-stream 事件流里拼出最终模型文本。"""
    text_parts = []
    completed_response = {}
    for line in raw_response_text.splitlines():
        line = line.strip()
        if not line.startswith("data:"):
            continue
        data_text = line[5:].strip()
        if not data_text or data_text == "[DONE]":
            continue
        try:
            event_payload = json.loads(data_text)
        except Exception:
            continue

        event_type = event_payload.get("type", "")
        if event_type.endswith(".delta"):
            delta = event_payload.get("delta", "")
            if isinstance(delta, str):
                text_parts.append(delta)
        elif event_type.endswith(".done"):
            text = event_payload.get("text", "")
            if isinstance(text, str) and text:
                return text
        elif event_type == "response.completed":
            response = event_payload.get("response", {})
            if isinstance(response, dict):
                completed_response = response

    if text_parts:
        return "".join(text_parts)
    if completed_response:
        return extract_ai_response_text(completed_response, raw_response_text)
    return ""


def extract_ai_response_text(response_payload: dict, raw_response_text: str) -> str:
    """从 Responses、Chat Completions 或 SSE 事件流返回里提取模型正文，兼容代理服务的格式差异。"""
    if not response_payload and "event:" in raw_response_text and "data:" in raw_response_text:
        sse_text = extract_text_from_sse_response(raw_response_text)
        if sse_text:
            return sse_text

    output_text = response_payload.get("output_text")
    if isinstance(output_text, str) and output_text.strip():
        return output_text

    output_parts = []
    for output_item in response_payload.get("output", []):
        if not isinstance(output_item, dict):
            continue
        for content_item in output_item.get("content", []):
            if not isinstance(content_item, dict):
                continue
            text = content_item.get("text")
            if isinstance(text, str) and text:
                output_parts.append(text)
    if output_parts:
        return "\n".join(output_parts)

    try:
        return response_payload["choices"][0]["message"]["content"]
    except Exception:
        if "event:" in raw_response_text and "data:" in raw_response_text:
            sse_text = extract_text_from_sse_response(raw_response_text)
            if sse_text:
                return sse_text
        return raw_response_text


def request_ai_candidates(
    trial_history: list[dict],
    best_parameters: dict[str, float],
    best_result: TrialResult,
    steps: dict[str, float],
) -> list[dict[str, float]]:
    """调用外部 AI API，返回经过范围限制的候选参数列表。"""
    if not AI_ANALYSIS_ENABLED:
        return []
    if not AI_API_KEY:
        print("AI_ANALYSIS_ENABLED=True，但 AI_API_KEY 为空，跳过 AI 分析。")
        return []

    prompt = build_ai_analysis_prompt(trial_history, best_parameters, best_result, steps)
    request_body = build_ai_request_body(prompt)

    request = urllib.request.Request(
        AI_API_URL,
        data=json.dumps(request_body).encode("utf-8"),
        headers={
            "Authorization": "Bearer {}".format(AI_API_KEY),
            "Content-Type": "application/json",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=AI_API_TIMEOUT_S) as response:
            response_text = response.read().decode("utf-8", errors="ignore")
    except urllib.error.HTTPError as exc:
        error_text = exc.read().decode("utf-8", errors="ignore")
        print("AI API 请求失败，HTTP {}：{}".format(exc.code, error_text[:300]))
        return []
    except TimeoutError as exc:
        print("AI API 请求超时，跳过本次 AI 分析：{}".format(exc))
        return []
    except urllib.error.URLError as exc:
        print("AI API 请求失败，跳过本次 AI 分析：{}".format(exc))
        return []

    try:
        response_payload = json.loads(response_text)
        content = extract_ai_response_text(response_payload, response_text)
    except Exception:
        content = extract_ai_response_text({}, response_text)

    ai_json = extract_json_object(content)
    if not ai_json:
        print("AI 回复不是可解析 JSON，跳过本次 AI 分析：{}".format(content[:200]))
        return []

    print("AI 分析建议：{}".format(ai_json.get("reason", "")))
    candidates = ai_json.get("candidate_parameters", [])
    if not isinstance(candidates, list):
        return []

    clamped_candidates = []
    for candidate in candidates[:AI_MAX_CANDIDATES_PER_CALL]:
        if isinstance(candidate, dict):
            clamped_candidates.append(clamp_candidate_parameters(candidate, best_parameters))
    return clamped_candidates


def run_ai_suggested_trials(
    bridge: SerialBridge,
    writer,
    label: str,
    trial_history: list[dict],
    best_parameters: dict[str, float],
    best_result: TrialResult,
    steps: dict[str, float],
) -> tuple[dict[str, float], TrialResult]:
    """把 AI 推荐候选参数交给实车验证，并始终优先基于历史最佳参数去做保守调参。"""
    raise_if_cancel_requested()
    update_gate_score = best_result.score
    ai_reference_parameters = best_parameters
    ai_reference_result = best_result
    saved_best_snapshot = None
    if COMPARE_WITH_SAVED_BEST_SCORE:
        saved_best_snapshot = load_saved_best_snapshot()
        if saved_best_snapshot is not None:
            saved_parameters, saved_result = saved_best_snapshot
            if best_result.score + SCORE_IMPROVEMENT_EPS < saved_result.score:
                print("当前最佳参数已经优于历史最佳，先把当前最佳升级为新的历史最佳，再基于它继续调参。")
                if SAVE_BEST_RESULT_ON_EACH_UPDATE:
                    save_best_result(best_parameters, best_result, "better_than_saved_{}".format(label))
                ai_reference_parameters = best_parameters
                ai_reference_result = best_result
                update_gate_score = best_result.score
            else:
                print("本轮 AI 调参以历史最佳参数为基础，只有优于历史最佳分数 {:.4f} 才会更新。".format(
                    saved_result.score,
                ))
                ai_reference_parameters = saved_parameters
                ai_reference_result = saved_result
                update_gate_score = saved_result.score

    candidates = request_ai_candidates(trial_history, ai_reference_parameters, ai_reference_result, steps)
    for index, candidate in enumerate(candidates, start=1):
        raise_if_cancel_requested()
        if all(parameter_equal(name, candidate[name], ai_reference_parameters[name]) for name in PARAMETER_SPACE):
            continue
        trial_name = "ai_{}_{}".format(label, index)
        result = run_trial(bridge, candidate, trial_name)
        record_trial(writer, trial_history, trial_name, candidate, result)

        if result.score + SCORE_IMPROVEMENT_EPS < best_result.score and result.score + SCORE_IMPROVEMENT_EPS < update_gate_score:
            print("接受 AI 推荐参数，损失分数 {:.4f} -> {:.4f}".format(best_result.score, result.score))
            best_parameters = candidate
            best_result = result
            update_gate_score = result.score
            if SAVE_BEST_RESULT_ON_EACH_UPDATE:
                save_best_result(best_parameters, best_result, trial_name)
        elif saved_best_snapshot is not None and result.score + SCORE_IMPROVEMENT_EPS >= update_gate_score:
            print("AI 候选 {} 分数 {:.4f} 未优于历史最佳门槛 {:.4f}，不更新最佳参数。".format(
                trial_name,
                result.score,
                update_gate_score,
            ))
        persist_session_summary(best_parameters, best_result, trial_history, status="running")
    return best_parameters, best_result


def coordinate_search(bridge: SerialBridge) -> tuple[dict[str, float], TrialResult]:
    raise_if_cancel_requested()
    current = read_current_parameters(bridge)
    bridge.applied_parameters = current.copy()
    steps = {name: float(spec[3]) for name, spec in PARAMETER_SPACE.items()}

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    fieldnames = ["trial", "score"] + list(PARAMETER_SPACE.keys()) + [
        "usable_samples",
        "mean_abs_yaw_error",
        "rms_color_error",
        "rotate_std",
        "vx_std",
        "vy_std",
        "settled",
        "settle_time_s",
        "steady_mean_abs_yaw_error",
        "steady_rms_color_error",
        "steady_rotate_std",
        "steady_vx_std",
        "steady_vy_std",
        "locked_ratio",
        "center_ratio",
        "failure_reason",
    ]

    trial_log_path = TRIAL_LOG_CSV if ACTIVE_SESSION_LOG_PATHS is None else ACTIVE_SESSION_LOG_PATHS.trial_log_csv
    with trial_log_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        trial_history = []
        saved_best_snapshot = load_saved_best_snapshot()
        saved_best_parameters = saved_best_snapshot[0] if saved_best_snapshot is not None else None

        best_parameters = current.copy()
        best_result = run_trial(bridge, best_parameters, "baseline")
        record_trial(writer, trial_history, "baseline", best_parameters, best_result)
        if saved_best_snapshot is not None:
            saved_parameters, saved_result = saved_best_snapshot
            trial_history.append(build_trial_record("saved_best_history", saved_parameters, saved_result))
            persist_trial_history(trial_history)
            if saved_result.score + SCORE_IMPROVEMENT_EPS < best_result.score:
                print("历史最佳分数 {:.4f} 仍优于当前 baseline {:.4f}，本轮继续以历史最佳参数作为默认最佳。".format(
                    saved_result.score,
                    best_result.score,
                ))
                best_parameters = saved_parameters.copy()
                best_result = saved_result
            elif best_result.score + SCORE_IMPROVEMENT_EPS < saved_result.score:
                print("当前 baseline 已优于历史最佳，先把 baseline 升级为新的历史最佳。")
                if SAVE_BEST_RESULT_ON_EACH_UPDATE:
                    save_best_result(best_parameters, best_result, "baseline_better_than_saved")
        if SAVE_BEST_RESULT_ON_EACH_UPDATE and saved_best_parameters is None:
            save_best_result(best_parameters, best_result, "baseline")
        csv_file.flush()
        persist_session_summary(best_parameters, best_result, trial_history, status="running")

        if not LOCAL_COORDINATE_SEARCH_ENABLED:
            if not AI_ANALYSIS_ENABLED:
                raise RuntimeError("LOCAL_COORDINATE_SEARCH_ENABLED=False 时必须启用 AI_ANALYSIS_ENABLED，否则没有参数推荐来源。")
            for ai_round_index in range(1, AI_RECOMMENDATION_ROUNDS + 1):
                raise_if_cancel_requested()
                print("\n=== AI 推荐轮次 {} ===".format(ai_round_index))
                best_parameters, best_result = run_ai_suggested_trials(
                    bridge,
                    writer,
                    "round{}".format(ai_round_index),
                    trial_history,
                    best_parameters,
                    best_result,
                    steps,
                )
                csv_file.flush()
            return best_parameters, best_result

        if AI_SUGGEST_AFTER_BASELINE:
            best_parameters, best_result = run_ai_suggested_trials(
                bridge,
                writer,
                "after_baseline",
                trial_history,
                best_parameters,
                best_result,
                steps,
            )
            csv_file.flush()

        for round_index in range(1, MAX_SEARCH_ROUNDS + 1):
            raise_if_cancel_requested()
            print("\n=== 搜索轮次 {} ===".format(round_index))
            improved = False
            for name, spec in PARAMETER_SPACE.items():
                raise_if_cancel_requested()
                _, lower, upper, _, _ = spec
                base_value = best_parameters[name]
                best_local_parameters = best_parameters
                best_local_result = best_result

                for direction in (1.0, -1.0):
                    raise_if_cancel_requested()
                    candidate = best_parameters.copy()
                    candidate[name] = clamp(base_value + direction * steps[name], float(lower), float(upper))
                    if abs(candidate[name] - base_value) <= 1e-9:
                        continue

                    trial_name = "r{}_{}_{}".format(round_index, name, "plus" if direction > 0 else "minus")
                    result = run_trial(bridge, candidate, trial_name)
                    record_trial(writer, trial_history, trial_name, candidate, result)
                    csv_file.flush()

                    if result.score + SCORE_IMPROVEMENT_EPS < best_local_result.score:
                        best_local_result = result
                        best_local_parameters = candidate

                if best_local_result.score + SCORE_IMPROVEMENT_EPS < best_result.score:
                    print("接受新参数 {}={}，损失分数 {:.4f} -> {:.4f}".format(
                        name,
                        format_value(name, best_local_parameters[name]),
                        best_result.score,
                        best_local_result.score,
                    ))
                    best_parameters = best_local_parameters
                    best_result = best_local_result
                    improved = True
                    if SAVE_BEST_RESULT_ON_EACH_UPDATE:
                        save_best_result(best_parameters, best_result, "r{}_{}".format(round_index, name))
                    persist_session_summary(best_parameters, best_result, trial_history, status="running")

            if not improved:
                for name in steps:
                    steps[name] *= STEP_SHRINK
                print("本轮无显著改善，缩小步长：{}".format({
                    name: round(value, 5) for name, value in steps.items()
                }))
                persist_session_summary(best_parameters, best_result, trial_history, status="running")

            if AI_SUGGEST_AFTER_EACH_ROUND:
                best_parameters, best_result = run_ai_suggested_trials(
                    bridge,
                    writer,
                    "round{}".format(round_index),
                    trial_history,
                    best_parameters,
                    best_result,
                    steps,
                )
                csv_file.flush()

    return best_parameters, best_result


def write_back_source(parameters: dict[str, float]) -> None:
    if not WRITE_BACK_TO_SOURCE:
        return

    text = SOURCE_PARAMETER_FILE.read_text(encoding="utf-8")
    for name, value in parameters.items():
        formatted = format_value(name, value)
        pattern = re.compile(r"^(\s*{}\s*=\s*)([^#\r\n]*?)(\s*(?:#.*)?)$".format(re.escape(name)), re.MULTILINE)

        def replace(match):
            old_value = match.group(2)
            padded = formatted.ljust(max(len(old_value), len(formatted)))
            return "{}{}{}".format(match.group(1), padded, match.group(3))

        text, count = pattern.subn(replace, text, count=1)
        if count != 1:
            print("警告：未在源文件中找到参数 {}".format(name))

    SOURCE_PARAMETER_FILE.write_text(text, encoding="utf-8")
    print("已把最佳参数写回：{}".format(SOURCE_PARAMETER_FILE))


def save_best_result(parameters: dict[str, float], result: TrialResult, update_reason: str = "") -> None:
    """把当前最优参数原子保存到 JSON 文件，避免实车中途故障时丢失已经验证过的好参数。"""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "score": result.score,
        "parameters": parameters,
        "detail": result.detail,
        "update_reason": update_reason,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    tmp_path = BEST_RESULT_JSON.with_name(BEST_RESULT_JSON.name + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_path.replace(BEST_RESULT_JSON)
    print("最佳结果已保存：{}".format(BEST_RESULT_JSON))


def run_tuning_session(
    serial_settings: SerialSettings | None = None,
    session_name: str | None = None,
) -> tuple[dict[str, float], TrialResult, SessionLogPaths]:
    """执行一次完整自动调参会话，供命令行和 GUI 共同调用。"""
    clear_cancel_request()
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)
    effective_serial_settings = resolve_serial_settings(serial_settings)
    session_paths = create_session_log_paths(session_name)
    activate_session_logging(session_paths, effective_serial_settings)
    serial_module, list_ports_module = load_pyserial()
    bridge = None
    try:
        bridge = open_bridge(serial_module, list_ports_module, effective_serial_settings)
        pause_carb_before_parameter_read(bridge)
        require_ai_firmware(bridge)
        best_parameters, best_result = coordinate_search(bridge)
        apply_parameters(bridge, best_parameters)
        require_ok(
            command_with_retry(bridge, "AI_START", "AI_START", timeout_s=CONTROL_COMMAND_TIMEOUT_S),
            "AI_START",
        )
        if not SAVE_BEST_RESULT_ON_EACH_UPDATE:
            save_best_result(best_parameters, best_result, "final")
        write_back_source(best_parameters)
        trial_history = load_persisted_trial_history()
        persist_session_summary(best_parameters, best_result, trial_history, status="finished")
        print("\n=== 最佳参数 ===")
        for name, value in best_parameters.items():
            print("{}={}".format(name, format_value(name, value)))
        print("最佳损失分数：{:.4f}（越低越好）".format(best_result.score))
        return best_parameters, best_result, session_paths
    except CancellationRequestedError as exc:
        trial_history = load_persisted_trial_history()
        if bridge is not None:
            try:
                bridge.command("AI_STOP", timeout_s=CONTROL_COMMAND_TIMEOUT_S, drain_before=False)
            except Exception:
                pass
        persist_session_summary(None, None, trial_history, status="cancelled", error_message=str(exc))
        print("自动调参已按请求终止。")
        raise
    except Exception as exc:
        trial_history = load_persisted_trial_history()
        persist_session_summary(None, None, trial_history, status="failed", error_message=str(exc))
        raise
    finally:
        if bridge is not None:
            bridge.close()
        activate_session_logging(None)


def main() -> int:
    run_tuning_session()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中断，脚本已退出。")
        raise SystemExit(130)
