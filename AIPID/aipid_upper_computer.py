# -*- coding: utf-8 -*-
"""AIPID 上位机界面。

主要能力：
1. 手动配置串口参数并连接/断开 CarB。
2. 快速发送 framed 协议命令，只显示协议内回包，忽略协议外串口噪声。
3. 一键启动/终止自动调参，并在界面中实时显示日志。
4. 直接打开日志目录，方便查看每轮参数、分数和历史最优结果。
"""

from __future__ import annotations

import contextlib
import json
import os
import queue
import threading
import time
import traceback
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import auto_tune_carb_pid as tune


# ---------------------------------------------------------------------------
# 人工调试区
# 所有现场可能需要手动调整的 GUI 变量统一放在文件开头，便于直接修改。
# ---------------------------------------------------------------------------

GUI_WINDOW_TITLE = "AIPID 上位机"
GUI_WINDOW_SIZE = "1180x760"
GUI_QUEUE_POLL_INTERVAL_MS = 80            # 主线程轮询日志队列的周期，越小界面刷新越及时
GUI_SERIAL_POLL_INTERVAL_MS = 60           # 手动串口监视轮询周期，越小越容易及时显示协议帧
GUI_LOG_TIME_FORMAT = "%H:%M:%S"           # 界面日志时间戳格式
GUI_SHOW_PROTOCOL_ONLY = True              # True 时手动串口窗口只显示 AIPID 协议帧，屏蔽外部串口噪声
GUI_DEFAULT_QUICK_COMMAND = "AI_STATUS"    # 手动命令输入框默认值
GUI_PRESET_COMMANDS = (
    "AI_STATUS",
    "AI_STOP",
    "AI_START",
    "LIST",
    "GET PID_KP",
    "GET COLOR_MAX_TRACKING_SPEED",
)
GUI_DEFAULT_SESSION_PREFIX = "gui"         # GUI 发起自动调参时的会话名前缀
GUI_OPEN_LOG_DIR_AFTER_FINISH = False      # 自动调参结束后是否自动打开日志目录
GUI_MANUAL_APPEND_TX_PREFIX = True         # True 时在日志框里显示手动发送的 TX 命令
GUI_ALLOW_RAW_VIEW_SWITCH = True           # True 时保留“显示原始串口文本”的开关，便于极端排障
GUI_HISTORY_PAUSE_BEFORE_SEND = True       # True 时历史参数下发前先发送 AI_STOP，避免边运动边改参数
GUI_HISTORY_START_AFTER_SEND = True        # True 时历史参数下发完成后自动发送 AI_START，让小车按该轮参数运行
GUI_HISTORY_DEFAULT_DIR = tune.LOG_DIR / "sessions"  # 默认打开的历史调参会话目录；只读取，不改动里面的日志


class QueueWriter:
    """把 stdout/stderr 重定向到线程安全队列，供 GUI 主线程统一显示。"""

    def __init__(self, output_queue: queue.Queue):
        self.output_queue = output_queue

    def write(self, text: str) -> int:
        if text:
            self.output_queue.put(("text", text))
        return len(text or "")

    def flush(self) -> None:
        return None


class AIPIDUpperComputerApp:
    """桌面上位机主界面。"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(GUI_WINDOW_TITLE)
        self.root.geometry(GUI_WINDOW_SIZE)

        self.serial_module, self.list_ports_module = tune.load_pyserial()
        self.output_queue: queue.Queue = queue.Queue()
        self.manual_serial = None
        self.manual_rx_text_buf = ""
        self.manual_last_open_settings: tune.SerialSettings | None = None
        self.tuning_thread: threading.Thread | None = None
        self.tuning_running = False
        self.history_apply_thread: threading.Thread | None = None
        self.history_apply_running = False
        self.history_json_path: Path | None = None
        self.history_trials: list[dict] = []
        self.history_trial_by_label: dict[str, dict] = {}
        self.latest_session_dir: Path | None = None
        self.latest_summary_path: Path | None = None

        self._build_vars()
        self._build_layout()
        self._bind_events()
        self.refresh_ports()
        self._set_status("就绪")
        self.log(
            "上位机已启动。协议帧格式：{}AIPID|...{}".format(
                tune.TUNING_PROTOCOL_FRAME_HEAD,
                tune.TUNING_PROTOCOL_FRAME_TAIL,
            )
        )

        self.root.after(GUI_QUEUE_POLL_INTERVAL_MS, self._poll_output_queue)
        self.root.after(GUI_SERIAL_POLL_INTERVAL_MS, self._poll_manual_serial)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_vars(self) -> None:
        defaults = tune.resolve_serial_settings(None)
        self.port_var = tk.StringVar(value=defaults.port)
        self.keyword_var = tk.StringVar(value=defaults.port_keyword)
        self.baudrate_var = tk.StringVar(value=str(defaults.baudrate))
        self.bytesize_var = tk.StringVar(value=str(defaults.bytesize))
        self.stopbits_var = tk.StringVar(value=str(defaults.stopbits))
        self.parity_var = tk.StringVar(value=defaults.parity)
        self.flow_control_var = tk.StringVar(value=defaults.flow_control)
        self.timeout_var = tk.StringVar(value=str(defaults.timeout_s))
        self.quick_command_var = tk.StringVar(value=GUI_DEFAULT_QUICK_COMMAND)
        self.show_protocol_only_var = tk.BooleanVar(value=GUI_SHOW_PROTOCOL_ONLY)
        self.status_var = tk.StringVar(value="未连接")
        self.history_file_var = tk.StringVar(value="")
        self.history_trial_var = tk.StringVar(value="")
        self.history_preview_var = tk.StringVar(value="未选择历史参数文件")

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        serial_frame = ttk.LabelFrame(self.root, text="串口配置")
        serial_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        for column in range(8):
            serial_frame.columnconfigure(column, weight=1 if column in (1, 3, 5, 7) else 0)

        ttk.Label(serial_frame, text="端口").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, state="normal")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="关键字").grid(row=0, column=2, sticky="w", padx=6, pady=6)
        ttk.Entry(serial_frame, textvariable=self.keyword_var).grid(row=0, column=3, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="波特率").grid(row=0, column=4, sticky="w", padx=6, pady=6)
        ttk.Entry(serial_frame, textvariable=self.baudrate_var).grid(row=0, column=5, sticky="ew", padx=6, pady=6)

        ttk.Button(serial_frame, text="刷新端口", command=self.refresh_ports).grid(
            row=0, column=6, sticky="ew", padx=6, pady=6
        )
        ttk.Button(serial_frame, text="打开日志目录", command=self._open_log_dir).grid(
            row=0, column=7, sticky="ew", padx=6, pady=6
        )

        ttk.Label(serial_frame, text="数据位").grid(row=1, column=0, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            serial_frame,
            textvariable=self.bytesize_var,
            values=("5", "6", "7", "8"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="停止位").grid(row=1, column=2, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            serial_frame,
            textvariable=self.stopbits_var,
            values=("1", "1.5", "2"),
            state="readonly",
        ).grid(row=1, column=3, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="校验位").grid(row=1, column=4, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            serial_frame,
            textvariable=self.parity_var,
            values=("N", "E", "O", "M", "S"),
            state="readonly",
        ).grid(row=1, column=5, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="流控").grid(row=1, column=6, sticky="w", padx=6, pady=6)
        ttk.Combobox(
            serial_frame,
            textvariable=self.flow_control_var,
            values=("none", "xonxoff", "rtscts", "dsrdtr"),
            state="readonly",
        ).grid(row=1, column=7, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="超时(s)").grid(row=2, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(serial_frame, textvariable=self.timeout_var).grid(row=2, column=1, sticky="ew", padx=6, pady=6)

        ttk.Label(serial_frame, text="状态").grid(row=2, column=2, sticky="w", padx=6, pady=6)
        ttk.Label(serial_frame, textvariable=self.status_var).grid(row=2, column=3, columnspan=5, sticky="w", padx=6, pady=6)

        action_frame = ttk.Frame(self.root)
        action_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        action_frame.columnconfigure(0, weight=1)
        action_frame.columnconfigure(1, weight=1)

        manual_frame = ttk.LabelFrame(action_frame, text="手动串口")
        manual_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        manual_frame.columnconfigure(0, weight=1)
        manual_frame.columnconfigure(1, weight=0)
        manual_frame.columnconfigure(2, weight=0)
        manual_frame.columnconfigure(3, weight=0)

        self.quick_command_combo = ttk.Combobox(
            manual_frame,
            textvariable=self.quick_command_var,
            values=GUI_PRESET_COMMANDS,
            state="normal",
        )
        self.quick_command_combo.grid(row=0, column=0, sticky="ew", padx=6, pady=6)

        self.connect_button = ttk.Button(manual_frame, text="连接", command=self._connect_manual_serial)
        self.connect_button.grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        self.disconnect_button = ttk.Button(
            manual_frame,
            text="断开",
            command=self._disconnect_manual_serial,
            state="disabled",
        )
        self.disconnect_button.grid(row=0, column=2, sticky="ew", padx=6, pady=6)

        self.send_button = ttk.Button(
            manual_frame,
            text="发送命令",
            command=self._send_manual_command,
            state="disabled",
        )
        self.send_button.grid(row=0, column=3, sticky="ew", padx=6, pady=6)

        option_frame = ttk.Frame(manual_frame)
        option_frame.grid(row=1, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 6))
        ttk.Checkbutton(
            option_frame,
            text="仅显示协议内容",
            variable=self.show_protocol_only_var,
        ).grid(row=0, column=0, sticky="w", padx=(0, 12))
        if GUI_ALLOW_RAW_VIEW_SWITCH:
            ttk.Label(
                option_frame,
                text="关闭后可临时查看原始串口文本，用于极端排障。",
            ).grid(row=0, column=1, sticky="w")

        tuning_frame = ttk.LabelFrame(action_frame, text="自动调参")
        tuning_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        for column in range(4):
            tuning_frame.columnconfigure(column, weight=1)

        self.start_tuning_button = ttk.Button(tuning_frame, text="启动自动调参", command=self._start_tuning)
        self.start_tuning_button.grid(row=0, column=0, sticky="ew", padx=6, pady=6)

        self.stop_tuning_button = ttk.Button(
            tuning_frame,
            text="终止自动调参",
            command=self._stop_tuning,
            state="disabled",
        )
        self.stop_tuning_button.grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        ttk.Button(tuning_frame, text="打开本次日志", command=self._open_latest_session_dir).grid(
            row=0, column=2, sticky="ew", padx=6, pady=6
        )
        ttk.Button(tuning_frame, text="清空界面日志", command=self._clear_log).grid(
            row=0, column=3, sticky="ew", padx=6, pady=6
        )

        desc = (
            "自动调参启动后，界面会调用 auto_tune_carb_pid.py 的完整流程。"
            "终止按钮会发送协作取消信号，脚本会尽快发送 AI_STOP 并收尾写日志。"
        )
        ttk.Label(tuning_frame, text=desc, wraplength=470, justify="left").grid(
            row=1, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 6)
        )

        history_frame = ttk.LabelFrame(action_frame, text="历史参数下发")
        history_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        history_frame.columnconfigure(1, weight=1)
        history_frame.columnconfigure(3, weight=1)

        ttk.Label(history_frame, text="历史文件").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(history_frame, textvariable=self.history_file_var, state="readonly").grid(
            row=0, column=1, sticky="ew", padx=6, pady=6
        )
        ttk.Button(history_frame, text="选择 trial_history.json", command=self._select_history_file).grid(
            row=0, column=2, sticky="ew", padx=6, pady=6
        )

        ttk.Label(history_frame, text="轮次").grid(row=0, column=3, sticky="w", padx=6, pady=6)
        self.history_trial_combo = ttk.Combobox(
            history_frame,
            textvariable=self.history_trial_var,
            state="readonly",
        )
        self.history_trial_combo.grid(row=0, column=4, sticky="ew", padx=6, pady=6)
        self.history_trial_combo.bind("<<ComboboxSelected>>", self._on_history_trial_selected)

        self.send_history_button = ttk.Button(
            history_frame,
            text="发送选中轮次并运行",
            command=self._send_selected_history_trial,
            state="disabled",
        )
        self.send_history_button.grid(row=0, column=5, sticky="ew", padx=6, pady=6)

        ttk.Label(history_frame, textvariable=self.history_preview_var, wraplength=1040, justify="left").grid(
            row=1, column=0, columnspan=6, sticky="w", padx=6, pady=(0, 6)
        )

        log_frame = ttk.LabelFrame(self.root, text="运行日志")
        log_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=(6, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, wrap="word", height=24)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state="disabled")

        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scroll.set)

    def _bind_events(self) -> None:
        self.root.bind("<Return>", self._on_enter_key)

    def _on_enter_key(self, _event) -> None:
        focus_widget = self.root.focus_get()
        if focus_widget in (self.quick_command_combo,):
            self._send_manual_command()

    def _build_serial_settings(self) -> tune.SerialSettings:
        return tune.SerialSettings(
            port=self.port_var.get().strip(),
            port_keyword=self.keyword_var.get().strip(),
            baudrate=int(float(self.baudrate_var.get().strip())),
            bytesize=int(float(self.bytesize_var.get().strip())),
            stopbits=float(self.stopbits_var.get().strip()),
            parity=self.parity_var.get().strip().upper(),
            flow_control=self.flow_control_var.get().strip().lower(),
            timeout_s=float(self.timeout_var.get().strip()),
        ).normalized()

    def refresh_ports(self) -> None:
        try:
            ports = tune.list_available_ports(self.list_ports_module, self.keyword_var.get().strip())
        except Exception as exc:
            self.log("刷新端口失败：{}".format(exc))
            return

        self.port_combo["values"] = ports
        if not self.port_var.get().strip() and ports:
            self.port_var.set(ports[0])
        self.log("已刷新串口列表：{}".format(", ".join(ports) if ports else "未发现串口"))

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def log(self, text: str) -> None:
        timestamp = time.strftime(GUI_LOG_TIME_FORMAT)
        normalized = str(text).replace("\r\n", "\n").replace("\r", "\n")

        self.log_text.configure(state="normal")
        for piece in normalized.split("\n"):
            if not piece:
                continue
            self.log_text.insert("end", "[{}] {}\n".format(timestamp, piece))
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _connect_manual_serial(self) -> None:
        if self.tuning_running:
            messagebox.showwarning("提示", "自动调参进行中，请先终止后再手动连接串口。")
            return
        if self.manual_serial is not None:
            self.log("手动串口已连接，无需重复连接。")
            return

        try:
            settings = self._build_serial_settings()
            if not settings.port:
                raise ValueError("请先选择串口号。")

            self.manual_serial = self.serial_module.Serial(
                **tune.build_serial_open_kwargs(self.serial_module, settings.port, settings)
            )
            if hasattr(self.manual_serial, "reset_input_buffer"):
                self.manual_serial.reset_input_buffer()
            if hasattr(self.manual_serial, "reset_output_buffer"):
                self.manual_serial.reset_output_buffer()
            self.manual_rx_text_buf = ""
            self.manual_last_open_settings = settings
            self.connect_button.configure(state="disabled")
            self.disconnect_button.configure(state="normal")
            self.send_button.configure(state="normal")
            self._set_status("手动已连接：{}".format(settings.port))
            self.log("手动串口已连接：{} @ {}".format(settings.port, settings.baudrate))
        except Exception as exc:
            self.manual_serial = None
            self.log("手动连接失败：{}".format(exc))
            messagebox.showerror("连接失败", str(exc))

    def _disconnect_manual_serial(self) -> None:
        if self.manual_serial is None:
            return

        try:
            self.manual_serial.close()
        except Exception:
            pass
        finally:
            self.manual_serial = None
            self.manual_rx_text_buf = ""
            self.connect_button.configure(state="normal")
            self.disconnect_button.configure(state="disabled")
            self.send_button.configure(state="disabled")
            self._set_status("手动串口已断开")
            self.log("手动串口已断开。")

    def _manual_write_protocol_command(self, command_text: str) -> None:
        if self.manual_serial is None:
            raise RuntimeError("手动串口未连接。")

        payload = tune.wrap_protocol_command(command_text)
        if not payload:
            raise ValueError("命令为空，无法发送。")

        self.manual_serial.write((payload + "\n").encode("utf-8"))
        self.manual_serial.flush()
        if GUI_MANUAL_APPEND_TX_PREFIX:
            self.log("TX {}".format(command_text.strip()))

    def _send_manual_command(self) -> None:
        command_text = self.quick_command_var.get().strip()
        if not command_text:
            messagebox.showwarning("提示", "请输入要发送的命令。")
            return

        try:
            self._manual_write_protocol_command(command_text)
        except Exception as exc:
            self.log("发送命令失败：{}".format(exc))
            messagebox.showerror("发送失败", str(exc))

    def _select_history_file(self) -> None:
        """选择并加载历史 trial_history.json；只读取文件，不修改历史日志目录。"""
        initial_dir = GUI_HISTORY_DEFAULT_DIR if GUI_HISTORY_DEFAULT_DIR.exists() else tune.LOG_DIR
        file_path = filedialog.askopenfilename(
            title="选择 trial_history.json",
            initialdir=str(initial_dir),
            filetypes=(
                ("trial_history.json", "trial_history.json"),
                ("JSON 文件", "*.json"),
                ("所有文件", "*.*"),
            ),
        )
        if not file_path:
            return
        try:
            self._load_history_file(Path(file_path))
        except Exception as exc:
            self.history_trials = []
            self.history_trial_by_label = {}
            self.history_trial_combo["values"] = ()
            self.history_trial_var.set("")
            self.send_history_button.configure(state="disabled")
            self.history_preview_var.set("历史文件加载失败：{}".format(exc))
            self.log("历史参数文件加载失败：{}".format(exc))
            messagebox.showerror("加载失败", str(exc))

    def _load_history_file(self, file_path: Path) -> None:
        """读取 trial_history.json 中的 trials 列表，并填充轮次下拉框。"""
        payload = json.loads(file_path.read_text(encoding="utf-8"))
        trials = payload.get("trials")
        if not isinstance(trials, list):
            raise ValueError("JSON 中没有 trials 列表")

        self.history_json_path = file_path
        self.history_trials = [trial for trial in trials if isinstance(trial.get("parameters"), dict)]
        self.history_trial_by_label = {}
        labels = []
        for index, trial in enumerate(self.history_trials, start=1):
            trial_name = str(trial.get("trial", "trial{}".format(index)))
            score = trial.get("score", "NA")
            params = trial.get("parameters", {})
            label = "{}. {} | score={} | {} params".format(index, trial_name, score, len(params))
            labels.append(label)
            self.history_trial_by_label[label] = trial

        if not labels:
            raise ValueError("没有找到带 parameters 的历史轮次")

        self.history_file_var.set(str(file_path))
        self.history_trial_combo["values"] = labels
        self.history_trial_var.set(labels[0])
        self.send_history_button.configure(state="normal")
        self._update_history_preview(labels[0])
        self.log("已加载历史参数文件：{}，可选轮次 {} 个。".format(file_path, len(labels)))

    def _on_history_trial_selected(self, _event=None) -> None:
        self._update_history_preview(self.history_trial_var.get())

    def _update_history_preview(self, label: str) -> None:
        trial = self.history_trial_by_label.get(label)
        if not trial:
            self.history_preview_var.set("未选择有效历史轮次")
            return

        parameters = trial.get("parameters", {})
        names = [name for name in tune.PARAMETER_SPACE.keys() if name in parameters]
        preview_items = []
        for name in names[:6]:
            preview_items.append("{}={}".format(name, parameters[name]))
        more_text = "" if len(names) <= 6 else "，另有 {} 个参数".format(len(names) - 6)
        self.history_preview_var.set(
            "将下发：{}；score={}；{}{}".format(
                trial.get("trial", "未命名轮次"),
                trial.get("score", "NA"),
                "，".join(preview_items),
                more_text,
            )
        )

    def _selected_history_parameters(self) -> tuple[str, dict[str, float]]:
        label = self.history_trial_var.get()
        trial = self.history_trial_by_label.get(label)
        if not trial:
            raise ValueError("请先选择一个历史轮次")

        raw_parameters = trial.get("parameters", {})
        parameters: dict[str, float] = {}
        for name, spec in tune.PARAMETER_SPACE.items():
            if name not in raw_parameters:
                continue
            value_type = spec[4]
            raw_value = raw_parameters[name]
            parameters[name] = value_type(raw_value) if value_type is int else float(raw_value)

        if not parameters:
            raise ValueError("该轮次没有可下发的受支持参数")

        trial_name = str(trial.get("trial", label))
        return trial_name, parameters

    def _send_selected_history_trial(self) -> None:
        if self.tuning_running:
            messagebox.showwarning("提示", "自动调参进行中，不能同时下发历史参数。")
            return
        if self.history_apply_running:
            return

        try:
            trial_name, parameters = self._selected_history_parameters()
            settings = self._build_serial_settings()
            if not settings.port and not settings.port_keyword:
                raise ValueError("请先选择串口，或填写串口关键字用于自动探测。")
        except Exception as exc:
            messagebox.showerror("历史参数下发失败", str(exc))
            return

        if self.manual_serial is not None:
            self.log("历史参数下发前先断开手动串口，避免同一个端口被重复占用。")
            self._disconnect_manual_serial()

        self.history_apply_running = True
        self.start_tuning_button.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.send_button.configure(state="disabled")
        self.disconnect_button.configure(state="disabled")
        self.send_history_button.configure(state="disabled")
        self._set_status("正在下发历史参数")
        self.log("准备下发历史轮次 {} 的 {} 个参数。".format(trial_name, len(parameters)))

        self.history_apply_thread = threading.Thread(
            target=self._run_history_apply_worker,
            args=(settings, trial_name, parameters),
            daemon=True,
        )
        self.history_apply_thread.start()

    def _run_history_apply_worker(
        self,
        serial_settings: tune.SerialSettings,
        trial_name: str,
        parameters: dict[str, float],
    ) -> None:
        writer = QueueWriter(self.output_queue)
        bridge = None
        try:
            with contextlib.redirect_stdout(writer), contextlib.redirect_stderr(writer):
                print("历史参数下发开始：{}".format(trial_name))
                bridge = tune.open_bridge(self.serial_module, self.list_ports_module, serial_settings)
                tune.require_ai_firmware(bridge)
                if GUI_HISTORY_PAUSE_BEFORE_SEND:
                    tune.require_ok(
                        tune.command_with_retry(
                            bridge,
                            "AI_STOP",
                            "AI_STOP",
                            timeout_s=tune.CONTROL_COMMAND_TIMEOUT_S,
                        ),
                        "AI_STOP",
                    )
                tune.apply_parameters(bridge, parameters)
                if GUI_HISTORY_START_AFTER_SEND:
                    tune.require_ok(
                        tune.command_with_retry(
                            bridge,
                            "AI_START",
                            "AI_START",
                            timeout_s=tune.CONTROL_COMMAND_TIMEOUT_S,
                        ),
                        "AI_START",
                    )
                print("历史参数下发完成：{}".format(trial_name))
            self.output_queue.put(("history_done", trial_name))
        except Exception:
            self.output_queue.put(("traceback", traceback.format_exc()))
            self.output_queue.put(("history_failed", trial_name))
        finally:
            if bridge is not None:
                with contextlib.suppress(Exception):
                    bridge.close()
            self.output_queue.put(("history_worker_finished", ""))

    def _emit_manual_protocol_frames(self, raw_text: str) -> None:
        self.manual_rx_text_buf += raw_text
        frames, self.manual_rx_text_buf = tune.extract_protocol_frames_from_buffer(
            self.manual_rx_text_buf,
            allow_legacy=False,
        )
        for payload in frames:
            self.log("RX {}".format(payload))

    def _emit_manual_raw_text(self, raw_text: str) -> None:
        normalized = raw_text.replace("\r", "\n")
        for piece in normalized.split("\n"):
            piece = piece.strip()
            if piece:
                self.log("RAW {}".format(piece))

    def _poll_manual_serial(self) -> None:
        try:
            if self.manual_serial is not None and not self.tuning_running:
                read_size = getattr(self.manual_serial, "in_waiting", 0) or 1
                raw = self.manual_serial.read(read_size)
                if raw:
                    raw_text = raw.decode("utf-8", errors="ignore")
                    if self.show_protocol_only_var.get():
                        self._emit_manual_protocol_frames(raw_text)
                    else:
                        self._emit_manual_raw_text(raw_text)
        except Exception as exc:
            self.log("手动串口读取异常：{}".format(exc))
            self._disconnect_manual_serial()
        finally:
            self.root.after(GUI_SERIAL_POLL_INTERVAL_MS, self._poll_manual_serial)

    def _start_tuning(self) -> None:
        if self.tuning_running:
            return

        try:
            settings = self._build_serial_settings()
        except Exception as exc:
            messagebox.showerror("串口配置错误", str(exc))
            return

        if self.manual_serial is not None:
            self.log("自动调参启动前，先断开手动串口，避免同一端口被重复占用。")
            self._disconnect_manual_serial()

        self.tuning_running = True
        self.start_tuning_button.configure(state="disabled")
        self.stop_tuning_button.configure(state="normal")
        self.connect_button.configure(state="disabled")
        self.send_button.configure(state="disabled")
        self.disconnect_button.configure(state="disabled")
        self.send_history_button.configure(state="disabled")
        self._set_status("自动调参运行中")
        self.log("准备启动自动调参。")

        session_name = "{}-{}".format(
            GUI_DEFAULT_SESSION_PREFIX,
            time.strftime("%Y-%m-%d-%H-%M-%S"),
        )
        self.tuning_thread = threading.Thread(
            target=self._run_tuning_worker,
            args=(settings, session_name),
            daemon=True,
        )
        self.tuning_thread.start()

    def _stop_tuning(self) -> None:
        if not self.tuning_running:
            return
        tune.request_cancel()
        self.stop_tuning_button.configure(state="disabled")
        self._set_status("正在终止自动调参")
        self.log("已请求终止自动调参，等待脚本完成收尾。")

    def _run_tuning_worker(self, serial_settings: tune.SerialSettings, session_name: str) -> None:
        writer = QueueWriter(self.output_queue)
        try:
            with contextlib.redirect_stdout(writer), contextlib.redirect_stderr(writer):
                best_parameters, best_result, session_paths = tune.run_tuning_session(
                    serial_settings=serial_settings,
                    session_name=session_name,
                )
            self.output_queue.put(("session_dir", str(session_paths.session_dir)))
            self.output_queue.put(("summary_path", str(session_paths.summary_json)))
            self.output_queue.put(
                (
                    "done",
                    "自动调参完成。最佳分数：{:.4f}，日志目录：{}".format(
                        best_result.score,
                        session_paths.session_dir,
                    ),
                )
            )
            self.output_queue.put(("best_params", best_parameters))
        except tune.CancellationRequestedError:
            self.output_queue.put(("cancelled", "自动调参已按请求终止。"))
        except Exception:
            self.output_queue.put(("traceback", traceback.format_exc()))
            self.output_queue.put(("failed", "自动调参失败。"))
        finally:
            self.output_queue.put(("worker_finished", ""))

    def _poll_output_queue(self) -> None:
        try:
            while True:
                item_type, payload = self.output_queue.get_nowait()
                if item_type == "text":
                    self.log(str(payload))
                elif item_type == "traceback":
                    self.log(str(payload))
                elif item_type == "session_dir":
                    self.latest_session_dir = Path(str(payload))
                elif item_type == "summary_path":
                    self.latest_summary_path = Path(str(payload))
                elif item_type == "best_params":
                    best_parameters = payload
                    self.log("本次最佳参数如下：")
                    for name, value in best_parameters.items():
                        self.log("  {}={}".format(name, tune.format_value(name, value)))
                elif item_type == "done":
                    self.log(str(payload))
                    self._set_status("自动调参已完成")
                    if GUI_OPEN_LOG_DIR_AFTER_FINISH:
                        self._open_latest_session_dir()
                elif item_type == "cancelled":
                    self.log(str(payload))
                    self._set_status("自动调参已终止")
                elif item_type == "failed":
                    self.log(str(payload))
                    self._set_status("自动调参失败")
                elif item_type == "worker_finished":
                    self._restore_idle_buttons_after_tuning()
                elif item_type == "history_done":
                    self.log("历史参数已下发并运行：{}".format(payload))
                    self._set_status("历史参数下发完成")
                elif item_type == "history_failed":
                    self.log("历史参数下发失败：{}".format(payload))
                    self._set_status("历史参数下发失败")
                elif item_type == "history_worker_finished":
                    self._restore_idle_buttons_after_history_apply()
        except queue.Empty:
            pass
        finally:
            self.root.after(GUI_QUEUE_POLL_INTERVAL_MS, self._poll_output_queue)

    def _restore_idle_buttons_after_tuning(self) -> None:
        self.tuning_running = False
        self.tuning_thread = None
        self.start_tuning_button.configure(state="normal")
        self.stop_tuning_button.configure(state="disabled")
        self.connect_button.configure(state="normal")
        if self.manual_serial is not None:
            self.disconnect_button.configure(state="normal")
            self.send_button.configure(state="normal")
        else:
            self.disconnect_button.configure(state="disabled")
            self.send_button.configure(state="disabled")
        if self.history_trial_by_label:
            self.send_history_button.configure(state="normal")

    def _restore_idle_buttons_after_history_apply(self) -> None:
        self.history_apply_running = False
        self.history_apply_thread = None
        self.start_tuning_button.configure(state="normal")
        self.connect_button.configure(state="normal")
        if self.manual_serial is not None:
            self.disconnect_button.configure(state="normal")
            self.send_button.configure(state="normal")
        else:
            self.disconnect_button.configure(state="disabled")
            self.send_button.configure(state="disabled")
        if self.history_trial_by_label:
            self.send_history_button.configure(state="normal")

    def _open_log_dir(self) -> None:
        self._open_path(tune.LOG_DIR)

    def _open_latest_session_dir(self) -> None:
        if self.latest_session_dir is not None and self.latest_session_dir.exists():
            self._open_path(self.latest_session_dir)
            return
        self._open_path(tune.LOG_DIR)

    def _open_path(self, path: Path) -> None:
        try:
            resolved = Path(path).resolve()
            resolved.mkdir(parents=True, exist_ok=True)
            os.startfile(str(resolved))
        except Exception as exc:
            self.log("打开目录失败：{}".format(exc))
            messagebox.showerror("打开失败", str(exc))

    def _on_close(self) -> None:
        if self.tuning_running:
            if not messagebox.askyesno("确认退出", "自动调参仍在运行，是否先发送终止请求并退出界面？"):
                return
            tune.request_cancel()

        if self.history_apply_running:
            if not messagebox.askyesno("确认退出", "历史参数仍在下发，强行退出可能导致参数未发完。是否继续退出？"):
                return

        self._disconnect_manual_serial()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    app = AIPIDUpperComputerApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
