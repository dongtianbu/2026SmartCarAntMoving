# -*- coding: utf-8 -*-
"""AIPID 上位机界面。"""

from __future__ import annotations

import os
import queue
import threading
import time
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from tkinter import BOTH, END, LEFT, W, X, messagebox, StringVar, Tk
from tkinter import scrolledtext, ttk

import auto_tune_carb_pid as tune


# ---------------------------------------------------------------------------
# 人工调试区
# 所有需要人工改动的变量集中放在文件顶部，便于现场快速调整界面默认行为。
# ---------------------------------------------------------------------------

APP_TITLE = "AIPID 上位机"                        # 窗口标题
WINDOW_SIZE = "1280x820"                          # 上位机默认窗口尺寸
UI_QUEUE_POLL_MS = 100                            # 主线程轮询日志队列的周期，单位毫秒
SERIAL_READ_THREAD_JOIN_MS = 800                  # 断开手动串口连接时，等待读线程退出的最长时间
STOP_CONFIRM_MESSAGE = "确认终止当前自动调参吗？程序会尽量发送 AI_STOP 让从车先停下。"  # 点击终止按钮时的确认提示

DEFAULT_BAUDRATE = str(tune.BAUDRATE)             # 波特率默认值
DEFAULT_BYTESIZE = str(tune.SERIAL_BYTESIZE)      # 数据位默认值
DEFAULT_STOPBITS = "1"                            # 停止位默认值
DEFAULT_PARITY_LABEL = "无"                       # 校验位默认值
DEFAULT_FLOW_CONTROL_LABEL = "无"                 # 流控默认值
DEFAULT_TIMEOUT_S = tune.SERIAL_TIMEOUT_S         # 串口超时默认值

PRESET_COMMANDS = [                               # 快速发送区的常用命令按钮
    ("AI_STATUS", "AI_STATUS"),
    ("AI_STOP", "AI_STOP"),
    ("AI_START", "AI_START"),
    ("LIST", "LIST"),
    ("HELP", "HELP"),
]


PARITY_OPTIONS = [("无", "N"), ("偶", "E"), ("奇", "O"), ("Mark", "M"), ("Space", "S")]
FLOW_CONTROL_OPTIONS = [
    ("无", "none"),
    ("软件流控(XON/XOFF)", "xonxoff"),
    ("RTS/CTS", "rtscts"),
    ("DSR/DTR", "dsrdtr"),
]
STOPBITS_OPTIONS = [("1", 1.0), ("1.5", 1.5), ("2", 2.0)]
BYTESIZE_OPTIONS = ["5", "6", "7", "8"]


class QueueLogWriter:
    """把 print 输出同时写入控制台文件和 GUI 文本框。"""

    def __init__(self, event_queue: queue.Queue, log_file):
        self.event_queue = event_queue
        self.log_file = log_file
        self._buffer = ""

    def write(self, text: str) -> int:
        if not text:
            return 0
        self.log_file.write(text)
        self.log_file.flush()

        self._buffer += text
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self.event_queue.put(("log_line", line))
        return len(text)

    def flush(self) -> None:
        self.log_file.flush()
        if self._buffer:
            self.event_queue.put(("log_line", self._buffer))
            self._buffer = ""


class UpperComputerApp:
    """AIPID 上位机主界面。"""

    def __init__(self, root: Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry(WINDOW_SIZE)

        self.serial_module, self.list_ports_module = tune.load_pyserial()
        self.event_queue: queue.Queue = queue.Queue()

        self.manual_serial = None
        self.manual_reader_thread = None
        self.manual_reader_stop = threading.Event()

        self.tuning_thread = None
        self.tuning_running = False
        self.current_session_dir = ""

        self.port_var = StringVar()
        self.baudrate_var = StringVar(value=DEFAULT_BAUDRATE)
        self.bytesize_var = StringVar(value=DEFAULT_BYTESIZE)
        self.stopbits_var = StringVar(value=DEFAULT_STOPBITS)
        self.parity_label_var = StringVar(value=DEFAULT_PARITY_LABEL)
        self.flow_control_label_var = StringVar(value=DEFAULT_FLOW_CONTROL_LABEL)
        self.timeout_var = StringVar(value=str(DEFAULT_TIMEOUT_S))
        self.quick_command_var = StringVar()
        self.connection_state_var = StringVar(value="未连接")
        self.session_state_var = StringVar(value="尚未启动自动调参")
        self.log_dir_var = StringVar(value=str(tune.LOG_DIR))

        self._build_ui()
        self._refresh_ports()
        self._poll_event_queue()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        serial_frame = ttk.LabelFrame(self.root, text="串口参数")
        serial_frame.pack(fill=X, padx=10, pady=8)

        ttk.Label(serial_frame, text="端口").grid(row=0, column=0, padx=6, pady=6, sticky=W)
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="波特率").grid(row=0, column=2, padx=6, pady=6, sticky=W)
        ttk.Entry(serial_frame, textvariable=self.baudrate_var, width=12).grid(row=0, column=3, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="数据位").grid(row=0, column=4, padx=6, pady=6, sticky=W)
        ttk.Combobox(serial_frame, textvariable=self.bytesize_var, width=8, state="readonly", values=BYTESIZE_OPTIONS).grid(row=0, column=5, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="停止位").grid(row=0, column=6, padx=6, pady=6, sticky=W)
        ttk.Combobox(
            serial_frame,
            textvariable=self.stopbits_var,
            width=8,
            state="readonly",
            values=[label for label, _ in STOPBITS_OPTIONS],
        ).grid(row=0, column=7, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="校验位").grid(row=1, column=0, padx=6, pady=6, sticky=W)
        ttk.Combobox(
            serial_frame,
            textvariable=self.parity_label_var,
            width=12,
            state="readonly",
            values=[label for label, _ in PARITY_OPTIONS],
        ).grid(row=1, column=1, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="流控").grid(row=1, column=2, padx=6, pady=6, sticky=W)
        ttk.Combobox(
            serial_frame,
            textvariable=self.flow_control_label_var,
            width=18,
            state="readonly",
            values=[label for label, _ in FLOW_CONTROL_OPTIONS],
        ).grid(row=1, column=3, padx=6, pady=6, sticky=W)

        ttk.Label(serial_frame, text="超时(s)").grid(row=1, column=4, padx=6, pady=6, sticky=W)
        ttk.Entry(serial_frame, textvariable=self.timeout_var, width=12).grid(row=1, column=5, padx=6, pady=6, sticky=W)

        ttk.Button(serial_frame, text="刷新端口", command=self._refresh_ports).grid(row=0, column=8, padx=8, pady=6)
        ttk.Button(serial_frame, text="连接串口", command=self._connect_manual_serial).grid(row=0, column=9, padx=8, pady=6)
        ttk.Button(serial_frame, text="断开串口", command=self._disconnect_manual_serial).grid(row=1, column=8, padx=8, pady=6)
        ttk.Label(serial_frame, textvariable=self.connection_state_var).grid(row=1, column=9, padx=8, pady=6, sticky=W)

        command_frame = ttk.LabelFrame(self.root, text="快速发送指令")
        command_frame.pack(fill=X, padx=10, pady=8)

        ttk.Entry(command_frame, textvariable=self.quick_command_var).pack(side=LEFT, fill=X, expand=True, padx=6, pady=8)
        ttk.Button(command_frame, text="发送", command=self._send_manual_command).pack(side=LEFT, padx=6, pady=8)

        preset_frame = ttk.Frame(command_frame)
        preset_frame.pack(side=LEFT, padx=10)
        for label, command in PRESET_COMMANDS:
            ttk.Button(
                preset_frame,
                text=label,
                command=lambda cmd=command: self._send_manual_command(cmd),
            ).pack(side=LEFT, padx=4)

        action_frame = ttk.LabelFrame(self.root, text="自动调参与日志")
        action_frame.pack(fill=X, padx=10, pady=8)

        self.start_tuning_button = ttk.Button(action_frame, text="启动自动调参", command=self._start_tuning)
        self.start_tuning_button.pack(side=LEFT, padx=6, pady=8)
        self.stop_tuning_button = ttk.Button(action_frame, text="终止自动调参", command=self._stop_tuning, state="disabled")
        self.stop_tuning_button.pack(side=LEFT, padx=6, pady=8)
        ttk.Button(action_frame, text="打开最新会话日志", command=self._open_latest_session_dir).pack(side=LEFT, padx=6, pady=8)
        ttk.Button(action_frame, text="打开总日志目录", command=self._open_log_root).pack(side=LEFT, padx=6, pady=8)

        ttk.Label(action_frame, text="当前状态：").pack(side=LEFT, padx=(18, 2))
        ttk.Label(action_frame, textvariable=self.session_state_var).pack(side=LEFT, padx=4)

        log_path_frame = ttk.Frame(self.root)
        log_path_frame.pack(fill=X, padx=10, pady=(0, 8))
        ttk.Label(log_path_frame, text="日志目录：").pack(side=LEFT)
        ttk.Label(log_path_frame, textvariable=self.log_dir_var).pack(side=LEFT, fill=X, expand=True)

        self.log_text = scrolledtext.ScrolledText(self.root, wrap="word", font=("Consolas", 10))
        self.log_text.pack(fill=BOTH, expand=True, padx=10, pady=(0, 10))
        self.log_text.insert(END, "AIPID 上位机已启动。\n")
        self.log_text.configure(state="disabled")

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert(END, text + "\n")
        self.log_text.see(END)
        self.log_text.configure(state="disabled")

    def _refresh_ports(self) -> None:
        ports = tune.list_available_ports(self.list_ports_module)
        self.port_combo["values"] = ports
        if ports and (not self.port_var.get() or self.port_var.get() not in ports):
            self.port_var.set(ports[0])
        self._append_log("已刷新串口列表：{}".format(", ".join(ports) if ports else "无"))

    def _build_serial_settings(self, require_port: bool) -> tune.SerialSettings:
        stopbits_value = next(value for label, value in STOPBITS_OPTIONS if label == self.stopbits_var.get())
        parity_value = next(value for label, value in PARITY_OPTIONS if label == self.parity_label_var.get())
        flow_control_value = next(value for label, value in FLOW_CONTROL_OPTIONS if label == self.flow_control_label_var.get())

        settings = tune.SerialSettings(
            port=self.port_var.get().strip(),
            baudrate=int(self.baudrate_var.get().strip()),
            bytesize=int(self.bytesize_var.get().strip()),
            stopbits=float(stopbits_value),
            parity=parity_value,
            flow_control=flow_control_value,
            timeout_s=float(self.timeout_var.get().strip()),
        ).normalized()
        if require_port and not settings.port:
            raise ValueError("请先选择串口端口。")
        return settings

    def _connect_manual_serial(self) -> None:
        if self.manual_serial is not None:
            self._append_log("手动串口已连接，无需重复连接。")
            return
        if self.tuning_running:
            messagebox.showwarning(APP_TITLE, "自动调参进行中，请等待当前任务结束后再手动连接串口。")
            return

        try:
            settings = self._build_serial_settings(require_port=True)
            kwargs = tune.build_serial_open_kwargs(self.serial_module, settings.port, settings)
            self.manual_serial = self.serial_module.Serial(**kwargs)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, "连接串口失败：{}".format(exc))
            return

        self.manual_reader_stop.clear()
        self.manual_reader_thread = threading.Thread(target=self._manual_serial_read_loop, daemon=True)
        self.manual_reader_thread.start()
        self.connection_state_var.set("已连接 {}".format(settings.port))
        self._append_log("已连接手动串口：{}，参数={} {} {} {} {}".format(
            settings.port,
            settings.baudrate,
            settings.bytesize,
            settings.stopbits,
            settings.parity,
            settings.flow_control,
        ))

    def _disconnect_manual_serial(self) -> None:
        if self.manual_serial is None:
            self.connection_state_var.set("未连接")
            return

        self.manual_reader_stop.set()
        serial_obj = self.manual_serial
        self.manual_serial = None
        try:
            serial_obj.close()
        except Exception:
            pass

        if self.manual_reader_thread is not None:
            self.manual_reader_thread.join(SERIAL_READ_THREAD_JOIN_MS / 1000.0)
            self.manual_reader_thread = None

        self.connection_state_var.set("未连接")
        self._append_log("手动串口已断开。")

    def _manual_serial_read_loop(self) -> None:
        while not self.manual_reader_stop.is_set():
            serial_obj = self.manual_serial
            if serial_obj is None:
                return
            try:
                raw = serial_obj.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").rstrip("\r\n")
                self.event_queue.put(("log_line", "[串口] {}".format(line)))
            except Exception as exc:
                self.event_queue.put(("log_line", "[串口] 读取异常：{}".format(exc)))
                return

    def _send_manual_command(self, command_text: str | None = None) -> None:
        serial_obj = self.manual_serial
        if serial_obj is None:
            messagebox.showwarning(APP_TITLE, "请先连接手动串口，再发送指令。")
            return

        line = (command_text if command_text is not None else self.quick_command_var.get()).strip()
        if not line:
            messagebox.showwarning(APP_TITLE, "请输入要发送的指令。")
            return

        try:
            serial_obj.write((line + "\n").encode("utf-8"))
            serial_obj.flush()
            self._append_log("[发送] {}".format(line))
            if command_text is None:
                self.quick_command_var.set("")
        except Exception as exc:
            messagebox.showerror(APP_TITLE, "发送失败：{}".format(exc))

    def _start_tuning(self) -> None:
        if self.tuning_running:
            messagebox.showwarning(APP_TITLE, "自动调参已经在运行，请等待当前会话结束。")
            return

        try:
            settings = self._build_serial_settings(require_port=True)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, "串口参数无效：{}".format(exc))
            return

        if self.manual_serial is not None:
            self._disconnect_manual_serial()

        session_name = time.strftime(tune.SESSION_NAME_FORMAT)
        preview_paths = tune.create_session_log_paths(session_name)
        self.current_session_dir = str(preview_paths.session_dir)
        self.log_dir_var.set(self.current_session_dir)
        self.session_state_var.set("自动调参运行中：{}".format(session_name))
        self._append_log("准备启动自动调参，会话目录：{}".format(preview_paths.session_dir))

        self.tuning_running = True
        self.start_tuning_button.configure(state="disabled")
        self.stop_tuning_button.configure(state="normal")
        self.tuning_thread = threading.Thread(
            target=self._run_tuning_worker,
            args=(settings, session_name, preview_paths.console_log),
            daemon=True,
        )
        self.tuning_thread.start()

    def _stop_tuning(self) -> None:
        if not self.tuning_running:
            self._append_log("当前没有正在运行的自动调参任务。")
            return
        if not messagebox.askyesno(APP_TITLE, STOP_CONFIRM_MESSAGE):
            return
        tune.request_cancel()
        self.session_state_var.set("正在请求终止自动调参")
        self.stop_tuning_button.configure(state="disabled")
        self._append_log("已发送终止请求，正在等待当前步骤安全退出...")

    def _run_tuning_worker(self, settings: tune.SerialSettings, session_name: str, console_log_path: Path) -> None:
        success = False
        error_message = ""
        cancelled = False
        try:
            with console_log_path.open("a", encoding="utf-8") as log_file:
                tee = QueueLogWriter(self.event_queue, log_file)
                with redirect_stdout(tee), redirect_stderr(tee):
                    print("=== AIPID 上位机自动调参启动 ===")
                    print("会话时间戳：{}".format(session_name))
                    print("会话日志目录：{}".format(console_log_path.parent))
                    print("串口配置：{}".format(tune.serial_settings_to_dict(settings)))
                    best_parameters, best_result, session_paths = tune.run_tuning_session(
                        serial_settings=settings,
                        session_name=session_name,
                    )
                    print("自动调参完成，最佳损失分数：{:.4f}".format(best_result.score))
                    print("最佳参数：{}".format(best_parameters))
                    print("会话摘要：{}".format(session_paths.summary_json))
            success = True
        except tune.CancellationRequestedError as exc:
            cancelled = True
            error_message = str(exc)
            self.event_queue.put(("log_line", "自动调参收到终止请求，正在收尾退出。"))
        except Exception as exc:
            error_message = str(exc)
            formatted = traceback.format_exc()
            self.event_queue.put(("log_line", formatted.rstrip()))
        finally:
            self.event_queue.put((
                "tuning_done",
                {
                    "success": success,
                    "cancelled": cancelled,
                    "error": error_message,
                    "session_dir": self.current_session_dir,
                },
            ))

    def _open_latest_session_dir(self) -> None:
        target = self.current_session_dir or str(tune.SESSION_LOG_ROOT)
        self._open_path(target)

    def _open_log_root(self) -> None:
        self._open_path(str(tune.LOG_DIR))

    def _open_path(self, path_text: str) -> None:
        path = Path(path_text)
        path.mkdir(parents=True, exist_ok=True)
        os.startfile(str(path))

    def _poll_event_queue(self) -> None:
        while True:
            try:
                event_type, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "log_line":
                self._append_log(payload)
            elif event_type == "tuning_done":
                self.tuning_running = False
                self.start_tuning_button.configure(state="normal")
                self.stop_tuning_button.configure(state="disabled")
                if payload["success"]:
                    self.session_state_var.set("自动调参完成")
                    self._append_log("自动调参已结束，会话目录：{}".format(payload["session_dir"]))
                elif payload.get("cancelled"):
                    self.session_state_var.set("自动调参已终止")
                    self._append_log("自动调参已按请求终止，会话目录：{}".format(payload["session_dir"]))
                else:
                    self.session_state_var.set("自动调参失败")
                    self._append_log("自动调参失败：{}".format(payload["error"]))

        self.root.after(UI_QUEUE_POLL_MS, self._poll_event_queue)

    def _on_close(self) -> None:
        if self.tuning_running:
            if not messagebox.askyesno(APP_TITLE, "自动调参仍在运行，关闭窗口会直接结束进程。确定继续吗？"):
                return
        self._disconnect_manual_serial()
        self.root.destroy()


def main() -> None:
    root = Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    UpperComputerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
