"""颜色目标追踪控制器。

本模块只负责根据视觉目标状态计算平移控制量 vx、vy。
在 CarB 的组合控制模式下，yaw 由上层独立闭环负责，
这里不直接覆盖旋转控制，只输出平移控制量供上层混合。
"""

import time

import MotorControl
from MCXVisionUsart import MCXVisionUsart


# ---------------------------------------------------------------------------
# 人工调参区
# 所有需要现场人工调试的参数都集中放在文件顶部，便于统一修改。
# ---------------------------------------------------------------------------

# 基础占空比。主要用于兼容旧状态字段和调试输出，不直接参与平移闭环。
COLOR_BASE_DUTY_DEFAULT = 6000

# 颜色追踪换算为电机占空比时允许的最大占空比。
COLOR_MAX_TRACKING_DUTY_DEFAULT = 6500

# 颜色追踪单独直驱电机时的最小非零占空比，用于抬高电机起步量。
COLOR_MIN_TRACKING_DUTY_DEFAULT = 5500

# 丢帧后继续沿用上一帧有效目标的保持时间，单位 ms。
COLOR_TARGET_HOLD_MS_DEFAULT = 100

# 左右居中 PID 参数。
COLOR_KP_X_DEFAULT = 1.2
COLOR_KD_X_DEFAULT = 0.08

# 横向修正输出方向系数。
# 如果目标在画面右侧时小车反而向左修正，就把它设为 -1.0；
# 如果目标在右侧时小车能正确向右修正，就设为 1.0。
COLOR_X_OUTPUT_SIGN_DEFAULT = -1.0

# 上下方向 PID 参数。用于让目标中点在屏幕竖直方向保持居中。
COLOR_KP_Y_DEFAULT = 1.2
COLOR_KD_Y_DEFAULT = 0.08

# 前后修正输出方向系数。
# 如果目标偏上/偏下时，小车前后修正方向相反，
# 就把它设为 -1.0；方向正确则设为 1.0。
COLOR_Y_OUTPUT_SIGN_DEFAULT = -1.0

# 进入/退出居中区域的阈值，单位像素。
COLOR_CENTER_ENTER_ZONE_DEFAULT = 12
COLOR_CENTER_EXIT_ZONE_DEFAULT = 22

# 视觉输入和速度指令的一阶滤波系数。
COLOR_INPUT_FILTER_ALPHA_DEFAULT = 0.25
COLOR_COMMAND_FILTER_ALPHA_DEFAULT = 0.18

# 平移速度最大值，使用 MotorControl 的 speed 标度 0~100。
COLOR_MAX_TRACKING_SPEED_DEFAULT = 36.0

# 每次更新允许的最大速度变化量，用于限制突变。
COLOR_COMMAND_RAMP_STEP_DEFAULT = 1.5


class PID:
    """简单 PID 控制器。"""

    def __init__(self, kp=0, ki=0, kd=0, output_min=-100, output_max=100, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.integral = 0
        self.last_error = 0

    def reset(self):
        self.integral = 0
        self.last_error = 0

    def compute(self, target, current):
        error = target - current
        p_out = self.kp * error

        self.integral += error
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_out = self.ki * self.integral

        d_out = self.kd * (error - self.last_error)
        self.last_error = error

        output = p_out + i_out + d_out
        return max(self.output_min, min(self.output_max, output))


class ColorTraceController:
    """根据视觉目标状态生成平移控制命令。"""

    SCREEN_W = 160
    SCREEN_H = 120
    CENTER_X = SCREEN_W // 2
    CENTER_Y = SCREEN_H // 2

    def __init__(
        self,
        mcx_usart,
        base_duty=COLOR_BASE_DUTY_DEFAULT,
        max_tracking_duty=COLOR_MAX_TRACKING_DUTY_DEFAULT,
        min_tracking_duty=COLOR_MIN_TRACKING_DUTY_DEFAULT,
        target_hold_ms=COLOR_TARGET_HOLD_MS_DEFAULT,
        kp_x=COLOR_KP_X_DEFAULT,
        kd_x=COLOR_KD_X_DEFAULT,
        x_output_sign=COLOR_X_OUTPUT_SIGN_DEFAULT,
        kp_y=COLOR_KP_Y_DEFAULT,
        kd_y=COLOR_KD_Y_DEFAULT,
        y_output_sign=COLOR_Y_OUTPUT_SIGN_DEFAULT,
        dead_zone=COLOR_CENTER_ENTER_ZONE_DEFAULT,
        center_exit_zone=COLOR_CENTER_EXIT_ZONE_DEFAULT,
        input_filter_alpha=COLOR_INPUT_FILTER_ALPHA_DEFAULT,
        command_filter_alpha=COLOR_COMMAND_FILTER_ALPHA_DEFAULT,
        max_tracking_speed=COLOR_MAX_TRACKING_SPEED_DEFAULT,
        command_ramp_step=COLOR_COMMAND_RAMP_STEP_DEFAULT,
    ):
        self.mcx = mcx_usart
        self.SCREEN_W = getattr(mcx_usart, "VIEW_WIDTH", self.SCREEN_W)
        self.SCREEN_H = getattr(mcx_usart, "VIEW_HEIGHT", self.SCREEN_H)
        self.CENTER_X = self.SCREEN_W // 2
        self.CENTER_Y = self.SCREEN_H // 2

        self.base_duty = base_duty
        self.dead_zone = max(0, int(dead_zone))
        self.center_exit_zone = max(self.dead_zone, int(center_exit_zone))
        self.x_output_sign = -1.0 if float(x_output_sign) < 0 else 1.0
        self.y_output_sign = -1.0 if float(y_output_sign) < 0 else 1.0
        self.input_filter_alpha = min(1.0, max(0.05, float(input_filter_alpha)))
        self.command_filter_alpha = min(1.0, max(0.05, float(command_filter_alpha)))
        self.command_ramp_step = max(0.0, float(command_ramp_step))
        self.buf = bytearray()
        self.max_tracking_duty = min(MotorControl.MAX_DUTY, int(max_tracking_duty))
        self.min_tracking_duty = min(self.max_tracking_duty, max(0, int(min_tracking_duty)))
        self.target_hold_ms = max(0, int(target_hold_ms))
        self.max_tracking_speed = min(
            int(self.max_tracking_duty * MotorControl.MAX_SPEED / MotorControl.MAX_DUTY),
            max(1.0, float(max_tracking_speed)),
        )

        self.x_pid = PID(
            kp=kp_x,
            ki=0,
            kd=kd_x,
            output_min=-self.max_tracking_speed,
            output_max=self.max_tracking_speed,
        )
        self.y_pid = PID(
            kp=kp_y,
            ki=0,
            kd=kd_y,
            output_min=-self.max_tracking_speed,
            output_max=self.max_tracking_speed,
        )

        self.running = False
        self._clear_runtime_state()

    def _clear_runtime_state(self):
        """清空运行时状态。"""
        self.last_frame = None
        self.last_frame_ms = None
        self.last_frame_idx = None
        self.correction_x = 0.0
        self.correction_y = 0.0
        self.moving = False
        self.last_raw_duty = (0, 0, 0)
        self.last_duty = (0, 0, 0)
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.command_vx = 0.0
        self.command_vy = 0.0
        self.centered = False
        self.box_width = 0
        self.box_height = 0

    def start(self):
        self.x_pid.reset()
        self.y_pid.reset()
        self.buf = bytearray()
        self.running = True
        self._clear_runtime_state()

    def stop(self):
        self.running = False
        self._clear_runtime_state()

    def _apply_duty(self, motor_1, motor_2, motor_3, duty):
        d1, d2, d3 = duty
        motor_1.duty(d1)
        motor_2.duty(d2)
        motor_3.duty(d3)
        self.last_duty = duty

    def _stop_motors(self, motor_1, motor_2, motor_3):
        self._apply_duty(motor_1, motor_2, motor_3, (0, 0, 0))
        self.last_raw_duty = (0, 0, 0)

    def _bias_duty(self, duty_value):
        if duty_value == 0:
            return 0

        duty_sign = 1 if duty_value > 0 else -1
        duty_mag = min(self.max_tracking_duty, abs(int(duty_value)))

        if self.min_tracking_duty <= 0 or self.min_tracking_duty >= self.max_tracking_duty:
            return duty_sign * max(self.min_tracking_duty, duty_mag)

        biased_mag = self.min_tracking_duty + int(
            duty_mag * (self.max_tracking_duty - self.min_tracking_duty) / self.max_tracking_duty
        )
        return duty_sign * min(self.max_tracking_duty, biased_mag)

    def _recv_frames(self):
        """从视觉串口接收并解析尽可能多的完整帧。"""
        frames = []

        while self.mcx.available() > 0:
            raw = self.mcx.recv_bytes()
            if raw is not None:
                self.buf.extend(raw)

        frame_size = getattr(self.mcx, "FRAME_SIZE", 11)
        while len(self.buf) >= frame_size:
            frame = MCXVisionUsart.parse_frame(bytes(self.buf[:frame_size]))
            if frame is not None:
                frames.append(frame)
                self.buf = self.buf[frame_size:]
            else:
                self.buf = self.buf[1:]

        return frames

    def _reset_motion_command(self):
        """在达到目标或丢失目标后，清空平移控制状态。"""
        self.correction_x = 0.0
        self.correction_y = 0.0
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.command_vx = 0.0
        self.command_vy = 0.0
        self.moving = False
        self.last_raw_duty = (0, 0, 0)
        self.last_duty = (0, 0, 0)
        self.x_pid.reset()
        self.y_pid.reset()

    def _filter_center(self, cx, cy):
        """对目标中心点做滤波，减小相邻帧抖动。"""
        if self.filtered_cx is None or self.filtered_cy is None:
            self.filtered_cx = cx
            self.filtered_cy = cy
        else:
            alpha = self.input_filter_alpha
            self.filtered_cx += alpha * (cx - self.filtered_cx)
            self.filtered_cy += alpha * (cy - self.filtered_cy)
        return self.filtered_cx, self.filtered_cy

    def _is_xy_in_center(self, err_x, err_y):
        """按 x/y 同时判断目标中点是否已经进入居中区域。"""
        if self.centered:
            if abs(err_x) > self.center_exit_zone or abs(err_y) > self.center_exit_zone:
                self.centered = False
        elif abs(err_x) <= self.dead_zone and abs(err_y) <= self.dead_zone:
            self.centered = True
        return self.centered

    def _filter_command(self, vx, vy):
        """对平移指令做滤波。"""
        alpha = self.command_filter_alpha
        self.filtered_vx += alpha * (vx - self.filtered_vx)
        self.filtered_vy += alpha * (vy - self.filtered_vy)
        return self.filtered_vx, self.filtered_vy

    def _step_towards(self, current, target):
        """限制每次更新的速度跳变量。"""
        step = self.command_ramp_step
        if step <= 0:
            return target
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _ramp_command(self, vx, vy):
        """分别对 vx、vy 做斜坡限制。"""
        self.command_vx = self._step_towards(self.command_vx, vx)
        self.command_vy = self._step_towards(self.command_vy, vy)
        return self.command_vx, self.command_vy

    def _frame_to_state(self, frame):
        """把视觉帧转换为控制层状态。"""
        cx, cy = self._filter_center(frame["center_x"], frame["center_y"])
        err_x = self.CENTER_X - cx
        err_y = self.CENTER_Y - cy
        box = (frame["x1"], frame["y1"], frame["x2"], frame["y2"])
        box_width = frame.get("width", 0)
        box_height = frame.get("height", 0)
        in_center = self._is_xy_in_center(err_x, err_y)

        return {
            "cx": cx,
            "cy": cy,
            "err_x": err_x,
            "err_y": err_y,
            "box": box,
            "box_width": box_width,
            "box_height": box_height,
            "in_center": in_center,
        }

    def _target_is_recent(self, now_ms):
        if self.last_frame is None or self.last_frame_ms is None:
            return False
        return time.ticks_diff(now_ms, self.last_frame_ms) <= self.target_hold_ms

    def _load_frame_state(self, frame_state):
        self.box_width = frame_state["box_width"]
        self.box_height = frame_state["box_height"]

    def update_tracking_command(self):
        """只计算平移控制量，不直接写入电机。"""
        if not self.running:
            return None

        now_ms = time.ticks_ms()
        frames = self._recv_frames()

        has_target_now = False
        target_locked = False
        cx = self.CENTER_X
        cy = self.CENTER_Y
        err_x = 0.0
        err_y = 0.0
        box = (0, 0, 0, 0)
        box_width = 0
        box_height = 0
        in_center = False

        if frames:
            frame = frames[-1]
            self.last_frame_idx = frame["idx"]

            if frame.get("has_target", False):
                self.last_frame = frame
                self.last_frame_ms = now_ms
                has_target_now = True
                target_locked = True

                frame_state = self._frame_to_state(frame)
                cx = frame_state["cx"]
                cy = frame_state["cy"]
                err_x = frame_state["err_x"]
                err_y = frame_state["err_y"]
                box = frame_state["box"]
                box_width = frame_state["box_width"]
                box_height = frame_state["box_height"]
                in_center = frame_state["in_center"]
                self._load_frame_state(frame_state)

                if not in_center:
                    self.correction_x = self.x_pid.compute(0, err_x) * self.x_output_sign
                    self.correction_y = self.y_pid.compute(0, err_y)
                    self.moving = True
                else:
                    self._reset_motion_command()
            else:
                self.last_frame = None
                self.last_frame_ms = None
                self.filtered_cx = None
                self.filtered_cy = None
                self.centered = False
                self._reset_motion_command()
        elif self._target_is_recent(now_ms):
            target_locked = True
            frame_state = self._frame_to_state(self.last_frame)
            cx = frame_state["cx"]
            cy = frame_state["cy"]
            err_x = frame_state["err_x"]
            err_y = frame_state["err_y"]
            box = frame_state["box"]
            box_width = frame_state["box_width"]
            box_height = frame_state["box_height"]
            in_center = frame_state["in_center"]
            self._load_frame_state(frame_state)
            if in_center:
                self._reset_motion_command()
        else:
            self.last_frame = None
            self.last_frame_ms = None
            self.last_frame_idx = None
            self.filtered_cx = None
            self.filtered_cy = None
            self.centered = False
            self.box_width = 0
            self.box_height = 0
            self._reset_motion_command()

        vx = 0.0
        vy = 0.0

        if self.moving and target_locked and not in_center and has_target_now:
            vy_target = -self.correction_y * self.y_output_sign

            filtered_vx, filtered_vy = self._filter_command(
                self.correction_x,
                vy_target,
            )
            vx, vy = self._ramp_command(filtered_vx, filtered_vy)
            raw_duty = MotorControl.vector_to_duty(
                vx,
                vy,
                max_duty=self.max_tracking_duty,
                min_duty_start=0,
            )
            duty = tuple(self._bias_duty(value) for value in raw_duty)
            self.last_raw_duty = raw_duty
            self.last_duty = duty
        elif self.moving and target_locked and not in_center:
            vx = self.command_vx
            vy = self.command_vy
            raw_duty = self.last_raw_duty
            duty = self.last_duty
        else:
            vx, vy = self._ramp_command(0.0, 0.0)
            raw_duty = (0, 0, 0)
            duty = (0, 0, 0)
            self.last_raw_duty = raw_duty
            self.last_duty = duty

        return {
            "has_target": has_target_now,
            "target_locked": target_locked,
            "frame_idx": self.last_frame_idx,
            "cx": cx,
            "cy": cy,
            "err_x": err_x,
            "err_y": err_y,
            "box": box,
            "box_width": box_width,
            "box_height": box_height,
            "correction_x": self.correction_x,
            "correction_y": self.correction_y,
            "x_output_sign": self.x_output_sign,
            "y_output_sign": self.y_output_sign,
            "base_duty": self.base_duty,
            "max_tracking_duty": self.max_tracking_duty,
            "min_tracking_duty": self.min_tracking_duty,
            "target_hold_ms": self.target_hold_ms,
            "moving": self.moving,
            "in_center": target_locked and in_center,
            "vx": vx,
            "vy": vy,
            "raw_duty": raw_duty,
            "duty": duty,
        }

    def update(self, motor_1, motor_2, motor_3):
        """兼容独立颜色追踪模式，直接把输出写到电机。"""
        state = self.update_tracking_command()
        if state is None:
            return None

        if state["moving"] and state["target_locked"] and not state["in_center"]:
            self._apply_duty(motor_1, motor_2, motor_3, state["duty"])
        else:
            self._stop_motors(motor_1, motor_2, motor_3)

        return state
