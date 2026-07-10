"""颜色目标追踪控制器。

本文件只处理视觉目标相对屏幕中心的位置误差，并输出底盘平移控制量。
组合控制模式下，本文件不会单独覆盖 yaw 控制，而是把 vx/vy 交给上层统一混合。
"""

import time
import MotorControl
from MCXVisionUsart import MCXVisionUsart


# ---------------------------------------------------------------------------
# 人工调参区
# ---------------------------------------------------------------------------

# 默认基础占空比。当前版本主要保留给状态输出和兼容旧调用使用，不直接作为电机输出。
COLOR_BASE_DUTY_DEFAULT = 6000

# 颜色追踪允许的最大 PWM 占空比。数值越大，目标偏离中心时平移越猛。
# 速度到占空比的基础换算在 MotorControl 中完成：
#   duty = speed * (max_duty / 100)
# 例如 max_duty=6500 时，speed=36 的理论占空比约为 36*65=2340。
# 之后 MotorControl 还会按三轮混合结果做等比例缩放，独立颜色追踪模式下
# `_bias_duty()` 会再把非零占空比抬到启动区间，避免电机因为占空比太低不动。
# 如果目标在中心附近大幅来回抖动，优先降低这个值或下面的 COLOR_MAX_TRACKING_SPEED_DEFAULT。
COLOR_MAX_TRACKING_DUTY_DEFAULT = 6500

# 颜色追踪非零输出的最小 PWM 占空比。独立调用 update(m1,m2,m3) 时会用它抬高电机启动量。
# 组合 yaw 跟随时主要使用 vx/vy，不直接使用这个占空比下限。
COLOR_MIN_TRACKING_DUTY_DEFAULT = 5500

# 丢失视觉帧后的短暂保持时间，单位 ms。数值过大可能导致目标丢失后还继续冲。
COLOR_TARGET_HOLD_MS_DEFAULT = 100

# X 方向比例系数。数值越大，目标左右偏离时横向修正越强；抖动大时降低。
COLOR_KP_X_DEFAULT = 1.2

# X 方向微分系数。用于抑制变化过快的误差；过大也可能让输出变尖锐。
COLOR_KD_X_DEFAULT = 0.08

# Y 方向比例系数。数值越大，目标上下偏离时前后修正越强；抖动大时降低。
COLOR_KP_Y_DEFAULT = 1.2

# Y 方向微分系数。用于抑制变化过快的误差；过大也可能让输出变尖锐。
COLOR_KD_Y_DEFAULT = 0.08

# 进入中心区阈值，单位像素。目标误差小于该值时认为已经居中并停止平移修正。
COLOR_CENTER_ENTER_ZONE_DEFAULT = 12

# 离开中心区阈值，单位像素。必须大于进入阈值，用迟滞避免目标在边界附近反复启停。
COLOR_CENTER_EXIT_ZONE_DEFAULT = 22

# 视觉目标中心点一阶滤波系数，范围 0.05~1.0。越小越稳但越慢，越大响应越快。
COLOR_INPUT_FILTER_ALPHA_DEFAULT = 0.25

# 平移速度指令一阶滤波系数，范围 0.05~1.0。越小电机输出越柔和，越大响应越快。
COLOR_COMMAND_FILTER_ALPHA_DEFAULT = 0.18

# 颜色追踪平移速度最大值，单位是 MotorControl 的 speed 标度 0~100。
# 这个值不是占空比，而是先作为底盘 vx/vy 速度参与三轮解算。
# 在默认 max_duty=6500 时，单个轮子的理论占空比约为 speed*65；
# 例如 speed=30 约等于 1950 duty，speed=40 约等于 2600 duty。
# 这是抑制大幅抖动最直接的限幅；若追踪太慢再逐步增加。
COLOR_MAX_TRACKING_SPEED_DEFAULT = 36.0

# 平移速度每次 update 最大变化量。它限制的是 speed 变化量，不是 duty 变化量。
# 在 max_duty=6500 时，每增加 1 speed 理论上约增加 65 duty；
# 当前 1.5 speed 约等于每次最多变化 98 duty，实际还会受三轮混合影响。
# 数值越小加减速越柔和，但太小会追不上快速目标。
COLOR_COMMAND_RAMP_STEP_DEFAULT = 1.5


class PID:
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
        output = max(self.output_min, min(self.output_max, output))
        return output


class ColorTraceController:
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
        kp_y=COLOR_KP_Y_DEFAULT,
        kd_y=COLOR_KD_Y_DEFAULT,
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
        self.last_frame = None
        self.last_frame_ms = None
        self.last_frame_idx = None
        self.correction_x = 0
        self.correction_y = 0
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

    def start(self):
        self.x_pid.reset()
        self.y_pid.reset()
        self.buf = bytearray()
        self.running = True
        self.last_frame = None
        self.last_frame_ms = None
        self.last_frame_idx = None
        self.correction_x = 0
        self.correction_y = 0
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

    def stop(self):
        self.running = False
        self.last_frame = None
        self.last_frame_ms = None
        self.last_frame_idx = None
        self.correction_x = 0
        self.correction_y = 0
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
        """清空平移控制量，避免进入中心或丢目标后旧速度继续残留。"""
        self.correction_x = 0
        self.correction_y = 0
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
        """对视觉中心点做一阶滤波，减少识别框跳动直接传到电机。"""
        if self.filtered_cx is None or self.filtered_cy is None:
            self.filtered_cx = cx
            self.filtered_cy = cy
        else:
            alpha = self.input_filter_alpha
            self.filtered_cx += alpha * (cx - self.filtered_cx)
            self.filtered_cy += alpha * (cy - self.filtered_cy)
        return self.filtered_cx, self.filtered_cy

    def _is_in_center(self, err_x, err_y):
        """带迟滞判断目标是否在中心区，避免中心边界附近反复启停。"""
        if self.centered:
            if abs(err_x) > self.center_exit_zone or abs(err_y) > self.center_exit_zone:
                self.centered = False
        elif abs(err_x) <= self.dead_zone and abs(err_y) <= self.dead_zone:
            self.centered = True
        return self.centered

    def _filter_command(self, vx, vy):
        """对平移速度做一阶滤波，让追踪动作更柔和。"""
        alpha = self.command_filter_alpha
        self.filtered_vx += alpha * (vx - self.filtered_vx)
        self.filtered_vy += alpha * (vy - self.filtered_vy)
        return self.filtered_vx, self.filtered_vy

    def _step_towards(self, current, target):
        """按固定步长逼近目标速度，限制每次循环的速度突变。"""
        step = self.command_ramp_step
        if step <= 0:
            return target
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _ramp_command(self, vx, vy):
        """分别限制 vx/vy 的变化率，降低中心附近大幅来回摆动。"""
        self.command_vx = self._step_towards(self.command_vx, vx)
        self.command_vy = self._step_towards(self.command_vy, vy)
        return self.command_vx, self.command_vy

    def _frame_to_state(self, frame):
        cx, cy = self._filter_center(frame["center_x"], frame["center_y"])
        err_x = self.CENTER_X - cx
        err_y = self.CENTER_Y - cy
        box = (frame["x1"], frame["y1"], frame["x2"], frame["y2"])
        in_center = self._is_in_center(err_x, err_y)
        return cx, cy, err_x, err_y, box, in_center

    def _target_is_recent(self, now_ms):
        if self.last_frame is None or self.last_frame_ms is None:
            return False
        return time.ticks_diff(now_ms, self.last_frame_ms) <= self.target_hold_ms

    def update_tracking_command(self):
        """只计算颜色追踪控制量，不直接驱动电机。

        组合控制需要把颜色追踪的平移速度和 yaw PID 的旋转速度混合后，
        再统一写入三个电机。这里单独提供计算接口，避免两个控制器先后写
        电机导致输出互相覆盖。
        """
        if not self.running:
            return None

        now_ms = time.ticks_ms()
        frames = self._recv_frames()

        has_target_now = False
        target_locked = False
        cx = self.CENTER_X
        cy = self.CENTER_Y
        err_x = 0
        err_y = 0
        box = (0, 0, 0, 0)
        in_center = False

        if frames:
            frame = frames[-1]
            self.last_frame_idx = frame["idx"]

            if frame.get("has_target", False):
                self.last_frame = frame
                self.last_frame_ms = now_ms
                has_target_now = True
                target_locked = True

                cx, cy, err_x, err_y, box, in_center = self._frame_to_state(frame)

                if not in_center:
                    self.correction_x = self.x_pid.compute(0, err_x)
                    self.correction_y = self.y_pid.compute(0, err_y)
                    self.moving = True
                else:
                    self._reset_motion_command()
            else:
                # 明确收到无目标帧时，立即清空控制量。
                self.last_frame = None
                self.last_frame_ms = None
                self.filtered_cx = None
                self.filtered_cy = None
                self.centered = False
                self._reset_motion_command()
        elif self._target_is_recent(now_ms):
            # 只允许短时间丢包复用上一帧有效目标。
            target_locked = True
            cx, cy, err_x, err_y, box, in_center = self._frame_to_state(self.last_frame)
            if in_center:
                self._reset_motion_command()
        else:
            self.last_frame = None
            self.last_frame_ms = None
            self.last_frame_idx = None
            self.filtered_cx = None
            self.filtered_cy = None
            self.centered = False
            self._reset_motion_command()

        vx = 0
        vy = 0

        if self.moving and target_locked and not in_center and has_target_now:
            filtered_vx, filtered_vy = self._filter_command(
                self.correction_x,
                -self.correction_y,
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
            "correction_x": self.correction_x,
            "correction_y": self.correction_y,
            "base_duty": self.base_duty,
            "max_tracking_duty": self.max_tracking_duty,
            "min_tracking_duty": self.min_tracking_duty,
            "target_hold_ms": self.target_hold_ms,
            "moving": self.moving,
            "in_center": target_locked and in_center,
            "box": box,
            "vx": vx,
            "vy": vy,
            "raw_duty": raw_duty,
            "duty": duty,
        }

    def update(self, motor_1, motor_2, motor_3):
        state = self.update_tracking_command()
        if state is None:
            return None

        if state["moving"] and state["target_locked"] and not state["in_center"]:
            self._apply_duty(motor_1, motor_2, motor_3, state["duty"])
        else:
            self._stop_motors(motor_1, motor_2, motor_3)

        return state
