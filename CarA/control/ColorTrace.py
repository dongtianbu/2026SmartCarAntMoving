import time
from control import MotorControl
from connection.MCXVisionUsart import MCXVisionUsart


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
        base_duty=6000,
        max_tracking_duty=7500,
        min_tracking_duty=5700,
        target_hold_ms=120,
        kp_x=3.0,
        kd_x=0.2,
        kp_y=3.0,
        kd_y=0.2,
        dead_zone=10,
        center_exit_zone=None,
        input_filter_alpha=0.35,
        command_filter_alpha=0.22,
        near_center_min_duty=2200,
        near_center_max_duty=4200,
        slow_zone=20,
        duty_ramp_step=220,
    ):
        self.mcx = mcx_usart
        self.SCREEN_W = getattr(mcx_usart, "VIEW_WIDTH", self.SCREEN_W)
        self.SCREEN_H = getattr(mcx_usart, "VIEW_HEIGHT", self.SCREEN_H)
        self.CENTER_X = self.SCREEN_W // 2
        self.CENTER_Y = self.SCREEN_H // 2

        self.base_duty = base_duty
        self.dead_zone = dead_zone
        if center_exit_zone is None:
            center_exit_zone = self.dead_zone + 2
        self.center_exit_zone = max(self.dead_zone, int(center_exit_zone))
        self.buf = bytearray()
        self.max_tracking_duty = min(MotorControl.MAX_DUTY, int(max_tracking_duty))
        self.min_tracking_duty = min(self.max_tracking_duty, max(0, int(min_tracking_duty)))
        self.near_center_min_duty = min(self.min_tracking_duty, max(0, int(near_center_min_duty)))
        self.near_center_max_duty = min(self.max_tracking_duty, max(self.near_center_min_duty, int(near_center_max_duty)))
        self.target_hold_ms = max(0, int(target_hold_ms))
        self.input_filter_alpha = min(1.0, max(0.05, float(input_filter_alpha)))
        self.command_filter_alpha = min(1.0, max(0.05, float(command_filter_alpha)))
        self.slow_zone = max(self.dead_zone + 2, int(slow_zone))
        self.duty_ramp_step = max(1, int(duty_ramp_step))
        self.max_tracking_speed = max(
            20, int(self.max_tracking_duty * MotorControl.MAX_SPEED / MotorControl.MAX_DUTY)
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
        self.target_duty = (0, 0, 0)
        self.filtered_cx = None
        self.filtered_cy = None
        self.centered = False
        self.filtered_cmd_vx = 0.0
        self.filtered_cmd_vy = 0.0

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
        self.target_duty = (0, 0, 0)
        self.filtered_cx = None
        self.filtered_cy = None
        self.centered = False
        self.filtered_cmd_vx = 0.0
        self.filtered_cmd_vy = 0.0

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
        self.target_duty = (0, 0, 0)
        self.filtered_cx = None
        self.filtered_cy = None
        self.centered = False
        self.filtered_cmd_vx = 0.0
        self.filtered_cmd_vy = 0.0

    def _apply_duty(self, motor_1, motor_2, motor_3, duty):
        d1, d2, d3 = duty
        motor_1.duty(d1)
        motor_2.duty(d2)
        motor_3.duty(d3)
        self.last_duty = duty

    def _stop_motors(self, motor_1, motor_2, motor_3):
        self._apply_duty(motor_1, motor_2, motor_3, (0, 0, 0))
        self.last_raw_duty = (0, 0, 0)
        self.target_duty = (0, 0, 0)

    def _scale_duty_vector(self, raw_duty, active_min_duty, active_max_duty):
        if active_max_duty <= 0:
            return (0, 0, 0)

        max_abs_duty = max(abs(int(value)) for value in raw_duty)
        if max_abs_duty <= 0:
            return (0, 0, 0)

        scale = 1.0
        if active_min_duty > 0 and max_abs_duty < active_min_duty:
            scale = active_min_duty / float(max_abs_duty)

        if max_abs_duty * scale > active_max_duty:
            scale = active_max_duty / float(max_abs_duty)

        return tuple(
            max(-active_max_duty, min(active_max_duty, int(round(value * scale))))
            for value in raw_duty
        )

    def _filter_center(self, cx, cy):
        if self.filtered_cx is None or self.filtered_cy is None:
            self.filtered_cx = cx
            self.filtered_cy = cy
        else:
            alpha = self.input_filter_alpha
            self.filtered_cx += alpha * (cx - self.filtered_cx)
            self.filtered_cy += alpha * (cy - self.filtered_cy)
        return self.filtered_cx, self.filtered_cy

    def _filter_command(self, vx, vy):
        alpha = self.command_filter_alpha
        self.filtered_cmd_vx += alpha * (vx - self.filtered_cmd_vx)
        self.filtered_cmd_vy += alpha * (vy - self.filtered_cmd_vy)
        return self.filtered_cmd_vx, self.filtered_cmd_vy

    def _motion_limits(self, err_x, err_y):
        error_mag = max(abs(err_x), abs(err_y))
        if error_mag <= self.dead_zone:
            return 0, self.near_center_max_duty

        span = max(1.0, float(self.slow_zone - self.dead_zone))
        ratio = min(1.0, max(0.0, (error_mag - self.dead_zone) / span))

        active_min_duty = self.near_center_min_duty + int(
            (self.min_tracking_duty - self.near_center_min_duty) * ratio
        )
        active_max_duty = self.near_center_max_duty + int(
            (self.max_tracking_duty - self.near_center_max_duty) * ratio
        )
        active_max_duty = max(active_min_duty, active_max_duty)
        return active_min_duty, active_max_duty

    def _step_towards(self, current, target, step):
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _ramp_duty_towards(self, motor_1, motor_2, motor_3, target_duty):
        duty = tuple(
            self._step_towards(current, target, self.duty_ramp_step)
            for current, target in zip(self.last_duty, target_duty)
        )
        self._apply_duty(motor_1, motor_2, motor_3, duty)
        return duty

    def _reset_tracking_state(self, clear_last_frame=True):
        if clear_last_frame:
            self.last_frame = None
            self.last_frame_ms = None
            self.last_frame_idx = None
        self.correction_x = 0
        self.correction_y = 0
        self.moving = False
        self.last_raw_duty = (0, 0, 0)
        self.target_duty = (0, 0, 0)
        self.filtered_cx = None
        self.filtered_cy = None
        self.centered = False
        self.filtered_cmd_vx = 0.0
        self.filtered_cmd_vy = 0.0
        self.x_pid.reset()
        self.y_pid.reset()

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

    def _is_in_center(self, err_x, err_y):
        if self.centered:
            if abs(err_x) > self.center_exit_zone or abs(err_y) > self.center_exit_zone:
                self.centered = False
        elif abs(err_x) <= self.dead_zone and abs(err_y) <= self.dead_zone:
            self.centered = True
        return self.centered

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

    def update(self, motor_1, motor_2, motor_3):
        if not self.running:
            return None

        now_ms = time.ticks_ms()
        frames = self._recv_frames()

        has_target_now = False
        target_locked = False
        explicit_no_target = False
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
                    self.correction_x = 0
                    self.correction_y = 0
                    self.moving = False
                    self.x_pid.reset()
                    self.y_pid.reset()
            else:
                explicit_no_target = True
                if self._target_is_recent(now_ms):
                    target_locked = True
                    cx, cy, err_x, err_y, box, in_center = self._frame_to_state(self.last_frame)
                else:
                    self._reset_tracking_state(clear_last_frame=True)
        elif self._target_is_recent(now_ms):
            # Only short packet loss is allowed to reuse the last valid target.
            target_locked = True
            cx, cy, err_x, err_y, box, in_center = self._frame_to_state(self.last_frame)
        else:
            self._reset_tracking_state(clear_last_frame=True)

        if self.moving and target_locked and not in_center and has_target_now:
            active_min_duty, active_max_duty = self._motion_limits(err_x, err_y)
            command_vx, command_vy = self._filter_command(
                self.correction_x,
                -self.correction_y,
            )
            raw_duty = MotorControl.vector_to_duty(
                command_vx,
                command_vy,
                max_duty=active_max_duty,
                min_duty_start=0,
            )
            self.target_duty = self._scale_duty_vector(
                raw_duty, active_min_duty, active_max_duty
            )
            self.last_raw_duty = raw_duty
            duty = self._ramp_duty_towards(motor_1, motor_2, motor_3, self.target_duty)
        elif self.moving and target_locked and not in_center:
            raw_duty = self.last_raw_duty
            duty = self._ramp_duty_towards(motor_1, motor_2, motor_3, self.target_duty)
        elif explicit_no_target:
            raw_duty = (0, 0, 0)
            if target_locked and self.target_duty != (0, 0, 0):
                duty = self._ramp_duty_towards(motor_1, motor_2, motor_3, self.target_duty)
            else:
                self._stop_motors(motor_1, motor_2, motor_3)
                duty = self.last_duty
        else:
            raw_duty = (0, 0, 0)
            self.target_duty = (0, 0, 0)
            duty = self._ramp_duty_towards(motor_1, motor_2, motor_3, self.target_duty)
            if duty == (0, 0, 0):
                self.last_raw_duty = (0, 0, 0)
                self.filtered_cmd_vx = 0.0
                self.filtered_cmd_vy = 0.0

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
            "raw_duty": raw_duty,
            "duty": self.last_duty,
            "target_duty": self.target_duty,
            "filtered_cx": cx,
            "filtered_cy": cy,
        }
