"""红外双点连线角度闭环辅助模块。

本模块单独拆分出来，目的是减小 FollowLeaderYawColorTraceLite.py
在上电导入阶段的解析内存占用。只有当线角度功能启用时，主控程序
才会延迟导入这里的实现。
"""

import math
import time


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _normalize_line_angle_deg(angle_deg):
    """把线段方向角归一化到 [-90, 90]，避免两点顺序变化导致角度跳变。"""
    while angle_deg > 90.0:
        angle_deg -= 180.0
    while angle_deg <= -90.0:
        angle_deg += 180.0
    return angle_deg


def _line_angle_error(target_deg, current_deg):
    """在线段方向按 180 度等价时计算最短角度误差。"""
    return _normalize_line_angle_deg(target_deg - current_deg)


def _compute_line_angle_deg(x1, y1, x2, y2):
    """根据视觉模块返回的两点坐标计算连线角度与线长。

    注意：这里默认协议中的 x1/y1/x2/y2 就是两个红外光点的坐标。
    如果后续视觉端改回发送包围框左上/右下角，需要同步调整这里的解释方式。
    """
    dx = float(x2) - float(x1)
    dy = float(y2) - float(y1)
    length = math.sqrt(dx * dx + dy * dy)
    if length <= 1e-6:
        return None, 0.0
    return _normalize_line_angle_deg(math.degrees(math.atan2(dy, dx))), length


class _LineAnglePID:
    """红外双点连线角度 PID，只生成附加旋转补偿量。"""

    def __init__(self, config, default_dt_s):
        self.default_dt_s = default_dt_s
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.output_limit = 0.0
        self.integral_limit = 0.0
        self.integral = 0.0
        self.last_error = None
        self.apply_runtime_config(config)

    def apply_runtime_config(self, config):
        self.kp = float(config["line_angle_kp"])
        self.ki = float(config["line_angle_ki"])
        self.kd = float(config["line_angle_kd"])
        self.output_limit = abs(float(config["line_angle_max_rotate_speed"]))
        self.integral_limit = abs(float(config["line_angle_integral_limit"]))

    def reset(self):
        self.integral = 0.0
        self.last_error = None

    def compute(self, error, dt_s):
        if dt_s <= 0:
            dt_s = self.default_dt_s
        self.integral = _clamp(self.integral + error * dt_s, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.last_error is None else (error - self.last_error) / dt_s
        self.last_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return _clamp(output, -self.output_limit, self.output_limit)


class VisionLineAngleController:
    """管理红外双点连线角度的测量、保持、滤波和 PID 输出。"""

    def __init__(self, config, default_dt_s):
        self._default_dt_s = default_dt_s
        self._pid = _LineAnglePID(config, default_dt_s)
        self.line_angle_deg = None
        self.line_angle_error = None
        self.line_angle_rotate_speed = 0.0
        self.line_angle_length_px = 0.0
        self.line_angle_last_valid_ms = None
        self.line_angle_filtered_deg = None
        self.line_angle_target_deg_runtime = None
        self.apply_runtime_config(config)

    def apply_runtime_config(self, config):
        self._pid.apply_runtime_config(config)
        mode = int(config.get("line_angle_target_mode", 0))
        if mode == 0:
            self.line_angle_target_deg_runtime = None

    def reset(self, clear_measurement=False, clear_target=False):
        self._pid.reset()
        self.line_angle_error = None
        self.line_angle_rotate_speed = 0.0
        if clear_measurement:
            self.clear_measurement()
        if clear_target:
            self.line_angle_target_deg_runtime = None

    def clear_measurement(self):
        self.line_angle_deg = None
        self.line_angle_error = None
        self.line_angle_rotate_speed = 0.0
        self.line_angle_length_px = 0.0
        self.line_angle_last_valid_ms = None
        self.line_angle_filtered_deg = None

    def _update_measurement(self, config, color_state, now_ms):
        if color_state is not None and color_state.get("target_locked", False):
            box = color_state.get("box")
            if box is not None and len(box) == 4:
                angle_deg, length_px = _compute_line_angle_deg(box[0], box[1], box[2], box[3])
                min_length_px = max(0.0, float(config["line_angle_min_length_px"]))
                if angle_deg is not None and length_px >= min_length_px:
                    if self.line_angle_filtered_deg is None:
                        self.line_angle_filtered_deg = angle_deg
                    else:
                        alpha = min(1.0, max(0.05, float(config["line_angle_filter_alpha"])))
                        delta = _line_angle_error(angle_deg, self.line_angle_filtered_deg)
                        self.line_angle_filtered_deg = _normalize_line_angle_deg(
                            self.line_angle_filtered_deg + alpha * delta
                        )
                    self.line_angle_deg = self.line_angle_filtered_deg
                    self.line_angle_length_px = length_px
                    self.line_angle_last_valid_ms = now_ms
                    return True

        hold_ms = max(0, int(config["line_angle_hold_ms"]))
        if self.line_angle_deg is not None and self.line_angle_last_valid_ms is not None:
            if time.ticks_diff(now_ms, self.line_angle_last_valid_ms) <= hold_ms:
                return True

        self.clear_measurement()
        return False

    def _get_target_angle_deg(self, config):
        """根据目标模式选择线角闭环参考值。

        模式 0：始终使用固定参数 LINE_ANGLE_TARGET_DEG。
        模式 1：首次稳定看到双点时，把当时线角锁定为运行时目标角。
        """
        mode = int(config.get("line_angle_target_mode", 0))
        if mode == 1:
            if self.line_angle_target_deg_runtime is None and self.line_angle_deg is not None:
                self.line_angle_target_deg_runtime = self.line_angle_deg
            if self.line_angle_target_deg_runtime is not None:
                return self.line_angle_target_deg_runtime
        return float(config["line_angle_target_deg"])

    def compute_rotate_speed(self, config, color_state, dt_s, now_ms):
        has_measurement = self._update_measurement(config, color_state, now_ms)
        if not has_measurement or self.line_angle_deg is None:
            self.reset(clear_measurement=not has_measurement)
            return 0.0

        target_angle_deg = self._get_target_angle_deg(config)
        self.line_angle_error = _line_angle_error(
            target_angle_deg,
            self.line_angle_deg,
        )
        if abs(self.line_angle_error) <= abs(float(config["line_angle_deadband_deg"])):
            self.reset(clear_measurement=False)
            return 0.0

        rotate_speed = self._pid.compute(self.line_angle_error, dt_s)
        rotate_speed *= float(config["line_angle_output_sign"])
        rotate_speed *= min(1.0, max(0.0, float(config["line_angle_blend_ratio"])))
        limit = abs(float(config["line_angle_max_rotate_speed"]))
        rotate_speed = _clamp(rotate_speed, -limit, limit)

        min_speed = abs(float(config["line_angle_min_command_speed"]))
        if 0.0 < abs(rotate_speed) < min_speed:
            rotate_speed = min_speed if rotate_speed > 0 else -min_speed

        self.line_angle_rotate_speed = rotate_speed
        return rotate_speed

    def export_state(self):
        return {
            "line_angle_deg": self.line_angle_deg,
            "line_angle_target_deg_runtime": self.line_angle_target_deg_runtime,
            "line_angle_error": self.line_angle_error,
            "line_angle_rotate_speed": self.line_angle_rotate_speed,
            "line_angle_length_px": self.line_angle_length_px,
        }
