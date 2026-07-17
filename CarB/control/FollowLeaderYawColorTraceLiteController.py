"""CarB 轻量组合控制器主体。"""

import gc
import math
import time

from FollowLeaderYawColorTraceLiteConfig import (
    YAW_CONTROL_MODE,
    _find_tunable,
    _is_legacy_yaw_follow_name,
    _normalize_yaw_control_mode,
    _parse_bool,
    _yaw_mode_uses_closed_loop,
    _yaw_mode_uses_leader,
)
from FollowLeaderYawColorTraceLiteSupport import (
    WirelessTuningSession,
    YawPID,
    _apply_min_vector_speed,
    _clamp,
    _continuous_yaw_error,
    _load_color_class,
    _load_imu_class,
    _load_leader_class,
    _load_line_angle_helper_class,
    _load_motor,
    _load_vision_class,
    _map_translate_vector_for_camera,
)


class FollowLeaderYawColorTraceLite:
    """视觉平移加 yaw 旋转的组合控制器。"""

    def __init__(self, config):
        self.config = config
        self.motor = _load_motor()
        gc.collect()
        self.pid = YawPID(config)
        self.line_angle_helper = None
        self.receiver = None
        self.tuner = None
        self.imu = None
        self.vision = None
        self.color = None
        self.leader_yaw = None
        self.last_loop_ms = time.ticks_ms()
        self.micro_cycle = 0
        self.ai_paused = bool(config["ai_control_paused_on_start"])
        self.ai_deviation_offset = 0.0
        self.ai_deviation_until = None
        self.ai_manual_until = None
        self.ai_vx = 0.0
        self.ai_vy = 0.0
        self.ai_rotate = 0.0
        self.ai_pause_after_manual = False
        self.running = False
        self.apply_runtime_config()

    def _ensure_line_angle_helper(self):
        """仅在启用线角闭环时创建辅助器，减少默认路径下的内存占用。"""
        if not self.config["line_angle_control_enabled"]:
            self.line_angle_helper = None
            return
        if self.line_angle_helper is None:
            helper_class = _load_line_angle_helper_class()
            self.line_angle_helper = helper_class(self.config, self.config["imu_tick_ms"] / 1000.0)
            gc.collect()
        else:
            self.line_angle_helper.apply_runtime_config(self.config)

    def _ensure_receiver(self):
        if self.receiver is None:
            self.receiver = _load_leader_class()(self.config)
            gc.collect()

    def _ensure_tuner(self):
        if self.tuner is None:
            self.tuner = WirelessTuningSession(self.config, self)
            gc.collect()

    def _ensure_yaw_runtime(self):
        """根据当前 yaw 模式，仅保留真正需要的无线对象。"""
        if _yaw_mode_uses_leader(self.config["yaw_control_mode"]):
            self._ensure_receiver()
            self.tuner = None
        else:
            self._ensure_tuner()
            self.receiver = None
        gc.collect()

    def _ensure_imu(self):
        if self.imu is None:
            imu_class = _load_imu_class()
            self.imu = imu_class(
                capture_div=self.config["imu_capture_div"],
                tick_ms=self.config["imu_tick_ms"],
                acc_range_g=self.config["imu_acc_range_g"],
                gyro_range_dps=self.config["imu_gyro_range_dps"],
                acc_alpha=self.config["imu_acc_alpha"],
                comp_alpha=self.config["imu_comp_alpha"],
                gyro_cali_n=self.config["imu_gyro_cali_n"],
                yaw_sign=self.config["imu_yaw_sign"],
            )
            gc.collect()

    def _ensure_color_pipeline(self):
        if self.vision is None:
            vision_class = _load_vision_class()
            self.vision = vision_class(baudrate=self.config["vision_baudrate"])
            gc.collect()
        if self.color is None:
            color_class = _load_color_class()
            self.color = color_class(
                self.vision,
                motor_control=self.motor,
                base_duty=self.config["color_base_duty"],
                max_tracking_duty=self.config["color_max_tracking_duty"],
                min_tracking_duty=self.config["color_min_tracking_duty"],
                target_hold_ms=self.config["color_target_hold_ms"],
                kp_x=self.config["color_kp_x"],
                kd_x=self.config["color_kd_x"],
                x_output_sign=self.config["color_x_output_sign"],
                kp_y=self.config["color_kp_y"],
                kd_y=self.config["color_kd_y"],
                y_output_sign=self.config["color_y_output_sign"],
                dead_zone=self.config["color_dead_zone"],
                center_exit_zone=self.config["color_center_exit_zone"],
                input_filter_alpha=self.config["color_input_filter_alpha"],
                command_filter_alpha=self.config["color_command_filter_alpha"],
                max_tracking_speed=self.config["color_max_tracking_speed"],
                command_ramp_step=self.config["color_command_ramp_step"],
            )
            gc.collect()

    def apply_runtime_config(self):
        self.config["yaw_control_mode"] = _normalize_yaw_control_mode(self.config.get("yaw_control_mode", YAW_CONTROL_MODE))
        self.config["yaw_follow_leader_enabled"] = _yaw_mode_uses_leader(self.config["yaw_control_mode"])
        self.pid.kp = float(self.config["pid_kp"])
        self.pid.ki = float(self.config["pid_ki"])
        self.pid.kd = float(self.config["pid_kd"])
        self.pid.output_limit = abs(float(self.config["max_rotate_speed"]))
        self.pid.integral_limit = abs(float(self.config["pid_integral_limit"]))
        if self.receiver is not None or self.tuner is not None:
            self._ensure_yaw_runtime()
        if self.config["line_angle_control_enabled"]:
            if self.line_angle_helper is not None or self.color is not None:
                self._ensure_line_angle_helper()
        else:
            self.line_angle_helper = None
        if self.color is None:
            return
        self.color.base_duty = int(self.config["color_base_duty"])
        self.color.dead_zone = max(0, int(self.config["color_dead_zone"]))
        self.color.center_exit_zone = max(self.color.dead_zone, int(self.config["color_center_exit_zone"]))
        self.color.input_filter_alpha = min(1.0, max(0.05, float(self.config["color_input_filter_alpha"])))
        self.color.command_filter_alpha = min(1.0, max(0.05, float(self.config["color_command_filter_alpha"])))
        self.color.command_ramp_step = max(0.0, float(self.config["color_command_ramp_step"]))
        self.color.max_tracking_duty = min(self.motor.MAX_DUTY, int(self.config["color_max_tracking_duty"]))
        self.color.min_tracking_duty = min(self.color.max_tracking_duty, max(0, int(self.config["color_min_tracking_duty"])))
        self.color.target_hold_ms = max(0, int(self.config["color_target_hold_ms"]))
        self.color.max_tracking_speed = min(
            int(self.color.max_tracking_duty * self.motor.MAX_SPEED / self.motor.MAX_DUTY),
            max(1.0, float(self.config["color_max_tracking_speed"])),
        )
        self.color.x_pid.kp = float(self.config["color_kp_x"])
        self.color.x_pid.kd = float(self.config["color_kd_x"])
        self.color.y_pid.kp = float(self.config["color_kp_y"])
        self.color.y_pid.kd = float(self.config["color_kd_y"])
        self.color.x_output_sign = -1.0 if float(self.config["color_x_output_sign"]) < 0 else 1.0
        self.color.y_output_sign = -1.0 if float(self.config["color_y_output_sign"]) < 0 else 1.0
        self.color.x_pid.output_min = -self.color.max_tracking_speed
        self.color.x_pid.output_max = self.color.max_tracking_speed
        self.color.y_pid.output_min = -self.color.max_tracking_speed
        self.color.y_pid.output_max = self.color.max_tracking_speed

    def apply_tunable(self, name, value_text):
        name = name.strip().upper()
        if _is_legacy_yaw_follow_name(name):
            try:
                follow_enabled = _parse_bool(value_text)
            except Exception:
                return False, "ERR invalid {}".format(name)
            self.config["yaw_control_mode"] = 2 if follow_enabled else 1
            self.apply_runtime_config()
            self.pid.reset()
            if self.line_angle_helper is not None:
                self.line_angle_helper.reset(clear_measurement=False, clear_target=True)
            if self.color is not None:
                self.color.x_pid.reset()
                self.color.y_pid.reset()
            return True, "OK {}={}".format(name, self.read_tunable(name))
        spec = _find_tunable(name)
        if spec is None:
            return False, "ERR unknown {}".format(name)
        _, key, value_type = spec
        try:
            value = _parse_bool(value_text) if value_type is bool else value_type(value_text)
        except Exception:
            return False, "ERR invalid {}".format(name)
        self.config[key] = value
        self.apply_runtime_config()
        self.pid.reset()
        if self.line_angle_helper is not None:
            clear_target = name.startswith("LINE_ANGLE_")
            self.line_angle_helper.reset(clear_measurement=False, clear_target=clear_target)
        if self.color is not None:
            self.color.x_pid.reset()
            self.color.y_pid.reset()
        return True, "OK {}={}".format(name, self.read_tunable(name))

    def read_tunable(self, name):
        name = name.strip().upper()
        if _is_legacy_yaw_follow_name(name):
            return self.config["yaw_follow_leader_enabled"]
        spec = _find_tunable(name)
        if spec is None:
            return ""
        return self.config[spec[1]]

    def handle_ai_command(self, line):
        if not self.config["ai_tuning_control_enabled"]:
            return True, "ERR AI_CONTROL_DISABLED"
        parts = line.strip().split()
        if not parts:
            return False, ""
        cmd = parts[0].upper()
        if cmd == "AI_STOP":
            self.ai_paused = True
            self.ai_deviation_until = None
            self.ai_manual_until = None
            self.ai_vx = self.ai_vy = self.ai_rotate = 0.0
            self.ai_pause_after_manual = False
            self.motor.stop(0)
            self.pid.reset()
            if self.line_angle_helper is not None:
                self.line_angle_helper.reset(clear_measurement=False)
            return True, "OK AI_STOP"
        if cmd == "AI_START":
            self.ai_paused = False
            self.ai_deviation_until = None
            self.ai_manual_until = None
            self.ai_vx = self.ai_vy = self.ai_rotate = 0.0
            self.ai_pause_after_manual = False
            self.pid.reset()
            if self.line_angle_helper is not None:
                self.line_angle_helper.reset(clear_measurement=False)
            return True, "OK AI_START"
        if cmd == "AI_STATUS":
            remain = 0 if self.ai_deviation_until is None else max(0, time.ticks_diff(self.ai_deviation_until, time.ticks_ms()))
            manual = 0 if self.ai_manual_until is None else max(0, time.ticks_diff(self.ai_manual_until, time.ticks_ms()))
            return True, "OK AI_STATUS paused={} deviation_offset={} remaining_ms={} manual_remaining_ms={}".format(
                self.ai_paused, self.ai_deviation_offset, remain, manual
            )
        if cmd == "AI_DEVIATE":
            if len(parts) < 3:
                return True, "ERR AI_DEVIATE_USAGE"
            try:
                self.ai_deviation_offset = float(parts[1])
                duration_ms = max(0, int(float(parts[2])))
            except Exception:
                return True, "ERR AI_DEVIATE_VALUE"
            self.ai_paused = False
            self.ai_deviation_until = time.ticks_ms() + duration_ms
            self.pid.reset()
            if self.line_angle_helper is not None:
                self.line_angle_helper.reset(clear_measurement=False)
            return True, "OK AI_DEVIATE offset={} duration_ms={}".format(self.ai_deviation_offset, duration_ms)
        if cmd == "AI_PERTURB":
            if len(parts) < 5:
                return True, "ERR AI_PERTURB_USAGE"
            try:
                angle = float(parts[1])
                speed = float(parts[2])
                rotate = float(parts[3])
                duration_ms = int(float(parts[4]))
            except Exception:
                return True, "ERR AI_PERTURB_VALUE"
            speed = _clamp(speed, -abs(self.config["ai_perturb_max_translate_speed"]), abs(self.config["ai_perturb_max_translate_speed"]))
            rotate = _clamp(rotate, -abs(self.config["ai_perturb_max_rotate_speed"]), abs(self.config["ai_perturb_max_rotate_speed"]))
            duration_ms = _clamp(duration_ms, 0, int(self.config["ai_perturb_max_duration_ms"]))
            rad = math.radians(angle)
            self.ai_vx = speed * math.cos(rad)
            self.ai_vy = speed * math.sin(rad)
            self.ai_rotate = rotate
            self.ai_manual_until = time.ticks_ms() + duration_ms
            self.ai_pause_after_manual = True
            self.ai_paused = False
            self.ai_deviation_until = None
            self.pid.reset()
            if self.line_angle_helper is not None:
                self.line_angle_helper.reset(clear_measurement=False)
            return True, "OK AI_PERTURB angle={} translate={} rotate={} duration_ms={}".format(angle, speed, rotate, duration_ms)
        return False, ""

    def start(self):
        self.motor.stop(0)
        gc.collect()
        self._ensure_imu()
        gc.collect()
        print("CarB IMU initializing...")
        self.imu.init()
        if self.config["imu_calibrate_on_start"]:
            print("CarB IMU calibrating, keep vehicle still.")
            time.sleep_ms(self.config["imu_calibration_settle_ms"])
            self.imu.calibrate()
            print("CarB IMU calibration finished.")
        else:
            print("CarB IMU calibration skipped.")
        gc.collect()
        self._ensure_yaw_runtime()
        if self.receiver is not None:
            self.receiver.start()
        if self.tuner is not None:
            self.tuner.start()
        gc.collect()
        self._ensure_color_pipeline()
        self._ensure_line_angle_helper()
        self.apply_runtime_config()
        self._reset_line_angle_control(clear_measurement=True, clear_target=True)
        self.vision.clear_rx()
        self.color.start()
        self.last_loop_ms = time.ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.pid.reset()
        self._reset_line_angle_control(clear_measurement=True, clear_target=True)
        if self.color is not None:
            self.color.stop()
        self.motor.stop(0)
        if self.imu is not None:
            self.imu.stop()

    def update_yaw_target(self):
        yaw_mode = _normalize_yaw_control_mode(self.config["yaw_control_mode"])
        if _yaw_mode_uses_leader(yaw_mode):
            if self.receiver is None:
                return False, "leader"
            data = self.receiver.update()
            if data is not None:
                self.leader_yaw = data["leader_yaw"]
            return self.leader_yaw is not None, "leader"
        if self.tuner is not None:
            self.tuner.update()
        if yaw_mode == 0:
            self.leader_yaw = None
            return True, "open"
        target = float(self.config["yaw_fixed_target_deg"])
        if self.ai_deviation_until is not None:
            if time.ticks_diff(self.ai_deviation_until, time.ticks_ms()) > 0:
                target += self.ai_deviation_offset
            else:
                self.ai_deviation_until = None
                self.ai_deviation_offset = 0.0
        self.leader_yaw = target
        return True, "fixed"

    def micro_adjust_enabled(self, state):
        if state is None or state.get("in_center", False):
            self.micro_cycle = 0
            return False if state is not None else True
        if max(abs(state.get("err_x", 0)), abs(state.get("err_y", 0))) > self.config["color_micro_adjust_zone"]:
            self.micro_cycle = 0
            return True
        on_cycles = max(1, int(self.config["color_micro_adjust_on_cycles"]))
        off_cycles = max(0, int(self.config["color_micro_adjust_off_cycles"]))
        period = on_cycles + off_cycles
        if period <= 1:
            return True
        enabled = self.micro_cycle < on_cycles
        self.micro_cycle = (self.micro_cycle + 1) % period
        return enabled

    def _reset_line_angle_control(self, clear_measurement=False, clear_target=False):
        if self.line_angle_helper is not None:
            self.line_angle_helper.reset(clear_measurement=clear_measurement, clear_target=clear_target)

    def _compute_line_angle_rotate_speed(self, color_state, dt_s, now_ms):
        if not self.config["line_angle_control_enabled"] or self.line_angle_helper is None:
            self._reset_line_angle_control(clear_measurement=True)
            return 0.0
        return self.line_angle_helper.compute_rotate_speed(self.config, color_state, dt_s, now_ms)

    def build_result(self, yaw_mode, imu_data, yaw_error, yaw_rotate_speed, rotate_speed, color_state, duty, vx, vy):
        if color_state is None:
            color_state = {}
        line_state = {
            "line_angle_deg": None,
            "line_angle_target_deg_runtime": None,
            "line_angle_error": None,
            "line_angle_rotate_speed": 0.0,
            "line_angle_length_px": 0.0,
        }
        if self.line_angle_helper is not None:
            line_state = self.line_angle_helper.export_state()
        return {
            "yaw_control_mode": self.config["yaw_control_mode"],
            "yaw_closed_loop_enabled": _yaw_mode_uses_closed_loop(self.config["yaw_control_mode"]),
            "yaw_mode": yaw_mode,
            "leader_yaw": self.leader_yaw,
            "follower_yaw": imu_data["yaw"],
            "yaw_error": yaw_error,
            "yaw_rotate_speed": yaw_rotate_speed,
            "line_angle_deg": line_state["line_angle_deg"],
            "line_angle_target_deg_runtime": line_state["line_angle_target_deg_runtime"],
            "line_angle_error": line_state["line_angle_error"],
            "line_angle_rotate_speed": line_state["line_angle_rotate_speed"],
            "line_angle_length_px": line_state["line_angle_length_px"],
            "line_angle_control_enabled": self.config["line_angle_control_enabled"],
            "rotate_speed": rotate_speed,
            "target_locked": color_state.get("target_locked", False),
            "in_center": color_state.get("in_center", False),
            "err_x": color_state.get("err_x", 0),
            "err_y": color_state.get("err_y", 0),
            "vx": vx,
            "vy": vy,
            "motor_duty": duty,
            "ai_control_paused": self.ai_paused,
            "ai_deviation_active": self.ai_deviation_until is not None,
            "ai_manual_active": self.ai_manual_until is not None,
        }

    def update(self):
        if not self.running or self.color is None or self.imu is None:
            return None
        color_state = self.color.update_tracking_command()
        imu_data = self.imu.update()
        if imu_data is None:
            return None
        now_ms = time.ticks_ms()
        dt_s = time.ticks_diff(now_ms, self.last_loop_ms) / 1000.0
        self.last_loop_ms = now_ms
        has_target, yaw_mode = self.update_yaw_target()
        yaw_closed_loop_enabled = _yaw_mode_uses_closed_loop(self.config["yaw_control_mode"])
        yaw_error = None if self.leader_yaw is None else _continuous_yaw_error(self.leader_yaw, imu_data["yaw"])

        if self.ai_manual_until is not None:
            if time.ticks_diff(self.ai_manual_until, now_ms) > 0:
                duty = self.motor.drive_vector(
                    self.ai_vx,
                    self.ai_vy,
                    omega=self.ai_rotate,
                    acceleration=0,
                    max_duty=self.config["drive_max_duty"],
                    min_duty_start=self.config["drive_min_duty_start"],
                    translate_duty_bias_start=self.config["drive_translate_bias_duty"],
                    rotate_duty_bias_start=self.config["drive_rotate_bias_duty"],
                )
                return self.build_result(
                    yaw_mode,
                    imu_data,
                    yaw_error,
                    self.ai_rotate,
                    self.ai_rotate,
                    color_state,
                    duty,
                    self.ai_vx,
                    self.ai_vy,
                )
            self.ai_manual_until = None
            self.ai_vx = self.ai_vy = self.ai_rotate = 0.0
            self.motor.stop(0)
            if self.ai_pause_after_manual:
                self.ai_paused = True
                self.ai_pause_after_manual = False

        if self.ai_paused or (yaw_closed_loop_enabled and not has_target):
            self.pid.reset()
            self._reset_line_angle_control(clear_measurement=False)
            self.motor.stop(0)
            return self.build_result(yaw_mode, imu_data, yaw_error, 0.0, 0.0, color_state, (0, 0, 0), 0.0, 0.0)

        if not yaw_closed_loop_enabled or yaw_error is None:
            self.pid.reset()
            yaw_rotate_speed = 0.0
        elif abs(yaw_error) <= self.config["yaw_deadband_deg"]:
            self.pid.reset()
            yaw_rotate_speed = 0.0
        else:
            yaw_rotate_speed = self.pid.compute(yaw_error, dt_s) * self.config["rotate_sign"]
            if 0.0 < abs(yaw_rotate_speed) < self.config["min_command_speed"]:
                yaw_rotate_speed = self.config["min_command_speed"] if yaw_rotate_speed > 0 else -self.config["min_command_speed"]

        line_angle_rotate_speed = self._compute_line_angle_rotate_speed(color_state, dt_s, now_ms)
        rotate_speed = _clamp(
            yaw_rotate_speed + line_angle_rotate_speed,
            -abs(float(self.config["max_rotate_speed"])),
            abs(float(self.config["max_rotate_speed"])),
        )

        vx = vy = 0.0
        if color_state and color_state["target_locked"] and color_state["moving"] and not color_state["in_center"]:
            vx = color_state["vx"]
            vy = color_state["vy"]
            vx, vy = _map_translate_vector_for_camera(vx, vy, self.config)
            vx, vy = _apply_min_vector_speed(vx, vy, self.config["min_translate_speed"])
            if not self.micro_adjust_enabled(color_state):
                vx = vy = 0.0
        else:
            self.micro_cycle = 0

        if abs(vx) <= 1e-6 and abs(vy) <= 1e-6 and abs(rotate_speed) <= 1e-6:
            self.motor.stop(0)
            duty = (0, 0, 0)
        else:
            duty = self.motor.drive_vector(
                vx,
                vy,
                omega=rotate_speed,
                acceleration=self.config["drive_acceleration"],
                max_duty=self.config["drive_max_duty"],
                min_duty_start=self.config["drive_min_duty_start"],
                translate_duty_bias_start=self.config["drive_translate_bias_duty"],
                rotate_duty_bias_start=self.config["drive_rotate_bias_duty"],
            )
        return self.build_result(yaw_mode, imu_data, yaw_error, yaw_rotate_speed, rotate_speed, color_state, duty, vx, vy)
