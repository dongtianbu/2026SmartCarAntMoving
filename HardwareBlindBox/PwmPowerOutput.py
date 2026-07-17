"""HardwareBlindBox 三路互补 PWM 功率输出程序。

功能说明：
1. 同时控制 D4/D5、C28/C29、C30/C31 三路互补 PWM 输出。
2. 按配置文件中的电池电压和目标输出电压，自动换算占空比。
3. 提供固定输出、扫压、脉冲三种快速调试模式。
"""

from machine import Pin
from seekfree import MOTOR_CONTROLLER
import gc
import time

from PwmPowerOutputConfig import build_config


CHANNEL_1 = MOTOR_CONTROLLER.PWM_C28_PWM_C29
CHANNEL_2 = MOTOR_CONTROLLER.PWM_D4_PWM_D5
CHANNEL_3 = MOTOR_CONTROLLER.PWM_C30_PWM_C31


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


class StatusLed:
    """状态灯控制：输出开启时常亮，关闭时慢闪，异常时快闪。"""

    def __init__(self, pin_name, active_level, heartbeat_interval_ms):
        self.active_level = active_level
        self.inactive_level = 0 if active_level else 1
        self.heartbeat_interval_ms = max(20, int(heartbeat_interval_ms))
        self.pin = Pin(pin_name, Pin.OUT, value=self.inactive_level)
        self.last_toggle_ms = time.ticks_ms()
        self.state = False

    def enabled(self):
        self.pin.value(self.active_level)

    def disabled(self):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self.last_toggle_ms) >= self.heartbeat_interval_ms:
            self.last_toggle_ms = now_ms
            self.state = not self.state
            self.pin.value(self.active_level if self.state else self.inactive_level)

    def error(self):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self.last_toggle_ms) >= 100:
            self.last_toggle_ms = now_ms
            self.state = not self.state
            self.pin.value(self.active_level if self.state else self.inactive_level)

    def off(self):
        self.pin.value(self.inactive_level)


class TripleComplementaryPwmOutput:
    """统一管理三路互补 PWM 输出。"""

    def __init__(self, config):
        self.config = config
        self.motor_1 = None
        self.motor_2 = None
        self.motor_3 = None
        self.current_duty = 0
        self.output_enabled = bool(config["output_enabled_on_start"])
        self.stop_switch = None
        self.stop_switch_state = None
        self.toggle_key = None
        self.toggle_key_state = None
        self.last_toggle_key_ms = time.ticks_ms()
        self.last_log_ms = time.ticks_ms()
        self.mode_tick_ms = time.ticks_ms()
        self.sweep_voltage = float(config["sweep_min_output_voltage"])
        self.sweep_direction = 1
        self.pulse_high_active = True
        self.led = StatusLed(
            config["status_led_pin"],
            config["status_led_active_level"],
            config["heartbeat_interval_ms"],
        )
        self._init_inputs()

    def _init_inputs(self):
        if self.config["stop_switch_enabled"]:
            self.stop_switch = Pin(
                self.config["stop_switch_pin"],
                Pin.IN,
                pull=self.config["stop_switch_pull"],
            )
            self.stop_switch_state = self.stop_switch.value()
        if self.config["toggle_key_enabled"]:
            self.toggle_key = Pin(
                self.config["toggle_key_pin"],
                Pin.IN,
                pull=self.config["toggle_key_pull"],
            )
            self.toggle_key_state = self.toggle_key.value()

    def _ensure_outputs(self):
        if self.motor_1 is not None and self.motor_2 is not None and self.motor_3 is not None:
            return
        gc.collect()
        self.motor_1 = MOTOR_CONTROLLER(
            CHANNEL_1,
            self.config["pwm_output_frequency_hz"],
            duty=0,
            invert=self.config["channel_1_invert"],
        )
        self.motor_2 = MOTOR_CONTROLLER(
            CHANNEL_2,
            self.config["pwm_output_frequency_hz"],
            duty=0,
            invert=self.config["channel_2_invert"],
        )
        self.motor_3 = MOTOR_CONTROLLER(
            CHANNEL_3,
            self.config["pwm_output_frequency_hz"],
            duty=0,
            invert=self.config["channel_3_invert"],
        )
        gc.collect()

    def _voltage_to_duty(self, target_voltage):
        """把目标输出电压换算成 duty。"""
        battery_voltage = float(self.config["battery_input_voltage"])
        if battery_voltage <= 0:
            return 0, 0.0

        limited_voltage = _clamp(
            float(target_voltage),
            -abs(float(self.config["max_allowed_output_voltage"])),
            abs(float(self.config["max_allowed_output_voltage"])),
        )
        limited_voltage = _clamp(limited_voltage, -abs(battery_voltage), abs(battery_voltage))

        signed_voltage = limited_voltage * (1 if self.config["output_polarity_sign"] >= 0 else -1)
        duty = int(round(signed_voltage / battery_voltage * self.config["pwm_max_duty"]))

        min_effective_duty = abs(int(self.config["min_effective_duty"]))
        if duty != 0 and min_effective_duty > 0 and abs(duty) < min_effective_duty:
            duty = min_effective_duty if duty > 0 else -min_effective_duty

        duty = int(_clamp(duty, -self.config["pwm_max_duty"], self.config["pwm_max_duty"]))
        actual_voltage = battery_voltage * duty / self.config["pwm_max_duty"]
        return duty, actual_voltage

    def _apply_duty_immediately(self, duty_value):
        self._ensure_outputs()
        self.motor_1.duty(duty_value)
        self.motor_2.duty(duty_value)
        self.motor_3.duty(duty_value)
        self.current_duty = duty_value

    def _apply_duty_with_ramp(self, target_duty):
        step = max(0, int(self.config["ramp_step_duty"]))
        step_ms = max(0, int(self.config["ramp_step_ms"]))
        if step <= 0:
            self._apply_duty_immediately(target_duty)
            return

        current = self.current_duty
        while current != target_duty:
            if current < target_duty:
                current = min(current + step, target_duty)
            else:
                current = max(current - step, target_duty)
            self._apply_duty_immediately(current)
            if step_ms > 0:
                time.sleep_ms(step_ms)

    def set_output_voltage(self, target_voltage):
        target_duty, actual_voltage = self._voltage_to_duty(target_voltage)
        self._apply_duty_with_ramp(target_duty)
        return target_duty, actual_voltage

    def disable_output(self):
        self._apply_duty_with_ramp(0)

    def should_stop(self):
        return self.stop_switch is not None and self.stop_switch.value() != self.stop_switch_state

    def update_toggle_key(self):
        if self.toggle_key is None:
            return False
        current_state = self.toggle_key.value()
        if current_state == 0 and self.toggle_key_state != 0:
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, self.last_toggle_key_ms) >= self.config["toggle_key_debounce_ms"]:
                self.last_toggle_key_ms = now_ms
                self.output_enabled = not self.output_enabled
                return True
        self.toggle_key_state = current_state
        return False

    def _mode_fixed(self):
        return float(self.config["target_output_voltage"])

    def _mode_sweep(self):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self.mode_tick_ms) >= self.config["sweep_step_interval_ms"]:
            self.mode_tick_ms = now_ms
            self.sweep_voltage += self.sweep_direction * float(self.config["sweep_step_voltage"])
            sweep_min = min(
                float(self.config["sweep_min_output_voltage"]),
                float(self.config["sweep_max_output_voltage"]),
            )
            sweep_max = max(
                float(self.config["sweep_min_output_voltage"]),
                float(self.config["sweep_max_output_voltage"]),
            )
            if self.sweep_voltage >= sweep_max:
                self.sweep_voltage = sweep_max
                self.sweep_direction = -1
            elif self.sweep_voltage <= sweep_min:
                self.sweep_voltage = sweep_min
                self.sweep_direction = 1
        return self.sweep_voltage

    def _mode_pulse(self):
        now_ms = time.ticks_ms()
        hold_ms = self.config["pulse_high_hold_ms"] if self.pulse_high_active else self.config["pulse_low_hold_ms"]
        if time.ticks_diff(now_ms, self.mode_tick_ms) >= hold_ms:
            self.mode_tick_ms = now_ms
            self.pulse_high_active = not self.pulse_high_active
        if self.pulse_high_active:
            return float(self.config["pulse_high_output_voltage"])
        return float(self.config["pulse_low_output_voltage"])

    def compute_target_voltage(self):
        mode = int(self.config["output_mode"])
        if mode == 1:
            return self._mode_sweep()
        if mode == 2:
            return self._mode_pulse()
        return self._mode_fixed()

    def log_status(self, target_voltage, target_duty, actual_voltage):
        if not self.config["enable_serial_log"]:
            return
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self.last_log_ms) < self.config["log_interval_ms"]:
            return
        self.last_log_ms = now_ms
        print(
            "enabled={} mode={} battery={:.2f}V target={:.2f}V actual={:.2f}V duty={} freq={}Hz".format(
                self.output_enabled,
                self.config["output_mode"],
                float(self.config["battery_input_voltage"]),
                float(target_voltage),
                float(actual_voltage),
                int(target_duty),
                int(self.config["pwm_output_frequency_hz"]),
            )
        )

    def print_startup_info(self):
        if not self.config["show_startup_info"]:
            return
        print("=== HardwareBlindBox Triple Complementary PWM Output ===")
        print("channels: C28/C29, D4/D5, C30/C31")
        print("battery_input_voltage={}V".format(self.config["battery_input_voltage"]))
        print("pwm_output_frequency={}Hz".format(self.config["pwm_output_frequency_hz"]))
        print("target_output_voltage={}V".format(self.config["target_output_voltage"]))
        print("output_mode={}".format(self.config["output_mode"]))
        print("output_enabled_on_start={}".format(self.config["output_enabled_on_start"]))
        print("toggle_key_enabled={}".format(self.config["toggle_key_enabled"]))
        print("stop_switch_enabled={}".format(self.config["stop_switch_enabled"]))

    def run(self):
        self.print_startup_info()
        self._ensure_outputs()
        if self.config["startup_delay_ms"] > 0:
            time.sleep_ms(int(self.config["startup_delay_ms"]))

        try:
            while True:
                if self.should_stop():
                    if self.config["enable_serial_log"]:
                        print("Stop switch triggered, exit program.")
                    break

                if self.update_toggle_key() and self.config["enable_serial_log"]:
                    print("Toggle key pressed, output_enabled={}".format(self.output_enabled))

                if self.output_enabled:
                    target_voltage = self.compute_target_voltage()
                    target_duty, actual_voltage = self.set_output_voltage(target_voltage)
                    self.led.enabled()
                    self.log_status(target_voltage, target_duty, actual_voltage)
                else:
                    self.disable_output()
                    self.led.disabled()

                gc.collect()
                time.sleep_ms(5)
        finally:
            if self.config["zero_output_on_exit"]:
                self.disable_output()
            self.led.off()


def run_pwm_power_output():
    config = build_config()
    app = TripleComplementaryPwmOutput(config)
    app.run()


if __name__ == "__main__":
    run_pwm_power_output()
