
from machine import Pin
import time


class PID:
    def __init__(self, kp=0, ki=0, kd=0, output_min=-9500, output_max=9500, integral_limit=None):
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


class StraightLineController:
    def __init__(self, imu, base_duty=6300, kp=50, ki=3, kd=40, dead_zone=0.3):
        self.imu = imu
        self.base_duty = base_duty
        self.dead_zone = dead_zone
        self.yaw_target = None
        self.yaw_pid = PID(kp=kp, ki=ki, kd=kd, output_min=-1500, output_max=1500, integral_limit=200)
        self.running = False

    def start(self, base_duty=None, yaw_target=None):
        if base_duty is not None:
            self.base_duty = base_duty
        self.yaw_target = yaw_target if yaw_target is not None else self.imu.yaw
        self.yaw_pid.reset()
        self.running = True

    def stop(self):
        self.running = False
        self.yaw_pid.reset()

    def update(self, motor_1, motor_3):
        data = self.imu.update()
        if data is None or not self.running:
            return None

        yaw_current = data["yaw"]
        yaw_err = (self.yaw_target - yaw_current) % 360
        if yaw_err > 180:
            yaw_err -= 360

        if abs(yaw_err) <= self.dead_zone:
            correction = 0
        else:
            correction = -self.yaw_pid.compute(0, yaw_err)

        duty_1 = -(self.base_duty + correction)
        duty_3 = self.base_duty - correction

        duty_1 = max(-7800, min(-4800, int(duty_1)))
        duty_3 = max(4800, min(7800, int(duty_3)))

        motor_1.duty(duty_1)
        motor_3.duty(duty_3)

        return {
            "yaw": yaw_current,
            "yaw_target": self.yaw_target,
            "yaw_error": yaw_err,
            "correction": correction,
            "duty_1": duty_1,
            "duty_3": duty_3
        }


if __name__ == "__main__":
    import gc
    from IMUVertical import ImuSensorVertical
    from seekfree import MOTOR_CONTROLLER

    led = Pin('C4', Pin.OUT, value=True)
    switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_PWM_D5, 15000, duty=0, invert=False)
    motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_PWM_C31, 15000, duty=0, invert=False)

    imu = ImuSensorVertical()
    imu.init()
    imu.calibrate()
    print("IMU ready.\n")

    controller = StraightLineController(imu, base_duty=6300, kp=50, ki=1, kd=10)

    print("=== PID Straight Line Demo ===")
    print(f"Base Duty: {controller.base_duty}")
    print(f"Yaw PID: Kp={controller.yaw_pid.kp}  Ki={controller.yaw_pid.ki}  Kd={controller.yaw_pid.kd}")
    print(f"Correction Range: ±{controller.yaw_pid.output_max}")
    print("Direct control: motor_1 & motor_3")
    print("\nPress switch2 (D9) to stop.")
    print("Starting in 3 seconds...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1...")
    time.sleep(1)

    controller.start(yaw_target=imu.yaw)
    print("GO! PID controlling motors directly!\n")

    while True:
        result = controller.update(motor_1, motor_3)
        if result is not None:
            led.toggle()
            if imu._tick_count % 10 == 0:
                print(f"yaw={result['yaw']:>7.1f}°  err={result['yaw_error']:>6.1f}°  corr={result['correction']:>6.0f}  M1={result['duty_1']:>5.0f} M3={result['duty_3']:>5.0f}")

        if switch2.value() != state2:
            controller.stop()
            motor_1.duty(0)
            motor_3.duty(0)
            print("\n=== PID demo stopped ===")
            break

        time.sleep_ms(1)
        gc.collect()
