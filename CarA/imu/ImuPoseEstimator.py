import math
import time

from imu.IMUVertical import ImuSensorVertical


# ===== Pose estimator config =====
# These values are meant for short-range relative pose estimation only.

# Gravity constant used for converting g to m/s^2.
GRAVITY_MPS2 = 9.80665

# Small plane acceleration is treated as noise and forced to zero.
PLANE_ACC_DEADBAND_G = 0.03

# Small velocity is forced to zero to reduce low-speed drift.
VELOCITY_DEADBAND_MPS = 0.02

# Stationary detection thresholds.
STATIONARY_ACC_THRESHOLD_G = 0.05
STATIONARY_GYRO_THRESHOLD_DPS = 2.0
STATIONARY_HOLD_MS = 0

# World-frame acceleration low-pass factor.
ACC_WORLD_ALPHA = 0.35

# Stationary bias adaptation for horizontal plane acceleration.
PLANE_BIAS_ALPHA = 0.08

# Confidence update rates.
CONFIDENCE_MIN = 0.05
CONFIDENCE_MAX = 1.0
CONFIDENCE_DECAY_PER_SEC = 0.08
CONFIDENCE_RECOVER_PER_SEC = 0.25

# Vertical-mount pose mapping.
# The current car reports yaw decreasing from 0, so use -1 to flip yaw direction.
YAW_SIGN = -1.0

# Select which IMUVertical horizontal plane axis becomes pose x / pose y.
# Current install:
# x positive = left
# y positive = down
# z positive = backward
# So the ground plane is x/z, and forward is -z.
POSE_X_SOURCE = "z"
POSE_X_SIGN = -1.0
POSE_Y_SOURCE = "x"
POSE_Y_SIGN = 1.0

# Console print throttling for standalone test mode.
TEST_PRINT_EVERY = 15


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _wrap_angle_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def _select_axis_value(axis_x, axis_z, source, sign):
    if source == "x":
        value = axis_x
    elif source == "z":
        value = axis_z
    else:
        raise ValueError("source must be 'x' or 'z'")
    return float(sign) * value


class ImuPoseEstimator:
    def __init__(
        self,
        imu=None,
        acc_world_alpha=ACC_WORLD_ALPHA,
        plane_acc_deadband_g=PLANE_ACC_DEADBAND_G,
        velocity_deadband_mps=VELOCITY_DEADBAND_MPS,
        stationary_acc_threshold_g=STATIONARY_ACC_THRESHOLD_G,
        stationary_gyro_threshold_dps=STATIONARY_GYRO_THRESHOLD_DPS,
        stationary_hold_ms=STATIONARY_HOLD_MS,
        plane_bias_alpha=PLANE_BIAS_ALPHA,
        confidence_decay_per_sec=CONFIDENCE_DECAY_PER_SEC,
        confidence_recover_per_sec=CONFIDENCE_RECOVER_PER_SEC,
        yaw_sign=YAW_SIGN,
        pose_x_source=POSE_X_SOURCE,
        pose_x_sign=POSE_X_SIGN,
        pose_y_source=POSE_Y_SOURCE,
        pose_y_sign=POSE_Y_SIGN,
    ):
        self.imu = imu if imu is not None else ImuSensorVertical()

        self.acc_world_alpha = _clamp(float(acc_world_alpha), 0.05, 1.0)
        self.plane_acc_deadband_g = max(0.0, float(plane_acc_deadband_g))
        self.velocity_deadband_mps = max(0.0, float(velocity_deadband_mps))
        self.stationary_acc_threshold_g = max(0.0, float(stationary_acc_threshold_g))
        self.stationary_gyro_threshold_dps = max(0.0, float(stationary_gyro_threshold_dps))
        self.stationary_hold_ms = max(0, int(stationary_hold_ms))
        self.plane_bias_alpha = _clamp(float(plane_bias_alpha), 0.0, 1.0)
        self.confidence_decay_per_sec = max(0.0, float(confidence_decay_per_sec))
        self.confidence_recover_per_sec = max(0.0, float(confidence_recover_per_sec))
        self.yaw_sign = -1.0 if float(yaw_sign) < 0 else 1.0
        self.pose_x_source = pose_x_source
        self.pose_x_sign = -1.0 if float(pose_x_sign) < 0 else 1.0
        self.pose_y_source = pose_y_source
        self.pose_y_sign = -1.0 if float(pose_y_sign) < 0 else 1.0

        self._last_update_ms = None
        self._stationary_since_ms = None

        self._yaw_origin_deg = 0.0
        self._yaw_reset_deg = 0.0

        self._ax_world = 0.0
        self._ay_world = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._x = 0.0
        self._y = 0.0
        self._plane_bias_x_g = 0.0
        self._plane_bias_y_g = 0.0
        self._confidence = CONFIDENCE_MAX
        self._stationary = True
        self._initialized = False
        self._print_counter = 0

    def init(self):
        ok = self.imu.init()
        if ok:
            self._initialized = True
            self._last_update_ms = None
        return ok

    def calibrate(self):
        ok = self.imu.calibrate()
        if ok:
            self.reset_pose()
        return ok

    def stop(self):
        self.imu.stop()

    def reset_pose(self, x=0.0, y=0.0, yaw=0.0):
        self._x = float(x)
        self._y = float(y)
        self._vx = 0.0
        self._vy = 0.0
        self._ax_world = 0.0
        self._ay_world = 0.0
        self._plane_bias_x_g = _select_axis_value(
            self.imu.acc_x, self.imu.acc_z, self.pose_x_source, self.pose_x_sign
        )
        self._plane_bias_y_g = _select_axis_value(
            self.imu.acc_x, self.imu.acc_z, self.pose_y_source, self.pose_y_sign
        )
        self._yaw_origin_deg = float(self.imu.yaw)
        self._yaw_reset_deg = float(yaw)
        self._last_update_ms = time.ticks_ms()
        self._stationary_since_ms = self._last_update_ms
        self._stationary = True
        self._confidence = CONFIDENCE_MAX
        self._print_counter = 0

    def _current_yaw_deg(self):
        yaw_delta = self.yaw_sign * (self.imu.yaw - self._yaw_origin_deg)
        return _wrap_angle_deg(yaw_delta + self._yaw_reset_deg)

    def _body_to_world(self, ax_body, ay_body, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        ax_world = (ax_body * cos_yaw) - (ay_body * sin_yaw)
        ay_world = (ax_body * sin_yaw) + (ay_body * cos_yaw)
        return ax_world, ay_world

    def _apply_deadband(self, value, threshold):
        if abs(value) < threshold:
            return 0.0
        return value

    def _update_stationary_state(self, now_ms, plane_acc_mag_g, gyro_mag_dps):
        candidate_stationary = (
            plane_acc_mag_g <= self.stationary_acc_threshold_g
            and gyro_mag_dps <= self.stationary_gyro_threshold_dps
        )

        if candidate_stationary:
            if self._stationary_since_ms is None:
                self._stationary_since_ms = now_ms
            self._stationary = (
                self.stationary_hold_ms <= 0
                or time.ticks_diff(now_ms, self._stationary_since_ms) >= self.stationary_hold_ms
            )
        else:
            self._stationary_since_ms = None
            self._stationary = False

        return self._stationary

    def _update_confidence(self, stationary, dt):
        if stationary:
            self._confidence += self.confidence_recover_per_sec * dt
        else:
            self._confidence -= self.confidence_decay_per_sec * dt
        self._confidence = _clamp(self._confidence, CONFIDENCE_MIN, CONFIDENCE_MAX)
        return self._confidence

    def update(self):
        if not self._initialized:
            return None

        imu_state = self.imu.update()
        if imu_state is None:
            return None

        now_ms = time.ticks_ms()
        if self._last_update_ms is None:
            self._last_update_ms = now_ms
            return None

        dt = time.ticks_diff(now_ms, self._last_update_ms) / 1000.0
        self._last_update_ms = now_ms
        if dt <= 0:
            dt = 0.01

        roll_deg = imu_state["roll"]
        pitch_deg = imu_state["pitch"]
        yaw_deg = self._current_yaw_deg()

        acc_x_g = imu_state["acc_x"]
        acc_z_g = imu_state["acc_z"]
        gyro_mag_dps = math.sqrt(
            (imu_state["gyro_x"] * imu_state["gyro_x"])
            + (imu_state["gyro_y"] * imu_state["gyro_y"])
            + (imu_state["gyro_z"] * imu_state["gyro_z"])
        )

        raw_pose_ax_body_g = _select_axis_value(
            acc_x_g, acc_z_g, self.pose_x_source, self.pose_x_sign
        )
        raw_pose_ay_body_g = _select_axis_value(
            acc_x_g, acc_z_g, self.pose_y_source, self.pose_y_sign
        )

        pose_ax_body_g = raw_pose_ax_body_g - self._plane_bias_x_g
        pose_ay_body_g = raw_pose_ay_body_g - self._plane_bias_y_g

        plane_acc_mag_g = math.sqrt(
            (pose_ax_body_g * pose_ax_body_g)
            + (pose_ay_body_g * pose_ay_body_g)
        )
        stationary = self._update_stationary_state(now_ms, plane_acc_mag_g, gyro_mag_dps)

        if stationary:
            self._plane_bias_x_g += self.plane_bias_alpha * (raw_pose_ax_body_g - self._plane_bias_x_g)
            self._plane_bias_y_g += self.plane_bias_alpha * (raw_pose_ay_body_g - self._plane_bias_y_g)
            pose_ax_body_g = raw_pose_ax_body_g - self._plane_bias_x_g
            pose_ay_body_g = raw_pose_ay_body_g - self._plane_bias_y_g

        pose_ax_body_g = self._apply_deadband(pose_ax_body_g, self.plane_acc_deadband_g)
        pose_ay_body_g = self._apply_deadband(pose_ay_body_g, self.plane_acc_deadband_g)

        lin_ax_world_g, lin_ay_world_g = self._body_to_world(pose_ax_body_g, pose_ay_body_g, yaw_deg)

        self._ax_world += self.acc_world_alpha * ((lin_ax_world_g * GRAVITY_MPS2) - self._ax_world)
        self._ay_world += self.acc_world_alpha * ((lin_ay_world_g * GRAVITY_MPS2) - self._ay_world)

        if stationary:
            self._vx = 0.0
            self._vy = 0.0
        else:
            self._vx += self._ax_world * dt
            self._vy += self._ay_world * dt
            self._vx = self._apply_deadband(self._vx, self.velocity_deadband_mps)
            self._vy = self._apply_deadband(self._vy, self.velocity_deadband_mps)

        self._x += self._vx * dt
        self._y += self._vy * dt

        confidence = self._update_confidence(stationary, dt)

        return {
            "x": self._x,
            "y": self._y,
            "vx": self._vx,
            "vy": self._vy,
            "yaw": yaw_deg,
            "roll": roll_deg,
            "pitch": pitch_deg,
            "ax_world": self._ax_world,
            "ay_world": self._ay_world,
            "ax_body_g": acc_x_g,
            "ay_body_g": imu_state["acc_y"],
            "az_body_g": acc_z_g,
            "pose_ax_body_g": pose_ax_body_g,
            "pose_ay_body_g": pose_ay_body_g,
            "pose_ax_raw_g": raw_pose_ax_body_g,
            "pose_ay_raw_g": raw_pose_ay_body_g,
            "stationary": stationary,
            "confidence": confidence,
            "dt": dt,
        }


if __name__ == "__main__":
    from machine import Pin
    import gc

    led = Pin("C4", Pin.OUT, value=True)
    switch2 = Pin("D9", Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    pose = ImuPoseEstimator()
    pose.init()
    pose.calibrate()
    pose.reset_pose()

    print("ImuPoseEstimator running...\n")

    while True:
        data = pose.update()
        if data is not None:
            led.toggle()
            pose._print_counter += 1
            if pose._print_counter % TEST_PRINT_EVERY == 0:
                print(
                    "pos=({:>7.3f},{:>7.3f})m vel=({:>6.3f},{:>6.3f})m/s yaw={:>7.2f} "
                    "acc=({:>6.3f},{:>6.3f}) raw=({:>6.3f},{:>6.3f}) body=({:>6.3f},{:>6.3f}) stationary={} conf={:>4.2f}".format(
                        data["x"],
                        data["y"],
                        data["vx"],
                        data["vy"],
                        data["yaw"],
                        data["ax_world"],
                        data["ay_world"],
                        data["pose_ax_raw_g"],
                        data["pose_ay_raw_g"],
                        data["pose_ax_body_g"],
                        data["pose_ay_body_g"],
                        data["stationary"],
                        data["confidence"],
                    )
                )

        if switch2.value() != state2:
            pose.stop()
            print("Test program stop.")
            break

        time.sleep_ms(1)
        gc.collect()
