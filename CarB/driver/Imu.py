from machine import Pin
from smartcar import ticker
import time
import math

IMU_CLASS_NAME = ""
try:
    from seekfree import IMU660RX as IMU660
    IMU_CLASS_NAME = "IMU660RX"
except ImportError:
    from seekfree import IMU660RA as IMU660
    IMU_CLASS_NAME = "IMU660RA"

ACC_LSB_PER_G_MAP = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_LSB_PER_DPS_MAP = {2000: 16.4, 1000: 32.8, 500: 65.6, 250: 131.2, 125: 262.4}


class ImuSensor:

    def __init__(self, capture_div=1, tick_ms=10,
                 acc_range_g=8, gyro_range_dps=2000,
                 acc_alpha=0.20, comp_alpha=0.98,
                 gyro_cali_n=300,
                 sign_ax=1, sign_ay=1, sign_az=1,
                 sign_gx=1, sign_gy=1, sign_gz=1):
        self._capture_div = capture_div
        self._tick_ms = tick_ms
        self._acc_lsb_per_g = ACC_LSB_PER_G_MAP.get(acc_range_g, 4096.0)
        self._gyro_lsb_per_dps = GYRO_LSB_PER_DPS_MAP.get(gyro_range_dps, 16.4)
        self._acc_alpha = acc_alpha
        self._comp_alpha = comp_alpha
        self._gyro_cali_n = gyro_cali_n
        self._sign_acc = (sign_ax, sign_ay, sign_az)
        self._sign_gyro = (sign_gx, sign_gy, sign_gz)

        self._imu = None
        self._imu_data = None
        self._pit = None
        self._ticker_flag = False

        self._bias_gx = 0.0
        self._bias_gy = 0.0
        self._bias_gz = 0.0

        self._ax_f = 0.0
        self._ay_f = 0.0
        self._az_f = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0
        self._gx = 0.0
        self._gy = 0.0
        self._gz = 0.0

        self._last_ms = 0
        self._tick_count = 0
        self._calibrated = False

    def init(self):
        IMU660.help()
        self._imu = IMU660(self._capture_div)
        self._imu.info()
        self._imu_data = self._imu.get()

        def on_tick(t):
            self._ticker_flag = True
            self._tick_count += 1

        self._pit = ticker(1)
        self._pit.capture_list(self._imu)
        self._pit.callback(on_tick)
        self._pit.start(self._tick_ms)

        count = 0
        while count < 20:
            if self._ticker_flag:
                self._ticker_flag = False
                count += 1
            time.sleep_ms(1)

        print("Using", IMU_CLASS_NAME)
        return True

    def calibrate(self):
        print("Gyro calibrating... keep still")
        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0
        n = 0
        while n < self._gyro_cali_n:
            if self._ticker_flag:
                self._ticker_flag = False
                sum_gx += self._imu_data[3]
                sum_gy += self._imu_data[4]
                sum_gz += self._imu_data[5]
                n += 1
                if n % 100 == 0:
                    print("cali:", n)
            time.sleep_ms(1)

        self._bias_gx = sum_gx / self._gyro_cali_n
        self._bias_gy = sum_gy / self._gyro_cali_n
        self._bias_gz = sum_gz / self._gyro_cali_n
        print("gyro bias(raw):", self._bias_gx, self._bias_gy, self._bias_gz)

        self._ax_f = self._sign_acc[0] * self._imu_data[0] / self._acc_lsb_per_g
        self._ay_f = self._sign_acc[1] * self._imu_data[1] / self._acc_lsb_per_g
        self._az_f = self._sign_acc[2] * self._imu_data[2] / self._acc_lsb_per_g

        self._roll = math.degrees(math.atan2(self._ay_f, self._az_f))
        self._pitch = math.degrees(math.atan2(-self._ax_f,
                                                math.sqrt(self._ay_f ** 2 + self._az_f ** 2)))
        self._yaw = 0.0
        self._last_ms = time.ticks_ms()
        self._calibrated = True
        print("Calibration done.")
        return True

    def update(self):
        if not self._ticker_flag:
            return None
        self._ticker_flag = False

        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, self._last_ms) / 1000.0
        self._last_ms = now_ms
        if dt <= 0:
            dt = self._tick_ms / 1000.0

        ax_raw = self._imu_data[0]
        ay_raw = self._imu_data[1]
        az_raw = self._imu_data[2]
        gx_raw = self._imu_data[3]
        gy_raw = self._imu_data[4]
        gz_raw = self._imu_data[5]

        ax = self._sign_acc[0] * ax_raw / self._acc_lsb_per_g
        ay = self._sign_acc[1] * ay_raw / self._acc_lsb_per_g
        az = self._sign_acc[2] * az_raw / self._acc_lsb_per_g

        gx = self._sign_gyro[0] * (gx_raw - self._bias_gx) / self._gyro_lsb_per_dps
        gy = self._sign_gyro[1] * (gy_raw - self._bias_gy) / self._gyro_lsb_per_dps
        gz = self._sign_gyro[2] * (gz_raw - self._bias_gz) / self._gyro_lsb_per_dps

        self._ax = ax
        self._ay = ay
        self._az = az
        self._gx = gx
        self._gy = gy
        self._gz = gz

        self._ax_f += self._acc_alpha * (ax - self._ax_f)
        self._ay_f += self._acc_alpha * (ay - self._ay_f)
        self._az_f += self._acc_alpha * (az - self._az_f)

        roll_acc = math.degrees(math.atan2(self._ay_f, self._az_f))
        pitch_acc = math.degrees(math.atan2(-self._ax_f,
                                             math.sqrt(self._ay_f ** 2 + self._az_f ** 2)))

        self._roll = self._comp_alpha * (self._roll + gx * dt) + (1.0 - self._comp_alpha) * roll_acc
        self._pitch = self._comp_alpha * (self._pitch + gy * dt) + (1.0 - self._comp_alpha) * pitch_acc

        # yaw 使用连续积分角度，不再限制到 [-180, 180]。
        # 因此正向转过 180 度后会继续变成 190、200...，负向同理会继续变成 -190、-200...。
        self._yaw += gz * dt

        return {
            "acc_x": self._ax_f, "acc_y": self._ay_f, "acc_z": self._az_f,
            "gyro_x": self._gx, "gyro_y": self._gy, "gyro_z": self._gz,
            "roll": self._roll, "pitch": self._pitch, "yaw": self._yaw
        }

    @property
    def acc_x(self):
        return self._ax_f

    @property
    def acc_y(self):
        return self._ay_f

    @property
    def acc_z(self):
        return self._az_f

    @property
    def gyro_x(self):
        return self._gx

    @property
    def gyro_y(self):
        return self._gy

    @property
    def gyro_z(self):
        return self._gz

    @property
    def roll(self):
        return self._roll

    @property
    def pitch(self):
        return self._pitch

    @property
    def yaw(self):
        return self._yaw

    @property
    def raw(self):
        return (
            self._imu_data[0], self._imu_data[1], self._imu_data[2],
            self._imu_data[3], self._imu_data[4], self._imu_data[5]
        )

    def stop(self):
        if self._pit is not None:
            self._pit.stop()


if __name__ == "__main__":
    import gc
    led = Pin('C4', Pin.OUT, value=True)
    switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    imu = ImuSensor()
    imu.init()
    imu.calibrate()
    print("Start running...\n")

    while True:
        data = imu.update()
        if data is not None:
            led.toggle()
            if imu._tick_count % 5 == 0:
                print("RPY: {:>7.2f}, {:>7.2f}, {:>7.2f}".format(
                    data["roll"], data["pitch"], data["yaw"]))
                print("GYRO(dps): {:>7.2f}, {:>7.2f}, {:>7.2f}".format(
                    data["gyro_x"], data["gyro_y"], data["gyro_z"]))
                print("ACC(g): {:>7.3f}, {:>7.3f}, {:>7.3f}".format(
                    data["acc_x"], data["acc_y"], data["acc_z"]))
                print("---")
        if switch2.value() != state2:
            imu.stop()
            print("Test program stop.")
            break
        time.sleep_ms(1)
        gc.collect()
