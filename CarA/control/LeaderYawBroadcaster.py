"""Leader-side IMU yaw broadcaster."""

from connection.CarFollowProtocol import LeaderYawSender
from imu.IMUVertical import ImuSensorVertical


class LeaderYawBroadcaster:
    """Read yaw from IMUVertical and push it to the follower."""

    def __init__(self, config):
        self.config = config
        self.imu = ImuSensorVertical(
            capture_div=config["imu_capture_div"],
            tick_ms=config["imu_tick_ms"],
            acc_range_g=config["imu_acc_range_g"],
            gyro_range_dps=config["imu_gyro_range_dps"],
            acc_alpha=config["imu_acc_alpha"],
            comp_alpha=config["imu_comp_alpha"],
            gyro_cali_n=config["imu_gyro_cali_n"],
        )
        self.link = LeaderYawSender(
            self_id=config["self_id"],
            peer_id=config["peer_id"],
            uart_id=config["uart_id"],
            baudrate=config["baudrate"],
            send_interval_ms=config["send_interval_ms"],
        )

    def start(self):
        self.imu.init()
        self.imu.calibrate()
        self.link.clear_rx()

    def update(self):
        data = self.imu.update()
        if data is None:
            return None

        sent = self.link.send_yaw(data["yaw"])
        data["sent"] = sent
        return data

    def stop(self):
        self.imu.stop()
