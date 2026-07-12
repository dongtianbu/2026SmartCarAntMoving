"""主车和从车之间的 yaw 通信辅助模块。"""

import time

from connection.WirelessUsartCars import WirelessUsartCars


STATE_ID_LEADER_YAW = 0x31


def wrap_angle_deg(angle_deg):
    """旧版角度折返工具，仅保留给需要等效朝向判断的代码使用。"""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def continuous_angle_error(target_deg, current_deg):
    """返回连续 yaw 误差，不做 180/-180 折返。"""
    return target_deg - current_deg


class LeaderYawSender:
    """通过无线串口把主车 yaw 发送给从车。"""

    def __init__(
        self,
        self_id=1,
        peer_id=2,
        uart_id=2,
        baudrate=115200,
        send_interval_ms=20,
    ):
        self.peer_id = peer_id
        self.send_interval_ms = max(5, int(send_interval_ms))
        self.radio = WirelessUsartCars(
            uart_id=uart_id,
            baudrate=baudrate,
            self_id=self_id,
        )
        self._last_send_ms = None

    def clear_rx(self):
        self.radio.clear_rx()
        self._last_send_ms = None

    def should_send(self, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        if self._last_send_ms is None:
            return True
        return time.ticks_diff(now_ms, self._last_send_ms) >= self.send_interval_ms

    def send_yaw(self, yaw_deg, now_ms=None):
        if now_ms is None:
            now_ms = time.ticks_ms()
        if not self.should_send(now_ms):
            return False
        # 第一阶段链路测试使用可读 ASCII 文本，而不是二进制协议帧。
        # 串口助手应直接看到类似 "YAW:190.00" 或 "YAW:-200.00" 的连续角度文本行。
        # CarB/control/LeaderYawReceiveTest.py 和 FollowLeaderYawPID.py
        # 都解析同一种格式；如果这里改格式，从车接收端也必须同步修改。
        # 注意：这里不能再 wrap 到 [-180, 180]，否则主车转到 190 度会被发送成 -170 度。
        self.radio.send_line("YAW:{:.2f}".format(float(yaw_deg)))
        self._last_send_ms = now_ms
        return True
