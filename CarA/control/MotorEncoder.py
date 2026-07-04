
from smartcar import encoder

ENCODER_PINS = {
    1: {"A": "C2", "B": "C3"},
    2: {"A": "D13", "B": "D14"},
    3: {"A": "D15", "B": "D16"}
}

PPR = 7


class MotorEncoder:

    def __init__(self, enc_id=1, invert=False):
        if enc_id not in ENCODER_PINS:
            raise ValueError("enc_id must be 1, 2 or 3")
        self._id = enc_id
        self._pins = ENCODER_PINS[enc_id]
        self._enc = encoder(self._pins["A"], self._pins["B"], invert)
        self._last_count = 0
        self._last_time = 0
        self._data = self._enc.get()

    def capture(self):
        self._enc.capture()

    def get(self):
        self._data = self._enc.get()
        return self._data

    def read(self):
        return self._enc.read()

    @property
    def count(self):
        return self.get()

    @property
    def pins(self):
        return self._pins

    def reset_speed(self):
        self._last_count = self.get()
        import time
        self._last_time = time.ticks_ms()

    def get_speed_rpm(self):
        import time
        current_count = self.get()
        current_time = time.ticks_ms()
        delta_count = current_count - self._last_count
        delta_time = time.ticks_diff(current_time, self._last_time)
        if delta_time == 0:
            return 0.0
        speed_cps = delta_count / (delta_time / 1000.0)
        speed_rpm = (speed_cps / PPR) * 60.0
        self._last_count = current_count
        self._last_time = current_time
        return speed_rpm

    def get_speed_radps(self):
        rpm = self.get_speed_rpm()
        return rpm * 6.28318530718 / 60.0


def create_encoder(enc_id=1, invert=False):
    return MotorEncoder(enc_id, invert)


def create_all_encoders(invert_list=(False, False, False)):
    encoders = []
    for i in range(3):
        encoders.append(MotorEncoder(i + 1, invert_list[i]))
    return encoders


if __name__ == "__main__":
    from machine import Pin
    from smartcar import ticker
    import gc

    print("[1] init pins...", end="")
    led = Pin('C4', Pin.OUT, value=True)
    switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()
    print("done")

    print("[2] create encoders...", end="")
    encoders = create_all_encoders()
    print("done")

    ticker_flag = False
    ticker_count = 0

    def pit_handler(instance):
        global ticker_flag, ticker_count
        ticker_flag = True
        ticker_count = (ticker_count + 1) if (ticker_count < 100) else 1

    print("[3] init ticker...", end="")
    pit = ticker(1)
    pit.capture_list(*(e._enc for e in encoders))
    pit.callback(pit_handler)
    pit.start(10)
    print("done")

    print("\nEncoder Test Started - 3 channels")
    print("enc_id | A_pin | B_pin | total_count")
    print("-" * 40)

    for i, enc in enumerate(encoders):
        print("   {}    | {:>5s} | {:>5s} | init".format(
            i + 1, enc.pins["A"], enc.pins["B"]))

    print("-" * 40)
    print("Press switch2 (D9) to stop.\n")

    total_counts = [0, 0, 0]

    while True:
        if ticker_flag and ticker_count % 20 == 0:
            led.toggle()
            line = ""
            for i, enc in enumerate(encoders):
                delta = enc.get()
                total_counts[i] += delta
                line += "ENC{}:{:>8d}  ".format(i + 1, total_counts[i])
            print("\r" + line, end="")
            ticker_flag = False

        if switch2.value() != state2:
            pit.stop()
            print("Encoder test stopped.")
            break

        gc.collect()
