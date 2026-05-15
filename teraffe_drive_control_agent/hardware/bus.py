import can


class CanBus:
    def __init__(self, channel="can0", bitrate=1000000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

    def open(self):
        self.bus = can.interface.Bus(
            channel=self.channel,
            bustype="socketcan",
        )

    def close(self):
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def send_extended(self, arbitration_id, data):
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=True,
        )

        self.bus.send(msg)