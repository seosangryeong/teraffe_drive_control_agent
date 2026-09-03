#!/usr/bin/env python3
"""PyQt status monitor for Dynamixel XH430 servos."""

import argparse
import math
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFontMetrics
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from teraffe_drive_control_agent.config.dynamixel_config import (
    ADDR_FIRMWARE_VERSION,
    ADDR_HARDWARE_ERROR_STATUS,
    ADDR_MODEL_NUMBER,
    ADDR_MOVING,
    ADDR_MOVING_STATUS,
    ADDR_OPERATING_MODE,
    ADDR_PRESENT_CURRENT,
    ADDR_PRESENT_INPUT_VOLTAGE,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VELOCITY,
    ADDR_TORQUE_ENABLE,
    PRESENT_VELOCITY_UNIT,
    PROTOCOL_VERSION,
)
from teraffe_drive_control_agent.config.teraffe_config import (
    DYNAMIXEL_BAUDRATE,
    DYNAMIXEL_PORT,
)


CURRENT_UNIT_A = 0.00269
POSITION_RESOLUTION = 4096.0

OPERATING_MODE_TEXT = {
    0: "Current",
    1: "Velocity",
    3: "Position",
    4: "Extended Position",
    5: "Current Position",
    16: "PWM",
}

HARDWARE_ERROR_BITS = {
    0: "Input voltage",
    2: "Overheating",
    3: "Motor encoder",
    4: "Electrical shock",
    5: "Overload",
}


def u16_to_i16(value):
    if value >= 2**15:
        value -= 2**16
    return value


def u32_to_i32(value):
    if value >= 2**31:
        value -= 2**32
    return value


def position_to_deg(raw):
    return raw * 360.0 / POSITION_RESOLUTION


def hardware_error_text(value):
    if value is None:
        return "--"
    if value == 0:
        return "OK"
    names = [name for bit, name in HARDWARE_ERROR_BITS.items() if value & (1 << bit)]
    return ", ".join(names) if names else f"0x{value:02X}"


class DynamixelReader(QObject):
    sample_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    finished = pyqtSignal()

    def __init__(self, port, baudrate, dxl_id, period_s):
        super().__init__()
        self.port_name = port
        self.baudrate = baudrate
        self.dxl_id = dxl_id
        self.period_s = period_s
        self.running = False
        self.port = None
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.ok_count = 0
        self.fail_count = 0

    def run(self):
        self.running = True
        self.port = PortHandler(self.port_name)
        try:
            if not self.port.openPort():
                raise RuntimeError("openPort returned false")
            if not self.port.setBaudRate(self.baudrate):
                raise RuntimeError("setBaudRate returned false")
        except Exception as exc:
            self.connection_changed.emit(False, f"Port open failed: {exc}")
            self.finished.emit()
            return

        self.connection_changed.emit(True, f"Connected to {self.port_name}")
        try:
            while self.running:
                started = time.monotonic()
                self.sample_received.emit(self.read_status())
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, self.period_s - elapsed))
        except Exception as exc:
            self.connection_changed.emit(False, f"Read failed: {exc}")
        finally:
            if self.port is not None:
                self.port.closePort()
                self.port = None
            self.connection_changed.emit(False, "Disconnected")
            self.finished.emit()

    def stop(self):
        self.running = False

    def read_status(self):
        errors = []
        model = self.read_u16(ADDR_MODEL_NUMBER, errors)
        firmware = self.read_u8(ADDR_FIRMWARE_VERSION, errors)
        torque = self.read_u8(ADDR_TORQUE_ENABLE, errors)
        operating_mode = self.read_u8(ADDR_OPERATING_MODE, errors)
        current_raw_u = self.read_u16(ADDR_PRESENT_CURRENT, errors)
        velocity_raw_u = self.read_u32(ADDR_PRESENT_VELOCITY, errors)
        position_raw_u = self.read_u32(ADDR_PRESENT_POSITION, errors)
        voltage_raw = self.read_u16(ADDR_PRESENT_INPUT_VOLTAGE, errors)
        temperature = self.read_u8(ADDR_PRESENT_TEMPERATURE, errors)
        moving = self.read_u8(ADDR_MOVING, errors)
        moving_status = self.read_u8(ADDR_MOVING_STATUS, errors)
        hw_error = self.read_u8(ADDR_HARDWARE_ERROR_STATUS, errors)

        current_raw = u16_to_i16(current_raw_u) if current_raw_u is not None else None
        velocity_raw = u32_to_i32(velocity_raw_u) if velocity_raw_u is not None else None
        position_raw = u32_to_i32(position_raw_u) if position_raw_u is not None else None

        ok = not errors
        if ok:
            self.ok_count += 1
        else:
            self.fail_count += 1

        return {
            "timestamp": time.time(),
            "id": self.dxl_id,
            "ok": ok,
            "errors": "; ".join(errors) if errors else "OK",
            "model": model,
            "firmware": firmware,
            "torque": torque,
            "operating_mode": operating_mode,
            "operating_mode_text": OPERATING_MODE_TEXT.get(operating_mode, str(operating_mode)),
            "current_raw": current_raw,
            "current_a": current_raw * CURRENT_UNIT_A if current_raw is not None else None,
            "velocity_raw": velocity_raw,
            "velocity_rad_s": velocity_raw * PRESENT_VELOCITY_UNIT if velocity_raw is not None else None,
            "velocity_rpm": (
                velocity_raw * PRESENT_VELOCITY_UNIT * 60.0 / (2.0 * math.pi)
                if velocity_raw is not None
                else None
            ),
            "position_raw": position_raw,
            "position_deg": position_to_deg(position_raw) if position_raw is not None else None,
            "position_rad": (
                position_to_deg(position_raw) * math.pi / 180.0 if position_raw is not None else None
            ),
            "voltage_v": voltage_raw * 0.1 if voltage_raw is not None else None,
            "temperature_c": temperature,
            "moving": moving,
            "moving_status": moving_status,
            "hardware_error": hw_error,
            "hardware_error_text": hardware_error_text(hw_error),
            "ok_count": self.ok_count,
            "fail_count": self.fail_count,
        }

    def read_u8(self, address, errors):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(
            self.port,
            self.dxl_id,
            address,
        )
        return self._check_read(address, value, comm_result, dxl_error, errors)

    def read_u16(self, address, errors):
        value, comm_result, dxl_error = self.packet.read2ByteTxRx(
            self.port,
            self.dxl_id,
            address,
        )
        return self._check_read(address, value, comm_result, dxl_error, errors)

    def read_u32(self, address, errors):
        value, comm_result, dxl_error = self.packet.read4ByteTxRx(
            self.port,
            self.dxl_id,
            address,
        )
        return self._check_read(address, value, comm_result, dxl_error, errors)

    def _check_read(self, address, value, comm_result, dxl_error, errors):
        if comm_result != COMM_SUCCESS:
            errors.append(f"addr {address}: {self.packet.getTxRxResult(comm_result)}")
            return None
        if dxl_error != 0:
            errors.append(f"addr {address}: {self.packet.getRxPacketError(dxl_error)}")
            return None
        return value


class DynamixelStatusWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.thread = None
        self.reader = None
        self.last_timestamp = None

        self.setWindowTitle("Dynamixel XH430 Status Monitor")
        self._build_ui()
        self.setMinimumSize(420, 520)
        self.setMaximumSize(620, 760)
        self.resize(520, 640)

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.refresh_age)
        self.age_timer.start(100)

        if args.autostart:
            self.start_reader()

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        controls = QGroupBox("Dynamixel")
        form = QFormLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        if self.args.port:
            index = self.port_combo.findText(self.args.port)
            if index < 0:
                self.port_combo.insertItem(0, self.args.port)
                index = 0
            self.port_combo.setCurrentIndex(index)

        self.baudrate_spin = QSpinBox()
        self.baudrate_spin.setRange(9600, 4500000)
        self.baudrate_spin.setValue(self.args.baudrate)

        self.id_spin = QSpinBox()
        self.id_spin.setRange(0, 252)
        self.id_spin.setValue(self.args.dxl_id)

        self.period_spin = QDoubleSpinBox()
        self.period_spin.setRange(0.02, 5.0)
        self.period_spin.setSingleStep(0.02)
        self.period_spin.setSuffix(" s")
        self.period_spin.setValue(self.args.period)

        form.addRow("Port", self.port_combo)
        form.addRow("Baudrate", self.baudrate_spin)
        form.addRow("DXL ID", self.id_spin)
        form.addRow("Period", self.period_spin)
        controls.setLayout(form)
        layout.addWidget(controls)

        buttons = QGridLayout()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.refresh_button = QPushButton("Refresh Ports")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_reader)
        self.stop_button.clicked.connect(self.stop_reader)
        self.refresh_button.clicked.connect(self.refresh_ports)
        buttons.addWidget(self.start_button, 0, 0)
        buttons.addWidget(self.stop_button, 0, 1)
        buttons.addWidget(self.refresh_button, 0, 2)
        layout.addLayout(buttons)

        values = QGroupBox("Status")
        grid = QGridLayout()
        self.labels = {}
        rows = [
            ("model", "Model"),
            ("firmware", "Firmware"),
            ("torque_text", "Torque"),
            ("operating_mode_text", "Mode"),
            ("current_a", "Current A"),
            ("current_raw", "Current raw"),
            ("voltage_v", "Voltage V"),
            ("velocity_rad_s", "Velocity rad/s"),
            ("velocity_rpm", "Velocity RPM"),
            ("velocity_raw", "Velocity raw"),
            ("position_deg", "Position deg"),
            ("position_rad", "Position rad"),
            ("position_raw", "Position raw"),
            ("temperature_c", "Temperature C"),
            ("moving_text", "Moving"),
            ("hardware_error_text", "Hardware error"),
            ("errors", "Read status"),
            ("ok_count", "OK count"),
            ("fail_count", "Fail count"),
            ("age_ms", "Age ms"),
        ]
        for row, (key, name) in enumerate(rows):
            grid.addWidget(QLabel(name), row, 0)
            label = QLabel("--")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            label.setMinimumWidth(160)
            label.setMaximumWidth(360)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            grid.addWidget(label, row, 1)
            self.labels[key] = label
        values.setLayout(grid)
        layout.addWidget(values)

        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def refresh_ports(self):
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = [DYNAMIXEL_PORT]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def start_reader(self):
        if self.thread is not None:
            return

        self.thread = QThread(self)
        self.reader = DynamixelReader(
            port=self.port_combo.currentText(),
            baudrate=self.baudrate_spin.value(),
            dxl_id=self.id_spin.value(),
            period_s=self.period_spin.value(),
        )
        self.reader.moveToThread(self.thread)
        self.thread.started.connect(self.reader.run)
        self.reader.sample_received.connect(self.update_sample)
        self.reader.connection_changed.connect(self.on_connection_changed)
        self.reader.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.reader.deleteLater)
        self.thread.finished.connect(self._reader_finished)
        self.thread.start()

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.statusBar().showMessage("Opening Dynamixel port...")

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait(1500)

    def _reader_finished(self):
        self.thread = None
        self.reader = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def on_connection_changed(self, connected, message):
        self.statusBar().showMessage(self._short_text(message, 80))

    def update_sample(self, sample):
        self.last_timestamp = sample["timestamp"]
        sample = dict(sample)
        sample["torque_text"] = "ON" if sample["torque"] == 1 else "OFF"
        sample["moving_text"] = "YES" if sample["moving"] == 1 else "NO"

        self._set_label("model", self._fmt(sample["model"], "{}"))
        self._set_label("firmware", self._fmt(sample["firmware"], "{}"))
        self._set_label("torque_text", sample["torque_text"])
        self._set_label("operating_mode_text", sample["operating_mode_text"])
        self._set_label("current_a", self._fmt(sample["current_a"], "{:.4f}"))
        self._set_label("current_raw", self._fmt(sample["current_raw"], "{}"))
        self._set_label("voltage_v", self._fmt(sample["voltage_v"], "{:.2f}"))
        self._set_label("velocity_rad_s", self._fmt(sample["velocity_rad_s"], "{:.4f}"))
        self._set_label("velocity_rpm", self._fmt(sample["velocity_rpm"], "{:.2f}"))
        self._set_label("velocity_raw", self._fmt(sample["velocity_raw"], "{}"))
        self._set_label("position_deg", self._fmt(sample["position_deg"], "{:.2f}"))
        self._set_label("position_rad", self._fmt(sample["position_rad"], "{:.4f}"))
        self._set_label("position_raw", self._fmt(sample["position_raw"], "{}"))
        self._set_label("temperature_c", self._fmt(sample["temperature_c"], "{}"))
        self._set_label("moving_text", sample["moving_text"])
        self._set_label("hardware_error_text", sample["hardware_error_text"])
        self._set_label("errors", sample["errors"])
        self._set_label("ok_count", str(sample["ok_count"]))
        self._set_label("fail_count", str(sample["fail_count"]))
        self.refresh_age()

    def refresh_age(self):
        if self.last_timestamp is None:
            return
        age_ms = (time.time() - self.last_timestamp) * 1000.0
        self._set_label("age_ms", f"{age_ms:.0f}")

    def closeEvent(self, event):
        self.stop_reader()
        super().closeEvent(event)

    @staticmethod
    def _fmt(value, fmt):
        if value is None:
            return "--"
        return fmt.format(value)

    def _set_label(self, key, text):
        label = self.labels[key]
        display_text = self._elide_text(label, text)
        label.setText(display_text)
        label.setToolTip(text if display_text != text else "")

    @staticmethod
    def _short_text(text, max_len):
        if len(text) <= max_len:
            return text
        return text[: max_len - 3] + "..."

    @staticmethod
    def _elide_text(label, text):
        metrics = QFontMetrics(label.font())
        return metrics.elidedText(text, Qt.ElideMiddle, label.maximumWidth())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Dynamixel XH430 status monitor UI")
    parser.add_argument("--port", default=DYNAMIXEL_PORT)
    parser.add_argument("--baudrate", type=int, default=DYNAMIXEL_BAUDRATE)
    parser.add_argument("--dxl-id", type=int, default=1)
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--no-autostart", dest="autostart", action="store_false")
    parser.set_defaults(autostart=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv)
    window = DynamixelStatusWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
