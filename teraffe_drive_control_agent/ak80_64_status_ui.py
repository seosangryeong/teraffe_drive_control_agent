#!/usr/bin/env python3
"""PyQt status monitor for CubeMars AK80-64 CAN feedback frames."""

import argparse
import math
import os
import struct
import sys
import time
from dataclasses import dataclass, field

import can
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from teraffe_drive_control_agent.config.ak80_64_config import (
    AK80_64_ERPM_TO_OUTPUT_RPM,
    AK80_64_POLE_PAIRS,
)


CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27

PACKET_NAMES = {
    CAN_PACKET_STATUS: "VESC STATUS",
    CAN_PACKET_STATUS_2: "VESC STATUS 2",
    CAN_PACKET_STATUS_3: "VESC STATUS 3",
    CAN_PACKET_STATUS_4: "VESC STATUS 4",
    CAN_PACKET_STATUS_5: "VESC STATUS 5",
}

SERVO_ERROR_TEXT = {
    0: "OK",
    1: "Motor over-temp",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temp",
    7: "Motor stall",
}

MIT_ERROR_TEXT = {
    0: "OK",
    1: "Over-voltage",
    2: "Under-voltage",
    3: "Driver fault",
    4: "Over-current",
    5: "MOSFET over-temp",
    6: "Motor over-temp",
    7: "Gate driver over-voltage",
    8: "Gate driver under-voltage",
    9: "MCU under-voltage",
    10: "Watchdog reset",
    11: "Encoder SPI",
    12: "Encoder sincos low",
    13: "Encoder sincos high",
    14: "Flash corruption",
    15: "Current sensor 1",
    16: "Current sensor 2",
    17: "Current sensor 3",
    18: "Unbalanced currents",
}

# AK80-64 MIT feedback ranges from the AK series driver manual.
MIT_P_MIN = -50.0
MIT_P_MAX = 50.0
MIT_V_MIN = -8.0
MIT_V_MAX = 8.0
MIT_I_MIN = -144.0
MIT_I_MAX = 144.0


@dataclass
class MotorStatus:
    motor_id: int
    mode: str = "--"
    position_deg: float | None = None
    position_rad: float | None = None
    erpm: float | None = None
    output_rpm: float | None = None
    speed_rad_s: float | None = None
    current_a: float | None = None
    input_current_a: float | None = None
    voltage_v: float | None = None
    duty: float | None = None
    mos_temp_c: float | None = None
    motor_temp_c: float | None = None
    tacho: int | None = None
    error_code: int | None = None
    error_text: str = "--"
    frame_count: int = 0
    last_packet: str = "--"
    last_can_id: str = "--"
    last_data: str = "--"
    last_seen: float = field(default_factory=time.time)


def be_i16(data, offset):
    return struct.unpack_from(">h", data, offset)[0]


def be_i32(data, offset):
    return struct.unpack_from(">i", data, offset)[0]


def uint_to_float(value, minimum, maximum, bits):
    span = maximum - minimum
    return float(value) * span / float((1 << bits) - 1) + minimum


def split_extended_id(arbitration_id):
    return arbitration_id & 0xFF, arbitration_id >> 8


def maybe_output_rad_s(erpm):
    if erpm is None:
        return None
    output_rpm = erpm * AK80_64_ERPM_TO_OUTPUT_RPM
    return output_rpm * 2.0 * math.pi / 60.0


class StatusStore:
    def __init__(self):
        self.status_by_id = {}

    def update_from_message(self, msg, selected_id=None):
        data = bytes(msg.data)
        if len(data) < 6:
            return None

        if msg.is_extended_id:
            decoded = self._decode_extended(msg.arbitration_id, data, selected_id)
        else:
            decoded = self._decode_standard(msg.arbitration_id, data, selected_id)

        if decoded is None:
            return None

        motor_id, packet_name, updates = decoded
        status = self.status_by_id.setdefault(motor_id, MotorStatus(motor_id=motor_id))
        for key, value in updates.items():
            setattr(status, key, value)
        status.frame_count += 1
        status.last_packet = packet_name
        status.last_can_id = f"0x{msg.arbitration_id:08X}" if msg.is_extended_id else f"0x{msg.arbitration_id:03X}"
        status.last_data = data.hex(" ").upper()
        status.last_seen = time.time()
        return status

    def _decode_extended(self, arbitration_id, data, selected_id):
        motor_id, packet_id = split_extended_id(arbitration_id)
        if selected_id is not None and motor_id != selected_id:
            return None

        if packet_id == CAN_PACKET_STATUS and len(data) >= 8:
            erpm = float(be_i32(data, 0))
            return motor_id, PACKET_NAMES[packet_id], {
                "mode": "VESC",
                "erpm": erpm,
                "output_rpm": erpm * AK80_64_ERPM_TO_OUTPUT_RPM,
                "speed_rad_s": maybe_output_rad_s(erpm),
                "current_a": be_i16(data, 4) / 10.0,
                "duty": be_i16(data, 6) / 1000.0,
            }

        if packet_id == CAN_PACKET_STATUS_4 and len(data) >= 8:
            position_deg = be_i16(data, 6) / 50.0
            return motor_id, PACKET_NAMES[packet_id], {
                "mode": "VESC",
                "mos_temp_c": be_i16(data, 0) / 10.0,
                "motor_temp_c": be_i16(data, 2) / 10.0,
                "input_current_a": be_i16(data, 4) / 10.0,
                "position_deg": position_deg,
                "position_rad": math.radians(position_deg),
            }

        if packet_id == CAN_PACKET_STATUS_5 and len(data) >= 6:
            return motor_id, PACKET_NAMES[packet_id], {
                "mode": "VESC",
                "tacho": be_i32(data, 0),
                "voltage_v": be_i16(data, 4) / 10.0,
            }

        if packet_id in PACKET_NAMES:
            return motor_id, PACKET_NAMES[packet_id], {"mode": "VESC"}

        if len(data) >= 8:
            return self._decode_servo_upload(motor_id, data, "Servo upload")

        return None

    def _decode_standard(self, arbitration_id, data, selected_id):
        can_id = arbitration_id & 0xFF
        if len(data) >= 8 and data[0] == can_id:
            motor_id = data[0]
            if selected_id is not None and motor_id != selected_id:
                return None
            return self._decode_mit_reply(motor_id, data)

        motor_id = can_id
        if selected_id is not None and motor_id != selected_id:
            return None
        if len(data) >= 8:
            return self._decode_servo_upload(motor_id, data, "Servo upload")
        return None

    def _decode_servo_upload(self, motor_id, data, packet_name):
        position_deg = be_i16(data, 0) * 0.1
        erpm = float(be_i16(data, 2) * 10.0)
        error_code = data[7]
        return motor_id, packet_name, {
            "mode": "Servo",
            "position_deg": position_deg,
            "position_rad": math.radians(position_deg),
            "erpm": erpm,
            "output_rpm": erpm * AK80_64_ERPM_TO_OUTPUT_RPM,
            "speed_rad_s": maybe_output_rad_s(erpm),
            "current_a": be_i16(data, 4) * 0.01,
            "voltage_v": None,
            "motor_temp_c": float(data[6]),
            "error_code": error_code,
            "error_text": SERVO_ERROR_TEXT.get(error_code, f"Fault {error_code}"),
        }

    def _decode_mit_reply(self, motor_id, data):
        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        i_int = ((data[4] & 0x0F) << 8) | data[5]
        position_rad = uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16)
        speed_rad_s = uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12)
        current_a = uint_to_float(i_int, MIT_I_MIN, MIT_I_MAX, 12)
        temp_c = float(data[6] - 40)
        error_code = data[7]
        return motor_id, "MIT reply", {
            "mode": "MIT",
            "position_rad": position_rad,
            "position_deg": math.degrees(position_rad),
            "speed_rad_s": speed_rad_s,
            "output_rpm": speed_rad_s * 60.0 / (2.0 * math.pi),
            "erpm": speed_rad_s * 60.0 / (2.0 * math.pi) * AK80_64_POLE_PAIRS,
            "current_a": current_a,
            "motor_temp_c": temp_c,
            "error_code": error_code,
            "error_text": MIT_ERROR_TEXT.get(error_code, f"Fault {error_code}"),
        }


class CanReader(QObject):
    status_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    finished = pyqtSignal()

    def __init__(self, channel, bitrate, motor_id, show_all, query_mit, query_period):
        super().__init__()
        self.channel = channel
        self.bitrate = bitrate
        self.motor_id = motor_id
        self.show_all = show_all
        self.query_mit = query_mit
        self.query_period = query_period
        self.running = False
        self.store = StatusStore()
        self.bus = None
        self.last_query = 0.0

    def run(self):
        self.running = True
        try:
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                raise RuntimeError(
                    f"{self.channel} does not exist. Bring up SocketCAN first."
                )
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface="socketcan",
                bitrate=self.bitrate,
            )
        except TypeError:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype="socketcan",
                bitrate=self.bitrate,
            )
        except Exception as exc:
            self.connection_changed.emit(False, f"CAN open failed: {exc}")
            self.finished.emit()
            return

        self.connection_changed.emit(True, f"Connected to {self.channel}")
        selected_id = None if self.show_all else self.motor_id

        try:
            while self.running:
                now = time.monotonic()
                if self.query_mit and now - self.last_query >= self.query_period:
                    self._send_mit_state_query()
                    self.last_query = now

                msg = self.bus.recv(timeout=0.05)
                if msg is None:
                    continue

                status = self.store.update_from_message(msg, selected_id=selected_id)
                if status is not None:
                    self.status_received.emit(status.__dict__.copy())
        except Exception as exc:
            self.connection_changed.emit(False, f"CAN read failed: {exc}")
        finally:
            if self.bus is not None:
                self.bus.shutdown()
                self.bus = None
            self.connection_changed.emit(False, "Disconnected")
            self.finished.emit()

    def stop(self):
        self.running = False

    def _send_mit_state_query(self):
        if self.bus is None:
            return
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            is_extended_id=False,
        )
        self.bus.send(msg, timeout=0.02)


class StatusCell(QTableWidgetItem):
    def __init__(self, text="--"):
        super().__init__(text)
        self.setTextAlignment(Qt.AlignCenter)


class Ak80StatusWindow(QMainWindow):
    COLUMNS = [
        "ID",
        "Mode",
        "Position deg",
        "Position rad",
        "Speed ERPM",
        "Output RPM",
        "Speed rad/s",
        "Current A",
        "Input A",
        "Voltage V",
        "Duty",
        "Motor C",
        "MOS C",
        "Error",
        "Packet",
        "CAN ID",
        "Raw data",
        "Age ms",
        "Frames",
    ]

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.thread = None
        self.reader = None
        self.rows_by_id = {}
        self.latest_status = {}

        self.setWindowTitle("AK80-64 Status Monitor")
        self.resize(1180, 520)
        self._build_ui()

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.refresh_ages)
        self.age_timer.start(100)

        if args.autostart:
            self.start_reader()

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        controls = QGroupBox("CAN")
        form = QFormLayout()

        self.channel_edit = QLineEdit(self.args.channel)
        self.bitrate_spin = QSpinBox()
        self.bitrate_spin.setRange(10000, 5000000)
        self.bitrate_spin.setSingleStep(100000)
        self.bitrate_spin.setValue(self.args.bitrate)
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(0, 255)
        self.motor_id_spin.setValue(self.args.motor_id)

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Selected motor only", False)
        self.mode_combo.addItem("Show all motors", True)
        self.mode_combo.setCurrentIndex(1 if self.args.show_all else 0)

        self.query_mit_check = QCheckBox("MIT state query")
        self.query_mit_check.setChecked(self.args.query_mit)
        self.query_period_spin = QDoubleSpinBox()
        self.query_period_spin.setRange(0.02, 5.0)
        self.query_period_spin.setSingleStep(0.05)
        self.query_period_spin.setSuffix(" s")
        self.query_period_spin.setValue(self.args.query_period)

        form.addRow("Channel", self.channel_edit)
        form.addRow("Bitrate", self.bitrate_spin)
        form.addRow("Motor ID", self.motor_id_spin)
        form.addRow("Filter", self.mode_combo)
        form.addRow(self.query_mit_check, self.query_period_spin)
        controls.setLayout(form)

        button_bar = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.clear_button = QPushButton("Clear")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_reader)
        self.stop_button.clicked.connect(self.stop_reader)
        self.clear_button.clicked.connect(self.clear_table)
        button_bar.addWidget(self.start_button)
        button_bar.addWidget(self.stop_button)
        button_bar.addWidget(self.clear_button)
        button_bar.addStretch()

        top = QGridLayout()
        top.addWidget(controls, 0, 0)
        top.addLayout(button_bar, 1, 0)
        layout.addLayout(top)

        self.table = QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        layout.addWidget(self.table)

        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def start_reader(self):
        if self.thread is not None:
            return

        self.clear_table()
        self.thread = QThread(self)
        self.reader = CanReader(
            channel=self.channel_edit.text().strip() or "can0",
            bitrate=self.bitrate_spin.value(),
            motor_id=self.motor_id_spin.value(),
            show_all=self.mode_combo.currentData(),
            query_mit=self.query_mit_check.isChecked(),
            query_period=self.query_period_spin.value(),
        )
        self.reader.moveToThread(self.thread)
        self.thread.started.connect(self.reader.run)
        self.reader.status_received.connect(self.update_status)
        self.reader.connection_changed.connect(self.on_connection_changed)
        self.reader.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.reader.deleteLater)
        self.thread.finished.connect(self._reader_finished)
        self.thread.start()

        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.statusBar().showMessage("Opening CAN...")

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

    def clear_table(self):
        self.rows_by_id.clear()
        self.latest_status.clear()
        self.table.setRowCount(0)

    def on_connection_changed(self, connected, message):
        self.statusBar().showMessage(message)
        if not connected and self.thread is not None and not self.thread.isRunning():
            self._reader_finished()

    def update_status(self, status):
        motor_id = status["motor_id"]
        self.latest_status[motor_id] = status
        row = self.rows_by_id.get(motor_id)
        if row is None:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.rows_by_id[motor_id] = row
            for column in range(len(self.COLUMNS)):
                self.table.setItem(row, column, StatusCell())

        values = [
            str(motor_id),
            status["mode"],
            self._fmt(status["position_deg"], "{:.2f}"),
            self._fmt(status["position_rad"], "{:.4f}"),
            self._fmt(status["erpm"], "{:.0f}"),
            self._fmt(status["output_rpm"], "{:.2f}"),
            self._fmt(status["speed_rad_s"], "{:.3f}"),
            self._fmt(status["current_a"], "{:.2f}"),
            self._fmt_missing(status["input_current_a"], "{:.2f}", status["mode"]),
            self._fmt_missing(status["voltage_v"], "{:.2f}", status["mode"]),
            self._fmt(status["duty"], "{:.3f}"),
            self._fmt(status["motor_temp_c"], "{:.1f}"),
            self._fmt(status["mos_temp_c"], "{:.1f}"),
            status["error_text"],
            status["last_packet"],
            status["last_can_id"],
            status["last_data"],
            self._age_ms(status),
            str(status["frame_count"]),
        ]

        for column, value in enumerate(values):
            self.table.item(row, column).setText(value)

        error_code = status.get("error_code")
        for column in range(len(self.COLUMNS)):
            item = self.table.item(row, column)
            if error_code is None or error_code == 0:
                item.setBackground(Qt.white)
            else:
                item.setBackground(Qt.yellow)

        if status["mode"] == "Servo" and status["voltage_v"] is None:
            self.statusBar().showMessage(
                "Servo upload frame has no voltage field. Use VESC STATUS_5 or UART values for Voltage V."
            )

        self.table.resizeColumnsToContents()

    def refresh_ages(self):
        for motor_id, status in self.latest_status.items():
            row = self.rows_by_id.get(motor_id)
            if row is not None:
                self.table.item(row, 17).setText(self._age_ms(status))

    def closeEvent(self, event):
        self.stop_reader()
        super().closeEvent(event)

    def _age_ms(self, status):
        return f"{(time.time() - status['last_seen']) * 1000.0:.0f}"

    @staticmethod
    def _fmt(value, fmt):
        if value is None:
            return "--"
        return fmt.format(value)

    @staticmethod
    def _fmt_missing(value, fmt, mode):
        if value is None and mode == "Servo":
            return "N/A"
        if value is None:
            return "--"
        return fmt.format(value)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="AK80-64 CAN status monitor UI")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate")
    parser.add_argument("--motor-id", type=int, default=1, help="Motor ID to monitor")
    parser.add_argument("--show-all", action="store_true", help="Show every decoded motor ID")
    parser.add_argument(
        "--query-mit",
        action="store_true",
        help="Periodically send the MIT stateless state query frame",
    )
    parser.add_argument(
        "--query-period",
        type=float,
        default=0.1,
        help="MIT query period in seconds when --query-mit is enabled",
    )
    parser.add_argument("--no-autostart", dest="autostart", action="store_false")
    parser.set_defaults(autostart=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv)
    window = Ak80StatusWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
