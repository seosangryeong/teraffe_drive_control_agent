#!/usr/bin/env python3
"""PyQt monitor/control UI for AK80-64, Dynamixel XH430, and AMT212."""

import argparse
import os
import sys
import threading
import time
from collections import deque

import can
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFontMetrics
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
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

from teraffe_drive_control_agent.ak80_64_status_ui import StatusStore
from teraffe_drive_control_agent.config.ak80_64_config import (
    AK80_64_CAN_BITRATE,
    AK80_64_ERPM_MAX,
    AK80_64_ERPM_MIN,
    CAN_PACKET_SET_RPM,
)
from teraffe_drive_control_agent.config.encoder_config import AMT212_BAUDRATE, AMT212_PORT
from teraffe_drive_control_agent.config.teraffe_config import (
    AK80_64_IDS,
    DYNAMIXEL_BAUDRATE,
    DYNAMIXEL_PORT,
)
from teraffe_drive_control_agent.dxl_encoder_status_ui import CombinedReader


class AkReader(QObject):
    status_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    command_finished = pyqtSignal(bool, str)
    finished = pyqtSignal()

    def __init__(self, channel, bitrate, motor_id, period_s):
        super().__init__()
        self.channel = channel
        self.bitrate = bitrate
        self.motor_id = motor_id
        self.period_s = period_s
        self.running = False
        self.bus = None
        self.store = StatusStore()
        self.commands = deque()
        self.commands_lock = threading.Lock()
        self.target_erpm = None
        self.last_erpm_send = 0.0
        self.erpm_command_period = 0.1

    def run(self):
        self.running = True
        try:
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                raise RuntimeError(f"{self.channel} does not exist")
            try:
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
        try:
            while self.running:
                self.process_commands()
                self.resend_target_erpm()
                msg = self.bus.recv(timeout=min(0.05, self.period_s))
                if msg is None:
                    continue
                status = self.store.update_from_message(msg, selected_id=self.motor_id)
                if status is not None:
                    self.status_received.emit(status.__dict__.copy())
        except Exception as exc:
            self.connection_changed.emit(False, f"CAN read failed: {exc}")
        finally:
            if self.bus is not None:
                self.bus.shutdown()
                self.bus = None
            self.connection_changed.emit(False, "CAN disconnected")
            self.finished.emit()

    def stop(self):
        self.target_erpm = None
        if self.bus is not None:
            try:
                self.send_erpm(0)
            except Exception:
                pass
        self.running = False

    def enqueue_command(self, command, value=None):
        with self.commands_lock:
            self.commands.append((command, value))

    def process_commands(self):
        while True:
            with self.commands_lock:
                if not self.commands:
                    return
                command, value = self.commands.popleft()
            try:
                if command == "set_erpm":
                    erpm = int(max(AK80_64_ERPM_MIN, min(AK80_64_ERPM_MAX, value)))
                    self.target_erpm = erpm
                    self.send_erpm(erpm)
                    self.command_finished.emit(True, f"AK80-64 holding {erpm} ERPM")
                elif command == "stop":
                    self.target_erpm = None
                    self.send_erpm(0)
                    self.command_finished.emit(True, "AK80-64 target 0 ERPM")
            except Exception as exc:
                self.command_finished.emit(False, f"AK command failed: {exc}")

    def resend_target_erpm(self):
        if self.target_erpm is None:
            return
        now = time.monotonic()
        if now - self.last_erpm_send < self.erpm_command_period:
            return
        self.send_erpm(self.target_erpm)

    def send_erpm(self, erpm):
        value = int(erpm)
        if value < 0:
            value += 2**32
        data = [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
        msg = can.Message(
            arbitration_id=self.motor_id | (CAN_PACKET_SET_RPM << 8),
            data=data,
            is_extended_id=True,
        )
        self.bus.send(msg, timeout=0.02)
        self.last_erpm_send = time.monotonic()


class RobotHardwareWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.serial_thread = None
        self.serial_reader = None
        self.ak_thread = None
        self.ak_reader = None
        self.labels = {}
        self.serial_ok_count = 0
        self.serial_fail_count = 0
        self.last_serial_timestamp = None
        self.last_ak_timestamp = None

        self.setWindowTitle("Robot Hardware Status + Control")
        self.build_ui()
        self.setMinimumSize(760, 560)
        self.setMaximumSize(1180, 820)
        self.resize(980, 680)

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.refresh_ages)
        self.age_timer.start(100)

        if args.autostart:
            self.start_readers()

    def build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        controls = QGroupBox("Connections")
        form = QFormLayout()
        self.serial_port_combo = QComboBox()
        self.refresh_ports()
        if self.args.serial_port:
            index = self.serial_port_combo.findText(self.args.serial_port)
            if index < 0:
                self.serial_port_combo.insertItem(0, self.args.serial_port)
                index = 0
            self.serial_port_combo.setCurrentIndex(index)
        self.dxl_baudrate_spin = self.make_spin(9600, 4500000, self.args.dxl_baudrate, 140)
        self.encoder_baudrate_spin = self.make_spin(9600, 3000000, self.args.encoder_baudrate, 140)
        self.dxl_id_spin = self.make_spin(0, 252, self.args.dxl_id, 90)
        self.encoder_id_spin = self.make_spin(0, 255, self.args.encoder_id, 90)
        self.can_channel_combo = QComboBox()
        self.can_channel_combo.addItems([self.args.can_channel, "can0", "can1"])
        self.can_channel_combo.setCurrentText(self.args.can_channel)
        self.can_bitrate_spin = self.make_spin(10000, 5000000, self.args.can_bitrate, 140)
        self.ak_id_spin = self.make_spin(0, 255, self.args.ak_id, 90)
        self.period_spin = QDoubleSpinBox()
        self.period_spin.setRange(0.05, 5.0)
        self.period_spin.setSingleStep(0.05)
        self.period_spin.setSuffix(" s")
        self.period_spin.setValue(self.args.period)
        self.period_spin.setMaximumWidth(90)
        form.addRow("Serial port", self.serial_port_combo)
        form.addRow("DXL baud", self.dxl_baudrate_spin)
        form.addRow("Encoder baud", self.encoder_baudrate_spin)
        form.addRow("DXL ID", self.dxl_id_spin)
        form.addRow("Encoder ID", self.encoder_id_spin)
        form.addRow("CAN channel", self.can_channel_combo)
        form.addRow("CAN bitrate", self.can_bitrate_spin)
        form.addRow("AK80 ID", self.ak_id_spin)
        form.addRow("Period", self.period_spin)
        controls.setLayout(form)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop Monitor")
        self.refresh_button = QPushButton("Refresh Ports")
        self.stop_button.setEnabled(False)
        for button in (self.start_button, self.stop_button, self.refresh_button):
            button.setMaximumWidth(130)
        self.start_button.clicked.connect(self.start_readers)
        self.stop_button.clicked.connect(self.stop_readers)
        self.refresh_button.clicked.connect(self.refresh_ports)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.refresh_button)
        buttons.addStretch()

        top = QHBoxLayout()
        top.addWidget(controls, 1)
        top.addLayout(self.make_dxl_control(), 1)
        top.addLayout(self.make_ak_control(), 1)
        layout.addLayout(top)
        layout.addLayout(buttons)

        status_row = QHBoxLayout()
        status_row.addWidget(self.make_value_group("CubeMars AK80-64", [
            ("ak_mode", "Mode"),
            ("ak_target", "Target ERPM"),
            ("ak_pos_deg", "Position deg"),
            ("ak_pos_rad", "Position rad"),
            ("ak_erpm", "Speed ERPM"),
            ("ak_out_rpm", "Output RPM"),
            ("ak_rad_s", "Speed rad/s"),
            ("ak_current", "Current A"),
            ("ak_motor_temp", "Motor C"),
            ("ak_error", "Error"),
            ("ak_packet", "Packet"),
            ("ak_age", "Age ms"),
        ]), 1)
        status_row.addWidget(self.make_value_group("Dynamixel XH430", [
            ("dxl_model", "Model"),
            ("dxl_torque", "Torque"),
            ("dxl_mode", "Mode"),
            ("dxl_current", "Current A"),
            ("dxl_voltage", "Voltage V"),
            ("dxl_vel_rad_s", "Velocity rad/s"),
            ("dxl_vel_rpm", "Velocity RPM"),
            ("dxl_pos_deg", "Position deg"),
            ("dxl_pos_rad", "Position rad"),
            ("dxl_temp", "Temperature C"),
            ("dxl_moving", "Moving"),
            ("dxl_error", "Hardware error"),
            ("dxl_status", "Read status"),
        ]), 1)
        status_row.addWidget(self.make_value_group("AMT212 Encoder", [
            ("enc_angle_deg", "Angle deg"),
            ("enc_angle_rad", "Angle rad"),
            ("enc_count", "Raw count"),
            ("enc_raw16", "Raw16"),
            ("enc_raw", "Raw bytes"),
            ("enc_status", "Read status"),
            ("serial_ok", "Serial OK"),
            ("serial_fail", "Serial Fail"),
            ("serial_age", "Serial age"),
        ]), 1)
        layout.addLayout(status_row)

        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def make_dxl_control(self):
        layout = QVBoxLayout()
        group = QGroupBox("DXL Current Control")
        grid = QGridLayout()
        self.goal_current_spin = QDoubleSpinBox()
        self.goal_current_spin.setRange(-2.0, 2.0)
        self.goal_current_spin.setDecimals(4)
        self.goal_current_spin.setSingleStep(0.01)
        self.goal_current_spin.setSuffix(" A")
        self.goal_current_spin.setValue(0.05)
        self.goal_current_spin.setMaximumWidth(120)
        self.dxl_current_mode_button = QPushButton("Current Mode")
        self.dxl_torque_on_button = QPushButton("Torque On")
        self.dxl_torque_off_button = QPushButton("Torque Off")
        self.dxl_apply_button = QPushButton("Apply Current")
        self.dxl_stop_button = QPushButton("Stop Current")
        self.dxl_current_mode_button.clicked.connect(lambda: self.enqueue_serial("current_mode"))
        self.dxl_torque_on_button.clicked.connect(lambda: self.enqueue_serial("torque_on"))
        self.dxl_torque_off_button.clicked.connect(lambda: self.enqueue_serial("torque_off"))
        self.dxl_apply_button.clicked.connect(
            lambda: self.enqueue_serial("set_current", self.goal_current_spin.value())
        )
        self.dxl_stop_button.clicked.connect(lambda: self.enqueue_serial("stop_current"))
        grid.addWidget(QLabel("Goal current"), 0, 0)
        grid.addWidget(self.goal_current_spin, 0, 1)
        grid.addWidget(self.dxl_current_mode_button, 1, 0)
        grid.addWidget(self.dxl_torque_on_button, 1, 1)
        grid.addWidget(self.dxl_torque_off_button, 1, 2)
        grid.addWidget(self.dxl_apply_button, 2, 0)
        grid.addWidget(self.dxl_stop_button, 2, 1)
        group.setLayout(grid)
        layout.addWidget(group)
        layout.addStretch()
        return layout

    def make_ak_control(self):
        layout = QVBoxLayout()
        group = QGroupBox("AK80-64 ERPM Control")
        grid = QGridLayout()
        self.goal_erpm_spin = QSpinBox()
        self.goal_erpm_spin.setRange(AK80_64_ERPM_MIN, AK80_64_ERPM_MAX)
        self.goal_erpm_spin.setSingleStep(100)
        self.goal_erpm_spin.setValue(-5000)
        self.goal_erpm_spin.setMaximumWidth(120)
        self.ak_apply_button = QPushButton("Apply ERPM")
        self.ak_stop_button = QPushButton("Stop ERPM")
        self.ak_apply_button.clicked.connect(
            lambda: self.enqueue_ak("set_erpm", self.goal_erpm_spin.value())
        )
        self.ak_stop_button.clicked.connect(lambda: self.enqueue_ak("stop"))
        grid.addWidget(QLabel("Goal ERPM"), 0, 0)
        grid.addWidget(self.goal_erpm_spin, 0, 1)
        grid.addWidget(self.ak_apply_button, 1, 0)
        grid.addWidget(self.ak_stop_button, 1, 1)
        group.setLayout(grid)
        layout.addWidget(group)
        layout.addStretch()
        return layout

    def make_value_group(self, title, rows):
        group = QGroupBox(title)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(3)
        grid.setContentsMargins(8, 8, 8, 8)
        for row, (key, name) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setMinimumWidth(92)
            name_label.setMaximumWidth(118)
            grid.addWidget(name_label, row, 0)
            label = QLabel("--")
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setMinimumWidth(88)
            label.setMaximumWidth(160)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            grid.addWidget(label, row, 1)
            self.labels[key] = label
        group.setLayout(grid)
        return group

    def start_readers(self):
        if self.serial_thread is not None or self.ak_thread is not None:
            return
        self.serial_ok_count = 0
        self.serial_fail_count = 0
        self.start_serial_reader()
        self.start_ak_reader()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def start_serial_reader(self):
        self.serial_thread = QThread(self)
        self.serial_reader = CombinedReader(
            port_name=self.serial_port_combo.currentText(),
            dxl_baudrate=self.dxl_baudrate_spin.value(),
            encoder_baudrate=self.encoder_baudrate_spin.value(),
            dxl_id=self.dxl_id_spin.value(),
            encoder_id=self.encoder_id_spin.value(),
            period_s=self.period_spin.value(),
        )
        self.serial_reader.moveToThread(self.serial_thread)
        self.serial_thread.started.connect(self.serial_reader.run)
        self.serial_reader.sample_received.connect(self.update_serial_sample)
        self.serial_reader.connection_changed.connect(
            lambda connected, message: self.show_status(f"Serial: {message}")
        )
        self.serial_reader.command_finished.connect(self.on_command_finished)
        self.serial_reader.finished.connect(self.serial_thread.quit)
        self.serial_thread.finished.connect(self.serial_reader.deleteLater)
        self.serial_thread.finished.connect(self.serial_finished)
        self.serial_thread.start()

    def start_ak_reader(self):
        self.ak_thread = QThread(self)
        self.ak_reader = AkReader(
            channel=self.can_channel_combo.currentText(),
            bitrate=self.can_bitrate_spin.value(),
            motor_id=self.ak_id_spin.value(),
            period_s=self.period_spin.value(),
        )
        self.ak_reader.moveToThread(self.ak_thread)
        self.ak_thread.started.connect(self.ak_reader.run)
        self.ak_reader.status_received.connect(self.update_ak_status)
        self.ak_reader.connection_changed.connect(
            lambda connected, message: self.show_status(f"CAN: {message}")
        )
        self.ak_reader.command_finished.connect(self.on_command_finished)
        self.ak_reader.finished.connect(self.ak_thread.quit)
        self.ak_thread.finished.connect(self.ak_reader.deleteLater)
        self.ak_thread.finished.connect(self.ak_finished)
        self.ak_thread.start()

    def stop_readers(self):
        if self.serial_reader is not None:
            self.serial_reader.stop()
        if self.ak_reader is not None:
            self.ak_reader.stop()
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait(1500)
        if self.ak_thread is not None:
            self.ak_thread.quit()
            self.ak_thread.wait(1500)

    def serial_finished(self):
        self.serial_thread = None
        self.serial_reader = None
        self.update_start_stop_buttons()

    def ak_finished(self):
        self.ak_thread = None
        self.ak_reader = None
        self.update_start_stop_buttons()

    def update_start_stop_buttons(self):
        running = self.serial_thread is not None or self.ak_thread is not None
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)

    def enqueue_serial(self, command, value=None):
        if self.serial_reader is None:
            self.show_status("Start monitor before sending DXL command")
            return
        self.serial_reader.enqueue_command(command, value)

    def enqueue_ak(self, command, value=None):
        if self.ak_reader is None:
            self.show_status("Start monitor before sending AK command")
            return
        self.ak_reader.enqueue_command(command, value)
        if command == "set_erpm":
            self.set_label("ak_target", str(value))
        elif command == "stop":
            self.set_label("ak_target", "0")

    def update_serial_sample(self, sample):
        self.last_serial_timestamp = sample["timestamp"]
        dxl = sample["dxl"]
        enc = sample["encoder"]
        ok = dxl["ok"] and enc["ok"]
        self.serial_ok_count += 1 if ok else 0
        self.serial_fail_count += 0 if ok else 1
        self.set_label("dxl_model", self.fmt(dxl["model"], "{}"))
        self.set_label("dxl_torque", "ON" if dxl["torque"] == 1 else "OFF")
        self.set_label("dxl_mode", dxl["mode"])
        self.set_label("dxl_current", self.fmt(dxl["current_a"], "{:.4f}"))
        self.set_label("dxl_voltage", self.fmt(dxl["voltage_v"], "{:.2f}"))
        self.set_label("dxl_vel_rad_s", self.fmt(dxl["velocity_rad_s"], "{:.4f}"))
        self.set_label("dxl_vel_rpm", self.fmt(dxl["velocity_rpm"], "{:.2f}"))
        self.set_label("dxl_pos_deg", self.fmt(dxl["position_deg"], "{:.2f}"))
        self.set_label("dxl_pos_rad", self.fmt(dxl["position_rad"], "{:.4f}"))
        self.set_label("dxl_temp", self.fmt(dxl["temperature_c"], "{}"))
        self.set_label("dxl_moving", "YES" if dxl["moving"] == 1 else "NO")
        self.set_label("dxl_error", dxl["hardware_error"])
        self.set_label("dxl_status", dxl["errors"])
        self.set_label("enc_angle_deg", self.fmt(enc["angle_deg"], "{:.3f}"))
        self.set_label("enc_angle_rad", self.fmt(enc["angle_rad"], "{:.5f}"))
        self.set_label("enc_count", self.fmt(enc["count"], "{}"))
        self.set_label("enc_raw16", enc["raw16"])
        self.set_label("enc_raw", enc["raw"] or "--")
        self.set_label("enc_status", enc["error"])
        self.set_label("serial_ok", str(self.serial_ok_count))
        self.set_label("serial_fail", str(self.serial_fail_count))
        self.refresh_ages()

    def update_ak_status(self, status):
        self.last_ak_timestamp = status["last_seen"]
        self.set_label("ak_mode", status["mode"])
        self.set_label("ak_pos_deg", self.fmt(status["position_deg"], "{:.2f}"))
        self.set_label("ak_pos_rad", self.fmt(status["position_rad"], "{:.4f}"))
        self.set_label("ak_erpm", self.fmt(status["erpm"], "{:.0f}"))
        self.set_label("ak_out_rpm", self.fmt(status["output_rpm"], "{:.2f}"))
        self.set_label("ak_rad_s", self.fmt(status["speed_rad_s"], "{:.3f}"))
        self.set_label("ak_current", self.fmt(status["current_a"], "{:.2f}"))
        self.set_label("ak_motor_temp", self.fmt(status["motor_temp_c"], "{:.1f}"))
        self.set_label("ak_error", status["error_text"])
        self.set_label("ak_packet", status["last_packet"])
        self.refresh_ages()

    def refresh_ages(self):
        if self.last_serial_timestamp is not None:
            self.set_label("serial_age", f"{(time.time() - self.last_serial_timestamp) * 1000.0:.0f}")
        if self.last_ak_timestamp is not None:
            self.set_label("ak_age", f"{(time.time() - self.last_ak_timestamp) * 1000.0:.0f}")

    def on_command_finished(self, ok, message):
        self.show_status(("OK: " if ok else "ERR: ") + message)

    def show_status(self, message):
        self.statusBar().showMessage(self.short_text(message, 120))

    def refresh_ports(self):
        current = self.serial_port_combo.currentText() if hasattr(self, "serial_port_combo") else ""
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = [DYNAMIXEL_PORT or AMT212_PORT]
        self.serial_port_combo.clear()
        self.serial_port_combo.addItems(ports)
        if current:
            self.serial_port_combo.setCurrentText(current)

    def closeEvent(self, event):
        self.stop_readers()
        super().closeEvent(event)

    def set_label(self, key, text):
        label = self.labels[key]
        display_text = self.elide_text(label, text)
        label.setText(display_text)
        label.setToolTip(text if display_text != text else "")

    @staticmethod
    def make_spin(minimum, maximum, value, width):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setMaximumWidth(width)
        return spin

    @staticmethod
    def fmt(value, fmt):
        if value is None:
            return "--"
        return fmt.format(value)

    @staticmethod
    def fmt_missing(value, fmt, mode):
        if value is None and mode == "Servo":
            return "N/A"
        if value is None:
            return "--"
        return fmt.format(value)

    @staticmethod
    def short_text(text, max_len):
        if len(text) <= max_len:
            return text
        return text[: max_len - 3] + "..."

    @staticmethod
    def elide_text(label, text):
        metrics = QFontMetrics(label.font())
        return metrics.elidedText(text, Qt.ElideMiddle, label.maximumWidth())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="AK80-64 + Dynamixel + encoder UI")
    parser.add_argument("--serial-port", default=DYNAMIXEL_PORT or AMT212_PORT)
    parser.add_argument("--dxl-baudrate", type=int, default=DYNAMIXEL_BAUDRATE)
    parser.add_argument("--encoder-baudrate", type=int, default=AMT212_BAUDRATE)
    parser.add_argument("--dxl-id", type=int, default=102)
    parser.add_argument("--encoder-id", type=lambda value: int(value, 0), default=0x2C)
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--can-bitrate", type=int, default=AK80_64_CAN_BITRATE)
    parser.add_argument("--ak-id", type=int, default=AK80_64_IDS.get("front_left_drive", 101))
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--no-autostart", dest="autostart", action="store_false")
    parser.set_defaults(autostart=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv)
    window = RobotHardwareWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
