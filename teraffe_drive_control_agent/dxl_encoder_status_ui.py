#!/usr/bin/env python3
"""PyQt monitor for Dynamixel XH430 and AMT212 on one serial bus."""

import argparse
import math
import sys
import threading
import time
from collections import deque

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

from teraffe_drive_control_agent.config.dynamixel_config import (
    ADDR_HARDWARE_ERROR_STATUS,
    ADDR_GOAL_CURRENT,
    ADDR_MODEL_NUMBER,
    ADDR_MOVING,
    ADDR_OPERATING_MODE,
    ADDR_PRESENT_CURRENT,
    ADDR_PRESENT_INPUT_VOLTAGE,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_PRESENT_VELOCITY,
    ADDR_TORQUE_ENABLE,
    CURRENT_CONTROL_MODE,
    PRESENT_VELOCITY_UNIT,
    PROTOCOL_VERSION,
    TORQUE_DISABLE,
    TORQUE_ENABLE,
)
from teraffe_drive_control_agent.config.encoder_config import (
    AMT212_BAUDRATE,
    AMT212_DEG_PER_COUNT,
    AMT212_PORT,
    AMT212_POSITION_MASK,
)
from teraffe_drive_control_agent.config.teraffe_config import (
    DYNAMIXEL_BAUDRATE,
    DYNAMIXEL_PORT,
)
from teraffe_drive_control_agent.hardware.encoder import verify_checksum


CURRENT_UNIT_A = 0.00269
POSITION_RESOLUTION = 4096.0
RTS_TX = False
RTS_RX = True

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
    if value is not None and value >= 2**15:
        value -= 2**16
    return value


def u32_to_i32(value):
    if value is not None and value >= 2**31:
        value -= 2**32
    return value


def hardware_error_text(value):
    if value is None:
        return "--"
    if value == 0:
        return "OK"
    names = [name for bit, name in HARDWARE_ERROR_BITS.items() if value & (1 << bit)]
    return ", ".join(names) if names else f"0x{value:02X}"


class CombinedReader(QObject):
    sample_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    command_finished = pyqtSignal(bool, str)
    finished = pyqtSignal()

    def __init__(
        self,
        port_name,
        dxl_baudrate,
        encoder_baudrate,
        dxl_id,
        encoder_id,
        period_s,
    ):
        super().__init__()
        self.port_name = port_name
        self.dxl_baudrate = dxl_baudrate
        self.encoder_baudrate = encoder_baudrate
        self.dxl_id = dxl_id
        self.encoder_id = encoder_id
        self.period_s = period_s
        self.running = False
        self.port = None
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.ok_count = 0
        self.fail_count = 0
        self.active_baudrate = None
        self.commands = deque()
        self.commands_lock = threading.Lock()

    def run(self):
        self.running = True
        self.port = PortHandler(self.port_name)
        try:
            if not self.port.openPort():
                raise RuntimeError("openPort returned false")
            self.set_baudrate(self.dxl_baudrate)
        except Exception as exc:
            self.connection_changed.emit(False, f"Port open failed: {exc}")
            self.finished.emit()
            return

        self.connection_changed.emit(True, f"Connected to {self.port_name}")
        try:
            while self.running:
                started = time.monotonic()
                self.process_commands()
                sample = {
                    "timestamp": time.time(),
                    "dxl": self.read_dynamixel(),
                    "encoder": self.read_encoder(),
                }
                self.sample_received.emit(sample)
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
                if command == "current_mode":
                    self.set_current_control_mode()
                    self.command_finished.emit(True, "DXL current mode enabled")
                elif command == "torque_on":
                    self.write_u8(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                    self.command_finished.emit(True, "DXL torque ON")
                elif command == "torque_off":
                    self.write_u8(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                    self.command_finished.emit(True, "DXL torque OFF")
                elif command == "set_current":
                    self.write_goal_current(value)
                    self.command_finished.emit(True, f"DXL goal current {value:.4f} A")
                elif command == "stop_current":
                    self.write_goal_current(0.0)
                    self.command_finished.emit(True, "DXL goal current 0.0000 A")
            except Exception as exc:
                self.command_finished.emit(False, f"Command failed: {exc}")

    def set_baudrate(self, baudrate):
        if self.active_baudrate == baudrate:
            return True
        if not self.port.setBaudRate(baudrate):
            raise RuntimeError(f"setBaudRate {baudrate} returned false")
        self.active_baudrate = baudrate
        time.sleep(0.002)
        return True

    def set_current_control_mode(self):
        self.set_baudrate(self.dxl_baudrate)
        self.write_u8(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.write_u8(ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
        self.write_u8(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def write_goal_current(self, current_a):
        raw = int(current_a / CURRENT_UNIT_A)
        raw = max(-32768, min(32767, raw))
        self.write_u16(ADDR_GOAL_CURRENT, raw & 0xFFFF)

    def read_dynamixel(self):
        self.set_baudrate(self.dxl_baudrate)
        errors = []
        model = self.read_u16(ADDR_MODEL_NUMBER, errors)
        torque = self.read_u8(ADDR_TORQUE_ENABLE, errors)
        operating_mode = self.read_u8(ADDR_OPERATING_MODE, errors)
        current_raw = u16_to_i16(self.read_u16(ADDR_PRESENT_CURRENT, errors))
        velocity_raw = u32_to_i32(self.read_u32(ADDR_PRESENT_VELOCITY, errors))
        position_raw = u32_to_i32(self.read_u32(ADDR_PRESENT_POSITION, errors))
        voltage_raw = self.read_u16(ADDR_PRESENT_INPUT_VOLTAGE, errors)
        temperature = self.read_u8(ADDR_PRESENT_TEMPERATURE, errors)
        moving = self.read_u8(ADDR_MOVING, errors)
        hw_error = self.read_u8(ADDR_HARDWARE_ERROR_STATUS, errors)

        return {
            "ok": not errors,
            "errors": "; ".join(errors) if errors else "OK",
            "model": model,
            "torque": torque,
            "mode": OPERATING_MODE_TEXT.get(operating_mode, str(operating_mode)),
            "current_a": current_raw * CURRENT_UNIT_A if current_raw is not None else None,
            "current_raw": current_raw,
            "voltage_v": voltage_raw * 0.1 if voltage_raw is not None else None,
            "velocity_rad_s": velocity_raw * PRESENT_VELOCITY_UNIT if velocity_raw is not None else None,
            "velocity_rpm": (
                velocity_raw * PRESENT_VELOCITY_UNIT * 60.0 / (2.0 * math.pi)
                if velocity_raw is not None
                else None
            ),
            "velocity_raw": velocity_raw,
            "position_deg": position_raw * 360.0 / POSITION_RESOLUTION if position_raw is not None else None,
            "position_rad": (
                position_raw * 2.0 * math.pi / POSITION_RESOLUTION if position_raw is not None else None
            ),
            "position_raw": position_raw,
            "temperature_c": temperature,
            "moving": moving,
            "hardware_error": hardware_error_text(hw_error),
        }

    def read_encoder(self):
        self.set_baudrate(self.encoder_baudrate)
        ser = self.port.ser
        ser.timeout = 0.02
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.rts = RTS_TX
        ser.write(bytes([self.encoder_id]))
        ser.flush()
        ser.rts = RTS_RX
        raw = ser.read(8)

        result = {
            "ok": False,
            "raw": raw.hex(" ").upper(),
            "raw16": "--",
            "count": None,
            "angle_deg": None,
            "angle_rad": None,
            "error": "short/no response",
        }
        if len(raw) < 2:
            return result

        raw16 = raw[-2] | (raw[-1] << 8)
        result["raw16"] = f"0x{raw16:04X}"
        if not verify_checksum(raw16):
            result["error"] = "checksum failed"
            return result

        count = raw16 & AMT212_POSITION_MASK
        angle_deg = count * AMT212_DEG_PER_COUNT
        result.update(
            {
                "ok": True,
                "count": count,
                "angle_deg": angle_deg,
                "angle_rad": math.radians(angle_deg),
                "error": "OK",
            }
        )
        return result

    def read_u8(self, address, errors):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(self.port, self.dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def read_u16(self, address, errors):
        value, comm_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def read_u32(self, address, errors):
        value, comm_result, dxl_error = self.packet.read4ByteTxRx(self.port, self.dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def check_read(self, address, value, comm_result, dxl_error, errors):
        if comm_result != COMM_SUCCESS:
            errors.append(f"{address}: {self.packet.getTxRxResult(comm_result)}")
            return None
        if dxl_error != 0:
            errors.append(f"{address}: {self.packet.getRxPacketError(dxl_error)}")
            return None
        return value

    def write_u8(self, address, value):
        self.set_baudrate(self.dxl_baudrate)
        comm_result, dxl_error = self.packet.write1ByteTxRx(
            self.port,
            self.dxl_id,
            address,
            value,
        )
        self.check_write(address, comm_result, dxl_error)

    def write_u16(self, address, value):
        self.set_baudrate(self.dxl_baudrate)
        comm_result, dxl_error = self.packet.write2ByteTxRx(
            self.port,
            self.dxl_id,
            address,
            value,
        )
        self.check_write(address, comm_result, dxl_error)

    def check_write(self, address, comm_result, dxl_error):
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(f"{address}: {self.packet.getTxRxResult(comm_result)}")
        if dxl_error != 0:
            raise RuntimeError(f"{address}: {self.packet.getRxPacketError(dxl_error)}")


class CombinedStatusWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.thread = None
        self.reader = None
        self.last_timestamp = None
        self.labels = {}
        self.ok_count = 0
        self.fail_count = 0

        self.setWindowTitle("DXL + Encoder Status Monitor")
        self.build_ui()
        self.setMinimumSize(560, 520)
        self.setMaximumSize(860, 720)
        self.resize(760, 620)

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.refresh_age)
        self.age_timer.start(100)

        if args.autostart:
            self.start_reader()

    def build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        controls = QGroupBox("Shared Serial Bus")
        form = QFormLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        if self.args.port:
            index = self.port_combo.findText(self.args.port)
            if index < 0:
                self.port_combo.insertItem(0, self.args.port)
                index = 0
            self.port_combo.setCurrentIndex(index)

        self.dxl_baudrate_spin = QSpinBox()
        self.dxl_baudrate_spin.setRange(9600, 4500000)
        self.dxl_baudrate_spin.setValue(self.args.dxl_baudrate)
        self.dxl_baudrate_spin.setMaximumWidth(140)

        self.encoder_baudrate_spin = QSpinBox()
        self.encoder_baudrate_spin.setRange(9600, 3000000)
        self.encoder_baudrate_spin.setValue(self.args.encoder_baudrate)
        self.encoder_baudrate_spin.setMaximumWidth(140)

        self.dxl_id_spin = QSpinBox()
        self.dxl_id_spin.setRange(0, 252)
        self.dxl_id_spin.setValue(self.args.dxl_id)
        self.dxl_id_spin.setMaximumWidth(90)

        self.encoder_id_spin = QSpinBox()
        self.encoder_id_spin.setRange(0, 255)
        self.encoder_id_spin.setValue(self.args.encoder_id)
        self.encoder_id_spin.setMaximumWidth(90)

        self.period_spin = QDoubleSpinBox()
        self.period_spin.setRange(0.05, 5.0)
        self.period_spin.setSingleStep(0.05)
        self.period_spin.setSuffix(" s")
        self.period_spin.setValue(self.args.period)
        self.period_spin.setMaximumWidth(90)

        form.addRow("Port", self.port_combo)
        form.addRow("DXL baudrate", self.dxl_baudrate_spin)
        form.addRow("Encoder baudrate", self.encoder_baudrate_spin)
        form.addRow("DXL ID", self.dxl_id_spin)
        form.addRow("Encoder ID", self.encoder_id_spin)
        form.addRow("Period", self.period_spin)
        controls.setLayout(form)
        layout.addWidget(controls)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.refresh_button = QPushButton("Refresh Ports")
        for button in (self.start_button, self.stop_button, self.refresh_button):
            button.setMaximumWidth(130)
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_reader)
        self.stop_button.clicked.connect(self.stop_reader)
        self.refresh_button.clicked.connect(self.refresh_ports)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.refresh_button)
        buttons.addStretch()
        layout.addLayout(buttons)

        control_group = QGroupBox("Dynamixel Current Control")
        control_grid = QGridLayout()
        self.goal_current_spin = QDoubleSpinBox()
        self.goal_current_spin.setRange(-2.0, 2.0)
        self.goal_current_spin.setDecimals(4)
        self.goal_current_spin.setSingleStep(0.01)
        self.goal_current_spin.setSuffix(" A")
        self.goal_current_spin.setValue(0.05)
        self.goal_current_spin.setMaximumWidth(120)

        self.current_mode_button = QPushButton("Current Mode")
        self.torque_on_button = QPushButton("Torque On")
        self.apply_current_button = QPushButton("Apply Current")
        self.stop_current_button = QPushButton("Stop Current")
        self.torque_off_button = QPushButton("Torque Off")
        for button in (
            self.current_mode_button,
            self.torque_on_button,
            self.apply_current_button,
            self.stop_current_button,
            self.torque_off_button,
        ):
            button.setMaximumWidth(130)

        self.current_mode_button.clicked.connect(self.on_current_mode)
        self.torque_on_button.clicked.connect(self.on_torque_on)
        self.apply_current_button.clicked.connect(self.on_apply_current)
        self.stop_current_button.clicked.connect(self.on_stop_current)
        self.torque_off_button.clicked.connect(self.on_torque_off)

        control_grid.addWidget(QLabel("Goal current"), 0, 0)
        control_grid.addWidget(self.goal_current_spin, 0, 1)
        control_grid.addWidget(self.current_mode_button, 1, 0)
        control_grid.addWidget(self.torque_on_button, 1, 1)
        control_grid.addWidget(self.torque_off_button, 1, 2)
        control_grid.addWidget(self.apply_current_button, 2, 0)
        control_grid.addWidget(self.stop_current_button, 2, 1)
        control_grid.setColumnStretch(3, 1)
        control_group.setLayout(control_grid)
        layout.addWidget(control_group)

        status_row = QHBoxLayout()
        status_row.addWidget(self.make_value_group("Dynamixel XH430", [
            ("dxl_model", "Model"),
            ("dxl_torque", "Torque"),
            ("dxl_mode", "Mode"),
            ("dxl_current_a", "Current A"),
            ("dxl_voltage_v", "Voltage V"),
            ("dxl_velocity_rad_s", "Velocity rad/s"),
            ("dxl_velocity_rpm", "Velocity RPM"),
            ("dxl_position_deg", "Position deg"),
            ("dxl_position_rad", "Position rad"),
            ("dxl_temperature_c", "Temperature C"),
            ("dxl_moving", "Moving"),
            ("dxl_hardware_error", "Hardware error"),
            ("dxl_status", "Read status"),
        ]), 1)

        right_column = QVBoxLayout()
        right_column.addWidget(self.make_value_group("AMT212 Encoder", [
            ("enc_angle_deg", "Angle deg"),
            ("enc_angle_rad", "Angle rad"),
            ("enc_count", "Raw count"),
            ("enc_raw16", "Raw16"),
            ("enc_raw", "Raw bytes"),
            ("enc_status", "Read status"),
        ]))

        right_column.addWidget(self.make_value_group("Loop", [
            ("ok_count", "OK count"),
            ("fail_count", "Fail count"),
            ("age_ms", "Age ms"),
        ]))
        right_column.addStretch()
        status_row.addLayout(right_column, 1)
        layout.addLayout(status_row)

        self.setCentralWidget(root)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def make_value_group(self, title, rows):
        group = QGroupBox(title)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(3)
        grid.setContentsMargins(8, 8, 8, 8)
        for row, (key, name) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setMinimumWidth(105)
            name_label.setMaximumWidth(125)
            grid.addWidget(name_label, row, 0)
            label = QLabel("--")
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setMinimumWidth(110)
            label.setMaximumWidth(190)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            grid.addWidget(label, row, 1)
            self.labels[key] = label
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 1)
        group.setLayout(grid)
        return group

    def refresh_ports(self):
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = [DYNAMIXEL_PORT or AMT212_PORT]
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
        self.reader = CombinedReader(
            port_name=self.port_combo.currentText(),
            dxl_baudrate=self.dxl_baudrate_spin.value(),
            encoder_baudrate=self.encoder_baudrate_spin.value(),
            dxl_id=self.dxl_id_spin.value(),
            encoder_id=self.encoder_id_spin.value(),
            period_s=self.period_spin.value(),
        )
        self.reader.moveToThread(self.thread)
        self.thread.started.connect(self.reader.run)
        self.reader.sample_received.connect(self.update_sample)
        self.reader.connection_changed.connect(self.on_connection_changed)
        self.reader.command_finished.connect(self.on_command_finished)
        self.reader.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.reader.deleteLater)
        self.thread.finished.connect(self.reader_finished)
        self.thread.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.statusBar().showMessage("Opening shared serial bus...")

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait(1500)

    def reader_finished(self):
        self.thread = None
        self.reader = None
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def on_connection_changed(self, connected, message):
        self.statusBar().showMessage(self.short_text(message, 100))

    def on_command_finished(self, ok, message):
        prefix = "OK" if ok else "ERR"
        self.statusBar().showMessage(self.short_text(f"{prefix}: {message}", 100))

    def on_current_mode(self):
        self.enqueue_reader_command("current_mode")

    def on_torque_on(self):
        self.enqueue_reader_command("torque_on")

    def on_torque_off(self):
        self.enqueue_reader_command("torque_off")

    def on_apply_current(self):
        self.enqueue_reader_command("set_current", self.goal_current_spin.value())

    def on_stop_current(self):
        self.enqueue_reader_command("stop_current")

    def enqueue_reader_command(self, command, value=None):
        if self.reader is None:
            self.statusBar().showMessage("Start the monitor before sending commands")
            return
        self.reader.enqueue_command(command, value)

    def update_sample(self, sample):
        self.last_timestamp = sample["timestamp"]
        dxl = sample["dxl"]
        enc = sample["encoder"]
        ok = dxl["ok"] and enc["ok"]

        self.set_label("dxl_model", self.fmt(dxl["model"], "{}"))
        self.set_label("dxl_torque", "ON" if dxl["torque"] == 1 else "OFF")
        self.set_label("dxl_mode", dxl["mode"])
        self.set_label("dxl_current_a", self.fmt(dxl["current_a"], "{:.4f}"))
        self.set_label("dxl_voltage_v", self.fmt(dxl["voltage_v"], "{:.2f}"))
        self.set_label("dxl_velocity_rad_s", self.fmt(dxl["velocity_rad_s"], "{:.4f}"))
        self.set_label("dxl_velocity_rpm", self.fmt(dxl["velocity_rpm"], "{:.2f}"))
        self.set_label("dxl_position_deg", self.fmt(dxl["position_deg"], "{:.2f}"))
        self.set_label("dxl_position_rad", self.fmt(dxl["position_rad"], "{:.4f}"))
        self.set_label("dxl_temperature_c", self.fmt(dxl["temperature_c"], "{}"))
        self.set_label("dxl_moving", "YES" if dxl["moving"] == 1 else "NO")
        self.set_label("dxl_hardware_error", dxl["hardware_error"])
        self.set_label("dxl_status", dxl["errors"])

        self.set_label("enc_angle_deg", self.fmt(enc["angle_deg"], "{:.3f}"))
        self.set_label("enc_angle_rad", self.fmt(enc["angle_rad"], "{:.5f}"))
        self.set_label("enc_count", self.fmt(enc["count"], "{}"))
        self.set_label("enc_raw16", enc["raw16"])
        self.set_label("enc_raw", enc["raw"] or "--")
        self.set_label("enc_status", enc["error"])

        if ok:
            self.ok_count += 1
        else:
            self.fail_count += 1
        self.set_label("ok_count", str(self.ok_count))
        self.set_label("fail_count", str(self.fail_count))
        self.refresh_age()

    def refresh_age(self):
        if self.last_timestamp is None:
            return
        self.set_label("age_ms", f"{(time.time() - self.last_timestamp) * 1000.0:.0f}")

    def closeEvent(self, event):
        self.stop_reader()
        super().closeEvent(event)

    def set_label(self, key, text):
        label = self.labels[key]
        display_text = self.elide_text(label, text)
        label.setText(display_text)
        label.setToolTip(text if display_text != text else "")

    @staticmethod
    def fmt(value, fmt):
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
    parser = argparse.ArgumentParser(description="Dynamixel + AMT212 status monitor UI")
    parser.add_argument("--port", default=DYNAMIXEL_PORT or AMT212_PORT)
    parser.add_argument("--dxl-baudrate", type=int, default=DYNAMIXEL_BAUDRATE)
    parser.add_argument("--encoder-baudrate", type=int, default=AMT212_BAUDRATE)
    parser.add_argument("--dxl-id", type=int, default=1)
    parser.add_argument("--encoder-id", type=lambda value: int(value, 0), default=0x2C)
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--no-autostart", dest="autostart", action="store_false")
    parser.set_defaults(autostart=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv)
    window = CombinedStatusWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
