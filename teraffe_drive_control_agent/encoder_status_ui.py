#!/usr/bin/env python3
"""PyQt monitor for AMT212 encoder position over USB-RS485."""

import argparse
import sys
import time

import serial
from serial.tools import list_ports
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
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
    QSpinBox,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from teraffe_drive_control_agent.config.encoder_config import (
    AMT212_BAUDRATE,
    AMT212_DEG_PER_COUNT,
    AMT212_PORT,
    AMT212_POSITION_MASK,
)
from teraffe_drive_control_agent.hardware.encoder import verify_checksum


RTS_TX = False
RTS_RX = True


class EncoderReader(QObject):
    sample_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    finished = pyqtSignal()

    def __init__(self, port, baudrate, encoder_id, period_s, read_size):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.encoder_id = encoder_id
        self.period_s = period_s
        self.read_size = read_size
        self.running = False
        self.ser = None
        self.ok_count = 0
        self.fail_count = 0

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
                write_timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            self.ser.rts = RTS_RX
            self.ser.dtr = False
        except Exception as exc:
            self.connection_changed.emit(False, f"Port open failed: {exc}")
            self.finished.emit()
            return

        self.connection_changed.emit(True, f"Connected to {self.port}")
        try:
            while self.running:
                started = time.monotonic()
                sample = self.read_once()
                self.sample_received.emit(sample)
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, self.period_s - elapsed))
        except Exception as exc:
            self.connection_changed.emit(False, f"Read failed: {exc}")
        finally:
            if self.ser is not None:
                self.ser.close()
                self.ser = None
            self.connection_changed.emit(False, "Disconnected")
            self.finished.emit()

    def stop(self):
        self.running = False

    def read_once(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.rts = RTS_TX
        self.ser.write(bytes([self.encoder_id]))
        self.ser.flush()
        self.ser.rts = RTS_RX

        raw = self.ser.read(self.read_size)
        sample = {
            "timestamp": time.time(),
            "encoder_id": self.encoder_id,
            "raw": raw.hex(" ").upper(),
            "raw_len": len(raw),
            "valid": False,
            "raw16": None,
            "position_count": None,
            "angle_deg": None,
            "angle_rad": None,
            "ok_count": self.ok_count,
            "fail_count": self.fail_count,
        }

        if len(raw) < 2:
            self.fail_count += 1
            sample["fail_count"] = self.fail_count
            sample["error"] = "short/no response"
            return sample

        raw16 = raw[-2] | (raw[-1] << 8)
        sample["raw16"] = f"0x{raw16:04X}"
        if not verify_checksum(raw16):
            self.fail_count += 1
            sample["fail_count"] = self.fail_count
            sample["error"] = "checksum failed"
            return sample

        position = raw16 & AMT212_POSITION_MASK
        angle_deg = position * AMT212_DEG_PER_COUNT
        self.ok_count += 1
        sample.update(
            {
                "valid": True,
                "position_count": position,
                "angle_deg": angle_deg,
                "angle_rad": angle_deg * 3.141592653589793 / 180.0,
                "ok_count": self.ok_count,
                "fail_count": self.fail_count,
                "error": "OK",
            }
        )
        return sample


class EncoderStatusWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.thread = None
        self.reader = None
        self.last_timestamp = None

        self.setWindowTitle("AMT212 Encoder Status Monitor")
        self.resize(620, 360)
        self._build_ui()

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self.refresh_age)
        self.age_timer.start(100)

        if args.autostart:
            self.start_reader()

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        controls = QGroupBox("Serial")
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
        self.baudrate_spin.setRange(1200, 3000000)
        self.baudrate_spin.setValue(self.args.baudrate)

        self.id_spin = QSpinBox()
        self.id_spin.setRange(0, 255)
        self.id_spin.setValue(self.args.encoder_id)
        self.id_spin.setDisplayIntegerBase(10)

        self.period_spin = QDoubleSpinBox()
        self.period_spin.setRange(0.01, 5.0)
        self.period_spin.setSingleStep(0.01)
        self.period_spin.setSuffix(" s")
        self.period_spin.setValue(self.args.period)

        form.addRow("Port", self.port_combo)
        form.addRow("Baudrate", self.baudrate_spin)
        form.addRow("Encoder ID", self.id_spin)
        form.addRow("Period", self.period_spin)
        controls.setLayout(form)
        layout.addWidget(controls)

        button_row = QGridLayout()
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.refresh_button = QPushButton("Refresh Ports")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_reader)
        self.stop_button.clicked.connect(self.stop_reader)
        self.refresh_button.clicked.connect(self.refresh_ports)
        button_row.addWidget(self.start_button, 0, 0)
        button_row.addWidget(self.stop_button, 0, 1)
        button_row.addWidget(self.refresh_button, 0, 2)
        layout.addLayout(button_row)

        values = QGroupBox("Encoder")
        grid = QGridLayout()
        self.labels = {}
        rows = [
            ("angle_deg", "Angle deg"),
            ("angle_rad", "Angle rad"),
            ("position_count", "Raw count"),
            ("raw16", "Raw16"),
            ("raw", "Raw bytes"),
            ("valid", "Checksum"),
            ("ok_count", "OK count"),
            ("fail_count", "Fail count"),
            ("age_ms", "Age ms"),
        ]
        for row, (key, name) in enumerate(rows):
            grid.addWidget(QLabel(name), row, 0)
            label = QLabel("--")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
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
            ports = [AMT212_PORT]
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
        self.reader = EncoderReader(
            port=self.port_combo.currentText(),
            baudrate=self.baudrate_spin.value(),
            encoder_id=self.id_spin.value(),
            period_s=self.period_spin.value(),
            read_size=self.args.read_size,
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
        self.statusBar().showMessage("Opening serial port...")

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
        self.statusBar().showMessage(message)

    def update_sample(self, sample):
        self.last_timestamp = sample["timestamp"]
        self.labels["angle_deg"].setText(self._fmt(sample["angle_deg"], "{:.3f}"))
        self.labels["angle_rad"].setText(self._fmt(sample["angle_rad"], "{:.5f}"))
        self.labels["position_count"].setText(self._fmt(sample["position_count"], "{}"))
        self.labels["raw16"].setText(self._fmt(sample["raw16"], "{}"))
        self.labels["raw"].setText(sample["raw"] or "--")
        self.labels["valid"].setText("OK" if sample["valid"] else sample.get("error", "ERR"))
        self.labels["ok_count"].setText(str(sample["ok_count"]))
        self.labels["fail_count"].setText(str(sample["fail_count"]))
        self.refresh_age()

    def refresh_age(self):
        if self.last_timestamp is None:
            return
        age_ms = (time.time() - self.last_timestamp) * 1000.0
        self.labels["age_ms"].setText(f"{age_ms:.0f}")

    def closeEvent(self, event):
        self.stop_reader()
        super().closeEvent(event)

    @staticmethod
    def _fmt(value, fmt):
        if value is None:
            return "--"
        return fmt.format(value)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="AMT212 encoder status monitor UI")
    parser.add_argument("--port", default=AMT212_PORT, help="Serial port")
    parser.add_argument("--baudrate", type=int, default=AMT212_BAUDRATE)
    parser.add_argument("--encoder-id", type=lambda value: int(value, 0), default=0x2C)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--read-size", type=int, default=8)
    parser.add_argument("--no-autostart", dest="autostart", action="store_false")
    parser.set_defaults(autostart=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv)
    window = EncoderStatusWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
