#!/usr/bin/env python3
"""Browser UI for AK80-64, Dynamixel XH430, and AMT212 status/control."""

import argparse
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import can
import serial
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from serial.tools import list_ports

from teraffe_drive_control_agent.ak80_64_status_ui import StatusStore
from teraffe_drive_control_agent.config.ak80_64_config import (
    AK80_64_CAN_BITRATE,
    AK80_64_ERPM_MAX,
    AK80_64_ERPM_MIN,
    CAN_PACKET_SET_RPM,
)
from teraffe_drive_control_agent.config.dynamixel_config import (
    ADDR_GOAL_CURRENT,
    ADDR_HARDWARE_ERROR_STATUS,
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
    AMT212_POSITION_MASK,
    AMT212_RESPONSE_DELAY_SEC,
    AMT212_RETRY_COUNT,
    AMT212_RETRY_DELAY_SEC,
)
from teraffe_drive_control_agent.config.teraffe_config import DYNAMIXEL_BAUDRATE, DYNAMIXEL_PORT
from teraffe_drive_control_agent.hardware.encoder import verify_checksum


CURRENT_UNIT_A = 0.00269
DXL_POSITION_RESOLUTION = 4096.0
RTS_TX = False
RTS_RX = True
STALE_SEC = 2.0
SERIAL_LOOP_SEC = 0.005
DXL_POLL_SEC = 0.15
DXL_RETRY_SEC = 2.0
ENCODER_TIMEOUT_SEC = 0.02
SERIAL_TURNAROUND_SEC = 0.003

MODULES = {
    "FL": {"ak": 101, "dxl": 111, "enc": 0x1C},
    "FR": {"ak": 102, "dxl": 112, "enc": 0x2C},
    "RL": {"ak": 103, "dxl": 113, "enc": 0x3C},
    "RR": {"ak": 104, "dxl": 114, "enc": 0x4C},
}

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


HTML = r"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Teraffe Hardware UI</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f6f7f8;
      --panel: #ffffff;
      --line: #cfd6dd;
      --text: #17202a;
      --muted: #66707a;
      --green: #18a957;
      --red: #d63737;
      --amber: #d98c00;
      --blue: #1f6feb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font: 16px/1.35 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    header {
      height: 44px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 16px;
      border-bottom: 1px solid var(--line);
      background: #fff;
      font-weight: 700;
    }
    main {
      display: grid;
      grid-template-columns: minmax(520px, 0.92fr) minmax(620px, 1.08fr);
      gap: 12px;
      padding: 12px 16px;
      width: 100%;
      max-width: none;
      margin: 0;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
    }
    h2 {
      font-size: 17px;
      margin: 0 0 8px;
    }
    .muted { color: var(--muted); }
    .layout-left {
      display: grid;
      gap: 12px;
      align-content: start;
    }
    .robot-wrap {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 210px;
      gap: 12px;
      align-items: start;
    }
    svg {
      width: 100%;
      max-height: 480px;
      background: #fff;
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .legend {
      display: grid;
      gap: 8px;
      font-size: 15px;
    }
    .legend-block {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      background: #fbfcfd;
    }
    .dot {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 5px;
      background: var(--red);
      vertical-align: -1px;
    }
    .ok { background: var(--green); }
    .bad { background: var(--red); }
    .warn { background: var(--amber); }
    .controls {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
      gap: 10px;
      align-items: start;
      overflow: hidden;
    }
    .controls > div {
      min-width: 0;
    }
    .control-grid {
      display: grid;
      grid-template-columns: 105px minmax(0, 1fr);
      gap: 7px 7px;
      align-items: center;
    }
    label {
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    input, select, button {
      width: 100%;
      min-width: 0;
      min-height: 34px;
      border: 1px solid #b8c1ca;
      border-radius: 6px;
      padding: 5px 10px;
      background: #fff;
      color: var(--text);
      font: inherit;
    }
    button {
      cursor: pointer;
      font-weight: 650;
    }
    button.primary { background: var(--blue); color: #fff; border-color: var(--blue); }
    button.stop { background: #fff1f1; color: #a90000; border-color: #e5a7a7; }
    .button-row {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(125px, 1fr));
      gap: 7px;
      margin-top: 8px;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(340px, 1fr));
      gap: 12px;
    }
    .module-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      overflow: hidden;
    }
    .module-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding: 8px 10px;
      background: #eef3f7;
      border-bottom: 1px solid var(--line);
      font-weight: 800;
    }
    .module-head small {
      display: block;
      margin-top: 2px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 650;
    }
    .chips {
      display: flex;
      gap: 6px;
      flex-wrap: wrap;
      font-size: 14px;
      font-weight: 650;
    }
    .chip {
      display: inline-flex;
      align-items: center;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 2px 7px;
      background: #fff;
    }
    .status-section {
      padding: 10px 12px;
      border-top: 1px solid var(--line);
    }
    .status-section:first-of-type {
      border-top: 0;
    }
    .status-title {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 5px;
      font-size: 15px;
      font-weight: 800;
      color: #334155;
    }
    .kv {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 4px 22px;
    }
    .pair {
      display: grid;
      grid-template-columns: 96px minmax(120px, 1fr);
      gap: 10px;
      white-space: nowrap;
    }
    .key { color: var(--muted); }
    .value {
      font-variant-numeric: tabular-nums;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .footer-status {
      font-size: 15px;
      color: var(--muted);
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .status-grid, .robot-wrap { grid-template-columns: 1fr; }
      .controls { grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <div>Teraffe Hardware Control</div>
    <div class="footer-status" id="serverStatus">connecting...</div>
  </header>
  <main>
    <div class="layout-left">
      <section>
        <h2>Robot Map</h2>
        <div class="robot-wrap">
          <svg viewBox="0 0 420 540" role="img" aria-label="Teraffe robot hardware map">
            <defs>
              <marker id="arrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
                <path d="M0,0 L0,6 L7,3 z" fill="#333"></path>
              </marker>
            </defs>
            <line x1="130" y1="150" x2="290" y2="150" stroke="#2a2f36" stroke-width="7"/>
            <line x1="150" y1="347" x2="254" y2="347" stroke="#2a2f36" stroke-width="7"/>
            <line x1="137" y1="175" x2="137" y2="365" stroke="#2a2f36" stroke-width="5"/>
            <line x1="267" y1="175" x2="267" y2="365" stroke="#2a2f36" stroke-width="5"/>
            <g id="map-FL" transform="translate(54 60)">
              <text x="0" y="0" font-size="30" font-weight="800">FL</text>
              <rect x="-6" y="34" width="72" height="74" fill="#a7a4a0" stroke="#333"/>
              <rect x="8" y="44" width="27" height="45" fill="#b7bfd1" stroke="#555"/>
              <rect x="72" y="34" width="20" height="125" transform="rotate(-24 82 96)" fill="#181923" stroke="#333"/>
              <circle cx="83" cy="98" r="13" fill="#bec8a9" stroke="#555"/>
            </g>
            <g id="map-FR" transform="translate(294 60)">
              <text x="0" y="0" font-size="30" font-weight="800">FR</text>
              <rect x="-20" y="34" width="72" height="74" fill="#a7a4a0" stroke="#333"/>
              <rect x="14" y="44" width="27" height="45" fill="#b7bfd1" stroke="#555"/>
              <rect x="-38" y="34" width="20" height="125" transform="rotate(24 -28 96)" fill="#181923" stroke="#333"/>
              <circle cx="-27" cy="98" r="13" fill="#bec8a9" stroke="#555"/>
            </g>
            <g id="map-RL" transform="translate(54 420)">
              <text x="0" y="56" font-size="30" font-weight="800">RL</text>
              <rect x="-6" y="-86" width="72" height="74" fill="#a7a4a0" stroke="#333"/>
              <rect x="8" y="-76" width="27" height="45" fill="#b7bfd1" stroke="#555"/>
              <rect x="72" y="-134" width="20" height="125" transform="rotate(24 82 -72)" fill="#181923" stroke="#333"/>
              <circle cx="83" cy="-73" r="13" fill="#bec8a9" stroke="#555"/>
            </g>
            <g id="map-RR" transform="translate(294 420)">
              <text x="0" y="56" font-size="30" font-weight="800">RR</text>
              <rect x="-20" y="-86" width="72" height="74" fill="#a7a4a0" stroke="#333"/>
              <rect x="14" y="-76" width="27" height="45" fill="#b7bfd1" stroke="#555"/>
              <rect x="-38" y="-134" width="20" height="125" transform="rotate(-24 -28 -72)" fill="#181923" stroke="#333"/>
              <circle cx="-27" cy="-73" r="13" fill="#bec8a9" stroke="#555"/>
            </g>
            <g id="lights">
              <circle id="light-FL" cx="92" cy="184" r="12" fill="#d63737"/>
              <circle id="light-FR" cx="328" cy="184" r="12" fill="#d63737"/>
              <circle id="light-RL" cx="92" cy="318" r="12" fill="#d63737"/>
              <circle id="light-RR" cx="328" cy="318" r="12" fill="#d63737"/>
            </g>
          </svg>
          <div class="legend">
            <div class="legend-block">
              <strong>Module IDs</strong><br>
              FL: 1 / FR: 2<br>
              RL: 3 / RR: 4
            </div>
            <div class="legend-block">
              <strong>Leg hardware</strong><br>
              CubeMars: 101-104<br>
              Dynamixel: 111-114<br>
              Encoder: 1C, 2C, 3C, 4C
            </div>
            <div class="legend-block">
              <strong>Connection</strong><br>
              <span class="dot ok"></span>connected<br>
              <span class="dot bad"></span>disconnected
            </div>
          </div>
        </div>
      </section>

      <section class="controls">
        <div>
          <h2>Connections</h2>
          <div class="control-grid">
            <label>Serial</label><select id="serialPort"></select>
            <label>DXL baud</label><input id="dxlBaud" type="number" value="115200">
            <label>ENC baud</label><input id="encBaud" type="number" value="115200">
            <label>CAN</label><input id="canChannel" value="can0">
          </div>
          <div class="button-row">
            <button onclick="post('/api/configure')" class="primary">Apply Connections</button>
          </div>
        </div>
        <div>
          <h2>DXL Current</h2>
          <div class="control-grid">
            <label>Module</label><select id="dxlModule"></select>
            <label>Current A</label><input id="dxlCurrent" type="number" value="0.05" step="0.01">
          </div>
          <div class="button-row">
            <button onclick="post('/api/dxl/current-mode', {module: val('dxlModule')})">Current Mode</button>
            <button onclick="post('/api/dxl/current', {module: val('dxlModule'), current: num('dxlCurrent')})" class="primary">Apply</button>
            <button onclick="post('/api/dxl/stop', {module: val('dxlModule')})" class="stop">Stop</button>
            <button onclick="post('/api/dxl/torque-off', {module: val('dxlModule')})">Torque Off</button>
          </div>
        </div>
        <div>
          <h2>AK80-64 ERPM</h2>
          <div class="control-grid">
            <label>Module</label><select id="akModule"></select>
            <label>ERPM</label><input id="akErpm" type="number" value="-5000" step="100">
          </div>
          <div class="button-row">
            <button onclick="post('/api/ak/erpm', {module: val('akModule'), erpm: num('akErpm')})" class="primary">Apply ERPM</button>
            <button onclick="post('/api/ak/stop', {module: val('akModule')})" class="stop">Stop ERPM</button>
          </div>
        </div>
      </section>
    </div>

    <section>
      <h2>Live Status</h2>
      <div class="status-grid" id="cards"></div>
    </section>
  </main>

  <script>
    const modules = ["FL", "FR", "RL", "RR"];
    const moduleIds = {
      FL: {ak: 101, dxl: 111, enc: "1C"},
      FR: {ak: 102, dxl: 112, enc: "2C"},
      RL: {ak: 103, dxl: 113, enc: "3C"},
      RR: {ak: 104, dxl: 114, enc: "4C"}
    };
    const groups = [
      ["AK", [
        ["ak", "Status"],
        ["ak_erpm", "ERPM"],
        ["ak_current", "Current A"],
        ["ak_pos", "Position deg"]
      ]],
      ["DXL", [
        ["dxl", "Status"],
        ["dxl_current", "Current A"],
        ["dxl_voltage", "Voltage V"],
        ["dxl_velocity", "Velocity rad/s"]
      ]],
      ["ENC", [
        ["enc", "Status"],
        ["enc_angle", "Angle deg"]
      ]]
    ];

    function init() {
      for (const id of ["dxlModule", "akModule"]) {
        const select = document.getElementById(id);
        select.innerHTML = modules.map(m => `<option>${m}</option>`).join("") + "<option>ALL</option>";
      }
      fetch("/api/config").then(r => r.json()).then(cfg => {
        const serial = document.getElementById("serialPort");
        serial.innerHTML = cfg.serial_ports.map(p => `<option>${p}</option>`).join("");
        serial.value = cfg.serial_port;
        document.getElementById("dxlBaud").value = cfg.dxl_baudrate;
        document.getElementById("encBaud").value = cfg.encoder_baudrate;
        document.getElementById("canChannel").value = cfg.can_channel;
      });
      drawCards();
      setInterval(refresh, 50);
    }

    function drawCards() {
      const cards = document.getElementById("cards");
      cards.innerHTML = modules.map(m => `
        <div class="module-card" id="card-${m}">
          <div class="module-head">
            <span>
              ${m}
              <small>AK ${moduleIds[m].ak} / DXL ${moduleIds[m].dxl} / ENC ${moduleIds[m].enc}</small>
            </span>
            <span class="chips">
              <span class="chip"><span class="dot bad" id="chip-${m}-ak"></span>AK</span>
              <span class="chip"><span class="dot bad" id="chip-${m}-dxl"></span>DXL</span>
              <span class="chip"><span class="dot bad" id="chip-${m}-enc"></span>ENC</span>
            </span>
          </div>
          ${groups.map(([group, fields]) => `
            <div class="status-section">
              <div class="status-title">${group}</div>
              <div class="kv">
                ${fields.map(([key, label]) => `
                  <div class="pair"><span class="key">${label}</span><span class="value" id="${m}-${key}">--</span></div>
                `).join("")}
              </div>
            </div>
          `).join("")}
        </div>
      `).join("");
    }

    async function refresh() {
      try {
        const status = await fetch("/api/status").then(r => r.json());
        document.getElementById("serverStatus").textContent = status.message;
        for (const m of modules) {
          const item = status.modules[m];
          setLight(`light-${m}`, item.connected);
          setChip(m, "ak", item.ak.connected);
          setChip(m, "dxl", item.dxl.connected);
          setChip(m, "enc", item.encoder.connected);
          setText(`${m}-ak`, item.ak.connected ? "OK" : "ERR");
          setText(`${m}-ak_erpm`, fmt(item.ak.erpm, 0));
          setText(`${m}-ak_current`, fmt(item.ak.current_a, 2));
          setText(`${m}-ak_pos`, fmt(item.ak.position_deg, 2));
          setText(`${m}-dxl`, item.dxl.connected ? "OK" : "ERR");
          setText(`${m}-dxl_current`, fmt(item.dxl.current_a, 4));
          setText(`${m}-dxl_voltage`, fmt(item.dxl.voltage_v, 2));
          setText(`${m}-dxl_velocity`, fmt(item.dxl.velocity_rad_s, 4));
          setText(`${m}-enc`, item.encoder.connected ? "OK" : "ERR");
          setText(`${m}-enc_angle`, fmt(item.encoder.angle_deg, 2));
        }
      } catch (err) {
        document.getElementById("serverStatus").textContent = "server disconnected";
      }
    }

    function setChip(module, name, ok) { setLight(`chip-${module}-${name}`, ok); }
    function setLight(id, ok) {
      const el = document.getElementById(id);
      if (!el) return;
      if (el.tagName.toLowerCase() === "circle") {
        el.setAttribute("fill", ok ? "#18a957" : "#d63737");
      } else {
        el.className = `dot ${ok ? "ok" : "bad"}`;
      }
    }
    function setText(id, text) { document.getElementById(id).textContent = text; }
    function fmt(value, digits) {
      if (value === null || value === undefined) return "--";
      return Number(value).toFixed(digits);
    }
    function val(id) { return document.getElementById(id).value; }
    function num(id) { return Number(document.getElementById(id).value); }
    async function post(path, body = {}) {
      body.serial_port = val("serialPort");
      body.dxl_baudrate = num("dxlBaud");
      body.encoder_baudrate = num("encBaud");
      body.can_channel = val("canChannel");
      const res = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(body)
      });
      const data = await res.json();
      document.getElementById("serverStatus").textContent = data.message;
    }
    init();
  </script>
</body>
</html>
"""


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


def serial_port_choices():
    return [port.device for port in list_ports.comports()]


def default_serial_port():
    return DYNAMIXEL_PORT


def setup_socketcan(channel, bitrate):
    if not os.path.exists(f"/sys/class/net/{channel}"):
        return False, f"{channel} does not exist"

    sudo = [] if os.geteuid() == 0 else ["sudo"]
    try:
        subprocess.run([*sudo, "ip", "link", "set", channel, "down"], check=False)
        subprocess.run(
            [*sudo, "ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
            check=True,
        )
        subprocess.run([*sudo, "ip", "link", "set", channel, "up"], check=True)
    except subprocess.CalledProcessError as exc:
        return False, f"CAN setup failed: {exc}"
    except Exception as exc:
        return False, f"CAN setup failed: {exc}"
    return True, f"{channel} up at {bitrate}"


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.message = "starting"
        self.modules = {
            module: {
                "ak": {"connected": False},
                "dxl": {"connected": False},
                "encoder": {"connected": False},
            }
            for module in MODULES
        }

    def update_module(self, module, key, values):
        with self.lock:
            item = self.modules[module][key]
            item.update(values)
            item["last_seen"] = time.time()
            item["connected"] = True

    def set_message(self, message):
        with self.lock:
            self.message = message

    def reset(self):
        with self.lock:
            self.modules = {
                module: {
                    "ak": {"connected": False},
                    "dxl": {"connected": False},
                    "encoder": {"connected": False},
                }
                for module in MODULES
            }

    def snapshot(self):
        now = time.time()
        with self.lock:
            modules = json.loads(json.dumps(self.modules))
            for module_data in modules.values():
                for status in module_data.values():
                    last_seen = status.get("last_seen")
                    if not last_seen or now - last_seen > STALE_SEC:
                        status["connected"] = False
                module_data["connected"] = all(
                    module_data[key].get("connected", False)
                    for key in ("ak", "dxl", "encoder")
                )
            return {"message": self.message, "modules": modules}


class AkWorker(threading.Thread):
    def __init__(self, state, channel, bitrate):
        super().__init__(daemon=True)
        self.state = state
        self.channel = channel
        self.bitrate = bitrate
        self.running = threading.Event()
        self.running.set()
        self.bus = None
        self.store = StatusStore()
        self.commands = deque()
        self.commands_lock = threading.Lock()
        self.targets = {}
        self.last_send = {}

    def run(self):
        try:
            ok, setup_message = setup_socketcan(self.channel, self.bitrate)
            if not ok:
                raise RuntimeError(setup_message)
            self.state.set_message(setup_message)
            if not os.path.exists(f"/sys/class/net/{self.channel}"):
                raise RuntimeError(f"{self.channel} does not exist")
            try:
                self.bus = can.interface.Bus(channel=self.channel, interface="socketcan", bitrate=self.bitrate)
            except TypeError:
                self.bus = can.interface.Bus(channel=self.channel, bustype="socketcan", bitrate=self.bitrate)
        except Exception as exc:
            self.state.set_message(f"CAN open failed: {exc}")
            return

        self.state.set_message(f"CAN connected: {self.channel}")
        while self.running.is_set():
            self.process_commands()
            self.resend_targets()
            try:
                msg = self.bus.recv(timeout=0.03)
            except can.CanError as exc:
                self.state.set_message(f"CAN receive failed: {exc}")
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            status = self.store.update_from_message(msg)
            if status is None:
                continue
            module = self.module_for_ak(status.motor_id)
            if module is None:
                continue
            self.state.update_module(module, "ak", {
                "id": status.motor_id,
                "mode": status.mode,
                "position_deg": status.position_deg,
                "position_rad": status.position_rad,
                "erpm": status.erpm,
                "output_rpm": status.output_rpm,
                "speed_rad_s": status.speed_rad_s,
                "current_a": status.current_a,
                "motor_temp_c": status.motor_temp_c,
                "error": status.error_text,
                "packet": status.last_packet,
                "target_erpm": self.targets.get(module, 0),
            })

        for module in list(self.targets):
            self.send_erpm(MODULES[module]["ak"], 0)
        if self.bus is not None:
            self.bus.shutdown()

    def stop(self):
        self.running.clear()

    def enqueue(self, module, erpm):
        with self.commands_lock:
            self.commands.append((module, erpm))

    def process_commands(self):
        while True:
            with self.commands_lock:
                if not self.commands:
                    return
                module, erpm = self.commands.popleft()
            modules = MODULES.keys() if module == "ALL" else [module]
            for name in modules:
                erpm = int(max(AK80_64_ERPM_MIN, min(AK80_64_ERPM_MAX, erpm)))
                self.targets[name] = erpm
                self.send_erpm(MODULES[name]["ak"], erpm)
            self.state.set_message(f"AK ERPM command: {module} {erpm}")

    def resend_targets(self):
        now = time.monotonic()
        for module, erpm in list(self.targets.items()):
            if now - self.last_send.get(module, 0.0) >= 0.1:
                self.send_erpm(MODULES[module]["ak"], erpm)

    def send_erpm(self, motor_id, erpm):
        if self.bus is None:
            return
        value = int(erpm)
        if value < 0:
            value += 2**32
        data = [(value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF]
        msg = can.Message(arbitration_id=motor_id | (CAN_PACKET_SET_RPM << 8), data=data, is_extended_id=True)
        try:
            self.bus.send(msg, timeout=0.02)
        except can.CanError as exc:
            self.state.set_message(f"CAN send failed: {exc}")
            return
        module = self.module_for_ak(motor_id)
        if module:
            self.last_send[module] = time.monotonic()

    @staticmethod
    def module_for_ak(motor_id):
        for module, ids in MODULES.items():
            if ids["ak"] == motor_id:
                return module
        return None


class SerialWorker(threading.Thread):
    def __init__(self, state, port_name, dxl_baudrate, encoder_baudrate):
        super().__init__(daemon=True)
        self.state = state
        self.port_name = port_name
        self.dxl_baudrate = dxl_baudrate
        self.encoder_baudrate = encoder_baudrate
        self.running = threading.Event()
        self.running.set()
        self.port = None
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.active_baudrate = None
        self.commands = deque()
        self.commands_lock = threading.Lock()
        self.dxl_index = 0
        self.encoder_index = 0
        self.last_dxl_poll = 0.0
        self.dxl_retry_after = {module: 0.0 for module in MODULES}

    def run(self):
        self.port = PortHandler(self.port_name)
        try:
            if not self.port.openPort():
                raise RuntimeError("openPort returned false")
            self.set_baudrate(self.dxl_baudrate)
        except Exception as exc:
            self.state.set_message(f"Serial open failed: {exc}")
            return

        self.state.set_message(f"Serial connected: {self.port_name}")
        while self.running.is_set():
            started = time.monotonic()
            self.process_commands()

            module_items = list(MODULES.items())
            module, ids = module_items[self.encoder_index % len(module_items)]
            self.encoder_index += 1
            self.read_encoder(module, ids["enc"])

            now = time.monotonic()
            if now - self.last_dxl_poll >= DXL_POLL_SEC:
                module, ids = self.next_dxl_poll(module_items, now)
                self.last_dxl_poll = now
                if module is not None:
                    self.read_dxl(module, ids["dxl"])

            time.sleep(max(0.0, SERIAL_LOOP_SEC - (time.monotonic() - started)))

        if self.port is not None:
            self.port.closePort()

    def stop(self):
        self.running.clear()

    def enqueue(self, command, module, value=0.0):
        with self.commands_lock:
            self.commands.append((command, module, value))

    def next_dxl_poll(self, module_items, now):
        for _ in module_items:
            module, ids = module_items[self.dxl_index % len(module_items)]
            self.dxl_index += 1
            if now >= self.dxl_retry_after.get(module, 0.0):
                return module, ids
        return None, None

    def process_commands(self):
        while True:
            with self.commands_lock:
                if not self.commands:
                    return
                command, module, value = self.commands.popleft()
            modules = MODULES.keys() if module == "ALL" else [module]
            failures = []
            for name in modules:
                dxl_id = MODULES[name]["dxl"]
                try:
                    if command == "current_mode":
                        self.set_current_mode(dxl_id)
                    elif command == "current":
                        self.write_goal_current(dxl_id, value)
                    elif command == "stop":
                        self.write_goal_current(dxl_id, 0.0)
                    elif command == "torque_off":
                        self.write_u8(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                except Exception as exc:
                    failures.append(f"{name}({dxl_id}): {exc}")
                    continue
            if failures:
                self.state.set_message(f"DXL {command} failed: {'; '.join(failures)}")
            else:
                self.state.set_message(f"DXL command done: {command} {module}")

    def set_baudrate(self, baudrate):
        if self.active_baudrate == baudrate:
            return
        if not self.port.setBaudRate(baudrate):
            raise RuntimeError(f"setBaudRate {baudrate} returned false")
        self.active_baudrate = baudrate
        time.sleep(0.002)

    def set_current_mode(self, dxl_id):
        self.write_u8(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.write_u8(dxl_id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
        self.write_u8(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def write_goal_current(self, dxl_id, current_a):
        raw = max(-32768, min(32767, int(current_a / CURRENT_UNIT_A)))
        self.write_u16(dxl_id, ADDR_GOAL_CURRENT, raw & 0xFFFF)

    def read_dxl(self, module, dxl_id):
        errors = []
        try:
            self.set_baudrate(self.dxl_baudrate)
            self.prepare_shared_serial()
        except Exception as exc:
            self.state.set_message(f"DXL baud failed: {exc}")
            return
        model = self.read_u16(dxl_id, ADDR_MODEL_NUMBER, errors)
        torque = self.read_u8(dxl_id, ADDR_TORQUE_ENABLE, errors)
        mode = self.read_u8(dxl_id, ADDR_OPERATING_MODE, errors)
        current_raw = u16_to_i16(self.read_u16(dxl_id, ADDR_PRESENT_CURRENT, errors))
        velocity_raw = u32_to_i32(self.read_u32(dxl_id, ADDR_PRESENT_VELOCITY, errors))
        position_raw = u32_to_i32(self.read_u32(dxl_id, ADDR_PRESENT_POSITION, errors))
        voltage_raw = self.read_u16(dxl_id, ADDR_PRESENT_INPUT_VOLTAGE, errors)
        temp = self.read_u8(dxl_id, ADDR_PRESENT_TEMPERATURE, errors)
        moving = self.read_u8(dxl_id, ADDR_MOVING, errors)
        hw_error = self.read_u8(dxl_id, ADDR_HARDWARE_ERROR_STATUS, errors)
        if all(value is None for value in (model, torque, mode, current_raw, velocity_raw, position_raw, voltage_raw, temp, moving, hw_error)):
            self.dxl_retry_after[module] = time.monotonic() + DXL_RETRY_SEC
            return
        self.dxl_retry_after[module] = 0.0
        self.state.update_module(module, "dxl", {
            "id": dxl_id,
            "model": model,
            "torque": torque,
            "mode": OPERATING_MODE_TEXT.get(mode, str(mode)),
            "current_a": current_raw * CURRENT_UNIT_A if current_raw is not None else None,
            "velocity_rad_s": velocity_raw * PRESENT_VELOCITY_UNIT if velocity_raw is not None else None,
            "velocity_rpm": (
                velocity_raw * PRESENT_VELOCITY_UNIT * 60.0 / (2.0 * math.pi)
                if velocity_raw is not None
                else None
            ),
            "position_deg": position_raw * 360.0 / DXL_POSITION_RESOLUTION if position_raw is not None else None,
            "position_rad": (
                position_raw * 2.0 * math.pi / DXL_POSITION_RESOLUTION if position_raw is not None else None
            ),
            "voltage_v": voltage_raw * 0.1 if voltage_raw is not None else None,
            "temperature_c": temp,
            "moving": moving,
            "hardware_error": hardware_error_text(hw_error),
            "read_status": "; ".join(errors) if errors else "OK",
        })

    def read_encoder(self, module, encoder_id):
        try:
            self.set_baudrate(self.encoder_baudrate)
        except Exception as exc:
            self.state.set_message(f"Encoder baud failed: {exc}")
            return
        ser = self.port.ser
        previous_timeout = ser.timeout
        try:
            ser.timeout = ENCODER_TIMEOUT_SEC
            for attempt in range(AMT212_RETRY_COUNT):
                self.prepare_shared_serial()
                ser.rts = RTS_TX
                ser.write(bytes([encoder_id]))
                ser.flush()
                ser.rts = RTS_RX
                time.sleep(AMT212_RESPONSE_DELAY_SEC)
                raw = ser.read(8)
                if len(raw) >= 2:
                    raw16 = raw[-2] | (raw[-1] << 8)
                    if verify_checksum(raw16):
                        count = raw16 & AMT212_POSITION_MASK
                        angle_deg = count * AMT212_DEG_PER_COUNT
                        self.state.update_module(module, "encoder", {
                            "id": encoder_id,
                            "angle_deg": angle_deg,
                            "angle_rad": math.radians(angle_deg),
                            "count": count,
                            "raw16": f"0x{raw16:04X}",
                        })
                        return
                if attempt < AMT212_RETRY_COUNT - 1:
                    time.sleep(AMT212_RETRY_DELAY_SEC)
        finally:
            ser.rts = RTS_RX
            ser.timeout = previous_timeout

    def prepare_shared_serial(self):
        ser = self.port.ser
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(SERIAL_TURNAROUND_SEC)

    def read_u8(self, dxl_id, address, errors):
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(self.port, dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def read_u16(self, dxl_id, address, errors):
        value, comm_result, dxl_error = self.packet.read2ByteTxRx(self.port, dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def read_u32(self, dxl_id, address, errors):
        value, comm_result, dxl_error = self.packet.read4ByteTxRx(self.port, dxl_id, address)
        return self.check_read(address, value, comm_result, dxl_error, errors)

    def check_read(self, address, value, comm_result, dxl_error, errors):
        if comm_result != COMM_SUCCESS:
            errors.append(f"{address}: {self.packet.getTxRxResult(comm_result)}")
            return None
        if dxl_error != 0:
            errors.append(f"{address}: {self.packet.getRxPacketError(dxl_error)}")
            return None
        return value

    def write_u8(self, dxl_id, address, value):
        self.set_baudrate(self.dxl_baudrate)
        comm_result, dxl_error = self.packet.write1ByteTxRx(self.port, dxl_id, address, value)
        self.check_write(address, comm_result, dxl_error)

    def write_u16(self, dxl_id, address, value):
        self.set_baudrate(self.dxl_baudrate)
        comm_result, dxl_error = self.packet.write2ByteTxRx(self.port, dxl_id, address, value)
        self.check_write(address, comm_result, dxl_error)

    def check_write(self, address, comm_result, dxl_error):
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(f"{address}: {self.packet.getTxRxResult(comm_result)}")
        if dxl_error != 0:
            raise RuntimeError(f"{address}: {self.packet.getRxPacketError(dxl_error)}")


class App:
    def __init__(self, args):
        self.args = args
        self.state = SharedState()
        self.lock = threading.Lock()
        self.serial_worker = SerialWorker(
            self.state,
            args.serial_port,
            args.dxl_baudrate,
            args.encoder_baudrate,
        )
        self.ak_worker = AkWorker(self.state, args.can_channel, args.can_bitrate)

    def start(self):
        self.serial_worker.start()
        self.ak_worker.start()

    def stop(self):
        self.serial_worker.stop()
        self.ak_worker.stop()
        self.serial_worker.join(timeout=1.0)
        self.ak_worker.join(timeout=1.0)

    def reconfigure(self, body):
        with self.lock:
            self.serial_worker.stop()
            self.ak_worker.stop()
            self.serial_worker.join(timeout=1.0)
            self.ak_worker.join(timeout=1.0)

            self.args.serial_port = body.get("serial_port", self.args.serial_port)
            self.args.dxl_baudrate = int(body.get("dxl_baudrate", self.args.dxl_baudrate))
            self.args.encoder_baudrate = int(body.get("encoder_baudrate", self.args.encoder_baudrate))
            self.args.can_channel = body.get("can_channel", self.args.can_channel)

            self.state.reset()
            self.serial_worker = SerialWorker(
                self.state,
                self.args.serial_port,
                self.args.dxl_baudrate,
                self.args.encoder_baudrate,
            )
            self.ak_worker = AkWorker(self.state, self.args.can_channel, self.args.can_bitrate)
            self.serial_worker.start()
            self.ak_worker.start()
            self.state.set_message("connections restarted")


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self.respond(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.respond_json(app.state.snapshot())
            elif path == "/api/config":
                ports = serial_port_choices()
                if not ports:
                    ports = [app.args.serial_port]
                if app.args.serial_port not in ports:
                    ports.insert(0, app.args.serial_port)
                self.respond_json({
                    "serial_ports": ports,
                    "serial_port": app.args.serial_port,
                    "dxl_baudrate": app.args.dxl_baudrate,
                    "encoder_baudrate": app.args.encoder_baudrate,
                    "can_channel": app.args.can_channel,
                })
            else:
                self.respond_json({"message": "not found"}, 404)

        def do_POST(self):
            path = urlparse(self.path).path
            body = self.read_body()
            try:
                if path == "/api/ak/erpm":
                    app.ak_worker.enqueue(body.get("module", "FL"), int(body.get("erpm", 0)))
                    self.respond_json({"message": "AK ERPM queued"})
                elif path == "/api/ak/stop":
                    app.ak_worker.enqueue(body.get("module", "FL"), 0)
                    self.respond_json({"message": "AK stop queued"})
                elif path == "/api/dxl/current-mode":
                    app.serial_worker.enqueue("current_mode", body.get("module", "ALL"))
                    self.respond_json({"message": "DXL current mode queued"})
                elif path == "/api/dxl/current":
                    app.serial_worker.enqueue("current", body.get("module", "FL"), float(body.get("current", 0.0)))
                    self.respond_json({"message": "DXL current queued"})
                elif path == "/api/dxl/stop":
                    app.serial_worker.enqueue("stop", body.get("module", "FL"))
                    self.respond_json({"message": "DXL stop queued"})
                elif path == "/api/dxl/torque-off":
                    app.serial_worker.enqueue("torque_off", body.get("module", "FL"))
                    self.respond_json({"message": "DXL torque off queued"})
                elif path == "/api/configure":
                    app.reconfigure(body)
                    self.respond_json({"message": "connections restarted"})
                else:
                    self.respond_json({"message": "not found"}, 404)
            except Exception as exc:
                self.respond_json({"message": str(exc)}, 400)

        def log_message(self, fmt, *args):
            return

        def read_body(self):
            size = int(self.headers.get("Content-Length", "0"))
            if size <= 0:
                return {}
            return json.loads(self.rfile.read(size).decode("utf-8"))

        def respond_json(self, payload, status=200):
            self.respond(status, json.dumps(payload).encode("utf-8"), "application/json")

        def respond(self, status, body, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="HTML hardware UI")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--serial-port", default=default_serial_port())
    parser.add_argument("--dxl-baudrate", type=int, default=DYNAMIXEL_BAUDRATE)
    parser.add_argument("--encoder-baudrate", type=int, default=AMT212_BAUDRATE)
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--can-bitrate", type=int, default=AK80_64_CAN_BITRATE)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = App(args)
    app.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    print(f"Open http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
        server.server_close()


if __name__ == "__main__":
    main()
