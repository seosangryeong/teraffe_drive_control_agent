#!/usr/bin/env python3
"""Browser UI for leg tension/contraction commands."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from teraffe_drive_control_agent.robot_hardware_web_ui import (
    App,
    HTML as BASE_HTML,
    MODULES,
    parse_args,
    serial_port_choices,
)


TENSION_ERPM = 5000
TENSION_DXL_CURRENT_A = -0.05
CONTRACTION_ERPM = -TENSION_ERPM
CONTRACTION_DXL_CURRENT_A = -TENSION_DXL_CURRENT_A


HTML = (
    BASE_HTML
    .replace("<title>Teraffe Hardware UI</title>", "<title>Teraffe Tension UI</title>")
    .replace("<div>Teraffe Hardware Control</div>", "<div>Teraffe Tension Control</div>")
    .replace(
        "    @media (max-width: 980px) {",
        """    .leg-map { cursor: pointer; }
    .leg-map.selected rect,
    .leg-map.selected circle {
      stroke: var(--blue);
      stroke-width: 4px;
    }
    .selected-module {
      font-size: 28px;
      font-weight: 850;
      color: var(--blue);
    }
    @media (max-width: 980px) {""",
    )
    .replace(
        """          <div class="button-row">
            <button onclick="post('/api/configure')" class="primary">Apply Connections</button>
          </div>
        </div>
        <div>
          <h2>DXL Current</h2>""",
        """          <div class="button-row">
            <button onclick="post('/api/configure')" class="primary">Apply Connections</button>
          </div>
        </div>
        <div>
          <h2>Leg Motion</h2>
          <div class="control-grid">
            <label>Selected</label><div class="selected-module" id="selectedModule">FL</div>
            <label>Tension</label><input value="AK +5000 / DXL -0.05 A" disabled>
            <label>Contraction</label><input value="AK -5000 / DXL +0.05 A" disabled>
          </div>
          <div class="button-row">
            <button onclick="commandMotion('tension')" class="primary">Tension</button>
            <button onclick="commandMotion('contraction')" class="primary">Contraction</button>
            <button onclick="stopLeg()" class="stop">Stop</button>
          </div>
        </div>
        <div>
          <h2>DXL Current</h2>""",
    )
    .replace(
        """    function init() {
      for (const id of ["dxlModule", "akModule"]) {""",
        """    let selectedModule = null;

    function init() {
      for (const id of ["dxlModule", "akModule"]) {""",
    )
    .replace(
        """      drawCards();
      setInterval(refresh, 50);""",
        """      drawCards();
      setupLegMapClicks();
      setInterval(refresh, 50);""",
    )
    .replace(
        """    function drawCards() {""",
        """    function setupLegMapClicks() {
      for (const m of modules) {
        const el = document.getElementById(`map-${m}`);
        if (!el) continue;
        el.classList.add("leg-map");
        el.addEventListener("click", () => selectModule(m));
      }
      selectModule("FL");
    }

    function selectModule(module) {
      if (!modules.includes(module)) return;
      selectedModule = module;
      document.getElementById("selectedModule").textContent = module;
      for (const m of modules) {
        const el = document.getElementById(`map-${m}`);
        if (el) el.classList.toggle("selected", m === module);
      }
      const dxl = document.getElementById("dxlModule");
      const ak = document.getElementById("akModule");
      if (dxl) dxl.value = module;
      if (ak) ak.value = module;
    }

    function commandMotion(kind) {
      if (!selectedModule) return;
      post(`/api/motion/${kind}`, {module: selectedModule});
    }

    function stopLeg() {
      if (!selectedModule) return;
      post("/api/motion/stop", {module: selectedModule});
    }

    function drawCards() {""",
    )
)


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                self.respond(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.respond_json(app.state.snapshot())
            elif path == "/api/config":
                ports = serial_port_choices() or [app.args.serial_port]
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
                if path == "/api/motion/tension":
                    module = self.motion_module(body)
                    self.enqueue_motion(module, TENSION_ERPM, TENSION_DXL_CURRENT_A)
                    self.respond_json({"message": f"Tension queued: {module}"})
                elif path == "/api/motion/contraction":
                    module = self.motion_module(body)
                    self.enqueue_motion(module, CONTRACTION_ERPM, CONTRACTION_DXL_CURRENT_A)
                    self.respond_json({"message": f"Contraction queued: {module}"})
                elif path == "/api/motion/stop":
                    module = self.motion_module(body)
                    app.ak_worker.enqueue(module, 0)
                    app.serial_worker.enqueue("stop", module)
                    self.respond_json({"message": f"Stop queued: {module}"})
                elif path == "/api/ak/erpm":
                    module = body.get("module", "FL")
                    app.ak_worker.enqueue(module, int(body.get("erpm", 0)))
                    self.respond_json({"message": "AK ERPM queued"})
                elif path == "/api/ak/stop":
                    module = body.get("module", "FL")
                    app.ak_worker.enqueue(module, 0)
                    self.respond_json({"message": "AK stop queued"})
                elif path == "/api/dxl/current-mode":
                    module = body.get("module", "ALL")
                    app.serial_worker.enqueue("current_mode", module)
                    self.respond_json({"message": "DXL current mode queued"})
                elif path == "/api/dxl/current":
                    module = body.get("module", "FL")
                    app.serial_worker.enqueue("current", module, float(body.get("current", 0.0)))
                    self.respond_json({"message": "DXL current queued"})
                elif path == "/api/dxl/stop":
                    module = body.get("module", "FL")
                    app.serial_worker.enqueue("stop", module)
                    self.respond_json({"message": "DXL stop queued"})
                elif path == "/api/dxl/torque-off":
                    module = body.get("module", "FL")
                    app.serial_worker.enqueue("torque_off", module)
                    self.respond_json({"message": "DXL torque off queued"})
                elif path == "/api/configure":
                    app.reconfigure(body)
                    self.respond_json({"message": "connections restarted"})
                else:
                    self.respond_json({"message": "not found"}, 404)
            except Exception as exc:
                self.respond_json({"message": str(exc)}, 400)

        def enqueue_motion(self, module, erpm, current_a):
            app.ak_worker.enqueue(module, erpm)
            app.serial_worker.enqueue("current_mode", module)
            app.serial_worker.enqueue("current", module, current_a)

        def motion_module(self, body):
            module = body.get("module")
            if module not in MODULES:
                raise ValueError("select one leg first")
            return module

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
