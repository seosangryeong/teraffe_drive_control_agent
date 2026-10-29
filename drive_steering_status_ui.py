#!/usr/bin/env python3
"""Read-only web UI for MD drive and steering status ROS 2 topics."""

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray


MOTORS = (
    {"name": "motor_11", "label": "Drive", "id": 11},
    {"name": "motor_21", "label": "Steering", "id": 21},
)


class StatusStore:
    def __init__(self, stale_after):
        self.lock = threading.Lock()
        self.stale_after = stale_after
        self.started_at = time.time()
        self.status_count = 0
        self.alarm_count = 0
        self.last_status_at = None
        self.last_alarm_at = None
        self.motors = {
            motor["name"]: {
                "name": motor["name"],
                "label": motor["label"],
                "id": motor["id"],
                "connected": False,
                "position_rad": None,
                "position_deg": None,
                "rpm": None,
                "current_ma": None,
                "current_a": None,
                "alarm": None,
                "last_seen": None,
            }
            for motor in MOTORS
        }

    def update_status(self, message):
        now = time.time()
        with self.lock:
            self.status_count += 1
            self.last_status_at = now
            for index, name in enumerate(message.name):
                if name not in self.motors:
                    continue
                motor = self.motors[name]
                position = self._value(message.position, index)
                rpm = self._value(message.velocity, index)
                current_ma = self._value(message.effort, index)
                motor.update(
                    connected=True,
                    position_rad=position,
                    position_deg=math.degrees(position) if position is not None else None,
                    rpm=rpm,
                    current_ma=current_ma,
                    current_a=current_ma / 1000.0 if current_ma is not None else None,
                    last_seen=now,
                )

    def update_alarms(self, message):
        now = time.time()
        with self.lock:
            self.alarm_count += 1
            self.last_alarm_at = now
            for index, motor in enumerate(MOTORS):
                if index < len(message.data):
                    self.motors[motor["name"]]["alarm"] = int(message.data[index])

    def snapshot(self):
        now = time.time()
        with self.lock:
            motors = json.loads(json.dumps(self.motors))
            result = {
                "timestamp": now,
                "uptime_s": round(now - self.started_at, 1),
                "status_count": self.status_count,
                "alarm_count": self.alarm_count,
                "status_age_s": self._age(now, self.last_status_at),
                "alarm_age_s": self._age(now, self.last_alarm_at),
                "motors": motors,
            }
        for motor in result["motors"].values():
            motor["age_s"] = self._age(now, motor["last_seen"])
            motor["connected"] = bool(
                motor["last_seen"] is not None
                and now - motor["last_seen"] <= self.stale_after
            )
        return result

    @staticmethod
    def _value(values, index):
        return float(values[index]) if index < len(values) else None

    @staticmethod
    def _age(now, timestamp):
        return round(now - timestamp, 2) if timestamp is not None else None


class MdStatusNode(Node):
    def __init__(self, store):
        super().__init__("md_status_web_ui")
        self.create_subscription(JointState, "/motor_status", store.update_status, 10)
        self.create_subscription(
            Int32MultiArray, "/motor_alarms", store.update_alarms, 10
        )


HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MD Motor Status</title>
<style>
:root{color-scheme:dark;--bg:#101315;--panel:#1a1f22;--line:#343b40;--text:#f3f5f6;--muted:#9da8af;--ok:#45ca82;--bad:#ff6969;--drive:#58a8ff;--steer:#f0c653}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 system-ui,sans-serif;letter-spacing:0}
header{display:flex;justify-content:space-between;align-items:center;gap:16px;padding:18px 24px;border-bottom:1px solid var(--line)}
h1{font-size:20px;margin:0}.topic-state{color:var(--muted);text-align:right;font-variant-numeric:tabular-nums}
main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px;max-width:1050px;margin:0 auto;padding:20px}
.motor{background:var(--panel);border:1px solid var(--line);border-radius:6px;overflow:hidden}
.motor-head{display:flex;justify-content:space-between;align-items:center;padding:14px 16px;border-bottom:1px solid var(--line)}
.motor-head h2{font-size:16px;margin:0}.drive h2{color:var(--drive)}.steering h2{color:var(--steer)}
.badge{font-size:12px;color:var(--bad)}.badge.ok{color:var(--ok)}
dl{display:grid;grid-template-columns:1fr auto;gap:10px 18px;margin:0;padding:18px 16px}dt{color:var(--muted)}dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}
.alarm-ok{color:var(--ok)}.alarm-bad{color:var(--bad);font-weight:700}.waiting{color:var(--muted)}
@media(max-width:700px){header{align-items:flex-start;padding:15px}.topic-state{font-size:12px}main{grid-template-columns:1fr;padding:10px}}
</style></head><body>
<header><h1>MD Motor Status</h1><div class="topic-state"><div id="status-topic">/motor_status: waiting</div><div id="alarm-topic">/motor_alarms: waiting</div></div></header>
<main id="motors"></main>
<script>
const fmt=(v,d=2,u='')=>v===null||v===undefined?'--':Number(v).toFixed(d)+u;
const age=v=>v===null?'waiting':fmt(v,1,' s ago');
function card(m){
 const steering=m.label==='Steering';
 const alarmClass=m.alarm===0?'alarm-ok':m.alarm===null?'waiting':'alarm-bad';
 const alarmText=m.alarm===0?'OK':m.alarm===null?'Waiting':'Code '+m.alarm;
 return `<article class="motor ${steering?'steering':'drive'}"><div class="motor-head"><h2>${m.label} / ID ${m.id}</h2><span class="badge ${m.connected?'ok':''}">${m.connected?'LIVE':'OFFLINE'} · ${age(m.age_s)}</span></div><dl><dt>ROS joint</dt><dd>${m.name}</dd><dt>RPM</dt><dd>${fmt(m.rpm,0)}</dd><dt>Position</dt><dd>${fmt(m.position_rad,3,' rad')}</dd>${steering?`<dt>Steering angle</dt><dd>${fmt(m.position_deg,2,' deg')}</dd>`:''}<dt>Current</dt><dd>${fmt(m.current_a,3,' A')}</dd><dt>Raw current</dt><dd>${fmt(m.current_ma,0,' mA')}</dd><dt>Alarm</dt><dd class="${alarmClass}">${alarmText}</dd></dl></article>`;
}
async function refresh(){try{const response=await fetch('/api/status',{cache:'no-store'});const data=await response.json();document.getElementById('status-topic').textContent='/motor_status: '+age(data.status_age_s)+' · '+data.status_count+' msgs';document.getElementById('alarm-topic').textContent='/motor_alarms: '+age(data.alarm_age_s)+' · '+data.alarm_count+' msgs';document.getElementById('motors').innerHTML=Object.values(data.motors).map(card).join('')}catch(error){document.getElementById('status-topic').textContent='UI connection lost'}}
refresh();setInterval(refresh,500);
</script></body></html>"""


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                self.respond(200, "text/html; charset=utf-8", HTML.encode())
            elif path == "/api/status":
                body = json.dumps(store.snapshot(), separators=(",", ":")).encode()
                self.respond(200, "application/json", body)
            else:
                self.respond(404, "text/plain", b"Not found")

        def do_POST(self):
            self.respond(405, "text/plain", b"Read-only monitor")

        def respond(self, status, content_type, body):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            return

    return Handler


def main(args=None):
    parser = argparse.ArgumentParser(description="Read-only MD motor status web UI")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--stale-after", type=float, default=1.5)
    parsed, ros_args = parser.parse_known_args(args)

    store = StatusStore(parsed.stale_after)
    rclpy.init(args=ros_args)
    node = MdStatusNode(store)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    server = ThreadingHTTPServer((parsed.host, parsed.port), make_handler(store))
    print(f"Read-only MD status UI: http://127.0.0.1:{parsed.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
