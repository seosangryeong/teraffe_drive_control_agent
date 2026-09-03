#!/usr/bin/env python3
"""Receive CubeMars/VESC CAN status frames without commanding the motor."""

import argparse
import struct
import time

import can


CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27

PACKET_NAMES = {
    CAN_PACKET_STATUS: "STATUS",
    CAN_PACKET_STATUS_2: "STATUS_2",
    CAN_PACKET_STATUS_3: "STATUS_3",
    CAN_PACKET_STATUS_4: "STATUS_4",
    CAN_PACKET_STATUS_5: "STATUS_5",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Listen for CubeMars/VESC extended CAN status frames. "
            "This script is receive-only and never sends motor commands."
        )
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN channel")
    parser.add_argument("--motor-id", type=int, default=101, help="Motor CAN ID to show")
    parser.add_argument("--timeout", type=float, default=10.0, help="Listen duration in seconds")
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="Show every extended CAN frame, not only the selected motor ID",
    )
    return parser.parse_args()


def split_extended_id(arbitration_id):
    return arbitration_id & 0xFF, arbitration_id >> 8


def i16(data, offset):
    return struct.unpack_from(">h", data, offset)[0]


def i32(data, offset):
    return struct.unpack_from(">i", data, offset)[0]


def decode_status(packet_id, data):
    if packet_id == CAN_PACKET_STATUS and len(data) >= 8:
        return (
            f"erpm={i32(data, 0)} "
            f"current={i16(data, 4) / 10.0:.1f}A "
            f"duty={i16(data, 6) / 1000.0:.3f}"
        )

    if packet_id == CAN_PACKET_STATUS_2 and len(data) >= 8:
        return (
            f"amp_hours={i32(data, 0) / 10000.0:.4f}Ah "
            f"amp_hours_charged={i32(data, 4) / 10000.0:.4f}Ah"
        )

    if packet_id == CAN_PACKET_STATUS_3 and len(data) >= 8:
        return (
            f"watt_hours={i32(data, 0) / 10000.0:.4f}Wh "
            f"watt_hours_charged={i32(data, 4) / 10000.0:.4f}Wh"
        )

    if packet_id == CAN_PACKET_STATUS_4 and len(data) >= 8:
        return (
            f"temp_fet={i16(data, 0) / 10.0:.1f}C "
            f"temp_motor={i16(data, 2) / 10.0:.1f}C "
            f"current_in={i16(data, 4) / 10.0:.1f}A "
            f"pid_pos={i16(data, 6) / 50.0:.2f}deg"
        )

    if packet_id == CAN_PACKET_STATUS_5 and len(data) >= 6:
        return f"tacho={i32(data, 0)} vin={i16(data, 4) / 10.0:.1f}V"

    return "raw=" + data.hex(" ")


def main():
    args = parse_args()
    deadline = time.monotonic() + args.timeout
    seen = 0

    print(
        f"Listening on {args.channel} for {args.timeout:.1f}s "
        f"(receive-only, motor commands are never sent)"
    )
    print(f"Filter: motor_id={args.motor_id}" if not args.show_all else "Filter: all extended frames")

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            msg = bus.recv(timeout=min(1.0, remaining))
            if msg is None:
                continue
            if not msg.is_extended_id:
                continue

            motor_id, packet_id = split_extended_id(msg.arbitration_id)
            if not args.show_all and motor_id != args.motor_id:
                continue

            seen += 1
            packet_name = PACKET_NAMES.get(packet_id, f"PACKET_{packet_id}")
            decoded = decode_status(packet_id, bytes(msg.data))
            print(
                f"{time.time():.3f} "
                f"id={motor_id} packet={packet_name} "
                f"can_id=0x{msg.arbitration_id:08X} {decoded}"
            )
    finally:
        bus.shutdown()

    if seen == 0:
        print("No matching extended CAN frames received.")


if __name__ == "__main__":
    main()
