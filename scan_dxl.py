import argparse

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from serial.tools import list_ports

from teraffe_drive_control_agent.config.dynamixel_config import PROTOCOL_VERSION
from teraffe_drive_control_agent.config.teraffe_config import (
    DYNAMIXEL_BAUDRATE,
    DYNAMIXEL_PORT,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Scan or ping Dynamixel IDs.")
    parser.add_argument("--port", default=DYNAMIXEL_PORT)
    parser.add_argument("--baudrate", type=int, default=DYNAMIXEL_BAUDRATE)
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument("--end-id", type=int, default=20)
    return parser.parse_args()


def main():
    args = parse_args()
    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    try:
        opened = port_handler.openPort()
    except Exception as exc:
        print(f"Failed to open {args.port}: {exc}")
        print_available_ports()
        return

    if not opened:
        print(f"Failed to open {args.port}")
        print_available_ports()
        return

    try:
        if not port_handler.setBaudRate(args.baudrate):
            print(f"Failed to set baudrate {args.baudrate} on {args.port}")
            return

        print(
            f"Scanning Dynamixel IDs {args.start_id}-{args.end_id} "
            f"on {args.port} at {args.baudrate} baud..."
        )
        found = []
        for dxl_id in range(args.start_id, args.end_id + 1):
            model_number, comm_result, dxl_error = packet_handler.ping(
                port_handler,
                dxl_id,
            )
            if comm_result == COMM_SUCCESS and dxl_error == 0:
                found.append((dxl_id, model_number))
                print(f"  ID {dxl_id}: model {model_number}")

        if found:
            print("\n[SUCCESS] Found Dynamixel devices:")
            for dxl_id, model_number in found:
                print(f"  ID {dxl_id}: model {model_number}")
        else:
            print("\n[FAIL] No Dynamixel devices responded.")
    finally:
        port_handler.closePort()


def print_available_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


if __name__ == "__main__":
    main()
